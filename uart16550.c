#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/slab.h>

#include "uart16550.h"

#define BUFFER_SIZE 8192
#define DATA_SIZE 14

/* Recieving parameters */
int option = OPTION_BOTH;
module_param(option, int, 0);
MODULE_PARM_DESC(option, "Int from 1 to 3");

/* UART setup */
struct uart16550_device {
    struct cdev cdev;
    struct uart_info uart_info;
    DECLARE_KFIFO(read_buf, unsigned char, BUFFER_SIZE);
    DECLARE_KFIFO(write_buf, unsigned char, BUFFER_SIZE);
    wait_queue_head_t read_queue;
    wait_queue_head_t write_queue;
    atomic_t is_read_done;
    atomic_t is_write_done;
    atomic_t is_open;
    unsigned long base;
    size_t num_wrote;
    int irq;
};

static struct uart16550_device devs[MAX_NUMBER_DEVICES];

static void uart_set_param(struct uart16550_device *uart) {
    unsigned int lcr_bits;
    lcr_bits = uart->uart_info.length |
               uart->uart_info.parity |
               uart->uart_info.stop;

    outb(lcr_bits | 0x80, uart->base + UART_LCR); // DLAB on
    outb(uart->uart_info.baud, uart->base);
    /* 2 lines below are for custom baud rates */
    outb(uart->uart_info.baud & 0xff, uart->base); // DLL Divisor Latch (least significant byte)
    outb((uart->uart_info.baud >> 8) & 0xff, uart->base + UART_DLM); // DLM Divisor Latch (most significant byte)
    outb(lcr_bits, uart->base + UART_LCR); // DLAB off
}

static irqreturn_t irq_handler(int irq, void *dev_id)
{
    struct uart16550_device *uart = (struct uart16550_device *) dev_id;

    unsigned int i, num_to_write;
    unsigned char buf[DATA_SIZE];
    unsigned int uart_isr = inb(uart->base + UART_ISR);

    /* Get data using Receive Holding Register (RHR)
     *
     * Interrupt Status Register (ISR)
     * UART_ISR_RXRDY - received data ready
     * 
     * Interrupt Enable Register (IER)
     * UART_IER_RDI - disable/enable the receiver ready interrupt.
     * 
     * Receive Holding Register (LSR)
     * UART_LSR_DR - data ready in RHR (Receive Holding Register)
     *   
     */

    if (uart_isr & UART_ISR_RXRDY) {
        /* Disable the receiver ready interrupt */
        outb(inb(uart->base + UART_IER) & ~UART_IER_RDI, uart->base + UART_IER);

        /* Read while there is data in the UART holding register or FIFO */
        i = 0;
        while ((inb(uart->base + UART_LSR)) & UART_LSR_DR && i < DATA_SIZE) {
            buf[i] = inb(uart->base);
            i++;
        }
        kfifo_in(&uart->read_buf, buf, i);

        atomic_set(&uart->is_read_done, 1);
        wake_up_interruptible(&uart->read_queue);
    }


    /* Send data using Transmit Holding Register (THR)
     * 
     * Interrupt Status Register (ISR)
     * UART_ISR_TXRDY - transmitter holding register empty
     * 
     * Interrupt Enable Register (IER)
     * UART_IER_THRI - disable/enable the transmitter ready interrupt
     * 
     * Receive Holding Register (LSR)
     * UART_LSR_THR - transmitter hold register (or FIFO) is empty
     * 
     */
    i = 0;
    uart->num_wrote = 0;
    if (uart_isr & UART_ISR_TXRDY) {
        /* Disable the transmitter ready interrupt */
        outb(inb(uart->base + UART_IER) & ~UART_IER_THRI, uart->base + UART_IER);

        num_to_write = kfifo_out(&uart->write_buf, buf, DATA_SIZE);

        /* Write while transmitter hold register (or FIFO) is empty */
        while (((inb(uart->base + UART_LSR)) & UART_LSR_THR) && i < num_to_write) {
            outb(buf[i++], uart->base);
        }

        uart->num_wrote = i; 

        atomic_set(&uart->is_write_done, 1);
        wake_up_interruptible(&uart->write_queue);
    }
    
    return IRQ_HANDLED;
}

static int uart16550_open(struct inode *inode, struct file *file)
{
    struct uart16550_device *uart;
    uart = container_of(inode->i_cdev, struct uart16550_device, cdev);

    if (atomic_cmpxchg(&uart->is_open, 0, 1)) {
        pr_err("Can't open this uart device\n");
        return -EBUSY;
    }

    file->private_data = uart;

	return 0;
}

static int uart16550_release(struct inode *inode, struct file *file)
{
    struct uart16550_device *uart = (struct uart16550_device *) file->private_data;
    atomic_set(&uart->is_open, 0);

	return 0;
}

static ssize_t uart16550_read(struct file *file, char __user *user_buffer, size_t size, loff_t *offset)
{
    struct uart16550_device *uart;
    size_t num_read;
    unsigned char *ch;
    unsigned long flags;
    uart = (struct uart16550_device *) file->private_data;

    if (size <= 0) {
        return 0;
    }

    wait_event_interruptible(uart->read_queue, atomic_read(&uart->is_read_done) == 1);

    ch = (unsigned char *)kmalloc(size, GFP_KERNEL);
    
    num_read = kfifo_out(&uart->read_buf, ch, size);

    if (num_read) {
        if (copy_to_user(user_buffer, ch, num_read)) {
            return -EFAULT;
        }
    }

    kfree(ch);

    atomic_set(&uart->is_read_done, 0);
    outb(inb(uart->base + UART_IER) | UART_IER_RDI, uart->base + UART_IER);

    return num_read;
}

static ssize_t uart16550_write(struct file *file, const char __user *user_buffer, size_t size, loff_t *offset)
{
    struct uart16550_device *uart;
    size_t max = size > DATA_SIZE ? DATA_SIZE : size;
    unsigned char *ch;
    uart = (struct uart16550_device *) file->private_data;

    if (size <= 0) {
        return 0;
    }

    ch = (unsigned char *)kmalloc(max, GFP_KERNEL);

    if (copy_from_user(ch, user_buffer, max)) {
        return -EFAULT;
    }
    atomic_set(&uart->is_write_done, 0);

    kfifo_in(&uart->write_buf, ch, max);

    kfree(ch);

    outb(inb(uart->base + UART_IER) | UART_IER_THRI, uart->base + UART_IER);
    wait_event_interruptible(uart->write_queue, atomic_read(&uart->is_write_done) == 1);

    return uart->num_wrote;
}

static long uart16550_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct uart16550_device *uart = (struct uart16550_device *) file->private_data;
	unsigned long flags;
    struct uart_info info;

    switch (cmd) {
    case 1:
        if (copy_from_user(&info, (void __user *)arg, sizeof(info))) {
            return -EFAULT;
        }

        uart->uart_info = info;

        uart_set_param(uart);

        break;
	default:
		return -ENOTTY;
	}

     return 0;
}

static const struct file_operations uart_fops = {
    .owner = THIS_MODULE,
    .open = uart16550_open,
    .release = uart16550_release,
    .read = uart16550_read,
    .write = uart16550_write,
    .unlocked_ioctl = uart16550_ioctl,
};

static int uart_setup(void) {
    int i, ret;
    int uart_minor;
    int dev_number;

    if (option == OPTION_BOTH) {
        dev_number = 2;
        uart_minor = 0;
    }
    else if (option == OPTION_COM1) {
        dev_number = 1;
        uart_minor = 0;
    }
    else if (option == OPTION_COM2) {
        dev_number = 1;
        uart_minor = 1;
    }

    ret = register_chrdev_region(MKDEV(UART_MAJOR, uart_minor),
                        dev_number, MODULE_NAME);
    if (ret != 0) {
        pr_err("register_region failed: %d\n", ret);
        goto exit_unregister;
    }

    for (i = 0; i < dev_number; i++) {
        struct uart16550_device *uart;
        uart = &devs[i];

        /* Assign base register, irq num & is_open variable */
        switch(option) {
        case OPTION_COM1:
            uart->base = COM1_REG;
            uart->irq = IRQ_COM1;
            break;
        case OPTION_COM2:
            uart->base = COM2_REG;
            uart->irq = IRQ_COM2;
            break;
        case OPTION_BOTH:
            if (i == 0) {
                uart->base = COM1_REG;
                uart->irq = IRQ_COM1;
            }
            else if (i == 1) {
                uart->base = COM2_REG;
                uart->irq = IRQ_COM2;
            }
            break;
        }

        atomic_set(&uart->is_open, 0);
        atomic_set(&uart->is_read_done, 0);
        atomic_set(&uart->is_write_done, 0);

        /* Request I/O region */
        if (!request_region(uart->base, 8, MODULE_NAME)) {
            pr_err("Failed to request I/O port for %d\n", i+1);
            ret = -EBUSY;
		    goto exit_unregister;
        }

        /* Request IRQ */
        ret = request_irq(uart->irq, irq_handler, IRQF_SHARED,
			  MODULE_NAME, uart);
        if (ret) {
            pr_err("COM%d request_irq failed: %d\n", i+1, ret);
            goto exit_release_ports;
        }

        /* Init kfifo, waitqueue */

        INIT_KFIFO(uart->read_buf);
        INIT_KFIFO(uart->write_buf);

        init_waitqueue_head(&uart->read_queue);
        init_waitqueue_head(&uart->write_queue);

        /* Init & add cdev */
        cdev_init(&uart->cdev, &uart_fops);
        if (option == OPTION_COM2) {
            ret = cdev_add(&uart->cdev, MKDEV(UART_MAJOR, 1), 1);
            if (ret) {
                goto exit_release_ports;
            }
        } else {
            ret = cdev_add(&uart->cdev, MKDEV(UART_MAJOR, i), 1);
            if (ret) {
                goto exit_release_ports;
            }
        }

        /* Enable and clear FIFO */
        outb(UART_FCR_FIFO_ENABLE |
             UART_FCR_RCVR_RESET |
             UART_FCR_XMIT_RESET, uart->base + UART_FCR);

        /* Enable IRQs */
        outb(UART_IER_RDI | UART_IER_THRI, uart->base + UART_IER);

        /* Config default transmission params */
        uart->uart_info.baud = UART16550_BAUD_9600;
        uart->uart_info.length = UART16550_LEN_8;
        uart->uart_info.stop = UART16550_STOP_1;
        uart->uart_info.parity = UART16550_PAR_NONE;
        uart_set_param(uart);
    }
    return 0;

exit_release_ports:
    if (option == OPTION_BOTH) {
        release_region(COM1_REG, 8);
	    release_region(COM2_REG, 8);
    }
    else if (option == OPTION_COM1) {
        release_region(COM1_REG, 8);
    }
    else if (option == OPTION_COM2) {
        release_region(COM2_REG, 8);
    }
exit_unregister:
    if (option == OPTION_BOTH) {
        unregister_chrdev_region(MKDEV(UART_MAJOR, 0),
                MAX_NUMBER_DEVICES);
    }
    else if (option == OPTION_COM1) {
        unregister_chrdev_region(MKDEV(UART_MAJOR, 0), 1);
    }
    else if (option == OPTION_COM2) {
        unregister_chrdev_region(MKDEV(UART_MAJOR, 1), 1);
    }

    return ret;
}

static int uart16550_init(void)
{
    int ret;

    if (option != 1 && option != 2 && option != 3) {
        pr_err("Wrong arg value\n");
        return -EINVAL;
    }

    ret = uart_setup();
    if (ret) {
        return ret;
    }

    return 0;
}

static void uart16550_exit(void)
{
    int uart_minor, dev_number, i;

    if (option == OPTION_BOTH) {
        dev_number = 2;
        uart_minor = 0;
    }
    else if (option == OPTION_COM1) {
        dev_number = 1;
        uart_minor = 0;
    }
    else if (option == OPTION_COM2) {
        dev_number = 1;
        uart_minor = 1;
    }

    for (i = 0; i < dev_number; i++) {
        free_irq(devs[i].irq, &devs[i]);
        cdev_del(&devs[i].cdev);
        release_region(devs[i].base, 8);
    }

    unregister_chrdev_region(MKDEV(UART_MAJOR, uart_minor), dev_number);
}

module_init(uart16550_init);
module_exit(uart16550_exit);
