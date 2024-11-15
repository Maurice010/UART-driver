#ifndef _UART16550_H
#define _UART16550_H

#define	OPTION_COM1			1
#define OPTION_COM2			2
#define OPTION_BOTH			3

#define COM1_REG            0x3f8
#define COM2_REG            0x2f8

#define UART16550_COM1_SELECTED		0x01
#define UART16550_COM2_SELECTED		0x02

#define MODULE_NAME		"uart16550"
#define UART_MAJOR		42
#define MAX_NUMBER_DEVICES		2

#define IRQ_COM1    4
#define IRQ_COM2    3

#ifndef _UART16550_REGS_H

/* REGISTERS */
/* Divisor Latch (most significant byte) */
#define UART_DLM    1 // when DLAB = 1

/* Interrupt Control Register */
#define UART_IER    0b001 // when DLAB = 0
#define UART_IER_RDI    0b001
#define UART_IER_THRI    0b010

/* FIFO Control Register */
#define UART_FCR    0b010
#define UART_FCR_FIFO_ENABLE 0b001
#define UART_FCR_RCVR_RESET 0b010
#define UART_FCR_XMIT_RESET 0b100

/* Interrupt Status Register */
#define UART_ISR    0b010
#define UART_ISR_TXRDY 0b010
#define UART_ISR_RXRDY  0b100

/* Line Control Register */
#define UART_LCR    0b011

/* Line Status Register */
#define UART_LSR    0b101
#define UART_LSR_DR     0b001
#define UART_LSR_THR    0b00100000

/* Baudrates */
#define UART16550_BAUD_1200		96
#define UART16550_BAUD_2400		48
#define UART16550_BAUD_4800		24
#define UART16550_BAUD_9600		12
#define UART16550_BAUD_19200		6
#define UART16550_BAUD_38400		3
#define UART16550_BAUD_56000		2
#define UART16550_BAUD_115200		1

/* Word lengths */
#define UART16550_LEN_5			0x00
#define UART16550_LEN_6			0x01
#define UART16550_LEN_7			0x02
#define UART16550_LEN_8			0x03

/* Stop bits */
#define UART16550_STOP_1		0x00
#define UART16550_STOP_2		0x04

/* Parity */
#define UART16550_PAR_NONE		0x00
#define UART16550_PAR_ODD		0x08
#define UART16550_PAR_EVEN		0x18
#define UART16550_PAR_STICK		0x20

#endif

struct uart_info {
    unsigned char baud, length, parity, stop;
};

#define UART16550_IOCTL_SET_LINE _IOW('k', 1, struct uart_info)

#endif