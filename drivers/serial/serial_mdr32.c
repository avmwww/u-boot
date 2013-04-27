/*
 * (C) Copyright 2013
 * Andrey Mitrofanov, <avmwww@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <common.h>
//#include <asm/arch/hardware.h>

#define UART_FR_RI	0x100
#define UART_FR_TXFE	0x80
#define UART_FR_RXFF	0x40
#define UART_FR_TXFF	0x20
#define UART_FR_RXFE	0x10
#define UART_FR_BUSY	0x08
#define UART_FR_DCD	0x04
#define UART_FR_DSR	0x02
#define UART_FR_CTS	0x01


#define UART1_BASE	0x40030000
#define UART2_BASE	0x40038000

#define PORTF_BASE          0x400E8000

#define RST_CLK_BASE        0x40020000

#define     __IO    volatile

struct mdr32_uart
{
  __IO uint32_t DR;
  __IO uint32_t RSR_ECR;
       uint32_t RESERVED0[4];
  __IO uint32_t FR;
       uint32_t RESERVED1;
  __IO uint32_t ILPR;
  __IO uint32_t IBRD;
  __IO uint32_t FBRD;
  __IO uint32_t LCR_H;
  __IO uint32_t CR;
  __IO uint32_t IFLS;
  __IO uint32_t IMSC;
  __IO uint32_t RIS;
  __IO uint32_t MIS;
  __IO uint32_t ICR;
  __IO uint32_t DMACR;
};

typedef struct
{
  __IO uint32_t RXTX;
  __IO uint32_t OE;
  __IO uint32_t FUNC;
  __IO uint32_t ANALOG;
  __IO uint32_t PULL;
  __IO uint32_t PD;
  __IO uint32_t PWR;
  __IO uint32_t GFEN;
}PORT_TypeDef;

typedef struct
{
  __IO uint32_t CLOCK_STATUS;
  __IO uint32_t PLL_CONTROL;
  __IO uint32_t HS_CONTROL;
  __IO uint32_t CPU_CLOCK;
  __IO uint32_t USB_CLOCK;
  __IO uint32_t ADC_MCO_CLOCK;
  __IO uint32_t RTC_CLOCK;
  __IO uint32_t PER_CLOCK;
  __IO uint32_t CAN_CLOCK;
  __IO uint32_t TIM_CLOCK;
  __IO uint32_t UART_CLOCK;
  __IO uint32_t SSP_CLOCK;
}RST_CLK_TypeDef;


#define PORTF               ((PORT_TypeDef*)    PORTF_BASE)
#define RST_CLK             ((RST_CLK_TypeDef*) RST_CLK_BASE)


DECLARE_GLOBAL_DATA_PTR;

#define CONFIG_SYS_MDR32_CONSOLE	UART2_BASE

//static struct mdr32_uart *uart = (struct mdr32_uart *)CONFIG_SYS_MDR32_CONSOLE;

#define uart ((struct mdr32_uart *)CONFIG_SYS_MDR32_CONSOLE)

void serial_setbrg (void)
{
	unsigned short divisor = 0;

	divisor = 48000000  * 4 / gd->baudrate;
	uart->CR = 0;
	/* Set baud rate divisor */
	uart->IBRD = (divisor / 16 / 4);
	uart->FBRD = (divisor & 077);

	/* Enable uart, rx, tx */
	uart->CR = 0x301;
}

int serial_init (void)
{
        /* Set UART2 for debug output. */
	RST_CLK->PER_CLOCK |= 1 << 29;	// вкл. тактирования PORTF
	PORTF->FUNC |= 0b1111; 	// переопределенная функция для
			 	// PF0(UART2_RXD) и PF1(UART2_TXD)
	PORTF->ANALOG |= 3;	// цифровые выводы
	PORTF->PWR &= ~0b1111;
	PORTF->PWR |= 0b0101;

	RST_CLK->PER_CLOCK |= 1 << 7;	// вкл. тактирования UART2
//	RST_CLK->PER_CLOCK |= 1 << 6;	// вкл. тактирования UART1

	RST_CLK->UART_CLOCK = (1 << 25);

	/* 8N1, FIFO enabled */
	uart->LCR_H = (3 << 5) | 0x10;
	serial_setbrg ();

	return (0);
}

void serial_putc (const char c)
{
	int i;
	if (c == '\n')
	{
	//	while ((uart->FR & UART_FR_TXFF) != 0);
		uart->DR = '\r';
	}
	for (i = 0; i < 1000; i++)
		__asm__ volatile (" nop");
//	while ((uart->FR & 0x20) != 0); /* Wait TXFF */
	uart->DR = c;
}

int serial_getc (void)
{
	while ((uart->FR & 0x10) != 0); /* Wait RXFE */
	return uart->DR & 0xFF;
}

void
serial_puts (const char *s)
{
	while (*s) {
		serial_putc (*s++);
	}
}

/* Test if there is a byte to read */
int serial_tstc (void)
{
	return (!(uart->FR & 0x10));
}
