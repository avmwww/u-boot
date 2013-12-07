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
#include <asm/arch/mdr32.h>

struct mdr32_uart
{
	u32 DR;
	u32 RSR_ECR;
	u32 RESERVED0[4];
	u32 FR;
	u32 RESERVED1;
	u32 ILPR;
	u32 IBRD;
	u32 FBRD;
	u32 LCR_H;
	u32 CR;
	u32 IFLS;
	u32 IMSC;
	u32 RIS;
	u32 MIS;
	u32 ICR;
	u32 DMACR;
};

DECLARE_GLOBAL_DATA_PTR;

#if  (CONFIG_SYS_MDR32_CONSOLE == 2)
# define CONSOLE_UART_BASE	UART2_BASE
#elif  (CONFIG_SYS_MDR32_CONSOLE == 1)
# define CONSOLE_UART_BASE	UART1_BASE
#else
# error "CONFIG_SYS_MDR32_CONSOLE wrong or not defined, should be 1 or 2"
#endif

static volatile struct mdr32_uart *uart = (volatile struct mdr32_uart *)CONSOLE_UART_BASE;

void serial_setbrg (void)
{
	unsigned short divisor = 0;

	divisor = 48000000  * 4 / gd->baudrate;
	uart->CR = 0;
	/* Set baud rate divisor */
	uart->IBRD = (divisor / 16 / 4);
	uart->FBRD = (divisor & 0x3f);

	/* 8N1, FIFO enabled */
	uart->LCR_H = UART_LCR_H_WLEN(3);
	/* Enable uart, rx, tx */
	uart->CR = UART_CR_UARTEN | UART_CR_RXE | UART_CR_TXE;
}

int serial_init (void)
{
#if (CONFIG_SYS_MDR32_CONSOLE == 2)
	/* Turn on clock of UART2 */
	RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_UART2;
#else
	/* Turn on clock of UART1 */
	RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_UART1;
#endif
	/* Enable clock on uart  */
	RST_CLK->UART_CLOCK = (1 << 25);

	serial_setbrg ();

	return (0);
}

void serial_putc (const char c)
{
	int i;
	if (c == '\n')
	{
		while (uart->FR & UART_FR_TXFF);
		uart->DR = '\r';
	}
	while (uart->FR & UART_FR_TXFF);
	uart->DR = c;
}

int serial_getc (void)
{
	while (uart->FR & UART_FR_RXFE);
	return uart->DR & 0xFF;
}

void serial_puts (const char *s)
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
