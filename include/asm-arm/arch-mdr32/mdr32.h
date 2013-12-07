/*
 * (C) Copyright 2013
 *
 * Andrey Mitrofanov, avmwww@gmail.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * MDR32 processor definitions
 */
#ifndef _MACH_MDR32_H_
#define _MACH_MDR32_H_

/*
 * Clocks enumeration
 */
enum clock {
	CLOCK_SYSCLK,		/* SYSCLK clock frequency expressed in Hz     */
	CLOCK_HCLK,		/* HCLK clock frequency expressed in Hz       */
	CLOCK_HSE,		/* HSE clock frequency expressed in Hz      */
	CLOCK_SYSTICK,		/* Systimer clock frequency expressed in Hz   */
	CLOCK_END		/* for internal usage			      */
};

unsigned long clock_get(enum clock clck);

#define PAGE_SIZE	4096

/*
 * MDR32 regsters
 */
struct mdr32_rst_clk {
	u32 CLOCK_STATUS;
	u32 PLL_CONTROL;
	u32 HS_CONTROL;
	u32 CPU_CLOCK;
	u32 USB_CLOCK;
	u32 ADC_MCO_CLOCK;
	u32 RTC_CLOCK;
	u32 PER_CLOCK;
	u32 CAN_CLOCK;
	u32 TIM_CLOCK;
	u32 UART_CLOCK;
	u32 SSP_CLOCK;
};

#define RST_CLK_BASE			0x40020000
#define RST_CLK				((volatile struct mdr32_rst_clk *)RST_CLK_BASE)

#define RST_CLK_PER_CLOCK_CAN1		(1 << 0)
#define RST_CLK_PER_CLOCK_CAN2		(1 << 1)
#define RST_CLK_PER_CLOCK_USB		(1 << 2)
#define RST_CLK_PER_CLOCK_EEPROM	(1 << 3)
#define RST_CLK_PER_CLOCK_RSTCLK	(1 << 4)
#define RST_CLK_PER_CLOCK_DMA		(1 << 5)
#define RST_CLK_PER_CLOCK_UART1		(1 << 6)
#define RST_CLK_PER_CLOCK_UART2		(1 << 7)
#define RST_CLK_PER_CLOCK_SSP1		(1 << 8)
#define RST_CLK_PER_CLOCK_I2C1		(1 << 10)
#define RST_CLK_PER_CLOCK_POWER		(1 << 11)
#define RST_CLK_PER_CLOCK_WWDT		(1 << 12)
#define RST_CLK_PER_CLOCK_IWDT		(1 << 13)
#define RST_CLK_PER_CLOCK_TIMER1	(1 << 14)
#define RST_CLK_PER_CLOCK_TIMER2	(1 << 15)
#define RST_CLK_PER_CLOCK_TIMER3	(1 << 16)
#define RST_CLK_PER_CLOCK_ADC		(1 << 17)
#define RST_CLK_PER_CLOCK_DAC		(1 << 18)
#define RST_CLK_PER_CLOCK_COMP		(1 << 19)
#define RST_CLK_PER_CLOCK_SSP2		(1 << 20)
#define RST_CLK_PER_CLOCK_PORTA		(1 << 21)
#define RST_CLK_PER_CLOCK_PORTB		(1 << 22)
#define RST_CLK_PER_CLOCK_PORTC		(1 << 23)
#define RST_CLK_PER_CLOCK_PORTD		(1 << 24)
#define RST_CLK_PER_CLOCK_PORTE		(1 << 25)
#define RST_CLK_PER_CLOCK_BKP		(1 << 27)
#define RST_CLK_PER_CLOCK_PORTF		(1 << 29)
#define RST_CLK_PER_CLOCK_EXT_BUS	(1 << 30)

#define PORTA_BASE		0x400A8000
#define PORTB_BASE		0x400B0000
#define PORTC_BASE		0x400B8000
#define PORTD_BASE		0x400C0000
#define PORTE_BASE		0x400C8000
#define PORTF_BASE		0x400E8000


/*
 * uart
 */
#define UART1_BASE		0x40030000
#define UART2_BASE		0x40038000

#define UART_FR_RI		0x100
#define UART_FR_TXFE		0x80
#define UART_FR_RXFF		0x40
#define UART_FR_TXFF		0x20
#define UART_FR_RXFE		0x10
#define UART_FR_BUSY		0x08
#define UART_FR_DCD		0x04
#define UART_FR_DSR		0x02
#define UART_FR_CTS		0x01

#define UART_LCR_H_BRK		0x01
#define UART_LCR_H_PEN		0x02
#define UART_LCR_H_EPS		0x04
#define UART_LCR_H_STP2		0x08
#define UART_LCR_H_FEN		0x010
#define UART_LCR_H_WLEN(x)	((x & 3) << 5)
#define UART_LCR_H_SPS		0x080

#define UART_CR_UARTEN		0x01
#define UART_CR_SIREN		0x02
#define UART_CR_SIRLP		0x04
#define UART_CR_LBE		0x080
#define UART_CR_TXE		0x0100
#define UART_CR_RXE		0x0200
#define UART_CR_DTR		0x400
#define UART_CR_RTS		0x800
#define UART_CR_Out1		0x1000
#define UART_CR_Out2		0x2000
#define UART_CR_RTSE		0x4000
#define UART_CR_CTSE		0x8000

#endif /* _MACH_MDR32_H_ */
