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

#include <common.h>

#include <asm/arch/mdr32.h>
#include "clock.h"

/*
 * Print the CPU specific information
 */
int print_cpuinfo(void)
{
	char	buf[2][32];

	printf("CPU  : MDR32F9Qx (Cortex-M3 %03x), part No: %d\n",
		(CM3_SCB_REGS->cpuid >> 4) & 0xfff,
		CM3_SCB_REGS->cpuid & 0xf);

	strmhz(buf[0], clock_get(CLOCK_SYSCLK));
	strmhz(buf[1], clock_get(CLOCK_HSE));
	printf("Freqs: SYSCLK=%sMHz,HSE=%sMHz\n",
		buf[0], buf[1]);

	return 0;
}

void abort(void)
{
  while (1);
}

