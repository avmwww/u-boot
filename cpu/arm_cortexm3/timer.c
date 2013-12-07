/*
 * (C) Copyright 2010,2011
 * Sergei Poselenov, Emcraft Systems, sposelenov@emcraft.com
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include <common.h>

/* Internal tick units */
static unsigned long long timestamp;	/* Monotonic incrementing timer */
static ulong              lastdec;	/* Last decrementer snapshot */

int timer_init()
{
	volatile struct cm3_systick *systick =
		(volatile struct cm3_systick *)CM3_SYSTICK_BASE;

	systick->load = CM3_SYSTICK_LOAD_RELOAD_MSK - 1;
	systick->val = 0;

#ifdef CONFIG_ARMCORTEXM3_SYSTICK_CPU
	/* Use CPU clock, no ints */
	systick->ctrl = CM3_SYSTICK_CTRL_EN | CM3_SYSTICK_CTRL_SYSTICK_CPU;
#else
	/* Use external clock, no ints */
	systick->ctrl = CM3_SYSTICK_CTRL_EN;
#endif

	timestamp = 0;

	return 0;
}

uint32_t __div64_32(uint64_t *n, uint32_t base)
{
        uint64_t rem = *n;
        uint64_t b = base;
        uint64_t res, d = 1;
        uint32_t high = rem >> 32;

        /* Reduce the thing a bit first */
        res = 0;
        if (high >= base) {
                high /= base;
                res = (uint64_t) high << 32;
                rem -= (uint64_t) (high*base) << 32;
        }

        while ((int64_t)b > 0 && b < rem) {
                b = b+b;
                d = d+d;
        }

        do {
                if (rem >= b) {
                        rem -= b;
                        res += d;
                }
                b >>= 1;
                d >>= 1;
        } while (d);

        *n = res;
        return rem;
}

ulong get_timer(ulong base)
{
	volatile struct cm3_systick *systick =
		(volatile struct cm3_systick *)CM3_SYSTICK_BASE;
	ulong now = systick->val;
	uint64_t ts;

	if (lastdec >= now)
		timestamp += lastdec - now;
	else
		timestamp += lastdec + CM3_SYSTICK_LOAD_RELOAD_MSK - 1 - now;

	lastdec = now;

	ts = timestamp;
	__div64_32(&ts, (clock_get(CLOCK_SYSTICK) / CONFIG_SYS_HZ));
	return ts - base;
//		return timestamp / (clock_get(CLOCK_SYSTICK) / CONFIG_SYS_HZ) - base;
}

void reset_timer(void)
{
	volatile struct cm3_systick *systick =
		(volatile struct cm3_systick *)(CM3_SYSTICK_BASE);
	lastdec = systick->val;
	timestamp = 0;
}

/* delay x useconds */
void __udelay(ulong usec)
{
	ulong clc, tmp;
	volatile struct cm3_systick *systick =
		(volatile struct cm3_systick *)(CM3_SYSTICK_BASE);


	clc = usec * (clock_get(CLOCK_SYSTICK) / 1000000);

	/*
	 * Get current timestamp
	 */
	tmp = systick->val;

	/*
	 * Loop till event
	 *
	 * The SYSTICK timer count downwards.
	 */
	if (tmp < clc) {
		while (systick->val < tmp ||
			   systick->val > (CM3_SYSTICK_LOAD_RELOAD_MSK - 1 -
				clc + tmp)) ;	/* nop */
	} else {
		while (systick->val > (tmp - clc) && systick->val <= tmp) ;
	}
}

/*
 * This function is derived from PowerPC code (timebase clock frequency).
 * On ARM it returns the number of timer ticks per second.
 */
ulong get_tbclk(void)
{
	return clock_get(CLOCK_SYSTICK);
}
