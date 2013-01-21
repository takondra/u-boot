/*
 * Copyright (c) 2012, Texas Instruments <www.ti.com>
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <common.h>
#include <asm/io.h>
#include <div64.h>

DECLARE_GLOBAL_DATA_PTR;

int timer_init(void)
{
	gd->tbl = 0;
	gd->tbu = 0;

	gd->timer_rate_hz = CONFIG_SYS_HZ_CLOCK / CONFIG_SYS_HZ;

	return 0;
}

unsigned long long get_ticks(void)
{
	ulong nowl, nowu;

	asm volatile("mrrc p15, 0, %0, %1, c14" : "=r" (nowl), "=r" (nowu));

	gd->tbl = nowl;
	gd->tbu = nowu;

	return (((unsigned long long)gd->tbu) << 32) | gd->tbl;
}


ulong get_timer(ulong base)
{
	return lldiv(get_ticks(), gd->timer_rate_hz) - base;
}

void __udelay(unsigned long usec)
{
	unsigned long long endtime;

	endtime = lldiv((unsigned long long)usec * gd->timer_rate_hz, 1000UL);

	endtime += get_ticks();

	while (get_ticks() < endtime)
		;
}

ulong get_tbclk(void)
{
	return gd->timer_rate_hz;
}
