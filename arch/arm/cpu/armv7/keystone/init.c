/*
 * Copyright (C) 2012 Texas Instruments
 *
 * Keystone: Architecture initialization
 *
 * See file CREDITS for list of people who contributed to this
 * project.
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
#include <asm/arch/clock.h>
#include <asm/arch/hardware.h>

void chip_configuration_unlock(void)
{
	__raw_writel(KEYSTONE_KICK0_MAGIC, KEYSTONE_KICK0);
	__raw_writel(KEYSTONE_KICK1_MAGIC, KEYSTONE_KICK1);
}

void share_all_segments(int priv_id);

int arch_cpu_init(void)
{
	chip_configuration_unlock();
	icache_enable();

#ifdef CONFIG_SOC_TCI6638
	share_all_segments(8);
	share_all_segments(9);
#endif	

	return 0;
}

void reset_cpu(ulong addr)
{
	volatile u32* rstctrl = (volatile u32*)	(CLOCK_BASE + 0xe8);
	u32 tmp;

	tmp = *rstctrl & 0xffff0000;
	*rstctrl = tmp | 0x5a69;

	*rstctrl &= 0xfffe0000;

	for (;;);
}

