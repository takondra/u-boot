/*
 * Copyright (C) 2011,2012 Texas Instruments.
 *
 * TCI6614: Clock management APIs
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

#ifndef __ASM_ARCH_CLOCK_TCI6614_H
#define __ASM_ARCH_CLOCK_TCI6614_H

#include <asm/arch/hardware.h>

#ifndef __ASSEMBLY__

/* PLL identifiers */
enum pll_type_e {
	SYS_PLL,
	DDR3_PLL,
	PASS_PLL
};
#define MAIN_PLL SYS_PLL

/* PLL configuration data */
struct pll_init_data {
	int pll;
	int div_enable;		/* Divider Enable/Disable */
	int pll_m;		/* PLL Multiplier */
	int pll_div2;		/* PLL Divider 2 */
	int pll_div5;		/* PLL Divider 5 */
	int pll_div8; 		/* PLL Divider 8 */
	int pll_d;		/* Pre Divider value */
	int pll_od;		/* PLL output divider */
};

void init_plls(int num_pll, struct pll_init_data *config);
void init_pll(const struct pll_init_data *data);
unsigned long clk_get_rate(unsigned int clk);
unsigned long clk_round_rate(unsigned int clk, unsigned long hz);
int clk_set_rate(unsigned int clk, unsigned long hz);

#endif

#endif
