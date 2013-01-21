/*
 * Copyright (C) 2012 Texas Instruments.
 *
 * TCI6638: Clock management APIs
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

#ifndef __ASM_ARCH_CLOCK_TCI6638_H
#define __ASM_ARCH_CLOCK_TCI6638_H

#include <asm/arch/hardware.h>

#ifndef __ASSEMBLY__

enum ext_clk_e {
	sys_clk,
	alt_core_clk,
	pa_clk,
	tetris_clk,
	ddr3a_clk,
	ddr3b_clk,
	mcm_clk,
	pcie_clk,
	sgmii_srio_clk,
	xgmii_clk,
	usb_clk,
	rp1_clk,
	ext_clk_count /* number of external clocks */
};

extern unsigned int external_clk[ext_clk_count];

enum clk_e {
	core_pll_clk,
	pass_pll_clk,
	tetris_pll_clk,
	ddr3a_pll_clk,
	ddr3b_pll_clk,
	sys_clk0_clk,
	sys_clk0_1_clk,
	sys_clk0_2_clk,
	sys_clk0_3_clk,
	sys_clk0_4_clk,
	sys_clk0_6_clk,
	sys_clk0_8_clk,
	sys_clk0_12_clk,
	sys_clk0_24_clk,
	sys_clk1_clk,
	sys_clk1_3_clk,
	sys_clk1_4_clk,
	sys_clk1_6_clk,
	sys_clk1_12_clk,
	sys_clk2_clk,
	sys_clk3_clk
};

#define TCI6638_CLK1_6	sys_clk0_6_clk

/* PLL identifiers */
enum pll_type_e {
	CORE_PLL,
	PASS_PLL,
	TETRIS_PLL,
	DDR3A_PLL,
	DDR3B_PLL,
};
#define MAIN_PLL CORE_PLL

/* PLL configuration data */
struct pll_init_data {
	int pll;
	int pll_m;		/* PLL Multiplier */
	int pll_d;		/* PLL divider */
	int pll_od;		/* PLL output divider    */
};

void init_plls(int num_pll, struct pll_init_data *config);
void init_pll(const struct pll_init_data *data);
unsigned long clk_get_rate(unsigned int clk);
unsigned long clk_round_rate(unsigned int clk, unsigned long hz);
int clk_set_rate(unsigned int clk, unsigned long hz);


#endif

#endif
