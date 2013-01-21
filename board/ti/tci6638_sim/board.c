/*
 * Copyright (C) 2012 Texas Instruments Inc.
 *
 * TCI6638 SIM: Board initialization
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
#include <asm/arch/hardware.h>
#include <asm/arch/clock.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/arch/nand_defs.h>


DECLARE_GLOBAL_DATA_PTR;

unsigned int external_clk[ext_clk_count] = {
	[sys_clk]	=	122880000,
	[alt_core_clk]	=	122880000,
	[pa_clk]	=	122880000,
	[tetris_clk]	=	122880000,
	[ddr3a_clk]	=	100000000,
	[ddr3b_clk]	=	100000000,
	[mcm_clk]	=	312500000,
	[pcie_clk]	=	100000000,
	[sgmii_srio_clk]=	156250000,
	[xgmii_clk]	=	156250000,
	[usb_clk]	=	100000000,
	[rp1_clk]	=	123456789    /* TODO: cannot find what is that */
};

int dram_init(void)
{
	gd->ram_size = CONFIG_MAX_RAM_BANK_SIZE;

	return 0;
}

#if defined(CONFIG_BOARD_EARLY_INIT_F)
int board_early_init_f(void)
{
	return 0;
}
#endif

int board_init(void)
{

	gd->bd->bi_arch_number = -1; /* MACH_TYPE_TCI6638_EVM; */
	gd->bd->bi_boot_params = LINUX_BOOT_PARAM_ADDR;

	return 0;
}

