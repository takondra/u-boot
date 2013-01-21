/*
 * Copyright (C) 2012 Texas Instruments
 *
 * See file CREDITS for list of people who contributed to this
 * project.
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
#include <command.h>
#include <asm/arch/hardware.h>
#include <asm/arch/clock.h>
#include <asm/arch/psc_defs.h>

static u32 atoui(char* pstr)
{
	u32 res = 0;

	for(; *pstr != 0; pstr++) {
		if (*pstr < '0' || *pstr > '9')
			break;

		res = (res * 10) + (*pstr - '0');
	}

	return res;
}

struct pll_init_data cmd_pll_data = {
	.pll			= MAIN_PLL,
	.pll_m			= 16,
	.pll_d			= 1,
	.pll_od			= 2,

};

int do_pll_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{

	if (argc != 5)
		goto pll_cmd_usage;

	if (strncmp(argv[1], "main", 4) == 0)
		cmd_pll_data.pll = MAIN_PLL;
	else if (strncmp(argv[1], "pa", 2) == 0)
		cmd_pll_data.pll = PASS_PLL;
#ifdef CONFIG_SOC_TCI6638
	else if (strncmp(argv[1], "arm", 3) == 0)
		cmd_pll_data.pll = TETRIS_PLL;
	else if (strncmp(argv[1], "ddr3a", 5) == 0)
		cmd_pll_data.pll = DDR3A_PLL;
	else if (strncmp(argv[1], "ddr3b", 5) == 0)
		cmd_pll_data.pll = DDR3B_PLL;
#endif
#ifdef CONFIG_SOC_TCI6614
	else if (strncmp(argv[1], "ddr3", 4) == 0)
		cmd_pll_data.pll = DDR3_PLL;
#endif
	else
		goto pll_cmd_usage;

	cmd_pll_data.pll_m   = atoui(argv[2]);
	cmd_pll_data.pll_d   = atoui(argv[3]);
	cmd_pll_data.pll_od  = atoui(argv[4]);

	printf("Trying to set pll %d; mult %d; div %d; OD %d\n",
		cmd_pll_data.pll, cmd_pll_data.pll_m,
		cmd_pll_data.pll_d, cmd_pll_data.pll_od);
	init_pll(&cmd_pll_data);

	return 0;

pll_cmd_usage:
	return cmd_usage(cmdtp);
}

#ifdef CONFIG_SOC_TCI6638
U_BOOT_CMD(
	pllset,	5,	0,	do_pll_cmd,
	"set pll multiplier and pre divider",
	"<main|pa|arm|ddr3a|ddr3b> <mult> <div> <OD>\n"
);
#endif
#ifdef CONFIG_SOC_TCI6614
U_BOOT_CMD(
	pllset,	5,	0,	do_pll_cmd,
	"set pll multiplier and pre divider",
	"<main|pa|ddr3> <mult> <div> <OD>\n"
);
#endif

int do_getclk_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned int clk;
	unsigned int freq;

	if (argc != 2)
		goto getclk_cmd_usage;

	clk = atoui(argv[1]);

	freq = clk_get_rate(clk);
	printf("clock index [%d] - frequency %u\n", clk, freq);
	return 0;

getclk_cmd_usage:
	return cmd_usage(cmdtp);
}

U_BOOT_CMD(
	getclk,	2,	0,	do_getclk_cmd,
	"get clock rate",
	"<clk index>\n"
#ifdef CONFIG_SOC_TCI6638
	"See the 'enum clk_e' in the tci6638 clock.h for clk indexes\n"
#else
	"See the hardware-tci6614.h for clk indexes\n"
#endif
);

int do_psc_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int	psc_module;
	int	res;

	if (argc != 3)
		goto psc_cmd_usage;

	psc_module = atoui(argv[1]);
	if (strcmp(argv[2], "en") == 0) {
		res = psc_enable_module(psc_module);
		printf("psc_enable_module(%d) - %s\n", psc_module,
			(res) ? "ERROR" : "OK");
		return 0;
	}

	if (strcmp(argv[2], "di") == 0) {
		res = psc_disable_module(psc_module);
		printf("psc_disable_module(%d) - %s\n", psc_module,
			(res) ? "ERROR" : "OK");
		return 0;
	}

psc_cmd_usage:
	return cmd_usage(cmdtp);
}

U_BOOT_CMD(
	psc,	3,	0,	do_psc_cmd,
	"<enable/disable psc module>",
	"<mod index> <en|di>\n"
	"See the hardware.h for Power and Sleep Controller (PSC) Domains\n"
);

