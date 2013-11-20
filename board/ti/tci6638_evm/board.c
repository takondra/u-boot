/*
 * Copyright (C) 2012 Texas Instruments Inc.
 *
 * TCI6638 EVM : Board initialization
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
#include <fdt_support.h>
#include <libfdt.h>

#include <asm/arch/hardware.h>
#include <asm/arch/clock.h>
#include <asm/arch/clock_defs.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/arch/nand_defs.h>
#include <asm/arch/emac_defs.h>


DECLARE_GLOBAL_DATA_PTR;

u32 device_big_endian = 0;

unsigned int external_clk[ext_clk_count] = {
	[sys_clk]	=	122880000,
	[alt_core_clk]	=	125000000,
	[pa_clk]	=	122880000,
	[tetris_clk]	=	125000000,
	[ddr3a_clk]	=	100000000,
	[ddr3b_clk]	=	100000000,
	[mcm_clk]	=	312500000,
	[pcie_clk]	=	100000000,
	[sgmii_srio_clk]=	156250000,
	[xgmii_clk]	=	156250000,
	[usb_clk]	=	100000000,
	[rp1_clk]	=	123456789    /* TODO: cannot find what is that */
};

static struct async_emif_config async_emif_config[ASYNC_EMIF_NUM_CS] = {
	{			/* CS0 */
		.mode		= ASYNC_EMIF_MODE_NAND,
		.wr_setup	= 0xf,
		.wr_strobe	= 0x3f,
		.wr_hold	= 7,
		.rd_setup	= 0xf,
		.rd_strobe	= 0x3f,
		.rd_hold	= 7,
		.turn_around	= 3,
		.width		= ASYNC_EMIF_8,
	},

};

static struct pll_init_data pll_config[] = {
	CORE_PLL_1228,
	PASS_PLL_983,
	TETRIS_PLL_1200,
};

#ifdef CONFIG_SPL_BOARD_INIT
static struct pll_init_data spl_pll_config[] = {
	CORE_PLL_799,
	TETRIS_PLL_500,
};

void spl_init_keystone_plls(void)
{
	init_plls(ARRAY_SIZE(spl_pll_config), spl_pll_config);
}
#endif

void init_ddr3( void );

int dram_init(void)
{
	init_ddr3();

	gd->ram_size = get_ram_size(CONFIG_SYS_SDRAM_BASE,
				    CONFIG_MAX_RAM_BANK_SIZE);
	init_async_emif(ARRAY_SIZE(async_emif_config), async_emif_config);
	return 0;
}

#ifdef CONFIG_DRIVER_TI_KEYSTONE_NET
eth_priv_t eth_priv_cfg[] = {
	{
		.int_name	= "TCI6638_EMAC",
		.rx_flow	= 22,
		.phy_addr	= 0,
		.slave_port	= 1,
		.sgmii_link_type = SGMII_LINK_MAC_PHY,
	},
	{
		.int_name	= "TCI6638_EMAC1",
		.rx_flow	= 23,
		.phy_addr	= 1,
		.slave_port	= 2,
		.sgmii_link_type = SGMII_LINK_MAC_PHY,
	},
};

inline int get_num_eth_ports(void)
{
	return (sizeof(eth_priv_cfg) / sizeof(eth_priv_t));
}

eth_priv_t *get_eth_priv_ptr(int port_num)
{
	if (port_num < 0 || port_num >= get_num_eth_ports())
		return NULL;

	return &eth_priv_cfg[port_num]; 
}

int get_eth_env_param(char *env_name)
{
	char * env;
	int  res = -1;

	env = getenv(env_name);
	if (env)
		res = simple_strtol(env, NULL, 0);

	return res;
}

int board_eth_init(bd_t *bis)
{
	int	j;
	int	res;
	int	has_mdio = 0; /* doesn't have mdio by default */
	int	link_type_name[32];

	if ((res = get_eth_env_param("has_mdio")) >= 0 )
		has_mdio = res;

	tci6614_emac_set_has_mdio(has_mdio);

	for (j=0; j < get_num_eth_ports(); j++) {
		sprintf(link_type_name, "sgmii%d_link_type", j);
		res = get_eth_env_param(link_type_name);
		if (res < 0) {
			/* setting default type */
			if (has_mdio == 1)
				eth_priv_cfg[j].sgmii_link_type = SGMII_LINK_MAC_PHY;
			else
				eth_priv_cfg[j].sgmii_link_type = SGMII_LINK_MAC_PHY_FORCED;
		} else {
			eth_priv_cfg[j].sgmii_link_type = res;
		}

		tci6614_emac_initialize(&eth_priv_cfg[j]);
	}

#ifdef CONFIG_SOC_TCI6638
	tci6638_eth_open_close(eth_priv_cfg[0].dev);
#endif

	return 0;
}
#endif

/* Byte swap the 32-bit data if the device is BE */
int cpu_to_bus(u32 *ptr, u32 length)
{
	u32 i;

	if (device_big_endian)
		for (i = 0; i < length; i++, ptr++)
			*ptr = __swab32(*ptr);

	return 0;
}

#if defined(CONFIG_BOARD_EARLY_INIT_F)
int board_early_init_f(void)
{
	init_plls(ARRAY_SIZE(pll_config), pll_config);
	return 0;
}
#endif

int turn_off_all_dsps(void)
{
	int j;
	int ret = 0;

	for (j=0; j<8; j++) {
		if (psc_disable_module(j + TCI6638_LPSC_GEM_0))
			ret = 1;
		if (psc_disable_domain(j + 8))
			ret = 1;
	}

	return ret;
}

int turn_off_myself(void)
{
	printf("Turning off ourselves\r\n");
	mon_power_off(0);

	psc_disable_module(TCI6638_LPSC_TETRIS);
	psc_disable_domain(31);

	asm volatile (
		      "isb\n"
		      "dsb\n"
		      "wfi\n"
		     );

	printf("What! Should not see that\n");
	return 0;
}
int do_killme_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	return turn_off_myself();
}

U_BOOT_CMD(
	killme,	1,	0,	do_killme_cmd,
	"turn off main ARM core",
	"turn off main ARM core. Should not live after that :(\n"
);


int board_init(void)
{

	gd->bd->bi_arch_number = -1; /* MACH_TYPE_TCI6638_EVM; */
	gd->bd->bi_boot_params = LINUX_BOOT_PARAM_ADDR;

	return 0;
}

int __weak misc_init_r(void)
{
	char *env;

	debug_options = 0;
	env = getenv("debug_options");

	if (env)
		debug_options = simple_strtol(env, NULL, 0);

	if((debug_options & DBG_LEAVE_DSPS_ON) == 0)
		turn_off_all_dsps();

	return 0;
}

#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
#define K2_DDR3_START_ADDR 0x80000000
void ft_board_setup(void *blob, bd_t *bd)
{
	u64 start[2];
	u64 size[2];
	char name[32], *env, *endp;
	int lpae, nodeoffset;
	u32 ddr3a_size;
	int nbanks;

	env = getenv("mem_lpae");
	lpae = env && simple_strtol(env, NULL, 0);

	ddr3a_size = 0;
	if (lpae) {
		env = getenv("ddr3a_size");
		if (env)
			ddr3a_size = simple_strtol(env, NULL, 10);
		if ((ddr3a_size != 8) && (ddr3a_size != 4))
			ddr3a_size = 0;
	}

	nbanks = 1;
	start[0] = bd->bi_dram[0].start;
	size[0]  = bd->bi_dram[0].size;

	/* adjust memory start address for LPAE */
	if (lpae) {
		start[0] -= K2_DDR3_START_ADDR;
		start[0] += CONFIG_SYS_LPAE_SDRAM_BASE;
	}

	if ((size[0] == 0x80000000) && (ddr3a_size != 0)) {
		size[1] = ((u64)ddr3a_size - 2) << 30;
		start[1] = 0x880000000;
		nbanks++;
	}

	/* reserve memory at start of bank */
	sprintf(name, "mem_reserve_head");
	env = getenv(name);
	if (env) {
		start[0] += ustrtoul(env, &endp, 0);
		size[0] -= ustrtoul(env, &endp, 0);
	}

	sprintf(name, "mem_reserve");
	env = getenv(name);
	if (env)
		size[0] -= ustrtoul(env, &endp, 0);

	fdt_fixup_memory_banks(blob, start, size, nbanks);

	/* Fix up the initrd */
	if (lpae) {
		u64 initrd_start, initrd_end, *reserve_start, size;
		u32 *prop1, *prop2;
		int err;
		nodeoffset = fdt_path_offset(blob, "/chosen");
		if (nodeoffset >= 0) {
			prop1 = fdt_getprop(blob, nodeoffset, "linux,initrd-start", NULL);
			prop2 = fdt_getprop(blob, nodeoffset, "linux,initrd-end", NULL);
			if (prop1 && prop2) {
				initrd_start = __be32_to_cpu(*prop1);
				initrd_start -= K2_DDR3_START_ADDR;
				initrd_start += CONFIG_SYS_LPAE_SDRAM_BASE;
				initrd_start = __cpu_to_be64(initrd_start);
				initrd_end = __be32_to_cpu(*prop2);
				initrd_end -= K2_DDR3_START_ADDR;
				initrd_end += CONFIG_SYS_LPAE_SDRAM_BASE;
				initrd_end = __cpu_to_be64(initrd_end);

				err = fdt_delprop(blob, nodeoffset, "linux,initrd-start");
				if (err < 0)
					printf("error deleting linux,initrd-start\n");

				err = fdt_delprop(blob, nodeoffset, "linux,initrd-end");
				if (err < 0)
					printf("error deleting linux,initrd-end\n");

				err = fdt_setprop(blob, nodeoffset, "linux,initrd-start",
						&initrd_start, sizeof(initrd_start));
				if (err < 0)
					printf("error adding linux,initrd-start\n");

				err = fdt_setprop(blob, nodeoffset, "linux,initrd-end",
						&initrd_end, sizeof(initrd_end));
				if (err < 0)
					printf("error adding linux,initrd-end\n");
			}

			/*
			 * the initrd and other reserved memory areas are embedded in
			 * in the DTB itslef. fix up these addresses to 36 bit format
			 */
			reserve_start = (char *)blob + fdt_off_mem_rsvmap(blob);
			while (1) {
				*reserve_start = __cpu_to_be64(*reserve_start);
				size = __cpu_to_be64(*(reserve_start + 1));
				if (size) {
					*reserve_start -= K2_DDR3_START_ADDR;
					*reserve_start += CONFIG_SYS_LPAE_SDRAM_BASE;
					*reserve_start = __cpu_to_be64(*reserve_start);
				} else
					break;
				reserve_start += 2;
			}
		}
	}
}
#endif

void enable_caches(void)
{
#ifndef CONFIG_SYS_DCACHE_OFF
	/* Enable D-cache. I-cache is already enabled in start.S */
	dcache_enable();
#endif
}
