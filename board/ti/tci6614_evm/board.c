/*
 * TNETV107X-EVM: Board initialization
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
#include <miiphy.h>
#include <linux/mtd/nand.h>
#include <asm/arch/hardware.h>
#include <asm/arch/clock.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/arch/nand_defs.h>

DECLARE_GLOBAL_DATA_PTR;

u32 device_big_endian = 0;

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
	{
		/* System clock = 122.88*16/(2*1) = 983 MHz */
		.pll			= SYS_PLL,
		.div_enable		= 1,
		.pll_m			= 16,
		.pll_div2		= 4,
		.pll_div5		= 6,
		.pll_div8		= 64,
		.pll_d			= 1,
		.pll_od			= 2,
	},

	{
		/* DDR3 clock = 66.667*20/(2*1) = 666.67MHz */
		.pll			= DDR3_PLL,
		.div_enable		= 1,
		.pll_m			= 20,
		.pll_div2		= 0,
		.pll_div5		= 0,
		.pll_div8		= 0,
		.pll_d			= 1,
		.pll_od			= 2,
	},

	{
		/* PASS clock = 122.88*17/(2*1) = 1044.48 MHz */
		.pll			= PASS_PLL,
		.div_enable		= 1,
		.pll_m			= 17,
		.pll_div2		= 0,
		.pll_div5		= 0,
		.pll_div8		= 0,
		.pll_d			= 1,
		.pll_od			= 2,
	},
};

int init_ddr3(void)
{
    u32	tmp;

	/**************** 3.0 Leveling Register Configuration ********************/
	/* Using partial automatic leveling due to errata */

	/**************** 3.2 Invert Clock Out and set DLL lock time to maximum ********************/
	tmp = __raw_readl(TCI6614_DDR3_CONFIG_REG_0);
	tmp &= ~(0x007FE000);  /* clear ctrl_slave_ratio field */
	__raw_writel(tmp, TCI6614_DDR3_CONFIG_REG_0);

	tmp = __raw_readl(TCI6614_DDR3_CONFIG_REG_0);
	tmp |= 0x00200000;     /* set ctrl_slave_ratio to 0x100 */
	__raw_writel(tmp, TCI6614_DDR3_CONFIG_REG_0);

	tmp = __raw_readl(TCI6614_DDR3_CONFIG_REG_12);
	tmp |= 0x08000000;     /* Set invert_clkout = 1 */
	__raw_writel(tmp, TCI6614_DDR3_CONFIG_REG_12);

	tmp = __raw_readl(TCI6614_DDR3_CONFIG_REG_0);
	tmp |= 0xF;            /* set dll_lock_diff to 15 */
	__raw_writel(tmp, TCI6614_DDR3_CONFIG_REG_0);

	/* Values with invertclkout = 1 */
	/**************** 3.3+3.4 Partial Automatic Leveling ********************/
	/* set bit 9, leave bits 8:0 default 0x34 */
	tmp = __raw_readl(TCI6614_DDR3_CONFIG_REG_52);
	__raw_writel((tmp | 0x00000200), TCI6614_DDR3_CONFIG_REG_52);

	tmp = __raw_readl(TCI6614_DDR3_CONFIG_REG_53);
	__raw_writel((tmp | 0x00000200), TCI6614_DDR3_CONFIG_REG_53);

	tmp = __raw_readl(TCI6614_DDR3_CONFIG_REG_54);
	__raw_writel((tmp | 0x00000200), TCI6614_DDR3_CONFIG_REG_54);

	tmp = __raw_readl(TCI6614_DDR3_CONFIG_REG_55);
	__raw_writel((tmp | 0x00000200), TCI6614_DDR3_CONFIG_REG_55);

	tmp = __raw_readl(TCI6614_DDR3_CONFIG_REG_56);
	__raw_writel((tmp | 0x00000200), TCI6614_DDR3_CONFIG_REG_56);

	tmp = __raw_readl(TCI6614_DDR3_CONFIG_REG_57);
	__raw_writel((tmp | 0x00000200), TCI6614_DDR3_CONFIG_REG_57);

	tmp = __raw_readl(TCI6614_DDR3_CONFIG_REG_58);
	__raw_writel((tmp | 0x00000200), TCI6614_DDR3_CONFIG_REG_58);

	tmp = __raw_readl(TCI6614_DDR3_CONFIG_REG_59);
	__raw_writel((tmp | 0x00000200), TCI6614_DDR3_CONFIG_REG_59);

	tmp = __raw_readl(TCI6614_DDR3_CONFIG_REG_60);
	__raw_writel((tmp | 0x00000200), TCI6614_DDR3_CONFIG_REG_60);

	__raw_writel(0x66, TCI6614_DATA0_WRLVL_INIT_RATIO); //byte-lane 7 write leveling init value
	__raw_writel(0x53, TCI6614_DATA1_WRLVL_INIT_RATIO); //byte-lane 6 write leveling init value
	__raw_writel(0x58, TCI6614_DATA2_WRLVL_INIT_RATIO); //byte-lane 5 write leveling init value
	__raw_writel(0x54, TCI6614_DATA3_WRLVL_INIT_RATIO); //byte-lane 4 write leveling init value
	__raw_writel(0x32, TCI6614_DATA4_WRLVL_INIT_RATIO); //byte-lane 3 write leveling init value
	__raw_writel(0x39, TCI6614_DATA5_WRLVL_INIT_RATIO); //byte-lane 2 write leveling init value
	__raw_writel(0x22, TCI6614_DATA6_WRLVL_INIT_RATIO); //byte-lane 1 write leveling init value
	__raw_writel(0x19, TCI6614_DATA7_WRLVL_INIT_RATIO); //byte-lane 0 write leveling init value
	__raw_writel(0x48, TCI6614_DATA8_WRLVL_INIT_RATIO); //byte-lane ECC write leveling init value

	__raw_writel(0xDC, TCI6614_DATA0_GTLVL_INIT_RATIO); //byte-lane 7 gate leveling init value
	__raw_writel(0xEF, TCI6614_DATA1_GTLVL_INIT_RATIO); //byte-lane 6 gate leveling init value
	__raw_writel(0xCE, TCI6614_DATA2_GTLVL_INIT_RATIO); //byte-lane 5 gate leveling init value
	__raw_writel(0xD2, TCI6614_DATA3_GTLVL_INIT_RATIO); //byte-lane 4 gate leveling init value
	__raw_writel(0xBA, TCI6614_DATA4_GTLVL_INIT_RATIO); //byte-lane 3 gate leveling init value
	__raw_writel(0xB3, TCI6614_DATA5_GTLVL_INIT_RATIO); //byte-lane 2 gate leveling init value
	__raw_writel(0xAC, TCI6614_DATA6_GTLVL_INIT_RATIO); //byte-lane 1 gate leveling init value
	__raw_writel(0xB5, TCI6614_DATA7_GTLVL_INIT_RATIO); //byte-lane 0 gate leveling init value
	__raw_writel(0xC0, TCI6614_DATA8_GTLVL_INIT_RATIO); //byte-lane ECC gate leveling init value

	//Do a PHY reset. Toggle DDR_PHY_CTRL_1 bit 15 0->1->0
	tmp = __raw_readl(TCI6614_DDR_DDRPHYC);
	tmp &= ~(0x00008000);
	__raw_writel(tmp, TCI6614_DDR_DDRPHYC);

	tmp = __raw_readl(TCI6614_DDR_DDRPHYC);
	tmp |= (0x00008000);
	__raw_writel(tmp, TCI6614_DDR_DDRPHYC);

	tmp = __raw_readl(TCI6614_DDR_DDRPHYC);
	tmp &= ~(0x00008000);
	__raw_writel(tmp, TCI6614_DDR_DDRPHYC);

	/***************** 2.3 Basic Controller and DRAM configuration ************/
	__raw_writel(0x00005162, TCI6614_DDR_SDRFC);	/* enable configuration */

	__raw_writel(0x1113783C, TCI6614_DDR_SDTIM1);
	__raw_writel(0x304F7FE3, TCI6614_DDR_SDTIM2);
	__raw_writel(0x559F849F, TCI6614_DDR_SDTIM3);

	__raw_writel(0x0010010F, TCI6614_DDR_DDRPHYC);

	__raw_writel(0x70073214, TCI6614_DDR_ZQCFG);

	__raw_writel(0x0, TCI6614_DDR_PMCTL);

	__raw_writel(0x63062A32, TCI6614_DDR_SDCFG); /* last config write DRAM init occurs */

	__raw_writel(0x00001450, TCI6614_DDR_SDRFC); /* Refresh rate = (7.8*666MHz) */

	__raw_writel(0x80000000, TCI6614_DDR_RDWR_LVL_RMP_CTRL); /* enable full leveling */
	__raw_writel(0x80000000, TCI6614_DDR_RDWR_LVL_CTRL); /* Trigger full leveling - This ignores read DQS
															leveling result and uses ratio forced value  */

	return 0;
}

#if defined(CONFIG_BOARD_EARLY_INIT_F)
int board_early_init_f(void)
{
	init_plls(ARRAY_SIZE(pll_config), pll_config);
	init_ddr3();

	return 0;
}
#endif

int mpu_enable_user_mode_access(void)
{
	unsigned int reg_addr, i, count;

	reg_addr = 0x2368208;
	count = 5;

	for(i = 1; i <= count; i++) {
	    __raw_writel(0x3FFFFFF, reg_addr);
		reg_addr += 0x10;
    }

	reg_addr = 0x2370208;
	count = 16;

	for(i = 1; i <= count; i++) {
	    __raw_writel(0x3FFFFFF, reg_addr);
		reg_addr += 0x10;
    }

	reg_addr = 0x2378208;
	count = 1;

	for(i = 1; i <= count; i++) {
	    __raw_writel(0x3FFFFFF, reg_addr);
		reg_addr += 0x10;
    }

    return 0;
}

int board_init(void)
{
	unsigned int tmp;
	char *s;

	gd->bd->bi_arch_number = MACH_TYPE_TCI6614_EVM;
	gd->bd->bi_boot_params = LINUX_BOOT_PARAM_ADDR;

	init_async_emif(ARRAY_SIZE(async_emif_config), async_emif_config);

	tmp = __raw_readl(TCI6614_DEVSTAT);

	/* Get the device endianness */
	if (!(tmp & 0x1))
		device_big_endian = 1;

	/* Set pll cfg to 122.88 MHz in devstat boot mode */
	tmp |= 0x3800;
	__raw_writel(tmp, TCI6614_DEVSTAT);

	/* Workaround an RBL problem to set the RAM table function pointer
	   to zero, so that GEM 0 will not tear down the linkram on exit */

	__raw_writel(0, 0x108ffeb4);

	/* Configure the MPU to enable user mode access */
#ifndef CONFIG_TCI6614_U_BOOT_MIN
	mpu_enable_user_mode_access();
#endif

	tmp = __raw_readl(JTAG_ID_REG);
	tmp &= 0xf0000000;
	tmp >>= 28; /* get the variant */
	if (!tmp) {
		/* Work around to fix the DDR corruption during ARM DSP IPC */
		__raw_writel(0, 0x02620580);
		__raw_writel(0, 0x21000054);
	}

	psc_disable_module(TCI6614_LPSC_SA);
	return 0;
}

int dram_init(void)
{
	gd->ram_size = PHYS_SDRAM_1_SIZE;

	return 0;
}

#ifdef CONFIG_NAND_DAVINCI
extern int tci6614_emac_initialize(void);
int board_nand_init(struct nand_chip *nand)
{
	davinci_nand_init(nand);

	return 0;
}
#endif

#ifdef CONFIG_DRIVER_TI_KEYSTONE_NET
int board_eth_init(bd_t *bis)
{
	tci6614_emac_initialize();

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
