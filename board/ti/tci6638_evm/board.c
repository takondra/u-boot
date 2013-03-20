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

#include <asm/arch/hardware.h>
#include <asm/arch/clock.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/arch/nand_defs.h>


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

#define CORE_PLL_799	{ CORE_PLL,	13,	1,	2 }
#define CORE_PLL_983	{ CORE_PLL,	16,	1,	2 }
#define CORE_PLL_1228	{ CORE_PLL,	20,	1,	2 }
#define PASS_PLL_1228	{ PASS_PLL,	20,	1,	2 }
#define PASS_PLL_983	{ PASS_PLL,	16,	1,	2 }
#define TETRIS_PLL_500  { TETRIS_PLL,	8,	1,	2 }
#define TETRIS_PLL_750  { TETRIS_PLL,	12,	1,	2 }
#define TETRIS_PLL_687  { TETRIS_PLL,	11,	1,	2 }
#define TETRIS_PLL_625  { TETRIS_PLL,	10,	1,	2 }
#define TETRIS_PLL_812  { TETRIS_PLL,	13,	1,	2 }
#define TETRIS_PLL_875  { TETRIS_PLL,	14,	1,	2 }
#define TETRIS_PLL_1375 { TETRIS_PLL,	22,	2,	1 }
#define DDR3_PLL_200(x)	{ DDR3##x##_PLL,4,	1,	2 }
#define DDR3_PLL_400(x)	{ DDR3##x##_PLL,8,	1,	2 }
#define DDR3_PLL_800(x)	{ DDR3##x##_PLL,16,	1,	2 }
#define DDR3_PLL_333(x)	{ DDR3##x##_PLL,20,	1,	6 }

static struct pll_init_data pll_config[] = {
	CORE_PLL_799,
	PASS_PLL_983,
	TETRIS_PLL_625,
	DDR3_PLL_333(A),
	DDR3_PLL_333(B)
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

static struct ddr3_emif_config ddr3_1600_64 = {
	.sdcfg		= 0x6200CE62ul,
	.sdtim1		= 0x16709C55ul,
	.sdtim2		= 0x00001D4Aul,
	.sdtim3		= 0x435DFF54ul,
	.sdtim4		= 0x553F0CFFul,
	.zqcfg		= 0xF0073200ul,
	.sdrfc		= 0x00001869ul,
};

static struct ddr3_emif_config ddr3_1600_32 = {
	.sdcfg		= 0x6200DE6Aul,
	.sdtim1		= 0x16709C55ul,
	.sdtim2		= 0x00001D4Aul,
	.sdtim3		= 0x435DFF54ul,
	.sdtim4		= 0x553F0CFFul,
	.zqcfg		= 0xF0073200ul,
	.sdrfc		= 0x00001869ul,
};

static struct ddr3_emif_config ddr3_1333_64 = {
	.sdcfg		= 0x62008C62ul,
	.sdtim1		= 0x125C8044ul,
	.sdtim2		= 0x00001D29ul,
	.sdtim3		= 0x32CDFF43ul,
	.sdtim4		= 0x543F0ADFul,
	.zqcfg		= 0xF0073200ul,
	.sdrfc		= 0x00001457ul,
};

static struct ddr3_emif_config ddr3_1333_32 = {
	.sdcfg		= 0x62009C62ul,
	.sdtim1		= 0x125C8044ul,
	.sdtim2		= 0x00001D29ul,
	.sdtim3		= 0x32CDFF43ul,
	.sdtim4		= 0x543F0ADFul,
	.zqcfg		= 0x70073200ul,
	.sdrfc		= 0x00001457ul,
};

static struct ddr3_emif_config ddr3_1066_64 = {
	.sdcfg		= 0x62004662ul,
	.sdtim1		= 0x0E4C6833ul,
	.sdtim2		= 0x00001CE7ul,
	.sdtim3		= 0x323DFF32ul,
	.sdtim4		= 0x533F08AFul,
	.zqcfg		= 0xF0073200ul,
	.sdrfc		= 0x00001044ul,
};

static struct ddr3_emif_config ddr3_1066_32 = {
	.sdcfg		= 0x62005662ul,
	.sdtim1		= 0x0E4C6833ul,
	.sdtim2		= 0x00001CE7ul,
	.sdtim3		= 0x323DFF32ul,
	.sdtim4		= 0x533F08AFul,
	.zqcfg		= 0xF0073200ul,
	.sdrfc		= 0x00001044ul,
};

static struct ddr3_emif_config ddr3_200_64 = {
	.sdcfg		= 0x62000462ul,
	.sdtim1		= 0x0A384C23ul,
	.sdtim2		= 0x00001CA5ul,
	.sdtim3		= 0x21ADFF32ul,
	.sdtim4		= 0x533F067Ful,
	.zqcfg		= 0xF0073200ul,
	.sdrfc		= 0x00000C34ul,
};

static struct ddr3_emif_config ddr3_200_32 = {
	.sdcfg		= 0x62001462ul,
	.sdtim1		= 0x0A384C23ul,
	.sdtim2		= 0x00001CA5ul,
	.sdtim3		= 0x21ADFF32ul,
	.sdtim4		= 0x533F067Ful,
	.zqcfg		= 0xF0073200ul,
	.sdrfc		= 0x00000C34ul,
};

static struct ddr3_phy_config ddr3phy_1600_64 = {
	.pllcr		= 0x0001C000ul,
	.pgcr1_mask	= (IODDRM_MASK | ZCKSEL_MASK | ZCKSEL_MASK),
	.pgcr1_val	= ((1 << 2) | (1 << 7) | (1 << 23)),
	.ptr0		= 0x42C21590ul,
	.ptr1		= 0xD05612C0ul,
	.ptr2		= 0, /* not set in gel */
	.ptr3		= 0x0D861A80ul,
	.ptr4		= 0x0C827100ul,
	.dcr_mask	= (PDQ_MASK | MPRDQ_MASK | BYTEMASK_MASK | NOSRA_MASK),
	.dcr_val	= ((1 << 10) | (1 << 27)),
	.dtpr0		= 0xA19DBB66ul,
	.dtpr1		= 0x12868300ul,
	.dtpr2		= 0x50035200ul,
	.mr0		= 0x00001C70ul,
	.mr1		= 0x00000006ul,
	.mr2		= 0x00000018ul,
	.dtcr		= 0x710035C7ul,
	.pgcr2		= 0x00F07A12ul,
	.zq0cr1		= 0x0000005Dul,
	.zq1cr1		= 0x0000005Bul,
	.zq2cr1		= 0x0000005Bul,
	.pir_v1		= 0x00000033ul,
	.pir_v2		= 0x0000FF81ul,
};

static struct ddr3_phy_config ddr3phy_1600_32 = {
	.pllcr		= 0x0001C000ul,
	.pgcr1_mask	= (IODDRM_MASK | ZCKSEL_MASK | ZCKSEL_MASK),
	.pgcr1_val	= ((1 << 2) | (1 << 7) | (1 << 23)),
	.ptr0		= 0x42C21590ul,
	.ptr1		= 0xD05612C0ul,
	.ptr2		= 0, /* not set in gel */
	.ptr3		= 0x0D861A80ul,
	.ptr4		= 0x0C827100ul,
	.dcr_mask	= (PDQ_MASK | MPRDQ_MASK | BYTEMASK_MASK | NOSRA_MASK | UDIMM_MASK),
	.dcr_val	= ((1 << 10) | (1 << 27) | (1 << 29)),
	.dtpr0		= 0xA19DBB66ul,
	.dtpr1		= 0x12868300ul,
	.dtpr2		= 0x50035200ul,
	.mr0		= 0x00001C70ul,
	.mr1		= 0x00000006ul,
	.mr2		= 0x00000018ul,
	.dtcr		= 0x710035C7ul,
	.pgcr2		= 0x00F07A12ul,
	.zq0cr1		= 0x0000005Dul,
	.zq1cr1		= 0x0000005Bul,
	.zq2cr1		= 0x0000005Bul,
	.pir_v1		= 0x00000033ul,
	.pir_v2		= 0x0000FF81ul,
};

static struct ddr3_phy_config ddr3phy_1333_64 = {
	.pllcr		= 0x0005C000ul,
	.pgcr1_mask	= (IODDRM_MASK | ZCKSEL_MASK | ZCKSEL_MASK),
	.pgcr1_val	= ((1 << 2) | (1 << 7) | (1 << 23)),
	.ptr0		= 0x42C21590ul,
	.ptr1		= 0xD05612C0ul,
	.ptr2		= 0, /* not set in gel */
	.ptr3		= 0x0B4515C2ul,
	.ptr4		= 0x0A6E08B4ul,
	.dcr_mask	= (PDQ_MASK | MPRDQ_MASK | BYTEMASK_MASK | NOSRA_MASK),
	.dcr_val	= ((1 << 10) | (1 << 27)),
	.dtpr0		= 0x8558AA55ul,
	.dtpr1		= 0x12857280ul,
	.dtpr2		= 0x5002C200ul,
	.mr0		= 0x00001A60ul,
	.mr1		= 0x00000006ul,
	.mr2		= 0x00000010ul,
	.dtcr		= 0x710035C7ul,
	.pgcr2		= 0x00F065B8ul,
	.zq0cr1		= 0x0000005Dul,
	.zq1cr1		= 0x0000005Bul,
	.zq2cr1		= 0x0000005Bul,
	.pir_v1		= 0x00000033ul,
	.pir_v2		= 0x0000FF81ul,
};

static struct ddr3_phy_config ddr3phy_1333_32 = {
	.pllcr		= 0x0005C000ul,
	.pgcr1_mask	= (IODDRM_MASK | ZCKSEL_MASK | ZCKSEL_MASK),
	.pgcr1_val	= ((1 << 2) | (1 << 7) | (1 << 23)),
	.ptr0		= 0x42C21590ul,
	.ptr1		= 0xD05612C0ul,
	.ptr2		= 0, /* not set in gel */
	.ptr3		= 0x0B4515C2ul,
	.ptr4		= 0x0A6E08B4ul,
	.dcr_mask	= (PDQ_MASK | MPRDQ_MASK | BYTEMASK_MASK | NOSRA_MASK | UDIMM_MASK),
	.dcr_val	= ((1 << 10) | (1 << 27) | (1 << 29)),
	.dtpr0		= 0x8558AA55ul,
	.dtpr1		= 0x12857280ul,
	.dtpr2		= 0x5002C200ul,
	.mr0		= 0x00001A60ul,
	.mr1		= 0x00000006ul,
	.mr2		= 0x00000010ul,
	.dtcr		= 0x710035C7ul,
	.pgcr2		= 0x00F065B8ul,
	.zq0cr1		= 0x0000005Dul,
	.zq1cr1		= 0x0000005Bul,
	.zq2cr1		= 0x0000005Bul,
	.pir_v1		= 0x00000033ul,
	.pir_v2		= 0x0000FF81ul,
};

static struct ddr3_phy_config ddr3phy_1066_64 = {
	.pllcr		= 0x000DC000ul,
	.pgcr1_mask	= (IODDRM_MASK | ZCKSEL_MASK | ZCKSEL_MASK),
	.pgcr1_val	= ((1 << 2) | (1 << 7) | (1 << 23)),
	.ptr0		= 0x42C21590ul,
	.ptr1		= 0xD05612C0ul,
	.ptr2		= 0, /* not set in gel */
	.ptr3		= 0x09041104ul,
	.ptr4		= 0x0855A068ul,
	.dcr_mask	= (PDQ_MASK | MPRDQ_MASK | BYTEMASK_MASK | NOSRA_MASK),
	.dcr_val	= ((1 << 10) | (1 << 27)),
	.dtpr0		= 0x6D148844ul,
	.dtpr1		= 0x12845A00ul,
	.dtpr2		= 0x50023600ul,
	.mr0		= 0x00001830ul,
	.mr1		= 0x00000006ul,
	.mr2		= 0x00000008ul,
	.dtcr		= 0x710035C7ul,
	.pgcr2		= 0x00F05159ul,
	.zq0cr1		= 0x0000005Dul,
	.zq1cr1		= 0x0000005Bul,
	.zq2cr1		= 0x0000005Bul,
	.pir_v1		= 0x00000033ul,
	.pir_v2		= 0x0000FF81ul,
};

static struct ddr3_phy_config ddr3phy_1066_32 = {
	.pllcr		= 0x000DC000ul,
	.pgcr1_mask	= (IODDRM_MASK | ZCKSEL_MASK | ZCKSEL_MASK),
	.pgcr1_val	= ((1 << 2) | (1 << 7) | (1 << 23)),
	.ptr0		= 0x42C21590ul,
	.ptr1		= 0xD05612C0ul,
	.ptr2		= 0, /* not set in gel */
	.ptr3		= 0x09041104ul,
	.ptr4		= 0x0855A068ul,
	.dcr_mask	= (PDQ_MASK | MPRDQ_MASK | BYTEMASK_MASK | NOSRA_MASK | UDIMM_MASK),
	.dcr_val	= ((1 << 10) | (1 << 27) | (1 << 29)),
	.dtpr0		= 0x6D148844ul,
	.dtpr1		= 0x12845A00ul,
	.dtpr2		= 0x50023600ul,
	.mr0		= 0x00001870ul,
	.mr1		= 0x00000044ul,
	.mr2		= 0x00000018ul,
	.dtcr		= 0x710035C7ul,
	.pgcr2		= 0x00F05159ul,
	.zq0cr1		= 0x0000005Dul,
	.zq1cr1		= 0x0000005Bul,
	.zq2cr1		= 0x0000005Bul,
	.pir_v1		= 0x00000033ul,
	.pir_v2		= 0x0000FF81ul,
};

static struct ddr3_phy_config ddr3phy_200_64 = {
	.pllcr		= 0x000DC000ul,
	.pgcr1_mask	= (IODDRM_MASK | ZCKSEL_MASK | ZCKSEL_MASK),
	.pgcr1_val	= ((1 << 2) | (1 << 7) | (1 << 23)),
	.ptr0		= 0x42C21590ul,
	.ptr1		= 0xD05612C0ul,
	.ptr2		= 0, /* not set in gel */
	.ptr3		= 0x06C30D40ul,
	.ptr4		= 0x06413880ul,
	.dcr_mask	= (PDQ_MASK | MPRDQ_MASK | BYTEMASK_MASK | NOSRA_MASK),
	.dcr_val	= ((1 << 10) | (1 << 27)),
	.dtpr0		= 0x50CE6644ul,
	.dtpr1		= 0x12834180ul,
	.dtpr2		= 0x50022A00ul,
	.mr0		= 0x00001420ul,
	.mr1		= 0x00000006ul,
	.mr2		= 0x00000000ul,
	.dtcr		= 0x710035C7ul,
	.pgcr2		= 0x00F03D09ul,
	.zq0cr1		= 0x0000005Dul,
	.zq1cr1		= 0x0000005Bul,
	.zq2cr1		= 0x0000005Bul,
	.pir_v1		= 0x00000033ul,
	.pir_v2		= 0x00000F81ul,
};

static struct ddr3_phy_config ddr3phy_200_32 = {
	.pllcr		= 0x000DC000ul,
	.pgcr1_mask	= (IODDRM_MASK | ZCKSEL_MASK | ZCKSEL_MASK),
	.pgcr1_val	= ((1 << 2) | (1 << 7) | (1 << 23)),
	.ptr0		= 0x42C21590ul,
	.ptr1		= 0xD05612C0ul,
	.ptr2		= 0, /* not set in gel */
	.ptr3		= 0x06C30D40ul,
	.ptr4		= 0x06413880ul,
	.dcr_mask	= (PDQ_MASK | MPRDQ_MASK | BYTEMASK_MASK | NOSRA_MASK | UDIMM_MASK),
	.dcr_val	= ((1 << 10) | (1 << 27) | (1 << 29)),
	.dtpr0		= 0x50CF6644ul,
	.dtpr1		= 0x12834180ul,
	.dtpr2		= 0x50022A00ul,
	.mr0		= 0x00001420ul,
	.mr1		= 0x00000006ul,
	.mr2		= 0x00000000ul,
	.dtcr		= 0x710035C7ul,
	.pgcr2		= 0x00F03D09ul,
	.zq0cr1		= 0x0000005Dul,
	.zq1cr1		= 0x0000005Bul,
	.zq2cr1		= 0x0000005Bul,
	.pir_v1		= 0x00000033ul,
	.pir_v2		= 0x00000F81ul,
};

void init_ddrphy(u32 base, struct ddr3_phy_config *phy_cfg)
{
	unsigned int tmp;

	while((__raw_readl(base + TCI6638_DDRPHY_PGSR0_OFFSET)
		 & 0x00000001) != 0x00000001);

	__raw_writel(phy_cfg->pllcr, base + TCI6638_DDRPHY_PLLCR_OFFSET);

	tmp = __raw_readl(base + TCI6638_DDRPHY_PGCR1_OFFSET);
	tmp &= ~(phy_cfg->pgcr1_mask);
	tmp |= phy_cfg->pgcr1_val;
        __raw_writel(tmp, TCI6638_DDRPHY_PGCR1_OFFSET);

	__raw_writel(phy_cfg->ptr0,   base + TCI6638_DDRPHY_PTR0_OFFSET);
	__raw_writel(phy_cfg->ptr1,   base + TCI6638_DDRPHY_PTR1_OFFSET);
	__raw_writel(phy_cfg->ptr3,  base + TCI6638_DDRPHY_PTR3_OFFSET);
	__raw_writel(phy_cfg->ptr4,  base + TCI6638_DDRPHY_PTR4_OFFSET);

	tmp =  __raw_readl(base + TCI6638_DDRPHY_DCR_OFFSET);
	tmp &= ~(phy_cfg->dcr_mask);
	tmp |= phy_cfg->dcr_val;
	__raw_writel(tmp, base + TCI6638_DDRPHY_DCR_OFFSET);

	__raw_writel(phy_cfg->dtpr0, base + TCI6638_DDRPHY_DTPR0_OFFSET);
	__raw_writel(phy_cfg->dtpr1, base + TCI6638_DDRPHY_DTPR1_OFFSET);
	__raw_writel(phy_cfg->dtpr2, base + TCI6638_DDRPHY_DTPR2_OFFSET);
	__raw_writel(phy_cfg->mr0,   base + TCI6638_DDRPHY_MR0_OFFSET);
	__raw_writel(phy_cfg->mr1,   base + TCI6638_DDRPHY_MR1_OFFSET);
	__raw_writel(phy_cfg->mr2,   base + TCI6638_DDRPHY_MR2_OFFSET);
	__raw_writel(phy_cfg->dtcr,  base + TCI6638_DDRPHY_DTCR_OFFSET);
	__raw_writel(phy_cfg->pgcr2, base + TCI6638_DDRPHY_PGCR2_OFFSET);

	__raw_writel(phy_cfg->zq0cr1, base + TCI6638_DDRPHY_ZQ0CR1_OFFSET);
	__raw_writel(phy_cfg->zq1cr1, base + TCI6638_DDRPHY_ZQ1CR1_OFFSET);
	__raw_writel(phy_cfg->zq2cr1, base + TCI6638_DDRPHY_ZQ2CR1_OFFSET);

	__raw_writel(phy_cfg->pir_v1, base + TCI6638_DDRPHY_PIR_OFFSET);
	while((__raw_readl(base + TCI6638_DDRPHY_PGSR0_OFFSET) & 0x1) != 0x1);

	__raw_writel(phy_cfg->pir_v2, base + TCI6638_DDRPHY_PIR_OFFSET);
	while((__raw_readl(base + TCI6638_DDRPHY_PGSR0_OFFSET) & 0x1) != 0x1);

}

void init_ddremif(u32 base, struct ddr3_emif_config *emif_cfg)
{
	__raw_writel(emif_cfg->sdcfg,  base + TCI6638_DDR3_SDCFG_OFFSET );
	__raw_writel(emif_cfg->sdtim1, base + TCI6638_DDR3_SDTIM1_OFFSET);
	__raw_writel(emif_cfg->sdtim2, base + TCI6638_DDR3_SDTIM2_OFFSET);
	__raw_writel(emif_cfg->sdtim3, base + TCI6638_DDR3_SDTIM3_OFFSET);
	__raw_writel(emif_cfg->sdtim4, base + TCI6638_DDR3_SDTIM4_OFFSET);
	__raw_writel(emif_cfg->zqcfg,  base + TCI6638_DDR3_ZQCFG_OFFSET );
	__raw_writel(emif_cfg->sdrfc,  base + TCI6638_DDR3_SDRFC_OFFSET );
}

int dram_init(void)
{
	gd->ram_size = get_ram_size(CONFIG_SYS_SDRAM_BASE,
				    CONFIG_MAX_RAM_BANK_SIZE);
	init_async_emif(ARRAY_SIZE(async_emif_config), async_emif_config);
	return 0;
}

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

#if defined(CONFIG_BOARD_EARLY_INIT_F)
int board_early_init_f(void)
{
	init_plls(ARRAY_SIZE(pll_config), pll_config);

	init_ddrphy(TCI6638_DDR3A_DDRPHYC, &ddr3phy_1333_32);
	init_ddremif(TCI6638_DDR3A_EMIF_CTRL_BASE, &ddr3_1333_32);

	init_ddrphy(TCI6638_DDR3B_DDRPHYC, &ddr3phy_1333_64);
	init_ddremif(TCI6638_DDR3B_EMIF_CTRL_BASE, &ddr3_1333_64);

	return 0;
}
#endif

int board_init(void)
{

	gd->bd->bi_arch_number = -1; /* MACH_TYPE_TCI6638_EVM; */
	gd->bd->bi_boot_params = LINUX_BOOT_PARAM_ADDR;

	return 0;
}

#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
void ft_board_setup(void *blob, bd_t *bd)
{
	u64 start[CONFIG_NR_DRAM_BANKS];
	u64 size[CONFIG_NR_DRAM_BANKS];
	char name[32], *env, *endp;
	int bank, lpae;

	env = getenv("mem_lpae");
	lpae = env && simple_strtol(env, NULL, 0);

	for (bank = 0; bank < CONFIG_NR_DRAM_BANKS; bank++) {
		start[bank] = bd->bi_dram[bank].start;
		size[bank]  = bd->bi_dram[bank].size;

		/* adjust memory start address for LPAE */
		if (lpae) {
			start[bank] -=  0x80000000;
			start[bank] += 0x800000000;
		}

		/* reserve memory at start of bank */
		if (bank)
			sprintf(name, "mem_reserve_head%d", bank);
		else
			sprintf(name, "mem_reserve_head");
		env = getenv(name);
		if (env)
			start[bank] += ustrtoul(env, &endp, 0);

		/* reserve memory at end of bank if needed */
		if (bank)
			sprintf(name, "mem_reserve%d", bank);
		else
			sprintf(name, "mem_reserve");
		env = getenv(name);
		if (env)
			size[bank] -= ustrtoul(env, &endp, 0);
	}

	fdt_fixup_memory_banks(blob, start, size, CONFIG_NR_DRAM_BANKS);
}
#endif
