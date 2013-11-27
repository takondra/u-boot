/*
 * Copyright (C) 2013 Texas Instruments Inc.
 *
 * Keystone2: DDR3 initialization
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

#include <asm/arch/hardware.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/clock_defs.h>

/************************* *****************************/
static struct ddr3_phy_config ddr3phy_1600_64A = {
	.pllcr		= 0x0001C000ul,
	.pgcr1_mask	= (IODDRM_MASK | ZCKSEL_MASK),
	.pgcr1_val	= ((1 << 2) | (1 << 7) | (1 << 23)),
	.ptr0		= 0x42C21590ul,
	.ptr1		= 0xD05612C0ul,
	.ptr2		= 0, /* not set in gel */
	.ptr3		= 0x0D861A80ul,
	.ptr4		= 0x0C827100ul,
	.dcr_mask	= (PDQ_MASK | MPRDQ_MASK | BYTEMASK_MASK),
	.dcr_val	= ((1 << 10)),
	.dtpr0		= 0xA19DBB66ul,
	.dtpr1		= 0x32868300ul,
	.dtpr2		= 0x50035200ul,
	.mr0		= 0x00001C70ul,
	.mr1		= 0x00000006ul,
	.mr2		= 0x00000018ul,
	.dtcr		= 0x730035C7ul,
	.pgcr2		= 0x00F07A12ul,
	.zq0cr1		= 0x0000005Dul,
	.zq1cr1		= 0x0000005Bul,
	.zq2cr1		= 0x0000005Bul,
	.pir_v1		= 0x00000033ul,
	.pir_v2		= 0x0000FF81ul,
};

static struct ddr3_emif_config ddr3_1600_64 = {
	.sdcfg		= 0x6200CE6Aul,
	.sdtim1		= 0x16709C55ul,
	.sdtim2		= 0x00001D4Aul,
	.sdtim3		= 0x435DFF54ul,
	.sdtim4		= 0x553F0CFFul,
	.zqcfg		= 0xF0073200ul,
	.sdrfc		= 0x00001869ul,
};

static struct ddr3_phy_config ddr3phy_1600_32 = {
	.pllcr		= 0x0001C000ul,
	.pgcr1_mask	= (IODDRM_MASK | ZCKSEL_MASK),
	.pgcr1_val	= ((1 << 2) | (1 << 7) | (1 << 23)),
	.ptr0		= 0x42C21590ul,
	.ptr1		= 0xD05612C0ul,
	.ptr2		= 0, /* not set in gel */
	.ptr3		= 0x0D861A80ul,
	.ptr4		= 0x0C827100ul,
	.dcr_mask	= (PDQ_MASK | MPRDQ_MASK | BYTEMASK_MASK),
	.dcr_val	= ((1 << 10)),
	.dtpr0		= 0xA19DBB66ul,
	.dtpr1		= 0x32868300ul,
	.dtpr2		= 0x50035200ul,
	.mr0		= 0x00001C70ul,
	.mr1		= 0x00000006ul,
	.mr2		= 0x00000018ul,
	.dtcr		= 0x730035C7ul,
	.pgcr2		= 0x00F07A12ul,
	.zq0cr1		= 0x0000005Dul,
	.zq1cr1		= 0x0000005Bul,
	.zq2cr1		= 0x0000005Bul,
	.pir_v1		= 0x00000033ul,
	.pir_v2		= 0x0000FF81ul,
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

/************************* *****************************/
static struct ddr3_phy_config ddr3phy_1333_64A = {
	.pllcr		= 0x0005C000ul,
	.pgcr1_mask	= (IODDRM_MASK | ZCKSEL_MASK),
	.pgcr1_val	= ((1 << 2) | (1 << 7) | (1 << 23)),
	.ptr0		= 0x42C21590ul,
	.ptr1		= 0xD05612C0ul,
	.ptr2		= 0, /* not set in gel */
	.ptr3		= 0x0B4515C2ul,
	.ptr4		= 0x0A6E08B4ul,
	.dcr_mask	= (PDQ_MASK | MPRDQ_MASK | BYTEMASK_MASK),
	.dcr_val	= ((1 << 10)),
	.dtpr0		= 0x8558AA55ul,
	.dtpr1		= 0x32857280ul,
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

static struct ddr3_emif_config ddr3_1333_64 = {
	.sdcfg		= 0x62008C62ul,
	.sdtim1		= 0x125C8044ul,
	.sdtim2		= 0x00001D29ul,
	.sdtim3		= 0x32CDFF43ul,
	.sdtim4		= 0x543F0ADFul,
	.zqcfg		= 0x70073200ul,
	.sdrfc		= 0x00001457ul,
};

static struct ddr3_phy_config ddr3phy_1333_32 = {
	.pllcr		= 0x0005C000ul,
	.pgcr1_mask	= (IODDRM_MASK | ZCKSEL_MASK),
	.pgcr1_val	= ((1 << 2) | (1 << 7) | (1 << 23)),
	.ptr0		= 0x42C21590ul,
	.ptr1		= 0xD05612C0ul,
	.ptr2		= 0, /* not set in gel */
	.ptr3		= 0x0B4515C2ul,
	.ptr4		= 0x0A6E08B4ul,
	.dcr_mask	= (PDQ_MASK | MPRDQ_MASK | BYTEMASK_MASK),
	.dcr_val	= ((1 << 10)),
	.dtpr0		= 0x8558AA55ul,
	.dtpr1		= 0x32857280ul,
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

static struct ddr3_emif_config ddr3_1333_32 = {
	.sdcfg		= 0x62009C62ul,
	.sdtim1		= 0x125C8044ul,
	.sdtim2		= 0x00001D29ul,
	.sdtim3		= 0x32CDFF43ul,
	.sdtim4		= 0x543F0ADFul,
	.zqcfg		= 0x70073200ul,
	.sdrfc		= 0x00001457ul,
};

/************************* *****************************/
static struct ddr3_phy_config ddr3phy_1333_64 = {
	.pllcr		= 0x0005C000ul,
	.pgcr1_mask	= (IODDRM_MASK | ZCKSEL_MASK),
	.pgcr1_val	= ((1 << 2) | (1 << 7) | (1 << 23)),
	.ptr0		= 0x42C21590ul,
	.ptr1		= 0xD05612C0ul,
	.ptr2		= 0, /* not set in gel */
	.ptr3		= 0x0B4515C2ul,
	.ptr4		= 0x0A6E08B4ul,
	.dcr_mask	= (PDQ_MASK | MPRDQ_MASK | BYTEMASK_MASK),
	.dcr_val	= ((1 << 10)),
	.dtpr0		= 0x8558AA55ul,
	.dtpr1		= 0x32857280ul,
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
/************************* *****************************/

void init_ddrphy(u32 base, struct ddr3_phy_config *phy_cfg)
{
	unsigned int tmp;

	while((__raw_readl(base + TCI6638_DDRPHY_PGSR0_OFFSET)
		 & 0x00000001) != 0x00000001);

	__raw_writel(phy_cfg->pllcr, base + TCI6638_DDRPHY_PLLCR_OFFSET);

	tmp = __raw_readl(base + TCI6638_DDRPHY_PGCR1_OFFSET);
	tmp &= ~(phy_cfg->pgcr1_mask);
	tmp |= phy_cfg->pgcr1_val;
        __raw_writel(tmp, base + TCI6638_DDRPHY_PGCR1_OFFSET);

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

/*********** Begin DDR Reset Workaround ***********/
void ddr_reset_workaround(void)
{

	unsigned int tmp, tmp_a, tmp_b;

	/*
	 * Check for PGSR0 error bits of DDR3 PHY.
	 * Check for WLERR, QSGERR, WLAERR,
	 * RDERR, WDERR, REERR, WEERR error to see if they are set or not
	 */
	tmp_a = __raw_readl(TCI6638_DDR3A_DDRPHYC +
				TCI6638_DDRPHY_PGSR0_OFFSET);
	tmp_b = __raw_readl(TCI6638_DDR3B_DDRPHYC +
				TCI6638_DDRPHY_PGSR0_OFFSET);

	if (((tmp_a & 0x0FE00000) != 0) || ((tmp_b & 0x0FE00000) != 0)) {

		printf("DDR Leveling Error Detected!\n");
		printf("DDR3A PGSR0 = 0x%x\n", tmp_a);
		printf("DDR3B PGSR0 = 0x%x\n", tmp_b);

		/*
		 * Write Keys to KICK registers to enable writes to registers
		 * in boot config space
		 */
		__raw_writel(KEYSTONE_KICK0_MAGIC, KEYSTONE_KICK0);
		__raw_writel(KEYSTONE_KICK1_MAGIC, KEYSTONE_KICK1);

		/*
		 * Move DDR3A Module out of reset isolation by setting
		 * MDCTL23[12] = 0
		 */
		tmp_a = __raw_readl(TCI6638_PSC_BASE + 0xA5C);
		tmp_a &= ~(0x1000);
		__raw_writel(tmp_a, TCI6638_PSC_BASE + 0xA5C);

		/*
		 * Move DDR3B Module out of reset isolation by setting
		 * MDCTL24[12] = 0
		 */
		 tmp_b = __raw_readl(TCI6638_PSC_BASE + 0xA60);
		 tmp_b &= ~(0x1000);
		__raw_writel(tmp_b, TCI6638_PSC_BASE + 0xA60);

		/*
		 * Write 0x5A69 Key to RSTCTRL[15:0] to unlock writes
		 * to RSTCTRL and RSTCFG
		 */
		tmp = __raw_readl(TCI6638_PLL_CNTRL_BASE +
					MAIN_PLL_CTRL_RSTCTRL);
		tmp &= ~(0xFFFF);
		tmp |= 0x5A69;
		__raw_writel(tmp, TCI6638_PLL_CNTRL_BASE +
					MAIN_PLL_CTRL_RSTCTRL);

		/*
		 * Set PLL Controller to drive hard reset on SW trigger by
		 * setting RSTCFG[13] = 0
		 */
		tmp = __raw_readl(TCI6638_PLL_CNTRL_BASE +
					MAIN_PLL_CTRL_RSTCFG);
		tmp &= ~(0x2000);
		tmp |= 0x0000;
		__raw_writel(tmp, TCI6638_PLL_CNTRL_BASE +
					MAIN_PLL_CTRL_RSTCFG);

		/*
		 * Write 0x5A69 Key to RSTCTRL[15:0] to unlock writes to
		 * RSTCTRL and RSTCFG
		 */
		tmp = __raw_readl(TCI6638_PLL_CNTRL_BASE +
					MAIN_PLL_CTRL_RSTCTRL);
		tmp &= ~(0xFFFF);
		tmp |= 0x5A69;
		__raw_writel(tmp, TCI6638_PLL_CNTRL_BASE +
					MAIN_PLL_CTRL_RSTCTRL);

		/*
		 * Write RSTCTRL[16] = 0 to initiate software reset via PLL
		 * controller
		 */
		tmp = __raw_readl(TCI6638_PLL_CNTRL_BASE +
					MAIN_PLL_CTRL_RSTCTRL);
		tmp &= ~(0x10000);
		tmp = 0x00000;
		__raw_writel(tmp, TCI6638_PLL_CNTRL_BASE +
					MAIN_PLL_CTRL_RSTCTRL);

		/* Wait for Reset */
		for(;;);
	}
}

int get_dimm_params(char *dimm_name)
{
	u8 spd_params[256];
	int ret, y, x;
	int old_bus;

	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);

	old_bus = i2c_get_bus_num();
	i2c_set_bus_num(1);

	ret = i2c_read(0x53, 0, 0, spd_params, 256);

	i2c_set_bus_num(old_bus);

	dimm_name[0] = '\0';

	if (ret) {
		puts("Cannot read DIMM params\n");
		return 1;
	}

#if 0
	for (y=0; y<16; y++) {
		for (x=0; x<16; x++)
			printf("%02x ", spd_params[y*16+x]);
		printf("   ");

		for (x=0; x<16; x++)
			if ((spd_params[y*16+x] >= 0x20) &&
			    (spd_params[y*16+x] <= 0x7e))
				printf("%c", spd_params[y*16+x]);
			else
				printf(".");
		printf("\n");
	}
#endif
	/*
	 * We need to convert spd data to dimm parameters
	 * and to DDR3 EMIF and PHY regirsters values.
	 * For now we just return DIMM type string value.
	 * Caller may use this value to choose appropriate
	 * a pre-set DDR3 configuration
	 */

	strncpy(dimm_name, &spd_params[0x80], 18);
	dimm_name[18] = '\0';

	return 0;
}

struct pll_init_data ddr3a_333 = DDR3_PLL_333(A);
struct pll_init_data ddr3b_333 = DDR3_PLL_333(B);
struct pll_init_data ddr3a_400 = DDR3_PLL_400(A);
struct pll_init_data ddr3b_400 = DDR3_PLL_400(B);

void init_ddr3( void ) {

	char dimm_name[32];

	if(~(__raw_readl(TCI6638_PLL_CNTRL_BASE + MAIN_PLL_CTRL_RSTYPE) & 0x1)) {
		if (cpu_revision() > 0)
			init_pll(&ddr3a_400);
		else
			init_pll(&ddr3a_333);
		init_pll(&ddr3b_333);
	}

	get_dimm_params(dimm_name);

	printf("Detected SO-DIMM [%s]\n", dimm_name);

	if (!strcmp(dimm_name, "18KSF1G72HZ-1G6E2 ")) {
		/* 8G SO-DIMM */
		if (cpu_revision() > 0) {
			init_ddrphy(TCI6638_DDR3A_DDRPHYC, &ddr3phy_1600_64A);
			init_ddremif(TCI6638_DDR3A_EMIF_CTRL_BASE, &ddr3_1600_64);
			printf("DRAM:  Capacity 8 GiB (includes reported below)\n");
		}
		else {
			init_ddrphy(TCI6638_DDR3A_DDRPHYC, &ddr3phy_1600_32);
			init_ddremif(TCI6638_DDR3A_EMIF_CTRL_BASE, &ddr3_1600_32);
			printf("DRAM:  Capacity 4 GiB (includes reported below)\n");
		}
	}
	else {
		/* 2G SO-DIMM */
		if (cpu_revision() > 0) {
			init_ddrphy(TCI6638_DDR3A_DDRPHYC, &ddr3phy_1333_64A);
			init_ddremif(TCI6638_DDR3A_EMIF_CTRL_BASE, &ddr3_1333_64);
		}
		else {
			init_ddrphy(TCI6638_DDR3A_DDRPHYC, &ddr3phy_1333_32);
			init_ddremif(TCI6638_DDR3A_EMIF_CTRL_BASE, &ddr3_1333_32);
		}
	}

	init_ddrphy(TCI6638_DDR3B_DDRPHYC, &ddr3phy_1333_64);
	init_ddremif(TCI6638_DDR3B_EMIF_CTRL_BASE, &ddr3_1333_64);

	/* Apply the workaround for PG 1.0 and 1.1 Silicons */
	if (cpu_revision() <= 1)
		ddr_reset_workaround();
}

