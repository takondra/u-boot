/*
 * Copyright (C) 2011 Texas Instruments.
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

static const struct pll_regs pll_regs[] = {
	[SYS_PLL]	= { TCI6614_MAINPLLCTL0, TCI6614_MAINPLLCTL1},
	[DDR3_PLL]	= { TCI6614_DDR3PLLCTL0, TCI6614_DDR3PLLCTL1},
	[PASS_PLL]	= { TCI6614_PASSPLLCTL0, TCI6614_PASSPLLCTL1},
};

struct lpsc_map {
	int	pll, div;
};

static int pll_div_offset[] = {
#define div_offset(reg)	offsetof(struct pllctl_regs, reg)
	div_offset(div1), div_offset(div2), div_offset(div3),
	div_offset(div4), div_offset(div5), div_offset(div6),
	div_offset(div7), div_offset(div8), div_offset(div9),
	div_offset(div10), div_offset(div11), div_offset(div12)
};

static unsigned long pll_div_mask[] = { 0xff };

/* Mappings from PLL+DIV to subsystem clocks */
#define sys_dsp_clk1	{SYS_PLL, 0}
#define sys_dsp_clk2	{SYS_PLL, 1}
#define sys_dsp_clk3	{SYS_PLL, 2}
#define sys_dsp_clk4	{SYS_PLL, 3}
#define sys_dsp_clk5	{SYS_PLL, 4}
#define sys_dsp_clk6	{SYS_PLL, 5}
#define sys_dsp_clk7	{SYS_PLL, 6}
#define sys_dsp_clk8	{SYS_PLL, 7}
#define sys_dsp_clk9	{SYS_PLL, 8}
#define sys_dsp_clk10	{SYS_PLL, 9}
#define sys_dsp_clk11	{SYS_PLL, 10}
#define sys_dsp_clk12	{SYS_PLL, 11}

#define sys_shared_clk	sys_dsp_clk7
#define sys_srio_clk	sys_dsp_clk10
#define sys_sr3_clk		sys_dsp_clk9

#define sys_dsp_clk		{SYS_PLL, 0}
#define pass_clk		{PASS_PLL, 0}
#define ddr3_clk		{DDR3_PLL, 0}

static const struct lpsc_map lpsc_clk_map[] = {
	[TCI6614_LPSC_SHARED]		= sys_shared_clk,
	[TCI6614_LPSC_SMARTREFLEX3]	= sys_sr3_clk,
	[TCI6614_LPSC_DDR_EMIF]		= ddr3_clk,
	[TCI6614_LPSC_TCP3E]		= sys_dsp_clk3,
	[TCI6614_LPSC_VCP2_A]		= sys_dsp_clk4,
	[TCI6614_LPSC_DEBUGSS_TRC]	= sys_dsp_clk3,
	[TCI6614_LPSC_TETB_TRC]		= sys_dsp_clk4,
	[TCI6614_LPSC_PA]		= pass_clk,
	[TCI6614_LPSC_CPGMAC]		= pass_clk,
	[TCI6614_LPSC_SA]		= pass_clk,
	[TCI6614_LPSC_PCIE]		= sys_dsp_clk4,
	[TCI6614_LPSC_SRIO]		= sys_sr3_clk,
	[TCI6614_LPSC_VUSR]		= sys_dsp_clk,
	[TCI6614_LPSC_UNUSED]		= sys_dsp_clk,
	[TCI6614_LPSC_MSMCSRAM]		= sys_dsp_clk3,
	[TCI6614_LPSC_RAC]		= sys_dsp_clk1,
	[TCI6614_LPSC_TAC]		= sys_dsp_clk4,
	[TCI6614_LPSC_FFTC]		= sys_dsp_clk4,
	[TCI6614_LPSC_DRFE]		= sys_dsp_clk4,
	[TCI6614_LPSC_TCP3D]		= sys_dsp_clk3,
	[TCI6614_LPSC_VCP2_B]		= sys_dsp_clk4,
	[TCI6614_LPSC_VCP2_C]		= sys_dsp_clk4,
	[TCI6614_LPSC_VCP2_D]		= sys_dsp_clk4,
	[TCI6614_LPSC_BCP]		= sys_dsp_clk4,
	[TCI6614_LPSC_GEM1]		= sys_dsp_clk1,
	[TCI6614_LPSC_RSA1]		= sys_dsp_clk1,
	[TCI6614_LPSC_GEM0]		= sys_dsp_clk1,
	[TCI6614_LPSC_RSA2]		= sys_dsp_clk1,
	[TCI6614_LPSC_ARM]		= sys_dsp_clk1,
	[TCI6614_LPSC_TCP2]		= sys_dsp_clk3,
	[TCI6614_LPSC_DXB]		= sys_dsp_clk,

};

static const unsigned long pll_ext_freq[] = {
	[SYS_PLL]  = CONFIG_PLL_SYS_EXT_FREQ,
	[PASS_PLL] = CONFIG_PLL_ETH_EXT_FREQ,
	[DDR3_PLL] = CONFIG_PLL_DDR_EXT_FREQ,
};

static unsigned long pll_freq_get(int pll)
{
	unsigned long mult = 1, prediv = 1, output_div = 2;
	unsigned long ret;
	u32 tmp, reg;

	switch (pll)	{
	case SYS_PLL:
		if (pllctl_reg_read(pll, ctl) & PLLCTL_PLLEN) {
			/* Bypass disabled */
			tmp = __raw_readl(TCI6614_MAINPLLCTL0);
			prediv = (tmp & 0x3f) + 1;
			mult = (((tmp & 0x7f000) >> 6) | (pllctl_reg_read(pll,
							mult) & 0x3f)) + 1;
		}
		break;

	case PASS_PLL:
		if (pll == DDR3_PLL)
			reg = TCI6614_DDR3PLLCTL0;
		else
			reg = TCI6614_PASSPLLCTL0;

		if (!(reg & 0x00800000)) {
			/* Bypass disabled */
			tmp = __raw_readl(reg);
			prediv = (tmp & 0x3f) + 1;
			mult = ((tmp >> 6) & 0x1fff) + 1;
		}
		break;
	case DDR3_PLL:
		/* Nothing to do for DDR PLL this is being done elsewhere. */
		break;
	default:
		break;

	}

	ret = (unsigned long)(pll_ext_freq[pll] * mult) / (prediv * output_div);

	return ret;
}

static unsigned long __pll_div_freq_get(int pll, unsigned int fpll,
					int div)
{
	int divider = 1;
	unsigned long divreg;

	divreg = __raw_readl((void *)pllctl_regs[pll] + pll_div_offset[div]);

	if (divreg & PLLDIV_ENABLE)
		divider = (divreg & pll_div_mask[pll]) + 1;

	return fpll / divider;
}

static unsigned long pll_div_freq_get(int pll, int div)
{
	unsigned int fpll = pll_freq_get(pll);

	if (pll == SYS_PLL)
		return __pll_div_freq_get(pll, fpll, div);
	else
		return fpll;
}

static void __pll_div_freq_set(int pll, unsigned int fpll, int div,
			       unsigned long hz)
{
	int divider = (fpll / hz - 1);

	divider &= pll_div_mask[pll];
	divider |= PLLDIV_ENABLE;

	__raw_writel(divider, (void *)pllctl_regs[pll] + pll_div_offset[div]);
	pllctl_reg_setbits(pll, alnctl, (1 << div));
	pllctl_reg_setbits(pll, dchange, (1 << div));
}

static unsigned long pll_div_freq_set(int pll, int div, unsigned long hz)
{
	unsigned int fpll = pll_freq_get(pll);

	__pll_div_freq_set(pll, fpll, div, hz);

	pllctl_reg_write(pll, cmd, 1);

	/* Wait until new divider takes effect */
	while (pllctl_reg_read(pll, stat) & 0x01);

	return __pll_div_freq_get(pll, fpll, div);
}

unsigned long clk_get_rate(unsigned int clk)
{
	return pll_div_freq_get(lpsc_clk_map[clk].pll, lpsc_clk_map[clk].div);
}

unsigned long clk_round_rate(unsigned int clk, unsigned long hz)
{
	unsigned long fpll, divider, pll;

	pll = lpsc_clk_map[clk].pll;
	fpll = pll_freq_get(pll);
	divider = (fpll / hz - 1);
	divider &= pll_div_mask[pll];

	return fpll / (divider + 1);
}

int clk_set_rate(unsigned int clk, unsigned long _hz)
{
	unsigned long hz;

    if (lpsc_clk_map[clk].pll != SYS_PLL)
    	return -EINVAL;	/* Cannot set to target freq */

	hz = clk_round_rate(clk, _hz);
	if (hz != _hz)
		return -EINVAL;	/* Cannot set to target freq */

	pll_div_freq_set(lpsc_clk_map[clk].pll, lpsc_clk_map[clk].div, hz);
	return 0;
}

