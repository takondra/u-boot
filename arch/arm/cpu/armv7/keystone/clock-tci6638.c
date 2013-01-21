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

static const struct pll_regs pll_regs[] = {
	[CORE_PLL]	= { TCI6638_MAINPLLCTL0, TCI6638_MAINPLLCTL1},
	[PASS_PLL]	= { TCI6638_PASSPLLCTL0, TCI6638_PASSPLLCTL1},
	[TETRIS_PLL]	= { TCI6638_ARMPLLCTL0,  TCI6638_ARMPLLCTL1},
	[DDR3A_PLL]	= { TCI6638_DDR3APLLCTL0,TCI6638_DDR3APLLCTL1},
	[DDR3B_PLL]	= { TCI6638_DDR3BPLLCTL0,TCI6638_DDR3BPLLCTL1},
};

/* Fout = Fref * NF(mult) / NR(prediv) / OD */
static unsigned long pll_freq_get(int pll)
{
	unsigned long mult = 1, prediv = 1, output_div = 2;
	unsigned long ret;
	u32 tmp, reg;

	if (pll == CORE_PLL) {
		ret = external_clk[sys_clk];
		if (pllctl_reg_read(pll, ctl) & PLLCTL_PLLEN) {
			/* PLL mode */
			tmp = __raw_readl(TCI6638_MAINPLLCTL0);
			prediv = (tmp & 0x3f) + 1;
			mult = (((tmp & 0x7f000) >> 6) | (pllctl_reg_read(pll,
							mult) & 0x3f)) + 1;
			output_div = ((pllctl_reg_read(pll, secctl) >> 19) & 0xf) + 1;

			ret = ret / prediv / output_div * mult;
		}
	}
	else {
		switch (pll) {
		case PASS_PLL:
			ret = external_clk[pa_clk];
			reg = TCI6638_PASSPLLCTL0;
			break;
		case TETRIS_PLL:
			ret = external_clk[tetris_clk];
			reg = TCI6638_ARMPLLCTL0;
			break;
		case DDR3A_PLL:
			ret = external_clk[ddr3a_clk];
			reg = TCI6638_DDR3APLLCTL0;
			break;
		case DDR3B_PLL:
			ret = external_clk[ddr3b_clk];
			reg = TCI6638_DDR3BPLLCTL0;
			break;
		default:
			return 0;
		}
		
		tmp = __raw_readl(reg);

		if (!(tmp & 0x00800000)) {
			/* Bypass disabled */
			prediv = (tmp & 0x3f) + 1;
			mult = ((tmp >> 6) & 0x1fff) + 1;
			output_div = ((tmp >> 19) & 0xf) + 1;
			ret = ret * mult / prediv / output_div;
		}
	}

	return ret;
}




unsigned long clk_get_rate(unsigned int clk)
{
	switch (clk) {
	case core_pll_clk:	return pll_freq_get(CORE_PLL);
	case pass_pll_clk:	return pll_freq_get(PASS_PLL);
	case tetris_pll_clk:	return pll_freq_get(TETRIS_PLL);
	case ddr3a_pll_clk:	return pll_freq_get(DDR3A_PLL);
	case ddr3b_pll_clk:	return pll_freq_get(DDR3B_PLL);
	case sys_clk0_1_clk:
	case sys_clk0_clk:	return pll_freq_get(CORE_PLL) / pll0div_read(1);
	case sys_clk1_clk:	return pll_freq_get(CORE_PLL) / pll0div_read(2);
	case sys_clk2_clk:	return pll_freq_get(CORE_PLL) / pll0div_read(3);
	case sys_clk3_clk:	return pll_freq_get(CORE_PLL) / pll0div_read(4);
	case sys_clk0_2_clk:	return clk_get_rate(sys_clk0_clk) / 2;
	case sys_clk0_3_clk:	return clk_get_rate(sys_clk0_clk) / 3;
	case sys_clk0_4_clk:	return clk_get_rate(sys_clk0_clk) / 4;
	case sys_clk0_6_clk:	return clk_get_rate(sys_clk0_clk) / 6;
	case sys_clk0_8_clk:	return clk_get_rate(sys_clk0_clk) / 8;
	case sys_clk0_12_clk:	return clk_get_rate(sys_clk0_clk) / 12;
	case sys_clk0_24_clk:	return clk_get_rate(sys_clk0_clk) / 24;
	case sys_clk1_3_clk:	return clk_get_rate(sys_clk1_clk) / 3;
	case sys_clk1_4_clk:	return clk_get_rate(sys_clk1_clk) / 4;
	case sys_clk1_6_clk:	return clk_get_rate(sys_clk1_clk) / 6;
	case sys_clk1_12_clk:	return clk_get_rate(sys_clk1_clk) / 12;
	default:
		break;
	}
	return 0;
}

