/*
 * Copyright (C) 2012 Texas Instruments.
 *
 * Keystone: Clock management APIs
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
#include <asm-generic/errno.h>
#include <asm/io.h>
#include <asm/processor.h>
#include <asm/arch/clock.h>
#include <asm/arch/clock_defs.h>

static void pll_delay(unsigned int loopCount)
{
	while (loopCount--) {
		asm("   NOP");
	}
}

static void wait_for_completion(const struct pll_init_data *data)
{
	volatile int i;
	for (i = 0; i < 100; i++) {
		pll_delay(300);
		if ( (pllctl_reg_read(data->pll, stat) & 0x00000001) == 0 ) {
			break;
		}
	}
}

struct pll_regs {
	u32	reg0, reg1;
};


#ifdef CONFIG_SOC_TCI6614
#include "clock-tci6614.c"
#endif
#ifdef CONFIG_SOC_TCI6638
#include "clock-tci6638.c"
#endif

void init_pll(const struct pll_init_data *data)
{
	u32 tmp, tmp_ctl, pllm, plld, pllod, reg[2], bwadj;

	pllm = data->pll_m - 1;
	plld = (data->pll_d - 1) & 0x3f;
	pllod = (data->pll_od - 1) & 0xf;

	if (data->pll == MAIN_PLL) {
		pll_delay(140000);

		/* if RBL configures the PLL, the BYPASS bit would be set to '0'           */
		tmp = pllctl_reg_read(data->pll, secctl);

		if (tmp & (PLLCTL_BYPASS)) {
			/* PLL BYPASS is enabled, we assume if not in Bypass ENSAT = 1 */

#ifdef CONFIG_SOC_TCI6614
			reg_setbits( pll_regs[data->pll].reg1, BIT(MAIN_ENSAT_OFFSET));

			pllctl_reg_clrbits(data->pll, ctl, PLLCTL_PLLEN);
#else
			reg_setbits( pll_regs[data->pll].reg1, BIT(MAIN_ENSAT_OFFSET));

			pllctl_reg_clrbits(data->pll, ctl, PLLCTL_PLLEN | PLLCTL_PLLENSRC);
			pll_delay(225); /* Wait for 4 RefClks */

			pllctl_reg_setbits(data->pll, secctl, PLLCTL_BYPASS);
#endif
			pllctl_reg_setbits(data->pll, ctl, PLLCTL_PLLPWRDN);
			pll_delay(14000);

			/* Power up the PLL */
			pllctl_reg_clrbits(data->pll, ctl, PLLCTL_PLLPWRDN);
		}
		else {
			pllctl_reg_clrbits(data->pll, ctl, PLLCTL_PLLEN | PLLCTL_PLLENSRC);
			pll_delay(225);
		}

		/* Set pll multipler (13 bit field) */

		/* Set the PLLM[5:0] to the PLL controller PLLM register */
		pllctl_reg_write(data->pll, mult, pllm & 0x3f);

		reg_rmw(pll_regs[data->pll].reg0, 0x0007F000, (pllm << 6));

		/* Set the BWADJ     (12 bit field)  */
		tmp_ctl = ((pllm + 1)>> 1) - 1; /* Divide the pllm by 2 */
		reg_rmw(pll_regs[data->pll].reg0, 0xFF000000, (tmp_ctl << 24));
		reg_rmw(pll_regs[data->pll].reg1, 0x0000000F, (tmp_ctl >> 8));

		/* Set the pll divider (6 bit field)                                         *
		 * PLLD[5:0] is located in MAINPLLCTL0                                       */
		reg_rmw(pll_regs[data->pll].reg0, 0x0000003F, plld);

		/* Set the OUTPUT DIVIDE (4 bit field) in SECCTL */
		pllctl_reg_rmw(data->pll, secctl, 0x00780000, (pllod << 19));
		wait_for_completion(data);

#ifdef CONFIG_SOC_TCI6614
		/* If necessary program PLLDIVn.  Note that must aplly the GO operation
		   to change these dividers to new ratios*/
		if (data->div_enable) {
			pllctl_reg_write(data->pll, div2, (data->pll_div2-1) | 0x00008000);
			pllctl_reg_write(data->pll, div5, (data->pll_div5-1) | 0x00008000);
			pllctl_reg_write(data->pll, div8, (data->pll_div8-1) | 0x00008000);
		}

		/* Program ALNCTLn */
		pllctl_reg_setbits(data->pll, alnctl, (1 << 1) | (1 << 4) | (1 << 7));
#else
		pllctl_reg_write(data->pll, div1, 0x00008000);
		pllctl_reg_write(data->pll, div2, 0x00008000);
		pllctl_reg_write(data->pll, div3, 0x00008001);
		pllctl_reg_write(data->pll, div4, 0x00008004);
		pllctl_reg_write(data->pll, div5, 0x000080017);

		pllctl_reg_setbits(data->pll, alnctl, 0x1f);

#endif

		/* Set GOSET bit in PLLCMD to initiate the GO operation to change the divide */
		pllctl_reg_setbits(data->pll, cmd, 0x1);
		pll_delay(1000); /* wait for the phase adj */
		wait_for_completion(data);

		/* Reset PLL */
		pllctl_reg_setbits(data->pll, ctl, PLLCTL_PLLRST);
		pll_delay (14000);		/* Wait for a minimum of 7 us*/
		pllctl_reg_clrbits(data->pll, ctl, PLLCTL_PLLRST);
		pll_delay (70000);	/* Wait for PLL Lock time (min 50 us) */

		/* Release Bypass */
		pllctl_reg_clrbits(data->pll, secctl, PLLCTL_BYPASS);

		/* Set the PLLEN */
		tmp = pllctl_reg_setbits(data->pll, ctl, PLLCTL_PLLEN);

	}
#ifdef CONFIG_SOC_TCI6638
	else if (data->pll == TETRIS_PLL) {
		bwadj = ((pllm + 1) >> 1) - 1 ;
//1.5 Set PLLCTL0[BYPASS] =1 (enable bypass),
		reg_setbits(pll_regs[data->pll].reg0,  0x00800000);
//Set CHIPMISCCTL1[13] = 0 (enable glitchfree bypass) only applicable for Kepler
		reg_clrbits( 0x02620704, (1<<13));
//2 In PLLCTL1, write PLLRST = 1 (PLL is reset)
		reg_setbits(pll_regs[data->pll].reg1 , (1<<14));
//3 Program PLLM and PLLD in PLLCTL0 register
//4 Program BWADJ[7:0] in PLLCTL0 and BWADJ[11:8] in PLLCTL1 register. BWADJ value must be set
//to ((PLLM + 1) >> 1) – 1)
		tmp = ((bwadj & 0xff) << 24) | (pllm << 6) | (plld & 0x3f) | (pllod<<19) | 0x00800000;
		__raw_writel(tmp, pll_regs[data->pll].reg0);

		/* Set BWADJ[11:8] bits */
		tmp = __raw_readl(pll_regs[data->pll].reg1);
		tmp &= ~(0xf);
		tmp |= ((bwadj>>8) & 0xf);
		__raw_writel(tmp, pll_regs[data->pll].reg1);
//5 Wait for at least 5 us based on the reference clock (PLL reset time)
		pll_delay (14000); 	/* Wait for a minimum of 7 us*/

//6 In PLLCTL1, write PLLRST = 0 (PLL reset is released)
		reg_clrbits(pll_regs[data->pll].reg1 , (1<<14));
//7 Wait for at least 500 * REFCLK cycles * (PLLD + 1) (PLL lock time)
		pll_delay (70000);
//8 Set PLLCTL0[BYPASS] =0 (disable bypass)
		reg_clrbits(pll_regs[data->pll].reg0,  0x00800000);
//9 Set CHIPMISCCTL1[13] = 1 (disable glitchfree bypass) only applicable for Kepler
		reg_setbits( 0x02620c7c, (1<<13));
	}
#endif	
	else {

		reg_setbits(pll_regs[data->pll].reg1, 0x00000040); /* Set ENSAT bit = 1 */
		/* process keeps state of Bypass bit while programming all other DDR PLL settings */
		tmp = __raw_readl(pll_regs[data->pll].reg0);
		tmp &= 0x00800000;	/* clear everything except Bypass */

		/* Set the BWADJ[7:0], PLLD[5:0] and PLLM to PLLCTL0, bypass disabled */
		bwadj = pllm >> 1;
		tmp |= ((bwadj & 0xff) << 24) | (pllm << 6) | (plld & 0x3f) | (pllod<<19);
		__raw_writel(tmp, pll_regs[data->pll].reg0);

		/* Set BWADJ[11:8] bits */
		tmp = __raw_readl(pll_regs[data->pll].reg1);
		tmp &= ~(0xf);
		tmp |= ((bwadj>>8) & 0xf);
		__raw_writel(tmp, pll_regs[data->pll].reg1);

		reg_setbits(pll_regs[data->pll].reg1, 0x00002000);	/* Set RESET bit = 1 */
		pll_delay (14000); 	/* Wait for a minimum of 7 us*/
		reg_clrbits(pll_regs[data->pll].reg1, 0x00002000);	/* Clear RESET bit */
		pll_delay (70000);

		reg_clrbits(pll_regs[data->pll].reg0, 0x00800000); /* clear BYPASS (Enable PLL Mode) */
		pll_delay (14000);	/* Wait for a minimum of 7 us*/
	}

	pll_delay (140000);
}

void init_plls(int num_pll, struct pll_init_data *config)
{
	int i;

	for (i = 0; i < num_pll; i++)
		init_pll(&config[i]);
}

