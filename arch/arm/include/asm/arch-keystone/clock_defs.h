/*
 * Copyright (C) 2012 Texas Instruments Incorporated
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef _CLOCK_DEFS_H_
#define _CLOCK_DEFS_H_

#include <asm/arch/hardware.h>

#define BIT(x)			(1 << (x))

/* PLL Control Registers */
struct pllctl_regs {
	u32	ctl;		/* 00 */
	u32	ocsel;		/* 04 */
	u32	secctl;		/* 08 */
	u32	__pad0;
	u32	mult;		/* 10 */
	u32	prediv;		/* 14 */
	u32	div1;		/* 18 */
	u32	div2;		/* 1c */
	u32	div3;		/* 20 */
	u32	oscdiv1;	/* 24 */
	u32	__pad1;		/* 28 */
	u32	bpdiv;		/* 2c */
	u32	wakeup;		/* 30 */
	u32	__pad2;
	u32	cmd;		/* 38 */
	u32	stat;		/* 3c */
	u32	alnctl;		/* 40 */
	u32	dchange;	/* 44 */
	u32	cken;		/* 48 */
	u32	ckstat;		/* 4c */
	u32	systat;		/* 50 */
	u32	ckctl;		/* 54 */
	u32	__pad3[2];
	u32	div4;		/* 60 */
	u32	div5;		/* 64 */
	u32	div6;		/* 68 */
	u32	div7;		/* 6c */
	u32	div8;		/* 70 */
	u32	div9;		/* 74 */
	u32	div10;		/* 78 */
	u32	div11;		/* 7c */
	u32	div12;		/* 80 */
};

static struct pllctl_regs *pllctl_regs[] = {
	(struct pllctl_regs *)(CLOCK_BASE + 0x100)
};

#define pllctl_reg(pll, reg)		(&(pllctl_regs[pll]->reg))
#define pllctl_reg_read(pll, reg)	__raw_readl(pllctl_reg(pll, reg))
#define pllctl_reg_write(pll, reg, val)	__raw_writel(val, pllctl_reg(pll, reg))

#define pllctl_reg_rmw(pll, reg, mask, val)			\
	pllctl_reg_write(pll, reg,				\
		(pllctl_reg_read(pll, reg) & ~(mask)) | val)

#define pllctl_reg_setbits(pll, reg, mask)			\
	pllctl_reg_rmw(pll, reg, 0, mask)

#define pllctl_reg_clrbits(pll, reg, mask)			\
	pllctl_reg_rmw(pll, reg, mask, 0)

#define reg_rmw(reg, mask, val) \
	__raw_writel((__raw_readl((reg)) & ~(mask)) | ((val) & (mask)) , \
	              (reg));

#define reg_setbits(reg, bits)	\
	__raw_writel( (__raw_readl(reg) | (bits)) , (reg));

#define reg_clrbits(reg, bits)	\
	__raw_writel( (__raw_readl(reg) & ~(bits)) , (reg));

#define pll0div_read(N) ((pllctl_reg_read(CORE_PLL, div##N) & 0xff) + 1)

/* PLLCTL Bits */
#define PLLCTL_BYPASS		BIT(23)
#define PLLCTL_CLKMODE		BIT(8)
#define PLLCTL_PLLSELB		BIT(7)
#define PLLCTL_PLLENSRC		BIT(5)
#define PLLCTL_PLLDIS		BIT(4)
#define PLLCTL_PLLRST		BIT(3)
#define PLLCTL_PLLPWRDN		BIT(1)
#define PLLCTL_PLLEN		BIT(0)

#define MAIN_ENSAT_OFFSET	6

#define PLLDIV_ENABLE		BIT(15)


#endif  /* _CLOCK_DEFS_H_ */
