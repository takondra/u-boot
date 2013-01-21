/*
 * Processor reset using WDT for TI TMS320TCI6614 SoC.
 *
 * Copyright (C) 2011 Texas Instruments Inc
 *
 * Copyright (C) 2007 Sergey Kubushyn <ksi@koi8.net>
 *
 * -----------------------------------------------------
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

#include <asm/arch/hardware.h>

.globl reset_cpu
reset_cpu:
	ldr	r3, RSTCTRL_REG
	ldr	r2, [r3]
	mov	r2, r2, lsr #16
	mov	r2, r2, asl #16
	orr	r2, r2, #0x5a00
	orr	r2, r2, #0x0069
	str	r2, [r3]
	ldr	r3, RSTCTRL_REG
	ldr	r2, [r3]
	mov	r2, r2, lsr #17
	mov	r2, r2, asl #17
	str	r2, [r3]
	nop
	nop
	nop
	nop
reset_cpu_loop:
	b	reset_cpu_loop

	.align	2
RSTCTRL_REG:
	.word	CLOCK_BASE + 0xe8

