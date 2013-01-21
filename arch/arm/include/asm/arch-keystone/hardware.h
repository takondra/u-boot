/*
 * Copyright (C) 2012 Texas Instruments
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <config.h>

#ifndef __ASSEMBLY__

#include <asm/sizes.h>

#define	REG(addr)	(*(volatile unsigned int *)(addr))
#define REG_P(addr)	((volatile unsigned int *)(addr))

typedef volatile unsigned int	dv_reg;
typedef volatile unsigned int *	dv_reg_p;

#define ASYNC_EMIF_NUM_CS		4
#define ASYNC_EMIF_MODE_NOR		0
#define ASYNC_EMIF_MODE_NAND		1
#define ASYNC_EMIF_MODE_ONENAND		2
#define ASYNC_EMIF_PRESERVE		-1

struct async_emif_config {
	unsigned mode;
	unsigned select_strobe;
	unsigned extend_wait;
	unsigned wr_setup;
	unsigned wr_strobe;
	unsigned wr_hold;
	unsigned rd_setup;
	unsigned rd_strobe;
	unsigned rd_hold;
	unsigned turn_around;
	enum {
		ASYNC_EMIF_8	= 0,
		ASYNC_EMIF_16	= 1,
		ASYNC_EMIF_32	= 2,
	} width;
};

void init_async_emif(int num_cs, struct async_emif_config *config);

struct ddr3_phy_config {
	unsigned int pllcr;
	unsigned int pgcr1_mask;
	unsigned int pgcr1_val;
	unsigned int ptr0;
	unsigned int ptr1;
	unsigned int ptr2;
	unsigned int ptr3;
	unsigned int ptr4;
	unsigned int dcr_mask;
	unsigned int dcr_val;
	unsigned int dtpr0;
	unsigned int dtpr1;
	unsigned int dtpr2;
	unsigned int mr0;
	unsigned int mr1;
	unsigned int mr2;
	unsigned int dtcr;
	unsigned int pgcr2;
	unsigned int zq0cr1;
	unsigned int zq1cr1;
	unsigned int zq2cr1;
	unsigned int pir_v1;
	unsigned int pir_v2;
};

struct ddr3_emif_config {
	unsigned int sdcfg;
	unsigned int sdtim1;
	unsigned int sdtim2;
	unsigned int sdtim3;
	unsigned int sdtim4;
	unsigned int zqcfg;
	unsigned int sdrfc;
};

#endif

#define		BIT(x)	(1 << (x))

#ifdef CONFIG_SOC_TCI6614
#include <asm/arch/hardware-tci6614.h>
#endif

#ifdef CONFIG_SOC_TCI6638
#include <asm/arch/hardware-tci6638.h>
#endif

#endif /* __ASM_ARCH_HARDWARE_H */
