/*
 * Copyright (C) 2013 Texas Instruments.
 *
 * Keystone: High Memory Test
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
/*
 * Keysone2 systems which have more then 2GB of DDR2A memory
 * cannot access that memory without MMU enable. In order to test
 * the entire memory we need to create MMU translation table,
 * enable MMU and test the memory in portions. After the test
 * we need to disable MMU.
 *
 * We assume that the memory size >= 2GB and u-boot code is relocated
 * to the upper GB of the PA address range and the 0x80000000-0xbfffffff
 * address range is available to use for testing.
 *
 * The code will create simple 1 level page table with 4 entries of
 * 1GB blocks. The block with VA 0x80000000-0xbfffffff will be used
 * to map different portions of the DDR3a. 
 * 
 */

#include <common.h>
#include <asm/armv7.h>

static unsigned long long pgd_table[4] __attribute__((aligned(0x1000)));
static unsigned long sctlr_save;
static int mmu_enabled = 0;

/* Invalidate TLB */
void my_v7_inval_tlb(void)
{
	/* Invalidate entire unified TLB */
	asm volatile ("mcr p15, 0, %0, c8, c7, 0" : : "r" (0));
	/* Invalidate entire data TLB */
	asm volatile ("mcr p15, 0, %0, c8, c6, 0" : : "r" (0));
	/* Invalidate entire instruction TLB */
	asm volatile ("mcr p15, 0, %0, c8, c5, 0" : : "r" (0));
	/* Full system DSB - make sure that the invalidation is complete */
	CP15DSB;
	/* Full system ISB - make sure the instruction stream sees it */
	CP15ISB;
}

static unsigned long enable_mmu(unsigned long ttbr0)
{
	unsigned long ret; 
	asm (
		"stmfd	r13!, {r10, r11}        \n"
		/* set ttbr0			*/
		"mov	r10, %1	                \n"
		"mov	r11, #0	                \n"
		"mcrr	p15, 0, r10, r11, c2    \n"
		/* ttbcr = 0x80000000		*/
		"mov	r10, #0 \n"
		"orr	r10, r10, #(1 << 31)    \n"
		"mcr	p15, 0, r10, c2, c0, 2   \n"
		/* save current SCTLR value	*/
		"mrc	p15, #0, r10, c1, c0, #0\n"
		"mov	%0, r10                 \n"
		/* disable cache		*/
		"bic	r10, r10,#(1 << 12)	\n"
		"bic	r10, r10,#(1 <<  2)	\n"
		"mcr	p15, #0, r10, c1, c0, #0\n"
		"isb				\n"
		"dsb				\n"
		"bl	invalidate_dcache_all   \n"
		"bl	my_v7_inval_tlb		\n"
		/* enable mmu			*/
		"mrc	p15, #0, r10, c1, c0, #0\n"
		"orr	r10, r10,#1		\n"
		"mcr	p15, #0, r10, c1, c0, #0\n"
		"ldmfd	r13!, {r10, r11} \n"
		: "=r" (ret) : "r" (ttbr0) : "r10", "r11", "cc"
	);

	return ret;
}

static void disable_mmu(unsigned long sctlr_old)
{
	asm volatile (
		"mov	r10, %0			\n"
		"bic	r10, r10,#1		\n"
		"mcr	p15, #0, r10, c1, c0, #0\n"
		: : "r" (sctlr_old): "r10", "cc"
	       );

}


static void pgd_table_init(void)
{
	/*
	 * we create identical mapping for the whole 4GB address range
	 * this table maps 0x0-0xffffffff VA to aliased PA range
	 * 0x0_0000_0000 - 0x0_ffff_ffff;
	 */

	pgd_table[0] = 0x000000000000071dULL;
	pgd_table[1] = 0x000000004000071dULL;
	pgd_table[2] = 0x000000008000071dULL;
	pgd_table[3] = 0x00000000c000071dULL;

}

int do_highmem_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned long	pgd2_pa;
	unsigned long long pgd_val;
	int j;

	if (argc < 2)
		goto highmem_cmd_usage;

	if (strcmp(argv[1], "mmuon") == 0) {
		if (mmu_enabled) 
			printf("MMU already enabled\n");
		else {
			pgd_table_init();
			sctlr_save = enable_mmu(pgd_table);
			mmu_enabled = 1;
			printf("MMU enabled\n");
		}
	} else if (strcmp(argv[1], "mmuoff") == 0) {
		if (mmu_enabled == 0)
			printf("MMU already disabled\n");
		else {
			disable_mmu(sctlr_save);
			mmu_enabled = 0;
			printf("MMU disabled\n");
		}
	} else if (strcmp(argv[1], "pgd2") == 0) {
		if (argc == 3) {
			pgd2_pa = simple_strtoul(argv[2], NULL, 16);
			pgd_val = pgd2_pa;
			pgd_val <<= 4;
			pgd_val |= 0x71dull;
			pgd_table[2] = pgd_val;
			my_v7_inval_tlb();
		}
		for (j=0; j<4; j++)
			printf("pgd%d - 0x%016llX\n", j, pgd_table[j]);
	} else
		goto highmem_cmd_usage;

	return 0;

highmem_cmd_usage:
	return cmd_usage(cmdtp);
}

U_BOOT_CMD(
	highmem,	3,	0,	do_highmem_cmd,
	"highmem test",
	"highmem <mmuon|mmuoff|pgd2> [<pgd2_pa >> 4> in hex] \n"
);

