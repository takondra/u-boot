/*
 * Copyright (C) 2012 Texas Instruments.
 *
 * TCI6638: secure kernel command file
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


/***********************************************************************************
 * FILE PURPOSE: command for installing secure kernel
 ***********************************************************************************
 * FILE NAME: cmd_skern.c
 *
 * DESCRIPTION: The secure kernel module is used for installing boot code that
 * 		does following things:-
 *			- install secure monitor code
 *				- secure kernel initialization for primary and
 *				  and secondary cores
 *				- smc handler for secure kernel services
 *			- initialize the data for secondary core
 *
 ***********************************************************************************/
#include <common.h>
#include <command.h>
int skern_install(u32 addr, u32 dpsc, u32 freq)
{
	int result;

	__asm__ __volatile__ (
		"stmfd r13!, {lr}\n"
		"mov r0, %1\n"
		"mov r1, %2\n"
		"mov r2, %3\n"
		"blx r0\n"
		"ldmfd r13!, {lr}\n"
	: "=&r" (result)
	: "r" (addr), "r" (dpsc), "r" (freq)
	: "cc", "memory");
	return result;
}

int do_install_skernel(cmd_tbl_t *cmdtp, int flag, int argc,
			char * const argv[])
{
	/* TODO: dpsc_base has to come from device tree */
	u32 addr, dpsc_base = 0x1E80000, freq = 1000000000;
	int     rcode = 0;

	if (argc < 2)
		return CMD_RET_USAGE;

	addr = simple_strtoul(argv[1], NULL, 16);

	printf ("## Starting boot kernel at 0x%08x ...\n", addr);
	rcode = skern_install(addr, dpsc_base, freq);
	printf ("## Started boot kernel successfully\n");
	return rcode;
}

U_BOOT_CMD(
	install_skernel, 2, 0, do_install_skernel,
	"Installing boot kernel at 'addr'\n",
	""
);

void core_spin(void)
{
	while (1);
}

int tetris_core_on(int core_id, void *ep)
{
	int result;

	asm volatile (
		"stmfd  r13!, {lr}\n"
		"mov r0, %1\n"
		"mov r1, %2\n"
		".inst  0xe1600070\n"
		"ldmfd  r13!, {lr}\n"
		: "=&r" (result)
		: "r" (core_id), "r" (ep)
		: "cc",  "memory");
	return  result;
}

int tetris_core_off(int core_id)
{
	int result;

	asm volatile (
		"stmfd  r13!, {lr}\n"
		"mov r0, %1\n"
		".inst  0xe1600071\n"
		"ldmfd  r13!, {lr}\n"
		: "=&r" (result)
		: "r" (core_id)
		: "cc",  "memory");
	return  result;
}

int do_tetris_power(cmd_tbl_t *cmdtp, int flag, int argc,
			char * const argv[])
{
	int     rcode = 0, core_id, on;
	void (*fn)(void);

	fn = core_spin;

	if (argc < 3)
		return CMD_RET_USAGE;

	core_id = simple_strtoul(argv[1], NULL, 16);
	on = simple_strtoul(argv[2], NULL, 16);
	if (on)
		rcode = tetris_core_on(core_id, fn);
	else
		rcode = tetris_core_off(core_id);

	if (on) {
		if (!rcode)
			printf("core %d powered on successfully\n", core_id);
		else
			printf("core %d powered on failure\n", core_id);
	} else {
		printf("core %d powered off successfully\n", core_id);
	}

	return rcode;
}

U_BOOT_CMD(
	tet_power, 3, 0, do_tetris_power,
	"Power On/Off Tetris secondary core\n",
	"tet_power <coreid> <oper>\n"
	"- coreid (1-3) and oper (1 - ON, 0 - OFF)\n"
	""
);

