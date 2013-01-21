/*
 * (C) Copyright 2012
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
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

/*
 * Boot support
 */
#include <common.h>
#include <command.h>
#include <asm/io.h>

extern void *keystone_rio_init(int riohdid);

static void *rio_handle;

static inline int abort_rioboot(int bootdelay)
{
	int abort = 0;

	printf("Hit any key to stop autoboot: %2d ", bootdelay);

	/*
	 * Check if key already pressed
	 * Don't check if bootdelay < 0
	 */
	if (bootdelay >= 0) {
		if (tstc()) {	/* we got a key press	*/
			(void) getc();  /* consume input	*/
			puts("\b\b\b 0");
			abort = 1;	/* don't auto boot	*/
		}
	}

	while ((bootdelay > 0) && (!abort)) {
		int i;

		--bootdelay;
		/* delay 100 * 10ms */
		for (i = 0; !abort && i < 100; ++i) {
			if (tstc()) {	/* we got a key press	*/
				abort  = 1;	/* don't auto boot	*/
				bootdelay = 0;	/* no more delay	*/

				(void) getc();  /* consume input	*/
				break;
			}
			udelay(10000);
		}

		printf("\b\b\b%2d ", bootdelay);
	}

	putc('\n');

	return abort;
}

static int
rioboot(cmd_tbl_t *cmdtp, int argc, char * const argv[])
{
	u32 kern_loaded_addr, fdt_loaded_addr;
	char kaddr_buff[12], taddr_buff[12];
	u32 old_val_1, old_val_2;
	int abort = 0, bootdelay;
	cmd_tbl_t *bm_cmdtp;
	char *bm_argv[4];
	int riohdid;
	ulong addr;
	char *s;
	char *end;

	/* hostid */
	s = getenv("riohdid");
	if (s != NULL)
		riohdid = simple_strtol(s, NULL, 10);
	else
		riohdid = -1;

	switch (argc) {
	case 1:
		break;

	case 2:	/*
		 * Only one arg - accept two forms:
		 * Just load address, or just boot file name. The latter
		 * form must be written in a format which can not be
		 * mis-interpreted as a valid number.
		 */
		addr = simple_strtoul(argv[1], &end, 16);
		load_addr = addr;
		break;

	default:
		return cmd_usage(cmdtp);
	}

	printf("rioboot: waiting for link up ...\n");

	rio_handle = keystone_rio_init(riohdid);
	if (!rio_handle)
		return 0;

	/* Wait for DIO write image to finish
	   and get the addresses */

	printf("rioboot: waiting for image download ...\n");

	old_val_1 = readl(0x80000000);
	writel(0, 0x80000000);
	old_val_2 = readl(0x80000004);
	writel(0, 0x80000004);

	/* kernel load address */
	while ((kern_loaded_addr = readl(0x80000000)) == 0)
		udelay(1000);
	writel(old_val_1, 0x80000000);

	/* fdt load address */
	while ((fdt_loaded_addr = readl(0x80000004)) == 0)
		udelay(1000);
	writel(old_val_2, 0x80000004);

	sprintf(kaddr_buff, "0x%08x", kern_loaded_addr);
	sprintf(taddr_buff, "0x%08x", fdt_loaded_addr);

	printf("rioboot: image download done: kern@%s fdt@%s\n",
		kaddr_buff, taddr_buff);

	s = getenv("bootdelay");
	bootdelay = s ? (int)simple_strtol(s, NULL, 10) : CONFIG_BOOTDELAY;
	abort = abort_rioboot(bootdelay);

	/* power off RIO before booting kernel */
	keystone_rio_shutdown(rio_handle);

	if (!abort) {
		bm_cmdtp = find_cmd("bootm");

		bm_argv[0] = "bootm";
		bm_argv[1] = &(kaddr_buff[0]);
		bm_argv[2] = "-";
		bm_argv[3] = &(taddr_buff[0]);
		do_bootm(bm_cmdtp, 0, 4, bm_argv);
	}

	return 0;
}

int do_rioboot(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	return rioboot(cmdtp, argc, argv);
}

U_BOOT_CMD(
	rioboot,	3,	1,	do_rioboot,
	"boot image via rio",
	""
);

