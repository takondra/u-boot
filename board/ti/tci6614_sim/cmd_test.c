/*
 * Copyright (C) 2010 Texas Instruments
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

#include <common.h>
#include <command.h>
#include <linux/mtd/mtd.h>
#include <nand.h>

#define NAND_BYTES_PER_PAGE		2048	/* in # of bytes */
#define NAND_PAGES_PER_BLOCK	64		/* in # of pages */
#define NAND_BYTES_PER_OOB		64		/* in # of bytes */

static int ddr_memory_test (int start_address, int end_address)
{
    unsigned int index, value;

    /* Write a pattern */
    for (index = start_address; index < end_address; index += 4) {
        *(volatile unsigned int *) index = (unsigned int)index;
    }

    /* Read and check the pattern */
    for (index = start_address; index < end_address; index += 4) {

    	value = *(unsigned int *) index;

        if (value  != index) {
            printf("ddr_memory_test: Failed at address index = 0x%x value = 0x%x *(index) = 0x%x\n",
                index, value, *(volatile unsigned int *) index);
            return (-1);
        }
    }

    /* Write a pattern for complementary values */
    for (index = start_address; index < end_address; index += 4) {
        *(volatile unsigned int *) index = (unsigned int)~index;
    }

    /* Read and check the pattern */
    for (index = start_address; index < end_address; index += 4) {

    	value = *(unsigned int *) index;

        if (value  != ~index) {
            printf("ddr_memory_test: Failed at address index = 0x%x value = 0x%x *(index) = 0x%x\n",
                index, value, *(volatile unsigned int *) index);
            return (-1);
        }
    }

    return 0;
}


static unsigned int hatoi(char *p, char **errp)
{
	unsigned int res = 0;

	while (1) {
		switch (*p) {
		case 'a':
		case 'b':
		case 'c':
		case 'd':
		case 'e':
		case 'f':
			res |= (*p - 'a' + 10);
			break;
		case 'A':
		case 'B':
		case 'C':
		case 'D':
		case 'E':
		case 'F':
			res |= (*p - 'A' + 10);
			break;
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
			res |= (*p - '0');
			break;
		default:
			if (errp) {
				*errp = p;
			}
		return res;
		}
		p++;
		if (*p == 0) {
			break;
		}
		res <<= 4;
	}

	if (errp) {
		*errp = NULL;
	}

	return res;
}

int do_ddr_test(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned int start_addr, end_addr;
	char *err;

	if (argc == 4) {
		if (strncmp(argv[1], "test", 5) == 0) {
			start_addr = hatoi(argv[2], &err);
			end_addr = hatoi(argv[3], &err);
			if (
				(start_addr < 0x80000000)	||
				(start_addr >= 0xa0000000)	||
				(end_addr < 0x80000000)		||
				(end_addr >= 0xa0000000)	||
				(start_addr >= end_addr)
			   ) {
				   printf ("Invalid start or end address!\n");
				   goto ddr_cmd_usage;
			}
			ddr_memory_test(start_addr, end_addr);
		}
	} else {
		goto ddr_cmd_usage;
	}
	return 0;

ddr_cmd_usage:
	return cmd_usage(cmdtp);
}


U_BOOT_CMD(
	ddr,	4,	1,	do_ddr_test,
	"DDR3 test",
	"ddr test <start_addr in hex> <end_addr in hex> - test DDR from start address to end address\n"
);

int do_oob_format(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned int start_blk, end_blk, i, j;
	char *err;
	unsigned char rd_buf[NAND_BYTES_PER_OOB], wr_buf[NAND_BYTES_PER_OOB];
	nand_info_t *nand;
	mtd_oob_ops_t ops;
	loff_t off;

	if (argc == 4) {
		if (strncmp(argv[1], "fmt", 4) == 0) {
			start_blk = hatoi(argv[2], &err);
			end_blk = hatoi(argv[3], &err);
			nand = &nand_info[nand_curr_device];
			memset (&ops, 0, sizeof(mtd_oob_ops_t));
			memset (wr_buf, 0xff, NAND_BYTES_PER_OOB);

			for (i = start_blk; i <= end_blk; i++) {
				for (j = 0; j < NAND_PAGES_PER_BLOCK; j++) {
					off = i * (NAND_BYTES_PER_PAGE * NAND_PAGES_PER_BLOCK) + NAND_BYTES_PER_PAGE * j;
					ops.oobbuf = rd_buf;
					ops.ooblen = NAND_BYTES_PER_OOB;
					ops.mode = MTD_OOB_RAW;
					nand->read_oob(nand, off, &ops);

					memcpy(wr_buf, rd_buf, 16);
					memcpy(&wr_buf[16], &rd_buf[16+6], 10);
					memcpy(&wr_buf[16+10], &rd_buf[32+6], 10);
					memcpy(&wr_buf[16+20], &rd_buf[48+6], 10);
					ops.oobbuf = wr_buf;
					nand->write_oob(nand, off, &ops);
				}
			}
		}
	} else {
		goto oob_cmd_usage;
	}
	return 0;

oob_cmd_usage:
	return cmd_usage(cmdtp);
}

U_BOOT_CMD(
	oob,	4,	1,	do_oob_format,
	"Re-format the oob data from the U-boot layout to the RBL readable layout",
	"oob fmt <start_block> <end_block> - Re-format the OOB data from start_block to end_block\n"
);
