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
#include <malloc.h>
#include <nand.h>

#define DDR_TEST_BURST_SIZE		1024
#define DDR_MIN_ADDR			0x80000000
#define DDR_MAX_ADDR			0xc0000000

int ddr_memory_test (unsigned int start_address, unsigned int end_address, int quick)
{
    unsigned int index_start, value, index;

    index_start = start_address;

	while(1) {
	    /* Write a pattern */
	    for (index = index_start; index < index_start+DDR_TEST_BURST_SIZE; index += 4) {
	        *(volatile unsigned int *) index = (unsigned int)index;
	    }

	    /* Read and check the pattern */
	    for (index = index_start; index < index_start+DDR_TEST_BURST_SIZE; index += 4) {
	    	value = *(volatile unsigned int *) index;
	        if (value  != index) {
	            printf("ddr_memory_test: Failed at address index = 0x%x value = 0x%x *(index) = 0x%x\n",
	                index, value, *(volatile unsigned int *) index);
	            return (-1);
	        }
	    }

	    index_start += DDR_TEST_BURST_SIZE;
	    if ((index_start >= end_address))
	    	break;

		if (quick)
			continue;

	    /* Write a pattern for complementary values */
    	for (index = index_start; index < index_start+DDR_TEST_BURST_SIZE; index += 4) {
        	*(volatile unsigned int *) index = (unsigned int)~index;
    	}

    	/* Read and check the pattern */
    	for (index = index_start; index < index_start+DDR_TEST_BURST_SIZE; index += 4) {
	    	value = *(volatile unsigned int *) index;

        	if (value  != ~index) {
            	printf("ddr_memory_test: Failed at address index = 0x%x value = 0x%x *(index) = 0x%x\n",
                	index, value, *(volatile unsigned int *) index);
            	return (-1);
        	}
    	}

	    index_start += DDR_TEST_BURST_SIZE;
	    if ((index_start >= end_address))
	    	break;

	    /* Write a pattern */
	    for (index = index_start; index < index_start+DDR_TEST_BURST_SIZE; index += 2) {
	        *(volatile unsigned short *) index = (unsigned short)index;
	    }

	    /* Read and check the pattern */
	    for (index = index_start; index < index_start+DDR_TEST_BURST_SIZE; index += 2) {
	    	value = *(volatile unsigned short *) index;
	        if (value  != (unsigned short)index) {
	            printf("ddr_memory_test: Failed at address index = 0x%x value = 0x%x *(index) = 0x%x\n",
	                index, value, *(volatile unsigned short *) index);
	            return (-1);
	        }
	    }

	    index_start += DDR_TEST_BURST_SIZE;
	    if ((index_start >= end_address))
	    	break;

	    /* Write a pattern */
	    for (index = index_start; index < index_start+DDR_TEST_BURST_SIZE; index += 1) {
	        *(volatile unsigned char *) index = (unsigned char)index;
	    }

	    /* Read and check the pattern */
	    for (index = index_start; index < index_start+DDR_TEST_BURST_SIZE; index += 1) {
	    	value = *(volatile unsigned char *) index;
	        if (value  != (unsigned char)index) {
	            printf("ddr_memory_test: Failed at address index = 0x%x value = 0x%x *(index) = 0x%x\n",
	                index, value, *(volatile unsigned char *) index);
	            return (-1);
	        }
	    }

	    index_start += DDR_TEST_BURST_SIZE;
	    if ((index_start >= end_address))
	    	break;
	}


    printf("ddr_memory_test PASSED!\n");
    return 0;
}

static int ddr_memory_compare (unsigned int address1, unsigned int address2, unsigned int size)
{
    unsigned int index, value, index2, value2;

   	for (index = address1, index2 = address2; index < address1+size; index += 4, index2 +=4) {

    	value = *(volatile unsigned int *) index;
    	value2 = *(volatile unsigned int *) index2;

        if (value  != value2) {
            printf("ddr_memory_test: Compare failed at address = 0x%x value = 0x%x, address2 = 0x%x value2 = 0x%x\n",
                index, value, index2, value2);
            return (-1);
        }
    }

    printf("ddr_memory_compare PASSED!\n");
    return 0;
}

static unsigned int hatoi(char *p, char **errp)
{
	unsigned int res = 0;

	if ((p[0] == '0') && ((p[1] == 'x') || (p[1] == 'X')))
		p += 2;

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
	unsigned int start_addr, end_addr, size;
	char *err;
	int test_case;

	if ((argc == 4) && (strncmp(argv[1], "test", 5) == 0)) {
		test_case = 1;
	}
	else if ((argc == 5) && (strncmp(argv[1], "compare", 8) == 0)) {
			test_case = 2;
	}
	else {
		test_case = -1;
	}

	if (test_case != -1)
	{
		start_addr = hatoi(argv[2], &err);
		end_addr = hatoi(argv[3], &err);
		if (
			(start_addr < DDR_MIN_ADDR)	||
			(start_addr >= DDR_MAX_ADDR)	||
			(end_addr < DDR_MIN_ADDR)		||
			(end_addr >= DDR_MAX_ADDR)	||
			(start_addr >= end_addr)
		   )
		{
			printf ("Invalid start or end address!\n");
			goto ddr_cmd_usage;
		}
		if (test_case == 1) {
			ddr_memory_test(start_addr, end_addr, 0);
		}
		else if (test_case == 2) {
			size = hatoi(argv[4], &err);
			ddr_memory_compare(start_addr, end_addr, size);
		}
	} else {
		goto ddr_cmd_usage;
	}
	return 0;

ddr_cmd_usage:
	return cmd_usage(cmdtp);
}


U_BOOT_CMD(
	ddr,	5,	1,	do_ddr_test,
	"DDR3 test",
	"test <start_addr in hex> <end_addr in hex> - test DDR from start address to end address\n"
	"ddr compare <start_addr in hex> <end_addr in hex> <size in hex> - compare DDR data of (size) bytes from start address to end address\n"
);

#define DATA_ADDR					0x80000000
#define OOB_RD_ADDR					0x81000000
#define OOB_WR_ADDR					0x82000000
#define NAND_NUM_PAGES_PER_BLOCK	64
int do_oob_format(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	ulong off, start_addr, fmt_size, fmt_pages, size;
	unsigned int page;
	int ret;
	char *err;
	u_char *data_buf, *oobbuf_rd, *oobbuf_wr;
	nand_info_t *nand;
	mtd_oob_ops_t oob_ops;
	nand_erase_options_t erase_ops;
	struct nand_chip *chip;

	if (argc == 4) {
		if (strncmp(argv[1], "fmt", 4) == 0) {
			start_addr = hatoi(argv[2], &err);
			fmt_size = hatoi(argv[3], &err);
			nand = &nand_info[nand_curr_device];
			start_addr &= ~(nand->writesize - 1);
			fmt_pages = fmt_size/nand->writesize;

			/* Initialize the read/write buffer */
			data_buf = (u_char *)DATA_ADDR;
			oobbuf_rd = (u_char *)OOB_RD_ADDR;
			oobbuf_wr = (u_char *)OOB_WR_ADDR;
			memset (oobbuf_wr, 0xff, fmt_pages*nand->oobsize);

			memset(&oob_ops, 0, sizeof(oob_ops));
			oob_ops.len = nand->writesize;
			oob_ops.ooblen = nand->oobsize;
			oob_ops.mode = MTD_OOB_RAW;

			/* Read the data and oob and copy to the DDR */
			off = start_addr;
			size = 0;
			while (size < fmt_size) {
				/* skip the  bad blocks */
				while(nand->block_isbad(nand, (loff_t)off))
				{
					off += (nand->writesize * NAND_NUM_PAGES_PER_BLOCK);
				}

				/* Read the data and oob of the page */
				oob_ops.datbuf = data_buf;
				oob_ops.oobbuf = oobbuf_rd; /* must exist, but oob data will be appended to oob_ops.datbuf */
				ret = nand->read_oob(nand, (loff_t)off, &oob_ops);
				if (ret < 0) {
					printf("Error (%d) reading page %08lx\n", ret, off);
					return -1;
				}

				/* re-format the oob layout for the RBL readable format, Linux uses
				   the last 40 bytes of ECC in the 64-byte oob, while RBL uses last 10 bytes
				   of each 16 bytes oob (total 4x16 bytes) */
				memcpy(&oobbuf_wr[16*0+6], &data_buf[nand->writesize+24+10*0], 10);
				memcpy(&oobbuf_wr[16*1+6], &data_buf[nand->writesize+24+10*1], 10);
				memcpy(&oobbuf_wr[16*2+6], &data_buf[nand->writesize+24+10*2], 10);
				memcpy(&oobbuf_wr[16*3+6], &data_buf[nand->writesize+24+10*3], 10);

				/* Increment one page */
				data_buf += (nand->writesize + nand->oobsize);
				oobbuf_rd += nand->oobsize;
				oobbuf_wr += nand->oobsize;
				size += nand->writesize;
				off += nand->writesize;
			}

			/* Erase the blocks */
			memset(&erase_ops, 0, sizeof(erase_ops));
			erase_ops.offset = start_addr;
			erase_ops.length = fmt_size;
			erase_ops.spread = 1;
			erase_ops.quiet = 1;
			ret = nand_erase_opts(nand, &erase_ops);
			if (ret < 0) {
				printf("Error (%d) erase off %08lx\n", ret, start_addr);
				return -1;
			}

			/* Write raw data and raw oob to NAND */
			off = start_addr;
			size = 0;
			chip = nand->priv;

			/* Initialize the write buffer */
			data_buf = (u_char *)DATA_ADDR;
			oobbuf_wr = (u_char *)OOB_WR_ADDR;

			while (size < fmt_size) {
				/* skip the  bad blocks */
				while(nand->block_isbad(nand, (loff_t)off))
				{
					off += (nand->writesize * NAND_NUM_PAGES_PER_BLOCK);
				}

				/* Write a raw page and oob */
				chip->state = FL_WRITING; /* Get the device */
				chip->ops.len = nand->writesize;
				chip->ops.datbuf = data_buf;
				memcpy(chip->oob_poi, oobbuf_wr, nand->oobsize);
				page = (int)(off >> chip->page_shift) & chip->pagemask;
				ret = chip->write_page(nand, chip, chip->ops.datbuf, page, 0, 1);
				if (ret < 0) {
					printf("Error (%d) write page off %08lx\n", ret, off);
					return -1;
				}
				chip->select_chip(nand, -1);	/* De-select the NAND device */

				/* Increment one page */
				data_buf += (nand->writesize + nand->oobsize);
				oobbuf_wr += nand->oobsize;
				size += nand->writesize;
				off += nand->writesize;
			}
		}
		else {
			goto oob_cmd_usage;
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
	"fmt <start_addr in hex> <size in hex> - Re-format the OOB data \n"
	"    from start offset byte addr with size bytes\n"
);

int do_factory_image(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	ulong off, off_addr, img_size, img_pages, size, data_addr, oob_addr;
	unsigned int page;
	int ret;
	char *err;
	u_char *data_buf, *oob_buf_rd, *oob_buf_wr;
	nand_info_t *nand;
	mtd_oob_ops_t oob_ops;
	nand_erase_options_t erase_ops;
	struct nand_chip *chip;

	if (argc == 5) {
		off_addr = hatoi(argv[1], &err);
		img_size = hatoi(argv[2], &err);
		data_addr = hatoi(argv[3], &err);
		oob_addr = hatoi(argv[4], &err);

		nand = &nand_info[nand_curr_device];
		off_addr &= ~(nand->writesize - 1);
		img_pages = img_size/nand->writesize;

		/* Initialize the read/write buffer */
		data_buf = (u_char *)data_addr;
		oob_buf_rd = (u_char *)oob_addr;

		memset(&oob_ops, 0, sizeof(oob_ops));
		oob_ops.len = nand->writesize;
		oob_ops.ooblen = nand->oobsize;
		oob_ops.mode = MTD_OOB_RAW;

		/* Read the data and oob */
		off = off_addr;
		size = 0;
		while (size < img_size) {
			/* skip the  bad blocks */
			while(nand->block_isbad(nand, (loff_t)off))
					off += (nand->writesize * NAND_NUM_PAGES_PER_BLOCK);

			/* Read the data and oob of the page */
			oob_ops.datbuf = data_buf;
			oob_ops.oobbuf = oob_buf_rd; /* must exist, but oob data will be appended to oob_ops.datbuf */
			ret = nand->read_oob(nand, (loff_t)off, &oob_ops);
			if (ret < 0) {
				printf("Error (%d) reading page %08lx\n", ret, off);
				return -1;
			}

			/* Copy the oob data to oob_buf_rd */
			memcpy(oob_buf_rd, &data_buf[nand->writesize], nand->oobsize);

			/* Increment one page */
			data_buf += nand->writesize;
			oob_buf_rd += nand->oobsize;
			size += nand->writesize;
			off += nand->writesize;
		}

#if 1
		/* Erase the blocks */
		memset(&erase_ops, 0, sizeof(erase_ops));
		erase_ops.offset = off_addr;
		erase_ops.length = img_size;
		erase_ops.spread = 1;
		erase_ops.quiet = 1;
		ret = nand_erase_opts(nand, &erase_ops);
		if (ret < 0) {
			printf("Error (%d) erase off %08lx\n", ret, off_addr);
			return -1;
		}

		/* Write raw data and raw oob to NAND */
		off = off_addr;
		size = 0;
		chip = nand->priv;

		/* Initialize the write buffer */
		data_buf = (u_char *)data_addr;
		oob_buf_wr = (u_char *)oob_addr;

		while (size < img_size) {
			/* skip the  bad blocks */
			while(nand->block_isbad(nand, (loff_t)off))
				off += (nand->writesize * NAND_NUM_PAGES_PER_BLOCK);

			/* Write a raw page and oob */
			chip->state = FL_WRITING; /* Get the device */
			chip->ops.len = nand->writesize;
			chip->ops.datbuf = data_buf;
			memcpy(chip->oob_poi, oob_buf_wr, nand->oobsize);
			page = (int)(off >> chip->page_shift) & chip->pagemask;
			ret = chip->write_page(nand, chip, chip->ops.datbuf, page, 0, 1);
			if (ret < 0) {
				printf("Error (%d) write page off %08lx\n", ret, off);
				return -1;
			}
			chip->select_chip(nand, -1);	/* De-select the NAND device */

			/* Increment one page */
			data_buf += nand->writesize;
			oob_buf_wr += nand->oobsize;
			size += nand->writesize;
			off += nand->writesize;
		}
#endif
	} else {
		goto factory_image_cmd_usage;
	}
	return 0;

factory_image_cmd_usage:
	return cmd_usage(cmdtp);
}

U_BOOT_CMD(
	facimg,	5,	1,	do_factory_image,
	"Read nand page data and oob data in raw format to memory",
	"<off in hex> <size in hex> <data_addr in hex> <oob_addr in hex>\n"
	"    read 'size' bytes starting at offset 'off'\n"
	"    to memory address 'data_addr' and 'oob_addr', skipping bad blocks.\n"
);
