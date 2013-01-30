/*
 * Copyright (C) 2013 Texas Instruments
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

struct spi_img_info {
	u32 loadaddr;
	u32 addr;
	u32 len;
	u32 blksz;
};

int spi_get_img_info(struct spi_img_info *p_info, int argc, char * const argv[])
{
	char *endp, *tmpp;

	if (argc < 1)
		return -1;

	p_info->loadaddr = simple_strtoul(argv[0], &endp, 16);

	/* Where is the image */
	if ((argc > 1) && (strcmp(argv[1], "fileaddr") != 0))
		tmpp = argv[1];
	else
		tmpp = getenv("fileaddr");

	if (!tmpp) {
		printf("env fileaddr does not exist\n");
		return -1;
	}
	p_info->addr = simple_strtoul(tmpp, &endp, 16);

	/* Size of image in bytes */
	if ((argc > 2) && (strcmp(argv[2], "filesize") != 0))
		tmpp = argv[2];
	else
		tmpp = getenv("filesize");

	if (!tmpp) {
		printf("env filesize does not exist\n");
		return -1;
	}
	p_info->len = simple_strtoul(tmpp, &endp, 16);

	/* Block size */
	p_info->blksz = p_info->len;

	debug("fmtimg: type=spi loadaddr=0x%08x addr=0x%08x len=0x%08x blksz=0x%08x\n",
		p_info->loadaddr, p_info->addr, p_info->len, p_info->blksz);

	return 0;
}

int spi_format_blk(u32 from_end, u32 to_end, u32 blksz, u32 laddr)
{
	u32 tmp, i;

	for (i=0; i<blksz; i++) {
		*((u8 *)to_end) = *((u8 *)from_end);
		--to_end;
		--from_end;
	}

	/* load addr of this block */
	tmp = cpu_to_be32(laddr);
	to_end = to_end - sizeof(unsigned long) + 1;
	*((unsigned long *)to_end) = tmp;

	/* block size of this block */
	tmp = cpu_to_be32(blksz);
	to_end = to_end - sizeof(unsigned long);
	*((unsigned long *)to_end) = tmp;

	return 0;
}

int spi_format_img(struct spi_img_info *p_info)
{
	u32 start_addr, from_end, to_end;
	u32 total_sz, hdr_sz;

	hdr_sz = 2 * sizeof(unsigned long);

	/* Size including the terminating block of 0 */
	total_sz = p_info->len + 2 * hdr_sz;

	/* Working backward */

	/* Write Terrminating block first */
	start_addr = p_info->addr + p_info->len + hdr_sz;
	memset((void *)start_addr, 0, hdr_sz);

	/* Format the image */
	from_end = p_info->addr + p_info->len - 1;
	to_end = p_info->addr + p_info->len + hdr_sz - 1;
	spi_format_blk(from_end, to_end, p_info->blksz, p_info->loadaddr);

	printf("formatted image - Addr 0x%08x (Bytes: %d(0x%x))\n",
		p_info->addr, total_sz, total_sz);

	return 0;
}

int do_fmtimg_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	struct spi_img_info img_info;
	int ret;

	if (argc < 3)
		return cmd_usage(cmdtp);

	if (strcmp(argv[1], "spi") != 0)
		return cmd_usage(cmdtp);

	argc -= 2;
	argv += 2;

	ret = spi_get_img_info(&img_info, argc, argv);
	if (ret < 0)
		return cmd_usage(cmdtp);

	ret = spi_format_img(&img_info);
	if (ret < 0)
		return cmd_usage(cmdtp);

	return 0;
}

U_BOOT_CMD(
	fmtimg,	5,	0,	do_fmtimg_cmd,
	"Format image file into TI's Keystone boot mode format",
	"<\"spi\"> <loadaddr> [addr] [len]\n"
	"    spi - SPI boot format\n"
	"    loadaddr - addr to be loaded to by ROM bootloader\n"
	"    addr - location of image file, default: env fileaddr\n"
	"    len - hex size of image file in bytes, default: env filesize\n"
);
