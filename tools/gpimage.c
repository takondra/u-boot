/*
 * (C) Copyright 2010
 * Linaro LTD, www.linaro.org
 * Author: John Rigby <john.rigby@linaro.org>
 * Based on TI's signGP.c
 *
 * (C) Copyright 2009
 * Stefano Babic, DENX Software Engineering, sbabic@denx.de.
 *
 * (C) Copyright 2008
 * Marvell Semiconductor <www.marvell.com>
 * Written-by: Prafulla Wadaskar <prafulla@marvell.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/* Required to obtain the getline prototype from stdio.h */
#define _GNU_SOURCE

#include "mkimage.h"
#include <image.h>
#include <compiler.h>

struct gp_header {
	uint32_t size;
	uint32_t load_addr;
};

#define GPIMAGE_HEADER_SIZE (sizeof(struct gp_header))

static uint32_t gpimage_swap32(uint32_t data)
{
	return cpu_to_be32(data);
}

static uint8_t gpimage_header[GPIMAGE_HEADER_SIZE];

static int gpimage_check_image_types(uint8_t type)
{
	if (type == IH_TYPE_GPIMAGE)
		return EXIT_SUCCESS;
	else {
		return EXIT_FAILURE;
	}
}

/*
 * Only the simplest image type is currently supported:
 * TOC pointing to CHSETTINGS
 * TOC terminator
 * CHSETTINGS
 *
 * padding to OMAP_CH_HDR_SIZE bytes
 *
 * gp header
 *   size
 *   load_addr
 */
static int valid_gph_size(uint32_t size)
{
	return size;
}

static int valid_gph_load_addr(uint32_t load_addr)
{
	return load_addr;
}

static int gpimage_verify_header(unsigned char *ptr, int image_size,
			struct mkimage_params *params)
{
	struct gp_header *gph = (struct gp_header *)ptr;
	uint32_t offset, size, gph_size, gph_load_addr;

	gph_size = gpimage_swap32(gph->size);
	gph_load_addr = gpimage_swap32(gph->load_addr);

	if (!valid_gph_size(gph_size))
		return -1;
	if (!valid_gph_load_addr(gph_load_addr))
		return -1;
	return 0;
}

static void gpimage_print_header(const void *ptr)
{
	const struct gp_header *gph = (struct gp_header *)ptr;
	uint32_t offset, size, gph_size, gph_load_addr;

	gph_size = gpimage_swap32(gph->size);
	gph_load_addr = gpimage_swap32(gph->load_addr);

	if (!valid_gph_size(gph_size)) {
		fprintf(stderr, "Error: invalid image size %x\n", gph_size);
		exit(EXIT_FAILURE);
	}

	if (!valid_gph_load_addr(gph_load_addr)) {
		fprintf(stderr, "Error: invalid image load address %x\n",
			gph_load_addr);
		exit(EXIT_FAILURE);
	}
	printf("GP Header: Size %x LoadAddr %x\n", gph_size, gph_load_addr);
}

static void gpimage_set_header(void *ptr, struct stat *sbuf, int ifd,
				struct mkimage_params *params)
{
	struct gp_header *gph = (struct gp_header *)ptr;

	gph->size = gpimage_swap32(sbuf->st_size - GPIMAGE_HEADER_SIZE);
	gph->load_addr = gpimage_swap32(params->addr);
}

int gpimage_check_params(struct mkimage_params *params)
{
	return	(params->dflag && (params->fflag || params->lflag)) ||
		(params->fflag && (params->dflag || params->lflag)) ||
		(params->lflag && (params->dflag || params->fflag));
}

/*
 * gpimage parameters
 */
static struct image_type_params gpimage_params = {
	.name		= "TI KeyStone GP Image support",
	.header_size	= GPIMAGE_HEADER_SIZE,
	.hdr		= (void *)&gpimage_header,
	.check_image_type = gpimage_check_image_types,
	.verify_header	= gpimage_verify_header,
	.print_header	= gpimage_print_header,
	.set_header	= gpimage_set_header,
	.check_params	= gpimage_check_params,
};

void init_gpimage_type(void)
{
	mkimage_register(&gpimage_params);
}
