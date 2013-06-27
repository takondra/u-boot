/*
 * Copyright 2013 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <common.h>
#include <asm/arch/hardware.h>

struct mpax {
	u32	mpaxl;
	u32	mpaxh;
};

struct msms_regs {
	u32	pid;
	u32	_res_04;
	u32	smcerrar;
	u32	smcerrxr;
	u32	smedcc;
	u32	smcea;
	u32	smsecc;
	u32	smpfar;
	u32	smpfxr;
	u32	smpfr;
	u32	smpfcr;
	u32	_res_2c;
	u32	sbndc[8];
	u32	sbndm;
	u32	sbnde;
	u32	_res_58;
	u32	cfglck;
	u32	cfgulck;
	u32	cfglckstat;
	u32	sms_mpax_lck;
	u32	sms_mpax_ulck;
	u32	sms_mpax_lckstat;
	u32	ses_mpax_lck;
	u32	ses_mpax_ulck;
	u32	ses_mpax_lckstat;
	u32	smestat;
	u32	smirstat;
	u32	smirc;
	u32	smiestat;
	u32	smiec;
	u32	_res_94_c0[12];
	u32	smncerrar;
	u32	smncerrxr;
	u32	smncea;
	u32	_res_d0_1fc[76];
	struct mpax sms[16][8];
	struct mpax ses[16][8];
};


void share_all_segments(int priv_id)
{
	struct msms_regs*	msmc = (struct msms_regs*) TCI6638_MSMC_CTRL_BASE;
	int j;

	for (j=0; j<8; j++) {
		msmc->sms[priv_id][j].mpaxh &= 0xffffff7ful;
		msmc->ses[priv_id][j].mpaxh &= 0xffffff7ful;
	}
}

void map_ses_segment(int priv_id, int ses_pair,
		     u32 src_pfn, u32 dst_pfn, mpax_seg_size size)
{
	struct msms_regs* msmc = (struct msms_regs*) TCI6638_MSMC_CTRL_BASE;

	msmc->ses[priv_id][ses_pair].mpaxh = src_pfn << 12 | (size & 0x1f) | 0x80;
	msmc->ses[priv_id][ses_pair].mpaxl = dst_pfn << 8 | 0x3f;
}


int do_setmpax_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	u32	src, dst, indx, size;


	if (argc != 5)
		goto mpax_cmd_usage;

	indx = simple_strtoul(argv[1], NULL, 10);
	src  = simple_strtoul(argv[2], NULL, 16);
	dst  = simple_strtoul(argv[3], NULL, 16);
	size = simple_strtoul(argv[4], NULL, 16);

	if (indx > 7)
		goto mpax_cmd_usage;

	src &= 0xfffff;
	dst &= 0xffffff;
	size &= 0x1f;

	if ((size < 0xb) && (size != 0))
		goto mpax_cmd_usage;

	map_ses_segment(8, indx, src, dst, size);

	return 0;

mpax_cmd_usage:
	return cmd_usage(cmdtp);
}

U_BOOT_CMD(
	setmpax,	5,	0,	do_setmpax_cmd,
	"set mpax ses for ARM privid",
	"<pair> <src_pfn in hex> <dst_pfn in hex> <size 0xb (4K), 0xc (8K) and so on up to 0x1F for 4G>\n"
);

