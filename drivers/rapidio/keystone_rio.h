/*
 * Copyright (C) 2010, 2011, 2012 Texas Instruments Incorporated
 * Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 * Copyright (C) 2012 Texas Instruments Incorporated
 * WingMan Kwok <w-kwok2@ti.com>
 * - Updated for support on TI KeyStone platform.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef KEYSTONE_RIO_H
#define KEYSTONE_RIO_H

#include <asm/setup.h>
#include <asm/cache.h>
#include <asm/io.h>

#define BIT(x)  (1 << (x))

/*#########################################################################*/
/*#########################################################################*/
/*#########################################################################*/

/* Direct I/O modes */
#define RIO_DIO_MODE_READ    0x0000
#define RIO_DIO_MODE_WRITER  0x0001
#define RIO_DIO_MODE_WRITE   0x0002
#define RIO_DIO_MODE_SWRITE  0x0003

/*
 * In RapidIO, each device has a 2MB configuration space that is
 * accessed via maintenance transactions.  Portions of configuration
 * space are standardized and/or reserved.
 */
#define RIO_DEV_ID_CAR		        0x00
#define RIO_DEV_INFO_CAR	        0x04
#define RIO_ASM_ID_CAR		        0x08
#define RIO_ASM_ID_MASK		        0xffff0000
#define RIO_ASM_VEN_ID_MASK		0x0000ffff

#define RIO_ASM_INFO_CAR	        0x0c
#define RIO_ASM_REV_MASK		0xffff0000
#define RIO_EXT_FTR_PTR_MASK		0x0000ffff

#define RIO_PEF_CAR		        0x10
#define RIO_PEF_BRIDGE			0x80000000
#define RIO_PEF_MEMORY			0x40000000
#define RIO_PEF_PROCESSOR		0x20000000
#define RIO_PEF_SWITCH			0x10000000
#define RIO_PEF_INB_MBOX		0x00f00000
#define RIO_PEF_INB_MBOX0		0x00800000
#define RIO_PEF_INB_MBOX1		0x00400000
#define RIO_PEF_INB_MBOX2		0x00200000
#define RIO_PEF_INB_MBOX3		0x00100000
#define RIO_PEF_INB_DOORBELL		0x00080000
#define RIO_PEF_EXT_RT			0x00000200
#define RIO_PEF_STD_RT			0x00000100
#define RIO_PEF_CTLS			0x00000010
#define RIO_PEF_FLOW_CONTROL		0x00000080
#define RIO_PEF_EXT_FEATURES		0x00000008
#define RIO_PEF_ADDR_66		        0x00000004
#define RIO_PEF_ADDR_50		        0x00000002
#define RIO_PEF_ADDR_34		        0x00000001

#define RIO_SWP_INFO_CAR	        0x14
#define RIO_SWP_INFO_PORT_TOTAL_MASK	0x0000ff00
#define RIO_SWP_INFO_PORT_NUM_MASK	0x000000ff
#define RIO_GET_TOTAL_PORTS(x)		((x & RIO_SWP_INFO_PORT_TOTAL_MASK) >> 8)

#define RIO_SRC_OPS_CAR		        0x18
#define RIO_SRC_OPS_READ		0x00008000
#define RIO_SRC_OPS_WRITE		0x00004000
#define RIO_SRC_OPS_STREAM_WRITE	0x00002000
#define RIO_SRC_OPS_WRITE_RESPONSE	0x00001000
#define RIO_SRC_OPS_DATA_MSG		0x00000800
#define RIO_SRC_OPS_DOORBELL		0x00000400
#define RIO_SRC_OPS_ATOMIC_TST_SWP	0x00000100
#define RIO_SRC_OPS_ATOMIC_INC		0x00000080
#define RIO_SRC_OPS_ATOMIC_DEC		0x00000040
#define RIO_SRC_OPS_ATOMIC_SET		0x00000020
#define RIO_SRC_OPS_ATOMIC_CLR		0x00000010
#define RIO_SRC_OPS_PORT_WRITE		0x00000004

#define RIO_DST_OPS_CAR		        0x1c
#define RIO_DST_OPS_READ		0x00008000
#define RIO_DST_OPS_WRITE		0x00004000
#define RIO_DST_OPS_STREAM_WRITE	0x00002000
#define RIO_DST_OPS_WRITE_RESPONSE	0x00001000
#define RIO_DST_OPS_DATA_MSG		0x00000800
#define RIO_DST_OPS_DOORBELL		0x00000400
#define RIO_DST_OPS_ATOMIC_TST_SWP	0x00000100
#define RIO_DST_OPS_ATOMIC_INC		0x00000080
#define RIO_DST_OPS_ATOMIC_DEC		0x00000040
#define RIO_DST_OPS_ATOMIC_SET		0x00000020
#define RIO_DST_OPS_ATOMIC_CLR		0x00000010
#define RIO_DST_OPS_PORT_WRITE		0x00000004

#define RIO_OPS_READ			0x00008000
#define RIO_OPS_WRITE			0x00004000
#define RIO_OPS_STREAM_WRITE		0x00002000
#define RIO_OPS_WRITE_RESPONSE		0x00001000
#define RIO_OPS_DATA_MSG		0x00000800
#define RIO_OPS_DOORBELL		0x00000400
#define RIO_OPS_ATOMIC_TST_SWP		0x00000100
#define RIO_OPS_ATOMIC_INC		0x00000080
#define RIO_OPS_ATOMIC_DEC		0x00000040
#define RIO_OPS_ATOMIC_SET		0x00000020
#define RIO_OPS_ATOMIC_CLR		0x00000010
#define RIO_OPS_PORT_WRITE		0x00000004

					/* 0x20-0x30 *//* Reserved */

#define	RIO_SWITCH_RT_LIMIT	        0x34
#define	RIO_RT_MAX_DESTID		0x0000ffff

#define RIO_MBOX_CSR		        0x40
#define RIO_MBOX0_AVAIL		        0x80000000
#define RIO_MBOX0_FULL			0x40000000
#define RIO_MBOX0_EMPTY		        0x20000000
#define RIO_MBOX0_BUSY			0x10000000
#define RIO_MBOX0_FAIL			0x08000000
#define RIO_MBOX0_ERROR		        0x04000000
#define RIO_MBOX1_AVAIL		        0x00800000
#define RIO_MBOX1_FULL			0x00200000
#define RIO_MBOX1_EMPTY		        0x00200000
#define RIO_MBOX1_BUSY			0x00100000
#define RIO_MBOX1_FAIL			0x00080000
#define RIO_MBOX1_ERROR		        0x00040000
#define RIO_MBOX2_AVAIL		        0x00008000
#define RIO_MBOX2_FULL			0x00004000
#define RIO_MBOX2_EMPTY		        0x00002000
#define RIO_MBOX2_BUSY			0x00001000
#define RIO_MBOX2_FAIL			0x00000800
#define RIO_MBOX2_ERROR		        0x00000400
#define RIO_MBOX3_AVAIL		        0x00000080
#define RIO_MBOX3_FULL			0x00000040
#define RIO_MBOX3_EMPTY		        0x00000020
#define RIO_MBOX3_BUSY			0x00000010
#define RIO_MBOX3_FAIL			0x00000008
#define RIO_MBOX3_ERROR		        0x00000004

#define RIO_WRITE_PORT_CSR	        0x44
#define RIO_DOORBELL_CSR	        0x44
#define RIO_DOORBELL_AVAIL		0x80000000
#define RIO_DOORBELL_FULL		0x40000000
#define RIO_DOORBELL_EMPTY		0x20000000
#define RIO_DOORBELL_BUSY		0x10000000
#define RIO_DOORBELL_FAILED		0x08000000
#define RIO_DOORBELL_ERROR		0x04000000
#define RIO_WRITE_PORT_AVAILABLE	0x00000080
#define RIO_WRITE_PORT_FULL		0x00000040
#define RIO_WRITE_PORT_EMPTY		0x00000020
#define RIO_WRITE_PORT_BUSY		0x00000010
#define RIO_WRITE_PORT_FAILED		0x00000008
#define RIO_WRITE_PORT_ERROR		0x00000004

					/* 0x48 *//* Reserved */

#define RIO_PELL_CTRL_CSR	        0x4c
#define RIO_PELL_ADDR_66		0x00000004
#define RIO_PELL_ADDR_50		0x00000002
#define RIO_PELL_ADDR_34		0x00000001

					/* 0x50-0x54 *//* Reserved */

#define RIO_LCSH_BA		        0x58
#define RIO_LCSL_BA		        0x5c

#define RIO_DID_CSR		        0x60

					/* 0x64 *//* Reserved */

#define RIO_HOST_DID_LOCK_CSR	        0x68
#define RIO_COMPONENT_TAG_CSR	        0x6c

#define RIO_STD_RTE_CONF_DESTID_SEL_CSR 0x70
#define RIO_STD_RTE_CONF_PORT_SEL_CSR   0x74
#define RIO_STD_RTE_DEFAULT_PORT        0x78

                                  /* 0x7c-0xf8 *//* Reserved */
		/* 0x100-0xfff8 *//* [I] Extended Features Space */

#define RIO_PORT_LINK_MAINT_REQ_CSR     0x140
#define RIO_PORT_LINK_MAINT_RESP_CSR    0x144
#define RIO_PORT_LOCAL_ACKID_CSR        0x148
#define RIO_PORT_ERR_STAT_CSR           0x158
#define RIO_PORT_CTRL_CSR               0x15c

#define RIO_PORT_OFFSET                 0x20

#define RIO_PORT_CTRL_TYPE_SERIAL       0x00000001
#define RIO_PORT_CTRL_ENUM_BOUNDARY     0x00020000
#define RIO_PORT_CTRL_MCAST_EVT_PART    0x00080000
#define RIO_PORT_CTRL_ERR_CHECK_DIS     0x00100000
#define RIO_PORT_CTRL_INPUT_PORT_EN     0x00200000
#define RIO_PORT_CTRL_OUTPUT_PORT_EN    0x00400000
#define RIO_PORT_CTRL_PORT_DIS          0x00800000

		/* 0x10000-0xfffff8 *//* [I] Implementation-defined Space */

/*
 * Extended Features Space is a configuration space area where
 * functionality is mapped into extended feature blocks via a
 * singly linked list of extended feature pointers (EFT_PTR).
 *
 * Each extended feature block can be identified/located in
 * Extended Features Space by walking the extended feature
 * list starting with the Extended Feature Pointer located
 * in the Assembly Information CAR.
 *
 * Extended Feature Blocks (EFBs) are identified with an assigned
 * EFB ID. Extended feature block offsets in the definitions are
 * relative to the offset of the EFB within the  Extended Features
 * Space.
 */

/* Helper macros to parse the Extended Feature Block header */
#define RIO_EFB_PTR_MASK	        0xffff0000
#define RIO_EFB_ID_MASK		        0x0000ffff
#define RIO_GET_BLOCK_PTR(x)	        ((x & RIO_EFB_PTR_MASK) >> 16)
#define RIO_GET_BLOCK_ID(x)	        (x & RIO_EFB_ID_MASK)

/* Extended Feature Block IDs */
#define RIO_EFB_PAR_EP_ID	        0x0001
#define RIO_EFB_PAR_EP_REC_ID	        0x0002
#define RIO_EFB_PAR_EP_FREE_ID	        0x0003
#define RIO_EFB_SER_EP_ID_V13P	        0x0001
#define RIO_EFB_SER_EP_REC_ID_V13P	0x0002
#define RIO_EFB_SER_EP_FREE_ID_V13P	0x0003
#define RIO_EFB_SER_EP_ID	        0x0004
#define RIO_EFB_SER_EP_REC_ID	        0x0005
#define RIO_EFB_SER_EP_FREE_ID	        0x0006
#define RIO_EFB_SER_EP_FREC_ID          0x0009
#define RIO_EFB_ERR_MGMNT	        0x0007

/*
 * Physical 8/16 LP-LVDS
 * ID=0x0001, Generic End Point Devices
 * ID=0x0002, Generic End Point Devices, software assisted recovery option
 * ID=0x0003, Generic End Point Free Devices
 *
 * Physical LP-Serial
 * ID=0x0004, Generic End Point Devices
 * ID=0x0005, Generic End Point Devices, software assisted recovery option
 * ID=0x0006, Generic End Point Free Devices
 */
#define RIO_PORT_MNT_HEADER		0x0000
#define RIO_PORT_REQ_CTL_CSR		0x0020
#define RIO_PORT_RSP_CTL_CSR		0x0024	/* 0x0001/0x0002 */
#define RIO_PORT_LINKTO_CTL_CSR		0x0020	/* Serial */
#define RIO_PORT_RSPTO_CTL_CSR		0x0024	/* Serial */
#define RIO_PORT_GEN_CTL_CSR		0x003c
#define  RIO_PORT_GEN_HOST		0x80000000
#define  RIO_PORT_GEN_MASTER		0x40000000
#define  RIO_PORT_GEN_DISCOVERED	0x20000000
#define RIO_PORT_N_MNT_REQ_CSR(x)	(0x0040 + x*0x20)
#define RIO_PORT_N_MNT_RSP_CSR(x)	(0x0044 + x*0x20)
#define  RIO_PORT_N_MNT_RSP_RVAL        0x80000000
#define  RIO_PORT_N_MNT_RSP_ASTAT	0x000003e0
#define  RIO_PORT_N_MNT_RSP_LSTAT	0x0000001f
#define RIO_PORT_N_ACK_STS_CSR(x)	(0x0048 + x*0x20)
#define  RIO_PORT_N_ACK_CLEAR		0x80000000
#define  RIO_PORT_N_ACK_INBOUND		0x1f000000
#define  RIO_PORT_N_ACK_OUTSTAND	0x00001f00
#define  RIO_PORT_N_ACK_OUTBOUND	0x0000001f
#define RIO_PORT_N_ERR_STS_CSR(x)	(0x0058 + x*0x20)
#define  RIO_PORT_N_ERR_STS_PW_OUT_ES	0x00010000 /* Output Error-stopped */
#define  RIO_PORT_N_ERR_STS_PW_INP_ES	0x00000100 /* Input Error-stopped */
#define  RIO_PORT_N_ERR_STS_PW_PEND	0x00000010 /* Port-Write Pending */
#define  RIO_PORT_N_ERR_STS_PORT_ERR	0x00000004
#define  RIO_PORT_N_ERR_STS_PORT_OK	0x00000002
#define  RIO_PORT_N_ERR_STS_PORT_UNINIT	0x00000001
#define  RIO_PORT_N_ERR_STS_CLR_MASK	0x07120204
#define RIO_PORT_N_CTL_CSR(x)		(0x005c + x*0x20)
#define  RIO_PORT_N_CTL_PWIDTH		0xc0000000
#define  RIO_PORT_N_CTL_PWIDTH_1	0x00000000
#define  RIO_PORT_N_CTL_PWIDTH_4	0x40000000
#define  RIO_PORT_N_CTL_P_TYP_SER	0x00000001
#define  RIO_PORT_N_CTL_LOCKOUT		0x00000002
#define  RIO_PORT_N_CTL_EN_RX_SER	0x00200000
#define  RIO_PORT_N_CTL_EN_TX_SER	0x00400000
#define  RIO_PORT_N_CTL_EN_RX_PAR	0x08000000
#define  RIO_PORT_N_CTL_EN_TX_PAR	0x40000000

/*
 * Error Management Extensions (RapidIO 1.3+, Part 8)
 *
 * Extended Features Block ID=0x0007
 */

/* General EM Registers (Common for all Ports) */

#define RIO_EM_EFB_HEADER	0x000
#define RIO_EM_LTL_ERR_DETECT	0x008
#define RIO_EM_LTL_ERR_EN	0x00c
#define RIO_EM_LTL_HIADDR_CAP	0x010
#define RIO_EM_LTL_ADDR_CAP	0x014
#define RIO_EM_LTL_DEVID_CAP	0x018
#define RIO_EM_LTL_CTRL_CAP	0x01c
#define RIO_EM_PW_TGT_DEVID	0x028
#define RIO_EM_PKT_TTL		0x02c

/* Per-Port EM Registers */

#define RIO_EM_PN_ERR_DETECT(x)	(0x040 + x*0x40)
#define  REM_PED_IMPL_SPEC	0x80000000
#define  REM_PED_LINK_TO	0x00000001
#define RIO_EM_PN_ERRRATE_EN(x) (0x044 + x*0x40)
#define RIO_EM_PN_ATTRIB_CAP(x)	(0x048 + x*0x40)
#define RIO_EM_PN_PKT_CAP_0(x)	(0x04c + x*0x40)
#define RIO_EM_PN_PKT_CAP_1(x)	(0x050 + x*0x40)
#define RIO_EM_PN_PKT_CAP_2(x)	(0x054 + x*0x40)
#define RIO_EM_PN_PKT_CAP_3(x)	(0x058 + x*0x40)
#define RIO_EM_PN_ERRRATE(x)	(0x068 + x*0x40)
#define RIO_EM_PN_ERRRATE_TR(x) (0x06c + x*0x40)

/*#########################################################################*/
/*#########################################################################*/
/*#########################################################################*/



#define KEYSTONE_RIO_MAP_FLAG_SEGMENT		BIT(0)
#define KEYSTONE_RIO_MAP_FLAG_SRC_PROMISC	BIT(1)
#define KEYSTONE_RIO_MAP_FLAG_TT_16		BIT(13)
#define KEYSTONE_RIO_MAP_FLAG_DST_PROMISC	BIT(15)
#define KEYSTONE_RIO_DESC_FLAG_TT_16		BIT(9)

#define KEYSTONE_RIO_BOOT_COMPLETE		BIT(24)
#define KEYSTONE_RIO_PER_EN			BIT(2)
#define KEYSTONE_RIO_PER_FREE			BIT(0)
#define KEYSTONE_RIO_PEF_FLOW_CONTROL		BIT(7)

/*
 * Packet types
 */
#define KEYSTONE_RIO_PACKET_TYPE_NREAD    0x24
#define KEYSTONE_RIO_PACKET_TYPE_NWRITE   0x54
#define KEYSTONE_RIO_PACKET_TYPE_NWRITE_R 0x55
#define KEYSTONE_RIO_PACKET_TYPE_SWRITE   0x60
#define KEYSTONE_RIO_PACKET_TYPE_DBELL    0xa0
#define KEYSTONE_RIO_PACKET_TYPE_MAINT_R  0x80
#define KEYSTONE_RIO_PACKET_TYPE_MAINT_W  0x81
#define KEYSTONE_RIO_PACKET_TYPE_MAINT_RR 0x82
#define KEYSTONE_RIO_PACKET_TYPE_MAINT_WR 0x83
#define KEYSTONE_RIO_PACKET_TYPE_MAINT_PW 0x84

/*
 * LSU defines
 */
#define KEYSTONE_RIO_LSU_PRIO             0

#define KEYSTONE_RIO_LSU_BUSY_MASK        BIT(31)
#define KEYSTONE_RIO_LSU_FULL_MASK        BIT(30)

#define KEYSTONE_RIO_LSU_CC_MASK          0x0f
#define KEYSTONE_RIO_LSU_CC_TIMEOUT       0x01
#define KEYSTONE_RIO_LSU_CC_XOFF          0x02
#define KEYSTONE_RIO_LSU_CC_ERROR         0x03
#define KEYSTONE_RIO_LSU_CC_INVALID       0x04
#define KEYSTONE_RIO_LSU_CC_DMA           0x05
#define KEYSTONE_RIO_LSU_CC_RETRY         0x06
#define KEYSTONE_RIO_LSU_CC_CANCELED      0x07

/* Mask for receiving both error and good completion LSU interrupts */
#define KEYSTONE_RIO_ICSR_LSU0(src_id)    ((0x10001) << (src_id))

/*
 * Various RIO defines
 */
#define KEYSTONE_RIO_TIMEOUT_CNT	1000
#define KEYSTONE_RIO_REGISTER_DELAY	(2*HZ)

/*
 * RIO error, reset and special event interrupt defines
 */
#define KEYSTONE_RIO_PORT_ERROR_OUT_PKT_DROP		BIT(26)
#define KEYSTONE_RIO_PORT_ERROR_OUT_FAILED		BIT(25)
#define KEYSTONE_RIO_PORT_ERROR_OUT_DEGRADED		BIT(24)
#define KEYSTONE_RIO_PORT_ERROR_OUT_RETRY		BIT(20)
#define KEYSTONE_RIO_PORT_ERROR_OUT_ERROR		BIT(17)
#define KEYSTONE_RIO_PORT_ERROR_IN_ERROR		BIT(9)
#define KEYSTONE_RIO_PORT_ERROR_PW_PENDING		BIT(4)
#define KEYSTONE_RIO_PORT_ERROR_PORT_ERR		BIT(2)

#define KEYSTONE_RIO_PORT_ERROR_MASK   \
	(KEYSTONE_RIO_PORT_ERROR_OUT_PKT_DROP	|\
	KEYSTONE_RIO_PORT_ERROR_OUT_FAILED	|\
	KEYSTONE_RIO_PORT_ERROR_OUT_DEGRADED	|\
	KEYSTONE_RIO_PORT_ERROR_OUT_RETRY	|\
	KEYSTONE_RIO_PORT_ERROR_OUT_ERROR	|\
	KEYSTONE_RIO_PORT_ERROR_IN_ERROR	|\
	KEYSTONE_RIO_PORT_ERROR_PW_PENDING	|\
	KEYSTONE_RIO_PORT_ERROR_PORT_ERR)


#define KEYSTONE_RIO_SP_HDR_NEXT_BLK_PTR	0x1000
#define KEYSTONE_RIO_SP_HDR_EP_REC_ID		0x0002
#define KEYSTONE_RIO_ERR_HDR_NEXT_BLK_PTR	0x3000
#define KEYSTONE_RIO_ERR_EXT_FEAT_ID		0x0007

/*
 * RapidIO global definitions
 */
#define KEYSTONE_RIO_MAX_PORT		4
#define KEYSTONE_RIO_BLK_NUM		9
#define KEYSTONE_RIO_MAX_MBOX		4	/* 4 in multi-segment,
						64 in single-segment */

#define KEYSTONE_RIO_MAINT_BUF_SIZE	64
#define KEYSTONE_RIO_MSG_SSIZE		0xe
#define KEYSTONE_RIO_SGLIST_SIZE	3

/*
 * Dev Id and dev revision
 */
#define KEYSTONE_RIO_DEV_ID_VAL	\
	((((__raw_readl(krio_priv->jtagid_reg)) << 4)  & 0xffff0000) | 0x30)

#define KEYSTONE_RIO_DEV_INFO_VAL \
	(((__raw_readl(krio_priv->jtagid_reg)) >> 28) & 0xf)

#define KEYSTONE_RIO_ID_TI		(0x00000030)
#define KEYSTONE_RIO_EXT_FEAT_PTR	(0x00000100)

/*
 * Maximum message size fo RIONET
 */
#define MACH_RIO_MAX_MSG_SIZE   1552

/*
 * Definition of the different RapidIO packet types according to the RapidIO
 * specification 2.0
 */
#define RIO_PACKET_TYPE_STREAM  9  /* Data Streaming */
#define RIO_PACKET_TYPE_MESSAGE 11 /* Message */

/*
 * SerDes configurations
 */
struct keystone_serdes_config {
	u32 cfg_cntl;            /* setting control register config */
	u16 serdes_cfg_pll;      /* SerDes PLL configuration */
	u16 prescalar_srv_clk;   /* prescalar fo ip_clk */

	/* SerDes receive channel configuration (per-port) */
	u32 rx_chan_config[KEYSTONE_RIO_MAX_PORT];

	/* SerDes transmit channel configuration (per-port) */
	u32 tx_chan_config[KEYSTONE_RIO_MAX_PORT];

	u32 path_mode[KEYSTONE_RIO_MAX_PORT];      /* path config for SerDes */
};

/*
 * Per board RIO devices controller configuration
 */
struct keystone_rio_board_controller_info {
	u32		rio_regs_base;
	u32		rio_regs_size;

	u32		boot_cfg_regs_base;
	u32		boot_cfg_regs_size;

	u16 ports;      /* bitfield of port(s) to probe on this controller */
	u16 mode;       /* hw mode (default serdes config).
			   index into serdes_config[] */
	u16 id;         /* host id */
	u16 size;       /* RapidIO common transport system size.
			 * 0 - Small size. 256 devices.
			 * 1 - Large size, 65536 devices. */
	u16 serdes_config_num;
	struct keystone_serdes_config serdes_config[4];
};

struct keystone_rio_data;

/*
 * RapidIO Registers
 */

struct keystone_srio_serdes_regs {
	u32	pll;

	struct {
		u32	rx;
		u32	tx;
	} channel[4];
};

/* RIO Registers  0000 - 2fff */
struct keystone_rio_regs {
/* Required Peripheral Registers */
	u32	pid;			/* 0000 */
	u32	pcr;			/* 0004 */
	u32	__rsvd0[3];		/* 0008 - 0010 */

/* Peripheral Settting Control Registers */
	u32	per_set_cntl;		/* 0014 */
	u32	per_set_cntl1;		/* 0018 */

	u32	__rsvd1[2];		/* 001c - 0020 */

	u32	gbl_en;			/* 0024 */
	u32	gbl_en_stat;		/* 0028 */

	struct {
		u32 enable;		/* 002c */
		u32 status;		/* 0030 */
	} blk[10];			/* 002c - 0078 */

	/* ID Registers */
	u32	__rsvd2[17];		/* 007c - 00bc */
	u32	multiid_reg[8];		/* 00c0 - 00dc */

/* Hardware Packet Forwarding Registers */
	struct {
		u32	pf_16b;
		u32	pf_8b;
	} pkt_fwd_cntl[8];		/* 00e0 - 011c */

	u32	__rsvd3[24];		/* 0120 - 017c */

/* Interrupt Registers */
	struct {
		u32	status;
		u32	__rsvd0;
		u32	clear;
		u32	__rsvd1;
	} doorbell_int[4];		/* 0180 - 01bc */

	struct {
		u32	status;
		u32	__rsvd0;
		u32	clear;
		u32	__rsvd1;
	} lsu_int[2];			/* 01c0 - 01dc */

	u32	err_rst_evnt_int_stat;	/* 01e0 */
	u32	__rsvd4;
	u32	err_rst_evnt_int_clear;	/* 01e8 */
	u32	__rsvd5;

	u32	__rsvd6[4];		/* 01f0 - 01fc */

	struct {
		u32 route;		/* 0200 */
		u32 route2;		/* 0204 */
		u32 __rsvd;		/* 0208 */
	} doorbell_int_route[4];	/* 0200 - 022c */

	u32	lsu0_int_route[4];		/* 0230 - 023c */
	u32	lsu1_int_route1;		/* 0240 */

	u32	__rsvd7[3];		/* 0244 - 024c */

	u32	err_rst_evnt_int_route[3];	/* 0250 - 0258 */

	u32	__rsvd8[2];		/* 025c - 0260 */

	u32	interupt_ctl;		/* 0264 */

	u32	__rsvd9[26];		/* 0268, 026c, 0270 - 02cc */

	u32	intdst_rate_cntl[16];	/* 02d0 - 030c */
	u32	intdst_rate_disable;	/* 0310 */

	u32	__rsvd10[59];		/* 0314 - 03fc */

/* RXU Registers */
	struct {
		u32	ltr_mbox_src;
		u32	dest_prom_seg;
		u32	flow_qid;
	} rxu_map[64];			/* 0400 - 06fc */

	struct {
		u32	cos_src;
		u32	dest_prom;
		u32	stream;
	} rxu_type9_map[64];		/* 0700 - 09fc */

	u32	__rsvd11[192];		/* 0a00 - 0cfc */

/* LSU/MAU Registers */
	struct {
		u32 addr_msb;		/* 0d00 */
		u32 addr_lsb_cfg_ofs;	/* 0d04 */
		u32 dsp_addr;		/* 0d08 */
		u32 dbell_val_byte_cnt;	/* 0d0c */
		u32 destid;		/* 0d10 */
		u32 dbell_info_fttype;	/* 0d14 */
		u32 busy_full;		/* 0d18 */
	} lsu_reg[8];			/* 0d00 - 0ddc */

	u32	lsu_setup_reg[2];	/* 0de0 - 0de4 */
	u32	lsu_stat_reg[6];	/* 0de8 - 0dfc */
	u32	lsu_flow_masks[4];	/* 0e00 - 0e0c */

	u32	__rsvd12[16];		/* 0e10 - 0e4c */

/* Flow Control Registers */
	u32	flow_cntl[16];		/* 0e50 - 0e8c */
	u32	__rsvd13[8];		/* 0e90 - 0eac */

/* TXU Registers 0eb0 - 0efc */
	u32	tx_cppi_flow_masks[8];	/* 0eb0 - 0ecc */
	u32	tx_queue_sch_info[4];	/* 0ed0 - 0edc */
	u32	garbage_coll_qid[3];	/* 0ee0 - 0ee8 */

	u32	__rsvd14[69];		/* 0eec, 0ef0 - 0ffc */

};

/* CDMAHP Registers 1000 - 2ffc */
struct keystone_rio_pktdma_regs {
	u32	__rsvd[2048];		/* 1000 - 2ffc */
};

/* CSR/CAR Registers  b000+ */
struct keystone_rio_car_csr_regs {
	u32	dev_id;			/* b000 */
	u32	dev_info;		/* b004 */
	u32	assembly_id;		/* b008 */
	u32	assembly_info;		/* b00c */
	u32	pe_feature;		/* b010 */

	u32	sw_port;		/* b014 */

	u32	src_op;			/* b018 */
	u32	dest_op;		/* b01c */

	u32	__rsvd1[7];		/* b020 - b038 */

	u32	data_stm_info;		/* b03c */

	u32	__rsvd2[2];		/* b040 - b044 */

	u32	data_stm_logical_ctl;	/* b048 */
	u32	pe_logical_ctl;		/* b04c */

	u32	__rsvd3[2];		/* b050 - b054 */

	u32	local_cfg_hbar;		/* b058 */
	u32	local_cfg_bar;		/* b05c */

	u32	base_dev_id;		/* b060 */
	u32	__rsvd4;
	u32	host_base_id_lock;	/* b068 */
	u32	component_tag;		/* b06c */
					/* b070 - b0fc */
};

struct keystone_rio_serial_port_regs {
	u32	sp_maint_blk_hdr;	/* b100 */
	u32	__rsvd6[7];		/* b104 - b11c */

	u32	sp_link_timeout_ctl;	/* b120 */
	u32	sp_rsp_timeout_ctl;	/* b124 */
	u32	__rsvd7[5];		/* b128 - b138 */
	u32	sp_gen_ctl;		/* b13c */

	struct {
		u32	link_maint_req;	/* b140 */
		u32	link_maint_resp;/* b144 */
		u32	ackid_stat;	/* b148 */
		u32	__rsvd[2];	/* b14c - b150 */
		u32	ctl2;		/* b154 */
		u32	err_stat;	/* b158 */
		u32	ctl;		/* b15c */
	} sp[4];			/* b140 - b1bc */

					/* b1c0 - bffc */
};

struct keystone_rio_err_mgmt_regs {
	u32	err_report_blk_hdr;	/* c000 */
	u32	__rsvd9;
	u32	err_det;		/* c008 */
	u32	err_en;			/* c00c */
	u32	h_addr_capt;		/* c010 */
	u32	addr_capt;		/* c014 */
	u32	id_capt;		/* c018 */
	u32	ctrl_capt;		/* c01c */
	u32	__rsvd10[2];		/* c020 - c024 */
	u32	port_write_tgt_id;	/* c028 */
	u32	__rsvd11[5];		/* c02c - c03c */

	struct {
		u32	det;		/* c040 */
		u32	rate_en;	/* c044 */
		u32	attr_capt_dbg0;	/* c048 */
		u32	capt_0_dbg1;	/* c04c */
		u32	capt_1_dbg2;	/* c050 */
		u32	capt_2_dbg3;	/* c054 */
		u32	capt_3_dbg4;	/* c058 */
		u32	__rsvd0[3];	/* c05c - c064 */
		u32	rate;		/* c068 */
		u32	thresh;		/* c06c */
		u32	__rsvd1[4];	/* c070 - c07c */
	} sp_err[4];			/* c040 - c13c */

	u32	__rsvd12[1972];		/* c140 - e00c */

	struct {
		u32	stat0;		/* e010 */
		u32	stat1;		/* e014 */
		u32	__rsvd[6];	/* e018 - e02c */
	} lane_stat[4];			/* e010 - e08c */

					/* e090 - 1affc */
};

struct keystone_rio_phy_layer_regs {
	u32	phy_blk_hdr;		/* 1b000 */
	u32	__rsvd14[31];		/* 1b004 - 1b07c */
	struct {
		u32	imp_spec_ctl;	/* 1b080 */
		u32	pwdn_ctl;	/* 1b084 */
		u32	__rsvd0[2];

		u32	status;		/* 1b090 */
		u32	int_enable;	/* 1b094 */
		u32	port_wr_enable;	/* 1b098 */
		u32	event_gen;	/* 1b09c */

		u32	all_int_en;	/* 1b0a0 */
		u32	all_port_wr_en;	/* 1b0a4 */
		u32	__rsvd1[2];

		u32	path_ctl;	/* 1b0b0 */
		u32	discovery_timer;/* 1b0b4 */
		u32	silence_timer;	/* 1b0b8 */
		u32	vmin_exp;	/* 1b0bc */

		u32	pol_ctl;	/* 1b0c0 */
		u32	__rsvd2;
		u32	denial_ctl;	/* 1b0c8 */
		u32	__rsvd3;

		u32	rcvd_mecs;	/* 1b0d0 */
		u32	__rsvd4;
		u32	mecs_fwd;	/* 1b0d8 */
		u32	__rsvd5;

		u32	long_cs_tx1;	/* 1b0e0 */
		u32	long_cs_tx2;	/* 1b0e4 */
		u32	__rsvd[6];	/* 1b0e8, 1b0ec, 1b0f0 - 1b0fc */
	} phy_sp[4];			/* 1b080 - 1b27c */

					/* 1b280 - 1b2fc */
};

struct keystone_rio_transport_layer_regs {
	u32	transport_blk_hdr;	/* 1b300 */
	u32	__rsvd16[31];		/* 1b304 - 1b37c */

	struct {
		u32	control;	/*1b380 */
		u32	__rsvd0[3];

		u32	status;		/* 1b390 */
		u32	int_enable;	/* 1b394 */
		u32	port_wr_enable;	/* 1b398 */
		u32	event_gen;	/* 1b39c */

		struct {
			u32	ctl;		/* 1b3a0 */
			u32	pattern_match;	/* 1b3a4 */
			u32	__rsvd[2];	/* 1b3a8 - 1b3ac */
		} base_route[4];		/* 1b3a0 - 1b3dc */

		u32	__rsvd1[8];		/* 1b3e0 - 1b3fc */

	} transport_sp[4];			/* 1b380 - 1b57c */

						/* 1b580 - 1b5fc */
};

struct keystone_rio_pkt_buf_regs {
	u32	pkt_buf_blk_hdr;	/* 1b600 */
	u32	__rsvd18[31];		/* 1b604 - 1b67c */

	struct {
		u32	control;	/* 1b680 */
		u32	__rsvd0[3];

		u32	status;		/* 1b690 */
		u32	int_enable;	/* 1b694 */
		u32	port_wr_enable;	/* 1b698 */
		u32	event_gen;	/* 1b69c */

		u32	ingress_rsc;	/* 1b6a0 */
		u32	egress_rsc;	/* 1b6a4 */
		u32	__rsvd1[2];

		u32	ingress_watermark[4];	/* 1b6b0 - 1b6bc */
		u32	__rsvd2[16];	/* 1b6c0 - 1b6fc */

	} pkt_buf_sp[4];		/* 1b680 - 1b87c */

					/* 1b880 - 1b8fc */
};

struct keystone_rio_evt_mgmt_regs {
	u32	evt_mgmt_blk_hdr;	/* 1b900 */
	u32	__rsvd20[3];

	u32	evt_mgmt_int_stat;	/* 1b910 */
	u32	evt_mgmt_int_enable;	/* 1b914 */
	u32	evt_mgmt_int_port_stat;	/* 1b918 */
	u32	__rsvd21;

	u32	evt_mgmt_port_wr_stat;	/* 1b920 */
	u32	evt_mgmt_port_wr_enable;/* 1b924 */
	u32	evt_mgmt_port_wr_port_stat;	/* 1b928 */
	u32	__rsvd22;

	u32	evt_mgmt_dev_int_en;	/* 1b930 */
	u32	evt_mgmt_dev_port_wr_en;	/* 1b934 */
	u32	__rsvd23;
	u32	evt_mgmt_mecs_stat;	/* 1b93c */

	u32	evt_mgmt_mecs_int_en;	/* 1b940 */
	u32	evt_mgmt_mecs_cap_en;	/* 1b944 */
	u32	evt_mgmt_mecs_trig_en;	/* 1b948 */
	u32	evt_mgmt_mecs_req;	/* 1b94c */

	u32	evt_mgmt_mecs_port_stat;/* 1b950 */
	u32	__rsvd24[2];
	u32	evt_mgmt_mecs_event_gen;/* 1b95c */

	u32	evt_mgmt_rst_port_stat;	/* 1b960 */
	u32	__rsvd25;
	u32	evt_mgmt_rst_int_en;	/* 1b968 */
	u32	__rsvd26;

	u32	evt_mgmt_rst_port_wr_en;/* 1b970 */
					/* 1b974 - 1b9fc */
};

struct keystone_rio_port_write_regs {
	u32	port_wr_blk_hdr;	/* 1ba00 */
	u32	port_wr_ctl;		/* 1ba04 */
	u32	port_wr_route;		/* 1ba08 */
	u32	__rsvd28;

	u32	port_wr_rx_stat;	/* 1ba10 */
	u32	port_wr_rx_event_gen;	/* 1ba14 */
	u32	__rsvd29[2];

	u32	port_wr_rx_capt[4];	/* 1ba20 - 1ba2c */
					/* 1ba30 - 1bcfc */
};

struct keystone_rio_link_layer_regs {
	u32	link_blk_hdr;		/* 1bd00 */
	u32	__rsvd31[8];		/* 1bd04 - 1bd20 */
	u32	whiteboard;		/* 1bd24 */
	u32	port_number;		/* 1bd28 */

	u32	__rsvd32;		/* 1bd2c */

	u32	prescalar_srv_clk;	/* 1bd30 */
	u32	reg_rst_ctl;		/* 1bd34 */
	u32	__rsvd33[4];		/* 1bd38, 1bd3c, 1bd40, 1bd44 */
	u32	local_err_det;		/* 1bd48 */
	u32	local_err_en;		/* 1bd4c */

	u32	local_h_addr_capt;	/* 1bd50 */
	u32	local_addr_capt;	/* 1bd54 */
	u32	local_id_capt;		/* 1bd58 */
	u32	local_ctrl_capt;	/* 1bd5c */

					/* 1bd60 - 1bdfc */
};

struct keystone_rio_fabric_regs {
	u32	fabric_hdr;		/* 1be00 */
	u32	__rsvd35[3];		/* 1be04 - 1be0c */

	u32	fabric_csr;		/* 1be10 */
	u32	__rsvd36[11];		/* 1be14 - 1be3c */

	u32	sp_fabric_status[4];	/* 1be40 - 1be4c */
};

#ifdef CONFIG_RIONET
/* built-in rionet */
extern int  rionet_init(void);
extern void rionet_exit(void);
#endif

#endif /* KEYSTONE_RIO_H */

