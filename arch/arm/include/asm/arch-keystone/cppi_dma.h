/*
 * Copyright (C) 2013 Texas Instruments Incorporated
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

#ifndef _CPPI_DMA_H_
#define _CPPI_DMA_H_

#include <asm/arch/hardware.h>
#include <asm/io.h>

#define BOOTBITMASK(x,y)      (   (   (  ((u32)1 << (((u32)x)-((u32)y)+(u32)1) ) - (u32)1 )   )   <<  ((u32)y)   )
#define BOOT_READ_BITFIELD(z,x,y)   (((u32)z) & BOOTBITMASK(x,y)) >> (y)
#define BOOT_SET_BITFIELD(z,f,x,y)  (((u32)z) & ~BOOTBITMASK(x,y)) | ( (((u32)f) << (y)) & BOOTBITMASK(x,y) )

#define MAX_SIZE_STREAM_BUFFER  1520

#define DEVICE_QM_FREE_Q                910
#define DEVICE_QM_LNK_BUF_Q             911
#define DEVICE_QM_RCV_Q                 912
#define DEVICE_QM_TX_Q                  913
//#define DEVICE_QM_PA_CFG_Q              640
#define DEVICE_QM_ETH_TX_Q              648

/******************************************************************************/
/******************************************************************************/

/* return values */
#define QM_OK                                0
#define QM_ERR                               -1

/* The driver supports only a single descriptor size */
#define QM_DESC_SIZE_BYTES      64

/* Descriptor values */
/* Descriptor Info: Descriptor type is host with any protocol specific info in the descriptor */
#define QM_DESC_TYPE_HOST           0
#define QM_DESC_PSINFO_IN_DESCR     0
#define QM_DESC_DEFAULT_DESCINFO       (QM_DESC_TYPE_HOST << 30)    |  \
                                       (QM_DESC_PSINFO_IN_DESCR << 22)
#define QM_DESC_INFO_GET_PSINFO_LOC(x)  BOOT_READ_BITFIELD((x), 22, 22)

#define QM_DESC_DESCINFO_SET_PKT_LEN(x,v)  (x) = BOOT_SET_BITFIELD((x), (v), 21, 0)
#define QM_DESC_DESCINFO_GET_PKT_LEN(x)    BOOT_READ_BITFIELD((x), 21, 0)


#define QM_QA_ENTRY_COUNT_MSB  18
#define QM_QA_ENTRY_COUNT_LSB   0

/* Packet Info */
#define QM_DESC_PINFO_EPIB              1
#define QM_DESC_PINFO_RETURN_OWN        1
#define QM_DESC_DEFAULT_PINFO           (QM_DESC_PINFO_EPIB << 31)          |   \
                                        (QM_DESC_PINFO_RETURN_OWN << 15)
#define QM_PKT_INFO_GET_EPIB(x)         BOOT_READ_BITFIELD((x), 31, 31)
#define QM_PKT_INFO_SET_PSINFO_SIZE(x,v)    (x) = BOOT_SET_BITFIELD((x), (v), 29, 24)

#define QM_DESC_PINFO_SET_QM(x,v)       (x) = BOOT_SET_BITFIELD((x), (v), 13, 12)
#define QM_DESC_PINFO_SET_QUEUE(x,v)    (x) = BOOT_SET_BITFIELD((x), (v), 11,  0)

/* QM setup configuration */
struct qm_config {
	u32 link_ram_base;
	u32 link_ram_size;
	u32 mem_region_base;
	u32 mem_reg_num_descs;
	u32 dest_q;
};

struct qm_host_desc {
	u32 desc_info;
	u32 tag_info;
	u32 packet_info;
	u32 buff_len;
	u32 buff_ptr;
	u32 next_bdptr;
	u32 orig_buff_len;
	u32 orig_buff_ptr;
	u32 timestamp;
	u32 swinfo0;
	u32 swinfo1;
	u32 swinfo2;
	u32 ps_data;
};

struct qm_cfg_regs {
	u32	revision;
	u32	__pad1;
	u32	divert;
	u32	link_ram_base0;
	u32	link_ram_size0;
	u32	link_ram_base1;
	u32	__pad2[2];
	u32	starvation[0];
};

struct qm_reg_region {
	u32	base;
	u32	start_index;
	u32	size_count;
	u32	__pad;
};

struct qm_reg_queue {
	u32		entry_count;
	u32		byte_count;
	u32		packet_size;
	u32		ptr_size_thresh;
};

struct reg_global {
	u32	revision;
	u32	perf_control;
	u32	emulation_control;
	u32	priority_control;
	u32	qm_base_address[4];
};

struct reg_chan {
	u32	control;
	u32	mode;
	u32	__rsvd[6];
};

struct reg_rx_flow {
	u32	control;
	u32	tags;
	u32	tag_sel;
	u32	fdq_sel[2];
	u32	thresh[3];
};

/* Descriptor type created by flows */
#define CPDMA_DESC_TYPE_HOST    1

/* CPPI Tx DMA channel control register A definitions */
#define CPDMA_REG_VAL_TCHAN_A_TX_ENABLE ((u32)1 << 31)
#define CPDMA_REG_VAL_TCHAN_A_TX_TDOWN  (1 << 30)

/* CPPI Tx DMA channel control register B definitions */
#define CPDMA_REG_VAL_TCHAN_B_TX_FILT_EINFO   (1 << 30)
#define CPDMA_REG_VAL_TCHAN_B_TX_FILT_PSWORDS (1 << 29)
#define CPDMA_REG_TCHAN_B_SET_DEFAULT_TDOWN_QMGR(x,v)  (x) = (BOOT_SET_BITFIELD((x), (v), 13, 12)
#define CPDMA_REG_TCHAN_B_SET_DEFAULT_TDOWN_QNUM(x,v)  (x) = (BOOT_SET_BITFIELD((x), (v), 11,  0)

/* CPPI Rx DMA channel control register A definitions */
#define CPDMA_REG_VAL_RCHAN_A_RX_ENABLE ((u32)1 << 31)
#define CPDMA_REG_VAL_RCHAN_A_RX_TDOWN  (1 << 30)

enum keystone_dma_tx_priority {
	DMA_PRIO_HIGH	= 0,
	DMA_PRIO_MED_H,
	DMA_PRIO_MED_L,
	DMA_PRIO_LOW,
};

#define DEVICE_RX_CDMA_TIMEOUT_COUNT    1000

struct packet_dma_rx_cfg {
	struct reg_chan  *rx_base; /* Base address of rx registers */
	u32  n_rx_chans;		/* The number of rx channels */
	struct reg_rx_flow *rx_flow; /* Add address of flow registers */
	u32  n_rx_flows;	/* Number of rx flows */
	u32  qm_num_free_buf;	/* Queue manager for descriptors/buffers for received packets */
	u32  queue_free_buf;	/* Queue that holds descriptors/buffers for received packets */
	u32  qm_num_rx;		/* Queue manager for received packets */
	u32  queue_rx;		/* Default Rx queue for received packets */
	u32  tdown_poll_count;	/* Number of loop iterations to wait for teardown */
};

struct packet_dma_tx_cfg {
	struct reg_global* gbl_ctl_base; /* Base address of global control registers */
	struct reg_chan  * tx_base;	/* Base address of the tx registers */
	u32 n_tx_chans;	/* The number of tx channels */
};

/* A very simply flow configuration is supported. No queue allocation by bins is supported */
#define CPDMA_REG_VAL_MAKE_RX_FLOW_A(einfo,psinfo,rxerr,desc,psloc,sopOff,qmgr,qnum)  \
        (   ((einfo & 1) << 30)       |   \
            ((psinfo & 1) << 29)      |   \
            ((rxerr & 1) << 28)       |   \
            ((desc & 3) << 26)        |   \
            ((psloc & 1) << 25)       |   \
            ((sopOff & 0x1ff) << 16)  |   \
            ((qmgr & 3) << 12)        |   \
            ((qnum & 0xfff) << 0)     )

#define CPDMA_REG_VAL_MAKE_RX_FLOW_D(fd0Qm, fd0Qnum, fd1Qm, fd1Qnum)   \
        (   ((fd0Qm & 3) << 28)         |   \
            ((fd0Qnum & 0xfff) << 16)   |   \
            ((fd1Qm & 3) << 12)         |   \
            ((fd1Qnum & 0xfff) <<  0)   )

#define CPDMA_REG_VAL_EMU_CTL_NO_LOOPBACK   0


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

struct	qm_host_desc *qm_pop(u32 qnum);
void	qm_push (struct qm_host_desc *hd, u32 qnum, u32 descrSize);
int	qm_setup (struct qm_config *cfg);
void	qm_teardown(void);
void	init_queues(void);
void	*targetGetQmConfig (void);
void	*targetGetCpdmaRxConfig (void);
void	*targetGetCpdmaTxConfig (void);
int	packet_dma_rx_configure(const struct packet_dma_rx_cfg *cfg);
int	packet_dma_tx_configure(const struct packet_dma_tx_cfg *cfg);
int	packet_dma_rx_disable (const struct packet_dma_rx_cfg *cfg);
int	packet_dma_tx_disable (const struct packet_dma_tx_cfg *cfg);

#endif  /* _CPPI_DMA_H_ */
