/*
 * Ethernet driver for TI Keystone 2 devices.
 *
 * Copyright (C) 2012,2013 Texas Instruments Incorporated
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
#include <asm/io.h>
#include <asm/arch/cppi_dma.h>

/* START ************************* moved from h *******************************/
/* Memory alignment requirements (bytes) */
#define QM_LINKRAM_ALIGN        4
#define QM_MEMR_ALIGN           16      /* Not specified in the doc */


/* Description region setup */
#define QM_REG_VAL_DESC_SETUP_SET_DESC_SIZE(x,v)  (x) = BOOT_SET_BITFIELD((x),((v) >> 4)-1, 28, 16)

/* Maximum linking RAM size mask */
#define QM_REG_LINKRAM_SIZE_MAX_MASK  0x7ffff

/* END *************************** moved from h *******************************/

extern int cpu_to_bus(u32 *ptr, u32 length);

#define DEVICE_NUM_RX_CPPIS     16
#define DEVICE_NUM_TX_CPPIS     1
#define DEVICE_NUM_CPPIS        (DEVICE_NUM_RX_CPPIS + DEVICE_NUM_TX_CPPIS)

/* The linking RAM */
u8 qm_linkram_buf[DEVICE_NUM_CPPIS * 2 * (sizeof(u32)/sizeof(u8))] __attribute__((aligned(16)));

/* The CPPI RAM */
u8 qm_cppi_buf[QM_DESC_SIZE_BYTES * DEVICE_NUM_CPPIS] __attribute__((aligned(16)));


/* The rx data buffers */
u8 qm_buffer[MAX_SIZE_STREAM_BUFFER * DEVICE_NUM_RX_CPPIS] __attribute__((aligned(16)));

const struct qm_config qm_cfg = {
	(u32) qm_linkram_buf,
	sizeof  (qm_cppi_buf),
	(u32) qm_cppi_buf,
	DEVICE_NUM_CPPIS,
	DEVICE_QM_FREE_Q
};

const struct packet_dma_rx_cfg cpdmaEthRxCfg =  {
	(struct reg_chan *) DEVICE_PA_CDMA_RX_CHAN_CFG_BASE,
	DEVICE_PA_CDMA_RX_NUM_CHANNELS,
	(struct reg_rx_flow *) DEVICE_PA_CDMA_RX_FLOW_CFG_BASE,
	DEVICE_PA_CDMA_RX_NUM_FLOWS,
	0,
	DEVICE_QM_LNK_BUF_Q,
	0,
	DEVICE_QM_RCV_Q,
	DEVICE_RX_CDMA_TIMEOUT_COUNT
};

const struct packet_dma_tx_cfg cpdmaEthTxCfg = {
	(struct reg_global*) DEVICE_PA_CDMA_GLOBAL_CFG_BASE,
	(struct reg_chan  *) DEVICE_PA_CDMA_TX_CHAN_CFG_BASE,
	DEVICE_PA_CDMA_TX_NUM_CHANNELS      /* Number of tx channels */
};

static void chip_delay(u32 del)
{
	volatile unsigned int i;

	for (i = 0; i < (del / 8); i++);
}

struct qm_host_desc *qm_pop(u32 qnum)
{
	struct qm_host_desc *hd;
	struct qm_reg_queue *queue = 
		(struct qm_reg_queue *)DEVICE_QM_MANAGER_QUEUES_BASE + qnum;
	u32 uhd;

	uhd = readl(&queue->ptr_size_thresh) & ~0xf;
	if (uhd)
		cpu_to_bus((u32 *)uhd, sizeof(struct qm_host_desc) / sizeof(u32));

	hd  = (struct qm_host_desc *)uhd;

	return (hd);
}

void qm_push (struct qm_host_desc *hd, u32 qnum, u32 descrSize)
{
	u32 regd;
	struct qm_reg_queue *queue = 
		(struct qm_reg_queue *)DEVICE_QM_MANAGER_QUEUES_BASE + qnum;

	cpu_to_bus((u32 *)hd, sizeof(struct qm_host_desc) / sizeof(u32));

	regd = ((u32) hd | ((descrSize >> 4) - 1));

	writel(regd, &queue->ptr_size_thresh);
}

u32 armLmbd (u32 val, u32 target)
{
    u32 i;

    val = val & 1;

    for (i = 0; i < 32; i++)
        if ( ((target >> (31-i)) & 1) == val)
            return (i);

    return (32);
}

int qm_setup (struct qm_config *cfg)
{
	u32 v, w, i;
	struct qm_cfg_regs *qm_base = (struct qm_cfg_regs *)DEVICE_QM_MANAGER_BASE;
	struct qm_reg_region *qm_region = (struct qm_reg_region *)DEVICE_QM_DESC_SETUP_BASE;
	struct qm_reg_queue *queue = (struct qm_reg_queue *)DEVICE_QM_MANAGER_QUEUES_BASE;
	struct qm_host_desc *hd;

	/* Verify that alignment requirements */
	if (
	    ((cfg->link_ram_base & (QM_LINKRAM_ALIGN - 1)) !=  0) ||
	    ((cfg->mem_region_base & (QM_MEMR_ALIGN - 1)) !=  0)  ||
	    ((cfg->link_ram_size & ~QM_REG_LINKRAM_SIZE_MAX_MASK) != 0) ||
	    (cfg->link_ram_size < cfg->mem_reg_num_descs)
	   ) 
		return (QM_ERR);

	writel(cfg->link_ram_base,	&qm_base->link_ram_base0);
	writel(cfg->link_ram_size,	&qm_base->link_ram_size0);
	writel(0, 			&qm_base->link_ram_base1);

	writel(cfg->mem_region_base,	&qm_region[0].base);
	writel(0, 			&qm_region[0].start_index);

	v = (31 - armLmbd (1, cfg->mem_reg_num_descs));
	v = (v >= 4) ? v - 4 : 0;

	/* Add the descriptor size field */
	QM_REG_VAL_DESC_SETUP_SET_DESC_SIZE(v, QM_DESC_SIZE_BYTES);
	writel(v, 			&qm_region[0].size_count);

	/* Now format the descriptors and put them in a queue */
	for (i = 0, v = cfg->mem_region_base; i < cfg->mem_reg_num_descs;
	     i++, v += QM_DESC_SIZE_BYTES) {
		hd = (struct qm_host_desc *)v;
		memset (hd, 0, sizeof(struct qm_host_desc));

		hd->desc_info   = QM_DESC_DEFAULT_DESCINFO;
		hd->packet_info = QM_DESC_DEFAULT_PINFO;

		if (QM_DESC_INFO_GET_PSINFO_LOC(hd->desc_info) ==
		    QM_DESC_PSINFO_IN_DESCR) {
			if (QM_PKT_INFO_GET_EPIB(hd->packet_info) ==
			    QM_DESC_PINFO_EPIB)
				/*
				 * 32 bytes min descriptor size,
				 * 16 bytes extended info
				 */
				w = QM_DESC_SIZE_BYTES - 32 - 16;
			else
				w = QM_DESC_SIZE_BYTES - 32;
		} else
			w = 0;

		QM_PKT_INFO_SET_PSINFO_SIZE(hd->packet_info, (w >> 2));

		/* Push the descriptor onto the queue */
		cpu_to_bus((u32 *)hd, sizeof(struct qm_host_desc) / sizeof(u32));
		writel(v, &queue[cfg->dest_q].ptr_size_thresh);
	}

	return (QM_OK);
}

void qm_teardown(void)
{
	u32 i;
	struct qm_cfg_regs	*qm_base = (struct qm_cfg_regs *)DEVICE_QM_MANAGER_BASE;
	struct qm_reg_region	*qm_region = (struct qm_reg_region *)DEVICE_QM_DESC_SETUP_BASE;

	writel(0,	&qm_base->link_ram_base0);
	writel(0,	&qm_base->link_ram_size0);
	writel(0,	&qm_base->link_ram_base1);

	/* Memory region info */
	for (i = 0; i < DEVICE_QM_NUM_MEMREGIONS; i++) {
		writel(0, &qm_region[i].base);
		writel(0, &qm_region[i].start_index);
		writel(0, &qm_region[i].size_count);
	}
}

/**
 *  @brief
 *      Attach a packet buffer to each descriptor and push onto the linked buffer queue
 */
void init_queues(void)
{
	int32_t i;
	struct qm_host_desc *hd;

	for (i = 0; i < DEVICE_NUM_RX_CPPIS; i++)  {
		hd			= qm_pop (DEVICE_QM_FREE_Q);
		hd->buff_len		= sizeof (qm_buffer) / DEVICE_NUM_CPPIS;
		hd->buff_ptr		= (u32) &(qm_buffer[MAX_SIZE_STREAM_BUFFER * i]);
		hd->next_bdptr		= 0;
		hd->orig_buff_len	= MAX_SIZE_STREAM_BUFFER;
		hd->orig_buff_ptr	= hd->buff_ptr;

		qm_push (hd, DEVICE_QM_LNK_BUF_Q, QM_DESC_SIZE_BYTES);
	}

	for (i = 0; i < DEVICE_NUM_TX_CPPIS; i++)  {
		hd			= qm_pop (DEVICE_QM_FREE_Q);
		hd->buff_len		= 0;
		hd->buff_ptr		= 0;
		hd->next_bdptr		= 0;
		hd->orig_buff_len	= 0;
		hd->orig_buff_ptr	= 0;

		qm_push (hd, DEVICE_QM_TX_Q, QM_DESC_SIZE_BYTES);
	}
}

/**
 *  @brief
 *      Return the queue manager memory configuration information
 */
void *targetGetQmConfig (void)
{
    return ((void *)&qm_cfg);
}

void *targetGetCpdmaRxConfig (void)
{
    return ((void *)&cpdmaEthRxCfg);
}

void *targetGetCpdmaTxConfig (void)
{
    return ((void *)&cpdmaEthTxCfg);
}

int packet_dma_rx_disable (const struct packet_dma_rx_cfg *cfg)
{
	u32 i, v;
	int done;

	for (i = 0; i < cfg->n_rx_chans; i++) {
		/* If enabled, set the teardown bit */
		v = readl(&cfg->rx_base[i].control);
		if ((v & CPDMA_REG_VAL_RCHAN_A_RX_ENABLE) ==
			CPDMA_REG_VAL_RCHAN_A_RX_ENABLE) {
			v = v | CPDMA_REG_VAL_RCHAN_A_RX_TDOWN;
			writel(v, &cfg->rx_base[i].control);		
		}
	}

	/* Poll for completion */
	for (i = 0, done = 0; ((i < cfg->tdown_poll_count) && (done == 0) ); i++) {
		chip_delay (100);
		done = 1;
		v = readl(&cfg->rx_base[i].control);
		if ((v & CPDMA_REG_VAL_RCHAN_A_RX_ENABLE) ==
		    CPDMA_REG_VAL_RCHAN_A_RX_ENABLE )
			done = 0;
	}

	if (done == 0)
		return (-1);

	/* Clear all of the flow registers */
	for (i = 0; i < cfg->n_rx_flows; i++)  {
		writel(0, &cfg->rx_flow[i].control	);
		writel(0, &cfg->rx_flow[i].tags	);
		writel(0, &cfg->rx_flow[i].tag_sel	);
		writel(0, &cfg->rx_flow[i].fdq_sel[0]	);
		writel(0, &cfg->rx_flow[i].fdq_sel[1]	);
		writel(0, &cfg->rx_flow[i].thresh[0]	);
		writel(0, &cfg->rx_flow[i].thresh[1]	);
		writel(0, &cfg->rx_flow[i].thresh[2]	);
	}

	return (0);
}

int packet_dma_rx_configure(const struct packet_dma_rx_cfg *cfg)
{
	u32 v, i;
	int ret = 0;

	if (packet_dma_rx_disable(cfg) != 0)
		ret = -1;

	v = CPDMA_REG_VAL_MAKE_RX_FLOW_A(1, 1, 0, CPDMA_DESC_TYPE_HOST, 0, 0,
						cfg->qm_num_rx, cfg->queue_rx);

	writel(v, &cfg->rx_flow[RX_FLOW_NUM].control	);
	writel(0, &cfg->rx_flow[RX_FLOW_NUM].tags	);
	writel(0, &cfg->rx_flow[RX_FLOW_NUM].tag_sel	);

	v = CPDMA_REG_VAL_MAKE_RX_FLOW_D(cfg->qm_num_free_buf,
					 cfg->queue_free_buf,
					 cfg->qm_num_free_buf,
					 cfg->queue_free_buf);

	writel(v, &cfg->rx_flow[RX_FLOW_NUM].fdq_sel[0]	);
	writel(v, &cfg->rx_flow[RX_FLOW_NUM].fdq_sel[1]	);
	writel(0, &cfg->rx_flow[RX_FLOW_NUM].thresh[0]	);
	writel(0, &cfg->rx_flow[RX_FLOW_NUM].thresh[1]	);
	writel(0, &cfg->rx_flow[RX_FLOW_NUM].thresh[2]	);

	for (i = 0; i < cfg->n_rx_chans; i++)
		writel(CPDMA_REG_VAL_RCHAN_A_RX_ENABLE, &cfg->rx_base[i].control); 

	return (ret);
}

int packet_dma_tx_configure(const struct packet_dma_tx_cfg *cfg)
{
	u32 i;

	/* Disable loopback in the tx direction */
	writel(CPDMA_REG_VAL_EMU_CTL_NO_LOOPBACK, &cfg->gbl_ctl_base->emulation_control); 

#ifdef CONFIG_SOC_TCI6638
	/* Set QM base address, only for K2x devices */
	writel(0x23a80000, &cfg->gbl_ctl_base->qm_base_address[0]); 
#endif

	/* Enable all channels. The current state isn't important */
	for (i = 0; i < cfg->n_tx_chans; i++)  {
		writel(0, &cfg->tx_base[i].mode); 
		writel(CPDMA_REG_VAL_TCHAN_A_TX_ENABLE, &cfg->tx_base[i].control); 
	}

	return (0);
}

int packet_dma_tx_disable (const struct packet_dma_tx_cfg *cfg)
{
	u32 i, v;

	for (i = 0; i < cfg->n_tx_chans; i++) {
		v = readl(&cfg->tx_base[i].control);
		if ((v & CPDMA_REG_VAL_TCHAN_A_TX_ENABLE) ==
		    CPDMA_REG_VAL_TCHAN_A_TX_ENABLE) {
			v = v | CPDMA_REG_VAL_TCHAN_A_TX_TDOWN;
			writel(v, &cfg->tx_base[i].control);	
		}
	}

	return (0);
}

