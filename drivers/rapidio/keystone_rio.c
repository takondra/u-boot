/*
 * Copyright (C) 2010, 2011, 2012 Texas Instruments Incorporated
 * Authors: Aurelien Jacquiot <a-jacquiot@ti.com>
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
#include <common.h>
#include <command.h>
#include <malloc.h>
#include <asm/dma-mapping.h>
#include <asm/io.h>
#include <asm/arch/psc_defs.h>
#include "keystone_rio.h"

unsigned int	rio_dbg = 0;
#define debug_rio(fmt, args...)	if (rio_dbg) printf(fmt, ##args)

#define DRIVER_VER	"v0.0"

/*
 * Main KeyStone RapidIO driver data
 */
struct keystone_rio_data {
	int			riohdid;
	u32			rio_pe_feat;

	u32			ports_registering;
	u32			port_chk_cnt;

	volatile u32		*jtagid_reg;
	volatile u32		*serdes_sts_reg;
	volatile struct keystone_srio_serdes_regs     *serdes_regs;
	volatile struct keystone_rio_regs	      *regs;

	volatile struct keystone_rio_car_csr_regs     *car_csr_regs;
	volatile struct keystone_rio_serial_port_regs *serial_port_regs;
	volatile struct keystone_rio_err_mgmt_regs    *err_mgmt_regs;
	volatile struct keystone_rio_phy_layer_regs   *phy_regs;
	volatile struct keystone_rio_transport_layer_regs *transport_regs;
	volatile struct keystone_rio_pkt_buf_regs     *pkt_buf_regs;
	volatile struct keystone_rio_evt_mgmt_regs    *evt_mgmt_regs;
	volatile struct keystone_rio_port_write_regs  *port_write_regs;
	volatile struct keystone_rio_link_layer_regs  *link_regs;
	volatile struct keystone_rio_fabric_regs      *fabric_regs;
	u32					      car_csr_regs_base;

	struct keystone_rio_board_controller_info     board_rio_cfg;
} __krio_priv;

static u32 __ffs(u32 mask)
{
	u32 i = 0;

	while (i < 32) {
		if (mask & (0x1 << i))
			return i;
		else
			++i;
	}
	return 32;
}

/*---------------------------- Direct I/O -------------------------------*/

static u32 keystone_rio_dio_get_lsu_cc(u32 lsu_id, u8 ltid, u8 *lcb,
				struct keystone_rio_data *krio_priv)
{
	u32 idx;
	u32 shift;
	u32 value;
	u32 cc;
	/* lSU shadow register status mapping */
	u32 lsu_index[8] = { 0, 9, 15, 20, 24, 33, 39, 44 };

	/* Compute LSU stat index from LSU id and LTID */
	idx   = (lsu_index[lsu_id] + ltid) >> 3;
	shift = ((lsu_index[lsu_id] + ltid) & 0x7) << 2;

	/* Get completion code and context */
	value  = readl(&(krio_priv->regs->lsu_stat_reg[idx]));
	cc     = (value >> (shift + 1)) & 0x7;
	*lcb   = (value >> shift) & 0x1;

	return cc;
}

/*--------------------- Maintenance Request Management  ---------------------*/

/**
 * maint_request - Perform a maintenance request
 * @index: ID of the RapidIO interface
 * @destid: destination ID of target device
 * @hopcount: hopcount for this request
 * @offset: offset in the RapidIO configuration space
 * @buff: dma address of the data on the host
 * @buff_len: length of the data
 * @size: 1 for 16bit, 0 for 8bit ID size
 * @type: packet type
 *
 * Returns %0 on success or %-EINVAL, %-EIO, %-EAGAIN or %-EBUSY on failure.
 */
static inline int maint_request(int index, u32 dest_id, u8 hopcount,
		u32 offset, dma_addr_t buff, int buff_len,
		u16 size, u16 type, struct keystone_rio_data *krio_priv)
{
	unsigned int count;
	unsigned int status = 0;
	unsigned int res    = 0;
	u8           context;
	u8           ltid;

	/* Check is there is space in the LSU shadow reg and that it is free */
	count = 0;
	while (1) {
		status = readl(&(krio_priv->regs->lsu_reg[0].busy_full));
		if (((status & KEYSTONE_RIO_LSU_FULL_MASK) == 0x0)
		    && ((status & KEYSTONE_RIO_LSU_BUSY_MASK) == 0x0))
			break;
		count++;

		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			debug_rio("no LSU available, status = 0x%x\n", status);
			res = -1;
			goto out;
		}
		udelay(1);
	}

	/* Get LCB and LTID, LSU reg 6 is already read */
	context = (status >> 4) & 0x1;
	ltid    = status & 0xf;

	/* LSU Reg 0 - MSB of RapidIO address */
	writel(0, &(krio_priv->regs->lsu_reg[0].addr_msb));

	/* LSU Reg 1 - LSB of destination */
	writel(offset, &(krio_priv->regs->lsu_reg[0].addr_lsb_cfg_ofs));

	/* LSU Reg 2 - source address */
	writel(buff, &(krio_priv->regs->lsu_reg[0].dsp_addr));

	/* LSU Reg 3 - byte count */
	writel(buff_len,
		&(krio_priv->regs->lsu_reg[0].dbell_val_byte_cnt));

	/* LSU Reg 4 - */
	writel(((index << 8)
			| (KEYSTONE_RIO_LSU_PRIO << 4)
			| (size ? (1 << 10) : 0)
			| ((u32) dest_id << 16)),
		&(krio_priv->regs->lsu_reg[0].destid));

	/* LSU Reg 5 */
	writel(((hopcount & 0xff) << 8) | (type & 0xff),
		&(krio_priv->regs->lsu_reg[0].dbell_info_fttype));

	/* Retrieve our completion code */
	count = 0;
	res   = 0;
	while (1) {
		u8 lcb;
		status = keystone_rio_dio_get_lsu_cc(0, ltid, &lcb, krio_priv);
		if (lcb == context)
			break;
		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			debug_rio("timeout %d, ltid = %d, context = %d, "
				"lcb = %d, cc = %d\n",
				count, ltid, context, lcb, status);
			res = -2;
			break;
		}
		udelay(1);
	}
out:
	if (res)
		return res;

	if (status)
		debug_rio("transfer error = 0x%x\n", status);

	switch (status) {
	case KEYSTONE_RIO_LSU_CC_TIMEOUT:
	case KEYSTONE_RIO_LSU_CC_XOFF:
	case KEYSTONE_RIO_LSU_CC_ERROR:
	case KEYSTONE_RIO_LSU_CC_INVALID:
	case KEYSTONE_RIO_LSU_CC_DMA:
		return -3;
		break;
	case KEYSTONE_RIO_LSU_CC_RETRY:
		return -4;
		break;
	case KEYSTONE_RIO_LSU_CC_CANCELED:
		return -5;
		break;
	default:
		break;
	}
	return 0;
}
/*------------------------- RapidIO hw controller setup ---------------------*/

/**
 * keystone_rio_hw_init - Configure a RapidIO controller
 * @mode: serdes configuration
 * @hostid: device id of the host
 */
static void keystone_rio_hw_init(u32 mode, struct keystone_rio_data *krio_priv)
{
	u32 val;
	u32 block;
	u32 port;
	struct keystone_serdes_config *serdes_config
		= &(krio_priv->board_rio_cfg.serdes_config[mode]);

	/* Set sRIO out of reset */
	writel(0x00000011, &krio_priv->regs->pcr);

	/* Clear BOOT_COMPLETE bit (allowing write) */
	writel(0x00000000, &krio_priv->regs->per_set_cntl);

	/* Enable blocks */
	writel(1, &krio_priv->regs->gbl_en);
	for (block = 0; block <= KEYSTONE_RIO_BLK_NUM; block++)
		writel(1, &(krio_priv->regs->blk[block].enable));

	/* Set control register 1 configuration */
	writel(0x00000000, &krio_priv->regs->per_set_cntl1);

	/* Set Control register */
	writel(serdes_config->cfg_cntl, &krio_priv->regs->per_set_cntl);

	/* Serdes main configuration */
	writel(serdes_config->serdes_cfg_pll,
		&krio_priv->serdes_regs->pll);

	/* Per-port SerDes configuration */
	for (port = 0; port < KEYSTONE_RIO_MAX_PORT; port++) {
		writel(serdes_config->rx_chan_config[port],
				&krio_priv->serdes_regs->channel[port].rx);
		writel(serdes_config->tx_chan_config[port],
				&krio_priv->serdes_regs->channel[port].tx);
	}

	/* Check for RIO SerDes PLL lock */
	do {
		val = readl(krio_priv->serdes_sts_reg);
	} while ((val & 0x1) != 0x1);

	/* Set prescalar for ip_clk */
	writel(serdes_config->prescalar_srv_clk,
			&krio_priv->link_regs->prescalar_srv_clk);

	/* Peripheral-specific configuration and capabilities */
	writel(KEYSTONE_RIO_DEV_ID_VAL,
		&krio_priv->car_csr_regs->dev_id);
	writel(KEYSTONE_RIO_DEV_INFO_VAL,
		&krio_priv->car_csr_regs->dev_info);
	writel(KEYSTONE_RIO_ID_TI,
		&krio_priv->car_csr_regs->assembly_id);
	writel(KEYSTONE_RIO_EXT_FEAT_PTR,
		&krio_priv->car_csr_regs->assembly_info);

	krio_priv->rio_pe_feat = RIO_PEF_PROCESSOR
		| RIO_PEF_CTLS
		| KEYSTONE_RIO_PEF_FLOW_CONTROL
		| RIO_PEF_EXT_FEATURES
		| RIO_PEF_ADDR_34
		| RIO_PEF_STD_RT
		| RIO_PEF_INB_DOORBELL
		| RIO_PEF_INB_MBOX;

	writel(krio_priv->rio_pe_feat,
		&krio_priv->car_csr_regs->pe_feature);

	writel(KEYSTONE_RIO_MAX_PORT << 8,
		&krio_priv->car_csr_regs->sw_port);

	writel((RIO_SRC_OPS_READ
		       | RIO_SRC_OPS_WRITE
		       | RIO_SRC_OPS_STREAM_WRITE
		       | RIO_SRC_OPS_WRITE_RESPONSE
		       | RIO_SRC_OPS_DATA_MSG
		       | RIO_SRC_OPS_DOORBELL
		       | RIO_SRC_OPS_ATOMIC_TST_SWP
		       | RIO_SRC_OPS_ATOMIC_INC
		       | RIO_SRC_OPS_ATOMIC_DEC
		       | RIO_SRC_OPS_ATOMIC_SET
		       | RIO_SRC_OPS_ATOMIC_CLR
		       | RIO_SRC_OPS_PORT_WRITE),
		&krio_priv->car_csr_regs->src_op);

	writel((RIO_DST_OPS_READ
		       | RIO_DST_OPS_WRITE
		       | RIO_DST_OPS_STREAM_WRITE
		       | RIO_DST_OPS_WRITE_RESPONSE
		       | RIO_DST_OPS_DATA_MSG
		       | RIO_DST_OPS_DOORBELL
		       | RIO_DST_OPS_PORT_WRITE),
		&krio_priv->car_csr_regs->dest_op);

	writel(RIO_PELL_ADDR_34,
		&krio_priv->car_csr_regs->pe_logical_ctl);

	val = (((KEYSTONE_RIO_SP_HDR_NEXT_BLK_PTR & 0xffff) << 16) |
		KEYSTONE_RIO_SP_HDR_EP_REC_ID);
	writel(val, &krio_priv->serial_port_regs->sp_maint_blk_hdr);

	/* clear high bits of local config space base addr */
	writel(0x00000000, &krio_priv->car_csr_regs->local_cfg_hbar);

	/* set local config space base addr */
	writel(0x00520000, &krio_priv->car_csr_regs->local_cfg_bar);

	/* Enable HOST & MASTER_ENABLE bits */
	writel(0xe0000000, &krio_priv->serial_port_regs->sp_gen_ctl);

	/* set link timeout value */
	writel(0x000FFF00,
		&krio_priv->serial_port_regs->sp_link_timeout_ctl);

	/* set response timeout value */
	writel(0x000FFF00,
		&krio_priv->serial_port_regs->sp_rsp_timeout_ctl);

	/* allows SELF_RESET and PWDN_PORT resets to clear stcky reg bits */
	writel(0x00000001, &krio_priv->link_regs->reg_rst_ctl);

	/* Set error detection mode */
	/* clear all errors */
	writel(0x00000000, &krio_priv->err_mgmt_regs->err_det);
	/* enable all error detection */
	writel(0x00000000, &krio_priv->err_mgmt_regs->err_en);

	/* set err det block header */
	val = (((KEYSTONE_RIO_ERR_HDR_NEXT_BLK_PTR & 0xffff) << 16) |
		KEYSTONE_RIO_ERR_EXT_FEAT_ID);
	writel(val, &krio_priv->err_mgmt_regs->err_report_blk_hdr);

	/* clear msb of err catptured addr reg */
	writel(0x00000000, &krio_priv->err_mgmt_regs->h_addr_capt);

	/* clear lsb of err catptured addr reg */
	writel(0x00000000, &krio_priv->err_mgmt_regs->addr_capt);

	/* clear err catptured source and dest devID reg */
	writel(0x00000000, &krio_priv->err_mgmt_regs->id_capt);

	/* clear err catptured packet info */
	writel(0x00000000, &krio_priv->err_mgmt_regs->ctrl_capt);

	/* Force all writes to finish */
	val = readl(&krio_priv->err_mgmt_regs->ctrl_capt);
}

/**
 * keystone_rio_start - Start RapidIO controller
 */
static void keystone_rio_start(struct keystone_rio_data *krio_priv)
{
	u32 val;

	/* Set PEREN bit to enable logical layer data flow */
	val = (KEYSTONE_RIO_PER_EN | KEYSTONE_RIO_PER_FREE);
	writel(val, &krio_priv->regs->pcr);

	/* Set BOOT_COMPLETE bit */
	val = readl(&krio_priv->regs->per_set_cntl);
	writel(val | KEYSTONE_RIO_BOOT_COMPLETE,
				&krio_priv->regs->per_set_cntl);
}

static int
keystone_rio_test_link(struct keystone_rio_data *krio_priv)
{
	u32 *tbuf;
	int res;
	dma_addr_t dma;
	size_t align_len = 64;
	u32 val;

	tbuf = malloc(align_len);
	if (!tbuf)
		return -1;

	memset(tbuf, 0, align_len);

	dma = dma_map_single(tbuf, 4, DMA_FROM_DEVICE);

	/* Send a maint req to test the link */
	res = maint_request(0, 0xff, 0, 0, dma, 4,
			krio_priv->board_rio_cfg.size,
			KEYSTONE_RIO_PACKET_TYPE_MAINT_R,
			krio_priv);

	dma_unmap_single((volatile void *)tbuf, 4, dma);

	if (res == 0) {
		/* Taking care of byteswap */
		val = ntohl(*((u32 *) tbuf));

		printf("SRIO link up: "
			"read remote reg offset 0 val 0x%08x\n", val);
	}

	return res;
}

/**
 * keystone_rio_port_status - Return if the port is OK or not
 * @port: index of the port
 *
 * Return %0 if the port is ready or %-EIO on failure.
 */
static int keystone_rio_port_status(int port,
		struct keystone_rio_data *krio_priv)
{
	unsigned int count, value;
	int res = 0;

	count  = 0;

	if (port >= KEYSTONE_RIO_MAX_PORT)
		return -1;

	/* Check port status */
	while (1) {
		value = readl(&(krio_priv->serial_port_regs->
				sp[port].err_stat));

		if ((value & RIO_PORT_N_ERR_STS_PORT_OK) != 0) {
			res = keystone_rio_test_link(krio_priv);
			if (0 == res)
				return 0; /* port must be solid OK */

		} else {
			count++;
			if (count >= 1000)
				return -2;
		}
		udelay(10000);
	}
	return 0;
}

/**
 * keystone_rio_port_init - Configure a RapidIO port
 * @port: index of the port to configure
 * @mode: serdes configuration
 */
static int keystone_rio_port_init(u32 port, u32 mode,
			struct keystone_rio_data *krio_priv)
{
	u32 path_mode =
		krio_priv->board_rio_cfg.serdes_config[mode].path_mode[port];

	if (port >= KEYSTONE_RIO_MAX_PORT)
		return -1;

	/* Send both link request and PNA control symbols
	   (this will clear error states) */
	writel(0x2003f044,
		&krio_priv->phy_regs->phy_sp[port].long_cs_tx1);

	/* Disable packet forwarding */
	writel(0xffffffff, &(krio_priv->regs->pkt_fwd_cntl[port].pf_16b));
	writel(0x0003ffff, &(krio_priv->regs->pkt_fwd_cntl[port].pf_8b));

	/* Silence and discovery timers */
	writel(0x20000000,
		&(krio_priv->phy_regs->phy_sp[port].silence_timer));
	writel(0x20000000,
		&(krio_priv->phy_regs->phy_sp[port].discovery_timer));

	/* Enable port in input and output */
	writel(0x600000, &(krio_priv->serial_port_regs->sp[port].ctl));

	/* Program channel allocation to ports (1x, 2x or 4x) */
	writel(path_mode, &(krio_priv->phy_regs->phy_sp[port].path_ctl));

	return 0;
}

/**
 * keystone_rio_port_activate - Start using a RapidIO port
 * @port: index of the port to configure
 */
static int keystone_rio_port_activate(u32 port,
		struct keystone_rio_data *krio_priv)
{
	u32 val;

	/* Enable interrupt for reset request */
	val = readl(&(krio_priv->evt_mgmt_regs->evt_mgmt_rst_int_en));
	writel(val | (1 << port),
		&(krio_priv->evt_mgmt_regs->evt_mgmt_rst_int_en));

	/* Enable all PLM interrupts */
	writel(0xffffffff,
		&(krio_priv->phy_regs->phy_sp[port].int_enable));
	writel(1, &(krio_priv->phy_regs->phy_sp[port].all_int_en));

	/* Enable all errors */
	writel(0xffffffff,
		&(krio_priv->err_mgmt_regs->sp_err[port].rate_en));

	/* Cleanup port error status */
	writel(KEYSTONE_RIO_PORT_ERROR_MASK,
		&(krio_priv->serial_port_regs->sp[port].err_stat));
	writel(0, &(krio_priv->err_mgmt_regs->sp_err[port].det));

	/* Enable promiscuous */
	writel(0x00309000,
		&(krio_priv->transport_regs->transport_sp[port].control));

	/* Enable Port-write reception capture */
	writel(0, &(krio_priv->port_write_regs->port_wr_rx_capt[port]));

	return 0;
}

/*------------------------ Main Linux driver functions -----------------------*/

static void keystone_rio_get_controller_defaults(
		struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_board_controller_info *c =
				&krio_priv->board_rio_cfg;
	int i;

	c->rio_regs_base = 0x2900000;
	c->rio_regs_size = 0x21000;
	c->boot_cfg_regs_base = 0x2620000;
	c->boot_cfg_regs_size = 0x3b0;

	/* dev-id-size */
	c->size = 0;

	c->ports = 0x1;

	/* Serdes config */
	c->serdes_config_num = 1;  /* total number of serdes_config[] entries */
	c->mode = 0;  /* default serdes_config[] entry to be used */

	/* Mode 0: sRIO config 0: MPY = 5x, div rate = half,
	   link rate = 3.125 Gbps, mode 1x */

	/* setting control register config */
	c->serdes_config[0].cfg_cntl		= 0x0c053860;
	/* SerDes PLL configuration */
	c->serdes_config[0].serdes_cfg_pll	= 0x0229;
	/* prescalar_srv_clk */
	c->serdes_config[0].prescalar_srv_clk	= 0x001e;

	/* serdes rx_chan_config */
	for (i = 0; i < KEYSTONE_RIO_MAX_PORT; i++)
		c->serdes_config[0].rx_chan_config[i] = 0x00440495;

	/* serdes tx_chan_config */
	for (i = 0; i < KEYSTONE_RIO_MAX_PORT; i++)
		c->serdes_config[0].tx_chan_config[i] = 0x00180795;

	/* path_mode */
	for (i = 0; i < KEYSTONE_RIO_MAX_PORT; i++)
		c->serdes_config[0].path_mode[i] = 0x00000000;
}

/*
 * Platform configuration setup
 */
static int  keystone_rio_setup_controller(struct keystone_rio_data *krio_priv)
{
	u32 ports;
	u32 p;
	u32 mode;
	u32 size = 0;
	int res = 0;

	size   = krio_priv->board_rio_cfg.size;
	ports  = krio_priv->board_rio_cfg.ports;
	mode   = krio_priv->board_rio_cfg.mode;

	debug_rio("size = %d, ports = 0x%x, mode = %d\n",
		size, ports, mode);

	if (mode >= krio_priv->board_rio_cfg.serdes_config_num) {
		mode = 0;
		printf("RIO: invalid port mode, forcing it to %d\n", mode);
	}

	/* Hardware set up of the controller */
	keystone_rio_hw_init(mode, krio_priv);

	/*
	 * Configure all ports even if we do not use all of them.
	 * This is needed for 2x and 4x modes.
	 */
	for (p = 0; p < KEYSTONE_RIO_MAX_PORT; p++) {
		res = keystone_rio_port_init(p, mode, krio_priv);
		if (res < 0) {
			printf("RIO: initialization of port %d failed\n", p);
			return res;
		}
	}

	/* Start the controller */
	keystone_rio_start(krio_priv);

	/* Use and check ports status (but only the requested ones) */
	krio_priv->ports_registering = 0;
	while (ports) {
		int status;
		u32 port = __ffs(ports);
		if (port > 32)
			return 0;
		ports &= ~(1 << port);

		/* Start the port */
		keystone_rio_port_activate(port, krio_priv);

		/*
		 * Check the port status here before calling the generic RapidIO
		 * layer. Port status check is done in rio_mport_is_active() as
		 * well but we need to do it our way first due to some delays in
		 * hw initialization.
		 */
		status = keystone_rio_port_status(port, krio_priv);
		if (status == 0)
			printf("RIO: port RIO%d ready\n", port);
		else
			printf("RIO: port %d not ready\n", port);
	}

	return res;
}


void *keystone_rio_init(int riohdid)
{
	struct keystone_rio_data *krio_priv = &__krio_priv;
	int res = 0;
	volatile void *regs;

	keystone_rio_get_controller_defaults(krio_priv);

	regs = (volatile void *)krio_priv->board_rio_cfg.boot_cfg_regs_base;
	krio_priv->jtagid_reg     = regs + 0x0018;
	krio_priv->serdes_sts_reg = regs + 0x154;
	krio_priv->serdes_regs    = regs + 0x360;

	regs = (volatile void *)krio_priv->board_rio_cfg.rio_regs_base;
	krio_priv->regs			= regs;
	krio_priv->car_csr_regs		= regs + 0xb000;
	krio_priv->serial_port_regs	= regs + 0xb100;
	krio_priv->err_mgmt_regs	= regs + 0xc000;
	krio_priv->phy_regs		= regs + 0x1b000;
	krio_priv->transport_regs	= regs + 0x1b300;
	krio_priv->pkt_buf_regs		= regs + 0x1b600;
	krio_priv->evt_mgmt_regs	= regs + 0x1b900;
	krio_priv->port_write_regs	= regs + 0x1ba00;
	krio_priv->link_regs		= regs + 0x1bd00;
	krio_priv->fabric_regs		= regs + 0x1be00;
	krio_priv->car_csr_regs_base = (u32)regs + 0xb000;

	krio_priv->riohdid = riohdid;

	/* Enable srio clock */
	psc_enable_module(TCI6614_LPSC_SRIO);

	printf("KeyStone RapidIO driver %s, hdid=%d\n", DRIVER_VER, riohdid);

	/* Setup the sRIO controller */
	res = keystone_rio_setup_controller(krio_priv);
	if (res < 0)
		return NULL;

	return (void *)krio_priv;
}

int keystone_rio_shutdown(void *handle)
{
	if (handle != &__krio_priv)
		return -1;

	/* power off */
	psc_disable_module(TCI6614_LPSC_SRIO);
	return 0;
}

