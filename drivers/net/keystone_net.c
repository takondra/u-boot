/*
 * Ethernet driver for TI TMS320TCI6614 EVM.
 *
 * Copyright (C) 2012 Texas Instruments Incorporated
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
#include <command.h>

#include <net.h>
#include <miiphy.h>
#include <malloc.h>
#include <asm/arch/emac_defs.h>
#include <asm/arch/psc_defs.h>
#include <asm/arch/cppi_dma.h>

unsigned int emac_dbg = 0;
#define debug_emac(fmt, args...) if (emac_dbg) printf(fmt, ##args)

unsigned int emac_open = 0;

#ifdef TCI6614_EMAC_GIG_ENABLE
#define emac_gigabit_enable()	tci6614_eth_gigabit_enable()
#else
#define emac_gigabit_enable()	/* no gigabit to enable */
#endif

extern int cpu_to_bus(u32 *ptr, u32 length);

static void tci6614_eth_mdio_enable(void);

static int gen_init_phy(int phy_addr);
static int gen_is_phy_connected(int phy_addr);
static int gen_get_link_speed(int phy_addr);
static int gen_auto_negotiate(int phy_addr);

static int marvell_88e1111_init_phy(int phy_addr);
static int marvell_88e1111_is_phy_connected(int phy_addr);
static int marvell_88e1111_get_link_speed(int phy_addr);
static int marvell_88e1111_auto_negotiate(int phy_addr);

#ifdef CONFIG_SOC_TCI6638
void SgmiiDefSerdesSetup156p25Mhz();
#endif

/* EMAC Addresses */
static volatile emac_regs	*adap_emac = (emac_regs *)EMAC_EMACSL_BASE_ADDR;
static volatile mdio_regs	*adap_mdio = (mdio_regs *)EMAC_MDIO_BASE_ADDR;

/* PHY address for a discovered PHY (0xff - not found) */
volatile u_int8_t active_phy_addr = 0xff;

phy_t phy;

static void chip_delay(u32 del)
{
	volatile unsigned int i;

	for (i = 0; i < (del / 8); i++);
}

int tci6614_eth_set_mac_addr(struct eth_device *dev)
{
	u32 maca, macb;

	/* Read the e-fuse mac address */
	maca = __raw_readl(MAC_ID_BASE_ADDR);
	macb = __raw_readl(MAC_ID_BASE_ADDR + 4);

	dev->enetaddr[0] = (macb >>  8) & 0xff;
	dev->enetaddr[1] = (macb >>  0) & 0xff;
	dev->enetaddr[2] = (maca >> 24) & 0xff;
	dev->enetaddr[3] = (maca >> 16) & 0xff;
	dev->enetaddr[4] = (maca >>  8) & 0xff;
	dev->enetaddr[5] = (maca >>  0) & 0xff;

	return 0;
}

static void tci6614_eth_mdio_enable(void)
{
	u_int32_t	clkdiv;

	clkdiv = (EMAC_MDIO_BUS_FREQ / EMAC_MDIO_CLOCK_FREQ) - 1;

	writel((clkdiv & 0xffff) |
	       MDIO_CONTROL_ENABLE |
	       MDIO_CONTROL_FAULT |
	       MDIO_CONTROL_FAULT_ENABLE,
	       &adap_mdio->CONTROL);

	while (readl(&adap_mdio->CONTROL) & MDIO_CONTROL_IDLE)
		;
}

/*
 * Tries to find an active connected PHY. Returns 1 if address if found.
 * If no active PHY (or more than one PHY) found returns 0.
 * Sets active_phy_addr variable.
 */
static int tci6614_eth_phy_detect(void)
{
	u_int32_t	phy_act_state;
	int		i;
	int		ret = -1;

	active_phy_addr = 0xff;

#ifdef CONFIG_SOC_TCI6638
	udelay(10000);
#endif

	phy_act_state = readl(&adap_mdio->ALIVE) & EMAC_MDIO_PHY_MASK;
	if (phy_act_state == 0)
		return(ret);				/* No active PHYs */

	debug_emac("tci6614_eth_phy_detect(), ALIVE = 0x%08x\n", phy_act_state);

	for (i = 0; i < 32; i++) {
		if (phy_act_state & (1 << i)) {
			if (phy_act_state & ~(1 << i))
				return(ret);		/* More than one PHY */
			else {
				active_phy_addr = i;
				return(0);
			}
		}
	}

	return(ret);	/* Just to make GCC happy */
}


/* Read a PHY register via MDIO inteface. Returns 1 on success, 0 otherwise */
int tci6614_eth_phy_read(u_int8_t phy_addr, u_int8_t reg_num, u_int16_t *data)
{
	int	tmp;

	while (readl(&adap_mdio->USERACCESS0) & MDIO_USERACCESS0_GO)
		;

	writel(MDIO_USERACCESS0_GO |
	       MDIO_USERACCESS0_WRITE_READ |
	       ((reg_num & 0x1f) << 21) |
	       ((phy_addr & 0x1f) << 16),
	       &adap_mdio->USERACCESS0);

	/* Wait for command to complete */
	while ((tmp = readl(&adap_mdio->USERACCESS0)) & MDIO_USERACCESS0_GO)
		;

	if (tmp & MDIO_USERACCESS0_ACK) {
		*data = tmp & 0xffff;
		return(0);
	}

	*data = -1;
	return(-1);
}

/* Write to a PHY register via MDIO inteface. Blocks until operation is complete. */
int tci6614_eth_phy_write(u_int8_t phy_addr, u_int8_t reg_num, u_int16_t data)
{

	while (readl(&adap_mdio->USERACCESS0) & MDIO_USERACCESS0_GO)
		;

	writel(MDIO_USERACCESS0_GO |
	       MDIO_USERACCESS0_WRITE_WRITE |
	       ((reg_num & 0x1f) << 21) |
	       ((phy_addr & 0x1f) << 16) |
	       (data & 0xffff),
	       &adap_mdio->USERACCESS0);

	/* Wait for command to complete */
	while (readl(&adap_mdio->USERACCESS0) & MDIO_USERACCESS0_GO)
		;

	return(0);
}

/* PHY functions for a generic PHY */
static int gen_init_phy(int phy_addr)
{
	int	ret = -1;

	if (!gen_get_link_speed(phy_addr)) {
		/* Try another time */
		ret = gen_get_link_speed(phy_addr);
	}

	return(ret);
}

static int gen_is_phy_connected(int phy_addr)
{
	u_int16_t	dummy;

	return(tci6614_eth_phy_read(phy_addr, MII_PHYSID1, &dummy));
}

static int gen_get_link_speed(int phy_addr)
{
	u_int16_t	tmp;

	if ((!tci6614_eth_phy_read(phy_addr, MII_STATUS_REG, &tmp)) &&
			(tmp & 0x04)) {
		return(0);
	}

	return(-1);
}

static int gen_auto_negotiate(int phy_addr)
{
	u_int16_t	tmp;

	if (tci6614_eth_phy_read(phy_addr, MII_BMCR, &tmp))
		return(-1);

	/* Restart Auto_negotiation  */
	tmp |= BMCR_ANENABLE;
	tci6614_eth_phy_write(phy_addr, MII_BMCR, tmp);

	/*check AutoNegotiate complete */
	udelay (10000);
	if (tci6614_eth_phy_read(phy_addr, MII_BMSR, &tmp))
		return(-1);

	if (!(tmp & BMSR_ANEGCOMPLETE))
		return(0);

	return(gen_get_link_speed(phy_addr));
}

/* PHY functions for Marvell 88E1111 PHY */
static int marvell_88e1111_init_phy(int phy_addr)
{
	int	ret = -1;

	if (!marvell_88e1111_get_link_speed(phy_addr)) {
		/* Try another time */
		ret = marvell_88e1111_get_link_speed(phy_addr);
	}

	return(ret);
}

static int marvell_88e1111_is_phy_connected(int phy_addr)
{
	u_int16_t	dummy;

	return(tci6614_eth_phy_read(phy_addr, MII_PHYSID1, &dummy));
}

static int marvell_88e1111_get_link_speed(int phy_addr)
{
	u_int16_t	tmp;

	if ((!tci6614_eth_phy_read(phy_addr, MII_STATUS_REG, &tmp)) &&
			(tmp & MII_STATUS_LINK_MASK)) {

		return(0);
	}

	return(-1);
}

static int marvell_88e1111_auto_negotiate(int phy_addr)
{
	u_int16_t	tmp;

	if (tci6614_eth_phy_read(phy_addr, MII_BMCR, &tmp))
		return(-1);

	/* Restart Auto_negotiation  */
	tmp |= BMCR_ANENABLE;
	tci6614_eth_phy_write(phy_addr, MII_BMCR, tmp);

	/*check AutoNegotiate complete */
	udelay (10000);
	if (tci6614_eth_phy_read(phy_addr, MII_BMSR, &tmp))
		return(-1);

	if (!(tmp & BMSR_ANEGCOMPLETE))
		return(0);

	return(marvell_88e1111_get_link_speed(phy_addr));
}

#if defined(CONFIG_MII) || defined(CONFIG_CMD_MII)
static int tci6614_mii_phy_read(const char *devname, unsigned char addr,
				unsigned char reg, unsigned short *value)
{
	return(tci6614_eth_phy_read(addr, reg, value) ? -1 : 0);
}

static int tci6614_mii_phy_write(const char *devname, unsigned char addr,
				 unsigned char reg, unsigned short value)
{
	return(tci6614_eth_phy_write(addr, reg, value) ? -1 : 0);
}
#endif

static void  __attribute__((unused)) tci6614_eth_gigabit_enable(void)
{
	u_int16_t data;

#ifndef CONFIG_SYS_NO_MDIO
	if (tci6614_eth_phy_read(active_phy_addr, 0, &data) ||
		!(data & (1 << 6))) /* speed selection MSB */
		return;
#endif

	/*
	 * Check if link detected is giga-bit
	 * If Gigabit mode detected, enable gigbit in MAC
	 */
	writel(readl(&adap_emac->MACCONTROL) | EMAC_MACCONTROL_GIGFORCE |
		EMAC_MACCONTROL_GIGABIT_ENABLE, &adap_emac->MACCONTROL);
}

#ifndef CONFIG_SOC_TCI6638
void serdes_configure(u32 base, struct serdes_config *scfg)
{
	u32 reg, regb;
	int i;

	/*
	 * If the serdes is already enabled and the new value does not match
	 * the current value, the serdes is first disabled. Dont compare
	 * the sleep value in the register, which can be toggled dynamically
	 */
	reg  = DEVICE_REG32_R(base + SERDES_REG_CFG);
	reg  = SERDES_SET_CFG_SLEEP(reg, 0);
	regb = SERDES_SET_CFG_SLEEP(scfg->cfg, 0);

	if ((SERDES_GET_ENABLE(reg) == 1) &&
		(SERDES_GET_ENABLE(scfg->cfg) == 1) &&
		(reg != regb)) {

		reg = SERDES_SET_ENABLE(reg, 0);
		DEVICE_REG32_W(base + SERDES_REG_CFG, reg);
		chip_delay (100);
	}

	/*
	 * Config register. After enable it takes upt to 350ns, or 200 cycles to
	 * stabalize. Although these are serdes clock cycles the delay here is
	 * in cpu cycles. The PLL status will be checked by the peripheral
	 * using the PLL
	 */

	DEVICE_REG32_W(base + SERDES_REG_CFG, regb);
	chip_delay (200);

	/*
	 * Some devices have unreliable lock status bits. Add an extra delay
	 * to allow the serdes to lock
	 */
	chip_delay (TARGET_SERDES_LOCK_DELAY);

	/* rx and tx config registers */
	for (i = 0; i < scfg->n_lanes; i++) {
		DEVICE_REG32_W(base + SERDES_REG_RX(i), scfg->rx_cfg[i]);
		DEVICE_REG32_W(base + SERDES_REG_TX(i), scfg->tx_cfg[i]);
	}

	chipKickClosedSerdes(base);
}
 
int serdes_wait_lock(u32 status_base)
{
	u32 reg, i;

	for (i = 0; i < 100; i++)  {
		reg = DEVICE_REG32_R(status_base);
		if (reg & 1)
			return (0);

		chip_delay (1000);
	}

	return (-1);
}
#endif

int keystone_sgmii_link_status(int port)
{
	u32 status = 0;

	status = __raw_readl(SGMII_STATUS_REG(port));

	if ((status & SGMII_REG_STATUS_LINK) != 0)
		return 1;

	return 0;
}

int keystone_get_link_status()
{
	int sgmii_interface[DEVICE_N_GMACSL_PORTS] = CONFIG_SYS_SGMII_INTERFACE;
	int sgmii_link, n;
	int link_state = 0;

	for (n = 0; n < DEVICE_N_GMACSL_PORTS; n++) {
		sgmii_link = keystone_sgmii_link_status(n);

		if (sgmii_link) {
			link_state |= BIT(n);

			if (sgmii_interface[n] == SGMII_LINK_MAC_PHY)
				if (phy.get_link_speed (active_phy_addr))
					link_state &= ~BIT(n);
		} else
			link_state &= ~BIT(n);
	}

	return link_state;
}

int keystone_sgmii_config(int port, int interface)
{
	unsigned int i, status, mask;
	unsigned int mr_adv_ability, control;

	switch (interface) {
	case SGMII_LINK_MAC_MAC_AUTONEG:
		mr_adv_ability	= (SGMII_REG_MR_ADV_ENABLE |
				   SGMII_REG_MR_ADV_LINK |
				   SGMII_REG_MR_ADV_FULL_DUPLEX |
				   SGMII_REG_MR_ADV_GIG_MODE);
		control		= (SGMII_REG_CONTROL_MASTER |
				   SGMII_REG_CONTROL_AUTONEG);

		break;
	case SGMII_LINK_MAC_PHY:
	case SGMII_LINK_MAC_PHY_FORCED:
		mr_adv_ability	= SGMII_REG_MR_ADV_ENABLE;
		control		= SGMII_REG_CONTROL_AUTONEG;

		break;
	case SGMII_LINK_MAC_MAC_FORCED:
		mr_adv_ability	= (SGMII_REG_MR_ADV_ENABLE |
				   SGMII_REG_MR_ADV_LINK |
				   SGMII_REG_MR_ADV_FULL_DUPLEX |
				   SGMII_REG_MR_ADV_GIG_MODE);
		control		= SGMII_REG_CONTROL_MASTER;

		break;
	case SGMII_LINK_MAC_FIBER:
		mr_adv_ability	= 0x20;
		control		= SGMII_REG_CONTROL_AUTONEG;

		break;
	default:
		mr_adv_ability	= SGMII_REG_MR_ADV_ENABLE;
		control		= SGMII_REG_CONTROL_AUTONEG;
	}

	__raw_writel(0, SGMII_CTL_REG(port));

	/*
	 * Wait for the SerDes pll to lock,
	 * but don't trap if lock is never read
	 */
	for (i = 0; i < 1000; i++)  {
		udelay(2000);
		status = __raw_readl(SGMII_STATUS_REG(port));
		if ((status & SGMII_REG_STATUS_LOCK) != 0)
			break;
	}

	__raw_writel(mr_adv_ability, SGMII_MRADV_REG(port));
	__raw_writel(control, SGMII_CTL_REG(port));


	mask = SGMII_REG_STATUS_LINK;

	if (control & SGMII_REG_CONTROL_AUTONEG)
		mask |= SGMII_REG_STATUS_AUTONEG;

	for (i = 0; i < 1000; i++)  {
		status = __raw_readl(SGMII_STATUS_REG(port));
		if ((status & mask) == mask)
			break;
	}

	return 0;
}

#ifndef CONFIG_SOC_TCI6638
int serdes_config()
{
	struct serdes_config scfg;
	u32 mult, mpy;
	u32 status;

	mult = ((CONFIG_SYS_SGMII_LINERATE_MHZ * CONFIG_SYS_SGMII_RATESCALE) /
		CONFIG_SYS_SGMII_REFCLK_MHZ);

	mpy = mult << 2;
	scfg.cfg = (SERDES_ENABLE_PLL | (mpy << 1));

	scfg.n_lanes = 2;
	scfg.rx_cfg[0] = scfg.rx_cfg[1] = (SERDES_RX_ENABLE | SERDES_RX_RATE |
					SERDES_RX_TERM | SERDES_RX_ALIGN |
					SERDES_RX_ENOC | SERDES_RX_EQ);
	scfg.tx_cfg[0] = scfg.tx_cfg[1] = (SERDES_TX_ENABLE | SERDES_TX_RATE |
					SERDES_TX_CM | SERDES_TX_SWING |
					SERDES_TX_MSYNC);

	serdes_configure(TARGET_SGMII_SERDES_BASE, &scfg);

	status = serdes_wait_lock(TARGET_SGMII_SERDES_STATUS_BASE);

	return status;
}
#endif 

int32_t cpmac_drv_send(u_int8_t* buffer, int num_bytes)
{
	struct qm_host_desc *hd;
	int i;

	/*
	 * Must always setup the descriptor to
	 * have the minimum packet length
	 */
	if (num_bytes < EMAC_MIN_ETHERNET_PKT_SIZE)
		num_bytes = EMAC_MIN_ETHERNET_PKT_SIZE;

	for (i = 0, hd = NULL; hd == NULL; i++, udelay(1000))
		hd = qm_pop (DEVICE_QM_TX_Q);

	if (hd == NULL)
		return (-1);

	QM_DESC_DESCINFO_SET_PKT_LEN(hd->desc_info, num_bytes);

	hd->buff_len		= num_bytes;
	hd->orig_buff_len	= num_bytes;
	hd->buff_ptr		= (u_int32_t)buffer;
	hd->orig_buff_ptr	= (u_int32_t)buffer;
	hd->swinfo2 |= hd->swinfo2 & ~(0x00070000) |
		((CONFIG_SLAVE_PORT_NUM + 1) << 16);

	/* Return the descriptor back to the transmit queue */
	QM_DESC_PINFO_SET_QM(hd->packet_info, 0);
	QM_DESC_PINFO_SET_QUEUE(hd->packet_info, DEVICE_QM_TX_Q);

	qm_push (hd, DEVICE_QM_ETH_TX_Q, QM_DESC_SIZE_BYTES);

	return (0);
}

void free_queues(void)
{
	struct qm_host_desc *hd;

	do {
		hd = qm_pop(DEVICE_QM_FREE_Q);
	} while(hd != NULL);

	do {
		hd = qm_pop(DEVICE_QM_LNK_BUF_Q);
	} while(hd != NULL);

	do {
		hd = qm_pop(DEVICE_QM_RCV_Q);
	} while(hd != NULL);

	do {
		hd = qm_pop(DEVICE_QM_TX_Q);
	} while(hd != NULL);
}

int mac_sl_reset(u32 port)
{
	u32 i, v;

	if (port >= DEVICE_N_GMACSL_PORTS)
		return (GMACSL_RET_INVALID_PORT);

	/* Set the soft reset bit */
	DEVICE_REG32_W(DEVICE_EMACSL_BASE(port) +
		       CPGMACSL_REG_RESET, CPGMAC_REG_RESET_VAL_RESET);

	/* Wait for the bit to clear */
	for (i = 0; i < DEVICE_EMACSL_RESET_POLL_COUNT; i++) {
		v = DEVICE_REG32_R (DEVICE_EMACSL_BASE(port) +
				    CPGMACSL_REG_RESET);
		if ((v & CPGMAC_REG_RESET_VAL_RESET_MASK) !=
		    CPGMAC_REG_RESET_VAL_RESET)
			return (GMACSL_RET_OK);
	}

	/* Timeout on the reset */
	return (GMACSL_RET_WARN_RESET_INCOMPLETE);
}

int mac_sl_config(u_int16_t port, struct mac_sl_cfg *cfg)
{
	u32 v, i;
	int ret = GMACSL_RET_OK;

	if (port >= DEVICE_N_GMACSL_PORTS)
		return (GMACSL_RET_INVALID_PORT);

	if (cfg->max_rx_len > CPGMAC_REG_MAXLEN_LEN) {
		cfg->max_rx_len = CPGMAC_REG_MAXLEN_LEN;
		ret = GMACSL_RET_WARN_MAXLEN_TOO_BIG;
	}

	/* Must wait if the device is undergoing reset */
	for (i = 0; i < DEVICE_EMACSL_RESET_POLL_COUNT; i++) {
		v = DEVICE_REG32_R(DEVICE_EMACSL_BASE(port) +
				   CPGMACSL_REG_RESET);
		if ((v & CPGMAC_REG_RESET_VAL_RESET_MASK) !=
		    CPGMAC_REG_RESET_VAL_RESET)
			break;
	}

	if (i == DEVICE_EMACSL_RESET_POLL_COUNT)
		return (GMACSL_RET_CONFIG_FAIL_RESET_ACTIVE);

	DEVICE_REG32_W(DEVICE_EMACSL_BASE(port) + CPGMACSL_REG_MAXLEN,
			cfg->max_rx_len);

	DEVICE_REG32_W(DEVICE_EMACSL_BASE(port) + CPGMACSL_REG_CTL,
			cfg->ctl);

	return (ret);
}

int ethss_config(u32 ctl, u32 max_pkt_size)
{
	u32 i;

	/* Max length register */
	DEVICE_REG32_W(DEVICE_CPSW_BASE + CPSW_REG_MAXLEN, max_pkt_size);

	/* Control register */
	DEVICE_REG32_W(DEVICE_CPSW_BASE + CPSW_REG_CTL, ctl);

	/* All statistics enabled by default */
	DEVICE_REG32_W(DEVICE_CPSW_BASE + CPSW_REG_STAT_PORT_EN,
		       CPSW_REG_VAL_STAT_ENABLE_ALL);

	/* Reset and enable the ALE */
	DEVICE_REG32_W(DEVICE_CPSW_BASE + CPSW_REG_ALE_CONTROL,
		       CPSW_REG_VAL_ALE_CTL_RESET_AND_ENABLE |
		       CPSW_REG_VAL_ALE_CTL_BYPASS);

	/* All ports put into forward mode */
	for (i = 0; i < DEVICE_CPSW_NUM_PORTS; i++)
		DEVICE_REG32_W(DEVICE_CPSW_BASE + CPSW_REG_ALE_PORTCTL(i),
			       CPSW_REG_VAL_PORTCTL_FORWARD_MODE);

	return (0);
}

int ethss_start(void)
{
	int i;
	struct mac_sl_cfg cfg;

	cfg.max_rx_len	= MAX_SIZE_STREAM_BUFFER;
	cfg.ctl		= GMACSL_ENABLE | GMACSL_RX_ENABLE_EXT_CTL;

	for (i = 0; i < DEVICE_N_GMACSL_PORTS; i++) {
		mac_sl_reset(i);
		mac_sl_config(i, &cfg);
	}

	return (0);
}

int ethss_stop(void)
{
	int i;

	for (i = 0; i < DEVICE_N_GMACSL_PORTS; i++)
		mac_sl_reset(i);

	return (0);
}

/* Eth device open */
static int tci6614_eth_open(struct eth_device *dev, bd_t *bis)
{
	int32_t n;
	u_int32_t clkdiv;
	int link, status;
	int sgmii_interface[DEVICE_N_GMACSL_PORTS] = CONFIG_SYS_SGMII_INTERFACE;

	u_int8_t pkt_buf[EMAC_MIN_ETHERNET_PKT_SIZE];

	debug_emac("+ emac_open\n");

	psc_enable_module(TCI66XX_LPSC_PA);
	psc_enable_module(TCI66XX_LPSC_CPGMAC);

#ifndef CONFIG_SOC_TCI6638
	status = serdes_config();
	if (status == -1)
		return status;
#endif

	for (n = 0; n < DEVICE_N_GMACSL_PORTS; n++)
		keystone_sgmii_config(n, sgmii_interface[n]);

	udelay(10000);

	/* On chip switch configuration */
	ethss_config (targetGetSwitchCtl(), targetGetSwitchMaxPktSize());

	/* Queue manager configuration */
	qm_setup ((struct qm_config *)(targetGetQmConfig()));
	init_queues();

	/* Cpdma configuration. */
	packet_dma_rx_configure ((struct packet_dma_rx_cfg *)targetGetCpdmaRxConfig());
	packet_dma_tx_configure ((struct packet_dma_tx_cfg *)targetGetCpdmaTxConfig());

	tci6614_eth_set_mac_addr(dev);

	/*
	 * Streaming switch configuration. If not present this
	 * statement is defined to void in target.h.
	 * If present this is usually defined to a series of register writes
	 */
	hwConfigStreamingSwitch();

#ifndef CONFIG_SYS_NO_MDIO
	/* Init MDIO & get link state */
	clkdiv = (EMAC_MDIO_BUS_FREQ / EMAC_MDIO_CLOCK_FREQ) - 1;
	writel((clkdiv & 0xff) | MDIO_CONTROL_ENABLE | MDIO_CONTROL_FAULT,
	       &adap_mdio->CONTROL);

	/* We need to wait for MDIO to start */
	udelay(1000);

	link = keystone_get_link_status();
	if (link == 0)
		return -1;
#endif

	emac_gigabit_enable();

	ethss_start();

	debug_emac("- emac_open\n");

	emac_open = 1;

	return(0);
}

/* Eth device close */
void tci6614_eth_close(struct eth_device *dev)
{
	debug_emac("+ emac_close\n");

	if (!emac_open)
		return;

	ethss_stop();

	/* packet dma close */
	packet_dma_rx_disable((struct packet_dma_rx_cfg *)targetGetCpdmaRxConfig());
	packet_dma_tx_disable((struct packet_dma_tx_cfg *)targetGetCpdmaTxConfig());

	free_queues();
	qm_teardown ();

	psc_disable_module(TCI66XX_LPSC_CPGMAC);
	psc_disable_module(TCI66XX_LPSC_PA);
	psc_disable_domain(2);

	emac_open = 0;

	debug_emac("- emac_close\n");
}

static int tx_send_loop = 0;

/*
 * This function sends a single packet on the network and returns
 * positive number (number of bytes transmitted) or negative for error
 */
static int tci6614_eth_send_packet (struct eth_device *dev,
					volatile void *packet, int length)
{
	int ret_status = -1;
	int link;

	tx_send_loop = 0;

	link = keystone_get_link_status();
	if (link == 0)
		return -1;

	emac_gigabit_enable();

	if (cpmac_drv_send ((u_int8_t *) packet, length) != 0)
		return ret_status;

	link = keystone_get_link_status();
	if (link == 0)
		return -1;

	emac_gigabit_enable();

	return (length);
}

/*
 * This function handles receipt of a packet from the network
 */
static int tci6614_eth_rcv_packet (struct eth_device *dev)
{
	struct qm_host_desc *hd;
	int32_t pkt_size;

	hd = qm_pop(DEVICE_QM_RCV_Q);
	if (hd == NULL)
		return (0);

	pkt_size = QM_DESC_DESCINFO_GET_PKT_LEN(hd->desc_info);

	NetReceive ((volatile uchar *)hd->buff_ptr, pkt_size);

	hd->buff_len = hd->orig_buff_len;
	hd->buff_ptr = hd->orig_buff_ptr;

	qm_push(hd, DEVICE_QM_LNK_BUF_Q, QM_DESC_SIZE_BYTES);

	return (pkt_size);
}

/*
 * This function initializes the EMAC hardware. It does NOT initialize
 * EMAC modules power or pin multiplexors, that is done by board_init()
 * much earlier in bootup process. Returns 1 on success, 0 otherwise.
 */
int tci6614_emac_initialize(void)
{
	u_int32_t phy_id;
	u_int16_t tmp;
	int i;
	struct eth_device *dev;

	dev = malloc(sizeof *dev);
	if (dev == NULL)
		return -1;

	memset(dev, 0, sizeof *dev);
	sprintf(dev->name, "TCI6614-EMAC");

	dev->iobase		= 0;
	dev->init		= tci6614_eth_open;
	dev->halt		= tci6614_eth_close;
	dev->send		= tci6614_eth_send_packet;
	dev->recv		= tci6614_eth_rcv_packet;
	dev->write_hwaddr	= tci6614_eth_set_mac_addr;

	eth_register(dev);

#ifdef TCI6614_U_BOOT_MIN
	psc_disable_module(TCI66XX_LPSC_CPGMAC);
	psc_disable_module(TCI66XX_LPSC_PA);
	udelay(100);
#endif

	psc_enable_module(TCI66XX_LPSC_PA);
	psc_enable_module(TCI66XX_LPSC_CPGMAC);

#ifdef CONFIG_SOC_TCI6638
	SgmiiDefSerdesSetup156p25Mhz();
#endif

#ifndef CONFIG_SYS_NO_MDIO
	tci6614_eth_mdio_enable();

	for (i = 0; i < 256; i++) {
		if (readl(&adap_mdio->ALIVE))
			break;
		udelay(10);
	}

	if (i >= 256) {
		printf("No ETH PHY detected!!!\n");
		return(-1);
	}

	/* Find if a PHY is connected and get it's address */
	if (tci6614_eth_phy_detect())
		return(-1);

	/* Get PHY ID and initialize phy_ops for a detected PHY */
	if (tci6614_eth_phy_read(active_phy_addr, MII_PHYSID1, &tmp)) {
		active_phy_addr = 0xff;
		return(-1);
	}

	phy_id = (tmp << 16) & 0xffff0000;

	if (tci6614_eth_phy_read(active_phy_addr, MII_PHYSID2, &tmp)) {
		active_phy_addr = 0xff;
		return(-1);
	}

	phy_id |= tmp & 0x0000fff0; /* Ignore the revision number, bit[3:0] */

	switch (phy_id) {
		case PHY_MARVELL_88E1111:
			sprintf(phy.name, "88E1111 @ 0x%02x", active_phy_addr);
			phy.init = marvell_88e1111_init_phy;
			phy.is_phy_connected = marvell_88e1111_is_phy_connected;
			phy.get_link_speed = marvell_88e1111_get_link_speed;
			phy.auto_negotiate = marvell_88e1111_auto_negotiate;
			break;
		default:
			sprintf(phy.name, "GENERIC @ 0x%02x", active_phy_addr);
			phy.init = gen_init_phy;
			phy.is_phy_connected = gen_is_phy_connected;
			phy.get_link_speed = gen_get_link_speed;
			phy.auto_negotiate = gen_auto_negotiate;
	}

	printf("Ethernet PHY: %s\n", phy.name);

	miiphy_register(phy.name, tci6614_mii_phy_read, tci6614_mii_phy_write);
#endif

	return(0);
}

#ifdef CONFIG_SOC_TCI6638
#define reg_rmw(addr, value, mask) \
	writel(((readl(addr) & (~(mask))) | (value) ), (addr))

void SgmiiDefSerdesSetup156p25Mhz()
{
	unsigned int cnt;

	reg_rmw(0x0232a000, 0x00800000, 0xffff0000);
	reg_rmw(0x0232a014, 0x00008282, 0x0000ffff);
	reg_rmw(0x0232a060, 0x00142438, 0x00ffffff);
	reg_rmw(0x0232a064, 0x00c3c700, 0x00ffff00);
	reg_rmw(0x0232a078, 0x0000c000, 0x0000ff00);

	reg_rmw(0x0232a204, 0x38000080, 0xff0000ff);
	reg_rmw(0x0232a208, 0x00000000, 0x000000ff);
	reg_rmw(0x0232a20c, 0x02000000, 0xff000000);
	reg_rmw(0x0232a210, 0x1b000000, 0xff000000);
	reg_rmw(0x0232a214, 0x00006fb8, 0x0000ffff);
	reg_rmw(0x0232a218, 0x758000e4, 0xffff00ff);
	reg_rmw(0x0232a2ac, 0x00004400, 0x0000ff00);
	reg_rmw(0x0232a22c, 0x00100800, 0x00ffff00);
	reg_rmw(0x0232a280, 0x00820082, 0x00ff00ff);
	reg_rmw(0x0232a284, 0x1D0F0385, 0xFFFFFFFF);

	reg_rmw(0x0232a404, 0x38000080, 0xff0000ff);
	reg_rmw(0x0232a408, 0x00000000, 0x000000ff);
	reg_rmw(0x0232a40c, 0x02000000, 0xff000000);
	reg_rmw(0x0232a410, 0x1b000000, 0xff000000);
	reg_rmw(0x0232a414, 0x00006FB8, 0x0000FFFF);
	reg_rmw(0x0232a418, 0x758000E4, 0xFFFF00FF);
	reg_rmw(0x0232a4AC, 0x00004400, 0x0000FF00);
	reg_rmw(0x0232a42C, 0x00100800, 0x00FFFF00);
	reg_rmw(0x0232a480, 0x00820082, 0x00FF00FF);
	reg_rmw(0x0232a484, 0x1D0F0385, 0xFFFFFFFF);

	reg_rmw(0x0232a604, 0x38000080, 0xFF0000FF);
	reg_rmw(0x0232a608, 0x00000000, 0x000000FF);
	reg_rmw(0x0232a60C, 0x02000000, 0xFF000000);
	reg_rmw(0x0232a610, 0x1B000000, 0xFF000000);
	reg_rmw(0x0232a614, 0x00006FB8, 0x0000FFFF);
	reg_rmw(0x0232a618, 0x758000E4, 0xFFFF00FF);
	reg_rmw(0x0232a6AC, 0x00004400, 0x0000FF00);
	reg_rmw(0x0232a62C, 0x00100800, 0x00FFFF00);
	reg_rmw(0x0232a680, 0x00820082, 0x00FF00FF);
	reg_rmw(0x0232a684, 0x1D0F0385, 0xFFFFFFFF);

	reg_rmw(0x0232a804, 0x38000080, 0xFF0000FF);
	reg_rmw(0x0232a808, 0x00000000, 0x000000FF);
	reg_rmw(0x0232a80C, 0x02000000, 0xFF000000);
	reg_rmw(0x0232a810, 0x1B000000, 0xFF000000);
	reg_rmw(0x0232a814, 0x00006FB8, 0x0000FFFF);
	reg_rmw(0x0232a818, 0x758000E4, 0xFFFF00FF);
	reg_rmw(0x0232a8AC, 0x00004400, 0x0000FF00);
	reg_rmw(0x0232a82C, 0x00100800, 0x00FFFF00);
	reg_rmw(0x0232a880, 0x00820082, 0x00FF00FF);
	reg_rmw(0x0232a884, 0x1D0F0385, 0xFFFFFFFF);

	reg_rmw(0x0232aa00, 0x00000800, 0x0000FF00);
	reg_rmw(0x0232aa08, 0x38A20000, 0xFFFF0000);
	reg_rmw(0x0232aa30, 0x008A8A00, 0x00FFFF00);
	reg_rmw(0x0232aa84, 0x00000600, 0x0000FF00);
	reg_rmw(0x0232aa94, 0x10000000, 0xFF000000);
	reg_rmw(0x0232aaa0, 0x81000000, 0xFF000000);
	reg_rmw(0x0232aabc, 0xFF000000, 0xFF000000);
	reg_rmw(0x0232aac0, 0x0000008B, 0x000000FF);
	reg_rmw(0x0232ab08, 0x583F0000, 0xFFFF0000);
	reg_rmw(0x0232ab0c, 0x0000004e, 0x000000FF);
	reg_rmw(0x0232a000, 0x00000003, 0x000000FF);
	reg_rmw(0x0232aa00, 0x0000005F, 0x000000FF);

	/* Enable TX and RX via the LANExCTL_STS 0x0000 + x*4 */
	writel(0xF800F8C0, 0x0232bfe0);
	writel(0xF800F8C0, 0x0232bfe4);
	writel(0xF800F8C0, 0x0232bfe8);
	writel(0xF800F8C0, 0x0232bfec);

	/*Enable pll via the pll_ctrl 0x0014*/
	writel(0xe0000000, 0x0232bff4);

	/*Waiting for SGMII Serdes PLL lock.*/
	for (cnt = 10000;
	     cnt > 0 && ((readl(0x02090114) & 0x10) == 0);
	     cnt--);

	for (cnt = 10000;
	     cnt > 0 && ((readl(0x02090214) & 0x10) == 0);
	     cnt--);

	for (cnt = 10000;
	     cnt > 0 && ((readl(0x02090414) & 0x10) == 0);
	     cnt--);

	for (cnt = 10000;
	     cnt > 0 && ((readl(0x02090514) & 0x10) == 0);
	     cnt--);

	chip_delay(45000);
}

#endif

