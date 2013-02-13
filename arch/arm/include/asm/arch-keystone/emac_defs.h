/*
 * Copyright (C) 2012 Texas Instruments Incorporated
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

#ifndef _EMAC_DEFS_H_
#define _EMAC_DEFS_H_

#include <asm/arch/hardware.h>
#include <asm/io.h>

typedef enum { FALSE = 0, TRUE = 1 } bool;

#define DEVICE_REG32_R(a)			readl(a)
#define DEVICE_REG32_W(a,v)			writel(v,a)

/*#define chipLmbd(x,y) _lmbd(x,y) */

#define EMAC_EMACSL_BASE_ADDR			(TCI66XX_PASS_BASE + 0x00090900)
#define EMAC_MDIO_BASE_ADDR			(TCI66XX_PASS_BASE + 0x00090300)
#define EMAC_SGMII_BASE_ADDR			(TCI66XX_PASS_BASE + 0x00090100)

#define TCI6614_EMAC_GIG_ENABLE

#define MAC_ID_BASE_ADDR			(TCI66XX_DEVICE_STATE_CTRL_BASE + 0x110)

#ifdef CONFIG_SOC_TCI6614
/* MDIO module input frequency */
#define EMAC_MDIO_BUS_FREQ			(clk_get_rate(TCI6614_LPSC_PA)/3)
/* MDIO clock output frequency */
#define EMAC_MDIO_CLOCK_FREQ		1000000		/* 1.0 MHz */
#endif

#ifdef CONFIG_SOC_TCI6638
/* MDIO module input frequency */
#define EMAC_MDIO_BUS_FREQ			(clk_get_rate(pass_pll_clk))
/* MDIO clock output frequency */
#define EMAC_MDIO_CLOCK_FREQ		1000000		/* 1.0 MHz */
#endif

/* PHY mask - set only those phy number bits where phy is/can be connected */
#define EMAC_MDIO_PHY_NUM           CONFIG_EMAC_MDIO_PHY_NUM
#define EMAC_MDIO_PHY_MASK          (1 << EMAC_MDIO_PHY_NUM)

/* MII Status Register */
#define MII_STATUS_REG			1
#define MII_STATUS_LINK_MASK		(0x4)

/* Marvell 88E1111 PHY ID */
#define PHY_MARVELL_88E1111		(0x01410cc0)



#define MDIO_CONTROL_IDLE		(0x80000000)
#define MDIO_CONTROL_ENABLE		(0x40000000)
#define MDIO_CONTROL_FAULT_ENABLE	(0x40000)
#define MDIO_CONTROL_FAULT		(0x80000)
#define MDIO_USERACCESS0_GO		(0x80000000)
#define MDIO_USERACCESS0_WRITE_READ	(0x0)
#define MDIO_USERACCESS0_WRITE_WRITE	(0x40000000)
#define MDIO_USERACCESS0_ACK		(0x20000000)

#define EMAC_MACCONTROL_MIIEN_ENABLE		(0x20)
#define EMAC_MACCONTROL_FULLDUPLEX_ENABLE	(0x1)
#define EMAC_MACCONTROL_GIGABIT_ENABLE		(1 << 7)
#define EMAC_MACCONTROL_GIGFORCE		(1 << 17)
#define EMAC_MACCONTROL_RMIISPEED_100		(1 << 15)

#define EMAC_MIN_ETHERNET_PKT_SIZE	60

struct mac_sl_cfg {
	u_int32_t max_rx_len;	/* Maximum receive packet length. */
	u_int32_t ctl;		/* Control bitfield */
};

/*******************************************************************************************
 * Definition: Control bitfields used in the ctl field of hwGmacSlCfg_t
 *******************************************************************************************/
#define GMACSL_RX_ENABLE_RCV_CONTROL_FRAMES       (1 << 24)
#define GMACSL_RX_ENABLE_RCV_SHORT_FRAMES         (1 << 23)
#define GMACSL_RX_ENABLE_RCV_ERROR_FRAMES         (1 << 22)
#define GMACSL_RX_ENABLE_EXT_CTL                  (1 << 18)  /* duplex and gig read from input pins */
#define GMACSL_RX_ENABLE_GIG_FORCE                (1 << 17)
#define GMACSL_RX_ENABLE_IFCTL_B                  (1 << 16)
#define GMACSL_RX_ENABLE_IFCTL_A                  (1 << 15)
#define GMACSL_RX_ENABLE_CMD_IDLE                 (1 << 11)
#define GMACSL_TX_ENABLE_SHORT_GAP                (1 << 10)
#define GMACSL_ENABLE_GIG_MODE                    (1 <<  7)
#define GMACSL_TX_ENABLE_PACE                     (1 <<  6)
#define GMACSL_ENABLE                             (1 <<  5)
#define GMACSL_TX_ENABLE_FLOW_CTL                 (1 <<  4)
#define GMACSL_RX_ENABLE_FLOW_CTL                 (1 <<  3)
#define GMACSL_ENABLE_LOOPBACK                    (1 <<  1)
#define GMACSL_ENABLE_FULL_DUPLEX                 (1 <<  0)


/********************************************************************************************
 * DEFINTITION: function return values
 ********************************************************************************************/
#define GMACSL_RET_OK                        0
#define GMACSL_RET_INVALID_PORT             -1
#define GMACSL_RET_WARN_RESET_INCOMPLETE    -2
#define GMACSL_RET_WARN_MAXLEN_TOO_BIG      -3
#define GMACSL_RET_CONFIG_FAIL_RESET_ACTIVE -4


/* Register offsets */
#define CPGMACSL_REG_ID         0x00
#define CPGMACSL_REG_CTL        0x04
#define CPGMACSL_REG_STATUS     0x08
#define CPGMACSL_REG_RESET      0x0c
#define CPGMACSL_REG_MAXLEN     0x10
#define CPGMACSL_REG_BOFF       0x14
#define CPGMACSL_REG_RX_PAUSE   0x18
#define CPGMACSL_REG_TX_PAURSE  0x1c
#define CPGMACSL_REG_EM_CTL     0x20
#define CPGMACSL_REG_PRI        0x24


/* Soft reset register values */
#define CPGMAC_REG_RESET_VAL_RESET_MASK      (1 << 0)
#define CPGMAC_REG_RESET_VAL_RESET           (1 << 0)



/* Maxlen register values */
#define CPGMAC_REG_MAXLEN_LEN                0x3fff

/**
 *  @brief
 *      Hardware network subsystem support, ethernet switch
 */

 /* Control bitfields */
#define CPSW_CTL_P2_PASS_PRI_TAGGED     (1 << 5)
#define CPSW_CTL_P1_PASS_PRI_TAGGED     (1 << 4)
#define CPSW_CTL_P0_PASS_PRI_TAGGED     (1 << 3)
#define CPSW_CTL_P0_ENABLE              (1 << 2)
#define CPSW_CTL_VLAN_AWARE             (1 << 1)
#define CPSW_CTL_FIFO_LOOPBACK          (1 << 0)

#define DEVICE_CPSW_NUM_PORTS       3                    /* 3 switch ports */
#define DEVICE_CPSW_BASE            (0x02090800)
#define targetGetSwitchCtl()        CPSW_CTL_P0_ENABLE   /* Enable port 0 */
#define targetGetSwitchMaxPktSize() 9000

/* Register offsets */
#define CPSW_REG_CTL                0x004
#define CPSW_REG_STAT_PORT_EN       0x00c
#define CPSW_REG_MAXLEN             0x040
#define CPSW_REG_ALE_CONTROL        0x608
#define CPSW_REG_ALE_PORTCTL(x)     (0x640 + (x)*4)


/* Register values */
#define CPSW_REG_VAL_STAT_ENABLE_ALL             0xf
#define CPSW_REG_VAL_ALE_CTL_RESET_AND_ENABLE   ((u_int32_t)0xc0000000)
#define CPSW_REG_VAL_PORTCTL_FORWARD_MODE        0x3

#define PA_MAGIC_ID  0x0CEC11E0

#define PA_REG_MAILBOX_SLOT(pdsp, slot)		(0x00 + ((pdsp)*0x10) + ((slot)*0x04))
#define PA_REG_PDSP_CTL(pdsp)               (0x1000 + ((pdsp)*0x100))
#define PA_MEM_PDSP_IRAM(pdsp)				(0x10000 + ((pdsp)*0x8000))


/* The pdsp control register */
#define PA_REG_VAL_PDSP_CTL_ENABLE_PDSP(pcVal)   (((pcVal) << 16) | 0x3)
#define PA_REG_VAL_PDSP_CTL_DISABLE_PDSP         0

/* Number of mailbox slots for each PDPS */
#define PA_NUM_MAILBOX_SLOTS                4

struct pa_config {
	u_int32_t  mac0ms;      /* 32 most significant bits of the mac address */
	u_int32_t  mac0ls;      /* 32 least significant bits of the mac address, in the 16msbs of this word */
	u_int32_t  mac1ms;      /* 32 most significant bits of the mac address */
	u_int32_t  mac1ls;      /* 32 least significant bits of the mac address, in the 16 msbs of this word */
	u_int32_t  rx_qnum;     /* Receive packet queue number */
	u_int8_t   *cmd_buf;    /* Buffer used to create PA command */
};

#define DEVICE_PA_BASE                  TCI66XX_PASS_BASE
#define DEVICE_PA_NUM_PDSPS             6
#define DEVICE_PA_RUN_CHECK_COUNT       100         /* Number of loops to verify PA firmware is running */
#define chipLower8(x)                   ((x) & 0x00ff)

#define SERDES_RX_ENABLE		BIT(0)
#define SERDES_RX_RATE			(CONFIG_SYS_SGMII_RATESCALE << 4)
#define SERDES_RX_TERM			(4 << 7)
#define SERDES_RX_ALIGN			BIT(10)
#define SERDES_RX_ENOC			BIT(22)
#define SERDES_RX_EQ			(12 << 18)

#define SERDES_TX_ENABLE		BIT(0)
#define SERDES_TX_RATE			(CONFIG_SYS_SGMII_RATESCALE << 4)
#define	SERDES_TX_CM			BIT(7)
#define SERDES_TX_SWING			(8 << 8)
#define SERDES_TX_MSYNC			BIT(16)

#define SERDES_ENABLE_PLL		BIT(0)

#define SGMII_REG_STATUS_LOCK		BIT(4)
#define	SGMII_REG_STATUS_LINK		BIT(0)
#define SGMII_REG_STATUS_AUTONEG	BIT(2)
#define SGMII_REG_CONTROL_AUTONEG	BIT(0)
#define SGMII_REG_CONTROL_MASTER	BIT(5)
#define	SGMII_REG_MR_ADV_ENABLE		BIT(0)
#define	SGMII_REG_MR_ADV_LINK		BIT(15)
#define	SGMII_REG_MR_ADV_FULL_DUPLEX	BIT(12)
#define SGMII_REG_MR_ADV_GIG_MODE	BIT(11)

#define SGMII_LINK_MAC_MAC_AUTONEG	0
#define SGMII_LINK_MAC_PHY		1
#define SGMII_LINK_MAC_MAC_FORCED	2
#define SGMII_LINK_MAC_FIBER		3
#define SGMII_LINK_MAC_PHY_FORCED	4

#define TARGET_SGMII_EXTERNAL_SERDES
#define TARGET_SGMII_TYPE_2             /* Use second sgmii setup sequence */
#define TARGET_SGMII_BASE		TCI66XX_PASS_BASE + 0x00090100
#define TARGET_SGMII_BASE_ADDRESSES    {TCI66XX_PASS_BASE+0x00090100, TCI66XX_PASS_BASE+0x00090200}
#define TARGET_SGMII_SERDES_BASE        (TCI66XX_DEVICE_STATE_CTRL_BASE + 0x340)
#define TARGET_SGMII_SERDES_STATUS_BASE (TCI66XX_DEVICE_STATE_CTRL_BASE + 0x158)
#define TARGET_SGMII_SOFT_RESET         0x04
#define TARGET_SGMII_CONTROL            0x10
#define TARGET_SGMII_MR_ADV_ABILITY     0x18
//#define chipKickOpenSerdes(x)           *((u_int32_t *)0x2620038) = 0x83e70b13; *((u_int32_t *)0x262003c) = 0x95a4f1e0

#define SGMII_OFFSET(x)	((x <= 1)? (x * 0x100): ((x * 0x100) + 0x100))

/*
 * SGMII registers
 */
#define SGMII_IDVER_REG(x)    (TARGET_SGMII_BASE + SGMII_OFFSET(x) + 0x000)
#define SGMII_SRESET_REG(x)   (TARGET_SGMII_BASE + SGMII_OFFSET(x) + 0x004)
#define SGMII_CTL_REG(x)      (TARGET_SGMII_BASE + SGMII_OFFSET(x) + 0x010)
#define SGMII_STATUS_REG(x)   (TARGET_SGMII_BASE + SGMII_OFFSET(x) + 0x014)
#define SGMII_MRADV_REG(x)    (TARGET_SGMII_BASE + SGMII_OFFSET(x) + 0x018)
#define SGMII_LPADV_REG(x)    (TARGET_SGMII_BASE + SGMII_OFFSET(x) + 0x020)
#define SGMII_TXCFG_REG(x)    (TARGET_SGMII_BASE + SGMII_OFFSET(x) + 0x030)
#define SGMII_RXCFG_REG(x)    (TARGET_SGMII_BASE + SGMII_OFFSET(x) + 0x034)
#define SGMII_AUXCFG_REG(x)   (TARGET_SGMII_BASE + SGMII_OFFSET(x) + 0x038)

#define chipKickClosedSerdes(x)         ;       /* never lock the registers */
#define TARGET_SERDES_LOCK_DELAY        (1600*1000)

#define DEVICE_EMACSL_BASE(x)           (TCI66XX_PASS_BASE + 0x00090900 + (x)*0x040)
#define DEVICE_N_GMACSL_PORTS           2
#define DEVICE_EMACSL_RESET_POLL_COUNT  100

#define DEVICE_PSTREAM_CFG_REG_ADDR                 (TCI66XX_PASS_BASE + 0x604)
#define DEVICE_PSTREAM_CFG_REG_VAL_ROUTE_PDSP0      0

#ifdef CONFIG_SOC_TCI6614
#define DEVICE_PSTREAM_CFG_REG_VAL_ROUTE_CPPI      0x00000606
#endif
#ifdef CONFIG_SOC_TCI6638
#define DEVICE_PSTREAM_CFG_REG_VAL_ROUTE_CPPI      0x06060606
#endif

#define hwConfigStreamingSwitch()                   DEVICE_REG32_W(DEVICE_PSTREAM_CFG_REG_ADDR, DEVICE_PSTREAM_CFG_REG_VAL_ROUTE_CPPI);

#define SERDES_MAX_LANES    4

struct serdes_config {
	u_int32_t cfg;
	u_int32_t n_lanes;
	u_int32_t rx_cfg[SERDES_MAX_LANES];
	u_int32_t tx_cfg[SERDES_MAX_LANES];
};

/* Offsets */
#define SERDES_REG_CFG      0
#define SERDES_REG_RX(x)    (4 + 8*(x))
#define SERDES_REG_TX(x)    (8 + 8*(x))


/* Cfg register */
#define SERDES_SET_CFG_SLEEP(x,v)  BOOT_SET_BITFIELD((x),(v),10,10)
#define SERDES_GET_ENABLE(x)       BOOT_READ_BITFIELD((x),0,0)
#define SERDES_SET_ENABLE(x,v)     BOOT_SET_BITFIELD((x),(v),0,0)
#define SERDES_SET_MULT(x,v)       BOOT_SET_BITFIELD((x),(v),7,1)
#define SERDES_SET_VRANGE(x,v)     BOOT_SET_BITFIELD((x),(v),9,9)


/* Rx Cfg register */
#define SERDES_RX_CFG_SET_ENABLE(x,v)   BOOT_SET_BITFIELD((x),(v),0,0)
#define SERDES_RX_CFG_SET_RATE(x,v)     BOOT_SET_BITFIELD((x),(v),5,4)

/* Tx Cfg register */
#define SERDES_TX_CFG_SET_ENABLE(x,v)   BOOT_SET_BITFIELD((x),(v),0,0)
#define SERDES_TX_CFG_SET_RATE(x,v)     BOOT_SET_BITFIELD((x),(v),5,4)


/**
 *  @brief
 *      This structure is used to control the operation of the ibl sgmii ports
 *
 *  @details
 *      The physical register configuration is provided
 */
typedef struct iblSgmii_s
{
    bool    configure;          /**< Set to false to disable configuration */
    u_int32_t  adviseAbility;      /**< The advise ability register           */
    u_int32_t  control;            /**< The control register                  */
    u_int32_t  txConfig;           /**< Serdes Tx config                      */
    u_int32_t  rxConfig;           /**< Serdes Rx config                      */
    u_int32_t  auxConfig;          /**< Serdes Aux config                     */

} iblSgmii_t;


/* EMAC MDIO Registers Structure */
typedef struct  {
	dv_reg		VERSION;
	dv_reg		CONTROL;
	dv_reg		ALIVE;
	dv_reg		LINK;
	dv_reg		LINKINTRAW;
	dv_reg		LINKINTMASKED;
	u_int8_t	RSVD0[8];
	dv_reg		USERINTRAW;
	dv_reg		USERINTMASKED;
	dv_reg		USERINTMASKSET;
	dv_reg		USERINTMASKCLEAR;
	u_int8_t	RSVD1[80];
	dv_reg		USERACCESS0;
	dv_reg		USERPHYSEL0;
	dv_reg		USERACCESS1;
	dv_reg		USERPHYSEL1;
} mdio_regs;


/* Ethernet MAC Registers Structure */
typedef struct  {
    dv_reg		IDVER;
    dv_reg		MACCONTROL;
    dv_reg		MACSTATUS;
    dv_reg		SOFT_RESET;
    dv_reg		RX_MAXLEN;
    u_int8_t	RSVD0[4];
    dv_reg		RX_PAUSE;
    dv_reg		TX_PAUSE;
    dv_reg		EMCONTROL;
    dv_reg		PRI_MAP;

} emac_regs;

typedef struct
{
	char	name[64];
	int	(*init)(int phy_addr);
	int	(*is_phy_connected)(int phy_addr);
	int	(*get_link_speed)(int phy_addr);
	int	(*auto_negotiate)(int phy_addr);
} phy_t;

#define SGMII_ACCESS(port,reg)   *((volatile unsigned int *)(sgmiis[port] + reg))

#endif  /* _EMAC_DEFS_H_ */
