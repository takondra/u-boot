/*
 * TCI6614-EVM: Power On Self Test
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * See file CREDITS for list of people who contributed to this
 * project.
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

#ifndef _POST_H_
#define _POST_H_

/******************************************************************************
 * POST version definitions
 ******************************************************************************/
#define POST_EVM_VERSION_MSG        " POST Version "
#define POST_VERSION                "01.00.00.01"


/******************************************************************************
 * PLL Controller Reset Type Status register
 *
 * Bit 31-29    28   27-12  11-8     7-3      2         1     0
 *     Rsvd  EMU-RST Rsvd  WDRST[N]  Rsvd PLLCTRLRST /RESET  POR
 ******************************************************************************/
#define TCI6614_PLL_CNTRL_RSTYPE	(TCI6614_PLL_CNTRL_BASE + 0xe4)

/******************************************************************************
 * UART definitions
 ******************************************************************************/
#define POST_UART_READ_TIMEOUT     (10 * 1000000) /* in usec */

/******************************************************************************
 * FPGA debug LED definitions
 ******************************************************************************/
typedef enum
{
    POST_LED_OFF     = 0,       /* LED is steady off */
    POST_LED_ON,                /* LED is steady on */
    POST_LED_BLINK              /* LED is blinking */
} POST_LED_STATE;

typedef enum
{
    POST_TEST_RESULT_STARTED = 0,   /* POST Test Result Started */
    POST_TEST_RESULT_PASSED,        /* POST Test Result Passed */
    POST_TEST_RESULT_FAILED         /* POST Test Result Failed */
} POST_TEST_RESULT;

typedef enum
{
    POST_TEST_IN_PROGRESS = 0,      /* POST running in progress */
    POST_TEST_COMPLETE,             /* POST done successfully */
    POST_TEST_DDR,                  /* POST external memory test */
    POST_TEST_EEPROM,               /* POST I2C EEPROM read test */
    POST_TEST_NAND,                 /* POST EMIF16 NAND read test */
    POST_TEST_NOR,                  /* POST SPI NOR read test */
    POST_TEST_UART,                 /* POST UART write test */
    POST_TEST_EMAC_LOOPBACK,        /* POST EMAC loopback test */
    POST_TEST_PLL_INIT,             /* POST PLL initialization */
    POST_TEST_NAND_INIT,            /* POST NAND initialization */
    POST_TEST_NOR_INIT,             /* POST NOR initialization  */
    POST_TEST_GENERAL,              /* POST test general */
    POST_MAX_TEST_NUM               /* Maximum number of POST LED tests */
} POST_TEST_ID;

#define POST_MAX_NUM_LED        4   /* Total number of LEDs on the EVM */
static uint8_t post_led_state[POST_MAX_TEST_NUM][POST_MAX_NUM_LED] =
{
    {POST_LED_ON, POST_LED_ON, POST_LED_ON, POST_LED_ON},           /* POST running in progress */
    {POST_LED_OFF, POST_LED_OFF, POST_LED_OFF, POST_LED_OFF},       /* POST done successfully */
    {POST_LED_BLINK, POST_LED_OFF, POST_LED_OFF, POST_LED_OFF},     /* POST external memory test failed */
    {POST_LED_OFF, POST_LED_BLINK, POST_LED_OFF, POST_LED_OFF},     /* POST I2C EEPROM read test failed */
    {POST_LED_OFF, POST_LED_OFF, POST_LED_BLINK, POST_LED_OFF},     /* POST EMIF16 NAND read test failed */
    {POST_LED_OFF, POST_LED_OFF, POST_LED_OFF, POST_LED_BLINK},     /* POST SPI NOR read test failed */
    {POST_LED_BLINK, POST_LED_BLINK, POST_LED_OFF, POST_LED_OFF},   /* POST UART write test failed */
    {POST_LED_OFF, POST_LED_BLINK, POST_LED_BLINK, POST_LED_OFF},   /* POST EMAC loopback test failed */
    {POST_LED_OFF, POST_LED_OFF, POST_LED_BLINK, POST_LED_BLINK},   /* POST PLL initialization failed */
    {POST_LED_BLINK, POST_LED_BLINK, POST_LED_BLINK, POST_LED_OFF}, /* POST NAND initialization failed */
    {POST_LED_OFF, POST_LED_BLINK, POST_LED_BLINK, POST_LED_BLINK}, /* POST NOR initialization failed */
    {POST_LED_BLINK, POST_LED_BLINK, POST_LED_BLINK, POST_LED_BLINK}, /* POST general failure */
};

#define POST_STATUS_MAX_NUM_CHAR            25      /* Maximum char length of the POST status string */
static char post_status[POST_MAX_TEST_NUM][POST_STATUS_MAX_NUM_CHAR] =
{
    "running in progress ...",
    "done successfully!",
    "external memory",
    "I2C EEPROM read",
    "EMIF16 NAND read",
    "SPI NOR read",
    "UART write",
    "EMAC loopback",
    "PLL initialization",
    "NAND initialization",
    "NOR initialization",
    "general ",
};

#define POST_LED_BLINK_DELAY    500000  /* 500,000 usec blinking delay */

/******************************************************************************
 * I2C EEPROM test definitions
 ******************************************************************************/
#define POST_EEPROM_TEST_DEVICE_ID      0x50				     	/* I2C slave bus address 0x50 */
#define POST_EEPROM_TEST_READ_ADDRESS   0                           /* Byte address */
#define POST_EEPROM_TEST_READ_LENGTH    12                          /* Read length in Bytes */

/******************************************************************************
 * NAND test definitions
 ******************************************************************************/
#define POST_NAND_TEST_READ_ADDR        0x100000 /* NAND read offset address, skip the U-boot partition */
#define POST_NAND_TEST_READ_LENGTH      2048     /* Read length in bytes */

/******************************************************************************
 * NOR test definitions
 ******************************************************************************/
#define POST_NOR_TEST_READ_ADDR         0       /* NOR read offset address */
#define POST_NOR_TEST_READ_LENGTH       256     /* Read length in bytes */

/******************************************************************************
 * EMAC test definitions
 ******************************************************************************/
#define POST_EMAC_TEST_PKT_LENGTH   256     /* Ethernet packet payload size in bytes */

/******************************************************************************
 * DDR test definitions
 ******************************************************************************/
#define POST_DDR3_START_ADDR 0xa0000000
#define POST_DDR3_END_ADDR   0xc0000000 /* 512 MB */

/******************************************************************************
 * Serial Number definitions
 ******************************************************************************/
#define POST_MAX_SN_SIZE       10          /* Maximum number of the chars of Serial Number for the EVM */
#define POST_SERIAL_NUM_ADDR   (65536-128) /* Last 128 bytes of EEPROM 0x50 stores the S/N */

/******************************************************************************
 * FPGA definitions
 ******************************************************************************/
#define FPGA_READ_REG_CMD(addr)         ((addr | 0x80) << 8)
#define FPGA_WRITE_REG_CMD(addr, value)   (((addr & 0x7f) << 8) | (value & 0xff))

#define FPGA_DEVICE_ID_LO_REG   0x00
#define FPGA_DEVICE_ID_HI_REG   0x01
#define FPGA_REV_ID_LO_REG      0x02
#define FPGA_REV_ID_HI_REG      0x03
#define FPGA_DEBUG_LED_REG      0x08

/******************************************************************************
 * SPI definitions
 ******************************************************************************/
/* Register offset */
#define SPI_REG_SPIGCR0         0x00
#define SPI_REG_SPIGCR1         0x04
#define SPI_REG_SPIFLG          0x10
#define SPI_REG_SPIPC0          0x14
#define SPI_REG_SPIDAT0         0x38
#define SPI_REG_SPIDAT1         0x3c
#define SPI_REG_SPIBUF          0x40
#define SPI_REG_SPIDELAY        0x48
#define SPI_REG_SPIFMT(x)       (0x50 + ((x)*4))

/* Register values */
#define SPI_REG_VAL_SPIGCR0_RESET           0x0
#define SPI_REG_VAL_SPIGCR0_ENABLE          0x1

#ifndef CONFIG_SF_DEFAULT_SPEED
# define CONFIG_SF_DEFAULT_SPEED	1000000
#endif
#ifndef CONFIG_SF_DEFAULT_MODE
# define CONFIG_SF_DEFAULT_MODE		SPI_MODE_3
#endif

#define PLATFORM_INFO_BOARD_NAME     "TMDXEVMTCI6614"

extern int ddr_memory_test (u32 start_address, u32 end_address, int quick);

extern void tci6614_eth_close(struct eth_device *dev);
extern struct qm_host_desc *qm_pop (u_int32_t qnum);
extern void qm_push (struct qm_host_desc *hd, u_int32_t qnum, u_int32_t descrSize);
extern int qm_setup (struct qm_config *cfg);
extern void *targetGetQmConfig (void);
extern void init_queues(void);
extern int packet_dma_rx_configure(const struct packet_dma_rx_cfg *cfg);
extern void *targetGetCpdmaRxConfig (void);
extern int packet_dma_tx_configure(const struct packet_dma_tx_cfg *cfg);
extern void *targetGetCpdmaTxConfig (void);
extern int tci6614_eth_set_mac_addr(struct eth_device *dev);
extern void pa_configure(u8 *mac_addr);
extern int32_t cpmac_drv_send (u_int8_t* buffer, int num_bytes);
extern u_int8_t	active_phy_addr;
extern void serdes_config(void);
extern int mac_sl_reset(u32 port);
extern int ethss_config(u32 ctl, u32 max_pkt_size);

#endif  /* _POST_H_ */
