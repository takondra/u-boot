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

#include <common.h>
#include <miiphy.h>
#include <linux/mtd/mtd.h>
#include <asm/arch/hardware.h>
#include <asm/arch/clock.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <ns16550.h>
#include <i2c.h>
#include <nand.h>
#include <spi_flash.h>
#include <asm/arch/nand_defs.h>
#include <asm/arch/emac_defs.h>
#include "post.h"

/* The linking RAM */
u8 pkt_buf[POST_EMAC_TEST_PKT_LENGTH+14] __attribute__((aligned(16)));

/******************************************************************************
 * Function:	fpga_spi_claim
 ******************************************************************************/
int fpga_spi_claim(void)
{
    /* Reset SPI */
    writel (SPI_REG_VAL_SPIGCR0_RESET, CONFIG_SYS_SPI_BASE + SPI_REG_SPIGCR0);
    udelay(1000);

    /* Release Reset */
    writel (SPI_REG_VAL_SPIGCR0_ENABLE, CONFIG_SYS_SPI_BASE + SPI_REG_SPIGCR0);

    /* CS1, CLK, in and out are functional pins, FPGA uses SPI CS1 */
    writel (0xe02, CONFIG_SYS_SPI_BASE + SPI_REG_SPIPC0);

    /* prescale=7, char len=16 */
    writel (0x710, CONFIG_SYS_SPI_BASE + SPI_REG_SPIFMT(0));

    /* C2TDELAY=0x6, T2CDELAY=0x3 */
    writel (0x6030000, CONFIG_SYS_SPI_BASE + SPI_REG_SPIDELAY);

    /* Master mode, enable SPI */
    writel (0x01000003, CONFIG_SYS_SPI_BASE + SPI_REG_SPIGCR1);

    return 0;
}

/******************************************************************************
 * Function:	fpga_spi_release
 ******************************************************************************/
int fpga_spi_release(void)
{
    /* Reset SPI */
    writel (SPI_REG_VAL_SPIGCR0_RESET, CONFIG_SYS_SPI_BASE + SPI_REG_SPIGCR0);

    return 0;
}

/******************************************************************************
 * Function:	fpga_spi_read
 ******************************************************************************/
int fpga_spi_read(u8 reg, u8* data)
{
	u32 v;

	writel (FPGA_READ_REG_CMD(reg), CONFIG_SYS_SPI_BASE + SPI_REG_SPIDAT0);
    udelay(10);

    v = readl(CONFIG_SYS_SPI_BASE + SPI_REG_SPIFLG);
    if ( v & 0x100)
        *data = readl(CONFIG_SYS_SPI_BASE + SPI_REG_SPIBUF) & 0xff;
    else
        return -1;

    return 0;
}

/******************************************************************************
 * Function:	fpga_spi_write
 ******************************************************************************/
int fpga_spi_write(u8 reg, u8 data)
{
	writel (FPGA_WRITE_REG_CMD(reg, data), CONFIG_SYS_SPI_BASE + SPI_REG_SPIDAT0);
    udelay(10);

    return 0;
}


/******************************************************************************
 * Function:	fpga_write_reg
 ******************************************************************************/
void fpga_write_reg(u8 addr, u8 data)
{
    fpga_spi_claim();
    fpga_spi_write(addr, data);
    fpga_spi_release();
 }

/******************************************************************************
 * Function:	fpga_read_reg
 ******************************************************************************/
int fpga_read_reg(u8 addr, uint8_t* data)
{
	int ret;

    fpga_spi_claim();
    ret = fpga_spi_read(addr, data);
    fpga_spi_release();
    if (ret)
        return -1;
	else
    	return 0;
}

/******************************************************************************
 * Function:	fpga_control_user_leds
 ******************************************************************************/
int fpga_control_user_leds(u32 led_num, POST_LED_STATE led_state)
{
    u8 led_value = 0; /* Default value */
    int ret;

    /* Read the current LED status */
    ret = fpga_read_reg(FPGA_DEBUG_LED_REG, &led_value);

    if (ret)
    	return -1;

    /* Turn on/off the corresponding LED bit number */
    switch (led_state) {
        case POST_LED_OFF:
            led_value |= (1 << led_num);
            break;
        case POST_LED_ON:
            led_value &= (~(1 << led_num));
            break;
        default: /* No action */
            break;
    }

    /* Send command to turn on/off the Corresponding LED */
    fpga_write_reg(FPGA_DEBUG_LED_REG, led_value);

    return 0;
}

/******************************************************************************
 * Function:	fpga_get_rev_id
 ******************************************************************************/
int fpga_get_rev_id(u32* rev_id)
{
    u8 rev = 0; /* Default value */
    int ret;

    /* Read the Rev ID Lo Reg */
    ret = fpga_read_reg(FPGA_REV_ID_LO_REG, &rev);

    if (ret)
    	return -1;

    *rev_id = rev;

    /* Read the Rev ID Hi Reg */
    ret = fpga_read_reg(FPGA_REV_ID_HI_REG, &rev);

    if (ret)
    	return -1;

    *rev_id |= (rev<<8);
    return 0;
}

/******************************************************************************
 * Function:    post_display_led_error
 ******************************************************************************/
void
post_display_led_error
(
    POST_TEST_ID     test_id
)
{
    POST_LED_STATE led_state[POST_MAX_NUM_LED];
    u32 i;
    u32 count = 20;

    memset(led_state, POST_LED_ON, POST_MAX_NUM_LED);
    while (TRUE)
    {
        for (i = 0; i < POST_MAX_NUM_LED; i++)
        {
            if (post_led_state[test_id][i] == POST_LED_BLINK)
            {
                if (led_state[i] == POST_LED_ON)
                {
                    led_state[i] = POST_LED_OFF;
                }
                else
                {
                    led_state[i] = POST_LED_ON;
                }
                fpga_control_user_leds(i, led_state[i]);
            }
            else
            {
                fpga_control_user_leds(i, post_led_state[test_id][i]);
            }
        }
        udelay(POST_LED_BLINK_DELAY);
        count--;
        if (count == 0)
        	return;
        /* POST in the while(1) loop to display the LED error status */
    }
}

/******************************************************************************
 * Function:    post_write_uart
 ******************************************************************************/
int post_write_uart
(
    char*      msg
)
{
	puts(msg);

    return 0;
}

/******************************************************************************
 * Function:    post_display_status
 ******************************************************************************/
void
post_display_status
(
    POST_TEST_ID        test_id,
    POST_TEST_RESULT    test_result
)
{
    u32    i;
    char        *msg;
    char        msg1[40] = "\r\n\rPOST ";
    char        msg2[] = " test passed!";
    char        msg3[] = " test failed!";
    char        msg4[] = " test started!";

    msg = strcat(msg1, post_status[test_id]);
    switch (test_id)
    {
    case POST_TEST_IN_PROGRESS:
    case POST_TEST_COMPLETE:
        /* Form the POST status message to write to the UART */
        if (post_write_uart(msg) != 0)
        {
            post_display_led_error(POST_TEST_UART);   /* Never return from this function */
        }

        for (i = 0; i < POST_MAX_NUM_LED; i++)
        {
            if (fpga_control_user_leds(i, post_led_state[test_id][i]) != 0)
            {
                post_write_uart("POST LED test failed \r\n");
            }
        }
        break;

    default:
        /* Form the POST status message to write to the UART */
        if (test_result == POST_TEST_RESULT_PASSED)
        {
            msg = strcat(msg, msg2);
            if (post_write_uart(msg) != 0)
            {
                post_display_led_error(POST_TEST_UART);   /* Never return from this function */
            }
        }
        else if (test_result == POST_TEST_RESULT_FAILED)
        {
            msg = strcat(msg, msg3);
            if (post_write_uart(msg) != 0)
            {
                post_display_led_error(POST_TEST_UART);   /* Never return from this function */
            }
            post_display_led_error(test_id);  /* Never return from this function */
        }
        else
        {
            msg = strcat(msg, msg4);
            if (post_write_uart(msg) != 0)
            {
                post_display_led_error(POST_TEST_UART);   /* Never return from this function */
            }
        }
        break;
    }
}

/******************************************************************************
 * Function:    post_test_external_memory
 ******************************************************************************/
POST_TEST_RESULT
post_test_external_memory
(
    void
)
{
    POST_TEST_RESULT    test_result = POST_TEST_RESULT_PASSED;

    if(ddr_memory_test(POST_DDR3_START_ADDR, POST_DDR3_END_ADDR, 1) != 0)
    {
        test_result = POST_TEST_RESULT_FAILED;
    }

    return test_result;
}

/******************************************************************************
 * Function:    post_test_eeprom
 ******************************************************************************/
POST_TEST_RESULT
post_test_eeprom
(
    void
)
{
    u8 test_buf[POST_EEPROM_TEST_READ_LENGTH];
    POST_TEST_RESULT test_result = POST_TEST_RESULT_PASSED;

	if (eeprom_read(POST_EEPROM_TEST_DEVICE_ID, POST_EEPROM_TEST_READ_ADDRESS, test_buf, POST_EEPROM_TEST_READ_LENGTH))
        test_result = POST_TEST_RESULT_FAILED;

    return test_result;
}

/******************************************************************************
 * Function:    post_test_nand
 ******************************************************************************/
POST_TEST_RESULT
post_test_nand
(
    void
)
{
    u8 test_buf[POST_NAND_TEST_READ_LENGTH];
    POST_TEST_RESULT test_result = POST_TEST_RESULT_PASSED;
    size_t rwsize = POST_NAND_TEST_READ_LENGTH;
    nand_info_t *nand = &nand_info[nand_curr_device];

    if(nand_read_skip_bad(nand, POST_NAND_TEST_READ_ADDR, &rwsize,
							 (u_char *)test_buf))
    {
        test_result = POST_TEST_RESULT_FAILED;
    }

    return test_result;
}

/******************************************************************************
 * Function:    post_test_nor
 ******************************************************************************/
POST_TEST_RESULT
post_test_nor
(
    void
)
{
    u8                 test_buf[POST_NOR_TEST_READ_LENGTH];
    POST_TEST_RESULT        test_result = POST_TEST_RESULT_PASSED;
	struct spi_flash *flash;

	flash = spi_flash_probe(0, 0, CONFIG_SF_DEFAULT_SPEED, CONFIG_SF_DEFAULT_MODE);
	if (!flash) {
		return POST_TEST_RESULT_FAILED;
	}

	if (spi_flash_read(flash, POST_NOR_TEST_READ_ADDR, POST_NOR_TEST_READ_LENGTH, test_buf))
        test_result = POST_TEST_RESULT_FAILED;

    spi_flash_free(flash);
    return test_result;
}

int32_t Init_sgmii (uint32_t port)
{
    u32 sgmiis[] = TARGET_SGMII_BASE_ADDRESSES;

    /* Reset the port before configuring it */
    SGMII_ACCESS(port, TARGET_SGMII_SOFT_RESET) = 1;
    udelay(100);

    /* Hold the port in soft reset and set up
     * the SGMII control register:
     *      (1) Enable Master Mode (default)
     *      (2) Enable Auto-negotiation
     */

    SGMII_ACCESS(port, TARGET_SGMII_SOFT_RESET) = 2;
    SGMII_ACCESS(port, TARGET_SGMII_CONTROL) = 0x31;

	/* Setup the Advertised Ability register for this port:
     *      (1) Enable Full duplex mode
     *      (2) Enable Auto Negotiation
     */
    SGMII_ACCESS(port, TARGET_SGMII_MR_ADV_ABILITY) = 0x1800;

    SGMII_ACCESS(port, TARGET_SGMII_SOFT_RESET) = 0;
    return 0;
}

int Init_MAC (uint32_t port, uint8_t* macAddress, uint32_t mtu)
{
    /* Reset MAC Sliver port */
	mac_sl_reset(port);

    /* Setup the MAC Control Register for this port:
     *      (1) Enable Full duplex
     *      (2) Enable GMII
     *      (3) Enable Gigabit
     *      (4) Enable External Configuration. This enables
     *          the "Full duplex" and "Gigabit" settings to be
     *          controlled externally from SGMII
     *      (5) Don't enable any control/error/short frames
     */
    writel(0x400a1, DEVICE_EMACSL_BASE(port) + CPGMACSL_REG_CTL);

    /* Configure the MAC address for this port */
    writel((macAddress[0]<<8)+macAddress[1], DEVICE_CPSW_BASE + 0x70 + 0x30*port);
    writel((macAddress[2]<<24)+(macAddress[3]<<16)+(macAddress[4]<<8)+macAddress[5],
    		DEVICE_CPSW_BASE + 0x74 + 0x30*port);

    /* Configure the Receive Maximum length on this port,
     * i.e., the maximum size the port can receive without
     * any errors.
     *
     * Set the Rx Max length to the MTU configured for the
     * interface.
     */
    writel(mtu, DEVICE_EMACSL_BASE(port) + CPGMACSL_REG_MAXLEN);

    return 0;
}

/******************************************************************************
 * Function:    post_test_emac_loopback
 ******************************************************************************/
extern unsigned int emac_open;
POST_TEST_RESULT
post_test_emac_loopback
(
    void
)
{
    struct eth_device dev;
    u32           pktSizeBytes;
    struct qm_host_desc   *hd;
    int   n;
	iblSgmii_t	sgmii_config[DEVICE_N_GMACSL_PORTS] = {
		{1, 1, 0x31, 0x108a1, 0x700621, 0x41},
		{1, 1, 0x31, 0x108a1, 0x700621, 0x41}
	};

	/* PHY loopback:
	tci6614_eth_phy_write(active_phy_addr, 0, 0x8140);
	if (!tci6614_eth_phy_read(active_phy_addr, 0, &tmp)) {
		tci6614_eth_phy_write(active_phy_addr, 0, (tmp |= 0x4000));
	}*/

	tci6614_eth_set_mac_addr(&dev);

    for (n = 0; n < DEVICE_N_GMACSL_PORTS; n++)  {
        if (sgmii_config[n].configure == TRUE)
		serdes_config();
        Init_sgmii(n);
        dev.enetaddr[5] += n;
        Init_MAC (n, dev.enetaddr, 1518);
        dev.enetaddr[5] -= n;
    }

    /* On chip switch configuration */
    ethss_config(CPSW_CTL_P0_ENABLE, 1518);
    /* Disable ALE */
    writel (0x80000010, DEVICE_CPSW_BASE + CPSW_REG_ALE_CONTROL);
    writel (0x13, DEVICE_CPSW_BASE+CPSW_REG_ALE_PORTCTL(0));
    writel (0x13, DEVICE_CPSW_BASE+CPSW_REG_ALE_PORTCTL(1));
    writel (0x13, DEVICE_CPSW_BASE+CPSW_REG_ALE_PORTCTL(2));

    /* Queue manager configuration */
    qm_setup ((struct qm_config *)(targetGetQmConfig()));
    init_queues();

    /* Cpdma configuration. */
    packet_dma_rx_configure((struct packet_dma_rx_cfg *)targetGetCpdmaRxConfig());
    packet_dma_tx_configure((struct packet_dma_tx_cfg *)targetGetCpdmaTxConfig());

    pa_configure(dev.enetaddr);

    /* Streaming switch configuration. If not present this statement is defined to void
     * in target.h.  If present this is usually defined to a series of register writes */
    hwConfigStreamingSwitch();

	memset (pkt_buf, 0, POST_EMAC_TEST_PKT_LENGTH+14);

    /* Set the dest MAC address to be broadcast, so that PA firmware will not filter out */
    memset(pkt_buf, 0xff, 6);

    /* Set the source MAC address */
    memcpy(&pkt_buf[6], dev.enetaddr, 6);

    /* set the payload length to 256 bytes */
    pkt_buf[12] = 0x01;
    pkt_buf[13] = 0x00;

	if (cpmac_drv_send (pkt_buf, POST_EMAC_TEST_PKT_LENGTH+14)!=0)
		return POST_TEST_RESULT_FAILED;

    udelay(100);

    /* Receive the loopback packet */
    hd = qm_pop (DEVICE_QM_RCV_Q);
    if (hd == NULL)
        return POST_TEST_RESULT_FAILED;

    pktSizeBytes = QM_DESC_DESCINFO_GET_PKT_LEN(hd->desc_info);

    hd->buff_len = hd->orig_buff_len;
    hd->buff_ptr = hd->orig_buff_ptr;

    qm_push (hd, DEVICE_QM_LNK_BUF_Q, QM_DESC_SIZE_BYTES);

	emac_open = 1;
	tci6614_eth_close(&dev);

    return POST_TEST_RESULT_PASSED;
}

void
post_hex_to_string
(
    u32    hex,
    u32    nibbles,
    char        *msg
)
{
    int32_t     i;
    u8     nibble;

    for (i = (nibbles-1); i >= 0; i--)
    {
        nibble = hex & 0xf;
        if (nibble <= 0x9)
        {
            nibble += '0';
        }
        else
        {
            nibble += ('A' - 0xa);
        }

        msg[i] = nibble;
        hex = hex >> 4;
    }
    msg[nibbles] = 0;
}

int post_serial_num_isvalid(char c)
{
    if (
        ((c >= '0') && (c <= '9'))    ||
        ((c >= 'a') && (c <= 'z'))    ||
        ((c >= 'A') && (c <= 'Z'))
       )
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

int post_uart_read(u8 *buf, NS16550_t com_port, u32 delay)
{
    u32 delay_count = delay/1000000;

    while(NS16550_tstc(com_port) == 0)
    {
        if (delay_count--)
        {
            udelay(1000000);
        }
        else
        {
            return -1;
        }
    }

    *buf = NS16550_getc(com_port);

    return 0;
}

/******************************************************************************
 * Function:    post_write_serial_no
 ******************************************************************************/
void
post_write_serial_no
(
    void
)
{
    u32                i, j;
    u8                 msg[20], msg2[2];

    /* Check if user key in the passcode "ti" to enter board serial number */
    if (post_uart_read(msg, (NS16550_t)CONFIG_SYS_NS16550_COM1, POST_UART_READ_TIMEOUT))
    {
        return;
    }
    if (msg[0] != 't')
    {
        return;
    }

    if (post_uart_read(msg, (NS16550_t)CONFIG_SYS_NS16550_COM1, POST_UART_READ_TIMEOUT))
    {
        return;
    }
    if (msg[0] != 'i')
    {
        return;
    }

    /* Passcode verified, prompt the user to enter serial number */
    post_write_uart("\r\n\r\nPlease enter the 10 digit serial number for this board, and then press ENTER key:\r\n\r\n");

    i = 0;
    msg2[1] = 0;
    while (TRUE)
    {
        if (post_uart_read(&msg[i], (NS16550_t)CONFIG_SYS_NS16550_COM1, POST_UART_READ_TIMEOUT))
        {
            post_write_uart("\r\n\r\nSerial number input time out!");
            return;
        }

        if (msg[i] == '\r')
        {
            break;
        }
        if ((i < POST_MAX_SN_SIZE) &&  !post_serial_num_isvalid(msg[i]))
        {
            msg2[0] = msg[i];
            post_write_uart((char *)msg2);
            i++;
        }
    }

    if (i < POST_MAX_SN_SIZE)
    {
        for (j = i; j < POST_MAX_SN_SIZE; j++)
        {
            msg[j] = 0xff;
        }
    }

    if (eeprom_write(CONFIG_SYS_I2C_EEPROM_ADDR, POST_SERIAL_NUM_ADDR, msg, 16) == 0)
    {
        post_write_uart("\r\n\r\nSerial number programmed to EEPROM successfully!\r\n\r\n");
    }
    else
    {
        post_write_uart("\r\n\r\nFailed to program the serial number to EEPROM!\r\n\r\n");
    }

}

void get_serial_number(char *buf)
{
    u32                i;

    buf[0] = 0;

    /* Serial number stored in the last 128 bytes of the EEPROM 0x50 */
	if (eeprom_read(POST_EEPROM_TEST_DEVICE_ID, POST_SERIAL_NUM_ADDR, (u_char  *)buf, 16))
        return;

    for (i = 0; i < POST_MAX_SN_SIZE; i++)
    {
        if(post_serial_num_isvalid(buf[i]))
            break;
    }

    buf[i] = 0;

    return;
}

/******************************************************************************
 * Function:    post_dump_register_val
 ******************************************************************************/
void
post_dump_register_val
(
    u32            reg_addr,
    char*               desc_string
)
{
   char                    msg[10];
   u32                reg_val;

   reg_val = *(volatile u32 *)reg_addr;
   post_write_uart(desc_string);
   post_hex_to_string(reg_val, 8, msg);
   msg[8] = ' ';
   msg[9] = 0;
   post_write_uart(msg);
}

int post_get_macaddr(u8* p_mac_address)
{
    u32 mac_addr2, mac_addr1;
    /* Read the e-fuse mac address */
    mac_addr1 = __raw_readl(MAC_ID_BASE_ADDR);
    mac_addr2 = __raw_readl(MAC_ID_BASE_ADDR+4);

    p_mac_address[0] = ((mac_addr2 & 0x0000ff00) >> 8);
    p_mac_address[1] =  (mac_addr2 & 0x000000ff);

    p_mac_address[2] = ((mac_addr1 & 0xff000000) >> 24);
    p_mac_address[3] = ((mac_addr1 & 0x00ff0000) >> 16);
    p_mac_address[4] = ((mac_addr1 & 0x0000ff00) >> 8);
    p_mac_address[5] =  (mac_addr1 & 0x000000ff);

	return 0;
}

/******************************************************************************
 * Function:    main function for POST
 ******************************************************************************/
void
tci6614_post
(
    void
)
{
    POST_TEST_ID            test_id = POST_TEST_IN_PROGRESS;
    POST_TEST_RESULT        test_result;
    u32                reset_type;
    int                 i;
    char                    msg[9];
    u8                 mac_addr[6];
    u32           board_rev;
    u32                sa_enable;
    u32                acc_fail;
	char serial_num[16];
	char *s;
	u32 no_post;

	if ((s = getenv ("no_post")) != NULL) {
		no_post = simple_strtoul (s, NULL, 16);
		if (no_post == 1)
			return;
	}

    acc_fail = 0;

	if (post_write_uart("\r\n\r\n") != 0)
		post_display_led_error(POST_TEST_UART);

	/* Display the board name */
	post_write_uart(PLATFORM_INFO_BOARD_NAME);

	/* Display the POST version */
	post_write_uart(POST_EVM_VERSION_MSG);
	post_write_uart(POST_VERSION);

	post_write_uart("\r\n\r------------------------------------------");
	post_write_uart("\r\n\rSOC Information");
	fpga_get_rev_id(&board_rev);
	post_hex_to_string(board_rev, 4, msg);
	post_write_uart("\r\n\r\nFPGA Version: ");
	post_write_uart(msg);

	get_serial_number(serial_num);
	if (serial_num[0] != 0) {
		post_write_uart("\r\n\rBoard Serial Number: ");
		post_write_uart(serial_num);
	}

	/* Display the EFUSE MAC address */
	post_get_macaddr(mac_addr);
	post_write_uart("\r\n\rEFUSE MAC ID is: ");
	for (i = 0; i < 6; i++) {
		post_hex_to_string(mac_addr[i], 2, msg);
		msg[2] = ' ';
		msg[3] = 0;
		post_write_uart(msg);
	}

	sa_enable = __raw_readl(0x20c0004);
	sa_enable &= 0x1;

	if (sa_enable)
		post_write_uart("\r\n\rSA is enabled on this board.");
	else
		post_write_uart("\r\n\rSA is disabled on this board.");

	/* Read the PLL Reset Type Status register and display on UART */
	reset_type = __raw_readl(TCI6614_PLL_CNTRL_RSTYPE);
	post_hex_to_string(reset_type, 8, msg);
	post_write_uart("\r\n\rPLL Reset Type Status Register: 0x");
	post_write_uart(msg);

    /* Dump Additional Information */
    post_write_uart("\r\n\rAdditional Information: ");
    post_dump_register_val (0x02350014, "\r\n\r   (0x02350014) :");
    post_dump_register_val (0x02350624, "\r\n\r   (0x02350624) :");
    post_dump_register_val (0x02350678, "\r\n\r   (0x02350678) :");
    post_dump_register_val (0x0235063C, "\r\n\r   (0x0235063C) :");
    post_dump_register_val (0x02350640, "\r\n\r   (0x02350640) :");
    post_dump_register_val (0x02350644, "\r\n\r   (0x02350644) :");
    post_dump_register_val (0x02350648, "\r\n\r   (0x02350648) :");
    post_dump_register_val (0x0235064C, "\r\n\r   (0x0235064C) :");
    post_dump_register_val (0x02350650, "\r\n\r   (0x02350650) :");
    post_dump_register_val (0x02350654, "\r\n\r   (0x02350654) :");
    post_dump_register_val (0x02350658, "\r\n\r   (0x02350658) :");
    post_dump_register_val (0x0235065C, "\r\n\r   (0x0235065C) :");
    post_dump_register_val (0x02350660, "\r\n\r   (0x02350660) :");
    post_dump_register_val (0x02350668, "\r\n\r   (0x02350668) :");
    post_dump_register_val (0x02350670, "\r\n\r   (0x02350670) :");

    post_dump_register_val (0x02620008, "\r\n\r   (0x02620008) :");
    post_dump_register_val (0x0262000c, "\r\n\r   (0x0262000c) :");
    post_dump_register_val (0x02620010, "\r\n\r   (0x02620010) :");
    post_dump_register_val (0x02620014, "\r\n\r   (0x02620014) :");
    post_dump_register_val (0x02620018, "\r\n\r   (0x02620018) :");
    post_dump_register_val (0x02620180, "\r\n\r   (0x02620180) :");

    post_write_uart("\r\n\r------------------------------------------");

    post_write_uart("\r\n\r\nPower On Self Test\n");

    /* Display test in progress UART/LED status or init error */
    post_display_status(test_id, POST_TEST_RESULT_STARTED);

    post_display_status(POST_TEST_EEPROM, POST_TEST_RESULT_STARTED);
    test_result = post_test_eeprom();
    if (test_result == POST_TEST_RESULT_FAILED)
    {
    	acc_fail++;
    }
    post_display_status(POST_TEST_EEPROM, test_result);

    post_display_status(POST_TEST_NOR, POST_TEST_RESULT_STARTED);
    test_result = post_test_nor();
    if (test_result == POST_TEST_RESULT_FAILED)
    {
    	acc_fail++;
    }
    post_display_status(POST_TEST_NOR, test_result);

    post_display_status(POST_TEST_NAND, POST_TEST_RESULT_STARTED);
    test_result = post_test_nand();
    if (test_result == POST_TEST_RESULT_FAILED)
    {
    	acc_fail++;
    }
    post_display_status(POST_TEST_NAND, test_result);

    post_display_status(POST_TEST_EMAC_LOOPBACK, POST_TEST_RESULT_STARTED);
    test_result = post_test_emac_loopback();
    if (test_result == POST_TEST_RESULT_FAILED)
    {
    	acc_fail++;
    }
    post_display_status(POST_TEST_EMAC_LOOPBACK, test_result);

    post_display_status(POST_TEST_DDR, POST_TEST_RESULT_STARTED);
    test_result = post_test_external_memory();
    if (test_result == POST_TEST_RESULT_FAILED)
    {
    	acc_fail++;
    }
    post_display_status(POST_TEST_DDR, test_result);

    post_display_status(POST_TEST_COMPLETE, POST_TEST_RESULT_PASSED);

    if (acc_fail == 0)
    {
    	post_write_uart("\r\n\r\nPOST result: PASS\r\n\r\n");
    }
    else
    {
    	post_write_uart("\r\n\r\nPOST result: FAIL\r\n\r\n");
    }

    post_write_serial_no();
}

