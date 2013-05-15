/*
 * TCI6638-EVM: Power On Self Test
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
#include <asm/arch/keystone_nav.h>
#include "post.h"

#define WAIT_OR_RETURN for(;;)
/* #define WAIT_OR_RETURN return (1) */

/******************************************************************************
 * Function:    post_test_external_memory
 ******************************************************************************/
int post_test_external_memory ( void )
{
	puts("POST extrenal memory ... ");
	if(ddr_memory_test(POST_DDR3_START_ADDR, POST_DDR3_END_ADDR, 1) != 0) {
		puts("FAILED\r\n");
		WAIT_OR_RETURN;
	}

	puts("PASSED\r\n"); 
	return 0;
}

/******************************************************************************
 * Function:    post_test_eeprom
 ******************************************************************************/
int post_test_eeprom (void)
{
	u8 test_buf[POST_EEPROM_TEST_READ_LENGTH];

	puts("POST I2C eeprom read ... ");
	if (eeprom_read(POST_EEPROM_TEST_DEVICE_ID, POST_EEPROM_TEST_READ_ADDRESS,
			 test_buf, POST_EEPROM_TEST_READ_LENGTH)) {
		puts("FAILED\r\n");
		WAIT_OR_RETURN;
	}
	puts("PASSED\r\n"); 
	return 0;
}

/******************************************************************************
 * Function:    post_test_nand
 ******************************************************************************/
int post_test_nand ( void )
{
	u8 test_buf[POST_NAND_TEST_READ_LENGTH];
	size_t rwsize = POST_NAND_TEST_READ_LENGTH;
	nand_info_t *nand = &nand_info[nand_curr_device];

	puts("POST EMIF NAND read ... ");

	if(nand_read_skip_bad(nand, POST_NAND_TEST_READ_ADDR, &rwsize,
					 (u_char *)test_buf)) {
		puts("FAILED\r\n");
		WAIT_OR_RETURN;
	}

	puts("PASSED\r\n"); 
	return 0;
}

void post_hex_to_string (u32 hex, u32 nibbles, char *msg)
{
    int32_t     i;
    u32 nbl;

    for (i = (nibbles-1); i >= 0; i--) {
	    nbl = hex & 0xf;
        msg[i] = (nbl < 10) ? '0' + nbl : 'a' - 10 + nbl;
        hex = hex >> 4;
    }
    msg[nibbles] = 0;
}

int post_uart_read(u8 *buf, NS16550_t com_port, u32 delay)
{
	u32 delay_count = delay/1000000;

	while(NS16550_tstc(com_port) == 0) {
		if (delay_count-- == 0)
			return -1;
		udelay(1000000);
	}

	*buf = NS16550_getc(com_port);
	return 0;
}

/******************************************************************************
 * Function:    post_write_serial_no
 ******************************************************************************/
void post_write_serial_no ( void )
{
    u32                i, j;
    u8                 msg[20], msg2[2];

    /* Check if user key in the passcode "ti" to enter board serial number */
    if (post_uart_read(msg, (NS16550_t)CONFIG_SYS_NS16550_COM1, POST_UART_READ_TIMEOUT))
        return;

    if (msg[0] != 't')
        return;

    if (post_uart_read(msg, (NS16550_t)CONFIG_SYS_NS16550_COM1, POST_UART_READ_TIMEOUT))
        return;
    if (msg[0] != 'i')
        return;

    /* Passcode verified, prompt the user to enter serial number */
    puts("\r\n\r\nPlease enter the 10 digit serial number for this board, and then press ENTER key:\r\n\r\n");

    i = 0;
    msg2[1] = 0;
    while (TRUE) {
        if (post_uart_read(&msg[i], (NS16550_t)CONFIG_SYS_NS16550_COM1, POST_UART_READ_TIMEOUT)) {
            puts("\r\n\r\nSerial number input time out!");
            return;
        }

        if (msg[i] == '\r') 
            break;
#if 0        
	if ((i < POST_MAX_SN_SIZE) &&  !post_serial_num_isvalid(msg[i])) {
            msg2[0] = msg[i];
            puts((char *)msg2);
            i++;
        }
#endif
    }

    if (i < POST_MAX_SN_SIZE) {
        for (j = i; j < POST_MAX_SN_SIZE; j++) {
            msg[j] = 0xff;
        }
    }

#if 0
    if (eeprom_write(CONFIG_SYS_I2C_EEPROM_ADDR, POST_SERIAL_NUM_ADDR, msg, 16) == 0)
        puts("\r\n\r\nSerial number programmed to EEPROM successfully!\r\n\r\n");
    else
#endif
        puts("\r\n\r\nFailed to program the serial number to EEPROM!\r\n\r\n");

}
#if 0
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
#endif
/******************************************************************************
 * Function:    post_dump_register_val
 ******************************************************************************/
void post_dump_register_val( u32 reg_addr, char* desc_string)
{
   char                    msg[10];
   u32                reg_val;

   reg_val = *(volatile u32 *)reg_addr;
   puts(desc_string);
   post_hex_to_string(reg_val, 8, msg);
   msg[8] = ' ';
   msg[9] = 0;
   puts(msg);
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
void tci6638_post ( void )
{
    u32         reset_type;
    int         i;
    char                    msg[9];
    u8                 mac_addr[6];
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

	puts("\r\n\r\n");

	/* Display the board name */
	puts(PLATFORM_INFO_BOARD_NAME);

	/* Display the POST version */
	puts(POST_EVM_VERSION_MSG);
	puts(POST_VERSION);

	puts("\r\n\r------------------------------------------");
	puts("\r\n\rSOC Information");
#if 0
	get_serial_number(serial_num);
	if (serial_num[0] != 0) {
		puts("\r\n\rBoard Serial Number: ");
		puts(serial_num);
	}
#endif
	/* Display the EFUSE MAC address */
	post_get_macaddr(mac_addr);
	puts("\r\n\rEFUSE MAC ID is: ");
	for (i = 0; i < 6; i++) {
		post_hex_to_string(mac_addr[i], 2, msg);
		msg[2] = ' ';
		msg[3] = 0;
		puts(msg);
	}

#if 0	
	/* Read the PLL Reset Type Status register and display on UART */
	reset_type = __raw_readl(TCI6614_PLL_CNTRL_RSTYPE);
	post_hex_to_string(reset_type, 8, msg);
	puts("\r\n\rPLL Reset Type Status Register: 0x");
	puts(msg);
#endif

	puts("\r\n\r------------------------------------------");
	puts("\r\n\r\nPower On Self Test\n");


	if (post_test_eeprom())
		acc_fail++;

	if (post_test_nand())
		acc_fail++;
    
	if (post_test_external_memory())
		acc_fail++;

	puts("Test complete\r\n"); 

	/* post_write_serial_no(); */
}
