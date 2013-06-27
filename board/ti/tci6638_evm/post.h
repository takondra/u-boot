/*
 * TCI6638-EVM: Power On Self Test
 *
 * Copyright (C) 2013 Texas Instruments
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
#define POST_VERSION                "01.00.02.01"


/******************************************************************************
 * UART definitions
 ******************************************************************************/
#define POST_UART_READ_TIMEOUT     (10 * 1000000) /* in usec */

/******************************************************************************
 * I2C EEPROM test definitions
 ******************************************************************************/
#define POST_EEPROM_TEST_DEVICE_ID      0x50	/* I2C slave bus address 0x50 */
#define POST_EEPROM_TEST_READ_ADDRESS   0       /* Byte address */
#define POST_EEPROM_TEST_READ_LENGTH    12      /* Read length in Bytes */

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
#define POST_DDR3_START_ADDR 0x80000000
#define POST_DDR3_END_ADDR   0xb0000000 /* 1.5 GB */

/******************************************************************************
 * Serial Number definitions
 ******************************************************************************/
#define POST_MAX_SN_SIZE       10          /* Maximum number of the chars of Serial Number for the EVM */
#define POST_SERIAL_NUM_ADDR   (65536-128) /* Last 128 bytes of EEPROM 0x50 stores the S/N */

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

#define PLATFORM_INFO_BOARD_NAME     "TCI6638_EVM"

extern int ddr_memory_test (u32 start_address, u32 end_address, int quick);

int tci6614_eth_phy_read(u_int8_t phy_addr, u_int8_t reg_num,
			 u_int16_t *data);
int tci6614_eth_phy_write(u_int8_t phy_addr, u_int8_t reg_num,
			  u_int16_t data);

#endif  /* _POST_H_ */
