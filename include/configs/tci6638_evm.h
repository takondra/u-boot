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

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_SOC_TCI6638

#include <asm/arch/hardware.h>
#include <asm/arch/clock.h>

/* TCI6638 EVM board */
#define CONFIG_TCI6638_EVM

#define CONFIG_SKIP_LOWLEVEL_INIT	/* U-Boot is a 2nd stage loader */
#define CONFIG_SYS_NO_FLASH		/* that is, no *NOR* flash */
#define CONFIG_SYS_CONSOLE_INFO_QUIET
#define CONFIG_BOARD_EARLY_INIT_F

/* SoC Configuration */
#define CONFIG_ARMV7						/* ARM Cortex A8 CPU */
#define CONFIG_ARCH_CPU_INIT
#define CONFIG_SYS_ARCH_TIMER
#define CONFIG_SYS_HZ_CLOCK		clk_get_rate(TCI6638_CLK1_6)
#define CONFIG_SYS_HZ			1000			/* ticks per second */

#define CONFIG_SYS_TEXT_BASE		0x0c001000

#define CONFIG_OF_LIBFDT	1

#define CONFIG_L2_OFF

/* Memory Info */
#define CONFIG_NR_DRAM_BANKS		1
#if 0
#define PHYS_SDRAM_1			0x80000000
#define PHYS_SDRAM_1_SIZE		(512 << 20)	/* 2 MiB */
#else
#define MSM_RELOCATE_ADDR		0x0c200000
#define MSM_SIZE			(2 << 20)	/* 2 MB */
#endif

/* SPL SPI Loader Configuration */
#define CONFIG_SPL_TEXT_BASE		0x0c200000
#define CONFIG_SPL_MAX_SIZE		(32 * 1024)
#define CONFIG_SPL_BSS_START_ADDR	(CONFIG_SPL_TEXT_BASE +		\
					 CONFIG_SPL_MAX_SIZE)
#define CONFIG_SPL_BSS_MAX_SIZE		(32 * 1024)
#define CONFIG_SYS_SPL_MALLOC_START	(CONFIG_SPL_BSS_START_ADDR +	\
					 CONFIG_SPL_BSS_MAX_SIZE)
#define CONFIG_SYS_SPL_MALLOC_SIZE	(32 * 1024)
#define CONFIG_SPL_STACK_SIZE		(8 * 1024)
#define CONFIG_SPL_STACK		(CONFIG_SYS_SPL_MALLOC_START +	\
					 CONFIG_SYS_SPL_MALLOC_SIZE +	\
					 CONFIG_SPL_STACK_SIZE - 4)
#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_LIBGENERIC_SUPPORT
#define CONFIG_SPL_SERIAL_SUPPORT
#define CONFIG_SPL_SPI_FLASH_SUPPORT
#define CONFIG_SPL_SPI_SUPPORT
#define CONFIG_SPL_BOARD_INIT
#define CONFIG_SPL_SPI_LOAD
#define CONFIG_SPL_SPI_BUS		0
#define CONFIG_SPL_SPI_CS		0
#define CONFIG_SYS_SPI_U_BOOT_OFFS	(64 * 1024)
#define CONFIG_SPL_FRAMEWORK

/* Serial Driver info: UART0 for console  */
#define CONFIG_SYS_NS16550
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	-4
#define CONFIG_SYS_NS16550_COM1		TCI6638_UART0_BASE
#define CONFIG_SYS_NS16550_CLK		clk_get_rate(TCI6638_CLK1_6)
#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

#define CONFIG_SPI
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_DAVINCI_SPI
#define CONFIG_SYS_SPI_BASE		TCI6638_SPI_BASE
#define CONFIG_SYS_SPI_CLK		clk_get_rate(TCI6638_LPSC_EMIF25_SPI)
#define CONFIG_SF_DEFAULT_SPEED		30000000
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#define CONFIG_CMD_SF

/* EEPROM definitions */
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN		2
#define CONFIG_SYS_I2C_EEPROM_ADDR		0x50
#define CONFIG_SYS_EEPROM_PAGE_WRITE_BITS	6
#define CONFIG_SYS_EEPROM_PAGE_WRITE_DELAY_MS	20

/* Network Configuration */
#define CONFIG_DRIVER_TI_KEYSTONE_NET
#define CONFIG_EMAC_MDIO_PHY_NUM	1
#define CONFIG_MII
#define CONFIG_BOOTP_DEFAULT
#define CONFIG_BOOTP_DNS
#define CONFIG_BOOTP_DNS2
#define CONFIG_BOOTP_SEND_HOSTNAME
#define CONFIG_NET_RETRY_COUNT	10
#define CONFIG_NET_MULTI
#define CONFIG_SYS_SGMII_INTERFACE	{SGMII_LINK_MAC_PHY, \
					SGMII_LINK_MAC_PHY   \
					}
#define CONFIG_SYS_SGMII_REFCLK_MHZ	312
#define CONFIG_SYS_SGMII_LINERATE_MHZ	1250
#define	CONFIG_SYS_SGMII_RATESCALE	2

/* I2C */
#define CONFIG_HARD_I2C
#define CONFIG_DRIVER_DAVINCI_I2C
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_SYS_I2C_SLAVE		0x10	/* SMBus host address */

/* NAND: socketed, two chipselects, normally 2 GBytes */
#define CONFIG_NAND_DAVINCI
#define CONFIG_SYS_NAND_CS		2
#define CONFIG_SYS_NAND_USE_FLASH_BBT
#define CONFIG_SYS_NAND_4BIT_HW_ECC_OOBFIRST
#define CONFIG_SYS_NAND_PAGE_2K

#define CONFIG_SYS_NAND_LARGEPAGE
#define CONFIG_SYS_NAND_BASE_LIST	{ 0x70000000, }
/* socket has two chipselects, nCE0 gated by address BIT(14) */
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_MAX_CHIPS	1

/* U-Boot command configuration */
#include <config_cmd_default.h>

#undef CONFIG_CMD_BDI
#undef CONFIG_CMD_FLASH
#undef CONFIG_CMD_FPGA
#undef CONFIG_CMD_SETGETDCR

#define CONFIG_CMD_ASKENV
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_I2C
#define CONFIG_CMD_PING
#define CONFIG_CMD_SAVES

#ifdef CONFIG_MMC
#define CONFIG_DOS_PARTITION
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_CMD_MMC
#endif

#ifdef CONFIG_NAND_DAVINCI
#define CONFIG_CMD_MTDPARTS
#define CONFIG_MTD_PARTITIONS
#define CONFIG_MTD_DEVICE
#define CONFIG_CMD_NAND
#define CONFIG_CMD_UBI
#define CONFIG_RBTREE
#endif

#define CONFIG_CRC32_VERIFY
#define CONFIG_MX_CYCLIC

/* U-Boot general configuration */
#undef CONFIG_USE_IRQ				/* No IRQ/FIQ in U-Boot */
#define CONFIG_BOOTFILE		"uImage"	/* Boot file name */
#define CONFIG_SYS_PROMPT	"TCI6638 EVM # "	/* Monitor Command Prompt */
#define CONFIG_SYS_CBSIZE	1024		/* Console I/O Buffer Size  */
#define CONFIG_SYS_PBSIZE			/* Print buffer size */ \
		(CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS	16		/* max number of command args */
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#define CONFIG_SYS_LONGHELP

#ifdef CONFIG_NAND_DAVINCI
#define CONFIG_ENV_SIZE		(256 << 10)	/* 256 KiB */
#define CONFIG_ENV_IS_IN_NAND
#define CONFIG_ENV_OFFSET	0x100000
#undef CONFIG_ENV_IS_IN_FLASH
#else
#define CONFIG_ENV_SIZE                (256 << 10)     /* 256 KiB */
#define CONFIG_ENV_IS_NOWHERE
#endif

#if 0
#if defined(CONFIG_MMC) && !defined(CONFIG_ENV_IS_IN_NAND)
#define CONFIG_CMD_ENV
#define CONFIG_ENV_SIZE		(16 << 10)	/* 16 KiB */
#define CONFIG_ENV_OFFSET	(51 << 9)	/* Sector 51 */
#define CONFIG_ENV_IS_IN_MMC
#undef CONFIG_ENV_IS_IN_FLASH
#endif
#endif

#define CONFIG_BOOTDELAY	99
#define CONFIG_BOOTCOMMAND \
	"xx"

#define CONFIG_BOOTARGS \
		"console=ttyS0,115200n8 debug earlyprintk " \
		" rdinit=/bin/ash rw root=/dev/ram0 initrd=0x85000000,9M"

#define CONFIG_CMDLINE_EDITING
#define CONFIG_VERSION_VARIABLE
#define CONFIG_TIMESTAMP

/* U-Boot memory configuration */
#define CONFIG_STACKSIZE		(512 << 10)	/* 512 KiB */
#define CONFIG_SYS_MALLOC_LEN		(512 << 10)	/* 512 KiB */
#define CONFIG_SYS_MEMTEST_START	0x87000000	/* physical address */
#define CONFIG_SYS_MEMTEST_END		0x88000000	/* test 16MB RAM */

/* Linux interfacing */
#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_SYS_BARGSIZE	1024			/* bootarg Size */
#define CONFIG_SYS_LOAD_ADDR	0x88000000		/* kernel load address */
#define LINUX_BOOT_PARAM_ADDR	(0x80000000 + 0x100)

/* NAND configuration issocketed with two chipselects just like the DM355 EVM.
 * It normally comes with a 2GByte SLC part with 2KB pages
 * (and 128KB erase blocks); other
 * 2GByte parts may have 4KB pages, 256KB erase blocks, and use MLC.  (MLC
 * pretty much demands the 4-bit ECC support.)  You can of course swap in
 * other parts, including small page ones.
 */
#define MTDIDS_DEFAULT		"nand0=davinci_nand.0"

#ifdef CONFIG_SYS_NAND_LARGEPAGE
/*  Use same layout for 128K/256K blocks; allow some bad blocks */
#define PART_BOOT		"1024k(bootloader)ro,"
#else
/* Assume 16K erase blocks; allow a few bad ones. */
#define PART_BOOT		"512k(bootloader)ro,"
#endif

#define PART_PARAMS     "512k(params)ro,"
#define PART_KERNEL		"4352k(kernel),"	/* kernel + initramfs */
#define PART_REST		"204928k(filesystem)"
#define PART_RESERVED	"-(reserved)"

#define MTDPARTS_DEFAULT	\
	"mtdparts=davinci_nand.0:" PART_BOOT PART_PARAMS PART_KERNEL PART_REST PART_RESERVED


#define CONFIG_MAX_RAM_BANK_SIZE	(128 << 20)	/* 128 MB */

#define CONFIG_SYS_SDRAM_BASE		0x80000000
#define CONFIG_SYS_INIT_SP_ADDR		\
	(CONFIG_SYS_TEXT_BASE - GENERATED_GBL_DATA_SIZE)

#define CONFIG_SYS_DCACHE_OFF

#endif /* __CONFIG_H */
