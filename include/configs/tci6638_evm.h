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

/* Platform type */
#define CONFIG_SOC_TCI6638
#define CONFIG_TCI6638_EVM

#include <asm/arch/hardware.h>
#include <asm/arch/clock.h>

/* U-Boot Build Configuration */
#define CONFIG_SKIP_LOWLEVEL_INIT	/* U-Boot is a 2nd stage loader */
#define CONFIG_SYS_NO_FLASH		/* that is, no *NOR* flash */
#define CONFIG_SYS_CONSOLE_INFO_QUIET
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_SYS_THUMB_BUILD

/* SoC Configuration */
#define CONFIG_ARMV7
#define CONFIG_ARCH_CPU_INIT
#define CONFIG_SYS_ARCH_TIMER
#define CONFIG_SYS_HZ_CLOCK		clk_get_rate(TCI6638_CLK1_6)
#define CONFIG_SYS_HZ			1000
#define CONFIG_SYS_TEXT_BASE		0x0c001000
#define CONFIG_OF_LIBFDT		1
#define CONFIG_OF_BOARD_SETUP
#define CONFIG_SYS_DCACHE_OFF

/* Memory Configuration */
#define CONFIG_NR_DRAM_BANKS		1
#define CONFIG_SYS_SDRAM_BASE		0x80000000
#define CONFIG_MAX_RAM_BANK_SIZE	(2 << 30)	/* 2GB */
#define CONFIG_STACKSIZE		(512 << 10)	/* 512 KiB */
#define CONFIG_SYS_MALLOC_LEN		(512 << 10)	/* 512 KiB */
#define CONFIG_SYS_MEMTEST_START	CONFIG_SYS_SDRAM_BASE
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_SDRAM_BASE + 32 << 20)
#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_TEXT_BASE -	\
					 GENERATED_GBL_DATA_SIZE)

/* SPL SPI Loader Configuration */
#define CONFIG_SPL_TEXT_BASE		0x0c200000
#define CONFIG_SPL_PAD_TO		65536
#define CONFIG_SPL_MAX_SIZE		(CONFIG_SPL_PAD_TO - 8)
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
#define CONFIG_SYS_SPI_U_BOOT_OFFS	CONFIG_SPL_PAD_TO
#define CONFIG_SPL_FRAMEWORK

/* UART Configuration */
#define CONFIG_SYS_NS16550
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	-4
#define CONFIG_SYS_NS16550_COM1		TCI6638_UART0_BASE
#define CONFIG_SYS_NS16550_CLK		clk_get_rate(TCI6638_CLK1_6)
#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

/* SPI Configuration */
#define CONFIG_SPI
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_DAVINCI_SPI
#define CONFIG_SYS_SPI_BASE		TCI6638_SPI_BASE
#define CONFIG_SYS_SPI_CLK		clk_get_rate(TCI6638_LPSC_EMIF25_SPI)
#define CONFIG_SF_DEFAULT_SPEED		30000000
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED

/* I2C Configuration */
#define CONFIG_HARD_I2C
#define CONFIG_DRIVER_DAVINCI_I2C
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_SYS_I2C_SLAVE		0x10	/* SMBus host address */

/* EEPROM definitions */
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN		2
#define CONFIG_SYS_I2C_EEPROM_ADDR		0x50
#define CONFIG_SYS_EEPROM_PAGE_WRITE_BITS	6
#define CONFIG_SYS_EEPROM_PAGE_WRITE_DELAY_MS	20

/* Network Configuration */
#define CONFIG_SLAVE_PORT_NUM		0
#define CONFIG_DRIVER_TI_KEYSTONE_NET
#define CONFIG_EMAC_MDIO_PHY_NUM	CONFIG_SLAVE_PORT_NUM
#define CONFIG_MII
#define CONFIG_BOOTP_DEFAULT
#define CONFIG_BOOTP_DNS
#define CONFIG_BOOTP_DNS2
#define CONFIG_BOOTP_SEND_HOSTNAME
#define CONFIG_NET_RETRY_COUNT		32
#define CONFIG_NET_MULTI
#define RX_FLOW_NUM			(22 + CONFIG_SLAVE_PORT_NUM)
#define CONFIG_SYS_SGMII_INTERFACE	{ SGMII_LINK_MAC_PHY,	\
					  SGMII_LINK_MAC_PHY }
#define CONFIG_SYS_SGMII_REFCLK_MHZ	312
#define CONFIG_SYS_SGMII_LINERATE_MHZ	1250
#define	CONFIG_SYS_SGMII_RATESCALE	2

/* NAND Configuration */
#define CONFIG_NAND_DAVINCI
#define CONFIG_SYS_NAND_CS		2
#define CONFIG_SYS_NAND_USE_FLASH_BBT
#define CONFIG_SYS_NAND_4BIT_HW_ECC_OOBFIRST
#define CONFIG_SYS_NAND_PAGE_2K
#define CONFIG_SYS_NAND_LARGEPAGE
#define CONFIG_SYS_NAND_BASE_LIST	{ 0x30000000, }
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_MAX_CHIPS	1
#define CONFIG_SYS_NAND_NO_SUBPAGE_WRITE
#define CONFIG_ENV_SIZE			(256 << 10)	/* 256 KiB */
#define CONFIG_ENV_IS_IN_NAND
#define CONFIG_ENV_OFFSET		0x100000
#define CONFIG_MTD_PARTITIONS
#define CONFIG_MTD_DEVICE
#define CONFIG_RBTREE
#define CONFIG_LZO
#define MTDIDS_DEFAULT			"nand0=davinci_nand.0"
#define PART_BOOT			"1024k(bootloader)ro,"
#define PART_PARAMS			"512k(params)ro,"
#define PART_UBI			"-(ubi)"
#define MTDPARTS_DEFAULT		"mtdparts=davinci_nand.0:"	\
					PART_BOOT PART_PARAMS PART_UBI

/* USB Configuration */
#define CONFIG_USB_XHCI
#define CONFIG_USB_XHCI_KEYSTONE
#define CONFIG_SYS_USB_XHCI_MAX_ROOT_PORTS	2
#define CONFIG_USB_STORAGE
#define CONFIG_DOS_PARTITION
#define CONFIG_EFI_PARTITION
#define CONFIG_FS_FAT

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
#define CONFIG_CMD_MTDPARTS
#define CONFIG_CMD_NAND
#define CONFIG_CMD_UBI
#define CONFIG_CMD_UBIFS
#define CONFIG_CMD_NAND_ECCLAYOUT
#define CONFIG_CMD_USB
#define CONFIG_CMD_FAT
#define CONFIG_CMD_SF

/* U-Boot general configuration */
#define CONFIG_SYS_PROMPT		"TCI6638 EVM # "
#define CONFIG_SYS_CBSIZE		1024
#define CONFIG_SYS_PBSIZE		2048
#define CONFIG_SYS_MAXARGS		16
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#define CONFIG_SYS_LONGHELP
#define CONFIG_CRC32_VERIFY
#define CONFIG_MX_CYCLIC
#define CONFIG_CMDLINE_EDITING
#define CONFIG_VERSION_VARIABLE
#define CONFIG_TIMESTAMP

#define CONFIG_BOOTDELAY		3
#define CONFIG_BOOTFILE			"uImage"
#define CONFIG_EXTRA_ENV_SETTINGS					\
	"boot=net\0"							\
	"tftp_root=/\0"							\
	"nfs_root=/export\0"						\
	"mem_lpae=0\0"							\
	"mem_reserve=512M\0"						\
	"addr_fdt=0x87000000\0"						\
	"addr_kern=0x88000000\0"					\
	"addr_mon=0x0c5f0000\0"						\
	"addr_uboot=0x87000000\0"					\
	"fdt_high=0xffffffff\0"						\
	"name_fdt=uImage-keystone-evm.dtb\0"				\
	"name_kern=uImage-keystone-evm.bin\0"				\
	"name_mon=skern-keystone-evm.bin\0"				\
	"name_uboot=u-boot-spi-keystone-evm.gph\0"			\
	"run_mon=mon_install ${addr_mon}\0"				\
	"run_kern=bootm ${addr_kern} - ${addr_fdt}\0"			\
	"init_net=run args_all args_net\0"				\
	"init_ubi=run args_all args_ubi; "				\
		"ubi part ubifs; ubifsmount boot\0"			\
	"get_fdt_net=dhcp ${addr_fdt} ${tftp_root}/${name_fdt}\0"	\
	"get_fdt_ubi=ubifsload ${addr_fdt} ${name_fdt}\0"		\
	"get_kern_net=dhcp ${addr_kern} ${tftp_root}/${name_kern}\0"	\
	"get_kern_ubi=ubifsload ${addr_kern} ${name_kern}\0"		\
	"get_mon_net=dhcp ${addr_mon} ${tftp_root}/${name_mon}\0"	\
	"get_mon_ubi=ubifsload ${addr_mon} ${name_mon}\0"		\
	"get_uboot_net=dhcp ${addr_uboot} ${tftp_root}/${name_uboot}\0"	\
	"burn_uboot=sf probe; sf erase 0 0x100000; "			\
		"sf write ${addr_uboot} 0 ${filesize}\0"		\
	"args_all=setenv bootargs console=ttyS0,115200n8 rootwait=1\0"	\
	"args_ubi=setenv bootargs ${bootargs} rootfstype=ubifs "	\
		"root=ubi0:rootfs rootflags=sync rw ubi.mtd=2,2048\0"	\
	"args_net=setenv bootargs ${bootargs} rootfstype=nfs "		\
		"root=/dev/nfs rw nfsroot=${serverip}:${nfs_root},"	\
		"${nfs_options} ip=dhcp\0"				\
	"nfs_options=v3,tcp,rsize=4096,wsize=4096\0"
#define CONFIG_BOOTCOMMAND						\
	"run init_${boot} get_fdt_${boot} get_mon_${boot} "		\
		"get_kern_${boot} run_mon run_kern"
#define CONFIG_BOOTARGS							\

/* Linux interfacing */
#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_SYS_BARGSIZE		1024
#define CONFIG_SYS_LOAD_ADDR		(CONFIG_SYS_SDRAM_BASE + 0x08000000)
#define LINUX_BOOT_PARAM_ADDR		(CONFIG_SYS_SDRAM_BASE + 0x100)

#endif /* __CONFIG_H */
