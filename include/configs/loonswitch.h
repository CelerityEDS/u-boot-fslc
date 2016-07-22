/*
 * (C) Copyright 2016 Google, Inc
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */


#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * 1024 * 1024)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_MISC_INIT_R
#define CONFIG_CI_UDC
#define CONFIG_USBD_HS
#define CONFIG_USB_GADGET_DUALSPEED
#define CONFIG_USB_ETHER
#define CONFIG_USB_ETH_CDC
#define CONFIG_NETCONSOLE

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE	       UART1_BASE

#define CONFIG_CMD_SF
#ifdef CONFIG_CMD_SF
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_SPI_FLASH_ATMEL
#define CONFIG_SPI_FLASH_MTD
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS  2
#define CONFIG_SF_DEFAULT_CS   0
#define CONFIG_SF_DEFAULT_SPEED 25000000
#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)
#endif

/* I2C Configs */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		100000

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR      0
#define CONFIG_SYS_FSL_USDHC_NUM       2

#ifdef CONFIG_MX6Q
#define CONFIG_CMD_SATA
#endif

/* PCIe Configs */
/*
#define CONFIG_CMD_PCI
#ifdef CONFIG_CMD_PCI
#define CONFIG_PCI
#define CONFIG_PCI_PNP
#define CONFIG_PCI_SCAN_SHOW
#define CONFIG_PCIE_IMX
#define CONFIG_PCIE_IMX_PERST_GPIO    IMX_GPIO_NR(7, 12)
#endif
 */

/*
 * SATA Configs
 */
#ifdef CONFIG_CMD_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA
#endif

#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		6
#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#define CONFIG_PHY_MICREL_KSZ9021

/* USB Configs */
#define CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_USB_ETHER_MCS7830
#define CONFIG_USB_ETHER_SMSC95XX
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET	/* For OTG port */
#define CONFIG_MXC_USB_PORTSC	(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS	0
#define CONFIG_USB_KEYBOARD
#define CONFIG_SYS_USB_EVENT_POLL_VIA_CONTROL_EP

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX	       1
#define CONFIG_BAUDRATE			       115200

/* Miscellaneous commands */
#define CONFIG_CMD_BMODE

#define CONFIG_PREBOOT                 "setenv stdout serial; setenv stdin serial; echo Google, Inc. 2016"

#ifdef CONFIG_CMD_SATA
#define CONFIG_DRIVE_SATA "sata "
#else
#define CONFIG_DRIVE_SATA
#endif

#ifdef CONFIG_CMD_MMC
#define CONFIG_DRIVE_MMC "mmc "
#else
#define CONFIG_DRIVE_MMC
#endif

#ifdef CONFIG_USB_STORAGE
#define CONFIG_DRIVE_USB "usb "
#else
#define CONFIG_DRIVE_USB
#endif

#define CONFIG_DRIVE_TYPES CONFIG_DRIVE_SATA CONFIG_DRIVE_MMC CONFIG_DRIVE_USB
#define CONFIG_UMSDEVS CONFIG_DRIVE_SATA CONFIG_DRIVE_MMC

#define CONFIG_EXTRA_ENV_SETTINGS \
	"bootdevs=" CONFIG_DRIVE_TYPES "\0" \
	"umsdevs=" CONFIG_UMSDEVS "\0" \
	"bootdevice=emmc\0" \
	"bootcmd=" \
		"setenv fdt_high 0xffffffff ; " \
		"setenv fdt_addr 0x13000000 ; " \
		"setenv bootargs 'console=ttymxc0,115200n8 no_console_suspend " \
		"root=/dev/mmcblk0p2 rw rootwait' ; " \
		"saveenv ; " \
		"if test \"${bootdevice}\" = \"emmc\"; then " \
		"echo && echo Celerity iMX6COM -- Booting from eMMC Card... && " \
		"run bootfromemmc; " \
		"elif test \"${bootdevice}\" = \"usb\"; then " \
		"echo && echo Celerity iMX6COM -- Booting from USB... && " \
		"run bootfromusb; " \
		"fi\0" \
	"prepemmc=ums 0 mmc 2\0" \
	"prepsd=ums 0  mmc 1\0" \
	"updateflashfromusb=if fatload usb 0 0x10800000 u-boot.imx; then " \
		"sf probe && sf erase 0 0xc0000 && sf write 0x10800000 0x400 $filesize && " \
		"echo flash is now updated ; fi\0" \
	"updateflashfromemmc=if fatload mmc 2 0x10800000 u-boot.imx; then " \
		"sf probe && sf erase 0 0xc0000 && sf write 0x10800000 0x400 $filesize && " \
		"echo flash is now updated ; fi\0" \
	"clearenv=if sf probe || sf probe || sf probe 1 ; then " \
		"sf erase 0xc0000 0x2000 && " \
		"echo restored environment to factory default ; fi\0" \
	"bootfromsd=fatload mmc 0 0x10800000 uImage; " \
		"fatload mmc 0 0x12000000 imx6q-imx6com.dtb; " \
		"bootm 0x10800000 - 0x12000000\0" \
	"bootfromemmc=fatload mmc 2 0x10800000 uImage; " \
		"fatload mmc 2 0x12000000 imx6q-imx6com.dtb; " \
		"bootm 0x10800000 - 0x12000000\0" \
	"bootfromusb=fatload usb 0 0x10800000 uImage; " \
		"fatload usb 0 0x12000000 imx6q-imx6com.dtb; " \
		"bootm 0x10800000 - 0x12000000\0" \

/* Miscellaneous configurable options */
#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END	       0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS	       1
#define PHYS_SDRAM		       MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE	       PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Environment organization */
#define CONFIG_ENV_SIZE			(8 * 1024)

#define CONFIG_ENV_IS_IN_SPI_FLASH
#if defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET		(768 * 1024)
#define CONFIG_ENV_SECT_SIZE		(8 * 1024)
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#endif

#define CONFIG_CMD_TIME
#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_ALT_MEMTEST

/*
 * PCI express
 */
#ifdef CONFIG_CMD_PCI
#define CONFIG_PCI
#define CONFIG_PCI_PNP
#define CONFIG_PCI_SCAN_SHOW
#define CONFIG_PCIE_IMX
#endif

#define CONFIG_CMD_ELF

#define CONFIG_USB_GADGET
#define CONFIG_CMD_USB_MASS_STORAGE
#define CONFIG_USB_FUNCTION_MASS_STORAGE
#define CONFIG_USB_GADGET_DOWNLOAD
#define CONFIG_USB_GADGET_VBUS_DRAW	2

 /* Todo: Fill in appropriate values */
#define CONFIG_G_DNL_VENDOR_NUM 0x0fff
#define CONFIG_G_DNL_PRODUCT_NUM 0xffff
#define CONFIG_G_DNL_MANUFACTURER "Google"

#define CONFIG_CMD_FASTBOOT
#define CONFIG_ANDROID_BOOT_IMAGE
#define CONFIG_USB_FASTBOOT_BUF_ADDR   CONFIG_SYS_LOAD_ADDR
#define CONFIG_USB_FASTBOOT_BUF_SIZE   0x07000000

#endif	       /* __CONFIG_H */