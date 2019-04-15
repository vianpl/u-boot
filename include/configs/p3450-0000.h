/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2018-2019 NVIDIA Corporation. All rights reserved.
 */

#ifndef _P3450_0000_H
#define _P3450_0000_H

#include <linux/sizes.h>

#include "tegra210-common.h"

/* High-level configuration options */
#define CONFIG_TEGRA_BOARD_STRING	"NVIDIA P3450-0000"

/* Board-specific serial config */
#define CONFIG_TEGRA_ENABLE_UARTA

/* Only MMC1/PXE/DHCP for now, add USB back in later when supported */
#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 0) \
	func(PXE, pxe, na) \
	func(DHCP, dhcp, na)

/* SPI */
#define CONFIG_SPI_FLASH_SIZE		(4 << 20)

#include "tegra-common-usb-gadget.h"
#include "tegra-common-post.h"

/* Crystal is 38.4MHz. clk_m runs at half that rate */
#define COUNTER_FREQUENCY	19200000

#define BOARD_EXTRA_ENV_SETTINGS \
        "fdtfile=tegra210-p3450-0000.dtb\0"

#endif /* _P3450_0000_H */
