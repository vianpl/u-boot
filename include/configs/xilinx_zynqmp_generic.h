/*
 * Configuration for the Xilinx ZynqMP generic platform
 *
 * (C) Copyright 2018 Alexander Graf <agraf@suse.de>
 * (C) Copyright 2015 Xilinx, Inc.
 * Michal Simek <michal.simek@xilinx.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_ZYNQMP_GENERIC_H
#define __CONFIG_ZYNQMP_GENERIC_H

/* This file should disappear as soon as xhci is converted to DT enumerated */

#define CONFIG_ZYNQMP_XHCI_LIST {ZYNQMP_USB0_XHCI_BASEADDR, \
				 ZYNQMP_USB1_XHCI_BASEADDR}

#include <configs/xilinx_zynqmp.h>

#endif /* __CONFIG_ZYNQMP_GENERIC_H */
