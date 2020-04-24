/* SPDX-License-Identifier: GPL-2.0 */
/*
 * (C) Copyright 2019 Matthias Brugger
 */

#ifndef _BCM283x_BASE_H_
#define _BCM283x_BASE_H_

#include <linux/types.h>

extern unsigned long rpi_bcm283x_base;

#ifdef CONFIG_ARMV7_LPAE
extern void *rpi4_phys_to_virt(phys_addr_t paddr);
#define phys_to_virt(x) rpi4_phys_to_virt(x)
#endif

#endif
