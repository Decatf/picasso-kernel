/*
 * arch/arm/mach-tegra/mc.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2011 NVIDIA Corporation
 *
 * Author:
 *	Erik Gilling <konkers@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/tegra_mc.h>

#include "iomap.h"

static DEFINE_SPINLOCK(tegra_mc_lock);

void tegra_mc_set_priority(unsigned long client, unsigned long prio)
{
	void __iomem *mc_base = IO_ADDRESS(TEGRA_MC_BASE);
	unsigned long reg = client >> 8;
	int field = client & 0xff;
	unsigned long val;
	unsigned long flags;

	spin_lock_irqsave(&tegra_mc_lock, flags);
	val = readl(mc_base + reg);
	val &= ~(TEGRA_MC_PRIO_MASK << field);
	val |= prio << field;
	writel(val, mc_base + reg);
	spin_unlock_irqrestore(&tegra_mc_lock, flags);

}

int tegra_mc_get_tiled_memory_bandwidth_multiplier(void)
{
	return 1;
}
