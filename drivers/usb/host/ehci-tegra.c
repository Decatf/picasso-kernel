/*
 * EHCI-compliant USB host controller driver for NVIDIA Tegra SoCs
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2009 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/irq.h>
#include <linux/usb/otg.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>

#include <linux/usb/tegra_usb_phy.h>

#define TEGRA_USB_BASE			0xC5000000
#define TEGRA_USB2_BASE			0xC5004000
#define TEGRA_USB3_BASE			0xC5008000

#define TEGRA_USB_USBMODE_REG_OFFSET		0x1a8

#define TEGRA_USB_DMA_ALIGN 32

#define STS_SRI	(1<<7)	/*	SOF Recieved	*/
#define PORT_LS_SE0	(0 << 10)
#define PORT_LS_K	(1 << 10)
#define PORT_LS_J	(2 << 10)
#define PORT_FS	(0 << 26)
#define PORT_LS	(1 << 26)
#define PORT_HS	(2 << 26)


struct tegra_ehci_hcd {
	struct ehci_hcd *ehci;
	struct tegra_usb_phy *phy;
	struct clk *clk;
	struct clk *emc_clk;
	struct usb_phy *transceiver;
	int host_resumed;
	int bus_suspended;
	int port_resuming;
	int power_down_on_bus_suspend;
	enum tegra_usb_phy_port_speed port_speed;
};

static void tegra_ehci_power_up(struct usb_hcd *hcd)
{
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);


	clk_prepare_enable(tegra->emc_clk);
	clk_prepare_enable(tegra->clk);

	usb_phy_set_suspend(&tegra->phy->u_phy, 0);
	tegra->host_resumed = 1;
}

static void tegra_ehci_power_down(struct usb_hcd *hcd)
{
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);

	tegra->host_resumed = 0;
	usb_phy_set_suspend(&tegra->phy->u_phy, 1);
	clk_disable_unprepare(tegra->clk);
	clk_disable_unprepare(tegra->emc_clk);
}

static int tegra_ehci_internal_port_reset(
	struct ehci_hcd	*ehci,
	u32 __iomem	*portsc_reg
)
{
	u32		temp;
	unsigned long	flags;
	int		retval = 0;
	int		i, tries;
	u32		saved_usbintr;

	spin_lock_irqsave(&ehci->lock, flags);
	saved_usbintr = ehci_readl(ehci, &ehci->regs->intr_enable);
	/* disable USB interrupt */
	ehci_writel(ehci, 0, &ehci->regs->intr_enable);
	spin_unlock_irqrestore(&ehci->lock, flags);

	/*
	 * Here we have to do Port Reset at most twice for
	 * Port Enable bit to be set.
	 */
	for (i = 0; i < 2; i++) {
		temp = ehci_readl(ehci, portsc_reg);
		temp |= PORT_RESET;
		ehci_writel(ehci, temp, portsc_reg);
		mdelay(10);
		temp &= ~PORT_RESET;
		ehci_writel(ehci, temp, portsc_reg);
		mdelay(1);
		tries = 100;
		do {
			mdelay(1);
			/*
			 * Up to this point, Port Enable bit is
			 * expected to be set after 2 ms waiting.
			 * USB1 usually takes extra 45 ms, for safety,
			 * we take 100 ms as timeout.
			 */
			temp = ehci_readl(ehci, portsc_reg);
		} while (!(temp & PORT_PE) && tries--);
		if (temp & PORT_PE)
			break;
	}
	if (i == 2)
		retval = -ETIMEDOUT;

	/*
	 * Clear Connect Status Change bit if it's set.
	 * We can't clear PORT_PEC. It will also cause PORT_PE to be cleared.
	 */
	if (temp & PORT_CSC)
		ehci_writel(ehci, PORT_CSC, portsc_reg);

	/*
	 * Write to clear any interrupt status bits that might be set
	 * during port reset.
	 */
	temp = ehci_readl(ehci, &ehci->regs->status);
	ehci_writel(ehci, temp, &ehci->regs->status);

	/* restore original interrupt enable bits */
	ehci_writel(ehci, saved_usbintr, &ehci->regs->intr_enable);
	return retval;
}

static int tegra_ehci_hub_control(
	struct usb_hcd	*hcd,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char		*buf,
	u16		wLength
)
{
	struct ehci_hcd	*ehci = hcd_to_ehci(hcd);
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);
	u32 __iomem	*status_reg;
	u32		temp;
	unsigned long	flags;
	int		retval = 0;
	u32		usbcmd;
	u32		usbsts_reg;


	status_reg = &ehci->regs->port_status[(wIndex & 0xff) - 1];

	spin_lock_irqsave(&ehci->lock, flags);
	/*
	 * In ehci_hub_control() for USB_PORT_FEAT_ENABLE clears the other bits
	 * that are write on clear, by writing back the register read value, so
	 * USB_PORT_FEAT_ENABLE is handled by masking the set on clear bits
	 */
	if (typeReq == ClearPortFeature && wValue == USB_PORT_FEAT_ENABLE) {
		temp = ehci_readl(ehci, status_reg);
		ehci_writel(ehci, (temp & ~PORT_RWC_BITS) & ~PORT_PE, status_reg);
		goto done;
	}

	else if (typeReq == GetPortStatus) {
		temp = ehci_readl(ehci, status_reg);
		if (tegra->port_resuming && !(temp & PORT_SUSPEND)) {
			/* Resume completed, re-enable disconnect detection */
			tegra->port_resuming = 0;
			tegra_usb_phy_postresume(tegra->phy);
			if (tegra->phy->instance == 1) {
				if (((temp & (3 << 10)) == PORT_LS_J) && !(temp & PORT_PE)) {
					pr_err("%s: sig = j, resume failed\n", __func__);

					retval = -EPIPE;
					goto done;
				}
				else if (((temp & (3 << 10)) == PORT_LS_J) && (temp & PORT_PE)) {
					udelay(5);
					temp = ehci_readl(ehci, status_reg);
					dbg_port (ehci, "BusGetPortStatus", 0, temp);
				}
			}

		}
	}

	else if (typeReq == SetPortFeature && wValue == USB_PORT_FEAT_SUSPEND) {
		temp = ehci_readl(ehci, status_reg);
		if ((temp & PORT_PE) == 0 || (temp & PORT_RESET) != 0) {
			retval = -EPIPE;
			goto done;
		}

		temp &= ~(PORT_RWC_BITS | PORT_WKCONN_E);
		temp |= PORT_WKDISC_E | PORT_WKOC_E;
		ehci_writel(ehci, temp | PORT_SUSPEND, status_reg);

		/*
		 * If a transaction is in progress, there may be a delay in
		 * suspending the port. Poll until the port is suspended.
		 */

		/* Need a 4ms delay before the controller goes to suspend*/
		mdelay(4);

		if (handshake(ehci, status_reg, PORT_SUSPEND,
						PORT_SUSPEND, 5000))
			pr_err("%s: timeout waiting for SUSPEND\n", __func__);
		set_bit((wIndex & 0xff) - 1, &ehci->suspended_ports);
		goto done;
	}

	/* For USB1 port we need to issue Port Reset twice internally */
	if (tegra->phy->instance == 0 &&
	   (typeReq == SetPortFeature && wValue == USB_PORT_FEAT_RESET)) {
		spin_unlock_irqrestore(&ehci->lock, flags);
		return tegra_ehci_internal_port_reset(ehci, status_reg);
	}

	/*
	 * Tegra host controller will time the resume operation to clear the bit
	 * when the port control state switches to HS or FS Idle. This behavior
	 * is different from EHCI where the host controller driver is required
	 * to set this bit to a zero after the resume duration is timed in the
	 * driver.
	 */
	else if (typeReq == ClearPortFeature &&
					wValue == USB_PORT_FEAT_SUSPEND) {
		temp = ehci_readl(ehci, status_reg);
		if ((temp & PORT_RESET) || !(temp & PORT_PE)) {
			retval = -EPIPE;
			goto done;
		}

		if (!(temp & PORT_SUSPEND))
			goto done;

		if (temp & PORT_RESUME) {
			usbcmd = ehci_readl(ehci, &ehci->regs->command);
			usbcmd &= ~CMD_RUN;
			ehci_writel(ehci, usbcmd, &ehci->regs->command);

			/* detect remote wakeup */
			ehci_dbg(ehci, "%s: usb-inst %d remote wakeup\n", __func__, tegra->phy->instance);
			spin_unlock_irq(&ehci->lock);
			msleep(20);
			spin_lock_irq(&ehci->lock);

			/* Poll until the controller clears RESUME and SUSPEND */
			if (handshake(ehci, status_reg, PORT_RESUME, 0, 2000))
				pr_err("%s: usb-inst %d timeout waiting for RESUME\n", __func__, tegra->phy->instance);
			if (handshake(ehci, status_reg, PORT_SUSPEND, 0, 2000))
				pr_err("%s: usb-inst %d timeout waiting for SUSPEND\n", __func__, tegra->phy->instance);

			/* Since we skip remote wakeup event, put controller in suspend again and resume port later */
			temp = ehci_readl(ehci, status_reg);
			temp |= PORT_SUSPEND;
			ehci_writel(ehci, temp, status_reg);
			mdelay(4);
			/* Wait until port suspend completes */
			if (handshake(ehci, status_reg, PORT_SUSPEND,
							 PORT_SUSPEND, 1000))
				pr_err("%s: usb-inst %d timeout waiting for PORT_SUSPEND\n",
								__func__, tegra->phy->instance);
		}

		tegra->port_resuming = 1;

		/* Disable disconnect detection during port resume */
		tegra_usb_phy_preresume(tegra->phy);

		usbsts_reg = ehci_readl(ehci, &ehci->regs->status);
		ehci_writel(ehci, usbsts_reg, &ehci->regs->status);
		usbsts_reg = ehci_readl(ehci, &ehci->regs->status);

		spin_unlock_irqrestore(&ehci->lock, flags);
		msleep(20);
		spin_lock_irqsave(&ehci->lock, flags);

		if (handshake(ehci, &ehci->regs->status, STS_SRI, STS_SRI, 2000))
			pr_err("%s: usb-inst %d timeout set for STS_SRI\n", __func__, tegra->phy->instance);

		usbsts_reg = ehci_readl(ehci, &ehci->regs->status);
		ehci_writel(ehci, usbsts_reg, &ehci->regs->status);

		if (handshake(ehci, &ehci->regs->status, STS_SRI, 0, 2000))
			pr_err("%s: usb-inst %d timeout clear STS_SRI\n", __func__, tegra->phy->instance);

		if (handshake(ehci, &ehci->regs->status, STS_SRI, STS_SRI, 2000))
			pr_err("%s: usb-inst %d timeout set STS_SRI\n", __func__, tegra->phy->instance);

		spin_unlock_irqrestore(&ehci->lock, flags);
		msleep(20);
		spin_lock_irqsave(&ehci->lock, flags);



		usbcmd = ehci_readl(ehci, &ehci->regs->command);
		usbcmd |= CMD_RUN;
		ehci_writel(ehci, usbcmd, &ehci->regs->command);

		ehci->reset_done[wIndex-1] = jiffies + msecs_to_jiffies(25);

		temp &= ~(PORT_RWC_BITS | PORT_WAKE_BITS);
		/* start resume signalling */
		ehci_writel(ehci, temp | PORT_RESUME, status_reg);

		set_bit(wIndex-1, &ehci->resuming_ports);

		spin_unlock_irqrestore(&ehci->lock, flags);
		msleep(20);
		spin_lock_irqsave(&ehci->lock, flags);

		/* Poll until the controller clears RESUME and SUSPEND */
		if (handshake(ehci, status_reg, PORT_RESUME, 0, 2000))
			pr_err("%s: timeout waiting for RESUME\n", __func__);
		if (handshake(ehci, status_reg, PORT_SUSPEND, 0, 2000))
			pr_err("%s: timeout waiting for SUSPEND\n", __func__);

		ehci->reset_done[wIndex-1] = 0;
		clear_bit(wIndex-1, &ehci->resuming_ports);

		goto done;
	}

	spin_unlock_irqrestore(&ehci->lock, flags);

	/* Handle the hub control events here */
	return ehci_hub_control(hcd, typeReq, wValue, wIndex, buf, wLength);
done:
	spin_unlock_irqrestore(&ehci->lock, flags);
	return retval;
}

static void tegra_ehci_restart(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);

	ehci_reset(ehci);

	/* setup the frame list and Async q heads */
	ehci_writel(ehci, ehci->periodic_dma, &ehci->regs->frame_list);
	ehci_writel(ehci, (u32)ehci->async->qh_dma, &ehci->regs->async_next);
	/* setup the command register and set the controller in RUN mode */
	ehci->command &= ~(CMD_LRESET|CMD_IAAD|CMD_PSE|CMD_ASE|CMD_RESET);
	ehci->command |= CMD_RUN;
	ehci_writel(ehci, ehci->command, &ehci->regs->command);

	down_write(&ehci_cf_port_reset_rwsem);
	ehci_writel(ehci, FLAG_CF, &ehci->regs->configured_flag);
	/* flush posted writes */
	ehci_readl(ehci, &ehci->regs->command);
	up_write(&ehci_cf_port_reset_rwsem);
}

static void tegra_ehci_shutdown(struct usb_hcd *hcd)
{
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);

	/* ehci_shutdown touches the USB controller registers, make sure
	 * controller has clocks to it */
	if (!tegra->host_resumed)
		tegra_ehci_power_up(hcd);

	ehci_shutdown(hcd);
}

static int tegra_ehci_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);

	/* EHCI registers start at offset 0x100 */
	ehci->caps = hcd->regs + 0x100;

	/* switch to host mode */
	hcd->has_tt = 1;

	return ehci_setup(hcd);
}

#ifdef CONFIG_PM
static int tegra_bus_suspend (struct usb_hcd *hcd)
{
	struct ehci_hcd		*ehci = hcd_to_ehci (hcd);
	int			port;
	int			mask;
	int			changed;

	ehci_dbg(ehci, "suspend root hub\n");

	if (time_before (jiffies, ehci->next_statechange))
		msleep(5);
	ehci_quiesce (ehci);

	spin_lock_irq(&ehci->lock);
	if (ehci->rh_state < EHCI_RH_RUNNING)
		goto done;

	/* Once the controller is stopped, port resumes that are already
	 * in progress won't complete.  Hence if remote wakeup is enabled
	 * for the root hub and any ports are in the middle of a resume or
	 * remote wakeup, we must fail the suspend.
	 */
	if (hcd->self.root_hub->do_remote_wakeup) {
		if (ehci->resuming_ports) {
			spin_unlock_irq(&ehci->lock);
			ehci_dbg(ehci, "suspend failed because a port is resuming\n");
			return -EBUSY;
		}
	}


	/* Unlike other USB host controller types, EHCI doesn't have
	 * any notion of "global" or bus-wide suspend.  The driver has
	 * to manually suspend all the active unsuspended ports, and
	 * then manually resume them in the bus_resume() routine.
	 */
	ehci->bus_suspended = 0;
	ehci->owned_ports = 0;
	changed = 0;
	port = HCS_N_PORTS(ehci->hcs_params);
	while (port--) {
		u32 __iomem	*reg = &ehci->regs->port_status [port];
		u32		t1 = ehci_readl(ehci, reg) & ~PORT_RWC_BITS;
		u32		t2 = t1 & ~PORT_WAKE_BITS;

		/* keep track of which ports we suspend */
		if (t1 & PORT_OWNER)
			set_bit(port, &ehci->owned_ports);
		else if ((t1 & PORT_PE) && !(t1 & PORT_SUSPEND)) {
			t2 |= PORT_SUSPEND;
			set_bit(port, &ehci->bus_suspended);
		}

		/* enable remote wakeup on all ports, if told to do so */
		if (hcd->self.root_hub->do_remote_wakeup) {
			/* only enable appropriate wake bits, otherwise the
			 * hardware can not go phy low power mode. If a race
			 * condition happens here(connection change during bits
			 * set), the port change detection will finally fix it.
			 */
			if (t1 & PORT_CONNECT)
				t2 |= PORT_WKOC_E | PORT_WKDISC_E;
			else
				t2 |= PORT_WKOC_E | PORT_WKCONN_E;
		}

		if (t1 != t2) {
			ehci_vdbg (ehci, "port %d, %08x -> %08x\n",
				port + 1, t1, t2);
			ehci_writel(ehci, t2, reg);
			changed = 1;
		}
	}

	if (changed && ehci->has_hostpc) {
		spin_unlock_irq(&ehci->lock);
		msleep(5);	/* 5 ms for HCD to enter low-power mode */
		spin_lock_irq(&ehci->lock);

		port = HCS_N_PORTS(ehci->hcs_params);
		while (port--) {
			u32 __iomem	*hostpc_reg = &ehci->regs->hostpc[port];
			u32		t3;

			t3 = ehci_readl(ehci, hostpc_reg);
			ehci_writel(ehci, t3 | HOSTPC_PHCD, hostpc_reg);
			t3 = ehci_readl(ehci, hostpc_reg);
			ehci_dbg(ehci, "Port %d phy low-power mode %s\n",
					port, (t3 & HOSTPC_PHCD) ?
					"succeeded" : "failed");
		}
	}

	spin_unlock_irq(&ehci->lock);

	/* Apparently some devices need a >= 1-uframe delay here */
	if (ehci->bus_suspended)
		udelay(150);

	/* turn off now-idle HC */
	ehci_halt (ehci);

	spin_lock_irq(&ehci->lock);
	if (ehci->enabled_hrtimer_events & BIT(EHCI_HRTIMER_POLL_DEAD))
		ehci_handle_controller_death(ehci);
	if (ehci->rh_state != EHCI_RH_RUNNING)
		goto done;
	ehci->rh_state = EHCI_RH_SUSPENDED;

		end_unlink_async(ehci);
		unlink_empty_async_suspended(ehci);
		ehci_handle_intr_unlinks(ehci);
		end_free_itds(ehci);

	/* allow remote wakeup */
	mask = INTR_MASK;
	if (!hcd->self.root_hub->do_remote_wakeup)
		mask &= ~STS_PCD;
	ehci_writel(ehci, mask, &ehci->regs->intr_enable);
	ehci_readl(ehci, &ehci->regs->intr_enable);

done:
	ehci->next_statechange = jiffies + msecs_to_jiffies(10);


	/* ehci_work() may have re-enabled the watchdog timer, which we do not
	 * want, and so we must delete any pending watchdog timer events.
	 */

	ehci->enabled_hrtimer_events = 0;
	ehci->next_hrtimer_event = EHCI_HRTIMER_NO_EVENT;
	spin_unlock_irq(&ehci->lock);

	hrtimer_cancel(&ehci->hrtimer);

	return 0;
}

/* caller has locked the root hub, and should reset/reinit on error */
static int tegra_bus_resume (struct usb_hcd *hcd)
{
	struct ehci_hcd		*ehci = hcd_to_ehci (hcd);
	struct			tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);
	u32			temp, usbsts_reg;
	u32			power_okay;

	u32 __iomem		*status_reg;
	u32			usbcmd_bef_rsm, usbsts_bef_rsm, usbmode_bef_rsm, portsc_bef_rsm;
	u32			usbcmd_in_rsm, usbsts_in_rsm, usbmode_in_rsm, portsc_in_rsm;

	status_reg = &ehci->regs->port_status[0];

	if (time_before (jiffies, ehci->next_statechange))
		msleep(5);

	spin_lock_irq (&ehci->lock);
	if (!HCD_HW_ACCESSIBLE(hcd) || ehci->shutdown)
			goto shutdown;

	if (unlikely(ehci->debug)) {
		if (!dbgp_reset_prep(hcd))
			ehci->debug = NULL;
		else
			dbgp_external_startup(hcd);
	}

	/* Ideally and we've got a real resume here, and no port's power
	 * was lost.  (For PCI, that means Vaux was maintained.)  But we
	 * could instead be restoring a swsusp snapshot -- so that BIOS was
	 * the last user of the controller, not reset/pm hardware keeping
	 * state we gave to it.
	 */
	power_okay = ehci_readl(ehci, &ehci->regs->intr_enable);
	ehci_dbg(ehci, "resume root hub%s\n",
			power_okay ? "" : " after power loss");

	/* at least some APM implementations will try to deliver
	 * IRQs right away, so delay them until we're ready.
	 */
	ehci_writel(ehci, 0, &ehci->regs->intr_enable);

	/* re-init operational registers */
	ehci_writel(ehci, 0, &ehci->regs->segment);
	ehci_writel(ehci, ehci->periodic_dma, &ehci->regs->frame_list);
	ehci_writel(ehci, (u32) ehci->async->qh_dma, &ehci->regs->async_next);
#if 0
	/* restore CMD_RUN, framelist size, and irq threshold */
	ehci_writel(ehci, ehci->command, &ehci->regs->command);
#endif

	/* wait for SOF edge before enable RUN bit*/
	ehci_dbg(ehci, "%s:usb-inst %d USBSTS = 0x%x", __func__, tegra->phy->instance,
	ehci_readl(ehci, &ehci->regs->status));
	usbsts_reg = ehci_readl(ehci, &ehci->regs->status);
	ehci_writel(ehci, usbsts_reg, &ehci->regs->status);
	usbsts_reg = ehci_readl(ehci, &ehci->regs->status);
	udelay(20);

	if (handshake(ehci, &ehci->regs->status, STS_SRI, STS_SRI, 2000))
		pr_err("%s: timeout set for STS_SRI\n", __func__);

	usbsts_reg = ehci_readl(ehci, &ehci->regs->status);
	ehci_writel(ehci, usbsts_reg, &ehci->regs->status);

	if (handshake(ehci, &ehci->regs->status, STS_SRI, 0, 2000))
		pr_err("%s: timeout clear STS_SRI\n", __func__);

	if (handshake(ehci, &ehci->regs->status, STS_SRI, STS_SRI, 2000))
		pr_err("%s: timeout set STS_SRI\n", __func__);

	udelay(20);

	usbsts_reg = ehci_readl(ehci, &ehci->regs->status);
	if (usbsts_reg & STS_PCD)
		ehci_dbg(ehci, "%s: usb-inst %d remote wakeup PCD\n", __func__, tegra->phy->instance);

	/* resume suspended port */
	temp = ehci_readl(ehci, status_reg);
	if ((usbsts_reg & STS_PCD) ||
		((temp & PORT_POWER) && (temp & PORT_PE) && (temp & PORT_RESUME))) {
		/* detect remote wakeup */

		ehci->command &= ~CMD_RUN;
		ehci_writel(ehci, ehci->command, &ehci->regs->command);

		ehci_dbg(ehci, "%s: usb-inst %d remote wakeup+\n", __func__, tegra->phy->instance);
		dbg_port (ehci, "BusGetPortStatus", 0, temp);
		spin_unlock_irq(&ehci->lock);
		msleep(20);
		spin_lock_irq(&ehci->lock);

		/* Poll until the controller clears RESUME and SUSPEND */
		if (handshake(ehci, status_reg, PORT_RESUME, 0, 2000))
			pr_err("%s: usb-inst %d timeout waiting for RESUME\n", __func__, tegra->phy->instance);
		if (handshake(ehci, status_reg, PORT_SUSPEND, 0, 2000))
			pr_err("%s: usb-inst %d timeout waiting for SUSPEND\n", __func__, tegra->phy->instance);

		temp = ehci_readl(ehci, status_reg);
		dbg_port (ehci, "BusGetPortStatus", 0, temp);
		ehci_dbg(ehci, "%s: usb-inst %d remote wakeup-\n", __func__, tegra->phy->instance);

		/* wait for SOF edge before enable RUN bit*/
		ehci_dbg(ehci, "%s:usb-inst %d USBSTS = 0x%x", __func__, tegra->phy->instance,
		ehci_readl(ehci, &ehci->regs->status));
		usbsts_reg = ehci_readl(ehci, &ehci->regs->status);
		ehci_writel(ehci, usbsts_reg, &ehci->regs->status);
		usbsts_reg = ehci_readl(ehci, &ehci->regs->status);
		udelay(20);

		if (handshake(ehci, &ehci->regs->status, STS_SRI, STS_SRI, 2000))
			pr_err("%s: timeout set for STS_SRI\n", __func__);

		usbsts_reg = ehci_readl(ehci, &ehci->regs->status);
		ehci_writel(ehci, usbsts_reg, &ehci->regs->status);

		if (handshake(ehci, &ehci->regs->status, STS_SRI, 0, 2000))
			pr_err("%s: timeout clear STS_SRI\n", __func__);

		if (handshake(ehci, &ehci->regs->status, STS_SRI, STS_SRI, 2000))
			pr_err("%s: timeout set STS_SRI\n", __func__);

		udelay(20);

		ehci_dbg(ehci, "%s: usb-inst %d after remote wakeup resuming_ports = %lu\n",
				__func__, tegra->phy->instance, ehci->resuming_ports);

		clear_bit(0, &ehci->suspended_ports);

		/* TRSMRCY = 10 msec */
		spin_unlock_irq(&ehci->lock);
		msleep(10);
		spin_lock_irq(&ehci->lock);
		temp = ehci_readl(ehci, status_reg);
		dbg_port (ehci, "BusGetPortStatus", 0, temp);
		ehci_dbg(ehci, "%s:USBSTS = 0x%x", __func__,
		ehci_readl(ehci, &ehci->regs->status));


	}

	/* restore CMD_RUN, framelist size, and irq threshold */
	ehci->command |= CMD_RUN;
	ehci_writel(ehci, ehci->command, &ehci->regs->command);
	ehci->rh_state = EHCI_RH_RUNNING;

	// Read the USBCMD, USBSTS, USBMODE, PORTSC register here, don't print it at this time.
	usbcmd_bef_rsm = ehci_readl(ehci, &ehci->regs->command);
	usbsts_bef_rsm = ehci_readl(ehci, &ehci->regs->status);
	usbmode_bef_rsm = readl(hcd->regs + TEGRA_USB_USBMODE_REG_OFFSET);
	portsc_bef_rsm = ehci_readl(ehci, status_reg);

	/* resume suspended port*/
	temp = ehci_readl(ehci, status_reg);
	if ((temp & PORT_POWER) && (temp & PORT_PE) && (temp & PORT_SUSPEND)) {
		temp &= ~(PORT_RWC_BITS | PORT_WAKE_BITS);
		temp |= PORT_RESUME;
		ehci_writel(ehci, temp, status_reg);

		// Read the USBCMD, USBSTS, USBMODE, PORTSC register here, don't print it at this time.
		usbcmd_in_rsm = ehci_readl(ehci, &ehci->regs->command);
		usbsts_in_rsm = ehci_readl(ehci, &ehci->regs->status);
		usbmode_in_rsm = readl(hcd->regs + TEGRA_USB_USBMODE_REG_OFFSET);
		portsc_in_rsm = ehci_readl(ehci, status_reg);

		ehci_dbg(ehci, "%s:resume port+\n", __func__);
		printk("%s: usb-inst %d reg before bus_resume USBCMD=%x, USBSTS=%x, USBMODE=%x, PORTSC=%x\n",
		__func__, tegra->phy->instance, usbcmd_bef_rsm, usbsts_bef_rsm, usbmode_bef_rsm, portsc_bef_rsm);
		printk("%s: usb-inst %d reg during bus_resume USBCMD=%x, USBSTS=%x, USBMODE=%x, PORTSC=%x\n",
                           __func__, tegra->phy->instance, usbcmd_in_rsm, usbsts_in_rsm, usbmode_in_rsm, portsc_in_rsm);

		spin_unlock_irq(&ehci->lock);
		msleep(20);
		spin_lock_irq(&ehci->lock);

		temp = ehci_readl(ehci, status_reg);
		dbg_port (ehci, "BusGetPortStatus", 0, temp);

		if (ehci->shutdown)
			goto shutdown;

		/* Poll until the controller clears RESUME and SUSPEND */
		if (handshake(ehci, status_reg, PORT_RESUME, 0, 2000))
			pr_err("%s: timeout waiting for RESUME\n", __func__);
		if (handshake(ehci, status_reg, PORT_SUSPEND, 0, 2000))
			pr_err("%s: timeout waiting for SUSPEND\n", __func__);

		clear_bit(0, &ehci->suspended_ports);

		/* TRSMRCY = 10 msec */
		spin_unlock_irq(&ehci->lock);
		msleep(10);
		spin_lock_irq(&ehci->lock);
		temp = ehci_readl(ehci, status_reg);
		dbg_port (ehci, "BusGetPortStatus", 0, temp);
		ehci_dbg(ehci, "%s:USBSTS = 0x%x", __func__,
		ehci_readl(ehci, &ehci->regs->status));
		ehci_dbg(ehci, "%s:resume port-\n", __func__);
	}

	(void) ehci_readl(ehci, &ehci->regs->command);

	temp = ehci_readl(ehci, status_reg);
	dbg_port (ehci, "BusGetPortStatus", 0, temp);
	if (((temp & (3 << 10)) == PORT_LS_J) && !(temp & PORT_PE)) {
		ehci_dbg(ehci, "%s: failed to resume port. let usb_port_resume to resume port later\n", __func__);
		temp = ehci_readl(ehci, status_reg);
		temp |= PORT_SUSPEND;
		ehci_writel(ehci, temp, status_reg);
		mdelay(4);
		/* Wait until port suspend completes */
		if (handshake(ehci, status_reg, PORT_SUSPEND,
							 PORT_SUSPEND, 1000))
			pr_err("%s: timeout waiting for PORT_SUSPEND\n",
								__func__);
		set_bit(0, &ehci->suspended_ports);
		temp = ehci_readl(ehci, status_reg);
		dbg_port (ehci, "BusGetPortStatus", 0, temp);
	} else if (((temp & (3 << 10)) == PORT_LS_J) && (temp & PORT_PE)) {
		udelay(5);
		temp = ehci_readl(ehci, status_reg);
		dbg_port (ehci, "BusGetPortStatus", 0, temp);
	}

	ehci->next_statechange = jiffies + msecs_to_jiffies(5);
	spin_unlock_irq(&ehci->lock);

	ehci_handover_companion_ports(ehci);

	/* Now we can safely re-enable irqs */
	spin_lock_irq(&ehci->lock);
	if (ehci->shutdown)
		goto shutdown;

	/* Now we can safely re-enable irqs */
	ehci_writel(ehci, INTR_MASK, &ehci->regs->intr_enable);
	(void) ehci_readl(ehci, &ehci->regs->intr_enable);
	spin_unlock_irq (&ehci->lock);

	return 0;

shutdown:
	spin_unlock_irq(&ehci->lock);
	return -ESHUTDOWN;
}
#endif

struct dma_aligned_buffer {
	void *kmalloc_ptr;
	void *old_xfer_buffer;
	u8 data[0];
};

static void free_dma_aligned_buffer(struct urb *urb)
{
	struct dma_aligned_buffer *temp;

	if (!(urb->transfer_flags & URB_ALIGNED_TEMP_BUFFER))
		return;

	temp = container_of(urb->transfer_buffer,
		struct dma_aligned_buffer, data);

	if (usb_urb_dir_in(urb))
		memcpy(temp->old_xfer_buffer, temp->data,
		       urb->transfer_buffer_length);
	urb->transfer_buffer = temp->old_xfer_buffer;
	kfree(temp->kmalloc_ptr);

	urb->transfer_flags &= ~URB_ALIGNED_TEMP_BUFFER;
}

static int alloc_dma_aligned_buffer(struct urb *urb, gfp_t mem_flags)
{
	struct dma_aligned_buffer *temp, *kmalloc_ptr;
	size_t kmalloc_size;

	if (urb->num_sgs || urb->sg ||
	    urb->transfer_buffer_length == 0 ||
	    !((uintptr_t)urb->transfer_buffer & (TEGRA_USB_DMA_ALIGN - 1)))
		return 0;

	/* Allocate a buffer with enough padding for alignment */
	kmalloc_size = urb->transfer_buffer_length +
		sizeof(struct dma_aligned_buffer) + TEGRA_USB_DMA_ALIGN - 1;

	kmalloc_ptr = kmalloc(kmalloc_size, mem_flags);
	if (!kmalloc_ptr)
		return -ENOMEM;

	/* Position our struct dma_aligned_buffer such that data is aligned */
	temp = PTR_ALIGN(kmalloc_ptr + 1, TEGRA_USB_DMA_ALIGN) - 1;
	temp->kmalloc_ptr = kmalloc_ptr;
	temp->old_xfer_buffer = urb->transfer_buffer;
	if (usb_urb_dir_out(urb))
		memcpy(temp->data, urb->transfer_buffer,
		       urb->transfer_buffer_length);
	urb->transfer_buffer = temp->data;

	urb->transfer_flags |= URB_ALIGNED_TEMP_BUFFER;

	return 0;
}

static int tegra_ehci_map_urb_for_dma(struct usb_hcd *hcd, struct urb *urb,
				      gfp_t mem_flags)
{
	int ret;

	ret = alloc_dma_aligned_buffer(urb, mem_flags);
	if (ret)
		return ret;

	ret = usb_hcd_map_urb_for_dma(hcd, urb, mem_flags);
	if (ret)
		free_dma_aligned_buffer(urb);

	return ret;
}

static void tegra_ehci_unmap_urb_for_dma(struct usb_hcd *hcd, struct urb *urb)
{
	usb_hcd_unmap_urb_for_dma(hcd, urb);
	free_dma_aligned_buffer(urb);
}
#ifdef CONFIG_PM

static int controller_suspend(struct device *dev)
{
	struct tegra_ehci_hcd *tegra =
			platform_get_drvdata(to_platform_device(dev));
	struct ehci_hcd	*ehci = tegra->ehci;
	struct usb_hcd *hcd = ehci_to_hcd(ehci);
	struct ehci_regs __iomem *hw = ehci->regs;
	u32				temp, usbsts_reg;
	unsigned long flags;

	if (ehci->bus_suspended)
             return 0;
	if (hcd->self.root_hub->do_remote_wakeup) {
		if (ehci->resuming_ports) {
			ehci_dbg(ehci, "power down failed because a port is resuming\n");
			return -EBUSY;
		}
	}

	ehci_dbg(ehci, "%s:usb-inst %d USBSTS = 0x%x", __func__, tegra->phy->instance,
	ehci_readl(ehci, &ehci->regs->status));

	dbg_port (ehci, "BusGetPortStatus", 0, ehci_readl(ehci, &hw->port_status[0]));

	ehci_halt(ehci);

	spin_lock_irqsave(&ehci->lock, flags);
	tegra->port_speed = (readl(&hw->port_status[0]) >> 26) & 0x3;

	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	spin_unlock_irqrestore(&ehci->lock, flags);

	ehci_dbg(ehci, "%s:usb-inst %d USBSTS = 0x%x", __func__, tegra->phy->instance,
	ehci_readl(ehci, &ehci->regs->status));
	dbg_port (ehci, "BusGetPortStatus", 0, ehci_readl(ehci, &hw->port_status[0]));

	if (hcd->self.root_hub->do_remote_wakeup) {

		usbsts_reg = ehci_readl(ehci, &ehci->regs->status);
		if (usbsts_reg & STS_PCD)
			ehci_dbg(ehci, "%s: usb-inst %d remote wakeup PCD\n", __func__, tegra->phy->instance);


		temp = ehci_readl(ehci, &hw->port_status[0]);
		if ( temp & PORT_RESUME ) {
			set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
			ehci_dbg(ehci, "power down failed because a port is resuming (resume not set)\n");
			return -EBUSY;
		}
	}

	tegra_ehci_power_down(hcd);

	return 0;
}

static int controller_resume(struct device *dev)
{
	struct tegra_ehci_hcd *tegra =
			platform_get_drvdata(to_platform_device(dev));
	struct ehci_hcd	*ehci = tegra->ehci;
	struct usb_hcd *hcd = ehci_to_hcd(ehci);
	struct ehci_regs __iomem *hw = ehci->regs;
	unsigned long val;

	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	tegra_ehci_power_up(hcd);

	if (tegra->port_speed > TEGRA_USB_PHY_PORT_SPEED_HIGH) {
		/* Wait for the phy to detect new devices
		 * before we restart the controller */
		msleep(10);
		goto restart;
	}

	/* Force the phy to keep data lines in suspend state */
	tegra_ehci_phy_restore_start(tegra->phy, tegra->port_speed);

	/* Enable host mode */
	tdi_reset(ehci);

	/* Enable Port Power */
	val = readl(&hw->port_status[0]);
	val |= PORT_POWER;
	writel(val, &hw->port_status[0]);
	udelay(10);

	/* Check if the phy resume from LP0. When the phy resume from LP0
	 * USB register will be reset. */
	if (!readl(&hw->async_next)) {
		/* Program the field PTC based on the saved speed mode */
		val = readl(&hw->port_status[0]);
		val &= ~PORT_TEST(~0);
		if (tegra->port_speed == TEGRA_USB_PHY_PORT_SPEED_HIGH)
			val |= PORT_TEST_FORCE;
		else if (tegra->port_speed == TEGRA_USB_PHY_PORT_SPEED_FULL)
			val |= PORT_TEST(6);
		else if (tegra->port_speed == TEGRA_USB_PHY_PORT_SPEED_LOW)
			val |= PORT_TEST(7);
		writel(val, &hw->port_status[0]);
		udelay(10);

		/* Disable test mode by setting PTC field to NORMAL_OP */
		val = readl(&hw->port_status[0]);
		val &= ~PORT_TEST(~0);
		writel(val, &hw->port_status[0]);
		udelay(10);
	}

	/* Poll until CCS is enabled */
	if (handshake(ehci, &hw->port_status[0], PORT_CONNECT,
						 PORT_CONNECT, 2000)) {
		pr_err("%s: timeout waiting for PORT_CONNECT\n", __func__);
		goto restart;
	}

	/* Poll until PE is enabled */
	if (handshake(ehci, &hw->port_status[0], PORT_PE,
						 PORT_PE, 2000)) {
		pr_err("%s: timeout waiting for USB_PORTSC1_PE\n", __func__);
		goto restart;
	}

	/* Clear the PCI status, to avoid an interrupt taken upon resume */
	val = readl(&hw->status);
	val |= STS_PCD;
	writel(val, &hw->status);

	/* Put controller in suspend mode by writing 1 to SUSP bit of PORTSC */
	val = readl(&hw->port_status[0]);
	if ((val & PORT_POWER) && (val & PORT_PE)) {
		val |= PORT_SUSPEND;
		writel(val, &hw->port_status[0]);

		/* Wait until port suspend completes */
		if (handshake(ehci, &hw->port_status[0], PORT_SUSPEND,
							 PORT_SUSPEND, 5000)) {
			pr_err("%s: timeout waiting for PORT_SUSPEND\n",
								__func__);
			goto restart;
		}
	}

	tegra_ehci_phy_restore_end(tegra->phy);
	goto done;

 restart:

	if (tegra->port_speed <= TEGRA_USB_PHY_PORT_SPEED_HIGH)
		tegra_ehci_phy_restore_end(tegra->phy);

	tegra_ehci_restart(hcd);

 done:
	tegra_usb_phy_preresume(tegra->phy);
	tegra->port_resuming = 1;
	return 0;
}

static int tegra_ehci_suspend(struct device *dev)
{
	struct tegra_ehci_hcd *tegra =
			platform_get_drvdata(to_platform_device(dev));
	struct usb_hcd *hcd = ehci_to_hcd(tegra->ehci);
	int rc = 0;


	if ((tegra->bus_suspended) && (tegra->power_down_on_bus_suspend)) {
		return 0;
	}
	if (time_before(jiffies, tegra->ehci->next_statechange))
		msleep(10);


	/*
	 * When system sleep is supported and USB controller wakeup is
	 * implemented: If the controller is runtime-suspended and the
	 * wakeup setting needs to be changed, call pm_runtime_resume().
	 */
	if (HCD_HW_ACCESSIBLE(hcd))
		rc = controller_suspend(dev);
	return rc;
}

static int tegra_ehci_resume(struct device *dev)
{
	struct tegra_ehci_hcd *tegra =
			platform_get_drvdata(to_platform_device(dev));
	int rc;

	if ((tegra->bus_suspended) && (tegra->power_down_on_bus_suspend)) {
		return 0;
	}

		rc = controller_resume(dev);
		if (rc == 0) {
			pm_runtime_disable(dev);
			pm_runtime_set_active(dev);
			pm_runtime_enable(dev);
		}

	return rc;
}

static int tegra_ehci_runtime_suspend(struct device *dev)
{
	int rc = 0;
	struct tegra_ehci_hcd *tegra =
			platform_get_drvdata(to_platform_device(dev));
	struct usb_hcd *hcd = ehci_to_hcd(tegra->ehci);

/* FIXME: Don't work correctly. Need sync with system suspend */

	if ((tegra->bus_suspended) && (tegra->power_down_on_bus_suspend)) {
		return 0;
	}

	if (time_before(jiffies, tegra->ehci->next_statechange))
		msleep(10);


	if (HCD_HW_ACCESSIBLE(hcd))
		rc = controller_suspend(dev);

	return rc;
}

static int tegra_ehci_runtime_resume(struct device *dev)
{
	struct tegra_ehci_hcd *tegra =
			platform_get_drvdata(to_platform_device(dev));
	int rc;

	/* FIXME: Don't work correctly. Need sync with system suspend */

	if ((tegra->bus_suspended) && (tegra->power_down_on_bus_suspend)) {
		return 0;
	}

	rc = controller_suspend(dev);

	return rc;
}




static const struct dev_pm_ops tegra_ehci_pm_ops = {
	.suspend	= tegra_ehci_suspend,
	.resume		= tegra_ehci_resume,
	.runtime_suspend = tegra_ehci_runtime_suspend,
	.runtime_resume	= tegra_ehci_runtime_resume,
};

#endif

#ifdef CONFIG_PM
static int tegra_ehci_bus_suspend(struct usb_hcd *hcd)
{
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);
	int error_status = 0;

	if (!tegra->host_resumed)
		tegra_ehci_power_up(hcd);

	error_status = tegra_bus_suspend(hcd);
	if (!(error_status) && tegra->power_down_on_bus_suspend) {
		error_status = controller_suspend(hcd->self.controller);
		if (!error_status)
			tegra->bus_suspended = 1;
		else
			tegra_bus_resume(hcd);
	}
	return error_status;
}

static int tegra_ehci_bus_resume(struct usb_hcd *hcd)
{
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);
	int rc;

	if (tegra->bus_suspended && tegra->power_down_on_bus_suspend) {
		rc = controller_resume(hcd->self.controller);
		tegra->bus_suspended = 0;

		if (rc == 0) {
			pm_runtime_disable(hcd->self.controller);
			pm_runtime_set_active(hcd->self.controller);
			pm_runtime_enable(hcd->self.controller);
		}
	}

	return tegra_bus_resume(hcd);
}
#endif

static const struct hc_driver tegra_ehci_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "Tegra EHCI Host Controller",
	.hcd_priv_size		= sizeof(struct ehci_hcd),
	.flags			= HCD_USB2 | HCD_MEMORY,

	/* standard ehci functions */
	.irq			= ehci_irq,
	.start			= ehci_run,
	.stop			= ehci_stop,
	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.endpoint_reset		= ehci_endpoint_reset,
	.get_frame_number	= ehci_get_frame,
	.hub_status_data	= ehci_hub_status_data,
	.clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,
	.relinquish_port	= ehci_relinquish_port,
	.port_handed_over	= ehci_port_handed_over,

	/* modified ehci functions for tegra */
	.reset			= tegra_ehci_setup,
	.shutdown		= tegra_ehci_shutdown,
	.map_urb_for_dma	= tegra_ehci_map_urb_for_dma,
	.unmap_urb_for_dma	= tegra_ehci_unmap_urb_for_dma,
	.hub_control		= tegra_ehci_hub_control,
#ifdef CONFIG_PM
	.bus_suspend		= tegra_ehci_bus_suspend,
	.bus_resume		= tegra_ehci_bus_resume,
#endif
};

static int setup_vbus_gpio(struct platform_device *pdev,
			   struct tegra_ehci_platform_data *pdata)
{
	int err = 0;
	int gpio;

	gpio = pdata->vbus_gpio;
	if (!gpio_is_valid(gpio))
		gpio = of_get_named_gpio(pdev->dev.of_node,
					 "nvidia,vbus-gpio", 0);
	if (!gpio_is_valid(gpio))
		return 0;

	err = gpio_request(gpio, "vbus_gpio");
	if (err) {
		dev_err(&pdev->dev, "can't request vbus gpio %d", gpio);
		return err;
	}
	err = gpio_direction_output(gpio, 1);
	if (err) {
		dev_err(&pdev->dev, "can't enable vbus\n");
		return err;
	}

	return err;
}


static u64 tegra_ehci_dma_mask = DMA_BIT_MASK(32);

static int tegra_ehci_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct usb_hcd *hcd;
	struct tegra_ehci_hcd *tegra;
	struct tegra_ehci_platform_data *pdata;
	int err = 0;
	int irq;
	int instance = pdev->id;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "Platform data missing\n");
		return -EINVAL;
	}

	/* Right now device-tree probed devices don't get dma_mask set.
	 * Since shared usb code relies on it, set it here for now.
	 * Once we have dma capability bindings this can go away.
	 */
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &tegra_ehci_dma_mask;

	setup_vbus_gpio(pdev, pdata);

	tegra = devm_kzalloc(&pdev->dev, sizeof(struct tegra_ehci_hcd),
			     GFP_KERNEL);
	if (!tegra)
		return -ENOMEM;

	hcd = usb_create_hcd(&tegra_ehci_hc_driver, &pdev->dev,
					dev_name(&pdev->dev));
	if (!hcd) {
		dev_err(&pdev->dev, "Unable to create HCD\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, tegra);

	tegra->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(tegra->clk)) {
		dev_err(&pdev->dev, "Can't get ehci clock\n");
		err = PTR_ERR(tegra->clk);
		goto fail_clk;
	}

	err = clk_prepare_enable(tegra->clk);
	if (err)
		goto fail_clk;

	tegra->emc_clk = devm_clk_get(&pdev->dev, "emc");
	if (IS_ERR(tegra->emc_clk)) {
		dev_err(&pdev->dev, "Can't get emc clock\n");
		err = PTR_ERR(tegra->emc_clk);
		goto fail_emc_clk;
	}

	clk_prepare_enable(tegra->emc_clk);
	clk_set_rate(tegra->emc_clk, 400000000);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get I/O memory\n");
		err = -ENXIO;
		goto fail_io;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	hcd->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!hcd->regs) {
		dev_err(&pdev->dev, "Failed to remap I/O memory\n");
		err = -ENOMEM;
		goto fail_io;
	}

	/* This is pretty ugly and needs to be fixed when we do only
	 * device-tree probing. Old code relies on the platform_device
	 * numbering that we lack for device-tree-instantiated devices.
	 */
	if (instance < 0) {
		switch (res->start) {
		case TEGRA_USB_BASE:
			instance = 0;
			break;
		case TEGRA_USB2_BASE:
			instance = 1;
			break;
		case TEGRA_USB3_BASE:
			instance = 2;
			break;
		default:
			err = -ENODEV;
			dev_err(&pdev->dev, "unknown usb instance\n");
			goto fail_io;
		}
	}

	tegra->phy = tegra_usb_phy_open(&pdev->dev, instance, hcd->regs,
					pdata->phy_config,
					TEGRA_USB_PHY_MODE_HOST);
	if (IS_ERR(tegra->phy)) {
		dev_err(&pdev->dev, "Failed to open USB phy\n");
		err = -ENXIO;
		goto fail_io;
	}

	usb_phy_init(&tegra->phy->u_phy);

	err = usb_phy_set_suspend(&tegra->phy->u_phy, 0);
	if (err) {
		dev_err(&pdev->dev, "Failed to power on the phy\n");
		goto fail;
	}

	tegra->host_resumed = 1;
	tegra->ehci = hcd_to_ehci(hcd);
	tegra->power_down_on_bus_suspend = pdata->power_down_on_bus_suspend;

	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(&pdev->dev, "Failed to get IRQ\n");
		err = -ENODEV;
		goto fail;
	}

#ifdef CONFIG_USB_OTG_UTILS
	if (pdata->operating_mode == TEGRA_USB_OTG) {
		tegra->transceiver =
			devm_usb_get_phy(&pdev->dev, USB_PHY_TYPE_USB2);
		if (!IS_ERR_OR_NULL(tegra->transceiver))
			otg_set_host(tegra->transceiver->otg, &hcd->self);
	}
#endif

	err = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (err) {
		dev_err(&pdev->dev, "Failed to add USB HCD\n");
		goto fail;
	}

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);

	/* Don't skip the pm_runtime_forbid call if wakeup isn't working */
//	if (!pdata->power_down_on_bus_suspend)
		pm_runtime_forbid(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_put_sync(&pdev->dev);
	return err;

fail:
#ifdef CONFIG_USB_OTG_UTILS
	if (!IS_ERR_OR_NULL(tegra->transceiver))
		otg_set_host(tegra->transceiver->otg, NULL);
#endif
	usb_phy_shutdown(&tegra->phy->u_phy);
fail_io:
	clk_disable_unprepare(tegra->emc_clk);
fail_emc_clk:
	clk_disable_unprepare(tegra->clk);
fail_clk:
	usb_put_hcd(hcd);
	return err;
}

static int tegra_ehci_remove(struct platform_device *pdev)
{
	struct tegra_ehci_hcd *tegra = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = ehci_to_hcd(tegra->ehci);

	pm_runtime_get_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);

#ifdef CONFIG_USB_OTG_UTILS
	if (!IS_ERR_OR_NULL(tegra->transceiver))
		otg_set_host(tegra->transceiver->otg, NULL);
#endif

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);

	usb_phy_shutdown(&tegra->phy->u_phy);

	clk_disable_unprepare(tegra->clk);

	clk_disable_unprepare(tegra->emc_clk);

	return 0;
}

static void tegra_ehci_hcd_shutdown(struct platform_device *pdev)
{
	struct tegra_ehci_hcd *tegra = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = ehci_to_hcd(tegra->ehci);

	if (hcd->driver->shutdown)
		hcd->driver->shutdown(hcd);
}

static struct of_device_id tegra_ehci_of_match[] = {
	{ .compatible = "nvidia,tegra20-ehci", },
	{ },
};

static struct platform_driver tegra_ehci_driver = {
	.probe		= tegra_ehci_probe,
	.remove		= tegra_ehci_remove,
	.shutdown	= tegra_ehci_hcd_shutdown,
	.driver		= {
		.name	= "tegra-ehci",
		.of_match_table = tegra_ehci_of_match,
#ifdef CONFIG_PM
		.pm	= &tegra_ehci_pm_ops,
#endif
	}
};
