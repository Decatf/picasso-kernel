/*
 * driver/misc/sec_misc.c
 *
 * driver supporting miscellaneous functions for Samsung P3 device
 *
 * COPYRIGHT(C) Samsung Electronics Co., Ltd. 2006-2010 All Right Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/blkdev.h>
#include <linux/gpio.h>

#include "sec_misc.h"

typedef enum
{
	USB_SEL_AP_USB = 0,
	USB_SEL_CP_USB,
	USB_SEL_ADC
} usb_path_type;

#define GPIO_USB_SEL1 10
#define GPIO_USB_SEL2 113
#define GPIO_IFCONSENSE 118

usb_path_type usb_sel_status = USB_SEL_AP_USB;
EXPORT_SYMBOL(usb_sel_status);

void p3_set_usb_path(usb_path_type usb_path)
{
	if (usb_path == USB_SEL_AP_USB) {
		gpio_set_value(GPIO_USB_SEL1, 1);
		gpio_set_value(GPIO_USB_SEL2, 1);
		usb_sel_status = USB_SEL_AP_USB;
		}
	else if (usb_path == USB_SEL_CP_USB) {
		gpio_set_value(GPIO_USB_SEL1, 0);
		gpio_set_value(GPIO_USB_SEL2, 0);
		usb_sel_status = USB_SEL_CP_USB;
		}
	else if (usb_path == USB_SEL_ADC) {
		gpio_set_value(GPIO_USB_SEL1, 0);
		gpio_set_value(GPIO_USB_SEL2, 1);
		usb_sel_status = USB_SEL_ADC;
	}
}

static __init int p3_usb_path_init(void){
	int usbsel1, usbsel2;

	pr_info("%s\n", __func__);

	gpio_request(GPIO_USB_SEL1, "GPIO_USB_SEL1");
	gpio_direction_output(GPIO_USB_SEL1, 0);

	gpio_request(GPIO_USB_SEL2, "GPIO_USB_SEL2");
	gpio_direction_input(GPIO_USB_SEL2);
	usbsel2 = gpio_get_value(GPIO_USB_SEL2);
	gpio_direction_output(GPIO_USB_SEL2, 0);

	if (usbsel2 == 1) {
		p3_set_usb_path(USB_SEL_AP_USB);
	} else if (usbsel2 == 0) {
		p3_set_usb_path(USB_SEL_CP_USB);
	}

	return 0;
}
arch_initcall(p3_usb_path_init);

int check_usb_status = CHARGER_BATTERY;

int check_jig_on(void)
{
	u32 value = gpio_get_value(GPIO_IFCONSENSE);
	return !value;
}

static __init int init_jig_on(void)
{
	pr_info("%s\n", __func__);
	gpio_request(GPIO_IFCONSENSE, "GPIO_IFCONSENSE");
	gpio_direction_input(GPIO_IFCONSENSE);

	return 0;
}
arch_initcall(init_jig_on);

struct class *sec_class;
EXPORT_SYMBOL(sec_class);

static __init int sec_class_init(void)
{
	sec_class = class_create(THIS_MODULE, "sec");
	if (IS_ERR(sec_class)) {
		pr_err("%s: Failed to create sec class!\n", __func__);
		return -ENODEV;
	}

	pr_info("%s: Created sec class.\n", __func__);
	return 0;
}
core_initcall(sec_class_init);
