/*
 * arch/arm/mach-tegra/board-p4wifi.c
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

#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/syscore_ops.h>
#include <linux/reboot.h>
#include <linux/platform_device.h>
#include <linux/highmem.h>
#include <linux/memblock.h>
#include <linux/regulator/consumer.h>
#include <linux/of_platform.h>
#include <linux/mmc/host.h>
#include <linux/file.h>
#include <linux/input.h>
#include <linux/atmel_mxt1386.h>
#include <linux/power/p4_battery.h>
#include <linux/gpio/machine.h>
#include <linux/rfkill-gpio.h>

#include <asm/setup.h>
#include <asm/system_info.h>

#include "iomap.h"
#include "pm.h"

int cmc623_current_type = 0;
EXPORT_SYMBOL(cmc623_current_type);

#ifdef CONFIG_SAMSUNG_LPM_MODE
int charging_mode_from_boot;

/* Get charging_mode status from kernel CMDLINE parameter. */
module_param_cb(charging_mode,  &param_ops_int,
		&charging_mode_from_boot, 0644);
MODULE_PARM_DESC(charging_mode_from_boot, "Charging mode parameter value.");
#endif

/******************************************************************************
* bootloader cmdline args
*****************************************************************************/
/*
 * Set androidboot.mode=charger based on the
 * Samsung p4 charging_mode parameter.
 */
static int __init charging_mode_arg(char *p)
{
	unsigned long charging_mode;

	if (kstrtoul(p, 16, &charging_mode))
		return 1;

	if (charging_mode)
		strlcat(boot_command_line, " androidboot.mode=charger", COMMAND_LINE_SIZE);

	return 0;
}
early_param("charging_mode", charging_mode_arg);

static int __init cmc623_arg(char *p)
{
	pr_info("%s: panel type=cmc623f\n", __func__);
	cmc623_current_type = 1;
	return 0;
}
early_param("cmc623f", cmc623_arg);

/******************************************************************************
* Memory reserve
*****************************************************************************/
unsigned long tegra_bootloader_fb_start;
unsigned long tegra_bootloader_fb_size;
unsigned long tegra_fb_start;
unsigned long tegra_fb_size;
unsigned long tegra_fb2_start;
unsigned long tegra_fb2_size;
unsigned long tegra_carveout_start;
unsigned long tegra_carveout_size;
unsigned long tegra_lp0_vec_start;
unsigned long tegra_lp0_vec_size;
unsigned long tegra_grhost_aperture;

static int __init tegra_lp0_vec_arg(char *options)
{
	char *p = options;

	tegra_lp0_vec_size = memparse(p, &p);
	if (*p == '@')
		tegra_lp0_vec_start = memparse(p+1, &p);
	if (!tegra_lp0_vec_size || !tegra_lp0_vec_start) {
		tegra_lp0_vec_size = 0;
		tegra_lp0_vec_start = 0;
	}

	return 0;
}
early_param("lp0_vec", tegra_lp0_vec_arg);

void __init tegra_reserve(unsigned long carveout_size, unsigned long fb_size,
	unsigned long fb2_size)
{
	if (tegra_lp0_vec_size)
		if (memblock_reserve(tegra_lp0_vec_start, tegra_lp0_vec_size))
			pr_err("Failed to reserve lp0_vec %08lx@%08lx\n",
				tegra_lp0_vec_size, tegra_lp0_vec_start);


	tegra_carveout_start = memblock_end_of_DRAM() - carveout_size;
	if (memblock_remove(tegra_carveout_start, carveout_size))
		pr_err("Failed to remove carveout %08lx@%08lx from memory "
			"map\n",
			tegra_carveout_start, carveout_size);
	else
		tegra_carveout_size = carveout_size;

	tegra_fb2_start = memblock_end_of_DRAM() - fb2_size;
	if (memblock_remove(tegra_fb2_start, fb2_size))
		pr_err("Failed to remove second framebuffer %08lx@%08lx from "
			"memory map\n",
			tegra_fb2_start, fb2_size);
	else
		tegra_fb2_size = fb2_size;

	tegra_fb_start = memblock_end_of_DRAM() - fb_size;
	if (memblock_remove(tegra_fb_start, fb_size))
		pr_err("Failed to remove framebuffer %08lx@%08lx from memory "
			"map\n",
			tegra_fb_start, fb_size);
	else
		tegra_fb_size = fb_size;

	if (tegra_fb_size)
		tegra_grhost_aperture = tegra_fb_start;

	if (tegra_fb2_size && tegra_fb2_start < tegra_grhost_aperture)
		tegra_grhost_aperture = tegra_fb2_start;

	if (tegra_carveout_size && tegra_carveout_start < tegra_grhost_aperture)
		tegra_grhost_aperture = tegra_carveout_start;

	/*
	 * TODO: We should copy the bootloader's framebuffer to the framebuffer
	 * allocated above, and then free this one.
	 */
	if (tegra_bootloader_fb_size)
		if (memblock_reserve(tegra_bootloader_fb_start,
				tegra_bootloader_fb_size))
			pr_err("Failed to reserve lp0_vec %08lx@%08lx\n",
				tegra_lp0_vec_size, tegra_lp0_vec_start);

	pr_info("Tegra reserved memory:\n"
		"LP0:                    %08lx - %08lx\n"
		"Bootloader framebuffer: %08lx - %08lx\n"
		"Framebuffer:            %08lx - %08lx\n"
		"2nd Framebuffer:         %08lx - %08lx\n"
		"Carveout:               %08lx - %08lx\n",
		tegra_lp0_vec_start,
		tegra_lp0_vec_start + tegra_lp0_vec_size - 1,
		tegra_bootloader_fb_start,
		tegra_bootloader_fb_start + tegra_bootloader_fb_size - 1,
		tegra_fb_start,
		tegra_fb_start + tegra_fb_size - 1,
		tegra_fb2_start,
		tegra_fb2_start + tegra_fb2_size - 1,
		tegra_carveout_start,
		tegra_carveout_start + tegra_carveout_size - 1);
}

void __init tegra_release_bootloader_fb(void)
{
	/* Since bootloader fb is reserved in common.c, it is freed here. */
	if (tegra_bootloader_fb_size)
		if (memblock_free(tegra_bootloader_fb_start,
						tegra_bootloader_fb_size))
			pr_err("Failed to free bootloader fb.\n");
}

static struct resource ram_console_resources[] = {
	{
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name 		= "ram_console",
	.id 		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
};

void __init tegra_ram_console_debug_reserve(unsigned long ram_console_size)
{
	struct resource *res;
	long ret;

	res = platform_get_resource(&ram_console_device, IORESOURCE_MEM, 0);
	if (!res)
		goto fail;
	res->start = memblock_end_of_DRAM() - ram_console_size;
	res->end = res->start + ram_console_size - 1;
	ret = memblock_remove(res->start, ram_console_size);
	if (ret)
		goto fail;

	return;

fail:
	ram_console_device.resource = NULL;
	ram_console_device.num_resources = 0;
	pr_err("Failed to reserve memory block for ram console\n");
}

void __init tegra_ram_console_debug_init(void)
{
	int err;

	err = platform_device_register(&ram_console_device);
	if (err) {
		pr_err("%s: ram console registration failed (%d)!\n", __func__, err);
	}
}

void __init p3_reserve(void)
{
	pr_info("%s()\n",__func__);
	if (memblock_reserve(0x0, 4096) < 0)
		pr_warn("Cannot reserve first 4K of memory for safety\n");

	tegra_reserve(SZ_256M, SZ_8M + SZ_1M, SZ_16M);
	tegra_ram_console_debug_reserve(SZ_1M);
}

/******************************************************************************
* Reboot
*****************************************************************************/

#define GPIO_TA_nCONNECTED 178

/*
 * These defines must be kept in sync with the bootloader.
 */
#define REBOOT_MODE_NONE                0
#define REBOOT_MODE_DOWNLOAD            1
#define REBOOT_MODE_NORMAL              2
#define REBOOT_MODE_UPDATE              3
#define REBOOT_MODE_RECOVERY            4
#define REBOOT_MODE_FOTA                5
#define REBOOT_MODE_FASTBOOT            7
#define REBOOT_MODE_DOWNLOAD_FAILED     8
#define REBOOT_MODE_DOWNLOAD_SUCCESS    9

#define MISC_DEVICE "/dev/block/mmcblk0p6"

struct bootloader_message {
	char command[32];
	char status[32];
};

static int write_bootloader_message(char *cmd, int mode)
{
	struct file *filp;
	mm_segment_t oldfs;
	int ret = 0;
	loff_t pos = 2048L;  /* bootloader message offset in MISC.*/

	struct bootloader_message  bootmsg;

	memset(&bootmsg, 0, sizeof(struct bootloader_message));

	if (mode == REBOOT_MODE_RECOVERY) {
		strcpy(bootmsg.command, "boot-recovery");
#ifdef CONFIG_KERNEL_DEBUG_SEC
		reboot_mode = REBOOT_MODE_RECOVERY;
		kernel_sec_set_debug_level(KERNEL_SEC_DEBUG_LEVEL_LOW);
		kernel_sec_clear_upload_magic_number();
#endif
	} else if (mode == REBOOT_MODE_FASTBOOT)
		strcpy(bootmsg.command, "boot-fastboot");
	else if (mode == REBOOT_MODE_NORMAL)
		strcpy(bootmsg.command, "boot-reboot");
	else if (mode == REBOOT_MODE_FOTA)
		strcpy(bootmsg.command, "boot-fota");
	else if (mode == REBOOT_MODE_NONE)
		strcpy(bootmsg.command, "boot-normal");
	else
		strcpy(bootmsg.command, cmd);

	bootmsg.status[0] = (char) mode;


	filp = filp_open(MISC_DEVICE, O_WRONLY | O_LARGEFILE, 0);

	if (IS_ERR(filp)) {
		pr_info("failed to open MISC : '%s'.\n", MISC_DEVICE);
		return 0;
	}

	oldfs = get_fs();
	set_fs(get_ds());

	ret = vfs_write(filp, (const char *)&bootmsg,
			sizeof(struct bootloader_message), &pos);

	set_fs(oldfs);

	vfs_fsync(filp, 0);
	filp_close(filp, NULL);

	if (ret < 0)
		pr_info("failed to write on MISC\n");
	else
		pr_info("command : %s written on MISC\n", bootmsg.command);

	return ret;
}

/* Boot Mode Physical Addresses and Magic Token */
#define BOOT_MODE_P_ADDR	(0x20000000 - 0x0C)
#define BOOT_MAGIC_P_ADDR	(0x20000000 - 0x10)
#define BOOT_MAGIC_TOKEN	0x626F6F74

static void write_bootloader_mode(char boot_mode)
{
	void __iomem *to_io;
#if 0
	to_io = ioremap(BOOT_MODE_P_ADDR, 4);
	writel((unsigned long)boot_mode, to_io);
	iounmap(to_io);
#endif
	/* Write a magic value to a 2nd memory location to distinguish between a
	 * cold boot and a reboot.
	 */
	to_io = ioremap(BOOT_MAGIC_P_ADDR, 4);
	writel(BOOT_MAGIC_TOKEN, to_io);
	iounmap(to_io);
}

static int p3_notifier_call(struct notifier_block *notifier,
				unsigned long event, void *cmd)
{
	int mode;
	u32 value;
	value = gpio_get_value(GPIO_TA_nCONNECTED);

	if (event == SYS_RESTART) {
		mode = REBOOT_MODE_NORMAL;
		if (cmd) {
			if (!strcmp((char *)cmd, "recovery"))
				mode = REBOOT_MODE_RECOVERY;
			else if (!strcmp((char *)cmd, "bootloader"))
				mode = REBOOT_MODE_FASTBOOT;
			else if (!strcmp((char *)cmd, "fota"))
				mode = REBOOT_MODE_FOTA;
			else if (!strcmp((char *)cmd, "download"))
				mode = REBOOT_MODE_DOWNLOAD;
		}
	} else if (event == SYS_POWER_OFF
#ifdef CONFIG_SAMSUNG_LPM_MODE
		&& (charging_mode_from_boot == true)
#endif
		&& !value)
		mode = REBOOT_MODE_NORMAL;
	else
		mode = REBOOT_MODE_NONE;

	pr_info("%s, Reboot Mode : %d\n", __func__, mode);

	write_bootloader_mode(mode);

	write_bootloader_message(cmd, mode);

	return NOTIFY_DONE;
}

static struct notifier_block p3_reboot_notifier = {
	.notifier_call = p3_notifier_call,
	.priority = INT_MAX,
};

/******************************************************************************
* System revision
*****************************************************************************/

#define GPIO_HW_REV0		9  /* TEGRA_GPIO_PB1 */
#define GPIO_HW_REV1		87 /* TEGRA_GPIO_PK7 */
#define GPIO_HW_REV2		50 /* TEGRA_GPIO_PG2 */
#define GPIO_HW_REV3		48 /* TEGRA_GPIO_PG0 */
#define GPIO_HW_REV4		49 /* TEGRA_GPIO_PG1 */

struct board_revision {
	unsigned int value;
	unsigned int gpio_value;
	char string[20];
};

/* We'll enumerate board revision from 10
 * to avoid a confliction with revision numbers of P3
*/
struct board_revision p4_board_rev[] = {
	{10, 0x16, "Rev00"},
	{11, 0x01, "Rev01"},
	{12, 0x02, "Rev02" },
	{13, 0x03, "Rev03" },
	{14, 0x04, "Rev04" },
};

static void p4_check_hwrev(void)
{
	unsigned int value, rev_no, i;
	struct board_revision *board_rev;

	board_rev = p4_board_rev;
	rev_no = ARRAY_SIZE(p4_board_rev);

	gpio_request(GPIO_HW_REV0, "GPIO_HW_REV0");
	gpio_request(GPIO_HW_REV1, "GPIO_HW_REV1");
	gpio_request(GPIO_HW_REV2, "GPIO_HW_REV2");
	gpio_request(GPIO_HW_REV3, "GPIO_HW_REV3");
	gpio_request(GPIO_HW_REV4, "GPIO_HW_REV4");

	gpio_direction_input(GPIO_HW_REV0);
	gpio_direction_input(GPIO_HW_REV1);
	gpio_direction_input(GPIO_HW_REV2);
	gpio_direction_input(GPIO_HW_REV3);
	gpio_direction_input(GPIO_HW_REV4);

	value = gpio_get_value(GPIO_HW_REV0) |
			(gpio_get_value(GPIO_HW_REV1)<<1) |
			(gpio_get_value(GPIO_HW_REV2)<<2) |
			(gpio_get_value(GPIO_HW_REV3)<<3) |
			(gpio_get_value(GPIO_HW_REV4)<<4);

	gpio_free(GPIO_HW_REV0);
	gpio_free(GPIO_HW_REV1);
	gpio_free(GPIO_HW_REV2);
	gpio_free(GPIO_HW_REV3);
	gpio_free(GPIO_HW_REV4);

	for (i = 0; i < rev_no; i++) {
		if (board_rev[i].gpio_value == value)
			break;
	}

	system_rev = (i == rev_no) ? board_rev[rev_no-1].value : board_rev[i].value;

	if (i == rev_no)
		pr_warn("%s: Valid revision NOT found! Latest one will be assigned!\n", __func__);

	pr_info("%s: system_rev = %d (gpio value = 0x%02x)\n", __func__, system_rev, value);
}

/******************************************************************************
* battery
*****************************************************************************/
extern struct mxt_callbacks *charger_callbacks;

static void  p3_inform_charger_connection(int mode)
{
	if (charger_callbacks && charger_callbacks->inform_charger)
		charger_callbacks->inform_charger(charger_callbacks, mode);
};

struct p3_battery_platform_data p3_battery_platform = {
	.inform_charger_connection = p3_inform_charger_connection,
};


/******************************************************************************
* bluetooth rfkill
*****************************************************************************/
#define GPIO_BT_EN          77     // TEGRA_GPIO_PJ5
#define GPIO_BT_nRST        177    // TEGRA_GPIO_PW1

static struct rfkill_gpio_platform_data bluetooth_rfkill_platform_data = {
	.name	= "bluetooth_rfkill",
	.type	= RFKILL_TYPE_BLUETOOTH,
};

static struct platform_device bluetooth_rfkill_device = {
	.name	= "rfkill_gpio",
	.id	= 1,
	.dev	= {
		.platform_data = &bluetooth_rfkill_platform_data,
	},
};

static struct gpiod_lookup_table bluetooth_gpio_lookup = {
	.dev_id = "rfkill_gpio.1",
	.table = {
		GPIO_LOOKUP("tegra-gpio", GPIO_BT_nRST, "reset", GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP("tegra-gpio", GPIO_BT_EN, "shutdown", GPIO_ACTIVE_HIGH),
		{ },
	},
};

/******************************************************************************
* gps rfkill
*****************************************************************************/
#define GPIO_GPS_PWR_EN		202 // TEGRA_GPIO_PZ2
#define GPIO_GPS_N_RST		109 // TEGRA_GPIO_PN5

static struct rfkill_gpio_platform_data gps_rfkill_platform_data = {
	.name	= "gps_rfkill",
	.type	= RFKILL_TYPE_GPS,
};

static struct platform_device gps_rfkill_device = {
	.name	= "rfkill_gpio",
	.id	= 0,
	.dev	= {
		.platform_data = &gps_rfkill_platform_data,
	},
};

static struct gpiod_lookup_table gps_gpio_lookup = {
	.dev_id = "rfkill_gpio.0",
	.table = {
		GPIO_LOOKUP("tegra-gpio", GPIO_GPS_N_RST, "reset", GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP("tegra-gpio", GPIO_GPS_PWR_EN, "shutdown", GPIO_ACTIVE_HIGH),
		{ },
	},
};

#include <soc/tegra/pmc.h>


#define GPIO_ACCESSORY_EN	70 // TEGRA_GPIO_PI6
#define GPIO_CP_ON		115 //TEGRA_GPIO_PO3
#define GPIO_CP_RST		185 // TEGRA_GPIO_PX1

void __init p4wifi_machine_init(void)
{
	pr_info("%s()\n", __func__);
	tegra_ram_console_debug_init();

	p4_check_hwrev();


	register_reboot_notifier(&p3_reboot_notifier);

	gpiod_add_lookup_table(&bluetooth_gpio_lookup);
	gpiod_add_lookup_table(&gps_gpio_lookup);
	platform_device_register(&bluetooth_rfkill_device);
	platform_device_register(&gps_rfkill_device);

	tegra_powergate_power_off(TEGRA_POWERGATE_PCIE);

	gpio_direction_output(GPIO_ACCESSORY_EN, 0);


	gpio_set_value(GPIO_CP_ON, 0);
	gpio_set_value(GPIO_CP_RST, 0);
}
