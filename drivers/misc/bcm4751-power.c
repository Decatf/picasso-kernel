/*
 * driver/misc/bcm4751-power.c
 *
 * driver supporting bcm4751(GPS) power control
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
#include <linux/regulator/machine.h>
#include <linux/blkdev.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

struct bcm4751_power_platform_data {
	int power_gpio;
	int reset_gpio;
	struct regulator *gps_lna;
};

static int s_nRstState, s_PwrEnState;
static DEFINE_MUTEX(gps_mutex);

static struct miscdevice sec_gps_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sec_gps",
};

extern struct class *sec_class;
struct device *sec_gps_dev;

static ssize_t gpio_n_rst_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", s_nRstState);
}

static ssize_t gpio_n_rst_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct bcm4751_power_platform_data *pdata;
	int stat;

	pdata = dev_get_platdata(dev);

	if (sscanf(buf, "%i", &stat) != 1 || (stat < 0 || stat > 1))
		return -EINVAL;

	s_nRstState = stat;
	gpio_set_value(pdata->reset_gpio, s_nRstState);
	dev_info(dev, "Set GPIO_GPS_nRST to %s.\n", (s_nRstState) ? "high" : "low");

	return size;
}

static DEVICE_ATTR(nrst, S_IRUGO | S_IWUSR, gpio_n_rst_show, gpio_n_rst_store);

static ssize_t gpio_pwr_en_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", s_PwrEnState);
}

static ssize_t gpio_pwr_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct bcm4751_power_platform_data *pdata;
	struct regulator* gps_lna;
	int stat;
	int ret;
	int lna_enabled;

	pdata = dev_get_platdata(dev);
	if (!pdata) {
		dev_err(dev, "no platform data.\n");
		return -ENODEV;
	}

	gps_lna = pdata->gps_lna;
	if (!gps_lna) {
		dev_err(dev, "no regulator.\n");
		return -ENODEV;
	}

	if (sscanf(buf, "%i", &stat) != 1 || (stat < 0 || stat > 1))
		return -EINVAL;

	if (stat != s_PwrEnState) {
		lna_enabled = regulator_is_enabled(gps_lna);
	} else
		return -EINVAL;

	mutex_lock(&gps_mutex);
	s_PwrEnState = stat;

	if (stat && !lna_enabled) {
		ret = regulator_enable(gps_lna);
		if (ret != 0) {
			dev_err(dev, "Failed to enable GPS_LNA_2.85V: %d\n", ret);
			goto end_of_function;
		}
		dev_info(dev, "GPS_LNA LDO turned on\n");
		msleep(10);
	}

	gpio_set_value(pdata->power_gpio, s_PwrEnState);
	dev_info(dev, "Set GPIO_GPS_PWR_EN to %s.\n", (s_PwrEnState) ? "high" : "low");

	if (!stat && lna_enabled) {
		ret = regulator_disable(gps_lna);
		if (ret != 0) {
			dev_err(dev, "Failed to disable GPS_LNA_2.85V: %d\n", ret);
			goto end_of_function;
		}
		dev_info(dev, "GPS_LNA LDO turned off\n");
	}

	ret = size;

end_of_function:
	mutex_unlock(&gps_mutex);
	return ret;
}

static DEVICE_ATTR(pwr_en, S_IRUGO | S_IWUSR, gpio_pwr_en_show, gpio_pwr_en_store);

static void sec_gps_hw_init(struct platform_device *pdev,
	struct bcm4751_power_platform_data *pdata)
{
	devm_gpio_request(&pdev->dev, pdata->reset_gpio, "GPS_N_RST");
	gpio_direction_output(pdata->reset_gpio, 1);
	s_nRstState  = 1;

	devm_gpio_request(&pdev->dev, pdata->power_gpio, "GPS_PWR_EN");
	gpio_direction_output(pdata->power_gpio, 0);
	s_PwrEnState = 0;
}

#ifdef CONFIG_OF
static struct bcm4751_power_platform_data* sec_jack_parse_dt(struct platform_device *pdev)
{
	struct bcm4751_power_platform_data *pdata;
	struct device_node *np = pdev->dev.of_node;

	pdata = kzalloc(sizeof(struct bcm4751_power_platform_data), GFP_KERNEL);
	if (!pdata) {
		pr_err("%s: could not allocate platform data.\n", __func__);
		return NULL;
	}

	pdata->power_gpio = of_get_named_gpio(np, "power-gpio", 0);
	pdata->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);

	if (of_find_property(np, "gps-lna-supply", NULL)) {
		pdata->gps_lna = devm_regulator_get(&pdev->dev, "gps-lna");
		if (IS_ERR(pdata->gps_lna)) {
			kfree(pdata);
			return NULL;
		}
	}

	return pdata;
}
#else
static struct bcm4751_power_platform_data* sec_jack_parse_dt(struct platform_device *pdev)
{
	return NULL;
}
#endif

static int bcm4751_power_probe(struct platform_device *pdev)
{
	struct bcm4751_power_platform_data *pdata;
	int ret = 0;

	pr_info("%s.\n", __func__);

	pdata = sec_jack_parse_dt(pdev);
	if (!pdata) {
		pr_err("%s : pdata is NULL.\n", __func__);
		return -ENODEV;
	}

	sec_gps_hw_init(pdev, pdata);

	ret = misc_register(&sec_gps_device);
	if (ret < 0) {
		printk(KERN_ERR "%s: misc_register failed!\n", __func__);
		goto fail;
	}

	sec_gps_dev = device_create(sec_class, NULL, 0, NULL, "gps");
	if (IS_ERR(sec_gps_dev)) {
		printk(KERN_ERR "%s: failed to create device!\n", __func__);
		ret = -EINVAL;
		goto fail_after_misc_reg;
	}

	if (device_create_file(sec_gps_dev, &dev_attr_nrst) < 0) {
		printk(KERN_ERR "%s: failed to create device file!(%s)!\n",
			__func__, dev_attr_nrst.attr.name);
		ret = -EINVAL;
		goto fail_after_device_create;
	}

	if (device_create_file(sec_gps_dev, &dev_attr_pwr_en) < 0) {
		printk(KERN_ERR "%s: failed to create device file!(%s)!\n",
			__func__, dev_attr_pwr_en.attr.name);
		device_remove_file(sec_gps_dev, &dev_attr_nrst);
		ret = -EINVAL;
		goto fail_after_device_create;
	}

	pdev->dev.platform_data = pdata;
	sec_gps_dev->platform_data = pdata;

	pr_info("%s: probed\n", __func__);

	return 0;

fail_after_device_create:
	device_destroy(sec_class, 0);
fail_after_misc_reg:
	misc_deregister(&sec_gps_device);
fail:
	return ret;
}

static int bcm4751_power_remove(struct platform_device *pdev)
{
	struct bcm4751_power_platform_data *pdata = dev_get_platdata(&pdev->dev);

	device_remove_file(sec_gps_dev, &dev_attr_pwr_en);
	device_remove_file(sec_gps_dev, &dev_attr_nrst);

	device_destroy(sec_class, 0);

	misc_deregister(&sec_gps_device);
	devm_regulator_put(pdata->gps_lna);

	devm_gpio_free(&pdev->dev, pdata->reset_gpio);
	devm_gpio_free(&pdev->dev, pdata->power_gpio);

	kfree(pdata);

	return 0;
}

static const struct of_device_id bcm4751_power_of_ids[] = {
	{ .compatible = "broadcom,bcm4751-power" },
	{ }
};

static struct platform_driver bcm4751_power_driver = {
	.probe = bcm4751_power_probe,
	.remove = bcm4751_power_remove,
	.driver = {
		.name = "bcm4751_power",
		.of_match_table = bcm4751_power_of_ids,
		.owner = THIS_MODULE,
	},
};

module_platform_driver(bcm4751_power_driver);

/* Module information */
MODULE_AUTHOR("Samsung");
MODULE_DESCRIPTION("GPS power control driver");
MODULE_LICENSE("GPL");
