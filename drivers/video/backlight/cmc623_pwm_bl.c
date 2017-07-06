/*
 *  cmc623_pwm Backlight Driver based on SWI Driver.
 *
 *  Copyright (c) 2009 Samsung Electronics
 *  InKi Dae <inki.dae@samsung.com>
 *
 *  Based on Sharp's Corgi Backlight Driver
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/delay.h>

#define CMC623_PWM_MAX_BRIGHTNESS			255
#define CMC623_PWM_MAX_BACKLIGHT			1600

struct cmc623_pwm_platform_data {
	struct backlight_device *bd;

	int default_brightness;

	int max_brightness;
	int mid_brightness;
	int low_brightness;
	int dim_brightness;

	int max_backlight;
	int mid_backlight;
	int low_backlight;
	int dim_backlight;
	int dark_backlight;
};


extern void set_backlight_pwm(int value);

static struct platform_device *bl_pdev;

static int current_backlight_level;

static int cmc623_pwm_suspended;
static int current_intensity;

static DEFINE_MUTEX(cmc623_pwm_mutex);


static void cmc623_pwm_apply_brightness(struct platform_device *pdev, int level)
{
	set_backlight_pwm(level);
	current_backlight_level = level;
}


static void cmc623_pwm_backlight_ctl(struct platform_device *pdev, int intensity)
{
	struct cmc623_pwm_platform_data *pdata = platform_get_drvdata(pdev);
	int tune_level;

	int max_brightness = pdata->max_brightness;
	int mid_brightness = pdata->mid_brightness;
	int low_brightness = pdata->low_brightness;
	int dim_brightness = pdata->dim_brightness;

	int max_backlight = pdata->max_backlight;
	int mid_backlight = pdata->mid_backlight;
	int low_backlight = pdata->low_backlight;
	int dim_backlight = pdata->dim_backlight;
	int dark_backlight = pdata->dark_backlight;

	/* brightness tuning*/
	if (intensity >= mid_brightness)
		tune_level =  mid_backlight +
			((intensity - mid_brightness) *
				(max_backlight-mid_backlight) /
				(max_brightness-mid_brightness));
	else if (intensity >= low_brightness)
		tune_level = low_backlight +
			((intensity - low_brightness) *
				(mid_backlight-low_backlight) /
				(mid_brightness-low_brightness));
	else if (intensity >= dim_brightness)
		tune_level = dim_backlight +
			((intensity - dim_brightness) *
				(low_backlight-dim_backlight) /
				(low_brightness-dim_brightness));
	else if (intensity > 0)
		tune_level = dark_backlight;
	else
		tune_level = intensity;

	/*printk("--- [cmc]%d(%d)---\n", intensity, tune_level);*/
   	//printk("[CMC623:INFO] Intensity : %d, Tuned Intensity : %d\n",intensity, tune_level);

	cmc623_pwm_apply_brightness(pdev, tune_level);
}


static void cmc623_pwm_send_intensity(struct backlight_device *bd)
{
	/*unsigned long flags;*/
	int intensity = bd->props.brightness;
	struct platform_device *pdev = NULL;

	dev_dbg(&bd->dev, "%s: intensity=%d\n", __func__, intensity);

	pdev = dev_get_drvdata(&bd->dev);
	if (pdev == NULL) {
		printk(KERN_ERR "%s:failed to get platform device.\n", __func__);
		return;
	}
#if 0
	if (bd->props.power != FB_BLANK_UNBLANK ||
		bd->props.fb_blank != FB_BLANK_UNBLANK ||
		cmc623_pwm_suspended) {
		printk("[cmc]i:%d(c:%d)\n", intensity, current_intensity);
		if (!current_intensity)
			return;
		msleep(1);
		intensity = 0;
	}
#endif

	mutex_lock(&cmc623_pwm_mutex);

	cmc623_pwm_backlight_ctl(pdev, intensity);

	mutex_unlock(&cmc623_pwm_mutex);

	current_intensity = intensity;
}

#if defined(CONFIG_PM)
static int cmc623_pwm_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct cmc623_pwm_platform_data *pdata = platform_get_drvdata(pdev);
	struct backlight_device *bd = pdata->bd;

	dev_info(dev, "%s\n", __func__);

	if (!cmc623_pwm_suspended) {
		cmc623_pwm_suspended = 1;
		cmc623_pwm_send_intensity(bd);
	}
	return 0;
}

static int cmc623_pwm_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct cmc623_pwm_platform_data *pdata = platform_get_drvdata(pdev);
	struct backlight_device *bd = pdata->bd;

	dev_info(dev, "%s\n", __func__);

	if (cmc623_pwm_suspended) {
		bd->props.brightness = pdata->default_brightness;
		cmc623_pwm_suspended = 0;
		cmc623_pwm_send_intensity(bd);
	}

	return 0;
}
#endif

static int cmc623_pwm_set_intensity(struct backlight_device *bd)
{
	dev_dbg(&bd->dev, "%s\n", __func__);

	cmc623_pwm_send_intensity(bd);

	return 0;
}


static int cmc623_pwm_get_intensity(struct backlight_device *bd)
{
	return current_intensity;
}


static struct backlight_ops cmc623_pwm_ops = {
	.get_brightness = cmc623_pwm_get_intensity,
	.update_status  = cmc623_pwm_set_intensity,
};

/*for measuring luminance*/
void cmc623_pwm_set_brightness(int brightness)
{
	/*unsigned long flags;*/

	printk("%s: value=%d\n", __func__, brightness);

	mutex_lock(&cmc623_pwm_mutex);

	cmc623_pwm_apply_brightness(bl_pdev, brightness);

	mutex_unlock(&cmc623_pwm_mutex);
}
EXPORT_SYMBOL(cmc623_pwm_set_brightness);

static int cmc623_pwm_validate_config(struct cmc623_pwm_platform_data *pdata)
{
	if (pdata->default_brightness < 0 ||
			pdata->default_brightness >= CMC623_PWM_MAX_BRIGHTNESS)
		return -EINVAL;

	if (pdata->max_brightness < 0 ||
			pdata->max_brightness > CMC623_PWM_MAX_BRIGHTNESS)
		return -EINVAL;
	if (pdata->mid_brightness < 0 ||
			pdata->mid_brightness > CMC623_PWM_MAX_BRIGHTNESS)
		return -EINVAL;
	if (pdata->low_brightness < 0 ||
			pdata->low_brightness > CMC623_PWM_MAX_BRIGHTNESS)
		return -EINVAL;
	if (pdata->dim_brightness < 0 ||
			pdata->dim_brightness > CMC623_PWM_MAX_BRIGHTNESS)
		return -EINVAL;

	if (pdata->max_backlight < 0 ||
			pdata->max_backlight > CMC623_PWM_MAX_BACKLIGHT)
		return -EINVAL;
	if (pdata->mid_backlight < 0 ||
			pdata->mid_backlight > CMC623_PWM_MAX_BACKLIGHT)
		return -EINVAL;
	if (pdata->low_backlight < 0 ||
			pdata->low_backlight > CMC623_PWM_MAX_BACKLIGHT)
		return -EINVAL;
	if (pdata->dim_backlight < 0 ||
			pdata->dim_backlight > CMC623_PWM_MAX_BACKLIGHT)
		return -EINVAL;
	if (pdata->dark_backlight < 0 ||
			pdata->dark_backlight > CMC623_PWM_MAX_BACKLIGHT)
		return -EINVAL;

	if (pdata->max_brightness < pdata->mid_brightness ||
			pdata->mid_brightness < pdata->low_brightness ||
			pdata->low_brightness < pdata->dim_brightness)
		return -EINVAL;

	if (pdata->max_backlight < pdata->mid_backlight ||
			pdata->mid_backlight < pdata->low_backlight ||
			pdata->low_backlight < pdata->dim_backlight ||
			pdata->dim_backlight < pdata->dark_backlight)
		return -EINVAL;

	return 0;
}

#ifdef CONFIG_OF
static struct cmc623_pwm_platform_data* cmc623_pwm_parse_dt(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct cmc623_pwm_platform_data *pdata;
	u32 val;

	pdata = kzalloc(sizeof(struct cmc623_pwm_platform_data), GFP_KERNEL);
	if (!pdata) {
		pr_err("%s: error allocating memory for platform data.\n", __func__);
		return -ENOMEM;
	}

	if (!of_property_read_u32(np, "default-brightness", &val))
		pdata->default_brightness = val;

	if (!of_property_read_u32(np, "max-brightness", &val))
		pdata->max_brightness = val;
	if (!of_property_read_u32(np, "mid-brightness", &val))
		pdata->mid_brightness = val;
	if (!of_property_read_u32(np, "low-brightness", &val))
		pdata->low_brightness = val;
	if (!of_property_read_u32(np, "dim-brightness", &val))
		pdata->dim_brightness = val;

	if (!of_property_read_u32(np, "max-backlight", &val))
		pdata->max_backlight = val;
	if (!of_property_read_u32(np, "mid-backlight", &val))
		pdata->mid_backlight = val;
	if (!of_property_read_u32(np, "low-backlight", &val))
		pdata->low_backlight = val;
	if (!of_property_read_u32(np, "dim-backlight", &val))
		pdata->dim_backlight = val;
	if (!of_property_read_u32(np, "dark-backlight", &val))
		pdata->dark_backlight = val;

	return pdata;
}
#else
static struct cmc623_pwm_platform_data* cmc623_pwm_parse_dt(struct device_node *, int *)
{
	return -EINVAL;
}
#endif

static int cmc623_pwm_probe(struct platform_device *pdev)
{
	struct cmc623_pwm_platform_data *pdata;
	struct backlight_properties props;
	struct backlight_device *bd;

	printk("cmc623_pwm Probe START!!!\n");

	pdata = cmc623_pwm_parse_dt(pdev);
	if (!pdata) {
		pr_err("%s: error parsing device tree\n", __func__);
		return -EINVAL;
	}

	if (cmc623_pwm_validate_config(pdata)) {
		pr_err("%s: invalid device tree configuation\n", __func__);
		kfree(pdata);
		return -EINVAL;
	}

	props.type = BACKLIGHT_RAW;

	bd = backlight_device_register("pwm-backlight",
		&pdev->dev, pdev, &cmc623_pwm_ops, &props);

	if (IS_ERR(bd)) {
		kfree(pdata);
		return PTR_ERR(bd);
	}

	pdata->bd = bd;
	platform_set_drvdata(pdev, pdata);

	bd->props.max_brightness = CMC623_PWM_MAX_BRIGHTNESS;
	bd->props.brightness = pdata->default_brightness;

	dev_info(&pdev->dev, "cmc623_pwm backlight driver is enabled.\n");

	current_backlight_level = pdata->mid_backlight;
	bl_pdev = pdev;

	printk("cmc623_pwm Probe END!!!\n");
	return 0;

}

static int cmc623_pwm_remove(struct platform_device *pdev)
{
	struct cmc623_pwm_platform_data *pdata = platform_get_drvdata(pdev);
	struct backlight_device *bd = pdata->bd;

	bd->props.brightness = 0;
	bd->props.power = 0;
	cmc623_pwm_send_intensity(bd);

	backlight_device_unregister(bd);

	return 0;
}

// static SIMPLE_DEV_PM_OPS(cmc623_pwm_pm_ops, cmc623_pwm_suspend, cmc623_pwm_resume);

static const struct of_device_id cmc623_pwm_of_ids[] = {
	{ .compatible = "samsung,cmc623-pwm" },
	{ }
};

static struct platform_driver cmc623_pwm_driver = {
	.driver		= {
		.name	= "cmc623_pwm_bl",
		.of_match_table = cmc623_pwm_of_ids,
		.owner	= THIS_MODULE,
		// .pm	= &cmc623_pwm_pm_ops,
	},
	.probe		= cmc623_pwm_probe,
	.remove		= cmc623_pwm_remove,
};

static int __init cmc623_pwm_init(void)
{
	return platform_driver_register(&cmc623_pwm_driver);
}

static void __exit cmc623_pwm_exit(void)
{
	platform_driver_unregister(&cmc623_pwm_driver);
}

module_init(cmc623_pwm_init);
module_exit(cmc623_pwm_exit);
