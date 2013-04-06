/*
 * Simple dvfs driver for tegra 2 boards
 *
 * Author: Dmitry Osipenko <digetx@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/cpufreq.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/clk-provider.h>

static DEFINE_MUTEX(dvfs_lock);
static bool dvfs_enabled;
static struct device *dvfs_dev;
static struct regulator *cpu_reg, *core_reg, *rtc_reg;

struct dvfs_domain;

struct dvfs_client {
	struct delayed_work	work;
	struct dvfs_domain	*dvfs;
	struct notifier_block	nb;
	struct list_head	node;
	struct mutex		deferred_work_lock;
	struct clk		*clk;
	const char		*clk_name;
	unsigned		index;
	int			freqs[];
};

struct dvfs_domain {
	struct list_head	active_clients;
	struct dvfs_client	**clients;
	int			voltages_mv[];
};

#define DVFS_CLIENT(_name, _freqs)					\
	static struct dvfs_client dvfs_##_name##_client = {		\
		.clk = NULL,						\
		.clk_name = #_name,					\
		.freqs = { _freqs, -1 },				\
	}

#define DVFS(_name, _voltages, _client...)				\
	static struct dvfs_client *dvfs_##_name##_clients[] = {		\
		_client, NULL						\
	};								\
	static struct dvfs_domain dvfs_##_name = {			\
		.clients = dvfs_##_name##_clients,			\
		.voltages_mv =  { _voltages, },				\
	}

#define _100uV			100000
#define _170uV			170000

/* cpu domain defines */
#define CPU_MAX_VDD		1125
#define CPU_NOMINAL_VDD		1000

#define CPU_MILLIVOLTS		750,	775,	825,	875,	925, \
				950,	1000,	1100,	CPU_MAX_VDD

#define CPU_FREQS		216,	312,	456,	608,	750, \
				760,	816,	912,	1000

/* core domain defines */
#define CORE_MAX_VDD		1300
#define CORE_NOMINAL_VDD	1225

#define CORE_MILLIVOLTS		1100,	1125,	1150,	1200, \
				1225,	1250,	1275,	CORE_MAX_VDD

#define EMC_FREQS		0,	150000,	300000,	600000

#define DISP1_FREQS		158000,	158000,	190000

#define DISP2_FREQS		158000,	158000,	190000

#define HDMI_FREQS		0,	0,	0,	148500

#define HOST1X_FREQS		104500,	133000,	166000

#define EPP_FREQS		133000,	171000,	247000,	300000

#define V2D_FREQS		133000,	171000,	247000,	300000

#define V3D_FREQS		142500,	190000,	275500,	300000

#define MPE_FREQS		142500,	190000,	275500,	300000

#define VI_FREQS		85000,	100000,	150000

#define CSI_FREQS		0,	0,	0,	0, \
				72000

#define SCLK_FREQS		123500,	159500,	207000,	240000, \
				264000,	277500

#define VDE_FREQS		123500,	152000,	237500,	300000

#define MIPI_FREQS		40000,	40000,	40000,	40000, \
				60000

#define USBD_FREQS		400000,	400000,	400000,	480000

#define USB2_FREQS		0,	0,	480000

#define USB3_FREQS		400000,	400000,	400000,	480000

/* rtc domain defines */
#define RTC_MAX_VDD		1300
#define RTC_MIN_VDD		950
#define RTC_NOMINAL_VDD		RTC_MAX_VDD

DVFS_CLIENT(cpu, CPU_FREQS);
DVFS(cpu, CPU_MILLIVOLTS, &dvfs_cpu_client);

// DVFS_CLIENT(disp1, DISP1_FREQS);
// DVFS_CLIENT(disp2, DISP2_FREQS);
DVFS_CLIENT(hdmi, HDMI_FREQS);
DVFS_CLIENT(avp_emc, EMC_FREQS);
DVFS_CLIENT(cpu_emc, EMC_FREQS);
DVFS_CLIENT(disp1_emc, EMC_FREQS);
DVFS_CLIENT(disp2_emc, EMC_FREQS);
DVFS_CLIENT(hdmi_emc, EMC_FREQS);
DVFS_CLIENT(3d_emc, EMC_FREQS);
DVFS_CLIENT(2d_emc, EMC_FREQS);
DVFS_CLIENT(mpe_emc, EMC_FREQS);
DVFS_CLIENT(usbd_emc, EMC_FREQS);
DVFS_CLIENT(usb1_emc, EMC_FREQS);
DVFS_CLIENT(usb2_emc, EMC_FREQS);
DVFS_CLIENT(usb3_emc, EMC_FREQS);
DVFS_CLIENT(camera_emc, EMC_FREQS);
DVFS_CLIENT(host1x, HOST1X_FREQS);
DVFS_CLIENT(epp, EPP_FREQS);
DVFS_CLIENT(2d, V2D_FREQS);
DVFS_CLIENT(3d, V3D_FREQS);
DVFS_CLIENT(mpe, MPE_FREQS);
DVFS_CLIENT(vi, VI_FREQS);
DVFS_CLIENT(csi, CSI_FREQS);
DVFS_CLIENT(sclk, SCLK_FREQS);
DVFS_CLIENT(vde, VDE_FREQS);
DVFS_CLIENT(mipi, MIPI_FREQS);
DVFS_CLIENT(usbd, USBD_FREQS);
DVFS_CLIENT(usb2, USB2_FREQS);
DVFS_CLIENT(usb3, USB3_FREQS);
DVFS(core, CORE_MILLIVOLTS,
	/*&dvfs_disp1_client,	&dvfs_disp2_client,*/	&dvfs_hdmi_client,
	&dvfs_avp_emc_client,	&dvfs_cpu_emc_client,	&dvfs_disp1_emc_client,
	&dvfs_disp2_emc_client,	&dvfs_hdmi_emc_client,	&dvfs_2d_emc_client,
	&dvfs_3d_emc_client,	&dvfs_mpe_emc_client,	&dvfs_usbd_emc_client,
	&dvfs_usb1_emc_client,	&dvfs_usb2_emc_client,	&dvfs_usb3_emc_client,
	&dvfs_camera_emc_client,&dvfs_host1x_client,	&dvfs_epp_client,
	&dvfs_2d_client,	&dvfs_3d_client,	&dvfs_mpe_client,
	&dvfs_vi_client,	&dvfs_csi_client,	&dvfs_sclk_client,
	&dvfs_vde_client,	&dvfs_mipi_client,	&dvfs_usbd_client,
	&dvfs_usb2_client,	&dvfs_usb3_client);

static void dvfs_update_cpu_voltage(int new_uV)
{
	int ret;

	dev_dbg(dvfs_dev, "setting cpu voltage %duV -> %duV\n",
		regulator_get_voltage(cpu_reg), new_uV);

	ret = regulator_set_voltage(cpu_reg, new_uV, CPU_MAX_VDD * 1000);
	if (ret)
		dev_err(dvfs_dev, "failed update cpu voltage\n");
}

static void dvfs_update_core_voltage(int new_uV)
{
	int ret;

	dev_dbg(dvfs_dev, "setting core voltage %duV -> %duV\n",
		regulator_get_voltage(core_reg), new_uV);

	ret = regulator_set_voltage(core_reg, new_uV, CORE_MAX_VDD * 1000);
	if (ret)
		dev_err(dvfs_dev, "failed update core voltage\n");
}

static void dvfs_update_rtc_voltage(int new_uV)
{
	int ret;

	dev_dbg(dvfs_dev, "setting rtc voltage %duV -> %duV\n",
		regulator_get_voltage(rtc_reg), new_uV);

	ret = regulator_set_voltage(rtc_reg, new_uV, RTC_MAX_VDD * 1000);
	if (ret)
		dev_err(dvfs_dev, "failed update rtc voltage\n");
}

#define UPDATE_STEP	150000
static void update_core_vdd(int new_core_vdd)
{
	int current_rtc_vdd = regulator_get_voltage(rtc_reg);
	int current_core_vdd = regulator_get_voltage(core_reg);
	int core_rtc_delta = new_core_vdd - current_rtc_vdd;

	if (abs(core_rtc_delta) > _170uV) {
		int update_step, steps_nb, new_rtc_vdd, rtc_vdd_limit;

		if (current_rtc_vdd != current_core_vdd)
			dvfs_update_rtc_voltage(current_core_vdd);

		if (core_rtc_delta > 0) {
			rtc_vdd_limit = RTC_MAX_VDD * 1000;
			update_step = UPDATE_STEP;
		} else {
			rtc_vdd_limit = RTC_MIN_VDD * 1000;
			update_step = -UPDATE_STEP;
		}

		new_rtc_vdd = max(new_core_vdd, rtc_vdd_limit);
		steps_nb = abs(current_rtc_vdd - new_rtc_vdd) / UPDATE_STEP - 1;

		dev_dbg(dvfs_dev, "steps_nb = %d\n", steps_nb);

		do {
			current_core_vdd += update_step;
			dvfs_update_rtc_voltage(current_core_vdd);
			dvfs_update_core_voltage(current_core_vdd);
		} while (--steps_nb > 0);
	}

	if (current_core_vdd != new_core_vdd)
		dvfs_update_core_voltage(new_core_vdd);
}

static void dvfs_set_nominal_voltages(void)
{
	update_core_vdd(CORE_NOMINAL_VDD * 1000);

	if (regulator_get_voltage(cpu_reg) != CPU_NOMINAL_VDD * 1000)
		dvfs_update_cpu_voltage(CPU_NOMINAL_VDD * 1000);
}

/*
 * Rules:
 * 	1) VDD_CORE – VDD_CPU >/= 100mV
 * 	2) VDD_CORE must stay within 170mV of VDD_RTC when VDD_CORE is powered
 */
static void update_voltages(void)
{
	struct dvfs_client *client, *cpu_client = dvfs_cpu.clients[0];
	unsigned dvfs_core_index = 0, dvfs_cpu_index;
	int new_cpu_vdd, new_core_vdd;
	int current_cpu_vdd;

	if (!dvfs_enabled)
		return;

	/*
	 * Get core index
	 */
	list_for_each_entry(client, &dvfs_core.active_clients, node) {
		dev_dbg(dvfs_dev, "active core client %s index = %d\n",
			client->clk_name, client->index);
		dvfs_core_index = max(client->index, dvfs_core_index);
	}

	dvfs_cpu_index = cpu_client->index;

	dev_dbg(dvfs_dev, "cpu client index = %d\n", dvfs_cpu_index);

	/*
	 * Get current local vdd's vals
	 * NOTE: cpu is the only client of it's power domain
	 */
	new_core_vdd = dvfs_core.voltages_mv[dvfs_core_index] * 1000;
	new_cpu_vdd = dvfs_cpu.voltages_mv[dvfs_cpu_index] * 1000;

	/*
	 * Recalc vdd's according to rules
	 */
	if (new_core_vdd - new_cpu_vdd < _100uV)
		new_core_vdd = new_cpu_vdd + _100uV;

	/*
	 * Perform regulators update
	 */
	current_cpu_vdd = regulator_get_voltage(cpu_reg);

	/*
	 * Who goes update first?
	 */
	if (new_core_vdd - current_cpu_vdd < _100uV) {
		if (current_cpu_vdd != new_cpu_vdd)
			dvfs_update_cpu_voltage(new_cpu_vdd);
		update_core_vdd(new_core_vdd);
	} else {
		update_core_vdd(new_core_vdd);
		if (current_cpu_vdd != new_cpu_vdd)
			dvfs_update_cpu_voltage(new_cpu_vdd);
	}
}

/*
 * Get lowest suitable voltage index
 */
static void update_freq_index(struct dvfs_client *c, unsigned long freq)
{
	for (c->index = 0; c->freqs[c->index + 1] >= 0; c->index++)
		if (c->freqs[c->index] * 1000 >= freq)
			break;
}

static int dvfs_cpu_change_notify(struct notifier_block *nb,
				  unsigned long flags, void *data)
{
	struct dvfs_client *client = container_of(nb, struct dvfs_client, nb);
	struct cpufreq_freqs *freqs = data;

	/*
	 * Ignore duplicated notify
	 */
	if (freqs->cpu > 0)
		return NOTIFY_OK;

	switch (flags) {
	case CPUFREQ_PRECHANGE:
		/*
		 * Perform update before going from low to high
		 */
		if (freqs->new > freqs->old) {
			dev_dbg(dvfs_dev, "%s PRE rate change %d -> %d\n",
				client->clk_name, freqs->old, freqs->new);

			mutex_lock(&dvfs_lock);

			update_freq_index(client, freqs->new);
			update_voltages();

			mutex_unlock(&dvfs_lock);
		}
		break;

	case CPUFREQ_POSTCHANGE:
		/*
		 * Perform update after going from high to low
		 */
		if (freqs->new < freqs->old) {
			dev_dbg(dvfs_dev, "%s POST rate change %d -> %d\n",
				client->clk_name, freqs->old, freqs->new);

			mutex_lock(&dvfs_lock);

			update_freq_index(client, freqs->new);
			update_voltages();

			mutex_unlock(&dvfs_lock);
		}
	}

	return NOTIFY_OK;
}

/*
 * Perform deferred disable in order to avoid slowdown
 * caused by fast clk enable/disable
 */
static void deferred_disable(struct work_struct *work)
{
	struct dvfs_client *client =
			container_of(work, struct dvfs_client, work.work);

	dev_dbg(dvfs_dev, "deferred %s POST disable change\n",
		client->clk_name);

	mutex_lock(&client->deferred_work_lock);
	mutex_lock(&dvfs_lock);

	list_del_init(&client->node);
	update_voltages();

	mutex_unlock(&dvfs_lock);
	mutex_unlock(&client->deferred_work_lock);
}

static int dvfs_core_change_notify(struct notifier_block *nb,
				   unsigned long flags, void *data)
{
	struct dvfs_client *client = container_of(nb, struct dvfs_client, nb);
	struct clk_notifier_data *cnd = data;

	switch (flags) {
	case PRE_RATE_SET:
		/*
		 * Perform update before going from low to high
		 */
		if (cnd->new_rate > cnd->old_rate) {
			dev_dbg(dvfs_dev, "%s PRE rate change %lu -> %lu\n",
				client->clk_name, cnd->old_rate, cnd->new_rate);

			mutex_lock(&dvfs_lock);

			update_freq_index(client, cnd->new_rate);
			update_voltages();

			mutex_unlock(&dvfs_lock);
		}
		break;

	case POST_RATE_SET:
		/*
		 * Perform update after going from high to low
		 */
		if (cnd->new_rate < cnd->old_rate) {
			dev_dbg(dvfs_dev, "%s POST rate change %lu -> %lu\n",
				client->clk_name, cnd->old_rate, cnd->new_rate);

			mutex_lock(&dvfs_lock);

			update_freq_index(client, cnd->new_rate);
			update_voltages();

			mutex_unlock(&dvfs_lock);
		}
		break;
	
	case PRE_ENABLE_CHANGE:
		dev_dbg(dvfs_dev, "%s PRE enable change\n", client->clk_name);

		if (__clk_get_enable_count(client->clk))
			break;

		mutex_lock(&client->deferred_work_lock);

		if (list_empty(&client->node)) {
			mutex_lock(&dvfs_lock);

			list_add(&client->node, &client->dvfs->active_clients);
			update_voltages();

			mutex_unlock(&dvfs_lock);
		} else {
			dev_dbg(dvfs_dev, "canceled delayed %s POST disable\n",
				client->clk_name);

			cancel_delayed_work(&client->work);
		}

		mutex_unlock(&client->deferred_work_lock);
		break;

	case POST_DISABLE_CHANGE:
		dev_dbg(dvfs_dev, "%s POST disable change\n", client->clk_name);

		if (__clk_get_enable_count(client->clk))
			break;

		mutex_lock(&client->deferred_work_lock);

		if (dvfs_enabled) {
			dev_dbg(dvfs_dev, "scheduled %s POST disable\n",
				client->clk_name);

			schedule_delayed_work(&client->work, HZ);
		} else {
			mutex_lock(&dvfs_lock);

			list_del_init(&client->node);

			mutex_unlock(&dvfs_lock);
		}

		mutex_unlock(&client->deferred_work_lock);
		break;

	case ABORT_RATE_CHANGE:
		dev_warn(dvfs_dev, "FIX ME! %s\n", client->clk_name);
	}

	return NOTIFY_OK;
}

static void dvfs_init(struct dvfs_domain *dvfs)
{
	int i;

	INIT_LIST_HEAD(&dvfs->active_clients);

	for (i = 0; dvfs->clients[i] != NULL; i++) {
		struct dvfs_client *client = dvfs->clients[i];
		int ret;

		client->clk = devm_clk_get(dvfs_dev, client->clk_name);
		if (IS_ERR(client->clk)) {
			dev_err(dvfs_dev, "Can't get %s clk\n",
				client->clk_name);

			client->clk = NULL;
			continue;
		}

		client->dvfs = dvfs;
		mutex_init(&client->deferred_work_lock);

		mutex_lock(&client->deferred_work_lock);

		if (dvfs == &dvfs_core) {
			client->nb.notifier_call = dvfs_core_change_notify;
			ret = clk_notifier_register(client->clk, &client->nb);
		} else {
			client->nb.notifier_call = dvfs_cpu_change_notify;
			ret = cpufreq_register_notifier(&client->nb,
						CPUFREQ_TRANSITION_NOTIFIER);
		}

		if (ret) {
			dev_err(dvfs_dev, "Can't register %s clk notifier\n",
				client->clk_name);

			devm_clk_put(dvfs_dev, client->clk);
			client->clk = NULL;
			mutex_unlock(&client->deferred_work_lock);
			continue;
		}

		INIT_DELAYED_WORK(&client->work, deferred_disable);
		INIT_LIST_HEAD(&client->node);

		if (__clk_get_enable_count(client->clk)) {
			unsigned long rate = clk_get_rate(client->clk);

			list_add(&client->node, &dvfs->active_clients);
			update_freq_index(client, rate);

			dev_dbg(dvfs_dev, "%s rate = %luHz index = %d\n",
				client->clk_name, rate, client->index);
		}

		mutex_unlock(&client->deferred_work_lock);
	}
}

static void dvfs_release(struct dvfs_domain *dvfs)
{
	int i;

	for (i = 0; dvfs->clients[i] != NULL; i++) {
		struct dvfs_client *client = dvfs->clients[i];

		if (client->clk) {
			if (dvfs == &dvfs_core)
				clk_notifier_unregister(client->clk,
							&client->nb);
			else
				cpufreq_unregister_notifier(&client->nb,
						CPUFREQ_TRANSITION_NOTIFIER);

			mutex_destroy(&client->deferred_work_lock);
		}
	}
}

static void dvfs_start_locked(void)
{
	dvfs_set_nominal_voltages();
	dvfs_enabled = true;
	update_voltages();
}

static void dvfs_stop_locked(void)
{
	dvfs_set_nominal_voltages();
	dvfs_enabled = false;
}

static int tegra_dvfs_probe(struct platform_device *pdev)
{
	dvfs_dev = &pdev->dev;

	cpu_reg = devm_regulator_get(dvfs_dev, "cpu");
	if (IS_ERR(cpu_reg)) {
		dev_err(dvfs_dev, "Can't get cpu regulator\n");
		return PTR_ERR(cpu_reg);
	}

	core_reg = devm_regulator_get(dvfs_dev, "core");
	if (IS_ERR(core_reg)) {
		dev_err(dvfs_dev, "Can't get core regulator\n");
		return PTR_ERR(core_reg);
	}

	rtc_reg = devm_regulator_get(dvfs_dev, "rtc");
	if (IS_ERR(rtc_reg)) {
		dev_err(dvfs_dev, "Can't get rtc regulator\n");
		return PTR_ERR(rtc_reg);
	}

	mutex_lock(&dvfs_lock);

	dvfs_init(&dvfs_cpu);
	dvfs_init(&dvfs_core);

	dvfs_start_locked();

	mutex_unlock(&dvfs_lock);

	dev_dbg(dvfs_dev, "registered\n");

	return 0;
}

static void dvfs_lock_all_core_clients(void)
{
	struct dvfs_client *client;
	int i;

	for (i = 0; dvfs_core.clients[i] != NULL; i++) {
		client = dvfs_core.clients[i];

		if (client->clk)
			mutex_lock(&client->deferred_work_lock);
	}
}

static void dvfs_unlock_all_core_clients(void)
{
	struct dvfs_client *client;
	int i;

	for (i = 0; dvfs_core.clients[i] != NULL; i++) {
		client = dvfs_core.clients[i];

		if (client->clk)
			mutex_unlock(&client->deferred_work_lock);
	}
}

static int dvfs_suspend(struct device *dev)
{
	struct dvfs_client *client, *tmp;

	dev_dbg(dvfs_dev, "suspending...\n");

	dvfs_lock_all_core_clients();
	mutex_lock(&dvfs_lock);

	list_for_each_entry_safe(client, tmp, &dvfs_core.active_clients, node) {
		if (work_busy(&client->work.work)) {
			cancel_delayed_work(&client->work);
			list_del_init(&client->node);
		}
	}

	dvfs_stop_locked();

	mutex_unlock(&dvfs_lock);
	dvfs_unlock_all_core_clients();

	return 0;
}

static int dvfs_resume(struct device *dev)
{
	dev_dbg(dvfs_dev, "resuming...\n");

	dvfs_lock_all_core_clients();
	mutex_lock(&dvfs_lock);

	dvfs_start_locked();

	mutex_unlock(&dvfs_lock);
	dvfs_unlock_all_core_clients();

	return 0;
}

static int tegra_dvfs_remove(struct platform_device *pdev)
{
	dvfs_suspend(dvfs_dev);

	dvfs_release(&dvfs_cpu);
	dvfs_release(&dvfs_core);

	return 0;
}

static SIMPLE_DEV_PM_OPS(tegra_dvfs_pm_ops, dvfs_suspend, dvfs_resume);

static struct of_device_id tegra_dvfs_of_match[] = {
	{ .compatible = "tegra20-dvfs", },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_dvfs_of_match);

static struct platform_driver tegra_dvfs_driver = {
	.probe		= tegra_dvfs_probe,
	.remove		= tegra_dvfs_remove,
	.driver		= {
		.name		= "tegra-dvfs",
		.owner		= THIS_MODULE,
		.of_match_table = tegra_dvfs_of_match,
		.pm		= &tegra_dvfs_pm_ops,
	},
};

module_platform_driver(tegra_dvfs_driver);

MODULE_AUTHOR("Dmitry Osipenko <digetx@gmail.com>");
MODULE_DESCRIPTION("tegra dvfs driver");
MODULE_LICENSE("GPL");
