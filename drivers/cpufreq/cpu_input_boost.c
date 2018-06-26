/*
 * Copyright (C) 2014-2015, Sultanxda <sultanxda@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "CPU-boost: " fmt

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/notifier.h>
#include <linux/slab.h>


/* Available bits for boost_drv state */
#define SCREEN_AWAKE		(1U << 0)
#define INPUT_BOOST		(1U << 1)
#define WAKE_BOOST		(1U << 2)
#define MAX_BOOST		(1U << 3)

struct boost_drv {
	struct workqueue_struct *wq;
	struct work_struct input_boost;
	struct delayed_work input_unboost;
	struct work_struct max_boost;
	struct delayed_work max_unboost;
	struct notifier_block cpu_notif;
	struct notifier_block fb_notif;
	atomic_t max_boost_dur;
	spinlock_t lock;
	u32 state;
};

enum boost_pwr {
	LOW,
	MID,
	HIGH,
};

static u32 get_boost_freq(struct boost_drv *b, u32 cpu)
{
	if (cpumask_test_cpu(cpu, cpu_lp_mask))
		return CONFIG_INPUT_BOOST_FREQ_LP;

/**
 * Auto boost freq calculation:
 * Requested boost freqs = maxfreq * boost_factor[i] / BOOST_FACTOR_DIVISOR,
 * so the lowest boost freq in this case would be maxfreq * 3 / 7
 */
static unsigned int boost_freq[3];
static unsigned int boost_factor[3] = {3, 4, 5};
#define BOOST_FACTOR_DIVISOR 7

/* Boost-freq level to use (high, mid, low) */
static enum boost_pwr boost_level;

/* Boost duration in millsecs */
static unsigned int boost_ms;

/* On/off switch */
static unsigned int enabled;
module_param(enabled, uint, 0644);

/**
 * Percentage threshold used to boost CPUs (default 30%). A higher
 * value will cause more CPUs to be boosted -- CPUs are boosted
 * when ((current_freq/max_freq) * 100) < up_threshold
 */
static unsigned int up_threshold = 30;
module_param(up_threshold, uint, 0644);

static void cpu_unboost_all(void)
{
	struct boost_policy *b;
	unsigned int cpu;

	get_online_cpus();
	for_each_possible_cpu(cpu) {
		b = &per_cpu(boost_info, cpu);
		if (b->boost_state == BOOST) {
			b->boost_state = UNBOOST;
			if (cpu_online(cpu))
				cpufreq_update_policy(cpu);
		}
	}
	put_online_cpus();
	boost_running = false;
}

static void __cpuinit cpu_boost_main(struct work_struct *work)
{
	if (!cancel_delayed_work_sync(&b->input_unboost) &&
		!cancel_delayed_work_sync(&b->max_unboost))
		return;

	clear_boost_bit(b, INPUT_BOOST | WAKE_BOOST | MAX_BOOST);
	update_online_cpu_policy();
}

void cpu_input_boost_kick(void)
{
	struct boost_drv *b = boost_drv_g;

	if (!b)
		return;

	queue_work(b->wq, &b->input_boost);
}

void cpu_input_boost_kick_max(unsigned int duration_ms)
{
	struct boost_drv *b = boost_drv_g;
	u32 state;

	if (!b)
		return;

	state = get_boost_state(b);

	/* Don't mess with wake boosts */
	if (state & WAKE_BOOST)
		return;

	atomic_set(&b->max_boost_dur, duration_ms);
	queue_work(b->wq, &b->max_boost);
}

static void input_boost_worker(struct work_struct *work)
{
	struct boost_drv *b = container_of(work, typeof(*b), input_boost);

	/* Nothing to boost */
	if (!num_cpus_to_boost) {
		put_online_cpus();
		boost_running = false;
		return;
	}

	/* Boost freq to use based on how many CPUs to boost */
	switch (num_cpus_to_boost * 100 / CONFIG_NR_CPUS) {
	case 25:
		boost_level = HIGH;
		break;
	case 50:
		boost_level = MID;
		break;
	default:
		boost_level = LOW;
	}

	/* Dual-core systems need more power */
	if (CONFIG_NR_CPUS == 2)
		boost_level++;

	/* Calculate boost duration */
	boost_ms = 3000 - ((num_cpus_to_boost * 750) + ((boost_level + 1) * 250));

static void max_boost_worker(struct work_struct *work)
{
	struct boost_drv *b = container_of(work, typeof(*b), max_boost);

	if (!cancel_delayed_work_sync(&b->max_unboost)) {
		set_boost_bit(b, MAX_BOOST);
		update_online_cpu_policy();
	}

	queue_delayed_work(b->wq, &b->max_unboost,
		msecs_to_jiffies(atomic_read(&b->max_boost_dur)));
}

static void max_unboost_worker(struct work_struct *work)
{
	struct boost_drv *b =
		container_of(to_delayed_work(work), typeof(*b), max_unboost);

	clear_boost_bit(b, WAKE_BOOST | MAX_BOOST);
	update_online_cpu_policy();
}

static int cpu_do_boost(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_policy *policy = data;
	struct boost_policy *b = &per_cpu(boost_info, policy->cpu);

	if (val != CPUFREQ_ADJUST)
		return NOTIFY_OK;

	state = get_boost_state(b);

	/* Boost CPU to max frequency for max boost */
	if (state & MAX_BOOST) {
		policy->min = policy->max;
		return NOTIFY_OK;
	}

	/*
	 * Boost to policy->max if the boost frequency is higher. When
	 * unboosting, set policy->min to the absolute min freq for the CPU.
	 */
	if (state & INPUT_BOOST) {
		boost_freq = get_boost_freq(b, policy->cpu);
		policy->min = min(policy->max, boost_freq);
	} else {
		policy->min = policy->cpuinfo.min_freq;
		break;
	case BOOST:
		if (boost_freq[boost_level] > policy->max)
			policy->min = policy->max;
		else
			policy->min = boost_freq[boost_level];
		break;
	}

	return NOTIFY_OK;
}

static int fb_notifier_cb(struct notifier_block *nb,
	unsigned long action, void *data)
{
	struct boost_drv *b = container_of(nb, typeof(*b), fb_notif);
	struct fb_event *evdata = data;
	int *blank = evdata->data;
	u32 state;

	/* Parse framebuffer blank events as soon as they occur */
	if (action != FB_EARLY_EVENT_BLANK)
		return NOTIFY_OK;

	state = get_boost_state(b);

	/* Boost when the screen turns on and unboost when it turns off */
	if (*blank == FB_BLANK_UNBLANK) {
		set_boost_bit(b, SCREEN_AWAKE);
		atomic_set(&b->max_boost_dur, CONFIG_INPUT_BOOST_DURATION_MS);
		queue_work(b->wq, &b->max_boost);
	} else {
		clear_boost_bit(b, SCREEN_AWAKE);
		unboost_all_cpus(b);
	}

	return NOTIFY_OK;
}

static void cpu_boost_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	u64 now;

	if (boost_running)
		return;
	if (!enabled)
		return;

	now = ktime_to_us(ktime_get());
	if (now - last_input_time < MIN_INPUT_INTERVAL)
		return;

	boost_running = true;
	queue_work(boost_wq, &boost_work);
	last_input_time = ktime_to_us(ktime_get());
}

static int cpu_boost_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpu_input_boost";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void cpu_boost_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id cpu_boost_ids[] = {
	/* multi-touch touchscreen */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			BIT_MASK(ABS_MT_POSITION_X) |
			BIT_MASK(ABS_MT_POSITION_Y) },
	},
	/* touchpad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] =
			BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) },
	},
	{ },
};

static struct input_handler cpu_boost_input_handler = {
	.event		= cpu_boost_input_event,
	.connect	= cpu_boost_input_connect,
	.disconnect	= cpu_boost_input_disconnect,
	.name		= "cpu_input_boost",
	.id_table	= cpu_boost_ids,
};

static int __init cpu_input_boost_init(void)
{
	struct cpufreq_frequency_table *table = cpufreq_frequency_get_table(0);
	int maxfreq = cpufreq_quick_get_max(0);
	int b_level = 0, req_freq[3];
	int i, ret = 1;

	if (!maxfreq) {
		pr_err("Failed to get max freq, input boost disabled\n");
		goto err;
	}

	spin_lock_init(&b->lock);
	INIT_WORK(&b->input_boost, input_boost_worker);
	INIT_DELAYED_WORK(&b->input_unboost, input_unboost_worker);
	INIT_WORK(&b->max_boost, max_boost_worker);
	INIT_DELAYED_WORK(&b->max_unboost, max_unboost_worker);
	b->state = SCREEN_AWAKE;

	b->cpu_notif.notifier_call = cpu_notifier_cb;
	ret = cpufreq_register_notifier(&b->cpu_notif, CPUFREQ_POLICY_NOTIFIER);
	if (ret) {
		pr_err("Failed to register cpufreq notifier, err: %d\n", ret);
		goto free_b;
	}

	boost_wq = alloc_workqueue("cpu_input_boost_wq", WQ_HIGHPRI | WQ_NON_REENTRANT, 0);
	if (!boost_wq) {
		pr_err("Failed to allocate workqueue\n");
		ret = -EFAULT;
		goto err;
	}

	cpufreq_register_notifier(&cpu_do_boost_nb, CPUFREQ_POLICY_NOTIFIER);

	INIT_DELAYED_WORK(&restore_work, cpu_restore_main);
	INIT_WORK(&boost_work, cpu_boost_main);

	ret = input_register_handler(&cpu_boost_input_handler);
	if (ret)
		pr_err("Failed to register input handler, err: %d\n", ret);
err:
	return ret;
}
late_initcall(cpu_input_boost_init);

MODULE_AUTHOR("Sultanxda <sultanxda@gmail.com>");
MODULE_DESCRIPTION("CPU Input Boost");
MODULE_LICENSE("GPLv2");
