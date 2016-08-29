/*
 * LED GPS PPS Activity Trigger
 *
 * Copyright 2006 Openedhand Ltd.
 *
 * Author: Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/leds.h>

#define BLINK_DELAY 100

DEFINE_LED_TRIGGER(ledtrig_pps);
static unsigned long pps_blink_delay = BLINK_DELAY;

void ledtrig_pps_activity(void)
{
	led_trigger_blink_oneshot(ledtrig_pps,
				  &pps_blink_delay, &pps_blink_delay, 0);
}
EXPORT_SYMBOL(ledtrig_pps_activity);

static int __init ledtrig_pps_init(void)
{
	led_trigger_register_simple("pps", &ledtrig_pps);
	return 0;
}

static void __exit ledtrig_pps_exit(void)
{
	led_trigger_unregister_simple(ledtrig_pps);
}

module_init(ledtrig_pps_init);
module_exit(ledtrig_pps_exit);

MODULE_AUTHOR("Richard Purdie <rpurdie@openedhand.com>");
MODULE_DESCRIPTION("LED GPS PPS Activity Trigger");
MODULE_LICENSE("GPL");
