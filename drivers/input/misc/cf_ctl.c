/**
 * The device control driver for chipsailing's fingerprint sensor.
 *
 * Copyright (C) 2016 chipsailing Corporation. <http://www.chipsailingcorp.com>
 * Copyright (C) 2016 XXX. <mailto:xxx@chipsailingcorp.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
**/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/uaccess.h>

#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <linux/ioctl.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/printk.h>

#include "cf_ctl.h"

#define MODULE_NAME "cf_ctl"
#define xprintk(level, fmt, args...) printk(level MODULE_NAME": "fmt, ##args)

#define CF_VDD_MIN_UV 2800000
#define CF_VDD_MAX_UV 2800000


#define ANDROID_WAKELOCK	1

#if ANDROID_WAKELOCK
#include <linux/wakelock.h>
#endif

#ifndef CONFIG_OF
# error "error: this driver 'MODULE_NAME' only support dts."
#endif

/**
 * Define the driver version string.
 * There is NO need to modify 'rXXXX_yyyymmdd', it should be updated automatically
 * by the building script (see the 'Driver-revision' section in 'build.sh').
 */
#define CF_DRV_VERSION "v1.9.1-rXXXX_20160502"
#define LONGQI_HAL_COMPATIBLE 1	

struct cf_ctl_device {
    struct miscdevice miscdev;
    int reset_num;
    int irq_num;
    int pwr_num;
    struct regulator* vdd;
    u8 isPowerOn;
    struct work_struct work_queue;
    struct input_dev* input;
	struct wake_lock wakelock;
	struct fasync_struct *async;
};

typedef enum {
    CF_PIN_STATE_PWR__ON,
    CF_PIN_STATE_PWR_OFF,
    CF_PIN_STATE_RST_SET,
    CF_PIN_STATE_RST_CLR,
    CF_PIN_STATE_INT_SET,

    /* Array size */
    CF_PIN_STATE_MAX
} cf_pin_state_t;

/*static const char *cf_pinctrl_state_names[CF_PIN_STATE_MAX] = {
    "power_on", "power_off", "reset_low", "reset_high", "eint_set",
};*/

static int cf_ctl_device_power(bool on);
static int cf_ctl_device_reset(unsigned char);
static void cf_ctl_device_event(struct work_struct* ws);
static irqreturn_t cf_ctl_device_irq(int irq, void* dev_id);
static int cf_ctl_report_key_event(struct input_dev* input, cf_key_event_t* kevent);
static const char* cf_ctl_get_version(void);
static long cf_ctl_ioctl(struct file* filp, unsigned int cmd, unsigned long arg);
static int cf_ctl_open(struct inode* inode, struct file* filp);
static int cf_ctl_release(struct inode* inode, struct file* filp);
static int cf_ctl_fasync(int fd, struct file *fp, int mode);
static int cf_ctl_init_gpio_pins(void);
static int cf_ctl_init_input(void);
static int __init cf_ctl_driver_init(void);
static void __exit cf_ctl_driver_exit(void);


//static struct pinctrl *cf_pinctrl = NULL;
//static struct pinctrl_state *cf_pin_states[CF_PIN_STATE_MAX] = {NULL, };

static struct file_operations cf_ctl_fops = {
    .owner          = THIS_MODULE,
    .unlocked_ioctl = cf_ctl_ioctl,
    .open           = cf_ctl_open,
    .release        = cf_ctl_release,
	.fasync         = cf_ctl_fasync,
};

static struct cf_ctl_device cf_ctl_dev = {
    .miscdev = {
        .minor  = MISC_DYNAMIC_MINOR,
        .name   = "cs_spi",
        .fops   = &cf_ctl_fops,
    }, 0,
};

static int cf_ctl_init_gpio(void)
{
    int err = 0;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);

    if (gpio_is_valid(cf_ctl_dev.reset_num)) {
        err = gpio_request(cf_ctl_dev.reset_num, "cf-reset");

        if (err) {
            xprintk(KERN_ERR, "Could not request reset gpio.\n");
            return err;
        }
    }
    else {
        xprintk(KERN_ERR, "not valid reset gpio\n");
        return -EIO;
    }

    if (gpio_is_valid(cf_ctl_dev.irq_num)) {
        err = pinctrl_request_gpio(cf_ctl_dev.irq_num);

        if (err) {
            xprintk(KERN_ERR, "Could not request irq gpio.\n");
            gpio_free(cf_ctl_dev.reset_num);
            return err;
        }
    }
    else {
        xprintk(KERN_ERR, "not valid irq gpio\n");
        gpio_free(cf_ctl_dev.reset_num);
        return -EIO;
    }

    pinctrl_gpio_direction_input(cf_ctl_dev.irq_num);

    xprintk(KERN_DEBUG, "%s(..) ok! exit.\n", __FUNCTION__);

    return err;
}


static int cf_ctl_free_gpio(void)
{
    int err = 0;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);

    if (gpio_is_valid(cf_ctl_dev.irq_num)) {
		pinctrl_free_gpio(cf_ctl_dev.irq_num);
        free_irq(gpio_to_irq(cf_ctl_dev.irq_num), (void*)&cf_ctl_dev);
    }

	if (gpio_is_valid(cf_ctl_dev.reset_num)) {
        gpio_free(cf_ctl_dev.reset_num);
    }
    xprintk(KERN_DEBUG, "%s(..) ok! exit.\n", __FUNCTION__);

    return err;
}


static int cf_ctl_device_power(bool on)
{
    int err = 0;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);

    /*if (on && (!cf_ctl_dev.isPowerOn)) {
        err = regulator_enable(cf_ctl_dev.vdd);

        if (err) {
            xprintk(KERN_ERR, "Regulator vdd enable failed err = %d\n", err);
            return err;
        }

        msleep(10);
        cf_ctl_dev.isPowerOn = 1;
    }
    else if (!on && (cf_ctl_dev.isPowerOn)) {
        err = regulator_disable(cf_ctl_dev.vdd);

        if (err) {
            xprintk(KERN_ERR, "Regulator vdd disable failed err = %d\n", err);
            return err;
        }

        cf_ctl_dev.isPowerOn = 0;
    }
    else {
        xprintk(KERN_ERR, "Ignore power status change from %d to %d\n",
                on, cf_ctl_dev.isPowerOn);
    }*/

    return err;
}

static int cf_ctl_device_reset(unsigned char th)
{
    int err = 0;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);

    if (cf_ctl_dev.reset_num == 0) {
        xprintk(KERN_ERR, "cf_ctl_dev.reset_num is not get.\n");
        return -1;
    }

    gpio_direction_output(cf_ctl_dev.reset_num, 1);
    msleep(1);
	

	gpio_set_value(cf_ctl_dev.reset_num, th);


    return err;
}

static void cf_ctl_device_event(struct work_struct* ws)
{
    struct cf_ctl_device* cf_ctl_dev =
        container_of(ws, struct cf_ctl_device, work_queue);
    //char* uevent_env[2] = { "SPI_STATE=finger", NULL };
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    //kobject_uevent_env(&cf_ctl_dev->miscdev.this_device->kobj, KOBJ_CHANGE, uevent_env);
	if(NULL != cf_ctl_dev->async );
		kill_fasync(&cf_ctl_dev->async,SIGIO,POLL_IN);
}

static irqreturn_t cf_ctl_device_irq(int irq, void* dev_id)
{
    struct cf_ctl_device* cf_ctl_dev = (struct cf_ctl_device*)dev_id;
    disable_irq_nosync(irq);
    xprintk(KERN_ERR, "%s(irq = %d, ..) toggled.\n", __FUNCTION__, irq);
    schedule_work(&cf_ctl_dev->work_queue);
#if ANDROID_WAKELOCK
    wake_lock_timeout(&cf_ctl_dev->wakelock, msecs_to_jiffies(5000));
#endif
    enable_irq(irq);
    return IRQ_HANDLED;
}

static int cf_ctl_report_key_event(struct input_dev* input, cf_key_event_t* kevent)
{
    int err = 0;
    unsigned int key_code = KEY_UNKNOWN;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);

    switch (kevent->key) {
		case CF_KEY_HOME:	key_code = KEY_HOME;   break;
		case CF_KEY_MENU:	key_code = KEY_MENU;   break;
		case CF_KEY_BACK:	key_code = KEY_BACK;   break;
		case CF_KEY_F18:	key_code = KEY_F18;    break;
		case CF_KEY_F19:	key_code = KEY_F19;    break;
		case CF_KEY_F20:	key_code = KEY_F20;    break;
		case CF_KEY_F21:	key_code = KEY_F21;    break;
		case CF_KEY_ENTER:	key_code = KEY_ENTER;  break;
		case CF_KEY_UP: 	key_code = KEY_UP;	   break;
		case CF_KEY_LEFT:	key_code = KEY_LEFT;   break;
		case CF_KEY_RIGHT:	key_code = KEY_RIGHT;  break;
		case CF_KEY_DOWN:	key_code = KEY_DOWN;   break;
		case CF_KEY_WAKEUP: key_code = KEY_WAKEUP; break;
		
		default: break;
    }

    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
	if(kevent->value == 2){
        input_report_key(input, key_code, 1);
        input_sync(input);	
        input_report_key(input, key_code, 0);
        input_sync(input);		
	}else{
        input_report_key(input, key_code, kevent->value);
        input_sync(input);		
	}
    xprintk(KERN_DEBUG, "%s(..) leave.\n", __FUNCTION__);
    return err;
}

static const char* cf_ctl_get_version(void)
{
    static char version[CF_DRV_VERSION_LEN] = {'\0', };
    strncpy(version, CF_DRV_VERSION, CF_DRV_VERSION_LEN);
    version[CF_DRV_VERSION_LEN - 1] = '\0';
    return (const char*)version;
}


static int cf_ctl_init_irq(void)
{
    int err = 0;
    //struct device_node* dev_node = NULL;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
	#if 0
    /* Initialize the INT pin. */
    err = pinctrl_select_state(cf_pinctrl, cf_pin_states[CF_PIN_STATE_INT_SET]);
    /* Get the irq number. */
    dev_node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint_dev");

    if (!dev_node) {
        xprintk(KERN_ERR, "of_find_compatible_node(..) failed.\n");
        return (-ENODEV);
    }

    cf_ctl_dev.irq_num = irq_of_parse_and_map(dev_node, 0);
    xprintk(KERN_INFO, "irq number is %d.\n", cf_ctl_dev.irq_num);
	#endif
    /* Register interrupt callback. */
    err = request_irq(gpio_to_irq(cf_ctl_dev.irq_num), cf_ctl_device_irq,
                      IRQF_TRIGGER_RISING | IRQF_ONESHOT, "cf-irq", (void*)&cf_ctl_dev);

    if (err) {
        xprintk(KERN_ERR, "request_irq(..) = %d.\n", err);
    }

    return err;
}


////////////////////////////////////////////////////////////////////////////////
extern void cf_spi_platform_free(void);

////////////////////////////////////////////////////////////////////////////////
// struct file_operations fields.

static long cf_ctl_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
    struct miscdevice* dev = (struct miscdevice*)filp->private_data;
    struct cf_ctl_device* cf_ctl_dev =
        container_of(dev, struct cf_ctl_device, miscdev);
    int err = 0;
	unsigned char th = 0;
    cf_key_event_t kevent;
    xprintk(KERN_DEBUG, "%s(cmd = 0x%08x, ..)\n", __FUNCTION__, cmd);

    switch (cmd) {
        case CF_IOC_INIT_GPIO: {
            xprintk(KERN_INFO, "CF_IOC_INIT_DRIVER.\n");
            cf_ctl_init_gpio();
            break;
        }

        case CF_IOC_DEINIT_GPIO: {
            xprintk(KERN_INFO, "CF_IOC_DEINIT_DRIVER.\n");
            cf_ctl_free_gpio();
            //cf_spi_platform_free();
            break;
        }

        case CF_IOC_RESET_DEVICE: {
			if(__get_user(th,(u8 __user*)arg)){
               xprintk(KERN_ERR, "copy_from_user(..) failed.\n");
                err = (-EFAULT);
                break;
			}
			
            cf_ctl_device_reset(th);
            break;
        }

        case CF_IOC_ENABLE_IRQ: {
            // TODO:
            break;
        }

        case CF_IOC_DISABLE_IRQ: {
            // TODO:
            break;
        }

        case CF_IOC_REQUEST_IRQ: {
            xprintk(KERN_INFO, "CF_IOC_REQUEST_IRQ.\n");
			cf_ctl_init_irq();

            break;
        }

        case CF_IOC_ENABLE_SPI_CLK: {
            // TODO:
            break;
        }

        case CF_IOC_DISABLE_SPI_CLK: {
            // TODO:
            break;
        }

        case CF_IOC_ENABLE_POWER: {
            // TODO:
            break;
        }

        case CF_IOC_DISABLE_POWER: {
            // TODO:
            break;
        }

        case CF_IOC_REPORT_KEY_EVENT: {
            if (copy_from_user(&kevent, (cf_key_event_t*)arg, sizeof(cf_key_event_t))) {
                xprintk(KERN_ERR, "copy_from_user(..) failed.\n");
                err = (-EFAULT);
                break;
            }

            err = cf_ctl_report_key_event(cf_ctl_dev->input, &kevent);
            break;
        }

        case CF_IOC_SYNC_CONFIG: {
            // TODO:
            break;
        }

        case CF_IOC_GET_VERSION: {
            if (copy_to_user((void*)arg, cf_ctl_get_version(), CF_DRV_VERSION_LEN)) {
                xprintk(KERN_ERR, "copy_to_user(..) failed.\n");
                err = (-EFAULT);
                break;
            }

            break;
        }

        default:
            err = (-EINVAL);
            break;
    }

    return err;
}

static int cf_ctl_open(struct inode* inode, struct file* filp)
{
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    return 0;
}

static int cf_ctl_release(struct inode* inode, struct file* filp)
{
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    return 0;
}

static int cf_ctl_fasync(int fd, struct file *fp, int mode)
{
	xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
	return fasync_helper(fd,fp,mode,&cf_ctl_dev.async);
}
////////////////////////////////////////////////////////////////////////////////



// see cf_spi.c
extern int  cf_spi_platform_init(void);
extern void cf_spi_platform_exit(void);

////////////////////////////////////////////////////////////////////////////////

static int cf_ctl_init_gpio_pins(void)
{
    int err = 0;
    //struct platform_device* pdev = NULL;
    struct device_node* dev_node = NULL;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    dev_node = of_find_compatible_node(NULL, NULL, "qcom,fingerprint");

    if (!dev_node) {
        xprintk(KERN_ERR, "of_find_compatible_node(..) failed.\n");
        return (-ENODEV);
    }

    cf_ctl_dev.reset_num = of_get_named_gpio(dev_node, "qcom,reset-gpio", 0);
    cf_ctl_dev.irq_num = of_get_named_gpio(dev_node, "qcom,irq-gpio", 0);

    xprintk(KERN_INFO, "reset_gpio_number = %d\n", cf_ctl_dev.reset_num);
    xprintk(KERN_INFO, "irq_gpio_number = %d\n", cf_ctl_dev.irq_num);

#if LONGQI_HAL_COMPATIBLE
    xprintk(KERN_INFO, "=== Do not request gpio resources!!! ===\n");
#else
	cf_ctl_init_gpio();
#if 0
    if (gpio_is_valid(cf_ctl_dev.reset_num)) {
        err = gpio_request(cf_ctl_dev.reset_num, "cf-reset");

        if (err) {
            xprintk(KERN_ERR, "Could not request reset gpio.\n");
            return err;
        }
    }
    else {
        xprintk(KERN_ERR, "not valid reset gpio\n");
        return -EIO;
    }

    if (gpio_is_valid(cf_ctl_dev.irq_num)) {
        err = pinctrl_request_gpio(cf_ctl_dev.irq_num);

        if (err) {
            xprintk(KERN_ERR, "Could not request irq gpio.\n");
            gpio_free(cf_ctl_dev.reset_num);
            return err;
        }
    }
    else {
        xprintk(KERN_ERR, "not valid irq gpio\n");
        gpio_free(cf_ctl_dev.reset_num);
        return -EIO;
    }

    pinctrl_gpio_direction_input(cf_ctl_dev.irq_num);
#endif
	
#endif
/*
	cf_ctl_dev.pwr_num = of_get_named_gpio(dev_node, "qcom,pwr-gpio", 0);

    if (gpio_is_valid(cf_ctl_dev.pwr_num)) {
        err = gpio_request(cf_ctl_dev.pwr_num, "cf-pwr");

        if (err) {
            xprintk(KERN_ERR, "Could not request pwr gpio.\n");
            return err;
        }
    }
    else {
        xprintk(KERN_ERR, "not valid pwr gpio\n");
        return -EIO;
    }
    gpio_direction_output(cf_ctl_dev.pwr_num, 1);
*/
	/*pdev = of_find_device_by_node(dev_node);

    if (!pdev) {
        xprintk(KERN_ERR, "of_find_device_by_node(..) failed.\n");
        return (-ENODEV);
    }
    
    cf_ctl_dev.vdd = regulator_get(&pdev->dev, "vdd");

    if (IS_ERR(cf_ctl_dev.vdd)) {
        err = PTR_ERR(cf_ctl_dev.vdd);
        xprintk(KERN_ERR, "Regulator get failed vdd err = %d\n", err);
        gpio_free(cf_ctl_dev.reset_num);
        pinctrl_free_gpio(cf_ctl_dev.irq_num);
        return err;
    }

    if (regulator_count_voltages(cf_ctl_dev.vdd) > 0) {
        err = regulator_set_voltage(cf_ctl_dev.vdd, CF_VDD_MIN_UV,
                                    CF_VDD_MAX_UV);

        if (err) {
            xprintk(KERN_ERR, "Regulator set_vtg failed vdd err = %d\n", err);
            gpio_free(cf_ctl_dev.reset_num);
            pinctrl_free_gpio(cf_ctl_dev.irq_num);
            regulator_put(cf_ctl_dev.vdd);
            return err;
        }
    }*/

    return err;
}



static int cf_ctl_init_input(void)
{
    int err = 0;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    cf_ctl_dev.input = input_allocate_device();

    if (!cf_ctl_dev.input) {
        xprintk(KERN_ERR, "input_allocate_device(..) failed.\n");
        return (-ENOMEM);
    }

    cf_ctl_dev.input->name = "cf-keys";
    __set_bit(EV_KEY  , cf_ctl_dev.input->evbit );
    __set_bit(KEY_HOME, cf_ctl_dev.input->keybit);
    __set_bit(KEY_MENU, cf_ctl_dev.input->keybit);
    __set_bit(KEY_BACK, cf_ctl_dev.input->keybit);
	__set_bit(KEY_F18, cf_ctl_dev.input->keybit);
	__set_bit(KEY_F19, cf_ctl_dev.input->keybit);
	__set_bit(KEY_F20, cf_ctl_dev.input->keybit);
	__set_bit(KEY_F21, cf_ctl_dev.input->keybit);
	__set_bit(KEY_ENTER, cf_ctl_dev.input->evbit );
    __set_bit(KEY_UP, cf_ctl_dev.input->keybit);
    __set_bit(KEY_LEFT, cf_ctl_dev.input->keybit);
    __set_bit(KEY_RIGHT, cf_ctl_dev.input->keybit);
	__set_bit(KEY_DOWN, cf_ctl_dev.input->keybit);
	__set_bit(KEY_WAKEUP, cf_ctl_dev.input->keybit);

    err = input_register_device(cf_ctl_dev.input);

    if (err) {
        xprintk(KERN_ERR, "input_register_device(..) = %d.\n", err);
        input_free_device(cf_ctl_dev.input);
        cf_ctl_dev.input = NULL;
        return (-ENODEV);
    }

    xprintk(KERN_DEBUG, "%s(..) leave.\n", __FUNCTION__);
    return err;
}

static int __init cf_ctl_driver_init(void)
{
    int err = 0;
    /* Initialize the GPIO pins. */
    err = cf_ctl_init_gpio_pins();

    if (err) {
        xprintk(KERN_ERR, "cf_ctl_init_gpio_pins failed with %d.\n", err);
        return err;
    }
#if LONGQI_HAL_COMPATIBLE
	xprintk(KERN_INFO, "=== Do not cf_ctl_init_irq!!! ===\n");
#else
    /* Initialize the interrupt callback. */
    err = cf_ctl_init_irq();
    if (err) {
        xprintk(KERN_ERR, "cf_ctl_init_irq failed with %d.\n", err);
        return err;
    }
#endif
    /* Initialize the input subsystem. */
    err = cf_ctl_init_input();

    if (err) {
        xprintk(KERN_ERR, "cf_ctl_init_input failed with %d.\n", err);
        //free_irq(cf_ctl_dev.irq_num, (void*)&cf_ctl_dev);
        return err;
    }

    err = cf_ctl_device_power(true);
    /* Register as a miscellaneous device. */
    err = misc_register(&cf_ctl_dev.miscdev);

    if (err) {
        xprintk(KERN_ERR, "misc_register(..) = %d.\n", err);
        input_unregister_device(cf_ctl_dev.input);
        //free_irq(cf_ctl_dev.irq_num, (void*)&cf_ctl_dev);
        return err;
    }

#if ANDROID_WAKELOCK
	wake_lock_init(&cf_ctl_dev.wakelock, WAKE_LOCK_SUSPEND, "chipsailing_intr");
#endif

    INIT_WORK(&cf_ctl_dev.work_queue, cf_ctl_device_event);
    //err = cf_spi_platform_init();
	//full_fp_chip_name("chipsailing_fp");
    xprintk(KERN_INFO, "chipsailing fingerprint device control driver registered.\n");
    xprintk(KERN_INFO, "driver version: '%s'.\n", cf_ctl_get_version());
    return err;
}

static void __exit cf_ctl_driver_exit(void)
{
    if (cf_ctl_dev.input) {
        input_unregister_device(cf_ctl_dev.input);
    }

    if (cf_ctl_dev.irq_num >= 0) {
		pinctrl_free_gpio(cf_ctl_dev.irq_num);
        free_irq(gpio_to_irq(cf_ctl_dev.irq_num), (void*)&cf_ctl_dev);
    }

	if (cf_ctl_dev.reset_num >= 0) {
        gpio_free(cf_ctl_dev.reset_num);
    }
	
    misc_deregister(&cf_ctl_dev.miscdev);
    //cf_spi_platform_exit();
#if ANDROID_WAKELOCK
    wake_lock_destroy(&cf_ctl_dev.wakelock);
#endif
    xprintk(KERN_INFO, "chipsailing fingerprint device control driver released.\n");
}

module_init(cf_ctl_driver_init);
module_exit(cf_ctl_driver_exit);

MODULE_DESCRIPTION("The device control driver for Chipsailing's fingerprint sensor.");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Chipsailing");

