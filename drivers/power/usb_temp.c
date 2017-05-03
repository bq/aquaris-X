#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/notifier.h>
#include <linux/wakelock.h>
#include <linux/timer.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/qpnp/qpnp-adc.h>


struct usb_temp_device_info {
	struct device   *dev;
	struct workqueue_struct *usb_temp_wq;
	struct work_struct usb_temp_check_wk;
	struct notifier_block   usb_nb;
	struct power_supply     *usb_psy;
	struct power_supply     *batt_psy;
	struct power_supply     *bms_psy;
	struct hrtimer timer;
	int gpio_usb_temp;
	int need_protect_temp;
	int need_recover_temp;
	int open_mosfet_temp;
	int close_mosfet_temp;
	int interval_switch_temp;
	int check_interval;
	int keep_check_cnt;
	struct regulator	*vdd;
	struct qpnp_vadc_chip	*vadc_dev;
	int no_need_usb_temp;
};

#define		INVALID_DELAY_TIME			0
#define		USB_TEMP_INSERT_CHG_CNT			1100
#define		USB_TEMP_START_CHK_CNT			0
#define		USB_TEMP_END_CHK_CNT			1001
#define		USB_TEMP_CNT				2
#define		FAST_MONITOR_INTERVAL			300
#define		NORMAL_MONITOR_INTERVAL			1000
#define		INVALID_DELTA_TEMP			0
#define		USB_TEMP_CHK_CNT_STEP			1
#define		USB_TEMP_DEFAULT_CHK_CNT		(-1)
#define		GPIO_HIGH				1
#define		GPIO_LOW				0
#define		INTERVAL_0				0
#define		INVALID_BATT_TEMP			(-255)
#define		COVERSE_TEMP_UNIT			10
#define		TUSB_TEMP_UPPER_LIMIT			100
#define		TUSB_TEMP_LOWER_LIMIT			(-30)
#define		TRUE					1
#define		FALSE					0

#undef	pr_info
#define	pr_info	pr_debug

static int protect_enable = FALSE;
static int protect_dmd_notify_enable = TRUE;
static int is_usb_protect_mode = 0;


static struct usb_temp_device_info* g_di = NULL;
static struct wake_lock usb_temp_wakelock;
extern int charger_register_notifier(struct notifier_block *nb);
int get_power_supply_info(struct power_supply *psy, int prop)
{
	union power_supply_propval ret = {0, };

	if ((g_di == NULL) || (g_di->batt_psy == NULL)
		|| (g_di->bms_psy == NULL) || (g_di->usb_psy == NULL)) {
		pr_err("usb_temp.g_di is NULL!\n");
		return -EINVAL;
	}

	switch (prop) {
	case POWER_SUPPLY_PROP_TYPE:
		g_di->usb_psy->get_property(g_di->usb_psy, prop, &ret);
		pr_info("usb_temp.charger type = %d\n", ret.intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		g_di->batt_psy->get_property(g_di->batt_psy, prop, &ret);
		pr_info("usb_temp.battery persent = %d\n", ret.intval);
		break;
	default:
		return -1;
	}
	return ret.intval;
}

#define UTP_VTG_MIN_UV		1800000
#define UTP_VTG_MAX_UV		1800000
static bool enable_vdd_for_adc(struct usb_temp_device_info *di)
{
	int rc;
	static int set_once = 0;

	if (set_once)
		return 1;

	di->vdd = devm_regulator_get(di->dev, "vdd");
	if (IS_ERR(di->vdd)) {
		pr_err("usb_temp.Regulator get failed vdd rc=%d\n", rc);
		goto get_vdd_failed;
	}

	if (regulator_count_voltages(di->vdd) > 0) {
		rc = regulator_set_voltage(di->vdd, UTP_VTG_MIN_UV,
					   UTP_VTG_MAX_UV);
		if (rc) {
			pr_err("usb_temp.Regulator set_vtg failed vdd rc=%d\n", rc);
		}
	}

	rc = regulator_enable(di->vdd);
	if (rc) {
		pr_err("usb_temp.Regulator vdd enable failed rc=%d\n", rc);
	}

	set_once ++;
	pr_err("usb_temp.enable vdd for adc success\n");
	return 1;

get_vdd_failed:
	return 0;
}

static bool is_factory_mode = false;

static int __init early_parse_factory_mode(char *cmdline)
{
	if ((cmdline) && !strncmp(cmdline, "ffbm", strlen("ffbm"))) {
		is_factory_mode = true;
	}

	return 0;
}
early_param("androidboot.mode", early_parse_factory_mode);

static void usb_temp_wake_lock(void)
{
	if(!wake_lock_active(&usb_temp_wakelock))
	{
		pr_info("usb_temp.wake lock\n");
		wake_lock(&usb_temp_wakelock);
	}
}

static void usb_temp_wake_unlock(void)
{
	if(wake_lock_active(&usb_temp_wakelock))
	{
		pr_info("usb_temp.wake unlock\n");
		wake_unlock(&usb_temp_wakelock);
	}
}

static void charge_type_handler(struct usb_temp_device_info* di, enum power_supply_type type)
{
	int interval = 0;
	static char first_time = 1;

	if ((!protect_enable)||(NULL == di))
		return;

	if ((POWER_SUPPLY_TYPE_USB_DCP == type) || (POWER_SUPPLY_TYPE_USB == type)
			|| (POWER_SUPPLY_TYPE_USB_CDP == type)
			|| (POWER_SUPPLY_TYPE_USB_HVDCP == type)
			|| (POWER_SUPPLY_TYPE_USB_HVDCP_3 == type))
	{
		if (hrtimer_active(&(di->timer)))
		{
			pr_err("usb_temp.timer already working , do nothing\n");
		}
		else
		{
			pr_err("usb_temp.start usb_temp check\n");
			interval = INTERVAL_0;
			/*record 30 seconds after the charger just insert; 30s = (1100 - 1001 + 1)*300ms */
			di->keep_check_cnt = USB_TEMP_INSERT_CHG_CNT;
			if (1 == first_time){//first time delay 10s for power on plug usb
			  hrtimer_start(&di->timer, ktime_set(10000/MSEC_PER_SEC, (10000 % MSEC_PER_SEC) * USEC_PER_SEC), HRTIMER_MODE_REL);
			  first_time = 0;
			}else{
			  hrtimer_start(&di->timer, ktime_set(interval/MSEC_PER_SEC, (interval % MSEC_PER_SEC) * USEC_PER_SEC), HRTIMER_MODE_REL);
			}
		}
	}
	else
	{
		pr_err("usb_temp.charger type = %d, do nothing\n", type);
	}
}

static int usb_notifier_call(struct notifier_block *usb_nb, unsigned long event, void *data)
{
	struct usb_temp_device_info *di = container_of(usb_nb, struct usb_temp_device_info, usb_nb);
	enum power_supply_type type = ((enum power_supply_type)event);
	pr_err("usb_temp.%s:%d\n", __func__, type);
	
	charge_type_handler(di, type);
	return NOTIFY_OK;
}

static int get_usb_temp_value(struct usb_temp_device_info* di)
{
	int ret = 0;
	struct qpnp_vadc_result results;

	ret = qpnp_vadc_read(di->vadc_dev,P_MUX4_1_1, &results);
	if (ret) {
		pr_err("usb_temp.Unable to read usb temperature rc=%d\n", ret);
		return ret;
	}
	pr_info("usb_temp.get_usb_temp_value %d, %lld\n", results.adc_code,
							results.physical);

	return (int)results.physical;
}

static int get_batt_temp_value(void)
{
	int rc = 0;
	union power_supply_propval ret = {0, };
	
	if((g_di == NULL)||(g_di->batt_psy == NULL) ||(g_di->bms_psy == NULL))
	{
		pr_err("usb_temp. %s g_di is NULL!\n",__func__);
		return INVALID_BATT_TEMP;
	}

	rc = g_di->batt_psy->get_property(g_di->batt_psy, POWER_SUPPLY_PROP_TEMP, &ret);
	if(rc)
	{
		pr_err("usb_temp. %s  get temp error!\n",__func__);
		return INVALID_BATT_TEMP;
	}

	pr_info("usb_temp.the battery temperature is %d\n", ret.intval);
	return ret.intval/COVERSE_TEMP_UNIT;

}

static void set_interval(struct usb_temp_device_info* di, int temp)
{
	if(NULL == di)
	{
		pr_err("usb_temp. %s di is NULL!\n",__func__);
		return;
	}

	if (temp > di->interval_switch_temp) {
		di->check_interval = FAST_MONITOR_INTERVAL;
		di->keep_check_cnt = USB_TEMP_START_CHK_CNT;
		is_usb_protect_mode = TRUE;
		pr_info("usb_temp.cnt = %d!\n", di->keep_check_cnt);
	} else {
		if (di->keep_check_cnt > USB_TEMP_END_CHK_CNT) {
			/*check the temperature per 0.3 second for 100 times ,when the charger just insert.*/
			pr_info("usb_temp.cnt = %d!\n", di->keep_check_cnt);
			di->keep_check_cnt -= USB_TEMP_CHK_CNT_STEP;
			di->check_interval = FAST_MONITOR_INTERVAL;
			is_usb_protect_mode = FALSE;
		} else if (di->keep_check_cnt == USB_TEMP_END_CHK_CNT) {
			/* reset the flag when the temperature status is stable*/
			pr_info("usb_temp.cnt = %d!\n", di->keep_check_cnt);
			di->keep_check_cnt = USB_TEMP_DEFAULT_CHK_CNT;
			di->check_interval = NORMAL_MONITOR_INTERVAL;
			is_usb_protect_mode = FALSE;
			usb_temp_wake_unlock();
		} else if (di->keep_check_cnt >= USB_TEMP_START_CHK_CNT) {
			pr_info("usb_temp.cnt = %d!\n", di->keep_check_cnt);
			di->keep_check_cnt = di->keep_check_cnt + USB_TEMP_CHK_CNT_STEP;
			di->check_interval = FAST_MONITOR_INTERVAL;
			is_usb_protect_mode = TRUE;
		} else {
			di->check_interval = NORMAL_MONITOR_INTERVAL;
			is_usb_protect_mode = FALSE;
		}
	}
}
static void protection_process(struct usb_temp_device_info* di, int temp, int usb_temp)
{
	int gpio_value = 0;
	
	if(NULL == di)
	{
		pr_err("usb_temp. %s di is NULL!\n",__func__);
		return;
	}
	
	gpio_value = gpio_get_value(di->gpio_usb_temp);
	if ((temp >= di->open_mosfet_temp) && (usb_temp >= di->need_protect_temp)){
		usb_temp_wake_lock();
		gpio_set_value(di->gpio_usb_temp, GPIO_HIGH);/*open mosfet*/
		pr_err("usb_temp.temp is wrong, pull pull gpio114. gpio114 value is:%d\n",gpio_value);
	} else if ((temp <= di->close_mosfet_temp) && (usb_temp <= di->need_recover_temp)){
		gpio_set_value(di->gpio_usb_temp, GPIO_LOW);/*close mosfet*/
		pr_info("usb_temp.temp is normal, pull down gpio114. gpio114 value is:%d\n",gpio_value);
	} else {
		/*do nothing*/
	}
}
static void check_temperature(struct usb_temp_device_info* di)
{
	int tusb = 0;
	int tbatt = 0;
	int tdiff = 0;
	
	if(NULL == di)
	{
		pr_err("usb_temp. %s di is NULL!\n", __func__);
		return;
	}
	
	tusb = get_usb_temp_value(di);
	tbatt = get_batt_temp_value();
	
	pr_info("usb_temp.tusb = %d, tbatt = %d\n", tusb, tbatt);
	tdiff = tusb - tbatt;
	if(INVALID_BATT_TEMP == tbatt)
	{
		tdiff = INVALID_DELTA_TEMP;
		pr_err("usb_temp.get battery adc temp err, not care!!!\n");
	}
	
	if (tdiff >= di->open_mosfet_temp) {
		// try do something
	}
	
	set_interval(di, tdiff);
	protection_process(di, tdiff, tusb);
}
static void usb_temp_check_work(struct work_struct *work)
{
	struct usb_temp_device_info *di = container_of(work,struct usb_temp_device_info, usb_temp_check_wk);
	int interval = 0;
	int type = 0;
	type = get_power_supply_info(di->usb_psy, POWER_SUPPLY_PROP_TYPE);
	
#ifdef CONFIG_HLTHERM_RUNTEST
		pr_info("usb_temp.Disable HLTHERM protect\n");
		return;
#endif
	
	if (((USB_TEMP_DEFAULT_CHK_CNT == di->keep_check_cnt) && (POWER_SUPPLY_TYPE_UNKNOWN == type)))
	{
		protect_dmd_notify_enable = TRUE;
		gpio_set_value(di->gpio_usb_temp, GPIO_LOW);//close mosfet
		di->keep_check_cnt = USB_TEMP_DEFAULT_CHK_CNT;
		di->check_interval = NORMAL_MONITOR_INTERVAL;
		is_usb_protect_mode = FALSE;
		di->keep_check_cnt = USB_TEMP_INSERT_CHG_CNT;
		pr_err("usb_temp.chargertype is %d,stop checking\n", type);
		return;
	}
	
	check_temperature(di);
	interval = di->check_interval;
	
	hrtimer_start(&di->timer, ktime_set(interval/MSEC_PER_SEC, (interval % MSEC_PER_SEC) * USEC_PER_SEC), HRTIMER_MODE_REL);

}

static enum hrtimer_restart usb_temp_timer_func(struct hrtimer *timer)
{
	struct usb_temp_device_info *di = NULL;
	
	di = container_of(timer, struct usb_temp_device_info, timer);
	queue_work(di->usb_temp_wq, &di->usb_temp_check_wk);
	return HRTIMER_NORESTART;
}

static void check_ntc_error(struct usb_temp_device_info* di)
{
	int temp = 0;
	int sum = 0;
	int i = 0;
	
	for (i = 0; i < USB_TEMP_CNT; ++i)
	{
		sum += get_usb_temp_value(di);
	}
	temp = sum / USB_TEMP_CNT;
	if (temp > TUSB_TEMP_UPPER_LIMIT || temp < TUSB_TEMP_LOWER_LIMIT) {
		protect_enable = FALSE;
	}
	else {
		pr_info("usb_temp.enable usb short protect\n");
		protect_enable = TRUE;
	}
}
static int usb_temp_probe(struct platform_device *pdev)
{
	struct device_node* np = NULL;
	struct usb_temp_device_info* di = NULL;
	enum power_supply_type type = POWER_SUPPLY_TYPE_UNKNOWN;
	struct power_supply *usb_psy = NULL;
	struct power_supply *batt_psy = NULL;
	struct power_supply *bms_psy = NULL;
	int ret = 0;
	int batt_present = TRUE;
	int need_usb_temp = TRUE;
	int gpio_127,gpio_128;

	gpio_127 = gpio_get_value(127);
	gpio_128 = gpio_get_value(128);
	if((0 == gpio_127)&&(1 == gpio_128))
		return 0;

	printk("usb_temp.enter into usb_temp probe\n");
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("usb_temp.usb supply not found deferring probe\\n");
		return -EPROBE_DEFER;
	}
	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		pr_err("usb_temp.batt supply not found deferring probe\\n");
		return -EPROBE_DEFER;
	}
	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		pr_err("usb_temp.bms supply not found deferring probe\\n");
		return -EPROBE_DEFER;
	}
	
	np = pdev->dev.of_node;
	if(NULL == np)
	{
		pr_err("usb_temp.np is NULL\\n");
		return -1;
	}
	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
	{
		pr_err("usb_temp.di is NULL\n");
		return -ENOMEM;
	
	}
	di->dev = &pdev->dev;
	dev_set_drvdata(&(pdev->dev), di);
	g_di = di;

	di->usb_psy = usb_psy;
	di->batt_psy = batt_psy;
	di->bms_psy = bms_psy;
	is_usb_protect_mode = FALSE;
	di->keep_check_cnt = USB_TEMP_INSERT_CHG_CNT;

	enable_vdd_for_adc(di);
	di->vadc_dev = qpnp_get_vadc(di->dev, "usb_temp");
	if (IS_ERR(di->vadc_dev)) {
		pr_err("usb_temp.vadc is not valid\n");
		return ret;
	}

	di->gpio_usb_temp = of_get_named_gpio(np, "shamrock,gpio_usb_temp_protect", 0);
	if (!gpio_is_valid(di->gpio_usb_temp))
	{
		pr_err("usb_temp.gpio_usb_temp is not valid\n");
		ret = -EINVAL;
		goto free_mem;
	}
	pr_info("usb_temp.gpio_usb_temp = %d\n", di->gpio_usb_temp);
	
	ret = gpio_request(di->gpio_usb_temp, "usb_temp_protect");
	if (ret)
	{
		pr_err("usb_temp.could not request gpio_usb_temp\n");
		ret = -EINVAL;
		goto free_mem;
	}
	gpio_direction_output(di->gpio_usb_temp, GPIO_LOW);
	
	ret = of_property_read_u32(np, "shamrock,no_need_usb_temp", &(di->no_need_usb_temp));
	if (ret)
	{
		pr_err("usb_temp.get shamrock,no_need_usb_temp fail!\n");
	}
	pr_info("usb_temp.no_need_usb_temp = %d\n", di->no_need_usb_temp);

	ret = of_property_read_u32(np, "shamrock,need_protect_temp", &(di->need_protect_temp));
	if (ret)
	{
		pr_err("usb_temp.get need_protect_temp info fail!\n");
		ret = -EINVAL;
		goto free_gpio;
	}
	pr_info("usb_temp.need_protect_temp = %d\n", di->need_protect_temp);

	ret = of_property_read_u32(np, "shamrock,need_recover_temp", &(di->need_recover_temp));
	if (ret)
	{
		pr_err("usb_temp.get need_recover_temp info fail!\n");
		ret = -EINVAL;
		goto free_gpio;
	}
	pr_info("usb_temp.need_recover_temp = %d\n", di->need_recover_temp);

	ret = of_property_read_u32(np, "shamrock,open_mosfet_temp", &(di->open_mosfet_temp));
	if (ret)
	{
		pr_err("usb_temp.get open_mosfet_temp info fail!\n");
		ret = -EINVAL;
		goto free_gpio;
	}
	pr_info("usb_temp.open_mosfet_temp = %d\n", di->open_mosfet_temp);

	ret = of_property_read_u32(np, "shamrock,close_mosfet_temp", &(di->close_mosfet_temp));
	if (ret)
	{
		pr_err("usb_temp.get close_mosfet_temp info fail!\n");
		ret = -EINVAL;
		goto free_gpio;
	}
	pr_info("usb_temp.close_mosfet_temp = %d\n", di->close_mosfet_temp);

	ret = of_property_read_u32(np, "shamrock,interval_switch_temp", &(di->interval_switch_temp));
	if (ret)
	{
		pr_err("usb_temp.get interval_switch_temp info fail!\n");
		ret = -EINVAL;
		goto free_gpio;
	}
	pr_info("usb_temp.interval_switch_temp = %d\n", di->interval_switch_temp);
	
	check_ntc_error(di);
	batt_present = get_power_supply_info(batt_psy, POWER_SUPPLY_PROP_PRESENT);
	
	if(is_factory_mode || (di->no_need_usb_temp == TRUE))
	{
		need_usb_temp = FALSE;
	}
	
	if ((!batt_present) || (FALSE == need_usb_temp)) {
		pr_err("usb_temp.battery is not exist or no need usb_temp in factory mode, disable usb short protect!\n");
		protect_enable = FALSE;
	}
	if (!protect_enable)
	{
		goto free_gpio;
	}
	wake_lock_init(&usb_temp_wakelock, WAKE_LOCK_SUSPEND, "usb_temp_protect_wakelock");
	di->usb_temp_wq = create_singlethread_workqueue("usb_temp_protect_wq");
	INIT_WORK(&di->usb_temp_check_wk, usb_temp_check_work);
	hrtimer_init(&di->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	di->timer.function = usb_temp_timer_func;
	di->usb_nb.notifier_call = usb_notifier_call;
	ret = charger_register_notifier(&di->usb_nb);
	if (ret < 0)
	{
		pr_err("usb_temp.charger_type_notifier_register failed\\n");
		ret = -EINVAL;
		goto free_gpio;
	}
	
	type = get_power_supply_info(usb_psy, POWER_SUPPLY_PROP_TYPE);
	
	pr_info("usb_temp.usb type = %d\n", type);
	charge_type_handler(di, type);
	
	pr_info("usb_temp.usb_temp probe ok!\n");
	return 0;
	
	free_gpio:
	gpio_free(di->gpio_usb_temp);
	free_mem:
	kfree(di);
	g_di = NULL;

	return ret;
}

static int usb_temp_remove(struct platform_device *pdev)
{
	struct usb_temp_device_info *di = dev_get_drvdata(&pdev->dev);
	
	gpio_free(di->gpio_usb_temp);
	kfree(di);
	g_di = NULL;
	
	return 0;
}
static struct of_device_id usb_temp_match_table[] =
{
	{
		.compatible = "shamrock,usb_temp_protect",
		.data = NULL,
	},
	{
	},
};
static struct platform_driver usb_temp_driver = {
	.probe = usb_temp_probe,
	.remove = usb_temp_remove,
	.driver = {
		.name = "shamrock,usb_temp_protect",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(usb_temp_match_table),
},
};

static int __init usb_temp_init(void)
{
	return platform_driver_register(&usb_temp_driver);
}

device_initcall_sync(usb_temp_init);

static void __exit usb_temp_exit(void)
{
	platform_driver_unregister(&usb_temp_driver);
}

module_exit(usb_temp_exit); 
