/*
 * es9118.c -- es9118 ALSA SoC audio driver
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <trace/events/asoc.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/wakelock.h>

#define INPUT_CONFIG_SOURCE 1
#define I2S_BIT_FORMAT_MASK (0x03 << 6)
#define GENERAL_SETTINGS  7
#define I2S_CHANNEL_MUTE_MASK 0x01
#define MASTER_MODE_CONTROL 10
#define MASTER_MODE_MASK (0x01 << 7)
#define I2S_CLK_DIVID_MASK (0x03 << 5)
#define RIGHT_CHANNEL_VOLUME_15 15
#define LEFT_CHANNEL_VOLUME_16 16
#define MASTER_TRIM_VOLUME_17 17
#define MASTER_TRIM_VOLUME_18 18
#define MASTER_TRIM_VOLUME_19 19
#define MASTER_TRIM_VOLUME_20 20
#define HEADPHONE_AMPLIFIER_CONTROL 42

struct es9118_priv {
	struct snd_soc_codec *codec;
	struct i2c_client *i2c_client;
	struct es9118_data *es9118_data;
	struct delayed_work sleep_work;
} es9118_priv;

struct es9118_data {
	int reset_gpio;
	int power_gpio;
	int cycle_gpio;	
	int enable_gpio;
	int mode;
	bool b_power_on;
	bool b_cycle_on;	
	bool b_closing;
	atomic_t clk_count_ref;
	long master_value;
	long divider_value;
	long i2s_length_value;
	struct delayed_work power_off_dwork;
};

struct es9118_reg {
	unsigned char num;
	unsigned char value;
};

static struct wake_lock es9118_wakelock;
static struct mutex es9118_lock;
static struct delayed_work clk_divider_dwork;
static int es9118_reg_no = 0;
#define POWER_SUPPLY_TIMEOUT 5000
#define CLK_DIVIDER_TIMEOUT 100000


/* We only include the analogue supplies here; the digital supplies
 * need to be available well before this driver can be probed.
 */
struct es9118_reg init_reg_es9118[] = {
	{0x00, 0x00},
	{0x01, 0xc0}, 
	{0x02, 0xb4},
	{0x03, 0x47},
	{0x04, 0x02},
	{0x05, 0x7a},
	{0x06, 0x02}, 
	{0x07, 0x80},
	{0x08, 0x40},
	{0x09, 0x22},
	{0x0a, 0x02},//0x82:master, 0x02:slave},
	{0x0b, 0x00},
	{0x0c, 0x1a},
	{0x0d, 0x00},
	{0x0e, 0x0a},
	{0x0f, 0x0c},
	{0x10, 0x0c},
	{0x11, 0xff},
	{0x12, 0xff},
	{0x13, 0xff},
	{0x14, 0x76},
	{0x15, 0x08},
	{0x16, 0x28},
	{0x17, 0x01},
	{0x18, 0xbe},
	{0x19, 0xff},
	{0x1a, 0x62},
	{0x1b, 0xd4},
	{0x1c, 0xf0},
	{0x1d, 0x0d},
	{0x1e, 0x00},
	{0x1f, 0x00},
	{0x20, 0x00}
};

static int es9118_register_dump = -1;
static struct es9118_priv *g_es9118_priv = NULL;
static int es9118_write_reg(struct i2c_client *client, int reg, u8 value);
static int es9118_read_reg(struct i2c_client *client, int reg);

#define es9118_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |	\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |	\
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |	\
		SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000)

#define es9118_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE | \
		SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S20_3BE | \
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_BE | \
		SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S32_BE)

static int es9118_register_dump_parm_set(const char *val, struct kernel_param *kp)
{
	int i, reg_val;
	param_set_int(val, kp);

	if (1 == es9118_register_dump) {
		for (i=0; i< 51; i++) {
			reg_val = es9118_read_reg(g_es9118_priv->i2c_client, i);
			pr_info("%s: regester[%#x] = %#x\n",
				__func__, i, reg_val);
			udelay(20);
		}
	} else {
		pr_info("enter %s, out of register dump!\n", __func__);
	}
	return 0;
}
module_param_call(es9118_register_dump, es9118_register_dump_parm_set,
			param_get_int, &es9118_register_dump, 0664);

static void es9118_update_register(u8 reg, u8 value)
{
	int i;
	int reg_num;
	
	reg_num = sizeof(init_reg_es9118)/sizeof(struct es9118_reg);
	for (i = 0; i < reg_num; i++) {
		if (reg == init_reg_es9118[i].num) {
			init_reg_es9118[i].value = value;
			break;
		}
	}
}

static void es9118_enable_gpio(int gpio, unsigned on)
{
	int ret;

	ret = gpio_direction_output(gpio, on);
	if (ret < 0) {
		pr_err("%s: set %d gpio %s failed\n",
				__func__, gpio, on ? "high" : "low");
		return;
	}
	pr_info("%s: set %d gpio %s success\n",
				__func__, gpio, on ? "high" : "low");
	return;
}

static void es9118_schedule_power_off(void)
{
    struct es9118_data *pdata = g_es9118_priv->es9118_data;
	
	pr_info("%s enter \n", __func__);
	
       mutex_lock(&es9118_lock);
	if  (pdata->b_power_on && (0 == atomic_read(&pdata->clk_count_ref))) {
	    if (pdata->b_closing) {
		    cancel_delayed_work(&pdata->power_off_dwork);
		} else {		   
		    pdata->b_closing = true;
		}
		schedule_delayed_work(&pdata->power_off_dwork,
			                  usecs_to_jiffies(POWER_SUPPLY_TIMEOUT * HZ));
	}
	mutex_unlock(&es9118_lock);
	
	return;
}

static void es9118_power_off_delayed_work(struct work_struct *work)
{
      struct es9118_data *es9118_ctrl = g_es9118_priv->es9118_data;

	pr_info("%s enter\n", __func__);
	
       mutex_lock(&es9118_lock);
	
	if (0 == atomic_read(&es9118_ctrl->clk_count_ref)) {
	    pr_info("%s: switch es9118 power off!\n", __func__);
		es9118_enable_gpio(es9118_ctrl->enable_gpio, 0);
		es9118_enable_gpio(es9118_ctrl->reset_gpio, 0);
		msleep(5);
		es9118_enable_gpio(es9118_ctrl->power_gpio, 0);
		es9118_enable_gpio(es9118_ctrl->cycle_gpio, 0);
	    es9118_ctrl->b_power_on = false;
	    es9118_ctrl->b_cycle_on = false;
	    wake_lock_timeout(&es9118_wakelock, 3*HZ);
	}
	es9118_ctrl->b_closing = false;
	
	mutex_unlock(&es9118_lock);
	
	return;
}

static int es9118_open(void)
{

	struct es9118_data *es9118_ctrl = g_es9118_priv->es9118_data;

	pr_info("%s: enter\n", __func__);
	
	mutex_lock(&es9118_lock);

	// Check if the mclk is in closing state
	if (es9118_ctrl->b_closing) {
		cancel_delayed_work(&es9118_ctrl->power_off_dwork);
		es9118_ctrl->b_closing = false;
	    pr_info("%s: cancel power off dwork\n",__func__);	
	}

   // Check power
	if (!es9118_ctrl->b_power_on) {
		pr_info("%s: switch es9118 power on\n", __func__);
		es9118_enable_gpio(es9118_ctrl->power_gpio, 1);
		msleep(20);
		es9118_ctrl->b_power_on = true;
	} else {
		pr_info("%s: es9118 has been power on!\n", __func__);
	}

       // Check whether the mclk has been enable or not
    es9118_enable_gpio(es9118_ctrl->enable_gpio, 0);
	   		// Check cycle
	if (!es9118_ctrl->b_cycle_on) {
		pr_info("%s: switch es9118 cycle on\n", __func__);
		es9118_enable_gpio(es9118_ctrl->cycle_gpio, 1);
		es9118_ctrl->b_cycle_on = true;
	} else {
		pr_info("%s: es9118 has been cycle on!\n", __func__);
	}
       es9118_enable_gpio(es9118_ctrl->reset_gpio, 1);
	if (atomic_inc_return(&es9118_ctrl->clk_count_ref) == 1) {
		msleep(5);
	} else {
		pr_info("%s: mclk 45M has been enabled!\n", __func__);
	}
	msleep(10);

	es9118_write_reg(g_es9118_priv->i2c_client,  1,  0x80);
	es9118_write_reg(g_es9118_priv->i2c_client,  12,  0xFA);

	es9118_write_reg(g_es9118_priv->i2c_client,  14,  0x45);
	es9118_write_reg(g_es9118_priv->i2c_client,  2,  0xB4);
	es9118_write_reg(g_es9118_priv->i2c_client,  5,  0x0);
	es9118_write_reg(g_es9118_priv->i2c_client,  4,  0xFF);
	es9118_write_reg(g_es9118_priv->i2c_client,  32,  0x80);
	msleep(5);
	es9118_write_reg(g_es9118_priv->i2c_client,  29,  0x0D);
	msleep(5);
	es9118_write_reg(g_es9118_priv->i2c_client,  46,  0x80);
	msleep(5);
	es9118_write_reg(g_es9118_priv->i2c_client,  32,  0x83);
	msleep(200);
	es9118_write_reg(g_es9118_priv->i2c_client,  46,  0x0);
	es9118_write_reg(g_es9118_priv->i2c_client,  5,  0x7F);
	msleep(5);
	es9118_write_reg(g_es9118_priv->i2c_client,  4,  0x02);
	es9118_write_reg(g_es9118_priv->i2c_client,  2,  0x04);
	es9118_write_reg(g_es9118_priv->i2c_client,  RIGHT_CHANNEL_VOLUME_15,
				init_reg_es9118[RIGHT_CHANNEL_VOLUME_15].value);
	es9118_write_reg(g_es9118_priv->i2c_client,  LEFT_CHANNEL_VOLUME_16,
				init_reg_es9118[LEFT_CHANNEL_VOLUME_16].value);
	es9118_write_reg(g_es9118_priv->i2c_client,  27,  0xC4);    //ABOVE  IS NO POP NOISE  SEQUENCE
	es9118_write_reg(g_es9118_priv->i2c_client,  13,  0x00);  //THD COMPEMSASTION
	es9118_write_reg(g_es9118_priv->i2c_client,  22,  0x28);
	es9118_write_reg(g_es9118_priv->i2c_client,  23,  0x01);
	es9118_write_reg(g_es9118_priv->i2c_client,  24,  0xBE);
	es9118_write_reg(g_es9118_priv->i2c_client,  25,  0xFF);

	es9118_write_reg(g_es9118_priv->i2c_client,  GENERAL_SETTINGS,
				init_reg_es9118[GENERAL_SETTINGS].value); //DIGITAL FILTERS

	es9118_ctrl->mode = 2;
	mutex_unlock(&es9118_lock);

	return 0;
}

static int es9118_close(void)
{    
    struct es9118_data *es9118_ctrl = g_es9118_priv->es9118_data;	

    pr_info("%s: enter mode:%d  cound:%d  ===\n", __func__, es9118_ctrl->mode, atomic_read(&es9118_ctrl->clk_count_ref) );

    mutex_lock(&es9118_lock);	

	if ((2 == es9118_ctrl->mode) && ((0 == atomic_read(&es9118_ctrl->clk_count_ref)) || (1 == atomic_read(&es9118_ctrl->clk_count_ref))))
	{
		pr_info("es9118_close set silent\n");
		es9118_write_reg(g_es9118_priv->i2c_client,  4,  0xFF);
		es9118_write_reg(g_es9118_priv->i2c_client,  15,  0xFF);
		es9118_write_reg(g_es9118_priv->i2c_client,  16,  0xFF);
		pr_info("es9118_close  4,  0xFF \n");
	}
    if (atomic_read(&es9118_ctrl->clk_count_ref) > 0)  {		
        if (atomic_dec_return(&es9118_ctrl->clk_count_ref) == 0) {	        
            pr_info("%s: mclk 45M clk_count_ref is 0\n", __func__);
	     es9118_ctrl->mode = 0;
	 }
    } else {
        es9118_ctrl->mode = 0;
    }
    mutex_unlock(&es9118_lock);
	
    pr_info("%s: mclk 45M clk_count_ref is %d\n", 
	 	__func__, atomic_read(&es9118_ctrl->clk_count_ref));				
    return 0;
}

static int es9118_bypass(void)
{
    struct es9118_data *es9118_ctrl = g_es9118_priv->es9118_data;

	pr_info("%s: enter\n", __func__);

	mutex_lock(&es9118_lock);

	// Check if the mclk is in closing state
	if (es9118_ctrl->b_closing) {
		cancel_delayed_work(&es9118_ctrl->power_off_dwork);
		es9118_ctrl->b_closing = false;
	    pr_info("%s: cancel power off dwork\n",__func__);	
	}
	if (atomic_inc_return(&es9118_ctrl->clk_count_ref) > 1) {
		pr_info("%s: es9118 has been enabled!\n", __func__);
	}

	if (es9118_ctrl->b_cycle_on) {
		pr_info("lj %s:  es9118 cycle on, turn off \n", __func__);
		es9118_enable_gpio(es9118_ctrl->cycle_gpio, 0);
		msleep(6);
		es9118_ctrl->b_cycle_on = false;
	} else {
		pr_info("lj %s: es9118 has been cycle on!\n", __func__);
	}

	// Check power
	if (!es9118_ctrl->b_power_on) {
		pr_info("%s: switch es9118 power on\n", __func__);
		es9118_enable_gpio(es9118_ctrl->power_gpio, 1);
		msleep(60);
		es9118_ctrl->b_power_on = true;
	} else {
		pr_info("%s: es9118 has been power on!\n", __func__);
	}
	
	es9118_enable_gpio(es9118_ctrl->reset_gpio, 0);

	es9118_enable_gpio(es9118_ctrl->enable_gpio, 1);

	es9118_ctrl->mode = 1;

	mutex_unlock(&es9118_lock);

	pr_info("%s: mclk 45M clk_count_ref is %d\n", __func__,
		                 atomic_read(&es9118_ctrl->clk_count_ref));
			
	return 0;
}

static int es9118_get_mode(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = g_es9118_priv->es9118_data->mode;

	pr_info("%s: mode = %ld\n", __func__, ucontrol->value.integer.value[0]);

	return 0;
}

static int es9118_set_mode(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	pr_info("%s: ucontrol->value.integer.value[0]  = %ld\n",
		__func__, ucontrol->value.integer.value[0]);

	switch(ucontrol->value.integer.value[0]) 
	{ 
	    case 1:
		 es9118_bypass();
	        break;
	    case 2:
		 es9118_open();
	        break;
	    case 0:
	    default:
		 es9118_close();
		 es9118_schedule_power_off();
	        break;
	}

	pr_info("%s: mode = %ld\n", __func__, ucontrol->value.integer.value[0]);
	
	return 0;
}

static int es9118_get_master(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u8 reg_val;
	int ret;

	ret = es9118_read_reg(g_es9118_priv->i2c_client,
				MASTER_MODE_CONTROL);
	if (ret < 0) {
		ucontrol->value.integer.value[0] = 0;
		return 0;
	}

	reg_val   = ret;
	reg_val &= MASTER_MODE_MASK;
	reg_val = reg_val >> 7;
	ucontrol->value.integer.value[0] = reg_val;

	pr_info("%s: master = %d\n", __func__, reg_val);

	return 0;
}

static int es9118_set_master(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u8 reg_val;
	struct es9118_data *es9118_ctrl = g_es9118_priv->es9118_data;

	pr_info("%s: ucontrol->value.integer.value[0]  = %ld\n",
		__func__, ucontrol->value.integer.value[0]);

	mutex_lock(&es9118_lock);
	es9118_ctrl->master_value = ucontrol->value.integer.value[0] << 7;
	if (es9118_ctrl->b_power_on) {
		reg_val = es9118_read_reg(g_es9118_priv->i2c_client,
					MASTER_MODE_CONTROL);

		reg_val &= ~(MASTER_MODE_MASK);
		reg_val |=  ucontrol->value.integer.value[0] << 7;

		reg_val &= ~(I2S_CLK_DIVID_MASK);
		reg_val |=  es9118_ctrl->divider_value;

		es9118_write_reg(g_es9118_priv->i2c_client,
					MASTER_MODE_CONTROL, reg_val);
	}
	mutex_unlock(&es9118_lock);

	pr_info("%s: new master = 0x%x\n", __func__, reg_val);
	return 0;
}


static int es9118_i2s_length_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u8 reg_val;
	int ret;

	ret = es9118_read_reg(g_es9118_priv->i2c_client,
				INPUT_CONFIG_SOURCE);
	if (ret < 0) {
		ucontrol->value.integer.value[0] = 3;
		return 0;
	}
	reg_val   = ret;
	reg_val &= I2S_BIT_FORMAT_MASK;
	reg_val = reg_val >> 6;
	ucontrol->value.integer.value[0] = reg_val;

	pr_info("%s: i2s_length = 0x%x\n", __func__, reg_val);

	return 0;
}

static int es9118_i2s_length_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u8 reg_val;
	struct es9118_data *es9118_ctrl = g_es9118_priv->es9118_data;

	pr_info("%s: ucontrol->value.integer.value[0]  = %ld\n",
		__func__, ucontrol->value.integer.value[0]);

       mutex_lock(&es9118_lock);
       es9118_ctrl->i2s_length_value = ucontrol->value.integer.value[0] << 6;
       if (es9118_ctrl->b_power_on) {
		reg_val = es9118_read_reg(g_es9118_priv->i2c_client,
					INPUT_CONFIG_SOURCE);

		reg_val &= ~(I2S_BIT_FORMAT_MASK);
		reg_val |=  ucontrol->value.integer.value[0] << 6;

		es9118_write_reg(g_es9118_priv->i2c_client,
					INPUT_CONFIG_SOURCE, reg_val);
       }
	mutex_unlock(&es9118_lock);

	pr_info("%s: i2s_length = 0x%x\n", __func__, reg_val);
	
	return 0;
}

static void clk_divider_delayed_work(struct work_struct *work)
{
	u8 reg_val;
	struct es9118_data *es9118_ctrl = g_es9118_priv->es9118_data;

	pr_info("%s enter\n", __func__);

	// Wait for mclk
	if (0 == atomic_read(&g_es9118_priv->es9118_data->clk_count_ref)) {
		msleep(50);
	}
	mutex_lock(&es9118_lock);
	if (es9118_ctrl->b_power_on) {
		reg_val = es9118_read_reg(g_es9118_priv->i2c_client,
					MASTER_MODE_CONTROL);

		reg_val &= ~(I2S_CLK_DIVID_MASK);
		reg_val |=  es9118_ctrl->divider_value;

		reg_val &= ~(MASTER_MODE_MASK);
		reg_val |=  es9118_ctrl->master_value;

		es9118_write_reg(g_es9118_priv->i2c_client,
					MASTER_MODE_CONTROL, reg_val);
	}
	mutex_unlock(&es9118_lock);

	return;
}

static int es9118_get_clk_divider(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u8 reg_val;
	int ret;

	ret = es9118_read_reg(g_es9118_priv->i2c_client,
				MASTER_MODE_CONTROL);
	if (ret < 0) {
		ucontrol->value.integer.value[0] = 0;
		return 0;
	}
	reg_val   = ret;
	reg_val &= I2S_CLK_DIVID_MASK;
	reg_val = reg_val >> 5;
	ucontrol->value.integer.value[0] = reg_val;

	pr_info("%s: clk_divider = 0x%x\n", __func__, reg_val);

	return 0;
}

static int es9118_set_clk_divider(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u8 reg_val;
	int ret;
	struct es9118_data *es9118_ctrl = g_es9118_priv->es9118_data;

	pr_info("%s: ucontrol->value.integer.value[0]  = %ld\n",
			__func__, ucontrol->value.integer.value[0]);

	mutex_lock(&es9118_lock);
	es9118_ctrl->divider_value = ucontrol->value.integer.value[0] << 5;
	if (es9118_ctrl->b_power_on) {
		reg_val = es9118_read_reg(g_es9118_priv->i2c_client,
			MASTER_MODE_CONTROL);
		if (g_es9118_priv->es9118_data->b_power_on && (reg_val < 0)) {
			pr_info("%s: read fail, scheduled!\n", __func__);
			schedule_delayed_work(&clk_divider_dwork,
					usecs_to_jiffies(CLK_DIVIDER_TIMEOUT));
			mutex_unlock(&es9118_lock);
			return 0;
		}

		reg_val &= ~(I2S_CLK_DIVID_MASK);
		reg_val |=  ucontrol->value.integer.value[0] << 5;

		reg_val &= ~(MASTER_MODE_MASK);
		reg_val |=  es9118_ctrl->master_value;

		ret = es9118_write_reg(g_es9118_priv->i2c_client,
				MASTER_MODE_CONTROL, reg_val);
		if (ret < 0) {
			pr_info("%s: write fail schedule!\n", __func__);
			schedule_delayed_work(&clk_divider_dwork,
					usecs_to_jiffies(CLK_DIVIDER_TIMEOUT));
	       }
	}
	mutex_unlock(&es9118_lock);

	pr_info("%s: clk_divider = 0x%x\n", __func__, reg_val);
	return 0;
}

static int es9118_get_boost(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int boost = 0;
	u8 reg_val;

	reg_val = es9118_read_reg(g_es9118_priv->i2c_client,
				LEFT_CHANNEL_VOLUME_16);
	if (reg_val == 1) {
		boost = 1;
	}
	ucontrol->value.integer.value[0] = boost;

	pr_info("%s: boost = %s\n", __func__, boost == 0 ? "Off" : "On");

	return 0;
}

static int es9118_set_boost(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int ret;
	int boost = ucontrol->value.integer.value[0];
	u8 reg_val = boost ? 1 : 12;

	ret = es9118_write_reg(g_es9118_priv->i2c_client,
			LEFT_CHANNEL_VOLUME_16, reg_val);
	es9118_update_register(LEFT_CHANNEL_VOLUME_16, reg_val);
	ret = es9118_write_reg(g_es9118_priv->i2c_client,
			RIGHT_CHANNEL_VOLUME_15, reg_val);
	es9118_update_register(RIGHT_CHANNEL_VOLUME_15, reg_val);

	pr_info("%s: boost = %s\n", __func__, boost == 0 ? "Off" : "On");
	return 0;
}

static int es9118_get_mute(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int mute = 0;
    u8 reg_val;

	reg_val = es9118_read_reg(g_es9118_priv->i2c_client,
				GENERAL_SETTINGS);
	reg_val &= I2S_CHANNEL_MUTE_MASK;
	if (reg_val) {
		mute = 1;
	}
	ucontrol->value.integer.value[0] = mute;

	pr_info("%s: mute = %s\n", __func__, mute == 0 ? "Off" : "On");

	return 0;
}

static int es9118_set_mute(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
    int ret;
	int mute = 0;
	u8 reg_val;

	pr_info("%s: ucontrol->value.integer.value[0]  = %ld\n",
			__func__, ucontrol->value.integer.value[0]);

	reg_val = es9118_read_reg(g_es9118_priv->i2c_client,
			GENERAL_SETTINGS);
	if ((int)reg_val < 0) {
		pr_info("%s: read fail!\n", __func__);
		return 0;
	}

	if (ucontrol->value.integer.value[0]) {
	    reg_val |=  I2S_CHANNEL_MUTE_MASK;
		mute = 1;
	} else {
	    reg_val &= ~(I2S_CHANNEL_MUTE_MASK);
		mute = 0;
	}
	
	ret = es9118_write_reg(g_es9118_priv->i2c_client,
			GENERAL_SETTINGS, reg_val);
	if (ret < 0) {
		pr_info("%s: write fail!\n", __func__);
	}
	es9118_update_register(GENERAL_SETTINGS, reg_val);
	
	pr_info("%s: mute = %s\n", __func__, mute == 0 ? "Off" : "On");
	return 0;
}

static int es9118_gain_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int left;
	int right;

	left = es9118_read_reg(g_es9118_priv->i2c_client, LEFT_CHANNEL_VOLUME_16);
	if (left < 0) {
		pr_info("%s: read left volume %d fail!\n", __func__, left);
	}
	ucontrol->value.integer.value[0] = left; 

	right = es9118_read_reg(g_es9118_priv->i2c_client, RIGHT_CHANNEL_VOLUME_15);
	if (right < 0) {
		pr_info("%s: read left volume %d fail!\n", __func__, right);
	}
	ucontrol->value.integer.value[1] = right;

	pr_info("%s: volume left = %d, right = %d\n", __func__, left, right);

	return 0;
}

static int es9118_gain_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
    int ret;
	int left;
	int right;
	
	pr_info("%s: ucontrol->value.integer.value[0] = %ld, ucontrol->value.integer.value[1] = %ld \n",
			__func__, ucontrol->value.integer.value[0], ucontrol->value.integer.value[1]);

	left = ucontrol->value.integer.value[0];
	ret = es9118_write_reg(g_es9118_priv->i2c_client,
			LEFT_CHANNEL_VOLUME_16, left);
	if (ret < 0) {
		pr_info("%s: write left volume %d fail!\n", __func__, left);
	}	
	
	right = ucontrol->value.integer.value[1];
	ret = es9118_write_reg(g_es9118_priv->i2c_client,
			RIGHT_CHANNEL_VOLUME_15, right);
	if (ret < 0) {
		pr_info("%s: write right volume %d fail!\n", __func__, right);
	}

	pr_info("%s: volume left = %d, right = %d\n", __func__, left, right);
	return 0;
}

static int es9118_reg_no_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = es9118_reg_no;

	return 0;
}

static int es9118_reg_no_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	es9118_reg_no = ucontrol->value.integer.value[0];

	return 0;
}

static int es9118_reg_val_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	u8 reg_val;
	
	reg_val = es9118_read_reg(g_es9118_priv->i2c_client,
				es9118_reg_no);
	ucontrol->value.integer.value[0] = reg_val;

	return 0;
}

static int es9118_reg_val_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	u8 reg_val;
	reg_val = ucontrol->value.integer.value[0];
    es9118_write_reg(g_es9118_priv->i2c_client,
				es9118_reg_no, reg_val);
	return 0;
}
static const char * const es9118_i2s_length_texts[] = {
	"16Bit", "24Bit", "32Bit", "32Bit"
};

static const char * const es9118_clk_divider_texts[] = {
	"DIV2", "DIV4", "DIV8", "DIV16"
};

static const char * const es9118_mute_texts[] = {
	"Off", "On"
};

static const char * const es9118_boost_texts[] = {
	"Off", "On"
};

static const char * const es9118_mode_texts[] = {
	"Standy", "Bypass", "Hifi", 
};

static const char * const es9118_master_texts[] = {
	"Slave", "Master"
};

static const struct soc_enum es9118_i2s_length_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(es9118_i2s_length_texts),
		es9118_i2s_length_texts);

static const struct soc_enum es9118_clk_divider_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(es9118_clk_divider_texts),
		es9118_clk_divider_texts);

static const struct soc_enum es9118_mute_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(es9118_mute_texts),
		es9118_mute_texts);

static const struct soc_enum es9118_boost_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(es9118_boost_texts),
		es9118_boost_texts);

static const struct soc_enum es9118_mode_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(es9118_mode_texts),
		es9118_mode_texts);

static const struct soc_enum es9118_master_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(es9118_master_texts),
		es9118_master_texts);

static struct snd_kcontrol_new es9118_snd_controls[] = {
	/* commit controls */
	SOC_ENUM_EXT("Es9118 Mode",
			es9118_mode_enum,
			es9118_get_mode,
			es9118_set_mode),
	SOC_ENUM_EXT("Es9118 Master",
			es9118_master_enum,
			es9118_get_master,
			es9118_set_master),
	SOC_ENUM_EXT("Es9118 I2S Length",
			es9118_i2s_length_enum,
			es9118_i2s_length_get,
			es9118_i2s_length_put),
	SOC_ENUM_EXT("Es9118 CLK Divider",
			es9118_clk_divider_enum,
			es9118_get_clk_divider,
			es9118_set_clk_divider),
	SOC_ENUM_EXT("Es9118 Mute",
			es9118_mute_enum,
			es9118_get_mute,
			es9118_set_mute),
	SOC_ENUM_EXT("Es9118 Boost",
			es9118_boost_enum,
			es9118_get_boost,
			es9118_set_boost),
	SOC_SINGLE_MULTI_EXT("Es9118 Gain", SND_SOC_NOPM, 0, 
            255, 0, 2, es9118_gain_get, es9118_gain_put),
	SOC_SINGLE_EXT("Es9118 Register No", SND_SOC_NOPM, 0, 80, 0,
			es9118_reg_no_get, es9118_reg_no_put),
	SOC_SINGLE_EXT("Es9118 Register Value", SND_SOC_NOPM, 0, 255, 0,
			es9118_reg_val_get, es9118_reg_val_put)
};

static int es9118_read_reg(struct i2c_client *client, int reg)
{
	int ret;

    if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        unsigned char addr;
		unsigned char val;
		
        addr = (unsigned char)reg;
        ret = i2c_master_send(client, &addr, 1);
		if (ret < 0)
            pr_err("%s:fail to write reg = %d\n", __func__, reg);
		
        ret = i2c_master_recv(client, &val, 1);
        if (ret < 0)
            pr_err("%s:fail to read reg = %d\n", __func__, reg);
		else
			ret = val;
    } else { 
	    ret = i2c_smbus_read_byte_data(client, reg);
	    if (ret < 0)
		    dev_err(&client->dev, "%s: err %d, reg %d\n", __func__, ret, reg);
    }

	return ret;
}

static int es9118_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;
    unsigned char block_data[2];
	
    if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        block_data[0] = (unsigned char)reg;
        block_data[1] = (unsigned char)value;
        ret = i2c_master_send(client, block_data, 2);
        if (ret < 0)
            pr_err("%s:fail to write reg = %d, value=0x%x\n", __func__, reg, value);
    } else { 
	    ret = i2c_smbus_write_byte_data(client, reg, value);
	    if (ret < 0)
		    dev_err(&client->dev, "%s: err %d, reg %d\n", __func__, ret, reg);
    }

	return ret;
}

static void es9118_request_gpio(struct es9118_data *pdata)
{
	int ret;

	ret = gpio_request(pdata->power_gpio, "power-gpio");
	if (ret < 0) {
		pr_err("%s(): power-gpio request failed",
				__func__);
		goto err;
	}

	ret = gpio_request(pdata->cycle_gpio, "cycle-gpio");
	if (ret < 0) {
		pr_err("%s(): cycle-gpio request failed",
				__func__);
		goto err;
	}
	
	ret = gpio_request(pdata->reset_gpio, "reset-gpio");
	if (ret < 0) {
		pr_err("%s(): reset-gpio request failed",
				__func__);
		goto err;
	}

	ret = gpio_request(pdata->enable_gpio, "enable-gpio");
	if (ret < 0) {
		pr_err("%s(): enable-gpio request failed",
				__func__);
		goto err;
	}
   
err:
	return;
}

static int es9118_populate_get_pdata(struct device *dev,
		struct es9118_data *pdata)
{
      pdata->power_gpio = of_get_named_gpio(dev->of_node,
			"es9118,power-gpio", 0);
	if (pdata->power_gpio < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
				"es9118,power-gpio", dev->of_node->full_name,
				pdata->power_gpio);
		goto err;
	}
	dev_err(dev, "%s: power-gpio %d", __func__, pdata->power_gpio);

	pdata->cycle_gpio = of_get_named_gpio(dev->of_node,
			"es9118,cycle-gpio", 0);
	if (pdata->cycle_gpio < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
				"es9118,cycle-gpio", dev->of_node->full_name,
				pdata->cycle_gpio);
		goto err;
	}
	dev_err(dev, "%s: power-gpio %d", __func__, pdata->cycle_gpio);
	
	pdata->reset_gpio = of_get_named_gpio(dev->of_node,
			"es9118,reset-gpio", 0);
	if (pdata->reset_gpio < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
				"es9118,reset-gpio", dev->of_node->full_name,
				pdata->reset_gpio);
		goto err;
	}
	dev_err(dev, "%s: reset-gpio %d", __func__, pdata->reset_gpio);

       pdata->enable_gpio = of_get_named_gpio(dev->of_node,
			"es9118,enable-gpio", 0);
	if (pdata->enable_gpio < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
				"es9118,enable-gpio", dev->of_node->full_name,
				pdata->enable_gpio);
		goto err;
	}
	dev_err(dev, "%s:enable-gpio %d", __func__, pdata->enable_gpio);

	return 0;
err:
	devm_kfree(dev, pdata);
	return -1;
}

static unsigned int es9118_codec_read(struct snd_soc_codec *codec,
		unsigned int reg)
{
	/* struct es9118_priv *priv = codec->control_data; */
	return 0;
}

static int es9118_codec_write(struct snd_soc_codec *codec, unsigned int reg,
		unsigned int value)
{
	/* struct es9118_priv *priv = codec->control_data; */
	return 0;
}

static int es9118_suspend(struct snd_soc_codec *codec)
{
	return 0;
}

static int es9118_resume(struct snd_soc_codec *codec)
{

	return 0;
}

static int es9118_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *codec_dai)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	/* struct es9118_priv *priv = codec->control_data; */

	return 0;
}

static int es9118_mute(struct snd_soc_dai *dai, int mute)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	/* struct es9118_priv *priv = codec->control_data; */

	return 0;

}

static int es9118_set_clkdiv(struct snd_soc_dai *codec_dai,
				int div_id, int div)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	/* struct es9118_priv *priv = codec->control_data; */

	return 0;
}

static int es9118_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	/* struct es9118_priv *priv = codec->control_data; */

	return 0;
}


static int es9118_set_dai_fmt(struct snd_soc_dai *codec_dai,
				unsigned int fmt)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	/* struct es9118_priv *priv = codec->control_data; */

	return 0;
}

static int es9118_set_fll(struct snd_soc_dai *codec_dai,
		int pll_id, int source, unsigned int freq_in,
		unsigned int freq_out)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	/* struct es9118_priv *priv = codec->control_data; */

	return 0;
}

static int es9118_pcm_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *codec_dai)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	/* struct es9118_priv *priv = codec->control_data; */

	return 0;
}

static const struct snd_soc_dai_ops es9118_dai_ops = {
	.hw_params	= es9118_pcm_hw_params,
	.digital_mute	= es9118_mute,
	.trigger	= es9118_pcm_trigger,
	.set_fmt	= es9118_set_dai_fmt,
	.set_sysclk	= es9118_set_dai_sysclk,
	.set_pll	= es9118_set_fll,
	.set_clkdiv	= es9118_set_clkdiv,
};

static struct snd_soc_dai_driver es9118_dai = {
	.name = "es9118-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = es9118_RATES,
		.formats = es9118_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = es9118_RATES,
		.formats = es9118_FORMATS,
	},
	.ops = &es9118_dai_ops,
};

static  int es9118_codec_probe(struct snd_soc_codec *codec)
{
	int rc = 0;
	struct es9118_priv *priv = snd_soc_codec_get_drvdata(codec);

	pr_debug("%s: enter\n", __func__);

	priv->codec = codec;

	codec->control_data = snd_soc_codec_get_drvdata(codec);


	rc = snd_soc_add_codec_controls(codec, es9118_snd_controls,
			ARRAY_SIZE(es9118_snd_controls));
	if (rc)
		dev_err(codec->dev, "%s(): es9118_snd_controls failed\n",
			__func__);

	return 0;
}

static int  es9118_codec_remove(struct snd_soc_codec *codec)
{
	struct es9118_priv *priv = snd_soc_codec_get_drvdata(codec);


	kfree(priv);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_es9118 = {
	.probe = es9118_codec_probe,
	.remove = es9118_codec_remove,
	.suspend = es9118_suspend,
	.resume = es9118_resume,
	.read = es9118_codec_read,
	.write = es9118_codec_write,
};

int es9118_exist=0;

static int es9118_check(void)
{
	int ret;//, reg_num, i;

	struct es9118_data *es9118_ctrl = g_es9118_priv->es9118_data;

	pr_info("%s: enter\n", __func__);
	es9118_enable_gpio(es9118_ctrl->power_gpio, 0);
	es9118_enable_gpio(es9118_ctrl->enable_gpio, 0);
	es9118_enable_gpio(es9118_ctrl->reset_gpio, 0);
	es9118_enable_gpio(es9118_ctrl->cycle_gpio, 0);
	msleep(10);

	// Check whether the mclk has been enable or not
	es9118_enable_gpio(es9118_ctrl->reset_gpio, 1);
	es9118_enable_gpio(es9118_ctrl->enable_gpio, 1);
	msleep(5);

	es9118_enable_gpio(es9118_ctrl->power_gpio, 1);
	msleep(20);
	pr_info("%s: switch es9118 cycle on\n", __func__);
	es9118_enable_gpio(es9118_ctrl->cycle_gpio, 1);
	msleep(20);

/*
	reg_num = sizeof(init_reg_es9118)/sizeof(struct es9118_reg);
	for (i = 0; i < reg_num; i++) {
		es9118_write_reg(g_es9118_priv->i2c_client,
				init_reg_es9118[i].num,
				init_reg_es9118[i].value);
	}
*/

	ret = es9118_read_reg(g_es9118_priv->i2c_client,
				MASTER_MODE_CONTROL);

	printk("es9118_check MASTER_MODE_CONTROL:%d\n", ret);

	return ret;
}

static int es9118_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct es9118_priv *priv;
	struct es9118_data *pdata;
	int ret = 0;

       dev_err(&client->dev, "%s: enter\n", __func__);
	
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "%s: no support for i2c read/write"
				"byte data\n", __func__);
		return -EIO;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct es9118_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = es9118_populate_get_pdata(&client->dev, pdata);
		if (ret) {
			dev_err(&client->dev, "Parsing DT failed(%d)", ret);
			return ret;
		}
	} else {
		pdata = client->dev.platform_data;
	}

	if (!pdata) {
		dev_err(&client->dev, "%s: no platform data\n", __func__);
		return -EINVAL;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct es9118_priv),
			GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;
	priv->i2c_client = client;
	priv->es9118_data = pdata;
	i2c_set_clientdata(client, priv);

	g_es9118_priv = priv;

	es9118_request_gpio(pdata);

	ret = es9118_check();
	if(ret<0) {
		//i2c_del_driver(&es9118_i2c_driver);
		printk("Can not find es9118!!\n");
		return -EIO;
	}
	printk("found es9118!!\n");

	es9118_exist=1;

	if (client->dev.of_node)
		dev_set_name(&client->dev, "%s", "es9118-codec");

	ret = snd_soc_register_codec(&client->dev, &soc_codec_dev_es9118,
			&es9118_dai, 1);

	pdata->b_power_on = false;
	pdata->b_cycle_on = false;	
	pdata->b_closing = false;
	pdata->mode = 0;
	pdata->divider_value = 0;
	pdata->master_value = 0;
	pdata->i2s_length_value = 3;

	es9118_enable_gpio(pdata->power_gpio, 0);
	es9118_enable_gpio(pdata->cycle_gpio, 0);
	es9118_enable_gpio(pdata->enable_gpio, 0);
	es9118_enable_gpio(pdata->reset_gpio, 0);
	atomic_set(&pdata->clk_count_ref, 0);
	
	INIT_DELAYED_WORK(&pdata->power_off_dwork, es9118_power_off_delayed_work);
	mutex_init(&es9118_lock);
	INIT_DELAYED_WORK(&clk_divider_dwork, clk_divider_delayed_work);
	wake_lock_init(&es9118_wakelock, WAKE_LOCK_SUSPEND, "es9118_wakelock");

	dev_err(&client->dev, "%s: exit %d\n", __func__, ret);
	return ret;
}

static int es9118_remove(struct i2c_client *client)
{
    dev_err(&client->dev, "Remove soc es9118 codec");
	
	gpio_free(g_es9118_priv->es9118_data->power_gpio);
	gpio_free(g_es9118_priv->es9118_data->cycle_gpio);
	gpio_free(g_es9118_priv->es9118_data->reset_gpio);
	gpio_free(g_es9118_priv->es9118_data->enable_gpio);
	wake_lock_destroy(&es9118_wakelock);
	mutex_destroy(&es9118_lock);

	return 0;
}

static struct of_device_id es9118_match_table[] = {
	{ .compatible = "dac,es9118-codec", },
	{}
};

static const struct i2c_device_id es9118_id[] = {
	{ "es9118", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, isa1200_id);

static struct i2c_driver es9118_i2c_driver = {
	.driver	= {
		.name	= "es9118-codec",
		.of_match_table = es9118_match_table,
	},
	.probe		= es9118_probe,
	.remove		= es9118_remove,
	.id_table	= es9118_id,
};

static int __init es9118_init(void)
{
	int ret;
	
	ret = i2c_add_driver(&es9118_i2c_driver);
	if ( ret != 0 ) {
		printk("es9118_init i2c_driver_register fail, Error : %d\n",ret);
		i2c_del_driver(&es9118_i2c_driver);	
			
	}		
	return ret;

}

static void __exit es9118_exit(void)
{
	i2c_del_driver(&es9118_i2c_driver);
}

module_init(es9118_init);
module_exit(es9118_exit);

MODULE_DESCRIPTION("ASoC es9118 driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:es9118-codec");

