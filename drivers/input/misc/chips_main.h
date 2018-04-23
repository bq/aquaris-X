#ifndef __CHIPS_MAIN_H__
#define __CHIPS_MAIN_H__

#include <linux/mutex.h>

#define CHIPS_W_SRAM 0xAA
#define CHIPS_R_SRAM 0xBB
#define CHIPS_W_SFR  0xCC
#define CHIPS_R_SFR  0xDD

struct chips_data {
	dev_t devt;
	unsigned char *buffer;
	int users;
	int irq;
	int irq_gpio;
	int reset_gpio;
	int pwr_gpio;
	bool irq_enabled;
	spinlock_t spin_lock;
	struct class *cls;
	struct mutex buf_lock;
	struct input_dev *input;
	struct list_head device_entry;
	struct fasync_struct *async;

    struct pinctrl *pinctrl;
    struct pinctrl_state *eint_as_int, *rst_output0, *rst_output1;
    struct pinctrl_state *cs_finger_spi0_mi_as_spi0_mi,*cs_finger_spi0_mi_as_gpio,*cs_finger_spi0_mo_as_spi0_mo,*cs_finger_spi0_mo_as_gpio;
    struct pinctrl_state *cs_finger_spi0_clk_as_spi0_clk,*cs_finger_spi0_clk_as_gpio,*cs_finger_spi0_cs_as_spi0_cs,*cs_finger_spi0_cs_as_gpio;
};

#define DEBUG

#ifdef DEBUG
#define chips_dbg(fmt, args...) do {\
    printk("[chipsailing]%5d:<%s>  "fmt,__LINE__,__func__,##args);\
} while(0)
	
#else
#define chips_dbg(fmt, args...)
#endif

#endif