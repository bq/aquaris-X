/*************************************************************************
  Copyright£º ShenZhen ChipSailing Technology Co., Ltd. All rights reserved.
  File name: chips_platform.c
  File description: Related to the hardware platform of the function interface
  Author: zwp    ID:58    Version:2.0   DateDate:2016/10/16
  Others:
  History:
      1. Date:           Author:          ID:
	     Modify description:
		  2.
 *************************************************************************/

#include <linux/of.h>
#include <linux/err.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>

#include <linux/delay.h>

#include "chips_main.h"

#define CHIPS_RESET_GPIO 140
#define CHIPS_IRQ_GPIO   48
#define CHIPS_PWR_GPIO   35  


 /**
 *  @brief chips_parse_dts parsing DTS, obtain hardware parameter information
 *  
 *  @param [in] chips_dev chips_dev structure pointer
 *  
 *  @return successfully return 0, failure return nonzero
 */
int chips_parse_dts(struct chips_data* chips_dev)
{
	struct device_node *node = NULL;
	struct platform_device *pdev = NULL;

	node = of_find_compatible_node(NULL, NULL, "mediatek,cs_finger");
	if (IS_ERR(node)) {
		chips_dbg("device node is null\n");
		return PTR_ERR(node);
	}
	
	pdev = of_find_device_by_node(node);
	if (IS_ERR(pdev)) {
		chips_dbg("platform device is null\n");
		return PTR_ERR(pdev);
	}

	chips_dev->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(chips_dev->pinctrl)) {
		chips_dbg("devm_pinctrl_get error\n");
		return  PTR_ERR(chips_dev->pinctrl);
	}

	/*int¡¢rst */
	chips_dev->rst_output1 = pinctrl_lookup_state(chips_dev->pinctrl, "cs_finger_reset_en1");
	if (IS_ERR(chips_dev->rst_output1)) {
		chips_dbg("Cannot find cs_finger_reset_en1\n");
		return PTR_ERR(chips_dev->rst_output1);
	}
 
	chips_dev->rst_output0 = pinctrl_lookup_state(chips_dev->pinctrl, "cs_finger_reset_en0");
	if (IS_ERR(chips_dev->rst_output0)) {
		chips_dbg("Cannot find cs_finger_reset_en0\n");
		return PTR_ERR(chips_dev->rst_output0);
	}
	
	chips_dev->cs_finger_spi0_mi_as_spi0_mi = pinctrl_lookup_state(chips_dev->pinctrl, "cs_finger_spi0_mi_as_spi0_mi");
	if (IS_ERR(chips_dev->cs_finger_spi0_mi_as_spi0_mi)) {
		chips_dbg("Cannot find cs_finger_spi0_mi_as_spi0_mi\n");
		return PTR_ERR(chips_dev->cs_finger_spi0_mi_as_spi0_mi);
	}
	
	chips_dev->cs_finger_spi0_mi_as_gpio = pinctrl_lookup_state(chips_dev->pinctrl, "cs_finger_spi0_mi_as_gpio");
	if (IS_ERR(chips_dev->cs_finger_spi0_mi_as_gpio)) {
		chips_dbg("Cannot find cs_finger_spi0_mi_as_gpio\n");
		return PTR_ERR(chips_dev->cs_finger_spi0_mi_as_gpio);
	}
	
	chips_dev->cs_finger_spi0_mo_as_spi0_mo = pinctrl_lookup_state(chips_dev->pinctrl, "cs_finger_spi0_mo_as_spi0_mo");
	if (IS_ERR(chips_dev->cs_finger_spi0_mo_as_spi0_mo)) {
		chips_dbg("Cannot find cs_finger_spi0_mo_as_spi0_mo\n");
		return PTR_ERR(chips_dev->cs_finger_spi0_mo_as_spi0_mo);
	}
	
	chips_dev->cs_finger_spi0_mo_as_gpio = pinctrl_lookup_state(chips_dev->pinctrl, "cs_finger_spi0_mo_as_gpio");
	if (IS_ERR(chips_dev->cs_finger_spi0_mo_as_gpio)) {
		chips_dbg("Cannot find cs_finger_spi0_mo_as_gpio\n");
		return PTR_ERR(chips_dev->cs_finger_spi0_mo_as_gpio);
	}
	
	chips_dev->cs_finger_spi0_clk_as_spi0_clk = pinctrl_lookup_state(chips_dev->pinctrl, "cs_finger_spi0_clk_as_spi0_clk");
	if (IS_ERR(chips_dev->cs_finger_spi0_clk_as_spi0_clk)) {
		chips_dbg("Cannot find cs_finger_spi0_clk_as_spi0_clk\n");
		return PTR_ERR(chips_dev->cs_finger_spi0_clk_as_spi0_clk);
	}
	
	chips_dev->cs_finger_spi0_clk_as_gpio = pinctrl_lookup_state(chips_dev->pinctrl, "cs_finger_spi0_clk_as_gpio");
	if (IS_ERR(chips_dev->cs_finger_spi0_clk_as_gpio)) {
		chips_dbg("Cannot find cs_finger_spi0_clk_as_gpio\n");
		return PTR_ERR(chips_dev->cs_finger_spi0_clk_as_gpio);
	}
	
	chips_dev->cs_finger_spi0_cs_as_spi0_cs = pinctrl_lookup_state(chips_dev->pinctrl, "cs_finger_spi0_cs_as_spi0_cs");
	if (IS_ERR(chips_dev->cs_finger_spi0_cs_as_spi0_cs)) {
		chips_dbg("Cannot find cs_finger_spi0_cs_as_spi0_cs\n");
		return PTR_ERR(chips_dev->cs_finger_spi0_cs_as_spi0_cs);
	}
	
	chips_dev->cs_finger_spi0_cs_as_gpio = pinctrl_lookup_state(chips_dev->pinctrl, "cs_finger_spi0_cs_as_gpio");
	if (IS_ERR(chips_dev->cs_finger_spi0_cs_as_gpio)) {
		chips_dbg("Cannot find cs_finger_spi0_cs_as_gpio\n");
		return PTR_ERR(chips_dev->cs_finger_spi0_cs_as_gpio);
	}
	
	chips_dev->eint_as_int = pinctrl_lookup_state(chips_dev->pinctrl, "cs_finger_int_as_int");
	if (IS_ERR(chips_dev->eint_as_int)) {
		chips_dbg("Cannot find s_finger_int_as_int\n");
		return PTR_ERR(chips_dev->eint_as_int);
	}
	
	chips_dbg("get pinctrl success\n");
	return 0;
}


 /**
 *  @brief chips_release_gpio release interrupt and reset gpio
 *  
 *  @param [in] chips_dev chips_dev structure pointer 
 *  
 *  @return no return value
 */
void chips_release_gpio(struct chips_data* chips_dev)
{
	/*
	if (gpio_is_valid(chips_dev->irq_gpio)){
		gpio_free(chips_dev->irq_gpio);
	}
	*/
	if (gpio_is_valid(chips_dev->reset_gpio)){
		gpio_free(chips_dev->reset_gpio);
	}
	/*
	if (gpio_is_valid(chips_dev->pwr_gpio)){
		gpio_free(chips_dev->pwr_gpio);
	}*/	
}


 /**
 *  @brief chips_set_reset_gpio set reset IO level state
 *  
 *  @param [in] chips_dev chips_dev structure pointer
 *  @param [in] delay_ms time delay parameters
 *  @return successfully return 0, failure return a negative number
 */
int chips_set_reset_gpio(struct chips_data *chips_dev, unsigned int level)
{

	chips_dev->reset_gpio = CHIPS_RESET_GPIO;
	if(gpio_request(chips_dev->reset_gpio,"reset_gpio")<0){
		gpio_free(chips_dev->reset_gpio);
		gpio_request(chips_dev->reset_gpio, "reset_gpio");
		gpio_direction_output(chips_dev->reset_gpio,1);
	}
	if(level != 0){
		gpio_set_value(chips_dev->reset_gpio,1);	
	}else{
		gpio_set_value(chips_dev->reset_gpio,0);
	}
	return 0;
/*
	int ret = -1;	

	if(level != 0){
		ret = pinctrl_select_state(chips_dev->pinctrl, chips_dev->rst_output1);
		if(ret < 0){
			chips_dbg("pinctrl_select_state error,ret = %d\n",ret);
			return ret;
		}
				
	}else{
		ret = pinctrl_select_state(chips_dev->pinctrl, chips_dev->rst_output0);
		if(ret < 0){
			chips_dbg("pinctrl_select_state error,ret = %d\n",ret);
			return ret;
		}
	}
	return 0
	*/
}


/**
*  @brief chips_hw_reset IC reset
*  
*  @param [in] chips_dev chips_dev structure pointer
*  @param [in] delay_ms time delay parameters
*  @return successfully return 0, failure return a negative number
*/
int chips_hw_reset(struct chips_data *chips_dev, unsigned int delay_ms)
{ 
	chips_set_reset_gpio(chips_dev,0);

	mdelay((delay_ms > 1)?delay_ms:1);

	chips_set_reset_gpio(chips_dev,1);

	mdelay(1);

	return 0;	 
}
  

 /**
 *  @brief chips_get_irqno access to interrupt number
 *  
 *  @param [in] chips_dev chips_dev structure pointer
 *  
 *  @return successfully return 0, failure return a negative number
 */
int chips_get_irqno(struct chips_data *chips_dev)
{


	chips_dev->irq_gpio = CHIPS_IRQ_GPIO;
	if(gpio_request(chips_dev->irq_gpio,"irq_gpio") < 0){
		gpio_free(chips_dev->irq_gpio);
		gpio_request(chips_dev->irq_gpio,"irq_gpio");
		gpio_direction_input(chips_dev->irq_gpio);		
	}
	
	chips_dev->irq = gpio_to_irq(chips_dev->irq_gpio);
	if(chips_dev->irq < 0){
		chips_dbg("gpio can not be mapped to irq\n");
	}
	
	return chips_dev->irq;

/*
	u32 ints[2]={0};
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mediatek,cs_finger");
	if(NULL != node){
		chips_dev->irq = irq_of_parse_and_map(node, 0);
		
		of_property_read_u32_array(node,"debounce",ints,ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);
	}else{
		chips_dbg("of_find_compatible_node error\n");
		return -ENODEV;
	}
	
	return chips_dev->irq;
	*/
}


 /**
 *  @brief chips_set_spi_mode set spi mode
 *  
 *  @param [in] chips_dev chips_data structure pointer
 *  
 *  @return successfully return 0, failure return a negative number
 */
int chips_set_spi_mode(struct chips_data *chips_dev)
{
	int ret = -1;
	
	ret = pinctrl_select_state(chips_dev->pinctrl, chips_dev->cs_finger_spi0_clk_as_spi0_clk);
	if(ret < 0){
		chips_dbg("pinctrl_select_state error,ret = %d\n",ret);
		return ret;
	}
			
	ret = pinctrl_select_state(chips_dev->pinctrl, chips_dev->cs_finger_spi0_cs_as_spi0_cs);
	if(ret < 0){
		chips_dbg("pinctrl_select_state error,ret = %d\n",ret);
		return ret;
	}
		
	ret = pinctrl_select_state(chips_dev->pinctrl, chips_dev->cs_finger_spi0_mi_as_spi0_mi);
	if(ret < 0){
		chips_dbg("pinctrl_select_state error,ret = %d\n",ret);
		return ret;
	}
		
	ret = pinctrl_select_state(chips_dev->pinctrl, chips_dev->cs_finger_spi0_mo_as_spi0_mo);
	if(ret < 0){
		chips_dbg("pinctrl_select_state error,ret = %d\n",ret);
		return ret;
	}
	
	return 0;
}

int chips_pwr_up(struct chips_data *chips_dev)
{
	chips_dev->pwr_gpio = CHIPS_PWR_GPIO;
	if(gpio_request(chips_dev->pwr_gpio,"pwr_gpio")<0){
		gpio_free(chips_dev->pwr_gpio);
		gpio_request(chips_dev->pwr_gpio, "pwr_gpio");
		gpio_direction_output(chips_dev->pwr_gpio,1);
	}	
	
	gpio_direction_output(chips_dev->pwr_gpio,1);
	
	return 0;
}
