/* Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
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
#ifndef MSM_EEPROM_H
#define MSM_EEPROM_H

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <soc/qcom/camera2.h>
#include <media/v4l2-subdev.h>
#include <media/msmb_camera.h>
#include "msm_camera_i2c.h"
#include "msm_camera_spi.h"
#include "msm_camera_io_util.h"
#include "msm_camera_dt_util.h"

struct msm_eeprom_ctrl_t;

#define DEFINE_MSM_MUTEX(mutexname) \
	static struct mutex mutexname = __MUTEX_INITIALIZER(mutexname)

#define PROPERTY_MAXSIZE 32

struct msm_eeprom_ctrl_t {
	struct platform_device *pdev;
	struct mutex *eeprom_mutex;

	struct v4l2_subdev sdev;
	struct v4l2_subdev_ops *eeprom_v4l2_subdev_ops;
	enum msm_camera_device_type_t eeprom_device_type;
	struct msm_sd_subdev msm_sd;
	enum cci_i2c_master_t cci_master;
	enum i2c_freq_mode_t i2c_freq_mode;

	struct msm_camera_i2c_client i2c_client;
	struct msm_eeprom_board_info *eboard_info;
	uint32_t subdev_id;
	int32_t userspace_probe;
	struct msm_eeprom_memory_block_t cal_data;
	uint8_t is_supported;
};

// EEPROM Vendor identifiers for EEPROM dynamic loading
#define CAMERA_SENSOR_NAME_MAX_LENGTH		128

typedef enum __camera_vendor_module_id{
	MID_NULL = 0,
	MID_SUNNY,
	MID_TRULY,
	MID_A_KERR,
	MID_LITEARRAY,
	MID_DARLING,
	MID_QTECH,
	MID_OFILM,
	MID_HUAQUAN,
	MID_KINGCOM = MID_HUAQUAN,
	MID_BOOYI,
	MID_LAIMU,
	MID_WDSEN,
	MID_SUNRISE,
	MID_PRIMAX = 0x17,
	MID_AVC,
	MID_MAX
} camera_vendor_module_id;

struct vendor_eeprom{
	char eeprom_name[CAMERA_SENSOR_NAME_MAX_LENGTH];
	camera_vendor_module_id module_id;
};

// Table for Sensor and EEPROM matching inside a sensor module
typedef struct __camera_vendor_module {
	char sensor_name[CAMERA_SENSOR_NAME_MAX_LENGTH];
	char eeprom_name[CAMERA_SENSOR_NAME_MAX_LENGTH];
	camera_vendor_module_id module_id;
	camera_vendor_module_id (*get_otp_id)(struct msm_eeprom_ctrl_t *);
} camera_vendor_module;

extern const uint32_t CAMERA_VENDOR_EEPROM_COUNT_MAX;
extern const camera_vendor_module camera_vendor_module_table[];
extern struct vendor_eeprom s_vendor_eeprom[];

#endif
