/*
 *Copyright 2015 NXP Semiconductors
 *
 *Licensed under the Apache License, Version 2.0 (the "License");
 *you may not use this file except in compliance with the License.
 *You may obtain a copy of the License at
 *
 *http://www.apache.org/licenses/LICENSE-2.0
 *
 *Unless required by applicable law or agreed to in writing, software
 *distributed under the License is distributed on an "AS IS" BASIS,
 *WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *See the License for the specific language governing permissions and
 *limitations under the License.
 */

#include <stdlib.h>
#include <string.h>
#include "tfa_dsp_fw.h"
#include "dbgprint.h"
#include "NXP_I2C.h"
#include "tfa_internal.h"
#include "lxScribo.h"

/* translate a I2C driver error into an error for Tfa9887 API */
static enum Tfa98xx_Error tfa98xx_classify_i2c_error(enum NXP_I2C_Error i2c_error)
{
	switch (i2c_error) {
		case NXP_I2C_Ok:
			return Tfa98xx_Error_Ok;
		case NXP_I2C_NoAck:
		case NXP_I2C_ArbLost:
		case NXP_I2C_TimeOut:
			return Tfa98xx_Error_I2C_NonFatal;
		default:
			return Tfa98xx_Error_I2C_Fatal;
	}
}
/*
 * write a 16 bit subaddress
 */
enum Tfa98xx_Error
tfa98xx_write_register16(Tfa98xx_handle_t handle,
			unsigned char subaddress, unsigned short value)
{
	enum NXP_I2C_Error i2c_error;
	unsigned char write_data[3]; /* subaddress and 2 bytes of the value */
	if (!tfa98xx_handle_is_open(handle))
		return Tfa98xx_Error_NotOpen;

	write_data[0] = subaddress;
	write_data[1] = (value >> 8) & 0xFF;
	write_data[2] = value & 0xFF;

	i2c_error = NXP_I2C_WriteRead(handles_local[handle].slave_address, sizeof(write_data), write_data, 0, NULL);

	return tfa98xx_classify_i2c_error(i2c_error);
}

enum Tfa98xx_Error
tfa98xx_read_register16(Tfa98xx_handle_t handle,
		       unsigned char subaddress, unsigned short *pValue)
{
	enum NXP_I2C_Error i2c_error;
	unsigned char write_data[1]; /* subaddress */
	unsigned char read_buffer[2]; /* 2 data bytes */

	_ASSERT(pValue != NULL);
	if (!tfa98xx_handle_is_open(handle))
		return Tfa98xx_Error_NotOpen;
	write_data[0] = subaddress;
	read_buffer[0] = read_buffer[1] = 0;

	i2c_error = NXP_I2C_WriteRead(handles_local[handle].slave_address, 
			sizeof(write_data), write_data, sizeof(read_buffer), read_buffer);
	if (tfa98xx_classify_i2c_error(i2c_error) != Tfa98xx_Error_Ok) {
		return tfa98xx_classify_i2c_error(i2c_error);
	} else {
		*pValue = (read_buffer[0] << 8) + read_buffer[1];
		return Tfa98xx_Error_Ok;
	}
}

enum Tfa98xx_Error
tfa98xx_read_data(Tfa98xx_handle_t handle,
		 unsigned char subaddress, int num_bytes, unsigned char data[])
{
	enum NXP_I2C_Error i2c_error;
	unsigned char write_data[1]; /* subaddress */

	if (!tfa98xx_handle_is_open(handle))
		return Tfa98xx_Error_NotOpen;
	if (num_bytes > handles_local[handle].buffer_size)
		return Tfa98xx_Error_Bad_Parameter;

	write_data[0] = subaddress;
	i2c_error =
	    NXP_I2C_WriteRead(handles_local[handle].slave_address, sizeof(write_data),
			      write_data, num_bytes, data);
	return tfa98xx_classify_i2c_error(i2c_error);
}

/*
 * Write raw I2C data with no sub address
 */
enum Tfa98xx_Error
tfa98xx_write_raw(Tfa98xx_handle_t handle,
		  int num_bytes,
		  const unsigned char data[])
{
	enum NXP_I2C_Error i2c_error;

	if (!tfa98xx_handle_is_open(handle))
		return Tfa98xx_Error_NotOpen;
	if (num_bytes > handles_local[handle].buffer_size)
		return Tfa98xx_Error_Bad_Parameter;
	i2c_error =
	    NXP_I2C_WriteRead(handles_local[handle].slave_address, num_bytes,
			  data, 0, NULL);
	return tfa98xx_classify_i2c_error(i2c_error);
}

#define NEWPARMREG 48
static enum Tfa98xx_Error dsp_ack_wait(Tfa98xx_handle_t handle)
{
	unsigned short new_param;
	int loop=50;

	do {
		msleep_interruptible(1); /* wait 1ms to avoid busload */
		reg_read(handle, NEWPARMREG, &new_param);
		if (new_param==0)
			return Tfa98xx_Error_Ok;
	} while(loop--);

	return Tfa98xx_Error_DSP_not_running;
}

enum NXP_I2C_Error MMAP_WriteRead(unsigned char slave_addr, int num_write_bytes,
		const unsigned char write_data[], int num_read_bytes, unsigned char read_data[] )
{
	enum NXP_I2C_Error i2c_error;
	int ret_read_bytes = 0;
	unsigned char *wbuffer = malloc(num_write_bytes+1);
	unsigned char *rbuffer = malloc(num_read_bytes+1);

#ifdef HAL_MMAP
	int write_only = (num_read_bytes == 0) || (read_data == NULL);
	const struct nxp_i2c_device interface = lxDspMmap_device;
	/* Initialize the mmap interface */
	(interface.init)("mmap");
#endif

	if (wbuffer == NULL || rbuffer == NULL) {
		free(wbuffer);
		free(rbuffer);
		return NXP_I2C_BufferOverRun;
	}

	wbuffer[0] = slave_addr;
	memcpy((void*)&wbuffer[1], (void*)write_data, num_write_bytes); // prepend slave address
	rbuffer[0] = slave_addr|1; //read slave

#ifdef HAL_MMAP
	if (write_only) {
		/* Write only I2C transaction */
		ret_read_bytes = (interface.write_read)(0, num_write_bytes+1, wbuffer, 0, NULL, &i2c_error);
	} else {
		/* WriteRead I2C transaction */
		ret_read_bytes = (interface.write_read)(0, num_write_bytes+1, wbuffer, num_read_bytes+1, rbuffer, &i2c_error);
	}
#else
	pr_debug("Error: MMAP access is not enabled or not supported (Windows)\n");
	i2c_error = NXP_I2C_UnassignedErrorCode;
#endif

	if(num_read_bytes) {
		if(ret_read_bytes > 0) {
			memcpy((void*)read_data, (void*)&rbuffer[1], ret_read_bytes-1); // remove slave address
		} else {
			pr_debug("empty read \n");
		}
	}

	free(wbuffer);
	free(rbuffer);
	return i2c_error;
}

enum Tfa98xx_Error
mmap_write_register16(Tfa98xx_handle_t handle,
			unsigned char subaddress, unsigned short value)
{
	enum NXP_I2C_Error i2c_error;
	char *scribo_name = malloc(FILENAME_MAX);
	unsigned char write_data[3]; /* subaddress and 2 bytes of the value */

	write_data[0] = subaddress;
	write_data[1] = (value >> 8) & 0xFF;
	write_data[2] = value & 0xFF;

	/* If the name contains a . assume it is an IP-adress */
	NXP_I2C_Scribo_Name(scribo_name, FILENAME_MAX);
	if(strchr(scribo_name, '.')) {
		i2c_error = NXP_I2C_WriteRead(handles_local[handle].slave_address, sizeof(write_data), write_data, 0, NULL);
	} else {
		i2c_error = MMAP_WriteRead(handles_local[handle].slave_address, sizeof(write_data), write_data, 0, NULL);
	}

	free(scribo_name);
	return tfa98xx_classify_i2c_error(i2c_error);
}

enum Tfa98xx_Error
mmap_read_register16(Tfa98xx_handle_t handle,
		       unsigned char subaddress, unsigned short *pValue)
{
	enum NXP_I2C_Error i2c_error;
	unsigned char write_data[1]; /* subaddress */
	unsigned char read_buffer[2]; /* 2 data bytes */
	char *scribo_name = malloc(FILENAME_MAX);

	write_data[0] = subaddress;

	/* If the name contains a . assume it is an IP-adress */
	NXP_I2C_Scribo_Name(scribo_name, FILENAME_MAX);
	if(strchr(scribo_name, '.')) {
		i2c_error = NXP_I2C_WriteRead(handles_local[handle].slave_address,
				sizeof(write_data), write_data, sizeof(read_buffer), read_buffer);
	} else {
		i2c_error = MMAP_WriteRead(handles_local[handle].slave_address,
				sizeof(write_data), write_data, sizeof(read_buffer), read_buffer);
	}

	free(scribo_name);

	if (tfa98xx_classify_i2c_error(i2c_error) != Tfa98xx_Error_Ok) {
		return tfa98xx_classify_i2c_error(i2c_error);
	} else {
		*pValue = (read_buffer[0] << 8) + read_buffer[1];
		return Tfa98xx_Error_Ok;
	}
}

enum Tfa98xx_Error mmap_dsp_msg(Tfa98xx_handle_t handle, int length, const char *buf)
{
	enum Tfa98xx_Error error;
	enum NXP_I2C_Error i2c_error;
	char *scribo_name = malloc(FILENAME_MAX);
	unsigned char *write_data = malloc(length+1);
	unsigned char slave_address = 0x04;
	int result;

	write_data[0] = 0x01; /* offset */
	memcpy((void*)&write_data[1], buf, length);

	/* If the name contains a . assume it is an IP-adress */
	NXP_I2C_Scribo_Name(scribo_name, FILENAME_MAX);
	if(strchr(scribo_name, '.')) {
		i2c_error = NXP_I2C_WriteRead(slave_address, sizeof(unsigned char)*(length+1), write_data, 0, NULL);
	} else {
		i2c_error = MMAP_WriteRead(slave_address, sizeof(unsigned char)*(length+1), write_data, 0, NULL);
	}
	
	free(scribo_name);
	free(write_data);

	if (tfa98xx_classify_i2c_error(i2c_error) != Tfa98xx_Error_Ok) {
		pr_debug("[%s] Status DSP = %d \n", __FUNCTION__, i2c_error);
		return tfa98xx_classify_i2c_error(i2c_error);
	}

	/* notify the DSP */
	reg_write(handle, NEWPARMREG, 0x01);

	error = dsp_ack_wait(handle);
	if (error == Tfa98xx_Error_Ok) {
		/* get the result from the DSP */
		mem_read(handle, 0, 1, &result);
		if (result) /* only print if bad result */
			pr_debug("[%s] Status DSP = %d \n", __FUNCTION__, result);
	}

	return error;
}

enum Tfa98xx_Error mmap_dsp_read_mem(Tfa98xx_handle_t handle,
		unsigned int start_offset, int num_words, int *pValues)
{
	enum NXP_I2C_Error i2c_error;
	const int bytes_per_word = 3;
	unsigned char write_data[1];
	unsigned char *read_buffer;
	unsigned char slave_address;
	unsigned int mmap_offset=0;
	int num_read_bytes = num_words * bytes_per_word;
	char *scribo_name = malloc(FILENAME_MAX);
	(void)(handle); /* Remove unreferenced warning */

	read_buffer = malloc(num_words*bytes_per_word);

	/* Remove the high bit and devide by 256 to get the actuall offset */
	mmap_offset = (start_offset & 0x0FFFF) / 256;
	/* Increase the slave address with the offset. Because this is shifted inside the MMAP we need to multiple */
	slave_address = (unsigned char)(0x04 + (mmap_offset*2));
	write_data[0] = (unsigned char)start_offset;

	NXP_I2C_Scribo_Name(scribo_name, FILENAME_MAX);
	/* If the name contains a . assume it is an IP-adress */
	if(strchr(scribo_name, '.')) {
		i2c_error = NXP_I2C_WriteRead(slave_address, sizeof(write_data), write_data, num_read_bytes, read_buffer);
	} else {
		i2c_error = MMAP_WriteRead(slave_address, sizeof(write_data), write_data, num_read_bytes, read_buffer);
	}

	free(scribo_name);
	
	if (tfa98xx_classify_i2c_error(i2c_error) != Tfa98xx_Error_Ok) {
		free(read_buffer);
		return tfa98xx_classify_i2c_error(i2c_error);
	} else {
		tfa98xx_convert_bytes2data(num_read_bytes, read_buffer, pValues);
		free(read_buffer);
		return Tfa98xx_Error_Ok;
	}
}

enum Tfa98xx_Error mmap_dsp_msg_read(Tfa98xx_handle_t handle,int length, unsigned char *bytes)
{
	enum NXP_I2C_Error i2c_error;
	unsigned char write_data[1];
	unsigned char slave_address = 0x04;
	char *scribo_name = malloc(FILENAME_MAX);
	(void)(handle); /* Remove unreferenced warning */

	write_data[0] = 0x02; /* offset */
	
	/* If the name contains a . assume it is an IP-adress */
	NXP_I2C_Scribo_Name(scribo_name, FILENAME_MAX);
	if(strchr(scribo_name, '.')) {
		i2c_error = NXP_I2C_WriteRead(slave_address, sizeof(write_data), write_data, length, bytes);
	} else {
		i2c_error = MMAP_WriteRead(slave_address, sizeof(write_data), write_data, length, bytes);
	}

	free(scribo_name);
	return tfa98xx_classify_i2c_error(i2c_error);
}

enum Tfa98xx_Error
mmap_dsp_write_mem_word(Tfa98xx_handle_t handle, unsigned short address, int value, int memtype)
{
	enum NXP_I2C_Error i2c_error;
	unsigned char write_data[4]; /* subaddress and 2 bytes of the value */
	unsigned char bytes[3];
	unsigned char slave_address, mmap_offset=0;
	char *scribo_name = malloc(FILENAME_MAX);
	(void)(handle); /* Remove unreferenced warning */
	(void)(memtype); /* Remove unreferenced warning */

	/* Remove the high bit and devide by 256 to get the actuall offset */
	mmap_offset = (unsigned char)((address & 0x0FFFF) / 256);
	/* Increase the slave address with the offset. Because this is shifted inside the MMAP we need to multiple */
	slave_address = 0x04 + (mmap_offset*2);

	tfa98xx_convert_data2bytes(1, &value, bytes);
	write_data[0] = (unsigned char)address;
	memcpy(&write_data[1], bytes, 3);

	/* If the name contains a . assume it is an IP-adress */
	NXP_I2C_Scribo_Name(scribo_name, FILENAME_MAX);
	if(strchr(scribo_name, '.')) {
		i2c_error = NXP_I2C_WriteRead(slave_address, sizeof(write_data), write_data, 0, NULL);
	} else {
		i2c_error = MMAP_WriteRead(slave_address, sizeof(write_data), write_data, 0, NULL);
	}

	free(scribo_name);

	if (tfa98xx_classify_i2c_error(i2c_error) != Tfa98xx_Error_Ok) {
		pr_debug("[%s] Status DSP = %d \n", __FUNCTION__, i2c_error);
		return tfa98xx_classify_i2c_error(i2c_error);
	}

	return Tfa98xx_Error_Ok;
}
