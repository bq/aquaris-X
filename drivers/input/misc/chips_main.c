/*************************************************************************
  Copyright£º ShenZhen ChipSailing Technology Co., Ltd. All rights reserved.    
  File name: chips_main.c
  File description: spi dev device driver
  Author: zwp    ID:58    Version:2.0   Date:2016/10/16
  Others:
  History:
    1. Date:           Author:          ID:
	  Modify description:
	  2.
**************************************************************************/

/*********************************Header file***********************************/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/irqreturn.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/wakelock.h>

#include <linux/fb.h>
#include <linux/notifier.h>

#include "chips_main.h"
#include "chips_platform.h"

/**********************************Macro definition************************************/
/*Device information*/
#define N_SPI_MINORS 33                   //The total number of the second device
#define CHIPS_FP_MAJOR 155                //Main device number 

#define VERSION "v2.0.7"
#define CHIPS_DEV_NAME    "cs_spi"        //Dev device name 
#define CHIPS_CLASS_NAME  "chips_fp"      //Class name
#define CHIPS_CHRD_NAME   "chips_fp"      //Device name
#define FB_EVENT_NOTIFIER

/*Key information*/
#define CHIPS_IOC_MAGIC			'k'
#define CHIPS_IOC_INPUT_KEY_EVENT	         _IOW(CHIPS_IOC_MAGIC, 7, struct chips_key_event)
#define CHIPS_IOC_SET_RESET_GPIO		     _IOW(CHIPS_IOC_MAGIC, 8,unsigned char)
//#define CHIPS_IOC_SPI_MESSAGE	             _IOWR(CHIPS_IOC_MAGIC, 9,struct chips_ioc_transfer)
//#define CHIPS_IOC_SPI_SEND_CMD               _IOW(CHIPS_IOC_MAGIC,10,unsigned char)
#define CHIPS_IOC_ENABLE_IRQ                 _IO(CHIPS_IOC_MAGIC,11)
#define CHIPS_IOC_DISABLE_IRQ                _IO(CHIPS_IOC_MAGIC,12)
//#define CHIPS_IOC_SENSOR_CONFIG              _IOW(CHIPS_IOC_MAGIC,13,void*)
#define CHIPS_IOC_DEV_INIT                   _IO(CHIPS_IOC_MAGIC,14)
#define CHIPS_IOC_DEV_UNINST                 _IO(CHIPS_IOC_MAGIC,15)
#define CHIPS_IOC_DEV_UNINIT                 _IO(CHIPS_IOC_MAGIC,16)


#define CHIPS_INPUT_KEY_FINGERDOWN        KEY_F18
#define CHIPS_INPUT_KEY_SINGLECLICK       KEY_F19 
#define CHIPS_INPUT_KEY_DOUBLECLICK 	  KEY_F20  
#define CHIPS_INPUT_KEY_LONGTOUCH     	  KEY_F21
#define CHIPS_INPUT_KEY_UP                KEY_F18
#define CHIPS_INPUT_KEY_DOWN              KEY_F18
#define CHIPS_INPUT_KEY_LEFT              KEY_F18
#define CHIPS_INPUT_KEY_RIGHT             KEY_F18
#define CHIPS_INPUT_KEY_POWER             KEY_POWER

/******************************************
          Global variable definition
******************************************/
const static unsigned bufsiz = 10240*2;   
static int display_blank_flag = -1;
static struct wake_lock g_wakelock;  
struct chips_data *g_spidev;

static DECLARE_BITMAP(minors,N_SPI_MINORS);    
static LIST_HEAD(device_list);                 
static DEFINE_MUTEX(device_list_lock);          

static int chips_dev_init(void);
static int chips_dev_uninst(void);
static int chips_dev_uninit(void);


/*******************************************
          Structure definition
*******************************************/
struct chips_key_event {
	int code;      
	int value;     
};



/*********************************************
          Function definition
*********************************************/
static inline void *chips_get_drvdata(void)
{
	return g_spidev?g_spidev:NULL;	
}

static inline void chips_set_drvdata(struct chips_data* data)
{
	g_spidev = data;
}


static void chips_kill_fasync(void)
{
	struct chips_data *chips_dev;
	chips_dev = chips_get_drvdata();
	
	if(chips_dev!=NULL && chips_dev->async!=NULL){
		kill_fasync(&chips_dev->async,SIGIO,POLL_IN);
	}else{
		chips_dbg("struct *async is null\n");
	}
}

/**
 *  @brief chips_enable_irq enable interrupt
 *  
 *  @param [in] chips_dev structure pointer of chip data
 *  
 *  @return no return value
 */
static void chips_enable_irq(struct chips_data *chips_dev)
{
	if(!chips_dev->irq_enabled){
		enable_irq(chips_dev->irq);
		chips_dev->irq_enabled = true;
	}
}


/**
 *  @brief chips_disable_irq close interrupt
 *  
 *  @param [in] chips_dev structure pointer of chip data
 *  
 *  @return no return value
 */
static void chips_disable_irq(struct chips_data *chips_dev)
{
	if(chips_dev->irq_enabled){
		disable_irq_nosync(chips_dev->irq);
		chips_dev->irq_enabled = false;
	}
}


 /**
 *  @brief chips_register_input register input device
 *  
 *  @param [in] chips_dev structure pointer of chip data
 *  
 *  @return successfully return 0, failure return a negative number
 */
static int chips_register_input(struct chips_data *chips_dev)
{
	chips_dev->input = input_allocate_device();
	if(NULL == chips_dev->input){
		chips_dbg("allocate input device error\n");
		return -ENOMEM;
	}
	 
	__set_bit(EV_KEY,chips_dev->input->evbit);

	__set_bit(CHIPS_INPUT_KEY_FINGERDOWN,chips_dev->input->keybit);
	__set_bit(CHIPS_INPUT_KEY_SINGLECLICK,chips_dev->input->keybit);
	__set_bit(CHIPS_INPUT_KEY_DOUBLECLICK,chips_dev->input->keybit);
	__set_bit(CHIPS_INPUT_KEY_LONGTOUCH,chips_dev->input->keybit);
	__set_bit(CHIPS_INPUT_KEY_UP,chips_dev->input->keybit);
	__set_bit(CHIPS_INPUT_KEY_DOWN,chips_dev->input->keybit);
	__set_bit(CHIPS_INPUT_KEY_LEFT,chips_dev->input->keybit);
    __set_bit(CHIPS_INPUT_KEY_RIGHT,chips_dev->input->keybit);
	__set_bit(CHIPS_INPUT_KEY_POWER,chips_dev->input->keybit);
	
	chips_dev->input->id.bustype = BUS_HOST;
	chips_dev->input->name = "fingerprint_key";
	chips_dev->input->id.version = 1;

	if(input_register_device(chips_dev->input) != 0){
		chips_dbg("input_register_device error\n");
		input_free_device(chips_dev->input);
		return -ENOMEM;
	}
	
	return 0;
}


 /**
 *  @brief chips_irq_handler interrupt handler
 *  
 *  @param [in] irq interrupt number
 *  @param [in] dev structure pointer of chip data
 *  
 *  @return always return IRQ_HANDLED
 */
static irqreturn_t chips_irq_handler(int irq,void *dev)
{
    chips_dbg("irq_handler\n");
    if(!wake_lock_active(&g_wakelock)){
		wake_lock_timeout(&g_wakelock, 6*HZ);
	}
	
	chips_kill_fasync();
	
	return IRQ_HANDLED;
}


/**
 *  @brief chips_read read count bytes of data from fp to buf
 *  
 *  @param [in] fp      file structure pointer represent a file open
 *  @param [in] buf     buffer pointer of user space
 *  @param [in] count   number of bytes to read
 *  @param [in] offset  file offset pointer
 *  
 *  @return successfully return number of bytes to read, failure return a negative number
 */
static ssize_t chips_read(struct file *fp, char __user *buf, size_t count, loff_t *offset)
{
	struct chips_data *chips_dev;
	unsigned long missing;
	int status = -1;
	
	if(NULL == buf)
		return -EINVAL;
	
	if(count > bufsiz)
		return -EMSGSIZE;
	
	chips_dev = (struct chips_data *)fp->private_data;
	
	mutex_lock(&chips_dev->buf_lock);
	missing = copy_to_user(buf,&display_blank_flag,count); 
	if(missing == count ){
		status = -EFAULT;
	}else{ 
		status = count - missing;
	}
	mutex_unlock(&chips_dev->buf_lock);
	
	return status;
}


/**
 *  @brief chips_write chip write, write count bytes of data from buffer to fp
 *  
 *  @param [in] fp      file structure pointer represent a file open
 *  @param [in] buf     buffer pointer of user space
 *  @param [in] count   number of bytes to write, no more than 128 bytes
 *  @param [in] offset  file offset pointer
 *  
 *  @return successfully return number of bytes to write, failure return a negative number
 */
static ssize_t chips_write(struct file *fp, const char __user *buf, size_t count, loff_t *offset)
{
	int status = -1;
	
	struct chips_data *chips_dev;
	unsigned long missing;
	unsigned char buffer[128] = {0};
	
	if(count > 128)
		return -EMSGSIZE;
	
	chips_dev = (struct chips_data*)fp->private_data;
	
	missing = copy_from_user(buffer,buf,count);
	if(missing != 0){
		status = -EFAULT;
	}else{
		if(strncmp(buffer,"irq",strlen("irq"))==0){
			chips_dbg("irq_num = %d\n",chips_dev->irq);
		
		}else if(strncmp(buffer,"reset",strlen("reset"))==0){
			status = chips_hw_reset(chips_dev,1);
			chips_dbg("reset status = %d\n", status);
		
		}
		
	    status = count - missing;
	}	
	
	return status;
}


/**
 *  @brief chips_open  chip open operation
 *  
 *  @param [in] inode  inode structure pointer 
 *  @param [in] fp     file structure pointer 
 *  
 *  @return successfully return 0, failure return a negative number
 */
static int chips_open(struct inode *inode, struct file *fp)
{
	int status = -ENXIO;
	struct chips_data *chips_dev;
	
	mutex_lock(&device_list_lock);
	
	/*Traversing global list and find chips_dev*/
	list_for_each_entry(chips_dev,&device_list,device_entry){
		if(chips_dev->devt == inode->i_rdev){
			status = 0;
			break;
		}
	}
	
	/*Initialize chips_dev->buffer, for chips_dev->buffer apply memory buffer*/
	if (status == 0) {
		if (NULL == chips_dev->buffer){
			chips_dev->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (NULL == chips_dev->buffer) {
				chips_dbg("kmalloc/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		
		/*Users of open the device file, count +1*/
		if (status == 0){
			chips_dev->users++;
			fp->private_data = chips_dev;
			nonseekable_open(inode, fp); //Forbid lseek operation
		}
	}else{
		chips_dbg("no device found with minor = %d\n", iminor(inode));
	}

	mutex_unlock(&device_list_lock);
	return status;
}



/**
 *  @brief chips_release close chip
 *  
 *  @param [in] inode  inode structure pointer  
 *  @param [in] fp     file structure pointer 
 *  
 *  @return always return 0
 */
static int chips_release(struct inode *inode, struct file *fp)
{
	struct chips_data *chips_dev;
	
	mutex_lock(&device_list_lock);
	chips_dev = (struct chips_data*)fp->private_data;
	fp->private_data = NULL;
	
	/*Users of open the device file, count -1*/
	chips_dev->users--;
	
	/*When users count of open device file is zero, release chips_dev->buffer*/
	if (chips_dev->users == 0){
		//int		dofree;

		kfree(chips_dev->buffer);
		chips_dev->buffer = NULL;

		//spin_lock_irq(&chips_dev->spin_lock);       
		//dofree = (chips_dev->spi == NULL);
		//spin_unlock_irq(&chips_dev->spin_lock);

		//if (dofree){
		//	kfree(chips_dev);
		//	chips_set_drvdata(NULL);
		//}
			
	}	

	mutex_unlock(&device_list_lock);
	return 0;
}


/**
 *  @brief chips_ioctl chip ioctl function
 *  
 *  @param [in] fp   file structure pointer 
 *  @param [in] cmd  commond
 *  @param [in] arg  parameters, is usually a pointer
 *  
 *  @return successfully return 0, failure return a negative number
 */
static long chips_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int retval = 0;
	int missing = 0;	
	unsigned char command = 0;

	struct chips_key_event *key = NULL;
	struct chips_data *chips_dev;
	
	/*Command judgment, magic code is 'K'*/
	if(_IOC_TYPE(cmd)!=CHIPS_IOC_MAGIC)
		return -ENOTTY;
		
	/*Kernel interaction environment determine whether can read or write*/
	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg,_IOC_SIZE(cmd));
	if(err==0 && _IOC_DIR(cmd)&_IOC_WRITE)
		err = !access_ok(VERIFY_READ,(void __user*)arg,_IOC_SIZE(cmd));
	if(err)
		return -EFAULT;

    /*Obtain spi_device*/
	chips_dev = (struct chips_data*)fp->private_data;
	//spin_lock_irq(&chips_dev->spin_lock);
	//spi = spi_dev_get(chips_dev->spi); 
	//spin_unlock_irq(&chips_dev->spin_lock);
	
	//if(spi == NULL)
	//	return -ESHUTDOWN;

    /* use the buffer lock here for triple duty:
     *  - prevent I/O (from us) so calling spi_setup() is safe;
     *  - prevent concurrent SPI_IOC_WR_* from morphing
     *    data fields while SPI_IOC_RD_* reads them;
     *  - CHIPS_IOC_SPI_MESSAGE needs the buffer locked "normally".
     */	
	mutex_lock(&chips_dev->buf_lock);

	switch(cmd)
	{
		/*Reset*/
		case CHIPS_IOC_SET_RESET_GPIO:		
			retval = __get_user(command,(u8 __user*)arg);
			if(retval == 0){
				retval = chips_set_reset_gpio(chips_dev,command);
				if(retval < 0){
				    chips_dbg("Failed to send spi cmd,cmd = 0x%x",command);
				    retval = -EFAULT;
				    break;	
				}		
			}
			break;	
			
		case CHIPS_IOC_DEV_INIT:
		    chips_dev_init();
			chips_dbg("dev init\n");
			break;
			
		case CHIPS_IOC_DEV_UNINIT:
			chips_dev_uninit();
			chips_dbg("dev uninit\n");
			break;
			
		case CHIPS_IOC_DEV_UNINST:
		    chips_dev_uninst();
			chips_dbg("dev uninst\n");
			break;
			
		/*Enable IRQ*/
		case CHIPS_IOC_ENABLE_IRQ:
			chips_enable_irq(chips_dev);
			chips_dbg("enable irq\n");
			break;
			
		/*Disable IRQ*/
		case CHIPS_IOC_DISABLE_IRQ:
			chips_disable_irq(chips_dev);
			chips_dbg("disable irq\n");
			break;
		
		/*Key event*/
		case CHIPS_IOC_INPUT_KEY_EVENT:
			chips_dbg("report key event\n");
			
			/*Application memory for key*/
			key = kzalloc(sizeof(*key),GFP_KERNEL);
			if(NULL == key){
				chips_dbg("Failed to allocate mem for key event\n");
				retval = -ENOMEM;
				break;
			}
			
			/*Obtain key data*/
			missing = copy_from_user(key,(u8 __user *)arg,sizeof(*key));
			if(missing != 0){
				chips_dbg("Failed to copy key from user space\n");
				retval = -EFAULT;
				break;
			}
			
			/*Key value type judgment: lift, press, a complete key values*/
			/*Report a complete key value (2)*/
			if(key->value == 2){
				chips_dbg("a key event reported,key->code = 0x%x,key->value = 0x%x\n",key->code,key->value);
				//__set_bit(key->code,chips_dev->input->keybit);
				input_report_key(chips_dev->input,key->code,1);
				input_sync(chips_dev->input);
				input_report_key(chips_dev->input,key->code,0);
				input_sync(chips_dev->input);
				
			/*Press (1) or lift (0)*/ 
			}else{  
				chips_dbg("a key event reported,key->code = 0x%x,key->value = 0x%x\n",key->code,key->value);
				//__set_bit(key->code,chips_dev->input->keybit);
				input_report_key(chips_dev->input,key->code,key->value);
				input_sync(chips_dev->input);
			}
			
			break;
			
		default:
			break;			 
	}
	if(NULL != key){
		kfree(key);
		key = NULL;
	}
		
	mutex_unlock(&chips_dev->buf_lock);
	//spi_dev_put(spi);
	
	return retval;
}

#ifdef CONFIG_COMPAT
/**
 *  @brief chips_compat_ioctl  chip ioctl function of compatible 
 *  
 *  @param [in] fp  file structure pointer 
 *  @param [in] cmd commond
 *  @param [in] arg parameters, is usually a pointer
 *  
 *  @return successfully return 0, failure return a negative number
 */
static long chips_compat_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	return chips_ioctl(fp, cmd, arg);
}
#else
#define chips_compat_ioctl NULL
#endif


static int chips_fasync(int fd, struct file *fp, int mode)
{
	struct chips_data* chips_dev;
	chips_dev = (struct chips_data*)fp->private_data;
	
	return fasync_helper(fd,fp,mode,&chips_dev->async);
}


static const struct file_operations chips_fops = {
	.owner = THIS_MODULE,

	.unlocked_ioctl = chips_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = chips_compat_ioctl,
#endif
	.open = chips_open,
	.release = chips_release,

	.read = chips_read,
	.write = chips_write,
	.fasync = chips_fasync,
};

#if defined(FB_EVENT_NOTIFIER)
static int stk3x1x_fb_event_notify(struct notifier_block *self,unsigned long action, void *data)
 {
	struct fb_event *event = data;
	int blank_mode = *((int *)event->data);
	if(action == FB_EVENT_BLANK){
		switch (blank_mode) 
		{
			case FB_BLANK_UNBLANK:
				display_blank_flag = 0;
				chips_kill_fasync();
				chips_dbg("display_blank_flag = %d\n",display_blank_flag);
				break;
			case FB_BLANK_POWERDOWN:
				display_blank_flag = 1;
				chips_kill_fasync();
				chips_dbg("display_blank_flag = %d\n",display_blank_flag);
				break;
			default:
				break;
		}
	} 
   return NOTIFY_OK;
}

static struct notifier_block stk3x1x_fb_notifier = {
        .notifier_call = stk3x1x_fb_event_notify,
};
#endif

static int chips_dev_inst(void)
{
	int status;
	unsigned long minor;
	struct chips_data *chips_dev;

	chips_dev = kzalloc(sizeof(struct chips_data),GFP_KERNEL);
	if(NULL == chips_dev){
		chips_dbg("kzalloc mem for chips_dev error\n");
		return -ENOMEM;
	}
	
	chips_set_drvdata(chips_dev);
	
	INIT_LIST_HEAD(&chips_dev->device_entry);
	//spin_lock_init(&chips_dev->spin_lock);
	mutex_init(&chips_dev->buf_lock);


	
	/*Parsing DTS file*/
//	if(chips_parse_dts(chips_dev) != 0){
//		chips_dbg("Failed to parse dts\n");
//		return -ENODEV;
//	}
	
	/*IC power on*/
	chips_pwr_up(chips_dev);

	/*Chip hard reset*/
//	chips_hw_reset(chips_dev,1);
	
	/*Set MI¡¢MO*/
//	chips_set_spi_mode(chips_dev);

	/*Distribution main device number, associated file_operation*/
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(CHIPS_FP_MAJOR,CHIPS_CHRD_NAME,&chips_fops);
	if(status < 0){
		chips_dbg("register chrdev error\n");
		return status;
	}
	
	/*Create device class*/
	chips_dev->cls = class_create(THIS_MODULE,CHIPS_CLASS_NAME);
	if(IS_ERR(chips_dev->cls)){
		chips_dbg("class_create error\n");
		unregister_chrdev(CHIPS_FP_MAJOR,CHIPS_CHRD_NAME);
		return PTR_ERR(chips_dev->cls);
	}

	/*Create a device node, and join device to equipment list*/
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors,N_SPI_MINORS);
	if(minor < N_SPI_MINORS){
		struct device *dev;

		chips_dev->devt = MKDEV(CHIPS_FP_MAJOR,minor);
		dev = device_create(chips_dev->cls,NULL,chips_dev->devt,chips_dev,CHIPS_DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev):0;

	}else{
 		chips_dbg("no minor number available,monir = %lu\n",minor);
		status = -ENODEV;
	}

	if(status == 0){
		set_bit(minor,minors);
		list_add(&chips_dev->device_entry,&device_list);
	}else{
		chips_dev->devt = 0;
	}
	
	mutex_unlock(&device_list_lock);
	
	return status;
}


static int chips_dev_uninst(void)
{
	struct chips_data* chips_dev;
	chips_dev =(struct chips_data*)chips_get_drvdata();

	/*Unregister device*/
	mutex_lock(&device_list_lock);
	device_destroy(chips_dev->cls,chips_dev->devt);
	list_del(&chips_dev->device_entry);
	clear_bit(MINOR(chips_dev->devt),minors);
	mutex_unlock(&device_list_lock);
	
	/*Unregister class*/
	class_destroy(chips_dev->cls);
	unregister_chrdev(CHIPS_FP_MAJOR,CHIPS_DEV_NAME);
	
	/*Release gpio*/
	chips_release_gpio(chips_dev);
	
	/*Unregister mutex*/
	mutex_destroy(&chips_dev->buf_lock);
	//mutex_destroy(&device_list_lock);
	
	if(chips_dev != NULL){
		kfree(chips_dev);
		chips_set_drvdata(NULL);
	}
	
	return 0;
}

static int chips_dev_init(void)
{
	int irq;
	int status;
	struct chips_data* chips_dev;
	chips_dev =(struct chips_data*)chips_get_drvdata();	
	
	/*Initialize wake lock*/
	wake_lock_init(&g_wakelock, WAKE_LOCK_SUSPEND, "chips_wakelock");	
	
	/*Register interrupt handler function, can view /proc/interrupts to confirm whether or not registered successfully*/
	irq = chips_get_irqno(chips_dev);
	status = request_irq(irq,chips_irq_handler,IRQF_TRIGGER_RISING | IRQF_ONESHOT,"fngr_irq",chips_dev);
	if(status != 0){
		chips_dbg("request_irq error\n");
		return -1;
	}
	
	enable_irq_wake(irq);
	chips_dev->irq_enabled = true;

/*
	//Enable irq wake up
	status = enable_irq_wake(irq);
	if(status != 0){
		free_irq(irq,chips_dev);
		chips_dbg("enable_irq_wake error\n");
		goto err;
	}
*/

	/*Registration input device to report the key event*/
	chips_register_input(chips_dev);
		
#if defined(FB_EVENT_NOTIFIER)
	fb_register_client(&stk3x1x_fb_notifier);
#endif

    return 0;	
}


static int chips_dev_uninit(void)
{
	struct chips_data* chips_dev;
	chips_dev =(struct chips_data*)chips_get_drvdata();
	
	/*Unregister shared data*/
	//spin_lock_irq(&chips_dev->spin_lock);
	//chips_dev->spi = NULL;
	//spi_set_drvdata(spi, NULL);
	//spin_unlock_irq(&chips_dev->spin_lock);

#if defined(FB_EVENT_NOTIFIER)
	fb_unregister_client(&stk3x1x_fb_notifier);
#endif

	/*Unregister input device*/
	if (chips_dev->input != NULL){
		input_unregister_device(chips_dev->input);
		//Once device has been successfully registered it can be unregistered with 
		//input_unregister_device(); input_free_device() should not be called in this case.
		//input_free_device(chips_dev->input);
	}
	
	/*Release interrupt number*/
	if (chips_dev->irq !=0)
		free_irq(chips_dev->irq, chips_dev);

	/*Unregister wake lock*/
	wake_lock_destroy(&g_wakelock);
		
	return 0;
}

static int __init chips_init(void)
{
	int status = -1;
	
	status = chips_dev_inst();
	if(status != 0){
		chips_dbg("Failed to install device\n");
	}
	
	chips_dbg("install device\n");
	return status;
}

static void __exit chips_exit(void)
{ 
    chips_dev_uninit();
    chips_dev_uninst();
}

module_init(chips_init);
module_exit(chips_exit);

MODULE_AUTHOR("ChipSailing Tech");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:cs15xx");
