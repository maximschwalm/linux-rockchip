/*
 * Infrared driver for Android 4.4 and upwards
 *
 * Copyright (C) 2013 Kelvin Ching <kelvin@techvision.com.cn>
 * Copyright (C) 2014 Joerg Pleumann <joerg.pleumann@medion.com>
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or (at
 *  your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/init.h> 
#include <linux/module.h> 
#include <linux/kernel.h> 
#include <linux/delay.h>
#include <linux/slab.h>
  
#include <linux/fs.h>            
#include <linux/mm.h>            
#include <linux/errno.h>         
#include <linux/types.h>         
#include <linux/fcntl.h>         
#include <linux/cdev.h>         
#include <linux/device.h>         
#include <linux/major.h>         
#include <linux/gpio.h>
#include <mach/iomux.h>

#include <linux/clk.h>
#include <asm/uaccess.h>  
#include <asm/io.h>  
#include <asm/mach-types.h>
#include <mach/board.h>
#include <plat/pwm.h>
  
#define DEV_NAME      "infrared"   
#define DEV_MAJOR     0         
#define DEBUG_ON               1           
                                
#define ired_dbg(fmt,arg...)     if(DEBUG_ON) printk("== ired debug == " fmt, ##arg)
	
struct class *irtest_class = NULL; 

static int      ired_major = DEV_MAJOR;
static dev_t    dev;
static struct   cdev ired_cdev;
static struct   class *ired_class;

#define read_pwm_reg(addr)              __raw_readl(pwm_base + addr)

static struct clk *pwm_clk;
static void __iomem *pwm_base;
u32 divh,div_total,irout;
unsigned long pwm_clk_rate;
int pwm_gpio;

void delay_usec(int useconds)
{
	int msec,usec;
	msec = useconds/1000;
	usec = useconds%1000;
	if (msec)
	  mdelay(msec);
	if (usec)
	  udelay(usec);
}

//  ************************************************************ //
//  Device Open : 
//  ************************************************************ //
static int infrared_open (struct inode *inode, struct file *filp)  
{
    //ired_dbg("infrared : infrared_open\n\n");
    return 0;  
}

//  ************************************************************ //
//  Device Release : 
//  ************************************************************ //
static int infrared_release (struct inode *inode, struct file *filp)  
{  
    //ired_dbg("infrared : infrared_close\n");
    return 0;  
}  
  
static void pwm_enable(int on)
{
	if (on){
		clk_enable(pwm_clk);
		rk_pwm_setup(0, PWM_DIV2, divh, div_total);
		iomux_set(PWM0);
	}
	else{
		pwm_gpio = iomux_mode_to_gpio(PWM0);
		gpio_request(pwm_gpio, "ir_pwm");
		gpio_direction_output(pwm_gpio, GPIO_LOW);
		gpio_free(pwm_gpio);
	}
}  
 
void mark(int time) {
  pwm_enable(1);
  delay_usec(time);
}

void space(int time) {
  pwm_enable(0);
  delay_usec(time);
}

void enableIROut(int khz) {
  div_total = pwm_clk_rate / khz;
  div_total >>= (1 + (PWM_DIV2 >> 9));
  div_total = (div_total) ? div_total : 1;
  divh = div_total/3;
}

//  ************************************************************ //
//  Device Release : 
//  IO Control
//  IOCTL  ired
//
//  "cmd" is used for passing frequency (Hz) and length of
//        pattern
//  "arg" is used for passing the actual on/off pattern in
//        the same format that the Android API expects
//  ************************************************************ //
static long infrared_ioctl (/*struct inode *inode, */struct file *filp,
                           unsigned int cmd, unsigned long arg)  
{
	unsigned int* msg;
	unsigned int __user *argp = (unsigned char __user *)arg;

	int freq = cmd >> 16;
	int length = cmd & 0xffff;

        printk("IR transmit: freq=%d, length=%d\n", freq, length);

	msg = (unsigned int*)kzalloc(length * 4, GFP_KERNEL);
	if (!msg) {
		return -ENOMEM;
	}
	if (copy_from_user(msg, argp, length * 4)) {
		kfree(msg);
		return -EFAULT;
	}

	preempt_disable();
	enableIROut(freq);
        
	int i;
	for (i = 0; i < length; i++) {
		if (i % 2 == 0) {
			mark(msg[i]);
		} else {
			space(msg[i]);
		}
        }

	if (length % 2) {
		space(0);
	}

	preempt_enable();
        
	kfree(msg);

	return 0;  
}  

//  ************************************************************ //
//  File Operation Struct :
//  ************************************************************ //
static struct file_operations infrared_fops =  
{  
    .owner    = THIS_MODULE,  
    .unlocked_ioctl = infrared_ioctl,  
    .open     = infrared_open,       
    .release  = infrared_release,    
};  

//  ************************************************************ //
//  Device Init :
//  ************************************************************ //
static int __init infrared_init(void)  
{  
  int result;  
	u32 id  =  0;
	int pre_div = 38*1000;
	if (0 == ired_major)
	{
		/* auto select a major */
		result = alloc_chrdev_region(&dev, 0, 1, DEV_NAME);
		ired_major = MAJOR(dev);
	}
	else
	{
		/* use load time defined major number */
		dev = MKDEV(ired_major, 0);
		result = register_chrdev_region(dev, 1, DEV_NAME);
	}

	memset(&ired_cdev, 0, sizeof(ired_cdev));

	/* initialize our char dev data */
	cdev_init(&ired_cdev, &infrared_fops);

	/* register char dev with the kernel */
	result = cdev_add(&ired_cdev, dev, 1);
    
	if (0 != result)
	{
		unregister_chrdev_region(dev, 1);
		printk("Error registrating mali device object with the kernel\n");
	}

    ired_class = class_create(THIS_MODULE, DEV_NAME);
    device_create(ired_class, NULL, MKDEV(ired_major, MINOR(dev)), NULL,
                  DEV_NAME);

    if (result < 0)
        return result;  
		
		iomux_set(PWM0);
		pwm_base = rk_pwm_get_base(0);
		pwm_clk = rk_pwm_get_clk(id);
		if (IS_ERR(pwm_clk) || !pwm_clk) {
			printk(KERN_ERR "failed to get pwm clock source\n");
			return -ENODEV;
		}
		pwm_clk_rate = clk_get_rate(pwm_clk);
		div_total = pwm_clk_rate / pre_div;
	
		div_total >>= (1 + (PWM_DIV2 >> 9));
		div_total = (div_total) ? div_total : 1;
		divh = div_total/3;
		clk_enable(pwm_clk);
		rk_pwm_setup(id, PWM_DIV2, divh, div_total);
		
		pwm_gpio = iomux_mode_to_gpio(PWM0);
		gpio_request(pwm_gpio, "ir_pwm");
		gpio_direction_output(pwm_gpio, GPIO_LOW);
		gpio_free(pwm_gpio);
		
    ired_dbg("IRED driver loaded\n");

    return 0;  
}  

//  ************************************************************ //
//  Device Exit :
//  ************************************************************ //
static void __exit infrared_exit(void)  
{  

//    printk("infrared_exit");
	
    device_destroy(ired_class, MKDEV(ired_major, 0));
    class_destroy(ired_class);

    cdev_del(&ired_cdev);
    unregister_chrdev_region(dev, 1);
    ired_dbg("IRED driver unloaded");
}  


module_init(infrared_init);  
module_exit(infrared_exit);  

MODULE_AUTHOR("Kelvin Ching <kelvin@techvision.com.cn>, Joerg Pleumann <joerg.pleumann@medion.com>");
MODULE_DESCRIPTION("IRED driver");
MODULE_LICENSE("GPL");
