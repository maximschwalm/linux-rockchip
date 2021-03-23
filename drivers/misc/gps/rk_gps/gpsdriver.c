///////////////////////////////////////////////////////////////////////////////////
//
// Filename: gpsdriver.c
// Author:	sjchen
// Copyright: 
// Date: 2012/07/09
// Description:
//			GPS driver
//
// Revision:
//		0.0.1
//
///////////////////////////////////////////////////////////////////////////////////
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/printk.h>
#include <asm/system.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/uaccess.h>
#include <linux/clk.h>
#include <linux/adc.h>

#include <asm/io.h>

//#include <mach/clock.h>


#include "gpsdrv.h"
#include "gpsdriver.h"
#include "lnxdrv.h"
#include "PALAPI.h"

#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <mach/io.h>
#include <mach/irqs.h>

#include "rk_gps.h"

///////////////////////////////////////////////////////////////////////////////////
// 
// macro declaration
//
///////////////////////////////////////////////////////////////////////////////////
#define GPS_MAJOR								( 61 )
//#define GPS_USE_SPI						


///////////////////////////////////////////////////////////////////////////////////
// 
// static variables declaration
//
///////////////////////////////////////////////////////////////////////////////////
static char   gpsstr[]="gps";
static struct miscdevice  gps_miscdev;
struct rk_gps_data *hv5820b_pdata ;
void *gps_mem =NULL;
unsigned long gps_mem_address; 		//BSP define for u32MemoryPhyAddr
unsigned long gps_mem_size = 4*0x100000;    //it must be more than 8MB

unsigned long u32GpsRegBase;
unsigned long u32MemoryPhyAddr;
unsigned long u32MemoryVirAddr;
unsigned long u32GpsMemroy;
static int *pGpsMem;



///////////////////////////////////////////////////////////////////////////////////
// 
// extern variables declaration
//
///////////////////////////////////////////////////////////////////////////////////
//extern unsigned long gps_mem_address; //BSP define for u32MemoryPhyAddr
//extern unsigned long gps_mem_size;    //it must be more than 8MB



#define GPS_SCLK  hv5820b_pdata->GpsSpiClk
#define GPS_MOSI  hv5820b_pdata->GpsSpiMOSI
#define GPS_SCS   hv5820b_pdata->GpsSpi_CSO

#include <linux/adc.h>
struct adc_client *adc_gps_cleint;
#define LOW_SCALE_0_65V 200
#define HIGH_SCALE_0_95V 295
void gps_callback(struct adc_client *client, void *callback_param, int result)
{
        return;
}


int GetVtuneAdcValue_rk()
{
	int adc_val = 0;
	
	adc_val = adc_sync_read(adc_gps_cleint);
	if(adc_val < 0)
        	printk("GetVtuneAdcValue error");
	
	return adc_val;
}

int AdcValueGet()
{
	// Read adc value
	int adc_value;
	int sampletime = 100;
	int AveValue;
	int SumValue;
	int i;	

	
	SumValue = 0;
	for(i = 0; i < sampletime; i++)
	{
		adc_value = GetVtuneAdcValue_rk();
		
		SumValue += adc_value;
	}
	AveValue = SumValue / sampletime;
	return AveValue;
}

void gps_spi_delay()
{
	int i;

	for(i = 0; i < 1000; i++);
}

void gps_spi_write_one_bit(int bit)
{
	PAL_Set_GPIO_Pin(GPS_SCLK);
	
	if(bit)
		PAL_Set_GPIO_Pin(GPS_MOSI); 
	else
		PAL_Clr_GPIO_Pin(GPS_MOSI);

	gps_spi_delay();

	PAL_Clr_GPIO_Pin(GPS_SCLK);
	gps_spi_delay();
}

void GpsSpiWrite(int Addr, int Val)
{
	int bit_num;
	int SpiVal = 0;

	SpiVal = (Addr & 0xf) << 12;    // [15:12] is address
	SpiVal = SpiVal | (0x3 << 10);  // [11:10] is 11 for write op
	SpiVal = SpiVal | (Val << 2);   // [9:2] is data	
	SpiVal = SpiVal & 0xFFFC;       // [1:0] is dummy data 0	


	PAL_Clr_GPIO_Pin(GPS_SCLK);
	PAL_Clr_GPIO_Pin(GPS_SCS);
	gps_spi_delay();

	for(bit_num = 0; bit_num < 16; bit_num++)
	{
		gps_spi_write_one_bit( (SpiVal >> (15-bit_num)) & 1);
	}	
	
	gps_spi_delay();
	PAL_Set_GPIO_Pin(GPS_SCS);
}


void VtuneAndSpiCheck_rk(int enable_check)
{
	int VtuneAvg;
	int SpiValue = 2;
	int cnt = 0;

	if( enable_check == 0)
		return;

	VtuneAvg = 0;	
	while((VtuneAvg < LOW_SCALE_0_65V) || (VtuneAvg > HIGH_SCALE_0_95V))
	{	
		if( (SpiValue >= 8) || (SpiValue <= 0) )
			break;	

		VtuneAvg = AdcValueGet();
		printk("%s:VtuneAvg=%d\n",__func__,VtuneAvg);
		
		if( VtuneAvg < LOW_SCALE_0_65V)
		{
			SpiValue++;
			GpsSpiWrite(0xe, SpiValue);
		}

		if( VtuneAvg > HIGH_SCALE_0_95V)
		{
			SpiValue--;
			GpsSpiWrite(0xe, SpiValue);
		}
		
		if(cnt ++ > 10)
			break;
		mdelay(1);
	}

}


static int hv5820b_gps_probe(struct platform_device *pdev)
{
	int err;
	int irq = 0;
	
	


	struct rk_gps_data *pdata = pdev->dev.platform_data;
	if(!pdata || !pdata->u32GpsPhyAddr)
		return -1;
	
	hv5820b_pdata = pdata;
	
	//clk_enable(clk_get(NULL, "aclk_gps"));
	
	irq = pdata->GpsIrq;
	
	err = request_irq(irq, gps_int_handler, IRQF_DISABLED, gpsstr, NULL );
	if (err)
	{
		printk ( "%s: gps request irq %d failed!\n",__func__, irq);

		return err;
	}

	err = misc_register( &gps_miscdev);
	if (err < 0)
	{
		return err;
	}

	pdata->u32GpsRegBase = ioremap(pdata->u32GpsPhyAddr, pdata->u32GpsPhySize);
	if (!pdata->u32GpsRegBase){
	    	release_mem_region(pdata->u32GpsPhyAddr, pdata->u32GpsPhySize);
		return -EBUSY;
	}


	pdata->u32MemoryVirAddr = ioremap(pdata->u32MemoryPhyAddr, SZ_8M);
	if (!pdata->u32MemoryVirAddr){
	    	release_mem_region(pdata->u32MemoryPhyAddr, SZ_8M);
		return -EBUSY;
	}


	pGpsMem = kmalloc(4*1024*1024, GFP_ATOMIC);
	if(!pGpsMem)
	{
		printk ( "%s: gps alloc %d failed!\n",__func__, pGpsMem);
		return -EBUSY;
	}

	
	//TODO: 
	//Set the GPIO (GPS_VCC_EN) to low level in here
	//
	if(pdata->io_init)
		pdata->io_init();
	
	gpio_direction_output(pdata->GpsVCCEn, GPIO_LOW);
			
	if(pdata->GpsSpiEn)
	{
		adc_gps_cleint = adc_register(pdata->GpsAdcCh, gps_callback, NULL);
	}
	
	
	printk("%s:u32GpsRegBase=0x%p,u32MemoryPhyAddr=0x%p,u32MemoryVirAddr=0x%p,irq=%d\n",__func__,pdata->u32GpsRegBase,pdata->u32MemoryPhyAddr,pdata->u32MemoryVirAddr,irq);
	return 0;	
}
static int hv5820b_gps_remove(struct platform_device *pdev)
{	
	struct rk_gps_data *pdata = pdev->dev.platform_data;

	if(!pGpsMem)
		kfree(pGpsMem);
	
	if(pdata->GpsSpiEn)
	adc_unregister(adc_gps_cleint);
	
	return 0;
}
static int hv5820b_gps_suspend(struct platform_device *pdev,  pm_message_t state)
{
	return 0;	
}
static int hv5820b_gps_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver hv5820b_gps_driver = {
	.probe	= hv5820b_gps_probe,
	.remove = hv5820b_gps_remove,
	.suspend  	= hv5820b_gps_suspend,
	.resume		= hv5820b_gps_resume,
	.driver	= {
		.name	= "gps_hv5820b",
		.owner	= THIS_MODULE,
	},
};

///////////////////////////////////////////////////////////////////////////////////
// 
// Function Name: gps_mod_init
// Parameters: 
// Description:
// Notes: sjchen 2010/11/04
//
///////////////////////////////////////////////////////////////////////////////////
static int __init gps_mod_init(void)
{
	return platform_driver_register(&hv5820b_gps_driver);
}



///////////////////////////////////////////////////////////////////////////////////
// 
// Function Name: gps_mod_exit
// Parameters:
// Description:
// Notes: sjchen 2010/11/04
//
///////////////////////////////////////////////////////////////////////////////////
static void __exit gps_mod_exit ( void )
{
	//Disable baseband interrupt
	WriteGpsRegisterUlong ( BB_INT_ENA_OFFSET, 0 );
	free_irq( GPS_BB_INT_MASK, NULL );

	//unregister_chrdev ( GPS_MAJOR, gpsstr );
	misc_deregister(&gps_miscdev);
	platform_driver_unregister(&hv5820b_gps_driver);
	printk ( "GPS exit ok!\n");
}

///////////////////////////////////////////////////////////////////////////////////
// 
// Function Name:gps_ioctl
// Parameters:
// Description:
// Notes: sjchen 2010/11/04
//
///////////////////////////////////////////////////////////////////////////////////
long gps_ioctl(struct file *file, unsigned int cmd,
	      unsigned long arg) 
{
	int ret = 0;
	BB_DRV_VERSION __user    * pVersion;
	GPS_DRV_INIT   GpsInitStruct;
	
	switch(cmd) 
	{

	case IOCTL_BB_GPS_START:
		//TODO:
		// Gps baseband module initialize in here. 
		// Module power up
		if(hv5820b_pdata->enable_hclk_gps)
			hv5820b_pdata->enable_hclk_gps();
		
		memset(&GpsInitStruct,0,sizeof(GpsInitStruct));
		GpsInitStruct.u32GpsRegBase    = hv5820b_pdata->u32GpsRegBase;                // GPS Reg Base address 
		GpsInitStruct.u32MemoryPhyAddr =  hv5820b_pdata->u32MemoryPhyAddr;		// sample code
		GpsInitStruct.u32MemoryVirAddr = hv5820b_pdata->u32MemoryVirAddr;		//__phys_to_virt(u32MemoryPhyAddr);
		GpsInitStruct.u32GpsMemory = (unsigned long)pGpsMem;
		GpsInitStruct.u32GpsSign = hv5820b_pdata->GpsSign;       //GPIO index
		GpsInitStruct.u32GpsMag = hv5820b_pdata->GpsMag;        //GPIO index
		GpsInitStruct.u32GpsClk = hv5820b_pdata->GpsClk;        //GPIO index
		GpsInitStruct.u32GpsVCCEn = hv5820b_pdata->GpsVCCEn;      //GPIO index
		GpsInitStruct.u32GpsSpi_CSO = hv5820b_pdata->GpsSpi_CSO;    //GPIO index
		GpsInitStruct.u32GpsSpiClk = hv5820b_pdata->GpsSpiClk;     //GPIO index
		GpsInitStruct.u32GpsSpiMOSI = hv5820b_pdata->GpsSpiMOSI;	  //GPIO index
		//TODO:
		//Add other member of struct GpsInitStruct
		Gps_Init( arg,&GpsInitStruct);

		if(hv5820b_pdata->GpsSpiEn)
		VtuneAndSpiCheck_rk(1);
		
		break;

	case IOCTL_BB_UPDATEDATA:
		Gps_UpdateData(arg);
		break;


	case IOCTL_BB_GPS_STOP:
		Gps_Stop();
		
		//TODO:
		// Set the GPIO(GPS_VCC_EN) to low level
		// Close the module clk.
		gpio_direction_output(hv5820b_pdata->GpsVCCEn, GPIO_LOW);
		if(hv5820b_pdata->disable_hclk_gps)
			hv5820b_pdata->disable_hclk_gps();
		break;

	case IOCTL_BB_GET_VERSION:
		pVersion	= ( void __user * ) arg;
		pVersion->u32Major = DRV_MAJOR_VERSION;
		pVersion->u32Minor = DRV_MINOR_VERSION; 
		PAL_Sprintf(pVersion->strCompileTime,"%s,%s",__DATE__,__TIME__);

		break;

	default:
		printk ( "gpsdrv: ioctl number is worng %d\n",cmd);
		break;
	};
	if(cmd == IOCTL_BB_GPS_START)
	printk("%s:cmd=%d\n",__func__,cmd);
	return ret;
}


///////////////////////////////////////////////////////////////////////////////////
// 
// Function Name:gps_read
// Parameters:
// Description: Read gps data
// Notes: sjchen 2010/11/04
//
///////////////////////////////////////////////////////////////////////////////////
ssize_t gps_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	ssize_t  nsize;
	if((nsize = GpsDrv_Read(buf,size)))
	{
		return nsize;
	}
	return -EFAULT;
}

///////////////////////////////////////////////////////////////////////////////////
// 
// Function Name:gps_open
// Parameters:
// Description:
// Notes: sjchen 2010/11/04
//
///////////////////////////////////////////////////////////////////////////////////
static int gps_open (struct inode *inode, struct file *filp)
{		
	printk("%s\n",__func__);
	return 0;          /* success */
}



///////////////////////////////////////////////////////////////////////////////////
// 
// Function Name:gps_close 
// Parameters:
// Description:
// Notes: sjchen 2010/11/04
//
///////////////////////////////////////////////////////////////////////////////////
static int gps_close (struct inode *inode, struct file *filp) 
{	
	printk("%s\n",__func__);
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////
// 
// Function Name: driver struct
// Parameters:
// Description:
// Notes: sjchen 2010/11/04
//
///////////////////////////////////////////////////////////////////////////////////
static const struct file_operations gps_fops = 
{
	. owner   =	THIS_MODULE,
	. open    =	gps_open,
	. release =	gps_close,
	. read    = 	gps_read,
	. unlocked_ioctl   =	gps_ioctl,
};

static struct miscdevice  gps_miscdev = 
{
	.minor   = MISC_DYNAMIC_MINOR,
	.name    = gpsstr,
	.fops    = &gps_fops,
};


///////////////////////////////////////////////////////////////////////////////////
// 
// Module Name:
// Parameters:
// Description:
// Notes: sjchen 2010/11/04
//
///////////////////////////////////////////////////////////////////////////////////
module_init ( gps_mod_init );
module_exit ( gps_mod_exit );


MODULE_LICENSE("GPL");


