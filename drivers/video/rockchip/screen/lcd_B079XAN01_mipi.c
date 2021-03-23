#ifndef __LCD_B079XAN01__
#define __LCD_B079XAN01__

#if defined(CONFIG_MIPI_DSI)
#include "../transmitter/mipi_dsi.h"
#endif
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/io.h>
#include <linux/delay.h>

#if  defined(CONFIG_RK610_LVDS) || defined(CONFIG_RK616_LVDS)
#define SCREEN_TYPE	    	SCREEN_LVDS
#else
#define SCREEN_TYPE	    	SCREEN_MIPI
#endif
#define LVDS_FORMAT         0     //mipi lcd don't need it, so 0 would be ok.
#define OUT_FACE	    	 OUT_D888_P666// OUT_P888


#define DCLK	         65*1000000//70000000    
#define LCDC_ACLK        300000000   //29 lcdc axi DMA

/* Timing */
#define H_PW			64
#define H_BP			56
#define H_VD			768
#define H_FP			60

#define V_PW			50
#define V_BP			30
#define V_VD			1024
#define V_FP			36

#define LCD_WIDTH       119    //uint mm the lenth of lcd active area
#define LCD_HEIGHT      159
/* Other */
#if defined(CONFIG_RK610_LVDS) || defined(CONFIG_RK616_LVDS) || defined(CONFIG_MIPI_DSI)
#define DCLK_POL	1
#else
#define DCLK_POL	0
#endif
#define DEN_POL		0
#define VSYNC_POL	0
#define HSYNC_POL	0

#define SWAP_RB		0
#define SWAP_RG		0
#define SWAP_GB		0

#define RK_SCREEN_INIT 	1
#define mipi_dsi_init(data) 				dsi_set_regs(data, ARRAY_SIZE(data))
#define mipi_dsi_suspend(data) 				dsi_set_regs(data, ARRAY_SIZE(data))


#if defined(RK_SCREEN_INIT)
static struct rk29lcd_info *gLcd_info = NULL;

//high speed mode

static unsigned int pre_initialize[] = {
// **************************************************
// Initizlize  -> Display On after power-on
// **************************************************
// **************************************************
// Power on TC358768XBG according to recommended power-on sequence
// Relase reset (RESX="H")
// Start input REFCK and PCLK
// **************************************************
// **************************************************
// TC358768XBG Software Reset
// **************************************************
    0x00020001, //SYSctl, S/W Reset
    10, 
    0x00020000, //SYSctl, S/W Reset release

// **************************************************
// TC358768XBG PLL,Clock Setting
// **************************************************
    0x00161045, //PLL Control Register 0 (PLL_PRD,PLL_FBD)
    0x00180203, //PLL_FRS,PLL_LBWS, PLL oscillation enable
    1000, 
    0x00180213, //PLL_FRS,PLL_LBWS, PLL clock out enable

// **************************************************
// TC358768XBG DPI Input Control
// **************************************************
    0x0006012C, //FIFO Control Register

// **************************************************
// TC358768XBG D-PHY Setting
// **************************************************
    0x01400000, //D-PHY Clock lane enable
    0x01420000, //
    0x01440000, //D-PHY Data lane0 enable
    0x01460000, //
    0x01480000, //D-PHY Data lane1 enable
    0x014A0000, //
    0x014C0000, //D-PHY Data lane2 enable
    0x014E0000, //
    0x01500000, //D-PHY Data lane3 enable
    0x01520000, //

		0x01000002,	//D-PHY Clock lane control
		0x01020000,	
		0x01040002,	//D-PHY Data lane0 control
		0x01060000,	
		0x01080002,	//D-PHY Data lane1 control
		0x010A0000,	
		0x010C0002,	//D-PHY Data lane2 control
		0x010E0000,	
		0x01100002,	//D-PHY Data lane3 control
		0x01120000,
			
// **************************************************
// TC358768XBG DSI-TX PPI Control
// **************************************************
    0x02101388, //LINEINITCNT
    0x02120000, //
    0x02140004, //LPTXTIMECNT
    0x02160000,
    0x02181303, //TCLK_HEADERCNT
    0x021A0000, //
//    0x021C0000, //TCLK_TRAILCNT
//   0x021E0000,
    0x02200003, //THS_HEADERCNT
    0x02220000, //
    0x02243A98, //TWAKEUPCNT
    0x02260000,
//    0x02280007, //
//   0x022A0000,
    0x022C0002, //THS_TRAILCNT
    0x022E0000, //
    0x02300005, //HSTXVREGCNT
    0x02320000, //
    0x0234001F, //HSTXVREGEN enable
    0x02360000, //
 //   0x02380001, //DSI clock Enable/Disable during LP
 //   0x023A0000, //
    0x023C0003, //BTACNTRL1
    0x023E0004, //
    0x02040001, //STARTCNTRL
    0x02060000, //

// **************************************************
// TC358768XBG DSI-TX Timing Control
// **************************************************
    0x06200001, //Sync Pulse/Sync Event mode setting
    0x06220050, //V Control Register1
    0x0624001E, //V Control Register2
    0x06260400, //V Control Register3
    0x0628020D, //H Control Register1
    0x062A00F5, //H Control Register2
    0x062C0900, //H Control Register3

    0x05180001, //DSI Start
    0x051A0000, //

		// **************************************************		
		// LCDD (Peripheral) Setting		
		// **************************************************		
		5000,		//Delay 5ms
		//exit_sleep		
		0x06021005,	//bit15:8=Packet Type:Short Packet, bit5:0=Data Type DCS Short Write (no parameter)
		0x06040000,	//bit7:6=Word Count(Lower Byte), bit15:8=Word Count(Upper Byte)
		0x06100011,	
		0x06000001,	//Packet Transfer
		100,		//Wait until packet sending finish
			
		// **************************************************		
		// Enable Continuous Clock		
		// **************************************************		
		0x02380001,	//DSI clock Enable/Disable during LP
		0x023A0000,	
		150000,		//Delay 150ms
			
		// **************************************************		
		// LCDD (Peripheral) Setting		
		// **************************************************		
		//set display on		
		0x06021005,	//bit15:8=Packet Type:Short Packet, bit5:0=Data Type DCS Short Write (no parameter)
		0x06040000,	//bit7:6=Word Count(Lower Byte), bit15:8=Word Count(Upper Byte)
		0x06100029,	
		0x06000001,	//Packet Transfer
		100,		//Wait until packet sending finish
			
		// **************************************************		
		// Set to HS mode		
		// **************************************************		
		0x05000086,	//DSI lane setting, DSI mode=HS
		0x0502A300,	//bit set
		0x05008000,	//Switch to DSI mode
		0x0502C300,	
			
		// **************************************************		
		// Host: RGB(DPI) input start		
		// **************************************************		
			
		0x00080037,	//DSI-TX Format setting
		0x0050003E,	//DSI-TX Pixel stream packet Data Type setting
		0x00320000,	//HSYNC Polarity
		0x00040044,	//Configuration Control Register
			
		// **************************************************		
		// LCDD (Peripheral) Setting		
		// **************************************************		
		// set display on		
	//	0x06021005,	//bit15:8=Packet Type:Short Packet, bit5:0=Data Type DCS Short Write (no parameter)
	//	0x06040000,	//bit7:6=Word Count(Lower Byte), bit15:8=Word Count(Upper Byte)
	//	0x06100029,			
	//	0x06000001,	//Packet Transfer		
		100,		//Wait until packet sending finish		
};

static unsigned int suspend_initialize[] = {
		//set display off
		0x06021005,	//bit15:8=Packet Type:Short Packet, bit5:0=Data Type DCS Short Write (no parameter)	
		0x06040000,	//bit7:6=Word Count(Lower Byte), bit15:8=Word Count(Upper Byte)	
		0x06100028,		
		0x06000001,	//Packet Transfer	
		40000,		//Wait more than 1frame	
				
		//Enter LCD sleep mode (How to enter depends on LCD spec. Following setting is a example)			
		//enter sleep		How to set LCD to sleep mode depends on LCD's specification.	
		0x06021005,	//bit15:8=Packet Type:Short Packet, bit5:0=Data Type DCS Short Write (no parameter)	
		0x06040000,	//bit7:6=Word Count(Lower Byte), bit15:8=Word Count(Upper Byte)	
		0x06100010,		
		0x06000001,	//Packet Transfer	
		80000,		//Wait more than 80ms	
				
		//RGB Port Disable (from "SetFrmStop to 1" to "Set RstRstPtr to 1")(Only for TC358768AXBG)		
		0x00328000,	//Set Frame Stop to 1
		40000,		//Wait more than 1frame
		0x00040004,	//Configuration Control Register Parallel input stop
		0x0032C000,	//Set RstPtr to 1
			
		//Stop DSI continuous clock		
		0x02380000,	
		0x023A0000,	
			
		//Disable D-PHY  By this setting, DSI line becomes HiZ.  LCD side should permit HiZ input.		
		0x01400001,
		0x01420000,
		0x01440001,
		0x01460000,
		0x01480001,
		0x014A0000,
		0x014C0001,
		0x014E0000,
		0x01500001,
		0x01520000,
};

int gfirst = 1;
int rk_lcd_init(void)
{	
    int ret = 0;
    
    //power on
    if (gfirst == 1)
    {
   	if(!gLcd_info)
   		return -EIO;
   	
   	ret = gpio_request(gLcd_info->reset_pin, NULL);
   	if (ret != 0) {
   		gpio_free(gLcd_info->reset_pin);
   		printk("%s: request LCD_RST_PIN error\n", __func__);
   		return -EIO;
   	}
   
   	ret = gpio_request(gLcd_info->cs_pin, NULL);
   
   	gfirst++;
    }
    
    gpio_direction_output(gLcd_info->cs_pin,GPIO_HIGH);//enable power
    msleep(50);
    gpio_direction_output(gLcd_info->reset_pin, GPIO_LOW);
    msleep(10);
    gpio_set_value(gLcd_info->reset_pin, !GPIO_LOW);
    msleep(20);
    dsi_probe_current_chip();
    mipi_dsi_init(pre_initialize);
    msleep(20);
    return 0;

}

int rk_lcd_standby(u8 enable)
{
	if(enable) {
    printk("suspend lcd\n");
    mipi_dsi_suspend(suspend_initialize);
    msleep(10);
		if(!gLcd_info)
			return -EIO;
		gpio_set_value(gLcd_info->reset_pin, GPIO_LOW);
		msleep(10);
    dsi_power_off();
	}
	else {
		rk_lcd_init();
	}

  return 0;
}
#endif
#endif  
