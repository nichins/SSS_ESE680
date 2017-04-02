/**
*  @file		nm_bsp_mc96F4548.c				
*  @brief		This module contains NMC1000 ABOV bsp APIs implementation 
*  @author		M.S.M
*  @date		22 OCT 2012
*  @version		1.0	
*/
#ifdef __MCF964548__
#include "bsp\include\nm_bsp.h"
#include "common\include\nm_common.h"
#include "bus_wrapper\include\nm_bus_wrapper.h"
#include "bsp\source\bsp_mc96f4548\MMA7660FC.h"

#define ADDR_TEMP_SENS 0x70
#define BSP_M_CLK_MHZ		(4)

#define LONG_BTN_PRESS_COUNT			(50)


/*GLOBAL*/
static volatile tpfNmBspIsr data gpfIsr;
static volatile tpfNmBspIsr data gpfIsrwake;
static volatile tstrTimer data  gstrTimer;
static volatile tpfNmBspBtnPress data gpfBtns;
static volatile tpfNmMotionDetectCb data gpfMSens;
static volatile uint32 data gu32Jiffies;
static volatile uint32 data	gu32BtnCounter;
/**/
static void nm_bsp_gpio_init(void)
{									

	// nc	nc	 nc   nc   nc	nc	 nc   nc		P2 control
	// b7	b6	 b5   b4   b3	b2	 b1   b0	
	P2IO		= 0xFF; 				// out	out  out  out  out	out  out  out
	P2OD		= 0x00; 				// PP	PP	 PP   PP   PP	PP	 PP   PP
	P2PU		= 0xFF; 				// off	off  off  off  off	off  off  off
	P2			= 0xFF; 				// 0	0	 0	  0    0	0	 0	  0
	P2DB		= 0x00; 				// 0	0	 0	  0    0	0	 0	  0

	P1IO		= 0x00; 				// in	 in    in	 in    in	 in    in	 in
	P1OD		= 0x00; 				// OD	 OD    OD	 OD    OD	 OD    OD	 OD
	P1PU		= 0x00; 				// on	 on    on	 on    on	 on    on	 on
	P1			= 0x00; 				// 1	 1	   1	 1	   1	 1	   1	 1
	P1DB		= 0x00; 				// 0	 0	   0	 0	   0	 0
	// b7	b6	 b5   b4   b3	b2	 b1   b0
	/*ledD3 =P36,D4 =P37*/				// nc	nc	 nc   nc   nc	nc	 nc   nc		P3 function
	P3IO		= 0xFF;/*LEDS*/ 		// out	out  out  out  out	out  out  out
	P3OD		= 0x00; 				// OD	OD	 PP   PP   PP	PP	 PP   PP
	P3PU		= 0xFF; 				// on	on	 off  off  off	off  off  off
	P3			= 0xFF; 				// 0	0	 0	  0    0	0	 0	  0
	P3DB		= 0x00; 				// 0	0	0	 0	  0    0	0	 0
	PSR 		= 0x00; 				//
	// b7	b6	 b5   b4   b3	b2	 b1   b0
	// -	-	 -	  CEC  SDA1 SCL1 SDA0 SCL0		P4 cntrol
	P4IO		= 0x1F; 				// -	-	 -	  out  out	out  out  out
	P4OD		= 0x1F; 				// -	-	 -	  OD   OD	OD	 OD   OD
	P4PU		= 0x1F; 				// -	-	 -	  on   on	on	 on   on
	P4			= 0x1F; 				// -	-	 -	  1    1	1	 1	  1
	P4DB		= 0x00; 				// -	-	 -	  0    0	0	 0	  0

	// b7	b6	 b5   b4   b3	b2	 b1   b0
	// -	-	 -	  XIN  XOUT SXIN SXOU nTest 	P5 control
	P5IO		= 0x1F; 				// -	-	 -	  out  out	out  out  out
	P5OD		= 0x00; 				// -	-	 -	  pp   pp	pp	 pp   pp
	P5PU		= 0x00; 				// -	-	 -	  off  off	off  off  off
	P5			= 0x00; 				//	-	 -	  -   0    0	0	 0	  0
	P5DB		= 0x00; 				// -	-	 -	  0    0	0	 0	  0
}
static void nm_i2c_init(void)
{

	P4IO		|= (BIT3 + BIT2); 				// out	out  out  out  out	out  out  out
	P4OD		&=~(BIT3 + BIT2); 				// PP	PP	 PP   PP   PP	PP	 PP   PP
	P4PU		&=~(BIT3 + BIT2); 				// off	off  off  off  off	off  off  off
	P4			|= (BIT3 + BIT2); 				// 0	0	 0	  0    0	0	 0	  0
	P4DB		&=~(BIT3 + BIT2); 				// 0	0	 0	  0    0	0	 0	  0

	I2CMR1 = 0x40;
	I2CSCLLR1 = 1;
	I2CSCLHR1 = 1;
	I2CSDAHR1 = 1;
	I2CSR1 = 0x00;
}
static void nm_i2c_deinit(void)
{
	/*i2c deinit*/
	I2CMR1 = 0x00; 
}

/**
*	@fn		nm_bsp_init
*	@brief	Initialize BSP
*	@return	0 in case of success and -1 in case of failure
*	@author	M.S.M
*	@date	22 OCT 2012
*	@version	1.0
*/
sint8 nm_bsp_init(void) 
{
	sint8 result = M2M_SUCCESS;
	nm_bsp_gpio_init();
#ifndef  _EVAL_ABOV_	
	nm_i2c_init();
#endif
	
	nm_bsp_buzzer(50);
	return result;	 
}
#ifdef _STATIC_PS_
/**
*	@fn		nm_bsp_register_wake_isr
*	@brief	REGISTER wake up timer
*	@author	M.S.M
*	@date	12 FEB 2013
*	@version	1.0
*/
void nm_bsp_register_wake_isr(tpfNmBspIsr pfIsr)
{
	gpfIsrwake = pfIsr;
}
/**
*	@fn		nm_bsp_wake_ctrl
*	@brief	control wake up timer
*	@author	M.S.M
*	@date	12 FEB 2013
*	@version	1.0
*/
void nm_bsp_wake_ctrl(uint8 en)
{
	if(en)
	{
		BCCR  = 0x0F;                       // 65mS BIT start
		IE3   |= 0x08;

	}
	else
	{
		IE3   &=~0x08;
		BCCR  = 0x00;
	}
}
#endif
#if (defined _STATIC_PS_)||(defined _DYNAMIC_PS_)
/**
*	@fn		nm_bsp_enable_mcu_ps
*	@brief	Start POWER SAVE FOR MCU
*	@author	M.S.M
*	@date	12 FEB 2013
*	@version	1.0
*/
void nm_bsp_enter_mcu_ps(void)
{
	if(gu32BtnCounter == 0)
	{
		SCCR  = 0x80;                       // change to internal oscillator 8MHz
		SCCR  = 0x84;
		PCON = 0x03;
		_nop_ ();
		_nop_ (); 
		_nop_ (); 
		_nop_ (); 
	}

}
#endif
/**
*	@fn		nm_bsp_buzzer
*	@brief	beep with the given delay
*	@author	M.S.M
*	@date	22 OCT 2012
*	@version	1.0
*/
void nm_bsp_buzzer(uint8 mdelay)
{
#ifdef _EVAL_ABOV_ 
	BUZDR	  = 0xFF;	   
	BUZCR	  = 0x01;
	nm_bsp_sleep(mdelay);
	BUZCR	  = 0x00;
#endif
}
/*
*	@fn		nm_bsp_btn_init
*	@brief	Initialize buttons driver
*	@author	M.S.M
*	@date	01 August 2012
*	@version	1.0
*/
void nm_bsp_btn_init(tpfNmBspBtnPress pfBtnCb)
{
	// out	out  out  out  out	out  out  out
	// D7	 D6    D5	 D4    D3	 D2    D1	 D0    P1 control									// MOSI1 MISO1 SCK1  MOSI0 MISO0 USS1  SCK0  USS0
	P3IO		&=~BIT5; 				// in	 in    in	 in    in	 in    in	 in
	P3OD		&=~BIT5; 				// OD	 OD    OD	 OD    OD	 OD    OD	 OD
	P3PU		|= BIT5; 				// on	 on    on	 on    on	 on    on	 on
	P3			&=~BIT5; 				// 1	 1	   1	 1	   1	 1	   1	 1
	P3DB		|= BIT5; 				// 0	 0	   0	 0	   0	 0

	EIEDGE	|=BIT3; 					// edge trigger
	EIPOLA	|=BIT3; 					// falling edge
	EIBOTH	&=~BIT3; 					// falling edge only			//
	IP &=~BIT3;
	IP1 &=~BIT3;

	EIENAB |= BIT3;
	IE|=(BIT3 << 1);
	gpfBtns = pfBtnCb;
}
/*
*	@fn		nm_bsp_motionsen_init
*	@brief	Initialize Motion sensor
*	@author	M.S.M
*	@date	01 August 2012
*	@version	1.0
*/
void nm_bsp_motionsen_init(tpfNmMotionDetectCb pfsensCb)
{
#ifndef  _EVAL_ABOV_
	gpfMSens = pfsensCb;
	mma7660fc_init();	
	/**/											// out	out  out  out  out	out  out  out
	// D7	 D6    D5	 D4    D3	 D2    D1	 D0    P1 control									// MOSI1 MISO1 SCK1  MOSI0 MISO0 USS1  SCK0  USS0
	P4IO		&=~BIT4; 				// in	 in    in	 in    in	 in    in	 in
	P4OD		&=~BIT4; 				// OD	 OD    OD	 OD    OD	 OD    OD	 OD
	P4PU		&=~BIT4; 				// on	 on    on	 on    on	 on    on	 on
	P4			&=~BIT4; 				// 1	 1	   1	 1	   1	 1	   1	 1
	P4DB		&=~BIT4; 				// 0	 0	   0	 0	   0	 0


	EIEDGE	    &= ~BIT4; 					// level trigger
	EIPOLA	    &= ~BIT4; 					// active high

	IP &=~BIT4;
	IP1 |=BIT4;

	EIENAB |= BIT4;
	IE4|=BIT0;
	/**/
#endif
}
/*
*	@fn		nm_bsp_sleep
*	@brief	Sleep in units of mSec
*	@param[IN]	u32TimeMsec
*				Time in milliseconds
*	@author	M.S.M
*	@date	22 OCT 2012
*	@version	1.0
*/
void nm_bsp_sleep(uint32 u32TimeMsec) 
{
	while(u32TimeMsec--) nm_bsp_usleep(1000);
}
/*
*	@fn		nm_bsp_register_isr
*	@brief	Register interrupt service routine
*	@param[IN]	pfIsr
*				Pointer to ISR handler
*	@author	M.S.M
*	@date	29 July 2012
*	@sa		tpfNmBspIsr
*	@version	1.0
*/
void nm_bsp_register_isr(tpfNmBspIsr pfIsr)
{    
	// out	out  out  out  out	out  out  out
	// D7	 D6    D5	 D4    D3	 D2    D1	 D0    P1 control									// MOSI1 MISO1 SCK1  MOSI0 MISO0 USS1  SCK0  USS0
	P3IO		&=~BIT4; 				// in	 in    in	 in    in	 in    in	 in
	P3OD		&=~BIT4; 				// OD	 OD    OD	 OD    OD	 OD    OD	 OD
	/* Enable Pull up on NMC1000 interrupt line. */
	P3PU		|= BIT4; 				// on	 on    on	 on    on	 on    on	 on
	P3			&=~BIT4; 				// 1	 1	   1	 1	   1	 1	   1	 1
	P3DB		&=~BIT4; 				// 0	 0	   0	 0	   0	 0


	EIEDGE	    &= ~BIT1; 					// level trigger
	EIPOLA	    |= BIT1; 					// active low

	IP |=BIT1;
	IP1 |=BIT1;

	EIENAB |= BIT1;
	IE|=(BIT1 << 1);
	gpfIsr = pfIsr;
}
/*
*	@fn		nm_bsp_interrupt_ctrl
*	@brief	Enable/Disable interrupts
*	@param[IN]	u8Enable
*				'0' disable interrupts. '1' enable interrupts
*	@author	M. Abdelmawla
*	@date	29 July 2012
*	@version	1.0
*/
void nm_bsp_interrupt_ctrl(uint8 u8Enable) 
{
	if(u8Enable)
	{
		IE |= (BIT1 << 1);	
	}
}
/**
*	@fn		nm_bsp_reset(void)
*	@brief	reset chip
*	@author	M.S.M
*	@date	22 OCT 2012
*	@version	1.0
*/
void nm_bsp_reset(void) 
{
	P2 &= ~RESET;	// Assert reset signal. Will be de-asserted after SPI init
	nm_bsp_sleep(200);
	P2 |= RESET;
	nm_bsp_sleep(100);
	EA 	= ON; 
}
/**
*	@fn		nm_bsp_timer_start(tpfNmBspTimerCb pfCb, uint32 u32PeriodTick)
*	@brief	timer init
*	@param[out]	fp
*				callback function for timer handler 
*	@param[IN]	u32Period
*			 toggle period (tick resolution =2.5ms)
*	@author	M.S.M
*	@date	22 OCT 2012
*	@version	1.0
*/
void nm_bsp_timer_start(tpfNmBspTimerCb pfCb,uint32 u32PeriodTick) 
{
	T3CR	= 0x94; 					// 1/4MHz * 256 = 64us
	T3CR1	= 0x00; 					//
	T3DRH	= 0x03; 					// internal 4MHz RC Oscillaor
	T3DRL	= 0x20;						// 64us x 800 = 51.200ms@4MHz interval time tick
	T3CR	= 0x97; 					// timer3 start
	IE2    |= 0x08; 					// -	-	 ADC  T4  (T3)	T2	 T1   T0
	gu32Jiffies =0;
	gstrTimer.u32Timeout = u32PeriodTick + NM_BSP_TIME_MSEC;
	gstrTimer.u32Period = u32PeriodTick;
	gstrTimer.pfCb  = pfCb;
}
/**
*	@fn		nm_bsp_stop_timer
*	@brief	Start periodic timer
*	@author	M. Abdelmawla
*	@date	01 August 2012
*	@version	1.0
*/
void nm_bsp_stop_timer(void)
{
	gstrTimer.pfCb = NULL;
}
/**
*	@fn		nm_bsp_usleep
*	@brief	delay micro second 
*	@author	M.S.M
*	@date	22 OCT 2012
*	@version	1.0
*/ 
void nm_bsp_usleep(uint32 u32Delay)
{
	uint32 jj;                              

	jj= (u32Delay/18)*7;                     

	while(jj)  jj--; 
}
/**
*	@fn		void nm_bsp_accx_clear_int(void)
*	@brief	clear motion sensor int 
*	@author	M.S.M
*	@date	22 OCT 2012
*	@version	1.0
*/ 
void nm_bsp_accx_clear_int(void)
{ 
#ifndef  _EVAL_ABOV_
	mma7660fc_clear();
	IE4 |= BIT0;
#endif
}
sint8 nm_bsp_read_temp(char* pcTempStr)
{
#ifndef  _EVAL_ABOV_
	uint8 u8Addr = 0;
	uint8 u8Data = 0;
	tstrNmI2cSpecial strI2c;
	sint8 s8Ret = M2M_SUCCESS;
	strI2c.u8SlaveAdr = ADDR_TEMP_SENS;
	strI2c.pu8Buf1= &u8Addr;
	strI2c.u16Sz1 = 1;
	strI2c.pu8Buf2 = &u8Data;
	strI2c.u16Sz2 = 1; 

	s8Ret = nm_bus_ioctl(NM_BUS_IOCTL_WR_RESTART, &strI2c); 
	/**
	replace the strign 255 by the correct temp. 
	if temp read was successful.
	**/
	if(s8Ret == M2M_SUCCESS) {		
		uint8 u8Tmp, started;
		started = 0; /* do not print leftmost zeros. */
		if(u8Data & 0x80) { /* Below 0C temp. */
			pcTempStr[0] = '-';
			u8Data = ~u8Data + 1; /* Get 2's comp. */
		} else {
			pcTempStr[0] = ' ';
		}
		
		u8Tmp = u8Data/100;
		if(u8Tmp) {
			pcTempStr[1] = u8Tmp+'0';
			started = 1;
		} else {
			pcTempStr[1] = ' ';
		}
		u8Data %= 100; 
		u8Tmp = u8Data/10;
		if(u8Tmp || started) {
			pcTempStr[2] = u8Tmp+'0';
			started = 1;
		} else {
			pcTempStr[2] = ' ';
		}
		u8Data %= 10;
		u8Tmp = u8Data;
		if(u8Tmp || started) {
			pcTempStr[3] = u8Tmp+'0';
		} else {
			pcTempStr[3] = ' ';
		}
	}
	return s8Ret;
#else
	return -1;
#endif
}
/*NMC1000 interrupt handler: level*/
void Ext1_Int_Handler(void) interrupt 2    using 1
{
	/* Must do this first thing in ISR. */
	IE &= ~(BIT1 << 1);
	EIFLAG &= ~BIT1;

	if(gpfIsr)
	{
		gpfIsr();
	}
}
/*btn handlers*/
void Ext3_Int_Handler(void) interrupt 4    using 1  /*button*/
{
#if 0
	if(gpfBtns)
		{
			gpfBtns();
		}
#endif
	gu32BtnCounter = LONG_BTN_PRESS_COUNT;  
}
/*Motion Sensor int*/
void Ext4_Int_Handler(void) interrupt 24    using 1  /*Motion Sensor int*/
{
	/* Must do this first thing in ISR. */
	IE4 &= ~BIT0;
	if(gpfMSens)
	{
		gpfMSens();
	}
}
/*timer handler*/
void T3_Int_Handler(void) interrupt 15    using 1  
{	
	gu32Jiffies++;
	if(gu32BtnCounter != 0)
	{
		gu32BtnCounter --;
		if(gu32BtnCounter == 0)
		{
			if(gpfBtns)
			{
				gpfBtns(LONG_BTN_PRESS);
			}
		}
		else if(P3 & BIT5)
		{
			if(gpfBtns)
			{
				gpfBtns(SHORT_BTN_PRESS);
			}
			gu32BtnCounter = 0;
		}
	}
	if(gstrTimer.pfCb)
	{
		if(NM_BSP_TIME_MSEC >= gstrTimer.u32Timeout)
		{
			gstrTimer.pfCb();
			gstrTimer.u32Timeout = 	NM_BSP_TIME_MSEC + gstrTimer.u32Period;
		}
	}
}
#ifdef _STATIC_PS_
/*BIT timer*/
void BIT_Int_Handler(void)  interrupt 21   using 1
{
	_nop_ ();
	_nop_ ();
	_nop_ (); 
	_nop_ (); 
	_nop_ ();
	if(gpfIsrwake)
	{
		gpfIsrwake();
	}
}
#endif
#endif
