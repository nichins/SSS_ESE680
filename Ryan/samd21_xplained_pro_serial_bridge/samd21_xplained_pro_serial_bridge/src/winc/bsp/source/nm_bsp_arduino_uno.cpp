/**
*  @file		nm_bsp_arduino_uno.cpp
*  @brief		This module contains arduino uno bsp APIs implementation
*  @author		M.S.M
*  @date		26 APRIL 2014
*  @version		1.0
*/

#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#include "bsp/include/nm_bsp_arduino_uno.h"



#define BSP_MIN(x,y) ((x)>(y)?(y):(x))

#define SHORT_PRESS_DEBOUNCE	(400/TICK_RES)
#define LONG_PRESS_DEBOUNCE		(1000/TICK_RES)

#define SW1_PIN		3
#define SW2_PIN		7

#define IRQ_PIN     2
#define RESET_PIN   8

#if (0 == SHORT_PRESS_DEBOUNCE)
#undef SHORT_PRESS_DEBOUNCE
#define SHORT_PRESS_DEBOUNCE 1
#endif
/*
* Structure
*
*/
typedef struct
{
	tpfNmBspTimerCb	pfCb;
	uint32 u32Timeout;
	uint32 u32Period;
} tstrTimer;


typedef struct
{
	tstrTimer strTimer;
	uint8 u8Enabled;
} tstrWakeTimer;
/**
*
* Global variables
*/
uint32 gu32Jiffies;

static tpfNmBspIsr gpfIsr;
static uint16 gu16Btn1Cnt, gu16Btn2Cnt;
static tpfNmBspBtnPress gpfBtns;
static uint8 gu8BtnIfg;
static tstrTimer gstrTimer;
#ifdef _STATIC_PS_
static tstrWakeTimer gstrWakeTimer;
#endif
/**
*
* Static functions
*/
static void btn_cb(void)
{
	gu8BtnIfg |= SW1;
	detachInterrupt(1);           // disable all interrupts
}
static void btn_poll(void)
{
	if (gu8BtnIfg & SW1) {
		gu16Btn1Cnt++;

		if (gu16Btn1Cnt >= SHORT_PRESS_DEBOUNCE) {
			if (digitalRead(SW1_PIN) == HIGH)
			{
				gpfBtns(SW1, 0); /* Short press callback */
				gu16Btn1Cnt = 0;
			} else {
				if (gu16Btn1Cnt >= LONG_PRESS_DEBOUNCE) {

					gpfBtns(SW1, 1); /* long press callback */
					gu16Btn1Cnt = 0;
				}
			}
		}
	}
	if (gu8BtnIfg & SW2) 
	{
		gu16Btn2Cnt++;
		if (gu16Btn2Cnt >= SHORT_PRESS_DEBOUNCE) 
		{
			if (digitalRead(SW2_PIN) == HIGH)
			{
				gpfBtns(SW2, 0); /* Short press callback */
				gu16Btn2Cnt = 0;
			} 
			else 
			{
				if (gu16Btn2Cnt >= LONG_PRESS_DEBOUNCE) 
				{
					gpfBtns(SW2, 1); /* long press callback */
					gu16Btn2Cnt = 0;
				}
			}
		}
	}


	if(!gu16Btn2Cnt)
	{
		gu8BtnIfg &= ~SW2;
		PCMSK2 = 0b10000000;
	}
	if(!gu16Btn1Cnt)
	{
		gu8BtnIfg &= ~SW1;
		attachInterrupt(1, btn_cb, FALLING);
	}
}
/*Timer interrupt*/
ISR(TIMER1_COMPA_vect)
{		
	gu32Jiffies++;
	if(gstrTimer.pfCb)
	{
		if(NM_BSP_TIME_MSEC >= gstrTimer.u32Timeout)
		{
			if(gstrTimer.pfCb)
			{
				gstrTimer.pfCb();
			}
			gstrTimer.u32Timeout = NM_BSP_TIME_MSEC + gstrTimer.u32Period;
		}
	}

	btn_poll();
}

ISR(PCINT2_vect) 
{    
	if (digitalRead(SW2_PIN) == LOW)  
	{
  		gu8BtnIfg |= SW2;
		PCMSK2 &=~ 0b10000000;           // disable all interrupts
	}	
}

void nm_bsp_reset(void)
{
	digitalWrite(RESET_PIN,HIGH);
	nm_bsp_sleep(10);
	digitalWrite(RESET_PIN,LOW);
	nm_bsp_sleep(10);
	digitalWrite(RESET_PIN,HIGH);

}



/*
*	@fn		nm_bsp_init
*	@brief	Initialize BSP
*	@return	0 in case of success and -1 in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/
sint8 nm_bsp_init(void)
{
	gu32Jiffies = 0;
	gpfIsr = NULL;
	gstrTimer.pfCb = NULL;
	gstrTimer.u32Timeout = 0;
	gstrTimer.u32Period = 0;

#ifdef _STATIC_PS_
	gstrWakeTimer.strTimer.pfCb = NULL;
	gstrWakeTimer.strTimer.u32Timeout = 0;
	gstrWakeTimer.strTimer.u32Period = ((uint32)-1);
	gstrWakeTimer.u8Enabled = 0;
#endif

	/*IRQ*/
	pinMode(IRQ_PIN,INPUT_PULLUP);
	
	/*reset*/
	pinMode(RESET_PIN,INPUT_PULLUP);
	
	
#ifndef ENABLE_UNO_BOARD
	// initialize timer1 
	noInterrupts();           // disable all interrupts
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1  = 0;

	OCR1A = 1200;            // compare match register 16MHz/256/2Hz
	TCCR1B |= (1 << WGM12);   // CTC mode
	TCCR1B |= (1 << CS12);    // 256 prescaler 
	TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
	interrupts();
#endif
	nm_bsp_reset();
	return 0;
}


/*
*	@fn		nm_bsp_btn_init
*	@brief	Initialize buttons driver
*	@author	M.S.M
*	@date	28 OCT 2013
*	@version	1.0
*/
void nm_bsp_btn_init(tpfNmBspBtnPress pfBtnCb)
{

	gpfBtns = pfBtnCb;
	gu8BtnIfg = 0;
	gu16Btn1Cnt = 0;
	gu16Btn2Cnt = 0;
	/*BUTTON sw1*/
	pinMode(SW1_PIN,INPUT_PULLUP);
	/*BUTTON sw1*/
	pinMode(SW2_PIN,INPUT_PULLUP);
	
	attachInterrupt(1, btn_cb, FALLING);
	
	PCICR = 0x04;          // Enable PCINT23 interrupt
	PCMSK2 = 0b10000000;

}

/*
*	@fn		nm_bsp_sleep
*	@brief	Sleep in units of mSec
*	@param[IN]	u32TimeMsec
*				Time in milliseconds
*	@author	M.S.M
*	@date	28 OCT 2013
*	@version	1.0
*/
void nm_bsp_sleep(uint32 u32TimeMsec)
{
	while(u32TimeMsec--) delay(1);
}
/*
*	@fn		nm_bsp_register_isr
*	@brief	Register interrupt service routine
*	@param[IN]	pfIsr
*				Pointer to ISR handler
*	@author	M.S.M
*	@date	28 OCT 2013
*	@sa		tpfNmBspIsr
*	@version	1.0
*/
void chip_isr()
{
	if(gpfIsr)
	{
		gpfIsr();
	}
}

void nm_bsp_register_isr(tpfNmBspIsr pfIsr)
{
	gpfIsr = pfIsr;
	pinMode(9,OUTPUT);
	pinMode(IRQ_PIN,INPUT_PULLUP);
	attachInterrupt(0, chip_isr, FALLING);
}

/*
*	@fn		nm_bsp_interrupt_ctrl
*	@brief	Enable/Disable interrupts
*	@param[IN]	u8Enable
*				'0' disable interrupts. '1' enable interrupts
*	@author	M.S.M
*	@date	28 OCT 2013
*	@version	1.0
*/
void nm_bsp_interrupt_ctrl(uint8 u8Enable)
{
	if(u8Enable)
	{
		digitalWrite(9,HIGH);
		attachInterrupt(0, chip_isr, FALLING);
	}
	else
	{
		digitalWrite(9,LOW);
		detachInterrupt(0);
	}
}

/*
*	@fn		nm_bsp_uart_sendnm_bsp_uart_send
*	@author	M.S.M
*	@date	28 OCT 2013
*	@version	1.0
*/
void nm_bsp_uart_send(const uint8 *pu8Buf, uint16 u16Sz)
{

}
/**
*	@fn		nm_bsp_start_timer
*	@brief	Start periodic timer
*	@author	M.S.M
*	@date	28 OCT 2013
*	@version	1.0
*/
void nm_bsp_start_timer(tpfNmBspTimerCb pfCb, uint32 u32Period)
{
	gstrTimer.pfCb = pfCb;
	gstrTimer.u32Timeout = u32Period+ NM_BSP_TIME_MSEC;
	gstrTimer.u32Period = u32Period;
}
/**
*	@fn		nm_bsp_stop_timer
*	@brief	Stop periodic timer
*	@author	M.S.M
*	@date	28 OCT 2013
*	@version	1.0
*/
void nm_bsp_stop_timer(void)
{
	gstrTimer.pfCb = NULL;
}
#ifdef _STATIC_PS_
/**
*	@fn		nm_bsp_register_wake_isr
*	@brief	REGISTER wake up timer 
*	@author	M.S.M
*	@date	28 OCT 2013
*	@version	1.0
*/
void nm_bsp_register_wake_isr(tpfNmBspIsr pfIsr,uint32 u32MsPeriod)
{

	gstrWakeTimer.strTimer.pfCb = pfIsr;
	gstrWakeTimer.strTimer.u32Timeout = u32MsPeriod + NM_BSP_TIME_MSEC;
	gstrWakeTimer.strTimer.u32Period = u32MsPeriod;
	gstrWakeTimer.u8Enabled = 0;
}
/**
*	@fn		nm_bsp_wake_ctrl
*	@brief	control wake up timer
*	@author	M.S.M
*	@date	28 OCT 2013
*	@version	1.0
*/
void nm_bsp_wake_ctrl(uint8 en)
{
	gstrWakeTimer.u8Enabled = en;
	if(en)
	{
		gstrWakeTimer.strTimer.u32Timeout = gstrWakeTimer.strTimer.u32Period + NM_BSP_TIME_MSEC;
	}
}
#endif
#if (defined _STATIC_PS_)||(defined _DYNAMIC_PS_)
/**
*	@fn		nm_bsp_enable_mcu_ps
*	@brief	Start POWER SAVE FOR MCU 
*	@author	M.S.M
*	@date	28 OCT 2013
*	@version	1.0
*/
void nm_bsp_enable_mcu_ps(void)
{
	if(!gu8BtnIfg)
	{
		if(gstrWakeTimer.u8Enabled)
		{

		}
	}
}
#endif


