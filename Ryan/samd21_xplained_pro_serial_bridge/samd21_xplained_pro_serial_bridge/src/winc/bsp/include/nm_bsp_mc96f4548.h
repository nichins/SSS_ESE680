/**
*  @file		nm_bsp_mC96f4548.h				
*  @brief		This module contains NMC1000 ABOV bsp APIs declarations 
*  @author		M.S.M
*  @date		21 OCT 2012
*  @version		1.0	
*/
#ifndef _NM_BSP_MC96F4548_H_
#define _NM_BSP_MC96F4548_H_
#include <intrins.h>
#include "bsp\source\bsp_mc96f4548\MC96F4548.h"

#define SHORT_BTN_PRESS			1
#define LONG_BTN_PRESS				2

/**/
#define BSP_MAX_BUF_CPY_SZ		256	/*!< This depends on available RAM for copying */
#define NM_BSP_PERM_FIRMWARE_SIZE (1024UL*256UL) /*perment size */
/*Leds*/
#define LED_CONNECT    BIT7
#define LED_GROWL      BIT6
#define LED_ERROR      BIT1
#define RESET          BIT0
/**/
/**/
#define ON		1
#define OFF		0
#define BLINK	2
/**/
#define BIT7                    ((uint32)(1 << 7))
#define BIT6                    ((uint32)(1 << 6))
#define BIT5                    ((uint32)(1 << 5))
#define BIT4                    ((uint32)(1 << 4))
#define BIT3                    ((uint32)(1 << 3))
#define BIT2                    ((uint32)(1 << 2))
#define BIT1                    ((uint32)(1 << 1))
#define BIT0                    ((uint32)(1 << 0))
/**/
extern volatile uint32 data gu32Jiffies;
/**/
#define TICK_RES					50		/*!< Tick resolution in milliseconds */
#define NM_BSP_TIME_MSEC	(gu32Jiffies * TICK_RES)
#define BIT_TIMER_TICK_MS  (65) /*65 ms*/
/**/
typedef void (*tpfNmBspTimerCb)(void);
typedef void (*tpfNmBspBtnPress)(uint32 u32Type);
typedef void (*tpfNmMotionDetectCb)(void);
/**
*	@struct		tstrMotionSens
*	@brief		array to hold motion sensor paramater
*	@author		M.S.M
*	@version	1.0
*/ 
typedef struct
{
	tpfNmMotionDetectCb	pfCb;
	
} tstrMotionSens;
/**
*	@struct		tstrTimer
*	@brief		array to hold timer paramater
*	@author		M.S.M
*	@version	1.0
*/ 
typedef struct
{
	tpfNmBspTimerCb	pfCb;
	uint32 u32Timeout;
	uint32 u32Period;
} tstrTimer;

/**
*	@fn		nm_bsp_btn_init
*	@brief	Initialize buttons driver
*	@author M.S.M
*	@date	01 August 2012
*	@version	1.0
*/
void nm_bsp_btn_init(tpfNmBspBtnPress pfBtnCb);
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
void nm_bsp_timer_start(tpfNmBspTimerCb pfCb,uint32 u32PeriodTick);
/**
*	@fn		nm_bsp_reset(void)
*	@brief	reset chip
*	@author	M.S.M
*	@date	22 OCT 2012
*	@version	1.0
*/
void nm_bsp_reset(void);
/**
*	@fn		nm_bsp_stop_timer
*	@brief	stop periodic timer
*	@author	M.S.M
*	@date	01 August 2012
*	@version	1.0
*/ 
void nm_bsp_stop_timer(void);
/**
*	@fn		nm_bsp_usleep
*	@brief	delay micro second 
*	@author	M.S.M
*	@date	22 OCT 2012
*	@version	1.0
*/ 
void nm_bsp_usleep(uint32 u32Delay);
/**
*	@fn		nm_bsp_buzzer
*	@brief	beep with the given delay
*	@author	M.S.M
*	@date	22 OCT 2012
*	@version	1.0
*/ 
void nm_bsp_buzzer(uint8 mdelay);
/*
*	@fn		nm_bsp_motionsen_init
*	@brief	Initialize Motion sensor
*	@author	M.S.M
*	@date	01 August 2012
*	@version	1.0
*/
void nm_bsp_motionsen_init(tpfNmMotionDetectCb pfsensCb);
/**
*	@fn		void nm_bsp_accx_clear_int(void)
*	@brief	clear motion sensor int 
*	@author	M.S.M
*	@date	22 OCT 2012
*	@version	1.0
*/ 
void nm_bsp_accx_clear_int(void);
/**
*	@fn		sint8 nm_bsp_read_temp(char* pcTempStr)
*	@brief	Read temp sensor
*	@author	M.S.M
*	@date	22 OCT 2012
*	@version	1.0
*/ 
sint8 nm_bsp_read_temp(char* pcTempStr);
#ifdef _STATIC_PS_
/**
*	@fn		nm_bsp_register_wake_isr
*	@brief	REGISTER wake up timer
*	@author	M.S.M
*	@date	12 FEB 2013
*	@version	1.0
*/
void nm_bsp_register_wake_isr(tpfNmBspIsr pfIsr);
/**
*	@fn		nm_bsp_wake_ctrl
*	@brief	control wake up timer
*	@author	M.S.M
*	@date	12 FEB 2013
*	@version	1.0
*/
void nm_bsp_wake_ctrl(uint8 en);
#endif
#if (defined _STATIC_PS_)||(defined _DYNAMIC_PS_)
/**
*	@fn		nm_bsp_enable_mcu_ps
*	@brief	Start POWER SAVE FOR MCU
*	@author	M.S.M
*	@date	12 FEB 2013
*	@version	1.0
*/
void nm_bsp_enter_mcu_ps(void);
#endif

#endif	/*_NM_BSP_MC96F4548_H_*/
