/**
*  @file		nm_bsp_mcf51cn128.h				
*  @brief		This module contains NMC1000 Win32 bsp APIs declarations 
*  @author		M. Abdelmawla
*  @date		10 JULY 2012
*  @version		1.0	
*/
#ifndef _NM_BSP_MCF51CN128_H_
#define _NM_BSP_MCF51CN128_H_

#include <mqx.h>
#include <bsp.h>
#include <fio.h>
#include <string.h>
#include <stdlib.h>

#define ABS(x) (((x)<0)?((x)*(-1)):(x))

#define BSP_MAX_BUF_CPY_SZ		256	/*!< This depends on available RAM for copying */
#define NM_BSP_PERM_FIRMWARE_SIZE (1024*256) /*perment size */

#define NM_BSP_PRINTF	printf
#ifdef _DEBUG
#define NM_DEBUG	1
#else
#define NM_DEBUG	0
#endif
/*Leds*/
#define LED_CONNECT    1
#define LED_DOWNLOAD   2
#define LED_GROWL      4
#define LED_ERROR      8
#define RESET          16
/**/
/*btn*/
#define SW2  0
#define SW3  1
#define DIP1 2
/**/
#define BLINK 2
#define ON  1
#define OFF 0
/**/
#define ACCX_CH 1
#define POT_CH  0
/**/
extern uint32 gu32Jiffies;

#define NM_BSP_TIME_MSEC	(gu32Jiffies)

typedef void (*tpfNmBspTimerCb)(void);
typedef void (*tpfNmBspAdcCb)(sint32,uint8);
/**
*	@struct		my_isr_struct
*	@brief		Structure to hold interupt handler
*	@author		Mahfouz Sheref
*	@version	1.0
*/ 
typedef struct my_isr_struct
{
	pointer               OLD_ISR_DATA;
	void      (_CODE_PTR_ OLD_ISR)(pointer);
	_mqx_uint             TICK_COUNT;
} MY_ISR_STRUCT, _PTR_ MY_ISR_STRUCT_PTR;
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



/*
*	@fn		nm_bsp_btn_init
*	@brief	Initialize buttons driver
*	@author	M. Abdelmawla
*	@date	01 August 2012
*	@version	1.0
*/
void nm_bsp_btn_init(void);
/**
*	@fn		nm_bsp_get_button_status(uint8 bnum)
*	@brief	get button state
*	@param[IN]	bnum
*				1=sw1 2=sw2
*   @return   true if pushed false if not
*	@author	M.S.M
*	@date	18 July 2012
*	@version	1.0
*/
boolean  nm_bsp_get_btn_status(uint8 bnum);
/**
*	@fn		nm_bsp_led(uint8 state,uint8 lednum)
*	@brief	led control
*	@param[IN]	state
*				1=ON  0=OFF
*	@param[IN]	lednum
*				from 1 to 4
*	@author	M.S.M
*	@date	18 July 2012
*	@version	1.0
*/
void nm_bsp_gpio_ctrl(uint8 state,uint8 PinNum);
/**
*	@fn		 ReadADC(_mqx_int channel)
*	@brief	read adc channel
*	@param[IN]	channel
*				channel number 1 or 2 depend on ADC_CH_COUNT
*   @return   value readed
*	@author	M.S.M
*	@date	18 July 2012
*	@version	1.0
*/
_mqx_int ReadADC(_mqx_int channel);
/**
*	@fn		nm_bsp_timer_start(tpfNmBspTimerCb pfCb, uint32 u32PeriodTick)
*	@brief	timer init
*	@param[out]	fp
*				callback function for timer handler 
*	@param[IN]	u32Period
*			 toggle period (tick resolution =2.5ms)
*	@author	M.S.M
*	@date	18 July 2012
*	@version	1.0
*/
void nm_bsp_timer_start(tpfNmBspTimerCb pfCb, uint32 u32PeriodTick);
/**
*	@fn		nm_bsp_reset(void)
*	@brief	reset chip
*	@author	M.S.M
*	@date	18 July 2012
*	@version	1.0
*/
void nm_bsp_reset(void);
/**
*	@fn		nm_bsp_stop_timer
*	@brief	stop periodic timer
*	@author	M. Abdelmawla
*	@date	01 August 2012
*	@version	1.0
*/ 
void nm_bsp_stop_timer(void);
/**
*	@fn		 nm_bsp_register_adc_cb
*	@brief	register adc callback
*	@param[IN]	pfCb
*				Callback function
*	@author	M.S.M
*	@date	08 August 2012
*	@version	1.0
*/
void nm_bsp_register_adc_cb(tpfNmBspAdcCb pfCb);
/**
*	@fn		nm_bsp_take_ADC_measurement
*	@brief	take adc measurement
*	@author	M.S.M
*	@date	08 August 2012
*	@version	1.0
*/
void nm_bsp_take_ADC_measurement(void);

#endif	/*_NM_BSP_MCF51CN128_H_*/
