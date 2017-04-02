/**
*  @file		nm_bsp_k20d50m.h				
*  @brief		This module contains NMC1000 ABOV bsp APIs declarations  
*  @author		Ahmad.Mohammad.Yahya
*  @date		21 MARCH 2013
*  @version		1.0	
*/
#ifndef _NM_BSP_K20D50M_H_
#define _NM_BSP_K20D50M_H_

#include "MK20D5.h"

#define NM_BSP_PRINTF	printf

#define CLK_MHZ  50


#define BSP_MAX_BUF_CPY_SZ		256	/*!< This depends on available RAM for copying */
#define NM_BSP_PERM_FIRMWARE_SIZE (1024UL*256UL) /*perment size */

/*SWITCH*/
#define SW2     BIT1
#define SW3		BIT2


/*BITS*/
#define BIT15                    ((uint32)(1 << 15))
#define BIT14                    ((uint32)(1 << 14))
#define BIT13                    ((uint32)(1 << 13))
#define BIT12                    ((uint32)(1 << 12))
#define BIT11                    ((uint32)(1 << 11))
#define BIT10                    ((uint32)(1 << 10))
#define BIT9                    ((uint32)(1 << 9))
#define BIT8                    ((uint32)(1 << 8))
#define BIT7                    ((uint32)(1 << 7))
#define BIT6                    ((uint32)(1 << 6))
#define BIT5                    ((uint32)(1 << 5))
#define BIT4                    ((uint32)(1 << 4))
#define BIT3                    ((uint32)(1 << 3))
#define BIT2                    ((uint32)(1 << 2))
#define BIT1                    ((uint32)(1 << 1))
#define BIT0                    ((uint32)(1 << 0))


/* LEDS*/
#define LED_WIFI	BIT0
#define LED_GROWL	BIT8
#define BUZZER_ERR		BIT1


/* LED STATE*/
#define ON		1
#define OFF		0
#define BLINK	2
/**/
#ifdef _DEBUG
#define NM_DEBUG	1
#else
#define NM_DEBUG	0
#endif

/**/
#define TICK_RES					20		/*!< Tick resolution in milliseconds */
#define TICK_RES_SLEEP				100		/*it must be equal or less than adc sample time */

#define NM_BSP_TIME_MSEC	(gu32Jiffies * TICK_RES)

#define SHORT_PRESS_DEBOUNCE	(20/TICK_RES)
#define LONG_PRESS_DEBOUNCE		(1000/TICK_RES)
/**/
typedef void (*tpfNmBspTimerCb)(void);
typedef void (*tpfNmBspBtnPress)(uint8 u8Btn, uint8 u8Type);

/**
*	@struct		tstrTimer
*	@brief		array to hold timer paramater
*	@author		Ahmad.Mohammad.Yahya
*	@version	1.0
*/ 
typedef struct
{
	tpfNmBspTimerCb	pfCb;
	uint32 u32Timeout;
	uint32 u32Period;
} tstrTimer;


extern volatile uint32  gu32Jiffies;

/**
*	@fn		nm_bsp_btn_init
*	@brief	Initialize buttons driver
*   @author		Ahmad.Mohammad.Yahya
*   @date		21 MARCH 2013
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
*   @author		Ahmad.Mohammad.Yahya
*   @date		21 MARCH 2013
*	@version	1.0
*/
void nm_bsp_timer_start(tpfNmBspTimerCb pfCb,uint32 u32PeriodTick);

/**
*	@fn		nm_bsp_reset(void)
*	@brief	reset chip
*   @author		Ahmad.Mohammad.Yahya
*   @date		21 MARCH 2013
*	@version	1.0
*/
void nm_bsp_reset(void);
/**
*	@fn		nm_bsp_stop_timer
*	@brief	stop periodic timer
*   @author		Ahmad.Mohammad.Yahya
*   @date		21 MARCH 2013
*	@version	1.0
*/ 
void nm_bsp_stop_timer(void);

/**
*	@fn		nm_bsp_buzzer
*	@brief	beep with the given delay
*   @author		Ahmad.Mohammad.Yahya
*   @date		21 MARCH 2013
*	@version	1.0
*/ 
void nm_bsp_buzzer(uint16 u16retry);



#endif	/*_NM_BSP_K20D50M_H_*/
