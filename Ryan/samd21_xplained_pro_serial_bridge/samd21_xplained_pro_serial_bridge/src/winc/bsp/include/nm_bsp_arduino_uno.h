/*
*nm_bsp_arduino_uno.h
*
*  Created on: 26 APRIL 2014
*      Author: MSM
*/

#ifndef _NM_BSP_ARDUINO_UNO_H_
#define _NM_BSP_ARDUINO_UNO_H_

#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#include <Arduino.h>


#ifdef _STATIC_PS_
#eror "_STATIC_PS_ is not enabled"
#endif

/**
*Extern global variables
*
*/
extern uint32 gu32Jiffies;
/**
*Macros
*
*/
#define BIT0                   (0x0001)
#define BIT1                   (0x0002)
#define BIT2                   (0x0004)
#define BIT3                   (0x0008)
#define BIT4                   (0x0010)
#define BIT5                   (0x0020)
#define BIT6                   (0x0040)
#define BIT7                   (0x0080)
/*
*
*/
#define SW1		BIT0
#define SW2		BIT1
/**/
#define NM_BUS_MAX_TRX_SZ					120	/*!< This depends on available RAM for copying */

#define NM_BSP_PERM_FIRMWARE_SIZE	(1024UL*256)	/* Permenant storage size available for the firmware */

#define TICK_RES							20		/*!< Tick resolution in milliseconds */

#define NM_BSP_TIME_MSEC			(gu32Jiffies * TICK_RES)


#ifdef _DEBUG
#define NM_DEBUG	1
#else
#define NM_DEBUG	0
#endif

/**
*
*Callback functions
*/
typedef void (*tpfNmBspBtnPress)(uint8 u8Btn, uint8 u8Type);
typedef void (*tpfNmBspTimerCb)(void);

#ifdef __cplusplus
     extern "C" {
 #endif

/**
*	@fn		nm_bsp_reset
*	@brief	Reset NMC1000 Chip
*	@author	M.S.M
*	@date	28 oct 2013
*	@version	1.0
*/
void nm_bsp_reset(void);
/**
*	@fn		nm_bsp_uart_send
*	@brief	send buffer to uart for debugging 
*	@author	M.S.M
*	@date	28 oct 2013
*	@version	1.0
*/
void nm_bsp_uart_send(const uint8 *pu8Buf, uint16 u16Sz);
/**
*	@fn		nm_bsp_btn_init
*	@brief	Initialize buttons driver
*	@author	M.S.M
*	@date	28 oct 2013
*	@version	1.0
*/
void nm_bsp_btn_init(tpfNmBspBtnPress pfBtnCb);
#ifdef _STATIC_PS_
/**
*	@fn		nm_bsp_register_wake_isr
*	@brief	REGISTER wake up timer 
*	@author	M.S.M
*	@date	28 oct 2013
*	@version	1.0
*/
void nm_bsp_register_wake_isr(tpfNmBspIsr pfIsr,uint32 u32MsPeriod);
/**
*	@fn		nm_bsp_wake_ctrl
*	@brief	control wake up timer
*	@author	M.S.M
*	@date	28 oct 2013
*	@version	1.0
*/
void nm_bsp_wake_ctrl(uint8 en);
#endif
#if (defined _STATIC_PS_)||(defined _DYNAMIC_PS_)
/**
*	@fn		nm_bsp_enable_mcu_ps
*	@brief	Start POWER SAVE FOR MCU 
*	@author	M.S.M
*	@date	28 oct 2013
*	@version	1.0
*/
void nm_bsp_enable_mcu_ps(void);
#endif

/**
*	@fn		nm_bsp_start_timer
*	@brief	Start periodic timer
*	@author	M.S.M
*	@date	28 oct 2013
*	@version	1.0
*/
void nm_bsp_start_timer(tpfNmBspTimerCb pfCb, uint32 u32Period);

/**
*	@fn		nm_bsp_stop_timer
*	@brief	stop periodic timer
*	@author	M.S.M
*	@date	28 oct 2013
*	@version	1.0
*/
void nm_bsp_stop_timer(void);


#ifdef __cplusplus
	 }
#endif
#endif /* _NM_BSP_ARDUINO_UNO_H_ */
