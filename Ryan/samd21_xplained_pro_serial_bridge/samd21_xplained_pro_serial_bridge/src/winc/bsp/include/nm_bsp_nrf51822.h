/*
* nm_bsp_nrf51822.h
*
*  Created on: oct 28, 2013
*      Author: MSM
*/

#ifndef BSP_NRF51822_H_
#define BSP_NRF51822_H_

#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

#define M2M_PRINTX(x)  nm_bsp_uart_send((const uint8_t *)x,sizeof(x))
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
#define BSP_MAX_BUF_CPY_SZ				256	/*!< This depends on available RAM for copying */
#define NM_BSP_PERM_FIRMWARE_SIZE	(1024UL*256)	/* Permenant storage size available for the firmware */
#ifdef _SOFT_DEVICE_
#define TICK_RES							100		/*!< Tick resolution in milliseconds */
#else
#define TICK_RES							20		/*!< Tick resolution in milliseconds */
#endif
#define TICK_RES_SLEEP						100		/*it must be equal or less than adc sample time */
#define NM_BSP_TIME_MSEC			(gu32Jiffies * TICK_RES)
/**
*
*Callback functions
*/
typedef void (*tpfNmBspBtnPress)(uint8 u8Btn, uint8 u8Type);
typedef void (*tpfNmBspTimerCb)(void);

void nm_bsp_reset(void);
/**
*	@fn		nm_bsp_btn_init
*	@brief	Initialize buttons driver
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
*	@brief	Start periodic timer
*	@author	M.S.M
*	@date	28 oct 2013
*	@version	1.0
*/
void nm_bsp_stop_timer(void);
#endif /* BSP_NRF51822_H_ */
