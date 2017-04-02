/*
 * bsp_msp430fr5739.h
 *
 *  Created on: Jul 22, 2012
 *      Author: mabdelmawla
 */

#ifndef BSP_MSP430FR5739_H_
#define BSP_MSP430FR5739_H_

#include <msp430fr5739.h>
#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"

#define M2M_PRINTX(x)  		nm_bsp_uart_send((uint8 *)x,sizeof(x))

/**
 *Extern global variables
 *
 */
extern uint32 gu32Jiffies;
/**
 *Macros
 *
 */
#ifdef _ALPS_D2_
#define SW1		BIT6
#else
#define SW1		BIT0
#define SW2		BIT1
#endif
/**/
#define BSP_MAX_BUF_CPY_SZ		256	/*!< This depends on available RAM for copying */
#define NM_BSP_PERM_FIRMWARE_SIZE	(1024UL*256)	/* Permenant storage size available for the firmware */
#define TICK_RES					10		/*!< Tick resolution in milliseconds */
#define TICK_RES_SLEEP				100		/*it must be equal or less than adc sample time */
#define NM_BSP_TIME_MSEC	(gu32Jiffies * TICK_RES)
/**
 *
 *Callback functions
 */
typedef void (*tpfNmBspBtnPress)(uint8 u8Btn, uint8 u8Type);
typedef void (*tpfNmBspTimerCb)(void);
#ifndef _ALPS_D2_
typedef void (*tpfNmBspAdcCb)(void);
#endif

void nm_bsp_reset(void);
/**
*	@fn		nm_bsp_btn_init
*	@brief	Initialize buttons driver
*	@author	M. Abdelmawla
*	@date	01 August 2012
*	@version	1.0
*/
void nm_bsp_uart_send(uint8 *pu8Buf, uint16 u16Sz);
/**
*	@fn		nm_bsp_btn_init
*	@brief	Initialize buttons driver
*	@author	M. Abdelmawla
*	@date	01 August 2012
*	@version	1.0
*/
void nm_bsp_btn_init(tpfNmBspBtnPress pfBtnCb);
#ifdef _STATIC_PS_
/**
*	@fn		nm_bsp_register_wake_isr
*	@brief	REGISTER wake up timer 
*	@author	M.S.M
*	@date	12 FEB 2013
*	@version	1.0
*/
void nm_bsp_register_wake_isr(tpfNmBspIsr pfIsr,uint32 u32MsPeriod);
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
void nm_bsp_enable_mcu_ps(void);
#endif

/**
*	@fn		nm_bsp_start_timer
*	@brief	Start periodic timer
*	@author	M. Abdelmawla
*	@date	01 August 2012
*	@version	1.0
*/
void nm_bsp_start_timer(tpfNmBspTimerCb pfCb, uint32 u32Period);

/**
*	@fn		nm_bsp_stop_timer
*	@brief	Start periodic timer
*	@author	M. Abdelmawla
*	@date	01 August 2012
*	@version	1.0
*/
void nm_bsp_stop_timer(void);
#ifndef _ALPS_D2_
/**
*	@fn		nm_bsp_enable_mcu_ps
*	@brief	Start POWER SAVE FOR MCU
*	@author	M.S.M
*	@date	12 FEB 2013
*	@version	1.0
*/
void nm_bsp_register_adc(tpfNmBspAdcCb pfCb,uint16 u16SampleTime);
/**
*	@fn		nm_bsp_enable_mcu_ps
*	@brief	Start POWER SAVE FOR MCU
*	@author	M.S.M
*	@date	12 FEB 2013
*	@version	1.0
*/
void nm_bsp_take_ADC_measurement(void);
#endif
#endif /* BSP_MSP430FR5739_H_ */
