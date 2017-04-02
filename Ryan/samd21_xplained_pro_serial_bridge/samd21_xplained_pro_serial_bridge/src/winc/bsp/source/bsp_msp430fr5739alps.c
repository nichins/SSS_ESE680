/**
*  @file		nm_bsp_win32.c
*  @brief		This module contains NMC1000 Win32 bsp APIs implementation
*  @author		M. Abdelmawla
*  @date		10 JULY 2012
*  @version		1.0
*/
#ifdef __MSP430FR5739__
#ifdef _ALPS_D2_
#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"


#define BSP_M_CLK_KHZ		(24000)
#define FW_SIZE			(128 * 1024UL)

#define BSP_ISR_PORT_VECTOR			PORT3_VECTOR
#define BSP_ISR_PIN					BIT3
#define BSP_ISR_PORT_SEL0			P3SEL0
#define BSP_ISR_PORT_SEL1			P3SEL1
#define BSP_ISR_PORT_SELC			P3SELC
#define BSP_ISR_PORT_IE				P3IE
#define BSP_ISR_PORT_IES			P3IES
#define BSP_ISR_PORT_IFG			P3IFG
#define BSP_ISR_PORT_DIR			P3DIR
#define BSP_ISR_PORT_REN			P3REN
#define BSP_ISR_PORT_OUT			P3OUT

#define SHORT_PRESS_DEBOUNCE	(40/TICK_RES)
#define LONG_PRESS_DEBOUNCE		(1000/TICK_RES)

#if (0 == SHORT_PRESS_DEBOUNCE)
#undef SHORT_PRESS_DEBOUNCE
#define SHORT_PRESS_DEBOUNCE 1
#endif

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

uint32 gu32Jiffies;

static tpfNmBspIsr gpfIsr;
static uint16 gu16Btn1Cnt;
static tpfNmBspBtnPress gpfBtns;
static uint8 gu8BtnIfg;
static tstrTimer gstrTimer;
static tstrWakeTimer gstrWakeTimer;

/**
 *
 *
 * Static Functions
 */
static void init_tick_timer(void)
{
  TA1CCTL0 = CCIE;
  TA1CTL = TASSEL__ACLK + MC__UP + ID__4;
  TA1EX0 = TAIDEX_2;

  gu32Jiffies = 0;
  /* Now the input clock to the timer block is 24M/24 = 1MHz. each CCR increasment is 1u second */
  /*Tackl 32KHZ/32 = 1KHz 1ms = */
  TA1CCR0 = TICK_RES;
}
static void btn_poll(void)
{
	if (gu8BtnIfg & SW1) {
		gu16Btn1Cnt++;

		if (gu16Btn1Cnt >= SHORT_PRESS_DEBOUNCE) {
			if (P1IN & SW1) {
				gpfBtns(SW1, 0); /* Short press callback */
				gu16Btn1Cnt = 0;

			} else {
				if (gu16Btn1Cnt >= LONG_PRESS_DEBOUNCE) {
					gpfBtns(SW1, 1); /* Short press callback */
					gu16Btn1Cnt = 0;
				}
			}
		}
	}

	if(!gu16Btn1Cnt)
	{
		gu8BtnIfg &= ~SW1;
		P1IE |= SW1;
		P1IFG &= ~SW1;
	}

}
/*
 *
 *
 * Interrupt handlers
 */
/*NMC1000 interrupt*/
#pragma vector=BSP_ISR_PORT_VECTOR
__interrupt void nm_bsp_isr(void)
{
	if(BSP_ISR_PORT_IFG & BSP_ISR_PIN)
	{
		__bic_SR_register_on_exit(LPM3_bits);
#ifdef _STATIC_PS_
		nm_bsp_wake_ctrl(0);
#endif
		if(gpfIsr)
		{
			gpfIsr();
		}
		BSP_ISR_PORT_IFG &= ~BSP_ISR_PIN;
	}
}
/*btn interrupt*/
#pragma vector=PORT1_VECTOR
__interrupt void btn_isr(void)
{
	P1IE &= ~(SW1 & P1IE);
	gu8BtnIfg |= SW1;
#ifdef _STATIC_PS_
	nm_bsp_wake_ctrl(0);
#endif
	__bic_SR_register_on_exit(LPM3_bits);
}
/*Timer interrupt*/
#pragma vector = TIMER1_A0_VECTOR
__interrupt void tick_timer_isr(void)
{
#ifdef _STATIC_PS_
	if((gstrWakeTimer.strTimer.pfCb)&&(gstrWakeTimer.u8Enabled))
	{
		gu32Jiffies+=(TICK_RES_SLEEP/TICK_RES);
		if(NM_BSP_TIME_MSEC >= gstrWakeTimer.strTimer.u32Timeout)
		{
			__bic_SR_register_on_exit(LPM3_bits);
			nm_bsp_wake_ctrl(0);
			gstrWakeTimer.strTimer.pfCb();
			gstrWakeTimer.strTimer.u32Timeout = NM_BSP_TIME_MSEC + gstrWakeTimer.strTimer.u32Period;
		}
	}
	else
#endif
	{
		gu32Jiffies++;
		if(gstrTimer.pfCb)
		{
			if(NM_BSP_TIME_MSEC >= gstrTimer.u32Timeout)
			{
				gstrTimer.pfCb();
				gstrTimer.u32Timeout = NM_BSP_TIME_MSEC + gstrTimer.u32Period;
			}
		}
	}
	btn_poll();
	TA0CCTL0 &= ~CCIFG;
}
/*
*	@fn		nm_bsp_init
*	@brief	Initialize BSP
*	@return	0 in case of success and -1 in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/
void nm_bsp_reset(void)
{

}
sint8 nm_bsp_init(void)
{
	WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
	// Init the device with 24 MHz DCOCLCK.
	// SMCLCK which will source also SPI will be sourced also by DCO
	//
	CSCTL0_H = 0xA5;
	CSCTL1 |= DCORSEL + DCOFSEL0 + DCOFSEL1;	 // Set max. DCO setting
	CSCTL2 = SELA_0 + SELS_3 + SELM_3;		// set ACLK - VLO, the rest  = MCLK = DCO
	CSCTL3 = DIVA_0 + DIVS_0 + DIVM_0;		// set all dividers to 0



	// Configure the SPI Flash SPI CS to be on P2.3
	P2OUT |= BIT3;
	P2DIR |= BIT3;
	P2SEL1 &= ~BIT3;
	P2SEL0 &= ~BIT3;



	gpfIsr = NULL;
	gstrTimer.pfCb = NULL;
	gstrTimer.u32Timeout = 0;
	gstrTimer.u32Period = 0;

	gstrWakeTimer.strTimer.pfCb = NULL;
	gstrWakeTimer.strTimer.u32Timeout = 0;
	gstrWakeTimer.strTimer.u32Period = 0;
	gstrWakeTimer.u8Enabled = 0;


	init_tick_timer();

	/*Confg power pin*/
	P4OUT &=~(BIT1);
	P4DIR |= (BIT1);
	P4SEL1 &= ~(BIT1);
	P4SEL0 &= ~(BIT1);

	/*Confg Chip Enable pin*/
	P3OUT &= ~(BIT5+BIT6);
	P3DIR |= (BIT5+BIT6);
	P3SEL1 &= ~(BIT5+BIT6);
	P3SEL0 &= ~(BIT5+BIT6);
	/*Confg  Reset pin*/
	P1OUT &= ~BIT4;
	P1DIR |= BIT4;
	P1SEL1 &= ~BIT4;
	P1SEL0 &= ~BIT4;

	/*Disable all pins*/
	P1OUT &= ~BIT4;	/*reset*/
	P4OUT &=~(BIT1);/*power*/
	P3OUT &= ~(BIT5+BIT6);/*wake + Chip En*/
	nm_bsp_sleep(50);
	P4OUT |= (BIT1);/*set power on */
	nm_bsp_sleep(100);
	P3OUT |= (BIT5+BIT6);/*set en and wake*/
	nm_bsp_sleep(200);
	P1OUT |= BIT4;/*set reset*/
	nm_bsp_sleep(100);

	return 0;
}

/*
*	@fn		nm_bsp_sleep
*	@brief	Sleep in units of mSec
*	@param[IN]	u32TimeMsec
*				Time in milliseconds
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/
void nm_bsp_sleep(uint32 u32TimeMsec)
{
	while(u32TimeMsec--) __delay_cycles(BSP_M_CLK_KHZ);
}

/*
*	@fn		nm_bsp_register_isr
*	@brief	Register interrupt service routine
*	@param[IN]	pfIsr
*				Pointer to ISR handler
*	@author	M. Abdelmawla
*	@date	29 July 2012
*	@sa		tpfNmBspIsr
*	@version	1.0
*/
void nm_bsp_register_isr(tpfNmBspIsr pfIsr)
{
	gpfIsr = pfIsr;

	__bis_SR_register(GIE);

	BSP_ISR_PORT_SEL0 &= ~BSP_ISR_PIN;
	BSP_ISR_PORT_SEL1 &= ~BSP_ISR_PIN;

	BSP_ISR_PORT_DIR &= ~BSP_ISR_PIN;
	BSP_ISR_PORT_OUT |= BSP_ISR_PIN;	/* Set pull up */
	BSP_ISR_PORT_REN |= BSP_ISR_PIN;	/* enable pull up */

	BSP_ISR_PORT_IFG &= ~BSP_ISR_PIN;
	BSP_ISR_PORT_IES |= BSP_ISR_PIN;
	BSP_ISR_PORT_IE |= BSP_ISR_PIN;
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
		BSP_ISR_PORT_IE |= BSP_ISR_PIN;
	}
	else
	{
		BSP_ISR_PORT_IE &= ~BSP_ISR_PIN;
	}
}
/*
*	@fn		nm_bsp_btn_init
*	@brief	Initialize buttons driver
*	@author	M. Abdelmawla
*	@date	01 August 2012
*	@version	1.0
*/
void nm_bsp_btn_init(tpfNmBspBtnPress pfBtnCb)
{
	P1SEL0 &= ~(SW1);
	P1SEL1 &= ~(SW1);
	P1DIR &= ~(SW1);
	P1OUT |= (SW1);
	P1REN |= (SW1);
	P1IES |= (SW1);
	P1IFG &= ~(SW1);
	P1IE |= (SW1);
	gpfBtns = pfBtnCb;

	gu8BtnIfg = 0;
	gu16Btn1Cnt = 0;

	__bis_SR_register(GIE);
}

/**
*	@fn		nm_bsp_start_timer
*	@brief	Start periodic timer
*	@author	M. Abdelmawla
*	@date	01 August 2012
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
*	@brief	Start periodic timer
*	@author	M. Abdelmawla
*	@date	01 August 2012
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
*	@date	12 FEB 2013
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
*	@date	12 FEB 2013
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
*	@date	12 FEB 2013
*	@version	1.0
*/
void nm_bsp_enable_mcu_ps(void)
{
	if(!gu8BtnIfg)
	{
		if(gstrWakeTimer.u8Enabled)
		{
			TA1CCR0 = TICK_RES_SLEEP;
		}
		__bis_SR_register(LPM3_bits + GIE);
		TA1CCR0 = TICK_RES;
	}

}
#endif
#endif
#endif


