/**
*  @file		nm_bsp_win32.c
*  @brief		This module contains NMC1000 Win32 bsp APIs implementation
*  @author		M. Abdelmawla
*  @date		10 JULY 2012
*  @version		1.0
*/
#ifdef __MSP430FR5739__
#ifndef _ALPS_D2_
#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"

#define BSP_M_CLK_KHZ		(24000)
#define FW_SIZE			(128 * 1024UL)
#define BSP_MIN(x,y) ((x)>(y)?(y):(x))

#define BSP_ISR_PORT_VECTOR			PORT1_VECTOR
#define BSP_ISR_PIN					BIT0
#define BSP_ISR_PORT_SEL0			P1SEL0
#define BSP_ISR_PORT_SEL1			P1SEL1
#define BSP_ISR_PORT_SELC			P1SELC
#define BSP_ISR_PORT_IE				P1IE
#define BSP_ISR_PORT_IES			P1IES
#define BSP_ISR_PORT_IFG			P1IFG
#define BSP_ISR_PORT_DIR			P1DIR
#define BSP_ISR_PORT_REN			P1REN
#define BSP_ISR_PORT_OUT			P1OUT

// Pin Definitions
#define ACC_PWR_PIN       BIT7
#define ACC_PWR_PORT_DIR  P2DIR
#define ACC_PWR_PORT_OUT  P2OUT
#define ACC_PORT_DIR      P3DIR
#define ACC_PORT_OUT      P3OUT
#define ACC_PORT_SEL0     P3SEL0
#define ACC_PORT_SEL1     P3SEL1
#define ACC_X_PIN         BIT0
#define ACC_Y_PIN         BIT1
#define ACC_Z_PIN         BIT2

// Accelerometer Input Channel Definitions
#define ACC_X_CHANNEL     ADC10INCH_12
#define ACC_Y_CHANNEL     ADC10INCH_13
#define ACC_Z_CHANNEL     ADC10INCH_14

#define SHORT_PRESS_DEBOUNCE	(40/TICK_RES)
#define LONG_PRESS_DEBOUNCE		(1000/TICK_RES)

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
	tpfNmBspAdcCb pfAdcCb;
	uint32 u32AdcSampleTime;
	uint32 u32AdcTimeout;
} tstrAdc;

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
static tstrWakeTimer gstrWakeTimer;
static tstrAdc gstrAdcParm;
static uint32 last_notification_time = 0;
static volatile int g = 0;
/**
 *
 * Static functions
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
			if (P4IN & SW1)
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

	if (gu8BtnIfg & SW2) {
		gu16Btn2Cnt++;

		if (gu16Btn2Cnt >= SHORT_PRESS_DEBOUNCE) {
			if (P4IN & SW2) {
				gpfBtns(SW2, 0); /* Short press callback */
				gu16Btn2Cnt = 0;
			} else {
				if (gu16Btn2Cnt >= LONG_PRESS_DEBOUNCE) {
					gpfBtns(SW2, 1); /* long press callback */
					gu16Btn2Cnt = 0;
				}
			}
		}
	}

	if(!gu16Btn1Cnt)
	{
		gu8BtnIfg &= ~SW1;
		P4IE |= SW1;
		P4IFG &= ~SW1;
	}
	if (!gu16Btn2Cnt) {
		gu8BtnIfg &= ~SW2;
		P4IE |= SW2;
		P4IFG &= ~SW2;
	}

}
static void SetupAccel(void) {
	//Setup  accelerometer
	// ~20KHz sampling
	//Configure GPIO
	ACC_PORT_SEL0 |= ACC_X_PIN + ACC_Y_PIN + ACC_Z_PIN; //Enable A/D channel inputs
	ACC_PORT_SEL1 |= ACC_X_PIN + ACC_Y_PIN + ACC_Z_PIN;
	ACC_PORT_DIR &= ~(ACC_X_PIN + ACC_Y_PIN + ACC_Z_PIN);
	ACC_PWR_PORT_DIR |= ACC_PWR_PIN; //Enable ACC_POWER
	ACC_PWR_PORT_OUT |= ACC_PWR_PIN;

	// Allow the accelerometer to settle before sampling any data
	__delay_cycles(200000);

	//Single channel, once,
	ADC10CTL0 &= ~ADC10ENC; // Ensure ENC is clear
	ADC10CTL0 = ADC10ON + ADC10SHT_5;
	ADC10CTL1 = ADC10SHS_0 + ADC10SHP + ADC10CONSEQ_0 + ADC10SSEL_0;
	ADC10CTL2 = ADC10RES;
	ADC10MCTL0 = ADC10SREF_0 + ADC10INCH_12;
	ADC10IV = 0x00; // Clear all ADC12 channel int flags
	ADC10IE |= ADC10IE0;

	gstrAdcParm.pfAdcCb = NULL;
}
static void uart_init(void)
{
	  // Configure UART 0

	// Configure UART pins P2.0 & P2.1
	P2SEL1 |= BIT0 + BIT1;
	P2SEL0 &= ~(BIT0 + BIT1);

#if 0
	UCA0CTL1 |= UCSWRST;
	UCA0CTL1 = UCSSEL_2; // Set SMCLK as UCLk
	//UCA0CTLW0 |= UCMSB;
	UCA0BR0 = 13; // 9600 baud
	// 8000000/(9600*16) - INT(8000000/(9600*16))=0.083
	UCA0BR1 = 0;
	// UCBRFx = 1, UCBRSx = 0x49, UCOS16 = 1 (Refer User Guide)
	UCA0MCTLW = UCOS16 + UCBRF_0 + (0x49U << 0x8);

	UCA0CTL1 &= ~UCSWRST; // release from reset

#else

	UCA0CTL1 |= UCSWRST;
	UCA0CTL1 = UCSSEL__SMCLK;                      // Set SMCLK as UCLk
	//UCA0MCTLW |= 0x5300;                      // 32768/9600 - INT(32768/9600)=0.41
	UCA0BR0 = 0xC4;                              // 9600 baud
	UCA0BR1 = 0x9;
	UCA0CTL1 &= ~UCSWRST;                     // release from reset
	UCA0IE |= UCRXIE;                         // enable RX interrupt
#endif
}
/**
 *
 * Interrupt handler
 */
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
				__bic_SR_register_on_exit(LPM3_bits);
				gstrTimer.u32Timeout = NM_BSP_TIME_MSEC + gstrTimer.u32Period;
			}
		}
	}
	btn_poll();
	if(NM_BSP_TIME_MSEC >= gstrAdcParm.u32AdcTimeout)
	{
		nm_bsp_take_ADC_measurement();
		__bic_SR_register_on_exit(LPM3_bits);
		gstrAdcParm.u32AdcTimeout = NM_BSP_TIME_MSEC + gstrAdcParm.u32AdcSampleTime;
	}
	TA1CCTL0 &= ~CCIFG;
}
/*ADC interrupt*/
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
  switch(__even_in_range(ADC10IV,ADC10IV_ADC10IFG))
  {
    case ADC10IV_NONE: break;               // No interrupt
    case ADC10IV_ADC10OVIFG: break;         // conversion result overflow
    case ADC10IV_ADC10TOVIFG: break;        // conversion time overflow
    case ADC10IV_ADC10HIIFG: break;         // ADC10HI
    case ADC10IV_ADC10LOIFG: break;         // ADC10LO
    case ADC10IV_ADC10INIFG: break;         // ADC10IN
    case ADC10IV_ADC10IFG:
    	if(gstrAdcParm.pfAdcCb)
    	{
			if ((ADC10MEM0 > (g + 10)) || (ADC10MEM0 < (g - 10)))
			{
				if((NM_BSP_TIME_MSEC - last_notification_time) >= 500)
				{
					__bic_SR_register_on_exit(LPM3_bits);
#ifdef _STATIC_PS_
					nm_bsp_wake_ctrl(0);
#endif
					gstrAdcParm.pfAdcCb();
					last_notification_time = NM_BSP_TIME_MSEC;
				}
				g = ADC10MEM0;
			}
    	}
             break;                          // Clear CPUOFF bit from 0(SR)
    default: break;
  }
}
/*Btn interrupt*/
#pragma vector=PORT4_VECTOR
__interrupt void btn_isr(void)
{
	gu8BtnIfg |= P4IFG;
	P4IE &= ~(P4IFG & P4IE);
#ifdef _STATIC_PS_
	nm_bsp_wake_ctrl(0);
#endif
	__bic_SR_register_on_exit(LPM3_bits);
}
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
void nm_bsp_reset(void)
{
	P1OUT &= ~(BIT1 | BIT2);/*disable chip en + reset*/
	//nm_bsp_sleep(100);
	P1OUT |= BIT1;/*chip enable*/
	__delay_cycles(3072);
	P1OUT |= BIT2; /*reset*/
	//nm_bsp_sleep(200);
#ifdef _PS_CLIENT_
	P1OUT &= ~(BIT2);
	P1OUT &= ~(BIT1);
#endif

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
	WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

	// Configure the NMC1000 RESET to be on P1.2 and CE to P1.1
	P1SEL1 &= ~(BIT1 | BIT2);
	P1SEL0 &= ~(BIT1 | BIT2);
	P1DIR |= (BIT1 | BIT2);
	P1OUT &= ~(BIT1 | BIT2);/*disable chip en + reset*/
	nm_bsp_sleep(10);

	BSP_ISR_PORT_SEL0 &= ~BSP_ISR_PIN;
	BSP_ISR_PORT_SEL1 &= ~BSP_ISR_PIN;

	BSP_ISR_PORT_DIR &= ~BSP_ISR_PIN;
	BSP_ISR_PORT_OUT |= BSP_ISR_PIN;	/* Set pull up */
	BSP_ISR_PORT_REN |= BSP_ISR_PIN;	/* enable pull up */

	// Init the device with 24 MHz DCOCLCK.
	// SMCLCK which will source also SPI will be sourced also by DCO
	//
	CSCTL0_H = 0xA5;
	CSCTL1 |= DCORSEL + DCOFSEL0 + DCOFSEL1;	 // Set max. DCO setting
	CSCTL2 = SELA_1 + SELS_3 + SELM_3;		// set ACLK - VLO, the rest  = MCLK = DCO
	CSCTL3 = DIVA_0 + DIVS_0 + DIVM_0;		// set all dividers to 0

	// Configure SPI IRQ line on P2.3
	//P2DIR  &= (~BIT3);
	//P2SEL1 &= ~BIT3;
	//P2SEL0 &= ~BIT3;

	// Configure the NMC1000 SPI CS to be on P1.3
	P1OUT |= BIT3;
	P1DIR |= BIT3;
	P1SEL1 &= ~BIT3;
	P1SEL0 &= ~BIT3;

	// Configure the SPI Flash SPI CS to be on P2.3
	P2OUT |= BIT3;
	P2DIR |= BIT3;
	P2SEL1 &= ~BIT3;
	P2SEL0 &= ~BIT3;

	// Configure UART pins P2.0 & P2.1
	P2SEL1 |= BIT0 + BIT1;
	P2SEL0 &= ~(BIT0 + BIT1);

	// P3.0,P3.1 and P3.2 are accelerometer inputs
	P3OUT &= ~(BIT0 + BIT1 + BIT2);
	P3DIR &= ~(BIT0 + BIT1 + BIT2);
	P3REN |= BIT0 + BIT1 + BIT2;

	gpfIsr = NULL;
	gstrTimer.pfCb = NULL;
	gstrTimer.u32Timeout = 0;
	gstrTimer.u32Period = 0;

	gstrAdcParm.pfAdcCb = NULL;
	gstrAdcParm.u32AdcSampleTime = ((uint32)-1);
	gstrAdcParm.u32AdcTimeout = 0;


	gstrWakeTimer.strTimer.pfCb = NULL;
	gstrWakeTimer.strTimer.u32Timeout = ((uint32)-1);;
	gstrWakeTimer.strTimer.u32Period = ((uint32)-1);
	gstrWakeTimer.u8Enabled = 0;


	nm_bsp_reset();

	uart_init();

	SetupAccel();

	//__delay_cycles(12000000);
	init_tick_timer();

	return 0;
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
	P4SEL0 &= ~(SW1 + SW2);
	P4SEL1 &= ~(SW1 + SW2);
	P4DIR &= ~(SW1 + SW2);
	P4OUT |= (SW1 + SW2);
	P4REN |= (SW1 + SW2);
	P4IES |= (SW1 + SW2);
	P4IFG &= ~(SW1 + SW2);
	P4IE |= (SW1 + SW2);
	gpfBtns = pfBtnCb;

	gu8BtnIfg = 0;
	gu16Btn1Cnt = 0;
	gu16Btn2Cnt = 0;

	__bis_SR_register(GIE);
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


	BSP_ISR_PORT_IFG &= ~BSP_ISR_PIN;
	BSP_ISR_PORT_IES |= BSP_ISR_PIN;
	BSP_ISR_PORT_IE	|= BSP_ISR_PIN;
	__bis_SR_register(GIE);
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
*	@fn		nm_bsp_uart_sendnm_bsp_uart_send
*	@author	M. Abdelmawla
*	@date	29 July 2012
*	@version	1.0
*/
void nm_bsp_uart_send(uint8 *pu8Buf, uint16 u16Sz)
{
	int i;
	for(i = 0; i < u16Sz; i++)
	{
		while (!(UCA0IFG & UCTXIFG)); // USCI_A0 TX buffer ready?
		UCA0TXBUF = pu8Buf[i];
	}
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
		if((gstrAdcParm.pfAdcCb !=NULL)||(gstrWakeTimer.u8Enabled))
		{
			TA1CCR0 = BSP_MIN(gstrAdcParm.u32AdcSampleTime,gstrWakeTimer.strTimer.u32Period);
		}
		__bis_SR_register(LPM3_bits + GIE);
		TA1CCR0 = TICK_RES;
	}
}
#endif
/**
*	@fn		nm_bsp_register_adc_cb
*	@brief	Start POWER SAVE FOR MCU
*	@author	M.S.M
*	@date	12 FEB 2013
*	@version	1.0
*/
void nm_bsp_register_adc(tpfNmBspAdcCb pfCb,uint16 u16SampleTime)
{
	gstrAdcParm.u32AdcSampleTime = u16SampleTime;
	gstrAdcParm.pfAdcCb = pfCb;
	gstrAdcParm.u32AdcTimeout = 0;
}
/**
*	@fn		nm_bsp_take_ADC_measurement
*	@brief	Start POWER SAVE FOR MCU
*	@author	M.S.M
*	@date	12 FEB 2013
*	@version	1.0
*/
void nm_bsp_take_ADC_measurement(void)
{
  while (ADC10CTL1 & BUSY);
  ADC10CTL0 |= ADC10ENC | ADC10SC ;       // Start conversion
  __no_operation();                       // For debug only
}
#endif
#endif
