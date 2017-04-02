/**
*  @file		nm_bsp_NRF51822.c
*  @brief		This module contains NORDIC bsp APIs implementation
*  @author		M.S.M
*  @date		28 OCT 2013
*  @version		1.0
*/

#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#include "bsp/include/nm_bsp_nrf51822.h"
#ifdef _SOFT_DEVICE_
#include "app_timer.h"
#endif

#define BSP_M_CLK_KHZ		(16000)
#define FW_SIZE			(128 * 1024UL)
#define BSP_MIN(x,y) ((x)>(y)?(y):(x))

#define NMC1500_RESET_PIN_NUMBER		(18)	/*nmc1000 reset pin number*/
#define BTN1_PIN_NUMBER							(0)	/*btn1  pin number*/
#define BTN2_PIN_NUMBER							(1)	/*btn2  pin number*/
#define NMC1500_ISR_PIN_NUMBER 			(19)	/*nmc1000 isr pin number*/
#define RX_PIN_NUMBER								(16)	/* UART RX pin number.	  */
#define TX_PIN_NUMBER								(17)	/* UART TX pin number.	  */



#define TIMER0_PRESCALER                (9UL)                                   /**< Timer 0 prescaler */
#define TIMER0_CLOCK                    (SystemCoreClock >> TIMER0_PRESCALER)   /**< Timer clock frequency */
#define MS_TO_TIMER0_TICKS(ms)          ((1000000UL * ms) / (TIMER0_CLOCK))     /**< Converts milliseconds to timer ticks */


#ifdef _SOFT_DEVICE_
#define SHORT_PRESS_DEBOUNCE	(400/TICK_RES)
#else

#endif
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
#ifdef _SOFT_DEVICE_
static app_timer_id_t                 m_detection_delay_timer_idxx;  /**< Polling timer id. */
#endif
/**
*
* Static functions
*/
static void btn_poll(void)
{
	if (gu8BtnIfg & SW1) {
		gu16Btn1Cnt++;

		if (gu16Btn1Cnt >= SHORT_PRESS_DEBOUNCE) {
			if (nrf_gpio_pin_read(BTN1_PIN_NUMBER))
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
			if (nrf_gpio_pin_read(BTN2_PIN_NUMBER)) {
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
		NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN1_Set << GPIOTE_INTENSET_IN1_Pos;
	}
	if (!gu16Btn2Cnt) {
		gu8BtnIfg &= ~SW2;
		NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN2_Set << GPIOTE_INTENSET_IN2_Pos;
	}
}
/*Timer interrupt*/
#ifdef _SOFT_DEVICE_
static void timer(void * p_context)
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
#endif
static void uart_init(void)
{

	nrf_gpio_cfg_output(TX_PIN_NUMBER);
	nrf_gpio_cfg_input(RX_PIN_NUMBER, NRF_GPIO_PIN_NOPULL);  

	NRF_UART0->PSELTXD = TX_PIN_NUMBER;
	NRF_UART0->PSELRXD = RX_PIN_NUMBER;
	NRF_UART0->BAUDRATE         = (UART_BAUDRATE_BAUDRATE_Baud115200 << UART_BAUDRATE_BAUDRATE_Pos);
	NRF_UART0->ENABLE           = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
	NRF_UART0->TASKS_STARTTX    = 1;
	NRF_UART0->TASKS_STARTRX    = 1;
	NRF_UART0->EVENTS_RXDRDY    = 0;
}

/**
*
* Interrupt handler
*/
void TIMER0_IRQHandler(void)
{
	if ((NRF_TIMER0->EVENTS_COMPARE[0] != 0) && \
		((NRF_TIMER0->INTENSET & TIMER_INTENSET_COMPARE0_Msk) != 0))
	{
		NRF_TIMER0->EVENTS_COMPARE[0] = 0;
		NRF_TIMER0->CC[0]            += MS_TO_TIMER0_TICKS(TICK_RES);
#ifdef _STATIC_PS_
		if((gstrWakeTimer.strTimer.pfCb)&&(gstrWakeTimer.u8Enabled))
		{
			gu32Jiffies+=(TICK_RES_SLEEP/TICK_RES);
			if(NM_BSP_TIME_MSEC >= gstrWakeTimer.strTimer.u32Timeout)
			{
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
	}
}

static void timer0_init(void)
{
#ifndef _SOFT_DEVICE_
	// Set the timer in Timer Mode
	NRF_TIMER0->MODE      = TIMER_MODE_MODE_Timer; 
	NRF_TIMER0->PRESCALER = TIMER0_PRESCALER;
	// 24-bit mode
	NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_24Bit;  
	// Enable interrupt for COMPARE[0]
	NRF_TIMER0->INTENSET    = (1UL << TIMER_INTENSET_COMPARE0_Pos);
	NRF_TIMER0->CC[0]       = MS_TO_TIMER0_TICKS(TICK_RES);
	NRF_TIMER0->TASKS_START = 1; // Start clocks

	// Enable Interrupt for the timer in the core
	NVIC_EnableIRQ(TIMER0_IRQn); 

#endif
}
void GPIOTE_IRQHandler(void)
{
	nrf_gpio_pin_toggle(NMC1500_RESET_PIN_NUMBER);
	/*nmc1000 isr*/
	if ((NRF_GPIOTE->EVENTS_IN[0] == 1) && 
		(NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN0_Msk))
	{
		// Event causing the interrupt must be cleared.
		NRF_GPIOTE->EVENTS_IN[0] = 0;

#ifdef _STATIC_PS_
		nm_bsp_wake_ctrl(0);
#endif
		if(gpfIsr)
		{
			gpfIsr();
		}
	}
	/*btn1 isr*/
	if ((NRF_GPIOTE->EVENTS_IN[1] == 1) && 
		(NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN1_Msk))
	{
		NRF_GPIOTE->EVENTS_IN[1] = 0 ;

#ifdef _STATIC_PS_
		nm_bsp_wake_ctrl(0);
#endif
		NRF_GPIOTE->INTENCLR  = GPIOTE_INTENCLR_IN1_Clear << GPIOTE_INTENCLR_IN1_Pos;
		gu8BtnIfg |= SW1;
	}
	/*btn2 isr*/
	if ((NRF_GPIOTE->EVENTS_IN[2] == 1) && 
		(NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN2_Msk))
	{
		NRF_GPIOTE->EVENTS_IN[2] = 0 ;

#ifdef _STATIC_PS_
		nm_bsp_wake_ctrl(0);
#endif
		NRF_GPIOTE->INTENCLR  = GPIOTE_INTENCLR_IN2_Clear << GPIOTE_INTENCLR_IN2_Pos;
		gu8BtnIfg |= SW2;
	}

}
void nm_bsp_reset(void)
{
	// Assert reset signal. Will be de-asserted after SPI init
	nrf_gpio_cfg_output(NMC1500_RESET_PIN_NUMBER);
	nrf_gpio_pin_clear(NMC1500_RESET_PIN_NUMBER);
	nm_bsp_sleep(100);
	nrf_gpio_pin_set(NMC1500_RESET_PIN_NUMBER);
	nm_bsp_sleep(200);
	nrf_gpio_pin_clear(NMC1500_RESET_PIN_NUMBER);
#ifndef _PS_CLIENT_
	nm_bsp_sleep(200);
	nrf_gpio_pin_set(NMC1500_RESET_PIN_NUMBER);
	nm_bsp_sleep(100);
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

	nm_bsp_reset();

	timer0_init();

	//__enable_irq();

	uart_init();
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

	nrf_gpio_cfg_input(BTN1_PIN_NUMBER, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_pin_write(BTN1_PIN_NUMBER, 1);


	//nrf_gpiote_event_config(BTN1_PIN_NUMBER,BTN1_PIN_NUMBER,GPIOTE_CONFIG_POLARITY_HiToLo);
	// Enable interrupt BTN1:
	NRF_GPIOTE->CONFIG[1] =  (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos)
		| (BTN1_PIN_NUMBER << GPIOTE_CONFIG_PSEL_Pos)  
		| (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
	NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN1_Set << GPIOTE_INTENSET_IN1_Pos;

	nrf_gpio_cfg_input(BTN2_PIN_NUMBER, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_pin_write(BTN2_PIN_NUMBER, 1);


	//nrf_gpiote_event_config(BTN1_PIN_NUMBER,BTN1_PIN_NUMBER,GPIOTE_CONFIG_POLARITY_HiToLo);
	// Enable interrupt BTN2:
	NRF_GPIOTE->CONFIG[2] =  (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos)
		| (BTN2_PIN_NUMBER << GPIOTE_CONFIG_PSEL_Pos)  
		| (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
	NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN2_Set << GPIOTE_INTENSET_IN2_Pos;
	NVIC_EnableIRQ(GPIOTE_IRQn);
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
	while(u32TimeMsec--) nrf_delay_us(999);
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
void nm_bsp_register_isr(tpfNmBspIsr pfIsr)
{
	gpfIsr = pfIsr;
	nrf_gpio_cfg_input(NMC1500_ISR_PIN_NUMBER, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_pin_write(NMC1500_ISR_PIN_NUMBER, 1);

	//nrf_gpiote_event_config(NMC1500_ISR_PIN,NMC1500_ISR_PIN,GPIOTE_CONFIG_POLARITY_HiToLo);
	// Enable interrupt:

	NRF_GPIOTE->CONFIG[0] =  (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos)
		| (NMC1500_ISR_PIN_NUMBER << GPIOTE_CONFIG_PSEL_Pos)  
		| (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
	NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Set << GPIOTE_INTENSET_IN0_Pos;

	NVIC_EnableIRQ(GPIOTE_IRQn);
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
		NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Set << GPIOTE_INTENSET_IN0_Pos;
	}
	else
	{
		NRF_GPIOTE->INTENCLR  = GPIOTE_INTENCLR_IN0_Clear << GPIOTE_INTENCLR_IN0_Pos;
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
	uint_fast8_t i = 0;
	uint8_t ch = pu8Buf[i++];
	//while (ch != '\0')
	while(u16Sz)	
	{
		NRF_UART0->TXD = (uint8_t)ch;
		while (NRF_UART0->EVENTS_TXDRDY!=1)
		{
			// Wait for TXD data to be sent
		}
		NRF_UART0->EVENTS_TXDRDY=0;
		ch = pu8Buf[i++];
		u16Sz--;
	}
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
	
#ifdef _SOFT_DEVICE_
	app_timer_create(&m_detection_delay_timer_idxx,APP_TIMER_MODE_REPEATED,timer);
	app_timer_start(m_detection_delay_timer_idxx,APP_TIMER_TICKS(TICK_RES,0),NULL);
#endif
}
/**
*	@fn		nm_bsp_stop_timer
*	@brief	Start periodic timer
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


