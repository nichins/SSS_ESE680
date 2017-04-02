/**
*  @file		nm_bsp_k20d50m.c				
*  @brief		This module contains NMC1000 ABOV bsp APIs implementation 
*  @author		Ahmad.Mohammad.Yahya
*  @date		21 MARCH 2013
*  @version		1.0	
*/
#ifdef __K20D50M__
#include "bsp\include\nm_bsp.h"
#include "common\include\nm_common.h"
#include "bus_wrapper\include\nm_bus_wrapper.h"

#define PORTC_PIN10_MASK  (1<<10) /* External interrupt */
#define PORTC_PIN2_MASK   (1<<2)  /* SW3 */
#define PORTC_PIN8_MASK	  (1<<8)  /* Connected to Led on J2(5) */
#define PORTD_PIN7_MASK   (1<<7)  /* Reset cortus chip */
#define PORTC_PIN0_MASK   (1<<0)  /* Connected to Led on J4(2) */
#define PORTC_PIN1_MASK	  (1<<1)  /* Connected to Buzzer on J1(1) */

#define PORTC10_ISR_PIN 10  /* PTC10 */
#define PORTC10_ISR_MASK (1<<PORTC10_ISR_PIN)

#define PORTC2_ISR_PIN 2  /* PTC2 */
#define PORTC2_ISR_MASK (1<<PORTC2_ISR_PIN)

#define PORTC1_ISR_PIN 1  /* PTC1 */
#define PORTC1_ISR_MASK (1<<PORTC1_ISR_PIN)



#define SHORT_PRESS_DEBOUNCE	(20/TICK_RES)
#define LONG_PRESS_DEBOUNCE		(1000/TICK_RES)

static volatile tpfNmBspIsr gpfIsr;
static volatile uint8 gu8BtnIfg;
static volatile tstrTimer gstrTimer;
static volatile tpfNmBspBtnPress  gpfBtns;
static volatile uint16 gu16Btn1Cnt;
volatile uint32 gu32Jiffies;

/**
*	@fn		External_Interrupt_PORTC_PIN10(void)
*	@brief	ISR for any External interrupt on PORTC "PIN10 OR SW3 on PIN2" in this case
*	@author	Ahmad.Mohammad.Yahya
*	@date	21 MARCH 2013
*	@version	1.0
*/
void External_Interrupt_PORTC_PIN10(void)
{
	/* Interrupt from SW3 ptc2 */
	if((PORTC_ISFR) & (PORTC2_ISR_MASK))
	{
		gu8BtnIfg |= PORTC_ISFR;
		PORTC_PCR2 |= PORT_PCR_IRQC(0);  /* Disable interrupt from SW3 */
		
	}
	
	
	/* External Interrupt from NMC1000 Chip ptc10 */
	if ((PORTC_ISFR) & (PORTC10_ISR_MASK)) 
	{
		if(gpfIsr)
		{
			gpfIsr();
		}
		
        PORTC_ISFR |= PORTC10_ISR_MASK;  /*Clear ISR flag */
	}
}

/**
*	@fn		btn_poll(void)
*	@brief	Short or Long press
*	@author	Ahmad.Mohammad.Yahya
*	@date	21 MARCH 2013
*	@version	1.0
*/

static void btn_poll(void)
{
	if (gu8BtnIfg & SW3) 
	{
		gu16Btn1Cnt++;

		if (gu16Btn1Cnt >= SHORT_PRESS_DEBOUNCE) 
		{
			if (GPIOC_PDIR &  SW3) 
			{
				gu16Btn1Cnt = 0;
				gpfBtns(SW3, 0); /* Short press callback */
			}
			else 
			{
				if (gu16Btn1Cnt >= LONG_PRESS_DEBOUNCE) 
				{
					gu16Btn1Cnt = 0;
					gpfBtns(SW3, 1); /* Short press callback */
				}
			}
		}
	}

	if(!gu16Btn1Cnt)
	{
		gu8BtnIfg &= ~SW3;
		PORTC_PCR2 |= PORT_PCR_IRQC(10); /* Enable interrupt */
		PORTC_ISFR |= PORTC2_ISR_MASK;  /*Clear ISR flag */
	}

}


/**
*	@fn		init_tick_timer(void)
*	@brief	timer tick init
*	@author	Ahmad.Mohammad.Yahya
*	@date	21 MARCH 2013
*	@version	1.0
*/
void init_tick_timer(void)
{

	SIM_SCGC6|=SIM_SCGC6_PIT_MASK; /* power on the PIT clock to initialize the timer clock */
	enable_irq(INT_PIT1-16); /* Enable PIT interrupt if overflow */
	
	PIT_MCR = 0x00;  /* turn on PIT */

	PIT_LDVAL1 = (TICK_RES * 49588);  /* setup timer 1 for  cycles */

	PIT_TCTRL1 = PIT_TCTRL_TIE_MASK; /* enable Timer 1 interrupts */
	PIT_TCTRL1 |= PIT_TCTRL_TEN_MASK; /* start Timer 1 */
}

void PIT_timer_isr(void)
{	
	gu32Jiffies++;
	btn_poll();
	
	if(gstrTimer.pfCb)
	{
		if(NM_BSP_TIME_MSEC >= gstrTimer.u32Timeout)
		{
			gstrTimer.pfCb();
			gstrTimer.u32Timeout = NM_BSP_TIME_MSEC + gstrTimer.u32Period;
		}
	}
	PIT_TFLG1 = PIT_TFLG_TIF_MASK; /* write 1 on the flag to clear it */

}

/**
*	@fn		nm_bsp_init
*	@brief	Initialize BSP
*	@return	0 in case of success and -1 in case of failure
*   @author		Ahmad.Mohammad.Yahya
*   @date		21 MARCH 2013
*	@version	1.0
*/
sint8 nm_bsp_init(void) 
{
	sint8 result = M2M_SUCCESS;
	
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; /* Enable clock on PORTC */ 
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK; /* Enable clock on PORTD */

	
	
	/* PORTD_PIN7 o/p ----> used to Reset Cortus */
	PORTD_PCR7 = PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0) | PORT_PCR_ISF_MASK | PORT_PCR_MUX(1);
	GPIOD_PDDR |= (PORTD_PIN7_MASK);   /* Configure PORTC_PIN2 -- as output gpio */
	
	/* Config. SW3 button */
	PORTC_PCR2 = PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(10) | PORT_PCR_ISF_MASK | PORT_PCR_MUX(1);
	GPIOC_PDDR &= ~(PORTC_PIN2_MASK);   /* Configure PORTC_PIN2 -- as input gpio */
	
	
	/* Config. PTC 8, 1, 11 as OUTPUT for 2 Leds & Buzzer: */
	
	/* LED_WIFI on J4(2)    -----> PTC0 J2(6) */
	PORTC_PCR0 = PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0) | PORT_PCR_ISF_MASK | PORT_PCR_MUX(1);
	GPIOC_PDDR |= (PORTC_PIN0_MASK);   /* Configure PORTC_PIN2 -- as output gpio */
	
	/* LED_GROWL on J6(1)    -----> PTC8 J6(2)  J6 is short */ 
	PORTC_PCR8 = PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0) | PORT_PCR_ISF_MASK | PORT_PCR_MUX(1);
	GPIOC_PDDR |= (PORTC_PIN8_MASK);   /* Configure PORTC_PIN2 -- as output gpio */
	
	/* Buzzer on J1(1) -----> PTC1 J2(4) */
	PORTC_PCR1 = PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0) | PORT_PCR_ISF_MASK | PORT_PCR_MUX(1);
	GPIOC_PDDR |= (PORTC_PIN1_MASK);   /* Configure PORTC_PIN2 -- as output gpio */

	
	init_tick_timer();

	nm_bsp_reset();
	
	
	return result;	 
}
/*
*	@fn		nm_bsp_btn_init
*	@brief	Initialize buttons driver
*   @author		Ahmad.Mohammad.Yahya
*   @date		21 MARCH 2013
*	@version	1.0
*/
void nm_bsp_btn_init(tpfNmBspBtnPress pfBtnCb)
{
	/* SW3 button on PTC2 as external interrupt */
	PORTC_PCR2 = PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(10) | PORT_PCR_ISF_MASK | PORT_PCR_MUX(1);
	
	gpfBtns = pfBtnCb;
	gu8BtnIfg = 0;
	gu16Btn1Cnt = 0;
	
}

/*
*	@fn		nm_bsp_sleep
*	@brief	Sleep in units of mSec
*	@param[IN]	u32TimeMsec
*				Time in milliseconds
*   @author		Ahmad.Mohammad.Yahya
*   @date		21 MARCH 2013
*	@version	1.0
*/
void nm_bsp_sleep(uint32 u32TimeMsec) 
{
	time_delay_ms(u32TimeMsec); /* Use internal LOW POWER TIMER for delay */
}

/*
*	@fn		nm_bsp_register_isr
*	@brief	Register interrupt service routine
*	@param[IN]	pfIsr
*				Pointer to ISR handler
*   @author		Ahmad.Mohammad.Yahya
*   @date		21 MARCH 2013
*	@sa		tpfNmBspIsr
*	@version	1.0
*/

void nm_bsp_register_isr(tpfNmBspIsr pfIsr)
{  
	/* Enable PORTC clock */
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; 
	
    /*Configure NVIC for PortC & Timer(PIT1) Isr */ 
    enable_irq(INT_PORTC-16);
	
	/*Configure external interrupt on PTC10 on falling edge */
	PORTC_PCR10 = PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(10) | PORT_PCR_ISF_MASK | PORT_PCR_MUX(1);
	
    GPIOC_PDDR &= ~(PORTC_PIN10_MASK);   /*Configure PORTC_PIN10 -- as input gpio*/
	gpfIsr = pfIsr;
}

/*
*	@fn		nm_bsp_interrupt_ctrl
*	@brief	Enable/Disable interrupts
*	@param[IN]	u8Enable
*				'0' disable interrupts. '1' enable interrupts
*   @author		Ahmad.Mohammad.Yahya
*   @date		21 MARCH 2013
*	@version	1.0
*/
void nm_bsp_interrupt_ctrl(uint8 u8Enable) 
{
	if(u8Enable)
	{	
		enable_irq(INT_PORTC-16);
	}
	else
	{
		disable_irq(INT_PORTC-16);
	}

}

/**
*	@fn		Buzzer(uint16 u16retry)
*	@brief	Turn on buzzer for (u16retry)ms
*	@param[IN]	u16retry
*				No. of ms turning on the Buzzer 
*   @author		Ahmad.Mohammad.Yahya
*   @date		21 MARCH 2013
*	@version	1.0
*/
void nm_bsp_buzzer(uint16 u16retry)
{
	while(u16retry)
	{
		GPIOC_PDOR |= BUZZER_ERR;
		nm_bsp_sleep(1);
		GPIOC_PDOR &= ~BUZZER_ERR;
		nm_bsp_sleep(1);
		u16retry--;
	}
}

/**
*	@fn		nm_bsp_reset(void)
*	@brief	reset chip Automatically
*   @author		Ahmad.Mohammad.Yahya
*   @date		21 MARCH 2013
*	@version	1.0
*/
void nm_bsp_reset(void)  
{
	/* PORTD_PIN7 ----> Reset */
	
	/* Enable PORTD clock */
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK; 
	GPIOD_PDOR |= (1<<7);  /* Set 1 on pin */
	nm_bsp_sleep(100);
	GPIOD_PDOR &= ~(1<<7); /* Set 1 on pin */
	nm_bsp_sleep(200);
	GPIOD_PDOR |= (1<<7);  /* Set 1 on pin */
	nm_bsp_sleep(200);


}

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

void nm_bsp_timer_start(tpfNmBspTimerCb pfCb,uint32 u32PeriodTick)  
{
	gu32Jiffies =0;
	gstrTimer.u32Timeout = u32PeriodTick + NM_BSP_TIME_MSEC;
	gstrTimer.u32Period = u32PeriodTick;
	gstrTimer.pfCb  = pfCb;
}
/**
*	@fn		nm_bsp_stop_timer
*	@brief	Start periodic timer
*   @author		Ahmad.Mohammad.Yahya
*   @date		21 MARCH 2013
*	@version	1.0
*/
void nm_bsp_stop_timer(void)
{
	gstrTimer.pfCb = NULL;
}
#endif
