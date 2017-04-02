/**
*  @file		nm_bsp_mcf51cn128.c				
*  @brief		This module contains NMC1000 freescale  bsp APIs implementation 
*  @author		M.S.M
*  @date		18 JULY 2012
*  @version		1.0	
*/
#ifdef _FREESCALE_MCF51CN128_ 
#include "bsp\include\nm_bsp.h"
#include "common\include\nm_common.h"
#include "bsp\include\nm_spi_flash.h"
#include "bus_wrapper\include\nm_bus_wrapper.h"

/**
number of channels used in adc
**/
#define ADC_CH_COUNT 2
/**
interput pin
**/
#define INT_PIN   (GPIO_PORT_TG | GPIO_PIN4)
/**
Global defination
**/
static FILE_PTR pFileInt = NULL;
static FILE_PTR pFileLed = NULL;
static FILE_PTR pFileButt = NULL;
static FILE_PTR pFileReset = NULL;
static FILE_PTR  fd_adc, fd_ch[ADC_CH_COUNT];
static MY_ISR_STRUCT_PTR  isr_ptr = NULL;
/**/
static volatile uint8 gu8IntEnable  =1;
static tpfNmBspIsr gpfIsr;
static tpfNmBspIsr gpfIsr;
static tstrTimer gstrTimer;
static tpfNmBspAdcCb gpfAdcCb;
uint32 gu32Jiffies;
/**/
/*LEDs*/
#define LED_1   BSP_LED1
#define LED_2   BSP_LED2
#define LED_3   BSP_LED3
#define LED_4   BSP_LED4
#define CHIP_RESET  (GPIO_PORT_TH | GPIO_PIN7)
/**/

const ADC_INIT_STRUCT adc_init = {
	ADC_RESOLUTION_DEFAULT,     /* resolution */
};
/**
*	@struct		ADC_INIT_CHANNEL_STRUCT
*	@brief		array to hold adc pins and its paramater
*	@author		M.S.M
*	@version	1.0
*/ 
const ADC_INIT_CHANNEL_STRUCT adc_ch_param[ADC_CH_COUNT] = {
	{
		ADC_SOURCE_AN7,
			ADC_CHANNEL_MEASURE_LOOP | ADC_CHANNEL_START_NOW,
			10,            /* number of samples in one run sequence */
			0,             /* time offset from trigger point in ns */
			50,          /* period in us (50ms) */
			0x10000,       /* scale range of result (not used now) */
			1,             /* circular buffer size (sample count) */
			ADC_TRIGGER_1  /* logical trigger ID that starts this ADC channel */
	},
	{
		ADC_SOURCE_AN6,
			ADC_CHANNEL_MEASURE_LOOP | ADC_CHANNEL_START_NOW,
			10,            /* number of samples in one run sequence */
			0,             /* time offset from trigger point in ns */
			50,     /* period in us (50ms) */
			0x10000,       /* scale range of result (not used now) */
			1,          /* circular buffer size (sample count) */
			ADC_TRIGGER_1  /* logical trigger ID that starts this ADC channel */
		}
};

/*
*  Setup ADC module to read in accelerometer and potentiometer values
*/   
static void nm_bsp_adc_init(void) {
	_mqx_int i;
	char dev_name[10];
	M2M_INFO("adc init\n\r");

	fd_adc = fopen("adc:", (const char*)&adc_init);
	if (NULL == fd_adc) {    
		M2M_ERR("ADC device open failed\n");
		_mqx_exit(-1);
	}
  _time_delay(1);
	for (i = 0; i < ADC_CH_COUNT; i++) {
		sprintf(dev_name, "adc:%d", i);
		fd_ch[i] = fopen(dev_name, (const char*)&adc_ch_param[i]);
		if (NULL == fd_ch[i]) {    
			M2M_ERR("adc:%d channel open failed\n", i);
			_mqx_exit(-1);
		}
		_time_delay(1);	
	}
	_time_delay (100);

}
static void tick_isr(pointer user_isr_ptr)
{
	MY_ISR_STRUCT_PTR  isr_ptr;

	isr_ptr = (MY_ISR_STRUCT_PTR)user_isr_ptr;
	isr_ptr->TICK_COUNT++;
	gu32Jiffies = isr_ptr->TICK_COUNT;
	if(gstrTimer.pfCb)
	{
		if(	isr_ptr->TICK_COUNT >= gstrTimer.u32Timeout)
		{
			gstrTimer.pfCb();
			gstrTimer.u32Timeout = 	isr_ptr->TICK_COUNT + gstrTimer.u32Period;
		}
	}
	/* Chain to the previous notifier */
	(*isr_ptr->OLD_ISR)(isr_ptr->OLD_ISR_DATA);
}
static void kbi_callback(pointer) {


	uint_32 data[] = {
		INT_PIN,	
		GPIO_LIST_END
	};
	if (pFileInt) {
		if (IO_OK !=ioctl(pFileInt, GPIO_IOCTL_READ,  &data)) {
			M2M_ERR("failed interupt\n\r");
			_mqx_exit(-1);
		}
	}
	if(gu8IntEnable) { 
		if((data[0] & GPIO_PIN_STATUS)==0) {
			if(gpfIsr)
				gpfIsr();
		}
	}
}
static sint8 burn_firmware(uint8 enable) {
	sint8 result = M2M_SUCCESS; 
	uint_32 resultx;
	if(enable) {
		unsigned long lSize;
		FILE_PTR fx;
		uint_32  param;
		uint32 u32Offset=0;
		uint8 buffer[BSP_MAX_BUF_CPY_SZ];
		uint16 g;
		nm_bsp_gpio_ctrl(ON,LED_DOWNLOAD);
		/*erase chip*/
		nm_bsp_buf_erase(NM_BSP_BUF_FIRMWARE);
		/*open new serial port*/
		fx=fopen("ttya:",NULL);
		if(fx==NULL) {
			M2M_ERR("failed\n\r");
			_mqx_exit(-1);
		}
		/**set baudrate*/
		param =115200;
		if (IO_OK != ioctl(fx, IO_IOCTL_SERIAL_SET_BAUD,&param)) {
			M2M_ERR("failed set baud rate\n\r"); 
			result = M2M_ERR_FIRMWARE_bURN;
			return result;

		}
		/*set data bus*/
		param = 8;
		if (IO_OK != ioctl(fx, IO_IOCTL_SERIAL_SET_DATA_BITS,&param)) {
			M2M_ERR("failed set data width\n\r"); 
			result = M2M_ERR_FIRMWARE_bURN;
			return result;

		}
		/*send alogrithem **/
		write(fx,"#",1);/*send start char**/
		fflush(fx);
		read(fx,(char *)&lSize,4);/**read total file size*/
		fflush(fx);
		nm_bsp_sleep(100);
		/*write size in spi flash **/
		nm_bsp_buf_write(NM_BSP_BUF_FIRMWARE, (unsigned char*)&lSize, u32Offset,4);
		/*swap to big endian from little**/
		lSize =NM_BSP_B_L_32(lSize);
		u32Offset+=4;
		/**send one block for every "*" sended */
		write(fx,"*",1);
		fflush(fx);
		while(lSize)
		{
			memset(buffer,0,sizeof(buffer));
			if(lSize<BSP_MAX_BUF_CPY_SZ) {
				g=read(fx,buffer,lSize);
			}else {
				g=read(fx,buffer,BSP_MAX_BUF_CPY_SZ);
			}
			fflush(fx);
			/**write the block in spi flash*/
			nm_bsp_buf_write(NM_BSP_BUF_FIRMWARE, buffer, u32Offset , g);
			lSize-=g;
			u32Offset+=g;
			nm_bsp_sleep(1);
			/*another block**/
			write(fx,"*",1);
			fflush(fx);
		}
		/* Close the uart */
		resultx = fclose(fx);
		if (resultx)
		{
			M2M_ERR("Error closing uart, returned: 0x%08x\n", resultx);
			result=M2M_ERR_FIRMWARE_bURN;
		}
	}
	nm_bsp_gpio_ctrl(OFF,LED_DOWNLOAD);
	return result;
}
static void Led_init(void) {
	const uint_32 output_set[] = {
		LED_1 | GPIO_PIN_STATUS_0,
		LED_2 | GPIO_PIN_STATUS_0,
		LED_3 | GPIO_PIN_STATUS_0,
		LED_4 | GPIO_PIN_STATUS_0,
		CHIP_RESET | GPIO_PIN_STATUS_1, 
		GPIO_LIST_END
	};
	/* and open with new specification */
	if (NULL == (pFileLed = fopen("gpio:write", (char_ptr) &output_set )))
	{
		M2M_ERR("Opening file1 GPIO for pins2 failed.\n");
		_mqx_exit(-1);
	}

}
/**
*	@fn		nm_bsp_init
*	@brief	Initialize BSP
*	@return	0 in case of success and -1 in case of failure
*	@author	M.S.M
*	@date	18 July 2012
*	@version	1.0
*/
sint8 nm_bsp_init(void) 
{
	sint8 result = M2M_SUCCESS;
	M2M_INFO("bsp init\n\r");
	Led_init();
	/*sensors init*/
	nm_bsp_btn_init();
	#ifndef _I2C_FIRMWARE_ 
	nm_spi_flash_init();
	/* Disable protection */
	nm_spi_flash_set_protection (FALSE);
	/*Read chip ID*/
	nm_spi_flash_read_Chipid();
	/* burn firmware */
	result= burn_firmware((uint8)nm_bsp_get_btn_status(DIP1));
	#endif
	nm_bsp_adc_init();
	return result;
}

/*
*	@fn		nm_bsp_sleep
*	@brief	Sleep in units of mSec
*	@param[IN]	u32TimeMsec
*				Time in milliseconds
*	@author	M.S.M
*	@date	18 July 2012
*	@version	1.0
*/
void nm_bsp_sleep(uint32 u32TimeMsec) 
{
	_time_delay(u32TimeMsec);
}
/*
*	@fn		nm_bsp_buf_read
*	@brief	Read from permenant storage buffer
*	@param[IN]	enuType
*				Buffer type 
*	@param[OUT]	pu8Buf
*				Pointer to buffer used to copy the read data form the permanent storage.
*				Must be allocated by the caller
*	@param[IN]	u32Offset
*				Start read offset with in the permanent storage buffer
*	@param[IN]	u32Sz
*				The requested read size
*	@return	0 in case of success and -1 in case of failure
*	@author	M.S.M
*	@date	18 July 2012
*	@sa		tenuNmBspBufType
*	@version	1.0
*/
sint8 nm_bsp_buf_read(tenuNmBspBufType enuType, uint8* pu8Buf,
					  uint32 u32Offset, uint32 u32Sz) 
{
	switch(enuType)
	{
	case NM_BSP_BUF_FIRMWARE:
		nm_spi_flash_read_data(u32Offset, u32Sz, pu8Buf);
		break;
	case NM_BSP_BUF_CONFIG:
	default:
		NM_BSP_PRINTF("invalid buffer type\n");
	}       
	return 0;
}
/*
*	@fn		nm_bsp_buf_write
*	@brief	write permenant storage buffer
*	@param[IN]	enuType
*				Buffer type 
*	@param[OUT]	pu8Buf
*				Pointer to buffer used to copy the read data form the permanent storage.
*				Must be allocated by the caller
*	@param[IN]	u32Offset
*				Start read offset with in the permanent storage buffer
*	@param[IN]	u32Sz
*				The requested read size
*	@return	0 in case of success and -1 in case of failure
*	@author	M.S.M
*	@date	18 July 2012
*	@sa		tenuNmBspBufType
*	@version	1.0
*/
sint8 nm_bsp_buf_write(tenuNmBspBufType enuType, uint8* pu8Buf,
					   uint32 u32Offset, uint32 u32Sz) 
{

	nm_spi_flash_write_data (u32Offset,u32Sz,pu8Buf);
	return M2M_SUCCESS;
}

/*
*	@fn		nm_bsp_buf_erase
*	@brief	erase storage buffer	(set to 0xff)
*	@param[IN]	enuType
*				Buffer type
*	@return	0 in case of success and -1 in case of failure
*	@author	M.S.M
*	@date	18 July 2012
*	@sa		tenuNmBspBufType
*	@version	1.0
*/
sint8 nm_bsp_buf_erase(tenuNmBspBufType enuType) 
{

	switch(enuType)
	{
	case NM_BSP_BUF_FIRMWARE:
		nm_spi_flash_chip_erase ();

		break;
	case NM_BSP_BUF_CONFIG:
	default:
		NM_BSP_PRINTF("invalid buffer type\n");
	} 
	return 0;
}
/*
*	@fn		nm_bsp_register_isr
*	@brief	Register interrupt service routine
*	@param[IN]	pfIsr
*				Pointer to ISR handler
*	@author	M.S.M
*	@date	29 July 2012
*	@sa		tpfNmBspIsr
*	@version	1.0
*/
void nm_bsp_register_isr(tpfNmBspIsr pfIsr)
{    

	const uint_32 input_set[] = {
		INT_PIN | GPIO_PIN_IRQ,
		GPIO_LIST_END
	};
	M2M_INFO("isr init\n\r");
	gpfIsr = pfIsr; 

	/* Open and set port DD as input to read value from switches */
	pFileInt = fopen("gpio:read", (char_ptr) &input_set);
	if(pFileInt==NULL) {
		M2M_ERR("failed open pin\n\r");
		_mqx_exit(-1);

	}
	if (IO_OK !=ioctl(pFileInt, GPIO_IOCTL_SET_IRQ_FUNCTION, (pointer) &kbi_callback)) {
		M2M_ERR("failed set irq\n\r");
		_mqx_exit(-1);

	}
}
/*
*	@fn		nm_bsp_btn_init
*	@brief	Initialize buttons driver
*	@author	M. Abdelmawla
*	@date	01 August 2012
*	@version	1.0
*/
void nm_bsp_btn_init(void)
{

	GPIO_PIN_STRUCT input_set[] = {
		BSP_BUTTON1 ,
		BSP_BUTTON2 ,
		BSP_SWITCH1_2 ,
		GPIO_LIST_END
	};

	/* Open and set port DD as input to read value from switches */
	pFileButt = fopen("gpio:read", (char_ptr) &input_set);
	if(pFileButt==NULL) {
		M2M_ERR("failed open pin\n\r");
		_mqx_exit(-1);
	}

}
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
boolean  nm_bsp_get_btn_status(uint8 bnum)
{
	GPIO_PIN_STRUCT input_set[] = {
		BSP_BUTTON1 ,
		BSP_BUTTON2 ,
		BSP_SWITCH1_2,
		GPIO_LIST_END
	};

	if (IO_OK !=ioctl(pFileButt, GPIO_IOCTL_READ,&input_set)) {
		M2M_ERR("failed read button state\n\r");
		_mqx_exit(-1);
	}
	return  ((input_set[bnum] & GPIO_PIN_STATUS)==0);

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
void nm_bsp_interrupt_ctrl(uint8 u8Enable) {
	gu8IntEnable = u8Enable; 
}
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
_mqx_int ReadADC(_mqx_int channel) {
	_mqx_int val;

	return (channel < ADC_CH_COUNT && read(fd_ch[channel], &val, sizeof(val))) ? val : 0;
}
/**
*	@fn		nm_bsp_reset(void)
*	@brief	reset chip
*	@author	M.S.M
*	@date	18 July 2012
*	@version	1.0
*/
void nm_bsp_reset(void) 
{

	M2M_INFO("Reset Chip\n\r");
	nm_bsp_gpio_ctrl(ON,RESET);
	nm_bsp_sleep(100);
	nm_bsp_gpio_ctrl(OFF,RESET);
	nm_bsp_sleep(100);
	nm_bsp_gpio_ctrl(ON,RESET);
	nm_bsp_sleep(100);

}
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
void nm_bsp_timer_start(tpfNmBspTimerCb pfCb, uint32 u32PeriodTick) {

	gu32Jiffies = 0;
	gstrTimer.pfCb = pfCb;
	gstrTimer.u32Timeout = u32PeriodTick+ NM_BSP_TIME_MSEC;
	gstrTimer.u32Period = u32PeriodTick; 
	if(isr_ptr == NULL)
	{
		M2M_INFO("timer init\n\r");
		isr_ptr = _mem_alloc_zero((_mem_size)sizeof(MY_ISR_STRUCT));

		isr_ptr->TICK_COUNT   = 0;
		isr_ptr->OLD_ISR_DATA = _int_get_isr_data(BSP_TIMER_INTERRUPT_VECTOR);
		isr_ptr->OLD_ISR = _int_get_isr(BSP_TIMER_INTERRUPT_VECTOR);

		_int_install_isr(BSP_TIMER_INTERRUPT_VECTOR, tick_isr,isr_ptr);
	}
	else
	{
		gu32Jiffies = 0;
		isr_ptr->TICK_COUNT = 0; 
	}


}
/**
*	@fn		nm_bsp_gpio_ctrl(uint8 state,uint8 lednum)
*	@brief	led control
*	@param[IN]	state
*				1=ON  0=OFF
*	@param[IN]	lednum
*				from 1 to 4
*	@author	M.S.M
*	@date	18 July 2012
*	@version	1.0
*/
void nm_bsp_gpio_ctrl(uint8 state,uint8 PinNum)
{
	uint_32  set_value=0;
	static const uint_32 led1[] = {
		LED_1,
		GPIO_LIST_END
	};
	static const uint_32 led2[] = {
		LED_2,
		GPIO_LIST_END
	};
	static const uint_32 led3[] = {
		LED_3,
		GPIO_LIST_END
	};
	static const uint_32 led4[] = {
		LED_4,
		GPIO_LIST_END
	};
	static const uint_32 reset[] = {
		CHIP_RESET,
		GPIO_LIST_END
	};

	if (pFileLed) {
		set_value = (state) ? GPIO_IOCTL_WRITE_LOG1 : GPIO_IOCTL_WRITE_LOG0;
		switch (PinNum) {
			 case LED_CONNECT:
				 if (IO_OK !=ioctl(pFileLed, set_value, (pointer) &led1))
				 {
					 M2M_ERR("failed set led\n\r");
					 _mqx_exit(-1);
				 }
				 break;
			 case LED_DOWNLOAD:

				 if (IO_OK !=ioctl(pFileLed, set_value, (pointer) &led3))
				 {
					 M2M_ERR("failed set led\n\r");
					 _mqx_exit(-1);
				 }
				 break;
			 case LED_GROWL:

				 if (IO_OK !=ioctl(pFileLed, set_value, (pointer) &led4))
				 {
					 M2M_ERR("failed set led\n\r");
					 _mqx_exit(-1);
				 }
				 break;
			 case RESET:

				 if (IO_OK !=ioctl(pFileLed, set_value, (pointer) &reset))
				 {
					 M2M_ERR("failed set led\n\r");
					 _mqx_exit(-1);
				 }
				 break;
			 case LED_ERROR:
				 if (IO_OK !=ioctl(pFileLed, set_value, (pointer) &led2))
				 {
					 M2M_ERR("failed set led\n\r");
					 _mqx_exit(-1);
				 }
				 break;
			 default:
				 break;

		}
	}

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
/**
*	@fn		 nm_bsp_register_adc_cb
*	@brief	register adc callback
*	@param[IN]	pfCb
*				Callback function
*	@author	M.S.M
*	@date	08 August 2012
*	@version	1.0
*/
void nm_bsp_register_adc_cb(tpfNmBspAdcCb pfCb)
{
	gpfAdcCb = pfCb;	 
}
/**
*	@fn		nm_bsp_take_ADC_measurement
*	@brief	take adc measurement
*	@author	M.S.M
*	@date	08 August 2012
*	@version	1.0
*/
void nm_bsp_take_ADC_measurement(void)
{
  if(gpfAdcCb)
  {  
    gpfAdcCb(ReadADC(POT_CH),POT_CH);
    nm_bsp_sleep(10);
    gpfAdcCb(ReadADC(ACCX_CH),ACCX_CH);
  }
}
#endif
