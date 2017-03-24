/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to system_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include <stdint.h>
typedef struct  
{
	uint8_t signature[4];
	uint8_t executing_image;
	uint8_t downloaded_image;
	uint8_t writenew_image:1;
}Firmware_Status_t;

static Firmware_Status_t FM_Status __attribute__ ((section (".status")));
struct usart_module usart_instance;

#define MAX_RX_BUFFER_LENGTH   5
#define EDBG_CDC_MODULE              SERCOM4
#define EDBG_CDC_SERCOM_MUX_SETTING  USART_RX_3_TX_2_XCK_3
#define EDBG_CDC_SERCOM_PINMUX_PAD0  PINMUX_UNUSED
#define EDBG_CDC_SERCOM_PINMUX_PAD1  PINMUX_UNUSED
#define EDBG_CDC_SERCOM_PINMUX_PAD2  PINMUX_PB10D_SERCOM4_PAD2
#define EDBG_CDC_SERCOM_PINMUX_PAD3  PINMUX_PB11D_SERCOM4_PAD3
#define APP_START_ADDRESS			0x4000
#define BOOT_PIN					PIN_PA04 //pin tied to button for stay in boot mode
static void configure_console(void)
{
	struct usart_config usart_conf;

	usart_get_config_defaults(&usart_conf);
	usart_conf.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	usart_conf.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	usart_conf.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	usart_conf.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	usart_conf.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	usart_conf.baudrate    = 115200;

	stdio_serial_init(&usart_instance, EDBG_CDC_MODULE, &usart_conf);
	usart_enable(&usart_instance);
}

// download firmware and write it to the spi flash
static void download_firmware()
{

	
}
int main (void)
{
	
	system_init();
	system_interrupt_enable_global();
	delay_init();
	configure_console();
	// boot pin config
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	 port_pin_set_config(BOOT_PIN, &pin_conf);
	printf("Init done.\n");
	/* Insert  code here, after the board has been initialized. */
	//handle writing to the flags
	FM_Status.executing_image = 1;
	FM_Status.downloaded_image = 2;

	void (*app_code_entry)(void);
	while(1)
	{
		uint16_t n=0; uint8_t remain_in_boot = 0;
		//check if button is pressed to lock in boot
		while(n++ < 1000)
		{
			if(!port_pin_get_input_level(BOOT_PIN)) 
			{
				remain_in_boot =1;
				break;
			}
		}
		
		if(!remain_in_boot)
		{
			// check for firmware download requested
			if(FM_Status.writenew_image)
			{
				download_firmware();
			}
			// vector table rebasing
			SCB->VTOR = ((uint32_t) APP_START_ADDRESS & SCB_VTOR_TBLOFF_Msk);

			// jump to reset handler
			app_code_entry =  (void(*)(void))(*(unsigned int*)(APP_START_ADDRESS+4));
			// jump
			printf("starting app\n");
			app_code_entry();
		}

	}
}
