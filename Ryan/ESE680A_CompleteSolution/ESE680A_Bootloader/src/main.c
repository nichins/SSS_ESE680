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
	uint8_t writenew_image;
}Firmware_Status_t;

//static Firmware_Status_t FM_Status __attribute__ ((section (".status")));		// this didn't let us write to the firmware status
static Firmware_Status_t FM_Status;
struct usart_module usart_instance;

#define MAX_RX_BUFFER_LENGTH   5
#define EDBG_CDC_MODULE              SERCOM4
#define EDBG_CDC_SERCOM_MUX_SETTING  USART_RX_3_TX_2_XCK_3
#define EDBG_CDC_SERCOM_PINMUX_PAD0  PINMUX_UNUSED
#define EDBG_CDC_SERCOM_PINMUX_PAD1  PINMUX_UNUSED
#define EDBG_CDC_SERCOM_PINMUX_PAD2  PINMUX_PB10D_SERCOM4_PAD2
#define EDBG_CDC_SERCOM_PINMUX_PAD3  PINMUX_PB11D_SERCOM4_PAD3
#define APP_START_ADDRESS			0x8000
#define FW_STAT						0x7F00
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


static Firmware_Status_t getFWStat() {
	Firmware_Status_t *fm_nvm = (unsigned int*)FW_STAT;			// Pointer to FW_STAT
	Firmware_Status_t thisFW = *fm_nvm;				// Read contents of FW_STAT
	return thisFW;
}

static void writeFWStat(Firmware_Status_t thisFW) {
	uint8_t page_buffer[NVMCTRL_PAGE_SIZE];
	page_buffer[0] = thisFW.signature[0];
	page_buffer[1] = thisFW.signature[1];
	page_buffer[2] = thisFW.signature[2];
	page_buffer[3] = thisFW.signature[3];
	page_buffer[4] = thisFW.executing_image;
	page_buffer[5] = thisFW.downloaded_image;
	page_buffer[6] = thisFW.writenew_image;
	
	enum status_code error_code;
	do
	{
		error_code = nvm_erase_row(FW_STAT);			// Erase FW stat row
	} while (error_code == STATUS_BUSY);
	
	do 
	{ 
		error_code = nvm_write_buffer(FW_STAT, page_buffer, NVMCTRL_PAGE_SIZE);	// Write buffer to FW_STAT page
	} while (error_code == STATUS_BUSY);
}

static void upgradeFW(Firmware_Status_t thisFW){
	printf("Upgrading firmware from location %d.\n", thisFW.downloaded_image);
	// upgrade firmware
	thisFW.executing_image = thisFW.downloaded_image;
	thisFW.writenew_image = 0;
	writeFWStat(thisFW);
	printf("Upgrade complete\n");
}

static void configure_nvm() {
	struct nvm_config config;
	nvm_get_config_defaults(&config);
	config.manual_page_write = false;
	nvm_set_config(&config);
}

int main (void)
{
	
	system_init();
	system_interrupt_enable_global();
	delay_init();
	configure_console();
	configure_nvm();
	// boot pin config
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);
	pin_conf.direction = PORT_PIN_DIR_INPUT;
	 port_pin_set_config(BOOT_PIN, &pin_conf);
	printf("Init done.\n");

	
	
	
	Firmware_Status_t thisFW = getFWStat();
	if (thisFW.signature[0] == NULL) {
		printf("Invalid FW stat, writing default\n");
		thisFW.signature[0] = 1;
		thisFW.signature[1] = 2;
		thisFW.signature[2] = 3;
		thisFW.signature[3] = 4;
		thisFW.executing_image = 1;
		thisFW.downloaded_image = 2;
		thisFW.writenew_image = 0;
		writeFWStat(thisFW);
	}
	
	//thisFW.writenew_image = 1;
	//thisFW.downloaded_image = 1;
	//writeFWStat(thisFW);

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
			if(thisFW.writenew_image)
			{
				upgradeFW(thisFW);
			}
			
			// vector table rebasing
			SCB->VTOR = ((uint32_t) APP_START_ADDRESS & SCB_VTOR_TBLOFF_Msk);

			// jump to reset handler
			app_code_entry =  (void(*)(void))(*(unsigned int*)(APP_START_ADDRESS+4));
			// jump
			printf("Starting app\n");
			app_code_entry();
		}

	}
}
