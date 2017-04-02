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

struct usart_module usart_instance;

#define MAX_RX_BUFFER_LENGTH   5
#define EDBG_CDC_MODULE              SERCOM3//SERCOM4
#define EDBG_CDC_SERCOM_MUX_SETTING  USART_RX_3_TX_0_XCK_1//USART_RX_3_TX_2_XCK_3
#define EDBG_CDC_SERCOM_PINMUX_PAD0  PINMUX_UNUSED
#define EDBG_CDC_SERCOM_PINMUX_PAD1  PINMUX_UNUSED
#define EDBG_CDC_SERCOM_PINMUX_PAD2  PINMUX_PA22C_SERCOM3_PAD0//PINMUX_PB10D_SERCOM4_PAD2	// TX
#define EDBG_CDC_SERCOM_PINMUX_PAD3  PINMUX_PA21D_SERCOM3_PAD3//PINMUX_PB11D_SERCOM4_PAD3	// RX
#define APP_START_ADDRESS			0x8000
#define FW_STAT						0x7F00
#define BOOT_PIN					PIN_PA06 //pin tied to button for stay in boot mode
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
	// write new firmware
	
	//...................
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

#define AT25DFX_BUFFER_SIZE  (10)
#define AT25DFX_CLOCK_SPEED				120000
#define AT25DFX_SPI_PINMUX_SETTING		SPI_SIGNAL_MUX_SETTING_E
#define AT25DFX_SPI_PINMUX_PAD0			PINMUX_PA16C_SERCOM1_PAD0
#define AT25DFX_SPI_PINMUX_PAD1			PINMUX_PA17C_SERCOM1_PAD1
#define AT25DFX_SPI_PINMUX_PAD2			PINMUX_PA18C_SERCOM1_PAD2
#define AT25DFX_SPI_PINMUX_PAD3			PINMUX_PA19C_SERCOM1_PAD3
#define AT25DFX_SPI						SERCOM1
#define AT25DFX_CS						PIN_PA07
#define AT25DFX_MEM_TYPE				AT25DFX_081A
static uint8_t read_buffer[AT25DFX_BUFFER_SIZE];
static uint8_t write_buffer[AT25DFX_BUFFER_SIZE] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
struct spi_module at25dfx_spi;
struct at25dfx_chip_module at25dfx_chip;

static void at25dfx_init(void)
{
	struct at25dfx_chip_config at25dfx_chip_config;
	struct spi_config at25dfx_spi_config;
	at25dfx_spi_get_config_defaults(&at25dfx_spi_config);
	at25dfx_spi_config.mode_specific.master.baudrate = AT25DFX_CLOCK_SPEED;
	at25dfx_spi_config.mux_setting = AT25DFX_SPI_PINMUX_SETTING;
	at25dfx_spi_config.pinmux_pad0 = AT25DFX_SPI_PINMUX_PAD0;
	at25dfx_spi_config.pinmux_pad1 = AT25DFX_SPI_PINMUX_PAD1;
	at25dfx_spi_config.pinmux_pad2 = AT25DFX_SPI_PINMUX_PAD2;
	at25dfx_spi_config.pinmux_pad3 = AT25DFX_SPI_PINMUX_PAD3;
	spi_init(&at25dfx_spi, AT25DFX_SPI, &at25dfx_spi_config);
	spi_enable(&at25dfx_spi);
	
	at25dfx_chip_config.type = AT25DFX_MEM_TYPE;
	at25dfx_chip_config.cs_pin = AT25DFX_CS;
	at25dfx_chip_init(&at25dfx_chip, &at25dfx_spi, &at25dfx_chip_config);
}

int main (void)
{
	
	system_init();
	system_interrupt_enable_global();
	delay_init();
	configure_console();
	configure_nvm();
	
	at25dfx_init();
	/* Insert application code here, after the board has been initialized. */
	at25dfx_chip_wake(&at25dfx_chip);
	
	if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK) {
		// Handle missing or non-responsive device
	}
	at25dfx_chip_set_sector_protect(&at25dfx_chip, 0x10000, false);				// unprotect sector
	at25dfx_chip_erase_block(&at25dfx_chip, 0x10000, AT25DFX_BLOCK_SIZE_4KB);	// erase block
	at25dfx_chip_write_buffer(&at25dfx_chip, 0x10000, write_buffer, AT25DFX_BUFFER_SIZE);	// write buffer
	at25dfx_chip_read_buffer(&at25dfx_chip, 0x10000, read_buffer, AT25DFX_BUFFER_SIZE);		// read same location
	at25dfx_chip_set_global_sector_protect(&at25dfx_chip, true);				// protect sector
	at25dfx_chip_sleep(&at25dfx_chip);											// back to sleep
	
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
	uint16_t n=0; uint8_t remain_in_boot = 0;
	//check if button is pressed to lock in boot
	while(n++ < 1000)
	{
		if(!port_pin_get_input_level(BOOT_PIN))
		{
			remain_in_boot = 1;
			break;
		}
	}
	while(1) {
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
		printf("in boot");
		delay_ms(500);
	}
}
