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

typedef struct
{
	uint8_t signature[4];
	uint8_t executing_image;
	uint8_t downloaded_image;
	uint8_t writenew_image;
}Firmware_Status_t;

bool write_firmware=false;

struct usart_module usart_instance;

#define FIRMWARE_VERSION			 0x01
#define LED_0_PIN					 PIN_PA23
#define B1							 PIN_PB23
#define FW_STAT_ADDRESS			     0x7F00
// usart stuff
#define EDBG_CDC_MODULE              SERCOM4
#define EDBG_CDC_SERCOM_MUX_SETTING  USART_RX_3_TX_2_XCK_3
#define EDBG_CDC_SERCOM_PINMUX_PAD0  PINMUX_UNUSED
#define EDBG_CDC_SERCOM_PINMUX_PAD1  PINMUX_UNUSED
#define EDBG_CDC_SERCOM_PINMUX_PAD2  PINMUX_PB10D_SERCOM4_PAD2
#define EDBG_CDC_SERCOM_PINMUX_PAD3  PINMUX_PB11D_SERCOM4_PAD3
// spi flash stufff
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

// begin source code
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


static void configure_nvm()
{
	struct nvm_config config;
	nvm_get_config_defaults(&config);
	config.manual_page_write = false;
	nvm_set_config(&config);
}

static void writeFWStat(Firmware_Status_t thisFW)
{
	uint8_t page_buffer[NVMCTRL_PAGE_SIZE]={0};
	page_buffer[0] = thisFW.signature[0];
	page_buffer[1] = thisFW.signature[1];
	page_buffer[2] = thisFW.signature[2];
	page_buffer[3] = thisFW.signature[3];
	page_buffer[4] = thisFW.executing_image;
	page_buffer[5] = thisFW.downloaded_image;
	page_buffer[6] = thisFW.writenew_image;
	
	status_code_genare_t error_code;
	do
	{
		error_code = nvm_erase_row(FW_STAT_ADDRESS);			// Erase FW stat row
	} while (error_code == STATUS_BUSY);
	
	do
	{
		error_code = nvm_write_buffer(FW_STAT_ADDRESS, page_buffer, NVMCTRL_PAGE_SIZE);	// Write buffer to FW_STAT page
	} while (error_code == STATUS_BUSY);
}
void configure_port_pins(void)
{
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED_0_PIN, &config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_INPUT;
	port_pin_set_config(B1, &config_port_pin);
}

static void configure_spi_flash()
{
	struct at25dfx_chip_config at25dfx_chip_config;
	struct spi_config at25dfx_spi_config;
	spi_get_config_defaults(&at25dfx_spi_config);
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

static void download_firmware()
{
	at25dfx_chip_wake(&at25dfx_chip);

	if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK)
	{
		printf("Flash Chip did not respond. Download failed !\n");
		return;
	}
	// 		.......CALL HTTP DOWNLOADER................ 
	at25dfx_chip_set_global_sector_protect(&at25dfx_chip, false);				// unprotect sector
	at25dfx_chip_erase_block(&at25dfx_chip, 0x00000, AT25DFX_BLOCK_SIZE_4KB);	// erase block
	at25dfx_chip_write_buffer(&at25dfx_chip, 0x00000, write_buffer, AT25DFX_BUFFER_SIZE);	// write buffer
	at25dfx_chip_read_buffer(&at25dfx_chip, 0x00000, read_buffer, AT25DFX_BUFFER_SIZE);		// read same location
	at25dfx_chip_set_global_sector_protect(&at25dfx_chip, true);				// protect sector
	at25dfx_chip_sleep(&at25dfx_chip);
}

int main (void)
{
	system_init();
	//system_interrupt_enable_global();
	configure_port_pins();
	//delay_init();
	//configure_console();
	//configure_nvm();
	//configure_spi_flash();
	//printf("app started\n");
	while (1) 
	{
		if (port_pin_get_input_level(B1) == true) {
			port_pin_set_output_level(LED_0_PIN, false);
		}
		else 
		{
			port_pin_set_output_level(LED_0_PIN, true);
		}
		
		// receive command from IBM BlueMix
		//....................
		write_firmware = false; //set this to true
		//write the updated status
		if(write_firmware)
		{
			// download firmware into serial flash and upgrade
			download_firmware();
			Firmware_Status_t fw_status = *(Firmware_Status_t*)FW_STAT_ADDRESS;
			*(uint32_t*)fw_status.signature = 0xEFBEADDE; //replace with checksum of downloaded image
// 			fw_status.downloaded_image = fill version for downloaded image
// 			fw_status.writenew_image = 0xFF;  // write image flag
			writeFWStat(fw_status);
			// reset to begin writing firmware
			system_reset();
		}
	}	
}
