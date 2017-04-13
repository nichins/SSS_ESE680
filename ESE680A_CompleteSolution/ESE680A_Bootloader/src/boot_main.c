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
	uint8_t executing_image;	//firmware version for executing image
	uint8_t downloaded_image;	//firmware version for downloaded image 
	uint8_t writenew_image;		//boolean flag to upgrade the firmware
	uint8_t reset_count;		// Reset counter for app recovery
}Firmware_Status_t;

struct usart_module usart_instance;
extern unsigned char params[];

// usart stuff
#define EDBG_CDC_MODULE              SERCOM4
#define EDBG_CDC_SERCOM_MUX_SETTING  USART_RX_3_TX_2_XCK_3
#define EDBG_CDC_SERCOM_PINMUX_PAD0  PINMUX_UNUSED
#define EDBG_CDC_SERCOM_PINMUX_PAD1  PINMUX_UNUSED
#define EDBG_CDC_SERCOM_PINMUX_PAD2  PINMUX_PB10D_SERCOM4_PAD2
#define EDBG_CDC_SERCOM_PINMUX_PAD3  PINMUX_PB11D_SERCOM4_PAD3
#define APP_START_ADDRESS			 0x8000
#define FW_STAT_ADDRESS			     0x7F00
#define BOOT_PIN					PIN_PB23 //pin tied to button for stay in boot mode
// spi flash stufff
#define AT25DFX_BUFFER_SIZE				NVMCTRL_PAGE_SIZE
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
struct spi_module at25dfx_spi;
struct at25dfx_chip_module at25dfx_chip;

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

static Firmware_Status_t getFWStat() 
{
	return *(Firmware_Status_t*)FW_STAT_ADDRESS;	// return the firmware status
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
	page_buffer[7] = thisFW.reset_count;
	
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

static void upgradeFW(Firmware_Status_t thisFW)
{
	printf("Upgrading firmware to version: %d.\n", thisFW.downloaded_image);
	/*
	if (*(uint32_t*)thisFW.signature == 0x0)
	{
		printf("Invalid signature. Upgrade failed !\n");
		return;
	}
	*/
	// write new firmware
	//--------------------------
	uint32_t flash_fw_addr = thisFW.downloaded_image * 0x40000;			// Pick right start address in flash for downloaded image
	at25dfx_chip_wake(&at25dfx_chip); //wake up the chip
	if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK)
	{
		printf("Flash Chip did not respond. Upgrade failed !\n");
		return;
	}
	//...................
	//read the firmware out and start burning NVM
	uint32_t addr_i = 0;
	status_code_genare_t error_code;
	while (APP_START_ADDRESS + addr_i < 0x40000) {	
		// write a row to NVM (256B)
		do
		{
			error_code = nvm_erase_row(APP_START_ADDRESS + addr_i);			// Erase NVM row
		} while (error_code == STATUS_BUSY);
		for (int i = 0; i < 4; i++) {
			// Read 64B of flash
			uint32_t thisFlashAddr = flash_fw_addr + addr_i + i*NVMCTRL_PAGE_SIZE;
			at25dfx_chip_read_buffer(&at25dfx_chip, thisFlashAddr, read_buffer, NVMCTRL_PAGE_SIZE);
			uint32_t thisNVMAddr = APP_START_ADDRESS + addr_i + i*NVMCTRL_PAGE_SIZE;
			do
			{
				error_code = nvm_write_buffer(thisNVMAddr, read_buffer, NVMCTRL_PAGE_SIZE);	// Write 64B to NVM page
			} while (error_code == STATUS_BUSY);
		}
		
		// increment addr_i
		addr_i += 256;
		//printf("addr_i: %x\r\n", addr_i);
	}
	//at25dfx_chip_read_buffer(&at25dfx_chip, flash_fw_addr, read_buffer, AT25DFX_BUFFER_SIZE);	
	//computer CRC for the entire block and compare with the CRC stored by the app.
	
	//------------------------
	// enter low power mode.	
	at25dfx_chip_sleep(&at25dfx_chip);
	// writing done. Update the flags and reset
	thisFW.executing_image = thisFW.downloaded_image;
	thisFW.writenew_image = 0;
	writeFWStat(thisFW);
	printf("Upgrade complete. Resetting device.\n");
	system_reset();
}

static void writeFWtoFlash(uint8_t slot)
{
	printf("Writing NVM to flash section: %d.\n", slot);
	//--------------------------
	uint32_t flash_fw_addr = slot * 0x40000;			// Pick right start address in flash for downloaded image
	at25dfx_chip_wake(&at25dfx_chip); //wake up the chip
	if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK)
	{
		printf("Flash Chip did not respond. Upgrade failed !\n");
		return;
	}
	//...................
	//read the firmware out and start burning NVM
	uint32_t addr_i = 0;
	status_code_genare_t error_code;
	at25dfx_chip_set_sector_protect(&at25dfx_chip, flash_fw_addr, false);				// unprotect sector
	at25dfx_chip_set_sector_protect(&at25dfx_chip, flash_fw_addr + 0x10000, false);				// unprotect sector
	at25dfx_chip_set_sector_protect(&at25dfx_chip, flash_fw_addr + 0x20000, false);				// unprotect sector
	at25dfx_chip_set_sector_protect(&at25dfx_chip, flash_fw_addr + 0x30000, false);				// unprotect sector
	at25dfx_chip_erase_block(&at25dfx_chip, flash_fw_addr, AT25DFX_BLOCK_SIZE_64KB);				// erase block 0
	at25dfx_chip_erase_block(&at25dfx_chip, flash_fw_addr + 0x10000, AT25DFX_BLOCK_SIZE_64KB);	// erase block 1
	at25dfx_chip_erase_block(&at25dfx_chip, flash_fw_addr + 0x20000, AT25DFX_BLOCK_SIZE_64KB);	// erase block 2
	at25dfx_chip_erase_block(&at25dfx_chip, flash_fw_addr + 0x30000, AT25DFX_BLOCK_SIZE_64KB);	// erase block 3
	while (addr_i < 0x38000) {
		// Read a page of NVM (64B)
		do
		{
			//error_code = nvm_read_buffer(APP_START_ADDRESS + addr_i, &read_buffer, AT25DFX_BUFFER_SIZE);	
			error_code = nvm_read_buffer(APP_START_ADDRESS + addr_i, read_buffer, NVMCTRL_PAGE_SIZE);
		} while (error_code == STATUS_BUSY);
		// Write 64B to flash

		at25dfx_chip_write_buffer(&at25dfx_chip, flash_fw_addr + addr_i, read_buffer, NVMCTRL_PAGE_SIZE);
		at25dfx_chip_read_buffer(&at25dfx_chip, flash_fw_addr + addr_i, read_buffer, NVMCTRL_PAGE_SIZE);
		
		
		// increment addr_i
		addr_i += 64;
		printf("addr_i: %x\r\n", addr_i);
	}
	//computer CRC for the entire block and compare with the CRC stored by the app.
	
	//------------------------
	at25dfx_chip_set_sector_protect(&at25dfx_chip, flash_fw_addr, true);				// unprotect sector
	at25dfx_chip_set_sector_protect(&at25dfx_chip, flash_fw_addr + 0x10000, true);				// unprotect sector
	at25dfx_chip_set_sector_protect(&at25dfx_chip, flash_fw_addr + 0x20000, true);				// unprotect sector
	at25dfx_chip_set_sector_protect(&at25dfx_chip, flash_fw_addr + 0x30000, true);				// unprotect sector
	// enter low power mode.
	at25dfx_chip_sleep(&at25dfx_chip);
	// writing done. Update the flags and reset
	//printf("Flash write complete.\n");
	//system_reset();
}

static void configure_nvm() 
{
	struct nvm_config config;
	nvm_get_config_defaults(&config);
	config.manual_page_write = false;
	nvm_set_config(&config);
}
static void configure_boot_button()
{
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);
	pin_conf.direction = PORT_PIN_DIR_INPUT;
	port_pin_set_config(BOOT_PIN, &pin_conf);
}

static void init_drivers()
{
	delay_init();
	configure_console();
	configure_nvm();
	configure_spi_flash();	
}
int main (void)
{
	system_init();
	system_interrupt_enable_global();
	configure_boot_button();
	// default boot params. Needed so that linker doesn't optimize it out.
	uint8_t* t = params; 
	
	// Test - write NVM app space to flash
	//init_drivers();
	//writeFWtoFlash(0);
	

	void (*app_code_entry)(void);
	uint16_t n=0; uint8_t remain_in_boot = 0;
	//check if button is pressed to lock in boot
	while(n++ < 1000)
	{
		/*if(!port_pin_get_input_level(BOOT_PIN)) // KILLED THIS SO WE CAN USE THE BUTTON IN APP TESTING FOR NOW ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		{
			remain_in_boot = 1;
			init_drivers();
			break;
		}*/
	}
	while(1) 
	{
		if(!remain_in_boot)
		{
			// check for firmware download requested
			Firmware_Status_t thisFW = getFWStat();
			//thisFW.downloaded_image = 0;
			thisFW.writenew_image = 0;		// Override writenew_image cause we're testing goddamnit
			if(thisFW.writenew_image)
			{
				init_drivers();
				upgradeFW(thisFW);
			}
			
			// vector table rebasing
			SCB->VTOR = ((uint32_t) APP_START_ADDRESS & SCB_VTOR_TBLOFF_Msk);

			// jump to reset handler
			app_code_entry =  (void(*)(void))(*(uint32_t*)(APP_START_ADDRESS+4));
			if (app_code_entry == 0x0) {		// Empty app space
				init_drivers();
				thisFW.downloaded_image = 0;
				thisFW.writenew_image = 1;
				upgradeFW(thisFW);
			}
			if (system_get_reset_cause() == SYSTEM_RESET_CAUSE_WDT) {
				thisFW.reset_count += 1;
				writeFWStat(thisFW);
				if (thisFW.reset_count > 10) {
					thisFW.reset_count = 0;
					init_drivers();
					thisFW.downloaded_image = 0;
					thisFW.writenew_image = 1;
					upgradeFW(thisFW);
				} 
			}
			app_code_entry();
		}
		
		printf("in boot\n\r");
		delay_ms(500);
	}
}
