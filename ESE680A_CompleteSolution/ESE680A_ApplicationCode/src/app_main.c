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
#include <errno.h>
#include "main.h"
#include "stdio_serial.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"
#include "iot/http/http_client.h"

typedef struct
{
	uint8_t signature[4];
	uint8_t executing_image;
	uint8_t downloaded_image;
	uint8_t writenew_image;
	uint8_t reset_count;		// Reset counter for app recovery
}Firmware_Status_t;

bool write_firmware=false;
crc32_t crcChecker;		// CRC object for calculating CRC
bool firstCRC = true;	// Flag for first CRC calc
bool download_CRC = false; //Are we downloading CRC?
uint32_t dlCRC;			// Downloaded CRC

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
#define AT25DFX_BUFFER_SIZE  (256)
#define AT25DFX_CLOCK_SPEED				120000
#define AT25DFX_SPI_PINMUX_SETTING		SPI_SIGNAL_MUX_SETTING_E
#define AT25DFX_SPI_PINMUX_PAD0			PINMUX_PA16C_SERCOM1_PAD0
#define AT25DFX_SPI_PINMUX_PAD1			PINMUX_UNUSED//PINMUX_PA17C_SERCOM1_PAD1
#define AT25DFX_SPI_PINMUX_PAD2			PINMUX_PA18C_SERCOM1_PAD2
#define AT25DFX_SPI_PINMUX_PAD3			PINMUX_PA19C_SERCOM1_PAD3
#define AT25DFX_SPI						SERCOM1
#define AT25DFX_CS						PIN_PA07
#define AT25DFX_MEM_TYPE				AT25DFX_081A
static uint8_t read_buffer[AT25DFX_BUFFER_SIZE];
//static uint8_t write_buffer[AT25DFX_BUFFER_SIZE] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
struct spi_module at25dfx_spi;
struct at25dfx_chip_module at25dfx_chip;

//http Downloading stufff
uint32_t flash_addr;
uint8_t http_buf [2048];
uint32_t http_buf_write_ptr = 0x00000;
uint32_t http_buf_read_ptr = 0x00000;

static download_state down_state = NOT_READY;
static struct usart_module cdc_uart_module;
struct sw_timer_module swt_module_inst;
struct http_client_module http_client_module_inst;


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

static Firmware_Status_t getFWStat()
{
	status_code_genare_t error_code;
	uint8_t read_buffer[NVMCTRL_PAGE_SIZE]={0};
	do
	{
		error_code = nvm_read_buffer(FW_STAT_ADDRESS, read_buffer, NVMCTRL_PAGE_SIZE);	// Write buffer to FW_STAT page
	} while (error_code == STATUS_BUSY);
	Firmware_Status_t thisFW;
	thisFW.signature[0]			= read_buffer[0];
	thisFW.signature[1]			= read_buffer[1];
	thisFW.signature[2]			= read_buffer[2];
	thisFW.signature[3]			= read_buffer[3];
	thisFW.executing_image		= read_buffer[4];
	thisFW.downloaded_image		= read_buffer[5];
	thisFW.writenew_image		= read_buffer[6];
	thisFW.reset_count			= read_buffer[7];
	return thisFW;
	//return *(Firmware_Status_t*)FW_STAT_ADDRESS;	// return the firmware status
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
	page_buffer[7] = 0;
	
	status_code_genare_t error_code;
	do
	{
		error_code = nvm_erase_row(FW_STAT_ADDRESS);			// Erase FW stat row
	} while (error_code == STATUS_BUSY);
	
	do
	{
		error_code = nvm_write_buffer(FW_STAT_ADDRESS, page_buffer, NVMCTRL_PAGE_SIZE);	// Write buffer to FW_STAT page
	} while (error_code == STATUS_BUSY);
	
	/* test - read back NVM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	//uint8_t read_buffer[NVMCTRL_PAGE_SIZE]={0};
	//do
	//{
		//error_code = nvm_read_buffer(FW_STAT_ADDRESS, read_buffer, NVMCTRL_PAGE_SIZE);	
	//} while (error_code == STATUS_BUSY);
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

//Http downloader source code

static void configure_timer(void)
{
	struct sw_timer_config swt_conf;
	sw_timer_get_config_defaults(&swt_conf);

	sw_timer_init(&swt_module_inst, &swt_conf);
	sw_timer_enable(&swt_module_inst);
}

static void init_state(void)
{
	down_state = NOT_READY;
}

static void clear_state(download_state mask)
{
	down_state &= ~mask;
}

static void add_state(download_state mask)
{
	down_state |= mask;
}

static inline bool is_state_set(download_state mask)
{
	return ((down_state & mask) != 0);
}

void write_spi_flash_frm_buf(uint32 len){
	at25dfx_chip_wake(&at25dfx_chip);
	if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK) {
		// Handle missing or non-responsive device
		printf("Chip didnt wake \r\n");
	}
	
	//at25dfx_chip_set_sector_protect(&at25dfx_chip, flash_addr, false);				// unprotect sector
	
	at25dfx_chip_write_buffer(&at25dfx_chip, flash_addr, http_buf + http_buf_read_ptr, len);	// write buffer
	//at25dfx_chip_read_buffer(&at25dfx_chip, flash_addr, read_buffer, len);		// read same location
	//at25dfx_chip_read_buffer(&at25dfx_chip, (flash_addr+0x0020), read_buffer, AT25DFX_BUFFER_SIZE);		// read same location
	//at25dfx_chip_set_global_sector_protect(&at25dfx_chip, true);				// protect sector
	at25dfx_chip_sleep(&at25dfx_chip);											// back to sleep
	flash_addr = flash_addr + len;
}

static void start_download(void)
{
	/*
	if (!is_state_set(STORAGE_READY)) {
		printf("start_download: Flash not initialized.\r\n");
		return;
	}
	*/
	if (!is_state_set(WIFI_CONNECTED)) {
		printf("start_download: Wi-Fi is not connected.\r\n");
		return;
	}

	if (is_state_set(GET_REQUESTED)) {
		printf("start_download: request is sent already.\r\n");
		return;
	}

	if (is_state_set(DOWNLOADING)) {
		printf("start_download: running download already.\r\n");
		return;
	}

	/* Send the HTTP request. */
	if(download_CRC == false){
		printf("start_download: sending HTTP request...\r\n");
		http_client_send_request(&http_client_module_inst, MAIN_HTTP_FILE_URL, HTTP_METHOD_GET, NULL, NULL);
	} else{
		printf("start_download CRC: sending HTTP request...\r\n");
		http_client_send_request(&http_client_module_inst, MAIN_HTTP_CRC_URL, HTTP_METHOD_GET, NULL, NULL);
	}
	
}

static void http_client_callback(struct http_client_module *module_inst, int type, union http_client_data *data)
{
	switch (type) {
	case HTTP_CLIENT_CALLBACK_SOCK_CONNECTED:
		printf("http_client_callback: HTTP client socket connected.\r\n");
		break;

	case HTTP_CLIENT_CALLBACK_REQUESTED:
		printf("http_client_callback: request completed.\r\n");
		add_state(GET_REQUESTED);
		break;

	case HTTP_CLIENT_CALLBACK_RECV_RESPONSE:
		printf("http_client_callback: received response %u data size %u\r\n",
				(unsigned int)data->recv_response.response_code,
				(unsigned int)data->recv_response.content_length);
		if ((unsigned int)data->recv_response.response_code == 200) {
		} 
		else {
			add_state(CANCELED);
			return;
		}
		if (data->recv_response.content_length <= MAIN_BUFFER_MAX_SIZE) {
			//***store_file_packet(data->recv_response.content, data->recv_response.content_length);
			
			//This is run only when file size < MAIN_BUFFER_MAX_SIZE which we assume never happens!
			printf("Callback: CRC download......\r\n");
			dlCRC = *(uint32_t *)data->recv_response.content;
			dlCRC =  ((dlCRC>>24)&0xff) | // move byte 3 to byte 0
                    ((dlCRC<<8)&0xff0000) | // move byte 1 to byte 2
                    ((dlCRC>>8)&0xff00) | // move byte 2 to byte 1
                    ((dlCRC<<24)&0xff000000); // byte 0 to byte 3
			printf("Received %x\r\n", (uint32_t)dlCRC);
			
			add_state(COMPLETED);
		}
		break;

	case HTTP_CLIENT_CALLBACK_RECV_CHUNKED_DATA:
		printf("http_client_callback_CHUNKED DATA: received response data size %u\r\n",
				(unsigned int)data->recv_chunked_data.length);
		// Calc CRC for this chunk
		if (firstCRC) {
			crc32_calculate(data->recv_chunked_data.data, (unsigned int)data->recv_chunked_data.length, &crcChecker);
			printf("First block length %d CRC: %u\r\n", (unsigned int)data->recv_chunked_data.length, crcChecker);
			firstCRC = false;
		}
		else {
			crc32_recalculate(data->recv_chunked_data.data, (unsigned int)data->recv_chunked_data.length, &crcChecker);
			printf("Block length %d CRC: %u\r\n", (unsigned int)data->recv_chunked_data.length, crcChecker);
		}
		
		//***store_file_packet(data->recv_chunked_data.data, data->recv_chunked_data.length);
		if (http_buf_write_ptr + data->recv_chunked_data.length > 2048){
			memcpy_ram2ram(http_buf + http_buf_write_ptr,data->recv_chunked_data.data,(2048-http_buf_write_ptr));
			memcpy_ram2ram(http_buf, data->recv_chunked_data.data + (2048-http_buf_write_ptr), data->recv_chunked_data.length-(2048-http_buf_write_ptr));
			http_buf_write_ptr = data->recv_chunked_data.length-(2048-http_buf_write_ptr);
		}
		else {
			memcpy_ram2ram(http_buf + http_buf_write_ptr, data->recv_chunked_data.data, data->recv_chunked_data.length);
			http_buf_write_ptr = http_buf_write_ptr + data->recv_chunked_data.length;
		}
		 
		if  (http_buf_write_ptr > http_buf_read_ptr){
			uint8 n = (http_buf_write_ptr-http_buf_read_ptr) / 256;
			for (int i=0 ; i<n ; i++ ){
				write_spi_flash_frm_buf(256);
				http_buf_read_ptr = http_buf_read_ptr + 256;
			}
		}
		else if (http_buf_write_ptr < http_buf_read_ptr){
			uint8 n = (2048 - http_buf_read_ptr) / 256;
			for (int i=0 ; i<n ; i++ ){
				write_spi_flash_frm_buf(256);
				http_buf_read_ptr = http_buf_read_ptr + 256;
			}
			http_buf_read_ptr = 0;
			n = (http_buf_write_ptr-http_buf_read_ptr) / 256;
			for (int i=0 ; i<n ; i++ ){
				write_spi_flash_frm_buf(256);
				http_buf_read_ptr = http_buf_read_ptr + 256;
			}
		}
		
		
		if (data->recv_chunked_data.is_complete) {
			add_state(COMPLETED);
			if  (http_buf_write_ptr < http_buf_read_ptr){
				http_buf_read_ptr =0;
				write_spi_flash_frm_buf(http_buf_write_ptr-http_buf_read_ptr);
			}
			else if(http_buf_write_ptr > http_buf_read_ptr){
				write_spi_flash_frm_buf(http_buf_write_ptr-http_buf_read_ptr);
			}
		}
		break;

	case HTTP_CLIENT_CALLBACK_DISCONNECTED:
		printf("http_client_callback: disconnection reason:%d\r\n", data->disconnected.reason);

		/* If disconnect reason is equal to -ECONNRESET(-104),
		 * It means the server has closed the connection (timeout).
		 * This is normal operation.
		 */
		if (data->disconnected.reason == -EAGAIN) {
			/* Server has not responded. Retry immediately. */
			if (is_state_set(DOWNLOADING)) {
				//f_close(&file_object);
				clear_state(DOWNLOADING);
			}

			if (is_state_set(GET_REQUESTED)) {
				clear_state(GET_REQUESTED);
			}

			start_download();
		}

		break;
	}
}

static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	http_client_socket_event_handler(sock, u8Msg, pvMsg);
}

static void resolve_cb(uint8_t *pu8DomainName, uint32_t u32ServerIP)
{
	printf("resolve_cb: %s IP address is %d.%d.%d.%d\r\n\r\n", pu8DomainName,
	(int)IPV4_BYTE(u32ServerIP, 0), (int)IPV4_BYTE(u32ServerIP, 1),
	(int)IPV4_BYTE(u32ServerIP, 2), (int)IPV4_BYTE(u32ServerIP, 3));
	http_client_socket_resolve_handler(pu8DomainName, u32ServerIP);
}

static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
		case M2M_WIFI_RESP_CON_STATE_CHANGED:
		{
			tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
			if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
				printf("wifi_cb: M2M_WIFI_CONNECTED\r\n");
				m2m_wifi_request_dhcp_client();
				} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
				printf("wifi_cb: M2M_WIFI_DISCONNECTED\r\n");
				clear_state(WIFI_CONNECTED);
				if (is_state_set(DOWNLOADING)) {
					clear_state(DOWNLOADING);
				}

				if (is_state_set(GET_REQUESTED)) {
					clear_state(GET_REQUESTED);
				}

				m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
				MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
			}

			break;
		}

		case M2M_WIFI_REQ_DHCP_CONF:
		{
			uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
			printf("wifi_cb: IP address is %u.%u.%u.%u\r\n",
			pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
			add_state(WIFI_CONNECTED);
			start_download();

		}

		default:
		break;
	}
}

static void configure_http_client(void)
{
	struct http_client_config httpc_conf;
	int ret;

	http_client_get_config_defaults(&httpc_conf);

	httpc_conf.recv_buffer_size = MAIN_BUFFER_MAX_SIZE;
	httpc_conf.timer_inst = &swt_module_inst;

	ret = http_client_init(&http_client_module_inst, &httpc_conf);
	if (ret < 0) {
		printf("configure_http_client: HTTP client initialization failed! (res %d)\r\n", ret);
		while (1) {
			} // Loop forever
		}

		http_client_register_callback(&http_client_module_inst, http_client_callback);
}

static void download_firmware(unsigned int slot)
{
	flash_addr = 0x40000 * slot; //Starting addr on flash where downloaded file is stored
	at25dfx_chip_wake(&at25dfx_chip);
	if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK) {
		// Handle missing or non-responsive device
	}
	
	at25dfx_chip_set_global_sector_protect(&at25dfx_chip, false);
	at25dfx_chip_erase_block(&at25dfx_chip, flash_addr, AT25DFX_BLOCK_SIZE_64KB);	// erase block
	at25dfx_chip_erase_block(&at25dfx_chip, flash_addr + 0x10000, AT25DFX_BLOCK_SIZE_64KB);
	at25dfx_chip_erase_block(&at25dfx_chip, flash_addr + 0x20000, AT25DFX_BLOCK_SIZE_64KB);
	at25dfx_chip_erase_block(&at25dfx_chip, flash_addr + 0x30000, AT25DFX_BLOCK_SIZE_64KB);
	
	/* Connect to router and download stuff and store it in flash */
	printf("download_firmware: connecting to WiFi AP %s...\r\n", (char *)MAIN_WLAN_SSID);
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
	while (!(is_state_set(COMPLETED) || is_state_set(CANCELED))) {
		m2m_wifi_handle_events(NULL);
		sw_timer_task(&swt_module_inst);
	}
	printf("download_firmware: done.\r\n");
	printf("Calculated CRC: %x\r\n", (uint32_t)crcChecker);
	
	//clear_state(COMPLETED|DOWNLOADING|GET_REQUESTED|CANCELED);
	m2m_wifi_disconnect();
	download_CRC = true;
	init_state();
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
	while (!(is_state_set(COMPLETED) || is_state_set(CANCELED))) {
		m2m_wifi_handle_events(NULL);
		sw_timer_task(&swt_module_inst);
	}
	
	download_CRC = false;
	//For debugging this shit
	//flash_addr = 0x00000;

}


int main (void)
{
	
	tstrWifiInitParam param;
	int8_t ret;
	
	
	system_init();
	init_state();
	//system_interrupt_enable_global();
	configure_port_pins();
	//delay_init();
	configure_console();
	configure_nvm();
	configure_spi_flash();
	configure_timer();
	configure_http_client();
	nm_bsp_init();
	
	
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));
	
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error! (res %d)\r\n", ret);
		while (1) {
		}
	}
	
	socketInit();
	registerSocketCallback(socket_cb, resolve_cb);
	write_firmware = false; 
	while (1) 
	{
		if (port_pin_get_input_level(B1) == true) {
			port_pin_set_output_level(LED_0_PIN, false);
		}
		else 
		{
			port_pin_set_output_level(LED_0_PIN, true);
			write_firmware = true;
		}
		
		// receive command from IBM BlueMix
		//....................
		//write the updated status
		if(write_firmware)
		{
			// download firmware into serial flash and upgrade
			Firmware_Status_t fw_status = getFWStat();
			if (fw_status.executing_image == 1) {
				fw_status.downloaded_image = 2;
			}
			else {
				fw_status.downloaded_image = 1;
			}
			printf("Executing image: %d, DL to: %d\r\n", fw_status.executing_image, fw_status.downloaded_image);
			firstCRC=true;
			download_firmware(fw_status.downloaded_image);
			printf("\n\r Main: Done downloading firmware and CRC\n\r");
			
			if (dlCRC == crcChecker){
				printf("\n\r Main: CRC MATCHED!\n\r");
			} else {
				printf("\n\r Main: CRC Check Fail!\n\r");
			}
				 
			//printf("fw_status.writenew_image = %d\n\r before mod\n\r", fw_status.writenew_image);
			*(uint32_t*)fw_status.signature = 0xEFBEADDE; //replace with checksum of downloaded image
 			fw_status.writenew_image = 1;  // write image flag
			//printf("fw_status.writenew_image = %d\n\r after mod before write\n\r", fw_status.writenew_image);
			writeFWStat(fw_status);
			// reset to begin writing firmware
			system_reset();
		}
	}	
}
