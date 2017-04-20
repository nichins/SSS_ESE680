#include "asf.h"
#include "main.h"
#include <errno.h>
#include "stdio_serial.h"
#include "driver/include/m2m_wifi.h"
#include "iot/http/http_client.h"
#include "iot/mqtt/mqtt.h"
#include "iot/sw_timer.h"
#include "socket/include/socket.h"

typedef struct
{
	uint8_t signature[4];
	uint8_t executing_image;
	uint8_t downloaded_image;
	uint8_t writenew_image;
	uint8_t reset_count;		// Reset counter for app recovery
} Firmware_Status_t;

bool write_firmware = false;
crc32_t crcChecker;		// CRC object for calculating CRC
bool firstCRC = true;	// Flag for first CRC calc
bool download_CRC = false; //Are we downloading CRC?
uint32_t dlCRC;			// Downloaded CRC

#define FIRMWARE_VERSION			 0x01
#define LED_0_PIN					 PIN_PA23
#define B1							 PIN_PB23
#define FW_STAT_ADDRESS			     0x7F00
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
struct spi_module at25dfx_spi;
struct at25dfx_chip_module at25dfx_chip;
//http Downloading stuff
uint32_t flash_addr;
uint8_t http_buf [2048];
uint32_t http_buf_write_ptr = 0x00000;
uint32_t http_buf_read_ptr = 0x00000;
static download_state down_state = NOT_READY;
struct http_client_module http_client_module_inst;

/** UART module for debug. */
static struct usart_module cdc_uart_module;

/** Instance of Timer module. */
struct sw_timer_module swt_module_inst;

/** User name of chat. */
//char mqtt_user[64] = "";

/** User name of chat. */
char mqtt_user[] = "aman";

/** Password of chat. */
char mqtt_pass[] = "aman";

/** Publishing text. */
char pub_text[64] = "";

/* Instance of MQTT service. */
static struct mqtt_module mqtt_inst;

/* Receive buffer of the MQTT service. */
static char mqtt_buffer[MAIN_MQTT_BUFFER_SIZE];

/** UART buffer. */
static char uart_buffer[MAIN_CHAT_BUFFER_SIZE];

/** Written size of UART buffer. */
static int uart_buffer_written = 0;

/** A buffer of character from the serial. */
static uint16_t uart_ch_buffer;

volatile int status;

/**
 * \brief Callback of USART input.
 *
 * \param[in] module USART module structure.
 */
static void uart_callback(const struct usart_module *const module)
{
	/* If input string is bigger than buffer size limit, ignore the excess part. */
	if (uart_buffer_written < MAIN_CHAT_BUFFER_SIZE) {
		uart_buffer[uart_buffer_written++] = uart_ch_buffer & 0xFF;
	}
}

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] msg_type type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_CONNTION_STATE](@ref M2M_WIFI_RESP_CONNTION_STATE)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS)
 *  - [M2M_WIFI_RESP_IP_CONFIGURED](@ref M2M_WIFI_RESP_IP_CONFIGURED)
 *  - [M2M_WIFI_RESP_IP_CONFLICT](@ref M2M_WIFI_RESP_IP_CONFLICT)
 *  - [M2M_WIFI_RESP_P2P](@ref M2M_WIFI_RESP_P2P)
 *  - [M2M_WIFI_RESP_AP](@ref M2M_WIFI_RESP_AP)
 *  - [M2M_WIFI_RESP_CLIENT_INFO](@ref M2M_WIFI_RESP_CLIENT_INFO)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type. Existing types are:
 *  - tstrM2mWifiStateChanged
 *  - tstrM2MWPSInfo
 *  - tstrM2MP2pResp
 *  - tstrM2MAPResp
 *  - tstrM2mScanDone
 *  - tstrM2mWifiscanResult
 */
static void wifi_callback(uint8 msg_type, void *msg_data)
{
	tstrM2mWifiStateChanged *msg_wifi_state;
	uint8 *msg_ip_addr;

	switch (msg_type) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
		msg_wifi_state = (tstrM2mWifiStateChanged *)msg_data;
		if (msg_wifi_state->u8CurrState == M2M_WIFI_CONNECTED) {
			/* If Wi-Fi is connected. */
			printf("Wi-Fi connected\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (msg_wifi_state->u8CurrState == M2M_WIFI_DISCONNECTED) {
			/* If Wi-Fi is disconnected. */
			printf("Wi-Fi disconnected\r\n");
			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
					MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
			/* Disconnect from MQTT broker. */
			/* Force close the MQTT connection, because cannot send a disconnect message to the broker when network is broken. */
			mqtt_disconnect(&mqtt_inst, 1);
		}

		break;

	case M2M_WIFI_REQ_DHCP_CONF:
		msg_ip_addr = (uint8 *)msg_data;
		printf("Wi-Fi IP is %u.%u.%u.%u\r\n",
				msg_ip_addr[0], msg_ip_addr[1], msg_ip_addr[2], msg_ip_addr[3]);
		/* Try to connect to MQTT broker when Wi-Fi was connected. */
		mqtt_connect(&mqtt_inst, main_mqtt_broker);
		break;

	default:
		break;
	}
}

/**
 * \brief Callback to get the Socket event.
 *
 * \param[in] Socket descriptor.
 * \param[in] msg_type type of Socket notification. Possible types are:
 *  - [SOCKET_MSG_CONNECT](@ref SOCKET_MSG_CONNECT)
 *  - [SOCKET_MSG_BIND](@ref SOCKET_MSG_BIND)
 *  - [SOCKET_MSG_LISTEN](@ref SOCKET_MSG_LISTEN)
 *  - [SOCKET_MSG_ACCEPT](@ref SOCKET_MSG_ACCEPT)
 *  - [SOCKET_MSG_RECV](@ref SOCKET_MSG_RECV)
 *  - [SOCKET_MSG_SEND](@ref SOCKET_MSG_SEND)
 *  - [SOCKET_MSG_SENDTO](@ref SOCKET_MSG_SENDTO)
 *  - [SOCKET_MSG_RECVFROM](@ref SOCKET_MSG_RECVFROM)
 * \param[in] msg_data A structure contains notification informations.
 */
static void socket_event_handler(SOCKET sock, uint8_t msg_type, void *msg_data)
{
	mqtt_socket_event_handler(sock, msg_type, msg_data);
}

/**
 * \brief Callback of gethostbyname function.
 *
 * \param[in] doamin_name Domain name.
 * \param[in] server_ip IP of server.
 */
static void socket_resolve_handler(uint8_t *doamin_name, uint32_t server_ip)
{
	mqtt_socket_resolve_handler(doamin_name, server_ip);
}

/**
 * \brief Callback to get the MQTT status update.
 *
 * \param[in] conn_id instance id of connection which is being used.
 * \param[in] type type of MQTT notification. Possible types are:
 *  - [MQTT_CALLBACK_SOCK_CONNECTED](@ref MQTT_CALLBACK_SOCK_CONNECTED)
 *  - [MQTT_CALLBACK_CONNECTED](@ref MQTT_CALLBACK_CONNECTED)
 *  - [MQTT_CALLBACK_PUBLISHED](@ref MQTT_CALLBACK_PUBLISHED)
 *  - [MQTT_CALLBACK_SUBSCRIBED](@ref MQTT_CALLBACK_SUBSCRIBED)
 *  - [MQTT_CALLBACK_UNSUBSCRIBED](@ref MQTT_CALLBACK_UNSUBSCRIBED)
 *  - [MQTT_CALLBACK_DISCONNECTED](@ref MQTT_CALLBACK_DISCONNECTED)
 *  - [MQTT_CALLBACK_RECV_PUBLISH](@ref MQTT_CALLBACK_RECV_PUBLISH)
 * \param[in] data A structure contains notification informations. @ref mqtt_data
 */
static void mqtt_callback(struct mqtt_module *module_inst, int type, union mqtt_data *data)
{
	switch (type) {
	case MQTT_CALLBACK_SOCK_CONNECTED:
	{
		/*
		 * If connecting to broker server is complete successfully, Start sending CONNECT message of MQTT.
		 * Or else retry to connect to broker server.
		 */
		if (data->sock_connected.result >= 0) {
			//mqtt_connect_broker(module_inst, 1, NULL, NULL, mqtt_user, NULL, NULL, 0, 0, 0);
			status = mqtt_connect_broker(module_inst, 1, mqtt_user, mqtt_pass, mqtt_user, NULL, NULL, 0, 0, 0);
		} else {
			printf("Connect fail to server(%s)! retry it automatically.\r\n", main_mqtt_broker);
			mqtt_connect(module_inst, main_mqtt_broker); /* Retry that. */
		}
	}
	break;

	case MQTT_CALLBACK_CONNECTED:
		if (data->connected.result == MQTT_CONN_RESULT_ACCEPT) {
			/* Subscribe chat topic. */
			printf("Trying to sub...\r\n");
			status = mqtt_subscribe(module_inst, MAIN_CHAT_TOPIC, 2);
			printf("Past chat sub\r\n");
			status = mqtt_subscribe(module_inst, SENSOR_TOPIC, 2);
			printf("Past sensor sub\r\n");
			status = mqtt_subscribe(module_inst, ACTUATOR_TOPIC, 2);
			printf("Past actuator sub\r\n");
			/* Enable USART receiving callback. */
			usart_enable_callback(&cdc_uart_module, USART_CALLBACK_BUFFER_RECEIVED);
			printf("Preparation of the chat has been completed.\r\n");
		} else {
			/* Cannot connect for some reason. */
			printf("MQTT broker decline your access! error code %d\r\n", data->connected.result);
		}

		break;

	case MQTT_CALLBACK_RECV_PUBLISH:
		/* You received publish message which you had subscribed. */
		if (data->recv_publish.topic != NULL && data->recv_publish.msg != NULL) {
			if (!strncmp(data->recv_publish.topic, MAIN_CHAT_TOPIC, strlen(MAIN_CHAT_TOPIC))) {
				/* Print user name and message */
				for (int i = strlen(MAIN_CHAT_TOPIC); i < data->recv_publish.topic_size; i++) {
					printf("%c", data->recv_publish.topic[i]);
				}
				printf(" >> ");
				for (int i = 0; i < data->recv_publish.msg_size; i++) {
					printf("%c", data->recv_publish.msg[i]);
				}
				printf("\r\n");
			}
		}

		break;

	case MQTT_CALLBACK_DISCONNECTED:
		/* Stop timer and USART callback. */
		printf("MQTT disconnected\r\n");
		usart_disable_callback(&cdc_uart_module, USART_CALLBACK_BUFFER_RECEIVED);
		break;
	}
}

/**
 * \brief Configure UART console.
 */
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

	stdio_serial_init(&cdc_uart_module, EDBG_CDC_MODULE, &usart_conf);
	/* Register USART callback for receiving user input. */
	usart_register_callback(&cdc_uart_module, (usart_callback_t)uart_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable(&cdc_uart_module);
}

/**
 * \brief Configure Timer module.
 */
static void configure_timer(void)
{
	struct sw_timer_config swt_conf;
	sw_timer_get_config_defaults(&swt_conf);

	sw_timer_init(&swt_module_inst, &swt_conf);
	sw_timer_enable(&swt_module_inst);
}

/**
 * \brief Configure MQTT service.
 */
static void configure_mqtt(void)
{
	struct mqtt_config mqtt_conf;
	int result;

	mqtt_get_config_defaults(&mqtt_conf);
	/* To use the MQTT service, it is necessary to always set the buffer and the timer. */
	mqtt_conf.timer_inst = &swt_module_inst;
	mqtt_conf.recv_buffer = mqtt_buffer;
	mqtt_conf.recv_buffer_size = MAIN_MQTT_BUFFER_SIZE;
	mqtt_conf.port = MQTT_PORT;

	result = mqtt_init(&mqtt_inst, &mqtt_conf);
	if (result < 0) {
		printf("MQTT initialization failed. Error code is (%d)\r\n", result);
		while (1) {
		}
	}

	result = mqtt_register_callback(&mqtt_inst, mqtt_callback);
	if (result < 0) {
		printf("MQTT register callback failed. Error code is (%d)\r\n", result);
		while (1) {
		}
	}
}

/**
 * \brief Checking the USART buffer.
 *
 * Finding the new line character(\n or \r\n) in the USART buffer.
 * If buffer was overflowed, Sending the buffer.
 */
static void check_usart_buffer(char *topic)
{
	int i;

	/* Publish the input string when newline was received or input string is bigger than buffer size limit. */
	if (uart_buffer_written >= MAIN_CHAT_BUFFER_SIZE) {
		mqtt_publish(&mqtt_inst, topic, uart_buffer, MAIN_CHAT_BUFFER_SIZE, 0, 0);
		uart_buffer_written = 0;
	} else {
		for (i = 0; i < uart_buffer_written; i++) {
			/* Find newline character ('\n' or '\r\n') and publish the previous string . */
			if (uart_buffer[i] == '\n') {
				mqtt_publish(&mqtt_inst, topic, uart_buffer, (i > 0 && uart_buffer[i - 1] == '\r') ? i - 1 : i, 0, 0);
				/* Move remain data to start of the buffer. */
				if (uart_buffer_written > i + 1) {
					memmove(uart_buffer, uart_buffer + i + 1, uart_buffer_written - i - 1);
					uart_buffer_written = uart_buffer_written - i - 1;
				} else {
					uart_buffer_written = 0;
				}

				break;
			}
		}
	}
}
/*~~~~~~~ HTTP DOWNLOADER FUNCTIONS ~~~~~~~~~~~~*/
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
	at25dfx_chip_write_buffer(&at25dfx_chip, flash_addr, http_buf + http_buf_read_ptr, len);	// write buffer
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
	
	clear_state(COMPLETED|DOWNLOADING|GET_REQUESTED|CANCELED);
	download_CRC = true;
	start_download();
	while (!(is_state_set(COMPLETED) || is_state_set(CANCELED))) {
		m2m_wifi_handle_events(NULL);
		sw_timer_task(&swt_module_inst);
	}
	
	download_CRC = false;
}
int main(void)
{
	tstrWifiInitParam param;
	int8_t ret;
	char topic[strlen(MAIN_CHAT_TOPIC) + MAIN_CHAT_USER_NAME_SIZE + 1];
	system_init();
	configure_console();
	configure_timer();
	configure_mqtt();
	nm_bsp_init();
	init_state();
	configure_nvm();
	configure_spi_flash();
	
	printf("User : %s\r\n", mqtt_user);
	printf("Password : %s\r\n", mqtt_pass);
	sprintf(topic, "%s", MAIN_CHAT_TOPIC);
	printf("Topic : %s\r\n", topic);
	
	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_callback; /* Set Wi-Fi event callback. */
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1) { /* Loop forever. */
		}
	}

	/* Initialize socket interface. */
	socketInit();
	registerSocketCallback(socket_event_handler, socket_resolve_handler);

	/* Connect to router. */
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
			MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);

	while (1) {
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);
		/* Try to read user input from USART. */
		usart_read_job(&cdc_uart_module, &uart_ch_buffer);
		/* Checks the timer timeout. */
		sw_timer_task(&swt_module_inst);
		/* Checks the USART buffer. */
		check_usart_buffer(topic);
		if (port_pin_get_input_level(BUTTON_0_PIN) == false) {
			sprintf(pub_text, "%d", 4);
			mqtt_publish(&mqtt_inst, SENSOR_TOPIC, pub_text, 1, 1, 1);
			delay_ms(300);
			//write_firmware = true;
		}
		
		if (write_firmware) {
			printf("Write_firmware was true\r\n");
			socketDeinit();
			mqtt_deinit(&mqtt_inst);
			ret = m2m_wifi_deinit(&param);
			if (M2M_SUCCESS != ret) {
				printf("main: m2m_wifi_deinit call error!(%d)\r\n", ret);
				while (1) { /* Loop forever. */
				}
			}
			/* ~~~~~~~~~~~~~~~~Begin HTTP client init~~~~~~~~~~~~~~~~~~*/
			configure_http_client();
			
			//memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));
			
			param.pfAppWifiCb = wifi_cb;
			
			ret = m2m_wifi_init(&param);
			if (M2M_SUCCESS != ret) {
				printf("main: m2m_wifi_init call error! (res %d)\r\n", ret);
				while (1) {
				}
			}
			
			socketInit();
			registerSocketCallback(socket_cb, resolve_cb);
			printf("Survived http client setup\r\n");
			/* ~~~~~~~~~~~~~~~~End HTTP client init~~~~~~~~~~~~~~~~~~*/
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
			printf("\n\rMain: Done downloading firmware and CRC\n\r");
			if (dlCRC == crcChecker){
				printf("\n\rMain: CRC MATCHED! starting firmware upgrade. \n\r");
				*(uint32_t*)fw_status.signature = (uint32_t)crcChecker; //replace with checksum of downloaded image
				fw_status.writenew_image = 1;  // write image flag
				writeFWStat(fw_status);
			} 
			else {
				printf("\n\r Main: CRC Check Fail!\n\r");
			}
			// reset to begin writing firmware
			system_reset();
		}
	}
}
