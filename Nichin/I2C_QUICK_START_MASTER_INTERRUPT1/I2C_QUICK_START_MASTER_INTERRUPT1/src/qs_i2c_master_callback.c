/**
 * \file
 *
 * \brief SAM SERCOM I2C Master Quick Start Guide with Callbacks
 *
 * Copyright (C) 2012-2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include <system_interrupt.h>

void i2c_write_complete_callback(
		struct i2c_master_module *const module);
void configure_i2c(void);
void configure_i2c_callbacks(void);

//! [packet_data]
#define DATA_LENGTH 4
/*
static uint8_t wr_buffer[DATA_LENGTH] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
};

static uint8_t wr_buffer_reversed[DATA_LENGTH] = {
	0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00
};*/
/// ========================================================= /// Configuration defines and functions. /// ========================================================= #define STRING_EOL                      "\r\n" #define STRING_HEADER                   "-- ESE680A: Starter Example --"STRING_EOL \ "-- "BOARD_NAME " --"STRING_EOL \ "-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL
#define EDBG_CDC_MODULE              SERCOM4
#define EDBG_CDC_SERCOM_MUX_SETTING  USART_RX_3_TX_2_XCK_3
#define EDBG_CDC_SERCOM_PINMUX_PAD0  PINMUX_UNUSED
#define EDBG_CDC_SERCOM_PINMUX_PAD1  PINMUX_UNUSED
#define EDBG_CDC_SERCOM_PINMUX_PAD2  PINMUX_PB10D_SERCOM4_PAD2
#define EDBG_CDC_SERCOM_PINMUX_PAD3  PINMUX_PB11D_SERCOM4_PAD3
#define EDBG_CDC_CLOCK                GCLK_GENERATOR_0

static struct usart_module uart_module;
/**  * \brief Configure UART console.  */ 
static void configure_console(void) {
	struct usart_config usart_conf;
	usart_get_config_defaults(&usart_conf);   
	usart_conf.generator_source = EDBG_CDC_CLOCK;
	usart_conf.mux_setting      = EDBG_CDC_SERCOM_MUX_SETTING; 
	usart_conf.pinmux_pad0      = EDBG_CDC_SERCOM_PINMUX_PAD0; 
	usart_conf.pinmux_pad1      = EDBG_CDC_SERCOM_PINMUX_PAD1; 
	usart_conf.pinmux_pad2      = EDBG_CDC_SERCOM_PINMUX_PAD2; 
	usart_conf.pinmux_pad3      = EDBG_CDC_SERCOM_PINMUX_PAD3; 
	usart_conf.baudrate         = 115200;
	stdio_serial_init(&uart_module, EDBG_CDC_MODULE, &usart_conf); 
	usart_enable(&uart_module);
}
static uint8_t rd_buffer[4] = {0x00};
//! [packet_data]

//! [address]
#define SLAVE_RHT_ADDRESS_R 0x40 // read
#define SLAVE_RHT_ADDRESS_W 0x40 // write
#define SLAVE_ALS_ADDRESS_R 0x29 // read
#define SLAVE_ALS_ADDRESS_W 0x29 // write
//! [address]

//! [packet_glob]
struct i2c_master_packet wr_packet;
struct i2c_master_packet rd_packet;
//! [packet_glob]

/* Init software module instance. */
//! [dev_inst]
struct i2c_master_module i2c_master_instance;
//! [dev_inst]

//! [callback_func]
void i2c_write_complete_callback(
		struct i2c_master_module *const module)
{
	/* Initiate new packet read */
	//! [read_next]
	i2c_master_read_packet_job(&i2c_master_instance,&rd_packet);
	//! [read_next]
}
//! [callback_func]

//! [initialize_i2c]
void configure_i2c(void)
{
	/* Initialize config structure and software module */
	//! [init_conf]
	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);
	//! [init_conf]

	/* Change buffer timeout to something longer */
	//! [conf_change]
	config_i2c_master.buffer_timeout = 65535;
	config_i2c_master.pinmux_pad0    = PINMUX_PA08C_SERCOM0_PAD0;
	config_i2c_master.pinmux_pad1    = PINMUX_PA09C_SERCOM0_PAD1;
	//! [conf_change]

	/* Initialize and enable device with config */
	//! [init_module]
	while(i2c_master_init(&i2c_master_instance, CONF_I2C_MASTER_MODULE, &config_i2c_master)     \
			!= STATUS_OK);
	//! [init_module]

	//! [enable_module]
	i2c_master_enable(&i2c_master_instance);
	//! [enable_module]
}
//! [initialize_i2c]

//! [setup_callback]
//void configure_i2c_callbacks(void)
//{
	///* Register callback function. */
	////! [callback_reg]
	//i2c_master_register_callback(&i2c_master_instance, i2c_write_complete_callback,
			//I2C_MASTER_CALLBACK_WRITE_COMPLETE);
	////! [callback_reg]
	////! [callback_en]
	//i2c_master_enable_callback(&i2c_master_instance,
			//I2C_MASTER_CALLBACK_WRITE_COMPLETE);
	////! [callback_en]
//}
//! [setup_callback]

int main(void)
{
	system_init();
	//! [run_initialize_i2c]
	/* Configure device and enable. */
	//! [config]
	configure_i2c();
	//! [config]
	/* Configure callbacks and enable. */
	//! [config_callback]
	//configure_i2c_callbacks();
	//! [config_callback]
	//! [run_initialize_i2c]
	configure_console();
	delay_init();
	/* Init i2c packet. */
	//! [write_packet]
	wr_packet.address     = SLAVE_RHT_ADDRESS_W;
	wr_packet.data_length = DATA_LENGTH;
	//wr_packet.data        = wr_buffer;
	//! [write_packet]
	//! [read_packet]
	rd_packet.address     = SLAVE_RHT_ADDRESS_R;
	rd_packet.data_length = DATA_LENGTH;
	rd_packet.data        = rd_buffer;
	//! [read_packet]
	printf("setup complete \r\n");
	uint32_t t1 =0;
	uint32_t h1 =0;
	uint8_t tempC = 0;
	uint8_t hum = 0;
	//! [while]
	while (true)
	{
		 delay_ms(1000);
		 //Wake up RHT and get both temp and humidity reading.
		uint8_t wr_buffer1[3] = {0x02, 0x15, 0x00};
		wr_packet.data = wr_buffer1; //setup
		wr_packet.data_length = 3;
		i2c_master_write_packet_job(&i2c_master_instance, &wr_packet);	
		
		while(i2c_master_get_job_status	(&i2c_master_instance));
		uint8_t wr_buffer2[1] = {0x00};
		wr_packet.data = wr_buffer2; //trigger a measurement
		wr_packet.data_length = 1;
		i2c_master_write_packet_job(&i2c_master_instance, &wr_packet);
	
		delay_ms(15); // wait for conversion 
	
		rd_packet.data_length = 4;
		i2c_master_read_packet_job(&i2c_master_instance, &rd_packet);
		t1 =  ((uint32_t)rd_packet.data[0]<<8 | (uint32_t)rd_packet.data[1]);
		h1 =  ((uint32_t)rd_packet.data[2]<<8 | (uint32_t)rd_packet.data[3]);
		tempC = (uint8_t)((t1 * 0xA5)/ 0x10000 )- (uint8_t)0x28;
		hum = (uint8_t)((h1*0x64)/ 0x10000 );
		printf("Temp and humidity acquired! \r\n");
	}
}

