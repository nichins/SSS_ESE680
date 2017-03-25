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
#define PORTB   PORT->Group[1]
#define LED_0_PIN PIN_PB11
#define LED_1_PIN PIN_PB10
#define RELAY PIN_PA05
#define B1 PIN_PA06

void configure_port_pins(void)
{
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED_0_PIN, &config_port_pin);
	port_pin_set_config(LED_1_PIN, &config_port_pin);
	port_pin_set_config(RELAY, &config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_INPUT;
	port_pin_set_config(B1, &config_port_pin);
}

int main (void)
{
	system_init();
	configure_port_pins();
	
	port_pin_set_output_level(LED_1_PIN, true);
	port_pin_set_output_level(RELAY, false);
	while (1) {
		if (port_pin_get_input_level(B1) == true) {
			port_pin_set_output_level(RELAY, false);
		}
		else {
			port_pin_set_output_level(RELAY, true);
			}
	}
	
}
