/*
* nm_bus_wrapper_nrf51822.c
*
*  Created on: oct 28, 2013
*      Author: M.S.M
*/

#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#include <spi_master.h>
#include "common.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"


#define SPI_OPERATING_FREQUENCY  ( 0x02000000UL << (uint32_t)Freq_2Mbps )  /*!< Slave clock frequency. */

/*  SPI1 */
#define SPI_PSELSCK1              22   /*!< GPIO pin number for SPI clock              */
#define SPI_PSELMOSI1             23   /*!< GPIO pin number for Master Out Slave In    */
#define SPI_PSELMISO1             21   /*!< GPIO pin number for Master In Slave Out    */
#define SPI_PSELSS1               20   /*!< GPIO pin number for Slave Select           */

#define TIMEOUT_COUNTER          0x3000UL  /*!< timeout for getting rx bytes from slave */

#define NM_BUS_MAX_TRX_SZ	512U


tstrNmBusCapabilities egstrNmBusCapabilities =
{
	NM_BUS_MAX_TRX_SZ
};

static sint8 spi_rw(uint8* pu8Mosi, uint8* pu8Miso, uint16 u16Sz)
{
	uint8 u8Dummy = 0;
	uint8 u8SkipMosi = 0, u8SkipMiso = 0;
		uint32_t counter = 0;
    uint32_t SEL_SS_PINOUT;

	NRF_SPI_Type *spi_base = (NRF_SPI_Type *)NRF_SPI1;
	SEL_SS_PINOUT = SPI_PSELSS1;

	if(!pu8Mosi)
	{
		pu8Mosi = &u8Dummy;
		u8SkipMosi = 1;
	}
	else if(!pu8Miso)
	{
		pu8Miso = &u8Dummy;
		u8SkipMiso = 1;
	}
	else
	{
		return M2M_ERR_BUS_FAIL;
	}

	/* enable slave (slave select active low) */
	nrf_gpio_pin_clear(SEL_SS_PINOUT);

	while(u16Sz)
	{
		spi_base->TXD = (uint32_t)(*pu8Mosi);

		/* Wait for the transaction complete or timeout (about 10ms - 20 ms) */
		while ((spi_base->EVENTS_READY == 0U) && (counter < TIMEOUT_COUNTER))
		{
			counter++;
		}
		if (counter == TIMEOUT_COUNTER)
		{
			/* timed out, disable slave (slave select active low) and return with error */
			nrf_gpio_pin_set(SEL_SS_PINOUT);
			return M2M_ERR_BUS_FAIL;
		}
		else
		{   /* clear the event to be ready to receive next messages */
			spi_base->EVENTS_READY = 0U;
		}

		*pu8Miso = (uint8_t)spi_base->RXD;
		u16Sz --;
		if(!u8SkipMiso)	pu8Miso++;
		if(!u8SkipMosi) pu8Mosi++;

	};

	/* disable slave (slave select active low) */
	nrf_gpio_pin_set(SEL_SS_PINOUT);
	return M2M_SUCCESS;
}

/*
*	@fn		nm_bus_init
*	@brief	Initialize the bus wrapper
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M.S.M
*	@date	28 oct 2013
*	@version	1.0
*/
sint8 nm_bus_init(void)
{
	sint8 result	= M2M_SUCCESS;
	uint32_t config_mode;

	NRF_SPI_Type *spi_base_address =(NRF_SPI_Type *)NRF_SPI1;

	/* Configure GPIO pins used for pselsck, pselmosi, pselmiso and pselss for SPI1*/
	nrf_gpio_cfg_output(SPI_PSELSCK1);
	nrf_gpio_cfg_output(SPI_PSELMOSI1);
	nrf_gpio_cfg_input(SPI_PSELMISO1, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_output(SPI_PSELSS1);

	/* Configure pins, frequency and mode */
	spi_base_address->PSELSCK  = SPI_PSELSCK1;
	spi_base_address->PSELMOSI = SPI_PSELMOSI1;
	spi_base_address->PSELMISO = SPI_PSELMISO1;
	nrf_gpio_pin_set(SPI_PSELSS1);         /* disable Set slave select (inactive high) */
	nrf_gpio_pin_set(SPI_PSELMISO1); /* disable Set slave select (inactive high) */

	spi_base_address->FREQUENCY = (uint32_t) SPI_OPERATING_FREQUENCY;
	/*SPI_MODE0 */
	config_mode = (SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
	/* msb first*/
	spi_base_address->CONFIG = (config_mode | (SPI_CONFIG_ORDER_MsbFirst << SPI_CONFIG_ORDER_Pos));

	spi_base_address->EVENTS_READY = 0U;
	/* Enable */
	spi_base_address->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);

	nrf_gpio_pin_set(SPI_PSELSS1);
	nrf_gpio_pin_set(SPI_PSELMISO1);
	nm_bsp_reset();
	return result;
}

/*
*	@fn		nm_bus_ioctl
*	@brief	send/receive from the bus
*	@param[IN]	u8Cmd
*					IOCTL command for the operation
*	@param[IN]	pvParameter
*					Arbitrary parameter depenging on IOCTL
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M.S.M
*	@date	28 oct 2013
*	@note	For SPI only, it's important to be able to send/receive at the same time
*	@version	1.0
*/
sint8 nm_bus_ioctl(uint8 u8Cmd, void* pvParameter)
{
	sint8 s8Ret = 0;
	switch(u8Cmd)
	{
	case NM_BUS_IOCTL_RW:
		{
			tstrNmSpiRw *pstrParam = (tstrNmSpiRw *)pvParameter;
			s8Ret = spi_rw(pstrParam->pu8InBuf, pstrParam->pu8OutBuf, pstrParam->u16Sz);
		}
		break;
	default:
		s8Ret = -1;
		M2M_ERR("invalide ioclt cmd\n");
		break;
	}
	return s8Ret;
}

/*
*	@fn		nm_bus_deinit
*	@brief	De-initialize the bus wrapper
*	@author	M.S.M
*	@date	28 oct 2013
*	@version	1.0
*/
sint8 nm_bus_deinit(void)
{
	return 0;
}


