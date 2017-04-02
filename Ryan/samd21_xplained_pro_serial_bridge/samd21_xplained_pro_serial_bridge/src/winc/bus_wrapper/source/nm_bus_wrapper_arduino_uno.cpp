/*
* nm_bus_wrapper_arduino_uno.cpp
*
*  Created on: oct 28, 2013
*      Author: M.S.M
*/

#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"
#include <stdio.h>
#include <Arduino.h>
#include <avr/pgmspace.h>
#include "Wire.h"

#define I2C_ADDR					0x60

#define SPI_SCK_PIN              13   /*!< GPIO pin number for SPI clock              */
#define SPI_MOSI_PIN             11   /*!< GPIO pin number for Master Out Slave In    */
#define SPI_MISO_PIN             12   /*!< GPIO pin number for Master In Slave Out    */
#define SPI_SS_PIN               10   /*!< GPIO pin number for Slave Select  */

#define NM_BUS_MAX_TRX_SZ		    32UL


tstrNmBusCapabilities egstrNmBusCapabilities =
{
	NM_BUS_MAX_TRX_SZ
};

uint8 SPI_transfer(uint8 _data) {
	SPDR = _data;
	while (!(SPSR & _BV(SPIF)))
		;
	return SPDR;
}


static sint8 spi_rw(uint8* pu8Mosi, uint8* pu8Miso, uint16 u16Sz)
{

	uint8 u8Dummy = 0;
	uint8 u8SkipMosi = 0, u8SkipMiso = 0;

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

	// take the SS pin low to select the chip:
	digitalWrite(SPI_SS_PIN,LOW);

	/* enable slave (slave select active low) */
	while(u16Sz)
	{
		*pu8Miso =  SPI_transfer(*pu8Mosi);
		u16Sz --;
		if(!u8SkipMiso)	pu8Miso++;
		if(!u8SkipMosi) pu8Mosi++;
	};
	// take the SS pin high to de-select the chip:
	digitalWrite(SPI_SS_PIN,HIGH); 

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
sint8 nm_bus_init(void *pvInitValue)
{
	sint8 result	= M2M_SUCCESS;

#ifdef USE_SPI
	// set the slaveSelectPin as an output:
	// initialize SPI:
	// Set SS to high so a connected chip will be "deselected" by default
	digitalWrite(SPI_SS_PIN, HIGH);

	// When the SS pin is set as OUTPUT, it can be used as
	// a general purpose output port (it doesn't influence
	// SPI operations).
	pinMode(SPI_SS_PIN, OUTPUT);

	pinMode(SPI_MISO_PIN,INPUT_PULLUP);

	digitalWrite(SPI_SCK_PIN, LOW);
	pinMode(SPI_SCK_PIN, OUTPUT);
	pinMode(SPI_MOSI_PIN, OUTPUT);

	// Warning: if the SS pin ever becomes a LOW INPUT then SPI
	// automatically switches to Slave, so the data direction of
	// the SS pin MUST be kept as OUTPUT.
	SPCR |= _BV(MSTR);
	SPCR |= _BV(SPE);


	digitalWrite(SPI_MOSI_PIN, LOW);
	digitalWrite(SPI_SS_PIN, HIGH);
#elif USE_I2C

	Wire.begin();        // join i2c bus (address optional for master)
#endif

	//delay(500);


	return result;
}

static sint8 nm_i2c_write(uint8 *b, uint16 sz)
{
	sint8 result = M2M_SUCCESS;

	Wire.beginTransmission(I2C_ADDR); // transmit to device #4
	while(sz)
	{
		Wire.write(*b);        // sends bytes
		sz--;  
		b++;
	}
	Wire.endTransmission();    // stop transmitting

	return result;

}

static sint8 nm_i2c_write_special(uint8 *wb1, uint16 sz1, uint8 *wb2, uint16 sz2)
{
	static uint8 tmp[NM_BUS_MAX_TRX_SZ];
	m2m_memcpy(tmp, wb1, sz1);
	m2m_memcpy(&tmp[sz1], wb2, sz2);

	return nm_i2c_write(tmp, sz1+sz2);
}

static sint8 nm_i2c_read(uint8 *rb, uint16 sz)
{
	sint8 result = M2M_SUCCESS;

	Wire.requestFrom(I2C_ADDR, sz);    // request 6 bytes from slave device #2

	while(Wire.available())    // slave may send less than requested
	{ 
		*rb = Wire.read(); // receive a byte as character        // print the character
		rb++;
	}

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
	case NM_BUS_IOCTL_R:
		{
			tstrNmI2cDefault *pstrParam = (tstrNmI2cDefault *)pvParameter;
			s8Ret = nm_i2c_read(pstrParam->pu8Buf, pstrParam->u16Sz);
		}
		break;
	case NM_BUS_IOCTL_W:
		{
			tstrNmI2cDefault *pstrParam = (tstrNmI2cDefault *)pvParameter;
			s8Ret = nm_i2c_write(pstrParam->pu8Buf, pstrParam->u16Sz);
		}
		break;
	case NM_BUS_IOCTL_W_SPECIAL:
		{
			tstrNmI2cSpecial *pstrParam = (tstrNmI2cSpecial *)pvParameter;
			s8Ret = nm_i2c_write_special(pstrParam->pu8Buf1, pstrParam->u16Sz1, pstrParam->pu8Buf2, pstrParam->u16Sz2);
		}
		break;
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

/*
*	@fn			nm_bus_reinit
*	@brief		re-initialize the bus wrapper
*	@param [in]	void *config
*					re-init configuration data
*	@return		M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author		Dina El Sissy
*	@date		19 Sept 2012
*	@version	1.0
*/
sint8 nm_bus_reinit(void* config)
{
	return M2M_SUCCESS;
}

