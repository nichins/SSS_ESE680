/**
*  @file		nm_bus_wrapper_fscale.c				
*  @brief		This module contains NMC1000 bus_wrapper APIs implementation for Aardvarc
*  @author		M.S.M
*  @date		10 JULY 2012
*  @version		1.0	
*/
#ifdef _FREESCALE_MCF51CN128_ 
#include "bus_wrapper\include\nm_bus_wrapper.h"
#include "bus_wrapper\include\fscale_spi.h"
#include "bus_wrapper\include\fscale_i2c.h"

/**
SPI parmaters for NMC1000 Chip
**/
#define SPI_BAUDRATE_NMC1000		(2000000UL)
#define SPI_CS_NMC1000		     	BSP_SPI1_GPIO_CS
#define SPI_PORT_NMC1000		    "spi1:"


/**
Global defination
**/
tstrSpiParam egstrSpiNmc1000Param; 

tstrNmBusCapabilities egstrNmBusCapabilities =
{
	NM_BUS_MAX_TRX_SZ 
};
/**
End Global
**/

static void set_CS(uint_32 cs_mask, uint_32 logic_level, pointer user_data) {
	FILE_PTR gpiofd = user_data;
	if (cs_mask & egstrSpiNmc1000Param.pincs)
	{
		if (logic_level)
		{
			if (IO_OK != ioctl(gpiofd, GPIO_IOCTL_WRITE_LOG1, NULL))
			{
				M2M_ERR("Setting CS pin failed.\n");
				_mqx_exit(-1);
			}
		} else
		{
			if (IO_OK != ioctl(gpiofd, GPIO_IOCTL_WRITE_LOG0, NULL))
			{
				M2M_ERR("Setting CS pin failed.\n");
				_mqx_exit(-1);
			}
		}
	}
}
/*
*	@fn		nm_bus_init
*	@brief	Initialize the bus wrapper
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M.S.M
*	@date	18 July 2012
*	@version	1.0
*/
sint8 nm_bus_init(void *pvInitValue) {

	sint8 result = M2M_SUCCESS;

	#ifdef USE_I2C
		if(M2M_SUCCESS!=i2c_init())
		{
			result = M2M_ERR_INIT;
			return result;

		}
	
	#elif defined (USE_SPI)
		egstrSpiNmc1000Param.baudrate =SPI_BAUDRATE_NMC1000;
		egstrSpiNmc1000Param.csfun=&set_CS;
		egstrSpiNmc1000Param.pincs=SPI_CS_NMC1000;
		egstrSpiNmc1000Param.mask=SPI_CS_NMC1000;
		egstrSpiNmc1000Param.port=SPI_PORT_NMC1000;

		if(M2M_SUCCESS!=spi_init(&egstrSpiNmc1000Param))
		{
			result = M2M_ERR_INIT;
			return result;		
		}
	#endif
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
*	@author	M. Abdelmawla
*	@note	For SPI only, it's important to be able to send/receive at the same time
*	@date	10 July 2012
*	@version	1.0
*/
sint8 nm_bus_ioctl(uint8 u8Cmd, void* pvParameter) {
	sint8 s8Ret = 0;
	switch (u8Cmd)
	{
	case NM_BUS_IOCTL_R:
		{
			tstrNmI2cDefault *pstrParam = (tstrNmI2cDefault *) pvParameter;
			s8Ret = nm_i2c_read(pstrParam->pu8Buf, pstrParam->u16Sz);
		}
		break;
	case NM_BUS_IOCTL_W:
		{
			tstrNmI2cDefault *pstrParam = (tstrNmI2cDefault *) pvParameter;
			s8Ret = nm_i2c_write(pstrParam->pu8Buf, pstrParam->u16Sz);
		}
		break;
	case NM_BUS_IOCTL_W_SPECIAL:
		{
			tstrNmI2cSpecial *pstrParam = (tstrNmI2cSpecial *) pvParameter;
			s8Ret = nm_i2c_write_special(pstrParam->pu8Buf1, pstrParam->u16Sz1, pstrParam->pu8Buf2,
				pstrParam->u16Sz2);
		}
		break;
	case NM_BUS_IOCTL_RW:
		{
			tstrNmSpiRw *pstrParam = (tstrNmSpiRw *) pvParameter;
			s8Ret = nm_spi_rw(&egstrSpiNmc1000Param,pstrParam->pu8InBuf, pstrParam->pu8OutBuf, pstrParam->u16Sz);

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
*	@date	18 July 2012
*	@version	1.0
*/
sint8 nm_bus_deinit(void) 
{
	#ifdef USE_I2C
		i2c_deinit();		
	
	#elif defined (USE_SPI)
	
		spi_deinit(&egstrSpiNmc1000Param);		
	#endif
	return 0;
}
#endif