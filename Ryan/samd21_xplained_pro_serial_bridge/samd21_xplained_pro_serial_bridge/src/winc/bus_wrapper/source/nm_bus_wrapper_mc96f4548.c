/**
*  @file		nm_bus_wrapper_MC96F4548.c				
*  @brief		This module contains NMC1000 bus_wrapper APIs implementation
*  @author		M.S.M
*  @date		21 OCT 2012
*  @version		1.0	
*/
#ifdef __MCF964548__
#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"


#define NM_BUS_MAX_TRX_SZ	512U
/*Spi*/
#define ASSERT_CS()         (P0 &= ~BIT1)

#define DEASSERT_CS()       (P0 |= BIT1)
/*i2c Declartion*/
#define ACK 	 1
#define NOACK    0

tstrNmBusCapabilities egstrNmBusCapabilities =
{
	NM_BUS_MAX_TRX_SZ
};

static sint8 spi_rw(uint8* pu8Mosi, uint8* pu8Miso, uint16 u16Sz)
{
	uint8 u8Dummy = 0;
	uint8 u8SkipMosi = 0, u8SkipMiso = 0;

	ASSERT_CS();

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

	while (u16Sz)
	{
		SPIDR0 = *pu8Mosi;
		while (!(SPISR0&(0x80)));
		*pu8Miso= SPIDR0;
		u16Sz --;
		if(!u8SkipMiso)pu8Miso++;
		if(!u8SkipMosi) pu8Mosi++;

	}

	DEASSERT_CS();
	return M2M_SUCCESS;
}
#ifndef _EVAL_ABOV_
static void i2c_stop(void)
{
	I2CMR1 |= 0x02;
	I2CSR1 = 0x00;

}
static uint8 i2c_start_W(uint8 address)
{

	I2CDR1 = (address << 1);
	I2CMR1 |= 0x1;
	I2CSR1 = 0x00;
	while((I2CSR1 != 0x87)&&(I2CSR1 != 0x86)&&(I2CSR1 != 0x0E));
	if(I2CSR1 == 0x87)
	{
		return 1;
	}
	i2c_stop();
	return 0;
}
static uint8 i2c_start_R(uint8 address)
{

	I2CDR1 = ((address << 1) +1);
	I2CMR1 |= 0x1;
	I2CSR1 = 0x00;
	while((I2CSR1 != 0x85)&&(I2CSR1 != 0x84)&&(I2CSR1 != 0x0C));
	if(I2CSR1 == 0x85)
	{
		return 1;
	}
	i2c_stop();
	return 0;	
}
static uint8 i2c_send_byte(uint8 u8byte)
{
	I2CDR1 = u8byte;
	I2CSR1 = 0x00;
	while((I2CSR1 != 0x47)&&(I2CSR1 != 0x46)&&(I2CSR1 != 0x0E));
	if(I2CSR1 == 0x47)
	{
		return 1;
	}
	i2c_stop();
	return 0;

}
static uint8 i2c_recv_byte(uint8 *u8byte,uint8 ack)
{
	if(ack)
	{
		I2CMR1 |= 0x08; 
	}
	else
	{
		I2CMR1 &=~0x08;
	}
	I2CSR1 = 0x00;
	while((I2CSR1 != 0x45)&&(I2CSR1 != 0x44)&&(I2CSR1 != 0x0C));
	if((I2CSR1 == 0x45)||((I2CSR1 == 0x44)&&(ack == NOACK)))
	{
		*u8byte = I2CDR1;
		return 1;
	}
	i2c_stop();
	return 0;
}
/*
*  @fn		uint8 nm_i2c_write(uint8 u8SlaveAddr, uint8 *pu8Buf, uint8 u8Sz)
*  @brief	Send data through I2C (UCB1)
*  @param [IN] u8SlaveAddr
* 				7-Bits slave address
*  @param [IN] pu8Buf
* 				Poitner to the data buffer
*  @param [IN] u8Sz
* 				Data buffer size
*  @return	0 in case of success and the number of remaining bytes in case of failure
*  @author	M.S.M
*  @date	18 Dec 2012
*  @version	1.0
*/
static uint8 nm_i2c_write(uint8 u8SlaveAddr, uint8 *pu8Buf, uint8 u8Sz)
{
	uint8 ret = M2M_SUCCESS;
	uint8 i = 0;

	if(i2c_start_W(u8SlaveAddr))
	{
		for(i=0;i<u8Sz;i++)
		{
			i2c_send_byte(pu8Buf[i]);
		}
	}
	else
	{
		ret = M2M_ERR_BUS_FAIL;
	}
	i2c_stop();

	return ret;
}
/**
*  @fn		uint8 nm_i2c_wr_restart(uint8 u8SlaveAddr, uint8 *pu8Buf1, uint8 u8Sz1, uint8 *pu8Buf2, uint8 u8Sz2)
*  @brief	Send data from first buffer then rsetart i2c then read data in the second buffer
*  @param [IN] u8SlaveAddr
* 				7-Bits slave address
*  @param [IN] pu8Buf1
* 				Poitner to 1st data buffer to be sent
*  @param [IN] u8Sz1
* 				1st data buffer size
*  @param [IN] pu8Buf2
* 				Poitner to 1st data buffer to be recvied
*  @param [IN] u8Sz2
* 				2st data buffer size
*  @return	0 in case of success and 1 in case of failure
*  @author	M.S.M
*  @date	18 Dec 2012
*  @version	1.0
*/
static uint8 nm_i2c_wr_restart(uint8 u8SlaveAddr, uint8 *pu8Buf1,
							   uint8 u8Sz1, uint8 *pu8Buf2, uint8 u8Sz2)
{
	uint8 ret = M2M_SUCCESS;
	uint8 i = 0;
	if(i2c_start_W(u8SlaveAddr))
	{
		for(i=0;i<u8Sz1;i++)
		{
			i2c_send_byte(pu8Buf1[i]);
		}
		if(i2c_start_R(u8SlaveAddr))
		{
			for(i=0;i<(u8Sz2-1);i++)
			{
				i2c_recv_byte(&pu8Buf2[i],ACK);
			}
			i2c_recv_byte(&pu8Buf2[i],NOACK);
		}
		else
		{
			ret = M2M_ERR_BUS_FAIL;
		}
	}
	else
	{
		ret = M2M_ERR_BUS_FAIL;
	}
	i2c_stop();

	return ret;
}

/**
*  @fn		uint8 nm_i2c_write_special(uint8 u8SlaveAddr, uint8 *pu8Buf1, uint8 u8Sz1, uint8 *pu8Buf2, uint8 u8Sz2)
*  @brief	Send data (from 2 data buffers) through I2C (UCB1)
*  @param [IN] u8SlaveAddr
* 				7-Bits slave address
*  @param [IN] pu8Buf1
* 				Poitner to 1st data buffer to be sent
*  @param [IN] u8Sz1
* 				1st data buffer size
*  @param [IN] pu8Buf2
* 				Poitner to 1st data buffer to be sent
*  @param [IN] u8Sz2
* 				1st data buffer size
*  @return	0 in case of success and 1 in case of failure
*  @author	M.S.M
*  @date	18 Dec 2012
*  @version	1.0
*/
static uint8 nm_i2c_write_special(uint8 u8SlaveAddr, uint8 *pu8Buf1,
								  uint8 u8Sz1, uint8 *pu8Buf2, uint8 u8Sz2)
{
	uint8 ret = M2M_SUCCESS;
	uint8 i = 0;
	if(i2c_start_W(u8SlaveAddr))
	{
		for(i=0;i<u8Sz1;i++)
		{
			i2c_send_byte(pu8Buf1[i]);
		}
		for(i=0;i<u8Sz2;i++)
		{
			i2c_send_byte(pu8Buf2[i]);
		}
	}
	else
	{
		ret = M2M_ERR_BUS_FAIL;
	}
	i2c_stop();

	return ret;
}

/*
*  @fn		uint8 nm_i2c_read(uint8 u8SlaveAddr, uint8 *pu8Buf, uint8 u8Sz)
*  @brief	Receive data through I2C 
*  @param [IN] u8SlaveAddr
* 				7-Bits slave address
*  @param [OUT] pu8Buf
* 				Poitner to the data buffer
*  @param [IN] u8Sz
* 				Data buffer size
*  @return	0 in case of success and the number of remaining bytes in case of failure
*  @author	M.S.M
*  @date	18 Dec 2012
*  @version	1.0
*/
static uint8 nm_i2c_read(uint8 u8SlaveAddr, uint8 *pu8Buf, uint8 u8Sz)
{
	uint8 ret = M2M_SUCCESS;
	uint8 i = 0;
	if(i2c_start_R(u8SlaveAddr))
	{
		for(i=0;i<(u8Sz-1);i++)
		{
			i2c_recv_byte(&pu8Buf[i],ACK);
		}
		i2c_recv_byte(&pu8Buf[i],NOACK);
	}
	else
	{
		ret = M2M_ERR_BUS_FAIL;
	}
	i2c_stop();

	return ret;

}
#endif
/*
*	@fn		nm_bus_init
*	@brief	Initialize the bus wrapper
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M.S.M
*	@date	22 OCT 2012
*	@version	1.0
*/
sint8 nm_bus_init(void *pvInitValue)
{
	sint8 result	= M2M_SUCCESS;

	P0IO		= 0xFF; 				// out	out  out  out  out	out  out  out
	P0OD		= 0xC0; 				// OD	OD	 PP   PP   PP	PP	 PP   PP
	P0PU		= 0xFF; 				// on	on	 on   on   on	on	 on   on
	P0			= 0xF3; 				// 1	1	 1	  1    1	1	 0	  1
	P0DB		= 0x00; 				// off	off  off  off  off	off  off  off
	/*Spi init*/
	SPICR0      = 0xA4;					/*2MBps*/
	SPISR0	    = 0x07;
	/*reset chip*/
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
*	@note	For SPI only, it's important to be able to send/receive at the same time
*	@date	22 OCT 2012
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
#ifndef _EVAL_ABOV_
	case NM_BUS_IOCTL_R:
		{
			tstrNmI2cDefault *pstrParam = (tstrNmI2cDefault *)pvParameter;
			s8Ret = nm_i2c_read(pstrParam->u8SlaveAdr,pstrParam->pu8Buf, pstrParam->u16Sz);
		}
		break;
	case NM_BUS_IOCTL_W:
		{
			tstrNmI2cDefault *pstrParam = (tstrNmI2cDefault *)pvParameter;
			s8Ret = nm_i2c_write(pstrParam->u8SlaveAdr,pstrParam->pu8Buf, pstrParam->u16Sz);
		}
		break;
	case NM_BUS_IOCTL_WR_RESTART:
		{
			tstrNmI2cSpecial *pstrParam = (tstrNmI2cSpecial *)pvParameter;
			s8Ret = nm_i2c_wr_restart(pstrParam->u8SlaveAdr,pstrParam->pu8Buf1, pstrParam->u16Sz1, pstrParam->pu8Buf2, pstrParam->u16Sz2);
		}
		break;
	case NM_BUS_IOCTL_W_SPECIAL:
		{
			tstrNmI2cSpecial *pstrParam = (tstrNmI2cSpecial *)pvParameter;
			s8Ret = nm_i2c_write_special(pstrParam->u8SlaveAdr,pstrParam->pu8Buf1, pstrParam->u16Sz1, pstrParam->pu8Buf2, pstrParam->u16Sz2);
		}
		break;
#endif
	default:
		s8Ret = -1;
		break;
	}
	return s8Ret;
}
/*
*	@fn		nm_bus_deinit
*	@brief	De-initialize the bus wrapper
*	@author	M.S.M
*	@date	22 OCT 2012
*	@version	1.0
*/
sint8 nm_bus_deinit(void)
{
	/*Spi deinit*/
	SPICR0      = 0x00;	
	P0			    = 0xF3;
	/**/
	return 0;
}
#endif
