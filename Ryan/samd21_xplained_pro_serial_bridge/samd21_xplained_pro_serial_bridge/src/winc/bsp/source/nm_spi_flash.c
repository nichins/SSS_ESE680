/**
*  @file		nm_spi_flash.c				
*  @brief		This module contains spi flash driver API
*  @author		M.S.M
*  @date		18 JULY 2012
*  @version		1.0	
*/
#ifdef _FREESCALE_MCF51CN128_
#include "bsp\include\nm_spi_flash.h"
#include "bus_wrapper\include\nm_bus_wrapper.h"
#include "bus_wrapper\include\fscale_spi.h"

/**Spi Flash parmateres*/
#define SPI_BAUDRATE_FLASH		(8000000UL)
#define SPI_PORT_FLASH 			"spi0:"
#define SPI_CS_FLASH			BSP_EEPROM_GPIO_SPI_CS
#define BSP_MIN(x,y) ((x)>(y)?(y):(x))
/*Global*/
static tstrSpiParam egstrSpiFlashParam;
static FILE_PTR      spifdee,gpiofdee;
/**/

static void set_CS(uint_32 cs_mask, uint_32 logic_level, pointer user_data)
{
	FILE_PTR gpiofd = user_data;
	if (cs_mask & egstrSpiFlashParam.mask) 
	{
		if (logic_level) 
		{
			if (IO_OK != ioctl (gpiofd, GPIO_IOCTL_WRITE_LOG1, NULL))
			{
				M2M_ERR ("Setting CS pin failed.\n\r");
				_mqx_exit (-1);
			}
		} else {
			if (IO_OK != ioctl (gpiofd, GPIO_IOCTL_WRITE_LOG0, NULL))
			{
				M2M_ERR ("Setting CS pin failed.\n\r");
				_mqx_exit (-1);
			}
		}
	}
}

/*
*  @fn		sint8	nm_spi_flash_init(void)
*  @brief	Initialize SPI Flash
*  @author	M.S.M
*  @date	22 JULy 2012
*  @version	1.0
*/
sint8 nm_spi_flash_init(void)
{
	sint8  result = M2M_SUCCESS;

	egstrSpiFlashParam.baudrate =SPI_BAUDRATE_FLASH;
	egstrSpiFlashParam.csfun=&set_CS;
	egstrSpiFlashParam.pincs=SPI_CS_FLASH;
	egstrSpiFlashParam.port=SPI_PORT_FLASH;
	egstrSpiFlashParam.mask=BSP_EEPROM_SPI_CS;

	if(M2M_SUCCESS!=spi_init(&egstrSpiFlashParam))
	{
		result = M2M_ERR_INIT;
	}
	spifdee = egstrSpiFlashParam.fpSpi;
	return result;		
}
/*
*  @fn		void	nm_spi_flash_deinit(void)
*  @brief	Initialize SPI Flash
*  @author	M.S.M
*  @date	22 JULy 2012
*  @version	1.0
*/
void nm_spi_flash_deinit(void)
{
	spi_deinit(&egstrSpiFlashParam);

}
/*
*  @fn		void  nm_spi_flash_chip_erase(void)
*  @brief	Erase whole chip
*  @author	M.S.M
*  @date	22 JULY 2012
*  @version	1.0
*/
void nm_spi_flash_chip_erase(void)
{
	uint_32 result;
	uint8 send_buffer;

	/* This operation must be write-enabled */
	nm_spi_flash_set_write_latch ( TRUE);

	nm_spi_flash_read_status ();

	M2M_INFO("Erase whole FLASH chip ... ");
	send_buffer= SPI_FLASH_CHIP_ERASE;

	/* Write instruction */
	result = fwrite (&send_buffer, 1, 1, spifdee);

	/* Wait till transfer end (and deactivate CS) */
	fflush (spifdee);

	if (result != 1) 
	{
		M2M_ERR("FAILED!\n");
		return;
	}

	while (nm_spi_flash_read_status () & 1)
	{
		_time_delay (1000);
	}
	NM_BSP_PRINTF("OK\n\r");

}

/*
*  @fn		void nm_spi_flash_set_write_latch(boolean enable)
*  @brief	Write enable OR disable
*  @param [IN] enable
* 				1 enable or 0 disable  
*  @author	M.S.M
*  @date	22 JULY 2012
*  @version	1.0
*/
void nm_spi_flash_set_write_latch ( boolean enable)
{
	uint_32 result;
	uint8 send_buffer;

	if (enable)
	{
		send_buffer = SPI_EEPROM_WRITE_LATCH_ENABLE;
	} else {

		send_buffer = SPI_EEPROM_WRITE_LATCH_DISABLE;
	}
	/* Write instruction */
	result = fwrite (&send_buffer, 1, 1, spifdee);

	/* Wait till transfer end (and deactivate CS) */
	fflush (spifdee);

	if (result != 1) 
	{
		M2M_ERR ("FAILED!\n");
	}
}

/*
*  @fn		nm_spi_flash_set_protection(boolean protect)
*  @brief	enable protection or disable
*  @param [IN] enable
* 				1 enable or 0 disable 
*  @author	M.S.M
*  @date	22 JULY 2012
*  @version	1.0
*/
void nm_spi_flash_set_protection ( boolean protect)
{
	uint_32 result, i;
	uint_8 protection;
	uint8 send_buffer[5];

	if (protect)
	{
		M2M_INFO ("Write protect the EEPROM ... \n\r");
		protection = 0xFF;
	} else {
		M2M_INFO ("Write unprotect the EEPROM ... \n\r");
		protection = 0x00;
	}
	send_buffer[0] = SPI_EEPROM_WRITE_STATUS;
	send_buffer[1] = protection;

	/* Must do it twice to ensure right transitions in protection status register */
	for (i = 0; i < 2; i++)
	{
		/* Each write operation must be enabled in EEPROM */
		nm_spi_flash_set_write_latch (TRUE);
		nm_spi_flash_read_status ();
		/* Write instruction */
		result = fwrite (send_buffer, 1, 2, spifdee);
		/* Wait till transfer end (and deactivate CS) */
		fflush (spifdee);
		if (result != 2) 
		{
			M2M_ERR ("FAILED!\n");
		}
	}
}

/*
*  @fn		uint_8  nm_spi_flash_read_status(void)
*  @brief	Read status register 
*  @return  status register value  
*  @author	M.S.M
*  @date	22 JULY 2012
*  @version	1.0
*/
uint_8 nm_spi_flash_read_status(void)
{
	uint_32 result;
	uint_8 state = 0xFF;
	uint8 send_buffer;
	send_buffer = SPI_EEPROM_READ_STATUS;

	/* Write instruction */
	result = fwrite (&send_buffer, 1, 1, spifdee);

	if (result != 1)
	{
		/* Stop transfer */
		M2M_ERR ("FAILED (1)!\n");
		return state;
	}

	/* Read EEPROM status */
	result = fread (&state, 1, 1, spifdee);
	/* Wait till transfer end (and deactivate CS) */
	fflush (spifdee);

	if (result != 1) 
	{
		M2M_ERR ("FAILED (2)!\n");
	}
	return state;
}

/*
*  @fn		void    nm_spi_flash_write_byte(uint_32 addr, uchar data);
*  @brief	Write data to spi-flash using page programming mode
*  @param [IN] addr
* 				 memory address
*  @param [OUT] data
* 				Poitner to the read data
*  @author	M.S.M
*  @date	22 JULY 2012
*  @version	1.0
*/
void nm_spi_flash_write_byte ( uint_32 addr, uchar data)
{
	uint_32 result;
	uint8 send_buffer[5];

	/* Each write operation must be enabled in EEPROM */
	nm_spi_flash_set_write_latch ( TRUE);

	nm_spi_flash_read_status ();

	send_buffer[0] = SPI_EEPROM_WRITE_DATA;                                               // Write instruction
	for (result = SPI_EEPROM_ADDRESS_BYTES; result != 0; result--)
	{
		send_buffer[result] = (addr >> ((SPI_EEPROM_ADDRESS_BYTES - result) << 3)) & 0xFF; // Address
	}
	send_buffer[1 + SPI_EEPROM_ADDRESS_BYTES] = data;                                     // Data
	/* Write instruction, address and byte */
	result = fwrite (send_buffer, 1, 1 + SPI_EEPROM_ADDRESS_BYTES + 1, spifdee);

	/* Wait till transfer end (and deactivate CS) */
	fflush (spifdee);

	if (result != 1 + SPI_EEPROM_ADDRESS_BYTES + 1) 
	{
		M2M_ERR ("FAILED!\n");
	}
	/* There is 5 ms internal write cycle needed for EEPROM */
	_time_delay (5);
}

/*
*  @fn		uint_8  nm_spi_flash_read_byte(uint_32 addr)
*  @brief	Fast read data from M25P
*  @param [IN] addr
* 				 memory address
*  @return  value readed 
*  @author	M.S.M
*  @date	22 JULY 2012
*  @version	1.0
*/
uint_8 nm_spi_flash_read_byte ( uint_32 addr)
{
	uint_32 result;
	uint_8 data = 0; 
	uint8 send_buffer[5];

	send_buffer[0] = SPI_EEPROM_READ_DATA;                                                // Read instruction
	for (result = SPI_EEPROM_ADDRESS_BYTES; result != 0; result--)
	{
		send_buffer[result] = (addr >> ((SPI_EEPROM_ADDRESS_BYTES - result) << 3)) & 0xFF; // Address
	}
	/* Write instruction and address */
	result = fwrite (send_buffer, 1, 1 + SPI_EEPROM_ADDRESS_BYTES, spifdee);

	if (result != 1 + SPI_EEPROM_ADDRESS_BYTES)
	{
		/* Stop transfer */
		M2M_ERR ("FAILED (1)!\n");
		return data;
	}

	/* Read data from EEPROM */
	result = fread (&data, 1, 1, spifdee);

	/* Wait till transfer end (and deactivate CS) */
	fflush (spifdee);

	if (result != 1) 
	{
		M2M_ERR ("FAILED (2)!\n");
	}

	return data;
}

/*
*  @fn		uint_32 nm_spi_flash_write_data(uint_32 addr, uint_32 size, uchar_ptr data);
*  @brief	Write data to spi-flash using page programming mode
*  @param [IN] addr
* 				 memory address
*  @param [OUT] data
* 				Poitner to the read data  buffer
*  @param [IN] size
* 				Data buffer size,max size 256 byte
*  @author	M.S.M
*  @date	22 JULY 2012
*  @version	1.0
*/
sint8 nm_spi_flash_write_data ( uint_32 addr, uint_32 size, uchar_ptr data)
{
	uint_32 i, len;
	uint_32 result = size;
	uint8 send_buffer[5];
	sint8 ret = M2M_SUCCESS;

	while (result > 0) 
	{
		/* Each write operation must be enabled in EEPROM */
		nm_spi_flash_set_write_latch ( TRUE);

		nm_spi_flash_read_status ();

		len = result;
		if (len > SPI_EEPROM_PAGE_SIZE - (addr & (SPI_EEPROM_PAGE_SIZE - 1))) len = SPI_EEPROM_PAGE_SIZE - (addr & (SPI_EEPROM_PAGE_SIZE - 1));
		result -= len;

		send_buffer[0] = SPI_EEPROM_WRITE_DATA;                                     // Write instruction
		for (i = SPI_EEPROM_ADDRESS_BYTES; i != 0; i--)
		{
			send_buffer[i] = (addr >> ((SPI_EEPROM_ADDRESS_BYTES - i) << 3)) & 0xFF; // Address
		}
		/* Write instruction, address and data */
		i = fwrite (send_buffer, 1, (1 + SPI_EEPROM_ADDRESS_BYTES) , spifdee);
		i=i+ fwrite (data, 1, len, spifdee);

		/* Wait till transfer end (and deactivate CS) */
		fflush (spifdee);

		if (i != (1 + SPI_EEPROM_ADDRESS_BYTES + len)) 
		{
			M2M_ERR ("FAILED!\n"); 
			ret=M2M_ERR_SEND;
			return ret;
		}
		/* Move to next block */
		addr += len;
		data += len;

		/* There is 5 ms internal write cycle needed for EEPROM */

		while (nm_spi_flash_read_status () & 1)
		{
			_time_delay (5);
		}

	}
	return ret;
}


/*
*  @fn		uint_32 nm_spi_flash_read_data(uint_32 addr, uint_32 size, uchar_ptr data);
*  @brief	read data 
*  @param [IN] addr
* 				 memory address
*  @param [OUT] data
* 				Poitner to the read data  buffer
*  @param [IN] size
* 				Data buffer size,max size 256 byte
*  @author	M.S.M
*  @date	22 JULY 2012
*  @version	1.0
*/
sint8 nm_spi_flash_read_data ( uint_32 addr, uint_32 size, uchar_ptr data)
{
	int i=0;
	sint8 ret=M2M_SUCCESS;
	uint8 result;
	uint32 u32Off=0;
	uint32 u32Rsz=0;

	static uint8 dummy[NM_BUS_MAX_TRX_SZ];
	static uint8 send_buffer[5];

	SPI_READ_WRITE_STRUCT rw;

	memset(send_buffer, 0, sizeof(send_buffer));/*clear dummy buffer*/
	send_buffer[0] = SPI_EEPROM_READ_DATA;                                                // Read instruction
	for (result = SPI_EEPROM_ADDRESS_BYTES; result != 0; result--)
	{
		send_buffer[result] = (addr >> ((SPI_EEPROM_ADDRESS_BYTES - result) << 3)) & 0xFF; // Address
	}

	send_buffer[SPI_EEPROM_ADDRESS_BYTES+1]=0;
	memset(dummy, 0, sizeof(dummy));/*clear dummy buffer*/
	/* add data */
	/*send sdress and command */
	rw.BUFFER_LENGTH = 5;
	rw.WRITE_BUFFER = (char_ptr) send_buffer;
	rw.READ_BUFFER = (char_ptr) dummy;

	/*start transfer */
	if (SPI_OK != ioctl(spifdee, IO_IOCTL_SPI_READ_WRITE, &rw))
	{
		M2M_ERR("Can't send spi rw\n");
		ret=M2M_ERR_RCV;

	}

	do
	{  /*read data with maximam size =NM_BUS_MAX_TRX_SZ**/
		memset(dummy, 0, sizeof(dummy));
		u32Rsz = BSP_MIN(size, NM_BUS_MAX_TRX_SZ);
		rw.BUFFER_LENGTH = u32Rsz;
		rw.WRITE_BUFFER = (char_ptr)dummy;
		rw.READ_BUFFER = (char_ptr)data;

		/*start transfer */
		if (SPI_OK != ioctl(spifdee, IO_IOCTL_SPI_READ_WRITE, &rw))
		{
			M2M_ERR("Can't send spi rw\n");
			ret=M2M_ERR_RCV;
		}
		data += u32Rsz;
		size -= u32Rsz;
	} while (size > 0);
	/*wait to transfer to finish then decativate the cs */
	fflush(spifdee);
	return ret;
}

/*
*  @fn		uint_8  nm_spi_flash_read_Chipid(void)
*  @brief	Read identification 
*  @author	M.S.M
*  @date	22 JULY 2012
*  @version	1.0
*/
void  nm_spi_flash_read_Chipid(void)
{
	uint_32 result;
	char data[5];
	int i=0;

	M2M_INFO("Reading chip id.. ");

	memset(data,0,5);
	data[0] = SPI_FLASH_CHIP_ID;
	/*Write instruction and address */
	result = fwrite (data, 1, 1, spifdee);
	if (result != 1)
	{
		/* Stop transfer */
		M2M_ERR ("FAILED (1)!\n");
		fflush (spifdee);
	}
	/* Read size bytes of data */
	memset(data,0,5);
	result = fread (data, 1, 3, spifdee);
	/* Stop transfer */
	fflush (spifdee);
	for(i=0;i<3;i++)
		M2M_PRINT("%02x ",data[i]);
	M2M_PRINT("\n\r");
	if (result != 3) 
	{
		M2M_ERR ("FAILED (2)!\n"); 
	} 
}

#endif

