/**
*  @file		fscale_i2c.c				
*  @brief		I2C APIs FOR FREESACLE 
*  @Section		
*  @author		M.S.M
*  @date		30 JULY 2012
*  @version		1.0	
*  @sa
*/
#ifdef _FREESCALE_MCF51CN128_
#include "bus_wrapper\include\nm_bus_wrapper.h"
#include "bus_wrapper\include\fscale_i2c.h"
#include <i2c.h>


/*Global definition */
static FILE_PTR fd;
/*end Global*/

/*
*  @fn		void i2c_init(void)
*  @brief	Initialize I2C interface
*  @author	M.S.M
*  @date	30 JULY 2011
*  @version	1.0
*/ 
sint8 i2c_init(void)
{     
	sint8 resultx = M2M_SUCCESS;
	uint_32                      param;
	I2C_STATISTICS_STRUCT        stats;

	/* Open the I2C driver */         
	fd = fopen (I2C_PORT, NULL);
	if (fd == NULL) 
	{
		M2M_INFO ("Failed to open the I2C driver!\n");
		resultx = M2M_ERR_INIT;
		goto _fail_;
	}
	/* Test ioctl commands */
	param = I2C_BAUDRATE;
	M2M_INFO ("Get current baud rate ... ");
	if (I2C_OK == ioctl (fd, IO_IOCTL_I2C_SET_BAUD, &param))
	{
		M2M_INFO ("%d\n", param);
	} else {
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_INIT;
		goto _fail_;

	}
	/* Test ioctl commands */
	M2M_INFO ("Get current baud rate ... ");
	if (I2C_OK == ioctl (fd, IO_IOCTL_I2C_GET_BAUD, &param))
	{
		M2M_INFO ("%d\n", param);
	} else {
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_INIT;
		goto _fail_;
	}

	M2M_INFO ("Set master mode ... ");
	if (I2C_OK == ioctl (fd, IO_IOCTL_I2C_SET_MASTER_MODE, NULL))
	{
		M2M_INFO ("OK\n");
	} else {
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_INIT;
		goto _fail_;
	}

	M2M_INFO ("Get current mode ... ");
	if (I2C_OK == ioctl (fd, IO_IOCTL_I2C_GET_MODE, &param))
	{
		M2M_INFO ("0x%02x\n", param);
	} else {
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_INIT;
		goto _fail_;
	}

	param = 0x50;
	M2M_INFO ("Set station address to 0x%02x ... ", param);
	if (I2C_OK == ioctl (fd, IO_IOCTL_I2C_SET_STATION_ADDRESS, &param))
	{
		M2M_INFO ("OK\n");
	} else {
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_INIT;
		goto _fail_;
	}
	param = 0x00;
	M2M_INFO ("Get station address ... ");
	if (I2C_OK == ioctl (fd, IO_IOCTL_I2C_GET_STATION_ADDRESS, &param))
	{
		M2M_INFO ("0x%02x\n", param);
	} else {
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_INIT;
		goto _fail_;
	}

	M2M_INFO ("Clear statistics ... ");
	if (I2C_OK == ioctl (fd, IO_IOCTL_I2C_CLEAR_STATISTICS, NULL))
	{
		M2M_INFO ("OK\n");
	} else {
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_INIT;
		goto _fail_;
	}

	M2M_INFO ("Get statistics ... ");
	if (I2C_OK == ioctl (fd, IO_IOCTL_I2C_GET_STATISTICS, (pointer)&stats))
	{
		M2M_INFO ("\nInterrupts:  %d\n", stats.INTERRUPTS);
		M2M_INFO ("Rx packets:  %d\n", stats.RX_PACKETS);
		M2M_INFO ("Tx packets:  %d\n", stats.TX_PACKETS);
		M2M_INFO ("Tx lost arb: %d\n", stats.TX_LOST_ARBITRATIONS);
		M2M_INFO ("Tx as slave: %d\n", stats.TX_ADDRESSED_AS_SLAVE);
		M2M_INFO ("Tx naks:     %d\n", stats.TX_NAKS);
	} else {
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_INIT;
		goto _fail_;
	}

	M2M_INFO ("Get current state ... ");
	if (I2C_OK == ioctl (fd, IO_IOCTL_I2C_GET_STATE, &param))
	{
		M2M_INFO ("0x%02x\n", param);
	} else {
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_INIT;
		goto _fail_;
	}

	param = I2C_ADDR;
	M2M_INFO ("Set destination address to 0x%02x ... ", param);
	if (I2C_OK == ioctl (fd, IO_IOCTL_I2C_SET_DESTINATION_ADDRESS, &param))
	{
		M2M_INFO ("OK\n");
	} else {
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_INIT;
		goto _fail_;
	}

	param = 0x00;
	M2M_INFO ("Get destination address ... ");
	if (I2C_OK == ioctl (fd, IO_IOCTL_I2C_GET_DESTINATION_ADDRESS, &param))
	{
		M2M_INFO ("0x%02x\n", param);
	} else {
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_INIT;
		goto _fail_;
	}
_fail_:
	return resultx;    

}

/*
*  @fn		sint8 nm_i2c_write(uint8 *b, uint16 sz)
*  @brief	Send data through I2C 
*  @param [IN] b
* 				Poitner to the data buffer
*  @param [IN] sz
* 				Data buffer size
*  @return	M2M_SUCCESS 
*  @author	M.S.M
*  @date	30 JULY 2012
*  @version	1.0
*/ 
sint8 nm_i2c_write(uint8 *b, uint16 sz) {

	sint8 resultx = M2M_SUCCESS;
	uint_32       param;
	uint_32       result;
	uint_8        mem;
	uint16 length;
	if(b==NULL)
	{
		M2M_ERR("write buffer is  null\n");
		resultx = M2M_ERR_SEND;
		return resultx;
	}

	/* I2C bus address also contains memory block index */
	param = I2C_ADDR;
	if (I2C_OK != ioctl (fd, IO_IOCTL_I2C_SET_DESTINATION_ADDRESS, &param))
	{
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_SEND;
		return resultx;
	}
	length = sz;
	/* Initiate start and send I2C bus address */
	fwrite (&mem, 1, 0, fd);
	/* Check ack (device exists) */
	if (I2C_OK == ioctl (fd, IO_IOCTL_FLUSH_OUTPUT, &param))
	{
		if (param) 
		{
			/* Stop I2C transfer */
			if (I2C_OK != ioctl (fd, IO_IOCTL_I2C_STOP, NULL))
			{
				M2M_ERR("Failed\n");
				resultx = M2M_ERR_SEND;
				return resultx;

			} 

		}
	} else {
		M2M_ERR("Failed\n");
		resultx= M2M_ERR_SEND;
		return resultx;
	}

	result = 0;
	result = fwrite (b, 1, length, fd);
	if (result != length)
	{
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_SEND;
		return resultx;
	}

	/* Wait for completion */
	result = fflush (fd);
	if (MQX_OK != result)
	{
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_SEND;
		return resultx;
	}
	/* Stop I2C transfer - initiate EEPROM write cycle */
	if (I2C_OK != ioctl (fd, IO_IOCTL_I2C_STOP, NULL))
	{
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_SEND;
		return resultx;
	}
	return resultx;
}


/**
*  @fn		sint8 nm_i2c_write_special(uint8 *wb1, uint16 sz1, uint8 *wb2, uint16 sz2)
*  @brief	Send data (from 2 data buffers) through I2C (UCB1)
*  @param [IN] wb1
* 				Poitner to 1st data buffer to be sent
*  @param [IN] Sz1
* 				1st data buffer size
*  @param [IN] wb2
* 				Poitner to 1st data buffer to be sent
*  @param [IN] Sz2
* 				1st data buffer size
*  @author	M.S.M
*  @date	30 JULY 2012
*  @version	1.0
*/ 
sint8 nm_i2c_write_special(uint8 *wb1, uint16 sz1, uint8 *wb2, uint16 sz2) {
#if 0
	static uint8 tmp[NM_BUS_MAX_TRX_SZ];
	memcpy(tmp, wb1, sz1);
	memcpy(&tmp[sz1], wb2, sz2);
	return nm_i2c_write(tmp, sz1+sz2);
	return 0;
#else
	sint8 resultx = M2M_SUCCESS;
	uint_32       param;
	uint_32       result;
	uint_8        mem;
	if((wb1==NULL)||(wb2==NULL))
	{
		M2M_ERR("write buffer is  null\n");
		resultx = M2M_ERR_SEND;
		return resultx;
	}

	/* I2C bus address also contains memory block index */
	param = I2C_ADDR;
	if (I2C_OK != ioctl (fd, IO_IOCTL_I2C_SET_DESTINATION_ADDRESS, &param))
	{
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_SEND;
		return resultx;
	}

	/* Initiate start and send I2C bus address */
	fwrite (&mem, 1, 0, fd);
	/* Check ack (device exists) */
	if (I2C_OK == ioctl (fd, IO_IOCTL_FLUSH_OUTPUT, &param))
	{
		if (param) 
		{
			/* Stop I2C transfer */
			if (I2C_OK != ioctl (fd, IO_IOCTL_I2C_STOP, NULL))
			{
				M2M_ERR("Failed\n");
				resultx = M2M_ERR_SEND;
				return resultx;

			} 

		}
	} else {
		M2M_ERR("Failed\n");
		resultx= M2M_ERR_SEND;
		return resultx;
	}

	result = 0;
	result = fwrite (wb1, 1, sz1, fd);
	if (result != sz1)
	{
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_SEND;
		return resultx;
	}

	result = 0;
	result = fwrite (wb2, 1, sz2, fd);
	if (result != sz2)
	{
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_SEND;
		return resultx;
	}

	/* Wait for completion */
	result = fflush (fd);
	if (MQX_OK != result)
	{
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_SEND;
		return resultx;
	}
	/* Stop I2C transfer - initiate EEPROM write cycle */
	if (I2C_OK != ioctl (fd, IO_IOCTL_I2C_STOP, NULL))
	{
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_SEND;
		return resultx;
	}
	return resultx;

#endif
}

/*
*  @fn		sint8 nm_i2c_read(uint8 *rb, uint16 sz)
*  @brief	Receive data through I2C 
*  @param [OUT] rb
* 				Poitner to the data buffer
*  @param [IN] Sz
* 				Data buffer size
*  @return	M2M_success
*  @author	M.S.M
*  @date	30 JULY 2012
*  @version	1.0
*/ 
sint8 nm_i2c_read(uint8 *rb, uint16 sz) {
	sint8 resultx = M2M_SUCCESS;
	uint_32       param;
	uint_32       result;
	uint_8        mem;

	/* Initiate start and send I2C bus address */
	fwrite (&mem, 1, 0, fd);

	/* Check ack (device exists) */
	if (I2C_OK == ioctl (fd, IO_IOCTL_FLUSH_OUTPUT, &param))
	{
		if (param) 
		{
			/* Stop I2C transfer */
			if (I2C_OK != ioctl (fd, IO_IOCTL_I2C_STOP, NULL))
			{
				M2M_ERR("Failed\n");
				resultx = M2M_ERR_RCV;
				return resultx;
			}

		}
	} else {
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_RCV;
		return resultx;
	}

	/* Restart I2C transfer for reading */
	if (I2C_OK != ioctl (fd, IO_IOCTL_I2C_REPEATED_START, NULL))
	{
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_RCV;
		return resultx;

	}

	/* Set read request */
	param = sz;
	if (I2C_OK != ioctl (fd, IO_IOCTL_I2C_SET_RX_REQUEST, &param))
	{
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_RCV;
		return resultx;
	}

	/* Read all data */
	result = fread (rb, 1, sz, fd);
	if (result != sz)
	{
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_RCV;
		return resultx;
	}

	/* Wait for completion */
	result = fflush (fd);
	if (MQX_OK != result)
	{
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_RCV;
		return resultx;
	}

	/* Stop I2C transfer - initiate EEPROM write cycle */
	if (I2C_OK != ioctl (fd, IO_IOCTL_I2C_STOP, NULL))
	{
		M2M_ERR("Failed\n");
		resultx = M2M_ERR_RCV;
		return resultx;
	}
	return resultx;
}
/*
*  @fn		void i2c_deinit(void)
*  @brief	De-Initialize I2C interface
*  @author	M.S.M
*  @date	30 JULY 2012
*  @version	1.0
*/ 
void i2c_deinit(void)
{

	uint_32 result;
	/* Close the GPIO */
	result = fclose(fd);
	if (result)
	{
		M2M_ERR("Error closing GPIO, returned: 0x%08x\n", result);
	}

}
#endif
