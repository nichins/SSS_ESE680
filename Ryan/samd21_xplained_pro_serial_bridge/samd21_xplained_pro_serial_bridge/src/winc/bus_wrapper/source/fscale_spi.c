/**
*  @file		fscale_spi.c				
*  @brief		SPI APIs Implementation
*  @Section		
*  @author		M.S.M
*  @date		29 July 2012
*  @version		1.0	
*  @sa
*/
#ifdef _FREESCALE_MCF51CN128_
#include "bus_wrapper\include\nm_bus_wrapper.h"
#include "bus_wrapper\include\fscale_spi.h"
#include <spi.h>


static int already_init = 0;
/*
*  @fn		sint8 spi_init(tstrSpiParam *ptstrSpi)
*  @brief	Initialize SPI interface
*  @param [IN] ptstrSpi
* 				spi parmeters structure
*  @author	M.S.M
*  @date	29 July 2012
*  @version	1.0
*/ 
sint8 spi_init(tstrSpiParam *ptstrSpi)
{
	FILE_PTR gpiofd, spifd;
	sint8 result = M2M_SUCCESS; 
	uint_32 i = 0 ;
	uint_32 param;
	SPI_CS_CALLBACK_STRUCT callback;
	uint_32 pin[] =
	{BSP_SPI0_GPIO_CS, GPIO_LIST_END };
	pin[0]=ptstrSpi->pincs;

	/* Open the SPI controller */
	M2M_INFO("open spi ...%s\n",ptstrSpi->port);
	spifd = fopen(ptstrSpi->port, NULL);
	if (NULL == spifd)
	{
		M2M_ERR("Failed to open the SPI driver, exiting ...\n");
		result = M2M_ERR_INIT;
		goto _fail_;
	}

	/* Open GPIO file containing SPI pin SS == chip select*/
	gpiofd = fopen("gpio:write", (char_ptr) & pin);
	if (NULL == gpiofd)
	{
		M2M_ERR("Opening GPIO failed.\n");
		result = M2M_ERR_INIT;
		goto _fail_;
	}

	/* Set CS callback */
	callback.MASK = ptstrSpi->mask;
	callback.CALLBACK = ptstrSpi->csfun;
	callback.USERDATA = gpiofd;

	/* Set CS callback */
	M2M_INFO("Setting CS callback\n");
	if (SPI_OK != ioctl(spifd, IO_IOCTL_SPI_SET_CS_CALLBACK, &callback))
	{

		M2M_ERR("Failed\n");
		result = M2M_ERR_INIT;
		goto _fail_;
	}
	/* Set a different rate */
	param = ptstrSpi->baudrate;
	M2M_INFO("Changing the baud rate to %d Hz ... \n", param);
	if (SPI_OK != ioctl(spifd, IO_IOCTL_SPI_SET_BAUD, &param))
	{
		M2M_ERR("Failed\n");
		result = M2M_ERR_INIT;
		goto _fail_;
	}

	/* Display baud rate */
	M2M_INFO("Current baud rate ... \n");
	if (SPI_OK == ioctl(spifd, IO_IOCTL_SPI_GET_BAUD, &param))
	{
		M2M_INFO("%d Hz\n", param);
	} else
	{
		M2M_ERR("Failed\n");
		result = M2M_ERR_INIT;
		goto _fail_;
	}

	/* Set clock mode */
	param = SPI_CLK_POL_PHA_MODE0;
	M2M_INFO("Setting clock mode to %d ... \n", param);
	if (SPI_OK != ioctl(spifd, IO_IOCTL_SPI_SET_MODE, &param))
	{
		M2M_ERR("Failed\n");
		result = M2M_ERR_INIT;
		goto _fail_;
	}

	/* Get clock mode */
	M2M_INFO("Getting clock mode ... \n");
	if (SPI_OK == ioctl(spifd, IO_IOCTL_SPI_GET_MODE, &param))
	{
		M2M_INFO("%d\n", param);
	} else
	{
		M2M_ERR("Failed\n");
		result = M2M_ERR_INIT;
		goto _fail_;
	}

	/* Set big endian */
	param = SPI_DEVICE_BIG_ENDIAN;
	M2M_INFO("Setting endian to %d ... \n", param);
	if (SPI_OK != ioctl(spifd, IO_IOCTL_SPI_SET_ENDIAN, &param))
	{
		M2M_ERR("Failed\n");
		result = M2M_ERR_INIT;
		goto _fail_;
	}

	/* Get endian */
	M2M_INFO("Getting endian ... \n");
	if (SPI_OK == ioctl(spifd, IO_IOCTL_SPI_GET_ENDIAN, &param))
	{
		M2M_INFO("%d\n", param);
	} else
	{
		M2M_ERR("Failed\n");
		result = M2M_ERR_INIT;
		goto _fail_;
	}

	/* Set transfer mode */
	param = SPI_DEVICE_MASTER_MODE;
	M2M_INFO("Setting transfer mode to %d ... \n", param);
	if (SPI_OK != ioctl(spifd, IO_IOCTL_SPI_SET_TRANSFER_MODE, &param))
	{
		M2M_ERR("Failed\n");
		result = M2M_ERR_INIT;
		goto _fail_;
	}

	/* Get transfer mode */
	M2M_INFO("Getting transfer mode ... \n");
	if (SPI_OK == ioctl(spifd, IO_IOCTL_SPI_GET_TRANSFER_MODE, &param))
	{
		M2M_INFO("%d\n", param);
	} else
	{
		M2M_ERR("Failed\n");
		result = M2M_ERR_INIT;
		goto _fail_;
	}

	/* Clear statistics */
	M2M_INFO("Clearing statistics ... \n");
	if (SPI_OK != ioctl(spifd, IO_IOCTL_SPI_CLEAR_STATS, NULL))
	{
		M2M_ERR("Failed\n");
		result = M2M_ERR_INIT;
		goto _fail_;
	}
#if 0
	/* Get statistics */
	M2M_INFO("Getting statistics ...\n");
	if (SPI_OK == ioctl(spifd, IO_IOCTL_SPI_GET_STATS, &stats))
	{
		M2M_INFO("Interrupts:   %d\n", stats.INTERRUPTS);
		M2M_INFO("Rx packets:   %d\n", stats.RX_PACKETS);
		M2M_INFO("Rx overflow:  %d\n", stats.RX_OVERFLOWS);
		M2M_INFO("Tx packets:   %d\n", stats.TX_PACKETS);
		M2M_INFO("Tx aborts :   %d\n", stats.TX_ABORTS);
		M2M_INFO("Tx underflow: %d\n", stats.TX_UNDERFLOWS);
	}
	else
	{
		M2M_ERR("Failed\n");
	}

#endif
	ptstrSpi->fpGpio =gpiofd;
	ptstrSpi->fpSpi=spifd;

	already_init = 1;/*spi now intilized */
_fail_: 
	return result;

}
/*
*  @fn		sint8 nm_spi_rw(tstrSpiParam *ptstrSpi,uint8 *wb, uint8 *rb, uint16 sz)
*  @brief	read and write spi 
*  @param [IN] ptstrSpi
* 				Poitner to SPI PARMATERS 
*  @param [IN] wb
* 				Poitner to the write data buffer
*  @param [OUT] rb
* 				Poitner to the receive data  buffer
*  @param [IN] Sz
* 				Data buffer size
*  @author	M.S.M
*  @date	30 JULY 2012
*  @version	1.0
*/
sint8 nm_spi_rw(tstrSpiParam *ptstrSpi,uint8 *wb, uint8 *rb, uint16 sz)
{
	sint8 result = M2M_SUCCESS;
	static uint8 dummy[NM_BUS_MAX_TRX_SZ];
	int i = 0;
	uint8 *pw, *pr;
	SPI_READ_WRITE_STRUCT rw;

	if (!wb && !rb)
	{
		M2M_ERR("wb & rb both are null");
		result = M2M_ERR_BUS_FAIL;
		return result;
	}
	memset(dummy, 0, sizeof(dummy));/*clear dummy buffer*/
	if (wb != NULL) pw = wb;
	else pw = dummy;

	if (rb != NULL) pr = rb;
	else pr = dummy;

	/* add data */
	rw.BUFFER_LENGTH = sz;
	rw.WRITE_BUFFER = (char_ptr) pw;
	rw.READ_BUFFER = (char_ptr) pr;
	/*start transfer */
	if (SPI_OK != ioctl(ptstrSpi->fpSpi, IO_IOCTL_SPI_READ_WRITE, &rw))
	{
		M2M_ERR("Can't send spi rw\n");
		result = M2M_ERR_BUS_FAIL;
	}
	/*wait to transfer to finish then decativate the cs */
	fflush(ptstrSpi->fpSpi);
	return result;


}

/*
*  @fn		spi_deinit(tstrSpiParam *ptstrSpi);
*  @brief	deinit spi
*  @author	M.S.M
*  @date	22 Jan 2012
*  @version	1.0
*/
void spi_deinit(tstrSpiParam *ptstrSpi)
{
	uint_32 result;
	/* Close the SPI */
	result = fclose(ptstrSpi->fpGpio);
	if (result)
	{
		M2M_ERR("Error closing GPIO, returned: 0x%08x\n", result);
		return;
	}
	/* Close the SPI */
	result = fclose(ptstrSpi->fpSpi);
	if (result)
	{
		M2M_ERR("Error closing SPI, returned: 0x%08x\n", result);
		return;
	}
	already_init = 0;
}
#endif
