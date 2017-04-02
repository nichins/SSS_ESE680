/**
*  @file		fscale_spi.h				
*  @brief		SPI APIs header
*  @Section		
*  @author		M.S.M
*  @date		29 July 2012
*  @version		1.0	
*  @sa
*/
#ifndef NMI_SPI_H_
#define NMI_SPI_H_

#include "common\include\nm_common.h"

#define NM_BUS_MAX_TRX_SZ 256

/*Spi must be enabled in the kernel */
#if ! BSPCFG_ENABLE_SPI1
#error This application requires BSPCFG_ENABLE_SPI1 defined non-zero in user_config.h. Please recompile kernel with this option.
#endif 
#if ! BSPCFG_ENABLE_SPI0
#error This application requires BSPCFG_ENABLE_SPI0 defined non-zero in user_config.h. Please recompile kernel with this option.
#endif 
/**
*	@struct	tstrSpiParam
*	@brief	Structure holding SPI PARMATERS
*	@author	M.S.M
*	@date	30 July 2012
*	@version	1.0
*/ 
typedef struct 
{
	uint32 pincs;			/*!< CS pin */
	uint32 baudrate;		/*!< baudrate in hz */
	char *port;				/*!< port name "spi0:"/"spi1:" */
	void *csfun;			/*!< callback function for CS */
	uint32 mask;			/*!< Mask in the call back function */
	FILE_PTR fpSpi;			/*!< spi handler */
	FILE_PTR fpGpio;		/*!< Gpio handler */
} tstrSpiParam;

/*
*  @fn		sint8 spi_init(tstrSpiParam *ptstrSpi)
*  @brief	Initialize SPI interface
*  @param [IN] tstrSpiParam *ptstrSpi
* 				spi parameter
*  @author	M.S.M
*  @date	29 July 2012
*  @version	1.0
*/ 
sint8 spi_init(tstrSpiParam *ptstrSpi);

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
sint8 nm_spi_rw(tstrSpiParam *ptstrSpi,uint8 *wb, uint8 *rb, uint16 sz);

/*
*  @fn		spi_deinit(tstrSpiParam *ptstrSpi);
*  @brief	deinit spi
*  @author	M.S.M
*  @date	22 Jan 2012
*  @version	1.0
*/

void spi_deinit(tstrSpiParam *ptstrSpi);

#endif /*SPI_H_*/
