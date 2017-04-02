/**
*  @file		fscale_i2c.h				
*  @brief		I2C APIs FOR FREESACLE 
*  @Section		
*  @author		M.S.M
*  @date		30 JULY 2012
*  @version		1.0	
*  @sa
*/

#ifndef _I2C_H_
#define _I2C_H_

#include "common\include\nm_common.h"

/**
I2C parmaters
**/
#define I2C_ADDR			    0x60
#define I2C_PORT				"i2c1:"
#define I2C_BAUDRATE			400000 

/*I2C must be enabled in the kernel */
#if ! BSPCFG_ENABLE_I2C1
#error This application requires BSPCFG_ENABLE_I2C1 defined non-zero in user_config.h. Please recompile BSP with this option.
#endif



/*
*  @fn		void i2c_init(void)
*  @brief	Initialize I2C interface
*  @author	M.S.M
*  @date	30 JULY 2011
*  @version	1.0
*/ 
sint8 i2c_init(void);

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
sint8 nm_i2c_write(uint8 *b, uint16 sz);
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
sint8 nm_i2c_write_special(uint8 *wb1, uint16 sz1, uint8 *wb2, uint16 sz2);

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
sint8 nm_i2c_read(uint8 *rb, uint16 sz);
/*
*  @fn		void i2c_deinit(void)
*  @brief	De-Initialize I2C interface
*  @author	M.S.M
*  @date	30 JULY 2012
*  @version	1.0
*/ 
void i2c_deinit(void);

#endif /*I2C_H_*/

