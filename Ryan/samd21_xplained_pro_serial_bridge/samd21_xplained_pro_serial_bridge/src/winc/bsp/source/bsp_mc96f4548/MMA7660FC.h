/**
*  @file		MMA7660FC.h				
*  @brief		This module MMA7660FC API 
*  @author		M.S.M
*  @date		20 DEC 2012
*  @version		1.0	
*/

#ifdef __MCF964548__
#ifndef MMA7660FC_H
#define MMA7660FC_H

#ifndef _EVAL_ABOV_

#define ADDR_MMA7660 0x4C

/** Initialization of device MMA7660FC (required)
*
*/
void mma7660fc_init(void);
void mma7660fc_clear(void);
/** Read the x register of the MMA7660FC
*
* @returns The value of x acceleration
*/
uint8 mma_read_x(void);

/** Read the y register of the MMA7660FC
*
* @returns The value of y acceleration
*/
uint8 mma_read_y(void);

/** Read the z register of the MMA7660FC
*
* @returns The value of z acceleration
*/
uint8 mma_read_z(void);

/** Read from specified MMA7660FC register
*
* @param addr The internal registeraddress of the MMA7660FC
* @returns The value of the register
*/
sint8 mma_read_reg(uint8 u8Addr,uint8 *pu8Data);
/** Write to specified MMA7660FC register
*
* @param addr The internal registeraddress of the MMA7660FC
* @param data New value of the register
*/    
sint8 mma_write_reg(uint8 u8Addr, uint8 u8Data); 
#endif 
#endif /*MMA7660FC_H*/
#endif
