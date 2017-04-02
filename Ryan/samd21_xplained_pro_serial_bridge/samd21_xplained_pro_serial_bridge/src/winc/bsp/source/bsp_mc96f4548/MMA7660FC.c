/**
*  @file		MMA7660FC.c				
*  @brief		This module MMA7660FC API 
*  @author		M.S.M
*  @date		20 DEC 2012
*  @version		1.0	
*/
#ifdef __MCF964548__
#include "bsp\include\nm_bsp.h"
#include "common\include\nm_common.h"
#include "bus_wrapper\include\nm_bus_wrapper.h"
#include "bsp\source\bsp_mc96f4548\MMA7660FC.h"

#ifndef _EVAL_ABOV_

#define MMA_OUT_X 0x00              // [6:0] are Read Only 6-bit output value X (XOUT[5] is 0 if the g direction is positive, 1 is negative)
#define MMA_OUT_Y 0x01              // [6:0] are Read Only 6-bit output value Y (YOUT[5] is 0 if the g direction is positive, 1 is negative)
#define MMA_OUT_Z 0x02              // [6:0] are Read Only 6-bit output value Z (ZOUT[5] is 0 if the g direction is positive, 1 is negative)
#define MMA_TILT_STATUS 0x03        // Tilt Status (Read only)
#define MMA_SRST_STATUS 0x04        // Sample Rate Status Register (Read only)
#define MMA_SPCNT_STATUS 0x05       // Sleep Count Register (Read/Write)
#define MMA_INTSU_STATUS 0x06       // Interrupt Setup Register
#define MMA_MODE_STATUS 0x07        // Mode Register (Read/Write)
#define MMA_SR_STATUS 0x08          // Auto-Wake and Active Mode Portrait/Landscape Samples per Seconds Register (Read/Write)
#define MMA_PDET_STATUS 0x09        // Tap/Pulse Detection Register (Read/Write)
#define MMA_PD_STATUS 0xA           // Tap/Pulse Debounce Count Register (Read/Write)

// Device initialization
void mma7660fc_init(void)
{
	mma_write_reg(MMA_MODE_STATUS, 0x00);       // Active Mode
	mma_write_reg(MMA_SPCNT_STATUS, 0x01);       // sleep counter 
	mma_write_reg(MMA_INTSU_STATUS, 0x20);      // automatic interrupt after every measurement
	mma_write_reg(MMA_PDET_STATUS, 0xE0);       // Active Mode
	mma_write_reg(MMA_SR_STATUS, 0x0A);         //0A// 32 Samples/Second w// 16 sample S
	mma_write_reg(MMA_PD_STATUS, 0x00);       // Active Mode
	mma_write_reg(MMA_MODE_STATUS, 0xD9);       // Active Mode
}

void mma7660fc_clear(void)
{
	volatile uint8 data xval = 0;
	mma_read_reg(MMA_TILT_STATUS,&xval);
}
// Reads x data
uint8 mma_read_x(void)
{
	uint8 val = 0;
	sint8 s8Ret = M2M_SUCCESS; 
	s8Ret = mma_read_reg(MMA_OUT_X,&val);
	if(s8Ret == M2M_SUCCESS)
		return val;
	return 0;
}


// Reads y data
uint8 mma_read_y(void)
{
	uint8 val = 0;
	sint8 s8Ret = M2M_SUCCESS; 
	s8Ret = mma_read_reg(MMA_OUT_Y,&val);
	if(s8Ret == M2M_SUCCESS)
		return val;
	return 0;

}


// Reads z data
uint8 mma_read_z(void)
{
	uint8 val = 0;
	sint8 s8Ret = M2M_SUCCESS; 
	s8Ret = mma_read_reg(MMA_OUT_Z,&val);
	if(s8Ret == M2M_SUCCESS)
		return val;
	return 0;
}
// Read from specified MMA7660FC register
sint8 mma_read_reg(uint8 u8Addr,uint8 *pu8Data)
{
	tstrNmI2cSpecial strI2c;
	sint8 s8Ret = M2M_SUCCESS;
	strI2c.u8SlaveAdr = ADDR_MMA7660;
	strI2c.pu8Buf1= &u8Addr;
	strI2c.u16Sz1 = 1;
	strI2c.pu8Buf2 = pu8Data;
	strI2c.u16Sz2 = 1; 
	s8Ret=nm_bus_ioctl(NM_BUS_IOCTL_WR_RESTART, &strI2c); 
	return s8Ret;
}
// Write register (The device must be placed in Standby Mode to change the value of the registers) 
sint8 mma_write_reg(uint8 u8Addr, uint8 u8Data)
{
	tstrNmI2cDefault strI2c;
	sint8 s8Ret = M2M_SUCCESS;
	uint8 cmd[2] = {0, 0};
	strI2c.u8SlaveAdr = ADDR_MMA7660;
	strI2c.pu8Buf = cmd ;
	strI2c.u16Sz = 2;
	cmd[0] = u8Addr;
	cmd[1] = u8Data;                          // New value of the register
	s8Ret=nm_bus_ioctl(NM_BUS_IOCTL_W, &strI2c); 
	return s8Ret;

}
#endif
#endif
