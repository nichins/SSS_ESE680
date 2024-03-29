#ifdef PROFILING
#include "windows.h"
#endif
#include "spi_flash.h"
#define DUMMY_REGISTER	(0x1084)

#define TIMEOUT 10000 /*MS*/

//#define DISABLE_UNSED_FLASH_FUNCTIONS

#define HOST_SHARE_MEM_BASE		(0xE0000)
#define CORTUS_SHARE_MEM_BASE	(0x60000000UL)
#define SHARED_PKT_MEM_BASE		(0xd0000UL)
#define NMI_SPI_FLASH_ADDR		(0x111c)
/***********************************************************
SPI Flash DMA 
***********************************************************/
#define GET_UINT32(X,Y)			(X[0+Y] + ((uint32)X[1+Y]<<8) + ((uint32)X[2+Y]<<16) +((uint32)X[3+Y]<<24))
#define SPI_FLASH_BASE			(0x10200)
#define SPI_FLASH_MODE			(SPI_FLASH_BASE + 0x00)
#define SPI_FLASH_CMD_CNT		(SPI_FLASH_BASE + 0x04)
#define SPI_FLASH_DATA_CNT		(SPI_FLASH_BASE + 0x08)
#define SPI_FLASH_BUF1			(SPI_FLASH_BASE + 0x0c)
#define SPI_FLASH_BUF2			(SPI_FLASH_BASE + 0x10)
#define SPI_FLASH_BUF_DIR		(SPI_FLASH_BASE + 0x14)
#define SPI_FLASH_TR_DONE		(SPI_FLASH_BASE + 0x18)
#define SPI_FLASH_DMA_ADDR		(SPI_FLASH_BASE + 0x1c)
#define SPI_FLASH_MSB_CTL		(SPI_FLASH_BASE + 0x20)
#define SPI_FLASH_TX_CTL		(SPI_FLASH_BASE + 0x24)

/*********************************************/
/* STATIC FUNCTIONS							 */
/*********************************************/

/**
*	@fn			spi_flash_read_status_reg
*	@brief		Read status register
*	@param[OUT]	val
					value of status reg
*	@return		Status of execution
*	@note		Compatible with MX25L6465E
*	@author		M. Abdelmawla
*	@version	1.0
*/ 
static sint8 spi_flash_read_status_reg(uint8 * val)
{
	uint8	cmd[1];
	uint32	reg;
	sint8	ret = M2M_SUCCESS;
	uint16	u16timeout = TIMEOUT;
	cmd[0] = 0x05;

	nm_write_reg(SPI_FLASH_DATA_CNT, 1);
	nm_write_reg(SPI_FLASH_BUF1, cmd[0]);
	nm_write_reg(SPI_FLASH_BUF_DIR, 0x01);
	
	nm_write_reg(SPI_FLASH_DMA_ADDR, HOST_SHARE_MEM_BASE);
	
	nm_write_reg(SPI_FLASH_CMD_CNT, 1 | (1<<7));
	do
	{
		nm_read_reg_with_ret(SPI_FLASH_TR_DONE, (uint32 *)&reg);
		u16timeout--;
	}
	while((reg != 1) && u16timeout);
	if(!u16timeout)
	{
		ret = M2M_ERR_BUS_FAIL;
	}
	
	*val = (uint8)reg & 0xFF;
	return ret;
}

#ifdef DISABLE_UNSED_FLASH_FUNCTIONS
/**
*	@fn			spi_flash_read_security_reg
*	@brief		Read security register
*	@return		Security register value
*	@note		Compatible with MX25L6465E
*	@author		M. Abdelmawla
*	@version	1.0
*/ 
static uint8 spi_flash_read_security_reg(void)
{
	uint8	cmd[1];
	uint32	reg;
	sint8	ret = M2M_SUCCESS;

	cmd[0] = 0x2b;

	nm_write_reg(SPI_FLASH_DATA_CNT, 1);
	nm_write_reg(SPI_FLASH_BUF1, cmd[0]);
	nm_write_reg(SPI_FLASH_BUF_DIR, 0x01);
	nm_write_reg(SPI_FLASH_DMA_ADDR, HOST_SHARE_MEM_BASE);
	nm_write_reg(SPI_FLASH_CMD_CNT, 1 | (1<<7));
	do
	{
		ret = nm_read_reg_with_ret(SPI_FLASH_TR_DONE, (uint32 *)&reg);
		if(M2M_SUCCESS != ret) break;
	}
	while(reg != 1);
	reg = (M2M_SUCCESS == ret)?(nm_read_reg(HOST_SHARE_MEM_BASE)):((uint8)M2M_ERR_BUS_FAIL);

	return (sint8)reg & 0xff;
}

/**
*	@fn			spi_flash_gang_unblock
*	@brief		Unblock all flash area
*	@note		Compatible with MX25L6465E
*	@author		M. Abdelmawla
*	@version	1.0
*/ 
static sint8 spi_flash_gang_unblock(void)
{
	uint8	cmd[1];
	uint32	val	= 0;
	sint8	ret = M2M_SUCCESS;

	cmd[0] = 0x98;

	nm_write_reg(SPI_FLASH_DATA_CNT, 0);
	nm_write_reg(SPI_FLASH_BUF1, cmd[0]);
	nm_write_reg(SPI_FLASH_BUF_DIR, 0x01);
	nm_write_reg(SPI_FLASH_DMA_ADDR, 0);
	nm_write_reg(SPI_FLASH_CMD_CNT, 1 | (1<<7));
	do
	{
		ret = nm_read_reg_with_ret(SPI_FLASH_TR_DONE, (uint32 *)&val);
		if(M2M_SUCCESS != ret) break;
	}
	while(val != 1);

	return ret;
}

/**
*	@fn			spi_flash_clear_security_flags
*	@brief		Clear all security flags
*	@note		Compatible with MX25L6465E
*	@author		M. Abdelmawla
*	@version	1.0
*/ 
static sint8 spi_flash_clear_security_flags(void)
{
	uint8 cmd[1];
	uint32	val	= 0;
	sint8	ret = M2M_SUCCESS;

	cmd[0] = 0x30;

	nm_write_reg(SPI_FLASH_DATA_CNT, 0);
	nm_write_reg(SPI_FLASH_BUF1, cmd[0]);
	nm_write_reg(SPI_FLASH_BUF_DIR, 0x01);
	nm_write_reg(SPI_FLASH_DMA_ADDR, 0);
	nm_write_reg(SPI_FLASH_CMD_CNT, 1 | (1<<7));
	do
	{
		ret = nm_read_reg_with_ret(SPI_FLASH_TR_DONE, (uint32 *)&val);
		if(M2M_SUCCESS != ret) break;
	}
	while(val != 1);

	return ret;
}
#endif

/**
*	@fn			spi_flash_load_to_cortus_mem
*	@brief		Load data from SPI flash into cortus memory
*	@param[IN]	u32MemAdr
*					Cortus load address. It must be set to its AHB access address
*	@param[IN]	u32FlashAdr
*					Address to read from at the SPI flash
*	@param[IN]	u32Sz
*					Data size
*	@return		Status of execution
*	@note		Compatible with MX25L6465E and should be working with other types
*	@author		M. Abdelmawla
*	@version	1.0
*/ 
static sint8 spi_flash_load_to_cortus_mem(uint32 u32MemAdr, uint32 u32FlashAdr, uint32 u32Sz)
{
	uint8 cmd[5];
	uint32	val	= 0;
	sint8	ret = M2M_SUCCESS;

	cmd[0] = 0x0b;
	cmd[1] = (uint8)(u32FlashAdr >> 16);
	cmd[2] = (uint8)(u32FlashAdr >> 8);
	cmd[3] = (uint8)(u32FlashAdr);
	cmd[4] = 0xA5;

	nm_write_reg(SPI_FLASH_DATA_CNT, u32Sz);
	nm_write_reg(SPI_FLASH_BUF1, cmd[0]|(cmd[1]<<8)|(cmd[2]<<16)|(cmd[3]<<24));
	nm_write_reg(SPI_FLASH_BUF2, cmd[4]);
	nm_write_reg(SPI_FLASH_BUF_DIR, 0x1f);
	nm_write_reg(SPI_FLASH_DMA_ADDR, u32MemAdr);
	nm_write_reg(SPI_FLASH_CMD_CNT, 5 | (1<<7));
	do
	{
		ret = nm_read_reg_with_ret(SPI_FLASH_TR_DONE, (uint32 *)&val);
		if(M2M_SUCCESS != ret) break;
	}
	while(val != 1);

	return ret;
}

/**
*	@fn			spi_flash_sector_erase
*	@brief		Erase sector (4KB)
*	@param[IN]	u32FlashAdr
*					Any memory address within the sector
*	@return		Status of execution
*	@note		Compatible with MX25L6465E and should be working with other types
*	@author		M. Abdelmawla
*	@version	1.0
*/ 
static sint8 spi_flash_sector_erase(uint32 u32FlashAdr)
{
	uint8 cmd[4];
	uint32	val	= 0;
	sint8	ret = M2M_SUCCESS;

	cmd[0] = 0x20;
	cmd[1] = (uint8)(u32FlashAdr >> 16);
	cmd[2] = (uint8)(u32FlashAdr >> 8);
	cmd[3] = (uint8)(u32FlashAdr);

	nm_write_reg(SPI_FLASH_DATA_CNT, 0);
	nm_write_reg(SPI_FLASH_BUF1, cmd[0]|(cmd[1]<<8)|(cmd[2]<<16)|(cmd[3]<<24));
	nm_write_reg(SPI_FLASH_BUF_DIR, 0x0f);
	nm_write_reg(SPI_FLASH_DMA_ADDR, 0);
	nm_write_reg(SPI_FLASH_CMD_CNT, 4 | (1<<7));
	do
	{
		ret = nm_read_reg_with_ret(SPI_FLASH_TR_DONE, (uint32 *)&val);
		if(M2M_SUCCESS != ret) break;
	}
	while(val != 1);

	return ret;
}

/**
*	@fn			spi_flash_write_enable
*	@brief		Send write enable command to SPI flash
*	@return		Status of execution
*	@note		Compatible with MX25L6465E and should be working with other types
*	@author		M. Abdelmawla
*	@version	1.0
*/ 
static sint8 spi_flash_write_enable(void)
{
	uint8 cmd[1];
	uint32	val	= 0;
	sint8	ret = M2M_SUCCESS;

	cmd[0] = 0x06;

	nm_write_reg(SPI_FLASH_DATA_CNT, 0);
	nm_write_reg(SPI_FLASH_BUF1, cmd[0]);
	nm_write_reg(SPI_FLASH_BUF_DIR, 0x01);
	nm_write_reg(SPI_FLASH_DMA_ADDR, 0);
	nm_write_reg(SPI_FLASH_CMD_CNT, 1 | (1<<7));
	do
	{
		ret = nm_read_reg_with_ret(SPI_FLASH_TR_DONE, (uint32 *)&val);
		if(M2M_SUCCESS != ret) break;
	}
	while(val != 1);

	return ret;
}

/**
*	@fn			spi_flash_write_disable
*	@brief		Send write disable command to SPI flash
*	@note		Compatible with MX25L6465E and should be working with other types
*	@author		M. Abdelmawla
*	@version	1.0
*/
static sint8 spi_flash_write_disable(void)
{
	uint8 cmd[1];
	uint32	val	= 0;
	sint8	ret = M2M_SUCCESS;
	cmd[0] = 0x04;

	nm_write_reg(SPI_FLASH_DATA_CNT, 0);
	nm_write_reg(SPI_FLASH_BUF1, cmd[0]);
	nm_write_reg(SPI_FLASH_BUF_DIR, 0x01);
	nm_write_reg(SPI_FLASH_DMA_ADDR, 0);
	nm_write_reg(SPI_FLASH_CMD_CNT, 1 | (1<<7));
	do
	{
		ret = nm_read_reg_with_ret(SPI_FLASH_TR_DONE, (uint32 *)&val);
		if(M2M_SUCCESS != ret) break;
	}
	while(val != 1);

	return ret;
}

/**
*	@fn			spi_flash_page_program
*	@brief		Write data (less than page size) from cortus memory to SPI flash
*	@param[IN]	u32MemAdr
*					Cortus data address. It must be set to its AHB access address
*	@param[IN]	u32FlashAdr
*					Address to write to at the SPI flash
*	@param[IN]	u32Sz
*					Data size
*	@note		Compatible with MX25L6465E and should be working with other types
*	@author		M. Abdelmawla
*	@version	1.0
*/ 
static sint8 spi_flash_page_program(uint32 u32MemAdr, uint32 u32FlashAdr, uint32 u32Sz)
{
	uint8 cmd[4];
	uint32	val	= 0;
	sint8	ret = M2M_SUCCESS;

	cmd[0] = 0x02;
	cmd[1] = (uint8)(u32FlashAdr >> 16);
	cmd[2] = (uint8)(u32FlashAdr >> 8);
	cmd[3] = (uint8)(u32FlashAdr);

	nm_write_reg(SPI_FLASH_DATA_CNT, 0);
	nm_write_reg(SPI_FLASH_BUF1, cmd[0]|(cmd[1]<<8)|(cmd[2]<<16)|(cmd[3]<<24));
	nm_write_reg(SPI_FLASH_BUF_DIR, 0x0f);
	nm_write_reg(SPI_FLASH_DMA_ADDR, u32MemAdr);
	nm_write_reg(SPI_FLASH_CMD_CNT, 4 | (1<<7) | ((u32Sz & 0xfffff) << 8));
	do
	{
		ret = nm_read_reg_with_ret(SPI_FLASH_TR_DONE, (uint32 *)&val);
		if(M2M_SUCCESS != ret) break;
	}
	while(val != 1);

	return ret;
}

/**
*	@fn			spi_flash_read_internal
*	@brief		Read from data from SPI flash
*	@param[OUT]	pu8Buf
*					Pointer to data buffer
*	@param[IN]	u32Addr
*					Address to read from at the SPI flash
*	@param[IN]	u32Sz
*					Data size
*	@note		Data size must be < 64KB (limitation imposed by the bus wrapper)
*	@author		M. Abdelmawla
*	@version	1.0
*/ 
static sint8 spi_flash_read_internal(uint8 *pu8Buf, uint32 u32Addr, uint32 u32Sz)
{
	sint8 ret = M2M_SUCCESS;
	/* read size must be < 64KB */
	ret = spi_flash_load_to_cortus_mem(HOST_SHARE_MEM_BASE, u32Addr, u32Sz);
	if(M2M_SUCCESS != ret) goto ERR;
	ret = nm_read_block(HOST_SHARE_MEM_BASE, pu8Buf, (uint16)u32Sz);
ERR:
	return ret;
} 

/**
*	@fn			spi_flash_pp
*	@brief		Program data of size less than a page (256 bytes) at the SPI flash
*	@param[IN]	u32Offset
*					Address to write to at the SPI flash
*	@param[IN]	pu8Buf
*					Pointer to data buffer
*	@param[IN]	u32Sz
*					Data size
*	@return		Status of execution
*	@author		M. Abdelmawla
*	@version	1.0
*/
static sint8 spi_flash_pp(uint32 u32Offset, uint8 *pu8Buf, uint16 u16Sz)
{
	sint8 ret = M2M_SUCCESS;
	uint8 tmp;
	spi_flash_write_enable();
	/* use shared packet memory as temp mem */
	ret = nm_write_block(SHARED_PKT_MEM_BASE, pu8Buf, u16Sz);
	ret = spi_flash_page_program(SHARED_PKT_MEM_BASE, u32Offset, u16Sz);
	if(M2M_SUCCESS != ret) goto ERR;
	do
	{
		if(ret == M2M_ERR_BUS_FAIL) goto ERR;
	}
	while((ret = spi_flash_read_status_reg(&tmp))& 0x01);
	ret = spi_flash_write_disable();
ERR:
	return ret;
}

/**
*	@fn			spi_flash_rdid
*	@brief		Read SPI Flash ID
*	@return		SPI FLash ID
*	@author		M.S.M
*	@version	1.0
*/
static uint32 spi_flash_rdid(void)
{
	unsigned char cmd[1];
	uint32 reg = 0;
	uint32 cnt = 0;
	sint8	ret = M2M_SUCCESS;

	cmd[0] = 0x9f;

	nm_write_reg(SPI_FLASH_DATA_CNT, 4);
	nm_write_reg(SPI_FLASH_BUF1, cmd[0]);
	nm_write_reg(SPI_FLASH_BUF_DIR, 0x1);
	nm_write_reg(SPI_FLASH_DMA_ADDR, DUMMY_REGISTER);
	nm_write_reg(SPI_FLASH_CMD_CNT, 1 | (1<<7));
	do
	{
		ret = nm_read_reg_with_ret(SPI_FLASH_TR_DONE, (uint32 *)&reg);
		if(M2M_SUCCESS != ret) break;
		if(++cnt > 500)
		{
			ret = M2M_ERR_INIT;
			break;
		}
	}
	while(reg != 1);
	reg = (M2M_SUCCESS == ret)?(nm_read_reg(DUMMY_REGISTER)):(0);
	M2M_PRINT("Flash ID %x \n",(unsigned int)reg);
	return reg;
}

/**
*	@fn			spi_flash_unlock
*	@brief		Unlock SPI Flash
*	@author		M.S.M
*	@version	1.0
*/
#if 0
static void spi_flash_unlock(void)
{
	uint8 tmp;
	tmp = spi_flash_read_security_reg();
	spi_flash_clear_security_flags();
	if(tmp & 0x80)
	{
		spi_flash_write_enable();
		spi_flash_gang_unblock();
	}
}
#endif

/*********************************************/
/* GLOBAL FUNCTIONS							 */
/*********************************************/

/**
*	@fn			spi_flash_read
*	@brief		Read from data from SPI flash
*	@param[OUT]	pu8Buf
*					Pointer to data buffer
*	@param[IN]	u32offset
*					Address to read from at the SPI flash
*	@param[IN]	u32Sz
*					Data size
*	@return		Status of execution
*	@note		Data size is limited by the SPI flash size only
*	@author		M. Abdelmawla
*	@version	1.0
*/ 
sint8 spi_flash_read(uint8 *pu8Buf, uint32 u32offset, uint32 u32Sz)
{
	sint8 ret = M2M_SUCCESS;
	if(u32Sz > FLASH_BLOCK_SIZE)
	{
		do
		{
			ret = spi_flash_read_internal(pu8Buf, u32offset, FLASH_BLOCK_SIZE);
			if(M2M_SUCCESS != ret) goto ERR;
			u32Sz -= FLASH_BLOCK_SIZE;
			u32offset += FLASH_BLOCK_SIZE;
			pu8Buf += FLASH_BLOCK_SIZE;
		} while(u32Sz > FLASH_BLOCK_SIZE);
	}
	
	ret = spi_flash_read_internal(pu8Buf, u32offset, u32Sz);

ERR:
	return ret;
}

/**
*	@fn			spi_flash_write
*	@brief		Proram SPI flash
*	@param[IN]	pu8Buf
*					Pointer to data buffer
*	@param[IN]	u32Offset
*					Address to write to at the SPI flash
*	@param[IN]	u32Sz
*					Data size
*	@return		Status of execution
*	@author		M. Abdelmawla
*	@version	1.0
*/ 
sint8 spi_flash_write(uint8* pu8Buf, uint32 u32Offset, uint32 u32Sz)
{
#ifdef PROFILING
	uint32 t1 = 0;
	uint32 percent =0;
	uint32 tpercent =0;
#endif
	sint8 ret = M2M_SUCCESS;
	uint32 u32wsz;
	uint32 u32off;
	uint32 u32Blksz;
	u32Blksz = FLASH_PAGE_SZ;
	u32off = u32Offset % u32Blksz;
#ifdef PROFILING
	tpercent = (u32Sz/u32Blksz)+((u32Sz%u32Blksz)>0);
	t1 = GetTickCount();
	M2M_PRINT(">Start programming..\r\n");
#endif
	if(u32Sz<=0)
	{
		M2M_ERR("Data size = %d",(int)u32Sz);
		ret = M2M_ERR_FAIL;
		goto ERR;
	}

	if (u32off)/*first part of data in the address page*/
	{
		u32wsz = u32Blksz - u32off;
		if(spi_flash_pp(u32Offset, pu8Buf, (uint16)BSP_MIN(u32Sz, u32wsz))!=M2M_SUCCESS)
		{
			ret = M2M_ERR_FAIL;
			goto ERR;
		}
		if (u32Sz < u32wsz) goto EXIT;
		pu8Buf += u32wsz;
		u32Offset += u32wsz;
		u32Sz -= u32wsz;
	}
	while (u32Sz > 0)
	{
		u32wsz = BSP_MIN(u32Sz, u32Blksz);

		/*write complete page or the remaining data*/
		if(spi_flash_pp(u32Offset, pu8Buf, (uint16)u32wsz)!=M2M_SUCCESS)
		{
			ret = M2M_ERR_FAIL;
			goto ERR;
		}
		pu8Buf += u32wsz;
		u32Offset += u32wsz;
		u32Sz -= u32wsz;
#ifdef PROFILING
		percent++;
		printf("\r>Complete Percentage = %d%%.\r",((percent*100)/tpercent));
#endif
	}
EXIT:
#ifdef PROFILING
	M2M_PRINT("\rDone\t\t\t\t\t\t");
	M2M_PRINT("\n#Programming time =%f sec\n\r",(GetTickCount() - t1)/1000.0);
#endif
ERR:
	return ret;
}

/**
*	@fn			spi_flash_erase
*	@brief		Erase from data from SPI flash
*	@param[IN]	u32Offset
*					Address to write to at the SPI flash
*	@param[IN]	u32Sz
*					Data size
*	@return		Status of execution
*	@note		Data size is limited by the SPI flash size only
*	@author		M. Abdelmawla
*	@version	1.0
*/ 
sint8 spi_flash_erase(uint32 u32Offset, uint32 u32Sz)
{
	uint32 i = 0;
	sint32 val = M2M_SUCCESS;
	uint8  tmp = 0;
#ifdef PROFILING
	uint32 t;
	t = GetTickCount();
#endif
	M2M_PRINT("\r\n>Start erasing...\r\n");
	for(i = u32Offset; i < (u32Sz +u32Offset); i += (16*FLASH_PAGE_SZ))
	{
		val += spi_flash_write_enable();
		val += spi_flash_read_status_reg(&tmp);
		val += spi_flash_sector_erase(i);
		val += spi_flash_read_status_reg(&tmp);
		do
		{
			if(val != M2M_SUCCESS) break;
		}
		while((val = spi_flash_read_status_reg(&tmp))& 0x01);
		if(val != M2M_SUCCESS) goto ERR;
	}
	M2M_PRINT("Done\r\n");
#ifdef PROFILING
	M2M_PRINT("#Erase time = %f sec\n", (GetTickCount()-t)/1000.0);
#endif
	return M2M_SUCCESS;
ERR:
	return M2M_ERR_BUS_FAIL;
}

/**
*	@fn			spi_flash_get_size
*	@brief		Get size of SPI Flash
*	@return		Size of Flash
*	@author		M.S.M
*	@version	1.0
*/
uint32 spi_flash_get_size(void)
{
	uint32 u32FlashId = 0, u32FlashPwr = 0;
	static uint32 gu32InernalFlashSize= 0;
	
	if(!gu32InernalFlashSize)
	{
		u32FlashId = spi_flash_rdid();//spi_flash_probe();
		if(u32FlashId != 0xffffffff)
		{
			/*flash size is the third byte from the FLASH RDID*/
			u32FlashPwr = ((u32FlashId>>16)&0xff) - 0x11; /*2MBIT is the min*/
			/*That number power 2 to get the flash size*/
			gu32InernalFlashSize = 1<<u32FlashPwr;
			M2M_INFO("Flash Size %lu Mb\n",gu32InernalFlashSize);
		}
		else
		{
			M2M_ERR("Cann't Detect Flash size\n");
		}
	}

	return gu32InernalFlashSize;
}