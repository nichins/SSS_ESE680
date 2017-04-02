/*
 * image_downloader.h
 *
 * Created: 12/4/2014 3:04:24 PM
 *  Author: aali
 */ 

#ifndef IMAGE_DOWNLOADER_H_
#define IMAGE_DOWNLOADER_H_

/**
* Include
*/
#include "spi_flash_map.h"
#include "spi_flash.h"
#include "programmer_apis.h"


#define ROOT_CERT_SIZE M2M_TLS_FLASH_ROOTCERT_CACHE_SIZE

#define	programmer_get_flash_size()						(((spi_flash_get_size()*1024)/8)*1024)
#define	programmer_write(pu8Buf, u32Offset, u32Sz)		spi_flash_write(pu8Buf, u32Offset, u32Sz)
#define	programmer_erase(u32Offset, u32Sz)				spi_flash_erase(u32Offset, u32Sz)
#define	programmer_eraseall()							programmer_erase(0, programmer_get_flash_size())
#define	programmer_read(pu8Buf, u32Offset, u32Sz)		spi_flash_read(pu8Buf, u32Offset, u32Sz)	

#endif /* IMAGE_DOWNLOADER_H_ */