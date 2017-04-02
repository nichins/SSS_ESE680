/**
*  @file		nm_bus_wrapper_K20D50M.c			
*  @brief		This module contains the implementation of the bus wrapper functions
*  @author		Ahmad.Mohammad.Yahya
*  @date		21 MARCH 2013
*  @version		1.0	
*/

#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"

#define ASSERT_CS()          ( GPIOC_PCOR |=SPI_CS_PIN_MASK)  /* clear cs */
#define DEASSERT_CS()        ( GPIOC_PSOR |=SPI_CS_PIN_MASK)  /*set cs */


#define SPI_CS_PIN   4 /* CS0 on PTC4 */
#define SPI_CS_PIN_MASK (1<<SPI_CS_PIN)

#define SPI_CLK_PIN   5 /* CLK on PTC5 */
#define SPI_CLK_PIN_MASK (1<<SPI_CLK_PIN)

#define SPI_MOSI_PIN   6 /* MOSI on PTC6 */
#define SPI_MOSI_PIN_MASK (1 << SPI_MOSI_PIN)

#define SPI_MISO_PIN   7 /* MISO on PTC7 */
#define SPI_MISO_PIN_MASK (1<<SPI_MISO_PIN)

#define M2M_SUCCESS         ((sint8)0)
#define M2M_ERR_BUS_FAIL    ((sint8)-6)

#define NM_BUS_MAX_TRX_SZ	256

tstrNmBusCapabilities egstrNmBusCapabilities = 
{
	NM_BUS_MAX_TRX_SZ	
};

static void nm_udelay_k20(uint32 delay)
{
#if (CLK_MHZ == 50)
	volatile uint32 d;
	d = delay * (CLK_MHZ/10);
	while(d--);
#else
#error "Re-configure nm_udelay_k20 function"
#endif
}

static sint8 spi_rw(uint8* pu8Mosi, uint8* pu8Miso, uint16 u16Sz)
{
	
  	uint8 u8Dummy = 0;
  	uint8 u8SkipMosi = 0;
  	uint8 u8SkipMiso = 0;
    
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
    	SPI0_PUSHR = (SPI0_PUSHR & 0xFFFFFF00)|(*pu8Mosi);
    	while (!(SPI0_SR & SPI_SR_TCF_MASK));
    	SPI0_SR    =	SPI_SR_TCF_MASK ;      /* Clear TCF flag. */
    	*pu8Miso   = SPI0_POPR; 
        
    	u16Sz --;
    	if(!u8SkipMiso)	pu8Miso++;
    	if(!u8SkipMosi) pu8Mosi++;
      
    }
  	
#if 1
  	nm_udelay_k20(4);
#endif
    
    DEASSERT_CS();

	return M2M_SUCCESS; 
}



sint8 nm_bus_init(void *pvInitValue)
{
    sint8 result	= M2M_SUCCESS;

    SIM_SCGC6 |= (SIM_SCGC6_SPI0_MASK); /*power SPI0 */

    /* Set PTC SPI pins as SPI, CS as GPIO : */
    PORTC_PCR4 = PORT_PCR_MUX(1);   /* CS0 -- PTC4 -- as gpio */
    PORTC_PCR5 = PORT_PCR_MUX(2);   /* CLK -- PTC5 */
    PORTC_PCR6 = PORT_PCR_MUX(2);   /* MOSI -- PTC6 */
    PORTC_PCR7 = PORT_PCR_MUX(2);   /* MISO-- PTC7 */
    
    /* Configure Input & Output */     
    GPIOC_PDDR |= SPI_CS_PIN_MASK;   /* CS0 -- as output gpio */

    GPIOC_PSOR |=SPI_CS_PIN_MASK;    /* set cs */
    
    SPI0_MCR   = SPI_MCR_MSTR_MASK | SPI_MCR_DIS_RXF_MASK |  /* Configure SPI as master. Disable rx/tx fifos. Set */ 
            SPI_MCR_DIS_TXF_MASK |
            SPI_MCR_ROOE_MASK |   SPI_MCR_HALT_MASK |   /* overwrite incoming data. Set state to STOPPED. */ 
            SPI_MCR_PCSIS(1) | 							/* Chipselects inactive high */                        
            SPI_MCR_DCONF(0) ;							/* DSPI Configuration as SPI */
                    
    

    SPI0_TCR = (uint32_t)0x00UL;     

    SPI0_RSER  = (uint32_t)0x00UL;   /* Disable interrupts and DMA. */                     

    
    SPI0_CTAR0 = (SPI_CTAR_DBR_MASK | SPI_CTAR_FMSZ(8-1) | SPI_CTAR_PDT(0) | SPI_CTAR_BR(3)|SPI_CTAR_PBR(2) ); 
    
    SPI0_SR    = SPI_SR_EOQF_MASK | SPI_SR_TCF_MASK|SPI_SR_TFUF_MASK|SPI_SR_TFFF_MASK|SPI_SR_RFOF_MASK|SPI_SR_RFDF_MASK; 

        
    /* Config. trans. parameters */
    SPI0_PUSHR =  SPI_PUSHR_CTAS(0) 	/* Transmit Data use ctar0 */                                 
                 | SPI_PUSHR_CONT_MASK  /* 1 Keep PCSn signals asserted between transfers. */
                 | SPI_PUSHR_PCS(0);    /* negate the pcs signal.  //| SPI_PUSHR_PCS(1); */
 

    /* Set RUNNING state. */
    SPI0_MCR    &= ~SPI_MCR_HALT_MASK; /* SPI0_MCR: HALT=0 */

  return result;	 
}


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
	  default:
		s8Ret = -1;
		M2M_ERR("invalide ioclt cmd\n");
		break;
	}
	return s8Ret;
}


