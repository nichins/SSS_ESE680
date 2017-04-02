/*
 * nm_bus_wrapper_msp430fr5739.c
 *
 *  Created on: Jul 22, 2012
 *      Author: mabdelmawla
 */

#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"

#include "bus_wrapper/include/nm_bus_wrapper.h"


#define NM_BUS_MAX_TRX_SZ	512U

#define I2C_ADDR	0x60
#ifdef _ALPS_D2_

#define ASSERT_CS()          (P2OUT &= ~BIT3)
#define DEASSERT_CS()        (P2OUT |= BIT3)
#else
#define ASSERT_CS()          (P1OUT &= ~BIT3)

#define DEASSERT_CS()        (P1OUT |= BIT3)
#endif

tstrNmBusCapabilities egstrNmBusCapabilities =
{
	NM_BUS_MAX_TRX_SZ
};

static sint8 spi_rw(uint8* pu8Mosi, uint8* pu8Miso, uint16 u16Sz)
{
	uint8 u8Dummy = 0;
	uint8 u8SkipMosi = 0, u8SkipMiso = 0;

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
#ifdef _ALPS_D2_	
		while (!(UCA1IFG&UCTXIFG));
		UCA1IFG &= ~UCRXIFG;
		UCA1TXBUF = *pu8Mosi;
		while (!(UCA1IFG&UCRXIFG));
		*pu8Miso = UCA1RXBUF;
#else
    	while (!(UCB0IFG&UCTXIFG));
		UCB0TXBUF = *pu8Mosi;
		while (!(UCB0IFG&UCRXIFG));
		*pu8Miso = UCB0RXBUF;
#endif
		u16Sz --;
		if(!u8SkipMiso)	pu8Miso++;
		if(!u8SkipMosi) pu8Mosi++;
    }

	DEASSERT_CS();

	return M2M_SUCCESS;
}

/*
*	@fn		nm_bus_init
*	@brief	Initialize the bus wrapper
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@date	10 July 2012
*	@version	1.0
*/
sint8 nm_bus_init(void *pvInitValue)
{
	sint8 result	= M2M_SUCCESS;
#ifdef _ALPS_D2_
	// Select the SPI lines: MISO/MOSI on P2.6,5 CLK on P2.4
	P2SEL1 |= (BIT4 + BIT5 + BIT6);
	P2SEL0 &= (~(BIT4 + BIT5 + BIT6));


	UCA1CTLW0 |= UCSWRST;                     // **Put state machine in reset**
	UCA1CTLW0 = (UCMST+UCSYNC+UCMSB+UCCKPH);   	// 3-pin, 8-bit SPI master
	                                          // Clock polarity high, MSB
	UCA1CTLW0 |= UCSSEL_2;                    // ACLK
	UCA1BR0 = 24;	                           // /2 change to /1
	UCA1BR1 = 0;                              //

	UCA1CTLW0 &= ~UCSWRST;                    // **Initialize USCI state machine**
#else
// Select the SPI lines: MISO/MOSI on P1.6,7 CLK on P2.2
	P1SEL1 |= (BIT6 + BIT7);
	P1SEL0 &= (~(BIT6 + BIT7));

	P2SEL1 |= (BIT2);
	P2SEL0 &= ~BIT2;

	UCB0CTLW0 |= UCSWRST;                     // **Put state machine in reset**
	UCB0CTLW0 = (UCMST+UCSYNC+UCMSB+UCCKPH);   	// 3-pin, 8-bit SPI master
	                                          // Clock polarity high, MSB
	UCB0CTLW0 |= UCSSEL_2;                    // ACLK
	UCB0BR0 = 2;                           // /2 change to /1
	UCB0BR1 = 0;                              //

	UCB0CTLW0 &= ~UCSWRST; 
#endif
	return result;
}

/*
*	@fn		nm_bus_ioctl
*	@brief	send/receive from the bus
*	@param[IN]	u8Cmd
*					IOCTL command for the operation
*	@param[IN]	pvParameter
*					Arbitrary parameter depenging on IOCTL
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@note	For SPI only, it's important to be able to send/receive at the same time
*	@date	10 July 2012
*	@version	1.0
*/
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
		M2M_ERR("Invalid ioclt cmd\n");
		break;
	}
	return s8Ret;
}

/*
*	@fn		nm_bus_deinit
*	@brief	De-initialize the bus wrapper
*	@author	M. Abdelmawla
*	@date	10 July 2012
*	@version	1.0
*/
sint8 nm_bus_deinit(void)
{
	return  0;
}


