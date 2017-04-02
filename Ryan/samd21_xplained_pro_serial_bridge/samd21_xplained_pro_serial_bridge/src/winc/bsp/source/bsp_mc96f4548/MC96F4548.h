//------------------------------------------------------------------------------
// This confidential and proprietary software may be used only as
// authorized by a licensing agreement from ABOV Semiconductor.
// In the event of publication, the following notice is applicable:
//
// (C)COPYRIGHT 2011 ABOV Semiconductor co., ltd.
// ALL RIGHTS RESERVED
//
// The entire notice above must be reproduced on all authorized copies.
//
// Project: Sub MCU
// File   : MC95F4548.h
// Author : J.T Han
// Date   : 01/01/2011
// Version: 0.1
// Abstract: This file defines constant variables such as SFR address.
//
// Modification History
// Date       By    Version   Change Description
// 01/01/2011 JTH   0.1       95F4548 application
// 20/03/2012 JL    0.2       sub MCU application
//
#ifdef __MCF964548__
//------------------------------------------------------------------------------
// PORT Control Register
//------------------------------------------------------------------------------
sfr     P0	    =       0x80;	// R/W	00H	P0 Data Register
	sbit    P00 	=       0x80;
	sbit    P01	  	=       0x81;
	sbit    P02	  	=       0x82;
	sbit    P03 	=       0x83;
	sbit    P04	  	=       0x84;
	sbit    P05	  	=       0x85;
	sbit    P06	  	=       0x86;
	sbit    P07	  	=       0x87;

sfr     P0DA        =       0x80;       // PORTCON      R/W 8'b0000_0000    P0 Data Register
	sbit    P0DA0 	=       0x80;
	sbit    P0DA1 	=       0x81;
	sbit    P0DA2 	=       0x82;
	sbit    P0DA3 	=       0x83;
	sbit    P0DA4 	=       0x84;
	sbit    P0DA5 	=       0x85;
	sbit    P0DA6 	=       0x86;
	sbit    P0DA7 	=       0x87;

sfr     P0IO        =       0x89;       // PORTCON      R/W 8'b0000_0000    P0 Direction Register

// defined Port 1 register name
sfr     P1	    =       0x88;	// R/W	00H	P1 Data Register
	sbit    P10     =       0x88;
	sbit    P11     =       0x89;
	sbit    P12     =       0x8A;
	sbit    P13     =       0x8B;
	sbit    P14     =       0x8C;
	sbit    P15     =       0x8D;
	sbit    P16     =       0x8E;
	//sbit    P17     =       0x8F;
				
sfr     P1DA        =       0x88;       // PORTCON      R/W 8'b0000_0000    P1 Data Register
	sbit    P1DA0   =       0x88;
	sbit    P1DA1   =       0x89;
	sbit    P1DA2   =       0x8A;
	sbit    P1DA3   =       0x8B;
	sbit    P1DA4   =       0x8C;
	sbit    P1DA5   =       0x8D;
	sbit    P1DA6   =       0x8E;
	//sbit    P1DA7   =       0x8F;
sfr     P1IO        =       0x91;       // PORTCON      R/W 8'b0000_0000    P1 Direction Register

// defined Port 2 register name
sfr     P2	    =       0x90;	// R/W	00H	P2 Data Register
	sbit    P20     =       0x90;
	sbit    P21     =       0x91;
	sbit    P22     =       0x92;
	sbit    P23     =       0x93;
	sbit    P24     =       0x94;
	sbit    P25     =       0x95;
	sbit    P26     =       0x96;
	sbit    P27     =       0x97;
sfr     P2DA        =       0x90;       // PORTCON      R/W 8'b0000_0000    P2 Data Register
	sbit    P2DA0   =       0x90;
	sbit    P2DA1   =       0x91;
	sbit    P2DA2   =       0x92;
	sbit    P2DA3   =       0x93;
	sbit    P2DA4   =       0x94;
	sbit    P2DA5   =       0x95;
	sbit    P2DA6   =       0x96;
	//sbit    P2DA7   =       0x97;
sfr     P2IO        =       0x99;       // PORTCON      R/W 8'b0000_0000    P2 Direction Register

// defined Port 3 register name
sfr     P3	    =       0x98;	// R/W	00H	P3 Data Register
	sbit    P30     =       0x98;
	sbit    P31     =       0x99;
	sbit    P32     =       0x9A;
	sbit    P33     =       0x9B;
	sbit    P34     =       0x9C;
	sbit    P35     =       0x9D;
	sbit    P36     =       0x9E;
	sbit    P37     =       0x9F;
sfr     P3DA        =       0x98;       // PORTCON      R/W 8'b0000_0000    P3 Data Register
	sbit    P3DA0   =       0x98;
	sbit    P3DA1   =       0x99;
	sbit    P3DA2   =       0x9A;
	sbit    P3DA3   =       0x9B;
	sbit    P3DA4   =       0x9C;
	sbit    P3DA5   =       0x9D;
	sbit    P3DA6   =       0x9E;
	sbit    P3DA7   =       0x9F;
sfr     P3IO        =       0xA1;       // PORTCON      R/W 8'b0000_0000    P3 Direction Register
								
sfr     P4          =       0xA0;       // PORTCON      R/W 8'b0000_0000    P4 Data Register
sfr     P4DA        =       0xA0;       // PORTCON      R/W 8'b0000_0000    P4 Data Register
sfr     P4IO        =       0xB1;       // PORTCON      R/W 8'b0000_0000    P4 Direction Register
								
sfr     P5          =       0xB0;       // PORTCON      R/W 8'b0000_0000    P5 Data Register
sfr     P5DA        =       0xB0;       // PORTCON      R/W 8'b0000_0000    P5 Data Register
sfr     P5IO        =       0xB9;       // PORTCON      R/W 8'b0000_0000    P5 Direction Register
								
sfr     P6          =       0xC0;       // PORTCON      R/W 8'b0000_0000    P6 Data Register
sfr     P6DA        =       0xC0;       // PORTCON      R/W 8'b0000_0000    P6 Data Register
sfr     P6IO        =       0xC1;       // PORTCON      R/W 8'b0000_0000    P6 Direction Register
								
sfr     P7          =       0xC8;       // PORTCON      R/W 8'b0000_0000    P7 Data Register
sfr     P7DA        =       0xC8;       // PORTCON      R/W 8'b0000_0000    P7 Data Register
sfr     P7IO        =       0xC9;       // PORTCON      R/W 8'b0000_0000    P7 Direction Register

#define     P0PU    *(volatile unsigned char xdata *) 0x2F00       // PORTCON      R/W 8'b0000_0000    P0 Pull-up Register
#define     P0OD    *(volatile unsigned char xdata *) 0x2F0C       // PORTCON      R/W 8'b0000_0000    P0 Open Drain Register
#define     P0DB    *(volatile unsigned char xdata *) 0x2F18       // PORTCON      R/W 8'b0000_0000    P0 DEBOUNCE Register
														
#define     P1PU    *(volatile unsigned char xdata *) 0x2F01       // PORTCON      R/W 8'b0000_0000    P1 Pull-up Register
#define     P1OD    *(volatile unsigned char xdata *) 0x2F0D       // PORTCON      R/W 8'b0000_0000    P1 Open Drain Register
#define     P1DB    *(volatile unsigned char xdata *) 0x2F19       // PORTCON      R/W 8'b0000_0000    P1 DEBOUNCE Register
														
#define     P2PU    *(volatile unsigned char xdata *) 0x2F02       // PORTCON      R/W 8'b0000_0000    P2 Pull-up Register
#define     P2OD    *(volatile unsigned char xdata *) 0x2F0E       // PORTCON      R/W 8'b0000_0000    P2 Open Drain Register
#define     P2DB    *(volatile unsigned char xdata *) 0x2F1A       // PORTCON      R/W 8'b0000_0000    P2 DEBOUNCE Register
														
#define     P3PU    *(volatile unsigned char xdata *) 0x2F03       // PORTCON      R/W 8'b0000_0000    P3 Pull-up Register
#define     P3OD    *(volatile unsigned char xdata *) 0x2F0F       // PORTCON      R/W 8'b0000_0000    P3 Open Drain Register
#define     P3DB    *(volatile unsigned char xdata *) 0x2F1B       // PORTCON      R/W 8'b0000_0000    P3 DEBOUNCE Register
													
#define     P4PU    *(volatile unsigned char xdata *) 0x2F04       // PORTCON      R/W 8'b0000_0000    P4 Pull-up Register
#define     P4OD    *(volatile unsigned char xdata *) 0x2F10       // PORTCON      R/W 8'b0000_0000    P4 Open Drain Register
#define     P4DB    *(volatile unsigned char xdata *) 0x2F1C       // PORTCON      R/W 8'b0000_0000    P4 DEBOUNCE Register
												
#define     P5PU    *(volatile unsigned char xdata *) 0x2F05       // PORTCON      R/W 8'b0000_0000    P5 Pull-up Register
#define     P5OD    *(volatile unsigned char xdata *) 0x2F11       // PORTCON      R/W 8'b0000_0000    P5 Open Drain Register
#define     P5DB    *(volatile unsigned char xdata *) 0x2F1D       // PORTCON      R/W 8'b0000_0000    P5 DEBOUNCE Register
													
#define     P6PU    *(volatile unsigned char xdata *) 0x2F06       // PORTCON      R/W 8'b0000_0000    P6 Pull-up Register
#define     P6OD    *(volatile unsigned char xdata *) 0x2F12       // PORTCON      R/W 8'b0000_0000    P6 Open Drain Register
#define     P6DB    *(volatile unsigned char xdata *) 0x2F1E       // PORTCON      R/W 8'b0000_0000    P6 DEBOUNCE Register
														  
#define     P7PU    *(volatile unsigned char xdata *) 0x2F07       // PORTCON      R/W 8'b0000_0000    P7 Pull-up Register
#define     P7OD    *(volatile unsigned char xdata *) 0x2F13       // PORTCON      R/W 8'b0000_0000    P7 Open Drain Register
#define     P7DB    *(volatile unsigned char xdata *) 0x2F1F       // PORTCON      R/W 8'b0000_0000    P7 DEBOUNCE Register

//------------------------------------------------------------------------------
// USART
//------------------------------------------------------------------------------
sfr     UCTRL01     =       0xE2;       // USART0       R/W 8'b0000_0000    USART Control Register 1
sfr     UCTRL02     =       0xE3;       // USART0       R/W 8'b0000_0000    USART Control Register 2
sfr     UCTRL03     =       0xE4;       // USART0       R/W 8'b0000_0000    USART Control Register 3
sfr     USTAT0      =       0xE5;       // USART0       R/W 8'b1000_0000    USART Status Register
sfr     UBAUD0      =       0xE6;       // USART0       R/W 8'b1111_1111    USART BaudRate Register
sfr     UDATA0      =       0xE7;       // USART0       R/W 8'b0000_0000    USART Data Register
								
sfr     UCTRL11     =       0xFA;       // USART1       R/W 8'b0000_0000    USART Control Register 1
sfr     UCTRL12     =       0xFB;       // USART1       R/W 8'b0000_0000    USART Control Register 2
sfr     UCTRL13     =       0xFC;       // USART1       R/W 8'b0000_0000    USART Control Register 3
sfr     USTAT1      =       0xFD;       // USART1       R/W 8'b1000_0000    USART Status Register
sfr     UBAUD1      =       0xFE;       // USART1       R/W 8'b1111_1111    USART BaudRate Register
sfr     UDATA1      =       0xFF;       // USART1       R/W 8'b0000_0000    USART Data Register

//------------------------------------------------------------------------------
// SYSCON
//------------------------------------------------------------------------------
sfr     BODR        =       0x86;       // BOD          R/W  8'b0000_0000    BOD Control Register
sfr     PCON        =       0x87;       // PCON         W    8'b0000_0000    Stop and Sleep Mode Control Register
sfr     SCCR        =       0x8A;       // CLKGEN       R/W  8'b0000_0100    System Clock Control Register
sfr     BCCR        =       0x8B;       // BIT          R/W  8'b0000_0111    BIT Clock Control Register
sfr     BITR        =       0x8C;       // BIT          R    8'b0000_0000    Clock Control Register
// sfr     PLLR        =       0xD9;       // PLL          R/W  8'b0000_0000    System Clock Control Register
sfr     PLLCR        =       0xD9;       // PLL          R/W  8'b0000_0000    System Clock Control Register

//------------------------------------------------------------------------------
// WDT
//------------------------------------------------------------------------------
sfr     WDTR        =       0x8E;       // WDT          W    8'b1111_1111    Watch Dog Timer Register
sfr     WDTMR       =       0x8D;       // WDT          W    8'b000-_---0    Watch Dog Timer Mode Register

//------------------------------------------------------------------------------
// WT
//------------------------------------------------------------------------------
sfr     WTR         =       0x9E;       // WT           W    8'b1111_1111    Watch Timer Register
sfr     WTMR        =       0x9D;       // WT           R/W  8'b0--0_0000    Watch Timer Mode Register

//------------------------------------------------------------------------------
// BUZZER
//------------------------------------------------------------------------------
sfr     BUZDR        =       0x8F;       // BUZZER       W    8'b1111_1111    Buzzer Data Register
sfr     BUZCR        =       0x9F;       // BUZZER       W    8'b0000_0000    Buzzer Control Register

//------------------------------------------------------------------------------
// INTCON
//------------------------------------------------------------------------------
sfr     EIENAB      =       0xA3;       // PORTCON      R/W  8'b----_0000    External Interrupt Enable Register
sfr     EIFLAG      =       0xA4;       // PORTCON      R/W  8'b----_0000    External Interrupt Flag Register
sfr     EIEDGE      =       0xA5;       // PORTCON      R/W  8'b----_0000    External Interrupt Edge Register
sfr     EIPOLA      =       0xA6;       // PORTCON      R/W  8'b----_0000    External Interrupt Polarity Register
sfr     EIBOTH      =       0xA7;       // PORTCON      R/W  8'b----_0000    External Interrupt Both Edge Register

sfr     IP        =       0xB8; // Interrupt Priority Register 

sfr     IP0       =       0xB8; // Interrupt Priority Register 

sfr     IP1       =       0xF8; // Interrupt Priority Register 1


sfr     IE          =       0xA8;       // INTCON       R/W  8'b0000_0000    Interrupt Enable Register Low
sbit    EA          =       0xAF;


sfr     IE0        =       0xA8;       // INTCON       R/W  8'b0000_0000    Interrupt Enable Register Low
sfr     IE1        =       0xA9;       // INTCON       R/W  8'b0000_0000    Interrupt Enable Register Low
sfr     IE2        =       0xAA;       // INTCON       R/W  8'b0000_0000    Interrupt Enable Register High
sfr     IE3        =       0xAB;       // INTCON       R/W  8'b0000_0000    Interrupt Enable Register High
sfr     IE4        =       0xAC;       // INTCON       R/W  8'b0000_0000    Interrupt Enable Register High


sfr     IEN0        =       0xA8;       // INTCON       R/W  8'b0000_0000    Interrupt Enable Register Low
sfr     IEN1        =       0xA9;       // INTCON       R/W  8'b0000_0000    Interrupt Enable Register Low
sfr     IEN2        =       0xAA;       // INTCON       R/W  8'b0000_0000    Interrupt Enable Register High
sfr     IEN3        =       0xAB;       // INTCON       R/W  8'b0000_0000    Interrupt Enable Register High
sfr     IEN4        =       0xAC;       // INTCON       R/W  8'b0000_0000    Interrupt Enable Register High
// sfr     IEN5        =       0xAD;       // INTCON       R/W  8'b0000_0000    Interrupt Enable Register High
								
// sfr     PCI0        =       0xAE;       // PORTCON      R/W 8'b0000_0000    P0 Pin Change Interrupt 
// sfr     PCI7        =       0xAF;       // PORTCON      R/W 8'b0000_0000    P0 Pin Change Interrupt 

//------------------------------------------------------------------------------
// TIMER
//------------------------------------------------------------------------------
// TIMER 0
sfr     T0CR      =       0xB2; // Timer 0 Mode Control Register
sfr     T0        =       0xB3; // Timer 0 Register
sfr     T0DR      =       0xB3; // Timer 0 Data Register
sfr     CDR0      =       0xB3; // Timer 0 Capture Data Register

// TIMER 1
sfr     T1CR      =       0xB4; // Timer 1 Mode Control Register
sfr     T1DR      =       0xB5; // Timer 1 Data Register
sfr     T1PPR     =       0xB5; // Timer 1 PWM Period Register
sfr     T1        =       0xB6; // Timer 1 Register
sfr     T1PDR     =       0xB6; // Timer 1 PWM Duty Register
sfr     CDR1      =       0xB6; // Timer 1 Capture Data Register
sfr     T1PWHR    =       0xB7; // Timer 1 PWM Control Register

// TIMER 2
sfr     T2CR      =       0xBA; // Timer 2 Mode Control Register
sfr     T2CR1     =       0xBB; // Timer 2 Mode Control Register 1
sfr     T2L       =       0xBC; // Timer 2 Register Low
sfr     PWM2LDR   =       0xBC; // Timer 2 PWM Duty Low Register
sfr     CDR2L     =       0xBC; // Timer 2 Capture Data Low Register
sfr     T2H       =       0xBD; // Timer 2 Register High
sfr     PWM2HDR   =       0xBD; // Timer 2 PWM Duty High Register
sfr     CDR2H     =       0xBD; // Timer 2 Capture Data High Register
sfr     T2DRL     =       0xBE; // Timer 2 Data Low Register
sfr     PWM2LPR   =       0xBE; // Timer 2 PWM Period Low Register
sfr     T2DRH     =       0xBF; // Timer 2 Data High Register
sfr     PWM2HPR   =       0xBF; // Timer 2 PWM Period Low Register

// TIMER 3
sfr     T3CR      =       0xC2; // Timer 3 Mode Control Register
sfr     T3CR1     =       0xC3; // Timer 3 Mode Control Register 1
sfr     T3L       =       0xC4; // Timer 3 Register Low
sfr     PWM3LDR   =       0xC4; // Timer 3 PWM Duty Low Register
sfr     CDR3L     =       0xC4; // Timer 3 Capture Data Low Register
sfr     T3H       =       0xC5; // Timer 3 Register High
sfr     PWM3HDR   =       0xC5; // Timer 3 PWM Duty High Register
sfr     CDR3H     =       0xC5; // Timer 3 Capture Data High Register
sfr     T3DRL     =       0xC6; // Timer 3 Data Low Register
sfr     PWM3LPR   =       0xC6; // Timer 3 PWM Period Low Register
sfr     T3DRH     =       0xC7; // Timer 3 Data High Register
sfr     PWM3HPR   =       0xC7; // Timer 3 PWM Period Low Register

// TIMER 4
sfr     T4CR      =       0xCA; // Timer 4 Mode Control Register
sfr     T4CR1     =       0xCB; // Timer 4 Mode Control Register 1
sfr     T4L       =       0xCC; // Timer 4 Register Low
sfr     PWM4LDR   =       0xCC; // Timer 4 PWM Duty Low Register
sfr     CDR4L     =       0xCC; // Timer 4 Capture Data Low Register
sfr     T4H       =       0xCD; // Timer 4 Register High
sfr     PWM4HDR   =       0xCD; // Timer 4 PWM Duty High Register
sfr     CDR4H     =       0xCD; // Timer 4 Capture Data High Register
sfr     T4DRL     =       0xCE; // Timer 4 Data Low Register
sfr     PWM4LPR   =       0xCE; // Timer 4 PWM Period Low Register
sfr     T4DRH     =       0xCF; // Timer 4 Data High Register
sfr     PWM4HPR   =       0xCF; // Timer 4 PWM Period Low Register
sfr     TMISR     =       0xD5; // Timer Interrupt Status Register

//------------------------------------------------------------------------------
// SPI
//------------------------------------------------------------------------------
sfr     SPICR0     =       0xD2;       // SPI0         R/W  8'b0000_0000     SPI0 control register
sfr     SPIDR0     =       0xD3;       // SPI0         R/W  8'b0000_0000     SPI0 data register
sfr     SPISR0     =       0xD4;       // SPI0         R/W  8'b0000_0000     SPI0 status register

//------------------------------------------------------------------------------
// I2C
//------------------------------------------------------------------------------
sfr     I2CMR      =       0xDA;        // I2C Mode Control Register
sfr     I2CCR      =       0xDA;        // I2C          R/W  8'b0000_0000     I2C Control Register
sfr     I2CSR      =       0xDB;        // I2C          R/W  8'b0000_0000     I2C Status Register
sfr     I2CSCLLR   =       0xDC;        // I2C          R/W  8'b0011_1111     I2C SCL Low Period Register
sfr     I2CSCLHR   =       0xDD;        // I2C          R/W  8'b0011_1111     I2C SCL High Period Register
sfr     I2CSDAHR   =       0xDE;        // I2C          R/W  8'b0000_0001     I2C SDA Hold Register
sfr     I2CDR      =       0xDF;        // I2C          R/W  8'b0000_0000     I2C Data Register
sfr     I2CSAR     =       0xD7;        // I2C          R/W  8'b0000_0000     I2C Slave Address Register
sfr     I2CSAR1    =       0xD6;        // I2C          R/W  8'b0000_0000     I2C Slave Address Register1

#define     I2CMR1      *(volatile unsigned char xdata *) 0x2F20    // I2C          R/W  8'b0000_0000     I2C Control Register
#define     I2CCR1      *(volatile unsigned char xdata *) 0x2F20    // I2C          R/W  8'b0000_0000     I2C Control Register
#define     I2CSR1      *(volatile unsigned char xdata *) 0x2F21    // I2C          R/W  8'b0000_0000     I2C Status Register
#define     I2CSCLLR1   *(volatile unsigned char xdata *) 0x2F22    // I2C          R/W  8'b0011_1111     I2C SCL Low Period Register
#define     I2CSCLHR1   *(volatile unsigned char xdata *) 0x2F23    // I2C          R/W  8'b0011_1111     I2C SCL High Period Register
#define     I2CSDAHR1   *(volatile unsigned char xdata *) 0x2F24    // I2C          R/W  8'b0000_0001     I2C SDA Hold Register
#define     I2CDR1      *(volatile unsigned char xdata *) 0x2F25    // I2C          R/W  8'b0000_0000     I2C Data Register
#define     I2CSAR10    *(volatile unsigned char xdata *) 0x2F26    // I2C          R/W  8'b0000_0000     I2C Slave Address Register
#define     I2CSAR11    *(volatile unsigned char xdata *) 0x2F27    // I2C          R/W  8'b0000_0000     I2C Slave Address Register1

//------------------------------------------------------------------------------
// CEC
//------------------------------------------------------------------------------
#define     CEC_PRES0   *(volatile unsigned char xdata *)  0x2F60  // CEC          R/W  8'b0000_0000     CEC Prescaler Low Register
#define     CEC_PRES1   *(volatile unsigned char xdata *)  0x2F61  // CEC          R/W  8'b0000_0000     CEC Prescaler High Register
#define     CEC_CONF0   *(volatile unsigned char xdata *)  0x2F62  // CEC          R/W  8'b0000_0000     CEC Configuration 0 Register
#define     CEC_CONF1   *(volatile unsigned char xdata *)  0x2F63  // CEC          R/W  8'b0000_0000     CEC Configuration 1 Register

#define     CEC_GCTRL   *(volatile unsigned char xdata *)  0x2F64   //

#define     CEC_ICTRL   *(volatile unsigned char xdata *)  0x2F65  // CEC          R/W  8'b0000_0000     CEC Initiator Control Register
#define     CEC_FCTRL   *(volatile unsigned char xdata *)  0x2F66  // CEC          R/W  8'b0000_0000     CEC Follower Control Register
#define     CEC_ISTAT   *(volatile unsigned char xdata *)  0x2F67  // CEC          R/W  8'b0000_0000     CEC Initiator Status Register
#define     CEC_FSTAT   *(volatile unsigned char xdata *)  0x2F68  // CEC          R/W  8'b0000_0000     CEC Follower Status Register
#define     CEC_ICLR    *(volatile unsigned char xdata *)  0x2F69  // CEC          R/W  8'b0000_0000     CEC Initiator Status Clear Register
#define     CEC_FCLR    *(volatile unsigned char xdata *)  0x2F6A  // CEC          R/W  8'b0000_0000     CEC Follower Status Clear Register

#define     CEC_TXH     *(volatile unsigned char xdata *)  0x2F6B  // CEC          R/W  8'b0000_0000     CEC TX Header Block Buffer Register
#define     CEC_TXD_1   *(volatile unsigned char xdata *)  0x2F6C  // CEC          R/W  8'b0000_0000     CEC TX Data Block 1 Buffer Register
#define     CEC_TXD_2   *(volatile unsigned char xdata *)  0x2F6D  // CEC          R/W  8'b0000_0000     CEC TX Data Block 2 Buffer Register
#define     CEC_TXD_3   *(volatile unsigned char xdata *)  0x2F6E  // CEC          R/W  8'b0000_0000     CEC TX Data Block 3 Buffer Register
#define     CEC_TXD_4   *(volatile unsigned char xdata *)  0x2F6F  // CEC          R/W  8'b0000_0000     CEC TX Data Block 4 Buffer Register
#define     CEC_TXD_5   *(volatile unsigned char xdata *)  0x2F70  // CEC          R/W  8'b0000_0000     CEC TX Data Block 5 Buffer Register
#define     CEC_TXD_6   *(volatile unsigned char xdata *)  0x2F71  // CEC          R/W  8'b0000_0000     CEC TX Data Block 6 Buffer Register
#define     CEC_TXD_7   *(volatile unsigned char xdata *)  0x2F72  // CEC          R/W  8'b0000_0000     CEC TX Data Block 7 Buffer Register
#define     CEC_TXD_8   *(volatile unsigned char xdata *)  0x2F73  // CEC          R/W  8'b0000_0000     CEC TX Data Block 8 Buffer Register
#define     CEC_TXD_9   *(volatile unsigned char xdata *)  0x2F74  // CEC          R/W  8'b0000_0000     CEC TX Data Block 9 Buffer Register
#define     CEC_TXD_10  *(volatile unsigned char xdata *)  0x2F75  // CEC          R/W  8'b0000_0000     CEC TX Data Block 10 Buffer Register
#define     CEC_TXD_11  *(volatile unsigned char xdata *)  0x2F76  // CEC          R/W  8'b0000_0000     CEC TX Data Block 11 Buffer Register
#define     CEC_TXD_12  *(volatile unsigned char xdata *)  0x2F77  // CEC          R/W  8'b0000_0000     CEC TX Data Block 12 Buffer Register
#define     CEC_TXD_13  *(volatile unsigned char xdata *)  0x2F78  // CEC          R/W  8'b0000_0000     CEC TX Data Block 13 Buffer Register
#define     CEC_TXD_14  *(volatile unsigned char xdata *)  0x2F79  // CEC          R/W  8'b0000_0000     CEC TX Data Block 14 Buffer Register
#define     CEC_TXD_15  *(volatile unsigned char xdata *)  0x2F7A  // CEC          R/W  8'b0000_0000     CEC TX Data Block 15 Buffer Register
#define     CEC_TXD_16  *(volatile unsigned char xdata *)  0x2F7B  // CEC          R/W  8'b0000_0000     CEC TX Data Block 16 Buffer Register
#define     CEC_TXD_17  *(volatile unsigned char xdata *)  0x2F7C  // CEC          R/W  8'b0000_0000     CEC TX Data Block 17 Buffer Register
#define     CEC_TXD_18  *(volatile unsigned char xdata *)  0x2F7D  // CEC          R/W  8'b0000_0000     CEC TX Data Block 18 Buffer Register
#define     CEC_TXD_19  *(volatile unsigned char xdata *)  0x2F7E  // CEC          R/W  8'b0000_0000     CEC TX Data Block 19 Buffer Register

#define     CEC_RXH     *(volatile unsigned char xdata *)  0x2F7F  // CEC          R/W  8'bXXXX_XXXX     CEC RX Header Block Buffer Register
#define     CEC_RXD_1   *(volatile unsigned char xdata *)  0x2F80  // CEC          R/W  8'bXXXX_XXXX     CEC RX Data Block 1 Buffer Register
#define     CEC_RXD_2   *(volatile unsigned char xdata *)  0x2F81  // CEC          R/W  8'bXXXX_XXXX     CEC RX Data Block 2 Buffer Register
#define     CEC_RXD_3   *(volatile unsigned char xdata *)  0x2F82  // CEC          R/W  8'bXXXX_XXXX     CEC RX Data Block 3 Buffer Register
#define     CEC_RXD_4   *(volatile unsigned char xdata *)  0x2F83  // CEC          R/W  8'bXXXX_XXXX     CEC RX Data Block 4 Buffer Register
#define     CEC_RXD_5   *(volatile unsigned char xdata *)  0x2F84  // CEC          R/W  8'bXXXX_XXXX     CEC RX Data Block 5 Buffer Register
#define     CEC_RXD_6   *(volatile unsigned char xdata *)  0x2F85  // CEC          R/W  8'bXXXX_XXXX     CEC RX Data Block 6 Buffer Register
#define     CEC_RXD_7   *(volatile unsigned char xdata *)  0x2F86  // CEC          R/W  8'bXXXX_XXXX     CEC RX Data Block 7 Buffer Register
#define     CEC_RXD_8   *(volatile unsigned char xdata *)  0x2F87  // CEC          R/W  8'bXXXX_XXXX     CEC RX Data Block 8 Buffer Register
#define     CEC_RXD_9   *(volatile unsigned char xdata *)  0x2F88  // CEC          R/W  8'bXXXX_XXXX     CEC RX Data Block 9 Buffer Register
#define     CEC_RXD_10  *(volatile unsigned char xdata *)  0x2F89  // CEC          R/W  8'bXXXX_XXXX     CEC RX Data Block 10 Buffer Register
#define     CEC_RXD_11  *(volatile unsigned char xdata *)  0x2F8A  // CEC          R/W  8'bXXXX_XXXX     CEC RX Data Block 11 Buffer Register
#define     CEC_RXD_12  *(volatile unsigned char xdata *)  0x2F8B  // CEC          R/W  8'bXXXX_XXXX     CEC RX Data Block 12 Buffer Register
#define     CEC_RXD_13  *(volatile unsigned char xdata *)  0x2F8C  // CEC          R/W  8'bXXXX_XXXX     CEC RX Data Block 13 Buffer Register
#define     CEC_RXD_14  *(volatile unsigned char xdata *)  0x2F8D  // CEC          R/W  8'bXXXX_XXXX     CEC RX Data Block 14 Buffer Register
#define     CEC_RXD_15  *(volatile unsigned char xdata *)  0x2F8E  // CEC          R/W  8'bXXXX_XXXX     CEC RX Data Block 15 Buffer Register
#define     CEC_RXD_16  *(volatile unsigned char xdata *)  0x2F8F  // CEC          R/W  8'bXXXX_XXXX     CEC RX Data Block 16 Buffer Register
#define     CEC_RXD_17  *(volatile unsigned char xdata *)  0x2F90  // CEC          R/W  8'bXXXX_XXXX     CEC RX Data Block 17 Buffer Register
#define     CEC_RXD_18  *(volatile unsigned char xdata *)  0x2F91  // CEC          R/W  8'bXXXX_XXXX     CEC RX Data Block 18 Buffer Register
#define     CEC_RXD_19  *(volatile unsigned char xdata *)  0x2F92  // CEC          R/W  8'bXXXX_XXXX     CEC RX Data Block 19 Buffer Register

//------------------------------------------------------------------------------
// IRCON
//------------------------------------------------------------------------------
#define     IRC_PRES0   *(volatile unsigned char xdata *) 0x2F48    // IRCON        R/W  8'b0000_0000     IRCON Pescaler Low Register
#define     IRC_PRES1   *(volatile unsigned char xdata *) 0x2F49    // IRCON        R/W  8'b0000_0000     IRCON Pescaler Low Register
#define     IRC_FRMP0   *(volatile unsigned char xdata *) 0x2F4A    // IRCON        R/W  8'b0000_0000     IRCON Frame Period Parameter Low Register
#define     IRC_FTD0    *(volatile unsigned char xdata *) 0x2F4A    // IRCON        R/W  8'b0000_0000     IRCON Frame Period Parameter Low Register
#define     IRC_FRMP1   *(volatile unsigned char xdata *) 0x2F4B    // IRCON        R/W  8'b0000_0000     IRCON Frame Period Parameter High Register
#define     IRC_FTD1    *(volatile unsigned char xdata *) 0x2F4B    // IRCON        R/W  8'b0000_0000     IRCON Frame Period Parameter High Register
#define     IRC_CONF    *(volatile unsigned char xdata *) 0x2F4C    // IRCON        R/W  8'b0000_0000     IRCON Configuration Register
// #define     IRC_CTRL    *(volatile unsigned char xdata *) 0x2F4D    // IRCON        R/W  8'b0000_0000     IRCON Control Register
#define     IRC_FLAG    *(volatile unsigned char xdata *) 0x2F4D    // IRCON        R/W  8'b0000_0000     IRCON Control Register
#define     IRC_EDGE0   *(volatile unsigned char xdata *) 0x2F4E    // IRCON        R/W  8'b0000_0000     IRCON Edge Counter Low Register
#define     IRC_ECD0    *(volatile unsigned char xdata *) 0x2F4E    // IRCON        R/W  8'b0000_0000     IRCON Edge Counter Low Register
#define     IRC_EDGE1   *(volatile unsigned char xdata *) 0x2F4F    // IRCON        R/W  8'b0000_0000     IRCON Edge Counter High Register
#define     IRC_ECD1    *(volatile unsigned char xdata *) 0x2F4F    // IRCON        R/W  8'b0000_0000     IRCON Edge Counter High Register

//------------------------------------------------------------------------------
// EXTLVI
//------------------------------------------------------------------------------
sfr     EXTLVIC     =       0xF9;       // EXTLVI       R/W  8'b0000_0000     External low voltage interrupt controller

//------------------------------------------------------------------------------
// ADC
//------------------------------------------------------------------------------
sfr     ADCM        =       0x9A;       // ADC          R/W  8'b0000_0000     8 bit A/D Converter Control Register
sfr     ADCM2       =       0x9B;       // ADC          R    8'b----_----     8 bit A/D Converter Control 2 Register
sfr     ADCR        =       0x9C;       // ADC          R    8'b----_----     8 bit A/D Converter Result Register
sfr     ADCSH       =       0x94;       // ADC          R    8'b0000_0010     8 bit A/D Converter Sample Hold Time Register

//------------------------------------------------------------------------------
// PSR
//------------------------------------------------------------------------------
#define PSR 		*(volatile unsigned char xdata *) 0x2F50       // PSR          R/W  8'b0000_0000     Port Select Register

//------------------------------------------------------------------------------
// Flash and EEPROM Memory REGISTER
//------------------------------------------------------------------------------
sfr FEMR          =       0xEA;             // Flash and EEPROM mode register
sfr FESR          =       0xEB;             // Flash and EEPROM status register
sfr FETCR         =       0xEC;             // Flash and EEPROM time control register
sfr FETR          =       0xED;             // Flash and EEPROM test register
sfr FEARH         =       0xF2;             // Flash and EEPROM address register high
sfr FEARM         =       0xF3;             // Flash and EEPROM address register middle
sfr FEARL         =       0xF4;             // Flash and EEPROM address register low
sfr FEDR          =       0xF5;             // Flash and EEPROM data register
sfr FECR          =       0xF6;             // Flash and EEPROM control register

//------------------------------------------------------------------------------
// TEST MODE REGISTER
//------------------------------------------------------------------------------
#define TEST_REGA	*(volatile unsigned char xdata *) 0x2F5F       // TEST_MODE    W    8'b0000_0000     Function Test Reg. A
#define TEST_REGB	*(volatile unsigned char xdata *) 0x2F5E       // TEST_MODE    W    8'b0000_0000     Function Test Reg. B
#define FUSE_CONF	*(volatile unsigned char xdata *) 0x2F5D       // TEST_MODE    R/W  8'b0000_0000     FUSE Config setting.
#define FUSE_CAL0	*(volatile unsigned char xdata *) 0x2F5C       // TEST_MODE    R/W  8'b0000_0000     FUSE VDC_BOD calibration.
#define FUSE_CAL1	*(volatile unsigned char xdata *) 0x2F5B       // TEST_MODE    R/W  8'b0000_0000     FUSE INTOSC calibration.
#define FUSE_CAL2	*(volatile unsigned char xdata *) 0x2F5A       // TEST_MODE    R/W  8'b0000_0000     FUSE VDC calibration.
#define FUSE_PKGx	*(volatile unsigned char xdata *) 0x2F59       // TEST_MODE    R/W  8'b0000_0000     FUSE pin PKG setting.
#define SystemClock14_75Mhz	   1
#define PLL_SystemClock	       1
#define TEST_PLL_Monitoring	   1
//------------------------------------------------------------------------------
#endif
