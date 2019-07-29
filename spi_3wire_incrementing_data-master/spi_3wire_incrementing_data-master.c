/*
 * -------------------------------------------
 *    MSP432 DriverLib - v3_21_00_05
 * -------------------------------------------
 *
 * --COPYRIGHT--,BSD,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/******************************************************************************
 * MSP432 SPI - 3-wire Master Incremented Data
 *
 * This example shows how SPI master talks to SPI slave using 3-wire mode.
 * Incrementing data is sent by the master starting at 0x01. Received data is
 * expected to be same as the previous transmission.  eUSCI RX ISR is used to
 * handle communication with the CPU, normally in LPM0. Because all execution
 * after LPM0 is in ISRs, initialization waits for DCO to stabilize against
 * ACLK.
 *
 * Note that in this example, EUSCIB0 is used for the SPI port. If the user
 * wants to use EUSCIA for SPI operation, they are able to with the same APIs
 * with the EUSCI_AX parameters.
 *
 * ACLK = ~32.768kHz, MCLK = SMCLK = DCO 3MHz
 *
 * Use with SPI Slave Data Echo code example.
 *
 *                MSP432P401
 *              -----------------
 *             |                 |
 *             |                 |
 *             |                 |
 *             |             P1.6|-> Data Out (UCB0SIMO)
 *             |                 |
 *             |             P1.7|<- Data In (UCB0SOMI)
 *             |                 |
 *             |             P1.5|-> Serial Clock Out (UCB0CLK)
 * Author: Timothy Logan
*******************************************************************************/
/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>

#include <stdbool.h>

/*
p1.6  SIMO
//p1.7  SOMI
p1.5  CLK

p2.4  RST
p2.5  CE
p2.6  DC
p2.7  BL
*/int delay_ms(int cnt)
{
    int conter = cnt;
    while (conter--) {
        for (int ii = 0; ii < 100; ii++);
    }
    return cnt;
}

typedef unsigned char alt_u8;
typedef unsigned short int alt_u16;

#define CMD	   0x00
#define DATA   0x01
#define LCD_X  84
#define LCD_Y  48

//write a byte
void LcdWrite(alt_u8 data,alt_u8 d_c);
//initial the 5110
void InitLcd(void);
//set the position of x axis and y axis
void SetXY(alt_u8 x,alt_u8 y);
//clear screen
void LcdClearAll(void);

void LcdWrite(alt_u8 data,alt_u8 d_c)
{
//	alt_u8 i;
	//chose the chip
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
//	IOWR_ALTERA_AVALON_PIO_DATA(SCE_BASE, 0x00);
	//chose data mode or command mode
    if (d_c)
       GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6);
    else
       GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6);
//	IOWR_ALTERA_AVALON_PIO_DATA(D_C_BASE, d_c);
	//sent 8 bits

    /* Polling to see if the TX buffer is ready */
    while (!(SPI_getInterruptStatus(EUSCI_B0_BASE,EUSCI_B_SPI_TRANSMIT_INTERRUPT)));
    /* Transmitting data to slave */
    SPI_transmitData(EUSCI_B0_BASE, data);
	//release the chip
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);
//	IOWR_ALTERA_AVALON_PIO_DATA(SCE_BASE, 0x01);
}
#if 1
const unsigned char si[504] = { /* 0X22,0X01,0X54,0X00,0X30,0X00, */
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XC0,0X20,0X10,0X10,0X08,0X08,0X04,
0X04,0X04,0X0C,0X12,0X22,0X22,0X42,0X62,0X1C,0X08,0X10,0X60,0X80,0X80,0X00,0X00,
0X00,0X00,0XC0,0X30,0X18,0X18,0X24,0X44,0X24,0X18,0X04,0X04,0X04,0X04,0X04,0X18,
0X24,0X44,0X24,0X18,0X30,0XC0,0XC0,0X00,0X00,0X00,0X00,0X80,0X60,0X10,0X08,0X08,
0X1C,0X62,0X42,0X22,0X12,0X0C,0X04,0X04,0X04,0X04,0X08,0X08,0X10,0X20,0XC0,0XC0,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X1F,0X60,0X80,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X03,0X80,0X40,
0X21,0X21,0X1E,0X00,0X00,0X3E,0X41,0X80,0X80,0X80,0X06,0X00,0X38,0X44,0X54,0X44,
0X54,0X44,0X44,0X38,0X00,0X06,0X00,0X80,0X80,0X41,0X41,0X3E,0X00,0X00,0X1E,0X21,
0X40,0X80,0X03,0X03,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X80,0X60,0X60,0X1F,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X81,0X81,0X72,0X0E,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X20,0X48,
0X50,0X61,0X46,0X98,0XE0,0XE0,0X00,0X00,0X00,0X18,0X26,0XF1,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XF1,0X26,0X26,0X18,
0X00,0X00,0X00,0XE0,0X98,0X46,0X61,0X61,0X50,0X48,0X20,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X0E,0X72,0X81,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X04,0X0B,0X0B,0X0C,0X14,0X20,0XC0,0X80,0X80,0X40,
0X40,0X40,0X40,0X40,0X40,0X80,0X80,0X7C,0X03,0X03,0X02,0X04,0X04,0X08,0X10,0X11,
0X26,0X26,0X58,0X50,0X48,0X84,0X84,0X84,0X84,0X84,0X84,0X84,0X48,0X50,0X58,0X26,
0X11,0X10,0X10,0X08,0X04,0X04,0X02,0X03,0X7C,0X80,0X80,0X80,0X40,0X40,0X40,0X40,
0X40,0X80,0X80,0X80,0XC0,0X20,0X14,0X0C,0X0B,0X04,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,};
#else
char si[] = {0xff,0xf0};
#endif

void test(int x, int y, const unsigned char *si, int n)
{
    alt_u16 i;
	SetXY(x,y);
	for(i = 0; i < n; i++)
	{
		LcdWrite(si[i], DATA);
	}
}

void InitLcd (void)
{
	delay_ms(1000);
	//sent a reset pulse(low) to reset all internal registers
//	IOWR_ALTERA_AVALON_PIO_DATA(REST_BASE, 0x00);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4);
	delay_ms(1);
//	IOWR_ALTERA_AVALON_PIO_DATA(REST_BASE, 0x01);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN4);
	delay_ms(1);
//	IOWR_ALTERA_AVALON_PIO_DATA(SCE_BASE, 0x00);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
	delay_ms(1);
//	IOWR_ALTERA_AVALON_PIO_DATA(SCE_BASE, 0x01);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);
	delay_ms(1);
	//function set PD = 0 and V = 0, select extended instruction set(H = 1mode)
	LcdWrite(0x21,CMD);
	//set VOP: VOP is set to a + 65 ¡Á b [V] = 3.06 + 65 * 0.06 = 6.96 V
	LcdWrite(0x98,CMD); //good
	//RECOMMENDED MUX RATE(1:24),you can chose other MUX RATE
	LcdWrite(0x17,CMD);
	//function set PD = 0 and V = 0, select normal instruction set(H = 0 mode)
	LcdWrite(0x20,CMD); //02
	//display control set normal mode (D=1andE=0)
	LcdWrite(0x0c,CMD); //00
	//clear screen
	LcdClearAll();
}

void SetXY(alt_u8 x,alt_u8 y)
{
	if(x > 83) x = 0;
	if(y > 5) y = 0;
	LcdWrite(0x40 | y,CMD);
	LcdWrite(0x80 | x,CMD);
}

void LcdClearAll(void)
{
	alt_u16 i;
	SetXY(0,0);
	for(i = 0;i < LCD_X * LCD_Y / 8;i ++)
	{
		LcdWrite(0,DATA);
	}
}

void LcdClearAll2(void)
{
	alt_u16 i;
	SetXY(0,0);
	for(i = 0;i < LCD_X * LCD_Y / 8;i ++)
	{
		LcdWrite(0xff,DATA);
	}
}

/* Statics */
static volatile uint8_t RXData = 0;
static uint8_t TXData = 0;

/* SPI Master Configuration Parameter */
const eUSCI_SPI_MasterConfig spiMasterConfig =
{
        EUSCI_B_SPI_CLOCKSOURCE_SMCLK,             // SMCLK Clock Source
        3000000,                                   // SMCLK = DCO = 3MHZ
        500000,                                    // SPICLK = 500khz
        EUSCI_B_SPI_MSB_FIRST,                     // MSB First
        EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,    // Phase
        EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH, // High polarity
        EUSCI_B_SPI_3PIN                           // 3Wire SPI Mode
};

int main(void)
{
    volatile uint32_t ii = 0;

    /* Halting WDT  */
    WDT_A_holdTimer();

    /* Selecting P1.5 P1.6 and P1.7 in SPI mode */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 | GPIO_PIN4);

/*
    while(1){
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        delay_ms(1000);
    }
*/
    /* Configuring SPI in 3wire master mode */
    SPI_initMaster(EUSCI_B0_BASE, &spiMasterConfig);

    /* Enable SPI module */
    SPI_enableModule(EUSCI_B0_BASE);

    /* Enabling interrupts */
//    SPI_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT);
//    Interrupt_enableInterrupt(INT_EUSCIB0);
//    Interrupt_enableSleepOnIsrExit();
    InitLcd();
    delay_ms(1000);
    test(0, 0, si, sizeof(si));
    while (true) {
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        delay_ms(1000);

//        TXData++;
//        /* Polling to see if the TX buffer is ready */
//        while (!(SPI_getInterruptStatus(EUSCI_B0_BASE,EUSCI_B_SPI_TRANSMIT_INTERRUPT)));
//        /* Transmitting data to slave */
//        SPI_transmitData(EUSCI_B0_BASE, TXData);
    }

    PCM_gotoLPM0();
    __no_operation();
}

//******************************************************************************
//
//This is the EUSCI_B0 interrupt vector service routine.
//
//******************************************************************************
void EUSCIB0_IRQHandler(void)
{
    uint32_t status = SPI_getEnabledInterruptStatus(EUSCI_B0_BASE);
    uint32_t jj;

    SPI_clearInterruptFlag(EUSCI_B0_BASE, status);

    if(status & EUSCI_B_SPI_RECEIVE_INTERRUPT)
    {
        /* USCI_B0 TX buffer ready? */
        while (!(SPI_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT)));

        RXData = SPI_receiveData(EUSCI_B0_BASE);

        /* Send the next data packet */
        SPI_transmitData(EUSCI_B0_BASE, ++TXData);

        /* Delay between transmissions for slave to process information */
        for(jj=50;jj<50;jj++);
    }

}
