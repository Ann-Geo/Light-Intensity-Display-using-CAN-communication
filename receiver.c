/*****************************************************************************************************************************************
 * File name: receiver.c
 * Program Objective: To understand how to utilize the LCD display in the Educational BoosterPack MKII board and CAN peripheral in Tiva C
 * Series TM4C123G LaunchPad.
 * Objectives[1]: 1. The code must be written in C using Code Composer Studio
 *                2. The light intensity value must get updated continuously in the transmitter Tiva board before sending it to the
 *                receiver.
                  3. The data read from the light sensor must be converted to a 16-bit integer in lux before transmitting it through the CAN
                  bus.
                  4. The data from the light sensor should be read in every 2 seconds.
                  5. The 16-bit value of light intensity must be transmitted to the receiver Tiva board by the transmitter Tiva board using
                  CAN interface in every 2 seconds.
                  6. The receiver Tiva board should receive 2 bytes of data transmitted from the transmitter board and convert it into
                  16-bit integer.
                  7. The LCD screen connected to the receiver Tiva board should display the intensity of the light in the format light
                  intensity: XXXX.
 * Description : Two TM4C123G LaunchPads and the Educational BoosterPack MKII are utilized in this experiment. The light sensor and the
 * LCD display are on the same Boosterpack. If you have two BoosterPacks you can take input from the light sensor from the first BoosterPack
 * and output to the LCD display in the second BoosterPack. This program is for the functionality to be implemented on the receiver side
 * and is run on the receiver Tiva board. The light sensor on the MKII board senses the ambient light intensity and sends the intensity
 * value to the transmitter Tiva board using I2C interface. The transmitter Tiva board sends the intensity value to the receiver Tiva board
 * using CAN communication interface. The receiver Tiva board displays the light intensity on the LCD screen of BoosterPack.
 * Circuit descriptions and connections: The following connections are made for the SPI communication between receiver Tiva board and
 * BoosterPack MKII board
 * ------------------------------------------------------------
 * Tiva board                           BoosterPack MKII board
 * ------------------------------------------------------------
 * PA3(SSI0Fss)                         Pin 13(LCDSPI_CS)
 * PA7(GPIO)                            Pin 17(LCDRST)
 * PA2(SSI0Clk)                         Pin 7(LCDSPI_CLK)
 * PA5(SSI0Tx)                          Pin 15(LCDSPI_MOSI)
 * PA6(GPIO)                            Pin 31(LCD_RS)
 * -------------------------------------------------------------
 * Other connections: Two MCP2551 CAN transceivers are used in this lab to provide the differential transmit and receive capability for
 * the CAN controllers in the Tiva board. The CAN_H and CAN_L pins of both the transceivers are interconnected using 120 ohm resistors.
 * PB4 is the CAN_Rx pin and PB5 is CAN_Tx pin. PB4 is connected to the RX pin of MCP2551 and PB5 is connected to the TX pin of MCP2551
 * The MCP2551 requires a 5V supply to operate. This can be achieved by connecting the VBUS pin of Tiva board to VDD of MCP2551. The RS
 * pin of the MCP2551 is pull down to the ground using a 1Kohm resistor. Please refer the final report to see the circuit connections for
 * CAN communication between transmitter and receiver Tiva boards.
 * References:  [1]Embedded System Design using TM4C LaunchPadTM Development Kit,SSQU015(Canvas file)
 *              [2]Tiva C Series TM4C123G LaunchPad Evaluation Board User's Guide
 *              [3]Tiva TM4C123GH6PM Microcontroller datasheet
 *              [4]TivaWare Peripheral Driver Library User guide
 *              [5]http://processors.wiki.ti.com/index.php/Tiva_TM4C123G_LaunchPad_Blink_the_RGB
 *              [6]Embedded Systems An Introduction using the Renesas RX63N Microcontroller BY JAMES M. CONRAD
 *              [7]CFAF128128B-0145TDatasheetReleaseDate2017-06-28.pdf
 *              [8]ST7735_V2.1_20100505.pdf
 *              [9]CFAF128128B-0145T_Data_Sheet_2014-05-07.pdf
 *              [10]TivaWare Graphics Library User's Guide
 *              [11]Meet the Educational BoosterPack MKII, Part number: BoostXL-EDUMKII
 *              [12]BOOSTXL-EDUMKII Educational BoosterPack Plug-in Module Mark II, User's guide
 *              [13]http://users.ece.utexas.edu/~valvano/
 *              [14]https://www.youtube.com/watch?v=MWIX7wgS9PM&feature=youtu.be
 *              [15]https://e2e.ti.com/support/microcontrollers/msp430/f/166/t/453698
 *              [16]http://edx-org-utaustinx.s3.amazonaws.com/UT601x/ValvanoWareTM4C123.zip
 *              [17]https://forum.crystalfontz.com/showthread.php/7394-Connecting-the-small-CFAF128128B-0145T-
 *                  TFT-to-an-Arduino-Uno-or-SparkFun-RedBoard
 *              [18]https://dev.ti.com/bpchecker/#/
 *              [19]http://ohm.ninja/tiva-c-series-can-bus-with-mcp2551/
 *              [20]http://www.ti.com/product/opt3001
 *              [21]http://www.ti.com/lit/ds/symlink/opt3001.pdf
 *              [22]https://eewiki.net/display/microcontroller/I2C+Communication+with+the+TI+Tiva+TM4C123GXL
 *              [23]https://e2e.ti.com/support/microcontrollers/tiva_arm/f/908/t/433816
 * TI provided code is used in this program and the copyright goes to,
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *********************************************************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "ST7735.h"
#include "PLL.h"
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "utils/uartstdio.c"

/*Variable to store the received CAN message count*/
volatile uint32_t g_ui32MsgCount = 0;

/*These are flag variables to store the message receive and error status*/
volatile bool g_bRXFlag = 0;
volatile bool g_bErrFlag = 0;
unsigned int ui32LightIntensity;

/***************************************************************************************************************************
 * Function: InitConsole()
 * Inputs: None
 * Outputs: None
 * Description: Initializes the UART0 peripheral for displaying error checking messages and information regarding the
 * data received from the transmitter Tiva board. This is not required for the experiment completion, but will help programmers
 * to understand the program flow and check error messages.
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 ****************************************************************************************************************************/

void InitConsole(void)
{
    /*Enable GPIO port A peripheral*/
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

        /*Configure Port pins A0 and A1 as UART0 Rx and Tx*/
        GPIOPinConfigure(GPIO_PA0_U0RX);
        GPIOPinConfigure(GPIO_PA1_U0TX);

        /*Enable UART0 peripheral*/
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

        /*USe the internal 16MHz oscillator PIOSC as the clock source*/
        UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

       /*Select alternate functions for PA0 and PA1 so that they act as UART0 Rx and Tx*/
        GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

       /*Initialize UART0 console I/O with baud rate 115200*/
        UARTStdioConfig(0, 115200, 16000000);
}

/***************************************************************************************************************************
 * Function: CANIntHandler()
 * Inputs: None
 * Outputs: None
 * Description: Interrupt handler for CAN peripheral. This checks the interrupt status of the controller. If cause of the
 * interrupt is not a controller status interrupt then receive flag is set and error flag is cleared.
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 ****************************************************************************************************************************/

void CANIntHandler(void)
{
    /*Variable stores the CAN interrupt status*/
    uint32_t ui32Status;

    /*Reads the CAN interrupt status to find the cause of the interrupt*/
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    /*If the cause is a controller status interrupt, this gets its status.CANStatusGet function returns error bits
     * indicating various errors.If the CAN bus is not properly connected then this error will occur.*/
    if(ui32Status == CAN_INT_INTID_STATUS)
    {
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
        /*Flag indicates that error has occurred during reception of the message data*/
        g_bErrFlag = 1;
    }

    /*If the cause of the interrupt id message reception then ui32Status will be set to 1*/
    else if(ui32Status == 1)
    {
        /*Clears the interrupt*/
        CANIntClear(CAN0_BASE, 1);

        /*Counter that keeps track of the number of messages received*/
        g_ui32MsgCount++;

        /*Blue LED is turn on here. This is not part of the requirements and is implemented for error checking to make
         * sure that message has been received*/
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

        /*Sets the message receive flag to show the messages in UART terminal*/
        g_bRXFlag = 1;

        /*Clears the error flag since message is received*/
        g_bErrFlag = 0;
    }
}


/***************************************************************************************************************************
 * Function: main
 * Inputs: None
 * Outputs: None
 * Description: This function initializes the CAN0 peripheral. The CAN bit rate is set as 500KHz. The two byte value received
 * from the transmitter is converted to a 16-bit integer before sending it to the LCD screen. The light intensity is displayed
 * on the LCD screen in the format light intensity:XXXX
 * Reference for APIs: TivaWare Peripheral Driver Library User guide
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 ***************************************************************************************************************************/


int main(void)
{

    /*Enables the GPIO port F. This is to indicate the transmission of I2C data. Not required to
     * complete the experiment*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    /*Turns off the blue LED initially. This LED is used to indicate the reception of CAN message*/
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);

    /*Instance that stores the CAN message object received*/
    tCANMsgObject sCANMessage;

    /*This stores the bytes of the message received. Here the transmitted message is 2 bytes(16 bit integer)*/
    uint8_t pui8MsgData[4];

    /*Set the clock frequency to 16MHz*/
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    InitConsole();

    /*Enables GPIO port B*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    /*Configures the pins PB4 and PB5 as CAN Rx and CAN Tx*/
    GPIOPinConfigure(GPIO_PB4_CAN0RX);
    GPIOPinConfigure(GPIO_PB5_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    /*CAN peripheral initialization and configuration*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

    /*Initializes CAN bus by setting bit rate to 500kHz. Here the clock frequency is already set to 16MHz
     * The SysCtlClockGet() function gets the clock frequency set by using SysCtlClockSet() function*/
    CANInit(CAN0_BASE);
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 500000);

    /*CAN_INT_MASTER allows the CAN controller to generate CAN interrupts, CAN_INT_ERROR allows CAN controller
     * to generate error interrupts, CAN_INT_STATUS allows CAN controller to generate status interrupts*/
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    /*Enables CAN 0 interrupt and the CAN0 module*/
    IntEnable(INT_CAN0);
    CANEnable(CAN0_BASE);

    /*Initialize a message object to be used for receiving CAN messages with any CAN ID.  In order to receive
     * any CAN ID, the ID and mask must both be set to 0, and the ID filter enabled.*/

    sCANMessage.ui32MsgID = 0;
    sCANMessage.ui32MsgIDMask = 0;
    sCANMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    sCANMessage.ui32MsgLen = 4;
    CANMessageSet(CAN0_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_RX);


    for(;;)
    {
        unsigned int uIdx;

        /*If the flag is set that means receive interrupt has occurred and CAN message is ready*/
        if(g_bRXFlag)
        {
            /*CAN message is received using the CAN message object initialized earlier. pui8MsgData is
             * message buffer that stores the received message*/
            sCANMessage.pui8MsgData = pui8MsgData;

            /*Reads the message from CAN. Message object number 1 is used. The interrupt is already cleared
             * in the interrupt handler*/
            CANMessageGet(CAN0_BASE, 1, &sCANMessage, 0);

            /*Clears the message receive flag*/
            g_bRXFlag = 0;

            /*Prints the received message contents in the UART console with the total message count*/
            UARTprintf("Msg ID=0x%08X len=%u data=0x", sCANMessage.ui32MsgID, sCANMessage.ui32MsgLen);
            for(uIdx = 0; uIdx < 2; uIdx++)
            {
                UARTprintf("%02X ", pui8MsgData[uIdx]);

            }
            UARTprintf("total count=%u\n", g_ui32MsgCount);
        }

        /*The received light intensity is in the form of bytes. The MSB byte is received in the array
         * element pui8MsgData[1]. So left shift it by 8 bits and LSB byte is received in the array
         * element pui8MsgData[0]. OR MSB and LSB to obtain the actual intensity*/
        ui32LightIntensity = (unsigned int)pui8MsgData[1]<<8 | (unsigned int)pui8MsgData[0];

        /*[18]Initialization for ST7735R screens (green or red tabs). Input: option one of the enumerated options
         * depending on tabs. The copyright of this function goes to:
         * Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu*/
        ST7735_InitR(INITR_REDTAB);

        /*This function prints the given string on the LCD screen. This is to print the light intensity after
         * two lines of the screen. The copyright of this function goes to:
         * Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu*/
        ST7735_OutString("\n\n");

        /*Prints the light intensity in the format light intensity: XXXX. The copyright of this function goes to:
         * Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu*/
        ST7735_OutString("light intensity:");

        /*Prints the light intensity as decimal integer on the LCD screen. The copyright of this function goes to:
         * Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu*/
        ST7735_OutUDec(ui32LightIntensity);
    }
}
