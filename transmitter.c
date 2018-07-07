/*****************************************************************************************************************************************
 * File name: transmitter.c
 * Program Objective: To understand how to utilize the light sensor in the Educational BoosterPack MKII board and CAN peripheral in Tiva C
 * Series TM4C123G LaunchPad.
 * Objectives   : 1. The code must be written in C using Code Composer Studio
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
 * and output to the LCD display in the second BoosterPack. This program is for the functionality to be implemented on the transmitter side
 * and is run on the transmitter Tiva board. The light sensor on the MKII board senses the ambient light intensity and sends the intensity
 * value to the transmitter Tiva board using I2C interface. The transmitter Tiva board sends the intensity value to the receiver Tiva board
 * using CAN communication interface. The receiver Tiva board displays the light intensity on the LCD screen of BoosterPack.
 * Circuit descriptions and connections: The following connections are made for the I2C communication between transmitter Tiva board and
 * BoosterPack MKII board
 * ------------------------------------------------------------
 * Tiva board        connected to       BoosterPack MKII board
 * ------------------------------------------------------------
 * PB2(I2C0 Clock)                      Pin 9(I2C_SCL)
 * PB3(I2C0 Data)                       Pin 10(I2C_SDA)
 * Vcc(3.3V)                            Pin 1(Vcc)
 * GND                                  Pin 20(GND)
 * -------------------------------------------------------------
 * Other connections: Two MCP2551 CAN transceivers are used in this project to provide the differential transmit and receive capability for
 * the CAN controllers in the Tiva boards. The CAN_H and CAN_L pins of both the transceivers are interconnected using 120 ohm resistors.
 * PB4 is the CAN_Rx pin and PB5 is CAN_Tx pin of CAN0 module. PB4 is connected to the RX pin of MCP2551 and PB5 is connected to the TX pin
 * of MCP2551. The MCP2551 requires a 5V supply to operate. This can be achieved by connecting the VBUS pin of Tiva board to VDD of MCP2551.
 * The Vss pin of the MCP2551 is connected to ground.The RS pin of the MCP2551 is pull down to the ground using a 1Kohm resistor. Please
 * refer the attached final report to see the circuit connections for CAN communication between transmitter and receiver Tiva boards.
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

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_can.h"
#include "driverlib/can.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "sensorlib/i2cm_drv.h"
#include "utils/uartstdio.h"
#include "utils/uartstdio.c"

/*address of the OPT3001from the Educational BoosterPack schematic*/
#define OPT3001_I2C_ADDRESS      0x44

/*variables store the I2C configuration instance, the data ready and error flags
 * and resulting light reading.*/
tI2CMInstance g_sI2CInst;
volatile uint_fast8_t g_vui8DataFlag;
volatile uint_fast8_t g_vui8ErrorFlag;
volatile uint16_t ui16Ambient;
volatile uint32_t g_ui32MsgCount = 0;
volatile bool g_bErrFlag = 0;

/***************************************************************************************************************************
 * Function: InitConsole()
 * Inputs: None
 * Outputs: None
 * Description: Initializes the UART0 peripheral for displaying error checking messages and information regarding the
 * data sent from the transmitter Tiva board. This is not required for the experiment completion, but will help programmers
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
 * Function: SimpleDelay()
 * Inputs: None
 * Outputs: None
 * Description: Provides a delay of 1 second. One second delay: Clock is configured as 16MHz. The SysCtlDelay(delay) function
 * will create a delay for one second. The formula to calculate the delay is, delay = DelayTimeInSeconds/((1/16000000)*3).
 * Here DelayTimeInSeconds = 1 second.
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 ****************************************************************************************************************************/
void SimpleDelay(void)
{
    SysCtlDelay(16000000 / 3);
}

/***************************************************************************************************************************
 * Function: CANIntHandler()
 * Inputs: None
 * Outputs: None
 * Description: Interrupt handler for CAN peripheral. This checks the interrupt status of the controller. If cause of the
 * interrupt is not a controller status interrupt then it clears the error flag and increments the message count.
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

        /*Flag indicates that error has occurred during transmission of the message data*/
        g_bErrFlag = 1;
    }

    /*If the cause of the interrupt id message transmission then ui32Status will be set to 1*/
    else if(ui32Status == 1)
    {
        /*Clears the interrupt*/
        CANIntClear(CAN0_BASE, 1);

        /*Counter that keeps track of the number of messages sent*/
        g_ui32MsgCount++;

        /*Clears the error flag since message is transmitted*/
        g_bErrFlag = 0;
    }
}

/***************************************************************************************************************************
 * Function: OPT3001AppCallback
 * Inputs: void *pvCallbackData, uint_fast8_t ui8Status
 * Outputs: None
 * Description: This is the OPT3001 Application Callback function which is a a blocking function since it will wait for the
 * write to complete before setting the data and error flags. This function is also available from the sensor library as a
 * non-blocking call.
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 ***************************************************************************************************************************/

void OPT3001AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    /*If the I2C data transfer is complete this sets the data flag to 1 and indicates that data is ready. */
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8DataFlag = 1;
    }

    /*Stores the most recent status in case of an error condition*/
    g_vui8ErrorFlag = ui8Status;
}

/***************************************************************************************************************************
 * Function: OPT3001AppErrorHandler
 * Inputs: char *pcFilename, uint_fast32_t ui32Line
 * Outputs: None
 * Description: This is the OPT3001 Application Error Handler function. If an error occurs and error flag is set this function
 * gets called and the program spin wait in an infinite loop.
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 ***************************************************************************************************************************/

void OPT3001AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line)
{
    while(1)
    {
    }
}

/***************************************************************************************************************************
 * Function: OPT3001I2CIntHandler
 * Inputs: None
 * Outputs: None
 * Description: This is the the interrupt handler for I2C0 peripheral. This calls the sensor library's built in interrupt
 * handler.
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 ***************************************************************************************************************************/

void OPT3001I2CIntHandler(void)
{
    I2CMIntHandler(&g_sI2CInst);
}

/***************************************************************************************************************************
 * Function: main
 * Inputs: None
 * Outputs: None
 * Description: This function initializes the I2C0 and CAN0 peripherals. The CAN bit rate is set as 500KHz. The 16-bit light
 * intensity value sent from the sensor has its upper 8 bits as exponent and the lower 12 bits as mantissa. It is converted to
 * a 16-bit integer using the formula, lux = 0.01*2^(exponent)*mantissa. Finally this two byte value is sent to the receiver
 * Tiva board as CAN messages.
 * Reference for APIs: TivaWare Peripheral Driver Library User guide
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 ***************************************************************************************************************************/

int main(void)
{
    /*CAN message object*/
    tCANMsgObject sCANMessage;

    /*16 bit message data transmitted from CAN peripheral*/
    uint16_t ui16MsgData;

    /*Pointer to 16 bit message data*/
    uint8_t *pui8MsgData;

    /*make the pointer point to the 16 bit message data*/
    pui8MsgData = (uint8_t *)&ui16MsgData;

    /*Variables that help to extract the digital data from the I2C data received*/
    uint16_t ui16Result;
    uint16_t ui16Exponent;
    uint8_t ui8RegisterOne;
    uint8_t ui8RegisterZero;

    /*Data to be read from the light sensor's I2C module*/
    uint8_t pui8Data[2];

    /*Commands to be send to the I2C module of the light sensor*/
    uint8_t pui8Command[3];

    /*Set the clock to 16MHz*/
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    InitConsole();

    /*I2C0 peripheral initialization*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    /*Enables GPIO port B*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    /*Configures PB3 as I2C SDA and PB2 as I2C SCL*/
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);

    /*Enables the GPIO port F. This is to indicate the transmission of I2C data. Not required to
     * complete the experiment*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    /*Turns off the red and blue LEDs initially. These LEDs are used to indicate the transmission of I2C data*/
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2, 0x00);


    /*CAN peripheral initialization and configuration*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

    /*Configures the pins PB4 and PB5 as CAN Rx and CAN Tx*/
    GPIOPinConfigure(GPIO_PB4_CAN0RX);
    GPIOPinConfigure(GPIO_PB5_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

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

    /*Initialize the message object that is used to send the CAN message.*/
    ui16MsgData = 0;
    sCANMessage.ui32MsgID = 1;
    sCANMessage.ui32MsgIDMask = 0;
    sCANMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    sCANMessage.ui32MsgLen = sizeof(pui8MsgData);
    sCANMessage.pui8MsgData = pui8MsgData;

    /*Enables master interrupt*/
    IntMasterEnable();

    /*I2CMInit() prepares the I2C port and driver for operation. Selects the I2C instance, I2C0, the
     * interrupt number, the TX and RX DMA channels (0xFF means OFF) and the clock frequency.*/
    I2CMInit(&g_sI2CInst, I2C0_BASE, INT_I2C0, 0xff, 0xff, 16000000);

    /*Initializes the commands to be sent to I2C module of the OPT3001 sensor*/
    /*register to be written.*/
    pui8Command[0] = 1;

    /*auto range, 800ms per conversion, continuous mode.*/
    pui8Command[1] = 0xCC;

    /*latch mode on*/
    pui8Command[2] = 0x10;

    /*Writes the commands to the I2C module of the OPT3001 which has address OPT3001_I2C_ADDRESS*/
    I2CMWrite(&g_sI2CInst, OPT3001_I2C_ADDRESS, pui8Command, 3, OPT3001AppCallback, 0);

    /*Wait for the OPT3001 to signal that data is ready*/
    while((g_vui8DataFlag == 0) && (g_vui8ErrorFlag == 0))
    {
    }

    ui8RegisterOne = 1;
    ui8RegisterZero = 0;

    while(1)
    {
        /*Gives a delay of 0.1 second because 800ms delay is allocated per conversion for I2C module*/
        SysCtlDelay(533333);

        /*Turns off the LEDs used for debugging*/
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2, 0x00);

        while(1)
        {
            /*Gives a delay of 0.1 second because 800ms delay is allocated per conversion for I2C module.
             * A delay of 0.2 second will allow the I2C to sample every one second*/
            SysCtlDelay(533333);

            /*Spin waits in a while loop until the data is read*/
            I2CMRead(&g_sI2CInst, OPT3001_I2C_ADDRESS, &ui8RegisterOne, 1, pui8Command, 2, OPT3001AppCallback, 0);
            while((g_vui8DataFlag == 0) && (g_vui8ErrorFlag == 0))
            {
            }

            /*Call the error handler if an error occurs*/
            if(g_vui8ErrorFlag)
            {
                OPT3001AppErrorHandler(__FILE__, __LINE__);
            }
            if(pui8Command[1] & 0x80)
            {
                 break;
            }

        }

        /*Spin waits in a while loop until the data is read*/
        I2CMRead(&g_sI2CInst, OPT3001_I2C_ADDRESS, &ui8RegisterZero, 1, pui8Data, 2, OPT3001AppCallback, 0);
        while((g_vui8DataFlag == 0) && (g_vui8ErrorFlag == 0))
        {
        }

        /*Call the error handler if an error occurs*/
        if(g_vui8ErrorFlag)
        {
            OPT3001AppErrorHandler(__FILE__, __LINE__);
        }

        /*Resets the flag*/
        g_vui8DataFlag = 0;

        /*The following steps are done to format the data read from the I2C module. Upper 8 bits of data are in pui8Data[0]
         * and lower 8 bits of data are in pui8Data[1]. This packs the data into a 16 bit integer.*/
        ui16Result = pui8Data[0];
        ui16Result <<= 8;
        ui16Result |= pui8Data[1];

        /*The upper 4-bits of the data are the exponent and the lower 12-bits are the mantissa. In order to get a single 16-bit
         * integer result we need scaling of the mantissa is required. Since large intensity lights like lasers are not used for
         * testing this scaling will give accurate result for most of the small intensity light values. ui16Result contains the
         * formatted data value in lux and is saved to the global variable ui16Ambient*/
        ui16Exponent = (ui16Result >> 12) & 0x000F;
        ui16Result = ui16Result & 0x0FFF;

        /*conversion raw readings to LUX*/
        switch(ui16Exponent)
        {
                case 0: //*0.015625
                    ui16Result = ui16Result>>6;
                    break;
                case 1: //*0.03125
                    ui16Result = ui16Result>>5;
                    break;
                case 2: //*0.0625
                    ui16Result = ui16Result>>4;
                    break;
                case 3: //*0.125
                    ui16Result = ui16Result>>3;
                    break;
                case 4: //*0.25
                    ui16Result = ui16Result>>2;
                    break;
                case 5: //*0.5
                    ui16Result = ui16Result>>1;
                    break;
                case 6:
                    ui16Result = ui16Result;
                    break;
                case 7: //*2
                    ui16Result = ui16Result<<1;
                    break;
                case 8: //*4
                    ui16Result = ui16Result<<2;
                    break;
                case 9: //*8
                    ui16Result = ui16Result<<3;
                    break;
                case 10: //*16
                    ui16Result = ui16Result<<4;
                    break;
                case 11: //*32
                    ui16Result = ui16Result<<5;
                    break;
        }

        /*Turns on the red and blue LEDs to indicate the successful sensor read*/
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_1 | GPIO_PIN_2);
        ui16Ambient = ui16Result;

        /*Sends the 16 bit message to the receiver using CAN0 module*/
        UARTprintf("Sending msg: 0x%02X %02X", pui8MsgData[0], pui8MsgData[1]);
        CANMessageSet(CAN0_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_TX);

        /*Provides a delay of 1 second*/
        SimpleDelay();

        /*Prints error message in the console*/
        if(g_bErrFlag)
        {
            UARTprintf(" error - cable connected?\n");
        }
        else
        {
            /*Prints the total transmitted message count in the console*/
            UARTprintf(" total count = %u\n", g_ui32MsgCount);
        }

        /*Saves the ambient light intensity value in lux to the CAN message data to be sent*/
        ui16MsgData = ui16Ambient;

    }
}
