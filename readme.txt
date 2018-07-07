Readme
------
Project Title: Light Intensity Display using CAN communication

IDE used for this project: Code Composer studio

Resources required
------------------
 --------------------------------------------------------------------------------------------------------
 * Components            							Quantity in numbers
 * ------------------------------------------------------------------------------------------------------
 * Tiva C Series TM4C123G LaunchPad Evaluation Boards					2                         
 * BoosterPack MKII									1                         
 * Microchip MCP2551 CAN transceiver 							2                       
 * 120-ohm resistors									2                        
 * 1Kohm resistors									2   
 * Bread board and jumper wires							   As per need
 * Laptop with CCS installed							1 if it has two USB ports
 * Different kinds of light sources with various intensity levels		   As per need                   
 ---------------------------------------------------------------------------------------------------------
					
Connections
-----------

Part 1
   ------------------------------------------------------------------------
 * Transmitter Tiva board         connected to      BoosterPack MKII board
 * ------------------------------------------------------------------------
 * PB2(I2C0 Clock)                      		Pin 9(I2C_SCL)
 * PB3(I2C0 Data)                       		Pin 10(I2C_SDA)
 * Vcc(3.3V)                            		Pin 1(Vcc)
 * GND                                  		Pin 20(GND)
 * ------------------------------------------------------------------------

Part 2

1. Interconnect the CAN_H and CAN_L pins of both the transceivers using 120 ohm resistors.
2. For both Tiva boards connect PB4 to the RX pin and PB5 to the TX pin of their respective MCP2551s.
3. Connect the VBUS pins of both the Tiva board to VDD of their respective MCP2551s.
4. Connect the VSS pins of both transceivers to ground
5. Pull down the RS pin of both the MCP2551s to the ground using a 1Kohm resistors.
Please refer the attached final report to see the circuit connections for CAN communication between transmitter and receiver Tiva boards. 

Part 3
 -----------------------------------------------------------------------------
 * Receiver Tiva board      connected to         BoosterPack MKII board
 * ---------------------------------------------------------------------------
 * PA3(SSI0Fss)                         		Pin 13(LCDSPI_CS)
 * PA7(GPIO)                            		Pin 17(LCDRST)
 * PA2(SSI0Clk)                         		Pin 7(LCDSPI_CLK)
 * PA5(SSI0Tx)                          		Pin 15(LCDSPI_MOSI)
 * PA6(GPIO)                            		Pin 31(LCD_RS)
 * ---------------------------------------------------------------------------


Steps to compile and demonstrate
--------------------------------
1. Create a new project namely transmitter
2. Copy the transmitter.c to the main program of this project
3. Include the Tiva C series folder TivaWare_C_Series-2.1.4.178 in the ARM Compiler->Include Options of the Project. Also include the Tiva driver library and sensor library in the File Search Path for the ARM Linker. 
4. Register the interrupt handlers in the interrupt vector table of the project which is present in tm4c123gh6pm_startup_ccs.c 
5. Build the project transmitter
6. Create another new project namely receiver
7. Copy the receiver.c to the main program of this project
8. Copy the files ST7735.h, PLL.h, ST7735.c and PLL.c from the ValvanoWareTM4C123/ST7735_4C123 folder to the project.
9. Include the Tiva C series folder TivaWare_C_Series-2.1.4.178 and ValvanoWareTM4C123\driverlib\ccs\Debug in the ARM Compiler->Include Options of the Project. Also include the Tiva driver library and ValvanoWareTM4C123 driver library in the File Search Path for the ARM Linker. 
10. Register the interrupt handlers in the interrupt vector table of the project which is present in tm4c123gh6pm_startup_ccs.c
11. Build the project receiver
12. Do the connections mentioned in the Connections section above
13. Power up the receiver Tiva board and execute the receiver program in it.
14. Open a CCS terminal
15. Open a serial terminal and run the program
16. Stop the debugging and put the receiver board in device mode
17. Power up the transmitter Tiva board using the same laptop. 
18. Go to debug mode for the transmitter program using the transmitter Tiva board
19. Open another serial terminal for the transmitter board connected now.
20. Add the u16Ambient variable to the watch window of expressions
21. Power up the receiver board
22. Run the transmitter program
23. Monitor both the serial terminals for transmitter and receiver
24. Also monitor the change in the values for u16Ambient in the expressions window by varying the light intensity using different kinds of lights
25. Check the LCD display has the same intensity value as the variable u16Ambient
26. Also check the LEDs on the transmitter Tiva board are blinking and the LED on the receiver board is turned on always.



Hints
-----
1. Make sure the receiver board is powered up before executing the program from transmitter Tiva board. Otherwise cable connected error will be displayed on the terminal.
2. Make sure to pull down the RS pin of the transceivers to ground using 1k resistors.
3. If you face any issues during the experiment https://e2e.ti.com/support/microcontrollers/tiva_arm/f/908/t/433816#pi239031350=2 is good thread to follow.
