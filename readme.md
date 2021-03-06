### Light Intensity Display Using CAN communication

##### Embedded Systems Project

  The objective of this project is to demonstrate CAN communication between two TIVA C series microcontrollers. Besides the TIVA boards, the proposed system consists of Educational Booster Pack MKII and Microchip MCP2551 CAN transceiver. The TI OPT3001 Light Sensor in the Booster Pack MKII is used to sense the intensity of the light signal. The digital output from the Booster Pack MKII is fed to the transmitter TIVA board using I2C interface. The transmitter TIVA board sends the digital signal to receiver TIVA board through CAN bus. To provide the differential transmit and receive capability for the CAN controller in the TIVA board, Microchip MCP2551 CAN transceiver is used. The receiver TIVA board displays the received signal from the CAN bus on the Color TFT LCD Display of the Booster Pack MKII.
