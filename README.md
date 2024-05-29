# nRF24L01_TxRx_Bluepill
 
This project interfaces 2 nRF24L01+ transceivers using the STM32F103C8 microcontroller to send 2 bytes of data. Specific configurations to the transceiver can be changed using user defined functions but this project uses predefined conditions to the transceivers as an example. I will work on these user-defined functions maybe later.

## Functions designed:
### void Receive();
This function configures the parameters of the receiver and scans the surroundings for data packets
### void Transmit(uint8_t data[]);
This function configures the parameters of the transmitter and sends the data contained in the array 'data'

Other user-defined functions (setup functions) defined in the code was for test purposes only.

