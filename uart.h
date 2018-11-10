#ifndef UART_H_
#define UART_H_
//9600 for atmel UCA1 p2.3 p2.2
//115200 for comp UCA0 p1.2 p1.3

//configure uart communication with computer
void uartCompConfigure();

//send a byte to the computer
void uartSendCompByte(uint8_t data);

//send an array of a specific length
void uartSendCompN(uint8_t * data, uint32_t length);

void uartAtmelConfigure();

void uartSendAtmelByte(uint8_t data);

void uartSendAtmelN(uint8_t * data, uint32_t length);

#endif /* UART_H_ */
