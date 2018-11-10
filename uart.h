/*
 * uart.h
 *
 *  Created on: Nov 10, 2018
 *      Author: Nathan
 */

#ifndef UART_H_
#define UART_H_
//9600 for atmel UCA1 p2.3 p2.2
//115200 for comp UCA0 p1.2 p1.3

//configure uart communication with computer
void uart_comp_configure();

//send a byte to the computer
void uart_send_comp_byte(uint8_t data);

//send an array of a specific length
void uart_send_comp_n(uint8_t * data, uint32_t length);


#endif /* UART_H_ */
