#include "msp.h"
#include "adc.h"
#include "i2c.h"
#include "uart.h"
#include "gpio.h"

/*
 * uart.c
 *
 *  Created on: Nov 10, 2018
 *      Author: Nathan
 */

void uart_comp_configure(){
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST;         // Put eUSCI in reset
    P1SEL0 |= (BIT2 | BIT3);                        // TX & Rx Primary mode
    P1SEL1 &= ~(BIT2 | BIT3);

    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK;   // use SMCLK
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_MODE0;        // uart mode
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_MODE1;
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_PEN;          // disable parity bit
    EUSCI_A0->CTLW0 &= ~EUSCI_B_CTLW0_MSB;          // LSB first
    EUSCI_A0->CTLW0 &= ~EUSCI_B_CTLW0_SEVENBIT;     // 8-bit data
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SPB;          // 1 stop bit

    // setting baud rate to 115200
    // set over sampling mode based on frequency of SMCLK
    EUSCI_A0->MCTLW |= EUSCI_A_MCTLW_OS16;
    EUSCI_A0->BRW   |= 0x13;                        // set baud
    EUSCI_A0->MCTLW |= 0xA0;                        // set BRF
    EUSCI_A0->MCTLW |= 0xB500;                      // set BRS

    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;        // Initialize eUSCI
}

// transmits a byte of data to serial terminal
void uart_send_comp_byte(uint8_t data) {
    while(!(EUSCI_A0->IFG & BIT1)); // block until transmitter is ready
    EUSCI_A0->TXBUF = data; // load data into buffer
}

// transmits a string of data to serial terminal
void uart_send_comp_n(uint8_t * data, uint32_t length) {
    uint32_t i = 0;
    for(i=0; i < length; i++){
        uart_send_comp_byte(data[i]); // send data byte by byte
    }
}

