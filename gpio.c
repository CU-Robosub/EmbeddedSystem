#include "gpio.h"


void gpioConfigure(void)
{
    // Pins for eUSCI_A0 UART line (Computer)
    P1SEL0  |= BIT2 | BIT3;             // Set pins to primary mode
    P1SEL1  &= ~(BIT2 | BIT3);          // P1.2 = RX, P1.3 = TX

    // Pins for eUSCI_A1 UART line (Pneumatics Board)
    P2SEL0  |= BIT2 | BIT3;             // Set pins to primary mode
    P2SEL1  &= ~(BIT2 | BIT3);          // P2.2 = RX, P2.3 = TX

    // Pins for eUSCI_B0 I2C line (Depth Sensor)
    P1SEL0  |= BIT6 | BIT7;             // Set pins to primary mode
    P1SEL0  &= ~(BIT6 | BIT7);          // P1.6 = SDA, P1.7 = SCL

    // Pins for ADC connections (Current Sensing)
    P4SEL0  |= BIT0 | BIT1 | BIT2 | BIT4 | BIT5 | BIT6 | BIT7;  // Set pins to tertiary mode
    P4SEL1  |= BIT0 | BIT1 | BIT2 | BIT4 | BIT5 | BIT6 | BIT7;
    P5SEL0  |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5;
    P5SEL1  |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5;

    // Pins for PWM connections (Motor Controls)
    P5SEL0  |= BIT6 | BIT7;                                     // Set pins to primary mode
    P6SEL0  |= BIT6 | BIT7;
    P7SEL0  |= BIT4 | BIT5 | BIT6 | BIT7;
    P5SEL1  &= ~(BIT6 | BIT7);
    P6SEL1  &= ~(BIT6 | BIT7);
    P7SEL1  &= ~(BIT4 | BIT5 | BIT6 | BIT7);
    P5DIR   |= BIT6 | BIT7;                                     // Set pins to outputs
    P6DIR   |= BIT6 | BIT7;
    P7DIR   |= BIT4 | BIT5 | BIT6 | BIT7;

    // P2.4 = Red, P2.5 = Green, P2.6 = Blue
    P2SEL0  &= ~(BIT4 | BIT5 | BIT6);
    P2SEL1  &= ~(BIT4 | BIT5 | BIT6);  // Set pins to gpio mode
    P2DIR   |= BIT4 | BIT5 | BIT6;     // Set pins to outputs
    P2OUT   &= ~(BIT4 | BIT5 | BIT6);  // Assure pins are off
}


void setLEDColor(uint16_t battV)
{
    // If read voltage < 2.33V, battery voltage < 14V
    if(battV < LOW_VOLTAGE)
    {
        P2OUT &= ~(BIT4 | BIT5 | BIT6);  // Assure pins are off
        P2OUT |= BIT4;                   // Turn red LED on
    }
    // If read voltage > 2.5V, battery voltage > 15V
    else if(battV > HIGH_VOLTAGE)
    {
        P2OUT &= ~(BIT4 | BIT5 | BIT6);  // Assure pins are off
        P2OUT |= BIT5;                   // Turn green LED on
    }
    else
    {
        P2OUT &= ~(BIT4 | BIT5 | BIT6);  // Assure pins are off
        P2OUT |= BIT6;                   // Turn blue LED on
    }
}
