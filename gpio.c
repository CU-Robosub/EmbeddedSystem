#include "msp.h"
#include "adc.h"
#include "i2c.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"


void gpioConfiguration()
{
    // P2.4 = Red, P2.5 = Green, P2.6 = Blue
    P2SEL0 &= ~(BIT4 | BIT5 | BIT6);
    P2SEL1 &= ~(BIT4 | BIT5 | BIT6);  // Set pins to gpio mode
    P2DIR  |= BIT4 | BIT5 | BIT6;     // Set pins to outputs
    P2OUT  &= ~(BIT4 | BIT5 | BIT6);  // Assure pins are off
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
