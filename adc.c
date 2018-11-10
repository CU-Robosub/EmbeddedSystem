#include "msp.h"
#include "adc.h"
#include "i2c.h"
#include "uart.h"
#include "gpio.h"

/*
 * adc.c
 *
 *  Created on: Nov 10, 2018
 *      Author: Nathan
 */

void adcConfiguration()
{
    ADC14->CTL0 &= ~ADC14_CTL0_ENC; // Disable ADC Encoding to allow changing settings
}

uint16_t* readMotorCurrents()
{

}

uint16_t* readPowerStatus()
{

}
