#ifndef GPIO_H_
#define GPIO_H_

#include "msp.h"
#include "main.h"

#define LOW_VOLTAGE    11528  // ADC reading of 2.33V = Battery voltage 14V
#define HIGH_VOLTAGE   12410  // ADC reading of 2.5V  = Battery voltage 15V


/**********************************************************************
 * FUNCTION NAME:       gpioConfigure
 * FUNCTION PURPOSE:    Configures all GPIO pins for communication,
 *                      ADC, and IO to assure proper functionality.
 * INPUTS:
 *  -None
 * OUTPUTS:
 *  -None
 *********************************************************************/
void gpioConfigure();


/**********************************************************************
 * FUNCTION NAME:       setLEDColor
 * FUNCTION PURPOSE:    Sets the color of the LED strip based on the
 *                      measured battery voltage.
 * INPUTS:
 *  -uint16_t battV:    Measured batter voltage (raw ADC reading)
 * OUTPUTS:
 *  -None
 *********************************************************************/
void setLEDColor(uint16_t battV);

#endif /* GPIO_H_ */
