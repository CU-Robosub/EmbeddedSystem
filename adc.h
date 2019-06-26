#ifndef ADC_H_
#define ADC_H_


#include "queue.h"
#include "msp.h"
#include "uart.h"
#include "main.h"


// Define constants
#define ADC_TO_DIFFV         0.010073
// Read Voltage = ADC Value * ADC resolution = MEM[x] * 0.20146 mV
// Sense resistor voltage drop (Differential Voltage) = Read Voltage / 20 = MEM[x] * 0.010073
// Sense resistor current = Differential Voltage / Sense resistance = Line Current


// Declare global variables
extern volatile queue_t* eventList;


/**********************************************************************
 * FUNCTION NAME:       uartCompConfigure
 * FUNCTION PURPOSE:    Configures the ADC to 32 cycle sample and
 *                      hold time, 14-bit resolution, and sequence mode
 *                      for reading. Sets up input channels and end of
 *                      sequence channels.
 * INPUTS:
 *  -None
 * OUTPUTS:
 *  -None
 *********************************************************************/
void adcConfigure();


#endif /* ADC_H_ */
