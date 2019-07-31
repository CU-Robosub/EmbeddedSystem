#ifndef TIMER_H_
#define TIMER_H_


#include "queue.h"
#include "adc.h"
#include "msp.h"
#include "stdint.h"
#include "main.h"

/*** Motor PWM Signals ****
 *    TA1.1 = Motor 1 = P7.7
 *    TA1.2 = Motor 2 = P7.6
 *    TA1.3 = Motor 3 = P7.5
 *    TA1.4 = Motor 4 = P7.4
 *    TA2.1 = Motor 5 = P5.6
 *    TA2.2 = Motor 6 = P5.7
 *    TA2.3 = Motor 7 = P6.6
 *    TA2.4 = Motor 8 = P6.7
 *************************/

// Defines for interrupts counts (Interrupt frequency = 600 Hz)
#define INT_COUNT_60HZ  10      // Read depth at 60 Hz
#define INT_COUNT_4HZ   150     // Read motor currents at 4 Hz
#define INT_COUNT_1HZ   600     // Interrupt at a 1 Hz rate

void timerConfigurePnumatics(void);

/**********************************************************************
 * FUNCTION NAME:       timerConfigure
 * FUNCTION PURPOSE:    Configures all required timer peripherals to
 *                      control interrupt calls and PWM signals.
 * INPUTS:
 *  -None
 * OUTPUTS:
 *  -None
 *********************************************************************/
void timerConfigure(void);

#endif /* TIMER_H_ */
