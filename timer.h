#ifndef TIMER_H_
#define TIMER_H_


#include "queue.h"
#include "msp.h"
#include "adc.h"
#include "main.h"

/*** Motor PWM Signals ****
 *    TA1.1 = Motor 1
 *    TA1.2 = Motor 2
 *    TA1.3 = Motor 3
 *    TA1.4 = Motor 4
 *    TA2.1 = Motor 5
 *    TA2.2 = Motor 6
 *    TA2.3 = Motor 7
 *    TA2.4 = Motor 8
 *************************/

// Defines for interrupts counts (Interrupt frequency = 60 Hz)
#define INT_COUNT_4HZ   15      // Read motor currents at 4 Hz
#define INT_COUNT_1HZ   60      // Interrupt at a 1 Hz rate


// Declare global variables
extern volatile queue_t* eventList;


/**********************************************************************
 * FUNCTION NAME:       timerConfigure
 * FUNCTION PURPOSE:    Configures all required timer peripherals to
 *                      control interrupt calls and PWM signals.
 * INPUTS:
 *  -None
 * OUTPUTS:
 *  -None
 *********************************************************************/
void timerConfigure();

#endif /* TIMER_H_ */
