#ifndef TIMER_H_
#define TIMER_H_

#include "msp.h"
#include "adc.h"
#include "main.h"

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
