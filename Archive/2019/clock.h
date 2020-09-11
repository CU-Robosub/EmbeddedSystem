#ifndef CLOCK_H_
#define CLOCK_H_


#include "msp.h"
#include "stdint.h"
#include "main.h"


/**************************CLOCK FREQUENCIES***************************
 * DCO Clock    = 48 MHz
 * MCLK Clock   = 48 MHz
 * SMCLK Clock  = 12 MHz
 * HSMCLK Clock = 48 MHz
 *********************************************************************/


/**********************************************************************
 * FUNCTION NAME:       clockConfigure
 * FUNCTION PURPOSE:    Configures DCOCLK, HSMCLK, and SMCLK speeds.
 * INPUTS:
 *  -None
 * OUTPUTS:
 *  -None
 *********************************************************************/
void clockConfigure(void);

#endif /* CLOCK_H_ */
