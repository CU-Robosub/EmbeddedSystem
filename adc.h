/*
 * adc.h
 *
 *  Created on: Nov 10, 2018
 *      Author: Nathan
 */

#ifndef ADC_H_
#define ADC_H_

void adcConfiguration();

uint16_t* readMotorCurrents();

uint16_t* readPowerStatus();

#endif /* ADC_H_ */
