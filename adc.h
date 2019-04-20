#ifndef ADC_H_
#define ADC_H_

// Define constants
#define ADC_TO_DIFFV         0.010073
// Read Voltage = ADC Value * ADC resolution = MEM[x] * 0.20146 mV
// Sense resistor voltage drop (Differential Voltage) = Read Voltage / 20 = MEM[x] * 0.010073
// Sense resistor current = Differential Voltage / Sense resistance = Line Current


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


/**********************************************************************
 * FUNCTION NAME:       startReadMotorCurrents
 * FUNCTION PURPOSE:    Begins an ADC conversion on the sequence of
 *                      channels corresponding to the motors.
 * INPUTS:
 *  -None
 * OUTPUTS:
 *  -None
 *********************************************************************/
void startReadMotorCurrents();


/**********************************************************************
 * FUNCTION NAME:       startReadPowerStatus
 * FUNCTION PURPOSE:    Begins an ADC conversion on the sequence of
 *                      channels corresponding to the current sensing
 *                      and battery voltage channels.
 * INPUTS:
 *  -None
 * OUTPUTS:
 *  -None
 *********************************************************************/
void startReadPowerStatus();

#endif /* ADC_H_ */
