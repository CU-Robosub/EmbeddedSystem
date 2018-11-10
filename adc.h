#ifndef ADC_H_
#define ADC_H_

// Define constants
#define ADC_TO_DIFFV         0.010073
// Read Voltage = ADC Value * ADC resolution = MEM[x] * 0.20146 mV
// Sense resistor voltage drop (Differential Voltage) = Read Voltage / 20 = MEM[x] * 0.010073
// Sense resistor current = Differential Voltage / Sense resistance = Line Current

void adcConfiguration();

void startReadMotorCurrents();

void startReadPowerStatus();

#endif /* ADC_H_ */
