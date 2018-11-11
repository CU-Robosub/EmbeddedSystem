#ifndef GPIO_H_
#define GPIO_H_

#define LOW_VOLTAGE    11528  // ADC reading of 2.33V = Battery voltage 14V
#define HIGH_VOLTAGE   12410  // ADC reading of 2.5V  = Battery voltage 15V

void gpioConfiguration();

void setLEDColor(uint16_t battV);

#endif /* GPIO_H_ */
