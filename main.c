// Include all headers
#include "msp.h"
#include "adc.h"
#include "i2c.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"
#include "main.h"


void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// Stop watchdog timer
	adcConfiguration();
	uartCompConfigure();
	uartAtmelConfigure();
	gpioConfiguration();
	timerConfiguration();
	startReadMotorCurrents();                       // Begin first motor ADC conversion

	while(1)
	{
	    // Literally nothing
	}
}
