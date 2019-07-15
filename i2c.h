#ifndef I2C_H_
#define I2C_H_


#include "queue.h"
#include "adc.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"
#include "msp.h"
#include "stdint.h"
#include "main.h"


// Define frames for communication with depth sensor
#define DEPTH_SENSOR_ADDRESS            0x86    // Only 7 bits
#define DEPTH_SENSOR_RESET_COMMAND      0x1E
#define DEPTH_SENSOR_DEPTH_CONVERSION   0x40
#define DEPTH_SENSOR_ADC_READ           0x00


// Enumeration for states during depth sensor reading
enum depthStates
{
    I2C_READY = 0,
    ADDRESS_SENT_CONVERSION,
    CONVERSION_STARTED,
    ADDRESS_SENT_ADC_COMMAND,
    ADC_READ_STARTED,
    ADDRESS_SENT_ADC_READING,
    DEPTH_1_RECEIVED,
    DEPTH_2_RECEIVED
};


/**********************************************************************
 * FUNCTION NAME:       i2cConfigure
 * FUNCTION PURPOSE:    Initializes EUSCI_B1 to work in I2C mode using
 *                      SMCLK, 400KBPS data rate, and interrupts for
 *                      both TX and RX. Also initializes globals
 * INPUTS:
 *  -None
 * OUTPUTS:
 *  -None
 *********************************************************************/
void i2cConfigure(void);


#endif /* I2C_H_ */
