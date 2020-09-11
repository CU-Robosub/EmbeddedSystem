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
#define DEPTH_SENSOR_ADDRESS            0x76    // Only 7 bits
#define DEPTH_SENSOR_ADC_READ           0x00
#define DEPTH_SENSOR_RESET_COMMAND      0x1E
#define DEPTH_SENSOR_DEPTH_CONVERSION   0x48    // Depth conversion, OSR = 4096
#define DEPTH_SENSOR_TEMP_CONVERSION    0x58    // Temperature conversion, OSR = 4096
#define DEPTH_SENSOR_PROM_2_READ        0xA2
#define DEPTH_SENSOR_PROM_4_READ        0xA4
#define DEPTH_SENSOR_PROM_6_READ        0xA6
#define DEPTH_SENSOR_PROM_8_READ        0xA8
#define DEPTH_SENSOR_PROM_A_READ        0xAA
#define DEPTH_SENSOR_PROM_C_READ        0xAC


// Enumeration for states during depth sensor reading
enum depthStates
{
    I2C_READY = 0,
    DEPTH_CONVERSION_STARTED,
    DEPTH_READ_STARTED,
    DEPTH_READING,
    DEPTH_1_RECEIVED,
    DEPTH_2_RECEIVED,
    TEMP_CONVERSION_STARTED,
    TEMP_READ_STARTED,
    TEMP_READING,
    TEMP_1_RECEIVED,
    TEMP_2_RECEIVED,
    I2C_TRANSMITTING
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


/**********************************************************************
 * FUNCTION NAME:       i2cTransmit
 * FUNCTION PURPOSE:    Puts MSP in transmit mode and transmits slave
 *                      address and command
 * INPUTS:
 *  -uint8_t address:   7-bit address (aligned right) of slave device
 *  -uint8_t command:   Command to transmit to slave
 * OUTPUTS:
 *  -uint8_t            1 on success, 0 on failure
 *********************************************************************/
uint8_t i2cTransmit(uint8_t address, uint8_t command);


/**********************************************************************
 * FUNCTION NAME:       i2cReceiveInit
 * FUNCTION PURPOSE:    Puts MSP in receive mode and transmits slave
 *                      address. This will only initiate the receive,
 *                      not read any data.
 * INPUTS:
 *  -uint8_t address:   7-bit address (aligned right) of slave device
 * OUTPUTS:
 *  -None
 *********************************************************************/
void i2cReceiveInit(uint8_t address);


/**********************************************************************
 * FUNCTION NAME:       depthSensorInit
 * FUNCTION PURPOSE:    Resets depth sensor and reads the PROM
 *                      calibration values, then sends them to the main
 *                      computer.
 * INPUTS:
 *  -None
 * OUTPUTS:
 *  -uint8_t            = 1 or 0, returns 0 on failure and 1 on success
 *********************************************************************/
uint8_t depthSensorInit(void);


#endif /* I2C_H_ */
