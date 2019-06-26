#ifndef I2C_H_
#define I2C_H_


#include "queue.h"
#include "msp.h"
#include "adc.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"
#include "main.h"


// Define frames for communication with depth sensor
#define DEPTH_SENSOR_ADDRESS            0x86    // Only 7 bits
#define DEPTH_SENSOR_RESET_COMMAND      0x1E
#define DEPTH_SENSOR_DEPTH_CONVERSION   0x40
#define DEPTH_SENSOR_ADC_READ           0x00

<<<<<<< HEAD

// Declare global variables
extern volatile queue_t* eventList;
extern volatile uint8_t i2cState;
extern volatile uint32_t depthRead;
=======
/**********************************************************************
 * FUNCTION NAME:       i2cConfigure
 * FUNCTION PURPOSE:    Initializes EUSCI_B0 to work in I2C mode using
 *                      SMCLK, 400KBPS data rate, and interrupts for
 *                      both TX and RX. Also initializes globals
 * INPUTS:
 *  -None
 * OUTPUTS:
 *  -None
 *********************************************************************/
void i2cConfigure(void);

/**********************************************************************
 * FUNCTION NAME:       i2cWrite8Start
 * FUNCTION PURPOSE:    Sets up EUSCI_B0 for transmission of the given
 *                      data "writeByte". Does not send data over I2C
 *                      yet, just configures registers and writes to
 *                      writeData.
 * INPUTS:
 *  -uint8_t writeByte  Data to be written
 * OUTPUTS:
 *  -None
 *********************************************************************/
void i2cWrite8Start (uint8_t writeByte);

/**********************************************************************
 * FUNCTION NAME:       i2cWrite8Data
 * FUNCTION PURPOSE:    Sends the current byte in writeData to the TX
 *                      buffer of EUSCI_B1
 * INPUTS:
 *  -None
 * OUTPUTS:
 *  -None
 *********************************************************************/
void i2cWrite8Data (void);

/**********************************************************************
 * FUNCTION NAME:       i2cStop
 * FUNCTION PURPOSE:    Waits for the TX buffer to be empty and then
 *                      sends a stop command
 * INPUTS:
 *  -None
 * OUTPUTS:
 *  -None
 *********************************************************************/
void i2cStop (void);
>>>>>>> 64c4418d77339039b6b1c5dc81dd7dfc4f40471d


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
