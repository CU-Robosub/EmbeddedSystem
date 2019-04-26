#ifndef I2C_H_
#define I2C_H_

#include "msp.h"
#include "adc.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"
#include "main.h"

//DEFINES
#define depthRead       ((uint8_t) 0b11101101)
#define depthWrite      ((uint8_t) 0b11101100)
#define depthReset      0x1E
#define depthD1Con256   0x40

//GLOBAL VARIABLES
uint8_t commState;
uint8_t writeData;
uint8_t depthOne;
uint8_t depthTwo;
uint8_t depthThree;

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

/////OLD FUNCTION HERE////
/**********************************************************************
 * FUNCTION NAME:       i2cRead16
 * FUNCTION PURPOSE:    Reads 16 bits out of I2C
 * INPUTS:
 *  -pointer
 * OUTPUTS:
 *  -16 bit value retrieved from RX buffer
 *********************************************************************/
int i2cRead16 (unsigned char pointer);

/**********************************************************************
 * FUNCTION NAME:       i2cRead24Start
 * FUNCTION PURPOSE:
 * INPUTS:
 *  -None
 * OUTPUTS:
 *  -None
 *********************************************************************/
void i2cRead24Start (void);

/**********************************************************************
 * FUNCTION NAME:       i2cRead24End
 * FUNCTION PURPOSE:
 * INPUTS:
 *  -None
 * OUTPUTS:
 *  -None
 *********************************************************************/
int i2cRead24End (void);

/**********************************************************************
 * FUNCTION NAME:       depthInit
 * FUNCTION PURPOSE:
 * INPUTS:
 *  -None
 * OUTPUTS:
 *  -None
 *********************************************************************/
void depthInit(void);

void depthD12Set(uint8_t D1, uint8_t D2);

void depthAdcStart(void);


#endif /* I2C_H_ */
