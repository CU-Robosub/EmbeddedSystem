#ifndef UART_H_
#define UART_H_

#include "msp.h"
#include "main.h"

//9600 for atmel UCA1 p2.3 p2.2
//115200 for comp UCA0 p1.2 p1.3

/**********************************************************************
 * FUNCTION NAME:       uartCompConfigure
 * FUNCTION PURPOSE:    Configures eUSCI0 as a 115200 baud, LSB first,
 *                      8-bit data, 1 stop bit, no start bit, no parity
 *                      UART communication line to communicate with the
 *                      main computer.
 * INPUTS:
 *  -None
 * OUTPUTS:
 *  -None
 *********************************************************************/
void uartCompConfigure();


/**********************************************************************
 * FUNCTION NAME:       uartSendCompByte
 * FUNCTION PURPOSE:    Transmits a byte through eUSCI0 to the
 *                      computer.
 * INPUTS:
 *  -uint8_t data:      Byte to be transmitted
 * OUTPUTS:
 *  -None
 *********************************************************************/
void uartSendCompByte(uint8_t data);


/**********************************************************************
 * FUNCTION NAME:       uartsendCompN
 * FUNCTION PURPOSE:    Transmits a string of length N through eUSCI0
 *                      to the computer using uartSendCompByte.
 * INPUTS:
 *  -uint8_t * data:    Base pointer of the string to be transmitted.
 *  -uint32_t length:   Length of the string to be transmitted.
 * OUTPUTS:
 *  -None
 *********************************************************************/
void uartSendCompN(uint8_t * data, uint32_t length);


/**********************************************************************
 * FUNCTION NAME:       uartPneumaticsConfigure
 * FUNCTION PURPOSE:    Configures eUSCI1 as a 9600 baud, LSB first,
 *                      8-bit data, 1 stop bit, no start bit, no parity
 *                      UART communication line to communicate with the
 *                      pneumatics board embedded system.
 * INPUTS:
 *  -None
 * OUTPUTS:
 *  -None
 *********************************************************************/
void uartPneumaticsConfigure();


/**********************************************************************
 * FUNCTION NAME:       uartSendPneumaticsByte
 * FUNCTION PURPOSE:    Transmits a byte through eUSCI1 to the
 *                      pneumatics board embedded system.
 * INPUTS:
 *  -uint8_t data:      Byte to be transmitted
 * OUTPUTS:
 *  -None
 *********************************************************************/
void uartSendPneumaticsByte(uint8_t data);


/**********************************************************************
 * FUNCTION NAME:       uartsendPneumaticsN
 * FUNCTION PURPOSE:    Transmits a string of length N through eUSCI1
 *                      to the pneumatics board embedded system using
 *                      uartSendPneumaticsByte.
 * INPUTS:
 *  -uint8_t * data:    Base pointer of the string to be transmitted.
 *  -uint32_t length:   Length of the string to be transmitted.
 * OUTPUTS:
 *  -None
 *********************************************************************/
void uartSendPneumaticsN(uint8_t * data, uint32_t length);

#endif /* UART_H_ */
