#ifndef UART_H_
#define UART_H_


#include "queue.h"
#include "msp.h"
#include "main.h"


// Define various frames used in UART communication
#define START_STOP_FRAME        0xC0
#define ESCAPE_FRAME            0xDB
#define C0_FRAME                0xDC
#define DB_FRAME                0xDD
#define CONFIRMATION_CORRECT    0xF0
#define CONFIRMATION_INCORRECT  0x0F
#define MOTOR_CHANGE_FRAME      0x11
#define PNEUMATICS_FIRE_FRAME   0x22
#define DEPTH_SENSOR_FRAME      0xCC
#define MOTOR_CURRENT_FRAME     0xDD
#define POWER_CURRENT_FRAME     0xEE
#define ERROR_FRAME             0xFF


// Declare global variables
extern volatile queue_t* eventList;
extern volatile queue_t* pneumaticsReceive;
extern volatile queue_t* motorReceive;
extern volatile queue_t* transmit;
extern volatile uint8_t pneumaticsState;
extern volatile uint8_t currentActuator;


// Enumeration for the different receiving states
enum compReceiveStates
{
    READY = 0,
    RECEIVING,
    MOTOR_RECEIVING,
    PNEUMATICS_RECEIVING
};


// 9600 baud for pneumatics eUSCI_A1 (P2.3 and P2.2)
// 115200 baud for computer eUSCI_A0 (P1.2 and P1.3)


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
 * FUNCTION NAME:       uartBeginCompTransmit
 * FUNCTION PURPOSE:    Begins a transmission to the computer by
 *                      loading data into TXBUF and enabling the TX
 *                      interrupt.
 * INPUTS:
 *  -None
 * OUTPUTS:
 *  -None
 *********************************************************************/
void uartBeginCompTransmit(void);


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
