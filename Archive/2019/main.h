#ifndef MAIN_H_
#define MAIN_H_


#include "msp.h"
#include "adc.h"
#include "i2c.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"
#include "clock.h"
#include "stdint.h"
#include "queue.h"
#include <stddef.h>


#define DCOCLK_FREQ     48000000    // DCO clock frequency
#define HSMCLK_FREQ     48000000    // HSM clock frequency
#define SMCLK_FREQ      12000000    // SM clock frequency
#define CRC8_INIT       0x00        // Init variable for crc8 function
#define CRC8_POLY       0x07        // Polynomial variable for crc8 function

#define UART_RX_BUFFER_SIZE 20

#define EMBEDDED_SYSTEM

#define I2C_ENABLED

enum scheduleEvents
{
    NO_EVENT = 0,
    DEPTH_SENSOR_READ_START,
    DEPTH_SENSOR_READ_FINISH,
    MOTOR_CURRENT_READ_START,
    MOTOR_CURRENT_READ_FINISH,
    POWER_CURRENT_READ_START,
    POWER_CURRENT_READ_FINISH,
    MOTOR_COMMAND_RECEIVED,
    PNEUMATICS_COMMAND_RECEIVED,
};


enum errorCodes
{
    EVENTLIST_QUEUE_FULL_ERROR = 100,
    MOTOR_RECEIVE_QUEUE_FULL_ERROR,
    PNEUMATICS_RECEIVE_QUEUE_FULL_ERROR,
    COMMUNICATION_ERROR
};


#endif /* MAIN_H_ */
