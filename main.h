#ifndef MAIN_H_
#define MAIN_H_


#define DCOCLK_FREQ     48000000    // DCO clock frequency
#define HSMCLK_FREQ     48000000    // HSM clock frequency
#define SMCLK_FREQ      12000000    // SM clock frequency


// Global variables
queue_t events;
queue_t pneumaticsReceiveQueue;
queue_t motorReceiveQueue;
queue_t transmitQueue;
volatile queue_t* eventList = &events;
volatile queue_t* pneumaticsReceive = &PneumaticsReceiveQueue;
volatile queue_t* motorReceive = &motorReceiveQueue;
volatile queue_t* transmit = &transmitQueue;
volatile uint8_t pneumaticsState;
volatile uint8_t currentActuator;
volatile uint8_t i2cState;
volatile uint32_t depthRead;


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
    PNEUMATICS_COMMAND_RECEIVED
};


enum firingStates
{
    PNEUMATICS_READY = 0,
    COMMAND_SENT,
};


enum errorCodes
{
    EVENTLIST_QUEUE_FULL_ERROR = 100,
    MOTOR_RECEIVE_QUEUE_FULL_ERROR,
    PNEUMATICS_RECEIVE_QUEUE_FULL_ERROR,
    COMMUNICATION_ERROR
};


#endif /* MAIN_H_ */
