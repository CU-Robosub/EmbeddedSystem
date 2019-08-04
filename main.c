// Include all headers
#include "main.h"


// Global variables
queue_t events;
queue_t pneumaticsReceiveQueue;
queue_t motorReceiveQueue;
queue_t transmitQueue;
volatile queue_t* eventList = &events;
volatile queue_t* pneumaticsReceive = &pneumaticsReceiveQueue;
volatile queue_t* motorReceive = &motorReceiveQueue;
volatile queue_t* transmit = &transmitQueue;
volatile uint8_t i2cState;
volatile uint8_t powerConversionDone;
volatile uint8_t motorConversionDone;
volatile uint8_t depthAdcConversionFlag;   // Gets set after starting depth conversion, cleared in timer interrupt
volatile uint8_t depthRead[6];
volatile uint8_t depthSensorConnected;
volatile uint16_t tickCounter;
volatile uint16_t pnumaticsOffTick[8];

volatile uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
volatile uint8_t uart_rx_buffer_pos;


uint8_t crc8(uint8_t crc, uint8_t poly, uint8_t *data, size_t len)
{
    uint8_t k;

    while (len--)
    {
        crc ^= *data++;

        for (k = 0; k < 8; k++)
        {
            crc = crc & 0x80 ? (crc << 1) ^ poly : crc << 1;
        }
    }

    crc &= 0xff;
    return crc;
}


void main_embedded()
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // Stop watchdog timer

    // Run configuration atomically
    __disable_irq();
    clockConfigure();
    gpioConfigure();
    adcConfigure();
    uartCompConfigure();
    uartPneumaticsConfigure();
    i2cConfigure();
    timerConfigure();

    // Declare/initialize variables for main loop
    uint8_t scheduleEvent = NO_EVENT;
    uint8_t motorCurrents[16];
    uint8_t powerStatus[12];
    i2cState = I2C_READY;
    powerConversionDone = 1;    // Indicate that ADC conversions are ready
    motorConversionDone = 1;
    depthAdcConversionFlag = 0;
    depthSensorConnected = 0;
    uint32_t i = 0;  // for loop index
    for(i = 0;i < 6;i++)
    {
        depthRead[i] = 0;
    }

    // Erase all queues/lists
    eventList->head = 0;
    eventList->tail = 0;
    eventList->quantity = 0;
    pneumaticsReceive->head = 0;
    pneumaticsReceive->tail = 0;
    pneumaticsReceive->quantity = 0;
    motorReceive->head = 0;
    motorReceive->tail = 0;
    motorReceive->quantity = 0;
    transmit->head = 0;
    transmit->tail = 0;
    transmit->quantity = 0;
    for(i = 0;i < QUEUE_SIZE;i++)
    {
        eventList->elements[i] = 0;
        pneumaticsReceive->elements[i] = 0;
        motorReceive->elements[i] = 0;
        transmit->elements[i] = 0;
    }
    __enable_irq();

    // Run depth sensor initialization function until it works
    if(depthSensorInit())
    {
        depthSensorConnected = 1;
    }

    // Run the scheduler in the main loop
    // TODO: Upgrade scheduler to an RTOS
    while(1)
    {
        // Begin each loop by grabbing the next event
        scheduleEvent = queuePop(eventList);

        // If the depth sensor was disconnected, attempt to reconnect
        if(!depthSensorConnected)
        {
            if(depthSensorInit())
            {
                depthSensorConnected = 1;
            }
        }

        // Nothing to perform, go to sleep and wait
        if(scheduleEvent == NO_EVENT)
        {
            // Code
        }

        // Begin the I2C communication with the depth sensor
        else if(scheduleEvent == DEPTH_SENSOR_READ_START)
        {
            // Check if an I2C reading is currently happening
            if(i2cState == I2C_READY && depthSensorConnected)
            {
                // Update i2c state for error handling in main
                i2cState = DEPTH_CONVERSION_STARTED;

                // Begin a depth sensor conversion
                if(!i2cTransmit(DEPTH_SENSOR_ADDRESS, DEPTH_SENSOR_DEPTH_CONVERSION))
                {
                    i2cState = I2C_READY;
                    depthSensorConnected = 0;
                    depthAdcConversionFlag = 0;
                }
            }
            // If I2C isn't ready for another conversion, just skip it because it does 60 per second
        }

        // Transmit depth sensor data to the computer
        else if(scheduleEvent == DEPTH_SENSOR_READ_FINISH)
        {
            // If there is room for a depth reading transmission
            if((QUEUE_SIZE - transmit->quantity) >= 9)
            {
                // Make sure we are done with a reading
                if(i2cState == I2C_TRANSMITTING)
                {
                    // Add starting frame and instruction byte to queue
                    queuePush(transmit, START_STOP_FRAME);
                    queuePush(transmit, DEPTH_SENSOR_FRAME);

                    // Check if any bytes are start stop or escape frames
                    for(i = 0;i < 6;i++)
                    {
                        if(depthRead[i] == START_STOP_FRAME)
                        {
                            // Queue escape and c0 frames
                            queuePush(transmit, ESCAPE_FRAME);
                            queuePush(transmit, C0_FRAME);
                        }
                        else if(depthRead[i] == ESCAPE_FRAME)
                        {
                            // Queue escape and db frames
                            queuePush(transmit, ESCAPE_FRAME);
                            queuePush(transmit, DB_FRAME);
                        }
                        else
                        {
                            // Queue whatever actual data is
                            queuePush(transmit, depthRead[i]);
                        }
                    }

                    // Add end frame to queue
                    queuePush(transmit, START_STOP_FRAME);

                    // Begin UART transmission
                    uartBeginCompTransmit();

                    // Update i2c state
                    i2cState = I2C_READY;
                }
            }
            // If the transmit queue isn't empty, try again later
            else
            {
                queuePush(eventList, DEPTH_SENSOR_READ_FINISH);
            }
        }

        // Begin an ADC conversion for the motor currents
        else if(scheduleEvent == MOTOR_CURRENT_READ_START)
        {
            // If the ADC is not busy
            if(!(ADC14->CTL0 & ADC14_CTL0_BUSY))
            {
                // Disable ADC encoding to change the conversion start address to MEM0
                ADC14->CTL0 &= ~ADC14_CTL0_ENC;
                ADC14->CTL1 &= ~ADC14_CTL1_CSTARTADD_MASK;

                // Enable ADC encoding and start a conversion
                ADC14->CTL0 |= ADC14_CTL0_ENC;
                ADC14->CTL0 |= ADC14_CTL0_SC;
            }

            // If the ADC is busy, try again later
            else
            {
                queuePush(eventList, MOTOR_CURRENT_READ_START);
            }
        }

        // Transmit the motor current data to the computer
        else if(scheduleEvent == MOTOR_CURRENT_READ_FINISH)
        {
            // Update motor currents data from MEM0 to MEM7
            for(i = 0;i < 8;i++)
            {
                motorCurrents[2*i] = (uint8_t) ADC14->MEM[i] & 0xFF;             // Store lower half of conversions
                motorCurrents[(2*i)+1] = (uint8_t) (ADC14->MEM[i] >> 8) & 0xFF;  // Store upper half of conversions
            }

            // If there is room for a motor current transmission
            if((QUEUE_SIZE - transmit->quantity) >= 34)
            {
                // Add starting frame and instruction byte to queue
                queuePush(transmit, START_STOP_FRAME);
                queuePush(transmit, MOTOR_CURRENT_FRAME);

                // Fill transmit queue with motorCurrents data
                for(i = 0;i < 16;i++)
                {
                    // Before filling check if the data frame is equal to a start stop frame
                    if(motorCurrents[i] == START_STOP_FRAME)
                    {
                        // If it is, send an escape frame and a C0 frame
                        queuePush(transmit, ESCAPE_FRAME);
                        queuePush(transmit, C0_FRAME);
                    }

                    // Also check if the data frame is equal to an escape frame
                    else if(motorCurrents[i] == ESCAPE_FRAME)
                    {
                        // If it is, send an escape frame and a DB frame
                        queuePush(transmit, ESCAPE_FRAME);
                        queuePush(transmit, DB_FRAME);
                    }

                    // If the data is not a unique frame, send it
                    else
                    {
                        queuePush(transmit, motorCurrents[i]);
                    }
                }

                // Add end frame to queue
                queuePush(transmit, START_STOP_FRAME);

                // If the interrupt is currently disabled
                if(!(EUSCI_A0->IE & EUSCI_A_IE_TXIE))
                {
                    // Begin UART transmission
                    uartBeginCompTransmit();
                }

                // Indicate that a motor conversion is done
                motorConversionDone = 1;
            }

            // If the transmit queue is busy, try again later
            else
            {
                queuePush(eventList, MOTOR_CURRENT_READ_FINISH);
            }
        }

        // Begin an ADC conversion for the power lines
        else if(scheduleEvent == POWER_CURRENT_READ_START)
        {
            // If the ADC is not busy
            if(!(ADC14->CTL0 & ADC14_CTL0_BUSY))
            {
                // Disable ADC encoding to change the conversion start address to MEM8
                ADC14->CTL0 &= ~ADC14_CTL0_ENC;
                ADC14->CTL1 &= ~ADC14_CTL1_CSTARTADD_MASK;
                ADC14->CTL1 |= 0x00080000;

                // Enable ADC encoding and start a conversion
                ADC14->CTL0 |= ADC14_CTL0_ENC;
                ADC14->CTL0 |= ADC14_CTL0_SC;
            }

            // If the ADC is busy, try again later
            else
            {
                queuePush(eventList, POWER_CURRENT_READ_START);
            }
        }

        // Transmit the power current data to the computer
        else if(scheduleEvent == POWER_CURRENT_READ_FINISH)
        {
            // Update power currents data from MEM8 to MEM13
            for(i = 0;i < 6;i++)
            {
                powerStatus[2*i] = (uint8_t) ADC14->MEM[i+8] & 0xFF;
                powerStatus[(2*i)+1] = (uint8_t) (ADC14->MEM[i+8] >> 8) & 0xFF;
            }

            // Based on the newest battery reading, update the LED color
            setLEDColor((uint16_t)ADC14->MEM[13] & 0xFFFF);

            // If there is room for a power current transmission
            if((QUEUE_SIZE - transmit->quantity) >= 27)
            {
                // Add starting frame and instruction byte to queue
                queuePush(transmit, START_STOP_FRAME);
                queuePush(transmit, POWER_CURRENT_FRAME);

                // Fill transmit queue with powerStatus data
                for(i = 0;i < 12;i++)
                {
                    // Before filling check if the data frame is equal to a start stop frame
                    if(powerStatus[i] == START_STOP_FRAME)
                    {
                        // If it is, send an escape frame and a C0 frame
                        queuePush(transmit, ESCAPE_FRAME);
                        queuePush(transmit, C0_FRAME);
                    }

                    // Also check if the data frame is equal to an escape frame
                    else if(powerStatus[i] == ESCAPE_FRAME)
                    {
                        // If it is, send an escape frame and a DB frame
                        queuePush(transmit, ESCAPE_FRAME);
                        queuePush(transmit, DB_FRAME);
                    }

                    // If the data is not a unique frame, send it
                    else
                    {
                        queuePush(transmit, powerStatus[i]);
                    }
                }

                // Add end frame to queue
                queuePush(transmit, START_STOP_FRAME);

                // If the interrupt is currently disabled
                if(!(EUSCI_A0->IE & EUSCI_A_IE_TXIE))
                {
                    // Begin UART transmission
                    uartBeginCompTransmit();
                }

                // Indicate that a power conversion is done
                powerConversionDone = 1;
            }

            // If the transmit queue is busy, try again later
            else
            {
                queuePush(eventList, POWER_CURRENT_READ_FINISH);
            }
        }

        // Change the PWM frequencies of motors based on received input
        else if(scheduleEvent == MOTOR_COMMAND_RECEIVED)
        {
            // Read motor number and least significant byte of the pulse length
            uint8_t motorNumber = queuePop(motorReceive);
            uint8_t pulseLengthLSB = queuePop(motorReceive);

            // Check if the pulse length was an escape frame
            if(pulseLengthLSB == ESCAPE_FRAME)
            {
                // Pull the next value from the queue
                uint8_t temp = queuePop(motorReceive);

                // If it is a C0 frame, update the value of pulseLength
                if(temp == C0_FRAME)
                {
                    pulseLengthLSB = 0xC0;
                }

                // If it is a DB frame, update the value of pulseLength
                if(temp == DB_FRAME)
                {
                    pulseLengthLSB = 0xDB;
                }
            }

            // Read the most significant byte of the pulse length
            uint8_t pulseLengthMSB = queuePop(motorReceive);

            // Check if the pulse length was an escape frame
            if(pulseLengthMSB == ESCAPE_FRAME)
            {
                // Pull the next value from the queue
                uint8_t temp = queuePop(motorReceive);

                // If it is a C0 frame, update the value of pulseLength
                if(temp == C0_FRAME)
                {
                    pulseLengthMSB = 0xC0;
                }

                // If it is a DB frame, update the value of pulseLength
                if(temp == DB_FRAME)
                {
                    pulseLengthMSB = 0xDB;
                }
            }

            // Check that this is followed by an end frame to assure proper communication
            if(queuePop(motorReceive) != START_STOP_FRAME)
            {
                // Enter infinite while loop if queue is full because that's an error
                while(1);
            }

            // Update motor as instructed
            uint16_t pulseLength = ((pulseLengthMSB << 8) | pulseLengthLSB);
            switch (motorNumber)
            {
                // Motor 1 source is TA1.1
                case 1:
                    TA1CCR1 = pulseLength;
                    break;
                // Motor 2 source is TA1.2
                case 2:
                    TA1CCR2 = pulseLength;
                    break;
                // Motor 3 source is TA1.3
                case 3:
                    TA1CCR3 = pulseLength;
                    break;
                // Motor 4 source is TA1.4
                case 4:
                    TA1CCR4 = pulseLength;
                    break;
                // Motor 5 source is TA2.1
                case 5:
                    TA2CCR1 = pulseLength;
                    break;
                // Motor 6 source is TA2.2
                case 6:
                    TA2CCR2 = pulseLength;
                    break;
                // Motor 7 source is TA2.3
                case 7:
                    TA2CCR3 = pulseLength;
                    break;
                // Motor 8 source is TA2.4
                case 8:
                    TA2CCR4 = pulseLength;
                    break;
                // If none of these conditions are met, there has been a communications error
                default:
                    // Enter infinite while loop if queue is full because that's an error
                    while(1);
            }
        }

        // Begin a transmission to the pneumatics board to fire
        else if(scheduleEvent == PNEUMATICS_COMMAND_RECEIVED)
        {
            // Read which actuator needs to be fired
            uint8_t actuator = queuePop(pneumaticsReceive);
            uint8_t duration = queuePop(pneumaticsReceive);

            // Check that this is followed by an end frame to assure proper communication
            if(queuePop(pneumaticsReceive) != START_STOP_FRAME)
            {
                // Enter infinite while loop if queue is full because that's an error
                while(1);
            }
            else
            {
                // If a stop frame was read, begin the pneumatics firing process
                uint8_t fire[5];
                fire[0] = START_STOP_FRAME;
                fire[1] = actuator;
                fire[2] = duration;
                fire[3] = crc8(CRC8_INIT, CRC8_POLY, &fire[1], 2);
                fire[4] = START_STOP_FRAME;

                // Transmit the fire command
                uartSendPneumaticsN(fire, 5);
            }
        }
    }
}


void main_pnumatics(){

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // Stop watchdog timer
    clockConfigure();
    gpioConfigurePnumatics();
    timerConfigurePnumatics();
    uartConfigurePnumatics();

    uint8_t scheduleEvent = NO_EVENT;

    while(1){

        scheduleEvent = queuePop(eventList);

        // Nothing to perform, go to sleep and wait
        if(scheduleEvent == NO_EVENT)
        {
            // Code
        }

        else if(scheduleEvent == PNEUMATICS_COMMAND_RECEIVED)
        {

            // Structure
            // ACTUATOR TIME_ON CRC-8

            // Validate packet CRC
            uint8_t crc = crc8(CRC8_INIT, CRC8_POLY, (uint8_t*)uart_rx_buffer, uart_rx_buffer_pos - 1);
            if(crc == uart_rx_buffer[uart_rx_buffer_pos - 1]){

                // packet should be 3 bytes long
                if(uart_rx_buffer_pos == 3){
                    switch(uart_rx_buffer[0]){
                        case 0x11:
                            P3OUT   |= BIT0;
                            pnumaticsOffTick[0] = tickCounter + uart_rx_buffer[1];
                            break;
                        case 0x22:
                            P3OUT   |= BIT1;
                            pnumaticsOffTick[1] = tickCounter + uart_rx_buffer[1];
                            break;
                        case 0x33:
                            P3OUT   |= BIT2;
                            pnumaticsOffTick[2] = tickCounter + uart_rx_buffer[1];
                            break;
                        case 0x44:
                            P3OUT   |= BIT3;
                            pnumaticsOffTick[3] = tickCounter + uart_rx_buffer[1];
                            break;
                        case 0x55:
                            P3OUT   |= BIT4;
                            pnumaticsOffTick[4] = tickCounter + uart_rx_buffer[1];
                            break;
                        case 0x66:
                            P3OUT   |= BIT5;
                            pnumaticsOffTick[5] = tickCounter + uart_rx_buffer[1];
                            break;
                        case 0x77:
                            P3OUT   |= BIT6;
                            pnumaticsOffTick[6] = tickCounter + uart_rx_buffer[1];
                            break;
                        case 0x88:
                            P3OUT   |= BIT7;
                            pnumaticsOffTick[7] = tickCounter + uart_rx_buffer[1];
                            break;
                        default:
                            // Invalid actuator
                            break;

                    }
                } else {
                    // Wrong packet length
                }
            } else {
                // CRC Fail
            }

            uart_rx_buffer_pos = 0;
            EUSCI_A0->IE |= EUSCI_A_IE_RXIE;
        }
    }
}

void main(void)
{
    #ifdef EMBEDDED_SYSTEM
    main_embedded();
    #endif
    #ifdef PNUMATICS_SYSTEM
    main_pnumatics();
    #endif
}
