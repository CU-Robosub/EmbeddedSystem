// Include all headers
#include "queue.h"
#include "msp.h"
#include "adc.h"
#include "i2c.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"
#include "clock.h"
#include "main.h"


void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// Stop watchdog timer
	clockConfigure();
	adcConfigure();
	uartCompConfigure();
	uartPneumaticsConfigure();
	gpioConfigure();
	timerConfigure();

	// Declare/initialize variables for main loop
	uint8_t scheduleEvent = NO_EVENT;
	uint8_t motorCurrents[16];
	uint8_t powerStatus[12];
	pneumaticsState = PNEUMATICS_READY;
	i2cState = I2C_READY;
	currentActuator = 0;
	depthRead = 0;

	// Run the scheduler in the main loop
	// TODO: Upgrade scheduler to an RTOS
	while(1)
	{
	    // Begin each loop by grabbing the next event
	    scheduleEvent = queuePop(eventList);

	    // Nothing to perform, go to sleep and wait
	    if(scheduleEvent == NO_EVENT)
	    {
	        // Code
	    }

	    // Begin the I2C communication with the depth sensor
	    else if(scheduleEvent == DEPTH_SENSOR_READ_START)
	    {
	        // Check if an I2C reading is currently happening
	        if(i2cState == I2C_READY)
	        {
	            // If I2C is ready for another conversion, begin the next conversion
	            i2cState = ADDRESS_SENT_CONVERSION;         // Update the state machine
	            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR;        // Set MSP to transmit mode
	            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;     // Transmit start condition and write mode slave address
	            EUSCI_B0->IFG   &= ~EUSCI_B_IFG_TXIFG0;     // Clear transmit flag before enabling the interupt
	            EUSCI_B0->IE    |= EUSCI_B_IE_TXIE0;        // Enable the TX interrupt for communication
	        }
	        // If I2C isn't ready for another conversion, just skip it because it does 60 per second
	    }

	    // Transmit depth sensor data to the computer
        else if(scheduleEvent == DEPTH_SENSOR_READ_FINISH)
        {
            // If the transmit queue is empty, begin queuing data to transmit
            if(queueEmpty(transmit))
            {
                // Make sure we aren't in the middle of a reading
                if(i2cState < ADDRESS_SENT_ADC_READING)
                {
                    // Add starting frame and instruction byte to queue
                    queuePush(transmit, START_STOP_FRAME);
                    queuePush(transmit, DEPTH_SENSOR_FRAME);

                    // Fill transmit queue with depth sensor data, MSB first
                    queuePush(transmit, (uint8_t)((depthRead >> 16) & 0xFF));
                    queuePush(transmit, (uint8_t)((depthRead >> 8) & 0xFF));
                    queuePush(transmit, (uint8_t)(depthRead & 0xFF));

                    // Add end frame to queue
                    queuePush(transmit, START_STOP_FRAME);

                    // Begin UART transmission
                    uartBeginCompTransmit();
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
            ADC14->CTL1 &= ~ADC14_CTL1_CSTARTADD_MASK;         // Set start address to MEM0
            while(!(ADC14->CTL0 & ADC14_CTL0_BUSY));           // Wait until the ADC is not busy
            ADC14->CTL0 |= ADC14_CTL0_SC;                      // Start a conversion
        }

	    // Transmit the motor current data to the computer
        else if(scheduleEvent == MOTOR_CURRENT_READ_FINISH)
        {
            // Update motor currents data from MEM0 to MEM7
            for(int i = 0;i < 8;i++)
            {
                motorCurrents[2*i] = (uint8_t) ADC14->MEM[i] & 0xFF;             // Store lower half of conversions
                motorCurrents[(2*i)+1] = (uint8_t) (ADC14->MEM[i] >> 8) & 0xFF;  // Store upper half of conversions
            }

            // If the transmit queue is empty, begin queuing data to transmit
            if(queueEmpty(transmit))
            {
                // Add starting frame and instruction byte to queue
                queuePush(transmit, START_STOP_FRAME);
                queuePush(transmit, MOTOR_CURRENT_FRAME);

                // Fill transmit queue with motorCurrents data
                for(int i = 0;i < 16;i++)
                {
                    queuePush(transmit, motorCurrents[i]);
                }

                // Add end frame to queue
                queuePush(transmit, START_STOP_FRAME);

                // Begin UART transmission
                uartBeginCompTransmit();
            }
            // If the transmit queue isn't empty, try again later
            else
            {
                queuePush(eventList, MOTOR_CURRENT_READ_FINISH);
            }
        }

	    // Begin an ADC conversion for the power lines
        else if(scheduleEvent == POWER_CURRENT_READ_START)
        {
            ADC14->CTL1 &= ~ADC14_CTL1_CSTARTADD_MASK;         // Clear start address bits
            ADC14->CTL1 |= 0x00080000;                         // Set start address to MEM8
            while(!(ADC14->CTL0 & ADC14_CTL0_BUSY));           // Wait until the ADC is not busy
            ADC14->CTL0 |= ADC14_CTL0_SC;                      // Start a conversion
        }

	    // Transmit the power current data to the computer
        else if(scheduleEvent == POWER_CURRENT_READ_FINISH)
        {
            // Update power currents data from MEM8 to MEM13
            for(int i = 0;i < 6;i++)
            {
                powerStatus[2*i] = (uint8_t) ADC14->MEM[i+8] & 0xFF;
                powerStatus[(2*i)+1] = (uint8_t) (ADC14->MEM[i+8] >> 8) & 0xFF;
            }

            // Based on the newest battery reading, update the LED color
            setLEDColor((uint16_t)ADC14->MEM[13] & 0xFFFF);

            // If the transmit queue is empty, begin queuing data to transmit
            if(queueEmpty(transmit))
            {
                // Add starting frame and instruction byte to queue
                queuePush(transmit, START_STOP_FRAME);
                queuePush(transmit, POWER_CURRENT_FRAME);

                // Fill transmit queue with powerStatus data
                for(int i = 0;i < 12;i++)
                {
                    queuePush(transmit, powerStatus[i]);
                }

                // Add end frame to queue
                queuePush(transmit, START_STOP_FRAME);

                // Begin UART transmission
                uartBeginCompTransmit();
            }
            // If the transmit queue isn't empty, try again later
            else
            {
                queuePush(eventList, POWER_CURRENT_READ_FINISH);
            }
        }

	    // Change the PWM frequencies of motors based on received input
        else if(scheduleEvent == MOTOR_COMMAND_RECEIVED)
        {
            // Read motor number and pulse length
            uint8_t motorNumber = queuePop(motorReceive);
            uint8_t pulseLength = queuePop(motorReceive) * 2;   // *2 because PWM clock period is 0.5usec

            // Check that this is followed by an end frame to assure proper communication
            if(queuePop(motorReceive) != START_STOP_FRAME)
            {
                // If frame is not a stop frame, follow error procedure
                while(!queueEmpty(transmit))
                {
                    // Empty out the transmit queue
                    queuePop(transmit);
                }

                // Fill transmit queue with error message
                queuePush(transmit, START_STOP_FRAME);
                queuePush(transmit, ERROR_FRAME);
                queuePush(transmit, COMMUNICATION_ERROR);
                queuePush(transmit, START_STOP_FRAME);

                // Begin transmission and enter infinite while loop
                uartBeginCompTransmit();
                while(1);
            }
            else
            {
                // If frame is a stop frame, update motor as instructed
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
                        // Follow error procedure
                        while(!queueEmpty(transmit))
                        {
                            // Empty out the transmit queue
                            queuePop(transmit);
                        }

                        // Fill transmit queue with error message
                        queuePush(transmit, START_STOP_FRAME);
                        queuePush(transmit, ERROR_FRAME);
                        queuePush(transmit, COMMUNICATION_ERROR);
                        queuePush(transmit, START_STOP_FRAME);

                        // Begin transmission and enter infinite while loop
                        uartBeginCompTransmit();
                        while(1);
                }
            }
        }

	    // Begin a transmission to the pneumatics board to fire
        else if(scheduleEvent == PNEUMATICS_COMMAND_RECEIVED)
        {
            // Check if there is currently an actuator being fired
            if(pneumaticsState == COMMAND_SENT)
            {
                // Queue up another decode for later when it is ready
                queuePush(eventList, PNEUMATICS_COMMAND_RECEIVED);
            }

            // If there is no actuator currently being fired
            else
            {
                // Read which actuator needs to be fired
                currentActuator = queuePop(pneumaticsReceive);

                // Check that this is followed by an end frame to assure proper communication
                if(queuePop(pneumaticsReceive) != START_STOP_FRAME)
                {
                    // If frame is not a stop frame, follow error procedure
                    while(!queueEmpty(transmit))
                    {
                        // Empty out the transmit queue
                        queuePop(transmit);
                    }

                    // Fill transmit queue with error message
                    queuePush(transmit, START_STOP_FRAME);
                    queuePush(transmit, ERROR_FRAME);
                    queuePush(transmit, COMMUNICATION_ERROR);
                    queuePush(transmit, START_STOP_FRAME);

                    // Begin transmission and enter infinite while loop
                    uartBeginCompTransmit();
                    while(1);
                }
                else
                {
                    // If a stop frame was read, begin the pneumatics firing process
                    uint8_t beginFire[3];
                    beginFire[0] = START_STOP_FRAME;
                    beginFire[1] = fireActuator;
                    beginFire[2] = START_STOP_FRAME;

                    // Update state to indicate that firing is beginning
                    pneumaticsState = COMMAND_SENT;

                    // Transmit the fire command
                    uartSendPneumaticsN(beginFire, 3);
                }
            }
        }
	}
}
