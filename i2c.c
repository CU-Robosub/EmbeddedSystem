#include "i2c.h"


/*********************************** NOTES ***********************************
 * - Depth Sensor SCL operates at a maximum frequency of 400kHz
 * - We use the OSR 256 depth reading because it is the fastest option
 *    ~ This gives a resolution of 1.57mbar, which is accurate enough
 ****************************************************************************/

void i2cConfigure(void)
{
    // Disable eUSCI to configure I2C
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST;

    // Configure I2C
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_MST;           // Set MSP as the master device
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_MODE_3;        // Operate in I2C mode
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SSEL__SMCLK;   // Use the SMCLK as the I2C clock source

    // Set clock divider for SMCLK at 3MHz for 400kHz data rate
    // Fscl = Fsmclk / BRW --> 400,000 = 12,000,000 / 30
    EUSCI_B0->BRW = 30;

    // Load Depth Sensor address into slave address register
    EUSCI_B0->I2CSA = DEPTH_SENSOR_ADDRESS;

    // Enable receive interrupt for communicating with depth sensor
    EUSCI_B0->IE |= EUSCI_B_IE_RXIE0;

    // Enable I2C Module to start operations
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;
}


void EUSCIB1_IRQHandler(void)
{
    // If a transmit interrupt called the handler, check what state we are in
    if(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0)
    {
        // If we have sent the address portion of a conversion command
        if(i2cState == ADDRESS_SENT_CONVERSION)
        {
            // Update state then transmit the actual conversion command and a stop bit
            i2cState = CONVERSION_STARTED;
            EUSCI_B0->TXBUF = DEPTH_SENSOR_DEPTH_CONVERSION;
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        }
        // If we have finished the conversion command and are moving to the ADC read command
        else if(i2cState == CONVERSION_STARTED)
        {
            // Update state then transmit the start/address frame again
            i2cState = ADDRESS_SENT_ADC_COMMAND;
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR;        // Set MSP to transmit mode
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;     // Transmit start condition and write mode slave address
        }
        // If we have sent the address portion of an ADC read command
        else if(i2cState == ADDRESS_SENT_ADC_COMMAND)
        {
            // Update state then transmit the actual ADC read command and a stop bit
            i2cState = ADC_READ_STARTED;
            EUSCI_B0->TXBUF = DEPTH_SENSOR_ADC_READ;
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        }
        // If an ADC read has begun, prepare to receive data
        else if(i2cState == ADC_READ_STARTED)
        {
            // Update state then set MSP to receive mode for incoming data and disable TX interrupt
            i2cState = ADDRESS_SENT_ADC_READING;
            EUSCI_B0->IE &= ~EUSCI_B_IE_TXIE0;          // Disable transmit interrupts
            EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;       // Set MSP to receive mode
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;     // Transmit start condition and read mode slave address
        }
        // If not in an expected state, clear the TX interrupt flag
        else
        {
            EUSCI_B0->IFG &= ~EUSCI_B_IFG_TXIFG0;
        }
    }

    // If a receive interrupt called the handler, check what state we are in
    if (EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0)
    {
        // If we just began reading an ADC conversion
        if (i2cState == ADDRESS_SENT_ADC_READING)
        {
            // Update state then read the most significant byte of the 24-bit depth reading
            i2cState = DEPTH_1_RECEIVED;
            depthRead = (EUSCI_B1->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK) << 16;
        }
        else if (i2cState == DEPTH_1_RECEIVED)
        {
            // Update state then read the middle byte of the 24-bit depth reading
            i2cState = DEPTH_2_RECEIVED;
            depthRead |= (EUSCI_B1->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK) << 8;
        }
        else if (i2cState == DEPTH_2_RECEIVED)
        {
            // Update state then read the middle byte of the 24-bit depth reading
            i2cState = I2C_READY;
            depthRead |= (EUSCI_B1->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK);

            // End receiving by sending a NACK and STOP signal
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXNACK;
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

            // Add an event to transmit the depth sensor reading
            if(!queuePush(eventList, DEPTH_SENSOR_READ_FINISH))
            {
                // If queue is full, this is an error
                while(!queueEmpty(transmit))
                {
                    // Empty out the transmit queue
                    queuePop(transmit);
                }

                // Fill transmit queue with error message
                queuePush(transmit, START_STOP_FRAME);
                queuePush(transmit, ERROR_FRAME);
                queuePush(transmit, EVENTLIST_QUEUE_FULL_ERROR);
                queuePush(transmit, START_STOP_FRAME);

                // Begin transmission and enter infinite while loop
                uartBeginCompTransmit();
                while(1);
            }
        }
    }
}




