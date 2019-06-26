#include "i2c.h"

<<<<<<< HEAD

/*********************************** NOTES ***********************************
 * - Depth Sensor SCL operates at a maximum frequency of 400kHz
 * - We use the OSR 256 depth reading because it is the fastest option
 *    ~ This gives a resolution of 1.57mbar, which is accurate enough
 ****************************************************************************/

=======

// TODO change to work with different clocks speeds
void i2cConfigure(void)
{
    // Initialize global variables
    commState = 0;
    writeData = 0;
    depthOne = 0;
    depthTwo = 0;
    depthThree = 0;

    // Initialize EUSCI_B0 and I2C Master to communicate with slave devices
    // Reset all registers/disable, master, I2C mode, SMCLK
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_MST;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_MODE_3;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SSEL__SMCLK;
    // No autostop to manually control when stop bit is sent
    EUSCI_B0->CTLW1 = EUSCI_B_CTLW1_ASTP_0;
    // Set clock divider for SMCLK at 3MHz for 400KBPS data rate
    EUSCI_B0->BRW = (uint16_t) (3000000 / 40000);

    // Enable interrupts from slave in I2COA0 and sending data
    EUSCI_B0->IE |= EUSCI_B_IE_TXIE0;
    EUSCI_B0->IE |= EUSCI_B_IE_RXIE0;
    // Enable I2C Module to start operations
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;
}

// TODO Merge the following two functions into one?
void i2cWrite8Start (uint8_t writeByte)
{
    commState = 0;
    // Set master to transmit mode
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR;
    // Clear any existing interrupt flag
    EUSCI_B0->IFG &= ~EUSCI_B_IFG_TXIFG0;
    // Wait until buffer is clear and ready to write
    while (EUSCI_B0->STATW & EUSCI_B_STATW_BBUSY);
    // Initiate start and send pointer in I2CSA buffer
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
    writeData = writeByte;
}

void i2cWrite8Data (void)
{
    commState = 1;
    // Send data to slave
    EUSCI_B0->TXBUF = writeData;
}

void i2cStop (void)
{
    // Wait for TX Buf to be empty then send stop command
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
}

/* TODO Check the following functions for behavior and see
 * if any of the code will be the same for our application
 */
/////OLD FUNCTION HERE////
int i2cRead16 (unsigned char pointer)
{
    volatile int val = 0;
    volatile int val2 = 0;
    volatile int val3 = 0;
    // Set master to transmit mode
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR;
    // Clear any existing interrupt flag
    EUSCI_B0->IFG &= ~EUSCI_B_IFG_RXIFG0;
    // Wait until ready to write PL
    while (EUSCI_B0->STATW & EUSCI_B_STATW_BBUSY);
    // Initiate start and send first character
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
    // Wait for TX Buf to be empty
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
    EUSCI_B0->TXBUF = pointer;
    // Wait for TX Buf to be empty and send stop data
    // For the slave to take control of communications
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));

    // Set to receive mode and send start condition
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
    // Wait for the RX Buf to fill
    while (!(EUSCI_B0->IFG & EUSCI_B_IE_RXIE0));
    // Read first 16 bits
    val = (EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK);
    // Send stop command and wait for TXBUF and RXBuf to finish
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
    while (!(EUSCI_B0->IFG & EUSCI_B_IE_RXIE0));
    // Read rest of RXBUF
    val2 = (EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK);
    // Shift val to top MSB
    val = (val << 8);
    // Read from I2C RX Register and write to LSB of val
    val |= val2;
    // Return temperature value
    return (int16_t)val;
}
>>>>>>> 64c4418d77339039b6b1c5dc81dd7dfc4f40471d

void i2cConfigure(void)
{
<<<<<<< HEAD
    // Disable eUSCI to configure I2C
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST;
=======
    commState = 2;
    // Set master to transmit mode
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR;
    // Clear any existing interrupt flag
    EUSCI_B0->IFG &= ~EUSCI_B_IFG_RXIFG0;
    // Wait until ready to write PL
    while (EUSCI_B0->STATW & EUSCI_B_STATW_BBUSY);
    // Initiate start and send first character
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
}
>>>>>>> 64c4418d77339039b6b1c5dc81dd7dfc4f40471d

    // Configure I2C
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_MST;           // Set MSP as the master device
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_MODE_3;        // Operate in I2C mode
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SSEL__SMCLK;   // Use the SMCLK as the I2C clock source

<<<<<<< HEAD
    // Set clock divider for SMCLK at 3MHz for 400kHz data rate
    // Fscl = Fsmclk / BRW --> 400,000 = 12,000,000 / 30
    EUSCI_B0->BRW = 30;
=======
void depthInit(void)
{
    // Specify slave address for depth sensor
    EUSCI_B0->I2CSA = depthWrite;
    // Set the read value into the self address for receive command
    EUSCI_B0->I2COA0 |= depthRead;
    EUSCI_B0->I2COA0 |= EUSCI_B_I2COA0_OAEN;
    // Reset the sensor
    i2cWrite8Start(depthReset);
}
>>>>>>> 64c4418d77339039b6b1c5dc81dd7dfc4f40471d

    // Load Depth Sensor address into slave address register
    EUSCI_B0->I2CSA = DEPTH_SENSOR_ADDRESS;

    // Enable receive interrupt for communicating with depth sensor
    EUSCI_B0->IE |= EUSCI_B_IE_RXIE0;

    // Enable I2C Module to start operations
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;
}


void EUSCIB1_IRQHandler(void)
{
<<<<<<< HEAD
    // If a transmit interrupt called the handler, check what state we are in
    if(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0)
=======
    // Transmit interrupt
    if (EUSCI_B0->IFG & BIT1)
>>>>>>> 64c4418d77339039b6b1c5dc81dd7dfc4f40471d
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
<<<<<<< HEAD
            // Update state then transmit the actual ADC read command and a stop bit
            i2cState = ADC_READ_STARTED;
            EUSCI_B0->TXBUF = DEPTH_SENSOR_ADC_READ;
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
=======
            i2cStop();
            // Going to be receiving next
            commState = 3;
            // Set to slave mode
            EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
>>>>>>> 64c4418d77339039b6b1c5dc81dd7dfc4f40471d
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
<<<<<<< HEAD
    }

    // If a receive interrupt called the handler, check what state we are in
    if (EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0)
=======
        EUSCI_B0->IFG &= ~(BIT1);
    }

    // Receive interrupt
    else if (EUSCI_B0->IFG & BIT0)
>>>>>>> 64c4418d77339039b6b1c5dc81dd7dfc4f40471d
    {
        // If we just began reading an ADC conversion
        if (i2cState == ADDRESS_SENT_ADC_READING)
        {
<<<<<<< HEAD
            // Update state then read the most significant byte of the 24-bit depth reading
            i2cState = DEPTH_1_RECEIVED;
            depthRead = (EUSCI_B1->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK) << 16;
=======
            depthOne = (EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK);
            commState  = 4;
>>>>>>> 64c4418d77339039b6b1c5dc81dd7dfc4f40471d
        }
        else if (i2cState == DEPTH_1_RECEIVED)
        {
<<<<<<< HEAD
            // Update state then read the middle byte of the 24-bit depth reading
            i2cState = DEPTH_2_RECEIVED;
            depthRead |= (EUSCI_B1->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK) << 8;
=======
            depthTwo = (EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK);
            commState  = 5;
>>>>>>> 64c4418d77339039b6b1c5dc81dd7dfc4f40471d
        }
        else if (i2cState == DEPTH_2_RECEIVED)
        {
<<<<<<< HEAD
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
=======
            depthThree = (EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK);
            commState  = 6;
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
>>>>>>> 64c4418d77339039b6b1c5dc81dd7dfc4f40471d
        }
    }
}




