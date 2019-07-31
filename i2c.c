#include "i2c.h"


// Declare global variables
extern volatile queue_t* eventList;
extern volatile queue_t* transmit;
extern volatile uint8_t i2cState;
extern volatile uint8_t depthConversionFlag;
extern volatile uint8_t depthRead[3];


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

    // Set clock divider for SMCLK at 12MHz for 200kHz data rate
    // Fscl = Fsmclk / BRW --> 200,000 = 12,000,000 / 60
    // Actually 130kHz because the MSP hates us?
    EUSCI_B0->BRW = 60;

    // Load Depth Sensor address into slave address register
    EUSCI_B0->I2CSA = DEPTH_SENSOR_ADDRESS;

    // Enable receive interrupt for communicating with depth sensor
    EUSCI_B0->IE |= EUSCI_B_IE_RXIE | EUSCI_B_IE_STPIE;

    // Enable I2C Module to start operations
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;

    // Enable interrupts in the NVIC
    NVIC_EnableIRQ(EUSCIB0_IRQn);
}


extern void EUSCIB0_IRQHandler(void)
{
    // If a stop interrupt called the handler, check what state we are in
    if(EUSCI_B0->IFG & EUSCI_B_IFG_STPIFG)
    {
        // If we have sent the conversion command, start timer delay so it actually finishes
        if(i2cState == CONVERSION_STARTED)
        {
            // Reset current timer A0 count
            TA0R = 0;

            // Set the depth conversion flag high
            depthConversionFlag = 1;
        }
        // If an ADC read has begun, prepare to receive data
        else if(i2cState == ADC_READ_STARTED)
        {
            // Update state then set MSP to receive mode for incoming data and disable TX interrupt
            EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;       // Set MSP to receive mode
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;     // Transmit start condition and read mode slave address
            i2cState = ADC_READING;
        }
        else if (i2cState == DEPTH_2_RECEIVED)
        {
            // Read the middle byte of the 24-bit depth reading
            depthRead[2] = EUSCI_B1->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;

            // Add an event to transmit the depth sensor reading
            if(!queuePush(eventList, DEPTH_SENSOR_READ_FINISH))
            {
                // Enter infinite while loop if queue is full because that's an error
                while(1);
            }

            // Update i2c state
            i2cState = I2C_TRANSMITTING;
        }

        // Clear interrupt flags
        EUSCI_B0->IFG &= EUSCI_B_IFG_STPIFG;
    }

    // If a receive interrupt called the handler, check what state we are in
    if (EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG)
    {
        // If we just began reading an ADC conversion
        if (i2cState == ADC_READING)
        {
            // Read the most significant byte of the 24-bit depth reading and update i2c state
            depthRead[0] = EUSCI_B1->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;
            i2cState = DEPTH_1_RECEIVED;
        }
        else if (i2cState == DEPTH_1_RECEIVED)
        {
            // Read the middle byte of the 24-bit depth reading
            depthRead[1] = EUSCI_B1->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;

            // Set stop condition to send NACK and STOP after next receive, and update i2c state
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
            i2cState = DEPTH_2_RECEIVED;
        }

        // Clear interrupt flags
        EUSCI_B0->IFG &= EUSCI_B_IFG_RXIFG;
    }
}




