#include "msp.h"
#include "adc.h"
#include "i2c.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"




void i2cConfigure(void)
{
    //Initialize global variables
    commState = 0;
    writeData = 0;
    depthOne = 0;
    depthTwo = 0;
    depthThree = 0;

    // Initialize USCI_B1 and I2C Master to communicate with slave devices
    //Reset all registers/disable, master, I2C mode, SMCLK
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_SWRST;
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_MST;
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_MODE_3;
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_SSEL__SMCLK;
    //no autostop to manually control when stop bit is sent
    EUSCI_B1->CTLW1 = EUSCI_B_CTLW1_ASTP_0;
    //set clock divider for SMCLK at 3MHz for 400KBPS data rate
    EUSCI_B1->BRW = (uint16_t) (3000000 / 40000);

    //enable interrupts from slave in I2COA0 and sending data
    EUSCI_B1->IE |= EUSCI_B_IE_TXIE0;
    EUSCI_B1->IE |= EUSCI_B_IE_RXIE0;
    /* Enable I2C Module to start operations */
    EUSCI_B1->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;
}

void i2cWrite8Start (uint8_t writeByte)
{
    commState = 0;
    // Set master to transmit mode
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TR;
    // Clear any existing interrupt flag
    EUSCI_B1->IFG &= ~EUSCI_B_IFG_TXIFG0;
    // Wait until buffer is clear and ready to write
    while (EUSCI_B1->STATW & EUSCI_B_STATW_BBUSY);
    // Initiate start and send pointer in I2CSA buffer
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
    writeData = writeByte;
}

void i2cWrite8Data (void)
{
    commState = 1;
    // send data to slave
    EUSCI_B1->TXBUF = writeData;
}

void i2cStop (void)
{
    //Wait for TX Buf to be empty then send stop command
    while (!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
}

/////OLD FUNCTION HERE////
int i2cRead16 (unsigned char pointer)
{
    volatile int val = 0;
    volatile int val2 = 0;
    volatile int val3 = 0;
    // Set master to transmit mode
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TR;
    // Clear any existing interrupt flag
    EUSCI_B1->IFG &= ~EUSCI_B_IFG_RXIFG0;
    // Wait until ready to write PL
    while (EUSCI_B1->STATW & EUSCI_B_STATW_BBUSY);
    // Initiate start and send first character
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
    //Wait for TX Buf to be empty
    while (!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));
    EUSCI_B1->TXBUF = pointer;
    //Wait for TX Buf to be empty and send stop data
    //for the slave to take control of communications
    while (!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
    while (!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));

    //Set to receive mode and send start condition
    EUSCI_B1->CTLW0 &= ~EUSCI_B_CTLW0_TR;
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
    //wait for the RX Buf to fill
    while (!(EUSCI_B1->IFG & EUSCI_B_IE_RXIE0));
    //read first 16 bits
    val = (EUSCI_B1->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK);
    //send stop command and wait for TXBUF and RXBuf to finish
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
    while (!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));
    while (!(EUSCI_B1->IFG & EUSCI_B_IE_RXIE0));
    //Read rest of RXBUF
    val2 = (EUSCI_B1->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK);
    // Shift val to top MSB
    val = (val << 8);
    // Read from I2C RX Register and write to LSB of val
    val |= val2;
    // Return temperature value
    return (int16_t)val;
}

void i2cRead24Start (void)
{
    commState = 2;
    // Set master to transmit mode
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TR;
    // Clear any existing interrupt flag
    EUSCI_B1->IFG &= ~EUSCI_B_IFG_RXIFG0;
    // Wait until ready to write PL
    while (EUSCI_B1->STATW & EUSCI_B_STATW_BBUSY);
    // Initiate start and send first character
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
}

int i2cRead24End (void)
{
    uint32_t finalRead = (depthOne << 16) | (depthTwo << 8) | (depthThree);
    uint8_t *sendRead = {0xDD, depthOne, depthTwo, depthThree};
    uartSendCompN(sendRead, 3);
}

void depthInit(void)
{
    // Specify slave address for depth sensor
    EUSCI_B1->I2CSA = depthWrite;
    //set the read value into the self address for receive command
    EUSCI_B1->I2COA0 |= depthRead;
    EUSCI_B1->I2COA0 |= EUSCI_B_I2COA0_OAEN;
    // reset the sensor
    i2cWrite8Start(depthReset);
}


void depthD12Set(uint8_t D1, uint8_t D2)
{
    //set the data in D1
    i2cWrite8Start(D1);
    //set the data in D2
    i2cWrite8Start(D2);
}

void depthAdcStart(void)
{
    //set the data in D1
    i2cWrite8Start(0x00);
    //start timer
}

//i2cRead24Start ()

void EUSCIB1_IRQHandler  (void)
{
    //transmit interrupt
    if (EUSCI_B1->IFG & BIT1)
    {
        if (commState == 1)
        {
            i2cStop();
        }
        else if (commState == 2)
        {
            i2cStop();
            //going to be receiving next
            commState = 3;
            //set to slave mode
            EUSCI_B1->CTLW0 &= ~EUSCI_B_CTLW0_TR;
            EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        }
        else if (commState == 6)
        {
            i2cRead24End ();
        }
        else
        {
            i2cWrite8Data ();
        }
        EUSCI_B1->IFG &= ~(BIT1);
    }

    //Receive interrupt
    else if (EUSCI_B1->IFG & BIT0)
    {
        if (commState == 3)
        {
            depthOne = (EUSCI_B1->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK);
            commState  = 4;
        }
        else if (commState == 4)
        {
            depthTwo = (EUSCI_B1->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK);
            commState  = 5;
        }
        else if (commState == 5)
        {
            depthThree = (EUSCI_B1->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK);
            commState  = 6;
            EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        }
    }
}




