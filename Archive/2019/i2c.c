#include "i2c.h"


// Declare global variables
extern volatile queue_t* eventList;
extern volatile queue_t* transmit;
extern volatile uint8_t i2cState;
extern volatile uint8_t depthAdcConversionFlag;
extern volatile uint8_t depthSensorConnected;
extern volatile uint8_t depthRead[6];


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
    // Fscl = Fsmclk / BRW --> 400,000 = 12,000,000 / 30
    // Actually 370ish kHz because the MSP hates us?
    EUSCI_B0->BRW = 30;

    // Clock low timeout occurs after 165000 SYSCLK cycles (about 34ms)
    EUSCI_B0->CTLW1 &= ~EUSCI_B_CTLW1_CLTO_MASK;
    EUSCI_B0->CTLW1 |= EUSCI_B_CTLW1_CLTO_3;

    // Enable I2C Module to start operations
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;

    // Enable interrupts in the NVIC
    NVIC_EnableIRQ(EUSCIB0_IRQn);
}


uint8_t i2cTransmit(uint8_t address, uint8_t command)
{
    // Create error handling variables
    uint32_t timeout = 0;

    // If I2C is ready for another conversion, begin the next conversion
    EUSCI_B0->I2CSA = address;                  // Load in the given address
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR;        // Set MSP to transmit mode
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;     // Transmit start condition and write mode slave address

    // Wait until we can send command data
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0))
    {
        timeout++;
        // If a timeout occurs, we have failed
        if(timeout >= 10000)
        {
            return 0;
        }
    }

    // Transmit the command
    EUSCI_B0->TXBUF = command;

    // Wait until we can send a stop
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0))
    {
        timeout++;
        // If a timeout occurs, we have failed
        if(timeout >= 10000)
        {
            return 0;
        }
    }

    // Clear the transmission flag before next transmission and send a stop
    EUSCI_B0->IFG &= ~EUSCI_B_IFG_TXIFG0;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
    return 1;
}


void i2cReceiveInit(uint8_t address)
{
    // If I2C is ready for another conversion, begin the next conversion
    EUSCI_B0->I2CSA = address;                  // Load in the given address
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;       // Set MSP to transmit mode
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;     // Transmit start condition and write mode slave address
}


uint8_t depthSensorInit(void)
{
    // Create error handling variables
    uint32_t timeout = 0;

    // Reset I2C bus, wait until depth sensor is not busy, then clear timeout flag
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST;
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;
    if(EUSCI_B0->STATW & EUSCI_B_STATW_BBUSY) return 0;

    // Reset depth sensor
    if(!i2cTransmit(DEPTH_SENSOR_ADDRESS, DEPTH_SENSOR_RESET_COMMAND))
    {
        return 0;
    }

    // Precision delay after reset
    int i = 0;
    for(i = 0;i < 1200037;i++);

    // Read and transmit PROM calibration data
    uint8_t calibrationData[] = {0,0,0,0,0,0,0,0,0,0,0,0};

    // Send read command for PROM address 2
    if(!i2cTransmit(DEPTH_SENSOR_ADDRESS, DEPTH_SENSOR_PROM_2_READ))
    {
        return 0;
    }

    // Read from PROM address 2
    i2cReceiveInit(DEPTH_SENSOR_ADDRESS);
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG))                        // Wait until data is received from depth sensor
    {
        timeout++;
        // If a timeout occurs, we have failed
        if(timeout >= 10000)
        {
            return 0;
        }
    }
    calibrationData[0] = EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;    // Read data from buffer once it is received
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;                             // Prepare stop before last byte is received
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG))                         // Wait until data is received from depth sensor
    {
        timeout++;
        // If a timeout occurs, we have failed
        if(timeout >= 10000)
        {
            return 0;
        }
    }
    calibrationData[1] = EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;    // Read data from buffer once it is received
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_STPIFG))                        // Wait until stop interrupt is set
    {
        timeout++;
        // If a timeout occurs, we have failed
        if(timeout >= 10000)
        {
            return 0;
        }
    }
    EUSCI_B0->IFG &= ~EUSCI_B_IFG_STPIFG;                               // Clear stop flag before moving on

    // Send read command for PROM address 4
    if(!i2cTransmit(DEPTH_SENSOR_ADDRESS, DEPTH_SENSOR_PROM_4_READ))
    {
        return 0;
    }

    // Read from PROM address 4
    i2cReceiveInit(DEPTH_SENSOR_ADDRESS);
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG))                        // Wait until data is received from depth sensor
    {
        timeout++;
        // If a timeout occurs, we have failed
        if(timeout >= 10000)
        {
            return 0;
        }
    }
    calibrationData[2] = EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;    // Read data from buffer once it is received
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;                             // Prepare stop before last byte is received
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG))                         // Wait until data is received from depth sensor
    {
        timeout++;
        // If a timeout occurs, we have failed
        if(timeout >= 10000)
        {
            return 0;
        }
    }
    calibrationData[3] = EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;    // Read data from buffer once it is received
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_STPIFG))                        // Wait until stop interrupt is set
    {
        timeout++;
        // If a timeout occurs, we have failed
        if(timeout >= 10000)
        {
            return 0;
        }
    }
    EUSCI_B0->IFG &= ~EUSCI_B_IFG_STPIFG;                               // Clear stop flag before moving on

    // Send read command for PROM address 6
    if(!i2cTransmit(DEPTH_SENSOR_ADDRESS, DEPTH_SENSOR_PROM_6_READ))
    {
        return 0;
    }

    // Read from PROM address 6
    i2cReceiveInit(DEPTH_SENSOR_ADDRESS);
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG))                        // Wait until data is received from depth sensor
    {
        timeout++;
        // If a timeout occurs, we have failed
        if(timeout >= 10000)
        {
            return 0;
        }
    }
    calibrationData[4] = EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;    // Read data from buffer once it is received
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;                             // Prepare stop before last byte is received
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG))                         // Wait until data is received from depth sensor
    {
        timeout++;
        // If a timeout occurs, we have failed
        if(timeout >= 10000)
        {
            return 0;
        }
    }
    calibrationData[5] = EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;    // Read data from buffer once it is received
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_STPIFG))                        // Wait until stop interrupt is set
    {
        timeout++;
        // If a timeout occurs, we have failed
        if(timeout >= 10000)
        {
            return 0;
        }
    }
    EUSCI_B0->IFG &= ~EUSCI_B_IFG_STPIFG;                               // Clear stop flag before moving on

    // Send read command for PROM address 8
    if(!i2cTransmit(DEPTH_SENSOR_ADDRESS, DEPTH_SENSOR_PROM_8_READ))
    {
        return 0;
    }

    // Read from PROM address 8
    i2cReceiveInit(DEPTH_SENSOR_ADDRESS);
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG))                        // Wait until data is received from depth sensor
    {
        timeout++;
        // If a timeout occurs, we have failed
        if(timeout >= 10000)
        {
            return 0;
        }
    }
    calibrationData[6] = EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;    // Read data from buffer once it is received
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;                             // Prepare stop before last byte is received
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG))                         // Wait until data is received from depth sensor
    {
        timeout++;
        // If a timeout occurs, we have failed
        if(timeout >= 10000)
        {
            return 0;
        }
    }
    calibrationData[7] = EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;    // Read data from buffer once it is received
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_STPIFG))                        // Wait until stop interrupt is set
    {
        timeout++;
        // If a timeout occurs, we have failed
        if(timeout >= 10000)
        {
            return 0;
        }
    }
    EUSCI_B0->IFG &= ~EUSCI_B_IFG_STPIFG;                               // Clear stop flag before moving on

    // Send read command for PROM address A
    if(!i2cTransmit(DEPTH_SENSOR_ADDRESS, DEPTH_SENSOR_PROM_A_READ))
    {
        return 0;
    }

    // Read from PROM address A
    i2cReceiveInit(DEPTH_SENSOR_ADDRESS);
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG))                        // Wait until data is received from depth sensor
    {
        timeout++;
        // If a timeout occurs, we have failed
        if(timeout >= 10000)
        {
            return 0;
        }
    }
    calibrationData[8] = EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;    // Read data from buffer once it is received
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;                             // Prepare stop before last byte is received
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG))                         // Wait until data is received from depth sensor
    {
        timeout++;
        // If a timeout occurs, we have failed
        if(timeout >= 10000)
        {
            return 0;
        }
    }
    calibrationData[9] = EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;    // Read data from buffer once it is received
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_STPIFG))                        // Wait until stop interrupt is set
    {
        timeout++;
        // If a timeout occurs, we have failed
        if(timeout >= 10000)
        {
            return 0;
        }
    }
    EUSCI_B0->IFG &= ~EUSCI_B_IFG_STPIFG;                               // Clear stop flag before moving on

    // Send read command for PROM address C
    if(!i2cTransmit(DEPTH_SENSOR_ADDRESS, DEPTH_SENSOR_PROM_C_READ))
    {
        return 0;
    }

    // Read from PROM address C
    i2cReceiveInit(DEPTH_SENSOR_ADDRESS);
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG))                        // Wait until data is received from depth sensor
    {
        timeout++;
        // If a timeout occurs, we have failed
        if(timeout >= 10000)
        {
            return 0;
        }
    }
    calibrationData[10] = EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;    // Read data from buffer once it is received
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;                             // Prepare stop before last byte is received
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG))                         // Wait until data is received from depth sensor
    {
        timeout++;
        // If a timeout occurs, we have failed
        if(timeout >= 10000)
        {
            return 0;
        }
    }
    calibrationData[11] = EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;    // Read data from buffer once it is received
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_STPIFG))                        // Wait until stop interrupt is set
    {
        timeout++;
        // If a timeout occurs, we have failed
        if(timeout >= 10000)
        {
            return 0;
        }
    }
    EUSCI_B0->IFG &= ~EUSCI_B_IFG_STPIFG;                               // Clear stop flag before moving on

    // Prepare data for transmission
    queuePush(transmit, START_STOP_FRAME);
    queuePush(transmit, CALIBRATION_DATA_FRAME);
    for(i = 0;i < 12;i++)
    {
        // If any frame is a start stop frame, queue proper escape sequence
        if(calibrationData[i] == START_STOP_FRAME)
        {
            queuePush(transmit, ESCAPE_FRAME);
            queuePush(transmit, C0_FRAME);
        }

        // If any frame is an escape frame, queue proper escape sequence
        else if(calibrationData[i] == ESCAPE_FRAME)
        {
            queuePush(transmit, ESCAPE_FRAME);
            queuePush(transmit, DB_FRAME);
        }

        // If the data is not a special case, queue it
        else
        {
            queuePush(transmit, calibrationData[i]);
        }
    }

    // Queue a stop frame and begin data transmission
    queuePush(transmit, START_STOP_FRAME);
    uartBeginCompTransmit();

    // Enable receive, stop and clock low timeout interrupts for communicating with depth sensor
    EUSCI_B0->IE |= EUSCI_B_IE_RXIE | EUSCI_B_IE_STPIE;
    return 1;
}

extern void EUSCIB0_IRQHandler(void)
{
    // If a stop interrupt called the handler, check what state we are in
    if(EUSCI_B0->IFG & EUSCI_B_IFG_STPIFG)
    {
        // Clear interrupt flags
        EUSCI_B0->IFG &= ~EUSCI_B_IFG_STPIFG;

        // If we have sent the conversion command, start timer delay so it actually finishes
        if(i2cState == DEPTH_CONVERSION_STARTED)
        {
            // Reset current timer A0 count
            TA0R = 0;

            // Set the depth conversion flag high
            depthAdcConversionFlag = 1;
        }

        // If a depth ADC read has begun, prepare to receive data
        else if(i2cState == DEPTH_READ_STARTED)
        {
            // Initiate data receiving and update the i2c state
            i2cReceiveInit(DEPTH_SENSOR_ADDRESS);
            i2cState = DEPTH_READING;
        }

        // If we have received the second byte of depth data, grab the last and begin a temperature conversion
        else if(i2cState == DEPTH_2_RECEIVED)
        {
            // Read the least significant byte of the 24-bit depth reading
            depthRead[2] = EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;

            // Update the i2c state
            i2cState = TEMP_CONVERSION_STARTED;

            // Begin a temperature conversion
            if(!i2cTransmit(DEPTH_SENSOR_ADDRESS, DEPTH_SENSOR_TEMP_CONVERSION))
            {
                i2cState = I2C_READY;
                depthSensorConnected = 0;
                depthAdcConversionFlag = 0;
            }
        }

        // If we have begun a temperature conversion, start the delay process
        else if(i2cState == TEMP_CONVERSION_STARTED)
        {
            // Reset current timer A0 count
            TA0R = 0;

            // Set the depth conversion flag high
            depthAdcConversionFlag = 1;
        }

        // If a temperature ADC read has begun, prepare to receive data
        else if(i2cState == TEMP_READ_STARTED)
        {
            // Initiate data receiving and update the i2c state
            i2cReceiveInit(DEPTH_SENSOR_ADDRESS);
            i2cState = TEMP_READING;
        }

        // If we have received the second byte of temperature data, grab the last and begin a transmission
        else if(i2cState == TEMP_2_RECEIVED)
        {
            // Read the least significant byte of the 24-bit depth reading
            depthRead[5] = EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;

            // Add an event to transmit the depth sensor reading
            if(!queuePush(eventList, DEPTH_SENSOR_READ_FINISH))
            {
                // Enter infinite while loop if queue is full because that's an error
                while(1);
            }

            // Update i2c state
            i2cState = I2C_TRANSMITTING;
        }
    }

    // If a receive interrupt called the handler, check what state we are in
    if(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG)
    {
        // NOTE: Reading EUSCI_B0->RXBUF clears the RXIFG automatically
        // If we just began reading a depth ADC conversion, read first data byte
        if(i2cState == DEPTH_READING)
        {
            // Read the most significant byte of the 24-bit depth reading and update i2c state
            depthRead[0] = EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;
            i2cState = DEPTH_1_RECEIVED;
        }

        // If we have received the first byte of depth data, read second data byte
        else if(i2cState == DEPTH_1_RECEIVED)
        {
            // Read the middle byte of the 24-bit depth reading
            depthRead[1] = EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;

            // Set stop condition to send NACK and STOP after next receive, and update i2c state
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
            i2cState = DEPTH_2_RECEIVED;
        }

        // If we just began reading a temperature ADC conversion, read first data byte
        else if(i2cState == TEMP_READING)
        {
            // Read the most significant byte of the 24-bit temperature reading and update i2c state
            depthRead[3] = EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;
            i2cState = TEMP_1_RECEIVED;
        }

        // If we have received the first byte of temperature data, read second data byte
        else if(i2cState == TEMP_1_RECEIVED)
        {
            // Read the middle byte of the 24-bit depth reading
            depthRead[4] = EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;

            // Set stop condition to send NACK and STOP after next receive, and update i2c state
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
            i2cState = TEMP_2_RECEIVED;
        }
    }
}




