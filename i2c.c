#include "msp.h"
#include "adc.h"
#include "i2c.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"

// OLD I2C CODE THAT MIGHT NOT WORK EXTERNALLY

void i2cConfigure(void)
{
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
    //Disable TX IE. Polling interrupt flags manually made more sense to me and the communication is fast
    //so the inefficiencies of polling the flags is minimal
    EUSCI_B1->IE &= ~EUSCI_B_IE_TXIE0;
    /* Enable I2C Module to start operations */
    EUSCI_B1->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;
}

void i2cWrite16 (unsigned char pointer, uint16_t writeByte)
{
    // Set master to transmit mode PL
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TR;
    // Clear any existing interrupt flag PL
    EUSCI_B1->IFG &= ~EUSCI_B_IE_TXIE0;
    // Wait until ready to write PL
    while (EUSCI_B1->STATW & EUSCI_B_STATW_BBUSY);
    // Initiate start and send first character
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
    //Wait for TX Buf to be empty
    while (!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));
    EUSCI_B1->TXBUF = pointer;

    // Send the MSB to SENSOR
    //Wait for TX Buf to be empty and send data
    while (!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));
    EUSCI_B1->TXBUF = (unsigned char)(writeByte>>8);
    while (!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));
    EUSCI_B1->TXBUF = (unsigned char)(writeByte&0xFF);
    //Wait for TX Buf to be empty then send stop command
    while (!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
}

int i2cRead16 (unsigned char pointer)
{
    volatile int val = 0;
    volatile int val2 = 0;
    // Set master to transmit mode PL
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TR;
    // Clear any existing interrupt flag PL
    EUSCI_B1->IFG &= ~EUSCI_B_IE_TXIE0 ;
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

    //Set to recieve mode and send start condition
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

void TMP006_init(void)
{
    // Specify slave address for temp sensor
    EUSCI_B1->I2CSA = TMP006_SLAVE_ADDRESS;
    // Reset TMP006
    I2C_write16(TMP006_WRITE_REG, TMP006_RST);
    //delay for the temperature sensor to fully reset
    volatile int i;
    for (i=10000; i>0;i--);

    // Power-up and re-enable device for sampling once a second
    I2C_write16(TMP006_WRITE_REG, TMP006_POWER_UP | TMP006_CR_1);
}

void OPT3001_init(void)
{
    // Specify slave address for light sensor
    EUSCI_B1->I2CSA = OPT3001_SLAVE_ADDRESS;
    //Set Default configuration
    I2C_write16(CONFIG_REG, DEFAULT_CONFIG_100);
}

void I2C_setslave(unsigned int slaveAdr)
{
    // Specify slave address for I2C
    EUSCI_B1->I2CSA = slaveAdr;
    // Enable and clear the interrupt flag to reset communications
    EUSCI_B1->IFG &= ~(EUSCI_B_IE_TXIE0  + EUSCI_B_IE_RXIE0 );
}

int TMP006_readObjectVoltage(void)
{
    // Specify slave address for TMP006
    I2C_setslave(TMP006_SLAVE_ADDRESS);
    //read the data
    return I2C_read16(TMP006_P_VOBJ);
}

int TMP006_readAmbientTemperature(void)
{
    // Specify slave address for TMP006
    I2C_setslave(TMP006_SLAVE_ADDRESS);
    //read the data
    return I2C_read16(TMP006_P_TABT);
}

//This function was taken directly from TI's example code due to the complexity
//of the temperature sensor's mathematics and the necessity of math.h
long double TMP006_getTemp(void)
{
    static long double S0 = 0;
    volatile int Vobj = 0;
    volatile int Tdie = 0;
    /* Read the object voltage */
    Vobj = TMP006_readObjectVoltage();
    /* Read the ambient temperature */
    Tdie = TMP006_readAmbientTemperature();
    Tdie = Tdie >> 2;

    /* Calculate TMP006. This needs to be reviewed and calibrated */
    long double Vobj2 = (double)Vobj*.00000015625;
    long double Tdie2 = (double)Tdie*.03525 + 273.15;

    /* Initialize constants */
    S0 = 6 * pow(10, -14);
    long double a1 = 1.75*pow(10, -3);
    long double a2 = -1.678*pow(10, -5);
    long double b0 = -2.94*pow(10, -5);
    long double b1 = -5.7*pow(10, -7);
    long double b2 = 4.63*pow(10, -9);
    long double c2 = 13.4;
    long double Tref = 298.15;

    /* Calculate values */
    long double S = S0*(1+a1*(Tdie2 - Tref)+a2*pow((Tdie2 - Tref),2));
    long double Vos = b0 + b1*(Tdie2 - Tref) + b2*pow((Tdie2 - Tref),2);
    volatile long double fObj = (Vobj2 - Vos) + c2*pow((Vobj2 - Vos),2);
    volatile long double Tobj = pow(pow(Tdie2,4) + (fObj/S),.25);
    Tobj = (9.0/5.0)*(Tobj - 273.15) + 20;

    /* Return temperature of object */
    return (Tobj);
}

//This function was taken directly from TI's example code due to the complexity
//of the light sensor's mathematics and all of the cases being correct
uint32_t OPT3001_getLux(void)
{
    /* Specify slave address for OPT3001 */
    I2C_setslave(OPT3001_SLAVE_ADDRESS);

    volatile uint16_t exponent = 0;
    volatile uint32_t result = 0;
    volatile uint32_t raw;
    raw = I2C_read16(RESULT_REG);
    /*Convert to LUX*/
    //extract result & exponent data from raw readings
    result = raw&0x0FFF;
    exponent = (raw>>12)&0x000F;
    //convert raw readings to LUX
    switch(exponent){
    case 0: //*0.015625
        result = result>>6;
        break;
    case 1: //*0.03125
        result = result>>5;
        break;
    case 2: //*0.0625
        result = result>>4;
        break;
    case 3: //*0.125
        result = result>>3;
        break;
    case 4: //*0.25
        result = result>>2;
        break;
    case 5: //*0.5
        result = result>>1;
        break;
    case 6:
        result = result;
        break;
    case 7: //*2
        result = result<<1;
        break;
    case 8: //*4
        result = result<<2;
        break;
    case 9: //*8
        result = result<<3;
        break;
    case 10: //*16
        result = result<<4;
        break;
    case 11: //*32
        result = result<<5;
        break;
    }
    return result;
}
