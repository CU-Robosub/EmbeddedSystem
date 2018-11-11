#include "msp.h"
#include "adc.h"
#include "i2c.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"


// Declare global variables
uint8_t motorCurrents[16];
uint8_t powerStatus[12];

void ADC14_IRQHandler(void)
{
    // If MEM7 triggered the interrupt, read motor currents
    if(ADC14->IFGR0 & ADC14_IFGR0_IFG7)
    {
        int i;
        for(i = 0;i < 8;i++)
        {
            // Read MEM0-MEM7
            motorCurrents[2*i] = (uint8_t) ADC14->MEM[i] & 0xFF;             // Store lower half of conversions
            motorCurrents[(2*i)+1] = (uint8_t) (ADC14->MEM[i] >> 8) & 0xFF;  // Store upper half of conversions
        }
        uartSendCompByte((uint8_t) 0xFF);                                    // Transmit 0xFF to denote motor data
        uartSendCompN(motorCurrents, 16);                                    // Transmit data to computer
        startReadMotorCurrents();                                            // Begin another motor read
    }

    // If MEM13 triggered the interrupt, read power status
    if(ADC14->IFGR0 & ADC14_IFGR0_IFG13)
    {
        int i;
        for(i = 0;i < 6;i++)
        {
            // Read MEM8-MEM13
            powerStatus[2*i] = (uint8_t) ADC14->MEM[i+8] & 0xFF;               // Store lower half of conversions
            powerStatus[(2*i)+1] = (uint8_t) (ADC14->MEM[i+8] >> 8) & 0xFF;    // Store upper half of conversions
        }
        setLEDColor((uint16_t)ADC14->MEM[13] & 0xFFFF);                        // Select color based on battery reading
        uartSendCompByte((uint8_t) 0xFE);                                      // Transmit 0xFE to denote power data
        uartSendCompN(powerStatus, 12);                                        // Transmit data to computer
    }
}

void adcConfiguration()
{
    ADC14->CTL0     &= ~ADC14_CTL0_ENC;                          // Disable ADC Encoding to allow changing settings
    ADC14->CTL0     |= ADC14_CTL0_SHT0_3 | ADC14_CTL0_SHT1_3 |   // 32 clock cycle sample-and-hold times
                       ADC14_CTL0_CONSEQ_1 | ADC14_CTL0_ON;      // Read in sequence mode and turn ADC14 on
    ADC14->CTL1     |= ADC14_CTL1_RES__14BIT;                    // Set ADC to 14-bit resolution

    // Map channel AXX to MCTL[XX] and set channels 7 and 13 to be end of sequence bits
    ADC14->MCTL[0]  |= ADC14_MCTLN_INCH_0;                       // Motor 1
    ADC14->MCTL[1]  |= ADC14_MCTLN_INCH_1;                       // Motor 2
    ADC14->MCTL[2]  |= ADC14_MCTLN_INCH_2;                       // Motor 3
    ADC14->MCTL[3]  |= ADC14_MCTLN_INCH_3;                       // Motor 4
    ADC14->MCTL[4]  |= ADC14_MCTLN_INCH_4;                       // Motor 5
    ADC14->MCTL[5]  |= ADC14_MCTLN_INCH_5;                       // Motor 6
    ADC14->MCTL[6]  |= ADC14_MCTLN_INCH_6;                       // Motor 7
    ADC14->MCTL[7]  |= ADC14_MCTLN_INCH_7 | ADC14_MCTLN_EOS;     // Motor 8
    ADC14->MCTL[8]  |= ADC14_MCTLN_INCH_8;                       // 3.3V or 5V current
    ADC14->MCTL[9]  |= ADC14_MCTLN_INCH_9;                       // 9V current
    ADC14->MCTL[10] |= ADC14_MCTLN_INCH_10;                      // 12V current
    ADC14->MCTL[11] |= ADC14_MCTLN_INCH_11;                      // 19V current
    ADC14->MCTL[12] |= ADC14_MCTLN_INCH_12;                      // 48V current
    ADC14->MCTL[13] |= ADC14_MCTLN_INCH_13 | ADC14_MCTLN_EOS;    // Battery voltage

    ADC14->IER0     |= ADC14_IER0_IE7 | ADC14_IER0_IE13;         // Allow interrupts on EOS bits
    ADC14->CTL0     |= ADC14_CTL0_ENC;                           // Enable ADC Encoding
    NVIC_EnableIRQ(ADC14_IRQn);                                  // Enable interrupts on NVIC
}

void startReadMotorCurrents()
{
    ADC14->CTL1 &= ~ADC14_CTL1_CSTARTADD_MASK;         // Set start address to MEM0
    while(!(ADC14->CTL0 & ADC14_CTL0_BUSY));           // Wait until the ADC is not busy
    ADC14->CTL0 |= ADC14_CTL0_SC;                      // Start a conversion
}

void startReadPowerStatus()
{
    ADC14->CTL1 &= ~ADC14_CTL1_CSTARTADD_MASK;         // Clear start address bits
    ADC14->CTL1 |= 0x00080000;                         // Set start address to MEM8
    while(!(ADC14->CTL0 & ADC14_CTL0_BUSY));           // Wait until the ADC is not busy
    ADC14->CTL0 |= ADC14_CTL0_SC;                      // Start a conversion
}
