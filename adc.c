#include "adc.h"


// Declare global variables
extern volatile queue_t* eventList;


void adcConfigure(void)
{
    // Prepare for configuration
    while(REF_A->CTL0 & REF_A_CTL0_GENBUSY);                    // Wait until reference generator isn't busy
    REF_A->CTL0 = REF_A_CTL0_VSEL_0 | REF_A_CTL0_ON;             // Enable internal 1.2V reference
    REF_A->CTL0 |= REF_A_CTL0_TCOFF;                            // Turn off the temperature sensor

    // Configure ADC control registers
    ADC14->CTL0     &= ~ADC14_CTL0_ENC;                         // Disable ADC Encoding to allow changing settings
    ADC14->CTL0     |= ADC14_CTL0_SHT0_3 | ADC14_CTL0_SHT1_3 |  // 32 clock cycle sample-and-hold times
                       ADC14_CTL0_CONSEQ_1 | ADC14_CTL0_ON |    // Read in sequence mode and turn ADC14 on
                       ADC14_CTL0_SHP | ADC14_CTL0_MSC;         // Source signal from the sampling timer and enable multiple samples
    ADC14->CTL1     &= ADC14_CTL1_RES_MASK;                     // Make sure resolution bits are cleared
    ADC14->CTL1     |= ADC14_CTL1_RES__14BIT;                   // Set ADC to 14-bit resolution

    // Map channel AXX to MCTL[XX] and set channels 7 and 13 to be end of sequence bits
    ADC14->MCTL[0]  |= ADC14_MCTLN_INCH_0;                      // Motor 1
    ADC14->MCTL[1]  |= ADC14_MCTLN_INCH_1;                      // Motor 2
    ADC14->MCTL[2]  |= ADC14_MCTLN_INCH_2;                      // Motor 3
    ADC14->MCTL[3]  |= ADC14_MCTLN_INCH_3;                      // Motor 4
    ADC14->MCTL[4]  |= ADC14_MCTLN_INCH_4;                      // Motor 5
    ADC14->MCTL[5]  |= ADC14_MCTLN_INCH_5;                      // Motor 6
    ADC14->MCTL[6]  |= ADC14_MCTLN_INCH_6;                      // Motor 7
    ADC14->MCTL[7]  |= ADC14_MCTLN_INCH_7 | ADC14_MCTLN_EOS;    // Motor 8 (End of motor conversion sequence)
    ADC14->MCTL[8]  |= ADC14_MCTLN_INCH_8;                      // 3.3V or 5V current
    ADC14->MCTL[9]  |= ADC14_MCTLN_INCH_9;                      // 9V current
    ADC14->MCTL[10] |= ADC14_MCTLN_INCH_10;                     // 12V current
    ADC14->MCTL[11] |= ADC14_MCTLN_INCH_11;                     // 19V current
    ADC14->MCTL[12] |= ADC14_MCTLN_INCH_12;                     // 48V current
    ADC14->MCTL[13] |= ADC14_MCTLN_INCH_13 | ADC14_MCTLN_EOS;   // Battery voltage (End of power status conversion sequence)

    // Enable interrupts for the EOS bits
    ADC14->IER0     |= ADC14_IER0_IE7 | ADC14_IER0_IE13;        // Allow interrupts on EOS bits

    // Wait for reference generator to be ready, then exit configuration
    while(!(REF_A->CTL0 & REF_A_CTL0_GENRDY));
    ADC14->CTL0     |= ADC14_CTL0_ENC;                          // Enable ADC Encoding
    NVIC_EnableIRQ(ADC14_IRQn);                                 // Enable interrupts on NVIC
}


extern void ADC14_IRQHandler(void)
{
    // If MEM7 triggered the interrupt, add a Motor Current Read Finish to the scheduler
    if(ADC14->IFGR0 & ADC14_IFGR0_IFG7)
    {
        if(!queuePush(eventList, MOTOR_CURRENT_READ_FINISH))
        {
            // If schedule stack is full, enter while loop for debugging purposes
            while(1);
        }

        // Clear MEM7 interrupt flag
        ADC14->CLRIFGR0 = ADC14_IFGR0_IFG7;
    }

    // If MEM13 triggered the interrupt, read power status
    if(ADC14->IFGR0 & ADC14_IFGR0_IFG13)
    {
        if(!queuePush(eventList, POWER_CURRENT_READ_FINISH))
        {
            // If schedule stack is full, enter while loop for debugging purposes
            while(1);
        }

        // Clear MEM13 interrupt flag
        ADC14->CLRIFGR0 = ADC14_IFGR0_IFG13;
    }
}
