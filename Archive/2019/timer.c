#include "timer.h"


// Declare global variables
extern volatile queue_t* eventList;
extern volatile queue_t* transmit;
extern volatile uint8_t powerConversionDone;
extern volatile uint8_t motorConversionDone;
extern volatile uint8_t depthAdcConversionFlag;
extern volatile uint8_t depthSensorConnected;
extern volatile uint8_t i2cState;
extern volatile uint16_t tickCounter;
extern volatile uint16_t pnumaticsOffTick[8];

void timerConfigurePnumatics(void)
{

    TA0CTL   |= TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__8 |   // Clock source: SMCLK, Clock divider: 8
                TIMER_A_CTL_MC__UP;                             // Counting: Up Mode
    TA0CCTL0 |= TIMER_A_CCTLN_CCIE;                             // Enable CCTL0 interrupt
    TA0CCTL0 &= ~TIMER_A_CCTLN_CCIFG;                           // Clear CCTL0 interrupt flag
    TA0EX0   |= TIMER_A_EX0_IDEX__5;                            // Input divider expansion: 5
    TA0CCR0   = 5000;                                           // 5000 clock cycles per interrupt
    NVIC_EnableIRQ(TA0_0_IRQn);                                 // Enable interrupts on the NVIC

}

void timerConfigure(void)
{
    /*** Configure TA0 to control interrupts ***
     * Clock Frequency = 300 kHz
     * Interrupt Frequency = 120 Hz
     ******************************************/
    TA0CTL   |= TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__8 |   // Clock source: SMCLK, Clock divider: 8
                TIMER_A_CTL_MC__UP;                             // Counting: Up Mode
    TA0CCTL0 |= TIMER_A_CCTLN_CCIE;                             // Enable CCTL0 interrupt
    TA0CCTL0 &= ~TIMER_A_CCTLN_CCIFG;                           // Clear CCTL0 interrupt flag
    TA0EX0   |= TIMER_A_EX0_IDEX__5;                            // Input divider expansion: 5
    TA0CCR0   = 2500;                                            // 2500 clock cycles per interrupt
    NVIC_EnableIRQ(TA0_0_IRQn);                                 // Enable interrupts on the NVIC

    /********* Configure TA1 and TA2 to control motor PWM signals **********
     * Timer Clock Frequency = 1 MHz, the SMCLK (12 MHz) divided by 12
     * Timer Clock Period = 1 microseconds, gives good speed resolution
     * PWM Period = 20000 Clock Cycles = 20000 microseconds
     *  NOTE: The desired PWM frequency is 50Hz, which is how this period
     *        was selected.
     * PWM On-Time Range = 1100 to 1900 microseconds
     *   - 1100 microseconds (1100 clocks) = Maximum Reverse Speed to Motor
     *   - 1500 microseconds (1500 clocks) = Motor Stopped
     *   - 1900 microseconds (1900 clocks) = Maximum Forward Speed to Motor
     **********************************************************************/
    TA1CTL   |= TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__4 |   // Clock source: SMCLK, Clock divider: 4
                TIMER_A_CTL_MC__UP;                             // Counting: Up Mode (Timer A1)
    TA2CTL   |= TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__4 |   // Clock source: SMCLK, Clock divider: 4
                TIMER_A_CTL_MC__UP;                             // Counting: Up Mode (Timer A2)
    TA1EX0   |= TIMER_A_EX0_IDEX__3;                            // Input divider expansion: 3 (Timer A1)
    TA2EX0   |= TIMER_A_EX0_IDEX__3;                            // Input divider expansion: 3 (Timer A2)
    TA1CCTL1 |= TIMER_A_CCTLN_OUTMOD_6;
    TA1CCTL2 |= TIMER_A_CCTLN_OUTMOD_6;                         // Timer A1 output channels
    TA1CCTL3 |= TIMER_A_CCTLN_OUTMOD_6;                         // * Set all PWM compare registers to output
    TA1CCTL4 |= TIMER_A_CCTLN_OUTMOD_6;                         // * mode 6, toggle/set, to make the PWM signal
    TA2CCTL1 |= TIMER_A_CCTLN_OUTMOD_6;                         // * simple. Info about output modes is on pages
    TA2CCTL2 |= TIMER_A_CCTLN_OUTMOD_6;                         // * 611-614 of the technical reference manual
    TA2CCTL3 |= TIMER_A_CCTLN_OUTMOD_6;                         // Timer A2 output channels
    TA2CCTL4 |= TIMER_A_CCTLN_OUTMOD_6;
    TA1CCR0   = 20000;                                          // 20000 microsecond PWM period
    TA2CCR0   = 20000;
    TA1CCR1   = 1500;                                           // 1500 microsecond on-time, motor stopped
    TA1CCR2   = 1500;                                           // 1500 microsecond on-time, motor stopped
    TA1CCR3   = 1500;                                           // 1500 microsecond on-time, motor stopped
    TA1CCR4   = 1500;                                           // 1500 microsecond on-time, motor stopped
    TA2CCR1   = 1500;                                           // 1500 microsecond on-time, motor stopped
    TA2CCR2   = 1500;                                           // 1500 microsecond on-time, motor stopped
    TA2CCR3   = 1500;                                           // 1500 microsecond on-time, motor stopped
    TA2CCR4   = 1500;                                           // 1500 microsecond on-time, motor stopped
}

void pnumatics_irq(){

    // Assure that the right interrupt flag triggered (should only be one)
    if(!(TA0CCTL0 & TIMER_A_CCTLN_CCIFG))
    {
        // If flag isn't set, return
        return;
    }


    // Clear flag after checking
    TA0CCTL0 &= ~TIMER_A_CCTLN_CCIFG;

    tickCounter++;

    // turn off pnumatics at x tacks
    int i;
    for(i = 0; i < 8; i++)
    {
        if(tickCounter == pnumaticsOffTick[i])
        {
            P3OUT &= ~(1 << i);
        }
    }

}

void embedded_irq(){
    // Assure that the right interrupt flag triggered (should only be one)
    if(!(TA0CCTL0 & TIMER_A_CCTLN_CCIFG))
    {
        // If flag isn't set, return
        return;
    }

    // Clear flag after checking
    TA0CCTL0 &= ~TIMER_A_CCTLN_CCIFG;

    // Declare static counter variables for interrupts which occur slower than 600 Hz
    static uint16_t depthReading = 0;
    static uint16_t motorCurrents = 0;
    static uint16_t powerStatus = 0;

    // If the depth conversion flag is set, start depth sensor read command
    if(depthAdcConversionFlag)
    {
        // Update i2c state
        if(i2cState == DEPTH_CONVERSION_STARTED)
        {
            i2cState = DEPTH_READ_STARTED;
        }
        else if(i2cState == TEMP_CONVERSION_STARTED)
        {
            i2cState = TEMP_READ_STARTED;
        }

        // Transmit a depth sensor read command
        if(!i2cTransmit(DEPTH_SENSOR_ADDRESS, DEPTH_SENSOR_ADC_READ))
        {
            i2cState = I2C_READY;
            depthSensorConnected = 0;
        }

        depthAdcConversionFlag = 0;
    }

#ifdef I2C_ENABLED
    // Increment counter and check if it is time for a depth sensor reading
    depthReading++;
    if(depthReading >= INT_COUNT_15HZ)
    {
        // If this is a 60 Hz interrupt, schedule a depth sensor reading
        if(!queuePush(eventList, DEPTH_SENSOR_READ_START))
        {
            // Enter infinite while loop if queue is full because that's an error
            while(1);
        }

        // Reset the depth reading counter
        depthReading = 0;
    }
#endif

    // Increment counter and check if it is time to read motor currents
    motorCurrents++;
    if((motorCurrents >= INT_COUNT_4HZ) && motorConversionDone)
    {
        // If this is a 4 Hz interrupt, schedule a motor currents reading
        if(!queuePush(eventList, MOTOR_CURRENT_READ_START))
        {
            // Enter infinite while loop if queue is full because that's an error
            while(1);
        }

        // Reset the motor currents counter
        motorCurrents = 0;

        // Indicate that a motor conversion has begun
        motorConversionDone = 0;
    }

    // Increment counter and check if it is time to read power currents
    powerStatus++;
    if((powerStatus >= INT_COUNT_1HZ) && powerConversionDone)
    {
        // If this is a 1 Hz interrupt, schedule a power check
        if(!queuePush(eventList, POWER_CURRENT_READ_START))
        {
            // Enter infinite while loop if queue is full because that's an error
            while(1);
        }

        // Reset the power check counter
        powerStatus = 0;

        // Indicate that a power conversion has begun
        powerConversionDone = 0;
    }
}

// Crude scheduler interrupt routine
extern void TA0_0_IRQHandler(void)
{
    #ifdef EMBEDDED_SYSTEM
    embedded_irq();
    #endif
    #ifdef PNUMATICS_SYSTEM
    pnumatics_irq();
    #endif
}
