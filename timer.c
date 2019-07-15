#include "timer.h"


// Declare global variables
extern volatile queue_t* eventList;
extern volatile queue_t* transmit;
extern volatile uint8_t powerConversionDone;
extern volatile uint8_t motorConversionDone;


void timerConfigure(void)
{
    /*** Configure TA0 to control interrupts ***
     * Clock Frequency = 300 kHz
     * Interrupt Frequency = 60 Hz
     ******************************************/
    TA0CTL   |= TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__8 |   // Clock source: SMCLK, Clock divider: 8
                TIMER_A_CTL_MC__UP;                             // Counting: Up Mode
    TA0CCTL0 |= TIMER_A_CCTLN_CCIE;                             // Enable CCTL0 interrupt
    TA0CCTL0 &= ~TIMER_A_CCTLN_CCIFG;                           // Clear CCTL0 interrupt flag
    TA0EX0   |= TIMER_A_EX0_IDEX__5;                            // Input divider expansion: 5
    TA0CCR0   = 5000;                                           // 5000 clock cycles per interrupt
    NVIC_EnableIRQ(TA0_0_IRQn);                                 // Enable interrupts on the NVIC

    /********* Configure TA1 and TA2 to control motor PWM signals **********
     * Timer Clock Frequency = 2 MHz, the SMCLK (12 MHz) divided by 6
     * Timer Clock Period = 0.5 microseconds, gives good speed resolution
     * PWM Period = 40000 Clock Cycles = 20000 microseconds
     *  NOTE: The desired PWM frequency is 50Hz, which is how this period
     *        was selected.
     * PWM On-Time Range = 1100 to 1900 microseconds
     *   - 1100 microseconds (2200 clocks) = Maximum Reverse Speed to Motor
     *   - 1500 microseconds (3000 clocks) = Motor Stopped
     *   - 1900 microseconds (3800 clocks) = Maximum Forward Speed to Motor
     * This setup gives a resolution of 800 different speeds in either
     *  motor direction, giving very high accuracy
     **********************************************************************/
    TA1CTL   |= TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__2 |   // Clock source: SMCLK, Clock divider: 2
                TIMER_A_CTL_MC__UP;                             // Counting: Up Mode (Timer A1)
    TA2CTL   |= TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__2 |   // Clock source: SMCLK, Clock divider: 2
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
    TA1CCR0   = 40000;                                          // 20000 microsecond PWM period
    TA2CCR0   = 40000;
    TA1CCR1   = 3000;                                           // 1500 microsecond on-time, motor stopped
    TA1CCR2   = 3000;                                           // 1500 microsecond on-time, motor stopped
    TA1CCR3   = 3000;                                           // 1500 microsecond on-time, motor stopped
    TA1CCR4   = 3000;                                           // 1500 microsecond on-time, motor stopped
    TA2CCR1   = 3000;                                           // 1500 microsecond on-time, motor stopped
    TA2CCR2   = 3000;                                           // 1500 microsecond on-time, motor stopped
    TA2CCR3   = 3000;                                           // 1500 microsecond on-time, motor stopped
    TA2CCR4   = 3000;                                           // 1500 microsecond on-time, motor stopped
}


// Crude scheduler interrupt routine
extern void TA0_0_IRQHandler(void)
{
    // Assure that the right interrupt flag triggered (should only be one)
    if(!(TA0CCTL0 & TIMER_A_CCTLN_CCIFG))
    {
        // If flag isn't set, return
        return;
    }

    // Clear flag after checking
    TA0CCTL0 &= ~TIMER_A_CCTLN_CCIFG;

    // Declare static counter variables for interrupts which occur slower than 60 Hz
    static uint8_t motorCurrents = 0;
    static uint8_t powerStatus = 0;

    // Increment counters
    motorCurrents++;
    powerStatus++;

    /* REMOVED I2C FROM CURRENT DESIGN, UNCOMMENT TO ADD IT BACK

    // Every interrupt, schedule a pressure sensor reading
    if(!queuePush(eventList, DEPTH_SENSOR_READ_START))
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

    */

    // Check if it is time to read motor currents
    if((motorCurrents >= INT_COUNT_4HZ) && motorConversionDone)
    {
        // If this is a 4 Hz interrupt, schedule a motor currents reading
        if(!queuePush(eventList, MOTOR_CURRENT_READ_START))
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

        // Reset the motor currents counter
        motorCurrents = 0;

        // Indicate that a motor conversion has begun
        motorConversionDone = 0;
    }

    if((powerStatus >= INT_COUNT_1HZ) && powerConversionDone)
    {
        // If this is a 1 Hz interrupt, schedule a power check
        if(!queuePush(eventList, POWER_CURRENT_READ_START))
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

        // Reset the power check counter
        powerStatus = 0;

        // Indicate that a power conversion has begun
        powerConversionDone = 0;
    }
}
