#include "timer.h"


// Declare global variables
uint8_t intCounter = 0;

void TA0_0_IRQHandler()
{
    // Every 10th interrupt (10 seconds)
    if(intCounter >= 10)
    {
        startReadPowerStatus();        // Begin a conversion for the power measurements
    }
    else
    {
        intCounter++;
    }
    TA0CCR0 &= ~TIMER_A_CCTLN_CCIFG;   // Clear interrupt flag
}

void timerConfigure()
{
    TA0CTL   |= TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__8 |    // Clock source: SMCLK, Clock divider: 8
                TIMER_A_CTL_MC__UP;                              // Counting: Up Mode
    TA0CCTL0 |= TIMER_A_CCTLN_CCIE;                              // Enable CCTL0 interrupt
    TA0CCTL0 &= ~TIMER_A_CCTLN_CCIFG;                            // Clear CCTL0 interrupt flag
    TA0EX0   |= TIMER_A_EX0_IDEX__8;                             // Input divider expansion: 8
    TA0CCR0   = 46875;                                           // One second delay per interrupt
    NVIC_EnableIRQ(TA0_0_IRQn);                                  // Enable interrupts on the NVIC
}
