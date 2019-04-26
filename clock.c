#include "clock.h"

void clockConfigure()
{
    // Set the core voltage higher to support higher clock frequency
    PCM->CTL0 |= PCM_CTL0_AMR__AM_LDO_VCORE1;

    // Increase Flash wait states to account for higher frequencies
    FLCTL->BANK0_RDCTL |= FLCTL_BANK0_RDCTL_WAIT2;
    FLCTL->BANK1_RDCTL |= FLCTL_BANK1_RDCTL_WAIT2;

    // Enter the clock key to allow editing clock registers
    CS->KEY = CS_KEY_VAL;

    // Configure the clocks used in the design
    CS->CTL0   = 0;                     // Clear main clock control register
    CS->CTL0  |= CS_CTL0_DCORSEL_5;     // Select DCO frequency 5 (48 MHz)
    CS->CTL1  |= CS_CTL1_SELM__DCOCLK;  // Set the source clock for MCLK to the DCOCLK
    CS->CTL1  |= CS_CTL1_SELS__DCOCLK;  // Set the source clock for SMCLK and HSMCLK to the DCOCLK
    CS->CTL1  |= CS_CTL1_DIVHS__1;      // Set the HSMCLK divider to 1
    CS->CTL1  |= CS_CTL1_DIVS__4;       // Set the SMCLK divider to 4
    CS->CLKEN |= CS_CLKEN_HSMCLK_EN;    // Enable HSMCLK
    CS->CLKEN |= CS_CLKEN_SMCLK_EN;     // Enable SMCLK

    // Clear the clock key to block editing clock registers
    CS->KEY = 0;
}
