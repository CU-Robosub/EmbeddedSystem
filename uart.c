#include "uart.h"

uint8_t ComputerCommand = 0;

void uartCompConfigure()
{
    // Put eUSCI in reset to configure UART
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST;

    // Configure UART
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SSEL_MASK;    // Clear clock select bits
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK;   // Use SMCLK as source clock
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SYNC;         // Enables asynchronous mode
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_MODE0;        // Select UART as asynchronous operation
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_MODE1;
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_PEN;          // Disable parity
    EUSCI_A0->CTLW0 &= ~EUSCI_B_CTLW0_MSB;          // LSB first
    EUSCI_A0->CTLW0 &= ~EUSCI_B_CTLW0_SEVENBIT;     // 8-bit data
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SPB;          // 1 stop bit

    // Makes the Baud Rate 115200
    // Information on setting Baud Rate can be found on page 735 of the Technical Reference Manual
    EUSCI_A0->MCTLW |= EUSCI_A_MCTLW_OS16;          // Enables over sampling mode
    EUSCI_A0->BRW   |= 0x13;                        // Prescaler of Baud
    EUSCI_A0->MCTLW |= 0xB5A0;                      // Determine the Mod pattern (Fx) and the Second Mod stage (Sx)

    // Enable the RX interrupt for eUSCI_A0
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;                // Enables the Rx Interrupt
    NVIC_EnableIRQ(EUSCIA0_IRQn);                   // Enables interrupts on the NVIC (watching flags)

    // Disable eUSCI reset to enable the line
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;
}

// transmits a byte of data to serial terminal
void uartSendCompByte(uint8_t data)
{
    while(!(EUSCI_A0->IFG & BIT1));                 // block until transmitter is ready
    EUSCI_A0->TXBUF = data;                         // load data into buffer
}

// transmits a string of data to serial terminal
void uartSendCompN(uint8_t * data, uint32_t length)
{
    uint32_t i = 0;
    for(i=0; i < length; i++)
    {
        uartSendCompByte(data[i]);                  // send data byte by byte
    }
}

void uartPneumaticsConfigure()
{
    // Put eUSCI in reset to configure UART
    EUSCI_A1->CTLW0 |= EUSCI_A_CTLW0_SWRST;

    // Configure UART
    EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_SSEL_MASK;    // Clear clock select bits
    EUSCI_A1->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK;   // Use SMCLK as source clock
    EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_SYNC;         // Enables Asynchronous (UART) mode
    EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_MODE0;        // Select UART as asynchronous operation
    EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_MODE1;
    EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_PEN;          // Disables parity
    EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_MSB;          // LSB first
    EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_SEVENBIT;     // 8-bit data length
    EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_SPB;          // 1 stop bit

    // Makes the Baud Rate 9600
    // Information on setting Baud Rate can be found on page 735 of the Technical Reference Manual
    EUSCI_A1->MCTLW |= EUSCI_A_MCTLW_OS16;          // Enables over sampling mode
    EUSCI_A1->BRW   |= 0x13;                        // Prescaler of Baud Rate
    EUSCI_A1->MCTLW |= 0xAA80;                      // Determine the Mod pattern (Fx) and the Second Mod stage (Sx)

    // Enable the RX interrupt for eUSCI_A0
    EUSCI_A1->IE |= EUSCI_A_IE_RXIE;                // Enables the Rx Interrupt
    NVIC_EnableIRQ(EUSCIA1_IRQn);                   // Enables interrupts on the NVIC (watching flags)

    // Disable eUSCI reset to enable the line
    EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;        // Disables reset
}

void uartSendPneumaticsByte(uint8_t data)
{
    while(!(EUSCI_A1->IFG & BIT1));                 // Checks the interrupt flag and Bit 1
    EUSCI_A1->TXBUF |= data;                        // Sets the transmitter to the data value
}

void uartSendPneumaticsN(uint8_t * data, uint32_t length)
{
    uint32_t i;
    for(i = 0; i < length; i++)                         // Takes in an array in order to send a byte to the UART line
    {
        uartSendPneumaticsByte(data[i]);
    }
}

void EUSCIA1_IRQHandler(void)
{

    if (EUSCI_A1->IFG & BIT0)                        // Checks the specific received byte interrupt is high
    {
        uint8_t data = EUSCI_A1->RXBUF;              // Reads data and puts it into a local variable
        if(ComputerCommand == data)                  // Compares echo data to previous data
        {
            uartSendPneumaticsByte(0xFF);                 // Passes through Ones if variables are equal
        }
        else
        {
            uartSendPneumaticsByte(0x00);                 // Passes through Zeros if not
        }
    }
    EUSCI_A1->IFG &= ~BIT0;                          // Clears the flag and Bit 0
}

void EUSCIA0_IRQHandler(void)
{
    if (EUSCI_A0->IFG & BIT0)
    {
        uint8_t data = EUSCI_A0->RXBUF;
        ComputerCommand = data;
        uartSendAtmelByte(data);
        EUSCI_A0->IFG &= ~(BIT0);
    }
}
