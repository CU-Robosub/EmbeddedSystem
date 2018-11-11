#include "msp.h"
#include "adc.h"
#include "i2c.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"

uint8_t ComputerCommand = 0;

void uartCompConfigure()
{
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;                // Enables the Rx Interrupt
    NVIC_EnableIRQ(EUSCIA1_IRQn);                   // Enables interrupts on the NVIC (watching flags)

    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST;         // Put eUSCI in reset
    P1SEL0 |= (BIT2 | BIT3);                        // TX & Rx Primary mode
    P1SEL1 &= ~(BIT2 | BIT3);

    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK;   // use SMCLK
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_MODE0;        // uart mode
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_MODE1;
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_PEN;          // disable parity bit
    EUSCI_A0->CTLW0 &= ~EUSCI_B_CTLW0_MSB;          // LSB first
    EUSCI_A0->CTLW0 &= ~EUSCI_B_CTLW0_SEVENBIT;     // 8-bit data
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SPB;          // 1 stop bit

    // setting baud rate to 115200
    // set over sampling mode based on frequency of SMCLK
    EUSCI_A0->MCTLW |= EUSCI_A_MCTLW_OS16;
    EUSCI_A0->BRW   |= 0x13;                        // set baud
    EUSCI_A0->MCTLW |= 0xA0;                        // set BRF
    EUSCI_A0->MCTLW |= 0xB500;                      // set BRS

    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;        // Initialize eUSCI
}

void uartAtmelConfigure()
{
   EUSCI_A1->IE |= EUSCI_A_IE_RXIE;                // Enables the Rx Interrupt
   NVIC_EnableIRQ(EUSCIA1_IRQn);                   // Enables interrupts on the NVIC (watching flags)

   // P2.2 is Rx and P2.3 is Tx
   P2SEL0 |= (BIT2 | BIT3);
   P2SEL1 &= ~(BIT2 | BIT3);

   EUSCI_A1->CTLW0 |= EUSCI_A_CTLW0_SWRST;         // Enables reset
   EUSCI_A1->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK;   // Enables System clock
   EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_SYNC;         // Enables Asynchronous mode
   EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_SPB;          // One stop bit
   EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_SEVENBIT;     // 8-bit data length
   EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_MSB;          // LSB first
   EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_PEN;          // Disables parity

   // Makes the Baud Rate 9600
   EUSCI_A1->BRW |= 0x13;                          // Prescaler of Baud Rate
   EUSCI_A1->MCTLW |= EUSCI_A_MCTLW_OS16;          // Enables over sampling mode
   EUSCI_A1->MCTLW |= 0xAA80;                      // Determine the Mod pattern (Fx) and the Second Mod stage (Sx)

   EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;        // Disables reset
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
    for(i = 0; i < length; i++)
    {
        uartSendCompByte(data[i]);                  // send data byte by byte
    }
}


void uartSendAtmelByte(uint8_t data)
{
   while(!(EUSCI_A1->IFG & BIT1));                 // Checks the interrupt flag and Bit 1
   EUSCI_A1->TXBUF |= data;                        // Sets the transmitter to the data value
}

void uartSendAtmelN(uint8_t * data, uint32_t length)
{
   uint32_t i = 0;
   for(i = 0; i < length; i++)                         // Takes in an array in order to send a byte to the UART line
   {
       uartSendAtmelByte(data[i]);
   }
}

void EUSCIA1_IRQHandler(void)
{
  if (EUSCI_A1->IFG & BIT0)                        // Checks the specific received byte interrupt is high
  {
      uint8_t data = EUSCI_A1->RXBUF;              // Reads data and puts it into a local variable
      if(ComputerCommand == data)                  // Compares echo data to previous data
      {
          uartSendAtmelByte(0xFF);                 // Passes through Ones if variables are equal
      }
      else
      {
          uartSendAtmelByte(0x00);                 // Passes through Zeros if not
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
