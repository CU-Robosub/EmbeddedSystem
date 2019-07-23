#include "uart.h"


// Declare global variables
extern volatile queue_t* eventList;
extern volatile queue_t* pneumaticsReceive;
extern volatile queue_t* motorReceive;
extern volatile queue_t* transmit;
extern volatile uint8_t pneumaticsState;
extern volatile uint8_t currentActuator;

void uartConfigurePnumatics(void)
{
    // Disable eUSCI to configure UART
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
    EUSCI_A0->BRW   |= 0x06;                        // Prescaler of Baud (UCBRx)
    EUSCI_A0->MCTLW |= 0xAA80;                      // Determine the Second Mod stage (UCBRSx) and the Mod pattern (UCBRFx)

    // Disable eUSCI reset to enable the line
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;

    // Enable the RX interrupt for eUSCI_A0
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;                // Enables the Rx Interrupt
    NVIC_EnableIRQ(EUSCIA0_IRQn);                   // Enables interrupts on the NVIC (watching flags)

}

void uartCompConfigure(void)
{
    // Disable eUSCI to configure UART
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
    EUSCI_A0->BRW   |= 0x06;                        // Prescaler of Baud (UCBRx)
    EUSCI_A0->MCTLW |= 0xAA80;                      // Determine the Second Mod stage (UCBRSx) and the Mod pattern (UCBRFx)

    // Enable the RX interrupt for eUSCI_A0
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;                // Enables the Rx Interrupt
    NVIC_EnableIRQ(EUSCIA0_IRQn);                   // Enables interrupts on the NVIC (watching flags)

    // Disable eUSCI reset to enable the line
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;

}


// transmits a byte of data to serial terminal
void uartBeginCompTransmit(void)
{
    // Load first byte of transmit queue into the transmit buffer
    EUSCI_A0->TXBUF = queuePop(transmit);

    // Enable transmit interrupt
    EUSCI_A0->IE |= EUSCI_A_IE_TXIE;
}


void uartPneumaticsConfigure(void)
{
    // Disable eUSCI to configure UART
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
    EUSCI_A1->BRW   |= 0x48;                        // Prescaler of Baud Rate (UCBRx)
    EUSCI_A1->MCTLW |= 0x0820;                      // Determine the Second Mod stage (UCBRSx) and the Mod pattern (UCBRFx)

    // Enable the RX interrupt for eUSCI_A0
    EUSCI_A1->IE |= EUSCI_A_IE_RXIE;                // Enables the Rx Interrupt
    NVIC_EnableIRQ(EUSCIA1_IRQn);                   // Enables interrupts on the NVIC (watching flags)

    // Disable eUSCI reset to enable the line
    EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;        // Disables reset
}


void uartSendPneumaticsByte(uint8_t data)
{
    while(!(EUSCI_A1->IFG & BIT1));                 // Checks the interrupt flag and Bit 1
    EUSCI_A1->TXBUF = data;                         // Sets the transmitter to the data value
}


void uartSendPneumaticsN(uint8_t * data, uint32_t length)
{
    uint32_t i;
    for(i = 0; i < length; i++)                         // Takes in an array in order to send a byte to the UART line
    {
        uartSendPneumaticsByte(data[i]);
    }
}

typedef enum {
    KISS_STATE_NORMAL,
    KISS_STATE_ESCAPE,
} KISS_STATE;


KISS_STATE uart_kiss_state;

extern volatile uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
extern volatile uint8_t uart_rx_buffer_pos;

void pnumatics_uart_irq(void)
{

    if(EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG)
    {

        uint8_t received = EUSCI_A0->RXBUF;         // Reading here also clears receive interrupt flag

        switch(uart_kiss_state)
        {
            case KISS_STATE_NORMAL:
                switch(received)
                {
                    case START_STOP_FRAME:
                        if(uart_rx_buffer_pos > 0){
                            queuePush(eventList, PNEUMATICS_COMMAND_RECEIVED);
                            EUSCI_A0->IE &= ~(EUSCI_A_IE_RXIE);
                        }
                        break;
                    case ESCAPE_FRAME:
                        uart_kiss_state = KISS_STATE_ESCAPE;
                        break;
                    default:
                        uart_rx_buffer[uart_rx_buffer_pos++] = received;
                        break;
                }
                break;
            case KISS_STATE_ESCAPE:
                switch(received)
                {
                    case C0_FRAME:
                        uart_rx_buffer[uart_rx_buffer_pos++] = 0xC0;
                        break;
                    case DB_FRAME:
                        uart_rx_buffer[uart_rx_buffer_pos++] = 0xDB;
                        break;
                }
                uart_kiss_state = KISS_STATE_NORMAL;
                break;
        }

        if(uart_rx_buffer_pos >= UART_RX_BUFFER_SIZE){
            uart_rx_buffer_pos--;
        }

    }

}

void embedded_uart_irq(void){
    // INTERRUPTS ARE CLEARED AT THE END TO AVOID CALLING ANOTHER INTERRUPT AFTER CLEARNING A FLAG
    // This will end up storing all data and the end frames in the corresponding queue (motor or pneumatics)
    //  NOTE: This does not queue the start frame or the instruction frame because there are function specific queues

    // If the interrupt was caused by a receive
    if(EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG)
    {
        // Make a static variable to keep track of receive state machine, and one for reading
        static uint8_t compReceiveState = READY;
        uint8_t received = EUSCI_A0->RXBUF;         // Reading here also clears receive interrupt flag

        // If there is no current receiving going on and a start frame was received
        if((compReceiveState == READY) && (received == START_STOP_FRAME))
        {
            // Move to the active receiving state
            compReceiveState = RECEIVING;
        }

        // If receiving has begun, check for a motor or pneumatics command
        else if(compReceiveState == RECEIVING)
        {
            // If a motor command was received
            if(received == MOTOR_CHANGE_FRAME)
            {
                // Move to the motor receiving state
                compReceiveState = MOTOR_RECEIVING;
            }

            // If a pneumatics command was received
            else if(received == PNEUMATICS_FIRE_FRAME)
            {
                // Move to the pneumatics receiving state
                compReceiveState = PNEUMATICS_RECEIVING;
            }
        }

        // If motor receiving has begun, record data until an end frame is found
        else if(compReceiveState == MOTOR_RECEIVING)
        {
            // Queue newly received data, including the end frame
            if(!queuePush(motorReceive, received))
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
                queuePush(transmit, MOTOR_RECEIVE_QUEUE_FULL_ERROR);
                queuePush(transmit, START_STOP_FRAME);

                // Begin transmission and enter infinite while loop
                uartBeginCompTransmit();
                while(1);
            }

            // If an end frame was received
            if(received == START_STOP_FRAME)
            {
                // Reset the receive state to be ready for the next receive
                compReceiveState = READY;

                // Queue up a motor instruction decoding
                if(!queuePush(eventList, MOTOR_COMMAND_RECEIVED))
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
            }
        }

        // If pneumatics receiving has begun, record data until an end frame is found
        else if(compReceiveState == PNEUMATICS_RECEIVING)
        {
            // Queue newly received data, including the end frame
            if(!queuePush(pneumaticsReceive, received))
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
                queuePush(transmit, PNEUMATICS_RECEIVE_QUEUE_FULL_ERROR);
                queuePush(transmit, START_STOP_FRAME);

                // Begin transmission and enter infinite while loop
                uartBeginCompTransmit();
                while(1);
            }

            // If an end frame was received
            if(received == START_STOP_FRAME)
            {
                // Reset the received state to be ready for the next received
                compReceiveState = READY;

                // Queue up a pneumatics instruction decoding
                if(!queuePush(eventList, PNEUMATICS_COMMAND_RECEIVED))
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
            }
        }

        // If not currently in a receive state, enter while loop for debugging purposes
        else
        {
            while(1);
        }
    }

    // If the interrupt was caused by a transmit
    if(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG)
    {
        // Check if the current transmission is complete
        if(queueEmpty(transmit))
        {
            // If the transmission is complete, disable the transmit interrupt
            EUSCI_A0->IE &= ~EUSCI_A_IE_TXIE;
        }
        else
        {
            // If the transmission is not complete, load the next byte into the transmit buffer
            EUSCI_A0->TXBUF = queuePop(transmit);
        }
    }

    // Clear interrupt flags
    EUSCI_A0->IFG &= ~(EUSCI_A_IFG_RXIFG | EUSCI_A_IFG_TXIFG);

}

extern void EUSCIA0_IRQHandler(void)
{
    #ifdef EMBEDDED_SYSTEM
    embedded_uart_irq();
    #endif
    #ifdef PNUMATICS_SYSTEM
    pnumatics_uart_irq();
    #endif

}


extern void EUSCIA1_IRQHandler(void)
{
    // Check if a receiving interrupt flag is set
    if (EUSCI_A1->IFG & EUSCI_A_IFG_RXIFG);
    {
        // Check if we are expecting to hear from the pneumatics board
        if(pneumaticsState == COMMAND_SENT)
        {
            // If we are expecting communication, check if the confirmation matches what is expected
            if(EUSCI_A1->RXBUF == currentActuator)
            {
                // Prepare a confirmation correct transmission to fire
                uint8_t fire[3];
                fire[0] = START_STOP_FRAME;
                fire[1] = CONFIRMATION_CORRECT;
                fire[2] = START_STOP_FRAME;

                // Update state to indicate that firing has occurred
                pneumaticsState = PNEUMATICS_READY;

                // Transmit the fire command
                uartSendPneumaticsN(fire, 3);
            }
            else
            {
                // If it is not right, send an error and resend the actuator number
                uint8_t fire[4];
                fire[0] = START_STOP_FRAME;
                fire[1] = CONFIRMATION_INCORRECT;
                fire[2] = currentActuator;
                fire[3] = START_STOP_FRAME;

                // Transmit the fire command
                uartSendPneumaticsN(fire, 4);
            }
        }
    }

    // Clear the receive interrupt flag (only one which is enabled for pneumatics)
    EUSCI_A1->IFG &= ~EUSCI_A_IFG_RXIFG;
}
