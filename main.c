#include  <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

// Define UART0, GPIO pins, and buttons
#define SW1_PIN   (1 << 4)  // Assume SW1 is connected to PF4
#define SW2_PIN   (1 << 0)  // Assume SW2 is connected to PF0
#define LED_GREEN (1 << 3)  // Assume Green LED is on PF3
#define LED_BLUE  (1 << 2)  // Assume Blue LED is on PF2
#define LED_RED   (1 << 1)  // Assume Red LED is on PF1

// Initialize UART0 for 9600 baud with odd parity
void UART0_Init(void) {
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;     // Enable UART0 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;     // Enable GPIO Port A clock
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R0) == 0) {} // Wait for Port A to be ready

    // Configure PA0 as RX and PA1 as TX for UART0
    GPIO_PORTA_AFSEL_R |= (1 << 0) | (1 << 1);   // Enable alternate functions for PA0 (RX) and PA1 (TX)
    GPIO_PORTA_PCTL_R &= ~((0xF << 0) | (0xF << 4)); // Clear the PCTL values for PA0 and PA1
    GPIO_PORTA_PCTL_R |= (0x1 << 0) | (0x1 << 4);  // Set PCTL for PA0 as U0RX and PA1 as U0TX
    GPIO_PORTA_DEN_R |= (1 << 0) | (1 << 1);     // Enable digital I/O for PA0 and PA1

    UART0_CTL_R &= ~UART_CTL_UARTEN;              // Disable UART0 to configure it
    UART0_IBRD_R = 130;                        // Set integer part of baud rate (9600 baud)
    UART0_FBRD_R = 13;                           // Set fractional part of baud rate
    UART0_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN); // 8 data bits, odd parity, 1 stop bit
    UART0_CTL_R |= UART_CTL_UARTEN;              // Enable UART0
}

// Write a byte of data to UART0
void UART0_Write(uint8_t data) {
    while((UART0_FR_R & UART_FR_TXFF) != 0) {}   // Wait until TX FIFO is not full
    UART0_DR_R = data;                           // Send data
}

// Read a byte of data from UART0
uint8_t UART0_Read(void) {
    if ((UART0_FR_R & UART_FR_RXFE) == 0) {      // Check if RX FIFO is not empty
        return (uint8_t)(UART0_DR_R & 0xFF);     // Read received data
    }
        return 0;  // Return 0 if no data
}

// Initialize Port F for LEDs and switches
void PortF_Init(void) {
    SYSCTL_RCGC2_R |= 0x00000020; /* enable clock to GPIOF */
    GPIO_PORTF_LOCK_R = 0x4C4F434B; /* unlock commit register */
    GPIO_PORTF_CR_R = 0x1F; /* make PORTF0 configurable */
    GPIO_PORTF_DEN_R = 0x1F; // Enable digital functionality for PF0-PF4
    GPIO_PORTF_DIR_R = 0x0E; /* set PF3, PF2, PF1 as output (LEDs) */
    GPIO_PORTF_PUR_R = 0x11; // Enable pull-up resistors for PF0 and PF4 (switches)
}

// Main function
int main(void) {
    UART0_Init();    // Initialize UART0
    PortF_Init();    // Initialize Port F for switches and LEDs

    while(1) {
        // Continuously check for incoming data
        uint8_t receivedData = UART0_Read();

        // If valid data is received, handle it
        if (receivedData != 0) {
            UART0_Write(receivedData);  // Echo back received data to confirm reception
            if (receivedData == 'R') {
                GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~0x0E) | LED_RED;
            }
            else if (receivedData == 'B') {
                GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~0x0E) | LED_BLUE;
            }
            else if (receivedData == 'G') {
                GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~0x0E) | LED_GREEN;
            }
            else {
                GPIO_PORTF_DATA_R &= ~0x0E;   // Turn off all LEDs
            }
        }
    }
}



