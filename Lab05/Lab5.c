/*
 *
 * Lab5.c
 *
 * Created: 4/14/2025
 * Author:  Moises Gomez
 *
 * C-program to remotely controlled logging system
 * that returns analog/digital signal data in the terminal
 *
 */


#define F_CPU 16000000UL    // Clock frequency (16MHz)
#include <avr/io.h>
#include <util/delay.h>

#define FOSC 8000000UL    // Oscillator frequency (8MHz)
#define BAUD 9600
#define MYUBRR FOSC / 8 / BAUD - 1

void USART_Init(unsigned int ubrr) {
    // Set baud rate
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    // Enable receiver and transmitter
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    // Set frame format: 8data, 2stop bit
    UCSR0C = (1 << USBS0) | (3 << UCSZ00);
}
void USART_Transmit(unsigned char data) {
    // Wait for the transmit buffer to be empty
    while (!(UCSR0A & (1 << UDRE0))) {
        // Do nothing (wait)
    }

    // Put data into buffer, sends the data
    UDR0 = data;
}
unsigned char USART_Receive(void) {
    // Wait for data to be received
    while (!(UCSR0A & (1 << RXC0))) {
        // Do nothing (wait)
    }

    // Get and return received data from the buffer
    return UDR0;
}

int main(void) {
    USART_Init(MYUBRR);

    while (1) {
        USART_Transmit('H');
        USART_Transmit('E');
        USART_Transmit('L');
        USART_Transmit('L');
        USART_Transmit('O');

        _delay_ms(1000);
    }
    
}
