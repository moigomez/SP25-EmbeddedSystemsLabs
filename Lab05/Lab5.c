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

#define FOSC 8000000UL      // Oscillator frequency (8MHz)
#define BAUD 9600
#define MYUBRR FOSC / 8 / BAUD - 1

//#define INPUT_PIN PD5       // Define test pin

void USART_Init(unsigned int ubrr) {
    // Set baud rate
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    // Enable receiver and transmitter
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    // Set frame format: 8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}
void USART_Transmit(unsigned char data) {
    // Wait for the transmit buffer to be empty
    while (!(UCSR0A & (1 << UDRE0))) {
        // Do nothing (wait)
    }

    // Put data into buffer, sends the data
    UDR0 = data;
}
void USART_SendString(const char *str) {
    while (*str) {
        USART_Transmit(*str++);
    }
}

void ADC_init() {
    // Set voltage reference
    ADMUX =  (1 << REFS1) | (1 << REFS0);


}
uint16_t ADC_read(uint8_t channel) {
    ADCSRA |= (1 << ADSC);

    while (ADCSRA & (1 << ADSC));

    return ADC
}


int main(void) {
    USART_Init(MYUBRR);

    /*
    // Set PD2 as input
    DDRD &= ~(1 << INPUT_PIN);

    // Optional: Enable pull-up resistor
    PORTD |= (1 << INPUT_PIN);
    */

    while (1) {
        /*
        // Read the pin state
        if (PIND & (1 << INPUT_PIN)) {
            USART_SendString("Pin is HIGH\r\n");
        } else {
            USART_SendString("Pin is LOW\r\n");
        }
        */

        uint16_t adc_value = ADC_read(0);

        _delay_ms(500);
    }
    
}
