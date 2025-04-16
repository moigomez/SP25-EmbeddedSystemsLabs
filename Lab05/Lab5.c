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


#define F_CPU 16000000UL            // Clock frequency (16MHz)
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FOSC 8000000UL              // Oscillator frequency (8MHz)
#define BAUD 9600
#define MYUBRR FOSC / 8 / BAUD - 1  // Calculate UBRR0 register value

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
    while (!(UCSR0A & (1 << UDRE0)));

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

    // Enable ADC and set prescaler of 128
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}
uint16_t ADC_read(uint8_t channel) {
    channel &= 0b00000111;
    ADMUX = (ADMUX & 0xF8) | channel;

    // Start single conversion
    ADCSRA |= (1 << ADSC);

    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC));

    return ADC;
}


int main(void) {
    USART_Init(MYUBRR);
    ADC_init();

    while (1) {
        char voltage_string[20];
        char voltage_expression[20];

        uint16_t adc_value = ADC_read(0);   // Read ADC value
        float voltage_value = (adc_value * 1.1) / 1024.0;   // Convert ADC value to voltage

        dtostrf(voltage_value, 6, 3, voltage_string);   // Convert voltage float number to string
        snprintf(voltage_expression, sizeof(voltage_expression), "v=%sV\r\n", voltage_string);  // Format voltage string

        // Return voltage
        USART_SendString(voltage_expression);

        _delay_ms(500);
    }
    
}
