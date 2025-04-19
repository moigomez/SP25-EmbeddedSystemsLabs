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

#define MAX_COMMAND_LEN 50

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
unsigned char USART_Receive(void) {
     // Wait for data
    while (!(UCSR0A & (1 << RXC0)));
    return UDR0;
}
void USART_SendString(const char *str) {
    while (*str) {
        USART_Transmit(*str++);
    }
}

//  *** ADC Initialization ***
void ADC_init() {
    // Set voltage reference
    ADMUX = (1 << REFS0);

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

void receive_command(char *command, u_int8_t max_len) {
    u_int8_t index = 0;
    char c;
    while(index < max_len - 1) {
        c = USART_Receive();
        // Ignore any returns in the command
        if (c == '\n') {
            break;
        }
        // Store incoming character
        command[index++] = c;
    }
    command[index] = '\0';  // Terminate string
}

int main(void) {
    USART_Init(MYUBRR);
    ADC_init();

    char command[MAX_COMMAND_LEN]

    while (1) {
        receive_command(command, MAX_COMMAND_LEN);
        char *token = strtok(command, ",")  // Split command by delimiter ',';

        if (token[0] == 'G' && token[0] == 'M' && token[0] == 'S' && token != NULL) {
            if (token[0] == 'G') {
                char voltage_string[20];
                char voltage_expression[20];

                uint16_t adc_value = ADC_read(0);   // Read ADC value
                float voltage_value = (adc_value * 5) / 1024.0;   // Convert ADC value to voltage

                dtostrf(voltage_value, 6, 3, voltage_string);   // Convert voltage float number to string
                snprintf(voltage_expression, sizeof(voltage_expression), "v=%sV\r\n", voltage_string);  // Format voltage string

                // Return voltage
                USART_SendString(voltage_expression);
            }
            if (token[0] == 'M') {
                USART_SendString("M Detected\n");
            }

        /*
        else if command.firstChar() == 'M' && command.length() > 1 {
            int n = command.secondChar();
            int dt = command.lastChar();

            if (n >= 2 && n <= 20) && (dt >= 1 && dt <= 10)  {
                for i = 0; i <= n; i++ {
                    // read ACD
                    // convert to voltage
                    // transmit string

                    delay_ms(dt * 1000);
                }
            } else {
                // error
            }
        }
        else if command.firstChar() == 'S' && command.length() > 1  {
            int c = command.secondChar();
            string v = command.lastChar();

            float voltage = 0;
            atoi(voltage, v); 

            if (c == 0 || c == 1) && voltage {
                // DAC_channel = c
                // DAC_voltage = voltage

                // set ADC5 as output and return configured D/A signal 
            }
        }
        */
        } else {
            USART_SendString("Invalid Command\n");
        }
    }
}
