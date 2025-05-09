/*
 *
 * Lab5.c
 *
 * Created: 4/14/2025
 * Author:  Moises Gomez
 *
 * C-program that implements a remotely controlled logging
 * system that uses USART and I2C communication to return
 * analog/digital signal data in the terminal
 *
 */


#define __DELAY_BACKWARD_COMPATIBLE__   // Allows variables to be used as arguments in delays
#define F_CPU 16000000UL                // Clock frequency (16MHz)
#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// USART related
#define FOSC 8000000UL                  // Oscillator frequency (8MHz)
#define BAUD 9600
#define MYUBRR FOSC / 8 / BAUD - 1      // Calculate UBRR0 register value

#define MAX_COMMAND_LEN 50

// DAC related
#define SCL_CLOCK 100000L
#define MAX518_ADDR 0x2C                // 7-bit address
#define MAX518_ADDR_WRITE (MAX518_ADDR << 1)

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

//  *** I2C Initialization for DAC ***
void TWI_Init(void) {
    // Set bit rate register (TWBR) for 100kHz with no prescaler
    TWSR = 0x00;
    TWBR = ((F_CPU / SCL_CLOCK) - 16) / 2;
}

void TWI_Start(void) {
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
}

void TWI_Stop(void) {
    TWCR = (1 << TWSTO) | (1 << TWINT) | (1 << TWEN);
    while (TWCR & (1 << TWSTO)); // Wait for stop to finish
}

void TWI_Write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

uint8_t MAX518_SetVoltage(uint8_t channel, float voltage) {
    if (voltage < 0.0) voltage = 0.0;
    if (voltage > 5.0) voltage = 5.0;

    uint8_t value = (uint8_t)((voltage / 5.0) * 255.0); // 0-255 scale

    TWI_Start();
    TWI_Write(MAX518_ADDR_WRITE);           // Address + Write
    TWI_Write(channel == 0 ? 0x00 : 0x01);  // Channel 0 or 1
    TWI_Write(value);                       // Voltage value
    TWI_Stop();
    
    return value;
}

void receive_command(char *command, uint8_t max_len) {
    uint8_t index = 0;
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

    char command[MAX_COMMAND_LEN];

    while (1) {
        receive_command(command, MAX_COMMAND_LEN);
        char *token = strtok(command, ",");         // Split command by delimiter ',';

        if ((token[0] == 'G' || token[0] == 'M' || token[0] == 'S') && token != NULL) {
            if (token[0] == 'G') {
                char voltage_string[20];
                char voltage_expression[20];

                uint16_t adc_value = ADC_read(0);   // Read ADC value
                float voltage_value = (adc_value * 5) / 1024.0; // Convert ADC value to voltage

                dtostrf(voltage_value, 6, 3, voltage_string);   // Convert voltage float number to string
                snprintf(voltage_expression, sizeof(voltage_expression), "v=%sV\r\n", voltage_string);  // Format voltage string

                // Return voltage
                USART_SendString(voltage_expression);
            }
            if (token[0] == 'M') {
                char *n_arg = strtok(NULL, ",");    // Extract first comma-separated value
                char *dt_arg = strtok(NULL, ",");   // Extract second comma-separated value
                // Convert arguments to integers
                int n = atoi(n_arg);
                int dt = atoi(dt_arg);

                // Return voltages if within bounds and format is valid
                if ((n >= 2 && n <= 20) && (dt >= 1 && dt <= 10)) {
                    for (int i = 1; i <= n; i++) {
                        char voltage_string[20];
                        char voltage_expression[20];
                        
                        uint16_t adc_value = ADC_read(0);   // Read ADC value
                        float voltage_value = (adc_value * 5) / 1024.0; // Convert ADC value to voltage
                        
                        dtostrf(voltage_value, 6, 3, voltage_string);   // Convert voltage float number to string
                        snprintf(voltage_expression, sizeof(voltage_expression), "v=%sV\r\n", voltage_string);  // Format voltage string
                        
                        // Return voltage
                        USART_SendString(voltage_expression);

                        int dt_ms = dt * 1000;  // Convert delay to milliseconds
                        _delay_ms(dt_ms);
                    }
                    } else {
                    USART_SendString("Invalid format!\n");
                    USART_SendString("Usage: M,<number of measurements>,<time delay>\n");
                    USART_SendString("  - <number of measurements>: integer (2-20)\n");
                    USART_SendString("  - <time delay>:             integer (1-10, seconds)\n");
                }
            }
            if (token[0] == 'S') {
                char *c_arg = strtok(NULL, ",");   // Extract first comma-separated value
                char *v_arg = strtok(NULL, ",");   // Extract second comma-separated value
                // Convert arguments to integers
                int c = atoi(c_arg);
                float v = atof(v_arg);

                char voltage_string[20];
                char channel_expression[70];
                dtostrf(v, 6, 2, voltage_string);  // Convert voltage argument to string

                if ((c == 0 || c == 1) && (v || v == 0)) {
                    uint8_t value =  MAX518_SetVoltage(c, v);   // Set voltage in channel and return dc value
                    snprintf(channel_expression, sizeof(channel_expression), "DAC channel %s set to %sV (%id)\r\n", c_arg, voltage_string, value);

                    // Return updated channel
                    USART_SendString(channel_expression);
                } else {
                    USART_SendString("Invalid format!\n");
                    USART_SendString("Usage: S,<DAC channel number>,<output voltage>\n");
                    USART_SendString("  - <DAC channel number>: integer (0 or 1)\n");
                    USART_SendString("  - <output voltage>:     float\n");
                }

            }
        } else {
            USART_SendString("\nInvalid command!\n\n");
        }
    }
}
