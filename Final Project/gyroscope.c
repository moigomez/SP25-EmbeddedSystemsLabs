/*
 *
 * gyroscope.c
 *
 * Created: 5/1/2025
 * Author:  Moises Gomez
 *
 * C-program for analyzing accelerometer and gyroscope data
 * from MPU6050
 *
 */


#define F_CPU 16000000UL                // Clock frequency (16MHz)
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>

#include "i2c.h"
#include "mpu6050.h"

// USART related
#define FOSC 8000000UL                  // Oscillator frequency (8MHz)
#define BAUD 9600
#define MYUBRR FOSC / 8 / BAUD - 1      // Calculate UBRR0 register value

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

int main(void) {
    // Initialize I2C and MPU6050
    i2c_init();
    mpu6050_init();

    // Buffers to store raw sensor data
    int16_t accel_buff[3];
    int16_t gyro_buff[3];
    char buffer[64];

    while (1) {
        mpu6050_read_accel_ALL(accel_buff);
        mpu6050_read_gyro_ALL(gyro_buff);

        // Format and send over UART
        sprintf(buffer, "ACC: X=%d Y=%d Z=%d\r\n", accel_buff[0], accel_buff[1],
                accel_buff[2]);
        USART_SendString(buffer);

        sprintf(buffer, "GYR: X=%d Y=%d Z=%d\r\n", gyro_buff[0], gyro_buff[1],
                gyro_buff[2]);
        USART_SendString(buffer);

        USART_SendString("----------------------\r\n");
        _delay_ms(500);
    }
}