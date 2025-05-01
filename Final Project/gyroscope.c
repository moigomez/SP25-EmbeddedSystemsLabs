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

// USART Definitions
#define FOSC 8000000UL
#define BAUD 9600
#define MYUBRR FOSC / 8 / BAUD - 1

// === USART Functions ===
void USART_Init(unsigned int ubrr) {
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void USART_Transmit(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void USART_SendString(const char *str) {
    while (*str) {
        USART_Transmit(*str++);
    }
}

// === TWI (I2C) Functions ===
void TWI_Init(void) {
    TWSR = 0x00;
    TWBR = 0x48;  // ~100kHz @ 16MHz
    TWCR = (1 << TWEN);
}

void TWI_Start(void) {
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
}

void TWI_Stop(void) { TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT); }

void TWI_Write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
}

uint8_t TWI_ReadACK(void) {
    TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t TWI_ReadNACK(void) {
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

// === MPU6050 Functions ===
#define MPU6050_ADDR 0x68

void MPU6050_Init(void) {
    TWI_Start();
    TWI_Write(MPU6050_ADDR << 1);  // Write mode
    TWI_Write(0x6B);               // PWR_MGMT_1 register
    TWI_Write(0x00);               // Set to zero (wakes up the MPU-6050)
    TWI_Stop();
}

void MPU6050_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az) {
    TWI_Start();
    TWI_Write(MPU6050_ADDR << 1);
    TWI_Write(0x3B);  // Starting register for accelerometer
    TWI_Start();
    TWI_Write((MPU6050_ADDR << 1) | 1);  // Read mode

    *ax = ((int16_t)TWI_ReadACK() << 8) | TWI_ReadACK();
    *ay = ((int16_t)TWI_ReadACK() << 8) | TWI_ReadACK();
    *az = ((int16_t)TWI_ReadACK() << 8) | TWI_ReadNACK();

    TWI_Stop();
}

// === Main ===
int main(void) {
    USART_Init(MYUBRR);
    TWI_Init();
    MPU6050_Init();

    char buffer[64];
    int16_t ax, ay, az;

    while (1) {
        MPU6050_ReadAccel(&ax, &ay, &az);
        snprintf(buffer, sizeof(buffer), "AX:%d AY:%d AZ:%d\r\n", ax, ay, az);
        USART_SendString(buffer);
        _delay_ms(500);
    }
}