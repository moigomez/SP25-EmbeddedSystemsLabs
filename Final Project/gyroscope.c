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

//  *** TWI Initialization ***
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

//  *** MPU6050 Initialization ***
#define MPU6050_ADDR 0x68

void MPU6050_Init(void) {
    TWI_Start();
    TWI_Write(MPU6050_ADDR << 1);  // Write mode
    TWI_Write(0x6B);               // PWR_MGMT_1 register
    TWI_Write(0x00);               // Set to zero (wakes up the MPU-6050)
    TWI_Stop();
}

// Global (previous) acceleration values
int16_t prev_ax = 0, prev_ay = 0, prev_az = 0;

// Read accelerometer data
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
// Read gyroscope data
void MPU6050_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz) {
    TWI_Start();
    TWI_Write(MPU6050_ADDR << 1);
    TWI_Write(0x43);  // Starting register for accelerometer
    TWI_Start();
    TWI_Write((MPU6050_ADDR << 1) | 1);  // Read mode

    *gx = ((int16_t)TWI_ReadACK() << 8) | TWI_ReadACK();
    *gy = ((int16_t)TWI_ReadACK() << 8) | TWI_ReadACK();
    *gz = ((int16_t)TWI_ReadACK() << 8) | TWI_ReadNACK();

    TWI_Stop();
}

volatile uint32_t millis_counter = 0;

void Timer1_Init(void) {
    // CTC mode
    TCCR1B |= (1 << WGM12);
    // Prescaler 64
    TCCR1B |= (1 << CS11) | (1 << CS10);
    // 1 ms at 16 MHz: (16e6 / (64 * 1000)) - 1 = 249
    OCR1A = 249;
    // Enable compare match interrupt
    TIMSK1 |= (1 << OCIE1A);
    // Enable global interrupts
    sei();
}
ISR(TIMER1_COMPA_vect) { 
    millis_counter++; 
}
uint32_t millis(void) {
    uint32_t ms;
    cli();
    ms = millis_counter;
    sei();
    return ms;
}

int main(void) {
    USART_Init(MYUBRR);
    TWI_Init();
    MPU6050_Init();
    Timer1_Init();

    char buffer[128];
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    static uint8_t header_printed = 0;

    uint32_t last_time = 0;
    static int16_t prev_az = 0;
    static uint32_t prev_time = 0;

    int count = 0;

    while (1) {
        uint32_t current_time = millis();
        float dt = current_time - prev_time;  // in milliseconds
        char derv_string[64];
        float daz_dt = 0;

        if ((current_time - last_time >= 1000)) {
            count = 0;
            last_time = current_time;
        }

        MPU6050_ReadAccel(&ax, &ay, &az);
        MPU6050_ReadGyro(&gx, &gy, &gz);

        if (dt > 0) {
            daz_dt = compute_derivative(az, prev_az, dt);  // jerk in LSB/ms
        }

        prev_az = az;
        prev_time = current_time;
        float mag = sqrt((double)ax * ax + (double)ay * ay + (double)az * az);

        if ((mag >= 36000.0) && (mag < 50000.0) || (daz_dt >= 250.0) && (daz_dt < 400.0)) {
            count++;
        }

        // Format accelerometer and gyroscope data
        if (!header_printed) {
            USART_SendString("AX\tAY\tAZ\tGX\tGY\tGZ\r\n");
            header_printed = 1;
        }
        snprintf(buffer, sizeof(buffer),"%d\t %d\t %d\t %d\t %d\t %d\t %i\r\n", ax, ay, az, gx, gy, gz, count);
        USART_SendString(buffer);
        _delay_ms(500);
    }
}