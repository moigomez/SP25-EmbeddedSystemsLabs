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
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
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

// Timer for millis()
volatile uint32_t millis_counter = 0;
ISR(TIMER1_COMPA_vect) { millis_counter++; }
void Timer1_Init(void) {
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11) | (1 << CS10);  // Prescaler 64
    OCR1A = 249;                          // 1ms @ 16MHz/64
    TIMSK1 |= (1 << OCIE1A);
    sei();
}
uint32_t millis(void) {
    uint32_t ms;
    cli();
    ms = millis_counter;
    sei();
    return ms;
}

float compute_derivative(int16_t current, int16_t previous, float dt_ms) {
    return (float)(current - previous) / dt_ms;
}

// Moving average related
#define MA_WINDOW_SIZE 20
float mag_buffer[MA_WINDOW_SIZE] = {0};
uint8_t mag_index = 0;
float mag_sum = 0;


int main(void) {
    USART_Init(MYUBRR);
    TWI_Init();
    MPU6050_Init();
    Timer1_Init();

    DDRB |= (1 << PB5);

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    int16_t prev_az = 0;
    uint32_t prev_time = 0;
    uint32_t last_reset_time = 0;

    int count = 0;
    bool isShaking = false;
    while (1) {
        uint32_t now = millis();
        float dt = now - prev_time;
        if ((now - last_reset_time) >= 300) {
            count = 0;
            isShaking = false;
            last_reset_time = now;
        }

        MPU6050_ReadAccel(&ax, &ay, &az);
        MPU6050_ReadGyro(&gx, &gy, &gz);

        float daz_dt = 0.0;

        if (dt > 0) { 
            daz_dt = compute_derivative(az, prev_az, dt);
        }
        prev_az = az;
        prev_time = now;

        // Compute current magnitude
        float mag = sqrt((double)ax * ax + (double)ay * ay + (double)az * az);

        // Subtract oldest value, add new value
        mag_sum -= mag_buffer[mag_index];
        mag_buffer[mag_index] = mag;
        mag_sum += mag;
        // Advance index circularly
        mag_index = (mag_index + 1) % MA_WINDOW_SIZE;
        // Compute average
        float mag_avg = mag_sum / MA_WINDOW_SIZE;

        // Check if conditions met for shaking
        if (((mag >= 36000.0 && mag < 40000.0 && mag_avg > 22000.0) || (daz_dt >= 250.0 && daz_dt < 400.0))) {
            count++;
        }

        if (count >= 4) {
            isShaking = true;
            PORTB |= (1 << PB5);
        } else {
            PORTB &= ~(1 << PB5);
        }
        _delay_ms(10);  // Added to stabilize loop rate
    }
}