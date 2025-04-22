/*
 *
 * DAC_Initialization.c
 *
 * Created: 4/21/2025
 * Author:  Moises Gomez
 *
 * C-program to test MAX518 D/A converter with ATmega328P
 *
 */


// The below is ChatGPT code

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

#define SCL_CLOCK 100000L // 100kHz I2C

// Change according to how A0 and A1 are wired
#define MAX518_ADDR 0x2C // 0b0101100x << 1 = 0x58, 7-bit address = 0x2C

void TWI_init(void) {
    TWSR = 0x00; // Prescaler = 1
    TWBR = ((F_CPU/SCL_CLOCK)-16)/2;
    TWCR = (1<<TWEN); // Enable TWI
}

void TWI_start(void) {
    TWCR = (1<<TWSTA)|(1<<TWEN)|(1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
}

void TWI_stop(void) {
    TWCR = (1<<TWSTO)|(1<<TWEN)|(1<<TWINT);
    while (TWCR & (1<<TWSTO)); // Wait for stop to be sent
}

void TWI_write(uint8_t data) {
    TWDR = data;
    TWCR = (1<<TWEN)|(1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
}

void MAX518_write(uint8_t value) {
    TWI_start();
    TWI_write((MAX518_ADDR << 1) | 0); // Write mode
    TWI_write(value); // DAC value (0-255)
    TWI_stop();
}

int main(void) {
    TWI_init();

    while (1) {
        for (uint8_t i = 0; i < 255; i++) {
            MAX518_write(i);
            _delay_ms(10);
        }
        for (int i = 255; i >= 0; i--) {
            MAX518_write(i);
            _delay_ms(10);
        }
    }
}