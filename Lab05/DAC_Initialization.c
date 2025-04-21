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

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>

#define F_CPU 16000000UL
#define SCL_CLOCK 100000L

#define MAX518_ADDR 0x58 // 7-bit address
#define MAX518_ADDR_WRITE (MAX518_ADDR << 1)

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

void MAX518_SetVoltage(uint8_t channel, float voltage) {
    if (voltage < 0.0) voltage = 0.0;
    if (voltage > 5.0) voltage = 5.0;

    uint8_t value = (uint8_t)((voltage / 5.0) * 255.0); // 0-255 scale

    TWI_Start();
    TWI_Write(MAX518_ADDR_WRITE);           // Address + Write
    TWI_Write(channel == 0 ? 0x00 : 0x01);  // Channel 0 or 1
    TWI_Write(value);                       // Voltage value
    TWI_Stop();
}

int main(void) {
    TWI_Init();

    while (1) {
        MAX518_SetVoltage(0, 2.50); // Set channel 0 to 2.5V
        _delay_ms(1000);
        MAX518_SetVoltage(1, 1.25); // Set channel 1 to 1.25V
        _delay_ms(1000);
    }
}