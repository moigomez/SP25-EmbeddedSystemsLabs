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
 #include <util/twi.h>
 
 #define F_CPU 16000000UL
 #define F_SCL 100000UL // 100kHz
 #define TWBR_VAL (((F_CPU / F_SCL) - 16) / 2)
 
 // Define the address (change if A0 is tied high)
 #define MAX518_ADDR 0xD0
 
 void i2c_init(void) {
     TWBR = (uint8_t)TWBR_VAL;
 }
 
 void i2c_start(void) {
     TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
     while (!(TWCR & (1 << TWINT)));
 }
 
 void i2c_stop(void) {
     TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
 }
 
 void i2c_write(uint8_t data) {
     TWDR = data;
     TWCR = (1 << TWEN) | (1 << TWINT);
     while (!(TWCR & (1 << TWINT)));
 }
 
 void max518_write(uint8_t channel, uint8_t value) {
     uint8_t command = 0;
 
     switch (channel) {
         case 0: command = 0x00; break; // DAC A
         case 1: command = 0x40; break; // DAC B
         case 2: command = 0x80; break; // Both DACs
         default: return; // Invalid
     }
 
     i2c_start();
     i2c_write(MAX518_ADDR);    // Write address
     i2c_write(command);        // Command byte
     i2c_write(value);          // DAC data
     i2c_stop();
 }

 int main(void) {
    i2c_init();

    while (1) {
        max518_write(0, 128);  // Set DAC A to mid-scale (Vout = Vref/2)
        _delay_ms(500);
        max518_write(1, 255);  // Set DAC B to full scale
        _delay_ms(500);
    }
}