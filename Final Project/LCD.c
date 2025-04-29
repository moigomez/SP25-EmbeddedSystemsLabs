/*
 *
 * LCD.c
 *
 * Created: 4/29/2025
 * Author:  Moises Gomez
 *
 * C-program for initializing I2C-based 20x4 LCD
 * 
 */


#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>

#define LCD_ADDR 0x27
#define SCL_CLOCK 100000L

// PCF8574 to LCD pin mapping
#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE    0x04
#define LCD_RW        0x02
#define LCD_RS        0x01

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

void LCD_Send_Nibble(uint8_t nibble, uint8_t mode) {
    uint8_t data = nibble << 4;

    data |= LCD_BACKLIGHT;
    if (mode) data |= LCD_RS;

    // EN pulse
    TWI_Start();
    TWI_Write(LCD_ADDR << 1);
    TWI_Write(data | LCD_ENABLE);
    TWI_Write(data & ~LCD_ENABLE);
    TWI_Stop();
}

void LCD_Send_Byte(uint8_t value, uint8_t mode) {
    LCD_Send_Nibble(value >> 4, mode);
    LCD_Send_Nibble(value & 0x0F, mode);
    _delay_us(50);
}

void LCD_Command(uint8_t cmd) {
    LCD_Send_Byte(cmd, 0);
    if (cmd == 0x01 || cmd == 0x02) _delay_ms(2);
}

void LCD_Data(uint8_t data) {
    LCD_Send_Byte(data, 1);
}

void LCD_Init(void) {
    _delay_ms(50); // Wait for LCD power-on

    // Init 4-bit mode
    for (uint8_t i = 0; i < 3; i++) {
        LCD_Send_Nibble(0x03, 0);
        _delay_ms(5);
    }
    LCD_Send_Nibble(0x02, 0); // 4-bit mode
    _delay_ms(1);

    LCD_Command(0x28); // 4-bit, 2-line, 5x8 dots
    LCD_Command(0x08); // Display off
    LCD_Command(0x01); // Clear display
    LCD_Command(0x06); // Entry mode
    LCD_Command(0x0C); // Display on, cursor off
}

void LCD_Set_Cursor(uint8_t row, uint8_t col) {
    const uint8_t offsets[] = {0x00, 0x40, 0x14, 0x54};
    LCD_Command(0x80 | (offsets[row] + col));
}

void LCD_Print(const char *str) {
    while (*str) LCD_Data(*str++);
}

int main(void) {
    TWI_Init();
    LCD_Init();

    LCD_Set_Cursor(0, 0);
    LCD_Print("Hello, World!");

    LCD_Set_Cursor(1, 0);
    LCD_Print("20x4 I2C LCD Demo");

    while (1) {
    }
}