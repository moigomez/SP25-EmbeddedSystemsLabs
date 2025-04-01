;
; LCD_Initialization.asm
;
; Created: 3/31/2025
; Author : Moises Gomez
;
; ASM file to initialize LCD functionality and display characters
;


; The below is ChatGPT code

.equ RS = 5      ; RS connected to PD0
.equ E  = 3      ; E connected to PD1
.equ LCD_PORT = PORTC  ; Data lines connected to PC0-PC3
.equ CTRL_PORT = PORTB ; Control lines on PD0, PD1
.equ CTRL_DDR = DDRB
.equ LCD_DDR = DDRC

.include "m328Pdef.inc"

.org 0x00
rjmp main

; Subroutine: Delay for approximately 1ms (adjust based on clock)
_delay_1ms:
    ldi R18, 250
    ldi R19, 250
_delay_1ms_loop:
    dec R19
    brne _delay_1ms_loop
    dec R18
    brne _delay_1ms_loop
    ret

; Subroutine: Generate Enable Pulse (E)
LCDStrobe:
    sbi CTRL_PORT, E    ; E = 1
    nop
    nop
    cbi CTRL_PORT, E    ; E = 0
    ret

; Subroutine to send a character to the LCD
; Input: R24 contains the character to send (e.g., 'E' = 0x45)
SendCharToLCD:
    ; Send upper nibble
    lsr R24            ; Shift out upper nibble to the lower nibble
    lsr R24
    lsr R24
    lsr R24
    out LCD_PORT, R24  ; Send upper nibble to LCD data lines (PC0-PC3)
    sbi CTRL_PORT, RS  ; Set RS = 1 (data mode)
    rcall LCDStrobe    ; Pulse E to latch upper nibble

    ; Send lower nibble
    swap R24           ; Swap nibbles to bring lower nibble to the upper side
    out LCD_PORT, R24  ; Send lower nibble to LCD data lines (PC0-PC3)
    rcall LCDStrobe    ; Pulse E to latch lower nibble

    rcall _delay_1ms   ; Optional: wait for 40 μs for the character to be processed
    ret

; Subroutine: Send Command to LCD (4-bit mode)
LCDCommand:
    out LCD_PORT, R24   ; Send upper nibble
    cbi CTRL_PORT, RS   ; RS = 0 (command mode)
    rcall LCDStrobe     ; Pulse E
    swap R24            ; Swap nibbles
    out LCD_PORT, R24   ; Send lower nibble
    rcall LCDStrobe     ; Pulse E
    rcall _delay_1ms    ; Wait for command to execute
    ret

; Subroutine: Initialize LCD
LCDInit:
    rcall _delay_1ms    ; Wait for 100ms
    rcall _delay_1ms
    rcall _delay_1ms
    rcall _delay_1ms
    rcall _delay_1ms
    rcall _delay_1ms
    rcall _delay_1ms
    rcall _delay_1ms
    rcall _delay_1ms
    rcall _delay_1ms

    ldi R24, 0x30       ; Function set: 8-bit mode
    rcall LCDCommand
    rcall _delay_1ms

    ldi R24, 0x30
    rcall LCDCommand
    rcall _delay_1ms

    ldi R24, 0x30
    rcall LCDCommand
    rcall _delay_1ms

    ldi R24, 0x20       ; Set to 4-bit mode
    rcall LCDCommand
    rcall _delay_1ms

    ; Now configure the LCD
    ldi R24, 0x28       ; 4-bit, 2-line, 5x8 dots
    rcall LCDCommand
    ldi R24, 0x0C       ; Display ON, cursor OFF, blink OFF
    rcall LCDCommand
    ldi R24, 0x06       ; Entry mode: Increment, no shift
    rcall LCDCommand
    ldi R24, 0x01       ; Clear display
    rcall LCDCommand
    rcall _delay_1ms    ; Wait for clear display (1.64ms)
    ret

; Main Program
main:
    ldi R16, (1 << RS) | (1 << E)  ; Set RS and E as outputs
    out CTRL_DDR, R16
    ldi R16, 0x0F
    out LCD_DDR, R16

    rcall LCDInit      ; Initialize LCD

    ldi R24, 0x45      ; ASCII 'E'
    rcall LCDCommand   ; Send 'E'

loop:
    rjmp loop          ; Infinite loop