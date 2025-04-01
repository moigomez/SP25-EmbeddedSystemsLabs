;
; LCD_Initialization.asm
;
; Created: 3/31/2025
; Author : Moises Gomez
;
; ASM file to initialize LCD functionality and display characters
;


; The below is ChatGPT code

.include "m328Pdef.inc" ; Include device-specific definitions

.equ RS = PB5  ; Register Select pin
.equ E  = PB3  ; Enable pin

.org 0x00  ; Start of program memory
rjmp main  ; Jump to main program

;=====================================
; LCD Initialization Sequence
;=====================================
LCD_Init:
    ; Wait for 100 ms
    rcall _delay_100ms  

    ; Set the device to 8-bit mode (send command 0x3)
    ldi R25, 0x30
    rcall LCD_SendCommand
    rcall _delay_5ms      ; Wait for 5 ms

    ; Set the device to 8-bit mode again (send command 0x3)
    ldi R25, 0x30
    rcall LCD_SendCommand
    rcall _delay_200us    ; Wait for 200 µs

    ; Set the device to 8-bit mode one last time (send command 0x3)
    ldi R25, 0x30
    rcall LCD_SendCommand
    rcall _delay_200us    ; Wait for 200 µs

    ; Set device to 4-bit mode (send command 0x2)
    ldi R25, 0x20
    rcall LCD_SendCommand
    rcall _delay_5ms      ; Wait for 5 ms

    ; Further LCD configuration (clear screen, entry mode, etc.)
    ldi R25, 0x01        ; Clear display command (0x01)
    rcall LCD_SendCommand
    rcall _delay_2ms      ; Wait for 1.64 ms

    ldi R25, 0x06        ; Entry mode set (0x06)
    rcall LCD_SendCommand
    rcall _delay_40us     ; Wait for 40 µs

    ret

;=====================================
; Send Command (RS = 0)
;=====================================
LCD_SendCommand:
    cbi PORTB, RS  ; RS = 0 (Command mode)
    rcall LCD_SendNibble
    ret

;=====================================
; Send Data (RS = 1)
;=====================================
LCD_SendData:
    sbi PORTB, RS  ; RS = 1 (Data mode)
    rcall LCD_SendNibble
    ret

;=====================================
; Send a Nibble
;=====================================
LCD_SendNibble:
    out PORTC, R25  ; Send upper or lower nibble to LCD
    rcall LCDStrobe  ; Pulse E line
    ret

;=====================================
; Strobe Enable Line
;=====================================
LCDStrobe:
    sbi PORTB, E    ; Enable high
    rcall _delay_100us  ; Short delay
    cbi PORTB, E    ; Enable low
    ret

;=====================================
; Delay Functions Using TimeR1
;=====================================
_delay_100ms:
    ldi R24, 100
_delay_100ms_loop:
    rcall _delay_1ms
    dec R24
    brne _delay_100ms_loop
    ret

_delay_5ms:
    ldi R24, 5
_delay_5ms_loop:
    rcall _delay_1ms
    dec R24
    brne _delay_5ms_loop
    ret

_delay_2ms:
    ldi R24, 2
_delay_2ms_loop:
    rcall _delay_1ms
    dec R24
    brne _delay_2ms_loop
    ret

_delay_1ms:
    ldi R24, 250
_delay_1ms_loop:
    ; sbi TCNT1, 0    ; Start TimeR1

    ldi R18, (1 << CS01) | (1 << CS00)  ; Set prescaler = 64
    out TCCR0B, R18

    sbis TIFR1, TOV1    ; Wait for overflow
    rjmp _delay_1ms_loop
    ret

_delay_40us:
    ldi R24, 40
_delay_40us_loop:
    rcall _delay_1ms
    dec R24
    brne _delay_40us_loop
    ret

_delay_200us:
    ldi R24, 5  ; 5 x 40us = 200us
_delay_200us_loop:
    rcall _delay_40us
    dec R24
    brne _delay_200us_loop
    ret

_delay_100us:
    ldi R24, 2
_delay_100us_loop:
    rcall _delay_40us
    dec R24
    brne _delay_100us_loop
    ret

;=====================================
; Main Program
;=====================================
main:
    ; Set PORTC as output (D4-D7 for LCD data)
    ldi R16, 0x0F
    out DDRC, R16
    
    ; Set PORTB as output (RS, E)
    ldi R16, 0x03
    out DDRC, R16
    
    rcall LCD_Init  ; Initialize LCD

    ; Send character 'E' (0x45)
    ldi R25, 0x04  ; Upper nibble of 'E'
    rcall LCD_SendData
    ldi R25, 0x05  ; Lower nibble of 'E'
    rcall LCD_SendData

    rjmp main  ; Loop indefinitely