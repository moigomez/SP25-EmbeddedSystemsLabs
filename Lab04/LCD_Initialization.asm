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
    rcall _delay_100ms  ; Wait for 100ms
    
    ldi r25, 0x30
    rcall LCD_SendCommand
    rcall _delay_5ms     ; Wait 5ms

    ldi r25, 0x30
    rcall LCD_SendCommand
    rcall _delay_200us   ; Wait 200us

    ldi r25, 0x30
    rcall LCD_SendCommand
    rcall _delay_200us   ; Wait 200us

    ldi r25, 0x20  ; Set to 4-bit mode
    rcall LCD_SendCommand
    rcall _delay_5ms     ; Wait 5ms

    ldi r25, 0x01  ; Clear display
    rcall LCD_SendCommand
    rcall _delay_2ms     ; Wait 1.64ms

    ldi r25, 0x06  ; Entry mode set
    rcall LCD_SendCommand
    rcall _delay_40us    ; Wait 40us

    ret

;=====================================
; Send Command (RS = 0)
;=====================================
LCD_SendCommand:
    cbi PORTD, RS  ; RS = 0 (Command mode)
    rcall LCD_SendNibble
    ret

;=====================================
; Send Data (RS = 1)
;=====================================
LCD_SendData:
    sbi PORTD, RS  ; RS = 1 (Data mode)
    rcall LCD_SendNibble
    ret

;=====================================
; Send a Nibble
;=====================================
LCD_SendNibble:
    out PORTC, r25  ; Send upper or lower nibble to LCD
    rcall LCDStrobe  ; Pulse E line
    ret

;=====================================
; Strobe Enable Line
;=====================================
LCDStrobe:
    sbi PORTD, E    ; Enable high
    rcall _delay_100us  ; Short delay
    cbi PORTD, E    ; Enable low
    ret

;=====================================
; Delay Functions Using Timer1
;=====================================
_delay_100ms:
    ldi r24, 100
_delay_100ms_loop:
    rcall _delay_1ms
    dec r24
    brne _delay_100ms_loop
    ret

_delay_5ms:
    ldi r24, 5
_delay_5ms_loop:
    rcall _delay_1ms
    dec r24
    brne _delay_5ms_loop
    ret

_delay_2ms:
    ldi r24, 2
_delay_2ms_loop:
    rcall _delay_1ms
    dec r24
    brne _delay_2ms_loop
    ret

_delay_1ms:
    ldi r24, 250
_delay_1ms_loop:
    sbi TCNT1, 0    ; Start Timer1
    sbi TCCR1B, CS10 ; Set prescaler
    sbis TIFR1, TOV1
    rjmp _delay_1ms_loop
    ret

_delay_40us:
    ldi r24, 40
_delay_40us_loop:
    rcall _delay_1us
    dec r24
    brne _delay_40us_loop
    ret

_delay_200us:
    ldi r24, 5  ; 5 x 40us = 200us
_delay_200us_loop:
    rcall _delay_40us
    dec r24
    brne _delay_200us_loop
    ret

_delay_100us:
    ldi r24, 2
_delay_100us_loop:
    rcall _delay_40us
    dec r24
    brne _delay_100us_loop
    ret

;=====================================
; Main Program
;=====================================
main:
    ; Set PORTC as output (D4-D7 for LCD data)
    ldi r16, 0x0F
    out DDRC, r16
    
    ; Set PORTD as output (RS, E)
    ldi r16, 0x03
    out DDRD, r16
    
    rcall LCD_Init  ; Initialize LCD

    ; Send character 'E' (0x45)
    ldi r25, 0x04  ; Upper nibble of 'E'
    rcall LCD_SendData
    ldi r25, 0x05  ; Lower nibble of 'E'
    rcall LCD_SendData

    rjmp main  ; Loop indefinitely