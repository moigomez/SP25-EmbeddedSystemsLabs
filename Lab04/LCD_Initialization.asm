;
; LCD_Initialization.asm
;
; Created: 3/31/2025
; Author : Moises Gomez
;
; ASM file to initialize LCD functionality and display characters
;


; The below is ChatGPT code

;-------------------------------------------------
; LCD Initialization Routine (4-bit Mode)
;-------------------------------------------------
.equ LCD_PORT = PORTB  ; LCD data lines
.equ LCD_DDR  = DDRB   ; Data Direction Register
.equ LCD_RS   = 5      ; Register Select pin (PB5)
.equ LCD_EN   = 3      ; Enable pin (PB3)

.include "m328Pdef.inc"

.org 0x00
rjmp main          ; Reset vector jump to main program

;-------------------------------------------------
; Subroutine: LCDStrobe
; Toggles the Enable pin to latch data into LCD
;-------------------------------------------------
LCDStrobe:
    sbi LCD_PORT, LCD_EN   ; Set Enable High
    rcall _delay_100u      ; Small delay
    cbi LCD_PORT, LCD_EN   ; Set Enable Low
    ret

;-------------------------------------------------
; Subroutine: SendNibble
; Sends a 4-bit nibble to the LCD
; Assumes data is in R25 (lower nibble ignored)
;-------------------------------------------------
SendNibble:
    out LCD_PORT, R25   ; Send 4-bit data
    rcall LCDStrobe     ; Strobe to latch data
    ret

;-------------------------------------------------
; Subroutine: SendCommand
; Sends a full 8-bit command in 4-bit mode
; Assumes command is in R24
;-------------------------------------------------
SendCommand:
    cbi LCD_PORT, LCD_RS   ; RS = 0 (Command Mode)
    mov R25, R24
    lsr R25               ; Shift right to get upper nibble
    lsr R25
    lsr R25
    lsr R25
    rcall SendNibble       ; Send upper nibble

    mov R25, R24
    andi R25, 0x0F        ; Mask lower nibble
    rcall SendNibble       ; Send lower nibble
    rcall _delay_2ms       ; Wait for processing
    ret

;-------------------------------------------------
; LCD Initialization
;-------------------------------------------------
LCD_Init:
    ; Set LCD pins as output
    ldi R25, 0x3F        ; Set PC0-PC5 as output
    out LCD_DDR, R25

    ; Wait for LCD to power up
    rcall _delay_100ms   ; Wait 100ms

    ; Send 8-bit mode command three times
    ldi R24, 0x30
    rcall SendNibble
    rcall _delay_5ms     ; Wait 5ms

    rcall SendNibble
    rcall _delay_200us   ; Wait 200µs

    rcall SendNibble
    rcall _delay_200us   ; Wait 200µs

    ; Set to 4-bit mode
    ldi R24, 0x20
    rcall SendNibble
    rcall _delay_5ms     ; Wait 5ms

    ; Send function set (4-bit, 2-line, 5x8 font)
    ldi R24, 0x28
    rcall SendCommand

    ; Display on, cursor off
    ldi R24, 0x0C
    rcall SendCommand

    ; Clear display
    ldi R24, 0x01
    rcall SendCommand
    rcall _delay_2ms

    ; Entry mode set
    ldi R24, 0x06
    rcall SendCommand

    ret

;-------------------------------------------------
; Subroutine: SendChar
; Sends a single character to the LCD
; Assumes character ASCII value is in R24
;-------------------------------------------------
SendChar:
    sbi LCD_PORT, LCD_RS  ; RS = 1 (Data Mode)

    ; Send upper nibble
    mov R25, R24
    lsr R25
    lsr R25
    lsr R25
    lsr R25
    rcall SendNibble      ; Send upper nibble

    ; Send lower nibble
    mov R25, R24
    andi R25, 0x0F        ; Mask lower nibble
    rcall SendNibble      ; Send lower nibble

    rcall _delay_100u     ; Short delay
    ret

;-------------------------------------------------
; Delays (Assume 16MHz Clock)
;-------------------------------------------------
_delay_100ms:
    ldi R18, 250
    rcall _delay_400us
    dec R18
    brne _delay_100ms
    ret

_delay_5ms:
    ldi R18, 50
    rcall _delay_100us
    dec R18
    brne _delay_5ms
    ret

_delay_2ms:
    ldi R18, 20
    rcall _delay_100us
    dec R18
    brne _delay_2ms
    ret

_delay_200us:
    ldi R18, 2
    rcall _delay_100us
    ret

_delay_100u:
    ldi R19, 50
_loop100u:
    dec R19
    brne _loop100u
    ret

_delay_400us:
    ldi R19, 200
_loop400u:
    dec R19
    brne _loop400u
    ret

;-------------------------------------------------
; Main Program
;-------------------------------------------------
main:
    rcall LCD_Init     ; Initialize LCD

    ; Send the character 'E'
    ldi R24, 0x45       ; Load ASCII value of 'E'
    rcall SendChar      ; Send it to LCD

    rjmp main          ; Loop forever