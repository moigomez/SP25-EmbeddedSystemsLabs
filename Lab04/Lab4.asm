;
; Lab4.asm
;
; Created: 4/7/2025
; Author : Emre Akgül & Moises Gomez
;
; ASM file incorporating all required functionality
; of Lab04.
;
; This code:
; 1. Uses an RPG encoder to increase/decrease the duty cycle of the PWM for controlling fan speed.
; 2. Uses a pushbutton switch to turn the fan on or off.
; 3. Uses an LCD to display the incrementing duty cycle and fan status.
;

; REGISTERS
; R16 - Stores the current state of the RPG; Also serves
;       as temporary register for configuring PCIs          (regCurrent)
; R17 - Stores the value of the duty cycle (OCR0B)          (regDutyCycle)
; R18 - Temporary register configuring PWM
; R19 - Stores the previous state of the RPG                (regPrevious)
; R20 - Temporary register storing RPG transition states    (regTransitions)
; R21 - Temporary register for 'building' RPG transitions
; R22 - Temporary register for checking if RPG is on detent
; R23 - Stores the value of the duty cycle to be displayed  (regDCDisplay)
; R24 - Temporary register for initializing LCD
; R25 - Temporary register for initializing LCD
;

.include "m328pdef.inc"

; Define constants
.equ RPG_A = 0                  ; PB0 - PCINT0
.equ RPG_B = 1                  ; PB1 - PCINT1
.equ LCD_RS = 5                 ; PB5 - Register Select
.equ LCD_E  = 3                 ; PB3 - Enable
.equ BUTTON_PIN = 2             ; PD2 - INT0
.equ FAN_PIN = 5                ; PD5 - Cooling fan

; Define register names
.def regCurrent = R16
.def regDutyCycle = R17
.def regPrevious = R19
.def regTransitions = R20
.def regDCDisplay = R23

; LCD Commands
.equ LCD_CLEAR      = 0x01
.equ LCD_ENTRY_MODE = 0x06      ; Increment cursor, no display shift
.equ LCD_DISPLAY_ON = 0x0C      ; Display on, cursor off, blink off
.equ LCD_FUNCTION_SET = 0x28    ; 4-bit mode, 2 lines, 5x8 font
.equ LCD_SET_DDRAM  = 0x80      ; Base address for DDRAM (add offset for position)

; Data segment
.dseg
.org SRAM_START
fan_state: .byte 1              ; 0 = off, 1 = on
update_flag: .byte 1            ; Flag to indicate LCD update needed (0 = no update, 1 = update)

; Code segment
.cseg
.org 0x0000
rjmp RESET
.org 0x0002                     ; Pushbutton interrupt vector
rjmp INT0_ISR
.org 0x0006                     ; RPG interrupt vector
rjmp RPG_ISR

RESET:
    ; Initialize stack pointer
    ldi R16, low(RAMEND)
    out SPL, R16
    ldi R16, high(RAMEND)
    out SPH, R16

    ; Set up I/O pins
    cbi DDRB, RPG_A             ; Set PB0 as input
    cbi DDRB, RPG_B             ; Set PB1 as input
    sbi DDRB, LCD_RS            ; PB5 as output (LCD RS)
    sbi DDRB, LCD_E             ; PB3 as output (LCD E)
    sbi DDRC, 0                 ; PC0 as output (LCD D4)
    sbi DDRC, 1                 ; PC1 as output (LCD D5)
    sbi DDRC, 2                 ; PC2 as output (LCD D6)
    sbi DDRC, 3                 ; PC3 as output (LCD D7)
    sbi DDRD, FAN_PIN           ; PD5 as output (Cooling Fan)
    cbi DDRD, BUTTON_PIN        ; PD2 as input (Pushbutton, INT0)
    sbi PORTD, BUTTON_PIN       ; Enable pull-up resistor on PD2

    clr regPrevious
    clr regCurrent
    clr regTransitions
    clr R22

    ; Initialize fan state to off
    clr R20
    sts fan_state, R20
    sts update_flag, R20        ; Clear update flag

    sbic PINB, RPG_A
    ori regPrevious, (1 << 1)   ; Save A to bit 1
    sbic PINB, RPG_B
    ori regPrevious, (1 << 0)   ; Save B to bit 0

    ldi R16, (1 << PCINT0) | (1 << PCINT1)
    sts PCMSK0, R16
    ldi R16, (1 << PCIE0)
    sts PCICR, R16

    ; Set up External Interrupt INT0 on PD2
    ldi R20, 0                  ; ISC01=0, ISC00=0: Low level on INT0 triggers interrupt
    sts EICRA, R20
    ldi R20, (1<<INT0)          ; Enable INT0 interrupt
    out EIMSK, R20

    sei

    ; Initialize LCD and display
    rcall lcd_init
    rcall display_fan_state

main_loop:
    ; Check if LCD update is needed
    lds R20, update_flag
    cpi R20, 1
    brne main_loop_no_update

    ; Update LCD
    rcall lcd_init
    rcall display_fan_state
    ldi R20, 0
    sts update_flag, R20

main_loop_no_update:
    rjmp main_loop

/* === PWM + RPG === */

RPG_ISR:
    ; Get current state
    in regCurrent, PINB
    andi regCurrent, 3          ; Mask for PB0 and PB1 (bits 1:0)

    mov R21, regPrevious
    lsl R21                     ; Shift left twice
    lsl R21
    or  R21, regCurrent
    mov regTransitions, R21

    cpi regTransitions, 0b0001
    breq clockwise
    cpi regTransitions, 0b0111
    breq clockwise
    cpi regTransitions, 0b1110
    breq clockwise
    cpi regTransitions, 0b1000
    breq clockwise

    cpi regTransitions, 0b0010
    breq counterclockwise
    cpi regTransitions, 0b1011
    breq counterclockwise
    cpi regTransitions, 0b1101
    breq counterclockwise
    cpi regTransitions, 0b0100
    breq counterclockwise

    rjmp save_state
clockwise:
    cpi regDutyCycle, 99        ; if (regDutyCycle >= 99) {
    brsh save_state             ;     save_state()
                                ; } else {
    inc R22                     ;     R22++
    inc regDutyCycle            ;     regDutyCycle++
    rcall start_pwm             ;     start_pwm()
                                ; }
    rjmp save_state

counterclockwise:
    cpi regDutyCycle, 0         ; if (regDutyCycle == 0) {
    breq save_state             ;     save_state()
                                ; } else {
    inc R22                     ;     R22++
    dec regDutyCycle            ;     regDutyCycle--
    rcall start_pwm             ;     start_pwm()
                                ; }
save_state:
    mov regPrevious, regCurrent
    reti

start_pwm:
    out OCR0B, regDutyCycle
    sbi DDRD, FAN_PIN

    ; Configure Timer0
    ldi R18, (1 << WGM01) | (1 << WGM00) | (1 << COM0B1)    ; Set Fast PWM and non-inverting mode
    out TCCR0A, R18

    ldi R18, (1 << WGM02) | (1 << CS00)                     ; Prescaler = 1 and finish mode selection
    out TCCR0B, R18

    ; DC = (OCR0A / OCR0B) * 100
    ldi R18, 199
    out OCR0A, R18

    cpi R22, 4                  ; if (R22 != 4) {
    brne end_pwm                ;     end_pwm()
    clr R22                     ; } else {
    out OCR0B, regDutyCycle     ;     OCR0B = regDutyCycle
                                ; }
end_pwm:
    ret

/* == END SECTION == */


/* === LCD + PBS === */

INT0_ISR:
    push R20
    in R20, SREG
    push R20
    push R24

    ; Disable INT0 to prevent repeated triggers while button is pressed
    ldi R20, 0
    out EIMSK, R20

    ; Toggle fan state
    lds R20, fan_state
    cpi R20, 0
    breq turn_fan_on_isr
turn_fan_off_isr:
    ldi R20, 0
    sts fan_state, R20
    cbi PORTD, FAN_PIN          ; Turn fan off
    rjmp set_update_flag
turn_fan_on_isr:
    ldi R20, 1
    sts fan_state, R20
    sbi PORTD, FAN_PIN          ; Turn fan on

set_update_flag:
    ldi R20, 1
    sts update_flag, R20

wait_loop:
    in R20, PIND
    andi R20, (1<<BUTTON_PIN)
    cpi R20, (1<<BUTTON_PIN)    ; Check if PD2 is high
    brne wait_loop

    ; Re-enable INT0
    ldi R20, (1<<INT0)
    out EIMSK, R20

isr_exit:
    pop R24
    pop R20
    out SREG, R20
    pop R20
    reti

/* DISPLAY TEXT ON LCD */

; Display fan state

display_fan_state:
    push R20
    push R24
    push R25
    push R23                    ; Preserve regDCDisplay (R23)

    ; Clear display
    ldi R20, LCD_CLEAR
    rcall lcd_command
    ldi R20, 50
    rcall delay_ms

    ; Set cursor to first row, first column (DDRAM address 0x00)
    ldi R20, LCD_SET_DDRAM | 0x00
    rcall lcd_command

    ; Display "DC="
    ldi R20, 'D'
    rcall lcd_data
    ldi R20, 'C'
    rcall lcd_data
    ldi R20, '='
    rcall lcd_data

    ; Convert regDCDisplay to ASCII and display
    mov regDCDisplay, R17
    mov R20, regDCDisplay       ; Load duty cycle (0-100)
    ; Extract tens digit
    ldi R24, 10
    clr R25                     ; Clear remainder
div_tens:
    sub R20, R24
    brlo tens_done
    inc R25
    rjmp div_tens
tens_done:
    add R20, R24                ; Add back the last subtraction
    mov R23, R20                ; Save ones digit in R23 (temporary, not regDCDisplay)
    mov R20, R25                ; Move tens digit to R20
    cpi R20, 0                  ; If tens digit is 0, display space (for 0-9)
    breq display_space
    subi R20, -48               ; Convert to ASCII ('0' = 48)
    rjmp display_tens
display_space:
    ldi R20, ' '                ; Display space for single-digit numbers
display_tens:
    rcall lcd_data

    ; Display ones digit
    mov R20, R23
    subi R20, -48               ; Convert to ASCII
    rcall lcd_data

    ; Display ".0%"
    ldi R20, '.'
    rcall lcd_data
    ldi R20, '0'
    rcall lcd_data
    ldi R20, '%'
    rcall lcd_data

    ; Set cursor to second row, first column
    ldi R20, LCD_SET_DDRAM | 0x40
    rcall lcd_command

    ; Display "Fan is "
    ldi R20, 'F'
    rcall lcd_data
    ldi R20, 'a'
    rcall lcd_data
    ldi R20, 'n'
    rcall lcd_data
    ldi R20, ' '
    rcall lcd_data
    ldi R20, 'i'
    rcall lcd_data
    ldi R20, 's'
    rcall lcd_data
    ldi R20, ' '
    rcall lcd_data

    ; Display "ON" or "OFF" depending on fan_state
    lds R20, fan_state
    cpi R20, 0
    breq display_off
display_on:
    ldi R20, 'O'
    rcall lcd_data
    ldi R20, 'N'
    rcall lcd_data
    rjmp display_end
display_off:
    ldi R20, 'O'
    rcall lcd_data
    ldi R20, 'F'
    rcall lcd_data
    ldi R20, 'F'
    rcall lcd_data

display_end:
    pop R23                     ; Restore regDCDisplay (R23)
    pop R25
    pop R24
    pop R20
    ret

/* INITIALIZE LCD */

lcd_init:
    ; Wait for LCD to power up (1000 ms = 250 ms * 4)
    ldi R20, 250
    rcall delay_ms
    ldi R20, 250
    rcall delay_ms
    ldi R20, 250
    rcall delay_ms
    ldi R20, 250
    rcall delay_ms

    ; Set to 8-bit mode (D7-D4 = 0x03)
    ldi R20, 0x03
    in R24, PORTC
    andi R24, 0xF0              ; Clear PC0-PC3 (D4-D7)
    or R24, R20                 ; Set D7-D4 on PC3-PC0
    out PORTC, R24
    cbi PORTB, LCD_RS           ; RS = 0 for command
    sbi PORTB, LCD_E
    ldi R20, 10                 ; 10 ms for Enable pulse
    rcall delay_ms
    cbi PORTB, LCD_E
    ldi R20, 50                 ; 50 ms delay
    rcall delay_ms

    ; Set to 8-bit mode again (D7-D4 = 0x03)
    ldi R20, 0x03
    in R24, PORTC
    andi R24, 0xF0
    or R24, R20
    out PORTC, R24
    cbi PORTB, LCD_RS
    sbi PORTB, LCD_E
    ldi R20, 10
    rcall delay_ms
    cbi PORTB, LCD_E
    ldi R20, 50
    rcall delay_ms

    ; Set to 8-bit mode a third time (D7-D4 = 0x03)
    ldi R20, 0x03
    in R24, PORTC
    andi R24, 0xF0
    or R24, R20
    out PORTC, R24
    cbi PORTB, LCD_RS
    sbi PORTB, LCD_E
    ldi R20, 10
    rcall delay_ms
    cbi PORTB, LCD_E
    ldi R20, 50
    rcall delay_ms

    ; Set to 4-bit mode (D7-D4 = 0x02)
    ldi R20, 0x02
    in R24, PORTC
    andi R24, 0xF0
    or R24, R20
    out PORTC, R24
    cbi PORTB, LCD_RS
    sbi PORTB, LCD_E
    ldi R20, 10
    rcall delay_ms
    cbi PORTB, LCD_E
    ldi R20, 50
    rcall delay_ms

    ; Send full commands in 4-bit mode
    ldi R20, LCD_FUNCTION_SET
    rcall lcd_command

    ; Display on, cursor off, blink off
    ldi R20, LCD_DISPLAY_ON
    rcall lcd_command

    ; Clear Display
    ldi R20, LCD_CLEAR
    rcall lcd_command
    ldi R20, 50
    rcall delay_ms

    ; Entry Mode: increment cursor, no display shift
    ldi R20, LCD_ENTRY_MODE
    rcall lcd_command

    ret

; Send a command to the LCD (4-bit mode)
lcd_command:
    ; High nibble
    push R20
    mov R24, R20
    andi R24, 0xF0              ; Mask to keep only upper 4 bits
    swap R24                    ; Swap to get upper nibble in lower 4 bits
    in R25, PORTC
    andi R25, 0xF0              ; Clear PC0-PC3
    or R25, R24
    out PORTC, R25
    cbi PORTB, LCD_RS           ; RS = 0 for command
    sbi PORTB, LCD_E
    ldi R20, 10                 ; 10 ms for Enable pulse
    rcall delay_ms
    cbi PORTB, LCD_E
    ldi R20, 1                  ; 100 µs between nibbles
    rcall delay_ms

    ; Low nibble
    pop R20
    andi R20, 0x0F              ; Mask to keep only lower 4 bits
    in R25, PORTC
    andi R25, 0xF0
    or R25, R20
    out PORTC, R25
    cbi PORTB, LCD_RS
    sbi PORTB, LCD_E
    ldi R20, 10
    rcall delay_ms
    cbi PORTB, LCD_E
    ldi R20, 50                 ; 50 ms after command
    rcall delay_ms
    ret

; Send data to the LCD (4-bit mode)
lcd_data:
    ; High nibble
    push R20
    mov R24, R20
    andi R24, 0xF0              ; Mask to keep only upper 4 bits
    swap R24                    ; Swap to get upper nibble in lower 4 bits
    in R25, PORTC
    andi R25, 0xF0              ; Clear PC0-PC3
    or R25, R24
    out PORTC, R25
    sbi PORTB, LCD_RS           ; RS = 1 for data
    sbi PORTB, LCD_E
    ldi R20, 10                 ; 10 ms for Enable pulse
    rcall delay_ms
    cbi PORTB, LCD_E
    ldi R20, 1                  ; 100 µs between nibbles
    rcall delay_ms

    ; Low nibble
    pop R20
    andi R20, 0x0F              ; Mask to keep only lower 4 bits
    in R25, PORTC
    andi R25, 0xF0
    or R25, R20
    out PORTC, R25
    sbi PORTB, LCD_RS
    sbi PORTB, LCD_E
    ldi R20, 10
    rcall delay_ms
    cbi PORTB, LCD_E
    ldi R20, 50                 ; 50 ms after data
    rcall delay_ms
    ret

/* LCD DELAYS */

delay_ms:
    push R20
    push R24
delay_ms_loop:
    cpi R20, 0
    breq delay_ms_done
    dec R20
    ldi R24, 250                ; 250 * 4 cycles = 1000 cycles 
                                ; = 1 ms at 1 MHz
delay_ms_inner:
    dec R24
    nop
    brne delay_ms_inner
    rjmp delay_ms_loop
delay_ms_done:
    pop R24
    pop R20
    ret

/* == END SECTION == */