;
; LCD_Initialization.asm
;
; Created: 3/31/2025
; Author : Moises Gomez
;
; ASM file to initialize LCD functionality and display characters.
; Additionaly, this code uses an interrupt to detect pushbutton press
;


; The below is tweaked ChatGPT code

.include "m328pdef.inc"

; Define constants
.equ LCD_RS = 5                 ; PB5 (Register Select, Arduino D13)
.equ LCD_E  = 3                 ; PB3 (Enable, Arduino D11)
.equ BUTTON_PIN = 2             ; PD2 (Pushbutton, INT0, Arduino D2)
.equ FAN_PIN = 5                ; PD5 (Cooling Fan, Arduino D5)

; LCD Commands
.equ LCD_CLEAR      = 0x01
.equ LCD_ENTRY_MODE = 0x06      ; Increment cursor, no display shift
.equ LCD_DISPLAY_ON = 0x0C      ; Display on, cursor off, blink off
.equ LCD_FUNCTION_SET = 0x28    ; 4-bit mode, 2 lines, 5x8 font
.equ LCD_SET_DDRAM  = 0x80      ; Base address for DDRAM (add offset for position)

 .def regTestNumber = R19

; Data segment
.dseg
.org SRAM_START
fan_state: .byte 1              ; 0 = off, 1 = on
update_flag: .byte 1            ; Flag to indicate LCD update needed (0 = no update, 1 = update)

; Code segment
.cseg
.org 0x0000
rjmp RESET
.org 0x0002
rjmp INT0_ISR

RESET:
    ; Initialize stack pointer
    ldi R16, low(RAMEND)
    out SPL, R16
    ldi R16, high(RAMEND)
    out SPH, R16

    ; Set up I/O pins
    sbi DDRB, LCD_RS            ; PB5 as output (LCD RS)
    sbi DDRB, LCD_E             ; PB3 as output (LCD E)
    sbi DDRC, 0                 ; PC0 as output (LCD D4)
    sbi DDRC, 1                 ; PC1 as output (LCD D5)
    sbi DDRC, 2                 ; PC2 as output (LCD D6)
    sbi DDRC, 3                 ; PC3 as output (LCD D7)
    sbi DDRD, FAN_PIN           ; PD5 as output (Cooling Fan)
    cbi DDRD, BUTTON_PIN        ; PD2 as input (Pushbutton, INT0)
    sbi PORTD, BUTTON_PIN       ; Enable pull-up resistor on PD2

    ; Clear PORTC, PORTB, and PORTD to a known state
    ldi R16, 0x00
    out PORTC, R16
    out PORTB, R16
    out PORTD, R16

    ; Initialize fan state to off
    ldi R16, 0
    sts fan_state, R16
    sts update_flag, R16        ; Clear update flag

    ldi regTestNumber, 75       ; Load arbitrary number to display on LCD

    ; Set up External Interrupt INT0 on PD2
    ldi R16, 0                  ; ISC01=0, ISC00=0: Low level on INT0 triggers interrupt
    sts EICRA, R16
    ldi R16, (1<<INT0)          ; Enable INT0 interrupt
    out EIMSK, R16

    sei                         ; Enable global interrupts

    ; Initialize LCD
    rcall lcd_init
    ; Display initial message
    rcall display_fan_state

main_loop:
    ; Check if LCD update is needed
    lds R16, update_flag
    cpi R16, 1
    brne main_loop_no_update

    ; Update LCD
    rcall lcd_init
    rcall display_fan_state
    ldi R16, 0
    sts update_flag, R16

main_loop_no_update:
    rjmp main_loop

INT0_ISR:
    push R16
    in R16, SREG
    push R16
    push R17

    ; Disable INT0 to prevent repeated triggers while button is pressed
    ldi R16, 0
    out EIMSK, R16

    ; Toggle fan state
    lds R16, fan_state
    cpi R16, 0
    breq turn_fan_on_isr
turn_fan_off_isr:
    ldi R16, 0
    sts fan_state, R16
    cbi PORTD, FAN_PIN  ; Turn fan off
    rjmp set_update_flag
turn_fan_on_isr:
    ldi R16, 1
    sts fan_state, R16
    sbi PORTD, FAN_PIN  ; Turn fan on

set_update_flag:
    ldi R16, 1
    sts update_flag, R16

wait_loop:
    in R16, PIND
    andi R16, (1<<BUTTON_PIN)
    cpi R16, (1<<BUTTON_PIN)  ; Check if PD2 is high
    brne wait_loop

    ; Re-enable INT0
    ldi R16, (1<<INT0)
    out EIMSK, R16

isr_exit:
    pop R17
    pop R16
    out SREG, R16
    pop R16
    reti

/* ******* DISPLAY TEXT ON LCD ****** */

; Display fan state

display_fan_state:
    push R16
    push R17
    push R18
    push R19  ; Preserve regTestNumber (R19)

    ; Clear display
    ldi R16, LCD_CLEAR
    rcall lcd_command
    ldi R16, 50
    rcall delay_ms

    ; Set cursor to first row, first column (DDRAM address 0x00)
    ldi R16, LCD_SET_DDRAM | 0x00
    rcall lcd_command

    ; Display "DC="
    ldi R16, 'D'
    rcall lcd_data
    ldi R16, 'C'
    rcall lcd_data
    ldi R16, '='
    rcall lcd_data

    ; Convert regTestNumber to ASCII and display
    mov R16, regTestNumber  ; Load duty cycle (0-100)
    ; Extract tens digit
    ldi R17, 10
    clr R18              ; Clear remainder
div_tens:
    sub R16, R17
    brlo tens_done
    inc R18
    rjmp div_tens
tens_done:
    add R16, R17                ; Add back the last subtraction
    mov R19, R16                ; Save ones digit in R19 (temporary, not regTestNumber)
    mov R16, R18                ; Move tens digit to R16
    cpi R16, 0                  ; If tens digit is 0, display space (for 0-9)
    breq display_space
    subi R16, -48               ; Convert to ASCII ('0' = 48)
    rjmp display_tens
display_space:
    ldi R16, ' '                ; Display space for single-digit numbers
display_tens:
    rcall lcd_data

    ; Display ones digit
    mov R16, R19
    subi R16, -48               ; Convert to ASCII
    rcall lcd_data

    ; Display ".0%"
    ldi R16, '.'
    rcall lcd_data
    ldi R16, '0'
    rcall lcd_data
    ldi R16, '%'
    rcall lcd_data

    ; Set cursor to second row, first column (DDRAM address 0x40 for 16x2 LCD)
    ldi R16, LCD_SET_DDRAM | 0x40
    rcall lcd_command

    ; Display "Fan is "
    ldi R16, 'F'
    rcall lcd_data
    ldi R16, 'a'
    rcall lcd_data
    ldi R16, 'n'
    rcall lcd_data
    ldi R16, ' '
    rcall lcd_data
    ldi R16, 'i'
    rcall lcd_data
    ldi R16, 's'
    rcall lcd_data
    ldi R16, ' '
    rcall lcd_data

    ; Display "ON" or "OFF" based on fan_state
    lds R16, fan_state
    cpi R16, 0
    breq display_off
display_on:
    ldi R16, 'O'
    rcall lcd_data
    ldi R16, 'N'
    rcall lcd_data
    rjmp display_end
display_off:
    ldi R16, 'O'
    rcall lcd_data
    ldi R16, 'F'
    rcall lcd_data
    ldi R16, 'F'
    rcall lcd_data

display_end:
    pop R19  ; Restore regTestNumber (R19)
    pop R18
    pop R17
    pop R16
    ret

/* ******* INITIALIZE LCD ****** */

lcd_init:
    ; Wait for LCD to power up (1000 ms = 250 ms * 4)
    ldi R16, 250
    rcall delay_ms
    ldi R16, 250
    rcall delay_ms
    ldi R16, 250
    rcall delay_ms
    ldi R16, 250
    rcall delay_ms

    ; Step 1: Set to 8-bit mode (D7-D4 = 0x03)
    ldi R16, 0x03
    in R17, PORTC
    andi R17, 0xF0              ; Clear PC0-PC3 (D4-D7)
    or R17, R16                 ; Set D7-D4 on PC3-PC0
    out PORTC, R17
    cbi PORTB, LCD_RS           ; RS = 0 for command
    sbi PORTB, LCD_E
    ldi R16, 10                 ; 10 ms for Enable pulse
    rcall delay_ms
    cbi PORTB, LCD_E
    ldi R16, 50                 ; 50 ms delay
    rcall delay_ms

    ; Step 3: Set to 8-bit mode again (D7-D4 = 0x03)
    ldi R16, 0x03
    in R17, PORTC
    andi R17, 0xF0
    or R17, R16
    out PORTC, R17
    cbi PORTB, LCD_RS
    sbi PORTB, LCD_E
    ldi R16, 10
    rcall delay_ms
    cbi PORTB, LCD_E
    ldi R16, 50
    rcall delay_ms

    ; Step 5: Set to 8-bit mode a third time (D7-D4 = 0x03)
    ldi R16, 0x03
    in R17, PORTC
    andi R17, 0xF0
    or R17, R16
    out PORTC, R17
    cbi PORTB, LCD_RS
    sbi PORTB, LCD_E
    ldi R16, 10
    rcall delay_ms
    cbi PORTB, LCD_E
    ldi R16, 50
    rcall delay_ms

    ; Step 7: Set to 4-bit mode (D7-D4 = 0x02)
    ldi R16, 0x02
    in R17, PORTC
    andi R17, 0xF0
    or R17, R16
    out PORTC, R17
    cbi PORTB, LCD_RS
    sbi PORTB, LCD_E
    ldi R16, 10
    rcall delay_ms
    cbi PORTB, LCD_E
    ldi R16, 50
    rcall delay_ms

    ; Step 9: Send full commands in 4-bit mode
    ldi R16, LCD_FUNCTION_SET
    rcall lcd_command

    ; Step 11: Display on, cursor off, blink off
    ldi R16, LCD_DISPLAY_ON
    rcall lcd_command

    ; Step 12: Clear Display
    ldi R16, LCD_CLEAR
    rcall lcd_command
    ldi R16, 50
    rcall delay_ms

    ; Step 13: Entry Mode: increment cursor, no display shift
    ldi R16, LCD_ENTRY_MODE
    rcall lcd_command

    ret

; Send a command to the LCD (4-bit mode)
lcd_command:
    ; High nibble
    push R16
    mov R17, R16
    andi R17, 0xF0              ; Mask to keep only upper 4 bits
    swap R17                    ; Swap to get upper nibble in lower 4 bits
    in R18, PORTC
    andi R18, 0xF0              ; Clear PC0-PC3
    or R18, R17
    out PORTC, R18
    cbi PORTB, LCD_RS           ; RS = 0 for command
    sbi PORTB, LCD_E
    ldi R16, 10                 ; 10 ms for Enable pulse
    rcall delay_ms
    cbi PORTB, LCD_E
    ldi R16, 1                  ; 100 µs between nibbles
    rcall delay_ms

    ; Low nibble
    pop R16
    andi R16, 0x0F              ; Mask to keep only lower 4 bits
    in R18, PORTC
    andi R18, 0xF0
    or R18, R16
    out PORTC, R18
    cbi PORTB, LCD_RS
    sbi PORTB, LCD_E
    ldi R16, 10
    rcall delay_ms
    cbi PORTB, LCD_E
    ldi R16, 50                 ; 50 ms after command
    rcall delay_ms
    ret

; Send data to the LCD (4-bit mode)
lcd_data:
    ; High nibble
    push R16
    mov R17, R16
    andi R17, 0xF0              ; Mask to keep only upper 4 bits
    swap R17                    ; Swap to get upper nibble in lower 4 bits
    in R18, PORTC
    andi R18, 0xF0              ; Clear PC0-PC3
    or R18, R17
    out PORTC, R18
    sbi PORTB, LCD_RS           ; RS = 1 for data
    sbi PORTB, LCD_E
    ldi R16, 10                 ; 10 ms for Enable pulse
    rcall delay_ms
    cbi PORTB, LCD_E
    ldi R16, 1                  ; 100 µs between nibbles
    rcall delay_ms

    ; Low nibble
    pop R16
    andi R16, 0x0F   ;           Mask to keep only lower 4 bits
    in R18, PORTC
    andi R18, 0xF0
    or R18, R16
    out PORTC, R18
    sbi PORTB, LCD_RS
    sbi PORTB, LCD_E
    ldi R16, 10
    rcall delay_ms
    cbi PORTB, LCD_E
    ldi R16, 50                 ; 50 ms after data
    rcall delay_ms
    ret

/* ******* LCD DELAYS ****** */

delay_ms:
    push R16
    push R17
delay_ms_loop:
    cpi R16, 0
    breq delay_ms_done
    dec R16
    ldi R17, 250  ; 250 * 4 cycles = 1000 cycles = 1 ms at 1 MHz
delay_ms_inner:
    dec R17
    nop
    brne delay_ms_inner
    rjmp delay_ms_loop
delay_ms_done:
    pop R17
    pop R16
    ret