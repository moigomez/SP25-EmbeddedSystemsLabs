;
; Lab4.asm
;
; Created: 4/7/2025
; Author : Moises Gomez
;
; ASM file incorporating all required functionality
; of Lab04.
;
; This code:
; 1. Uses an RPG encoder to increase/decrease the duty cycle of the PWM for controlling fan speed.
; 2. Uses a pushbutton switch to turn the fan on or off.
; 3. Uses an LCD to display the incrementing duty cycle and fan status.
;

/* === PWM + RPG === */

; REGISTERS
;
; R16 - Stores the current state of the RPG; Also serves
;       as temporary register for configuring PCIs          (regCurrent)
; R17 - Stores the value of the duty cycle (OCR0B)          (regDutyCycle)
; R18 - Temporary register configuring PWM
; R19 - Stores the previous state of the RPG                (regPrevious)
; R20 - Temporary register storing RPG transition states    (regTransitions)
; R21 - Temporary register for 'building' RPG transitions
; R22 - Temporary register for checking if RPG is on detent
;


.include "m328pdef.inc"

.equ RPG_A = 0                  ; PB0 - PCINT0
.equ RPG_B = 1                  ; PB1 - PCINT1
.equ TEST_PIN = 6               ; PD6 - Testing purposes

.def regTransitions = R20
.def regPrevious = R19
.def regCurrent = R16
.def regDutyCycle = R17

.org 0x0000
rjmp RESET
.org 0x0006
rjmp RPG_ISR

RESET:
    ldi R16, low(RAMEND)
    out SPL, R16
    ldi R16, high(RAMEND)
    out SPH, R16

    cbi DDRB, RPG_A            ; Set PB0 as input
    cbi DDRB, RPG_B            ; Set PB1 as input
    sbi DDRD, TEST_PIN         ; PD6 (Test) - Output

    ldi R16, (1 << PCINT0) | (1 << PCINT1)
    sts PCMSK0, R16
    ldi R16, (1 << PCIE0)
    sts PCICR, R16

    clr regPrevious
    clr regCurrent
    clr regTransitions
    clr R22

    sbic PINB, RPG_A
    ori regPrevious, (1 << 1)   ; Save A to bit 1
    sbic PINB, RPG_B
    ori regPrevious, (1 << 0)   ; Save B to bit 0

    sei

main_loop:
    rjmp main_loop

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
    rcall pwm_start             ;     pwm_start()
                                ; }
    ;sbi PORTD, TEST_PIN
    rjmp save_state

counterclockwise:
    cpi regDutyCycle, 0         ; if (regDutyCycle == 0) {
    breq save_state             ;     save_state()
                                ; } else {
    inc R22                     ;     R22++
    dec regDutyCycle            ;     regDutyCycle--
    rcall pwm_start             ;     pwm_start()
                                ; }
    ;cbi PORTD, TEST_PIN
save_state:
    mov regPrevious, regCurrent
    reti

pwm_start:
    out OCR0B, regDutyCycle
    sbi DDRD, 5                                             ; Set PD6 (OC0B) as output

    ; Configure Timer0
    ldi R18, (1 << WGM01) | (1 << WGM00) | (1 << COM0B1)    ; Set Fast PWM and non-inverting mode
    out TCCR0A, R18

    ldi R18, (1 << WGM02) | (1 << CS00)                     ; Prescaler = 1 and finish mode selection
    out TCCR0B, R18

    ; DC = (OCR0B / OCR0A) * 100
    ldi R18, 99
    out OCR0A, R18

    cpi R22, 4                  ; if (R22 != 4) {
    brne end                    ;     end()
    clr R22                     ; } else {
    out OCR0B, regDutyCycle     ;     OCR0B = regDutyCycle
                                ; }
    end:
    ret

/* ===============++ */