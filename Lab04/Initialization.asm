;
; Initialization.asm
;
; Created: 3/27/2025
; Author : Moises Gomez
;
; ASM file to initialize PWM functionality for fan
;

.include "m328pdef.inc" 

.cseg
.org 0x00
rjmp start

.org 0x30
start:
    ; Set PB1 (OC1A) as output
    sbi DDRB, PB1  

    ; Configure Timer1 in Fast PWM mode (Mode 14)
    ldi r16, (1 << WGM11)  ; WGM11: Fast PWM mode
    sts TCCR1A, r16

    ldi r16, (1 << WGM13) | (1 << WGM12) | (1 << CS11)  ; Mode 14, Prescaler = 8
    sts TCCR1B, r16

    ; Set ICR1 for 25 kHz PWM frequency
    ldi r16, LOW(79)
    sts ICR1L, r16
    ldi r16, HIGH(79)
    sts ICR1H, r16

    ; Set initial duty cycle (50%)
    ldi r16, LOW(40)
    sts OCR1AL, r16
    ldi r16, HIGH(40)
    sts OCR1AH, r16

    ; Enable output compare mode (non-inverting mode)
    ldi r16, (1 << COM1A1)
    sts TCCR1A, r16

loop:
    rjmp loop  ; Infinite loop