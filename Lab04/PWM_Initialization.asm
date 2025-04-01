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
    ; Set PD5 (OC0B) as output
    sbi DDRD, 5  

    ; Configure Timer0
    ldi R16, (1 << WGM01) | (1 << WGM00) | (1 << COM0B1)  ; Set Fast PWM and non-inverting mode
    out TCCR0A, R16

    ldi R16, (1 << WGM02) | (1 << CS00)  ; Prescaler = 1 and finish mode selection
    out TCCR0B, R16

    ; DC = (OCR0B / OCR0A) * 100
    ldi R16, 99
    out OCR0A, R16

    ldi R16, 49
    out OCR0B, R16

loop:
    rjmp loop