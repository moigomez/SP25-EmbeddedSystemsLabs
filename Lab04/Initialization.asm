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
    sbi DDRD, PD5  

    ; Set Fast PWM Mode 7 (WGM02:0 = 7) -> TOP = OCR0A
    ldi R16, (1 << WGM00) | (1 << WGM01)  ; Set WGM01 and WGM00
    sts TCCR0A, R16

    ldi R16, (1 << WGM02) | (1 << CS00)   ; Set WGM02 and Prescaler = 1
    sts TCCR0B, R16

    ; Set OCR0A for 80 kHz PWM frequency (TOP = 199)
    ldi R16, 199
    sts OCR0A, R16

    ; Set initial duty cycle (50% -> OCR0B = 100)
    ldi R16, 100
    sts OCR0B, R16

    ; Enable non-inverting mode on OC0B (PD5)
    ldi R16, (1 << COM0B1)  ; Clear OC0B on Compare Match, set at BOTTOM
    sts TCCR0A, R16

loop:
    rjmp loop  ; Infinite loop