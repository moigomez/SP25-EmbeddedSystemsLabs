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

    ; Configure Timer0 for Fast PWM (Mode 7: WGM02:0 = 7)
    ldi R16, (1 << WGM02)  ; Set WGM02
    sts TCCR0A, R16

    ldi R16, (1 << WGM01) | (1 << WGM00) | (1 << COM0B1)  ; Fast PWM, non-inverting
    sts TCCR0A, R16

    ; Set OCR0A for 25 kHz PWM frequency
    ldi R16, 9
    sts OCR0A, R16

    ; Set OCR0B for 50% duty cycle
    ldi R16, 5
    sts OCR0B, R16

    ; Set prescaler to 64 and start Timer0
    ldi R16, (1 << CS01) | (1 << CS00)  ; Prescaler 64
    sts TCCR0B, R16

loop:
    rjmp loop  ; Infinite loop