;
; RPG_Interrupts.asm
;
; Created: 4/1/2025
; Author : Moises Gomez
;
; ASM file to implement RPG counter-clockwise and clockwise
; detection using interrupts
;

; The below is ChatGPT code

.include "m328pdef.inc"

; Enable Pin Change Interrupt for PB0 (PCINT8) and PB1 (PCINT9)
ldi R16, (1 << PCINT8) | (1 << PCINT9)  ; Enable PCINT8 and PCINT9
sts PCMSK0, R16                          ; Set mask register
ldi R16, (1 << PCIE0)                   ; Enable pin change interrupt for PCINT[7:0]
sts PCICR, R16                           ; Enable pin change interrupt control register

ldi R18, (1 << PD5)  ; Load value with bit 5 set
out DDRD, R18        ; Set PB5 as output

sei   ; Enable global interrupts

ISR_PCINT0:
    in R16, PINB      ; Read PINB (current state of A and B)
    mov R17, R16      ; Copy for comparison

    sbrs R16, PB0     ; Check if A is HIGH (skip next if set)
    rjmp Check_B      ; If A is LOW, check B next

    ; If A is HIGH, check previous state of B
    sbrc R17, PB1     ; If B was HIGH before
    rjmp CW_Rot       ; Clockwise rotation
    rjmp CCW_Rot      ; Counter-clockwise rotation

Check_B:
    sbrs R16, PB1     ; Check if B is HIGH (skip next if set)
    rjmp Done         ; If B is LOW, ignore (no rotation)

    ; If B is HIGH, check previous state of A
    sbrc R17, PB0     ; If A was HIGH before
    rjmp CCW_Rot      ; Counter-clockwise rotation
    rjmp CW_Rot       ; Clockwise rotation

CW_Rot:
    ; Handle clockwise rotation
    sbi PORTD, 5 
    rjmp Done

CCW_Rot:
    ; Handle counter-clockwise rotation
    cbi PORTD, 5 
    rjmp Done

Done:
    reti