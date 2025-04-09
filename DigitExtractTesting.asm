;
; DigitExtractTesting.asm
;
; Created: 4/9/2025
; Author : moigomez
;
; ASM file to test out a subroutine that hopefully
; allows us to extract each digit of a multidigit number.
;

; The below is ChatGPT code

.include "m328pdef.inc"

; Define duty cycle value for demonstration
.duty_cycle_val: .byte 2   ; Two bytes (e.g. 0756 for 75.6%)

; Program Start
.org 0x0000
rjmp main

; --- Main Program ---
main:
    ; Initialize Registers
    ldi R16, 75      ; Set LSB of duty cycle (example: 75)
    ldi R17, 6       ; Set MSB of duty cycle (example: 6)
    sts duty_cycle_val, R16  ; Store LSB in memory
    sts duty_cycle_val+1, R17 ; Store MSB in memory

    ; Call divide_by_100
    lds R16, duty_cycle_val    ; Load LSB
    lds R17, duty_cycle_val+1  ; Load MSB
    rcall divide_by_100        ; Divides R17:R16 by 100, result in R18

    ; Now R18 contains the hundreds place (quotient), R17:R16 contains the remainder

    ; Call divide_by_10 with the remainder to get tens and ones digits
    rcall divide_by_10        ; Divides the remainder (R17:R16) by 10
    ; Now R18 contains tens place, R17 contains ones place

    rjmp main  ; Loop forever (to simulate continuous display)

; --- Subroutine to divide by 100 ---
divide_by_100:
    clr R18          ; Clear quotient (R18 = 0)
div100_loop:
    ldi R19, low(100)   ; Load low byte of 100
    ldi R20, high(100)  ; Load high byte of 100
    cp  R16, R19        ; Compare LSB
    cpc R17, R20        ; Compare MSB
    brlo div100_done    ; If R17:R16 < 100, we're done

    ; Subtract 100 from R17:R16
    subi R16, low(100)
    sbci R17, high(100)

    inc R18             ; Increment quotient (R18)
    rjmp div100_loop    ; Repeat

div100_done:
    ret

; --- Subroutine to divide by 10 ---
divide_by_10:
    clr R18          ; Clear quotient (R18 = 0)
div10_loop:
    ldi R19, 10      ; Load 10 to compare
    clr R20
    cp  R16, R19     ; Compare LSB with 10
    cpc R17, R20     ; Compare MSB with 0
    brlo div10_done  ; If less than 10, done

    ; Subtract 10 from R17:R16
    subi R16, 10
    sbci R17, 0

    inc R18          ; Increment quotient (R18)
    rjmp div10_loop  ; Repeat

div10_done:
    mov R17, R16     ; Copy LSB to R17 (ones digit)
    ret
