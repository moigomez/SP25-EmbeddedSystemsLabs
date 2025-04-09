;
; DigitExtractTesting.asm
;
; Created: 4/9/2025
; Author : moigomez
;
; ASM file to test out a subroutine that hopefully
; allows us to extract each digit of a multidigit number.
;


.include "m328Ppef.inc"

.def temp = R16             ; Temporary register
.def input_value = R24      ; Stores the chosen input
.def result = R20           ; Stores the result of the function

.org 0x0000
  rjmp main

my_routine:
  mov R18, R24
  lsr R18
  mov R25, R24
  andi R25, lo8(-2)
  add R25, R18
  lsr R24
  lsr R24
  lsr R24
  add R24, R25
  swap R24
  andi R24, lo8(15)
  ret

main:
  ldi input_value, 0b10110110   ; 182

  rcall my_routine
  ; The result of my_routine will be in R24 upon return.
  mov result, R24

infinite_loop:
  rjmp infinite_loop
