.include "m328Pdef.inc"

.def temp = R16
.def counter = R17
.def digit_idx = R23
.def time_count_low = R24  ; Low byte of 16-bit time_count
.def time_count_high = R25 ; High byte of 16-bit time_count

.cseg
.org 0x00

;-------------------------------------------------
; Configure I/O Lines and Timer0
;-------------------------------------------------
sbi DDRB, 0            ; Shift register outputs
sbi DDRB, 1
sbi DDRB, 2
cbi DDRB, 3            ; Button input with pull-up
sbi PORTB, 3
sbi DDRB, 5            ; LED (L) on PB5 as output
cbi PORTB, 5           ; Ensure LED is off at start
cbi DDRD, 6            ; RPG A on PD6 as input with pull-up
sbi PORTD, 6
cbi DDRD, 7            ; RPG B on PD7 as input with pull-up
sbi PORTD, 7

; Setup Timer0 (Normal mode, prescaler 64)
ldi temp, (1 << CS01) | (1 << CS00)  ; Prescaler 64 (16MHz / 64 = 250kHz)
out TCCR0B, temp
clr temp
out TCCR0A, temp       ; Normal mode

;-------------------------------------------------
; Initialize State, SRAM, and Display to "-"
;-------------------------------------------------
in R21, PIND           ; Read RPG state from PD6/PD7
andi R21, 0b11000000   ; Mask PD6 and PD7
ldi R18, 16
ldi digit_idx, 0

; Initialize unlock_code in SRAM
ldi ZL, LOW(unlock_code)
ldi ZH, HIGH(unlock_code)
ldi temp, 0x06
st Z+, temp
ldi temp, 0x0D
st Z+, temp
ldi temp, 0x07
st Z+, temp
ldi temp, 0x07
st Z+, temp
ldi temp, 0x0D
st Z+, temp

rcall update_display

;-------------------------------------------------
; Main Program Loop
;-------------------------------------------------
mainLoop:
    rcall check_rpg
    rcall check_button
    rjmp mainLoop

;-------------------------------------------------
; Timer0 500us Delay Subroutine
;-------------------------------------------------
delay_TCO_500us:
    push temp
    in temp, TCCR0B        ; Save config
    clr temp
    out TCCR0B, temp       ; Stop timer
    ldi temp, 0x83         ; Initial count (256 - 125 = 131)
    out TCNT0, temp        ; Load counter
    ldi temp, (1 << CS01) | (1 << CS00)  ; Prescaler 64
    out TCCR0B, temp       ; Restart timer
wait:
    sbis TIFR0, TOV0       ; Check overflow flag
    rjmp wait
    ldi temp, 1<<TOV0      ; Clear overflow flag
    out TIFR0, temp
    pop temp
    ret

;-------------------------------------------------
; Button Detection with Timer0
;-------------------------------------------------
check_button:
    in temp, PINB
    sbrc temp, 3           ; Skip if button not pressed (PB3 high)
    rjmp button_not_pressed
    clr R24                ; Reset 16-bit time_count (R25:R24)
    clr R25
button_pressed:
    in temp, PINB          ; Check button state
    sbrs temp, 3           ; Skip if still pressed
    rjmp button_still_pressed
    rjmp button_released   ; Button released, process
button_still_pressed:
    rcall delay_TCO_500us  ; Wait 500us
    adiw R25:R24, 1        ; Increment counter
    rjmp button_pressed
button_released:
    cpi R24, LOW(2000)     ; 1s = 2000 * 500us
    ldi temp, HIGH(2000)
    cpc R25, temp
    brsh check_long_press
    inc digit_idx          ; Short press: increment digit index
    cpi digit_idx, 5
    brne next_digit
    rcall check_code       ; Check code after 5 digits
    ldi digit_idx, 0
    ldi R18, 16            ; Reset to "-"
    rcall update_display
    rjmp button_end
next_digit:
    ldi R18, 16            ; Short press: show "-"
    rcall update_display
    rjmp button_end
check_long_press:
    cpi R24, LOW(10000)    ; 5s = 10000 * 500us
    ldi temp, HIGH(10000)
    cpc R25, temp
    brlo button_end        ; Ignore if < 5s
    ldi digit_idx, 0
    ldi R18, 0             ; Long press: reset to "0"
    rcall update_display
button_end:
    clr R24                ; Reset time_count
    clr R25
    ret
button_not_pressed:
    clr R24
    clr R25
    ret

;-------------------------------------------------
; Check Entered Code
;-------------------------------------------------
check_code:
    ldi ZL, LOW(code_buffer)
    ldi ZH, HIGH(code_buffer)
    ldi YL, LOW(unlock_code)
    ldi YH, HIGH(unlock_code)
    ldi counter, 5
check_loop:
    ld temp, Z+
    ld R20, Y+
    cp temp, R20
    brne code_wrong
    dec counter
    brne check_loop
    ldi R16, 0b01000000    ; Display "."
    rcall display
    sbi PORTB, 5           ; Turn on LED (PB5)
    rcall delay_4s
    cbi PORTB, 5           ; Turn off LED
    ret
code_wrong:
    ldi R16, 0b00001000    ; Display "_"
    rcall display
    rcall delay_7s
    ret

;-------------------------------------------------
; RPG Detection
;-------------------------------------------------
check_rpg:
    in R19, PIND           ; Read PIND for PD6 and PD7
    andi R19, 0b11000000   ; Mask PD6 (bit 6) and PD7 (bit 7)
    cp R19, R21
    breq rpg_end
    mov R22, R21
    mov R21, R19
    cpi R22, 0b00000000    ; Previous state 00
    brne skip_cw
    cpi R19, 0b01000000    ; Current state 01 (PD6 high)
    breq increment
skip_cw:
    cpi R22, 0b00000000    ; Previous state 00
    brne rpg_end
    cpi R19, 0b10000000    ; Current state 10 (PD7 high)
    breq decrement
rpg_end:
    ret

increment:
    cpi R18, 15          ; If at "F", stop
    breq rpg_end
    cpi R18, 16          ; If at "-", reset to 0
    breq reset_to_zero
    inc R18              ; Otherwise, increment
    rjmp update_now
reset_to_zero:
    ldi R18, 0           ; Reset from "-" to "0"
    rjmp update_now

decrement:
    cpi R18, 0
    breq rpg_end
    dec R18
    rjmp update_now

update_now:
    rcall update_display
    ldi ZL, LOW(code_buffer)
    ldi ZH, HIGH(code_buffer)
    add ZL, digit_idx
    brcc no_carry_store
    inc ZH
no_carry_store:
    st Z, R18
    ret

;-------------------------------------------------
; Display Update Routine
;-------------------------------------------------
update_display:
    cpi R18, 16
    breq dash_display
    cpi R18, 0
    brlo clamp_to_zero
    cpi R18, 15
    brsh clamp_to_f
    rjmp normal_display
dash_display:
    ldi R16, 0b10000000
    rjmp send_to_display
normal_display:
    ldi ZL, LOW(digit_patterns << 1)
    ldi ZH, HIGH(digit_patterns << 1)
    add ZL, R18
    brcc no_carry
    inc ZH
no_carry:
    lpm R16, Z
    rjmp send_to_display
clamp_to_zero:
    ldi R18, 0
    ldi R16, 0b01101111
    rjmp send_to_display
clamp_to_f:
    ldi R18, 15
    ldi R16, 0b11100001
    rjmp send_to_display
send_to_display:
    rcall display
    ret

;-------------------------------------------------
; 7-Segment Display Patterns
;-------------------------------------------------
.align 2
digit_patterns:
    .db 0b01101111, 0b00000110, 0b11001011, 0b10001111, 0b10100110, 0b10101101, 0b11101101, 0b00000111
    .db 0b11101111, 0b10101111, 0b11100111, 0b11101100, 0b01101001, 0b11001110, 0b11101001, 0b11100001

;-------------------------------------------------
; Display Subroutine
;-------------------------------------------------
display:
    push temp
    push counter
    in counter, SREG
    push counter
    ldi counter, 8
shift_loop:
    rol R16
    brcs set_ser_1
    cbi PORTB, 0
    rjmp clk_pulse
set_ser_1:
    sbi PORTB, 0
clk_pulse:
    sbi PORTB, 2
    nop
    cbi PORTB, 2
    dec counter
    brne shift_loop
    nop
    sbi PORTB, 1
    nop
    cbi PORTB, 1
    pop counter
    out SREG, counter
    pop counter
    pop temp
    ret

;-------------------------------------------------
; Delays (Using Timer0)
;-------------------------------------------------
delay_4s:  ; 4s = 8000 * 500us
    ldi XL, LOW(8000)
    ldi XH, HIGH(8000)
delay_4s_loop:
    rcall delay_TCO_500us
    sbiw XH:XL, 1
    brne delay_4s_loop
    ret

delay_7s:  ; 7s = 14000 * 500us
    ldi XL, LOW(14000)
    ldi XH, HIGH(14000)
delay_7s_loop:
    rcall delay_TCO_500us
    sbiw XH:XL, 1
    brne delay_7s_loop
    ret

;-------------------------------------------------
; SRAM Buffers
;-------------------------------------------------
.dseg
.org SRAM_START
code_buffer: .byte 5
unlock_code: .byte 5