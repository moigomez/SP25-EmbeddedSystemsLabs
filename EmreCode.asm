;
; LAB3.asm
;
; Created: 3/2/2025 3:01:26 PM
; Author : Emre Akgül, Moises Gomez
;


.include "m328Pdef.inc"

.def temp1 = R16
.def temp2 = R17
.def input_digit_index = R23
.cseg
.org 0

;-------------------------------------------------
; Configure I/O Lines and Timer0
;-------------------------------------------------
sbi DDRB, 0         ; PB0 (SER) - Output
sbi DDRB, 1         ; PB1 (RCLK) - Output
sbi DDRB, 2         ; PB2 (SRCLK) - Output
cbi DDRB, 3         ; PB3 (BUTTON) - Input
cbi DDRD, 6         ; PB4 (RPG A) - Input
cbi DDRD, 7         ; PB5 (RPG B) - Input

sbi PORTB, 3        ; Enable pull-up resistor on PB3
sbi PORTD, 6        ; Enable pull-up on PB4
sbi PORTD, 7        ; Enable pull-up on PB5

sbi DDRB, 5         ; Turn Off LED (L)
cbi PORTB, 5

; Setup Timer0
ldi temp1, (1 << CS01) | (1 << CS00)    ; Prescaler 64 (16MHz / 64 = 250kHz)
out TCCR0B, temp1
clr temp1
out TCCR0A, temp1                       ; Normal mode

;-------------------------------------------------
; Initialize State, SRAM, and Display to "-"
;-------------------------------------------------
in R21, PIND                        ; Read RPG state from PD6/PD7
andi R21, 0b11000000                ; Mask PD6 and PD7
ldi R18, 16
ldi input_digit_index, 0

; Initialize unlock_code in SRAM
ldi ZL, LOW(unlock_code)
ldi ZH, HIGH(unlock_code)
ldi temp1, 0x06
st Z+, temp1
ldi temp1, 0x0D
st Z+, temp1
ldi temp1, 0x07
st Z+, temp1
ldi temp1, 0x07
st Z+, temp1
ldi temp1, 0x0D
st Z+, temp1

rcall update_display

;-------------------------------------------------
; Main Program Loop
;-------------------------------------------------
main_loop:          rcall check_rpg
                    rcall check_button
                    rjmp main_loop

;-------------------------------------------------
; Timer0 500us Delay Subroutine
;-------------------------------------------------
delay_TCO_500us:    push temp1
                    in temp1, TCCR0B                        ; Save config
                    clr temp1
                    out TCCR0B, temp1                       ; Stop timer
                    ldi temp1, 0x83                         ; Initial count (256 - 125 = 131)
                    out TCNT0, temp1                        ; Load counter
                    ldi temp1, (1 << CS01) | (1 << CS00)    ; Prescaler 64
                    out TCCR0B, temp1                       ; Restart timer

wait:               sbis TIFR0, TOV0                        ; Check overflow flag
                    rjmp wait
                    ldi temp1, 1<<TOV0                      ; Clear overflow flag
                    out TIFR0, temp1
                    pop temp1
                    ret

;-------------------------------------------------
; Button Detection with Timer0
;-------------------------------------------------
check_button:       in temp1, PINB
                    sbrc temp1, 3                       ; Skip if button not pressed (PB3 high)
                    rjmp btn_not_pressed
                    clr R24                             ; Reset 16-bit time_count (R25:R24)
                    clr R25
                    
btn_pressed:        in temp1, PINB                      ; Check button state
                    sbrs temp1, 3                       ; Skip if still pressed
                    rjmp btn_still_pressed
                    rjmp btn_released                   ; Button released, process

btn_still_pressed:
                    rcall delay_TCO_500us               ; Wait 500us
                    adiw R25:R24, 1                     ; Increment counter
                    rjmp btn_pressed

btn_released:       cpi R24, LOW(2000)                  ; 1s = 2000 * 500us
                    ldi temp1, HIGH(2000)
                    cpc R25, temp1
                    brsh check_long_press
                    inc input_digit_index               ; Short press: increment digit index
                    cpi input_digit_index, 5
                    brne digit_stored
                    rcall check_code                    ; Check code after 5 digits
                    ldi input_digit_index, 0
                    ldi R18, 16                         ; Reset to "-"
                    rcall update_display
                    rjmp btn_end
                    
digit_stored:       rcall update_display
                    ldi ZL, LOW(input_temp_storage)
                    ldi ZH, HIGH(input_temp_storage)  
                    add ZL, input_digit_index
                    brcc no_carry_store_btn       
                    inc ZH

no_carry_store_btn:
                    st Z, R18
                    rjmp btn_end

check_long_press:   cpi R24, LOW(4000)                  ; 2s = 4000 * 500us
                    ldi temp1, HIGH(4000)
                    cpc R25, temp1
                    brlo btn_end                        ; Ignore if < 5s
                    ldi input_digit_index, 0
                    ldi R18, 16                         ; Long press: reset to "-"
                    rcall update_display

btn_end:            clr R24                                ; Reset time_count
                    clr R25
                    ret

btn_not_pressed:    clr R24
                    clr R25
                    ret

;-------------------------------------------------
; Check Entered Code
;-------------------------------------------------
check_code:         ldi ZL, LOW(input_temp_storage)
                    ldi ZH, HIGH(input_temp_storage)
                    ldi YL, LOW(unlock_code)
                    ldi YH, HIGH(unlock_code)
                    ldi temp2, 5

check_loop:         ld temp1, Z+
                    ld R20, Y+
                    cp temp1, R20
                    brne code_wrong
                    dec temp2
                    brne check_loop
                    ldi R16, 0b00010000                 ; Display "."
                    rcall display
                    sbi PORTB, 5                        ; Turn on LED (PB5)
                    rcall delay_4s
                    cbi PORTB, 5                        ; Turn off LED
                    ret

code_wrong:         ldi R16, 0b00001000                 ; Display "_"
                    rcall display
                    rcall delay_7s
                    ret

;-------------------------------------------------
; RPG Detection
;-------------------------------------------------
check_rpg:          in R19, PIND                        ; Read PIND for PD6 and PD7
                    andi R19, 0b11000000                ; Mask PD6 (bit 6) and PD7 (bit 7)
                    cp R19, R21
                    breq rpg_end
                    mov R22, R21
                    mov R21, R19
                    cpi R22, 0b00000000                 ; Previous state 00
                    brne skip_cw
                    cpi R19, 0b01000000                 ; Current state 01 (PD6 high)
                    breq increment

skip_cw:            cpi R22, 0b00000000                 ; Previous state 00
                    brne rpg_end
                    cpi R19, 0b10000000                 ; Current state 10 (PD7 high)
                    breq decrement

rpg_end:            ret

increment:          cpi R18, 15                         ; If at "F", stop
                    breq rpg_end
                    cpi R18, 16                         ; If at "-", reset to 0
                    breq reset_to_zero
                    inc R18                             ; Otherwise, increment
                    rjmp update_now

reset_to_zero:      ldi R18, 0                          ; Reset from "-" to "0"
                    rjmp update_now

decrement:          cpi R18, 0
                    breq rpg_end
                    dec R18
                    rjmp update_now

update_now:         rcall update_display
                    ldi ZL, LOW(input_temp_storage)
                    ldi ZH, HIGH(input_temp_storage)
                    add ZL, input_digit_index
                    brcc no_carry_store
                    inc ZH

no_carry_store:     st Z, R18
                    ret

;-------------------------------------------------
; Display Update Routine
;-------------------------------------------------
update_display:     cpi R18, 16                         ; if (R18 == 16) {
                    breq dash_display                   ;     dash_display()
                    cpi R18, 0                          ; } else if (R18 < 0) {
                    brlo clamp_to_zero                  ;     clamp_to_zero()
                    cpi R18, 15                         ; } else if (R18 >= 15) {
                    brsh clamp_to_f                     ;     clamp_to_f()

                    rjmp normal_display                 ; otherwise

dash_display:       ldi R16, 0b10000000
                    rjmp send_to_display

normal_display:     ldi ZL, LOW(digit_patterns << 1)
                    ldi ZH, HIGH(digit_patterns << 1)
                    add ZL, R18
                    brcc no_carry
                    inc ZH

no_carry:           lpm R16, Z
                    rjmp send_to_display

clamp_to_zero:      ldi R18, 0
                    ldi R16, 0b01101111
                    rjmp send_to_display

clamp_to_f:         ldi R18, 15
                    ldi R16, 0b11100001
                    rjmp send_to_display

send_to_display:    rcall display
                    ret

;-------------------------------------------------
; 7-Segment Display Patterns
;-------------------------------------------------
.align 2
digit_patterns:    ; Bit Representation:    0bGEFPDCBA
    .db 0b01101111, 0b00000110, 0b11001011, 0b10001111, 0b10100110, 0b10101101, 0b11101101, 0b00000111  ; 0-7
    .db 0b11101111, 0b10101111, 0b11100111, 0b11101100, 0b01101001, 0b11001110, 0b11101001, 0b11100001  ; 8-F

;-------------------------------------------------
; Display Subroutine
;-------------------------------------------------
display:            push temp1
                    push temp2
                    in temp2, SREG
                    push temp2
                    ldi temp2, 8

shift_loop:         rol R16
                    brcs set_ser_1
                    cbi PORTB, 0
                    rjmp clk_pulse
                    
set_ser_1:          sbi PORTB, 0

clk_pulse:          sbi PORTB, 2
                    nop
                    cbi PORTB, 2
                    dec temp2
                    brne shift_loop
                    nop
                    sbi PORTB, 1
                    nop
                    cbi PORTB, 1
                    pop temp2
                    out SREG, temp2
                    pop temp2
                    pop temp1
                    ret

;-------------------------------------------------
; Delays (Using Timer0)
;-------------------------------------------------
delay_4s:           ldi XL, LOW(8000)                   ; 4s = 8000 * 500us
                    ldi XH, HIGH(8000)

delay_4s_loop:      rcall delay_TCO_500us
                    sbiw XH:XL, 1
                    brne delay_4s_loop
                    ret

delay_7s:           ldi XL, LOW(14000)                  ; 7s = 14000 * 500us
                    ldi XH, HIGH(14000)
                    
delay_7s_loop:      rcall delay_TCO_500us
                    sbiw XH:XL, 1
                    brne delay_7s_loop
                    ret

;-------------------------------------------------
; SRAM Buffers (reserve SRAM space)
;-------------------------------------------------
.dseg
.org SRAM_START
input_temp_storage: .byte 5
unlock_code: .byte 5
