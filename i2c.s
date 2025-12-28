;
; I2C.S - Librería I2C en ensamblador para 6502
;
; Versión optimizada en ASM - Compatible con el header i2c.h
; Polling mode (sin interrupciones)
;
; CONVENCIÓN CC65:
;   - 1 byte: A
;   - 2 bytes: A (low), X (high)  
;   - Más parámetros: stack (primeros params) + A/X (últimos)
;

.export _i2c_init
.export _i2c_start
.export _i2c_stop
.export _i2c_write_byte
.export _i2c_read_byte
.export _i2c_write
.export _i2c_read
.export _i2c_irq_handler

.importzp sp, ptr1, tmp1, tmp2, tmp3, tmp4

; ============================================
; Registros I2C - Hardware FPGA
; ============================================
I2C_BASE      = $C010
I2C_PRESCALE0 = I2C_BASE + $00
I2C_PRESCALE1 = I2C_BASE + $01
I2C_CONTROL   = I2C_BASE + $02
I2C_TX_RX     = I2C_BASE + $03
I2C_CMD_STAT  = I2C_BASE + $04

; Bits del registro CONTROL
I2C_CTRL_EN   = $80

; Bits del registro STATUS
I2C_STAT_RX_ACK = $80
I2C_STAT_TIP    = $02

; Bits del registro COMMAND
I2C_CMD_STA   = $80
I2C_CMD_STO   = $40
I2C_CMD_RD    = $20
I2C_CMD_WR    = $10
I2C_CMD_ACK   = $08

; ============================================
; Variables locales
; ============================================
.segment "BSS"
i2c_device:     .res 1
i2c_count:      .res 1
i2c_addr_lo:    .res 1
i2c_addr_hi:    .res 1
i2c_addr_bytes: .res 1
i2c_ptr_lo:     .res 1
i2c_ptr_hi:     .res 1

.segment "CODE"

; ============================================
; i2c_wait_tip - Esperar fin de transferencia
; Interno, no exportado
; Preserva: nada
; ============================================
i2c_wait_tip:
    ldy #$FF              ; Timeout counter high
@loop:
    ldx #$FF              ; Timeout counter low
@inner:
    lda I2C_CMD_STAT
    and #I2C_STAT_TIP
    beq @done             ; TIP=0, transferencia completa
    dex
    bne @inner
    dey
    bne @loop
@done:
    rts

; ============================================
; void i2c_init(void)
; Inicializar controlador I2C
; ============================================
_i2c_init:
    lda #$0D              ; Prescaler para 100kHz @ 6.75MHz
    sta I2C_PRESCALE0
    lda #$00
    sta I2C_PRESCALE1
    lda #I2C_CTRL_EN      ; Habilitar I2C (sin interrupciones)
    sta I2C_CONTROL
    rts

; ============================================
; uint8_t i2c_start(uint8_t device_addr, uint8_t rw)
; CC65: A = rw (último param), stack[0] = device_addr
; Retorna: A = 1 (ACK), A = 0 (NACK)
; ============================================
_i2c_start:
    sta tmp1              ; Guardar rw
    
    ; Obtener device_addr del stack
    ldy #0
    lda (sp),y            ; device_addr
    
    ; Calcular addr_byte = (device_addr << 1) | rw
    asl a                 ; device_addr << 1
    ora tmp1              ; | rw
    sta I2C_TX_RX
    
    lda #(I2C_CMD_STA | I2C_CMD_WR)
    sta I2C_CMD_STAT
    jsr i2c_wait_tip
    
    ; Limpiar stack (1 byte)
    inc sp
    bne @no_wrap
    inc sp+1
@no_wrap:
    
    ; Verificar ACK
    lda I2C_CMD_STAT
    and #I2C_STAT_RX_ACK
    bne @nack
    lda #$01              ; ACK recibido
    rts
@nack:
    lda #$00              ; NACK recibido
    rts

; ============================================
; void i2c_stop(void)
; ============================================
_i2c_stop:
    lda #I2C_CMD_STO
    sta I2C_CMD_STAT
    rts

; ============================================
; uint8_t i2c_write_byte(uint8_t data)
; CC65: A = data
; Retorna: A = 1 (ACK), A = 0 (NACK)
; ============================================
_i2c_write_byte:
    sta I2C_TX_RX
    lda #I2C_CMD_WR
    sta I2C_CMD_STAT
    jsr i2c_wait_tip
    ; Verificar ACK
    lda I2C_CMD_STAT
    and #I2C_STAT_RX_ACK
    bne @nack
    lda #$01              ; ACK
    rts
@nack:
    lda #$00              ; NACK
    rts

; ============================================
; uint8_t i2c_read_byte(uint8_t ack)
; CC65: A = ack (1=ACK continuar, 0=NACK último byte)
; Retorna: A = byte leído
; ============================================
_i2c_read_byte:
    cmp #$00
    beq @last_byte
    ; Byte intermedio - enviar ACK
    lda #I2C_CMD_RD
    jmp @do_read
@last_byte:
    ; Último byte - enviar NACK + STOP
    lda #(I2C_CMD_RD | I2C_CMD_ACK | I2C_CMD_STO)
@do_read:
    sta I2C_CMD_STAT
    jsr i2c_wait_tip
    lda I2C_TX_RX         ; Leer byte recibido
    rts

; ============================================
; uint8_t i2c_write(uint8_t device_addr, uint16_t mem_addr, 
;                   uint8_t addr_bytes, const uint8_t* buffer, uint8_t count)
;
; CC65 stack layout (7 bytes total):
;   A = count (último parámetro)
;   sp+0,sp+1 = buffer (ptr, 2 bytes)
;   sp+2 = addr_bytes
;   sp+3,sp+4 = mem_addr (16-bit, little endian)
;   sp+5 = device_addr
;
; Retorna: A = bytes escritos
; ============================================
_i2c_write:
    sta i2c_count         ; Guardar count
    
    ; Verificar count > 0
    cmp #$00
    bne @count_ok
    jmp @cleanup_zero
@count_ok:
    
    ; Obtener parámetros del stack
    ldy #0
    lda (sp),y            ; buffer_lo
    sta i2c_ptr_lo
    iny
    lda (sp),y            ; buffer_hi
    sta i2c_ptr_hi
    iny
    lda (sp),y            ; addr_bytes
    sta i2c_addr_bytes
    iny
    lda (sp),y            ; mem_addr_lo
    sta i2c_addr_lo
    iny
    lda (sp),y            ; mem_addr_hi
    sta i2c_addr_hi
    iny
    lda (sp),y            ; device_addr
    sta i2c_device
    
    ; i2c_start(device_addr, I2C_WRITE=0)
    ; Empujar device_addr al stack para cc65
    lda i2c_device
    jsr pushax_a          ; Push A al stack
    lda #$00              ; rw = WRITE
    jsr _i2c_start
    cmp #$00
    bne @start_ok
    jsr _i2c_stop
    jmp @cleanup_zero
@start_ok:

    ; Enviar dirección de memoria
    lda i2c_addr_bytes
    cmp #2
    bne @one_byte_addr
    
    ; 2 bytes de dirección - enviar high primero
    lda i2c_addr_hi
    jsr _i2c_write_byte
    cmp #$00
    bne @addr_hi_ok
    jsr _i2c_stop
    jmp @cleanup_zero
@addr_hi_ok:
    
@one_byte_addr:
    lda i2c_addr_lo
    jsr _i2c_write_byte
    cmp #$00
    bne @addr_lo_ok
    jsr _i2c_stop
    jmp @cleanup_zero
@addr_lo_ok:

    ; Escribir datos
    ldy #$00
@write_loop:
    cpy i2c_count
    beq @write_done
    
    ; Leer byte del buffer
    sty tmp3              ; Guardar índice
    lda i2c_ptr_lo
    sta ptr1
    lda i2c_ptr_hi
    sta ptr1+1
    lda (ptr1),y
    
    jsr _i2c_write_byte
    ldy tmp3              ; Restaurar índice
    cmp #$00
    bne @byte_ok
    ; Error - retornar bytes escritos
    jsr _i2c_stop
    tya                   ; A = bytes escritos
    jmp @cleanup_ret
@byte_ok:
    iny
    jmp @write_loop
    
@write_done:
    jsr _i2c_stop
    lda i2c_count         ; Retornar count
    jmp @cleanup_ret

@cleanup_zero:
    lda #$00
@cleanup_ret:
    ; Limpiar stack (6 bytes de parámetros)
    pha                   ; Guardar resultado
    lda sp
    clc
    adc #6
    sta sp
    bcc @no_carry
    inc sp+1
@no_carry:
    pla                   ; Restaurar resultado
    rts

; ============================================
; uint8_t i2c_read(uint8_t device_addr, uint16_t mem_addr,
;                  uint8_t addr_bytes, uint8_t* buffer, uint8_t count)
;
; Misma estructura de stack que i2c_write
; Retorna: A = bytes leídos
; ============================================
_i2c_read:
    sta i2c_count
    
    cmp #$00
    bne @count_ok_r
    jmp @cleanup_zero_r
@count_ok_r:
    
    ; Obtener parámetros del stack
    ldy #0
    lda (sp),y
    sta i2c_ptr_lo
    iny
    lda (sp),y
    sta i2c_ptr_hi
    iny
    lda (sp),y
    sta i2c_addr_bytes
    iny
    lda (sp),y
    sta i2c_addr_lo
    iny
    lda (sp),y
    sta i2c_addr_hi
    iny
    lda (sp),y
    sta i2c_device
    
    ; Fase de escritura - enviar dirección
    lda i2c_device
    jsr pushax_a
    lda #$00              ; WRITE
    jsr _i2c_start
    cmp #$00
    bne @start1_ok
    jsr _i2c_stop
    jmp @cleanup_zero_r
@start1_ok:

    ; Enviar dirección de memoria
    lda i2c_addr_bytes
    cmp #2
    bne @one_byte_r
    
    lda i2c_addr_hi
    jsr _i2c_write_byte
    cmp #$00
    bne @addr_hi_r_ok
    jsr _i2c_stop
    jmp @cleanup_zero_r
@addr_hi_r_ok:
    
@one_byte_r:
    lda i2c_addr_lo
    jsr _i2c_write_byte
    cmp #$00
    bne @addr_lo_r_ok
    jsr _i2c_stop
    jmp @cleanup_zero_r
@addr_lo_r_ok:

    ; Repeated start para lectura
    lda i2c_device
    jsr pushax_a
    lda #$01              ; READ
    jsr _i2c_start
    cmp #$00
    bne @start2_ok
    jsr _i2c_stop
    jmp @cleanup_zero_r
@start2_ok:

    ; Leer datos
    ldy #$00
@read_loop:
    cpy i2c_count
    beq @read_done
    
    ; Determinar si es último byte
    sty tmp3
    tya
    clc
    adc #1
    cmp i2c_count
    beq @is_last
    lda #$01              ; ACK (más bytes)
    jmp @do_read_byte
@is_last:
    lda #$00              ; NACK (último byte)
@do_read_byte:
    jsr _i2c_read_byte
    sta tmp4              ; Guardar byte leído
    
    ; Guardar en buffer
    lda i2c_ptr_lo
    sta ptr1
    lda i2c_ptr_hi
    sta ptr1+1
    ldy tmp3
    lda tmp4
    sta (ptr1),y
    iny
    jmp @read_loop
    
@read_done:
    lda i2c_count
    jmp @cleanup_ret_r

@cleanup_zero_r:
    lda #$00
@cleanup_ret_r:
    pha
    lda sp
    clc
    adc #6
    sta sp
    bcc @no_carry_r
    inc sp+1
@no_carry_r:
    pla
    rts

; ============================================
; pushax_a - Push A to cc65 software stack
; Helper para preparar parámetros
; ============================================
pushax_a:
    pha                   ; Guardar valor
    lda sp
    sec
    sbc #1
    sta sp
    bcs @no_borrow
    dec sp+1
@no_borrow:
    pla                   ; Recuperar valor
    ldy #0
    sta (sp),y            ; Guardar en stack
    rts

; ============================================
; void i2c_irq_handler(void)
; Handler vacío - no se usa en modo polling
; ============================================
_i2c_irq_handler:
    rts
