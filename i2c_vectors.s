; Vectores de interrupción para I2C
; Compatible con CC65 para 6502

.import   _i2c_irq_handler

.segment "VECTORS"

; Vector NMI ($FFFA-$FFFB)
.addr nmi_handler

; Vector RESET ($FFFC-$FFFD) - Hardcoded a inicio de ROM
.addr $8000

; Vector IRQ/BRK ($FFFE-$FFFF)
.addr irq_handler

.segment "CODE"

; Manejador NMI (no usado)
nmi_handler:
    rti

; Manejador IRQ (llama al handler de I2C)
irq_handler:
    ; Preservar registros
    pha
    txa
    pha
    tya
    pha
    
    ; Llamar al handler I2C en C
    jsr _i2c_irq_handler
    
    ; Restaurar registros
    pla
    tay
    pla
    tax
    pla
    
    rti
