/**
 * I2C_POLLING.C - Versión con polling (sin interrupciones)
 * 
 * IMPORTANTE: Llamar i2c_init() antes de usar cualquier función
 */

#include "i2c.h"

/* Esperar usando el bit TIP (Transfer In Progress) */
static void i2c_wait_tip(void) {
    uint16_t timeout = 10000;
    while ((I2C_CMD_STAT & I2C_STAT_TIP) && timeout > 0) {
        timeout--;
    }
}

void i2c_init(void) {
    /* Configurar prescaler para 100 kHz con 6.75MHz */
    I2C_PRESCALE0 = 0x0D;
    I2C_PRESCALE1 = 0x00;
    
    /* Habilitar controlador I2C SIN interrupciones (polling mode) */
    I2C_CONTROL = I2C_CTRL_EN;  /* Solo EN, no IEN */
}

uint8_t i2c_start(uint8_t device_addr, uint8_t rw) {
    uint8_t addr_byte;
    
    addr_byte = (device_addr << 1) | (rw & 0x01);
    I2C_TX_RX = addr_byte;
    I2C_CMD_STAT = I2C_CMD_STA | I2C_CMD_WR;
    
    /* Esperar usando polling */
    i2c_wait_tip();
    
    /* Verificar ACK: bit 7 = 0 significa ACK */
    if (I2C_CMD_STAT & I2C_STAT_RX_ACK) {
        return 0;  /* NACK recibido */
    }
    
    return 1;  /* ACK recibido */
}

uint8_t i2c_write_byte(uint8_t data) {
    I2C_TX_RX = data;
    I2C_CMD_STAT = I2C_CMD_WR;
    
    i2c_wait_tip();
    
    if (I2C_CMD_STAT & I2C_STAT_RX_ACK) {
        return 0;
    }
    
    return 1;
}

uint8_t i2c_read_byte(uint8_t ack) {
    uint8_t data;
    
    /* ack=1 significa enviar ACK (continuar), ack=0 significa NACK (último byte)
     * En el controlador: bit ACK=0 envía ACK, bit ACK=1 envía NACK
     * Para el último byte (ack=0), enviamos NACK + STOP juntos */
    if (ack) {
        I2C_CMD_STAT = I2C_CMD_RD;  /* ACK - sin bit ACK */
    } else {
        I2C_CMD_STAT = I2C_CMD_RD | I2C_CMD_ACK | I2C_CMD_STO;  /* NACK + STOP */
    }
    
    i2c_wait_tip();
    data = I2C_TX_RX;
    
    return data;
}

void i2c_stop(void) {
    
    I2C_CMD_STAT = I2C_CMD_STO;
}

uint8_t i2c_write(uint8_t device_addr, uint16_t mem_addr, uint8_t addr_bytes, 
                  const uint8_t* buffer, uint8_t count) {
    uint8_t i;
    
    if (buffer == 0 || count == 0) {
        return 0;
    }
    
    if (!i2c_start(device_addr, I2C_WRITE)) {
        i2c_stop();
        return 0;
    }
    
    if (addr_bytes == 2) {
        if (!i2c_write_byte((uint8_t)(mem_addr >> 8))) {
            i2c_stop();
            return 0;
        }
        if (!i2c_write_byte((uint8_t)(mem_addr & 0xFF))) {
            i2c_stop();
            return 0;
        }
    } else {
        if (!i2c_write_byte((uint8_t)(mem_addr & 0xFF))) {
            i2c_stop();
            return 0;
        }
    }
    
    for (i = 0; i < count; i++) {
        if (!i2c_write_byte(buffer[i])) {
            i2c_stop();
            return i;
        }
    }
    
    i2c_stop();
    return count;
}

uint8_t i2c_read(uint8_t device_addr, uint16_t mem_addr, uint8_t addr_bytes, 
                 uint8_t* buffer, uint8_t count) {
    uint8_t i;
    
    if (buffer == 0 || count == 0) {
        return 0;
    }
    
    if (!i2c_start(device_addr, I2C_WRITE)) {
        i2c_stop();
        return 0;
    }
    
    if (addr_bytes == 2) {
        if (!i2c_write_byte((uint8_t)(mem_addr >> 8))) {
            i2c_stop();
            return 0;
        }
        if (!i2c_write_byte((uint8_t)(mem_addr & 0xFF))) {
            i2c_stop();
            return 0;
        }
    } else {
        if (!i2c_write_byte((uint8_t)(mem_addr & 0xFF))) {
            i2c_stop();
            return 0;
        }
    }
    
    if (!i2c_start(device_addr, I2C_READ)) {
        i2c_stop();
        return 0;
    }
    
    for (i = 0; i < count; i++) {
        if (i < (count - 1)) {
            buffer[i] = i2c_read_byte(1);
        } else {
            buffer[i] = i2c_read_byte(0);
        }
    }
    
    i2c_stop();
    return count;
}

/* Handler vacío - no se usa en modo polling */
void i2c_irq_handler(void) {
    /* No hace nada en modo polling */
}
