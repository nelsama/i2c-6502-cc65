/**
 * I2C.H - Librería I2C genérica para 6502 compatible con cc65
 * 
 * Librería genérica para comunicación I2C que puede ser usada
 * por cualquier aplicación (EEPROM, sensores, displays, etc.)
 */

#ifndef I2C_H
#define I2C_H

#include <stdint.h>

/* Registros I2C - Hardware FPGA I2C Controller */
#define I2C_BASE        0xC010

/* Registros del controlador I2C en FPGA */
#define I2C_PRESCALE0   (*(volatile uint8_t *)(I2C_BASE + 0x00))
#define I2C_PRESCALE1   (*(volatile uint8_t *)(I2C_BASE + 0x01))
#define I2C_CONTROL     (*(volatile uint8_t *)(I2C_BASE + 0x02))
#define I2C_TX_RX       (*(volatile uint8_t *)(I2C_BASE + 0x03))
#define I2C_CMD_STAT    (*(volatile uint8_t *)(I2C_BASE + 0x04))

/* Bits del registro CONTROL */
#define I2C_CTRL_EN         0x80
#define I2C_CTRL_IEN        0x40

/* Bits del registro STATUS */
#define I2C_STAT_RX_ACK     0x80
#define I2C_STAT_BUSY       0x40
#define I2C_STAT_AL         0x20
#define I2C_STAT_TIP        0x02
#define I2C_STAT_IF         0x01

/* Bits del registro COMMAND */
#define I2C_CMD_STA         0x80
#define I2C_CMD_STO         0x40
#define I2C_CMD_RD          0x20
#define I2C_CMD_WR          0x10
#define I2C_CMD_ACK         0x08
#define I2C_CMD_IACK        0x01

/* Direcciones de lectura/escritura */
#define I2C_WRITE           0x00
#define I2C_READ            0x01

/* Funciones basicas de I2C */
void i2c_init(void);
uint8_t i2c_start(uint8_t device_addr, uint8_t rw);
void i2c_stop(void);
uint8_t i2c_write_byte(uint8_t data);
uint8_t i2c_read_byte(uint8_t ack);

/* Manejador de interrupciones (llamado desde ASM) */
void i2c_irq_handler(void);

/* Funciones de alto nivel */
uint8_t i2c_write(uint8_t device_addr, uint16_t mem_addr, uint8_t addr_bytes, 
                  const uint8_t* buffer, uint8_t count);
uint8_t i2c_read(uint8_t device_addr, uint16_t mem_addr, uint8_t addr_bytes, 
                 uint8_t* buffer, uint8_t count);

#endif /* I2C_H */