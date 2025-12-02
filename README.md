# Librería I2C para 6502

## Descripción
Librería para comunicación I2C compatible con cc65. Utiliza el controlador I2C implementado en FPGA (OpenCores I2C Master) en modo **polling** (sin interrupciones).

## Archivos
- `i2c.h` - Definiciones y prototipos
- `i2c.c` - Implementación (polling)
- `i2c_vectors.s` - Vectores de interrupción (para compatibilidad)

## Configuración de Hardware
```c
#define I2C_BASE        0xC010  // Dirección base de registros I2C en FPGA
```

**Registros del controlador I2C:**
| Offset | Registro | Descripción |
|--------|----------|-------------|
| +0x00 | PRESCALE0 | Prescaler byte bajo |
| +0x01 | PRESCALE1 | Prescaler byte alto |
| +0x02 | CONTROL | Control (EN, IEN) |
| +0x03 | TX_RX | Transmisión/Recepción |
| +0x04 | CMD_STAT | Comando/Estado |

## Características
- ✅ Modo polling (sin interrupciones)
- ✅ Velocidad: 100 kHz @ 6.75MHz (PRESCALE=0x0D)
- ✅ API genérica independiente del dispositivo
- ✅ Soporte para direcciones de memoria de 1 o 2 bytes

## Funciones

### Inicialización
```c
void i2c_init(void);
```
Configura prescaler para 100kHz y habilita el controlador.

### Bajo Nivel
```c
uint8_t i2c_start(uint8_t device_addr, uint8_t rw);  // Retorna 1=ACK, 0=NACK
uint8_t i2c_write_byte(uint8_t data);                 // Retorna 1=ACK, 0=NACK
uint8_t i2c_read_byte(uint8_t ack);                   // ack=1 continuar, ack=0 último byte (NACK+STOP)
void i2c_stop(void);
```

### Alto Nivel
```c
uint8_t i2c_write(uint8_t device_addr, uint16_t mem_addr, 
                  uint8_t addr_bytes, const uint8_t* buffer, uint8_t count);

uint8_t i2c_read(uint8_t device_addr, uint16_t mem_addr, 
                 uint8_t addr_bytes, uint8_t* buffer, uint8_t count);
```

**Parámetros:**
- `device_addr`: Dirección I2C del dispositivo (7 bits, ej: 0x50)
- `mem_addr`: Dirección de memoria interna (8 o 16 bits)
- `addr_bytes`: 1 o 2 (tamaño de dirección)
- `buffer`: Buffer de datos
- `count`: Número de bytes

**Retorno:** Número de bytes transferidos (0 = error)

## Ejemplo de Uso

```c
#include "i2c.h"

void main(void) {
    uint8_t data;
    
    // Inicializar
    i2c_init();
    
    // Verificar dispositivo en 0x50
    if (i2c_start(0x50, I2C_WRITE)) {
        i2c_stop();
        // Dispositivo presente
    }
    
    // Leer byte de EEPROM (dirección 16 bits)
    if (i2c_start(0x50, I2C_WRITE)) {
        i2c_write_byte(0x00);  // Addr high
        i2c_write_byte(0x00);  // Addr low
        i2c_start(0x50, I2C_READ);  // Repeated start
        data = i2c_read_byte(0);    // Último byte (NACK+STOP automático)
    }
    
    // O usar funciones de alto nivel
    uint8_t buffer[4];
    i2c_read(0x50, 0x0000, 2, buffer, 4);  // Leer 4 bytes desde addr 0
}
```

## Notas Importantes

1. **Llamar `i2c_init()` antes de usar cualquier función**
2. `i2c_read_byte(0)` envía NACK+STOP automáticamente (último byte de lectura)
3. `i2c_read_byte(1)` envía ACK (para continuar leyendo más bytes)
4. La librería usa polling, espera activa hasta completar cada operación

## Compilación

### Compilar la librería

```bash
# Compilar i2c.c a objeto
cl65 -t none -O --cpu 65c02 -c i2c.c -o i2c.o

# O usando ca65 desde ensamblador pre-compilado
ca65 --cpu 65c02 i2c.s -o i2c.o
```

### Integración en Makefile

```makefile
# Directorios
LIBS_DIR = libs
I2C_DIR = $(LIBS_DIR)/i2c

# Archivo objeto
I2C_OBJ = $(I2C_DIR)/i2c.o

# Flags del compilador
CC = cl65
CFLAGS = -t none -O --cpu 65c02

# Regla para compilar i2c
$(I2C_DIR)/i2c.o: $(I2C_DIR)/i2c.c $(I2C_DIR)/i2c.h
	$(CC) $(CFLAGS) -c $< -o $@

# Linkear con tu programa
mi_programa.bin: main.o $(I2C_OBJ) vectors.o
	ld65 -C config/fpga.cfg -o $@ $^
```

### Estructura de proyecto recomendada

```
mi_proyecto/
├── libs/
│   └── i2c/
│       ├── i2c.c
│       └── i2c.h
├── src/
│   └── main.c
├── config/
│   └── fpga.cfg
└── makefile
```

### Include en tu código

```c
// Desde src/main.c
#include "../libs/i2c/i2c.h"
```

## Compatibilidad

- ✅ cc65 compiler
- ✅ C89 estándar
- ✅ Controlador OpenCores I2C Master
- ✅ 6502/65C02
