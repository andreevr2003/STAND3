#ifndef MAX31865_H
#define MAX31865_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- MAX31865 Register addresses ---- */
#define MAX31865_REG_CONFIG       0x00
#define MAX31865_REG_RTD_MSB      0x01
#define MAX31865_REG_RTD_LSB      0x02
#define MAX31865_REG_HFT_MSB      0x03
#define MAX31865_REG_HFT_LSB      0x04
#define MAX31865_REG_LFT_MSB      0x05
#define MAX31865_REG_LFT_LSB      0x06
#define MAX31865_REG_FAULT        0x07

/* ---- Configuration bits ---- */
#define MAX31865_CFG_BIAS         0x80
#define MAX31865_CFG_MODEAUTO     0x40
#define MAX31865_CFG_1SHOT        0x20
#define MAX31865_CFG_3WIRE        0x10
#define MAX31865_CFG_FAULT_CLR    0x02
#define MAX31865_CFG_FILTER_50HZ  0x01

/* ---- RTD type ---- */
typedef enum {
    MAX31865_RTD_PT100 = 0,
    MAX31865_RTD_PT1000
} max31865_rtd_type_t;

/* ---- Wiring type ---- */
typedef enum {
    MAX31865_WIRE_2 = 0,
    MAX31865_WIRE_3,
    MAX31865_WIRE_4
} max31865_wire_t;

/* ---- Device handle ---- */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    max31865_rtd_type_t rtd_type;
    max31865_wire_t     wire;
    float              ref_resistor;   /* Reference resistor value (e.g. 430.0 for PT100) */
    float              rtd_nominal;    /* RTD nominal resistance at 0C (100.0 for PT100) */
} max31865_t;

/* ---- API ---- */
bool     max31865_init(max31865_t *dev);
uint16_t max31865_read_rtd_raw(max31865_t *dev);
float    max31865_read_temperature(max31865_t *dev);
uint8_t  max31865_read_fault(max31865_t *dev);
void     max31865_clear_fault(max31865_t *dev);

#endif /* MAX31865_H */
