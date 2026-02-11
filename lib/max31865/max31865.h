/**
 * MAX31865 RTD-to-Digital Converter Library
 * Based on NimaLTD's library (github.com/NimaLTD)
 * Adapted for STAND3 project - STM32F103RB
 */
#ifndef _MAX31865_H
#define _MAX31865_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ---- Configuration ---- */
#define MAX31865_RREF      430.0f    /* Reference resistor on board */
#define MAX31865_RNOMINAL  100.0f    /* PT100 = 100 ohm at 0 C */

/* ---- Device handle ---- */
typedef struct
{
    GPIO_TypeDef      *cs_gpio;
    uint16_t           cs_pin;
    SPI_HandleTypeDef *spi;
    uint8_t            lock;
    uint16_t           last_rtd;   /* raw 15-bit ADC from last readTempC */
} Max31865_t;

/* ---- Public API ---- */
void  Max31865_init(Max31865_t *max31865, SPI_HandleTypeDef *spi,
                    GPIO_TypeDef *cs_gpio, uint16_t cs_pin,
                    uint8_t numwires, uint8_t filterHz);
bool  Max31865_readTempC(Max31865_t *max31865, float *readTemp);
bool  Max31865_readTempF(Max31865_t *max31865, float *readTemp);
float Max31865_Filter(float newInput, float lastOutput, float effectiveFactor);
uint8_t Max31865_readFault(Max31865_t *max31865);
uint16_t Max31865_readRTD(Max31865_t *max31865);
void Max31865_readAllRegisters(Max31865_t *max31865, uint8_t regs[8]);

#ifdef __cplusplus
}
#endif

#endif /* _MAX31865_H */
