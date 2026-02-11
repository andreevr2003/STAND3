/**
 * MAX31865 RTD-to-Digital Converter Library
 * Based on NimaLTD's library (github.com/NimaLTD)
 * Adapted for STAND3 project - STM32F103RB
 *
 * Uses SOFTWARE SPI (GPIO bit-bang) to eliminate HAL SPI issues.
 */
#include "max31865.h"
#include <math.h>

/* ---- Register addresses ---- */
#define MAX31856_CONFIG_REG        0x00
#define MAX31856_RTDMSB_REG        0x01
#define MAX31856_RTDLSB_REG        0x02
#define MAX31856_HFAULTMSB_REG     0x03
#define MAX31856_HFAULTLSB_REG     0x04
#define MAX31856_LFAULTMSB_REG     0x05
#define MAX31856_LFAULTLSB_REG     0x06
#define MAX31856_FAULTSTAT_REG     0x07

/* ---- Config bits ---- */
#define MAX31856_CONFIG_BIAS       0x80
#define MAX31856_CONFIG_MODEAUTO   0x40
#define MAX31856_CONFIG_MODEOFF    0x00
#define MAX31856_CONFIG_1SHOT      0x20
#define MAX31856_CONFIG_3WIRE      0x10
#define MAX31856_CONFIG_24WIRE     0x00
#define MAX31856_CONFIG_FAULTSTAT  0x02
#define MAX31856_CONFIG_FILT50HZ   0x01
#define MAX31856_CONFIG_FILT60HZ   0x00

/* ---- CVD coefficients ---- */
#define RTD_A  3.9083e-3
#define RTD_B -5.775e-7

/* ================================================================
   SOFTWARE SPI  (GPIO bit-bang)
   Tested approach — same method as the 4ilo MAX31865 library.
   Eliminates all HAL SPI peripheral timing / buffer issues.

   STAND3 pinout:
     PB12  CS   (already GPIO output from gpio.c)
     PB13  SCK  (reconfigured from SPI2 AF → GPIO output)
     PB14  MISO (GPIO input)
     PB15  MOSI (reconfigured from SPI2 AF → GPIO output)
   ================================================================ */

#define SW_SCK_PORT    GPIOB
#define SW_SCK_PIN     GPIO_PIN_13
#define SW_MISO_PORT   GPIOB
#define SW_MISO_PIN    GPIO_PIN_14
#define SW_MOSI_PORT   GPIOB
#define SW_MOSI_PIN    GPIO_PIN_15

/* ~2 us busy-wait at 72 MHz — well beyond MAX31865 timing requirements */
static inline void sw_delay(void)
{
    volatile uint32_t n = 50;
    while (n--) ;
}

/*
 * Write one byte to the MAX31865.
 * Clock idles LOW after this function.
 */
static void sw_spi_write_byte(uint8_t data)
{
    for (int8_t i = 7; i >= 0; i--)
    {
        /* Set MOSI */
        HAL_GPIO_WritePin(SW_MOSI_PORT, SW_MOSI_PIN,
                          (data & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        sw_delay();
        /* Rising edge — MAX31865 latches MOSI */
        HAL_GPIO_WritePin(SW_SCK_PORT, SW_SCK_PIN, GPIO_PIN_SET);
        sw_delay();
        /* Falling edge */
        HAL_GPIO_WritePin(SW_SCK_PORT, SW_SCK_PIN, GPIO_PIN_RESET);
    }
}

/*
 * Read one byte from the MAX31865.
 * Data is read after the rising edge (data was set up on previous falling edge).
 * Clock idles LOW after this function.
 */
static uint8_t sw_spi_read_byte(void)
{
    uint8_t result = 0;
    for (int8_t i = 7; i >= 0; i--)
    {
        result <<= 1;
        /* Rising edge — data from previous falling edge is stable */
        HAL_GPIO_WritePin(SW_SCK_PORT, SW_SCK_PIN, GPIO_PIN_SET);
        sw_delay();
        /* Sample MISO */
        if (HAL_GPIO_ReadPin(SW_MISO_PORT, SW_MISO_PIN))
            result |= 1;
        sw_delay();
        /* Falling edge — MAX31865 shifts out next bit */
        HAL_GPIO_WritePin(SW_SCK_PORT, SW_SCK_PIN, GPIO_PIN_RESET);
        sw_delay();
    }
    return result;
}

/* ---- Register-level access ---- */

static void Max31865_readRegisterN(Max31865_t *max31865, uint8_t addr,
                                   uint8_t *buffer, uint8_t n)
{
    addr &= 0x7F;       /* Read: bit 7 = 0 */

    HAL_GPIO_WritePin(max31865->cs_gpio, max31865->cs_pin, GPIO_PIN_RESET);
    sw_delay();

    sw_spi_write_byte(addr);

    while (n--)
    {
        *buffer = sw_spi_read_byte();
        buffer++;
    }

    HAL_GPIO_WritePin(max31865->cs_gpio, max31865->cs_pin, GPIO_PIN_SET);
    sw_delay();
}

static uint8_t Max31865_readRegister8(Max31865_t *max31865, uint8_t addr)
{
    uint8_t ret = 0;
    Max31865_readRegisterN(max31865, addr, &ret, 1);
    return ret;
}

static uint16_t Max31865_readRegister16(Max31865_t *max31865, uint8_t addr)
{
    uint8_t buffer[2] = {0, 0};
    Max31865_readRegisterN(max31865, addr, buffer, 2);
    uint16_t ret = buffer[0];
    ret <<= 8;
    ret |= buffer[1];
    return ret;
}

static void Max31865_writeRegister8(Max31865_t *max31865, uint8_t addr, uint8_t data)
{
    HAL_GPIO_WritePin(max31865->cs_gpio, max31865->cs_pin, GPIO_PIN_RESET);
    sw_delay();

    sw_spi_write_byte(addr | 0x80);   /* Write: bit 7 = 1 */
    sw_spi_write_byte(data);

    HAL_GPIO_WritePin(max31865->cs_gpio, max31865->cs_pin, GPIO_PIN_SET);
    sw_delay();
}

/* ================================================================
   Internal helpers
   ================================================================ */

uint8_t Max31865_readFault(Max31865_t *max31865)
{
    return Max31865_readRegister8(max31865, MAX31856_FAULTSTAT_REG);
}

static void Max31865_clearFault(Max31865_t *max31865)
{
    uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
    t &= ~0x2C;
    t |= MAX31856_CONFIG_FAULTSTAT;
    Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
}

static void Max31865_enableBias(Max31865_t *max31865, uint8_t enable)
{
    uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
    if (enable)
        t |= MAX31856_CONFIG_BIAS;
    else
        t &= ~MAX31856_CONFIG_BIAS;
    Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
}

static void Max31865_autoConvert(Max31865_t *max31865, uint8_t enable)
{
    uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
    if (enable)
        t |= MAX31856_CONFIG_MODEAUTO;
    else
        t &= ~MAX31856_CONFIG_MODEAUTO;
    Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
}

static void Max31865_setWires(Max31865_t *max31865, uint8_t numWires)
{
    uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
    if (numWires == 3)
        t |= MAX31856_CONFIG_3WIRE;
    else
        t &= ~MAX31856_CONFIG_3WIRE;
    Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
}

static void Max31865_setFilter(Max31865_t *max31865, uint8_t filterHz)
{
    uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
    if (filterHz == 50)
        t |= MAX31856_CONFIG_FILT50HZ;
    else
        t &= ~MAX31856_CONFIG_FILT50HZ;
    Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
}

/* ================================================================
   Public API
   ================================================================ */

uint16_t Max31865_readRTD(Max31865_t *max31865)
{
    /* In auto-convert mode the RTD register is always fresh.
       Just read both bytes in ONE CS transaction. */
    uint8_t buf[2] = {0, 0};
    Max31865_readRegisterN(max31865, MAX31856_RTDMSB_REG, buf, 2);

    uint16_t rtd = ((uint16_t)buf[0] << 8) | buf[1];
    rtd >>= 1;   /* bit 0 is fault flag */
    max31865->last_rtd = rtd;
    return rtd;
}

void Max31865_init(Max31865_t *max31865, SPI_HandleTypeDef *spi,
                   GPIO_TypeDef *cs_gpio, uint16_t cs_pin,
                   uint8_t numwires, uint8_t filterHz)
{
    if (max31865->lock == 1)
        HAL_Delay(1);
    max31865->lock = 1;
    max31865->spi = spi;
    max31865->cs_gpio = cs_gpio;
    max31865->cs_pin = cs_pin;

    /* ---- Switch from hardware SPI to software (GPIO) SPI ---- */
    HAL_SPI_DeInit(spi);

    GPIO_InitTypeDef gi = {0};
    /* SCK + MOSI → GPIO output push-pull */
    gi.Pin   = SW_SCK_PIN | SW_MOSI_PIN;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SW_SCK_PORT, &gi);

    /* MISO → GPIO input no-pull */
    gi.Pin  = SW_MISO_PIN;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SW_MISO_PORT, &gi);

    /* Idle state */
    HAL_GPIO_WritePin(SW_SCK_PORT,  SW_SCK_PIN,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SW_MOSI_PORT, SW_MOSI_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(max31865->cs_gpio, max31865->cs_pin, GPIO_PIN_SET);
    HAL_Delay(100);

    /* Configure: wires + filter */
    Max31865_setWires(max31865, numwires);
    Max31865_setFilter(max31865, filterHz);
    Max31865_clearFault(max31865);

    /* Enable BIAS + AUTO-CONVERT — stays on permanently.
       This eliminates all 1-shot timing issues. Config value:
       0x80 (bias) | 0x40 (auto) | filter bit = 0xC0 | filter */
    {
        uint8_t cfg = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
        cfg |= MAX31856_CONFIG_BIAS | MAX31856_CONFIG_MODEAUTO;
        Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, cfg);
    }
    HAL_Delay(200);   /* let first conversion complete */

    max31865->lock = 0;
}

bool Max31865_readTempC(Max31865_t *max31865, float *readTemp)
{
    if (max31865->lock == 1)
        HAL_Delay(1);
    max31865->lock = 1;

    bool isOk = false;
    float Z1, Z2, Z3, Z4, Rt, temp;

    /* Read raw 15-bit ADC (auto-convert keeps this updated) */
    uint16_t raw = Max31865_readRTD(max31865);

    Rt = (float)raw;
    Rt /= 32768.0f;
    Rt *= MAX31865_RREF;

    Z1 = -RTD_A;
    Z2 = RTD_A * RTD_A - (4 * RTD_B);
    Z3 = (4 * RTD_B) / MAX31865_RNOMINAL;
    Z4 = 2 * RTD_B;

    temp = Z2 + (Z3 * Rt);
    temp = (sqrtf(temp) + Z1) / Z4;

    if (temp >= 0)
    {
        *readTemp = temp;
        if (Max31865_readFault(max31865) == 0)
            isOk = true;
        max31865->lock = 0;
        return isOk;
    }

    /* For T < 0 C, use polynomial approximation */
    Rt /= MAX31865_RNOMINAL;
    Rt *= 100;
    float rpoly = Rt;
    temp = -242.02;
    temp += 2.2228 * rpoly;
    rpoly *= Rt;
    temp += 2.5859e-3 * rpoly;
    rpoly *= Rt;
    temp -= 4.8260e-6 * rpoly;
    rpoly *= Rt;
    temp -= 2.8183e-8 * rpoly;
    rpoly *= Rt;
    temp += 1.5243e-10 * rpoly;

    *readTemp = temp;
    if (Max31865_readFault(max31865) == 0)
        isOk = true;
    max31865->lock = 0;
    return isOk;
}

bool Max31865_readTempF(Max31865_t *max31865, float *readTemp)
{
    bool isOk = Max31865_readTempC(max31865, readTemp);
    *readTemp = (*readTemp * 9.0f / 5.0f) + 32.0f;
    return isOk;
}

float Max31865_Filter(float newInput, float lastOutput, float effectiveFactor)
{
    return (lastOutput * (1.0f - effectiveFactor)) + (newInput * effectiveFactor);
}

void Max31865_readAllRegisters(Max31865_t *max31865, uint8_t regs[8])
{
    for (uint8_t i = 0; i < 8; i++)
        regs[i] = Max31865_readRegister8(max31865, i);
}
