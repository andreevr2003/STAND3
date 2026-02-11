#include "max31865.h"
#include <math.h>

/* Callendar-Van Dusen coefficients for PT100/PT1000 */
#define CVD_A   3.9083e-3f
#define CVD_B  -5.775e-7f

/* SPI timeout */
#define MAX31865_SPI_TIMEOUT  100u

/* ---- Low-level SPI helpers ---- */

static void cs_low(max31865_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void cs_high(max31865_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static uint8_t spi_read_reg(max31865_t *dev, uint8_t reg)
{
    uint8_t tx[2] = { reg & 0x7F, 0xFF };
    uint8_t rx[2] = { 0, 0 };

    cs_low(dev);
    HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2, MAX31865_SPI_TIMEOUT);
    cs_high(dev);

    return rx[1];
}

static void spi_write_reg(max31865_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { reg | 0x80, value };

    cs_low(dev);
    HAL_SPI_Transmit(dev->hspi, tx, 2, MAX31865_SPI_TIMEOUT);
    cs_high(dev);
}

/* ---- Public API ---- */

bool max31865_init(max31865_t *dev)
{
    if (dev == NULL || dev->hspi == NULL) return false;

    /* Ensure CS is high (deselected) */
    cs_high(dev);
    HAL_Delay(10);

    /* Build config register value:
     *   BIAS on, auto conversion, fault-clear, 50 Hz filter */
    uint8_t cfg = MAX31865_CFG_BIAS
                | MAX31865_CFG_MODEAUTO
                | MAX31865_CFG_FAULT_CLR
                | MAX31865_CFG_FILTER_50HZ;

    if (dev->wire == MAX31865_WIRE_3) {
        cfg |= MAX31865_CFG_3WIRE;
    }

    spi_write_reg(dev, MAX31865_REG_CONFIG, cfg);
    HAL_Delay(10);

    /* Verify config was written */
    uint8_t readback = spi_read_reg(dev, MAX31865_REG_CONFIG);
    /* Mask out fault-clear bit (self-clearing) */
    if ((readback & 0xFD) != (cfg & 0xFD)) {
        return false;
    }

    return true;
}

uint16_t max31865_read_rtd_raw(max31865_t *dev)
{
    uint8_t msb = spi_read_reg(dev, MAX31865_REG_RTD_MSB);
    uint8_t lsb = spi_read_reg(dev, MAX31865_REG_RTD_LSB);

    uint16_t rtd = ((uint16_t)msb << 8) | lsb;

    /* Check fault bit (LSB) */
    if (rtd & 0x01) {
        /* Fault detected - clear it */
        max31865_clear_fault(dev);
        return 0;
    }

    /* Remove fault bit, RTD value is bits [15:1] */
    rtd >>= 1;

    return rtd;
}

float max31865_read_temperature(max31865_t *dev)
{
    uint16_t rtd_raw = max31865_read_rtd_raw(dev);
    if (rtd_raw == 0) return -999.0f;  /* error indicator */

    /* Calculate RTD resistance:
     * R_rtd = (rtd_raw / 32768) * R_ref */
    float r_rtd = (float)rtd_raw * dev->ref_resistor / 32768.0f;

    /* Callendar-Van Dusen equation (simplified for T > 0 C):
     * T = (-A + sqrt(A^2 - 4*B*(1 - R/R0))) / (2*B)
     *
     * For T < 0 C a more complex formula is needed but
     * for Peltier heating applications T > 0 is typical. */
    float z1 = -CVD_A;
    float z2 = CVD_A * CVD_A - 4.0f * CVD_B * (1.0f - r_rtd / dev->rtd_nominal);
    float z3 = 2.0f * CVD_B;

    if (z2 < 0.0f) return -999.0f;  /* math error */

    float temp = (z1 + sqrtf(z2)) / z3;

    return temp;
}

uint8_t max31865_read_fault(max31865_t *dev)
{
    return spi_read_reg(dev, MAX31865_REG_FAULT);
}

void max31865_clear_fault(max31865_t *dev)
{
    uint8_t cfg = spi_read_reg(dev, MAX31865_REG_CONFIG);
    cfg |= MAX31865_CFG_FAULT_CLR;
    spi_write_reg(dev, MAX31865_REG_CONFIG, cfg);
}
