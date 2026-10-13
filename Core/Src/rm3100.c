/**
 * @file    rm3100.c
 * @brief   PNI RM3100 driver — I2C implementation for STM32U5 HAL
 */

#include "rm3100.h"

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

static RM3100_Status rm3100_write(RM3100_HandleTypeDef *dev,
                                   uint8_t reg,
                                   const uint8_t *data,
                                   uint16_t len)
{
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(
        dev->hi2c,
        dev->i2cAddr,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t *)data,
        len,
        RM3100_HAL_TIMEOUT
    );
    return (ret == HAL_OK) ? RM3100_OK : RM3100_ERROR;
}

static RM3100_Status rm3100_read(RM3100_HandleTypeDef *dev,
                                  uint8_t reg,
                                  uint8_t *data,
                                  uint16_t len)
{
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
        dev->hi2c,
        dev->i2cAddr,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        len,
        RM3100_HAL_TIMEOUT
    );
    return (ret == HAL_OK) ? RM3100_OK : RM3100_ERROR;
}

/**
 * Convert three raw bytes (MSB first) to a sign-extended int32_t.
 * The RM3100 result is 24-bit 2's complement; bit 23 is the sign bit.
 */
static int32_t sign_extend24(uint8_t b2, uint8_t b1, uint8_t b0)
{
    int32_t val = ((int32_t)b2 << 16) | ((int32_t)b1 << 8) | b0;
    if (val & 0x00800000)
        val |= 0xFF000000;
    return val;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

RM3100_Status RM3100_Init(RM3100_HandleTypeDef *dev)
{
    HAL_Delay(10);   /* allow sensor to settle after power-up */

    uint8_t revid = 0;
    if (RM3100_ReadRevID(dev, &revid) != RM3100_OK)
        return RM3100_ERROR;

    /* A reading of 0x00 or 0xFF almost always means no device on the bus */
    if (revid == 0x00 || revid == 0xFF)
        return RM3100_ERROR;

    return RM3100_OK;
}

RM3100_Status RM3100_ReadRevID(RM3100_HandleTypeDef *dev, uint8_t *revid)
{
    return rm3100_read(dev, RM3100_REG_REVID, revid, 1);
}

RM3100_Status RM3100_SetCycleCount(RM3100_HandleTypeDef *dev,
                                    uint16_t cycleCount)
{
    /*
     * Write 6 bytes starting at CCX (0x04).
     * Layout: [X_MSB][X_LSB][Y_MSB][Y_LSB][Z_MSB][Z_LSB]
     * Registers auto-increment so one transaction covers all three axes.
     */
    uint8_t msb = (uint8_t)(cycleCount >> 8);
    uint8_t lsb = (uint8_t)(cycleCount & 0xFF);
    uint8_t buf[6] = { msb, lsb, msb, lsb, msb, lsb };

    return rm3100_write(dev, RM3100_REG_CCX, buf, 6);
}

bool RM3100_DataReady(RM3100_HandleTypeDef *dev)
{
    /* Hardware DRDY pin is faster — use it if wired */
    if (dev->drdyPort != NULL)
        return HAL_GPIO_ReadPin(dev->drdyPort, dev->drdyPin) == GPIO_PIN_SET;

    /* Fallback: check bit 7 of STATUS register */
    uint8_t status = 0;
    if (rm3100_read(dev, RM3100_REG_STATUS, &status, 1) != RM3100_OK)
        return false;
    return (status & RM3100_STATUS_DRDY) != 0;
}

RM3100_Status RM3100_ReadData(RM3100_HandleTypeDef *dev, RM3100_Data *out)
{
    /*
     * Read 9 bytes from MX (0x24) in one burst.
     * [MX2][MX1][MX0][MY2][MY1][MY0][MZ2][MZ1][MZ0]
     * Reading these registers automatically clears DRDY.
     */
    uint8_t buf[9];
    RM3100_Status ret = rm3100_read(dev, RM3100_REG_MX, buf, 9);
    if (ret != RM3100_OK)
        return ret;

    out->x = sign_extend24(buf[0], buf[1], buf[2]);
    out->y = sign_extend24(buf[3], buf[4], buf[5]);
    out->z = sign_extend24(buf[6], buf[7], buf[8]);

    return RM3100_OK;
}

RM3100_Status RM3100_SingleMeasurement(RM3100_HandleTypeDef *dev,
                                        RM3100_Data *out)
{
    /* Write 0x70 to POLL: measure X, Y, and Z */
    uint8_t poll = RM3100_POLL_ALL;
    RM3100_Status ret = rm3100_write(dev, RM3100_REG_POLL, &poll, 1);
    if (ret != RM3100_OK)
        return ret;

    /*
     * At CC=200 the 3-axis conversion takes ~7 ms.
     * Poll with a 50 ms timeout to be safe.
     */
    uint32_t t0 = HAL_GetTick();
    while (!RM3100_DataReady(dev)) {
        if ((HAL_GetTick() - t0) > 50)
            return RM3100_TIMEOUT;
        HAL_Delay(1);
    }

    return RM3100_ReadData(dev, out);
}

RM3100_Status RM3100_StartCMM(RM3100_HandleTypeDef *dev, uint8_t tmrc)
{
    /* Set update rate first (writing TMRC while CMM is running stops CMM) */
    RM3100_Status ret = rm3100_write(dev, RM3100_REG_TMRC, &tmrc, 1);
    if (ret != RM3100_OK)
        return ret;

    /* Enable CMM on all 3 axes; DRDY goes HIGH after all three complete */
    uint8_t cmm = RM3100_CMM_ALL_AXES;   /* 0x79 */
    return rm3100_write(dev, RM3100_REG_CMM, &cmm, 1);
}

RM3100_Status RM3100_StopCMM(RM3100_HandleTypeDef *dev)
{
    uint8_t cmm = 0x00;
    return rm3100_write(dev, RM3100_REG_CMM, &cmm, 1);
}

float RM3100_ToMicroTesla(int32_t raw, uint16_t cycleCount)
{
    /*
     * From Table 3-1: CC=200 -> gain=75 LSB/uT
     * Gain scales linearly with cycle count:
     *   gain = cycleCount * (75 / 200) = cycleCount * 0.375
     */
    float gain = (float)cycleCount * 0.375f;
    return (float)raw / gain;
}
