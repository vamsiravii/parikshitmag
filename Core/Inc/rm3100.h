/**
 * @file    rm3100.h
 * @brief   PNI RM3100 Geomagnetic Sensor Driver for STM32U5 HAL — I2C only
 */

#ifndef RM3100_H
#define RM3100_H

#include "stm32u5xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* -----------------------------------------------------------------------
 * Register addresses
 * ----------------------------------------------------------------------- */
#define RM3100_REG_POLL     0x00
#define RM3100_REG_CMM      0x01
#define RM3100_REG_CCX      0x04
#define RM3100_REG_TMRC     0x0B
#define RM3100_REG_MX       0x24
#define RM3100_REG_BIST     0x33
#define RM3100_REG_STATUS   0x34
#define RM3100_REG_HSHAKE   0x35
#define RM3100_REG_REVID    0x36

/* -----------------------------------------------------------------------
 * CMM register bits
 * ----------------------------------------------------------------------- */
#define RM3100_CMM_START        (1 << 0)
#define RM3100_CMM_DRDM_ALL     (0 << 1)  /* DRDY after all requested axes */
#define RM3100_CMM_DRDM_ANY     (1 << 1)  /* DRDY after any single axis    */
#define RM3100_CMM_CMX          (1 << 4)
#define RM3100_CMM_CMY          (1 << 5)
#define RM3100_CMM_CMZ          (1 << 6)
#define RM3100_CMM_ALL_AXES     (RM3100_CMM_CMX | RM3100_CMM_CMY | \
                                 RM3100_CMM_CMZ | RM3100_CMM_DRDM_ALL | \
                                 RM3100_CMM_START)   /* = 0x79 */

/* -----------------------------------------------------------------------
 * POLL register bits
 * ----------------------------------------------------------------------- */
#define RM3100_POLL_PMX     (1 << 4)
#define RM3100_POLL_PMY     (1 << 5)
#define RM3100_POLL_PMZ     (1 << 6)
#define RM3100_POLL_ALL     (RM3100_POLL_PMX | RM3100_POLL_PMY | \
                             RM3100_POLL_PMZ)        /* = 0x70 */

/* -----------------------------------------------------------------------
 * STATUS register
 * ----------------------------------------------------------------------- */
#define RM3100_STATUS_DRDY  (1 << 7)

/* -----------------------------------------------------------------------
 * TMRC update rates
 * Default 0x96 -> ~37 Hz
 * ----------------------------------------------------------------------- */
#define RM3100_TMRC_600HZ   0x92
#define RM3100_TMRC_300HZ   0x93
#define RM3100_TMRC_150HZ   0x94
#define RM3100_TMRC_75HZ    0x95
#define RM3100_TMRC_37HZ    0x96
#define RM3100_TMRC_18HZ    0x97
#define RM3100_TMRC_9HZ     0x98
#define RM3100_TMRC_4_5HZ   0x99
#define RM3100_TMRC_2_3HZ   0x9A

/* -----------------------------------------------------------------------
 * Cycle count presets (practical range: 30-400)
 * ----------------------------------------------------------------------- */
#define RM3100_CC_50        50
#define RM3100_CC_100       100
#define RM3100_CC_200       200    /* default */

/* -----------------------------------------------------------------------
 * I2C slave address (8-bit HAL format)
 *
 * Base address bits [6:2] = 0b01000 (hardware-fixed by PNI).
 * SA1 = MagI2C pin 28,  SA0 = MagI2C pin 3.
 *
 *   SA1  SA0   8-bit addr
 *    0    0      0x20  <- most common, both address pins to GND
 *    0    1      0x22
 *    1    0      0x24
 *    1    1      0x26
 *
 * Also: pin 22 (I2CEN) must be pulled HIGH to enable I2C mode.
 * ----------------------------------------------------------------------- */
#define RM3100_I2C_ADDR_00  0x20
#define RM3100_I2C_ADDR_01  0x22
#define RM3100_I2C_ADDR_10  0x24
#define RM3100_I2C_ADDR_11  0x26

#define RM3100_HAL_TIMEOUT  10     /* ms */

/* -----------------------------------------------------------------------
 * Driver handle — fill this in before calling any API function
 * ----------------------------------------------------------------------- */
typedef struct {
    I2C_HandleTypeDef  *hi2c;
    uint8_t             i2cAddr;    /* use one of RM3100_I2C_ADDR_xx above */

    /* DRDY pin — optional but recommended.
     * Set drdyPort = NULL to fall back to polling the STATUS register.    */
    GPIO_TypeDef       *drdyPort;
    uint16_t            drdyPin;
} RM3100_HandleTypeDef;

/* -----------------------------------------------------------------------
 * Measurement result
 * Each axis is a 24-bit 2's complement value sign-extended into int32_t.
 * ----------------------------------------------------------------------- */
typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} RM3100_Data;

/* -----------------------------------------------------------------------
 * Return codes
 * ----------------------------------------------------------------------- */
typedef enum {
    RM3100_OK      =  0,
    RM3100_ERROR   = -1,
    RM3100_TIMEOUT = -2,
} RM3100_Status;

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

/** Verify comms by reading REVID. Call once after HAL_I2C_Init(). */
RM3100_Status RM3100_Init(RM3100_HandleTypeDef *dev);

/** Write the same cycle count to all three axes (default is 200). */
RM3100_Status RM3100_SetCycleCount(RM3100_HandleTypeDef *dev,
                                    uint16_t cycleCount);

/** Trigger one measurement on all axes, block until done, return results. */
RM3100_Status RM3100_SingleMeasurement(RM3100_HandleTypeDef *dev,
                                        RM3100_Data *out);

/** Start continuous measurement mode at the given TMRC rate. */
RM3100_Status RM3100_StartCMM(RM3100_HandleTypeDef *dev, uint8_t tmrc);

/** Stop continuous measurement mode. */
RM3100_Status RM3100_StopCMM(RM3100_HandleTypeDef *dev);

/** Returns true when a fresh measurement is available. */
bool RM3100_DataReady(RM3100_HandleTypeDef *dev);

/** Read the 9 result bytes from MX/MY/MZ. Call after DataReady() == true. */
RM3100_Status RM3100_ReadData(RM3100_HandleTypeDef *dev, RM3100_Data *out);

/**
 * Convert a raw axis count to microtesla.
 * Pass the same cycleCount you used when taking the measurement.
 */
float RM3100_ToMicroTesla(int32_t raw, uint16_t cycleCount);

/** Read REVID — handy sanity check during bring-up. */
RM3100_Status RM3100_ReadRevID(RM3100_HandleTypeDef *dev, uint8_t *revid);

#endif /* RM3100_H */
