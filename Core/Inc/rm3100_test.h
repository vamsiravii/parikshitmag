/**
 * @file    rm3100_test.h
 */

#ifndef RM3100_TEST_H
#define RM3100_TEST_H

#include "stm32u5xx_hal.h"
#include "rm3100.h"

/**
 * @brief  Run all three tests sequentially.
 * @param  hi2c     Your CubeMX I2C handle, e.g. &hi2c1
 * @param  i2cAddr  8-bit address, e.g. RM3100_I2C_ADDR_00 (0x20)
 */
void RM3100_Test_RunAll(I2C_HandleTypeDef *hi2c, uint8_t i2cAddr);

#endif /* RM3100_TEST_H */
