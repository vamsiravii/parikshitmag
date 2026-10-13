/**
 * @file    rm3100_example.c
 * @brief   RM3100 I2C usage examples for STM32U5
 *
 * Paste the relevant snippet into main.c after MX_I2Cx_Init() is called.
 *
 * CubeMX I2C settings:
 *   Mode:            I2C
 *   Speed:           Standard (100 kHz) or Fast (400 kHz)
 *   Address bits:    7-bit
 *   No DMA required for basic usage.
 *
 * Wiring:
 *   STM32 SDA   ->  RM3100 pin 1  (SDA)        — with 4.7k pull-up to VDD
 *   STM32 SCL   ->  RM3100 pin 27 (SCL)        — with 4.7k pull-up to VDD
 *   STM32 GPIO  ->  RM3100 pin 23 (DRDY)       — optional input
 *   RM3100 pin 22 (I2CEN) pulled HIGH           — REQUIRED for I2C mode
 *   RM3100 pin  3 (SA1)   to GND or VDD        — address bit 1
 *   RM3100 pin 28 (SA0)   to GND or VDD        — address bit 0
 *   RM3100 pins 2, 20 (RES) pulled to GND
 */

#include "rm3100.h"
#include <stdio.h>    /* printf via SWO or UART redirect */

extern I2C_HandleTypeDef hi2c1;   /* match your CubeMX handle name */

/* Adjust to match your GPIO assignment for the DRDY pin.
 * If DRDY is not wired, set drdyPort = NULL in the handle below.        */
#define DRDY_PORT   GPIOB
#define DRDY_PIN    GPIO_PIN_0

/* =========================================================================
 * Shared handle — initialise once, reuse everywhere
 * ========================================================================= */
static RM3100_HandleTypeDef rm = {
    .hi2c      = &hi2c1,
    .i2cAddr   = RM3100_I2C_ADDR_00,   /* SA1=GND, SA0=GND -> 0x20      */
    .drdyPort  = DRDY_PORT,            /* set NULL to poll STATUS reg     */
    .drdyPin   = DRDY_PIN,
};

/* =========================================================================
 * Example 1 — Single polled measurement
 *
 * Simplest way to get a reading. The call blocks until data is ready
 * (typically ~7 ms at CC=200) then returns all three axes.
 * ========================================================================= */
void example_single_shot(void)
{
    if (RM3100_Init(&rm) != RM3100_OK) {
        /* Likely causes: wrong address, I2CEN pin not HIGH, bad wiring   */
        Error_Handler();
    }

    RM3100_SetCycleCount(&rm, RM3100_CC_200);   /* optional, 200 is default */

    RM3100_Data data;
    if (RM3100_SingleMeasurement(&rm, &data) == RM3100_OK) {
        float x = RM3100_ToMicroTesla(data.x, RM3100_CC_200);
        float y = RM3100_ToMicroTesla(data.y, RM3100_CC_200);
        float z = RM3100_ToMicroTesla(data.z, RM3100_CC_200);
        printf("X=%6.2f uT  Y=%6.2f uT  Z=%6.2f uT\r\n", x, y, z);
    }
}

/* =========================================================================
 * Example 2 — Continuous measurement mode (polling DRDY in main loop)
 *
 * Best for applications that need a steady stream of readings.
 * The sensor fires automatically at the chosen rate; your loop just waits
 * for DRDY and reads whenever convenient.
 * ========================================================================= */
void example_continuous(void)
{
    if (RM3100_Init(&rm) != RM3100_OK)
        Error_Handler();

    RM3100_SetCycleCount(&rm, RM3100_CC_200);

    /* Start at ~37 Hz. Replace with any RM3100_TMRC_xxx constant.        */
    if (RM3100_StartCMM(&rm, RM3100_TMRC_37HZ) != RM3100_OK)
        Error_Handler();

    RM3100_Data data;

    while (1) {
        /* Wait for the sensor to finish a conversion */
        while (!RM3100_DataReady(&rm))
            HAL_Delay(1);

        if (RM3100_ReadData(&rm, &data) == RM3100_OK) {
            float x = RM3100_ToMicroTesla(data.x, RM3100_CC_200);
            float y = RM3100_ToMicroTesla(data.y, RM3100_CC_200);
            float z = RM3100_ToMicroTesla(data.z, RM3100_CC_200);
            printf("X=%6.2f  Y=%6.2f  Z=%6.2f uT\r\n", x, y, z);
        }
    }
    /* To stop: RM3100_StopCMM(&rm); */
}

/* =========================================================================
 * Example 3 — Interrupt-driven continuous mode (most efficient)
 *
 * Wire DRDY to an STM32 EXTI pin configured for rising-edge trigger.
 * In CubeMX: GPIO -> EXTI with "Rising edge" and NVIC enabled.
 *
 * This avoids busy-waiting and is the right approach if the MCU has other
 * work to do between samples.
 * ========================================================================= */

volatile bool drdy_flag = false;

/* In stm32u5xx_it.c, inside EXTIx_IRQHandler -> HAL_GPIO_EXTI_IRQHandler,
 * the HAL will call this callback automatically:                           */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == DRDY_PIN)
        drdy_flag = true;
}

void example_interrupt_driven(void)
{
    if (RM3100_Init(&rm) != RM3100_OK)
        Error_Handler();

    RM3100_SetCycleCount(&rm, RM3100_CC_200);
    RM3100_StartCMM(&rm, RM3100_TMRC_37HZ);

    RM3100_Data data;

    while (1) {
        if (drdy_flag) {
            drdy_flag = false;

            if (RM3100_ReadData(&rm, &data) == RM3100_OK) {
                float x = RM3100_ToMicroTesla(data.x, RM3100_CC_200);
                float y = RM3100_ToMicroTesla(data.y, RM3100_CC_200);
                float z = RM3100_ToMicroTesla(data.z, RM3100_CC_200);
                printf("X=%6.2f  Y=%6.2f  Z=%6.2f uT\r\n", x, y, z);
            }
        }

        /* MCU is free to do other work here between readings */
        HAL_Delay(1);
    }
}
