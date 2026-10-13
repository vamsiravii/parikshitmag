/**
 * @file    rm3100_test.c
 * @brief   RM3100 test routines — output via STM32CubeIDE Live Expressions
 *
 * HOW TO USE LIVE EXPRESSIONS:
 *   1. Open STM32CubeIDE and start a debug session.
 *   2. Go to Window -> Show View -> Live Expressions.
 *   3. Click "Add new expression" and add each variable below:
 *        test_status
 *        single.x_raw      single.y_raw      single.z_raw
 *        single.x_ut       single.y_ut       single.z_ut
 *        cont.x_raw        cont.y_raw        cont.z_raw
 *        cont.x_ut         cont.y_ut         cont.z_ut
 *        cont.sample_count
 *        cmp.raw_value     cmp.ut_value      cmp.cycle_count
 *   4. Set Live Expressions to refresh automatically (click the clock icon).
 *   5. Run RM3100_Test_RunAll() from main() after MX_I2C1_Init().
 */

#include "rm3100_test.h"
#include "rm3100.h"

/* =========================================================================
 * All test state is in volatile globals so Live Expressions can see them
 * while the MCU is running — no breakpoint needed.
 * ========================================================================= */

/* Overall test result — watch this first */
volatile char test_status[32] = "NOT STARTED";

/* --- Test 1: Single-shot ------------------------------------------------ */
typedef struct {
    volatile int32_t x_raw;
    volatile int32_t y_raw;
    volatile int32_t z_raw;
    volatile float   x_ut;
    volatile float   y_ut;
    volatile float   z_ut;
    volatile char    result[16];
} SingleShotResult;

volatile SingleShotResult single = {0};

/* --- Test 2: Continuous mode -------------------------------------------- */
typedef struct {
    volatile int32_t x_raw;
    volatile int32_t y_raw;
    volatile int32_t z_raw;
    volatile float   x_ut;
    volatile float   y_ut;
    volatile float   z_ut;
    volatile uint32_t sample_count;   /* watch this increment in real-time */
    volatile char    result[16];
} ContinuousResult;

volatile ContinuousResult cont = {0};

/* --- Test 3: Raw vs microtesla comparison ------------------------------- */
typedef struct {
    volatile int32_t raw_value;    /* raw count from sensor          */
    volatile float   ut_value;     /* same reading in microtesla     */
    volatile uint16_t cycle_count; /* which CC was used              */
    volatile float   expected_gain;/* cycleCount * 0.375             */
} ComparisonResult;

volatile ComparisonResult cmp = {0};

/* =========================================================================
 * Internal driver handle — DRDY not wired, driver polls STATUS register
 * ========================================================================= */
static RM3100_HandleTypeDef rm;

/* =========================================================================
 * Helper: write a short string into a volatile char array
 * (can't use strcpy directly on volatile)
 * ========================================================================= */
static void set_str(volatile char *dst, const char *src, uint8_t maxlen)
{
    uint8_t i = 0;
    while (src[i] && i < (maxlen - 1)) {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
}

/* =========================================================================
 * Test 1 — Single-shot read
 *
 * Triggers one measurement, reads back X/Y/Z as both raw counts and µT.
 * A healthy Earth-field reading should be roughly 25–65 µT on at least
 * one axis depending on your orientation.
 * ========================================================================= */
static void test_single_shot(void)
{
    set_str(test_status, "SINGLE SHOT...", sizeof(test_status));

    RM3100_Data data;
    RM3100_Status ret = RM3100_SingleMeasurement(&rm, &data);

    if (ret == RM3100_OK) {
        single.x_raw = data.x;
        single.y_raw = data.y;
        single.z_raw = data.z;
        single.x_ut  = RM3100_ToMicroTesla(data.x, RM3100_CC_200);
        single.y_ut  = RM3100_ToMicroTesla(data.y, RM3100_CC_200);
        single.z_ut  = RM3100_ToMicroTesla(data.z, RM3100_CC_200);
        set_str(single.result, "PASS", sizeof(single.result));
    } else if (ret == RM3100_TIMEOUT) {
        set_str(single.result, "TIMEOUT", sizeof(single.result));
    } else {
        set_str(single.result, "FAIL", sizeof(single.result));
    }
}

/* =========================================================================
 * Test 2 — Continuous mode stream (50 samples)
 *
 * Starts CMM at ~37 Hz and reads 50 samples.  Watch cont.sample_count
 * increment in Live Expressions — it should tick up roughly once every
 * 27ms.  x_raw/y_raw/z_raw and x_ut/y_ut/z_ut update each sample.
 * ========================================================================= */
static void test_continuous(void)
{
    set_str(test_status, "CONTINUOUS...", sizeof(test_status));
    cont.sample_count = 0;

    if (RM3100_StartCMM(&rm, RM3100_TMRC_37HZ) != RM3100_OK) {
        set_str(cont.result, "START FAIL", sizeof(cont.result));
        return;
    }

    for (uint32_t i = 0; i < 50; i++) {
        /* Poll STATUS register until DRDY (no DRDY pin wired) */
        uint32_t t0 = HAL_GetTick();
        while (!RM3100_DataReady(&rm)) {
            if ((HAL_GetTick() - t0) > 200) {
                set_str(cont.result, "TIMEOUT", sizeof(cont.result));
                RM3100_StopCMM(&rm);
                return;
            }
            HAL_Delay(1);
        }

        RM3100_Data data;
        if (RM3100_ReadData(&rm, &data) == RM3100_OK) {
            cont.x_raw = data.x;
            cont.y_raw = data.y;
            cont.z_raw = data.z;
            cont.x_ut  = RM3100_ToMicroTesla(data.x, RM3100_CC_200);
            cont.y_ut  = RM3100_ToMicroTesla(data.y, RM3100_CC_200);
            cont.z_ut  = RM3100_ToMicroTesla(data.z, RM3100_CC_200);
            cont.sample_count++;
        }
    }

    RM3100_StopCMM(&rm);
    set_str(cont.result, "PASS", sizeof(cont.result));
}

/* =========================================================================
 * Test 3 — Raw vs microtesla comparison
 *
 * Takes one reading and shows you the raw count alongside the converted
 * µT value and the gain factor used in the conversion, so you can verify
 * the maths manually:
 *
 *   ut_value = raw_value / expected_gain
 *
 * Expected gain at CC=200:  200 * 0.375 = 75 LSB/µT
 * ========================================================================= */
static void test_raw_vs_ut(void)
{
    set_str(test_status, "RAW VS UT...", sizeof(test_status));

    RM3100_Data data;
    if (RM3100_SingleMeasurement(&rm, &data) != RM3100_OK) {
        return;
    }

    /* Use X axis for the comparison (change to data.y or data.z if needed) */
    cmp.raw_value    = data.x;
    cmp.cycle_count  = RM3100_CC_200;
    cmp.expected_gain = (float)RM3100_CC_200 * 0.375f;   /* = 75.0 */
    cmp.ut_value     = RM3100_ToMicroTesla(data.x, RM3100_CC_200);

    /* Verify: raw_value / expected_gain should equal ut_value exactly */
}

/* =========================================================================
 * Public entry point — call this from main() after MX_I2C1_Init()
 * ========================================================================= */
void RM3100_Test_RunAll(I2C_HandleTypeDef *hi2c, uint8_t i2cAddr)
{
    /* Set up the handle — no DRDY pin, poll STATUS register instead */
    rm.hi2c      = hi2c;
    rm.i2cAddr   = i2cAddr;
    rm.drdyPort  = NULL;   /* not wired — DataReady() will poll STATUS reg */
    rm.drdyPin   = 0;

    /* Init — if this fails, check I2CEN pin (must be HIGH), address, wiring */
    if (RM3100_Init(&rm) != RM3100_OK) {
        set_str(test_status, "INIT FAILED", sizeof(test_status));
        return;
    }

    RM3100_SetCycleCount(&rm, RM3100_CC_200);

    test_single_shot();
    HAL_Delay(100);

    test_raw_vs_ut();
    HAL_Delay(100);

    test_continuous();     /* takes ~1.4 s for 50 samples at 37 Hz */

    set_str(test_status, "ALL DONE", sizeof(test_status));
}
