/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    Lab02_01_CG2271_Term_Assignment.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
// NXP SDK I2C master driver (ensure SDK component is enabled in your project)
#include "fsl_i2c.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

// --- TiltGuard: Minimal I2C + MPU-6050 bring-up ---
// Wiring (recommendation - confirm with your FRDM-MCXC444 pinout/pin_mux):
//   - MPU-6050 (GY-521): VCC=3.3V, GND=GND, SCL=I2C_SCL, SDA=I2C_SDA, AD0=GND (address 0x68)
//   - Choose the FRDM I2C instance brought out on the Arduino headers (commonly I2C0 on PTB2/PTB3 or similar).
//   - Make sure pin_mux/peripherals configure these pins for I2C before calling init.

#define MPU6050_ADDR      (0x68U)   // AD0=GND

#define REG_SMPLRT_DIV    (0x19U)
#define REG_CONFIG        (0x1AU)
#define REG_GYRO_CONFIG   (0x1BU)
#define REG_ACCEL_CONFIG  (0x1CU)
#define REG_PWR_MGMT_1    (0x6BU)
#define REG_WHO_AM_I      (0x75U)

// Select the I2C instance to use. If your project uses a different instance
// (e.g., I2C1), change these two macros accordingly.
#ifndef TILTGUARD_I2C_BASE
#define TILTGUARD_I2C_BASE I2C0
#endif

// Helper to get the source clock for I2C. Adjust for your board if needed.
static uint32_t TG_GetI2CClockHz(void)
{
    // Many FRDM boards route I2C clock from BusClk; if your BSP provides
    // a dedicated getter (e.g., CLOCK_GetFreq(kCLOCK_I2c0)), use that instead.
#ifdef FSL_CLOCK_DRIVER_VERSION
    return CLOCK_GetFreq(kCLOCK_BusClk);
#else
    return CLOCK_GetFreq(kCLOCK_BusClk);
#endif
}

static status_t TG_I2C_ReadRegs(I2C_Type *base, uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    i2c_master_transfer_t xfer = {0};
    xfer.slaveAddress   = addr;
    xfer.direction      = kI2C_Read;
    xfer.subaddress     = reg;
    xfer.subaddressSize = 1;
    xfer.data           = data;
    xfer.dataSize       = (uint32_t)len;
    xfer.flags          = kI2C_TransferDefaultFlag;
    return I2C_MasterTransferBlocking(base, &xfer);
}

static status_t TG_I2C_WriteReg(I2C_Type *base, uint8_t addr, uint8_t reg, uint8_t val)
{
    i2c_master_transfer_t xfer = {0};
    xfer.slaveAddress   = addr;
    xfer.direction      = kI2C_Write;
    xfer.subaddress     = reg;
    xfer.subaddressSize = 1;
    xfer.data           = &val;
    xfer.dataSize       = 1U;
    xfer.flags          = kI2C_TransferDefaultFlag;
    return I2C_MasterTransferBlocking(base, &xfer);
}

static status_t TG_MPU6050_Init(I2C_Type *base)
{
    // Wake up device
    status_t st = TG_I2C_WriteReg(base, MPU6050_ADDR, REG_PWR_MGMT_1, 0x00U);
    if (st != kStatus_Success) return st;

    // Optional basic configuration (sample rate, filters). Keep defaults for bring-up.
    // TG_I2C_WriteReg(base, MPU6050_ADDR, REG_SMPLRT_DIV, 0x07);
    // TG_I2C_WriteReg(base, MPU6050_ADDR, REG_CONFIG, 0x03);
    // TG_I2C_WriteReg(base, MPU6050_ADDR, REG_GYRO_CONFIG, 0x00);   // ±250 dps
    // TG_I2C_WriteReg(base, MPU6050_ADDR, REG_ACCEL_CONFIG, 0x00);  // ±2 g

    return kStatus_Success;
}

static status_t TG_MPU6050_WhoAmI(I2C_Type *base, uint8_t *who)
{
    return TG_I2C_ReadRegs(base, MPU6050_ADDR, REG_WHO_AM_I, who, 1U);
}

static void TG_I2C_MasterInit(I2C_Type *base)
{
    i2c_master_config_t cfg;
    I2C_MasterGetDefaultConfig(&cfg);
    cfg.baudRate_Bps = 100000U; // 100 kHz for bring-up

    // If your SDK requires explicit clock enable for the I2C instance, ensure
    // it is enabled in peripherals.c or add it here (device-specific macro).
    // Example (device dependent): CLOCK_EnableClock(kCLOCK_I2c0);

    I2C_MasterInit(base, &cfg, TG_GetI2CClockHz());
}

/*
 * @brief   Application entry point.
 */
int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    PRINTF("TiltGuard: IMU bring-up\r\n");

    // Initialize I2C master for MPU-6050
    TG_I2C_MasterInit(TILTGUARD_I2C_BASE);

    // Initialize MPU-6050 (wake from sleep)
    status_t st = TG_MPU6050_Init(TILTGUARD_I2C_BASE);
    if (st != kStatus_Success) {
        PRINTF("MPU6050 init failed: %d\r\n", (int)st);
    } else {
        // Read WHO_AM_I register; expect 0x68
        uint8_t who = 0x00;
        st = TG_MPU6050_WhoAmI(TILTGUARD_I2C_BASE, &who);
        if (st == kStatus_Success) {
            PRINTF("WHO_AM_I=0x%02X (expect 0x68)\r\n", who);
        } else {
            PRINTF("WHO_AM_I read failed: %d\r\n", (int)st);
        }
    }

    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++;
        SDK_DelayAtLeastUs(1000000U, CLOCK_GetFreq(kCLOCK_CoreSysClk)); // 1s delay
    }
    return 0 ;
}
