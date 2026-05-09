
/*
 * mpu.c
 *
 *  Created on: Jan 16, 2026
 *      Author: Loshaputraa
 */

#include "mpu.h"
#include <stdio.h>

/* ===== Private constants ===== */
#define SampleRate           2000
#define gyro_z_threshold     0.50f

/* ===== Private variables ===== */
static uint16_t mpu_addr = (0x68U << 1);   // will be updated during init

static uint8_t  data;
static float    CalibrationZ;
static int      RateCalibrationNumber;

static uint8_t  GyroZ_Buffer[2];
static int16_t  GyroZ;

static float    angleZ;
static float    angleZ_prev;

static uint32_t Time;
static uint32_t prevTime;
static float    dt;

/* Optional: expose a debug variable you can watch easily */
volatile float  angleZ_dbg;

/* ===== Functions ===== */

HAL_StatusTypeDef MPU6050_Init(void)
{
    HAL_StatusTypeDef ret;

    /* 1) Probe 0x68, then 0x69 */
    uint16_t try1 = (0x68U << 1);
    uint16_t try2 = (0x69U << 1);

    ret = HAL_I2C_IsDeviceReady(&hi2c2, try1, 1, 100);
    if (ret == HAL_OK) {
        mpu_addr = try1;
    } else {
        ret = HAL_I2C_IsDeviceReady(&hi2c2, try2, 1, 100);
        if (ret == HAL_OK) {
            mpu_addr = try2;
        } else {
            printf("MPU6050 not responding at 0x68 or 0x69\r\n");
            return ret;
        }
    }
    printf("MPU6050 found at 0x%02X\r\n", mpu_addr >> 1);

    /* 2) Configure */
    data = DATA_PWR_MGMT_1;  // wake up
    ret = HAL_I2C_Mem_Write(&hi2c2, mpu_addr, PWR_MGMT_1, 1, &data, 1, 100);
    if (ret != HAL_OK) return ret;

    data = DLPF_CFG;         // DLPF config
    ret = HAL_I2C_Mem_Write(&hi2c2, mpu_addr, DLPF, 1, &data, 1, 100);
    if (ret != HAL_OK) return ret;

    data = FS_SEL_1;         // ±500 dps
    ret = HAL_I2C_Mem_Write(&hi2c2, mpu_addr, GYRO_CONFIG, 1, &data, 1, 100);
    if (ret != HAL_OK) return ret;

    /* 3) Calibrate Z (sensor must be still) */
    printf("Calibrating... DO NOT MOVE THE MPU6050\r\n");
    CalibrationZ = 0.0f;

    for (RateCalibrationNumber = 0; RateCalibrationNumber < SampleRate; RateCalibrationNumber++) {
        ret = HAL_I2C_Mem_Read(&hi2c2, mpu_addr, GYRO_ZOUT, 1, GyroZ_Buffer, 2, 10);
        if (ret == HAL_OK) {
            GyroZ = (int16_t)((GyroZ_Buffer[0] << 8) | GyroZ_Buffer[1]);
            CalibrationZ += (float)GyroZ / 65.5f;   // deg/s for FS=±500 dps
        } else {
            printf("Read during calibration failed (iter %d)\r\n", RateCalibrationNumber);
        }
        HAL_Delay(1);
    }

    CalibrationZ /= SampleRate;
    angleZ = angleZ_prev = 0.0f;
    prevTime = HAL_GetTick();

    printf("Calibration Done! Z bias = %.3f deg/s\r\n", CalibrationZ);
    return HAL_OK;
}

float MPU6050_GYRO_READ(void)
{
    if (HAL_I2C_Mem_Read(&hi2c2, mpu_addr, GYRO_ZOUT, 1, GyroZ_Buffer, 2, 100) != HAL_OK) {
        return angleZ; // keep last value if read fails
    }

    GyroZ = (int16_t)((GyroZ_Buffer[0] << 8) | GyroZ_Buffer[1]);

    Time = HAL_GetTick();
    dt = (Time - prevTime) / 1000.0f;
    if (dt > 0.1f) dt = 0.005f;  // clamp if we had a long pause/breakpoint
    prevTime = Time;

    float z_rate_dps = ((float)GyroZ / 65.5f) - CalibrationZ;  // deg/s

    if (z_rate_dps < gyro_z_threshold && z_rate_dps > -gyro_z_threshold)
        z_rate_dps = 0.0f;

    angleZ = z_rate_dps * dt + angleZ_prev;  // integrate
    angleZ_prev = angleZ;

    angleZ_dbg = angleZ; // easy to watch in debugger
    return angleZ;
}

float MPU6050_Get_GyroZ_Rad(void)
{
    return angleZ * 0.0174532925f; // deg -> rad
}

/* Optional: expose degrees for convenience */
float MPU6050_Get_GyroZ_Deg(void)
{
    return angleZ;
}

