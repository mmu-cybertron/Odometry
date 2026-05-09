
#ifndef INC_MPU_H_
#define INC_MPU_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* I2C handle comes from main.c or another module */
extern I2C_HandleTypeDef hi2c2;

/* MPU6050 I2C address (HAL uses 8-bit left‑shifted) */
#define MPU6050_I2C_ADDR     (0x68U << 1)

/* Registers */
#define PWR_MGMT_1           0x6B
#define DLPF                 0x1A
#define GYRO_CONFIG          0x1B
#define GYRO_ZOUT            0x47

/* Config values */
#define DATA_PWR_MGMT_1      0x00
#define DLPF_CFG             0x03
#define FS_SEL_1             0x08

/* Data structure */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} mpu6050_raw_t;

/* API Prototypes */
HAL_StatusTypeDef MPU6050_Init(void);
HAL_StatusTypeDef MPU6050_ReadWhoAmI(uint8_t *who);
HAL_StatusTypeDef MPU6050_ReadRaw(mpu6050_raw_t *out);
float MPU6050_GYRO_READ(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_MPU_H_ */
