/*
 * Bosch BMI088 6-Axis IMU Driver
 * High-performance accelerometer + gyroscope
 */

#ifndef IMU_BMI088_H
#define IMU_BMI088_H

#include "../core/sensor_base.h"

#ifdef __cplusplus
extern "C" {
#endif

/* BMI088 has separate addresses for accel and gyro */
#define BMI088_ACCEL_I2C_ADDR      0x18
#define BMI088_ACCEL_I2C_ADDR_ALT  0x19
#define BMI088_GYRO_I2C_ADDR       0x68
#define BMI088_GYRO_I2C_ADDR_ALT   0x69

#define BMI088_ACCEL_CHIP_ID       0x1E
#define BMI088_GYRO_CHIP_ID        0x0F

/* Accelerometer ranges */
typedef enum {
    BMI088_ACCEL_RANGE_3G = 0,
    BMI088_ACCEL_RANGE_6G = 1,
    BMI088_ACCEL_RANGE_12G = 2,
    BMI088_ACCEL_RANGE_24G = 3
} bmi088_accel_range_t;

/* Gyroscope ranges */
typedef enum {
    BMI088_GYRO_RANGE_2000DPS = 0,
    BMI088_GYRO_RANGE_1000DPS = 1,
    BMI088_GYRO_RANGE_500DPS = 2,
    BMI088_GYRO_RANGE_250DPS = 3,
    BMI088_GYRO_RANGE_125DPS = 4
} bmi088_gyro_range_t;

/* Output data rates */
typedef enum {
    BMI088_ACCEL_ODR_12_5_HZ = 0x05,
    BMI088_ACCEL_ODR_25_HZ = 0x06,
    BMI088_ACCEL_ODR_50_HZ = 0x07,
    BMI088_ACCEL_ODR_100_HZ = 0x08,
    BMI088_ACCEL_ODR_200_HZ = 0x09,
    BMI088_ACCEL_ODR_400_HZ = 0x0A,
    BMI088_ACCEL_ODR_800_HZ = 0x0B,
    BMI088_ACCEL_ODR_1600_HZ = 0x0C
} bmi088_accel_odr_t;

typedef enum {
    BMI088_GYRO_ODR_2000_HZ_BW_532 = 0x00,
    BMI088_GYRO_ODR_2000_HZ_BW_230 = 0x01,
    BMI088_GYRO_ODR_1000_HZ_BW_116 = 0x02,
    BMI088_GYRO_ODR_400_HZ_BW_47 = 0x03,
    BMI088_GYRO_ODR_200_HZ_BW_23 = 0x04,
    BMI088_GYRO_ODR_100_HZ_BW_12 = 0x05,
    BMI088_GYRO_ODR_200_HZ_BW_64 = 0x06,
    BMI088_GYRO_ODR_100_HZ_BW_32 = 0x07
} bmi088_gyro_odr_t;

typedef struct {
    char i2c_bus[32];
    uint8_t accel_addr;
    uint8_t gyro_addr;
    bmi088_accel_range_t accel_range;
    bmi088_gyro_range_t gyro_range;
    bmi088_accel_odr_t accel_odr;
    bmi088_gyro_odr_t gyro_odr;
    bool enable_int1;
    bool enable_int2;
    int int1_gpio;
    int int2_gpio;
    bool enable_fifo;
    uint8_t fifo_wm;
} bmi088_config_t;

typedef struct {
    vector3_t accel;
    vector3_t gyro;
    float temperature;
    uint32_t timestamp_us;
    uint8_t status;
} bmi088_data_t;

/* BMI088 specific features */
typedef struct {
    uint8_t orientation;       /* Portrait/landscape detection */
    bool tap_detected;
    bool double_tap_detected;
    bool motion_detected;
    uint8_t motion_slope;
} bmi088_features_t;

/* BMI088 API */
sensor_base_t *bmi088_create(const char *name);
sensor_error_t bmi088_set_accel_range(sensor_base_t *sensor, bmi088_accel_range_t range);
sensor_error_t bmi088_set_gyro_range(sensor_base_t *sensor, bmi088_gyro_range_t range);
sensor_error_t bmi088_set_accel_odr(sensor_base_t *sensor, bmi088_accel_odr_t odr);
sensor_error_t bmi088_set_gyro_odr(sensor_base_t *sensor, bmi088_gyro_odr_t odr);
sensor_error_t bmi088_enable_fifo(sensor_base_t *sensor, bool accel, bool gyro);
sensor_error_t bmi088_read_fifo(sensor_base_t *sensor, bmi088_data_t *data, uint16_t max_samples);

/* Motion detection */
sensor_error_t bmi088_config_motion_int(sensor_base_t *sensor, uint8_t slope_th, uint8_t duration);
sensor_error_t bmi088_enable_motion_int(sensor_base_t *sensor, bool enable);
sensor_error_t bmi088_config_tap_int(sensor_base_t *sensor, uint8_t sensitivity);
sensor_error_t bmi088_enable_tap_int(sensor_base_t *sensor, bool enable);

/* Self test */
sensor_error_t bmi088_self_test_accel(sensor_base_t *sensor, bool *pass);
sensor_error_t bmi088_self_test_gyro(sensor_base_t *sensor, bool *pass);

/* Bias compensation */
sensor_error_t bmi088_set_accel_offset(sensor_base_t *sensor, vector3_t *offset);
sensor_error_t bmi088_set_gyro_offset(sensor_base_t *sensor, vector3_t *offset);
sensor_error_t bmi088_auto_calibrate_gyro(sensor_base_t *sensor);

#ifdef __cplusplus
}
#endif

#endif /* IMU_BMI088_H */
