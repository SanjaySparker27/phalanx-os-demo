/*
 * TDK InvenSense ICM-20948 9-Axis IMU Driver
 * Combined accelerometer, gyroscope, magnetometer
 */

#ifndef IMU_ICM20948_H
#define IMU_ICM20948_H

#include "../core/sensor_base.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ICM20948_I2C_ADDR      0x68
#define ICM20948_I2C_ADDR_ALT  0x69
#define ICM20948_WHOAMI_VAL    0xEA

/* Accelerometer ranges */
typedef enum {
    ICM20948_ACCEL_RANGE_2G = 0,
    ICM20948_ACCEL_RANGE_4G = 1,
    ICM20948_ACCEL_RANGE_8G = 2,
    ICM20948_ACCEL_RANGE_16G = 3
} icm20948_accel_range_t;

/* Gyroscope ranges */
typedef enum {
    ICM20948_GYRO_RANGE_250DPS = 0,
    ICM20948_GYRO_RANGE_500DPS = 1,
    ICM20948_GYRO_RANGE_1000DPS = 2,
    ICM20948_GYRO_RANGE_2000DPS = 3
} icm20948_gyro_range_t;

/* Output data rates */
typedef enum {
    ICM20948_ODR_12_5_HZ = 0,
    ICM20948_ODR_25_HZ = 1,
    ICM20948_ODR_50_HZ = 2,
    ICM20948_ODR_100_HZ = 3,
    ICM20948_ODR_200_HZ = 4,
    ICM20948_ODR_400_HZ = 5,
    ICM20948_ODR_800_HZ = 6,
    ICM20948_ODR_1600_HZ = 7
} icm20948_odr_t;

typedef struct {
    char i2c_bus[32];
    uint8_t i2c_addr;
    icm20948_accel_range_t accel_range;
    icm20948_gyro_range_t gyro_range;
    icm20948_odr_t accel_odr;
    icm20948_odr_t gyro_odr;
    bool enable_mag;           /* Enable AK09916 magnetometer */
    bool enable_dmp;           /* Enable Digital Motion Processor */
    bool enable_fifo;
    uint16_t fifo_wm;          /* FIFO watermark level */
    int interrupt_gpio;        /* DRDY interrupt GPIO (-1 if polling) */
} icm20948_config_t;

typedef struct {
    imu_data_t raw;
    quaternion_t orientation;  /* DMP output if enabled */
    float temperature;
    uint16_t sample_rate;
    uint32_t timestamp_us;
} icm20948_data_t;

/* DMP features */
typedef enum {
    ICM20948_DMP_FEATURE_6X_QUAT = 0x001,
    ICM20948_DMP_FEATURE_GYRO_CAL = 0x002,
    ICM20948_DMP_FEATURE_SEND_RAW_ACCEL = 0x004,
    ICM20948_DMP_FEATURE_SEND_RAW_GYRO = 0x008,
    ICM20948_DMP_FEATURE_SEND_CAL_GYRO = 0x010
} icm20948_dmp_features_t;

/* ICM-20948 API */
sensor_base_t *icm20948_create(const char *name);
sensor_error_t icm20948_set_accel_range(sensor_base_t *sensor, icm20948_accel_range_t range);
sensor_error_t icm20948_set_gyro_range(sensor_base_t *sensor, icm20948_gyro_range_t range);
sensor_error_t icm20948_set_odr(sensor_base_t *sensor, icm20948_odr_t accel_odr, icm20948_odr_t gyro_odr);
sensor_error_t icm20948_enable_dmp(sensor_base_t *sensor, bool enable);
sensor_error_t icm20948_set_dmp_features(sensor_base_t *sensor, uint16_t features);
sensor_error_t icm20948_read_fifo(sensor_base_t *sensor, icm20948_data_t *data, uint16_t max_samples, uint16_t *num_read);
sensor_error_t icm20948_self_test(sensor_base_t *sensor, bool *accel_pass, bool *gyro_pass);
sensor_error_t icm20948_calibrate(sensor_base_t *sensor, uint16_t num_samples);

/* Bias compensation */
sensor_error_t icm20948_set_accel_bias(sensor_base_t *sensor, vector3_t *bias);
sensor_error_t icm20948_set_gyro_bias(sensor_base_t *sensor, vector3_t *bias);
sensor_error_t icm20948_get_accel_bias(sensor_base_t *sensor, vector3_t *bias);
sensor_error_t icm20948_get_gyro_bias(sensor_base_t *sensor, vector3_t *bias);

#ifdef __cplusplus
}
#endif

#endif /* IMU_ICM20948_H */
