/*
 * STMicroelectronics LIS3MDL Magnetometer
 * Ultra-low-power high-performance 3-axis magnetic sensor
 */

#ifndef MAG_LIS3MDL_H
#define MAG_LIS3MDL_H

#include "../core/sensor_base.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LIS3MDL_I2C_ADDR        0x1C
#define LIS3MDL_I2C_ADDR_ALT    0x1E
#define LIS3MDL_CHIP_ID         0x3D

typedef enum {
    LIS3MDL_RANGE_4GAUSS = 0,
    LIS3MDL_RANGE_8GAUSS = 1,
    LIS3MDL_RANGE_12GAUSS = 2,
    LIS3MDL_RANGE_16GAUSS = 3
} lis3mdl_range_t;

typedef enum {
    LIS3MDL_ODR_0_625_HZ = 0,
    LIS3MDL_ODR_1_25_HZ = 1,
    LIS3MDL_ODR_2_5_HZ = 2,
    LIS3MDL_ODR_5_HZ = 3,
    LIS3MDL_ODR_10_HZ = 4,
    LIS3MDL_ODR_20_HZ = 5,
    LIS3MDL_ODR_40_HZ = 6,
    LIS3MDL_ODR_80_HZ = 7
} lis3mdl_odr_t;

typedef enum {
    LIS3MDL_MODE_LOW = 0,
    LIS3MDL_MODE_MEDIUM = 1,
    LIS3MDL_MODE_HIGH = 2,
    LIS3MDL_MODE_ULTRA_HIGH = 3
} lis3mdl_perf_mode_t;

typedef struct {
    char i2c_bus[32];
    uint8_t i2c_addr;
    lis3mdl_range_t range;
    lis3mdl_odr_t odr;
    lis3mdl_perf_mode_t xy_mode;
    lis3mdl_perf_mode_t z_mode;
    bool fast_odr;
    bool bdu;               /* Block data update */
    bool temp_en;
} lis3mdl_config_t;

typedef struct {
    float mag_x;            /* Gauss */
    float mag_y;
    float mag_z;
    float temperature;      /* Celsius */
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
    sensor_timestamp_t timestamp;
} lis3mdl_data_t;

/* Interrupt configuration */
typedef struct {
    bool int_en;
    bool iea;               /* Interrupt active high/low */
    bool iel;               /* Latched/pulsed */
    bool xien, yien, zien;
    float threshold;        /* Gauss */
} lis3mdl_int_cfg_t;

/* LIS3MDL API */
sensor_base_t *lis3mdl_create(const char *name);
sensor_error_t lis3mdl_set_range(sensor_base_t *sensor, lis3mdl_range_t range);
sensor_error_t lis3mdl_set_odr(sensor_base_t *sensor, lis3mdl_odr_t odr);
sensor_error_t lis3mdl_set_perf_mode(sensor_base_t *sensor, 
                                      lis3mdl_perf_mode_t xy_mode,
                                      lis3mdl_perf_mode_t z_mode);
sensor_error_t lis3mdl_config_int(sensor_base_t *sensor, lis3mdl_int_cfg_t *cfg);
sensor_error_t lis3mdl_read(sensor_base_t *sensor, lis3mdl_data_t *data);

/* Hard/soft iron compensation */
sensor_error_t lis3mdl_set_hard_iron_offset(sensor_base_t *sensor, vector3_t *offset);
sensor_error_t lis3mdl_set_soft_iron_matrix(sensor_base_t *sensor, float matrix[3][3]);

/* Temperature */
sensor_error_t lis3mdl_read_temp(sensor_base_t *sensor, float *temp_c);

#ifdef __cplusplus
}
#endif

#endif /* MAG_LIS3MDL_H */
