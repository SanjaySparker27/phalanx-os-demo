/*
 * PNI Sensor RM3100 Magnetometer
 * High-performance geomagnetic sensor
 */

#ifndef MAG_RM3100_H
#define MAG_RM3100_H

#include "../core/sensor_base.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RM3100_I2C_ADDR         0x20
#define RM3100_I2C_ADDR_ALT     0x21
#define RM3100_REVID            0x22

/* Cycle counts for different resolutions */
#define RM3100_CC_75HZ          0x0C00   /* ~75 Hz, 200 counts */
#define RM3100_CC_150HZ         0x0600   /* ~150 Hz, 100 counts */
#define RM3100_CC_300HZ         0x0300   /* ~300 Hz, 50 counts */
#define RM3100_CC_600HZ         0x0180   /* ~600 Hz, 25 counts */

typedef struct {
    char i2c_bus[32];
    uint8_t i2c_addr;
    uint16_t cycle_count_x;
    uint16_t cycle_count_y;
    uint16_t cycle_count_z;
    uint8_t sample_rate;        /* Hz: 600, 300, 150, 75, 37, 18, 9 */
    bool cmm_mode;              /* Continuous measurement mode */
    bool drdy_on_por;
    uint8_t alarm_threshold;
} rm3100_config_t;

typedef struct {
    float mag_x;            /* uT */
    float mag_y;
    float mag_z;
    float magnitude;
    int32_t raw_x;
    int32_t raw_y;
    int32_t raw_z;
    sensor_timestamp_t timestamp;
} rm3100_data_t;

/* RM3100 API */
sensor_base_t *rm3100_create(const char *name);
sensor_error_t rm3100_reset(sensor_base_t *sensor);
sensor_error_t rm3100_set_cycle_counts(sensor_base_t *sensor, 
                                        uint16_t x, uint16_t y, uint16_t z);
sensor_error_t rm3100_set_sample_rate(sensor_base_t *sensor, uint8_t rate_hz);
sensor_error_t rm3100_set_cmm(sensor_base_t *sensor, bool enable, uint8_t cmm_rate);
sensor_error_t rm3100_single_measurement(sensor_base_t *sensor);
sensor_error_t rm3100_read(sensor_base_t *sensor, rm3100_data_t *data);

/* Calibration */
sensor_error_t rm3100_get_gain(sensor_base_t *sensor, float *gain);
sensor_error_t rm3100_set_calibration(sensor_base_t *sensor, float hard_iron[3], float soft_iron[3][3]);

/* Status */
sensor_error_t rm3100_get_revid(sensor_base_t *sensor, uint8_t *revid);
sensor_error_t rm3100_check_drdy(sensor_base_t *sensor, bool *ready);

#ifdef __cplusplus
}
#endif

#endif /* MAG_RM3100_H */
