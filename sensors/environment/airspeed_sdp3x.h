/*
 * Sensirion SDP3x Differential Pressure Sensor
 * High-precision digital airspeed sensor
 */

#ifndef AIRSPEED_SDP3X_H
#define AIRSPEED_SDP3X_H

#include "../core/sensor_base.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SDP31_I2C_ADDR          0x21
#define SDP32_I2C_ADDR          0x22
#define SDP33_I2C_ADDR          0x23

/* Measurement commands */
typedef enum {
    SDP3X_CMD_MASS_FLOW_AVG = 0x3603,   /* Mass flow, avg until read */
    SDP3X_CMD_MASS_FLOW = 0x3608,       /* Mass flow, continuous */
    SDP3X_CMD_DIFF_PRESS_AVG = 0x3615,  /* Diff pressure, avg until read */
    SDP3X_CMD_DIFF_PRESS = 0x361E,      /* Diff pressure, continuous */
    SDP3X_CMD_SLEEP = 0x3677,
    SDP3X_CMD_WAKEUP = 0x3677,
    SDP3X_CMD_ENTER_CLI = 0x367D,
    SDP3X_CMD_EXIT_CLI = 0x367D,
    SDP3X_CMD_RESET = 0x001D
} sdp3x_cmd_t;

typedef enum {
    SDP31_RANGE_500_PA = 500,
    SDP32_RANGE_125_PA = 125,
    SDP33_RANGE_500_PA = 500
} sdp3x_range_t;

typedef struct {
    char i2c_bus[32];
    uint8_t i2c_addr;
    sdp3x_range_t range;
    sdp3x_cmd_t mode;
    float pitot_ratio;
    float air_density;          /* kg/m³ */
    bool temp_compensation;     /* Use on-chip temp compensation */
} sdp3x_config_t;

typedef struct {
    float differential_pressure_pa;
    float temperature_c;
    float airspeed_m_s;
    float airspeed_knots;
    float mass_flow;            /* Only valid in mass flow mode */
    int16_t raw_pressure;
    int16_t raw_temperature;
    float scale_factor;
    sensor_timestamp_t timestamp;
} sdp3x_data_t;

/* Product info */
typedef struct {
    uint32_t product_number;
    char serial_number[8];
    uint8_t scale_factor;
} sdp3x_info_t;

/* SDP3x API */
sensor_base_t *sdp3x_create(const char *name);
sensor_error_t sdp3x_reset(sensor_base_t *sensor);
sensor_error_t sdp3x_set_mode(sensor_base_t *sensor, sdp3x_cmd_t mode);
sensor_error_t sdp3x_sleep(sensor_base_t *sensor);
sensor_error_t sdp3x_wakeup(sensor_base_t *sensor);
sensor_error_t sdp3x_read(sensor_base_t *sensor, sdp3x_data_t *data);

/* Scale factor depends on sensor model */
sensor_error_t sdp3x_read_scale_factor(sensor_base_t *sensor, float *scale);
sensor_error_t sdp3x_get_product_info(sensor_base_t *sensor, sdp3x_info_t *info);

/* Calibration */
sensor_error_t sdp3x_set_pitot_ratio(sensor_base_t *sensor, float ratio);
sensor_error_t sdp3x_calibrate_zero(sensor_base_t *sensor);

/* Conversion */
float sdp3x_pressure_to_airspeed(float pressure_pa, float air_density);

#ifdef __cplusplus
}
#endif

#endif /* AIRSPEED_SDP3X_H */
