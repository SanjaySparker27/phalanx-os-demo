/*
 * TE Connectivity MS4525DO Digital Airspeed Sensor
 * I2C differential pressure sensor
 */

#ifndef AIRSPEED_MS4525_H
#define AIRSPEED_MS4525_H

#include "../core/sensor_base.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MS4525_I2C_ADDR         0x28
#define MS4525_I2C_ADDR_ALT1    0x36
#define MS4525_I2C_ADDR_ALT2    0x46

/* Sensor types */
typedef enum {
    MS4525_TYPE_A = 0,      /* ±1 PSI, 10-90% output */
    MS4525_TYPE_B = 1,      /* ±1 PSI, 5-95% output */
    MS4525_TYPE_C = 2       /* ±0.5 PSI, 5-95% output */
} ms4525_type_t;

typedef struct {
    char i2c_bus[32];
    uint8_t i2c_addr;
    ms4525_type_t type;
    float pitot_ratio;          /* Pitot tube correction factor */
    float zero_offset;          /* Calibration offset */
    bool use_temperature;       /* Read temperature sensor */
    float air_density;          /* kg/m³ (standard = 1.225) */
} ms4525_config_t;

typedef struct {
    float differential_pressure_pa;
    float temperature_c;
    float airspeed_m_s;
    float airspeed_knots;
    uint16_t raw_pressure;
    uint16_t raw_temperature;
    bool status_ok;
    sensor_timestamp_t timestamp;
} ms4525_data_t;

/* Status bits from sensor */
typedef struct {
    bool power_on;
    bool memory_error;
    bool logic_error;
    bool math_saturation;
} ms4525_status_t;

/* MS4525 API */
sensor_base_t *ms4525_create(const char *name);
sensor_error_t ms4525_set_type(sensor_base_t *sensor, ms4525_type_t type);
sensor_error_t ms4525_calibrate_zero(sensor_base_t *sensor);
sensor_error_t ms4525_set_air_density(sensor_base_t *sensor, float density);
sensor_error_t ms4525_read(sensor_base_t *sensor, ms4525_data_t *data);

/* Conversion utilities */
float ms4525_pressure_to_airspeed(float pressure_pa, float air_density);
float ms4525_airspeed_to_pressure(float airspeed_m_s, float air_density);

/* Calibration */
sensor_error_t ms4525_set_zero_offset(sensor_base_t *sensor, float offset_pa);
sensor_error_t ms4525_set_pitot_ratio(sensor_base_t *sensor, float ratio);

/* Status */
sensor_error_t ms4525_get_status(sensor_base_t *sensor, ms4525_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* AIRSPEED_MS4525_H */
