/*
 * TE Connectivity MS5611 Barometer
 * High resolution pressure and temperature sensor
 */

#ifndef BARO_MS5611_H
#define BARO_MS5611_H

#include "../core/sensor_base.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MS5611_I2C_ADDR        0x76
#define MS5611_I2C_ADDR_ALT    0x77
#define MS5611_CMD_RESET       0x1E
#define MS5611_CMD_D1_256      0x40
#define MS5611_CMD_D1_512      0x42
#define MS5611_CMD_D1_1024     0x44
#define MS5611_CMD_D1_2048     0x46
#define MS5611_CMD_D1_4096     0x48
#define MS5611_CMD_D2_256      0x50
#define MS5611_CMD_D2_512      0x52
#define MS5611_CMD_D2_1024     0x54
#define MS5611_CMD_D2_2048     0x56
#define MS5611_CMD_D2_4096     0x58
#define MS5611_CMD_ADC_READ    0x00
#define MS5611_CMD_PROM_READ   0xA0

typedef enum {
    MS5611_OSR_256 = 256,
    MS5611_OSR_512 = 512,
    MS5611_OSR_1024 = 1024,
    MS5611_OSR_2048 = 2048,
    MS5611_OSR_4096 = 4096
} ms5611_osr_t;

typedef struct {
    char i2c_bus[32];
    uint8_t i2c_addr;
    ms5611_osr_t pressure_osr;
    ms5611_osr_t temp_osr;
    uint32_t sample_rate_hz;
} ms5611_config_t;

typedef struct {
    float pressure_pa;
    float temperature_c;
    float altitude_m;
    uint32_t raw_pressure;
    uint32_t raw_temperature;
    sensor_timestamp_t timestamp;
} ms5611_data_t;

/* Calibration coefficients */
typedef struct {
    uint16_t c1;    /* Pressure sensitivity */
    uint16_t c2;    /* Pressure offset */
    uint16_t c3;    /* Temperature coeff of pressure sensitivity */
    uint16_t c4;    /* Temperature coeff of pressure offset */
    uint16_t c5;    /* Reference temperature */
    uint16_t c6;    /* Temperature coeff of temperature */
    uint16_t crc;
} ms5611_prom_t;

/* MS5611 API */
sensor_base_t *ms5611_create(const char *name);
sensor_error_t ms5611_reset(sensor_base_t *sensor);
sensor_error_t ms5611_read_prom(sensor_base_t *sensor);
sensor_error_t ms5611_set_osr(sensor_base_t *sensor, ms5611_osr_t pressure_osr, ms5611_osr_t temp_osr);
sensor_error_t ms5611_read(sensor_base_t *sensor, ms5611_data_t *data);

/* Conversion functions */
float ms5611_pressure_to_altitude(float pressure_pa, float sea_level_pa);
float ms5611_altitude_to_pressure(float altitude_m, float sea_level_pa);

#ifdef __cplusplus
}
#endif

#endif /* BARO_MS5611_H */
