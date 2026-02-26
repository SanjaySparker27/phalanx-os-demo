/*
 * Bosch BMP388 Barometer
 * Low power, high precision pressure sensor
 */

#ifndef BARO_BMP388_H
#define BARO_BMP388_H

#include "../core/sensor_base.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BMP388_I2C_ADDR         0x76
#define BMP388_I2C_ADDR_ALT     0x77
#define BMP388_SPI_DEV          "/dev/spidev0.0"
#define BMP388_CHIP_ID          0x50

typedef enum {
    BMP388_ODR_200_HZ = 0x00,
    BMP388_ODR_100_HZ = 0x01,
    BMP388_ODR_50_HZ = 0x02,
    BMP388_ODR_25_HZ = 0x03,
    BMP388_ODR_12_5_HZ = 0x04,
    BMP388_ODR_6_25_HZ = 0x05,
    BMP388_ODR_3_1_HZ = 0x06,
    BMP388_ODR_1_5_HZ = 0x07,
    BMP388_ODR_0_78_HZ = 0x08,
    BMP388_ODR_0_39_HZ = 0x09,
    BMP388_ODR_0_2_HZ = 0x0A,
    BMP388_ODR_0_1_HZ = 0x0B,
    BMP388_ODR_0_05_HZ = 0x0C,
    BMP388_ODR_0_02_HZ = 0x0D,
    BMP388_ODR_0_01_HZ = 0x0E,
    BMP388_ODR_0_006_HZ = 0x0F
} bmp388_odr_t;

typedef enum {
    BMP388_OVERSAMPLING_1X = 0,
    BMP388_OVERSAMPLING_2X = 1,
    BMP388_OVERSAMPLING_4X = 2,
    BMP388_OVERSAMPLING_8X = 3,
    BMP388_OVERSAMPLING_16X = 4,
    BMP388_OVERSAMPLING_32X = 5
} bmp388_oversampling_t;

typedef enum {
    BMP388_IIR_OFF = 0,
    BMP388_IIR_2X = 1,
    BMP388_IIR_4X = 2,
    BMP388_IIR_8X = 3,
    BMP388_IIR_16X = 4,
    BMP388_IIR_32X = 5,
    BMP388_IIR_64X = 6,
    BMP388_IIR_128X = 7
} bmp388_iir_coeff_t;

typedef struct {
    char i2c_bus[32];
    uint8_t i2c_addr;
    bmp388_odr_t odr;
    bmp388_oversampling_t osr_pressure;
    bmp388_oversampling_t osr_temperature;
    bmp388_iir_coeff_t iir_filter;
    bool use_fifo;
    bool pressure_en;
    bool temperature_en;
} bmp388_config_t;

typedef struct {
    float pressure_pa;
    float temperature_c;
    float altitude_m;
    uint32_t raw_pressure;
    uint32_t raw_temperature;
    uint32_t sensor_time;
    sensor_timestamp_t timestamp;
} bmp388_data_t;

/* Interrupt configuration */
typedef struct {
    bool drdy_en;
    bool fifo_full_en;
    bool fifo_wm_en;
    bool data_rdy_int;
    bool int_level;      /* 0=active low, 1=active high */
    bool int_od;         /* 0=push-pull, 1=open drain */
    bool int_latch;
} bmp388_int_config_t;

/* BMP388 API */
sensor_base_t *bmp388_create(const char *name);
sensor_error_t bmp388_soft_reset(sensor_base_t *sensor);
sensor_error_t bmp388_set_odr(sensor_base_t *sensor, bmp388_odr_t odr);
sensor_error_t bmp388_set_oversampling(sensor_base_t *sensor, 
                                        bmp388_oversampling_t press_os,
                                        bmp388_oversampling_t temp_os);
sensor_error_t bmp388_set_iir_filter(sensor_base_t *sensor, bmp388_iir_coeff_t coeff);
sensor_error_t bmp388_config_interrupts(sensor_base_t *sensor, bmp388_int_config_t *cfg);
sensor_error_t bmp388_read(sensor_base_t *sensor, bmp388_data_t *data);

/* FIFO operations */
sensor_error_t bmp388_fifo_enable(sensor_base_t *sensor, bool enable);
sensor_error_t bmp388_fifo_flush(sensor_base_t *sensor);
sensor_error_t bmp388_fifo_read(sensor_base_t *sensor, bmp388_data_t *data, uint8_t max_samples);

/* Calibration NVM */
sensor_error_t bmp388_get_calibration(sensor_base_t *sensor, void *cal_data);

#ifdef __cplusplus
}
#endif

#endif /* BARO_BMP388_H */
