/*
 * SiK Telemetry Radio Driver
 * 3DR / Holybro SiK radio (433/915 MHz)
 */

#ifndef TELEMETRY_SIK_H
#define TELEMETRY_SIK_H

#include "../core/sensor_base.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SIK_DEFAULT_BAUD        57600
#define SIK_MAX_PACKET_SIZE     252
#define SIK_DEFAULT_NETID       25
#define SIK_EEPROM_VERSION      237

/* Frequency bands */
typedef enum {
    SIK_BAND_433 = 433,
    SIK_BAND_470 = 470,
    SIK_BAND_868 = 868,
    SIK_BAND_915 = 915
} sik_frequency_band_t;

typedef enum {
    SIK_AIR_SPEED_64 = 64,
    SIK_AIR_SPEED_128 = 128,
    SIK_AIR_SPEED_192 = 192,
    SIK_AIR_SPEED_250 = 250,
    SIK_AIR_SPEED_500 = 500
} sik_air_speed_t;

typedef struct {
    char uart_device[64];
    uint32_t baud_rate;
    uint32_t air_speed;
    uint8_t netid;
    uint32_t frequency_min_hz;
    uint32_t frequency_max_hz;
    uint32_t num_channels;
    uint8_t tx_power;           /* 0-20 (dBm) */
    uint8_t max_window;
    bool rtscts;
    bool ecc;
    bool mavlink;
    bool duty_cycle;
    bool lbt;                   /* Listen before talk */
    uint8_t max_freq_hop;
} sik_config_t;

typedef struct {
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t packets_lost;
    uint32_t bytes_sent;
    uint32_t bytes_received;
    uint32_t errors;
    uint32_t corrected_errors;
    int16_t noise_floor;
    int16_t rssi;
    uint16_t rx_errors;
    uint16_t tx_errors;
    uint16_t serial_overruns;
    uint16_t ecc_corrected;
} sik_stats_t;

/* EEPROM parameter structure */
typedef struct {
    uint8_t format;
    uint32_t serial_speed;
    uint32_t air_speed;
    uint8_t netid;
    uint16_t txpower;
    uint8_t ecc;
    uint8_t mavlink;
    uint8_t op_resend;
    uint8_t min_freq;
    uint8_t max_freq;
    uint8_t num_channels;
    uint8_t duty_cycle;
    uint8_t lbt_rssi;
    uint8_t max_window;
} sik_eeprom_params_t;

/* SiK API */
sensor_base_t *sik_create(const char *name);
sensor_error_t sik_enter_at_mode(sensor_base_t *sensor);
sensor_error_t sik_exit_at_mode(sensor_base_t *sensor);
sensor_error_t sik_at_cmd(sensor_base_t *sensor, const char *cmd, char *response, size_t resp_size);
sensor_error_t sik_load_params(sensor_base_t *sensor, sik_eeprom_params_t *params);
sensor_error_t sik_save_params(sensor_base_t *sensor);
sensor_error_t sik_set_frequency_range(sensor_base_t *sensor, uint32_t min_hz, uint32_t max_hz);
sensor_error_t sik_set_channels(sensor_base_t *sensor, uint8_t num_channels);
sensor_error_t sik_set_tx_power(sensor_base_t *sensor, uint8_t power_dbm);

/* Data transmission */
sensor_error_t sik_send(sensor_base_t *sensor, uint8_t *data, uint16_t len);
sensor_error_t sik_receive(sensor_base_t *sensor, uint8_t *buffer, uint16_t max_len, int timeout_ms);

/* Diagnostics */
sensor_error_t sik_get_rssi(sensor_base_t *sensor, int16_t *rssi);
sensor_error_t sik_get_stats(sensor_base_t *sensor, sik_stats_t *stats);
sensor_error_t sik_reset_stats(sensor_base_t *sensor);
sensor_error_t sik_get_radio_version(sensor_base_t *sensor, char *version, size_t size);
sensor_error_t sik_scan_channels(sensor_base_t *sensor, uint8_t *channels, uint8_t *num_channels);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_SIK_H */
