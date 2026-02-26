/*
 * RFD900 Long-Range Telemetry Radio
 * 900MHz FHSS telemetry for UAVs
 */

#ifndef TELEMETRY_RFD900_H
#define TELEMETRY_RFD900_H

#include "../core/sensor_base.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RFD900_DEFAULT_BAUD         57600
#define RFD900_MAX_PACKET_SIZE      252
#define RFD900_DEFAULT_NETID        25
#define RFD900_MIN_FREQ             902000000UL
#define RFD900_MAX_FREQ             928000000UL

typedef enum {
    RFD900_AIR_SPEED_4 = 4,
    RFD900_AIR_SPEED_8 = 8,
    RFD900_AIR_SPEED_16 = 16,
    RFD900_AIR_SPEED_24 = 24,
    RFD900_AIR_SPEED_48 = 48,
    RFD900_AIR_SPEED_64 = 64,
    RFD900_AIR_SPEED_96 = 96,
    RFD900_AIR_SPEED_128 = 128,
    RFD900_AIR_SPEED_192 = 192,
    RFD900_AIR_SPEED_250 = 250
} rfd900_air_speed_t;

typedef enum {
    RFD900_TX_POWER_1 = 1,      /* 1.3mW */
    RFD900_TX_POWER_2 = 2,      /* 1.5mW */
    RFD900_TX_POWER_5 = 5,      /* 3.2mW */
    RFD900_TX_POWER_8 = 8,      /* 6.3mW */
    RFD900_TX_POWER_11 = 11,    /* 12.5mW */
    RFD900_TX_POWER_14 = 14,    /* 25mW */
    RFD900_TX_POWER_17 = 17,    /* 50mW */
    RFD900_TX_POWER_20 = 20,    /* 100mW */
    RFD900_TX_POWER_21 = 21,    /* 125mW */
    RFD900_TX_POWER_22 = 22,    /* 160mW */
    RFD900_TX_POWER_23 = 23,    /* 200mW */
    RFD900_TX_POWER_24 = 24,    /* 250mW */
    RFD900_TX_POWER_25 = 25,    /* 316mW */
    RFD900_TX_POWER_26 = 26,    /* 398mW */
    RFD900_TX_POWER_27 = 27,    /* 500mW */
    RFD900_TX_POWER_28 = 28,    /* 630mW */
    RFD900_TX_POWER_29 = 29,    /* 794mW */
    RFD900_TX_POWER_30 = 30     /* 1W */
} rfd900_tx_power_t;

typedef struct {
    char uart_device[64];
    uint32_t baud_rate;
    uint32_t air_speed;         /* RFD900 air speed in kbps */
    uint8_t netid;
    uint32_t frequency_hz;
    uint8_t tx_power;           /* 1-30 */
    uint8_t max_window;         /* Packet window size */
    bool rtscts;                /* Hardware flow control */
    bool ecc;                   /* Error correction */
    bool mavlink;               /* MAVLink framing */
    bool op_resend;             /* Opportunistic resend */
} rfd900_config_t;

typedef struct {
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t packets_lost;
    uint32_t bytes_sent;
    uint32_t bytes_received;
    uint32_t errors;
    uint32_t noise_floor;
    uint16_t local_rssi;
    uint16_t remote_rssi;
    uint16_t local_noise;
    uint16_t remote_noise;
    uint8_t tx_buffer;
    uint8_t rx_buffer;
} rfd900_stats_t;

typedef struct {
    uint32_t destination;
    uint8_t data[RFD900_MAX_PACKET_SIZE];
    uint16_t length;
    uint8_t rssi;
    uint16_t lqi;
} rfd900_packet_t;

/* RFD900 API */
sensor_base_t *rfd900_create(const char *name);
sensor_error_t rfd900_enter_at_mode(sensor_base_t *sensor);
sensor_error_t rfd900_exit_at_mode(sensor_base_t *sensor);
sensor_error_t rfd900_at_cmd(sensor_base_t *sensor, const char *cmd, char *response, size_t resp_size);
sensor_error_t rfd900_set_frequency(sensor_base_t *sensor, uint32_t freq_hz);
sensor_error_t rfd900_set_air_speed(sensor_base_t *sensor, rfd900_air_speed_t speed);
sensor_error_t rfd900_set_tx_power(sensor_base_t *sensor, rfd900_tx_power_t power);
sensor_error_t rfd900_save_settings(sensor_base_t *sensor);
sensor_error_t rfd900_reboot(sensor_base_t *sensor);

/* Data transmission */
sensor_error_t rfd900_send(sensor_base_t *sensor, uint8_t *data, uint16_t len);
sensor_error_t rfd900_receive(sensor_base_t *sensor, uint8_t *buffer, uint16_t max_len, int timeout_ms);
sensor_error_t rfd900_send_packet(sensor_base_t *sensor, rfd900_packet_t *packet);

/* Statistics and diagnostics */
sensor_error_t rfd900_get_rssi(sensor_base_t *sensor, int16_t *local_rssi, int16_t *remote_rssi);
sensor_error_t rfd900_get_stats(sensor_base_t *sensor, rfd900_stats_t *stats);
sensor_error_t rfd900_reset_stats(sensor_base_t *sensor);
sensor_error_t rfd900_get_firmware_version(sensor_base_t *sensor, char *version, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_RFD900_H */
