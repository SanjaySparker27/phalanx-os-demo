/*
 * Iridium Short Burst Data (SBD) Satellite Communication
 * Global satellite messaging for beyond-LOS operations
 */

#ifndef SATCOM_IRIDIUM_H
#define SATCOM_IRIDIUM_H

#include "../core/sensor_base.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IRIDIUM_SBD_MAX_MO_MSG    340   /* Mobile-originated max bytes */
#define IRIDIUM_SBD_MAX_MT_MSG    270   /* Mobile-terminated max bytes */
#define IRIDIUM_DEFAULT_BAUD      19200
#define IRIDIUM_SIGNAL_MINIMUM    2     /* Minimum signal strength 0-5 */

typedef enum {
    IRIDIUM_STATE_OFF,
    IRIDIUM_STATE_INITIALIZING,
    IRIDIUM_STATE_READY,
    IRIDIUM_STATE_TRANSMITTING,
    IRIDIUM_STATE_RECEIVING,
    IRIDIUM_STATE_ERROR
} iridium_state_t;

typedef struct {
    char uart_device[64];
    uint32_t baud_rate;
    uint16_t mo_max_size;       /* Mobile originated message max size */
    uint16_t mt_max_size;       /* Mobile terminated message max size */
    uint32_t retry_interval_ms;
    uint8_t max_retries;
    bool ring_alert;            /* Enable ring indication for MT messages */
    bool auto_send;             /* Auto-transmit on message queue */
} iridium_config_t;

typedef struct {
    uint8_t data[IRIDIUM_SBD_MAX_MO_MSG];
    uint16_t length;
    uint16_t sequence;
    uint32_t priority;
    uint64_t timestamp_ns;
} iridium_mo_message_t;

typedef struct {
    uint8_t data[IRIDIUM_SBD_MAX_MT_MSG];
    uint16_t length;
    uint16_t msg_num;
    uint16_t momsn;             /* Mobile originated message sequence number */
    uint16_t mtmsn;             /* Mobile terminated message sequence number */
    uint64_t timestamp_ns;
} iridium_mt_message_t;

typedef struct {
    uint8_t signal_strength;    /* 0-5 (0=no signal, 5=excellent) */
    uint8_t service_available;
    uint8_t antenna_ok;
    uint8_t network_available;
    uint32_t ring_indication;
    int16_t rssi_dbm;
} iridium_status_t;

typedef struct {
    uint32_t mo_messages_sent;
    uint32_t mo_success_count;
    uint32_t mo_failure_count;
    uint32_t mt_messages_received;
    uint32_t mt_discarded;
    uint32_t failed_sessions;
    uint32_t last_session_time_ms;
} iridium_stats_t;

/* Iridium SBD API */
sensor_base_t *iridium_create(const char *name);
sensor_error_t iridium_power_on(sensor_base_t *sensor);
sensor_error_t iridium_power_off(sensor_base_t *sensor);
sensor_error_t iridium_get_signal_strength(sensor_base_t *sensor, uint8_t *strength);
sensor_error_t iridium_send_message(sensor_base_t *sensor, iridium_mo_message_t *msg);
sensor_error_t iridium_check_mailbox(sensor_base_t *sensor, iridium_mt_message_t *msg);
sensor_error_t iridium_get_status(sensor_base_t *sensor, iridium_status_t *status);

/* Queued message handling */
sensor_error_t iridium_queue_message(sensor_base_t *sensor, iridium_mo_message_t *msg);
sensor_error_t iridium_process_queue(sensor_base_t *sensor);
sensor_error_t iridium_clear_mo_buffer(sensor_base_t *sensor);
sensor_error_t iridium_clear_mt_buffer(sensor_base_t *sensor);

/* Extended AT commands */
sensor_error_t iridium_at_cmd(sensor_base_t *sensor, const char *cmd, char *response, size_t resp_size, int timeout_ms);
sensor_error_t iridium_set_mo_max_size(sensor_base_t *sensor, uint16_t size);
sensor_error_t iridium_set_mt_max_size(sensor_base_t *sensor, uint16_t size);

/* Binary transfer mode */
sensor_error_t iridium_send_binary(sensor_base_t *sensor, uint8_t *data, uint16_t len);
sensor_error_t iridium_receive_binary(sensor_base_t *sensor, uint8_t *buffer, uint16_t *len);

/* Statistics */
sensor_error_t iridium_get_stats(sensor_base_t *sensor, iridium_stats_t *stats);
sensor_error_t iridium_reset_stats(sensor_base_t *sensor);

#ifdef __cplusplus
}
#endif

#endif /* SATCOM_IRIDIUM_H */
