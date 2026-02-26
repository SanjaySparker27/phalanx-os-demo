/*
 * u-blox ZED-F9P GNSS/RTK Driver
 * High-precision GPS with RTK support
 */

#ifndef GNSS_UBLOX_H
#define GNSS_UBLOX_H

#include "../core/sensor_base.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UBLOX_MAX_SATELLITES 32
#define UBLOX_UART_BAUD_DEFAULT 38400
#define UBLOX_UART_BAUD_HIGH 115200

typedef enum {
    UBLOX_FIX_NONE = 0,
    UBLOX_FIX_DEAD_RECKONING = 1,
    UBLOX_FIX_2D = 2,
    UBLOX_FIX_3D = 3,
    UBLOX_FIX_GNSS_DR = 4,
    UBLOX_FIX_TIME_ONLY = 5,
    UBLOX_FIX_RTK_FLOAT = 6,
    UBLOX_FIX_RTK_FIXED = 7
} ublox_fix_type_t;

typedef struct {
    char uart_device[64];
    uint32_t baud_rate;
    bool enable_rtk;
    char rtk_correction_source[256];  /* NTRIP caster or local base */
    float survey_in_duration;         /* Seconds for base station mode */
    bool enable_pps;
    uint8_t constellations;           /* GPS, GLONASS, Galileo, BeiDou flags */
    uint8_t update_rate_hz;
} ublox_config_t;

typedef struct {
    gnss_position_t position;
    gnss_velocity_t velocity;
    float pps_offset_ns;
    float rtk_age_sec;
    uint16_t base_station_id;
    bool rtk_valid;
} ublox_nav_data_t;

typedef struct {
    uint8_t gnss_id;
    uint8_t sv_id;
    uint8_t cno;           /* Carrier-to-noise ratio (dBHz) */
    int8_t elevation;      /* Degrees */
    int16_t azimuth;       /* Degrees */
    bool used;
} ublox_satellite_t;

typedef struct {
    ublox_satellite_t sats[UBLOX_MAX_SATELLITES];
    uint8_t num_visible;
    uint8_t num_used;
} ublox_sat_info_t;

/* u-blox specific API */
sensor_base_t *ublox_create(const char *name);
sensor_error_t ublox_load_config(sensor_base_t *sensor, const char *config_file);
sensor_error_t ublox_enable_constellation(sensor_base_t *sensor, uint8_t gnss_id, bool enable);
sensor_error_t ublox_set_update_rate(sensor_base_t *sensor, uint8_t rate_hz);
sensor_error_t ublox_start_survey_in(sensor_base_t *sensor, float duration_sec);
sensor_error_t ublox_inject_rtcm(sensor_base_t *sensor, uint8_t *data, size_t len);
sensor_error_t ublox_get_satellites(sensor_base_t *sensor, ublox_sat_info_t *info);

/* UBX protocol helpers */
sensor_error_t ublox_send_ubx(sensor_base_t *sensor, uint8_t class_id, uint8_t msg_id, 
                               uint8_t *payload, uint16_t len);
int ublox_parse_ubx(uint8_t *data, size_t len, ublox_nav_data_t *nav);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_UBLOX_H */
