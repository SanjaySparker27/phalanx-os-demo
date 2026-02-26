/*
 * Sensor Types Definition
 * Unified type system for all autonomous vehicle sensors
 */

#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Sensor categories */
typedef enum {
    SENSOR_CATEGORY_CAMERA,
    SENSOR_CATEGORY_GNSS,
    SENSOR_CATEGORY_IMU,
    SENSOR_CATEGORY_LIDAR,
    SENSOR_CATEGORY_SATCOM,
    SENSOR_CATEGORY_TELEMETRY,
    SENSOR_CATEGORY_ENVIRONMENT,
    SENSOR_CATEGORY_UNKNOWN
} sensor_category_t;

/* Sensor operational states */
typedef enum {
    SENSOR_STATE_UNINITIALIZED,
    SENSOR_STATE_INITIALIZING,
    SENSOR_STATE_CALIBRATING,
    SENSOR_STATE_ACTIVE,
    SENSOR_STATE_STANDBY,
    SENSOR_STATE_ERROR,
    SENSOR_STATE_SHUTDOWN
} sensor_state_t;

/* Common timestamp structure with nanosecond precision */
typedef struct {
    uint64_t nanoseconds;       /* Monotonic clock time */
    uint64_t realtime_ns;       /* Real-time clock */
    uint64_t sample_count;      /* Sequential sample counter */
} sensor_timestamp_t;

/* Common sensor metadata */
typedef struct {
    char name[64];
    char model[32];
    char serial[32];
    char version[16];
    sensor_category_t category;
    uint32_t data_rate_hz;
    uint32_t priority;
} sensor_info_t;

/* 3D vector for accelerometer/gyroscope/magnetometer */
typedef struct {
    float x;
    float y;
    float z;
} vector3_t;

/* 4D quaternion for orientation */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

/* GPS/RTK position data */
typedef struct {
    double latitude;            /* Degrees */
    double longitude;           /* Degrees */
    double altitude_msl;        /* Meters above sea level */
    double altitude_ellipsoid;  /* Meters above WGS84 ellipsoid */
    float horizontal_accuracy;  /* Meters (1-sigma) */
    float vertical_accuracy;    /* Meters (1-sigma) */
    uint8_t fix_type;           /* 0=none, 2=2D, 3=3D, 4=RTK float, 5=RTK fixed */
    uint8_t num_satellites;
    float hdop;                 /* Horizontal dilution of precision */
    float pdop;                 /* Position dilution of precision */
} gnss_position_t;

/* GPS/RTK velocity data */
typedef struct {
    float north_m_s;
    float east_m_s;
    float down_m_s;
    float ground_speed_m_s;
    float heading_deg;
    float speed_accuracy;       /* m/s */
    float heading_accuracy;     /* degrees */
} gnss_velocity_t;

/* IMU data - raw and processed */
typedef struct {
    vector3_t accel;            /* m/s^2 */
    vector3_t gyro;             /* rad/s */
    vector3_t mag;              /* uT */
    float accel_range_g;
    float gyro_range_dps;
    float mag_range_uT;
    float temperature;
} imu_data_t;

/* Camera frame metadata */
typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    uint32_t format;            /* FourCC format */
    uint32_t exposure_us;
    float gain_db;
    float focus_distance;
} camera_frame_info_t;

/* LIDAR point data */
typedef struct {
    float x;                    /* Meters */
    float y;
    float z;
    float intensity;
    uint16_t ring;              /* Laser ring/channel */
    uint16_t timestamp_offset_us;
} lidar_point_t;

/* SATCOM message status */
typedef enum {
    SATCOM_STATUS_IDLE,
    SATCOM_STATUS_TRANSMITTING,
    SATCOM_STATUS_RECEIVING,
    SATCOM_STATUS_COMPLETED,
    SATCOM_STATUS_ERROR,
    SATCOM_STATUS_NO_SIGNAL
} satcom_status_t;

/* Telemetry radio status */
typedef enum {
    TELEM_STATUS_DISCONNECTED,
    TELEM_STATUS_CONNECTING,
    TELEM_STATUS_CONNECTED,
    TELEM_STATUS_LOW_SIGNAL,
    TELEM_STATUS_ERROR
} telemetry_status_t;

/* Error codes */
typedef enum {
    SENSOR_OK = 0,
    SENSOR_ERROR_INIT = -1,
    SENSOR_ERROR_IO = -2,
    SENSOR_ERROR_TIMEOUT = -3,
    SENSOR_ERROR_CALIBRATION = -4,
    SENSOR_ERROR_OVERFLOW = -5,
    SENSOR_ERROR_INVALID_PARAM = -6,
    SENSOR_ERROR_NOT_SUPPORTED = -7,
    SENSOR_ERROR_BUSY = -8
} sensor_error_t;

/* Callback function types */
typedef void (*sensor_data_callback_t)(void *userdata, void *data, size_t size, sensor_timestamp_t *ts);
typedef void (*sensor_status_callback_t)(void *userdata, sensor_state_t old_state, sensor_state_t new_state);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_TYPES_H */
