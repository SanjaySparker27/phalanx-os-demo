/*
 * Ouster OS1/OS2 LiDAR Driver
 * High-resolution 3D scanning LiDAR
 */

#ifndef LIDAR_OUSTER_H
#define LIDAR_OUSTER_H

#include "../core/sensor_base.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OUSTER_DEFAULT_UDP_PORT_LIDAR  7502
#define OUSTER_DEFAULT_UDP_PORT_IMU    7503
#define OUSTER_MAX_POINTS_PER_FRAME    131072

typedef enum {
    OUSTER_OS1_16 = 16,
    OUSTER_OS1_32 = 32,
    OUSTER_OS1_64 = 64,
    OUSTER_OS1_128 = 128,
    OUSTER_OS2_32 = 32,
    OUSTER_OS2_64 = 64,
    OUSTER_OS2_128 = 128
} ouster_model_t;

typedef enum {
    OUSTER_RES_512 = 512,
    OUSTER_RES_1024 = 1024,
    OUSTER_RES_2048 = 2048
} ouster_resolution_t;

typedef enum {
    OUSTER_MODE_512x10 = 0,
    OUSTER_MODE_512x20 = 1,
    OUSTER_MODE_1024x10 = 2,
    OUSTER_MODE_1024x20 = 3,
    OUSTER_MODE_2048x10 = 4,
    OUSTER_MODE_4096x5 = 5
} ouster_mode_t;

typedef struct {
    char sensor_ip[16];
    char host_ip[16];
    uint16_t udp_port_lidar;
    uint16_t udp_port_imu;
    ouster_mode_t mode;
    bool enable_imu;
    bool enable_phase_lock;
    uint16_t phase_lock_offset;
    bool dual_return;
    uint8_t signal_multiplier;
    uint16_t min_range_m;      /* Minimum range in meters */
    uint16_t max_range_m;      /* Maximum range in meters */
} ouster_config_t;

typedef struct {
    uint16_t measurement_id;
    uint16_t frame_id;
    uint64_t timestamp_ns;
    uint32_t encoder_count;
    uint16_t valid;
} ouster_column_header_t;

typedef struct {
    ouster_column_header_t header;
    lidar_point_t points[64];  /* Max beams per column */
    uint8_t num_points;
} ouster_column_t;

typedef struct {
    uint32_t num_points;
    lidar_point_t points[OUSTER_MAX_POINTS_PER_FRAME];
    uint16_t frame_id;
    uint64_t timestamp_ns;
    uint8_t num_columns;
} ouster_frame_t;

typedef struct {
    vector3_t accel;
    vector3_t gyro;
    uint64_t timestamp_ns;
} ouster_imu_packet_t;

typedef struct {
    float altitude_m;
    float azimuth_deg;
    float range_m;
} ouster_beam_intrinsics_t;

typedef struct {
    float lidar_to_sensor_transform[16];
    float imu_to_sensor_transform[16];
    ouster_beam_intrinsics_t beams[128];
    uint8_t num_beams;
} ouster_intrinsics_t;

/* Ouster API */
sensor_base_t *ouster_create(const char *name);
sensor_error_t ouster_connect(sensor_base_t *sensor, const char *sensor_ip, const char *host_ip);
sensor_error_t ouster_set_mode(sensor_base_t *sensor, ouster_mode_t mode);
sensor_error_t ouster_get_intrinsics(sensor_base_t *sensor, ouster_intrinsics_t *intrinsics);
sensor_error_t ouster_start_streaming(sensor_base_t *sensor);
sensor_error_t ouster_stop_streaming(sensor_base_t *sensor);
sensor_error_t ouster_get_frame(sensor_base_t *sensor, ouster_frame_t *frame, int timeout_ms);

/* TCP configuration commands */
sensor_error_t ouster_tcp_cmd(sensor_base_t *sensor, const char *cmd, char *response, size_t resp_size);
sensor_error_t ouster_get_sensor_info(sensor_base_t *sensor, char *info, size_t size);
sensor_error_t ouster_reinitialize(sensor_base_t *sensor);
sensor_error_t ouster_save_config(sensor_base_t *sensor);

/* Point cloud processing */
void ouster_cartesian_from_range(ouster_intrinsics_t *intrinsics, uint16_t beam_id, 
                                  uint16_t encoder, uint32_t range_mm,
                                  float *x, float *y, float *z);

#ifdef __cplusplus
}
#endif

#endif /* LIDAR_OUSTER_H */
