/*
 * Velodyne LiDAR Driver (VLP-16, VLP-32C, HDL-64E, VLS-128)
 * Industry-standard LiDAR for autonomous vehicles
 */

#ifndef LIDAR_VELODYNE_H
#define LIDAR_VELODYNE_H

#include "../core/sensor_base.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VELODYNE_DATA_PORT      2368
#define VELODYNE_POSITION_PORT  8308
#define VELODYNE_MAX_POINTS     300000
#define VELODYNE_PACKET_SIZE    1206
#define VELODYNE_BLOCKS_PER_PKT 12
#define VELODYNE_CHANNELS_PER_BLOCK 32

typedef enum {
    VELODYNE_VLP_16 = 16,
    VELODYNE_VLP_32C = 32,
    VELODYNE_HDL_32E = 32,
    VELODYNE_HDL_64E = 64,
    VELODYNE_VLS_128 = 128
} velodyne_model_t;

typedef enum {
    VELODYNE_RETURN_STRONGEST = 0x37,
    VELODYNE_RETURN_LAST = 0x38,
    VELODYNE_RETURN_DUAL = 0x39
} velodyne_return_mode_t;

typedef struct {
    char sensor_ip[16];
    char host_ip[16];
    uint16_t data_port;
    uint16_t position_port;
    velodyne_model_t model;
    velodyne_return_mode_t return_mode;
    uint16_t rpm;               /* Motor speed 300-1200 */
    bool gps_pps_sync;
    bool ethernet_power;
    float min_range_m;
    float max_range_m;
} velodyne_config_t;

typedef struct {
    uint16_t distance;          /* 2mm units */
    uint8_t intensity;
    uint8_t channel;
    float azimuth;
    float elevation;
    uint32_t timestamp_us;
} velodyne_point_t;

typedef struct {
    uint16_t flag;
    uint16_t azimuth;           /* 0-35999 (0.01 deg units) */
    velodyne_point_t channels[VELODYNE_CHANNELS_PER_BLOCK];
} velodyne_data_block_t;

typedef struct {
    velodyne_data_block_t blocks[VELODYNE_BLOCKS_PER_PKT];
    uint32_t gps_timestamp;
    uint8_t return_mode;
    uint8_t model_id;
    uint8_t raw_data[VELODYNE_PACKET_SIZE];
} velodyne_packet_t;

typedef struct {
    uint32_t num_points;
    velodyne_point_t points[VELODYNE_MAX_POINTS];
    uint64_t timestamp_ns;
    uint16_t revolution;
    bool complete;
} velodyne_frame_t;

/* GPS/IMU packet from position port */
typedef struct {
    uint8_t nmea_sentence[128];
    uint8_t gprmc[128];
    uint8_t timestamp[4];
    uint8_t pps_status;
    uint8_t temperature;
} velodyne_position_packet_t;

/* Calibration data */
typedef struct {
    float vert_correction[128];
    float rot_correction[128];
    float dist_correction[128];
    float vert_offset_correction[128];
    float horiz_offset_correction[128];
    float focal_distance[128];
    float focal_slope[128];
    float min_intensity[128];
    float max_intensity[128];
    uint8_t laser_count;
} velodyne_calibration_t;

/* Velodyne API */
sensor_base_t *velodyne_create(const char *name);
sensor_error_t velodyne_load_calibration(sensor_base_t *sensor, const char *filename);
sensor_error_t velodyne_set_rpm(sensor_base_t *sensor, uint16_t rpm);
sensor_error_t velodyne_set_return_mode(sensor_base_t *sensor, velodyne_return_mode_t mode);
sensor_error_t velodyne_get_frame(sensor_base_t *sensor, velodyne_frame_t *frame, int timeout_ms);

/* Web interface commands */
sensor_error_t velodyne_web_get_config(sensor_base_t *sensor, char *config, size_t size);
sensor_error_t velodyne_web_set_config(sensor_base_t *sensor, const char *param, const char *value);

/* Point processing */
void velodyne_xyz_from_raw(velodyne_calibration_t *cal, velodyne_point_t *point,
                            float *x, float *y, float *z);

#ifdef __cplusplus
}
#endif

#endif /* LIDAR_VELODYNE_H */
