/*
 * Sony IMX219 Camera Driver
 * 8MP Raspberry Pi Camera Module V2 compatible
 */

#ifndef CAMERA_IMX219_H
#define CAMERA_IMX219_H

#include "../core/sensor_base.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMX219_DEFAULT_WIDTH    3280
#define IMX219_DEFAULT_HEIGHT   2464
#define IMX219_DEFAULT_FPS      30

/* Supported modes */
typedef enum {
    IMX219_MODE_3280x2464_21FPS,
    IMX219_MODE_1920x1080_30FPS,
    IMX219_MODE_1640x1232_30FPS,
    IMX219_MODE_1280x720_60FPS,
    IMX219_MODE_640x480_90FPS
} imx219_mode_t;

typedef struct {
    imx219_mode_t mode;
    uint32_t format;
    char device_path[64];
    bool enable_hdr;
    bool enable_awb;
} imx219_config_t;

typedef struct {
    uint8_t *frame_data;
    size_t frame_size;
    camera_frame_info_t info;
    sensor_timestamp_t timestamp;
} imx219_frame_t;

/* IMX219 specific API */
sensor_base_t *imx219_create(const char *name);
sensor_error_t imx219_set_mode(sensor_base_t *sensor, imx219_mode_t mode);
sensor_error_t imx219_enable_hdr(sensor_base_t *sensor, bool enable);
sensor_error_t imx219_set_awb_mode(sensor_base_t *sensor, uint32_t mode);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_IMX219_H */
