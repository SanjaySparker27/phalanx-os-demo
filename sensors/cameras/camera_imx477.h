/*
 * Sony IMX477 Camera Driver
 * High-resolution 12MP camera for Jetson platforms
 */

#ifndef CAMERA_IMX477_H
#define CAMERA_IMX477_H

#include "../core/sensor_base.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMX477_DEFAULT_WIDTH    4056
#define IMX477_DEFAULT_HEIGHT   3040
#define IMX477_DEFAULT_FPS      30

typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t fps;
    uint32_t format;            /* NV12, YUY2, etc. */
    uint32_t exposure_min_us;
    uint32_t exposure_max_us;
    float gain_min;
    float gain_max;
    char device_path[64];       /* /dev/video0 */
    char sensor_id[16];         /* i2c bus address */
} imx477_config_t;

typedef struct {
    uint8_t *frame_data;
    size_t frame_size;
    camera_frame_info_t info;
    sensor_timestamp_t timestamp;
} imx477_frame_t;

/* IMX477 specific API */
sensor_base_t *imx477_create(const char *name);
sensor_error_t imx477_set_exposure(sensor_base_t *sensor, uint32_t exposure_us);
sensor_error_t imx477_set_gain(sensor_base_t *sensor, float gain_db);
sensor_error_t imx477_set_frame_rate(sensor_base_t *sensor, uint32_t fps);
sensor_error_t imx477_trigger_capture(sensor_base_t *sensor);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_IMX477_H */
