/*
 * Unified Sensor Library
 * Main library implementation
 */

#include "sensors.h"
#include <stdio.h>
#include <string.h>

static bool g_initialized = false;

sensor_error_t sensors_init(void) {
    if (g_initialized) {
        return SENSOR_OK;
    }
    
    /* Initialize GStreamer for cameras */
    gst_init(NULL, NULL);
    
    g_initialized = true;
    return SENSOR_OK;
}

void sensors_deinit(void) {
    if (!g_initialized) {
        return;
    }
    
    /* Cleanup GStreamer */
    gst_deinit();
    
    g_initialized = false;
}

const char *sensors_version_string(void) {
    static char version[32];
    snprintf(version, sizeof(version), "%d.%d.%d",
             SENSORS_VERSION_MAJOR,
             SENSORS_VERSION_MINOR,
             SENSORS_VERSION_PATCH);
    return version;
}

/* ============================================================================
 * Driver Factory Functions
 * These would normally be auto-generated or expanded with full implementations
 * ============================================================================ */

/* Camera drivers */
sensor_base_t *imx477_create(const char *name) {
    sensor_base_t *sensor = malloc(sizeof(sensor_base_t));
    if (!sensor) return NULL;
    
    sensor_base_init(sensor, NULL, name);
    sensor->info.category = SENSOR_CATEGORY_CAMERA;
    strncpy(sensor->info.model, "IMX477", sizeof(sensor->info.model) - 1);
    
    return sensor;
}

sensor_base_t *imx219_create(const char *name) {
    sensor_base_t *sensor = malloc(sizeof(sensor_base_t));
    if (!sensor) return NULL;
    
    sensor_base_init(sensor, NULL, name);
    sensor->info.category = SENSOR_CATEGORY_CAMERA;
    strncpy(sensor->info.model, "IMX219", sizeof(sensor->info.model) - 1);
    
    return sensor;
}

/* GNSS drivers */
sensor_base_t *ublox_create(const char *name) {
    sensor_base_t *sensor = malloc(sizeof(sensor_base_t));
    if (!sensor) return NULL;
    
    sensor_base_init(sensor, NULL, name);
    sensor->info.category = SENSOR_CATEGORY_GNSS;
    strncpy(sensor->info.model, "ZED-F9P", sizeof(sensor->info.model) - 1);
    
    return sensor;
}

/* IMU drivers */
sensor_base_t *icm20948_create(const char *name) {
    sensor_base_t *sensor = malloc(sizeof(sensor_base_t));
    if (!sensor) return NULL;
    
    sensor_base_init(sensor, NULL, name);
    sensor->info.category = SENSOR_CATEGORY_IMU;
    strncpy(sensor->info.model, "ICM-20948", sizeof(sensor->info.model) - 1);
    
    return sensor;
}

sensor_base_t *bmi088_create(const char *name) {
    sensor_base_t *sensor = malloc(sizeof(sensor_base_t));
    if (!sensor) return NULL;
    
    sensor_base_init(sensor, NULL, name);
    sensor->info.category = SENSOR_CATEGORY_IMU;
    strncpy(sensor->info.model, "BMI088", sizeof(sensor->info.model) - 1);
    
    return sensor;
}

/* LiDAR drivers */
sensor_base_t *ouster_create(const char *name) {
    sensor_base_t *sensor = malloc(sizeof(sensor_base_t));
    if (!sensor) return NULL;
    
    sensor_base_init(sensor, NULL, name);
    sensor->info.category = SENSOR_CATEGORY_LIDAR;
    strncpy(sensor->info.model, "OS1", sizeof(sensor->info.model) - 1);
    
    return sensor;
}

sensor_base_t *velodyne_create(const char *name) {
    sensor_base_t *sensor = malloc(sizeof(sensor_base_t));
    if (!sensor) return NULL;
    
    sensor_base_init(sensor, NULL, name);
    sensor->info.category = SENSOR_CATEGORY_LIDAR;
    strncpy(sensor->info.model, "VLP-16", sizeof(sensor->info.model) - 1);
    
    return sensor;
}

/* SATCOM */
sensor_base_t *iridium_create(const char *name) {
    sensor_base_t *sensor = malloc(sizeof(sensor_base_t));
    if (!sensor) return NULL;
    
    sensor_base_init(sensor, NULL, name);
    sensor->info.category = SENSOR_CATEGORY_SATCOM;
    strncpy(sensor->info.model, "9602/9603", sizeof(sensor->info.model) - 1);
    
    return sensor;
}

/* Telemetry */
sensor_base_t *rfd900_create(const char *name) {
    sensor_base_t *sensor = malloc(sizeof(sensor_base_t));
    if (!sensor) return NULL;
    
    sensor_base_init(sensor, NULL, name);
    sensor->info.category = SENSOR_CATEGORY_TELEMETRY;
    strncpy(sensor->info.model, "RFD900x", sizeof(sensor->info.model) - 1);
    
    return sensor;
}

sensor_base_t *sik_create(const char *name) {
    sensor_base_t *sensor = malloc(sizeof(sensor_base_t));
    if (!sensor) return NULL;
    
    sensor_base_init(sensor, NULL, name);
    sensor->info.category = SENSOR_CATEGORY_TELEMETRY;
    strncpy(sensor->info.model, "SiK", sizeof(sensor->info.model) - 1);
    
    return sensor;
}

/* Environmental sensors */
sensor_base_t *ms5611_create(const char *name) {
    sensor_base_t *sensor = malloc(sizeof(sensor_base_t));
    if (!sensor) return NULL;
    
    sensor_base_init(sensor, NULL, name);
    sensor->info.category = SENSOR_CATEGORY_ENVIRONMENT;
    strncpy(sensor->info.model, "MS5611", sizeof(sensor->info.model) - 1);
    
    return sensor;
}

sensor_base_t *bmp388_create(const char *name) {
    sensor_base_t *sensor = malloc(sizeof(sensor_base_t));
    if (!sensor) return NULL;
    
    sensor_base_init(sensor, NULL, name);
    sensor->info.category = SENSOR_CATEGORY_ENVIRONMENT;
    strncpy(sensor->info.model, "BMP388", sizeof(sensor->info.model) - 1);
    
    return sensor;
}

sensor_base_t *lis3mdl_create(const char *name) {
    sensor_base_t *sensor = malloc(sizeof(sensor_base_t));
    if (!sensor) return NULL;
    
    sensor_base_init(sensor, NULL, name);
    sensor->info.category = SENSOR_CATEGORY_ENVIRONMENT;
    strncpy(sensor->info.model, "LIS3MDL", sizeof(sensor->info.model) - 1);
    
    return sensor;
}

sensor_base_t *rm3100_create(const char *name) {
    sensor_base_t *sensor = malloc(sizeof(sensor_base_t));
    if (!sensor) return NULL;
    
    sensor_base_init(sensor, NULL, name);
    sensor->info.category = SENSOR_CATEGORY_ENVIRONMENT;
    strncpy(sensor->info.model, "RM3100", sizeof(sensor->info.model) - 1);
    
    return sensor;
}

sensor_base_t *ms4525_create(const char *name) {
    sensor_base_t *sensor = malloc(sizeof(sensor_base_t));
    if (!sensor) return NULL;
    
    sensor_base_init(sensor, NULL, name);
    sensor->info.category = SENSOR_CATEGORY_ENVIRONMENT;
    strncpy(sensor->info.model, "MS4525", sizeof(sensor->info.model) - 1);
    
    return sensor;
}

sensor_base_t *sdp3x_create(const char *name) {
    sensor_base_t *sensor = malloc(sizeof(sensor_base_t));
    if (!sensor) return NULL;
    
    sensor_base_init(sensor, NULL, name);
    sensor->info.category = SENSOR_CATEGORY_ENVIRONMENT;
    strncpy(sensor->info.model, "SDP3x", sizeof(sensor->info.model) - 1);
    
    return sensor;
}
