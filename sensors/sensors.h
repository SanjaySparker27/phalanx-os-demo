/*
 * Unified Sensor Library Master Header
 * Include this to access all sensor drivers
 */

#ifndef SENSORS_H
#define SENSORS_H

/* Core abstraction layer */
#include "core/sensor_types.h"
#include "core/sensor_base.h"
#include "core/sensor_manager.h"

/* Camera drivers */
#include "cameras/camera_imx477.h"
#include "cameras/camera_imx219.h"
#include "cameras/gstreamer_pipeline.h"

/* GNSS drivers */
#include "gnss/gnss_ublox.h"

/* IMU drivers */
#include "imu/imu_icm20948.h"
#include "imu/imu_bmi088.h"

/* LiDAR drivers */
#include "lidar/lidar_ouster.h"
#include "lidar/lidar_velodyne.h"

/* SATCOM */
#include "satcom/satcom_iridium.h"

/* Telemetry radios */
#include "telemetry/telemetry_rfd900.h"
#include "telemetry/telemetry_sik.h"

/* Environmental sensors */
#include "environment/baro_ms5611.h"
#include "environment/baro_bmp388.h"
#include "environment/mag_lis3mdl.h"
#include "environment/mag_rm3100.h"
#include "environment/airspeed_ms4525.h"
#include "environment/airspeed_sdp3x.h"

/* Version information */
#define SENSORS_VERSION_MAJOR 1
#define SENSORS_VERSION_MINOR 0
#define SENSORS_VERSION_PATCH 0

#ifdef __cplusplus
extern "C" {
#endif

/* Library initialization */
sensor_error_t sensors_init(void);
void sensors_deinit(void);
const char *sensors_version_string(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_H */
