/*
 * Unified Sensor Manager
 * Central coordination for all vehicle sensors
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "sensor_base.h"
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_MANAGER_MAX_SENSORS 32
#define SENSOR_MANAGER_MAX_GROUPS 8

typedef enum {
    SENSOR_GROUP_CAMERA,
    SENSOR_GROUP_GNSS,
    SENSOR_GROUP_IMU,
    SENSOR_GROUP_LIDAR,
    SENSOR_GROUP_SATCOM,
    SENSOR_GROUP_TELEMETRY,
    SENSOR_GROUP_ENVIRONMENT,
    SENSOR_GROUP_CUSTOM
} sensor_group_t;

typedef struct {
    sensor_base_t *sensor;
    sensor_group_t group;
    bool auto_start;
    bool health_critical;
    uint32_t watchdog_timeout_ms;
    uint32_t priority;
} sensor_entry_t;

typedef struct {
    sensor_entry_t sensors[SENSOR_MANAGER_MAX_SENSORS];
    uint32_t num_sensors;
    pthread_mutex_t lock;
    bool running;
    pthread_t health_thread;
    pthread_t sync_thread;
    
    /* Global timestamp reference */
    uint64_t epoch_ns;
    int64_t time_offset_ns;
    
    /* Synchronization */
    bool pps_enabled;
    int pps_gpio;
    uint64_t pps_count;
    
    /* Callbacks */
    void (*on_sensor_error)(const char *name, sensor_error_t err);
    void (*on_sync_pulse)(uint64_t timestamp_ns);
} sensor_manager_t;

typedef struct {
    uint32_t total_sensors;
    uint32_t active_sensors;
    uint32_t error_count;
    uint64_t total_samples;
    uint32_t buffer_overflows;
    float sync_accuracy_ns;
} sensor_manager_stats_t;

/* Manager lifecycle */
sensor_manager_t *sensor_manager_create(void);
void sensor_manager_destroy(sensor_manager_t *mgr);
sensor_error_t sensor_manager_init(sensor_manager_t *mgr);

/* Sensor registration */
sensor_error_t sensor_manager_register(sensor_manager_t *mgr, 
                                        sensor_base_t *sensor,
                                        sensor_group_t group,
                                        bool auto_start);
sensor_error_t sensor_manager_unregister(sensor_manager_t *mgr, 
                                          const char *sensor_name);
sensor_base_t *sensor_manager_find(sensor_manager_t *mgr, const char *name);

/* Group operations */
sensor_error_t sensor_manager_start_group(sensor_manager_t *mgr, sensor_group_t group);
sensor_error_t sensor_manager_stop_group(sensor_manager_t *mgr, sensor_group_t group);
sensor_error_t sensor_manager_get_group_stats(sensor_manager_t *mgr, sensor_group_t group,
                                               uint32_t *active, uint32_t *total);

/* Global control */
sensor_error_t sensor_manager_start_all(sensor_manager_t *mgr);
sensor_error_t sensor_manager_stop_all(sensor_manager_t *mgr);
sensor_error_t sensor_manager_calibrate_all(sensor_manager_t *mgr);

/* Synchronization */
sensor_error_t sensor_manager_enable_pps_sync(sensor_manager_t *mgr, int gpio);
sensor_error_t sensor_manager_software_sync(sensor_manager_t *mgr);
sensor_error_t sensor_manager_get_timestamp(sensor_manager_t *mgr, sensor_timestamp_t *ts);

/* Health monitoring */
sensor_error_t sensor_manager_start_health_monitor(sensor_manager_t *mgr);
sensor_error_t sensor_manager_stop_health_monitor(sensor_manager_t *mgr);
sensor_error_t sensor_manager_get_health(sensor_manager_t *mgr, 
                                          const char *name, 
                                          bool *healthy);
sensor_error_t sensor_manager_get_stats(sensor_manager_t *mgr, 
                                         sensor_manager_stats_t *stats);

/* Configuration */
sensor_error_t sensor_manager_load_config(sensor_manager_t *mgr, const char *filename);
sensor_error_t sensor_manager_save_config(sensor_manager_t *mgr, const char *filename);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_MANAGER_H */
