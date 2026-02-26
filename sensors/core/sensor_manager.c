/*
 * Unified Sensor Manager Implementation
 * Coordinates multiple sensors with synchronization
 */

#include "sensor_manager.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static void *health_monitor_thread(void *arg);
static void *sync_thread(void *arg);

sensor_manager_t *sensor_manager_create(void) {
    sensor_manager_t *mgr = calloc(1, sizeof(sensor_manager_t));
    if (!mgr) {
        return NULL;
    }
    
    pthread_mutex_init(&mgr->lock, NULL);
    mgr->running = false;
    mgr->num_sensors = 0;
    mgr->epoch_ns = 0;
    
    return mgr;
}

void sensor_manager_destroy(sensor_manager_t *mgr) {
    if (!mgr) {
        return;
    }
    
    sensor_manager_stop_all(mgr);
    sensor_manager_stop_health_monitor(mgr);
    
    pthread_mutex_destroy(&mgr->lock);
    free(mgr);
}

sensor_error_t sensor_manager_init(sensor_manager_t *mgr) {
    if (!mgr) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    pthread_mutex_lock(&mgr->lock);
    mgr->epoch_ns = 0;
    mgr->time_offset_ns = 0;
    pthread_mutex_unlock(&mgr->lock);
    
    return SENSOR_OK;
}

sensor_error_t sensor_manager_register(sensor_manager_t *mgr,
                                        sensor_base_t *sensor,
                                        sensor_group_t group,
                                        bool auto_start) {
    if (!mgr || !sensor) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    pthread_mutex_lock(&mgr->lock);
    
    if (mgr->num_sensors >= SENSOR_MANAGER_MAX_SENSORS) {
        pthread_mutex_unlock(&mgr->lock);
        return SENSOR_ERROR_OVERFLOW;
    }
    
    /* Check for duplicate names */
    for (uint32_t i = 0; i < mgr->num_sensors; i++) {
        if (strcmp(mgr->sensors[i].sensor->info.name, sensor->info.name) == 0) {
            pthread_mutex_unlock(&mgr->lock);
            return SENSOR_ERROR_INVALID_PARAM;
        }
    }
    
    /* Add to list */
    sensor_entry_t *entry = &mgr->sensors[mgr->num_sensors];
    entry->sensor = sensor;
    entry->group = group;
    entry->auto_start = auto_start;
    entry->health_critical = false;
    entry->watchdog_timeout_ms = 1000;
    entry->priority = 0;
    
    mgr->num_sensors++;
    
    pthread_mutex_unlock(&mgr->lock);
    
    if (auto_start) {
        return sensor_manager_start_group(mgr, group);
    }
    
    return SENSOR_OK;
}

sensor_error_t sensor_manager_unregister(sensor_manager_t *mgr, const char *sensor_name) {
    if (!mgr || !sensor_name) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    pthread_mutex_lock(&mgr->lock);
    
    for (uint32_t i = 0; i < mgr->num_sensors; i++) {
        if (strcmp(mgr->sensors[i].sensor->info.name, sensor_name) == 0) {
            /* Stop sensor if running */
            sensor_base_stop_capture(mgr->sensors[i].sensor);
            
            /* Remove from list by shifting */
            for (uint32_t j = i; j < mgr->num_sensors - 1; j++) {
                mgr->sensors[j] = mgr->sensors[j + 1];
            }
            mgr->num_sensors--;
            
            pthread_mutex_unlock(&mgr->lock);
            return SENSOR_OK;
        }
    }
    
    pthread_mutex_unlock(&mgr->lock);
    return SENSOR_ERROR_INVALID_PARAM;
}

sensor_base_t *sensor_manager_find(sensor_manager_t *mgr, const char *name) {
    if (!mgr || !name) {
        return NULL;
    }
    
    pthread_mutex_lock(&mgr->lock);
    
    for (uint32_t i = 0; i < mgr->num_sensors; i++) {
        if (strcmp(mgr->sensors[i].sensor->info.name, name) == 0) {
            sensor_base_t *result = mgr->sensors[i].sensor;
            pthread_mutex_unlock(&mgr->lock);
            return result;
        }
    }
    
    pthread_mutex_unlock(&mgr->lock);
    return NULL;
}

sensor_error_t sensor_manager_start_group(sensor_manager_t *mgr, sensor_group_t group) {
    if (!mgr) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    pthread_mutex_lock(&mgr->lock);
    
    for (uint32_t i = 0; i < mgr->num_sensors; i++) {
        if (mgr->sensors[i].group == group) {
            sensor_base_start_capture(mgr->sensors[i].sensor, 10);
        }
    }
    
    pthread_mutex_unlock(&mgr->lock);
    return SENSOR_OK;
}

sensor_error_t sensor_manager_stop_group(sensor_manager_t *mgr, sensor_group_t group) {
    if (!mgr) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    pthread_mutex_lock(&mgr->lock);
    
    for (uint32_t i = 0; i < mgr->num_sensors; i++) {
        if (mgr->sensors[i].group == group) {
            sensor_base_stop_capture(mgr->sensors[i].sensor);
        }
    }
    
    pthread_mutex_unlock(&mgr->lock);
    return SENSOR_OK;
}

sensor_error_t sensor_manager_start_all(sensor_manager_t *mgr) {
    if (!mgr) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    pthread_mutex_lock(&mgr->lock);
    mgr->running = true;
    
    for (uint32_t i = 0; i < mgr->num_sensors; i++) {
        if (mgr->sensors[i].auto_start) {
            sensor_base_start_capture(mgr->sensors[i].sensor, 10);
        }
    }
    
    pthread_mutex_unlock(&mgr->lock);
    return SENSOR_OK;
}

sensor_error_t sensor_manager_stop_all(sensor_manager_t *mgr) {
    if (!mgr) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    pthread_mutex_lock(&mgr->lock);
    mgr->running = false;
    
    for (uint32_t i = 0; i < mgr->num_sensors; i++) {
        sensor_base_stop_capture(mgr->sensors[i].sensor);
    }
    
    pthread_mutex_unlock(&mgr->lock);
    return SENSOR_OK;
}

sensor_error_t sensor_manager_calibrate_all(sensor_manager_t *mgr) {
    if (!mgr) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    pthread_mutex_lock(&mgr->lock);
    
    for (uint32_t i = 0; i < mgr->num_sensors; i++) {
        sensor_base_t *sensor = mgr->sensors[i].sensor;
        if (sensor->ops && sensor->ops->calibrate) {
            sensor->ops->calibrate(sensor);
        }
    }
    
    pthread_mutex_unlock(&mgr->lock);
    return SENSOR_OK;
}

sensor_error_t sensor_manager_get_stats(sensor_manager_t *mgr, sensor_manager_stats_t *stats) {
    if (!mgr || !stats) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    memset(stats, 0, sizeof(sensor_manager_stats_t));
    
    pthread_mutex_lock(&mgr->lock);
    
    stats->total_sensors = mgr->num_sensors;
    
    for (uint32_t i = 0; i < mgr->num_sensors; i++) {
        sensor_base_t *sensor = mgr->sensors[i].sensor;
        
        if (sensor->state == SENSOR_STATE_ACTIVE) {
            stats->active_sensors++;
        }
        
        stats->total_samples += sensor->samples_received;
        stats->buffer_overflows += sensor->samples_dropped;
    }
    
    pthread_mutex_unlock(&mgr->lock);
    return SENSOR_OK;
}

static void *health_monitor_thread(void *arg) {
    sensor_manager_t *mgr = (sensor_manager_t *)arg;
    
    while (mgr->running) {
        pthread_mutex_lock(&mgr->lock);
        
        for (uint32_t i = 0; i < mgr->num_sensors; i++) {
            sensor_entry_t *entry = &mgr->sensors[i];
            sensor_base_t *sensor = entry->sensor;
            
            if (entry->health_critical && sensor->state == SENSOR_STATE_ERROR) {
                if (mgr->on_sensor_error) {
                    mgr->on_sensor_error(sensor->info.name, SENSOR_ERROR_IO);
                }
            }
            
            if (sensor->ops && sensor->ops->health_check) {
                sensor_error_t err = sensor->ops->health_check(sensor);
                if (err != SENSOR_OK) {
                    sensor->error_count++;
                }
            }
        }
        
        pthread_mutex_unlock(&mgr->lock);
        usleep(100000);  /* 100ms interval */
    }
    
    return NULL;
}

static void *sync_thread(void *arg) {
    sensor_manager_t *mgr = (sensor_manager_t *)arg;
    
    /* PPS sync implementation would go here */
    (void)mgr;
    
    return NULL;
}
