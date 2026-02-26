/*
 * Sensor Base Interface
 * Abstract base class for all sensor drivers
 */

#ifndef SENSOR_BASE_H
#define SENSOR_BASE_H

#include "sensor_types.h"
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct sensor_base;
typedef struct sensor_base sensor_base_t;

/* Virtual function table for sensor operations */
typedef struct {
    /* Initialize the sensor with configuration */
    sensor_error_t (*init)(sensor_base_t *sensor, void *config);
    
    /* Start data acquisition */
    sensor_error_t (*start)(sensor_base_t *sensor);
    
    /* Stop data acquisition */
    sensor_error_t (*stop)(sensor_base_t *sensor);
    
    /* Calibrate the sensor */
    sensor_error_t (*calibrate)(sensor_base_t *sensor);
    
    /* Get latest data (non-blocking) */
    sensor_error_t (*get_data)(sensor_base_t *sensor, void *buffer, size_t buffer_size, size_t *data_size);
    
    /* Configure sensor parameters */
    sensor_error_t (*configure)(sensor_base_t *sensor, const char *param, void *value);
    
    /* Get sensor information */
    sensor_error_t (*get_info)(sensor_base_t *sensor, sensor_info_t *info);
    
    /* Check sensor health */
    sensor_error_t (*health_check)(sensor_base_t *sensor);
    
    /* Shutdown and cleanup */
    void (*shutdown)(sensor_base_t *sensor);
} sensor_ops_t;

/* Ring buffer for sensor data */
typedef struct {
    uint8_t *buffer;
    size_t capacity;
    size_t head;
    size_t tail;
    size_t element_size;
    pthread_mutex_t mutex;
    pthread_cond_t not_empty;
    pthread_cond_t not_full;
    uint32_t overflow_count;
} sensor_ringbuffer_t;

/* Base sensor structure */
struct sensor_base {
    /* Public - read-only after init */
    sensor_info_t info;
    sensor_state_t state;
    
    /* Protected - internal use */
    sensor_ops_t *ops;
    sensor_ringbuffer_t *data_buffer;
    
    /* Threading */
    pthread_t worker_thread;
    bool worker_running;
    
    /* Callbacks */
    sensor_data_callback_t data_callback;
    void *data_callback_userdata;
    sensor_status_callback_t status_callback;
    void *status_callback_userdata;
    
    /* Statistics */
    uint64_t samples_received;
    uint64_t samples_dropped;
    uint64_t error_count;
    
    /* Private - implementation specific */
    void *private_data;
};

/* Base sensor API */
sensor_error_t sensor_base_init(sensor_base_t *sensor, sensor_ops_t *ops, const char *name);
sensor_error_t sensor_base_start_capture(sensor_base_t *sensor, uint32_t buffer_count);
sensor_error_t sensor_base_stop_capture(sensor_base_t *sensor);
sensor_error_t sensor_base_set_callback(sensor_base_t *sensor, sensor_data_callback_t cb, void *userdata);
sensor_error_t sensor_base_set_status_callback(sensor_base_t *sensor, sensor_status_callback_t cb, void *userdata);
void sensor_base_set_state(sensor_base_t *sensor, sensor_state_t new_state);
sensor_error_t sensor_base_push_data(sensor_base_t *sensor, void *data, size_t size, sensor_timestamp_t *ts);
sensor_error_t sensor_base_get_data(sensor_base_t *sensor, void *buffer, size_t size, size_t *received, int timeout_ms);
void sensor_base_shutdown(sensor_base_t *sensor);

/* Ring buffer operations */
sensor_ringbuffer_t *ringbuffer_create(size_t element_size, size_t count);
void ringbuffer_destroy(sensor_ringbuffer_t *rb);
sensor_error_t ringbuffer_push(sensor_ringbuffer_t *rb, void *data);
sensor_error_t ringbuffer_pop(sensor_ringbuffer_t *rb, void *data, int timeout_ms);
void ringbuffer_clear(sensor_ringbuffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_BASE_H */
