/*
 * Sensor Base Implementation
 * Common functionality for all sensor drivers
 */

#include "sensor_base.h"
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>

/* Get current timestamp */
static void get_timestamp(sensor_timestamp_t *ts) {
    struct timespec now;
    
    clock_gettime(CLOCK_MONOTONIC, &now);
    ts->nanoseconds = (uint64_t)now.tv_sec * 1000000000ULL + now.tv_nsec;
    
    clock_gettime(CLOCK_REALTIME, &now);
    ts->realtime_ns = (uint64_t)now.tv_sec * 1000000000ULL + now.tv_nsec;
}

/* Initialize base sensor */
sensor_error_t sensor_base_init(sensor_base_t *sensor, sensor_ops_t *ops, const char *name) {
    if (!sensor || !ops || !name) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    memset(sensor, 0, sizeof(sensor_base_t));
    sensor->ops = ops;
    sensor->state = SENSOR_STATE_UNINITIALIZED;
    
    strncpy(sensor->info.name, name, sizeof(sensor->info.name) - 1);
    sensor->info.name[sizeof(sensor->info.name) - 1] = '\0';
    
    return SENSOR_OK;
}

/* Start data capture thread */
sensor_error_t sensor_base_start_capture(sensor_base_t *sensor, uint32_t buffer_count) {
    if (!sensor || !sensor->ops) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    if (sensor->state == SENSOR_STATE_ACTIVE) {
        return SENSOR_OK;  /* Already running */
    }
    
    if (!sensor->data_buffer && buffer_count > 0) {
        /* Buffer size will be set by derived class */
        sensor->data_buffer = ringbuffer_create(1024, buffer_count);
        if (!sensor->data_buffer) {
            return SENSOR_ERROR_INIT;
        }
    }
    
    if (sensor->ops->start) {
        sensor_error_t err = sensor->ops->start(sensor);
        if (err != SENSOR_OK) {
            return err;
        }
    }
    
    sensor_base_set_state(sensor, SENSOR_STATE_ACTIVE);
    return SENSOR_OK;
}

/* Stop data capture */
sensor_error_t sensor_base_stop_capture(sensor_base_t *sensor) {
    if (!sensor) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    sensor->worker_running = false;
    
    if (sensor->worker_thread) {
        pthread_join(sensor->worker_thread, NULL);
        sensor->worker_thread = 0;
    }
    
    if (sensor->ops && sensor->ops->stop) {
        sensor->ops->stop(sensor);
    }
    
    sensor_base_set_state(sensor, SENSOR_STATE_STANDBY);
    return SENSOR_OK;
}

/* Set data callback */
sensor_error_t sensor_base_set_callback(sensor_base_t *sensor, sensor_data_callback_t cb, void *userdata) {
    if (!sensor) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    sensor->data_callback = cb;
    sensor->data_callback_userdata = userdata;
    return SENSOR_OK;
}

/* Set status callback */
sensor_error_t sensor_base_set_status_callback(sensor_base_t *sensor, sensor_status_callback_t cb, void *userdata) {
    if (!sensor) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    sensor->status_callback = cb;
    sensor->status_callback_userdata = userdata;
    return SENSOR_OK;
}

/* Set sensor state with callback notification */
void sensor_base_set_state(sensor_base_t *sensor, sensor_state_t new_state) {
    if (!sensor || sensor->state == new_state) {
        return;
    }
    
    sensor_state_t old_state = sensor->state;
    sensor->state = new_state;
    
    if (sensor->status_callback) {
        sensor->status_callback(sensor->status_callback_userdata, old_state, new_state);
    }
}

/* Push data to ring buffer and callback */
sensor_error_t sensor_base_push_data(sensor_base_t *sensor, void *data, size_t size, sensor_timestamp_t *ts) {
    if (!sensor || !data) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    sensor_timestamp_t timestamp;
    if (!ts) {
        get_timestamp(&timestamp);
        ts = &timestamp;
    }
    
    /* Push to ring buffer if available */
    if (sensor->data_buffer) {
        if (ringbuffer_push(sensor->data_buffer, data) != SENSOR_OK) {
            sensor->samples_dropped++;
            return SENSOR_ERROR_OVERFLOW;
        }
    }
    
    /* Call registered callback */
    if (sensor->data_callback) {
        sensor->data_callback(sensor->data_callback_userdata, data, size, ts);
    }
    
    sensor->samples_received++;
    return SENSOR_OK;
}

/* Get data from ring buffer */
sensor_error_t sensor_base_get_data(sensor_base_t *sensor, void *buffer, size_t size, 
                                     size_t *received, int timeout_ms) {
    if (!sensor || !buffer || !received) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    *received = 0;
    
    if (!sensor->data_buffer) {
        return SENSOR_ERROR_NOT_SUPPORTED;
    }
    
    return ringbuffer_pop(sensor->data_buffer, buffer, timeout_ms);
}

/* Shutdown sensor */
void sensor_base_shutdown(sensor_base_t *sensor) {
    if (!sensor) {
        return;
    }
    
    sensor_base_stop_capture(sensor);
    
    if (sensor->ops && sensor->ops->shutdown) {
        sensor->ops->shutdown(sensor);
    }
    
    if (sensor->data_buffer) {
        ringbuffer_destroy(sensor->data_buffer);
        sensor->data_buffer = NULL;
    }
    
    sensor_base_set_state(sensor, SENSOR_STATE_SHUTDOWN);
}

/* ============================================================================
 * Ring Buffer Implementation
 * ============================================================================ */

sensor_ringbuffer_t *ringbuffer_create(size_t element_size, size_t count) {
    sensor_ringbuffer_t *rb = calloc(1, sizeof(sensor_ringbuffer_t));
    if (!rb) {
        return NULL;
    }
    
    rb->capacity = count;
    rb->element_size = element_size;
    rb->head = 0;
    rb->tail = 0;
    
    rb->buffer = calloc(count, element_size);
    if (!rb->buffer) {
        free(rb);
        return NULL;
    }
    
    pthread_mutex_init(&rb->mutex, NULL);
    pthread_cond_init(&rb->not_empty, NULL);
    pthread_cond_init(&rb->not_full, NULL);
    
    return rb;
}

void ringbuffer_destroy(sensor_ringbuffer_t *rb) {
    if (!rb) {
        return;
    }
    
    pthread_mutex_destroy(&rb->mutex);
    pthread_cond_destroy(&rb->not_empty);
    pthread_cond_destroy(&rb->not_full);
    
    free(rb->buffer);
    free(rb);
}

sensor_error_t ringbuffer_push(sensor_ringbuffer_t *rb, void *data) {
    if (!rb || !data) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    pthread_mutex_lock(&rb->mutex);
    
    size_t next_head = (rb->head + 1) % rb->capacity;
    if (next_head == rb->tail) {
        /* Buffer full - overwrite oldest data */
        rb->tail = (rb->tail + 1) % rb->capacity;
        rb->overflow_count++;
    }
    
    memcpy(rb->buffer + rb->head * rb->element_size, data, rb->element_size);
    rb->head = next_head;
    
    pthread_cond_signal(&rb->not_empty);
    pthread_mutex_unlock(&rb->mutex);
    
    return SENSOR_OK;
}

sensor_error_t ringbuffer_pop(sensor_ringbuffer_t *rb, void *data, int timeout_ms) {
    if (!rb || !data) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    pthread_mutex_lock(&rb->mutex);
    
    /* Wait for data if buffer is empty */
    if (rb->head == rb->tail) {
        if (timeout_ms < 0) {
            /* Block indefinitely */
            pthread_cond_wait(&rb->not_empty, &rb->mutex);
        } else if (timeout_ms == 0) {
            /* Non-blocking */
            pthread_mutex_unlock(&rb->mutex);
            return SENSOR_ERROR_BUSY;
        } else {
            /* Timed wait */
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += timeout_ms / 1000;
            ts.tv_nsec += (timeout_ms % 1000) * 1000000;
            
            if (ts.tv_nsec >= 1000000000) {
                ts.tv_sec++;
                ts.tv_nsec -= 1000000000;
            }
            
            int ret = pthread_cond_timedwait(&rb->not_empty, &rb->mutex, &ts);
            if (ret == ETIMEDOUT) {
                pthread_mutex_unlock(&rb->mutex);
                return SENSOR_ERROR_TIMEOUT;
            }
        }
    }
    
    /* Check again after wait */
    if (rb->head == rb->tail) {
        pthread_mutex_unlock(&rb->mutex);
        return SENSOR_ERROR_BUSY;
    }
    
    memcpy(data, rb->buffer + rb->tail * rb->element_size, rb->element_size);
    rb->tail = (rb->tail + 1) % rb->capacity;
    
    pthread_cond_signal(&rb->not_full);
    pthread_mutex_unlock(&rb->mutex);
    
    return SENSOR_OK;
}

void ringbuffer_clear(sensor_ringbuffer_t *rb) {
    if (!rb) {
        return;
    }
    
    pthread_mutex_lock(&rb->mutex);
    rb->head = 0;
    rb->tail = 0;
    rb->overflow_count = 0;
    pthread_mutex_unlock(&rb->mutex);
}
