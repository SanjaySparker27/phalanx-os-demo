/*
 * Ainstein Radar Driver
 * UART-based automotive radar with CAN fallback
 * Supports K-79, US-D1, and US-D2 models
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>
#include <math.h>

#define AINSTEIN_DEFAULT_BAUD   115200
#define AINSTEIN_MAX_TARGETS    20
#define AINSTEIN_FRAME_HEADER   0xAA
#define AINSTEIN_MAX_RETRY      5
#define AINSTEIN_HEALTH_TIMEOUT 1000

typedef enum {
    AINSTEIN_MODEL_K79,
    AINSTEIN_MODEL_US_D1,
    AINSTEIN_MODEL_US_D2
} ainstien_model_t;

typedef enum {
    AINSTEIN_STATE_DISCONNECTED,
    AINSTEIN_STATE_CONNECTING,
    AINSTEIN_STATE_CONNECTED,
    AINSTEIN_STATE_ERROR
} ainstein_state_t;

typedef struct {
    uint16_t target_id;
    float range_m;
    float velocity_m_s;
    float azimuth_deg;
    float elevation_deg;
    float snr_db;
    float rcs_dbsm;
    bool valid;
} ainstein_target_t;

typedef struct {
    ainstein_target_t targets[AINSTEIN_MAX_TARGETS];
    uint8_t num_targets;
    uint64_t timestamp_ns;
    uint16_t frame_counter;
} ainstein_frame_t;

typedef struct {
    char uart_device[64];
    uint32_t baud_rate;
    ainstien_model_t model;
    float max_range_m;
    float min_range_m;
    float max_velocity_m_s;
    float min_snr_db;
    uint8_t output_rate_hz;
} ainstein_config_t;

typedef struct {
    int uart_fd;
    struct termios old_tio;
    
    ainstein_state_t state;
    ainstein_config_t config;
    
    pthread_t rx_thread;
    pthread_t health_thread;
    pthread_mutex_t mutex;
    volatile bool running;
    
    /* Hotplug detection */
    time_t last_frame_time;
    int missed_frames;
    bool hotplug_enabled;
    
    /* Health monitoring */
    uint32_t frames_received;
    uint32_t frames_dropped;
    uint32_t crc_errors;
    uint32_t reconnect_count;
    
    /* RX buffer */
    uint8_t rx_buffer[512];
    size_t rx_len;
    
    /* Callbacks */
    void (*on_frame)(ainstein_frame_t *frame, void *userdata);
    void (*on_health_change)(bool healthy, void *userdata);
    void *userdata;
} ainstein_handle_t;

static uint64_t get_timestamp_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

void ainstein_default_config(ainstein_config_t *cfg) {
    memset(cfg, 0, sizeof(ainstein_config_t));
    strcpy(cfg->uart_device, "/dev/ttyUSB0");
    cfg->baud_rate = AINSTEIN_DEFAULT_BAUD;
    cfg->model = AINSTEIN_MODEL_K79;
    cfg->max_range_m = 150.0f;
    cfg->min_range_m = 0.5f;
    cfg->max_velocity_m_s = 55.0f;
    cfg->min_snr_db = 10.0f;
    cfg->output_rate_hz = 20;
}

ainstein_handle_t *ainstein_create(const char *name) {
    (void)name;
    ainstein_handle_t *handle = calloc(1, sizeof(ainstein_handle_t));
    if (!handle) return NULL;
    
    ainstein_default_config(&handle->config);
    pthread_mutex_init(&handle->mutex, NULL);
    handle->state = AINSTEIN_STATE_DISCONNECTED;
    handle->hotplug_enabled = true;
    handle->uart_fd = -1;
    
    return handle;
}

static int ainstein_open_uart(ainstein_handle_t *handle) {
    handle->uart_fd = open(handle->config.uart_device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (handle->uart_fd < 0) {
        return -1;
    }
    
    fcntl(handle->uart_fd, F_SETFL, 0);
    
    struct termios tio;
    tcgetattr(handle->uart_fd, &tio);
    handle->old_tio = tio;
    
    cfsetispeed(&tio, handle->config.baud_rate);
    cfsetospeed(&tio, handle->config.baud_rate);
    
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag |= CREAD | CLOCAL;
    
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_oflag &= ~OPOST;
    
    tcsetattr(handle->uart_fd, TCSANOW, &tio);
    tcflush(handle->uart_fd, TCIOFLUSH);
    
    return 0;
}

static void ainstein_close_uart(ainstein_handle_t *handle) {
    if (handle->uart_fd >= 0) {
        tcsetattr(handle->uart_fd, TCSANOW, &handle->old_tio);
        close(handle->uart_fd);
        handle->uart_fd = -1;
    }
}

static uint16_t ainstein_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static int parse_k79_frame(uint8_t *data, size_t len, ainstein_frame_t *frame) {
    if (len < 6 || data[0] != AINSTEIN_FRAME_HEADER) return -1;
    
    uint16_t payload_len = (data[2] << 8) | data[3];
    if (len < payload_len + 6) return -1;
    
    /* Verify CRC */
    uint16_t rx_crc = (data[len-2] << 8) | data[len-1];
    uint16_t calc_crc = ainstein_crc16(data, len-2);
    if (rx_crc != calc_crc) return -1;
    
    uint8_t msg_id = data[1];
    uint8_t num_targets = data[4];
    
    frame->num_targets = (num_targets > AINSTEIN_MAX_TARGETS) ? AINSTEIN_MAX_TARGETS : num_targets;
    frame->timestamp_ns = get_timestamp_ns();
    frame->frame_counter = msg_id;
    
    for (int i = 0; i < frame->num_targets; i++) {
        size_t offset = 5 + i * 12;
        if (offset + 12 > len - 2) break;
        
        ainstein_target_t *t = &frame->targets[i];
        t->target_id = i;
        
        /* Range (0.01m units) */
        uint16_t range_raw = (data[offset] << 8) | data[offset+1];
        t->range_m = range_raw * 0.01f;
        
        /* Velocity (0.01 m/s units, signed) */
        int16_t vel_raw = (data[offset+2] << 8) | data[offset+3];
        t->velocity_m_s = vel_raw * 0.01f;
        
        /* Azimuth (0.01 deg units, signed) */
        int16_t az_raw = (data[offset+4] << 8) | data[offset+5];
        t->azimuth_deg = az_raw * 0.01f;
        
        /* Elevation (0.01 deg units, signed) */
        int16_t el_raw = (data[offset+6] << 8) | data[offset+7];
        t->elevation_deg = el_raw * 0.01f;
        
        /* SNR (0.01 dB units) */
        uint16_t snr_raw = (data[offset+8] << 8) | data[offset+9];
        t->snr_db = snr_raw * 0.01f;
        
        /* RCS (0.01 dBsm units, signed) */
        int16_t rcs_raw = (data[offset+10] << 8) | data[offset+11];
        t->rcs_dbsm = rcs_raw * 0.01f;
        
        t->valid = (t->snr_db > 5.0f);
    }
    
    return 0;
}

static void *ainstein_rx_thread(void *arg) {
    ainstein_handle_t *handle = (ainstein_handle_t *)arg;
    uint8_t rx_byte;
    ainstein_frame_t frame;
    
    while (handle->running) {
        ssize_t n = read(handle->uart_fd, &rx_byte, 1);
        
        if (n > 0) {
            if (handle->rx_len < sizeof(handle->rx_buffer)) {
                handle->rx_buffer[handle->rx_len++] = rx_byte;
                
                /* Look for frame header */
                if (handle->rx_len >= 6 && handle->rx_buffer[0] == AINSTEIN_FRAME_HEADER) {
                    uint16_t payload_len = (handle->rx_buffer[2] << 8) | handle->rx_buffer[3];
                    size_t frame_len = payload_len + 6;
                    
                    if (handle->rx_len >= frame_len) {
                        if (parse_k79_frame(handle->rx_buffer, frame_len, &frame) == 0) {
                            pthread_mutex_lock(&handle->mutex);
                            handle->last_frame_time = time(NULL);
                            handle->frames_received++;
                            pthread_mutex_unlock(&handle->mutex);
                            
                            if (handle->on_frame) {
                                handle->on_frame(&frame, handle->userdata);
                            }
                        } else {
                            pthread_mutex_lock(&handle->mutex);
                            handle->crc_errors++;
                            pthread_mutex_unlock(&handle->mutex);
                        }
                        
                        /* Shift remaining data */
                        memmove(handle->rx_buffer, handle->rx_buffer + frame_len, 
                                handle->rx_len - frame_len);
                        handle->rx_len -= frame_len;
                    }
                }
            } else {
                handle->rx_len = 0;
            }
        } else if (n < 0 && errno != EAGAIN) {
            usleep(1000);
        }
    }
    
    return NULL;
}

static void *ainstein_health_thread(void *arg) {
    ainstein_handle_t *handle = (ainstein_handle_t *)arg;
    bool was_healthy = false;
    
    while (handle->running) {
        sleep(1);
        
        if (!handle->hotplug_enabled) continue;
        
        pthread_mutex_lock(&handle->mutex);
        time_t now = time(NULL);
        double elapsed = difftime(now, handle->last_frame_time);
        bool is_healthy = (handle->state == AINSTEIN_STATE_CONNECTED) && 
                          (elapsed < AINSTEIN_HEALTH_TIMEOUT / 1000.0);
        pthread_mutex_unlock(&handle->mutex);
        
        if (is_healthy != was_healthy) {
            if (handle->on_health_change) {
                handle->on_health_change(is_healthy, handle->userdata);
            }
            was_healthy = is_healthy;
        }
        
        if (!is_healthy && handle->state == AINSTEIN_STATE_CONNECTED) {
            if (handle->reconnect_count < AINSTEIN_MAX_RETRY) {
                ainstein_stop(handle);
                sleep(1);
                if (ainstein_connect(handle) == 0) {
                    ainstein_start(handle);
                }
                handle->reconnect_count++;
            }
        }
        
        if (is_healthy && handle->reconnect_count > 0) {
            handle->reconnect_count = 0;
        }
    }
    
    return NULL;
}

int ainstein_connect(ainstein_handle_t *handle) {
    pthread_mutex_lock(&handle->mutex);
    
    handle->state = AINSTEIN_STATE_CONNECTING;
    
    if (ainstein_open_uart(handle) < 0) {
        handle->state = AINSTEIN_STATE_ERROR;
        pthread_mutex_unlock(&handle->mutex);
        return -1;
    }
    
    handle->state = AINSTEIN_STATE_CONNECTED;
    handle->last_frame_time = time(NULL);
    handle->rx_len = 0;
    
    pthread_mutex_unlock(&handle->mutex);
    return 0;
}

int ainstein_start(ainstein_handle_t *handle) {
    if (handle->state != AINSTEIN_STATE_CONNECTED) return -1;
    
    handle->running = true;
    pthread_create(&handle->rx_thread, NULL, ainstein_rx_thread, handle);
    pthread_create(&handle->health_thread, NULL, ainstein_health_thread, handle);
    
    return 0;
}

void ainstein_stop(ainstein_handle_t *handle) {
    handle->running = false;
    
    if (handle->rx_thread) {
        pthread_join(handle->rx_thread, NULL);
        handle->rx_thread = 0;
    }
    
    if (handle->health_thread) {
        pthread_join(handle->health_thread, NULL);
        handle->health_thread = 0;
    }
    
    ainstein_close_uart(handle);
    handle->state = AINSTEIN_STATE_DISCONNECTED;
}

void ainstein_destroy(ainstein_handle_t *handle) {
    if (!handle) return;
    
    ainstein_stop(handle);
    pthread_mutex_destroy(&handle->mutex);
    free(handle);
}

void ainstein_set_callbacks(ainstein_handle_t *handle,
                             void (*on_frame)(ainstein_frame_t *, void *),
                             void (*on_health)(bool, void *),
                             void *userdata) {
    handle->on_frame = on_frame;
    handle->on_health_change = on_health;
    handle->userdata = userdata;
}

void ainstein_set_hotplug(ainstein_handle_t *handle, bool enable) {
    handle->hotplug_enabled = enable;
}

int ainstein_get_health(ainstein_handle_t *handle, uint32_t *frames,
                         uint32_t *crc_err, uint32_t *dropped) {
    pthread_mutex_lock(&handle->mutex);
    
    if (frames) *frames = handle->frames_received;
    if (crc_err) *crc_err = handle->crc_errors;
    if (dropped) *dropped = handle->frames_dropped;
    
    pthread_mutex_unlock(&handle->mutex);
    return 0;
}

/* Configure radar parameters via UART commands */
int ainstein_set_output_rate(ainstein_handle_t *handle, uint8_t rate_hz) {
    if (handle->uart_fd < 0) return -1;
    
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "SET_OUTPUT_RATE %d\r\n", rate_hz);
    
    ssize_t n = write(handle->uart_fd, cmd, strlen(cmd));
    tcdrain(handle->uart_fd);
    
    return (n == (ssize_t)strlen(cmd)) ? 0 : -1;
}

int ainstein_set_range(ainstein_handle_t *handle, float min_m, float max_m) {
    if (handle->uart_fd < 0) return -1;
    
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "SET_RANGE %.1f %.1f\r\n", min_m, max_m);
    
    ssize_t n = write(handle->uart_fd, cmd, strlen(cmd));
    tcdrain(handle->uart_fd);
    
    return (n == (ssize_t)strlen(cmd)) ? 0 : -1;
}
