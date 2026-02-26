/*
 * Velodyne VLP-16 LiDAR Driver
 * UDP-based LiDAR with hotplug detection and health monitoring
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <math.h>

#define VELODYNE_DATA_PORT      2368
#define VELODYNE_POSITION_PORT  8308
#define VELODYNE_PACKET_SIZE    1206
#define VELODYNE_MAX_POINTS     300000
#define VELODYNE_BLOCKS_PER_PKT 12
#define VELODYNE_CHANNELS       16
#define VELODYNE_MAX_RETRY      5
#define VELODYNE_HEALTH_TIMEOUT 2000

typedef enum {
    VLP16_STATE_DISCONNECTED,
    VLP16_STATE_CONNECTING,
    VLP16_STATE_CONNECTED,
    VLP16_STATE_ERROR
} vlp16_state_t;

typedef struct {
    float x, y, z;
    float intensity;
    uint16_t ring;
    uint32_t timestamp_us;
    float azimuth;
} vlp16_point_t;

typedef struct {
    vlp16_point_t points[VELODYNE_MAX_POINTS];
    uint32_t num_points;
    uint64_t timestamp_ns;
    uint16_t revolution;
    bool complete;
} vlp16_frame_t;

typedef struct {
    char sensor_ip[16];
    uint16_t data_port;
    uint16_t position_port;
    uint16_t rpm;
    uint8_t return_mode;
    bool gps_pps_sync;
} vlp16_config_t;

typedef struct {
    int data_sock;
    int position_sock;
    struct sockaddr_in host_data_addr;
    struct sockaddr_in host_position_addr;
    
    vlp16_state_t state;
    vlp16_config_t config;
    
    pthread_t rx_thread;
    pthread_t health_thread;
    pthread_mutex_t mutex;
    volatile bool running;
    
    /* Hotplug detection */
    time_t last_packet_time;
    bool hotplug_enabled;
    
    /* Health monitoring */
    uint32_t packets_received;
    uint32_t packets_dropped;
    uint32_t reconnect_count;
    uint32_t total_frames;
    
    /* Calibration */
    float vert_correction[16];
    float rot_correction[16];
    float dist_correction[16];
    float cos_vert[16];
    float sin_vert[16];
    
    /* Callbacks */
    void (*on_frame)(vlp16_frame_t *frame, void *userdata);
    void (*on_health_change)(bool healthy, void *userdata);
    void *userdata;
} vlp16_handle_t;

/* Vertical angles for VLP-16 */
static const float vlp16_vert_angles[16] = {
    -15.0f, 1.0f, -13.0f, 3.0f, -11.0f, 5.0f, -9.0f, 7.0f,
    -7.0f, 9.0f, -5.0f, 11.0f, -3.0f, 13.0f, -1.0f, 15.0f
};

static uint64_t get_timestamp_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

void vlp16_default_config(vlp16_config_t *cfg) {
    memset(cfg, 0, sizeof(vlp16_config_t));
    strcpy(cfg->sensor_ip, "0.0.0.0");
    cfg->data_port = VELODYNE_DATA_PORT;
    cfg->position_port = VELODYNE_POSITION_PORT;
    cfg->rpm = 600;
    cfg->return_mode = 0x37;
    cfg->gps_pps_sync = true;
}

vlp16_handle_t *vlp16_create(const char *name) {
    (void)name;
    vlp16_handle_t *handle = calloc(1, sizeof(vlp16_handle_t));
    if (!handle) return NULL;
    
    vlp16_default_config(&handle->config);
    pthread_mutex_init(&handle->mutex, NULL);
    handle->state = VLP16_STATE_DISCONNECTED;
    handle->hotplug_enabled = true;
    handle->data_sock = -1;
    handle->position_sock = -1;
    
    /* Initialize calibration */
    for (int i = 0; i < 16; i++) {
        handle->vert_correction[i] = vlp16_vert_angles[i];
        handle->rot_correction[i] = 0.0f;
        handle->dist_correction[i] = 0.0f;
        float rad = handle->vert_correction[i] * M_PI / 180.0f;
        handle->cos_vert[i] = cosf(rad);
        handle->sin_vert[i] = sinf(rad);
    }
    
    return handle;
}

int vlp16_setup_sockets(vlp16_handle_t *handle) {
    /* Data socket */
    handle->data_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (handle->data_sock < 0) return -1;
    
    int opt = 1;
    setsockopt(handle->data_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    int bufsize = 1024 * 1024;
    setsockopt(handle->data_sock, SOL_SOCKET, SO_RCVBUF, &bufsize, sizeof(bufsize));
    
    memset(&handle->host_data_addr, 0, sizeof(handle->host_data_addr));
    handle->host_data_addr.sin_family = AF_INET;
    handle->host_data_addr.sin_port = htons(handle->config.data_port);
    
    if (strcmp(handle->config.sensor_ip, "0.0.0.0") == 0) {
        handle->host_data_addr.sin_addr.s_addr = INADDR_ANY;
    } else {
        inet_pton(AF_INET, handle->config.sensor_ip, &handle->host_data_addr.sin_addr);
    }
    
    if (bind(handle->data_sock, (struct sockaddr *)&handle->host_data_addr,
             sizeof(handle->host_data_addr)) < 0) {
        close(handle->data_sock);
        handle->data_sock = -1;
        return -1;
    }
    
    /* Position socket */
    handle->position_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (handle->position_sock >= 0) {
        setsockopt(handle->position_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        memset(&handle->host_position_addr, 0, sizeof(handle->host_position_addr));
        handle->host_position_addr.sin_family = AF_INET;
        handle->host_position_addr.sin_port = htons(handle->config.position_port);
        handle->host_position_addr.sin_addr.s_addr = INADDR_ANY;
        
        bind(handle->position_sock, (struct sockaddr *)&handle->host_position_addr,
             sizeof(handle->host_position_addr));
    }
    
    return 0;
}

static void parse_data_packet(vlp16_handle_t *handle, uint8_t *data, size_t len,
                               vlp16_frame_t *frame) {
    if (len != VELODYNE_PACKET_SIZE) return;
    
    uint32_t gps_timestamp = *(uint32_t *)(data + 1200);
    uint8_t return_mode = data[1204];
    
    (void)return_mode;
    (void)gps_timestamp;
    
    for (int block = 0; block < VELODYNE_BLOCKS_PER_PKT; block++) {
        size_t block_offset = 100 * block;
        
        uint16_t flag = *(uint16_t *)(data + block_offset);
        if (flag != 0xeeff) continue;
        
        uint16_t azimuth = *(uint16_t *)(data + block_offset + 2);
        float azimuth_rad = azimuth / 100.0f * M_PI / 180.0f;
        
        /* Two firing sequences per block for VLP-16 */
        for (int firing = 0; firing < 2; firing++) {
            float azimuth_offset = (firing == 0) ? 0.0f : 
                                   (2.0f * M_PI * 0.2f / 360.0f);
            float current_azimuth = azimuth_rad + azimuth_offset;
            
            for (int ch = 0; ch < 8; ch++) {
                int channel = firing * 8 + ch;
                size_t ch_offset = block_offset + 4 + (firing * 48) + (ch * 3);
                
                uint16_t distance_raw = *(uint16_t *)(data + ch_offset);
                uint8_t intensity = data[ch_offset + 2];
                
                if (distance_raw == 0) continue;
                
                float distance_m = distance_raw * 0.002f;
                
                int idx = frame->num_points++;
                if (idx >= VELODYNE_MAX_POINTS) return;
                
                vlp16_point_t *pt = &frame->points[idx];
                pt->ring = channel;
                pt->intensity = intensity;
                pt->azimuth = current_azimuth;
                pt->timestamp_us = gps_timestamp;
                
                /* Spherical to cartesian */
                float cos_vert = handle->cos_vert[channel];
                float sin_vert = handle->sin_vert[channel];
                float cos_az = cosf(current_azimuth);
                float sin_az = sinf(current_azimuth);
                
                pt->x = distance_m * cos_vert * sin_az;
                pt->y = distance_m * cos_vert * cos_az;
                pt->z = distance_m * sin_vert;
            }
        }
    }
}

static void *vlp16_rx_thread(void *arg) {
    vlp16_handle_t *handle = (vlp16_handle_t *)arg;
    uint8_t buffer[VELODYNE_PACKET_SIZE];
    
    vlp16_frame_t current_frame;
    memset(&current_frame, 0, sizeof(current_frame));
    
    fd_set readfds;
    int max_fd = handle->data_sock + 1;
    
    while (handle->running) {
        FD_ZERO(&readfds);
        FD_SET(handle->data_sock, &readfds);
        
        struct timeval tv = {.tv_sec = 0, .tv_usec = 10000};
        int ret = select(max_fd, &readfds, NULL, NULL, &tv);
        
        if (ret > 0 && FD_ISSET(handle->data_sock, &readfds)) {
            ssize_t n = recv(handle->data_sock, buffer, sizeof(buffer), 0);
            if (n == VELODYNE_PACKET_SIZE) {
                pthread_mutex_lock(&handle->mutex);
                handle->last_packet_time = time(NULL);
                handle->packets_received++;
                pthread_mutex_unlock(&handle->mutex);
                
                parse_data_packet(handle, buffer, n, &current_frame);
            }
        }
    }
    
    return NULL;
}

static void *vlp16_health_thread(void *arg) {
    vlp16_handle_t *handle = (vlp16_handle_t *)arg;
    bool was_healthy = false;
    
    while (handle->running) {
        sleep(1);
        
        if (!handle->hotplug_enabled) continue;
        
        pthread_mutex_lock(&handle->mutex);
        time_t now = time(NULL);
        double elapsed = difftime(now, handle->last_packet_time);
        bool is_healthy = (handle->state == VLP16_STATE_CONNECTED) && 
                          (elapsed < VELODYNE_HEALTH_TIMEOUT / 1000.0);
        pthread_mutex_unlock(&handle->mutex);
        
        if (is_healthy != was_healthy) {
            if (handle->on_health_change) {
                handle->on_health_change(is_healthy, handle->userdata);
            }
            was_healthy = is_healthy;
        }
        
        if (!is_healthy && handle->state == VLP16_STATE_CONNECTED) {
            if (handle->reconnect_count < VELODYNE_MAX_RETRY) {
                vlp16_stop(handle);
                sleep(1);
                if (vlp16_connect(handle) == 0) {
                    vlp16_start(handle);
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

int vlp16_connect(vlp16_handle_t *handle) {
    pthread_mutex_lock(&handle->mutex);
    
    handle->state = VLP16_STATE_CONNECTING;
    
    if (vlp16_setup_sockets(handle) < 0) {
        handle->state = VLP16_STATE_ERROR;
        pthread_mutex_unlock(&handle->mutex);
        return -1;
    }
    
    handle->state = VLP16_STATE_CONNECTED;
    handle->last_packet_time = time(NULL);
    
    pthread_mutex_unlock(&handle->mutex);
    return 0;
}

int vlp16_start(vlp16_handle_t *handle) {
    if (handle->state != VLP16_STATE_CONNECTED) return -1;
    
    handle->running = true;
    pthread_create(&handle->rx_thread, NULL, vlp16_rx_thread, handle);
    pthread_create(&handle->health_thread, NULL, vlp16_health_thread, handle);
    
    return 0;
}

void vlp16_stop(vlp16_handle_t *handle) {
    handle->running = false;
    
    if (handle->rx_thread) {
        pthread_join(handle->rx_thread, NULL);
        handle->rx_thread = 0;
    }
    
    if (handle->health_thread) {
        pthread_join(handle->health_thread, NULL);
        handle->health_thread = 0;
    }
    
    if (handle->data_sock >= 0) {
        close(handle->data_sock);
        handle->data_sock = -1;
    }
    
    if (handle->position_sock >= 0) {
        close(handle->position_sock);
        handle->position_sock = -1;
    }
    
    handle->state = VLP16_STATE_DISCONNECTED;
}

void vlp16_destroy(vlp16_handle_t *handle) {
    if (!handle) return;
    
    vlp16_stop(handle);
    pthread_mutex_destroy(&handle->mutex);
    free(handle);
}

void vlp16_set_callbacks(vlp16_handle_t *handle,
                          void (*on_frame)(vlp16_frame_t *, void *),
                          void (*on_health)(bool, void *),
                          void *userdata) {
    handle->on_frame = on_frame;
    handle->on_health_change = on_health;
    handle->userdata = userdata;
}

void vlp16_set_hotplug(vlp16_handle_t *handle, bool enable) {
    handle->hotplug_enabled = enable;
}

int vlp16_get_health(vlp16_handle_t *handle, uint32_t *packets, 
                      uint32_t *dropped, double *loss_rate) {
    pthread_mutex_lock(&handle->mutex);
    
    if (packets) *packets = handle->packets_received;
    if (dropped) *dropped = handle->packets_dropped;
    if (loss_rate) {
        uint32_t total = handle->packets_received + handle->packets_dropped;
        *loss_rate = (total > 0) ? (double)handle->packets_dropped / total : 0.0;
    }
    
    pthread_mutex_unlock(&handle->mutex);
    return 0;
}
