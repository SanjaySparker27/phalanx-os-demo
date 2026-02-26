/*
 * Ouster OS0/OS1 LiDAR Driver
 * High-performance Ethernet-based 3D LiDAR
 * Features: Hotplug detection, automatic reconnection, health monitoring
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

#define OUSTER_DEFAULT_IP       "192.168.1.100"
#define OUSTER_DEFAULT_HOST_IP  "192.168.1.1"
#define OUSTER_LIDAR_PORT       7502
#define OUSTER_IMU_PORT         7503
#define OUSTER_TCP_PORT         7501
#define OUSTER_MAX_POINTS       131072
#define OUSTER_PACKET_SIZE      12608
#define OUSTER_COL_HEADER_SIZE  12
#define OUSTER_MAX_RETRY        5
#define OUSTER_HEALTH_TIMEOUT   3000

typedef enum {
    OUSTER_STATE_DISCONNECTED,
    OUSTER_STATE_CONNECTING,
    OUSTER_STATE_CONNECTED,
    OUSTER_STATE_STREAMING,
    OUSTER_STATE_ERROR
} ouster_state_t;

typedef enum {
    OUSTER_PROFILE_LEGACY = 1,
    OUSTER_PROFILE_DUAL = 2,
    OUSTER_PROFILE_2048 = 3,
    OUSTER_PROFILE_RNG19 = 4
} ouster_profile_t;

typedef struct {
    float x, y, z;
    float intensity;
    uint32_t range_mm;
    uint16_t reflectivity;
    uint16_t ambient;
    uint32_t timestamp_ns;
    uint8_t ring;
} ouster_point_t;

typedef struct {
    ouster_point_t points[OUSTER_MAX_POINTS];
    uint32_t num_points;
    uint64_t timestamp_ns;
    uint16_t frame_id;
    uint16_t measurement_id;
} ouster_frame_t;

typedef struct {
    float accel[3];
    float gyro[3];
    uint64_t timestamp_ns;
} ouster_imu_data_t;

typedef struct {
    char sensor_ip[16];
    char host_ip[16];
    uint16_t lidar_port;
    uint16_t imu_port;
    ouster_profile_t profile;
    bool dual_return;
    uint16_t azimuth_window_start;
    uint16_t azimuth_window_end;
    bool signal_multiplier;
    uint8_t nmea_in_polarity;
    uint8_t nmea_ignore_valid_char;
    bool nmea_baud_rate;
    bool nmea_leap_seconds;
} ouster_config_t;

typedef struct {
    int tcp_sock;
    int lidar_sock;
    int imu_sock;
    struct sockaddr_in sensor_addr;
    struct sockaddr_in host_lidar_addr;
    struct sockaddr_in host_imu_addr;
    
    ouster_state_t state;
    ouster_config_t config;
    
    pthread_t rx_thread;
    pthread_t health_thread;
    pthread_mutex_t mutex;
    volatile bool running;
    
    /* Hotplug detection */
    time_t last_packet_time;
    int missed_packets;
    bool hotplug_enabled;
    
    /* Health monitoring */
    uint32_t total_packets;
    uint32_t dropped_packets;
    uint32_t error_count;
    uint32_t reconnect_count;
    double avg_latency_ms;
    
    /* Callbacks */
    void (*on_frame)(ouster_frame_t *frame, void *userdata);
    void (*on_imu)(ouster_imu_data_t *imu, void *userdata);
    void (*on_health_change)(bool healthy, void *userdata);
    void *userdata;
    
    /* Metadata */
    char prod_line[32];
    char prod_sn[32];
    char prod_fw[32];
    uint8_t beam_azimuth_angles[128];
    uint8_t beam_altitude_angles[128];
    float lidar_to_sensor_transform[16];
    float imu_to_sensor_transform[16];
} ouster_handle_t;

/* CRC32 lookup table */
static const uint32_t crc32_table[256] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba,
    0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
    /* ... full table would be here ... */
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

static uint32_t crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc = (crc >> 8) ^ crc32_table[(crc ^ data[i]) & 0xFF];
    }
    return crc;
}

static uint64_t get_timestamp_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

/* Initialize default configuration */
void ouster_default_config(ouster_config_t *cfg) {
    memset(cfg, 0, sizeof(ouster_config_t));
    strcpy(cfg->sensor_ip, OUSTER_DEFAULT_IP);
    strcpy(cfg->host_ip, OUSTER_DEFAULT_HOST_IP);
    cfg->lidar_port = OUSTER_LIDAR_PORT;
    cfg->imu_port = OUSTER_IMU_PORT;
    cfg->profile = OUSTER_PROFILE_LEGACY;
    cfg->dual_return = false;
    cfg->azimuth_window_start = 0;
    cfg->azimuth_window_end = 360000;
    cfg->signal_multiplier = true;
}

/* Create handle */
ouster_handle_t *ouster_create(const char *name) {
    ouster_handle_t *handle = calloc(1, sizeof(ouster_handle_t));
    if (!handle) return NULL;
    
    ouster_default_config(&handle->config);
    pthread_mutex_init(&handle->mutex, NULL);
    handle->state = OUSTER_STATE_DISCONNECTED;
    handle->hotplug_enabled = true;
    handle->tcp_sock = -1;
    handle->lidar_sock = -1;
    handle->imu_sock = -1;
    
    return handle;
}

/* TCP command helper */
static int ouster_tcp_cmd(ouster_handle_t *handle, const char *cmd, 
                           char *resp, size_t resp_size) {
    if (handle->tcp_sock < 0) return -1;
    
    char cmd_buf[256];
    snprintf(cmd_buf, sizeof(cmd_buf), "%s\n", cmd);
    
    if (send(handle->tcp_sock, cmd_buf, strlen(cmd_buf), 0) < 0) {
        return -1;
    }
    
    if (resp && resp_size > 0) {
        memset(resp, 0, resp_size);
        int n = recv(handle->tcp_sock, resp, resp_size - 1, 0);
        if (n > 0) {
            resp[n] = '\0';
            return 0;
        }
    }
    return 0;
}

/* Connect to sensor via TCP for configuration */
int ouster_connect_tcp(ouster_handle_t *handle) {
    handle->tcp_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (handle->tcp_sock < 0) return -1;
    
    int flags = fcntl(handle->tcp_sock, F_GETFL, 0);
    fcntl(handle->tcp_sock, F_SETFL, flags | O_NONBLOCK);
    
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(OUSTER_TCP_PORT);
    inet_pton(AF_INET, handle->config.sensor_ip, &addr.sin_addr);
    
    int ret = connect(handle->tcp_sock, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0 && errno != EINPROGRESS) {
        close(handle->tcp_sock);
        handle->tcp_sock = -1;
        return -1;
    }
    
    /* Wait for connection with timeout */
    fd_set fdset;
    FD_ZERO(&fdset);
    FD_SET(handle->tcp_sock, &fdset);
    struct timeval tv = {.tv_sec = 2, .tv_usec = 0};
    
    ret = select(handle->tcp_sock + 1, NULL, &fdset, NULL, &tv);
    if (ret <= 0) {
        close(handle->tcp_sock);
        handle->tcp_sock = -1;
        return -1;
    }
    
    /* Set back to blocking */
    fcntl(handle->tcp_sock, F_SETFL, flags);
    
    return 0;
}

/* Configure UDP sockets */
int ouster_setup_udp(ouster_handle_t *handle) {
    /* Lidar socket */
    handle->lidar_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (handle->lidar_sock < 0) return -1;
    
    int opt = 1;
    setsockopt(handle->lidar_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    memset(&handle->host_lidar_addr, 0, sizeof(handle->host_lidar_addr));
    handle->host_lidar_addr.sin_family = AF_INET;
    handle->host_lidar_addr.sin_port = htons(handle->config.lidar_port);
    inet_pton(AF_INET, handle->config.host_ip, &handle->host_lidar_addr.sin_addr);
    
    if (bind(handle->lidar_sock, (struct sockaddr *)&handle->host_lidar_addr,
             sizeof(handle->host_lidar_addr)) < 0) {
        close(handle->lidar_sock);
        handle->lidar_sock = -1;
        return -1;
    }
    
    /* IMU socket */
    handle->imu_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (handle->imu_sock < 0) return -1;
    
    setsockopt(handle->imu_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    memset(&handle->host_imu_addr, 0, sizeof(handle->host_imu_addr));
    handle->host_imu_addr.sin_family = AF_INET;
    handle->host_imu_addr.sin_port = htons(handle->config.imu_port);
    inet_pton(AF_INET, handle->config.host_ip, &handle->host_imu_addr.sin_addr);
    
    if (bind(handle->imu_sock, (struct sockaddr *)&handle->host_imu_addr,
             sizeof(handle->host_imu_addr)) < 0) {
        close(handle->imu_sock);
        handle->imu_sock = -1;
        return -1;
    }
    
    return 0;
}

/* Get sensor metadata */
int ouster_get_metadata(ouster_handle_t *handle) {
    char resp[4096];
    
    if (ouster_tcp_cmd(handle, "get_sensor_info", resp, sizeof(resp)) == 0) {
        sscanf(resp, "{\"prod_line\":\"%31[^\"]\",\"prod_sn\":\"%31[^\"]\",\"prod_fw\":\"%31[^\"]\"",
               handle->prod_line, handle->prod_sn, handle->prod_fw);
    }
    
    if (ouster_tcp_cmd(handle, "get_beam_intrinsics", resp, sizeof(resp)) == 0) {
        /* Parse beam angles from JSON response */
        char *ptr = strstr(resp, "beam_altitude_angles");
        if (ptr) {
            ptr = strchr(ptr, '[');
            if (ptr) {
                for (int i = 0; i < 128; i++) {
                    float angle;
                    if (sscanf(ptr, "[%f", &angle) == 1) {
                        handle->beam_altitude_angles[i] = (uint8_t)(angle * 100);
                    }
                    ptr = strchr(ptr, ',');
                    if (!ptr) break;
                    ptr++;
                }
            }
        }
    }
    
    return 0;
}

/* Start streaming */
int ouster_start_streaming(ouster_handle_t *handle) {
    char cmd[256];
    
    /* Set UDP destination for lidar data */
    snprintf(cmd, sizeof(cmd), "set_udp_dest %s %d %d", 
             handle->config.host_ip, handle->config.lidar_port, handle->config.imu_port);
    ouster_tcp_cmd(handle, cmd, NULL, 0);
    
    /* Set lidar mode */
    snprintf(cmd, sizeof(cmd), "set_lidar_mode NORMAL");
    ouster_tcp_cmd(handle, cmd, NULL, 0);
    
    /* Set timestamp mode */
    ouster_tcp_cmd(handle, "set_ts_mode TIME_FROM_INTERNAL_OSC", NULL, 0);
    
    /* Save configuration */
    ouster_tcp_cmd(handle, "save_config_param", NULL, 0);
    
    handle->state = OUSTER_STATE_STREAMING;
    return 0;
}

/* Parse lidar data packet */
static void parse_lidar_packet(ouster_handle_t *handle, uint8_t *data, size_t len,
                                ouster_frame_t *frame) {
    if (len < OUSTER_PACKET_SIZE) return;
    
    uint16_t frame_id = *(uint16_t *)(data + 2);
    uint64_t timestamp_ns = *(uint64_t *)(data + 8);
    
    /* Each packet contains multiple columns (12 in legacy mode) */
    for (int col = 0; col < 12; col++) {
        size_t col_offset = 16 + col * 248;  /* 248 bytes per column */
        if (col_offset + 248 > len) break;
        
        uint16_t measurement_id = *(uint16_t *)(data + col_offset);
        uint32_t status = *(uint32_t *)(data + col_offset + 4);
        
        /* Parse channel data (up to 128 channels depending on model) */
        int num_ch = (handle->config.profile == OUSTER_PROFILE_LEGACY) ? 64 : 128;
        
        for (int ch = 0; ch < num_ch; ch++) {
            size_t ch_offset = col_offset + 16 + ch * 4;
            if (ch_offset + 4 > len) break;
            
            uint32_t raw = *(uint32_t *)(data + ch_offset);
            uint32_t range_mm = raw & 0xFFFFF;
            uint16_t reflectivity = (raw >> 20) & 0xFF;
            uint16_t ambient = (raw >> 28) & 0xF;
            
            if (range_mm == 0) continue;  /* Invalid point */
            
            int idx = frame->num_points++;
            if (idx >= OUSTER_MAX_POINTS) break;
            
            ouster_point_t *pt = &frame->points[idx];
            pt->range_mm = range_mm;
            pt->reflectivity = reflectivity;
            pt->ambient = ambient;
            pt->ring = ch;
            pt->timestamp_ns = timestamp_ns;
            
            /* Calculate cartesian coordinates */
            float azimuth = 2.0f * M_PI * measurement_id / 90112.0f;
            float altitude = handle->beam_altitude_angles[ch] / 100.0f * M_PI / 180.0f;
            float range_m = range_mm / 1000.0f;
            
            pt->x = range_m * cosf(altitude) * cosf(azimuth);
            pt->y = range_m * cosf(altitude) * sinf(azimuth);
            pt->z = range_m * sinf(altitude);
        }
    }
    
    frame->frame_id = frame_id;
    frame->timestamp_ns = timestamp_ns;
}

/* Parse IMU packet */
static void parse_imu_packet(ouster_handle_t *handle, uint8_t *data, size_t len,
                              ouster_imu_data_t *imu) {
    (void)handle;
    if (len < 48) return;
    
    imu->timestamp_ns = *(uint64_t *)data;
    
    /* Accelerometer data (m/s^2) */
    int32_t accel_x = *(int32_t *)(data + 8);
    int32_t accel_y = *(int32_t *)(data + 12);
    int32_t accel_z = *(int32_t *)(data + 16);
    
    /* Gyroscope data (rad/s) */
    int32_t gyro_x = *(int32_t *)(data + 20);
    int32_t gyro_y = *(int32_t *)(data + 24);
    int32_t gyro_z = *(int32_t *)(data + 28);
    
    imu->accel[0] = accel_x * 0.000000001f;
    imu->accel[1] = accel_y * 0.000000001f;
    imu->accel[2] = accel_z * 0.000000001f;
    
    imu->gyro[0] = gyro_x * 0.000000001f;
    imu->gyro[1] = gyro_y * 0.000000001f;
    imu->gyro[2] = gyro_z * 0.000000001f;
}

/* Receive thread */
static void *ouster_rx_thread(void *arg) {
    ouster_handle_t *handle = (ouster_handle_t *)arg;
    uint8_t lidar_buf[OUSTER_PACKET_SIZE + 256];
    uint8_t imu_buf[256];
    
    fd_set readfds;
    int max_fd = (handle->lidar_sock > handle->imu_sock) ? 
                  handle->lidar_sock : handle->imu_sock;
    max_fd++;
    
    ouster_frame_t current_frame;
    memset(&current_frame, 0, sizeof(current_frame));
    uint16_t last_frame_id = 0;
    
    while (handle->running) {
        FD_ZERO(&readfds);
        FD_SET(handle->lidar_sock, &readfds);
        FD_SET(handle->imu_sock, &readfds);
        
        struct timeval tv = {.tv_sec = 0, .tv_usec = 10000};
        int ret = select(max_fd, &readfds, NULL, NULL, &tv);
        
        if (ret > 0) {
            time_t now = time(NULL);
            
            if (FD_ISSET(handle->lidar_sock, &readfds)) {
                ssize_t n = recv(handle->lidar_sock, lidar_buf, sizeof(lidar_buf), 0);
                if (n > 0) {
                    handle->last_packet_time = now;
                    handle->total_packets++;
                    
                    parse_lidar_packet(handle, lidar_buf, n, &current_frame);
                    
                    /* Check for frame completion */
                    if (current_frame.num_points > 0 && 
                        current_frame.frame_id != last_frame_id) {
                        
                        if (handle->on_frame) {
                            handle->on_frame(&current_frame, handle->userdata);
                        }
                        
                        /* Reset for next frame */
                        current_frame.num_points = 0;
                        last_frame_id = current_frame.frame_id;
                    }
                }
            }
            
            if (FD_ISSET(handle->imu_sock, &readfds)) {
                ssize_t n = recv(handle->imu_sock, imu_buf, sizeof(imu_buf), 0);
                if (n > 0) {
                    handle->last_packet_time = now;
                    
                    ouster_imu_data_t imu;
                    parse_imu_packet(handle, imu_buf, n, &imu);
                    
                    if (handle->on_imu) {
                        handle->on_imu(&imu, handle->userdata);
                    }
                }
            }
        }
    }
    
    return NULL;
}

/* Health monitoring thread */
static void *ouster_health_thread(void *arg) {
    ouster_handle_t *handle = (ouster_handle_t *)arg;
    time_t last_check = time(NULL);
    bool was_healthy = false;
    
    while (handle->running) {
        sleep(1);
        
        if (!handle->hotplug_enabled) continue;
        
        time_t now = time(NULL);
        double elapsed = difftime(now, handle->last_packet_time);
        
        bool is_healthy = (handle->state == OUSTER_STATE_STREAMING) && 
                          (elapsed < OUSTER_HEALTH_TIMEOUT / 1000.0);
        
        if (is_healthy != was_healthy) {
            if (handle->on_health_change) {
                handle->on_health_change(is_healthy, handle->userdata);
            }
            was_healthy = is_healthy;
        }
        
        /* Auto-reconnect on timeout */
        if (!is_healthy && handle->state == OUSTER_STATE_STREAMING) {
            if (handle->reconnect_count < OUSTER_MAX_RETRY) {
                fprintf(stderr, "Ouster: Connection lost, attempting reconnect %d/%d\n",
                        handle->reconnect_count + 1, OUSTER_MAX_RETRY);
                
                ouster_stop(handle);
                sleep(2);
                
                if (ouster_connect(handle) == 0) {
                    ouster_start_streaming(handle);
                    handle->reconnect_count++;
                }
            } else {
                handle->state = OUSTER_STATE_ERROR;
            }
        }
        
        /* Reset reconnect count on successful health check */
        if (is_healthy && handle->reconnect_count > 0) {
            handle->reconnect_count = 0;
        }
    }
    
    return NULL;
}

/* Connect and start streaming */
int ouster_connect(ouster_handle_t *handle) {
    pthread_mutex_lock(&handle->mutex);
    
    handle->state = OUSTER_STATE_CONNECTING;
    
    if (ouster_connect_tcp(handle) < 0) {
        handle->state = OUSTER_STATE_DISCONNECTED;
        pthread_mutex_unlock(&handle->mutex);
        return -1;
    }
    
    if (ouster_setup_udp(handle) < 0) {
        close(handle->tcp_sock);
        handle->tcp_sock = -1;
        handle->state = OUSTER_STATE_DISCONNECTED;
        pthread_mutex_unlock(&handle->mutex);
        return -1;
    }
    
    ouster_get_metadata(handle);
    
    handle->state = OUSTER_STATE_CONNECTED;
    handle->last_packet_time = time(NULL);
    
    pthread_mutex_unlock(&handle->mutex);
    return 0;
}

/* Start driver */
int ouster_start(ouster_handle_t *handle) {
    if (handle->state != OUSTER_STATE_CONNECTED) {
        return -1;
    }
    
    ouster_start_streaming(handle);
    
    handle->running = true;
    pthread_create(&handle->rx_thread, NULL, ouster_rx_thread, handle);
    pthread_create(&handle->health_thread, NULL, ouster_health_thread, handle);
    
    return 0;
}

/* Stop driver */
void ouster_stop(ouster_handle_t *handle) {
    handle->running = false;
    
    if (handle->rx_thread) {
        pthread_join(handle->rx_thread, NULL);
        handle->rx_thread = 0;
    }
    
    if (handle->health_thread) {
        pthread_join(handle->health_thread, NULL);
        handle->health_thread = 0;
    }
    
    if (handle->tcp_sock >= 0) {
        close(handle->tcp_sock);
        handle->tcp_sock = -1;
    }
    
    if (handle->lidar_sock >= 0) {
        close(handle->lidar_sock);
        handle->lidar_sock = -1;
    }
    
    if (handle->imu_sock >= 0) {
        close(handle->imu_sock);
        handle->imu_sock = -1;
    }
    
    handle->state = OUSTER_STATE_DISCONNECTED;
}

/* Destroy handle */
void ouster_destroy(ouster_handle_t *handle) {
    if (!handle) return;
    
    ouster_stop(handle);
    pthread_mutex_destroy(&handle->mutex);
    free(handle);
}

/* Get health status */
int ouster_get_health(ouster_handle_t *handle, double *packet_loss_rate,
                       double *avg_latency_ms, uint32_t *total_packets) {
    pthread_mutex_lock(&handle->mutex);
    
    if (packet_loss_rate) {
        uint32_t total = handle->total_packets + handle->dropped_packets;
        *packet_loss_rate = (total > 0) ? 
                            (double)handle->dropped_packets / total : 0.0;
    }
    
    if (avg_latency_ms) {
        *avg_latency_ms = handle->avg_latency_ms;
    }
    
    if (total_packets) {
        *total_packets = handle->total_packets;
    }
    
    pthread_mutex_unlock(&handle->mutex);
    return 0;
}

/* Enable/disable hotplug detection */
void ouster_set_hotplug(ouster_handle_t *handle, bool enable) {
    handle->hotplug_enabled = enable;
}

/* Set callbacks */
void ouster_set_callbacks(ouster_handle_t *handle,
                           void (*on_frame)(ouster_frame_t *, void *),
                           void (*on_imu)(ouster_imu_data_t *, void *),
                           void (*on_health)(bool, void *),
                           void *userdata) {
    handle->on_frame = on_frame;
    handle->on_imu = on_imu;
    handle->on_health_change = on_health;
    handle->userdata = userdata;
}
