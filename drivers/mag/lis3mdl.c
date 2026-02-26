/*
 * LIS3MDL Magnetometer Driver
 * STMicroelectronics 3-axis magnetic sensor with I2C/SPI interface
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <pthread.h>
#include <time.h>
#include <math.h>

#define LIS3MDL_I2C_ADDR        0x1C
#define LIS3MDL_I2C_ADDR_ALT    0x1E
#define LIS3MDL_WHO_AM_I        0x0F
#define LIS3MDL_CTRL_REG1       0x20
#define LIS3MDL_CTRL_REG2       0x21
#define LIS3MDL_CTRL_REG3       0x22
#define LIS3MDL_CTRL_REG4       0x23
#define LIS3MDL_CTRL_REG5       0x24
#define LIS3MDL_STATUS_REG      0x27
#define LIS3MDL_OUT_X_L         0x28
#define LIS3MDL_OUT_X_H         0x29
#define LIS3MDL_OUT_Y_L         0x2A
#define LIS3MDL_OUT_Y_H         0x2B
#define LIS3MDL_OUT_Z_L         0x2C
#define LIS3MDL_OUT_Z_H         0x2D
#define LIS3MDL_TEMP_OUT_L      0x2E
#define LIS3MDL_TEMP_OUT_H      0x2F
#define LIS3MDL_INT_CFG         0x30
#define LIS3MDL_INT_SRC         0x31
#define LIS3MDL_INT_THS_L       0x32
#define LIS3MDL_INT_THS_H       0x33
#define LIS3MDL_CHIP_ID         0x3D
#define LIS3MDL_HEALTH_TIMEOUT  2000

typedef enum {
    LIS3MDL_RANGE_4GAUSS = 0,
    LIS3MDL_RANGE_8GAUSS = 1,
    LIS3MDL_RANGE_12GAUSS = 2,
    LIS3MDL_RANGE_16GAUSS = 3
} lis3mdl_range_t;

typedef enum {
    LIS3MDL_ODR_0_625_HZ = 0,
    LIS3MDL_ODR_1_25_HZ = 1,
    LIS3MDL_ODR_2_5_HZ = 2,
    LIS3MDL_ODR_5_HZ = 3,
    LIS3MDL_ODR_10_HZ = 4,
    LIS3MDL_ODR_20_HZ = 5,
    LIS3MDL_ODR_40_HZ = 6,
    LIS3MDL_ODR_80_HZ = 7
} lis3mdl_odr_t;

typedef enum {
    LIS3MDL_MODE_LOW = 0,
    LIS3MDL_MODE_MEDIUM = 1,
    LIS3MDL_MODE_HIGH = 2,
    LIS3MDL_MODE_ULTRA_HIGH = 3
} lis3mdl_perf_mode_t;

typedef enum {
    LIS3MDL_STATE_DISCONNECTED,
    LIS3MDL_STATE_CONNECTED,
    LIS3MDL_STATE_ERROR
} lis3mdl_state_t;

typedef enum {
    LIS3MDL_IFACE_I2C,
    LIS3MDL_IFACE_SPI
} lis3mdl_interface_t;

typedef struct {
    float mag_x;        /* Gauss */
    float mag_y;
    float mag_z;
    float heading_deg;
    float temperature;
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
    uint64_t timestamp_ns;
} lis3mdl_sample_t;

typedef struct {
    char device[64];
    lis3mdl_interface_t interface;
    uint8_t i2c_addr;
    uint32_t spi_speed;
    lis3mdl_range_t range;
    lis3mdl_odr_t odr;
    lis3mdl_perf_mode_t xy_mode;
    lis3mdl_perf_mode_t z_mode;
    bool temp_en;
    bool bdu;
} lis3mdl_config_t;

typedef struct {
    int fd;
    lis3mdl_state_t state;
    lis3mdl_config_t config;
    
    pthread_t sample_thread;
    pthread_t health_thread;
    pthread_mutex_t mutex;
    volatile bool running;
    
    /* Hotplug detection */
    time_t last_sample_time;
    bool hotplug_enabled;
    
    /* Health monitoring */
    uint32_t samples_collected;
    uint32_t sample_errors;
    
    /* Calibration */
    float hard_iron_offset[3];
    float soft_iron_matrix[3][3];
    float scale_gauss_per_lsb;
    
    /* Latest data */
    lis3mdl_sample_t latest;
    
    /* Callbacks */
    void (*on_sample)(lis3mdl_sample_t *sample, void *userdata);
    void (*on_health_change)(bool healthy, void *userdata);
    void *userdata;
} lis3mdl_handle_t;

static uint64_t get_timestamp_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

void lis3mdl_default_config(lis3mdl_config_t *cfg) {
    memset(cfg, 0, sizeof(lis3mdl_config_t));
    strcpy(cfg->device, "/dev/i2c-1");
    cfg->interface = LIS3MDL_IFACE_I2C;
    cfg->i2c_addr = LIS3MDL_I2C_ADDR;
    cfg->spi_speed = 1000000;
    cfg->range = LIS3MDL_RANGE_4GAUSS;
    cfg->odr = LIS3MDL_ODR_80_HZ;
    cfg->xy_mode = LIS3MDL_MODE_ULTRA_HIGH;
    cfg->z_mode = LIS3MDL_MODE_ULTRA_HIGH;
    cfg->temp_en = true;
    cfg->bdu = true;
}

lis3mdl_handle_t *lis3mdl_create(const char *name) {
    (void)name;
    lis3mdl_handle_t *handle = calloc(1, sizeof(lis3mdl_handle_t));
    if (!handle) return NULL;
    
    lis3mdl_default_config(&handle->config);
    pthread_mutex_init(&handle->mutex, NULL);
    handle->state = LIS3MDL_STATE_DISCONNECTED;
    handle->hotplug_enabled = true;
    handle->fd = -1;
    
    /* Initialize identity matrix for soft iron */
    handle->soft_iron_matrix[0][0] = 1.0f;
    handle->soft_iron_matrix[1][1] = 1.0f;
    handle->soft_iron_matrix[2][2] = 1.0f;
    
    return handle;
}

static int lis3mdl_i2c_open(lis3mdl_handle_t *handle) {
    handle->fd = open(handle->config.device, O_RDWR);
    if (handle->fd < 0) return -1;
    
    if (ioctl(handle->fd, I2C_SLAVE, handle->config.i2c_addr) < 0) {
        close(handle->fd);
        handle->fd = -1;
        return -1;
    }
    
    return 0;
}

static int lis3mdl_write_reg(lis3mdl_handle_t *handle, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    
    if (handle->config.interface == LIS3MDL_IFACE_I2C) {
        return write(handle->fd, buf, 2) == 2 ? 0 : -1;
    } else {
        struct spi_ioc_transfer tr[2] = {
            {.tx_buf = (unsigned long)&reg, .len = 1},
            {.tx_buf = (unsigned long)&val, .len = 1}
        };
        return ioctl(handle->fd, SPI_IOC_MESSAGE(2), tr);
    }
}

static int lis3mdl_read_reg(lis3mdl_handle_t *handle, uint8_t reg, uint8_t *val) {
    if (handle->config.interface == LIS3MDL_IFACE_I2C) {
        if (write(handle->fd, &reg, 1) != 1) return -1;
        return read(handle->fd, val, 1) == 1 ? 0 : -1;
    } else {
        reg |= 0x80;  /* Read bit for SPI */
        struct spi_ioc_transfer tr[2] = {
            {.tx_buf = (unsigned long)&reg, .len = 1},
            {.rx_buf = (unsigned long)val, .len = 1}
        };
        return ioctl(handle->fd, SPI_IOC_MESSAGE(2), tr);
    }
}

static int lis3mdl_read_bytes(lis3mdl_handle_t *handle, uint8_t reg, uint8_t *buf, size_t len) {
    if (handle->config.interface == LIS3MDL_IFACE_I2C) {
        if (write(handle->fd, &reg, 1) != 1) return -1;
        return read(handle->fd, buf, len) == (ssize_t)len ? 0 : -1;
    } else {
        reg |= 0xC0;  /* Read + auto-increment bits for SPI */
        struct spi_ioc_transfer tr[2] = {
            {.tx_buf = (unsigned long)&reg, .len = 1},
            {.rx_buf = (unsigned long)buf, .len = len}
        };
        return ioctl(handle->fd, SPI_IOC_MESSAGE(2), tr);
    }
}

static int lis3mdl_check_id(lis3mdl_handle_t *handle) {
    uint8_t id;
    if (lis3mdl_read_reg(handle, LIS3MDL_WHO_AM_I, &id) < 0) return -1;
    return (id == LIS3MDL_CHIP_ID) ? 0 : -1;
}

static int lis3mdl_configure(lis3mdl_handle_t *handle) {
    lis3mdl_config_t *cfg = &handle->config;
    
    /* CTRL_REG1: Temperature enable, XY performance mode, ODR */
    uint8_t ctrl1 = (cfg->temp_en ? 0x80 : 0x00) |
                    ((cfg->xy_mode & 0x03) << 5) |
                    ((cfg->odr & 0x07) << 2) |
                    (cfg->odr == LIS3MDL_ODR_80_HZ ? 0x02 : 0x00);
    if (lis3mdl_write_reg(handle, LIS3MDL_CTRL_REG1, ctrl1) < 0) return -1;
    
    /* CTRL_REG2: Full scale selection */
    uint8_t ctrl2 = ((cfg->range & 0x03) << 5);
    if (lis3mdl_write_reg(handle, LIS3MDL_CTRL_REG2, ctrl2) < 0) return -1;
    
    /* CTRL_REG3: Operating mode (continuous) */
    if (lis3mdl_write_reg(handle, LIS3MDL_CTRL_REG3, 0x00) < 0) return -1;
    
    /* CTRL_REG4: Z-axis performance mode */
    uint8_t ctrl4 = ((cfg->z_mode & 0x03) << 2);
    if (lis3mdl_write_reg(handle, LIS3MDL_CTRL_REG4, ctrl4) < 0) return -1;
    
    /* CTRL_REG5: Block data update, fast read */
    uint8_t ctrl5 = (cfg->bdu ? 0x40 : 0x00);
    if (lis3mdl_write_reg(handle, LIS3MDL_CTRL_REG5, ctrl5) < 0) return -1;
    
    /* Set scale factor based on range */
    switch (cfg->range) {
        case LIS3MDL_RANGE_4GAUSS:  handle->scale_gauss_per_lsb = 4.0f / 32768.0f; break;
        case LIS3MDL_RANGE_8GAUSS:  handle->scale_gauss_per_lsb = 8.0f / 32768.0f; break;
        case LIS3MDL_RANGE_12GAUSS: handle->scale_gauss_per_lsb = 12.0f / 32768.0f; break;
        case LIS3MDL_RANGE_16GAUSS: handle->scale_gauss_per_lsb = 16.0f / 32768.0f; break;
    }
    
    return 0;
}

static int lis3mdl_read_sample(lis3mdl_handle_t *handle, lis3mdl_sample_t *sample) {
    uint8_t buf[8];
    
    if (lis3mdl_read_bytes(handle, LIS3MDL_OUT_X_L | 0x80, buf, 6) < 0) return -1;
    
    sample->raw_x = (int16_t)(buf[1] << 8) | buf[0];
    sample->raw_y = (int16_t)(buf[3] << 8) | buf[2];
    sample->raw_z = (int16_t)(buf[5] << 8) | buf[4];
    
    /* Apply calibration */
    float mx = (sample->raw_x * handle->scale_gauss_per_lsb) - handle->hard_iron_offset[0];
    float my = (sample->raw_y * handle->scale_gauss_per_lsb) - handle->hard_iron_offset[1];
    float mz = (sample->raw_z * handle->scale_gauss_per_lsb) - handle->hard_iron_offset[2];
    
    /* Apply soft iron compensation */
    sample->mag_x = handle->soft_iron_matrix[0][0] * mx + 
                    handle->soft_iron_matrix[0][1] * my + 
                    handle->soft_iron_matrix[0][2] * mz;
    sample->mag_y = handle->soft_iron_matrix[1][0] * mx + 
                    handle->soft_iron_matrix[1][1] * my + 
                    handle->soft_iron_matrix[1][2] * mz;
    sample->mag_z = handle->soft_iron_matrix[2][0] * mx + 
                    handle->soft_iron_matrix[2][1] * my + 
                    handle->soft_iron_matrix[2][2] * mz;
    
    /* Calculate heading (assuming level) */
    sample->heading_deg = atan2f(-sample->mag_y, sample->mag_x) * 180.0f / M_PI;
    if (sample->heading_deg < 0) sample->heading_deg += 360.0f;
    
    /* Read temperature if enabled */
    if (handle->config.temp_en) {
        if (lis3mdl_read_bytes(handle, LIS3MDL_TEMP_OUT_L | 0x80, buf, 2) == 0) {
            int16_t temp_raw = (int16_t)(buf[1] << 8) | buf[0];
            sample->temperature = 25.0f + (temp_raw / 256.0f);
        }
    }
    
    sample->timestamp_ns = get_timestamp_ns();
    
    return 0;
}

static void *lis3mdl_sample_thread(void *arg) {
    lis3mdl_handle_t *handle = (lis3mdl_handle_t *)arg;
    lis3mdl_sample_t sample;
    
    while (handle->running) {
        if (lis3mdl_read_sample(handle, &sample) == 0) {
            pthread_mutex_lock(&handle->mutex);
            handle->latest = sample;
            handle->last_sample_time = time(NULL);
            handle->samples_collected++;
            pthread_mutex_unlock(&handle->mutex);
            
            if (handle->on_sample) {
                handle->on_sample(&sample, handle->userdata);
            }
        } else {
            pthread_mutex_lock(&handle->mutex);
            handle->sample_errors++;
            pthread_mutex_unlock(&handle->mutex);
        }
        
        usleep(1000000 / 80);  /* 80 Hz max */
    }
    
    return NULL;
}

static void *lis3mdl_health_thread(void *arg) {
    lis3mdl_handle_t *handle = (lis3mdl_handle_t *)arg;
    bool was_healthy = false;
    
    while (handle->running) {
        sleep(1);
        
        if (!handle->hotplug_enabled) continue;
        
        pthread_mutex_lock(&handle->mutex);
        time_t now = time(NULL);
        double elapsed = difftime(now, handle->last_sample_time);
        bool is_healthy = (handle->state == LIS3MDL_STATE_CONNECTED) && 
                          (elapsed < LIS3MDL_HEALTH_TIMEOUT / 1000.0);
        pthread_mutex_unlock(&handle->mutex);
        
        if (is_healthy != was_healthy) {
            if (handle->on_health_change) {
                handle->on_health_change(is_healthy, handle->userdata);
            }
            was_healthy = is_healthy;
        }
    }
    
    return NULL;
}

int lis3mdl_connect(lis3mdl_handle_t *handle) {
    pthread_mutex_lock(&handle->mutex);
    
    if (lis3mdl_i2c_open(handle) < 0) {
        handle->state = LIS3MDL_STATE_ERROR;
        pthread_mutex_unlock(&handle->mutex);
        return -1;
    }
    
    if (lis3mdl_check_id(handle) < 0 || lis3mdl_configure(handle) < 0) {
        close(handle->fd);
        handle->fd = -1;
        handle->state = LIS3MDL_STATE_ERROR;
        pthread_mutex_unlock(&handle->mutex);
        return -1;
    }
    
    handle->state = LIS3MDL_STATE_CONNECTED;
    handle->last_sample_time = time(NULL);
    
    pthread_mutex_unlock(&handle->mutex);
    return 0;
}

int lis3mdl_start(lis3mdl_handle_t *handle) {
    if (handle->state != LIS3MDL_STATE_CONNECTED) return -1;
    
    handle->running = true;
    pthread_create(&handle->sample_thread, NULL, lis3mdl_sample_thread, handle);
    pthread_create(&handle->health_thread, NULL, lis3mdl_health_thread, handle);
    
    return 0;
}

void lis3mdl_stop(lis3mdl_handle_t *handle) {
    handle->running = false;
    
    if (handle->sample_thread) {
        pthread_join(handle->sample_thread, NULL);
        handle->sample_thread = 0;
    }
    
    if (handle->health_thread) {
        pthread_join(handle->health_thread, NULL);
        handle->health_thread = 0;
    }
    
    if (handle->fd >= 0) {
        close(handle->fd);
        handle->fd = -1;
    }
    
    handle->state = LIS3MDL_STATE_DISCONNECTED;
}

void lis3mdl_destroy(lis3mdl_handle_t *handle) {
    if (!handle) return;
    
    lis3mdl_stop(handle);
    pthread_mutex_destroy(&handle->mutex);
    free(handle);
}

void lis3mdl_set_callbacks(lis3mdl_handle_t *handle,
                            void (*on_sample)(lis3mdl_sample_t *, void *),
                            void (*on_health)(bool, void *),
                            void *userdata) {
    handle->on_sample = on_sample;
    handle->on_health_change = on_health;
    handle->userdata = userdata;
}

void lis3mdl_set_hotplug(lis3mdl_handle_t *handle, bool enable) {
    handle->hotplug_enabled = enable;
}

int lis3mdl_get_sample(lis3mdl_handle_t *handle, lis3mdl_sample_t *sample) {
    pthread_mutex_lock(&handle->mutex);
    *sample = handle->latest;
    pthread_mutex_unlock(&handle->mutex);
    return 0;
}

int lis3mdl_get_health(lis3mdl_handle_t *handle, uint32_t *samples, uint32_t *errors) {
    pthread_mutex_lock(&handle->mutex);
    if (samples) *samples = handle->samples_collected;
    if (errors) *errors = handle->sample_errors;
    pthread_mutex_unlock(&handle->mutex);
    return 0;
}

void lis3mdl_set_hard_iron(lis3mdl_handle_t *handle, float x, float y, float z) {
    handle->hard_iron_offset[0] = x;
    handle->hard_iron_offset[1] = y;
    handle->hard_iron_offset[2] = z;
}

void lis3mdl_set_soft_iron(lis3mdl_handle_t *handle, float matrix[3][3]) {
    memcpy(handle->soft_iron_matrix, matrix, sizeof(float) * 9);
}
