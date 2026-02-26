/*
 * MS5611 Barometric Pressure Sensor Driver
 * High-resolution pressure and temperature sensor via I2C/SPI
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

#define MS5611_I2C_ADDR         0x76
#define MS5611_I2C_ADDR_ALT     0x77
#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_D1_OSR_256   0x40
#define MS5611_CMD_D1_OSR_512   0x42
#define MS5611_CMD_D1_OSR_1024  0x44
#define MS5611_CMD_D1_OSR_2048  0x46
#define MS5611_CMD_D1_OSR_4096  0x48
#define MS5611_CMD_D2_OSR_256   0x50
#define MS5611_CMD_D2_OSR_512   0x52
#define MS5611_CMD_D2_OSR_1024  0x54
#define MS5611_CMD_D2_OSR_2048  0x56
#define MS5611_CMD_D2_OSR_4096  0x58
#define MS5611_CMD_ADC_READ     0x00
#define MS5611_CMD_PROM_READ    0xA0
#define MS5611_HEALTH_TIMEOUT   2000

typedef enum {
    MS5611_OSR_256 = 256,
    MS5611_OSR_512 = 512,
    MS5611_OSR_1024 = 1024,
    MS5611_OSR_2048 = 2048,
    MS5611_OSR_4096 = 4096
} ms5611_osr_t;

typedef enum {
    MS5611_STATE_DISCONNECTED,
    MS5611_STATE_CONNECTED,
    MS5611_STATE_ERROR
} ms5611_state_t;

typedef enum {
    MS5611_IFACE_I2C,
    MS5611_IFACE_SPI
} ms5611_interface_t;

typedef struct {
    uint16_t c1;    /* Pressure sensitivity */
    uint16_t c2;    /* Pressure offset */
    uint16_t c3;    /* Temperature coefficient of pressure sensitivity */
    uint16_t c4;    /* Temperature coefficient of pressure offset */
    uint16_t c5;    /* Reference temperature */
    uint16_t c6;    /* Temperature coefficient of the temperature */
    uint16_t crc;
} ms5611_prom_t;

typedef struct {
    float pressure_pa;
    float temperature_c;
    float altitude_m;
    float pressure_raw;
    float temperature_raw;
    uint64_t timestamp_ns;
} ms5611_sample_t;

typedef struct {
    char device[64];
    ms5611_interface_t interface;
    uint8_t i2c_addr;
    uint32_t spi_speed;
    ms5611_osr_t pressure_osr;
    ms5611_osr_t temp_osr;
    float sea_level_pa;
    uint32_t sample_rate_hz;
} ms5611_config_t;

typedef struct {
    int fd;
    ms5611_state_t state;
    ms5611_config_t config;
    ms5611_prom_t prom;
    
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
    uint32_t crc_errors;
    float avg_sample_time_ms;
    
    /* Latest data */
    ms5611_sample_t latest;
    
    /* Callbacks */
    void (*on_sample)(ms5611_sample_t *sample, void *userdata);
    void (*on_health_change)(bool healthy, void *userdata);
    void *userdata;
} ms5611_handle_t;

static uint64_t get_timestamp_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

void ms5611_default_config(ms5611_config_t *cfg) {
    memset(cfg, 0, sizeof(ms5611_config_t));
    strcpy(cfg->device, "/dev/i2c-1");
    cfg->interface = MS5611_IFACE_I2C;
    cfg->i2c_addr = MS5611_I2C_ADDR;
    cfg->spi_speed = 1000000;
    cfg->pressure_osr = MS5611_OSR_4096;
    cfg->temp_osr = MS5611_OSR_4096;
    cfg->sea_level_pa = 101325.0f;
    cfg->sample_rate_hz = 50;
}

ms5611_handle_t *ms5611_create(const char *name) {
    (void)name;
    ms5611_handle_t *handle = calloc(1, sizeof(ms5611_handle_t));
    if (!handle) return NULL;
    
    ms5611_default_config(&handle->config);
    pthread_mutex_init(&handle->mutex, NULL);
    handle->state = MS5611_STATE_DISCONNECTED;
    handle->hotplug_enabled = true;
    handle->fd = -1;
    
    return handle;
}

static int ms5611_i2c_open(ms5611_handle_t *handle) {
    handle->fd = open(handle->config.device, O_RDWR);
    if (handle->fd < 0) return -1;
    
    if (ioctl(handle->fd, I2C_SLAVE, handle->config.i2c_addr) < 0) {
        close(handle->fd);
        handle->fd = -1;
        return -1;
    }
    
    return 0;
}

static int ms5611_spi_open(ms5611_handle_t *handle) {
    handle->fd = open(handle->config.device, O_RDWR);
    if (handle->fd < 0) return -1;
    
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    
    ioctl(handle->fd, SPI_IOC_WR_MODE, &mode);
    ioctl(handle->fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(handle->fd, SPI_IOC_WR_MAX_SPEED_HZ, &handle->config.spi_speed);
    
    return 0;
}

static int ms5611_write_cmd(ms5611_handle_t *handle, uint8_t cmd) {
    if (handle->config.interface == MS5611_IFACE_I2C) {
        return write(handle->fd, &cmd, 1) == 1 ? 0 : -1;
    } else {
        struct spi_ioc_transfer tr = {
            .tx_buf = (unsigned long)&cmd,
            .len = 1,
            .speed_hz = handle->config.spi_speed,
            .delay_usecs = 0,
        };
        return ioctl(handle->fd, SPI_IOC_MESSAGE(1), &tr);
    }
}

static int ms5611_read_bytes(ms5611_handle_t *handle, uint8_t *buf, size_t len) {
    if (handle->config.interface == MS5611_IFACE_I2C) {
        return read(handle->fd, buf, len) == (ssize_t)len ? 0 : -1;
    } else {
        struct spi_ioc_transfer tr = {
            .rx_buf = (unsigned long)buf,
            .len = len,
            .speed_hz = handle->config.spi_speed,
            .delay_usecs = 0,
        };
        return ioctl(handle->fd, SPI_IOC_MESSAGE(1), &tr);
    }
}

static int ms5611_reset(ms5611_handle_t *handle) {
    if (ms5611_write_cmd(handle, MS5611_CMD_RESET) < 0) return -1;
    usleep(3000);  /* Wait for reset */
    return 0;
}

static uint16_t ms5611_crc4(uint16_t *prom) {
    uint16_t crc = 0;
    uint16_t rem = 0;
    
    prom[7] = 0;
    prom[7] = (prom[0] & 0x0F00) >> 8;
    
    for (int i = 0; i < 16; i++) {
        if (i % 2 == 1) {
            rem ^= prom[i >> 1] & 0x00FF;
        } else {
            rem ^= prom[i >> 1] >> 8;
        }
        
        for (int j = 0; j < 8; j++) {
            if (rem & 0x8000) {
                rem = (rem << 1) ^ 0x3000;
            } else {
                rem <<= 1;
            }
        }
    }
    
    crc = (rem >> 12) & 0x000F;
    return crc;
}

static int ms5611_read_prom(ms5611_handle_t *handle) {
    uint16_t prom[8];
    
    for (int i = 0; i < 8; i++) {
        uint8_t cmd = MS5611_CMD_PROM_READ + (i * 2);
        if (ms5611_write_cmd(handle, cmd) < 0) return -1;
        
        uint8_t buf[2];
        if (ms5611_read_bytes(handle, buf, 2) < 0) return -1;
        
        prom[i] = (buf[0] << 8) | buf[1];
    }
    
    /* Verify CRC */
    uint16_t crc_read = prom[7] & 0x000F;
    uint16_t crc_calc = ms5611_crc4(prom);
    
    if (crc_read != crc_calc) {
        handle->crc_errors++;
        return -1;
    }
    
    handle->prom.c1 = prom[1];
    handle->prom.c2 = prom[2];
    handle->prom.c3 = prom[3];
    handle->prom.c4 = prom[4];
    handle->prom.c5 = prom[5];
    handle->prom.c6 = prom[6];
    handle->prom.crc = prom[7];
    
    return 0;
}

static int ms5611_start_conversion(ms5611_handle_t *handle, uint8_t cmd) {
    return ms5611_write_cmd(handle, cmd);
}

static int ms5611_read_adc(ms5611_handle_t *handle, uint32_t *value) {
    if (ms5611_write_cmd(handle, MS5611_CMD_ADC_READ) < 0) return -1;
    
    uint8_t buf[3];
    if (ms5611_read_bytes(handle, buf, 3) < 0) return -1;
    
    *value = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    return 0;
}

static int ms5611_read_sample(ms5611_handle_t *handle, ms5611_sample_t *sample) {
    uint32_t d1 = 0, d2 = 0;
    
    /* Start D1 conversion (pressure) */
    uint8_t d1_cmd;
    switch (handle->config.pressure_osr) {
        case MS5611_OSR_256:  d1_cmd = MS5611_CMD_D1_OSR_256; break;
        case MS5611_OSR_512:  d1_cmd = MS5611_CMD_D1_OSR_512; break;
        case MS5611_OSR_1024: d1_cmd = MS5611_CMD_D1_OSR_1024; break;
        case MS5611_OSR_2048: d1_cmd = MS5611_CMD_D1_OSR_2048; break;
        default:              d1_cmd = MS5611_CMD_D1_OSR_4096; break;
    }
    
    if (ms5611_start_conversion(handle, d1_cmd) < 0) return -1;
    usleep(handle->config.pressure_osr == MS5611_OSR_4096 ? 9040 :
           handle->config.pressure_osr == MS5611_OSR_2048 ? 4560 :
           handle->config.pressure_osr == MS5611_OSR_1024 ? 2280 :
           handle->config.pressure_osr == MS5611_OSR_512 ? 1170 : 600);
    
    if (ms5611_read_adc(handle, &d1) < 0) return -1;
    
    /* Start D2 conversion (temperature) */
    uint8_t d2_cmd;
    switch (handle->config.temp_osr) {
        case MS5611_OSR_256:  d2_cmd = MS5611_CMD_D2_OSR_256; break;
        case MS5611_OSR_512:  d2_cmd = MS5611_CMD_D2_OSR_512; break;
        case MS5611_OSR_1024: d2_cmd = MS5611_CMD_D2_OSR_1024; break;
        case MS5611_OSR_2048: d2_cmd = MS5611_CMD_D2_OSR_2048; break;
        default:              d2_cmd = MS5611_CMD_D2_OSR_4096; break;
    }
    
    if (ms5611_start_conversion(handle, d2_cmd) < 0) return -1;
    usleep(handle->config.temp_osr == MS5611_OSR_4096 ? 9040 :
           handle->config.temp_osr == MS5611_OSR_2048 ? 4560 :
           handle->config.temp_osr == MS5611_OSR_1024 ? 2280 :
           handle->config.temp_osr == MS5611_OSR_512 ? 1170 : 600);
    
    if (ms5611_read_adc(handle, &d2) < 0) return -1;
    
    /* Calculate temperature */
    int32_t dT = (int32_t)d2 - ((int32_t)handle->prom.c5 << 8);
    int32_t temp = 2000 + (((int64_t)dT * handle->prom.c6) >> 23);
    
    /* Calculate temperature compensated pressure */
    int64_t off = ((int64_t)handle->prom.c2 << 16) + 
                  (((int64_t)handle->prom.c4 * dT) >> 7);
    int64_t sens = ((int64_t)handle->prom.c1 << 15) + 
                   (((int64_t)handle->prom.c3 * dT) >> 8);
    
    /* Second order temperature compensation */
    int32_t t2 = 0;
    int64_t off2 = 0;
    int64_t sens2 = 0;
    
    if (temp < 2000) {
        t2 = ((int64_t)dT * dT) >> 31;
        off2 = (5 * ((temp - 2000) * (temp - 2000))) >> 1;
        sens2 = (5 * ((temp - 2000) * (temp - 2000))) >> 2;
        
        if (temp < -1500) {
            off2 += 7 * ((temp + 1500) * (temp + 1500));
            sens2 += (11 * ((temp + 1500) * (temp + 1500))) >> 1;
        }
    }
    
    temp -= t2;
    off -= off2;
    sens -= sens2;
    
    int32_t pressure = ((((int64_t)d1 * sens) >> 21) - off) >> 15;
    
    sample->pressure_pa = (float)pressure / 100.0f;
    sample->temperature_c = (float)temp / 100.0f;
    sample->pressure_raw = d1;
    sample->temperature_raw = d2;
    sample->timestamp_ns = get_timestamp_ns();
    
    /* Calculate altitude */
    sample->altitude_m = 44330.0f * (1.0f - powf(sample->pressure_pa / handle->config.sea_level_pa, 0.190295f));
    
    return 0;
}

static void *ms5611_sample_thread(void *arg) {
    ms5611_handle_t *handle = (ms5611_handle_t *)arg;
    ms5611_sample_t sample;
    
    uint32_t interval_us = 1000000 / handle->config.sample_rate_hz;
    
    while (handle->running) {
        uint64_t start_ns = get_timestamp_ns();
        
        if (ms5611_read_sample(handle, &sample) == 0) {
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
        
        uint64_t elapsed_us = (get_timestamp_ns() - start_ns) / 1000;
        if (elapsed_us < interval_us) {
            usleep(interval_us - elapsed_us);
        }
    }
    
    return NULL;
}

static void *ms5611_health_thread(void *arg) {
    ms5611_handle_t *handle = (ms5611_handle_t *)arg;
    bool was_healthy = false;
    
    while (handle->running) {
        sleep(1);
        
        if (!handle->hotplug_enabled) continue;
        
        pthread_mutex_lock(&handle->mutex);
        time_t now = time(NULL);
        double elapsed = difftime(now, handle->last_sample_time);
        bool is_healthy = (handle->state == MS5611_STATE_CONNECTED) && 
                          (elapsed < MS5611_HEALTH_TIMEOUT / 1000.0);
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

int ms5611_connect(ms5611_handle_t *handle) {
    pthread_mutex_lock(&handle->mutex);
    
    int ret = (handle->config.interface == MS5611_IFACE_I2C) ? 
              ms5611_i2c_open(handle) : ms5611_spi_open(handle);
    
    if (ret < 0) {
        handle->state = MS5611_STATE_ERROR;
        pthread_mutex_unlock(&handle->mutex);
        return -1;
    }
    
    if (ms5611_reset(handle) < 0 || ms5611_read_prom(handle) < 0) {
        close(handle->fd);
        handle->fd = -1;
        handle->state = MS5611_STATE_ERROR;
        pthread_mutex_unlock(&handle->mutex);
        return -1;
    }
    
    handle->state = MS5611_STATE_CONNECTED;
    handle->last_sample_time = time(NULL);
    
    pthread_mutex_unlock(&handle->mutex);
    return 0;
}

int ms5611_start(ms5611_handle_t *handle) {
    if (handle->state != MS5611_STATE_CONNECTED) return -1;
    
    handle->running = true;
    pthread_create(&handle->sample_thread, NULL, ms5611_sample_thread, handle);
    pthread_create(&handle->health_thread, NULL, ms5611_health_thread, handle);
    
    return 0;
}

void ms5611_stop(ms5611_handle_t *handle) {
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
    
    handle->state = MS5611_STATE_DISCONNECTED;
}

void ms5611_destroy(ms5611_handle_t *handle) {
    if (!handle) return;
    
    ms5611_stop(handle);
    pthread_mutex_destroy(&handle->mutex);
    free(handle);
}

void ms5611_set_callbacks(ms5611_handle_t *handle,
                           void (*on_sample)(ms5611_sample_t *, void *),
                           void (*on_health)(bool, void *),
                           void *userdata) {
    handle->on_sample = on_sample;
    handle->on_health_change = on_health;
    handle->userdata = userdata;
}

void ms5611_set_hotplug(ms5611_handle_t *handle, bool enable) {
    handle->hotplug_enabled = enable;
}

int ms5611_get_sample(ms5611_handle_t *handle, ms5611_sample_t *sample) {
    pthread_mutex_lock(&handle->mutex);
    *sample = handle->latest;
    pthread_mutex_unlock(&handle->mutex);
    return 0;
}

int ms5611_get_health(ms5611_handle_t *handle, uint32_t *samples,
                       uint32_t *errors, float *avg_time_ms) {
    pthread_mutex_lock(&handle->mutex);
    
    if (samples) *samples = handle->samples_collected;
    if (errors) *errors = handle->sample_errors;
    if (avg_time_ms) *avg_time_ms = handle->avg_sample_time_ms;
    
    pthread_mutex_unlock(&handle->mutex);
    return 0;
}

void ms5611_set_sea_level(ms5611_handle_t *handle, float sea_level_pa) {
    handle->config.sea_level_pa = sea_level_pa;
}

float ms5611_pressure_to_altitude(float pressure_pa, float sea_level_pa) {
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.190295f));
}
