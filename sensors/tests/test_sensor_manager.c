/*
 * Sensor Manager Test Suite
 * Tests the unified sensor management system
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include "sensors.h"

static volatile bool running = true;

void signal_handler(int sig) {
    (void)sig;
    running = false;
}

void on_sensor_error(const char *name, sensor_error_t err) {
    fprintf(stderr, "Sensor error: %s (code %d)\n", name, err);
}

void on_gnss_data(void *userdata, void *data, size_t size, sensor_timestamp_t *ts) {
    (void)userdata;
    (void)size;
    (void)ts;
    ublox_nav_data_t *nav = (ublox_nav_data_t *)data;
    printf("GNSS: Lat=%.6f Lon=%.6f Alt=%.2f Fix=%d\n",
           nav->position.latitude,
           nav->position.longitude,
           nav->position.altitude_msl,
           nav->position.fix_type);
}

void on_imu_data(void *userdata, void *data, size_t size, sensor_timestamp_t *ts) {
    (void)userdata;
    (void)size;
    (void)ts;
    icm20948_data_t *imu = (icm20948_data_t *)data;
    printf("IMU: Accel=%.3f,%.3f,%.3f Gyro=%.3f,%.3f,%.3f\n",
           imu->raw.accel.x, imu->raw.accel.y, imu->raw.accel.z,
           imu->raw.gyro.x, imu->raw.gyro.y, imu->raw.gyro.z);
}

void test_sensor_manager(void) {
    printf("=== Sensor Manager Test ===\n");
    
    /* Create manager */
    sensor_manager_t *mgr = sensor_manager_create();
    if (!mgr) {
        fprintf(stderr, "Failed to create sensor manager\n");
        return;
    }
    
    sensor_manager_init(mgr);
    mgr->on_sensor_error = on_sensor_error;
    
    /* Create test sensors (would connect to real hardware in production) */
    printf("Creating sensors...\n");
    
    /* GNSS */
    sensor_base_t *gnss = ublox_create("primary_gnss");
    if (gnss) {
        sensor_manager_register(mgr, gnss, SENSOR_GROUP_GNSS, false);
        sensor_base_set_callback(gnss, on_gnss_data, NULL);
        printf("  - Registered GNSS: %s\n", gnss->info.name);
    }
    
    /* IMU */
    sensor_base_t *imu = icm20948_create("primary_imu");
    if (imu) {
        sensor_manager_register(mgr, imu, SENSOR_GROUP_IMU, false);
        sensor_base_set_callback(imu, on_imu_data, NULL);
        printf("  - Registered IMU: %s\n", imu->info.name);
    }
    
    /* Barometer */
    sensor_base_t *baro = ms5611_create("primary_baro");
    if (baro) {
        sensor_manager_register(mgr, baro, SENSOR_GROUP_ENVIRONMENT, false);
        printf("  - Registered Baro: %s\n", baro->info.name);
    }
    
    /* Print stats */
    sensor_manager_stats_t stats;
    sensor_manager_get_stats(mgr, &stats);
    printf("\nRegistered %u sensors\n", stats.total_sensors);
    
    /* Test find operation */
    sensor_base_t *found = sensor_manager_find(mgr, "primary_gnss");
    if (found) {
        printf("Found sensor: %s\n", found->info.name);
    }
    
    /* Cleanup */
    sensor_manager_destroy(mgr);
    printf("\nSensor manager test complete.\n");
}

void test_imu_driver(void) {
    printf("\n=== ICM-20948 IMU Test ===\n");
    
    sensor_base_t *imu = icm20948_create("test_imu");
    if (!imu) {
        fprintf(stderr, "Failed to create IMU\n");
        return;
    }
    
    printf("IMU created: %s\n", imu->info.name);
    printf("Category: IMU\n");
    
    /* Configuration test */
    icm20948_config_t config = {
        .i2c_addr = ICM20948_I2C_ADDR,
        .accel_range = ICM20948_ACCEL_RANGE_16G,
        .gyro_range = ICM20948_GYRO_RANGE_2000DPS,
        .accel_odr = ICM20948_ODR_100_HZ,
        .gyro_odr = ICM20948_ODR_100_HZ,
        .enable_mag = true,
        .enable_dmp = false,
        .enable_fifo = true,
        .interrupt_gpio = -1
    };
    strncpy(config.i2c_bus, "/dev/i2c-1", sizeof(config.i2c_bus) - 1);
    
    printf("Configuration:\n");
    printf("  I2C Bus: %s\n", config.i2c_bus);
    printf("  I2C Addr: 0x%02X\n", config.i2c_addr);
    printf("  Accel Range: %dG\n", 2 << config.accel_range);
    printf("  Gyro Range: %dDPS\n", 250 << config.gyro_range);
    printf("  ODR: 100Hz\n");
    
    /* Cleanup */
    sensor_base_shutdown(imu);
    free(imu);
    
    printf("IMU test complete.\n");
}

void test_gnss_driver(void) {
    printf("\n=== u-blox ZED-F9P GNSS Test ===\n");
    
    sensor_base_t *gnss = ublox_create("test_gnss");
    if (!gnss) {
        fprintf(stderr, "Failed to create GNSS\n");
        return;
    }
    
    printf("GNSS created: %s\n", gnss->info.name);
    printf("Category: GNSS\n");
    
    ublox_config_t config = {
        .baud_rate = UBLOX_UART_BAUD_HIGH,
        .enable_rtk = true,
        .survey_in_duration = 60.0f,
        .enable_pps = true,
        .constellations = 0x0F,  /* GPS+GLONASS+Galileo+BeiDou */
        .update_rate_hz = 10
    };
    strncpy(config.uart_device, "/dev/ttyACM0", sizeof(config.uart_device) - 1);
    strncpy(config.rtk_correction_source, "ntrip://user:pass@ caster.example.com:2101/MOUNT", 
            sizeof(config.rtk_correction_source) - 1);
    
    printf("Configuration:\n");
    printf("  UART: %s @ %d baud\n", config.uart_device, config.baud_rate);
    printf("  RTK: %s\n", config.enable_rtk ? "enabled" : "disabled");
    printf("  Update rate: %d Hz\n", config.update_rate_hz);
    printf("  Constellations: GPS/GLONASS/Galileo/BeiDou\n");
    
    /* Cleanup */
    sensor_base_shutdown(gnss);
    free(gnss);
    
    printf("GNSS test complete.\n");
}

void test_camera_pipeline(void) {
    printf("\n=== GStreamer Camera Pipeline Test ===\n");
    
    gst_pipeline_t *pipe = gst_pipeline_create("test_camera");
    if (!pipe) {
        fprintf(stderr, "Failed to create pipeline\n");
        return;
    }
    
    printf("Pipeline created: %s\n", pipe->sensor_name);
    
    /* Test IMX477 configuration */
    printf("\nTesting IMX477 pipeline...\n");
    sensor_error_t err = gst_pipeline_configure(pipe, "imx477", 4056, 3040, 30, GST_FORMAT_NV12);
    if (err == SENSOR_OK) {
        printf("  Resolution: 4056x3040 @ 30fps\n");
        printf("  Format: NV12\n");
    }
    
    /* Generate pipeline string */
    char *pipeline_str = gst_pipeline_imx477_create(pipe, 4056, 3040, 30);
    if (pipeline_str) {
        printf("  Pipeline: %s\n", pipeline_str);
    }
    
    /* Test IMX219 */
    printf("\nTesting IMX219 pipeline...\n");
    err = gst_pipeline_configure(pipe, "imx219", 3280, 2464, 30, GST_FORMAT_NV12);
    if (err == SENSOR_OK) {
        printf("  Resolution: 3280x2464 @ 30fps\n");
    }
    
    pipeline_str = gst_pipeline_imx219_create(pipe, 3280, 2464, 30);
    if (pipeline_str) {
        printf("  Pipeline: %s\n", pipeline_str);
    }
    
    /* Cleanup */
    gst_pipeline_destroy(pipe);
    
    printf("\nCamera pipeline test complete.\n");
}

void test_telemetry_radio(void) {
    printf("\n=== Telemetry Radio Test ===\n");
    
    /* RFD900 */
    printf("\nRFD900 Configuration:\n");
    rfd900_config_t rfd_cfg = {
        .baud_rate = RFD900_DEFAULT_BAUD,
        .air_speed = RFD900_AIR_SPEED_128,
        .netid = 25,
        .frequency_hz = 915000000,
        .tx_power = RFD900_TX_POWER_20,
        .rtscts = false,
        .ecc = true,
        .mavlink = true
    };
    strncpy(rfd_cfg.uart_device, "/dev/ttyUSB0", sizeof(rfd_cfg.uart_device) - 1);
    
    printf("  Device: %s\n", rfd_cfg.uart_device);
    printf("  Frequency: %.3f MHz\n", rfd_cfg.frequency_hz / 1000000.0f);
    printf("  Air speed: %d kbps\n", rfd_cfg.air_speed);
    printf("  TX Power: %d (100mW)\n", rfd_cfg.tx_power);
    
    /* SiK */
    printf("\nSiK Radio Configuration:\n");
    sik_config_t sik_cfg = {
        .baud_rate = SIK_DEFAULT_BAUD,
        .air_speed = SIK_AIR_SPEED_64,
        .netid = 25,
        .frequency_min_hz = 915000000,
        .frequency_max_hz = 928000000,
        .num_channels = 50,
        .tx_power = 20,
        .ecc = true,
        .mavlink = true
    };
    strncpy(sik_cfg.i2c_bus, "/dev/ttyUSB1", sizeof(sik_cfg.i2c_bus) - 1);
    
    printf("  Device: %s\n", sik_cfg.i2c_bus);
    printf("  Frequency range: %.3f-%.3f MHz\n", 
           sik_cfg.frequency_min_hz / 1000000.0f,
           sik_cfg.frequency_max_hz / 1000000.0f);
    printf("  Channels: %d\n", sik_cfg.num_channels);
    
    printf("\nTelemetry radio test complete.\n");
}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    
    printf("========================================\n");
    printf("Unified Sensor Library Test Suite\n");
    printf("Version: %d.%d.%d\n", 
           SENSORS_VERSION_MAJOR,
           SENSORS_VERSION_MINOR,
           SENSORS_VERSION_PATCH);
    printf("========================================\n\n");
    
    /* Set up signal handler */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    /* Initialize library */
    sensor_error_t err = sensors_init();
    if (err != SENSOR_OK) {
        fprintf(stderr, "Failed to initialize sensor library\n");
        return 1;
    }
    
    printf("Library initialized: %s\n\n", sensors_version_string());
    
    /* Run tests */
    test_sensor_manager();
    test_imu_driver();
    test_gnss_driver();
    test_camera_pipeline();
    test_telemetry_radio();
    
    printf("\n========================================\n");
    printf("All tests complete.\n");
    printf("========================================\n");
    
    /* Cleanup */
    sensors_deinit();
    
    return 0;
}
