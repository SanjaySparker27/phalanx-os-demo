/*
 * I2C Helper Functions
 * Common I2C operations for sensor drivers
 */

#ifndef I2C_HELPER_H
#define I2C_HELPER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int fd;
    char bus_path[32];
    uint8_t current_addr;
    uint32_t timeout_ms;
} i2c_handle_t;

/* Bus operations */
i2c_handle_t *i2c_open(const char *bus);
void i2c_close(i2c_handle_t *handle);
int i2c_select_device(i2c_handle_t *handle, uint8_t addr);

/* Raw read/write */
int i2c_write(i2c_handle_t *handle, uint8_t *data, size_t len);
int i2c_read(i2c_handle_t *handle, uint8_t *data, size_t len);
int i2c_write_read(i2c_handle_t *handle, uint8_t *tx, size_t tx_len,
                    uint8_t *rx, size_t rx_len);

/* Register operations */
int i2c_write_reg(i2c_handle_t *handle, uint8_t reg, uint8_t val);
int i2c_write_reg16(i2c_handle_t *handle, uint8_t reg, uint16_t val);
int i2c_read_reg(i2c_handle_t *handle, uint8_t reg, uint8_t *val);
int i2c_read_reg16(i2c_handle_t *handle, uint8_t reg, uint16_t *val);
int i2c_read_reg_buf(i2c_handle_t *handle, uint8_t reg, uint8_t *buf, size_t len);

/* Bit operations */
int i2c_modify_reg(i2c_handle_t *handle, uint8_t reg, uint8_t mask, uint8_t val);
int i2c_set_bit(i2c_handle_t *handle, uint8_t reg, uint8_t bit);
int i2c_clear_bit(i2c_handle_t *handle, uint8_t reg, uint8_t bit);

/* Scan and detect */
int i2c_scan_bus(const char *bus, uint8_t *found_addrs, size_t max_addrs);
bool i2c_detect_device(const char *bus, uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* I2C_HELPER_H */
