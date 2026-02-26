/*
 * SPI Helper Functions
 * Common SPI operations for sensor drivers
 */

#ifndef SPI_HELPER_H
#define SPI_HELPER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int fd;
    char device[32];
    uint32_t speed_hz;
    uint8_t mode;
    uint8_t bits;
    int cs_gpio;  /* -1 if using hardware CS */
} spi_handle_t;

/* Mode flags */
#define SPI_MODE_0  0  /* CPOL=0, CPHA=0 */
#define SPI_MODE_1  1  /* CPOL=0, CPHA=1 */
#define SPI_MODE_2  2  /* CPOL=1, CPHA=0 */
#define SPI_MODE_3  3  /* CPOL=1, CPHA=1 */
#define SPI_CS_HIGH 4  /* Chip select active high */
#define SPI_LSB_FIRST 8

/* Bus operations */
spi_handle_t *spi_open(const char *device, uint32_t speed, uint8_t mode);
void spi_close(spi_handle_t *handle);
int spi_configure(spi_handle_t *handle, uint32_t speed, uint8_t mode, uint8_t bits);

/* Transfer operations */
int spi_transfer(spi_handle_t *handle, uint8_t *tx, uint8_t *rx, size_t len);
int spi_write(spi_handle_t *handle, uint8_t *data, size_t len);
int spi_read(spi_handle_t *handle, uint8_t *data, size_t len);
int spi_write_then_read(spi_handle_t *handle, uint8_t *tx, size_t tx_len,
                         uint8_t *rx, size_t rx_len);

/* Register operations (common pattern for sensors) */
int spi_write_reg(spi_handle_t *handle, uint8_t reg, uint8_t val);
int spi_write_reg16(spi_handle_t *handle, uint8_t reg, uint16_t val);
int spi_read_reg(spi_handle_t *handle, uint8_t reg, uint8_t *val);
int spi_read_reg16(spi_handle_t *handle, uint8_t reg, uint16_t *val);
int spi_read_reg_buf(spi_handle_t *handle, uint8_t reg, uint8_t *buf, size_t len);

/* GPIO chip select (for devices without hardware CS) */
int spi_cs_init(int gpio);
void spi_cs_set(int gpio, bool high);

#ifdef __cplusplus
}
#endif

#endif /* SPI_HELPER_H */
