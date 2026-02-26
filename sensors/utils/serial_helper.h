/*
 * Serial/UART Helper Functions
 * Common serial operations for sensor drivers
 */

#ifndef SERIAL_HELPER_H
#define SERIAL_HELPER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int fd;
    char device[64];
    uint32_t baud;
    uint8_t data_bits;
    uint8_t stop_bits;
    char parity;  /* 'N', 'E', 'O' */
    bool rtscts;
    bool dsrdtr;
    int timeout_ms;
} serial_handle_t;

/* Baud rates */
#define SERIAL_BAUD_9600    9600
#define SERIAL_BAUD_19200   19200
#define SERIAL_BAUD_38400   38400
#define SERIAL_BAUD_57600   57600
#define SERIAL_BAUD_115200  115200
#define SERIAL_BAUD_230400  230400
#define SERIAL_BAUD_460800  460800
#define SERIAL_BAUD_921600  921600

/* Open/close */
serial_handle_t *serial_open(const char *device, uint32_t baud);
void serial_close(serial_handle_t *handle);
int serial_reconfigure(serial_handle_t *handle, uint32_t baud, 
                        uint8_t data_bits, uint8_t stop_bits, char parity);

/* Basic I/O */
int serial_write(serial_handle_t *handle, const uint8_t *data, size_t len);
int serial_read(serial_handle_t *handle, uint8_t *data, size_t max_len, int timeout_ms);
int serial_read_exact(serial_handle_t *handle, uint8_t *data, size_t len, int timeout_ms);

/* Line-based I/O (for AT command protocols) */
int serial_write_line(serial_handle_t *handle, const char *line);
int serial_read_line(serial_handle_t *handle, char *line, size_t max_len, int timeout_ms);
int serial_wait_for(serial_handle_t *handle, const char *pattern, int timeout_ms);

/* Buffer management */
int serial_flush_rx(serial_handle_t *handle);
int serial_flush_tx(serial_handle_t *handle);
int serial_flush_both(serial_handle_t *handle);
int serial_get_available(serial_handle_t *handle);

/* Control signals */
int serial_set_rts(serial_handle_t *handle, bool high);
int serial_set_dtr(serial_handle_t *handle, bool high);
int serial_get_cts(serial_handle_t *handle);
int serial_get_dsr(serial_handle_t *handle);

/* Break and sync */
int serial_send_break(serial_handle_t *handle, int duration_ms);
int serial_send_sync(serial_handle_t *handle, uint8_t sync_byte, int count);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_HELPER_H */
