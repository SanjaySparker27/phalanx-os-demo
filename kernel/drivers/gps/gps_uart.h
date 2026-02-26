#ifndef _GPS_UART_H
#define _GPS_UART_H

#include "linux/types.h"

#define GPS_UART_BASE       0x3100000
#define GPS_BAUDRATE        115200
#define GPS_BUFFER_SIZE     256

#define NMEA_MAX_LEN        82
#define UBX_SYNC_CHAR_1     0xB5
#define UBX_SYNC_CHAR_2     0x62

enum gps_protocol {
    GPS_PROTO_NMEA,
    GPS_PROTO_UBX,
    GPS_PROTO_RTCM,
};

struct gps_position {
    double latitude;
    double longitude;
    double altitude;
    float speed;
    float course;
    uint8_t fix_type;
    uint8_t num_satellites;
    float hdop;
    uint64_t timestamp_us;
};

struct gps_uart {
    uint32_t port;
    void __iomem *base;
    uint32_t baudrate;
    enum gps_protocol protocol;
    struct gps_position position;
    spinlock_t lock;
    uint8_t rx_buffer[GPS_BUFFER_SIZE];
    uint32_t rx_head;
    uint32_t rx_tail;
};

int gps_init(uint32_t port, uint32_t baud);
void gps_deinit(void);
int gps_read_position(struct gps_position *pos);
bool gps_has_fix(void);
void gps_get_raw_data(uint8_t *data, uint32_t *len);

#endif