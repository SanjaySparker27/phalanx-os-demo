#include "gps_uart.h"
#include "linux/hal.h"
#include <string.h>
#include <stdio.h>

#define UART_THR    0x00
#define UART_RBR    0x00
#define UART_DLL    0x00
#define UART_DLH    0x04
#define UART_IER    0x04
#define UART_IIR    0x08
#define UART_FCR    0x08
#define UART_LCR    0x0C
#define UART_MCR    0x10
#define UART_LSR    0x14
#define UART_MSR    0x18
#define UART_SCR    0x1C

#define LSR_DR      0x01
#define LSR_THRE    0x20
#define LCR_DLAB    0x80
#define LCR_8N1     0x03
#define FCR_FIFO_EN 0x01
#define FCR_RX_RST  0x02
#define FCR_TX_RST  0x04
#define IER_RX_INT  0x01

static struct gps_uart gps_dev;

static void uart_write_reg(uint32_t reg, uint32_t val)
{
    writel(val, gps_dev.base + reg);
}

static uint32_t uart_read_reg(uint32_t reg)
{
    return readl(gps_dev.base + reg);
}

static void uart_set_baudrate(uint32_t baud)
{
    uint32_t div = 48000000 / (16 * baud);
    
    uint32_t lcr = uart_read_reg(UART_LCR);
    uart_write_reg(UART_LCR, lcr | LCR_DLAB);
    uart_write_reg(UART_DLL, div & 0xFF);
    uart_write_reg(UART_DLH, (div >> 8) & 0xFF);
    uart_write_reg(UART_LCR, lcr & ~LCR_DLAB);
}

int gps_init(uint32_t port, uint32_t baud)
{
    gps_dev.port = port;
    gps_dev.baudrate = baud;
    gps_dev.protocol = GPS_PROTO_NMEA;
    spin_lock_init(&gps_dev.lock);
    
    gps_dev.base = ioremap(GPS_UART_BASE + (port * 0x1000), 0x100);
    if (!gps_dev.base)
        return -ENOMEM;
    
    uart_write_reg(UART_FCR, FCR_FIFO_EN | FCR_RX_RST | FCR_TX_RST);
    uart_write_reg(UART_LCR, LCR_8N1);
    uart_set_baudrate(baud);
    uart_write_reg(UART_IER, IER_RX_INT);
    uart_write_reg(UART_MCR, 0x03);
    
    return 0;
}

void gps_deinit(void)
{
    uart_write_reg(UART_IER, 0);
    iounmap(gps_dev.base);
}

static bool uart_data_ready(void)
{
    return uart_read_reg(UART_LSR) & LSR_DR;
}

static uint8_t uart_read_byte(void)
{
    return uart_read_reg(UART_RBR) & 0xFF;
}

static int nmea_parse_gga(const char *sentence, struct gps_position *pos)
{
    char *p;
    int deg;
    float min;
    char dir;
    
    p = strchr(sentence, ',');
    if (!p) return -1;
    p = strchr(p + 1, ',');
    if (!p) return -1;
    
    sscanf(p + 1, "%2d%f", &deg, &min);
    pos->latitude = deg + min / 60.0;
    p = strchr(p + 1, ',');
    if (!p) return -1;
    dir = *(p + 1);
    if (dir == 'S') pos->latitude = -pos->latitude;
    
    p = strchr(p + 1, ',');
    if (!p) return -1;
    sscanf(p + 1, "%3d%f", &deg, &min);
    pos->longitude = deg + min / 60.0;
    p = strchr(p + 1, ',');
    if (!p) return -1;
    dir = *(p + 1);
    if (dir == 'W') pos->longitude = -pos->longitude;
    
    p = strchr(p + 1, ',');
    if (!p) return -1;
    sscanf(p + 1, "%hhd", &pos->fix_type);
    
    p = strchr(p + 1, ',');
    if (!p) return -1;
    sscanf(p + 1, "%hhd", &pos->num_satellites);
    
    p = strchr(p + 1, ',');
    if (!p) return -1;
    sscanf(p + 1, "%f", &pos->hdop);
    
    p = strchr(p + 1, ',');
    if (!p) return -1;
    sscanf(p + 1, "%lf", &pos->altitude);
    
    return 0;
}

static void gps_process_nmea(const char *sentence)
{
    if (strncmp(sentence, "$GPGGA", 6) == 0 ||
        strncmp(sentence, "$GNGGA", 6) == 0) {
        nmea_parse_gga(sentence, &gps_dev.position);
        gps_dev.position.timestamp_us = get_time_us();
    }
}

void gps_irq_handler(void)
{
    uint8_t data;
    static char nmea_buf[NMEA_MAX_LEN];
    static int nmea_pos = 0;
    
    spin_lock(&gps_dev.lock);
    
    while (uart_data_ready()) {
        data = uart_read_byte();
        
        gps_dev.rx_buffer[gps_dev.rx_head] = data;
        gps_dev.rx_head = (gps_dev.rx_head + 1) % GPS_BUFFER_SIZE;
        
        if (data == '$' || data == '!' || data == '#') {
            nmea_pos = 0;
            nmea_buf[nmea_pos++] = data;
        } else if (nmea_pos > 0 && nmea_pos < NMEA_MAX_LEN - 1) {
            nmea_buf[nmea_pos++] = data;
            if (data == '\n' || data == '\r') {
                nmea_buf[nmea_pos] = '\0';
                gps_process_nmea(nmea_buf);
                nmea_pos = 0;
            }
        }
    }
    
    spin_unlock(&gps_dev.lock);
}

int gps_read_position(struct gps_position *pos)
{
    unsigned long flags;
    
    spin_lock_irqsave(&gps_dev.lock, flags);
    memcpy(pos, &gps_dev.position, sizeof(*pos));
    spin_unlock_irqrestore(&gps_dev.lock, flags);
    
    return 0;
}

bool gps_has_fix(void)
{
    return gps_dev.position.fix_type > 0;
}

void gps_get_raw_data(uint8_t *data, uint32_t *len)
{
    unsigned long flags;
    uint32_t size = 0;
    
    spin_lock_irqsave(&gps_dev.lock, flags);
    
    while (gps_dev.rx_tail != gps_dev.rx_head && size < *len) {
        data[size++] = gps_dev.rx_buffer[gps_dev.rx_tail];
        gps_dev.rx_tail = (gps_dev.rx_tail + 1) % GPS_BUFFER_SIZE;
    }
    
    *len = size;
    spin_unlock_irqrestore(&gps_dev.lock, flags);
}
