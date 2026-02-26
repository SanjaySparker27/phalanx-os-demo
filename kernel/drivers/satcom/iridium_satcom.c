#include "iridium_satcom.h"
#include "linux/hal.h"
#include <string.h>

#define UART_THR    0x00
#define UART_RBR    0x00
#define UART_IER    0x04
#define UART_IIR    0x08
#define UART_FCR    0x08
#define UART_LCR    0x0C
#define UART_LSR    0x14

#define LSR_DR      0x01
#define LSR_THRE    0x20
#define LCR_8N1     0x03
#define FCR_FIFO_EN 0x01
#define IER_RX_INT  0x01

static struct iridium_modem iridium_dev;

static void uart_write(uint32_t reg, uint32_t val)
{
    writel(val, iridium_dev.uart_base + reg);
}

static uint32_t uart_read(uint32_t reg)
{
    return readl(iridium_dev.uart_base + reg);
}

static bool uart_tx_ready(void)
{
    return uart_read(UART_LSR) & LSR_THRE;
}

static bool uart_rx_ready(void)
{
    return uart_read(UART_LSR) & LSR_DR;
}

static void uart_putc(uint8_t c)
{
    while (!uart_tx_ready())
        ;
    uart_write(UART_THR, c);
}

static uint8_t uart_getc(void)
{
    while (!uart_rx_ready())
        ;
    return uart_read(UART_RBR) & 0xFF;
}

static void uart_write_buf(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
        uart_putc(buf[i]);
}

static void uart_read_buf(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
        buf[i] = uart_getc();
}

static uint16_t sbd_checksum(const uint8_t *data, uint32_t len)
{
    uint16_t sum = 0;
    for (uint32_t i = 0; i < len; i++)
        sum += data[i];
    return sum;
}

static int sbd_send_command(uint8_t cmd, const uint8_t *payload, 
                             uint32_t payload_len, uint8_t *resp, 
                             uint32_t *resp_len)
{
    uint8_t frame[512];
    uint32_t pos = 0;
    uint16_t checksum;
    
    frame[pos++] = cmd;
    
    if (payload && payload_len > 0) {
        memcpy(&frame[pos], payload, payload_len);
        pos += payload_len;
    }
    
    checksum = sbd_checksum(frame, pos);
    frame[pos++] = (checksum >> 8) & 0xFF;
    frame[pos++] = checksum & 0xFF;
    
    uart_write_buf(frame, pos);
    
    if (resp && resp_len) {
        *resp_len = uart_getc();
        if (*resp_len > 0 && *resp_len < 512)
            uart_read_buf(resp, *resp_len);
    }
    
    return 0;
}

int iridium_init(void)
{
    spin_lock_init(&iridium_dev.lock);
    iridium_dev.state = IRIDIUM_IDLE;
    
    iridium_dev.uart_base = ioremap(IRIDIUM_UART_BASE, 0x100);
    if (!iridium_dev.uart_base)
        return -ENOMEM;
    
    uart_write(UART_FCR, FCR_FIFO_EN);
    uart_write(UART_LCR, LCR_8N1);
    
    uint32_t div = 48000000 / (16 * IRIDIUM_BAUDRATE);
    uart_write(UART_LCR, LCR_8N1 | 0x80);
    uart_write(UART_THR, div & 0xFF);
    uart_write(UART_IER, (div >> 8) & 0xFF);
    uart_write(UART_LCR, LCR_8N1);
    uart_write(UART_IER, IER_RX_INT);
    
    mdelay(100);
    
    iridium_dev.state = IRIDIUM_READY;
    
    return 0;
}

void iridium_deinit(void)
{
    uart_write(UART_IER, 0);
    iridium_dev.state = IRIDIUM_IDLE;
    iounmap(iridium_dev.uart_base);
}

int iridium_get_signal_strength(uint32_t *strength)
{
    uint8_t resp[16];
    uint32_t resp_len;
    int ret;
    
    ret = sbd_send_command(SBD_CMD_CHECK_STATUS, NULL, 0, resp, &resp_len);
    if (ret < 0)
        return ret;
    
    if (resp_len >= 1)
        *strength = resp[0];
    else
        *strength = 0;
    
    spin_lock(&iridium_dev.lock);
    iridium_dev.signal_strength = *strength;
    spin_unlock(&iridium_dev.lock);
    
    return 0;
}

int iridium_send_sbd(const uint8_t *data, uint32_t len)
{
    uint8_t resp[16];
    uint32_t resp_len;
    int ret;
    
    if (len > SBD_MO_MSG_SIZE)
        return -EINVAL;
    
    spin_lock(&iridium_dev.lock);
    iridium_dev.state = IRIDIUM_TRANSMITTING;
    memcpy(iridium_dev.mo_buffer, data, len);
    spin_unlock(&iridium_dev.lock);
    
    ret = sbd_send_command(SBD_CMD_MO_MSG_WRITE, data, len, resp, &resp_len);
    if (ret < 0)
        goto error;
    
    ret = sbd_send_command(SBD_CMD_INITIATE_SES, NULL, 0, resp, &resp_len);
    if (ret < 0)
        goto error;
    
    if (resp_len >= 1 && resp[0] == 0) {
        spin_lock(&iridium_dev.lock);
        iridium_dev.momsn++;
        iridium_dev.state = IRIDIUM_READY;
        spin_unlock(&iridium_dev.lock);
        return 0;
    }
    
error:
    spin_lock(&iridium_dev.lock);
    iridium_dev.state = IRIDIUM_ERROR;
    spin_unlock(&iridium_dev.lock);
    return -EIO;
}

int iridium_receive_sbd(uint8_t *data, uint32_t *len)
{
    uint8_t resp[SBD_MT_MSG_SIZE + 16];
    uint32_t resp_len;
    int ret;
    
    ret = sbd_send_command(SBD_CMD_MT_MSG_READ, NULL, 0, resp, &resp_len);
    if (ret < 0)
        return ret;
    
    if (resp_len > SBD_MT_HEADER_SIZE) {
        uint32_t msg_len = resp_len - SBD_MT_HEADER_SIZE;
        if (msg_len > *len)
            msg_len = *len;
        
        memcpy(data, &resp[SBD_MT_HEADER_SIZE], msg_len);
        *len = msg_len;
        
        spin_lock(&iridium_dev.lock);
        iridium_dev.mtmsn = (resp[1] << 8) | resp[2];
        spin_unlock(&iridium_dev.lock);
        
        return 0;
    }
    
    *len = 0;
    return 0;
}

bool iridium_is_ready(void)
{
    return iridium_dev.state == IRIDIUM_READY;
}

int iridium_poll_mt_message(void)
{
    uint8_t resp[16];
    uint32_t resp_len;
    
    sbd_send_command(SBD_CMD_CHECK_STATUS, NULL, 0, resp, &resp_len);
    
    if (resp_len >= 5 && resp[4] > 0) {
        spin_lock(&iridium_dev.lock);
        iridium_dev.ring_indicator = true;
        spin_unlock(&iridium_dev.lock);
        return resp[4];
    }
    
    return 0;
}
