#include "mavlink_telemetry.h"
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

static const uint8_t MAVLINK_MESSAGE_CRCS[] = {
    50, 124, 137, 0, 237, 217, 104, 119, 0, 0,
    0, 89, 0, 0, 0, 0, 0, 0, 0, 0,
    214, 159, 220, 168, 24, 23, 170, 144, 67, 115,
    39, 246, 185, 104, 237, 244, 222, 212, 9, 254,
    230, 28, 28, 132, 221, 232, 11, 153, 41, 39,
    78, 0, 0, 0, 15, 3, 0, 0, 0, 0,
    0, 153, 183, 35, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 90, 104, 85, 95, 130,
    0, 0, 0, 8, 204, 49, 170, 44, 83, 46,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

static struct mavlink_link mavlink_dev;

static uint16_t mavlink_crc_accumulate(uint8_t data, uint16_t crc_acc)
{
    uint8_t tmp;
    tmp = data ^ (crc_acc & 0xff);
    tmp ^= (tmp << 4);
    crc_acc = (crc_acc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    return crc_acc;
}

static uint16_t mavlink_finalize_message_buffer(struct mavlink_message *msg, 
                                                 uint8_t system_id,
                                                 uint8_t component_id,
                                                 uint8_t length,
                                                 uint32_t msgid)
{
    uint16_t crc_acc = 0xFFFF;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint32_t pos = 0;
    
    msg->magic = MAVLINK_STX_V2;
    msg->len = length;
    msg->incompat_flags = 0;
    msg->compat_flags = 0;
    msg->seq = mavlink_dev.tx_seq++;
    msg->sysid = system_id;
    msg->compid = component_id;
    msg->msgid[0] = msgid & 0xFF;
    msg->msgid[1] = (msgid >> 8) & 0xFF;
    msg->msgid[2] = (msgid >> 16) & 0xFF;
    
    buf[pos++] = msg->magic;
    buf[pos++] = msg->len;
    buf[pos++] = msg->incompat_flags;
    buf[pos++] = msg->compat_flags;
    buf[pos++] = msg->seq;
    buf[pos++] = msg->sysid;
    buf[pos++] = msg->compid;
    buf[pos++] = msg->msgid[0];
    buf[pos++] = msg->msgid[1];
    buf[pos++] = msg->msgid[2];
    
    for (int i = 0; i < length; i++)
        buf[pos++] = msg->payload[i];
    
    for (uint32_t i = 1; i < pos; i++)
        crc_acc = mavlink_crc_accumulate(buf[i], crc_acc);
    
    crc_acc = mavlink_crc_accumulate(MAVLINK_MESSAGE_CRCS[msgid < sizeof(MAVLINK_MESSAGE_CRCS) ? msgid : 0], crc_acc);
    
    msg->ck[0] = crc_acc & 0xFF;
    msg->ck[1] = (crc_acc >> 8) & 0xFF;
    
    return crc_acc;
}

static void uart_putc(uint8_t c)
{
    while (!(readl(mavlink_dev.uart_base + UART_LSR) & LSR_THRE))
        ;
    writel(c, mavlink_dev.uart_base + UART_THR);
}

static void uart_write(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
        uart_putc(data[i]);
}

static int uart_getc_timeout(uint8_t *c, uint32_t timeout_ms)
{
    uint64_t start = get_time_us();
    
    while ((get_time_us() - start) < (timeout_ms * 1000)) {
        if (readl(mavlink_dev.uart_base + UART_LSR) & LSR_DR) {
            *c = readl(mavlink_dev.uart_base + UART_RBR) & 0xFF;
            return 0;
        }
    }
    return -ETIMEDOUT;
}

int mavlink_init(uint8_t sysid, uint8_t compid)
{
    mavlink_dev.system_id = sysid;
    mavlink_dev.component_id = compid;
    mavlink_dev.tx_seq = 0;
    spin_lock_init(&mavlink_dev.tx_lock);
    spin_lock_init(&mavlink_dev.rx_lock);
    
    mavlink_dev.uart_base = ioremap(MAVLINK_UART_BASE, 0x100);
    if (!mavlink_dev.uart_base)
        return -ENOMEM;
    
    writel(FCR_FIFO_EN, mavlink_dev.uart_base + UART_FCR);
    writel(LCR_8N1, mavlink_dev.uart_base + UART_LCR);
    
    uint32_t div = 48000000 / (16 * MAVLINK_BAUDRATE);
    writel(LCR_8N1 | 0x80, mavlink_dev.uart_base + UART_LCR);
    writel(div & 0xFF, mavlink_dev.uart_base + UART_THR);
    writel((div >> 8) & 0xFF, mavlink_dev.uart_base + UART_IER);
    writel(LCR_8N1, mavlink_dev.uart_base + UART_LCR);
    writel(IER_RX_INT, mavlink_dev.uart_base + UART_IER);
    
    return 0;
}

void mavlink_deinit(void)
{
    writel(0, mavlink_dev.uart_base + UART_IER);
    iounmap(mavlink_dev.uart_base);
}

static int mavlink_send_message(struct mavlink_message *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint32_t len = 0;
    
    buf[len++] = msg->magic;
    buf[len++] = msg->len;
    buf[len++] = msg->incompat_flags;
    buf[len++] = msg->compat_flags;
    buf[len++] = msg->seq;
    buf[len++] = msg->sysid;
    buf[len++] = msg->compid;
    buf[len++] = msg->msgid[0];
    buf[len++] = msg->msgid[1];
    buf[len++] = msg->msgid[2];
    
    memcpy(&buf[len], msg->payload, msg->len);
    len += msg->len;
    buf[len++] = msg->ck[0];
    buf[len++] = msg->ck[1];
    
    spin_lock(&mavlink_dev.tx_lock);
    uart_write(buf, len);
    spin_unlock(&mavlink_dev.tx_lock);
    
    return 0;
}

int mavlink_send_heartbeat(const struct mavlink_heartbeat *hb)
{
    struct mavlink_message msg;
    uint32_t pos = 0;
    
    memcpy(&msg.payload[pos], &hb->custom_mode, sizeof(hb->custom_mode));
    pos += sizeof(hb->custom_mode);
    msg.payload[pos++] = hb->type;
    msg.payload[pos++] = hb->autopilot;
    msg.payload[pos++] = hb->base_mode;
    msg.payload[pos++] = hb->system_status;
    msg.payload[pos++] = hb->mavlink_version;
    
    mavlink_finalize_message_buffer(&msg, mavlink_dev.system_id, 
                                    mavlink_dev.component_id, pos,
                                    MAVLINK_MSG_ID_HEARTBEAT);
    
    return mavlink_send_message(&msg);
}

int mavlink_send_gps_raw(const struct mavlink_gps_raw *gps)
{
    struct mavlink_message msg;
    uint32_t pos = 0;
    
    memcpy(&msg.payload[pos], &gps->time_usec, sizeof(gps->time_usec));
    pos += sizeof(gps->time_usec);
    memcpy(&msg.payload[pos], &gps->lat, sizeof(gps->lat));
    pos += sizeof(gps->lat);
    memcpy(&msg.payload[pos], &gps->lon, sizeof(gps->lon));
    pos += sizeof(gps->lon);
    memcpy(&msg.payload[pos], &gps->alt, sizeof(gps->alt));
    pos += sizeof(gps->alt);
    memcpy(&msg.payload[pos], &gps->eph, sizeof(gps->eph));
    pos += sizeof(gps->eph);
    memcpy(&msg.payload[pos], &gps->epv, sizeof(gps->epv));
    pos += sizeof(gps->epv);
    memcpy(&msg.payload[pos], &gps->vel, sizeof(gps->vel));
    pos += sizeof(gps->vel);
    memcpy(&msg.payload[pos], &gps->cog, sizeof(gps->cog));
    pos += sizeof(gps->cog);
    msg.payload[pos++] = gps->fix_type;
    msg.payload[pos++] = gps->satellites_visible;
    
    mavlink_finalize_message_buffer(&msg, mavlink_dev.system_id,
                                    mavlink_dev.component_id, pos,
                                    MAVLINK_MSG_ID_GPS_RAW_INT);
    
    return mavlink_send_message(&msg);
}

int mavlink_send_attitude(const struct mavlink_attitude *att)
{
    struct mavlink_message msg;
    uint32_t pos = 0;
    
    memcpy(&msg.payload[pos], &att->time_boot_ms, sizeof(att->time_boot_ms));
    pos += sizeof(att->time_boot_ms);
    memcpy(&msg.payload[pos], &att->roll, sizeof(att->roll));
    pos += sizeof(att->roll);
    memcpy(&msg.payload[pos], &att->pitch, sizeof(att->pitch));
    pos += sizeof(att->pitch);
    memcpy(&msg.payload[pos], &att->yaw, sizeof(att->yaw));
    pos += sizeof(att->yaw);
    memcpy(&msg.payload[pos], &att->rollspeed, sizeof(att->rollspeed));
    pos += sizeof(att->rollspeed);
    memcpy(&msg.payload[pos], &att->pitchspeed, sizeof(att->pitchspeed));
    pos += sizeof(att->pitchspeed);
    memcpy(&msg.payload[pos], &att->yawspeed, sizeof(att->yawspeed));
    pos += sizeof(att->yawspeed);
    
    mavlink_finalize_message_buffer(&msg, mavlink_dev.system_id,
                                    mavlink_dev.component_id, pos,
                                    MAVLINK_MSG_ID_ATTITUDE);
    
    return mavlink_send_message(&msg);
}

int mavlink_send_global_position(const struct mavlink_global_position *pos)
{
    struct mavlink_message msg;
    uint32_t p = 0;
    
    memcpy(&msg.payload[p], &pos->time_boot_ms, sizeof(pos->time_boot_ms));
    p += sizeof(pos->time_boot_ms);
    memcpy(&msg.payload[p], &pos->lat, sizeof(pos->lat));
    p += sizeof(pos->lat);
    memcpy(&msg.payload[p], &pos->lon, sizeof(pos->lon));
    p += sizeof(pos->lon);
    memcpy(&msg.payload[p], &pos->alt, sizeof(pos->alt));
    p += sizeof(pos->alt);
    memcpy(&msg.payload[p], &pos->relative_alt, sizeof(pos->relative_alt));
    p += sizeof(pos->relative_alt);
    memcpy(&msg.payload[p], &pos->vx, sizeof(pos->vx));
    p += sizeof(pos->vx);
    memcpy(&msg.payload[p], &pos->vy, sizeof(pos->vy));
    p += sizeof(pos->vy);
    memcpy(&msg.payload[p], &pos->vz, sizeof(pos->vz));
    p += sizeof(pos->vz);
    memcpy(&msg.payload[p], &pos->hdg, sizeof(pos->hdg));
    p += sizeof(pos->hdg);
    
    mavlink_finalize_message_buffer(&msg, mavlink_dev.system_id,
                                    mavlink_dev.component_id, p,
                                    MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
    
    return mavlink_send_message(&msg);
}

int mavlink_receive(struct mavlink_message *msg, uint32_t timeout_ms)
{
    uint8_t c;
    int ret;
    uint64_t start = get_time_us();
    
    while ((get_time_us() - start) < (timeout_ms * 1000)) {
        ret = uart_getc_timeout(&c, 10);
        if (ret < 0)
            continue;
        
        if (c == MAVLINK_STX_V2) {
            msg->magic = c;
            
            if (uart_getc_timeout(&msg->len, 100) < 0) continue;
            if (uart_getc_timeout(&msg->incompat_flags, 100) < 0) continue;
            if (uart_getc_timeout(&msg->compat_flags, 100) < 0) continue;
            if (uart_getc_timeout(&msg->seq, 100) < 0) continue;
            if (uart_getc_timeout(&msg->sysid, 100) < 0) continue;
            if (uart_getc_timeout(&msg->compid, 100) < 0) continue;
            if (uart_getc_timeout(&msg->msgid[0], 100) < 0) continue;
            if (uart_getc_timeout(&msg->msgid[1], 100) < 0) continue;
            if (uart_getc_timeout(&msg->msgid[2], 100) < 0) continue;
            
            for (int i = 0; i < msg->len; i++) {
                if (uart_getc_timeout(&msg->payload[i], 100) < 0)
                    return -EIO;
            }
            
            if (uart_getc_timeout(&msg->ck[0], 100) < 0) continue;
            if (uart_getc_timeout(&msg->ck[1], 100) < 0) continue;
            
            return 0;
        }
    }
    
    return -ETIMEDOUT;
}

int mavlink_send_command_ack(uint16_t command, uint8_t result)
{
    struct mavlink_message msg;
    uint32_t pos = 0;
    
    memcpy(&msg.payload[pos], &command, sizeof(command));
    pos += sizeof(command);
    msg.payload[pos++] = result;
    msg.payload[pos++] = 0;
    msg.payload[pos++] = 0;
    msg.payload[pos++] = 0;
    msg.payload[pos++] = 0;
    msg.payload[pos++] = 0;
    msg.payload[pos++] = 0;
    msg.payload[pos++] = 0;
    msg.payload[pos++] = 0;
    
    mavlink_finalize_message_buffer(&msg, mavlink_dev.system_id,
                                    mavlink_dev.component_id, pos,
                                    MAVLINK_MSG_ID_COMMAND_ACK);
    
    return mavlink_send_message(&msg);
}
