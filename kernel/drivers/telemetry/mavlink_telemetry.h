#ifndef _MAVLINK_TELEMETRY_H
#define _MAVLINK_TELEMETRY_H

#include "linux/types.h"

#define MAVLINK_UART_BASE       0x3130000
#define MAVLINK_BAUDRATE        57600

#define MAVLINK_MAX_PAYLOAD_LEN 255
#define MAVLINK_NUM_CHECKSUM_BYTES 2
#define MAVLINK_SIGNATURE_BLOCK_LEN 13
#define MAVLINK_MAX_PACKET_LEN (MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_CHECKSUM_BYTES + 7)

#define MAVLINK_STX 0xFD
#define MAVLINK_STX_V1 0xFE
#define MAVLINK_STX_V2 0xFD

#define MAVLINK_MSG_ID_HEARTBEAT            0
#define MAVLINK_MSG_ID_SYS_STATUS           1
#define MAVLINK_MSG_ID_SYSTEM_TIME          2
#define MAVLINK_MSG_ID_GPS_RAW_INT          24
#define MAVLINK_MSG_ID_ATTITUDE             30
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT  33
#define MAVLINK_MSG_ID_VFR_HUD              74
#define MAVLINK_MSG_ID_COMMAND_LONG         76
#define MAVLINK_MSG_ID_COMMAND_ACK          77
#define MAVLINK_MSG_ID_DISTANCE_SENSOR      132
#define MAVLINK_MSG_ID_BATTERY_STATUS       147

struct __attribute__((packed)) mavlink_message {
    uint8_t magic;
    uint8_t len;
    uint8_t incompat_flags;
    uint8_t compat_flags;
    uint8_t seq;
    uint8_t sysid;
    uint8_t compid;
    uint8_t msgid[3];
    uint8_t payload[MAVLINK_MAX_PAYLOAD_LEN];
    uint8_t ck[2];
};

struct mavlink_link {
    void __iomem *uart_base;
    uint8_t system_id;
    uint8_t component_id;
    uint8_t tx_seq;
    spinlock_t tx_lock;
    spinlock_t rx_lock;
    uint8_t rx_buffer[MAVLINK_MAX_PACKET_LEN];
    uint32_t rx_pos;
};

struct mavlink_heartbeat {
    uint32_t custom_mode;
    uint8_t type;
    uint8_t autopilot;
    uint8_t base_mode;
    uint8_t system_status;
    uint8_t mavlink_version;
};

struct mavlink_gps_raw {
    uint64_t time_usec;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    uint16_t eph;
    uint16_t epv;
    uint16_t vel;
    uint16_t cog;
    uint8_t fix_type;
    uint8_t satellites_visible;
};

struct mavlink_attitude {
    uint32_t time_boot_ms;
    float roll;
    float pitch;
    float yaw;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
};

struct mavlink_global_position {
    uint32_t time_boot_ms;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t relative_alt;
    int16_t vx;
    int16_t vy;
    int16_t vz;
    uint16_t hdg;
};

int mavlink_init(uint8_t sysid, uint8_t compid);
void mavlink_deinit(void);
int mavlink_send_heartbeat(const struct mavlink_heartbeat *hb);
int mavlink_send_gps_raw(const struct mavlink_gps_raw *gps);
int mavlink_send_attitude(const struct mavlink_attitude *att);
int mavlink_send_global_position(const struct mavlink_global_position *pos);
int mavlink_receive(struct mavlink_message *msg, uint32_t timeout_ms);
int mavlink_send_command_ack(uint16_t command, uint8_t result);

#endif