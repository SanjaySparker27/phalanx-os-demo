#ifndef _IRIDIUM_SATCOM_H
#define _IRIDIUM_SATCOM_H

#include "linux/types.h"

#define IRIDIUM_UART_BASE   0x3140000
#define IRIDIUM_BAUDRATE    19200

#define SBD_MO_MSG_SIZE     340
#define SBD_MT_MSG_SIZE     270
#define SBD_MT_HEADER_SIZE  3

#define SBD_CMD_CHECK_STATUS    0x01
#define SBD_CMD_INITIATE_SES    0x02
#define SBD_CMD_MT_MSG_POLL     0x03
#define SBD_CMD_MT_MSG_READ     0x04
#define SBD_CMD_MO_MSG_WRITE    0x05

enum iridium_state {
    IRIDIUM_IDLE,
    IRIDIUM_REGISTERING,
    IRIDIUM_READY,
    IRIDIUM_TRANSMITTING,
    IRIDIUM_RECEIVING,
    IRIDIUM_ERROR,
};

struct iridium_modem {
    void __iomem *uart_base;
    enum iridium_state state;
    uint32_t signal_strength;
    uint32_t momsn;
    uint32_t mtmsn;
    bool ring_indicator;
    spinlock_t lock;
    uint8_t mo_buffer[SBD_MO_MSG_SIZE];
    uint8_t mt_buffer[SBD_MT_MSG_SIZE];
};

struct sbd_status {
    uint8_t mo_status;
    uint8_t mt_status;
    uint16_t mt_msg_length;
    uint8_t mt_queued;
    uint8_t ring_pending;
};

int iridium_init(void);
void iridium_deinit(void);
int iridium_send_sbd(const uint8_t *data, uint32_t len);
int iridium_receive_sbd(uint8_t *data, uint32_t *len);
int iridium_get_signal_strength(uint32_t *strength);
bool iridium_is_ready(void);
int iridium_poll_mt_message(void);

#endif