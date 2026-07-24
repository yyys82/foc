#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>
#include "foc_types.h"
#include "hal/hal_can.h"
#include "foc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_ID_CTRL_BASE        0x100
#define CAN_ID_PARAM_READ       0x140
#define CAN_ID_PARAM_WRITE      0x141
#define CAN_ID_STATUS           0x180
#define CAN_ID_ACK              0x1C0
#define CAN_ID_DEBUG_DATA       0x190

#define CAN_CTRL_IDLE           0
#define CAN_CTRL_POSITION       1
#define CAN_CTRL_SPEED          2
#define CAN_CTRL_TORQUE         3
#define CAN_CTRL_ENABLE         4
#define CAN_CTRL_DISABLE        5
#define CAN_CTRL_STOP           6
#define CAN_CTRL_CALIB          7
#define CAN_CTRL_ALIGN          8

#define CAN_PARAM_POS_KP        0
#define CAN_PARAM_POS_KI        1
#define CAN_PARAM_SPD_KP        2
#define CAN_PARAM_SPD_KI        3
#define CAN_PARAM_CUR_KP        4
#define CAN_PARAM_CUR_KI        5
#define CAN_PARAM_CUR_LIMIT     6
#define CAN_PARAM_SPD_LIMIT     7

typedef struct {
    foc_core_t *core;
    const hal_can_t *hal;
    uint32_t tx_id_status;
    uint32_t tx_id_ack;
    uint32_t tx_id_debug;
    uint32_t status_interval_ms;
    uint32_t last_status_ms;
    uint8_t node_id;
} foc_can_t;

void can_protocol_init(foc_can_t *ctx, foc_core_t *core,
                       const hal_can_t *hal, uint8_t node_id);

void can_protocol_process_rx(foc_can_t *ctx, uint32_t id,
                             uint8_t *data, uint8_t len);

void can_protocol_send_status(foc_can_t *ctx);

void can_protocol_send_ack(foc_can_t *ctx, uint32_t req_id, uint8_t status);

#ifdef __cplusplus
}
#endif

#endif
