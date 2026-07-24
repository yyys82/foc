#include "can_protocol.h"
#include "foc_config.h"
#include "foc_control.h"
#include <string.h>

void can_protocol_init(foc_can_t *ctx, foc_core_t *core,
                       const hal_can_t *hal, uint8_t node_id)
{
    ctx->core = core;
    ctx->hal = hal;
    ctx->node_id = node_id;
    ctx->tx_id_status = CAN_ID_STATUS + node_id;
    ctx->tx_id_ack = CAN_ID_ACK + node_id;
    ctx->tx_id_debug = CAN_ID_DEBUG_DATA + node_id;
    ctx->status_interval_ms = 100;
    ctx->last_status_ms = 0;

    if (hal && hal->init)
        hal->init(FOC_CAN_BAUDRATE);
}

static void _handle_ctrl(foc_can_t *ctx, uint8_t *data, uint8_t len)
{
    if (len < 1 || !ctx->core) return;

    uint8_t cmd = data[0];
    float value = 0.0f;
    if (len >= 5)
        memcpy(&value, &data[1], sizeof(float));

    switch (cmd)
    {
    case CAN_CTRL_IDLE:
        foc_control_set_mode(&ctx->core->ctrl, FOC_MODE_IDLE);
        break;
    case CAN_CTRL_POSITION:
        foc_control_set_mode(&ctx->core->ctrl, FOC_MODE_POSITION);
        foc_control_set_target_pos(&ctx->core->ctrl, value);
        break;
    case CAN_CTRL_SPEED:
        foc_control_set_mode(&ctx->core->ctrl, FOC_MODE_SPEED);
        foc_control_set_target_spd(&ctx->core->ctrl, value);
        break;
    case CAN_CTRL_TORQUE:
        foc_control_set_mode(&ctx->core->ctrl, FOC_MODE_TORQUE);
        foc_control_set_target_iq(&ctx->core->ctrl, value);
        break;
    case CAN_CTRL_ENABLE:
        foc_core_enable(ctx->core);
        break;
    case CAN_CTRL_DISABLE:
        foc_core_disable(ctx->core);
        break;
    case CAN_CTRL_STOP:
        foc_control_set_target_spd(&ctx->core->ctrl, 0.0f);
        foc_control_set_target_iq(&ctx->core->ctrl, 0.0f);
        break;
    default:
        break;
    }

    can_protocol_send_ack(ctx, CAN_ID_CTRL_BASE, 1);
}

static const struct {
    uint32_t loop;
    uint8_t  mask;
} _param_map[] = {
    [CAN_PARAM_POS_KP] = { FOC_CTRL_POS, 0x01 },
    [CAN_PARAM_POS_KI] = { FOC_CTRL_POS, 0x02 },
    [CAN_PARAM_SPD_KP] = { FOC_CTRL_SPD, 0x01 },
    [CAN_PARAM_SPD_KI] = { FOC_CTRL_SPD, 0x02 },
    [CAN_PARAM_CUR_KP] = { FOC_CTRL_IQ,  0x01 },
    [CAN_PARAM_CUR_KI] = { FOC_CTRL_IQ,  0x02 },
};

static void _handle_param_read(foc_can_t *ctx, uint8_t *data, uint8_t len)
{
    if (len < 1) return;

    uint8_t pid = data[0];
    float kp, ki, kd;
    foc_control_get_pid(&ctx->core->ctrl, _param_map[pid].loop, &kp, &ki, &kd);

    float val = 0.0f;
    if (_param_map[pid].mask & 0x01) val = kp;
    if (_param_map[pid].mask & 0x02) val = ki;

    uint8_t resp[8];
    memset(resp, 0, 8);
    resp[0] = pid;
    memcpy(&resp[1], &val, 4);

    if (ctx->hal && ctx->hal->send)
        ctx->hal->send(CAN_ID_PARAM_READ + ctx->node_id, resp, 8);
}

static void _handle_param_write(foc_can_t *ctx, uint8_t *data, uint8_t len)
{
    if (len < 5) return;

    uint8_t pid = data[0];
    float value;
    memcpy(&value, &data[1], sizeof(float));

    uint32_t loop = _param_map[pid].loop;
    uint8_t  mask = _param_map[pid].mask;

    float kp, ki, kd;
    foc_control_get_pid(&ctx->core->ctrl, loop, &kp, &ki, &kd);

    if (mask & 0x01) kp = value;
    if (mask & 0x02) ki = value;
    if (mask & 0x04) kd = value;

    foc_control_set_pid(&ctx->core->ctrl, loop, kp, ki, kd);
    can_protocol_send_ack(ctx, CAN_ID_PARAM_WRITE, 1);
}

void can_protocol_process_rx(foc_can_t *ctx, uint32_t id,
                             uint8_t *data, uint8_t len)
{
    if (!ctx || !data) return;

    if (id >= CAN_ID_CTRL_BASE && id < CAN_ID_CTRL_BASE + 0x40)
        _handle_ctrl(ctx, data, len);
    else if (id == CAN_ID_PARAM_READ)
        _handle_param_read(ctx, data, len);
    else if (id == CAN_ID_PARAM_WRITE)
        _handle_param_write(ctx, data, len);
}

void can_protocol_send_status(foc_can_t *ctx)
{
    if (!ctx || !ctx->core || !ctx->hal || !ctx->hal->send) return;

    uint8_t data[8];
    memset(data, 0, 8);

    float pos = foc_core_get_angle(ctx->core);
    float iq  = foc_core_get_iq(ctx->core);

    data[0] = (uint8_t)foc_core_get_state(ctx->core);
    data[1] = (uint8_t)foc_control_get_mode(&ctx->core->ctrl);
    memcpy(&data[2], &pos, 4);
    data[6] = (uint8_t)foc_core_get_fault(ctx->core);
    {
        int8_t iq_int = (int8_t)(iq * 10.0f);
        data[7] = (uint8_t)iq_int;
    }

    ctx->hal->send(ctx->tx_id_status, data, 8);
}

void can_protocol_send_ack(foc_can_t *ctx, uint32_t req_id, uint8_t status)
{
    if (!ctx || !ctx->hal || !ctx->hal->send) return;

    uint8_t data[8];
    memset(data, 0, 8);
    data[0] = (uint8_t)(req_id & 0xFF);
    data[1] = (uint8_t)((req_id >> 8) & 0xFF);
    data[2] = status;

    ctx->hal->send(ctx->tx_id_ack, data, 3);
}
