#include "foc_control.h"
#include "foc_config.h"
#include <stddef.h>
#include <math.h>

void foc_control_init(foc_control_t *ctrl,
                      float current_limit, float speed_limit,
                      float voltage_limit,
                      float dt_current, float dt_speed, float dt_position)
{
    ctrl->mode = FOC_MODE_IDLE;
    ctrl->target_pos = 0.0f;
    ctrl->target_spd = 0.0f;
    ctrl->target_iq  = 0.0f;
    ctrl->target_id  = 0.0f;
    ctrl->vd = 0.0f;
    ctrl->vq = 0.0f;
    ctrl->dt_current  = dt_current;
    ctrl->dt_speed    = dt_speed;
    ctrl->dt_position = dt_position;
    ctrl->current_limit = current_limit;
    ctrl->speed_limit   = speed_limit;
    ctrl->voltage_limit = voltage_limit;

    float cur_out = voltage_limit;
    float cur_int = current_limit * 0.3f;

    pi_init(&ctrl->pi_id,  0.5f, 10.0f, 0.0f, -cur_out, cur_out, cur_int);
    pi_init(&ctrl->pi_iq,  0.5f, 10.0f, 0.0f, -cur_out, cur_out, cur_int);
    pi_init(&ctrl->pi_spd, 0.01f, 0.05f, 0.0f, -current_limit, current_limit, current_limit * 0.1f);
    pi_init(&ctrl->pi_pos, 2.0f, 0.0f,  0.0f, -speed_limit, speed_limit, speed_limit * 0.5f);
}

void foc_control_reset_all(foc_control_t *ctrl)
{
    pi_reset(&ctrl->pi_pos);
    pi_reset(&ctrl->pi_spd);
    pi_reset(&ctrl->pi_id);
    pi_reset(&ctrl->pi_iq);
}

void foc_control_set_mode(foc_control_t *ctrl, foc_ctrl_mode_t mode)
{
    ctrl->mode = mode;
    pi_reset(&ctrl->pi_pos);
    pi_reset(&ctrl->pi_spd);
}

foc_ctrl_mode_t foc_control_get_mode(const foc_control_t *ctrl)
{
    return ctrl->mode;
}

void foc_control_set_target_pos(foc_control_t *ctrl, float pos) { ctrl->target_pos = pos; }
void foc_control_set_target_spd(foc_control_t *ctrl, float spd) { ctrl->target_spd = spd; }
void foc_control_set_target_iq(foc_control_t *ctrl, float iq)   { ctrl->target_iq  = iq; }

void foc_control_get_targets(const foc_control_t *ctrl,
                             float *pos, float *spd, float *iq)
{
    if (pos) *pos = ctrl->target_pos;
    if (spd) *spd = ctrl->target_spd;
    if (iq)  *iq  = ctrl->target_iq;
}

void foc_control_set_limits(foc_control_t *ctrl,
                            float current, float speed, float voltage)
{
    ctrl->current_limit = current;
    ctrl->speed_limit   = speed;
    ctrl->voltage_limit = voltage;
}

void foc_control_current_loop(foc_control_t *ctrl,
                              float id_fb, float iq_fb, float vbus)
{
    ctrl->vd = 0.0f;
    ctrl->vq = pi_update(&ctrl->pi_iq, ctrl->target_iq - iq_fb, ctrl->dt_current);

    float vmax = vbus * FOC_VOLTAGE_LIMIT_RATIO * FOC_INV_SQRT3;
    float v_mag = sqrtf(ctrl->vd * ctrl->vd + ctrl->vq * ctrl->vq);
    if (v_mag > vmax && v_mag > 0.0f)
    {
        float scale = vmax / v_mag;
        ctrl->vd *= scale;
        ctrl->vq *= scale;
    }
}

void foc_control_speed_loop(foc_control_t *ctrl, float speed_fb)
{
    if (ctrl->mode == FOC_MODE_IDLE)    return;
    if (ctrl->mode == FOC_MODE_TORQUE)  return;

    ctrl->target_iq = pi_update(&ctrl->pi_spd,
                                ctrl->target_spd - speed_fb,
                                ctrl->dt_speed);

    if (ctrl->target_iq > ctrl->current_limit)
        ctrl->target_iq = ctrl->current_limit;
    else if (ctrl->target_iq < -ctrl->current_limit)
        ctrl->target_iq = -ctrl->current_limit;
}

void foc_control_position_loop(foc_control_t *ctrl, float pos_fb)
{
    if (ctrl->mode != FOC_MODE_POSITION) return;

    ctrl->target_spd = pi_update(&ctrl->pi_pos,
                                  ctrl->target_pos - pos_fb,
                                  ctrl->dt_position);

    if (ctrl->target_spd > ctrl->speed_limit)
        ctrl->target_spd = ctrl->speed_limit;
    else if (ctrl->target_spd < -ctrl->speed_limit)
        ctrl->target_spd = -ctrl->speed_limit;
}

void foc_control_get_voltage(const foc_control_t *ctrl, float *vd, float *vq)
{
    if (vd) *vd = ctrl->vd;
    if (vq) *vq = ctrl->vq;
}

void foc_control_set_pid(foc_control_t *ctrl,
                         uint32_t loop, float kp, float ki, float kd)
{
    foc_pi_t *pi = NULL;
    switch (loop)
    {
    case FOC_CTRL_POS: pi = &ctrl->pi_pos; break;
    case FOC_CTRL_SPD: pi = &ctrl->pi_spd; break;
    case FOC_CTRL_ID:  pi = &ctrl->pi_id;  break;
    case FOC_CTRL_IQ:  pi = &ctrl->pi_iq;  break;
    default: return;
    }
    pi_set_kp(pi, kp);
    pi_set_ki(pi, ki);
    pi_set_kd(pi, kd);
}

void foc_control_get_pid(const foc_control_t *ctrl,
                         uint32_t loop, float *kp, float *ki, float *kd)
{
    if (!kp || !ki || !kd) return;

    const foc_pi_t *pi = NULL;
    switch (loop)
    {
    case FOC_CTRL_POS: pi = &ctrl->pi_pos; break;
    case FOC_CTRL_SPD: pi = &ctrl->pi_spd; break;
    case FOC_CTRL_ID:  pi = &ctrl->pi_id;  break;
    case FOC_CTRL_IQ:  pi = &ctrl->pi_iq;  break;
    default: return;
    }
    *kp = pi_get_kp(pi);
    *ki = pi_get_ki(pi);
    *kd = pi_get_kd(pi);
}
