#include "foc_core.h"
#include "foc_config.h"
#include "foc_fault.h"
#include "park.h"
#include "svpwm.h"
#include <string.h>
#include <math.h>

void foc_core_init(foc_core_t *core,
                   const foc_motor_params_t *motor,
                   const hal_current_t *current_hal,
                   const hal_encoder_t *encoder_hal,
                   const hal_pwm_t *pwm_hal)
{
    memset(core, 0, sizeof(foc_core_t));

    if (motor) core->motor = *motor;

    core->current_hal = current_hal;
    core->encoder_hal = encoder_hal;
    core->pwm_hal     = pwm_hal;

    core->state      = FOC_STATE_IDLE;
    core->fault      = FOC_FAULT_NONE;
    core->enable     = 0;
    core->loop_count = 0;
    core->speed_div  = FOC_SPEED_LOOP_DIV;
    core->pos_div    = FOC_POS_LOOP_DIV;

    foc_sense_current_init(&core->sense.current);
    foc_sense_encoder_init(&core->sense.encoder,
                           motor ? (float)motor->pole_pairs : 1.0f);

    if (current_hal && current_hal->init)
        current_hal->init();
    if (encoder_hal && encoder_hal->init)
        encoder_hal->init();

    foc_control_init(&core->ctrl,
                     FOC_CURRENT_LIMIT_DEFAULT,
                     FOC_SPEED_LIMIT_DEFAULT,
                     FOC_VOLTAGE_LIMIT_DEFAULT,
                     FOC_DT_CURRENT, FOC_DT_SPEED, FOC_DT_POSITION);
}

void foc_core_enable(foc_core_t *core)
{
    core->enable = 1;
    core->state  = FOC_STATE_ALIGN;
}

void foc_core_disable(foc_core_t *core)
{
    core->enable = 0;
    core->state  = FOC_STATE_IDLE;
    foc_control_reset_all(&core->ctrl);

    if (core->pwm_hal)
    {
        if (core->pwm_hal->set_duty)
            core->pwm_hal->set_duty(0.0f, 0.0f, 0.0f);
        if (core->pwm_hal->enable)
            core->pwm_hal->enable(0);
    }
}

uint8_t      foc_core_is_enabled(const foc_core_t *core) { return core->enable; }
foc_state_t  foc_core_get_state(const foc_core_t *core)  { return core->state; }
foc_fault_t  foc_core_get_fault(const foc_core_t *core)  { return core->fault; }

void foc_core_clear_fault(foc_core_t *core)
{
    core->fault = FOC_FAULT_NONE;
    if (core->state == FOC_STATE_FAULT)
        core->state = FOC_STATE_IDLE;
}

float foc_core_get_angle(const foc_core_t *core) { return core->sense.encoder.angle.rad; }
float foc_core_get_speed(const foc_core_t *core) { return core->sense.encoder.speed; }
float foc_core_get_id(const foc_core_t *core)    { return core->sense.current.dq.d; }
float foc_core_get_iq(const foc_core_t *core)    { return core->sense.current.dq.q; }
float foc_core_get_vbus(const foc_core_t *core)  { return core->sense.bus_voltage; }

void foc_core_loop_current(foc_core_t *core)
{
    if (!core->enable || core->state != FOC_STATE_RUN)
    {
        if (core->pwm_hal && core->pwm_hal->set_duty)
            core->pwm_hal->set_duty(0.0f, 0.0f, 0.0f);
        return;
    }

    float vbus = core->ctrl.voltage_limit;
    if (core->pwm_hal && core->pwm_hal->get_vbus)
    {
        vbus = core->pwm_hal->get_vbus();
        if (vbus < 1.0f) vbus = core->ctrl.voltage_limit;
    }
    core->sense.bus_voltage = vbus;

    foc_sense_current_sample(&core->sense.current, core->current_hal);
    if (!core->sense.current.calibrated) return;

    foc_sense_encoder_update(&core->sense.encoder, core->encoder_hal);

    foc_sense_current_park(&core->sense.current, &core->sense.encoder.angle);

    float id = core->sense.current.dq.d;
    float iq = core->sense.current.dq.q;

    {
        float i_amp = sqrtf(id * id + iq * iq);
        foc_fault_t ft = foc_fault_check(
            i_amp, vbus,
            core->motor.rated_current, core->ctrl.voltage_limit);
        if (ft != FOC_FAULT_NONE)
        {
            core->fault = ft;
            core->state = FOC_STATE_FAULT;
            if (core->pwm_hal && core->pwm_hal->enable)
                core->pwm_hal->enable(0);
            return;
        }
    }

    foc_control_current_loop(&core->ctrl, id, iq, vbus);

    foc_dq_t v_dq = { .d = core->ctrl.vd, .q = core->ctrl.vq };
    foc_alphabeta_t v_ab;
    park_transform_inv(&v_dq, &core->sense.encoder.angle, &v_ab);

    foc_svpwm_t svpwm;
    svpwm_calc(&v_ab, vbus, &svpwm);
#if FOC_SVPWM_OVM_ENABLE
    svpwm_apply_ovm(&svpwm, FOC_SVPWM_OVM_LIMIT);
#endif

    if (core->pwm_hal && core->pwm_hal->set_duty)
        core->pwm_hal->set_duty(svpwm.ta, svpwm.tb, svpwm.tc);

    core->loop_count++;
}

void foc_core_loop_speed(foc_core_t *core)
{
    foc_control_speed_loop(&core->ctrl, core->sense.encoder.speed);
}

extern float g_dbg_mech;   /* hal_stm32g4_encoder.c: 多圈机械角(rad) */

void foc_core_loop_position(foc_core_t *core)
{
    /* 位置环用多圈机械角，不用电角度（电角度 wrap 会跳变，一圈内14个周期无法区分位置） */
    /* 输出 × pole_pairs → 电角速度，匹配速度环反馈单位 */
    foc_control_position_loop(&core->ctrl, g_dbg_mech);
    core->ctrl.target_spd *= core->sense.encoder.pole_pairs;
}
