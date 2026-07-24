#ifndef FOC_CONTROL_H
#define FOC_CONTROL_H

#include "foc_types.h"
#include "pi_ctrl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 控制环参数存取索引 */
#define FOC_CTRL_POS  0
#define FOC_CTRL_SPD  1
#define FOC_CTRL_ID   2
#define FOC_CTRL_IQ   3

/* 三级级联控制上下文 */
typedef struct {
    foc_ctrl_mode_t mode;

    foc_pi_t pi_pos;
    foc_pi_t pi_spd;
    foc_pi_t pi_id;
    foc_pi_t pi_iq;

    float target_pos;
    float target_spd;
    float target_iq;
    float target_id;

    float vd, vq;

    float dt_current;
    float dt_speed;
    float dt_position;

    float current_limit;
    float speed_limit;
    float voltage_limit;
} foc_control_t;

void foc_control_init(foc_control_t *ctrl,
                      float current_limit, float speed_limit,
                      float voltage_limit,
                      float dt_current, float dt_speed, float dt_position);

void foc_control_reset_all(foc_control_t *ctrl);

void foc_control_set_mode(foc_control_t *ctrl, foc_ctrl_mode_t mode);
foc_ctrl_mode_t foc_control_get_mode(const foc_control_t *ctrl);

void foc_control_set_target_pos(foc_control_t *ctrl, float pos);
void foc_control_set_target_spd(foc_control_t *ctrl, float spd);
void foc_control_set_target_iq(foc_control_t *ctrl, float iq);
void foc_control_get_targets(const foc_control_t *ctrl,
                             float *pos, float *spd, float *iq);

void foc_control_set_limits(foc_control_t *ctrl,
                            float current, float speed, float voltage);

void foc_control_current_loop(foc_control_t *ctrl,
                              float id_fb, float iq_fb, float vbus);

void foc_control_speed_loop(foc_control_t *ctrl, float speed_fb);

void foc_control_position_loop(foc_control_t *ctrl, float pos_fb);

void foc_control_get_voltage(const foc_control_t *ctrl,
                             float *vd, float *vq);

void foc_control_set_pid(foc_control_t *ctrl,
                         uint32_t loop, float kp, float ki, float kd);

void foc_control_get_pid(const foc_control_t *ctrl,
                         uint32_t loop, float *kp, float *ki, float *kd);

#ifdef __cplusplus
}
#endif

#endif
