#ifndef PI_CTRL_H
#define PI_CTRL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PI 控制器运行状态 */
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float out_min, out_max;
    float integral_limit;
    float out;
} foc_pi_t;

void pi_init(foc_pi_t *pi, float kp, float ki, float kd,
             float out_min, float out_max, float integral_limit);

void pi_reset(foc_pi_t *pi);

float pi_update(foc_pi_t *pi, float error, float dt);

void pi_set_kp(foc_pi_t *pi, float kp);
void pi_set_ki(foc_pi_t *pi, float ki);
void pi_set_kd(foc_pi_t *pi, float kd);
float pi_get_kp(const foc_pi_t *pi);
float pi_get_ki(const foc_pi_t *pi);
float pi_get_kd(const foc_pi_t *pi);

#ifdef __cplusplus
}
#endif

#endif
