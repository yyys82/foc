#include "pi_ctrl.h"

void pi_init(foc_pi_t *pi, float kp, float ki, float kd,
             float out_min, float out_max, float integral_limit)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->kd = kd;
    pi->out_min = out_min;
    pi->out_max = out_max;
    pi->integral_limit = integral_limit;
    pi->integral = 0.0f;
    pi->prev_error = 0.0f;
    pi->out = 0.0f;
}

void pi_reset(foc_pi_t *pi)
{
    pi->integral = 0.0f;
    pi->prev_error = 0.0f;
    pi->out = 0.0f;
}

float pi_update(foc_pi_t *pi, float error, float dt)
{
    float p_term = pi->kp * error;

    pi->integral += pi->ki * error * dt;
    if (pi->integral > pi->integral_limit)
        pi->integral = pi->integral_limit;
    else if (pi->integral < -pi->integral_limit)
        pi->integral = -pi->integral_limit;

    float d_term = pi->kd * (error - pi->prev_error) / dt;

    float out = p_term + pi->integral + d_term;

    if (out > pi->out_max)
    {
        out = pi->out_max;
        pi->integral -= pi->ki * error * dt;
    }
    else if (out < pi->out_min)
    {
        out = pi->out_min;
        pi->integral -= pi->ki * error * dt;
    }

    pi->prev_error = error;
    pi->out = out;

    return out;
}

void pi_set_kp(foc_pi_t *pi, float kp) { pi->kp = kp; }
void pi_set_ki(foc_pi_t *pi, float ki) { pi->ki = ki; }
void pi_set_kd(foc_pi_t *pi, float kd) { pi->kd = kd; }
float pi_get_kp(const foc_pi_t *pi) { return pi->kp; }
float pi_get_ki(const foc_pi_t *pi) { return pi->ki; }
float pi_get_kd(const foc_pi_t *pi) { return pi->kd; }
