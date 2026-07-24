#include "svpwm.h"
#include "foc_config.h"
#include <math.h>

void svpwm_calc(const foc_alphabeta_t *v_ab, float vdc, foc_svpwm_t *svpwm)
{
    float v_alpha = v_ab->alpha;
    float v_beta  = v_ab->beta;
    float vdc_inv = 1.0f / vdc;

    float v_a = v_alpha * vdc_inv;
    float v_b = (-0.5f * v_alpha + FOC_SQRT3_2 * v_beta) * vdc_inv;
    float v_c = (-0.5f * v_alpha - FOC_SQRT3_2 * v_beta) * vdc_inv;

    float v_min = fminf(fminf(v_a, v_b), v_c);
    float v_max = fmaxf(fmaxf(v_a, v_b), v_c);
    float v_offset = -0.5f * (v_min + v_max);

    float ta = v_a + v_offset + 0.5f;
    float tb = v_b + v_offset + 0.5f;
    float tc = v_c + v_offset + 0.5f;

    float t_min = FOC_SVPWM_MIN_PULSE;
    float t_max = FOC_SVPWM_MAX_PULSE;
    if (ta < t_min) ta = t_min;
    if (tb < t_min) tb = t_min;
    if (tc < t_min) tc = t_min;
    if (ta > t_max) ta = t_max;
    if (tb > t_max) tb = t_max;
    if (tc > t_max) tc = t_max;

    svpwm->ta = ta;
    svpwm->tb = tb;
    svpwm->tc = tc;
    svpwm->sector = 0;
    svpwm->t1 = 0.0f;
    svpwm->t2 = 0.0f;
    svpwm->adc_trig_u = ta * 0.5f;
    svpwm->adc_trig_v = tb * 0.5f;
}

void svpwm_apply_ovm(foc_svpwm_t *svpwm, float limit)
{
    float ta = svpwm->ta;
    float tb = svpwm->tb;
    float tc = svpwm->tc;

    if (ta > limit || tb > limit || tc > limit)
    {
        float max_d = fmaxf(fmaxf(ta, tb), tc);
        float scale = limit / max_d;
        svpwm->ta = ta * scale;
        svpwm->tb = tb * scale;
        svpwm->tc = tc * scale;
    }
}
