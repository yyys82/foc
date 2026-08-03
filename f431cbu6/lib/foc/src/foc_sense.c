#include "foc_sense.h"
#include "clarke.h"
#include "park.h"
#include "svpwm.h"
#include "foc_config.h"
#include <math.h>

void foc_sense_current_init(foc_current_ctx_t *ctx)
{
    ctx->raw.a = ctx->raw.b = ctx->raw.c = 0.0f;
    ctx->ab.alpha = ctx->ab.beta = 0.0f;
    ctx->dq.d = ctx->dq.q = 0.0f;
    ctx->offset.u = ctx->offset.v = ctx->offset.w = 0.0f;
    ctx->calibrated = 0;
}

void foc_sense_current_calib(foc_current_ctx_t *ctx, const hal_current_t *hal)
{
    if (!hal || !hal->read) return;

    uint32_t n = 256;
    float sa = 0.0f, sb = 0.0f, sc = 0.0f;

    for (uint32_t i = 0; i < n; i++)
    {
        float ia, ib, ic;
        hal->read(&ia, &ib, &ic);
        sa += ia; sb += ib; sc += ic;
        if (hal->delay_us) hal->delay_us(100);
    }

    ctx->offset.u = sa / (float)n;
    ctx->offset.v = sb / (float)n;
    ctx->offset.w = sc / (float)n;
    ctx->calibrated = 1;
}

void foc_sense_current_sample(foc_current_ctx_t *ctx, const hal_current_t *hal)
{
    if (!hal || !hal->read) return;

    float ia, ib, ic;
    hal->read(&ia, &ib, &ic);

    if (ctx->calibrated)
    {
        ia -= ctx->offset.u;
        ib -= ctx->offset.v;
        ic -= ctx->offset.w;
    }

    ctx->raw.a = ia;
    ctx->raw.b = ib;
    ctx->raw.c = ic;

    foc_abc_t abc = { .a = ia, .b = ib, .c = ic };
    clarke_transform(&abc, &ctx->ab);
}

void foc_sense_current_park(foc_current_ctx_t *ctx, const foc_angle_t *angle)
{
    park_transform(&ctx->ab, angle, &ctx->dq);
}

void foc_sense_encoder_init(foc_encoder_ctx_t *ctx, float pole_pairs)
{
    ctx->angle.rad = 0.0f;
    ctx->angle.sin = 0.0f;
    ctx->angle.cos = 1.0f;
    ctx->speed = 0.0f;
    ctx->offset_rad = 0.0f;
    ctx->pole_pairs = pole_pairs;
}

static float _wrap(float a)
{
    /* O(1) 归一化到 (-π, π]：get_angle 返回无界多圈角，×极对数后幅度随圈数线性增长，
     * 若用 while 迭代减 2π 会变成 O(圈数)，持续旋转几十圈后拖垮 25kHz 电流环 ISR。 */
    a -= roundf(a / FOC_2PI) * FOC_2PI;
    return a;
}

void foc_sense_encoder_update(foc_encoder_ctx_t *ctx, const hal_encoder_t *hal)
{
    if (!hal || !hal->get_angle) return;

    float mech = hal->get_angle();
    float elec = mech * ctx->pole_pairs - ctx->offset_rad + FOC_FRAME_CORR;
    elec = _wrap(elec);

    ctx->angle.rad = elec;
    ctx->angle.sin = sinf(elec);
    ctx->angle.cos = cosf(elec);

    if (hal->get_speed)
        ctx->speed = hal->get_speed() * ctx->pole_pairs;
}

void foc_sense_encoder_set_offset(foc_encoder_ctx_t *ctx, float offset_rad)
{
    ctx->offset_rad = offset_rad;
}

uint8_t foc_sense_encoder_align(foc_encoder_ctx_t *enc_ctx,
                                const hal_encoder_t *hal,
                                const hal_pwm_t *pwm,
                                void (*delay_ms)(uint32_t ms))
{
    if (!pwm || !pwm->set_duty || !hal || !hal->get_angle) return 0;

    foc_angle_t a0 = { .rad = 0.0f, .sin = 0.0f, .cos = 1.0f };
    foc_dq_t v0 = { .d = 4.0f, .q = 0.0f };
    foc_alphabeta_t ab0;
    park_transform_inv(&v0, &a0, &ab0);

    float vbus = pwm->get_vbus ? pwm->get_vbus() : FOC_VOLTAGE_LIMIT_DEFAULT;

    foc_svpwm_t sv0;
    svpwm_calc(&ab0, vbus, &sv0);
    pwm->set_duty(sv0.ta, sv0.tb, sv0.tc);
    if (pwm->enable) pwm->enable(1);

    if (delay_ms) delay_ms(500);

    float mech = hal->get_angle();
    foc_sense_encoder_set_offset(enc_ctx, mech * enc_ctx->pole_pairs);

    pwm->set_duty(0.0f, 0.0f, 0.0f);
    return 1;
}
