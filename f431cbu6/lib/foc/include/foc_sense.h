#ifndef FOC_SENSE_H
#define FOC_SENSE_H

#include "foc_types.h"
#include "hal/hal_current.h"
#include "hal/hal_encoder.h"
#include "hal/hal_pwm.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { float u, v, w; } foc_offset_t;

typedef struct {
    foc_abc_t raw;
    foc_alphabeta_t ab;
    foc_dq_t dq;
    foc_offset_t offset;
    uint8_t calibrated;
} foc_current_ctx_t;

typedef struct {
    foc_angle_t angle;
    float speed;
    float offset_rad;
    float pole_pairs;
} foc_encoder_ctx_t;

typedef struct {
    foc_current_ctx_t current;
    foc_encoder_ctx_t encoder;
    float bus_voltage;
} foc_sense_t;

void foc_sense_current_init(foc_current_ctx_t *ctx);
void foc_sense_current_calib(foc_current_ctx_t *ctx,
                             const hal_current_t *hal);
void foc_sense_current_sample(foc_current_ctx_t *ctx,
                              const hal_current_t *hal);
void foc_sense_current_park(foc_current_ctx_t *ctx,
                            const foc_angle_t *angle);

void foc_sense_encoder_init(foc_encoder_ctx_t *ctx, float pole_pairs);
void foc_sense_encoder_update(foc_encoder_ctx_t *ctx,
                              const hal_encoder_t *hal);
void foc_sense_encoder_set_offset(foc_encoder_ctx_t *ctx,
                                  float offset_rad);
uint8_t foc_sense_encoder_align(foc_encoder_ctx_t *enc_ctx,
                                const hal_encoder_t *hal,
                                const hal_pwm_t *pwm,
                                void (*delay_ms)(uint32_t ms));

#ifdef __cplusplus
}
#endif

#endif
