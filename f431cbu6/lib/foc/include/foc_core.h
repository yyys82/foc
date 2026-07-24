#ifndef FOC_CORE_H
#define FOC_CORE_H

#include "foc_types.h"
#include "foc_sense.h"
#include "foc_control.h"
#include "hal/hal_current.h"
#include "hal/hal_encoder.h"
#include "hal/hal_pwm.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    foc_state_t state;
    foc_fault_t fault;
    uint8_t enable;

    foc_sense_t sense;
    foc_control_t ctrl;

    const hal_current_t *current_hal;
    const hal_encoder_t *encoder_hal;
    const hal_pwm_t     *pwm_hal;

    foc_motor_params_t motor;
    uint32_t loop_count;
    uint32_t speed_div;
    uint32_t pos_div;
} foc_core_t;

void foc_core_init(foc_core_t *core,
                   const foc_motor_params_t *motor,
                   const hal_current_t *current_hal,
                   const hal_encoder_t *encoder_hal,
                   const hal_pwm_t *pwm_hal);

void foc_core_enable(foc_core_t *core);
void foc_core_disable(foc_core_t *core);
uint8_t foc_core_is_enabled(const foc_core_t *core);

foc_state_t foc_core_get_state(const foc_core_t *core);
foc_fault_t foc_core_get_fault(const foc_core_t *core);
void        foc_core_clear_fault(foc_core_t *core);

float foc_core_get_angle(const foc_core_t *core);
float foc_core_get_speed(const foc_core_t *core);
float foc_core_get_id(const foc_core_t *core);
float foc_core_get_iq(const foc_core_t *core);
float foc_core_get_vbus(const foc_core_t *core);

void foc_core_loop_current(foc_core_t *core);
void foc_core_loop_speed(foc_core_t *core);
void foc_core_loop_position(foc_core_t *core);

#ifdef __cplusplus
}
#endif

#endif
