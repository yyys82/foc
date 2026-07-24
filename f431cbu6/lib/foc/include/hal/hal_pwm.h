#ifndef HAL_PWM_H
#define HAL_PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    void  (*set_duty)(float ta, float tb, float tc);
    void  (*enable)(uint8_t en);
    float (*get_vbus)(void);
} hal_pwm_t;

#ifdef __cplusplus
}
#endif

#endif
