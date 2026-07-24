#ifndef HAL_ENCODER_H
#define HAL_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    void  (*init)(void);
    float (*get_angle)(void);
    float (*get_speed)(void);
} hal_encoder_t;

#ifdef __cplusplus
}
#endif

#endif
