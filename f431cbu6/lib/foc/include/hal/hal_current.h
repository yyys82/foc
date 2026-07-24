#ifndef HAL_CURRENT_H
#define HAL_CURRENT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    void  (*init)(void);
    void  (*read)(float *ia, float *ib, float *ic);
    void  (*delay_us)(uint32_t us);
} hal_current_t;

#ifdef __cplusplus
}
#endif

#endif
