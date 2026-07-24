#ifndef HAL_CAN_H
#define HAL_CAN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*hal_can_rx_cb_t)(uint32_t id, uint8_t *data, uint8_t len);

typedef struct {
    void (*init)(uint32_t baud);
    void (*send)(uint32_t id, uint8_t *data, uint8_t len);
    void (*set_rx_cb)(hal_can_rx_cb_t cb);
} hal_can_t;

#ifdef __cplusplus
}
#endif

#endif
