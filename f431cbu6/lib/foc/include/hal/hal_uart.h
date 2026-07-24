#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    void    (*init)(uint32_t baud);
    void    (*tx)(char c);
    uint8_t (*rx)(char *c);
} hal_uart_t;

#ifdef __cplusplus
}
#endif

#endif
