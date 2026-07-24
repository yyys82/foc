#ifndef COMM_HOST_H
#define COMM_HOST_H

#include <stdint.h>
#include "foc_core.h"
#include "hal/hal_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define COMM_MAX_VARS  8

typedef struct {
    const hal_uart_t *uart;
    foc_core_t *core;

    char rx_buf[64];
    uint32_t rx_index;
    uint8_t rx_ready;

    uint8_t monitor_enabled;
    const char *monitor_vars[COMM_MAX_VARS];
    char monitor_copy[COMM_MAX_VARS][16];
    uint32_t monitor_count;
    uint32_t monitor_interval_ms;
    uint32_t last_monitor_ms;
} foc_comm_t;

void comm_init(foc_comm_t *comm, foc_core_t *core, const hal_uart_t *uart);
void comm_process_char(foc_comm_t *comm, char c);
void comm_process_line(foc_comm_t *comm);
void comm_tick(foc_comm_t *comm);

#ifdef __cplusplus
}
#endif

#endif
