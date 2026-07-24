#ifndef FOC_TYPES_H
#define FOC_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { float a, b, c; } foc_abc_t;

typedef struct { float alpha, beta; } foc_alphabeta_t;

typedef struct { float d, q; } foc_dq_t;

typedef struct { float rad, sin, cos; } foc_angle_t;

typedef struct {
    float rs;
    float ld, lq;
    float flux_linkage;
    uint32_t pole_pairs;
    float rated_current;
    float max_speed;
} foc_motor_params_t;

typedef enum {
    FOC_STATE_IDLE = 0,
    FOC_STATE_CALIB,
    FOC_STATE_ALIGN,
    FOC_STATE_RUN,
    FOC_STATE_FAULT
} foc_state_t;

typedef enum {
    FOC_MODE_IDLE = 0,
    FOC_MODE_TORQUE,
    FOC_MODE_SPEED,
    FOC_MODE_POSITION
} foc_ctrl_mode_t;

typedef enum {
    FOC_FAULT_NONE         = 0,
    FOC_FAULT_OVERCURRENT  = 1 << 0,
    FOC_FAULT_OVERVOLTAGE  = 1 << 1,
    FOC_FAULT_UNDERVOLTAGE = 1 << 2,
    FOC_FAULT_OVER_TEMP    = 1 << 3,
    FOC_FAULT_ENCODER      = 1 << 4,
} foc_fault_t;

typedef struct {
    uint32_t signature;
    uint16_t version;
    uint16_t crc16;
    float current_offset_u, current_offset_v, current_offset_w;
    float encoder_offset;
    float pos_kp, pos_ki, pos_kd;
    float spd_kp, spd_ki, spd_kd;
    float cur_kp, cur_ki;
    float current_limit, speed_limit, bus_voltage_nom;
    uint32_t padding[4];
} foc_storage_t;

#define FOC_STORAGE_SIGNATURE 0xA5F0C0DE
#define FOC_STORAGE_VERSION   1

#ifdef __cplusplus
}
#endif

#endif
