#ifndef FOC_H
#define FOC_H

/* Layer 1: Math */
#include "clarke.h"
#include "park.h"
#include "svpwm.h"
#include "pi_ctrl.h"
#include "filter.h"

/* Layer 2: HAL Interfaces + Platform Impls */
#include "hal/hal_current.h"
#include "hal/hal_encoder.h"
#include "hal/hal_pwm.h"
#include "hal/hal_uart.h"
#include "hal/hal_can.h"

/* Layer 3: Sensing */
#include "foc_sense.h"

/* Layer 4: Control */
#include "foc_control.h"

/* Layer 5: Core + Fault */
#include "foc_core.h"
#include "foc_fault.h"

/* Layer 6: Communication */
#include "can_protocol.h"
#include "comm_host.h"

/* Shared */
#include "foc_types.h"
#include "foc_config.h"

#endif
