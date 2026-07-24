#include "foc_fault.h"

foc_fault_t foc_fault_check(float i_max, float vbus,
                            float rated_current, float vq_limit)
{
    foc_fault_t fault = FOC_FAULT_NONE;

    if (i_max > rated_current * 1.5f)
        fault |= FOC_FAULT_OVERCURRENT;

    if (vbus > vq_limit * 1.2f)
        fault |= FOC_FAULT_OVERVOLTAGE;
    else if (vbus < 0.5f * vq_limit)
        fault |= FOC_FAULT_UNDERVOLTAGE;

    return fault;
}
