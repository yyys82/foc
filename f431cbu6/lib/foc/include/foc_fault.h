#ifndef FOC_FAULT_H
#define FOC_FAULT_H

#include "foc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 故障检测: 检查过流/过压/欠压，返回故障位掩码 */
foc_fault_t foc_fault_check(float i_max, float vbus,
                            float rated_current, float vq_limit);

#ifdef __cplusplus
}
#endif

#endif
