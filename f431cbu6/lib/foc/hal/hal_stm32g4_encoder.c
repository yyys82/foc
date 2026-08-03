/*
 * STM32G4 AS5600 磁编码器 (I2C2 DMA) —— 发布端/消费端架构
 *
 *   发布端 = HAL_I2C_MemRxCpltCallback。
 *     真实执行上下文是 DMA1_Channel1_IRQHandler（HAL master-receive-DMA 路径：
 *     DMA TC → DMA1_Channel1 ISR → HAL 内部 → 本回调）。该中断在 foc_main_init
 *     里被设为优先级 1，可被电流环 ADC ISR(prio0) 抢占 → 电流环抖动归零。
 *     职责：收完 2 字节 → 解算(防抖 sanity + 解缠 + 滑动窗口测速) → 短临界区发布
 *     快照 → 背靠背重启下一次 DMA 读（顶到 I2C 极限 ~120µs/次 ≈ 8kHz，消除原先
 *     "等下一次电流环 ISR 才重启"的最长 ~40µs 空转）。
 *
 *   消费端 = get_angle（电流环 ADC ISR prio0 + 线程态对齐/位置环）。
 *     短临界区拷贝 3-float 快照 → predicted = 真实角 + 速度×(now-发布时刻)。
 *     不碰任何 I2C/状态机/重启。发现 >500µs 无新角度 → 仅置 _need_recovery 标志
 *     并钳住外推(时长上限 + 速度衰减)防飞车；真正的阻塞恢复在线程态
 *     encoder_try_recovery()（foc_main_loop 调用），避免在电流环 ISR 里阻塞。
 *
 *   撕裂防护：发布端(prio1)可抢占线程态读者、电流环(prio0)可抢占发布端，故发布/
 *     读取快照都用 PRIMASK 短临界区（3 条访存，<100ns，对 40µs 电流环可忽略）。
 *   单写者：角度内部状态只由发布端写（首次建基准/线程态恢复为例外，均在临界区内）。
 *
 *   调试计数 _pub_seq/_to_cnt/_suspect_cnt/_err_cnt 可经 comm/printf 暴露用于验证。
 *   本文件覆盖 HAL 弱符号 HAL_I2C_MemRxCpltCallback / HAL_I2C_ErrorCallback。
 */

#include "hal/hal_encoder.h"
#include "i2c.h"
#include "dma.h"
#include "hw_config.h"
#include <math.h>

#define AS5600_RESOLUTION       (float)ENCODER_RESOLUTION       /* 4096 LSB/圈 */
#define AS5600_I2C_ADDR         ENCODER_I2C_ADDR                /* 0x36<<1 */
#define AS5600_REG_RAW_ANGLE    0x0CU                           /* RAW ANGLE (12bit) */
#define AS5600_READ_TIMEOUT_MS  10U                             /* 线程态阻塞读超时 */
#define _2PI                    6.28318530718f
#define RAD_PER_COUNT           (_2PI / AS5600_RESOLUTION)
#define FOC_HCLK_HZ             (float)SYS_CLOCK_HZ             /* G431 HCLK = 170MHz，DWT 计数频率 */
#define AS5600_DIR_SIGN         -1  /* 编码器方向：实测 +iq 命令产生反向旋转(mech_spd 为负)，
                                       翻 -1 使 +iq→正向转矩、速度反馈与命令同向。 */
#define SPD_EST_WINDOW          256   /* 速度估计滑动窗口：~8kHz 真实更新下 ≈32ms。
                                        低速时单样本位移 <1LSB，长窗把量化噪声平均掉，测速更平滑。
                                        响应滞后 ~16ms，低速应用可接受；若需快响应可回调 128。 */
#define ENC_STEP_MAX            0.3f  /* 单步合法性阈值 rad：300rad/s@8kHz≈0.0375，取 ~8× 裕量；
                                        超阈值判为 I2C 脏读，丢这一拍（连续丢→看门狗兜底） */
#define ENC_WDG_S               0.0005f  /* 500µs 无新角度 → 标记需线程态恢复（正常 ~125µs/次） */
#define SPD_VALID_MAX           1000.0f  /* 速度合法性上限 rad/s（机械上限 300） */

/* ---- 开环合成角（调试）：与 foc_main g_motor 的 14 对极保持同步 ---- */
#define OPENLOOP_EL_STEP        0.002f    /* 每拍电角度增量 ≈ 12.5kHz 更新 × 0.002 ≈ 25 rad/s el */
#define OPENLOOP_UPDATE_HZ      12500.0f  /* 合成角等效更新率（PWM 半周期） */
#define OPENLOOP_SPD_SCALE      0.05f     /* 上报机械速度系数 */
#define OPENLOOP_POLE_PAIRS     14

/* ---- 发布端内部状态（只由发布端写；首次建基准/线程态恢复在临界区内例外写） ---- */
static int32_t  _prev_raw_val;
static float    _full_turns;       /* 累计整圈 (rad) */
static float    _real_angle;       /* 解缠后真实机械角 = _full_turns + 单圈角 */
static uint32_t _last_real_cyc;    /* 发布端内部测速时基 */
static float    _mech_speed;       /* 机械角速度 (rad/s) */
static float    _spd_acc;
static uint32_t _spd_acc_cyc;
static int      _spd_samples;

/* ---- 发布快照（发布端写、消费端读，均经临界区） ---- */
static volatile float    _pub_angle;
static volatile float    _pub_speed;
static volatile uint32_t _pub_last_cyc;
static volatile uint8_t  _need_recovery;

/* ---- 调试计数 ---- */
static volatile uint32_t _pub_seq, _to_cnt, _suspect_cnt, _err_cnt;

static uint8_t  _dma_buf[2];       /* 发布端私有 DMA 缓冲 */
static uint8_t  _ready;

uint8_t g_openloop;  /* 1=合成角开环, 0=编码器 */
float   g_dbg_mech;  /* 调试：最近一次返回的机械角(多圈, rad)；200Hz 位置环也读它 */

/* 临界区发布快照 */
static void _publish(float angle, uint32_t now_cyc)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    _pub_angle    = angle;
    _pub_speed    = _mech_speed;
    _pub_last_cyc = now_cyc;
    __set_PRIMASK(pm);
    _pub_seq++;
}

/* 背靠背重启 DMA 读（唯一重启入口：发布端/错误回调/线程态恢复共用）。
 * 失败只计数并置恢复标志，绝不在回调内循环重试。 */
static void _start_dma(void)
{
    if (HAL_I2C_Mem_Read_DMA(&hi2c2, AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE,
                             I2C_MEMADD_SIZE_8BIT, _dma_buf, 2) != HAL_OK)
    {
        _err_cnt++;
        _need_recovery = 1;   /* 交线程态 encoder_try_recovery */
    }
}

/* 发布端主体：解算一帧角度 → 发布 → 背靠背重启 DMA */
static void _on_i2c_done(void)
{
    uint32_t now_cyc = DWT->CYCCNT;
    int32_t raw = (int32_t)((((uint16_t)_dma_buf[0]) << 8) | (uint16_t)_dma_buf[1]) & 0x0FFF;

    float a  = (float)raw * RAD_PER_COUNT;
    float pa = (float)_prev_raw_val * RAD_PER_COUNT;
    float da = a - pa;

    /* 归一化单步到 (-π, π]，并记录解缠方向（沿用原符号约定：da>π ⇒ _full_turns-=2π） */
    float step = da;
    int wrap = 0;
    if (step > 3.14159f)       { step -= _2PI; wrap = -1; }
    else if (step < -3.14159f) { step += _2PI; wrap = +1; }

    if (step <= ENC_STEP_MAX && step >= -ENC_STEP_MAX)
    {
        /* 合法样：解缠 + 滑动窗口测速 + 更新基准 + 发布 */
        if (wrap == -1)      _full_turns -= _2PI;
        else if (wrap == +1) _full_turns += _2PI;

        float dt_real = (float)(now_cyc - _last_real_cyc) / FOC_HCLK_HZ;
        if (dt_real > 1e-5f && dt_real < 0.05f)
        {
            _spd_acc     += step;                 /* 累计归一化单步差，静止时 ±1LSB 抵消 */
            _spd_acc_cyc += (now_cyc - _last_real_cyc);
            if (++_spd_samples >= SPD_EST_WINDOW)
            {
                float dtime = (float)_spd_acc_cyc / FOC_HCLK_HZ;
                if (dtime > 1e-4f && dtime < 0.5f)
                {
                    float inst = _spd_acc / dtime;
                    if (inst > -SPD_VALID_MAX && inst < SPD_VALID_MAX)
                        _mech_speed = _mech_speed * 0.85f + inst * 0.15f;
                }
                _spd_acc = 0.0f; _spd_acc_cyc = 0; _spd_samples = 0;
            }
        }
        _prev_raw_val  = raw;
        _real_angle    = _full_turns + a;
        _last_real_cyc = now_cyc;
        _publish(_real_angle, now_cyc);
    }
    else
    {
        _suspect_cnt++;   /* 疑似 I2C 脏读：完全不更新状态，丢这一拍 */
    }

    /* 背靠背重启：先消费 _dma_buf 再重启，单缓冲安全（重启 ~120µs 后才再写缓冲） */
    _start_dma();
}

/*
 * I2C Mem Read DMA 完成回调（覆盖 HAL 弱符号），运行于 DMA1_Channel1 ISR (prio 1)。
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c2 && _ready && !g_openloop)
        _on_i2c_done();
}

/* I2C 错误回调（覆盖 HAL 弱符号）：NAK 类瞬时错误尝试直接重启自愈，否则交线程态 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != &hi2c2) return;
    _err_cnt++;
    if (_ready && !g_openloop)
        _start_dma();
    else
        _need_recovery = 1;
}

/*
 * 线程态恢复（foc_main_loop 调用）。电流环 ISR 只置标志、不阻塞；真正可能耗时
 * ~10ms 的阻塞读放这里。线程态可被发布端(prio1)抢占，故状态更新段用临界区。
 */
void encoder_try_recovery(void)
{
    if (!_need_recovery) return;

    /* 已自愈？(发布端又出了新角度) */
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    uint32_t t0 = _pub_last_cyc;
    __set_PRIMASK(pm);
    if ((float)(DWT->CYCCNT - t0) / FOC_HCLK_HZ < ENC_WDG_S)
    {
        _need_recovery = 0;
        return;
    }

    /* 真卡死：阻塞恢复（局部 buf，不碰发布端私有 _dma_buf）。
     * 本 STM32G4 HAL 版本无阻塞式 HAL_I2C_Abort：用 DeInit+Init 完整复位 I2C
     * 外设与句柄（HAL_I2C_MspInit 会重新初始化并链接 DMA，Init 结构体里的
     * Timing 等参数会被重新套用）。随后清 DMA1_Channel1 挂起，避免恢复期间
     * 一个已锁存的 TC 误触发发布端去读陈旧 _dma_buf。 */
    HAL_I2C_DeInit(&hi2c2);
    HAL_I2C_Init(&hi2c2);
    HAL_NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(&hi2c2, AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE,
                         I2C_MEMADD_SIZE_8BIT, buf, 2, AS5600_READ_TIMEOUT_MS) == HAL_OK)
    {
        int32_t raw = (int32_t)((((uint16_t)buf[0]) << 8) | (uint16_t)buf[1]) & 0x0FFF;
        float a = (float)raw * RAD_PER_COUNT;
        uint32_t now = DWT->CYCCNT;
        pm = __get_PRIMASK();
        __disable_irq();
        /* 让 _real_angle 连续：选 _full_turns 使 (_full_turns+a) 最接近当前 _real_angle */
        _full_turns   = roundf((_real_angle - a) / _2PI) * _2PI;
        _prev_raw_val = raw;
        _real_angle   = _full_turns + a;
        _last_real_cyc = now;
        _publish(_real_angle, now);   /* 嵌套临界区安全（PRIMASK 已禁） */
        __set_PRIMASK(pm);
    }
    /* 无论读成功与否都重启 DMA，让发布端恢复后台刷新 */
    _start_dma();
    _need_recovery = 0;
}

static void _init(void)
{
    _prev_raw_val  = 0;
    _full_turns    = 0.0f;
    _real_angle    = 0.0f;
    _last_real_cyc = 0;
    _mech_speed    = 0.0f;
    _spd_acc       = 0.0f;
    _spd_acc_cyc   = 0;
    _spd_samples   = 0;
    _pub_angle     = 0.0f;
    _pub_speed     = 0.0f;
    _pub_last_cyc  = 0;
    _need_recovery = 0;
    _pub_seq = _to_cnt = _suspect_cnt = _err_cnt = 0;
    _ready         = 0;
}

static float _get_angle(void)
{
    /* DWT 时基自愈：若被调试器/复位清掉 DEMCR.TRCENA 或 DWT.CYCCNTENA，
     * CYCCNT 停走 → 测速窗口 dt_real≈0 永远不算 → _mech_speed=0。
     * 此处每次调用重新断言使能（2 次寄存器写，25kHz 下可忽略）。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= 1;

    if (g_openloop)
    {
        static float syn = 0;
        syn += OPENLOOP_EL_STEP;
        if (syn > _2PI) syn -= _2PI;
        _mech_speed = OPENLOOP_SPD_SCALE * OPENLOOP_UPDATE_HZ / (float)OPENLOOP_POLE_PAIRS;
        g_dbg_mech  = syn * AS5600_DIR_SIGN;   /* 位置环反馈须与 get_angle 同符号 */
        return syn * AS5600_DIR_SIGN;
    }

    if (!_ready)
    {
        /* 线程态首次：阻塞读建基准（此时无 DMA 在途，发布端不会触发） */
        uint8_t buf[2];
        HAL_I2C_Mem_Read(&hi2c2, AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE,
                         I2C_MEMADD_SIZE_8BIT, buf, 2, AS5600_READ_TIMEOUT_MS);
        int32_t raw = (int32_t)((((uint16_t)buf[0]) << 8) | (uint16_t)buf[1]) & 0x0FFF;
        float a = (float)raw * RAD_PER_COUNT;
        uint32_t now = DWT->CYCCNT;
        _full_turns    = 0.0f;
        _prev_raw_val  = raw;
        _real_angle    = a;
        _last_real_cyc = now;
        _mech_speed    = 0.0f;
        _publish(a, now);
        _ready = 1;
        _start_dma();
        g_dbg_mech = a * AS5600_DIR_SIGN;   /* 位置环反馈须与 get_angle 同符号 */
        return a * AS5600_DIR_SIGN;
    }

    /* 临界区拷贝快照 */
    float a0, spd;
    uint32_t t0;
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    a0  = _pub_angle;
    spd = _pub_speed;
    t0  = _pub_last_cyc;
    __set_PRIMASK(pm);

    uint32_t now_cyc = DWT->CYCCNT;
    float dt = (float)(now_cyc - t0) / FOC_HCLK_HZ;   /* 无符号差，CYCCNT ~25s 回绕安全 */
    if (dt > ENC_WDG_S)
    {
        /* 太久无新角度：标记线程态恢复，钳住外推时长 + 速度衰减，防飞车 */
        if (!_need_recovery) { _need_recovery = 1; _to_cnt++; }
        dt = ENC_WDG_S;
        spd *= 0.9f;
    }
    float predicted = a0 + spd * dt;

    g_dbg_mech = predicted * AS5600_DIR_SIGN;   /* 位置环反馈须与 get_angle 同符号(否则位置环正反馈→一直转) */
    return predicted * AS5600_DIR_SIGN;
}

/* 调试：暴露发布端内部状态（线程态读，供 comm dbg 命令定位测速为何为 0） */
void encoder_get_debug(uint32_t *pub_seq, uint32_t *suspect, int *spd_samples,
                       float *mech_speed, float *real_angle, uint32_t *last_real_cyc)
{
    if (pub_seq)      *pub_seq      = _pub_seq;
    if (suspect)      *suspect      = _suspect_cnt;
    if (spd_samples)  *spd_samples  = _spd_samples;
    if (mech_speed)   *mech_speed   = _mech_speed;
    if (real_angle)   *real_angle   = _real_angle;
    if (last_real_cyc)*last_real_cyc= _last_real_cyc;
}

static float _get_speed(void) { return _mech_speed * AS5600_DIR_SIGN; }

const hal_encoder_t g_hal_encoder = {
    .init      = _init,
    .get_angle = _get_angle,
    .get_speed = _get_speed,
};
