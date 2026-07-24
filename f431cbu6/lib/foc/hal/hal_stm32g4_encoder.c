/*
 * STM32G4 AS5600 磁编码器 (I2C2 DMA)
 *   ISR内非阻塞: 检查上次DMA是否完成 → 处理数据 → 启动新DMA
 *   DMA未完成时: 速度外推填补
 *   get_angle: 多圈机械角度 rad
 *   get_speed: 机械角速度 rad/s
 *
 * 时序:
 *   ISR_t:  DMA done? → process + restart DMA → 正常角度
 *   ISR_t+1: DMA busy → extrapolate (speed × 100μs)
 *   ISR_t+2: DMA done? → process + restart DMA → 新角度 (约200μs更新)
 *
 * 本文件替换 HAL 弱回调 HAL_I2C_MasterRxCpltCallback 来清除 _busy 标志。
 * 不依赖 DMA 中断 — 在 TIM6 ISR 内轮询 I2C State。
 */

#include "hal/hal_encoder.h"
#include "i2c.h"
#include <math.h>
#include "dma.h"

#define AS5600_RESOLUTION  4096.0f
#define _2PI               6.28318530718f

static int32_t  _prev_raw_val;
static float    _full_turns;
static float    _cached_angle;
static float    _mech_speed;
static float    _prev_real_angle;
static uint32_t _real_cnt;         /* 上次真实读的 ISR 计数 */
static uint32_t _isr_cnt;         /* _get_angle 调用次数 */
static uint8_t  _dma_buf[2];
static uint8_t  _busy;            /* 1=DMA传输中 */
static uint32_t _busy_cnt;        /* busy 计数器，超时则恢复 */
static uint8_t  _ready;

uint8_t g_openloop;  /* 1=合成角开环, 0=编码器 */

/*
 * I2C Mem Read DMA 完成回调（STM32G4 HAL 弱符号，此处覆盖）
 * DMA 接收完成后 HAL 框架触发此回调，在此清除 busy 标志。
 * _get_angle() 在下一次 ISR 调用时检测 busy→idle 转换并处理数据。
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c2)
        _busy = 0;
}

static void _init(void)
{
    _prev_raw_val    = 0;
    _full_turns      = 0.0f;
    _cached_angle    = 0.0f;
    _mech_speed      = 0.0f;
    _prev_real_angle = 0.0f;
    _real_cnt        = 0;
    _isr_cnt         = 0;
    _busy            = 0;
    _busy_cnt        = 0;
    _ready           = 0;
}

static float _get_angle(void)
{
    if (g_openloop)
    {
        static float syn = 0;
        syn += 0.002f;  /* ~12.5kHz * 0.002 ≈ 25 rad/s el */
        if (syn > 6.283185f) syn -= 6.283185f;
        _mech_speed = 0.05f * 12500.0f / 14.0f;  /* 等效机械速度 */
        _cached_angle = syn;
        _ready = 1;
        return syn;
    }

    if (!_ready)
    {
        /* 首次阻塞读，获取初始角度 */
        uint8_t buf[2];
        HAL_I2C_Mem_Read(&hi2c2, 0x36 << 1, 0x0C, I2C_MEMADD_SIZE_8BIT, buf, 2, 10);
        int32_t raw = (int32_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
        _prev_raw_val    = raw;
        _cached_angle    = (float)raw / AS5600_RESOLUTION * _2PI;
        _prev_real_angle = _cached_angle;
        _ready           = 1;

        /* 立即启动后台 DMA */
        HAL_I2C_Mem_Read_DMA(&hi2c2, 0x36 << 1, 0x0C, 1, _dma_buf, 2);
        _busy = 1;
        return _cached_angle;
    }

    _isr_cnt++;

    /* 检查 DMA 是否刚刚完成（busy→idle 转换） */
    if (!_busy)
    {
        /* 新数据就绪：处理 raw → angle → 跨圈 → 速度 */
        int32_t raw = (int32_t)(((uint16_t)_dma_buf[0] << 8) | (uint16_t)_dma_buf[1]);

        float a = (float)raw / AS5600_RESOLUTION * _2PI;
        float da = a - (float)_prev_raw_val / AS5600_RESOLUTION * _2PI;
        if (da > 3.14159f)       _full_turns -= _2PI;
        else if (da < -3.14159f) _full_turns += _2PI;

        _prev_raw_val = raw;
        _cached_angle = _full_turns + a;

        /* 速度 = 真实角度差 / 时间差 */
        uint32_t dt = _isr_cnt - _real_cnt;
        if (dt > 0)
        {
            _mech_speed = (_cached_angle - _prev_real_angle) / ((float)dt * 0.0001f);
        }
        _prev_real_angle = _cached_angle;
        _real_cnt = _isr_cnt;

        /* 立即启动下一次 DMA */
        HAL_I2C_Mem_Read_DMA(&hi2c2, 0x36 << 1, 0x0C, 1, _dma_buf, 2);
        _busy = 1;
        _busy_cnt = 0;
    }
    else
    {
        /* DMA 还在忙，用上一次的速度做外推 */
        _cached_angle += _mech_speed * 0.0001f;
        _busy_cnt++;

        /* 超过 50 次 ISR (5ms) 未完成 → DMA 卡死，阻塞恢复 */
        if (_busy_cnt > 50)
        {
            HAL_I2C_Mem_Read(&hi2c2, 0x36 << 1, 0x0C, I2C_MEMADD_SIZE_8BIT, _dma_buf, 2, 10);
            _busy = 0;   /* 触发下次 ISR 处理 */
            _busy_cnt = 0;
        }
    }

    return _cached_angle;
}

static float _get_speed(void) { return _mech_speed; }

const hal_encoder_t g_hal_encoder = {
    .init      = _init,
    .get_angle = _get_angle,
    .get_speed = _get_speed,
};
