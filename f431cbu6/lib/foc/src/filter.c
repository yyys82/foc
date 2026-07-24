#include "filter.h"

void lpf_init(foc_lpf_t *filt, float alpha)
{
    filt->alpha = alpha;
    filt->y_prev = 0.0f;
    filt->initialized = 0;
}

void lpf_set_alpha(foc_lpf_t *filt, float alpha)
{
    filt->alpha = alpha;
}

float lpf_update(foc_lpf_t *filt, float x)
{
    if (!filt->initialized)
    {
        filt->y_prev = x;
        filt->initialized = 1;
        return x;
    }

    float y = filt->alpha * x + (1.0f - filt->alpha) * filt->y_prev;
    filt->y_prev = y;
    return y;
}

void movavg_init(foc_movavg_t *ma)
{
    for (uint32_t i = 0; i < 4; i++)
        ma->buf[i] = 0.0f;
    ma->index = 0;
}

float movavg_update(foc_movavg_t *ma, float x, uint32_t n)
{
    if (n > 4) n = 4;
    if (n == 0) return x;

    ma->buf[ma->index % n] = x;
    ma->index = (ma->index + 1) % n;

    float sum = 0.0f;
    for (uint32_t i = 0; i < n; i++)
        sum += ma->buf[i];

    return sum / (float)n;
}
