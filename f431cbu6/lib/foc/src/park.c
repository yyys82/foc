#include "park.h"

void park_transform(const foc_alphabeta_t *ab, const foc_angle_t *angle, foc_dq_t *dq)
{
    float sin_a = angle->sin;
    float cos_a = angle->cos;
    float alpha = ab->alpha;
    float beta  = ab->beta;

    dq->d =  cos_a * alpha + sin_a * beta;
    dq->q = -sin_a * alpha + cos_a * beta;
}

void park_transform_inv(const foc_dq_t *dq, const foc_angle_t *angle, foc_alphabeta_t *ab)
{
    float sin_a = angle->sin;
    float cos_a = angle->cos;
    float d = dq->d;
    float q = dq->q;

    ab->alpha = cos_a * d - sin_a * q;
    ab->beta  = sin_a * d + cos_a * q;
}
