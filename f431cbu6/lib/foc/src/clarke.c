#include "clarke.h"
#include "foc_config.h"

void clarke_transform(const foc_abc_t *abc, foc_alphabeta_t *ab)
{
    float ia = abc->a;
    float ib = abc->b;

    ab->alpha = ia;
    ab->beta  = (ia + 2.0f * ib) * FOC_INV_SQRT3;
}

void clarke_transform_inv(const foc_alphabeta_t *ab, foc_abc_t *abc)
{
    float alpha = ab->alpha;
    float beta  = ab->beta;

    abc->a = alpha;
    abc->b = -0.5f * alpha + FOC_SQRT3_2 * beta;
    abc->c = -0.5f * alpha - FOC_SQRT3_2 * beta;
}
