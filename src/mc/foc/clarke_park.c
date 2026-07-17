#include <stdint.h>
#include "clarke_park.h"

#define SQRT3_BY2       (float)0.866025403788
#define ONE_BY_SQRT3    (float)0.5773502691

void MC_TransformClarkeInverseSwappedInput(const MC_ALPHABETA_T *pAlphaBeta, MC_ABC_T *pABC)
{
    pABC->a = pAlphaBeta->beta;
    pABC->b = (-pAlphaBeta->beta / 2.0f + SQRT3_BY2 * pAlphaBeta->alpha);
    pABC->c = (-pAlphaBeta->beta / 2.0f - SQRT3_BY2 * pAlphaBeta->alpha);
}

void MC_TransformParkInverse(const MC_DQ_T *pDQ, const MC_SINCOS_T *pSinCos, MC_ALPHABETA_T *pAlphaBeta)
{
    pAlphaBeta->alpha = (float)(pDQ->d * pSinCos->cos - pDQ->q * pSinCos->sin);
    pAlphaBeta->beta  = (float)(pDQ->d * pSinCos->sin + pDQ->q * pSinCos->cos);
}

void MC_TransformPark(const MC_ALPHABETA_T *pAlphaBeta, const MC_SINCOS_T *pSinCos, MC_DQ_T *pDQ)
{
    pDQ->d = (pAlphaBeta->alpha * pSinCos->cos) + (pAlphaBeta->beta * pSinCos->sin);
    pDQ->q = (-pAlphaBeta->alpha * pSinCos->sin) + (pAlphaBeta->beta * pSinCos->cos);
}

void MC_TransformClarke(const MC_ABC_T *pABC, MC_ALPHABETA_T *pAlphaBeta)
{
    pAlphaBeta->alpha = pABC->a;
    pAlphaBeta->beta = (pABC->a * ONE_BY_SQRT3) + (pABC->b * 2.0f * ONE_BY_SQRT3);
}
