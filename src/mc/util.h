#ifndef UTIL_H
#define UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <math.h>

#define SQRT_3          1.732050807568877f
#define Q15_MAX         32767.0f
#define Q30_MAX         1073741824.0f

#define Q15_TO_RADIAN       (float)(M_PI / Q15_MAX)
#define RPM_TO_ELEC_RAD_PER_S (float)(M_PI / 30.0f)

inline static float SquareFloat(const float x)
{
    return (x * x);
}

inline static void SaturateFloat(float * const input, const float min, const float max)
{
    if (max < (*input))
    {
        *input = max;
    }
    else if (min > (*input))
    {
        *input = min;
    }
    else
    {
        /* no action */
    }
}

inline static void LowPassFilter(float input, float filterCoeff, float *output)
{
    *output = *output + ((input - *output) * filterCoeff);
}

#ifdef __cplusplus
}
#endif

#endif /* UTIL_H */
