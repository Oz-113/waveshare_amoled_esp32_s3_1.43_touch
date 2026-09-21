/*
 * AMOLED_Colors.cpp
 *
 * The only part of the colour handling that needs <math.h>: the HSV wheel.
 */
#include "AMOLED_Colors.h"
#include <math.h>

uint32_t amoledHSV(float h, float s, float v)
{
    if (v <= 0.0f) return AMOLED_BLACK;
    if (s <= 0.0f)
    {
        const uint8_t g = (uint8_t)(v * 255.0f + 0.5f);
        return amoledRGB(g, g, g);
    }

    /* wrap the hue into 0..360 */
    h = fmodf(h, 360.0f);
    if (h < 0.0f) h += 360.0f;

    const float c  = v * s;                       /* chroma            */
    const float hh = h / 60.0f;
    const float x  = c * (1.0f - fabsf(fmodf(hh, 2.0f) - 1.0f));
    const float m  = v - c;                       /* to lift the result */

    float r = 0.0f, g = 0.0f, b = 0.0f;
    if      (hh < 1.0f) { r = c; g = x; }
    else if (hh < 2.0f) { r = x; g = c; }
    else if (hh < 3.0f) { g = c; b = x; }
    else if (hh < 4.0f) { g = x; b = c; }
    else if (hh < 5.0f) { r = x; b = c; }
    else                { r = c; b = x; }

    r = (r + m) * 255.0f + 0.5f;
    g = (g + m) * 255.0f + 0.5f;
    b = (b + m) * 255.0f + 0.5f;

    return amoledRGB((uint8_t)(r > 255.0f ? 255.0f : r),
                     (uint8_t)(g > 255.0f ? 255.0f : g),
                     (uint8_t)(b > 255.0f ? 255.0f : b));
}
