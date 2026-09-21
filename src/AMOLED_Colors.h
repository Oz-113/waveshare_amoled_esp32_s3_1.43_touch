/*
 * AMOLED_Colors.h
 * ============================================================================
 *  Colours for the WaveshareAMOLED library.
 *
 *  A colour is always a 24 bit value 0xRRGGBB ("RGB888") - exactly the format
 *  the panel takes, so nothing is lost on the way.  Use amoledRGB() to build
 *  one, the AMOLED_* constants for the usual suspects, or amoledHSV() for
 *  anything from the colour wheel.
 * ============================================================================
 */
#pragma once

#include <stdint.h>
#include "AMOLED_Config.h"

/* -------------------------------------------------------------------- */
/*  Named colours (0xRRGGBB)                                            */
/* -------------------------------------------------------------------- */
#define AMOLED_BLACK        0x000000u
#define AMOLED_WHITE        0xFFFFFFu
#define AMOLED_RED          0xFF0000u
#define AMOLED_GREEN        0x00FF00u
#define AMOLED_BLUE         0x0000FFu
#define AMOLED_CYAN         0x00FFFFu
#define AMOLED_MAGENTA      0xFF00FFu
#define AMOLED_YELLOW       0xFFFF00u
#define AMOLED_ORANGE       0xFFA500u
#define AMOLED_PURPLE       0xA020F0u
#define AMOLED_PINK         0xFFC0CBu
#define AMOLED_LIME         0xC0FF00u
#define AMOLED_TEAL         0x008080u
#define AMOLED_NAVY         0x000080u
#define AMOLED_MAROON       0x800000u
#define AMOLED_OLIVE        0x808000u
#define AMOLED_GREY         0x808080u
#define AMOLED_DARKGREY     0x404040u
#define AMOLED_LIGHTGREY    0xC0C0C0u
#define AMOLED_GOLD         0xFFD700u
#define AMOLED_SKYBLUE      0x87CEEBu
#define AMOLED_DEEPPINK     0xFF1493u
#define AMOLED_SPRINGGREEN  0x00FF7Fu
#define AMOLED_ORANGERED    0xFF4500u
#define AMOLED_DODGERBLUE   0x1E90FFu
#define AMOLED_TURQUOISE    0x40E0D0u
#define AMOLED_CRIMSON      0xDC143Cu
#define AMOLED_CHOCOLATE    0xD2691Eu
#define AMOLED_INDIGO       0x4B0082u
#define AMOLED_SALMON       0xFA8072u

/* -------------------------------------------------------------------- */
/*  Inline colour maths                                                 */
/* -------------------------------------------------------------------- */

/* build a colour from its three channels: amoledRGB(255, 128, 0) */
static inline uint32_t amoledRGB(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

static inline uint8_t amoledR(uint32_t c) { return (uint8_t)(c >> 16); }
static inline uint8_t amoledG(uint32_t c) { return (uint8_t)(c >> 8); }
static inline uint8_t amoledB(uint32_t c) { return (uint8_t)(c); }

/* scale a colour: amoledScale(AMOLED_RED, 1, 2) = half brightness red */
static inline uint32_t amoledScale(uint32_t c, uint16_t num, uint16_t den)
{
    return amoledRGB((uint8_t)((amoledR(c) * num) / den),
                     (uint8_t)((amoledG(c) * num) / den),
                     (uint8_t)((amoledB(c) * num) / den));
}

/* linear mix, t = 0 gives "a", t = 255 gives "b" */
static inline uint32_t amoledMix(uint32_t a, uint32_t b, uint8_t t)
{
    const int r = amoledR(a) + (((int)amoledR(b) - (int)amoledR(a)) * t >> 8);
    const int g = amoledG(a) + (((int)amoledG(b) - (int)amoledG(a)) * t >> 8);
    const int bl = amoledB(a) + (((int)amoledB(b) - (int)amoledB(a)) * t >> 8);
    return amoledRGB((uint8_t)r, (uint8_t)g, (uint8_t)bl);
}

/* 24 bit -> RGB565 (only used when AMOLED_COLOR_DEPTH is 16) */
static inline uint16_t amoledTo565(uint32_t c)
{
    return (uint16_t)(((amoledR(c) & 0xF8) << 8) | ((amoledG(c) & 0xFC) << 3) | (amoledB(c) >> 3));
}

/* RGB565 -> 24 bit */
static inline uint32_t amoledFrom565(uint16_t c)
{
    const uint8_t r = (uint8_t)((c >> 11) & 0x1F);
    const uint8_t g = (uint8_t)((c >> 5) & 0x3F);
    const uint8_t b = (uint8_t)(c & 0x1F);
    return amoledRGB((uint8_t)((r << 3) | (r >> 2)),
                     (uint8_t)((g << 2) | (g >> 4)),
                     (uint8_t)((b << 3) | (b >> 2)));
}

/* -------------------------------------------------------------------- */
/*  Colour wheel                                                        */
/* -------------------------------------------------------------------- */

/* h = 0..360 degrees, s and v = 0..1  ->  0xRRGGBB */
uint32_t amoledHSV(float h, float s, float v);

/* h = 0..1 hue only, full saturation and value */
static inline uint32_t amoledHue(float h) { return amoledHSV(h * 360.0f, 1.0f, 1.0f); }
