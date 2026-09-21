/*
 * AMOLED_Canvas.h
 * ============================================================================
 *  The drawing engine of the WaveshareAMOLED library.
 *
 *  A canvas is nothing more than a window onto pixel memory plus a clip
 *  rectangle - it knows nothing about the panel.  That is what makes the
 *  double buffered strip pipeline possible: the driver hands the canvas one
 *  horizontal strip at a time and the very same drawing code can be used for
 *
 *    - one strip of the screen  (beginStrip, streaming mode) and
 *    - a complete framebuffer   (beginFull,  framebuffer mode).
 *
 *  Coordinates are always in the *logical* (i.e. rotated) screen space; the
 *  rotation is applied by strided pixel addressing and costs one add per
 *  pixel.  Everything is clipped automatically.
 *
 *  Colour handling: a colour is 0xRRGGBB; whether that ends up as 3 bytes
 *  (RGB888) or 2 bytes (RGB565) in memory is decided by AMOLED_COLOR_DEPTH in
 *  AMOLED_Config.h.  There are two ways to draw:
 *
 *    set / fill / draw...   opaque   - replaces the pixel
 *    add / glow / blend...  additive - light *on top* of what is there
 *
 *  On an AMOLED the additive calls are the interesting ones: black is a pixel
 *  that is simply off, so glowing on black reaches the full 24 bit space.
 * ============================================================================
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "AMOLED_Config.h"
#include "AMOLED_Colors.h"

#if (AMOLED_COLOR_DEPTH == 16)
  #define AMOLED_BPP 2
#else
  #define AMOLED_BPP 3
#endif

/* ==================================================================== */
/*  Raw pixel helpers - the only place where the colour depth matters    */
/* ==================================================================== */
#if (AMOLED_BPP == 3)

/* store one pixel, memory order R, G, B */
static inline void amoledPxSet(uint8_t *p, uint8_t r, uint8_t g, uint8_t b)
{
    p[0] = r; p[1] = g; p[2] = b;
}

/* add light, saturated at 255 */
static inline void amoledPxAdd(uint8_t *p, uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t t;
    t = (uint16_t)p[0] + r; p[0] = (uint8_t)(t > 255u ? 255u : t);
    t = (uint16_t)p[1] + g; p[1] = (uint8_t)(t > 255u ? 255u : t);
    t = (uint16_t)p[2] + b; p[2] = (uint8_t)(t > 255u ? 255u : t);
}

/* add light with an 8 bit gain (255 = full) */
static inline void amoledPxAddScaled(uint8_t *p, uint8_t r, uint8_t g, uint8_t b, uint8_t m)
{
    uint16_t t;
    t = (uint16_t)p[0] + (uint16_t)(((uint16_t)r * m) >> 8); p[0] = (uint8_t)(t > 255u ? 255u : t);
    t = (uint16_t)p[1] + (uint16_t)(((uint16_t)g * m) >> 8); p[1] = (uint8_t)(t > 255u ? 255u : t);
    t = (uint16_t)p[2] + (uint16_t)(((uint16_t)b * m) >> 8); p[2] = (uint8_t)(t > 255u ? 255u : t);
}

/* multiply the pixel (fade towards black) */
static inline void amoledPxMul(uint8_t *p, uint8_t m)
{
    p[0] = (uint8_t)(((uint16_t)p[0] * m) >> 8);
    p[1] = (uint8_t)(((uint16_t)p[1] * m) >> 8);
    p[2] = (uint8_t)(((uint16_t)p[2] * m) >> 8);
}

/* alpha blend: a = 0 keeps the background, a = 255 takes the colour */
static inline void amoledPxBlend(uint8_t *p, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    int16_t d;
    d = (int16_t)r - (int16_t)p[0]; p[0] = (uint8_t)((int16_t)p[0] + ((d * a) >> 8));
    d = (int16_t)g - (int16_t)p[1]; p[1] = (uint8_t)((int16_t)p[1] + ((d * a) >> 8));
    d = (int16_t)b - (int16_t)p[2]; p[2] = (uint8_t)((int16_t)p[2] + ((d * a) >> 8));
}

#else  /* ------------------------------ RGB565 fallback --------------- */

static inline uint16_t amoledPack565(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | (uint16_t)(b >> 3));
}

static inline uint16_t amoledPxGet565(const uint8_t *p)
{
#if AMOLED_RGB565_BYTE_SWAP
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
#else
    return (uint16_t)(((uint16_t)p[1] << 8) | (uint16_t)p[0]);
#endif
}

static inline void amoledPxSet565(uint8_t *p, uint16_t v)
{
#if AMOLED_RGB565_BYTE_SWAP
    p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)(v & 0xFF);
#else
    p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)(v >> 8);
#endif
}

static inline void amoledPxSet(uint8_t *p, uint8_t r, uint8_t g, uint8_t b)
{
    amoledPxSet565(p, amoledPack565(r, g, b));
}

static inline void amoledPxAdd(uint8_t *p, uint8_t r, uint8_t g, uint8_t b)
{
    const uint16_t v = amoledPxGet565(p);
    int c, rr, gg, bb;
    c = (v >> 11) & 0x1F; rr = (c << 3) | (c >> 2); rr += r; if (rr > 255) rr = 255;
    c = (v >> 5) & 0x3F;  gg = (c << 2) | (c >> 4); gg += g; if (gg > 255) gg = 255;
    c = v & 0x1F;         bb = (c << 3) | (c >> 2); bb += b; if (bb > 255) bb = 255;
    amoledPxSet565(p, amoledPack565((uint8_t)rr, (uint8_t)gg, (uint8_t)bb));
}

static inline void amoledPxAddScaled(uint8_t *p, uint8_t r, uint8_t g, uint8_t b, uint8_t m)
{
    amoledPxAdd(p, (uint8_t)(((uint16_t)r * m) >> 8),
                   (uint8_t)(((uint16_t)g * m) >> 8),
                   (uint8_t)(((uint16_t)b * m) >> 8));
}

static inline void amoledPxMul(uint8_t *p, uint8_t m)
{
    const uint16_t v = amoledPxGet565(p);
    int c, rr, gg, bb;
    c = (v >> 11) & 0x1F; rr = ((c << 3) | (c >> 2)) * m >> 8;
    c = (v >> 5) & 0x3F;  gg = ((c << 2) | (c >> 4)) * m >> 8;
    c = v & 0x1F;         bb = ((c << 3) | (c >> 2)) * m >> 8;
    amoledPxSet565(p, amoledPack565((uint8_t)rr, (uint8_t)gg, (uint8_t)bb));
}

static inline void amoledPxBlend(uint8_t *p, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    const uint16_t v = amoledPxGet565(p);
    int c, rr, gg, bb;
    c = (v >> 11) & 0x1F; c = (c << 3) | (c >> 2); rr = c + (((int)r - c) * a >> 8);
    c = (v >> 5) & 0x3F;  c = (c << 2) | (c >> 4); gg = c + (((int)g - c) * a >> 8);
    c = v & 0x1F;         c = (c << 3) | (c >> 2); bb = c + (((int)b - c) * a >> 8);
    if (rr < 0) { rr = 0; }
    if (rr > 255) { rr = 255; }
    if (gg < 0) { gg = 0; }
    if (gg > 255) { gg = 255; }
    if (bb < 0) { bb = 0; }
    if (bb > 255) { bb = 255; }
    amoledPxSet565(p, amoledPack565((uint8_t)rr, (uint8_t)gg, (uint8_t)bb));
}

#endif

static inline int amoledClamp(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* ==================================================================== */
/*  AMOLED_Canvas - the drawing surface                                  */
/*                                                                       */
/*  Colours are always 0xRRGGBB.  Coordinates are logical (rotated)      */
/*  screen coordinates and are clipped automatically.                    */
/* ==================================================================== */
class AMOLED_Canvas
{
public:
    /* ----------------------------------------------------------------
     *  Window set up (done for you by the AMOLED class, but public so
     *  you can point a canvas at your own buffer if you want to)
     * ---------------------------------------------------------------- */

    /* one horizontal strip of "lines" screen lines starting at stripY */
    void beginStrip(uint8_t *buffer, int stripY, int lines);

    /* a complete framebuffer, AMOLED_WIDTH * AMOLED_HEIGHT * AMOLED_BPP bytes */
    void beginFull(uint8_t *framebuffer);

    /* rotation in degrees: 0, 90, 180 or 270 (library wide setting) */
    static void     setRotation(uint16_t degrees);
    static uint16_t rotation(void) { return s_rotation; }

    /* build the lookup tables (palette, glow curve, arc directions).
     * The AMOLED class calls it from begin(), you only need it if you
     * use a canvas completely on your own. */
    static void initTables(void);

    /* ----------------------------------------------------------------
     *  Geometry
     * ---------------------------------------------------------------- */
    int width(void) const   { return _lx1 - _lx0 + 1; }   /* logical width  */
    int height(void) const  { return _ly1 - _ly0 + 1; }   /* logical height */
    int originX(void) const { return _lx0; }
    int originY(void) const { return _ly0; }
    int clipX0(void) const  { return _lx0; }
    int clipY0(void) const  { return _ly0; }
    int clipX1(void) const  { return _lx1; }
    int clipY1(void) const  { return _ly1; }

    /* pixel address and byte steps - for hand written loops.
     * A row loop looks like:  p = ptr(clipX0(), y); ... p += strideX();  */
    uint8_t *ptr(int x, int y) const;
    int      strideX(void) const { return _stepX * AMOLED_BPP; }
    int      strideY(void) const { return _stepY * AMOLED_BPP; }

    /* ----------------------------------------------------------------
     *  Opaque drawing (replaces what is there)
     * ---------------------------------------------------------------- */
    void clearScreen(void);                              /* everything #000000  */
    void fillScreen(uint32_t c);
    void fillRect(int x0, int y0, int x1, int y1, uint32_t c);
    void drawRect(int x0, int y0, int x1, int y1, uint32_t c);      /* outline */
    void drawPixel(int x, int y, uint32_t c);
    void drawHLine(int y, int x0, int x1, uint32_t c);
    void drawVLine(int x, int y0, int y1, uint32_t c);
    void drawLine(int x0, int y0, int x1, int y1, uint32_t c);
    void drawCircle(int cx, int cy, int radius, uint32_t c);       /* outline */
    void fillCircle(int cx, int cy, int radius, uint32_t c);
    /* translucent (alpha 0..255) rectangle - good for HUD backgrounds */
    void shadeRect(int x0, int y0, int x1, int y1, uint32_t c, uint8_t alpha);

    /* ----------------------------------------------------------------
     *  Additive drawing (adds light, saturating) - the AMOLED special
     * ---------------------------------------------------------------- */
    void addPixel(int x, int y, uint32_t c);
    void addPixelScaled(int x, int y, uint32_t c, uint8_t gain);   /* gain 0..255 */
    void blendPixel(int x, int y, uint32_t c, uint8_t alpha);
    void addRect(int x0, int y0, int x1, int y1, uint32_t c);
    void addHLine(int y, int x0, int x1, uint32_t c);
    void addVLine(int x, int y0, int y1, uint32_t c);
    void addLine(int x0, int y0, int x1, int y1, uint32_t c);
    void addCircle(int cx, int cy, int radius, uint32_t c, uint8_t gain = 255);

    /* soft radial light blob - radius is the visible radius in pixels */
    void addGlow(int cx, int cy, int radius, uint32_t c, uint8_t gain = 255);

    /* arc / ring: angles in degrees, 0 = 3 o'clock, clockwise,
     * -90 = 12 o'clock.  thickness is the line width in pixels. */
    void addArc(int cx, int cy, int radius, int startDeg, int endDeg,
                int thickness, uint32_t c, uint8_t gain = 255);

    /* ----------------------------------------------------------------
     *  Text (built in 5x7 font, size 1..4)
     * ---------------------------------------------------------------- */
    static int  textWidth(const char *s, int size = 1);
    static int  textHeight(int size = 1) { return 7 * (size < 1 ? 1 : size); }
    void drawText(int x, int y, const char *s, uint32_t c, int size = 1);
    void drawTextAdd(int x, int y, const char *s, uint32_t c, int size = 1);
    void drawTextBlend(int x, int y, const char *s, uint32_t c, int size = 1, uint8_t alpha = 255);
    /* centred on cx - exactly what you want on a round display */
    void drawTextCentered(int cx, int y, const char *s, uint32_t c, int size = 1);
    void drawTextCenteredBlend(int cx, int y, const char *s, uint32_t c, int size, uint8_t alpha);

    /* ----------------------------------------------------------------
     *  Shared lookup tables
     * ---------------------------------------------------------------- */

    /* 1024 entry rainbow, perfectly smooth 24 bit hues */
    static uint32_t palette(uint16_t i) { return s_palette[i & 1023u]; }
    /* cos/sin * 256 of the 512 direction steps used by addArc() */
    static int cosStep(int i) { return s_cosT[i & 511]; }
    static int sinStep(int i) { return s_sinT[i & 511]; }
    /* the radial falloff curve used by addGlow (index = 256 * d^2 / r^2) */
    static uint8_t glowCurve(uint8_t i) { return s_glow[i]; }

private:
    uint8_t *_fb;      /* strip buffer / framebuffer                  */
    int      _idx0;    /* buffer pixel index of logical (_lx0,_ly0)   */
    int      _stepX;   /* buffer pixel step for one logical x         */
    int      _stepY;   /* buffer pixel step for one logical y         */
    int      _lx0, _ly0, _lx1, _ly1;      /* logical clip rectangle */

    static uint16_t s_rotation;        /* 0 / 90 / 180 / 270         */
    static uint8_t  s_glow[256];       /* radial falloff curve       */
    static uint32_t s_palette[1024];   /* 24 bit rainbow             */
    static int16_t  s_cosT[512];       /* arc direction steps        */
    static int16_t  s_sinT[512];
};

