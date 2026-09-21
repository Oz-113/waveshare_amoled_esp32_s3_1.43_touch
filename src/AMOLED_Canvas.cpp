/*
 * AMOLED_Canvas.cpp
 * ============================================================================
 *  Drawing engine of the WaveshareAMOLED library.
 *  See AMOLED_Canvas.h for the API description.
 * ============================================================================
 */
#include "AMOLED_Canvas.h"
#include "AMOLED_Font5x7.h"

#include <math.h>
#include <stdlib.h>

uint16_t AMOLED_Canvas::s_rotation = AMOLED_DEFAULT_ROTATION;
uint8_t  AMOLED_Canvas::s_glow[256];
uint32_t AMOLED_Canvas::s_palette[1024];
int16_t  AMOLED_Canvas::s_cosT[512];
int16_t  AMOLED_Canvas::s_sinT[512];

/* ------------------------------------------------------------------ */
/*  Lookup tables                                                      */
/* ------------------------------------------------------------------ */
void AMOLED_Canvas::initTables(void)
{
    /* --- radial glow curve -----------------------------------------
     * index i = 256 * d*d/R/R (quadratic distance); the curve is
     * evaluated on the linear distance t = sqrt(i/255), so a glow has a
     * hot core of 10 % and falls off smoothly to nothing. */
    for (int i = 0; i < 256; i++)
    {
        const float t = sqrtf((float)i / 255.0f);
        const float f = (t < 0.10f) ? 1.0f : (1.0f - (t - 0.10f) / 0.90f);
        float v = 255.0f * f * f;
        if (v < 0.0f) v = 0.0f;
        if (v > 255.0f) v = 255.0f;
        s_glow[i] = (uint8_t)v;
    }

    /* --- 1024 entry, perfectly smooth RGB888 rainbow ----------------
     * 1024 * 3 bytes of hue resolution is far beyond what the eye can
     * resolve - and far beyond what RGB565 could display. */
    for (int i = 0; i < 1024; i++)
    {
        const float h = (float)i / 1024.0f;
        float r = 0.5f + 0.5f * sinf(6.28318530718f * h);
        float g = 0.5f + 0.5f * sinf(6.28318530718f * h + 2.09439510239f);
        float b = 0.5f + 0.5f * sinf(6.28318530718f * h + 4.18879020479f);
        r = (r - 0.5f) * 1.35f + 0.5f;      /* saturation boost */
        g = (g - 0.5f) * 1.35f + 0.5f;
        b = (b - 0.5f) * 1.35f + 0.5f;
        if (r < 0.0f) { r = 0.0f; }
        if (r > 1.0f) { r = 1.0f; }
        if (g < 0.0f) { g = 0.0f; }
        if (g > 1.0f) { g = 1.0f; }
        if (b < 0.0f) { b = 0.0f; }
        if (b > 1.0f) { b = 1.0f; }
        s_palette[i] = amoledRGB((uint8_t)(r * 255.0f + 0.5f),
                                 (uint8_t)(g * 255.0f + 0.5f),
                                 (uint8_t)(b * 255.0f + 0.5f));
    }

    /* --- 512 direction steps for addArc() (cos/sin * 256) ----------
     * Screen coordinates: y grows downwards, so angle 0 is 3 o'clock and
     * growing angles walk clockwise - what you want for a HUD. */
    for (int i = 0; i < 512; i++)
    {
        const float a = (float)i * 6.28318530718f / 512.0f;
        s_cosT[i] = (int16_t)(cosf(a) * 256.0f);
        s_sinT[i] = (int16_t)(sinf(a) * 256.0f);
    }
}

void AMOLED_Canvas::setRotation(uint16_t degrees)
{
    /* normalise to the four values the mapping supports */
    switch (degrees)
    {
        case 90:
        case 180:
        case 270: s_rotation = degrees; break;
        default:  s_rotation = 0;       break;
    }
}

/* ------------------------------------------------------------------ */
/*  Window set up                                                      */
/*                                                                     */
/*  buffer index = _idx0 + (x - _lx0) * _stepX + (y - _ly0) * _stepY   */
/*                                                                     */
/*  For a strip the buffer holds exactly one band of screen lines, so   */
/*  a rotated image makes the band a *column* band - hence the strided  */
/*  addressing.  For a whole framebuffer beginFull() is the same call   */
/*  with stripY = 0 and lines = AMOLED_HEIGHT.                          */
/*                                                                     */
/*  Rotation 90/270 assume a square panel (the 1.43" module is          */
/*  466 x 466, so that is always true here).                            */
/* ------------------------------------------------------------------ */
void AMOLED_Canvas::beginStrip(uint8_t *buffer, int stripY, int lines)
{
    _fb = buffer;

    const int W = AMOLED_WIDTH;
    const int H = AMOLED_HEIGHT;
    const int sy0 = amoledClamp(stripY, 0, H - 1);
    const int sy1 = amoledClamp(stripY + lines - 1, 0, H - 1);
    const int n = sy1 - sy0 + 1;

    switch (s_rotation)
    {
        case 90:
            _lx0 = sy0;  _lx1 = sy1;   _ly0 = 0;   _ly1 = W - 1;
            _idx0 = W - 1;   _stepX = W;   _stepY = -1;
            break;
        case 180:
            _lx0 = 0;    _lx1 = W - 1; _ly0 = H - sy0 - n; _ly1 = H - 1 - sy0;
            _idx0 = n * W - 1;  _stepX = -1;  _stepY = -W;
            break;
        case 270:
            _lx0 = H - sy0 - n; _lx1 = H - 1 - sy0; _ly0 = 0; _ly1 = W - 1;
            _idx0 = (n - 1) * W; _stepX = -W; _stepY = 1;
            break;
        default:
            _lx0 = 0;    _lx1 = W - 1; _ly0 = sy0; _ly1 = sy1;
            _idx0 = 0;   _stepX = 1;   _stepY = W;
            break;
    }
}

void AMOLED_Canvas::beginFull(uint8_t *framebuffer)
{
    beginStrip(framebuffer, 0, AMOLED_HEIGHT);
}

/* ------------------------------------------------------------------ */
/*  Clearing                                                           */
/* ------------------------------------------------------------------ */
void AMOLED_Canvas::clearScreen(void)
{
    /* a true #000000 - on an AMOLED that means the pixels are off */
    memset(_fb, 0, (size_t)width() * (size_t)height() * AMOLED_BPP);
}

void AMOLED_Canvas::fillScreen(uint32_t c)
{
    fillRect(_lx0, _ly0, _lx1, _ly1, c);
}

/* ------------------------------------------------------------------ */
/*  Pixels                                                             */
/* ------------------------------------------------------------------ */
void AMOLED_Canvas::drawPixel(int x, int y, uint32_t c)
{
    if (x < _lx0 || x > _lx1 || y < _ly0 || y > _ly1) return;
    amoledPxSet(ptr(x, y), amoledR(c), amoledG(c), amoledB(c));
}

void AMOLED_Canvas::addPixel(int x, int y, uint32_t c)
{
    if (x < _lx0 || x > _lx1 || y < _ly0 || y > _ly1) return;
    amoledPxAdd(ptr(x, y), amoledR(c), amoledG(c), amoledB(c));
}

void AMOLED_Canvas::addPixelScaled(int x, int y, uint32_t c, uint8_t gain)
{
    if (x < _lx0 || x > _lx1 || y < _ly0 || y > _ly1) return;
    amoledPxAddScaled(ptr(x, y), amoledR(c), amoledG(c), amoledB(c), gain);
}

void AMOLED_Canvas::blendPixel(int x, int y, uint32_t c, uint8_t alpha)
{
    if (x < _lx0 || x > _lx1 || y < _ly0 || y > _ly1) return;
    amoledPxBlend(ptr(x, y), amoledR(c), amoledG(c), amoledB(c), alpha);
}

/* ------------------------------------------------------------------ */
/*  Rectangles                                                         */
/* ------------------------------------------------------------------ */
void AMOLED_Canvas::fillRect(int x0, int y0, int x1, int y1, uint32_t c)
{
    int t;
    if (x0 > x1) { t = x0; x0 = x1; x1 = t; }
    if (y0 > y1) { t = y0; y0 = y1; y1 = t; }
    x0 = amoledClamp(x0, _lx0, _lx1); x1 = amoledClamp(x1, _lx0, _lx1);
    y0 = amoledClamp(y0, _ly0, _ly1); y1 = amoledClamp(y1, _ly0, _ly1);
    if (x0 > x1 || y0 > y1) return;

    const uint8_t r = amoledR(c);
    const uint8_t g = amoledG(c);
    const uint8_t b = amoledB(c);
    const ptrdiff_t step = (ptrdiff_t)strideX();

    for (int y = y0; y <= y1; y++)
    {
        uint8_t *p = ptr(x0, y);
        for (int x = x0; x <= x1; x++)
        {
            amoledPxSet(p, r, g, b);
            p += step;
        }
    }
}

void AMOLED_Canvas::drawRect(int x0, int y0, int x1, int y1, uint32_t c)
{
    drawHLine(y0, x0, x1, c);
    drawHLine(y1, x0, x1, c);
    drawVLine(x0, y0, y1, c);
    drawVLine(x1, y0, y1, c);
}

void AMOLED_Canvas::drawHLine(int y, int x0, int x1, uint32_t c)
{
    int t;
    if (x0 > x1) { t = x0; x0 = x1; x1 = t; }
    x0 = amoledClamp(x0, _lx0, _lx1);
    x1 = amoledClamp(x1, _lx0, _lx1);
    if (y < _ly0 || y > _ly1 || x0 > x1) return;

    const uint8_t r = amoledR(c);
    const uint8_t g = amoledG(c);
    const uint8_t b = amoledB(c);
    const ptrdiff_t step = (ptrdiff_t)strideX();
    uint8_t *p = ptr(x0, y);
    for (int x = x0; x <= x1; x++)
    {
        amoledPxSet(p, r, g, b);
        p += step;
    }
}

void AMOLED_Canvas::drawVLine(int x, int y0, int y1, uint32_t c)
{
    int t;
    if (y0 > y1) { t = y0; y0 = y1; y1 = t; }
    y0 = amoledClamp(y0, _ly0, _ly1);
    y1 = amoledClamp(y1, _ly0, _ly1);
    if (x < _lx0 || x > _lx1 || y0 > y1) return;

    const uint8_t r = amoledR(c);
    const uint8_t g = amoledG(c);
    const uint8_t b = amoledB(c);
    const ptrdiff_t step = (ptrdiff_t)strideY();
    uint8_t *p = ptr(x, y0);
    for (int y = y0; y <= y1; y++)
    {
        amoledPxSet(p, r, g, b);
        p += step;
    }
}

void AMOLED_Canvas::shadeRect(int x0, int y0, int x1, int y1, uint32_t c, uint8_t alpha)
{
    int t;
    if (x0 > x1) { t = x0; x0 = x1; x1 = t; }
    if (y0 > y1) { t = y0; y0 = y1; y1 = t; }
    x0 = amoledClamp(x0, _lx0, _lx1); x1 = amoledClamp(x1, _lx0, _lx1);
    y0 = amoledClamp(y0, _ly0, _ly1); y1 = amoledClamp(y1, _ly0, _ly1);
    if (x0 > x1 || y0 > y1) return;

    const uint8_t r = amoledR(c);
    const uint8_t g = amoledG(c);
    const uint8_t b = amoledB(c);
    const ptrdiff_t step = (ptrdiff_t)strideX();

    for (int y = y0; y <= y1; y++)
    {
        uint8_t *p = ptr(x0, y);
        for (int x = x0; x <= x1; x++)
        {
            amoledPxBlend(p, r, g, b, alpha);
            p += step;
        }
    }
}

uint8_t *AMOLED_Canvas::ptr(int x, int y) const
{
    return _fb + (size_t)(_idx0 + (x - _lx0) * _stepX + (y - _ly0) * _stepY) * AMOLED_BPP;
}

/* ------------------------------------------------------------------ */
/*  Lines and circles                                                  */
/* ------------------------------------------------------------------ */
void AMOLED_Canvas::drawLine(int x0, int y0, int x1, int y1, uint32_t c)
{
    const int dx = abs(x1 - x0);
    const int dy = -abs(y1 - y0);
    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    for (;;)
    {
        drawPixel(x0, y0, c);
        if (x0 == x1 && y0 == y1) break;
        const int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void AMOLED_Canvas::drawCircle(int cx, int cy, int radius, uint32_t c)
{
    if (radius <= 0) { drawPixel(cx, cy, c); return; }
    int x = radius;
    int y = 0;
    int err = 1 - radius;

    while (x >= y)
    {
        drawPixel(cx + x, cy + y, c); drawPixel(cx + y, cy + x, c);
        drawPixel(cx - y, cy + x, c); drawPixel(cx - x, cy + y, c);
        drawPixel(cx - x, cy - y, c); drawPixel(cx - y, cy - x, c);
        drawPixel(cx + y, cy - x, c); drawPixel(cx + x, cy - y, c);
        y++;
        if (err < 0) err += 2 * y + 1;
        else { x--; err += 2 * (y - x) + 1; }
    }
}

void AMOLED_Canvas::fillCircle(int cx, int cy, int radius, uint32_t c)
{
    if (radius <= 0) { drawPixel(cx, cy, c); return; }
    for (int dy = -radius; dy <= radius; dy++)
    {
        const int w = (int)(sqrtf((float)(radius * radius - dy * dy)) + 0.5f);
        drawHLine(cy + dy, cx - w, cx + w, c);
    }
}

/* ------------------------------------------------------------------ */
/*  Additive rectangles and lines (light on top of what is there)      */
/* ------------------------------------------------------------------ */
void AMOLED_Canvas::addRect(int x0, int y0, int x1, int y1, uint32_t c)
{
    int t;
    if (x0 > x1) { t = x0; x0 = x1; x1 = t; }
    if (y0 > y1) { t = y0; y0 = y1; y1 = t; }
    x0 = amoledClamp(x0, _lx0, _lx1); x1 = amoledClamp(x1, _lx0, _lx1);
    y0 = amoledClamp(y0, _ly0, _ly1); y1 = amoledClamp(y1, _ly0, _ly1);
    if (x0 > x1 || y0 > y1) return;

    const uint8_t r = amoledR(c);
    const uint8_t g = amoledG(c);
    const uint8_t b = amoledB(c);
    const ptrdiff_t step = (ptrdiff_t)strideX();

    for (int y = y0; y <= y1; y++)
    {
        uint8_t *p = ptr(x0, y);
        for (int x = x0; x <= x1; x++)
        {
            amoledPxAdd(p, r, g, b);
            p += step;
        }
    }
}

void AMOLED_Canvas::addHLine(int y, int x0, int x1, uint32_t c)
{
    int t;
    if (x0 > x1) { t = x0; x0 = x1; x1 = t; }
    x0 = amoledClamp(x0, _lx0, _lx1);
    x1 = amoledClamp(x1, _lx0, _lx1);
    if (y < _ly0 || y > _ly1 || x0 > x1) return;

    const uint8_t r = amoledR(c);
    const uint8_t g = amoledG(c);
    const uint8_t b = amoledB(c);
    const ptrdiff_t step = (ptrdiff_t)strideX();
    uint8_t *p = ptr(x0, y);
    for (int x = x0; x <= x1; x++)
    {
        amoledPxAdd(p, r, g, b);
        p += step;
    }
}

void AMOLED_Canvas::addVLine(int x, int y0, int y1, uint32_t c)
{
    int t;
    if (y0 > y1) { t = y0; y0 = y1; y1 = t; }
    y0 = amoledClamp(y0, _ly0, _ly1);
    y1 = amoledClamp(y1, _ly0, _ly1);
    if (x < _lx0 || x > _lx1 || y0 > y1) return;

    const uint8_t r = amoledR(c);
    const uint8_t g = amoledG(c);
    const uint8_t b = amoledB(c);
    const ptrdiff_t step = (ptrdiff_t)strideY();
    uint8_t *p = ptr(x, y0);
    for (int y = y0; y <= y1; y++)
    {
        amoledPxAdd(p, r, g, b);
        p += step;
    }
}

void AMOLED_Canvas::addLine(int x0, int y0, int x1, int y1, uint32_t c)
{
    const int dx = abs(x1 - x0);
    const int dy = -abs(y1 - y0);
    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    for (;;)
    {
        addPixel(x0, y0, c);
        if (x0 == x1 && y0 == y1) break;
        const int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void AMOLED_Canvas::addCircle(int cx, int cy, int radius, uint32_t c, uint8_t gain)
{
    if (radius <= 0) { addPixelScaled(cx, cy, c, gain); return; }
    int x = radius;
    int y = 0;
    int err = 1 - radius;

    while (x >= y)
    {
        addPixelScaled(cx + x, cy + y, c, gain); addPixelScaled(cx + y, cy + x, c, gain);
        addPixelScaled(cx - y, cy + x, c, gain); addPixelScaled(cx - x, cy + y, c, gain);
        addPixelScaled(cx - x, cy - y, c, gain); addPixelScaled(cx - y, cy - x, c, gain);
        addPixelScaled(cx + y, cy - x, c, gain); addPixelScaled(cx + x, cy - y, c, gain);
        y++;
        if (err < 0) err += 2 * y + 1;
        else { x--; err += 2 * (y - x) + 1; }
    }
}

/* ------------------------------------------------------------------ */
/*  Radial glow                                                        */
/*                                                                     */
/*  For every pixel of the bounding box the squared distance is        */
/*  projected onto the 256 entry falloff curve, so a whole blob costs  */
/*  one multiply, one shift and one table lookup per channel.  On pure */
/*  black this looks like real light emission - the AMOLED party trick. */
/* ------------------------------------------------------------------ */
void AMOLED_Canvas::addGlow(int cx, int cy, int radius, uint32_t c, uint8_t gain)
{
    if (radius <= 0) return;

    const int x0 = amoledClamp(cx - radius, _lx0, _lx1);
    const int x1 = amoledClamp(cx + radius, _lx0, _lx1);
    const int y0 = amoledClamp(cy - radius, _ly0, _ly1);
    const int y1 = amoledClamp(cy + radius, _ly0, _ly1);
    if (x0 > x1 || y0 > y1) return;

    const uint8_t r = amoledR(c);
    const uint8_t g = amoledG(c);
    const uint8_t b = amoledB(c);
    const uint32_t r2 = (uint32_t)radius * (uint32_t)radius;
    /* (1<<22)/r2 lets the loop compute  k = 256 * d2 / r2  with a multiply
     * and a shift instead of a division */
    const uint32_t recip = (uint32_t)((1UL << 22) / r2);
    const ptrdiff_t step = (ptrdiff_t)strideX();

    for (int y = y0; y <= y1; y++)
    {
        const int dy = y - cy;
        const uint32_t dy2 = (uint32_t)(dy * dy);
        uint8_t *p = ptr(x0, y);
        for (int x = x0; x <= x1; x++)
        {
            const int dx = x - cx;
            const uint32_t k = (((uint32_t)(dx * dx) + dy2) * recip) >> 14;
            if (k < 256u)
            {
                uint32_t a = s_glow[k];
                if (gain != 255u) a = (a * gain) >> 8;
                if (a) amoledPxAddScaled(p, r, g, b, (uint8_t)a);
            }
            p += step;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Arc / ring                                                         */
/*                                                                     */
/*  Angles in degrees, 0 = 3 o'clock, clockwise, -90 = 12 o'clock.      */
/*  512 direction steps are pre-computed, each one is stamped as a small */
/*  disc; a second stamp at the midpoint between two steps keeps the    */
/*  arc perfectly continuous (a single stamp every 2.4 px would look    */
/*  dotted on a 466 px display).                                        */
/* ------------------------------------------------------------------ */
void AMOLED_Canvas::addArc(int cx, int cy, int radius, int startDeg, int endDeg,
                           int thickness, uint32_t c, uint8_t gain)
{
    if (radius <= 0 || endDeg <= startDeg) return;

    int i0 = (int)((int32_t)startDeg * 512 / 360);
    int i1 = (int)((int32_t)endDeg * 512 / 360);
    if (i1 - i0 > 512) i1 = i0 + 512;          /* never more than one turn */

    int tr = thickness / 2;
    if (tr < 1) tr = 1;
    if (tr > 4) tr = 4;
    const int tr2 = tr * tr;

    const int clipY0 = _ly0;
    const int clipY1 = _ly1;

    auto stamp = [&](int px, int py)
    {
        if (py + tr < clipY0 || py - tr > clipY1) return;   /* not in this strip */
        for (int dy = -tr; dy <= tr; dy++)
        {
            const int dy2 = dy * dy;
            for (int dx = -tr; dx <= tr; dx++)
            {
                if (dx * dx + dy2 > tr2) continue;
                addPixelScaled(px + dx, py + dy, c, gain);
            }
        }
    };

    for (int i = i0; i <= i1; i++)
    {
        const int k  = i & 511;
        const int k2 = (i + 1) & 511;
        const int xa = cx + ((s_cosT[k]  * radius) >> 8);
        const int ya = cy + ((s_sinT[k]  * radius) >> 8);
        const int xb = cx + ((s_cosT[k2] * radius) >> 8);
        const int yb = cy + ((s_sinT[k2] * radius) >> 8);
        stamp(xa, ya);
        stamp((xa + xb) / 2, (ya + yb) / 2);
    }
}

/* ------------------------------------------------------------------ */
/*  5x7 text                                                           */
/*                                                                     */
/*  size is the integer scale factor: 1 = 5x7 pixels per character,    */
/*  2 = 10x14, up to 4.  Character pitch is 6*size pixels, so           */
/*  textWidth("ABC", 2) = (3*6-1)*2 = 34.                               */
/* ------------------------------------------------------------------ */
int AMOLED_Canvas::textWidth(const char *s, int size)
{
    if (size < 1) size = 1;
    if (size > 4) size = 4;
    int n = 0;
    while (s && s[n]) n++;
    if (n == 0) return 0;
    return (n * 6 - 1) * size;
}

/* mode: 0 = opaque, 1 = additive, 2 = alpha blend */
static void amoledDrawGlyph(AMOLED_Canvas *cv, int x, int y, const uint8_t *rows, int size,
                            uint32_t c, int mode, uint8_t alpha)
{
    for (int gy = 0; gy < 7; gy++)
    {
        const uint8_t bits = rows[gy];
        for (int gx = 0; gx < 5; gx++)
        {
            if (!(bits & (0x10u >> gx))) continue;
            for (int sy = 0; sy < size; sy++)
            {
                for (int sx = 0; sx < size; sx++)
                {
                    const int px = x + gx * size + sx;
                    const int py = y + gy * size + sy;
                    if      (mode == 0) cv->drawPixel(px, py, c);
                    else if (mode == 1) cv->addPixel(px, py, c);
                    else                cv->blendPixel(px, py, c, alpha);
                }
            }
        }
    }
}

static void amoledDrawText(AMOLED_Canvas *cv, int x, int y, const char *s, uint32_t c,
                           int size, int mode, uint8_t alpha)
{
    if (size < 1) size = 1;
    if (size > 4) size = 4;
    int cx = x;
    for (const char *p = s; p && *p; p++)
    {
        const uint8_t *rows = amoledFontFind(*p);
        if (rows) amoledDrawGlyph(cv, cx, y, rows, size, c, mode, alpha);
        cx += 6 * size;
    }
}

void AMOLED_Canvas::drawText(int x, int y, const char *s, uint32_t c, int size)
{
    amoledDrawText(this, x, y, s, c, size, 0, 255);
}

void AMOLED_Canvas::drawTextAdd(int x, int y, const char *s, uint32_t c, int size)
{
    amoledDrawText(this, x, y, s, c, size, 1, 255);
}

void AMOLED_Canvas::drawTextBlend(int x, int y, const char *s, uint32_t c, int size, uint8_t alpha)
{
    amoledDrawText(this, x, y, s, c, size, 2, alpha);
}

void AMOLED_Canvas::drawTextCentered(int cx, int y, const char *s, uint32_t c, int size)
{
    drawText(cx - textWidth(s, size) / 2, y, s, c, size);
}

void AMOLED_Canvas::drawTextCenteredBlend(int cx, int y, const char *s, uint32_t c, int size, uint8_t alpha)
{
    drawTextBlend(cx - textWidth(s, size) / 2, y, s, c, size, alpha);
}



