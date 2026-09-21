/*
 * 03_Contrast_Test - WaveshareAMOLED library
 * ============================================================================
 *  A test card for the panel, drawn in streaming mode (no framebuffer).
 *
 *  What it proves:
 *    - saturated primaries at full RGB888 depth
 *    - a grey ramp with 466 real steps (no banding)
 *    - the 16 *darkest* and the 16 *brightest* grey levels next to each other
 *      (only an AMOLED shows #000000 next to #010101)
 *    - a moving 1 pixel checkerboard: maximum contrast plus maximum detail
 *    - a frequency sweep and a full 24 bit hue sweep
 *
 *  Everything is kept inside the circle the panel actually shows.
 * ============================================================================
 */
#include <WaveshareAMOLED.h>

AMOLED amoled;

static int  W, H, CX;
static uint8_t gray[466];

static const char *labels[5] =
{
    "8 BIT GREY RAMP 0-255",
    "DEEP BLACK 000-015 / 240-255",
    "1 PIXEL CHECKERBOARD",
    "FREQUENCY SWEEP 1-32 PX",
    "24 BIT HUE SWEEP 466 STEPS",
};
static const int labelY[5] = {134, 174, 218, 274, 354};

void setup()
{
    Serial.begin(115200);
    amoled.begin();
    amoled.setBrightness(255);

    W = amoled.width();
    H = amoled.height();
    CX = W / 2;

    for (int x = 0; x < W; x++) gray[x] = (uint8_t)((x * 255) / (W - 1));

    Serial.printf("contrast test on %s, %dx%d, %d bpp\n",
                  amoled.controllerName(), W, H, amoled.colorDepth());
}

void loop()
{
    AMOLED_Canvas &cv = amoled.canvas();
    const uint32_t now = millis();
    const int scroll = (int)(now / 4u);            /* ~250 px per second */
    const int x0 = cv.clipX0(), x1 = cv.clipX1();

    static const uint32_t swatch[8] =
    {
        0xFF0000u, 0xFF8000u, 0xFFF000u, 0x00FF20u,
        0x00E0FFu, 0x0030FFu, 0x9000FFu, 0xFF00A0u
    };

    amoled.beginFrame();
    while (amoled.nextStrip(cv))
    {
        cv.clearScreen();

        /* ---- 8 saturated primaries, centred ------------------------- */
        for (int i = 0; i < 8; i++)
        {
            const int sx = 69 + i * 42;
            cv.fillRect(sx, 96, sx + 34, 128, swatch[i]);
        }

        /* ---- the patterns, one screen row at a time ----------------- */
        for (int y = cv.clipY0(); y <= cv.clipY1(); y++)
        {
            uint8_t *p = cv.ptr(x0, y);
            const int step = cv.strideX();

            if (y >= 144 && y <= 168)                          /* grey ramp */
            {
                for (int x = x0; x <= x1; x++) { const uint8_t g = gray[x]; amoledPxSet(p, g, g, g); p += step; }
            }
            else if (y >= 182 && y <= 212)                     /* floor / ceiling */
            {
                const bool floorBand = (y <= 196);
                for (int x = x0; x <= x1; x++)
                {
                    int blk = x / 29; if (blk > 15) blk = 15;
                    const int v = floorBand ? blk : (240 + blk);
                    amoledPxSet(p, (uint8_t)v, (uint8_t)v, (uint8_t)v);
                    p += step;
                }
            }
            else if (y >= 226 && y <= 268)                     /* 1 px checkerboard */
            {
                for (int x = x0; x <= x1; x++)
                {
                    const uint8_t v = ((x + scroll) & 1) ? 0 : 255;
                    amoledPxSet(p, v, v, v); p += step;
                }
            }
            else if (y >= 282 && y <= 348)                     /* frequency sweep */
            {
                const int shift = (y - 282) / 11;
                for (int x = x0; x <= x1; x++)
                {
                    const uint8_t v = (((x + scroll) >> shift) & 1) ? 0 : 255;
                    amoledPxSet(p, v, v, v); p += step;
                }
            }
            else if (y >= 362 && y <= 394)                     /* 24 bit hue sweep */
            {
                for (int x = x0; x <= x1; x++)
                {
                    const uint32_t c = amoled.palette((uint16_t)((x * 1024L) / W));
                    amoledPxSet(p, amoledR(c), amoledG(c), amoledB(c));
                    p += step;
                }
            }
        }

        /* ---- centred labels ---------------------------------------- */
        for (int i = 0; i < 5; i++)
        {
            cv.drawTextCenteredBlend(CX, labelY[i], labels[i], AMOLED_WHITE, 1, 215);
        }

        amoled.pushStrip();
    }
    amoled.endFrame();
    delay(1);
}
