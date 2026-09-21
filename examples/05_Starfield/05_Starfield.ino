/*
 * 05_Starfield - WaveshareAMOLED library
 * ============================================================================
 *  240 point lights flying out of the middle of the screen, pure black sky.
 *  An AMOLED draws a star as a *point of light*: one pixel, no backlight
 *  bleeding through, no grey.  Hold a finger on the panel and the whole field
 *  accelerates into a hyperjump.
 *
 *  Shows: streaming mode + additive drawing + touch().held().
 * ============================================================================
 */
#include <WaveshareAMOLED.h>

AMOLED amoled;

#define STAR_COUNT 240

struct Star
{
    float   x, y, z;                  /* normalised space position           */
    int16_t sx, sy, px, py;           /* current / previous screen position  */
    uint32_t col;                     /* colour at full brightness           */
};

static Star  stars[STAR_COUNT];
static int   W, H, CX, CY;
static float speed = 0.60f;

static inline float rnd01() { return random(0, 10000) / 10000.0f; }

static void project(Star &s)
{
    const float inv = 1.0f / s.z;
    s.sx = (int16_t)(CX + s.x * inv * 260.0f);
    s.sy = (int16_t)(CY + s.y * inv * 260.0f);
    s.px = s.sx;
    s.py = s.sy;
}

static void spawn(Star &s)
{
    s.x = (rnd01() * 2.0f - 1.0f) * 0.30f;
    s.y = (rnd01() * 2.0f - 1.0f) * 0.30f;
    s.z = 0.72f + rnd01() * 0.28f;

    const int pick = random(0, 10);
    if      (pick < 6) s.col = AMOLED_WHITE;
    else if (pick < 8) s.col = amoledRGB(255, 233, 191);   /* warm */
    else if (pick < 9) s.col = amoledRGB(176, 222, 255);   /* blue */
    else               s.col = amoledRGB(255, 186, 238);   /* pink */

    project(s);
}

void setup()
{
    Serial.begin(115200);
    amoled.begin();
    amoled.setBrightness(255);

    W = amoled.width();
    H = amoled.height();
    CX = W / 2;
    CY = H / 2;

    randomSeed(esp_random());
    for (int i = 0; i < STAR_COUNT; i++) spawn(stars[i]);
}

void loop()
{
    static uint32_t last = 0;
    const uint32_t now = millis();
    float dt = (now - last) * 0.001f;
    last = now;
    if (dt <= 0.0f) dt = 0.001f;
    if (dt > 0.2f)  dt = 0.2f;

    amoled.touch().update();

    /* hold the screen = warp lever */
    const float target = amoled.touch().held(150) ? 2.4f : 0.60f;
    speed += (target - speed) * (target > 1.0f ? 0.25f : 0.045f);

    /* ---- simulation, once per frame ---------------------------------- */
    for (int i = 0; i < STAR_COUNT; i++)
    {
        Star &s = stars[i];
        s.px = s.sx;
        s.py = s.sy;
        s.z -= speed * dt;
        if (s.z <= 0.22f) { spawn(s); continue; }

        const float inv = 1.0f / s.z;
        s.sx = (int16_t)(CX + s.x * inv * 260.0f);
        s.sy = (int16_t)(CY + s.y * inv * 260.0f);
    }

    /* ---- draw, strip by strip ---------------------------------------- */
    AMOLED_Canvas &cv = amoled.canvas();
    amoled.beginFrame();
    while (amoled.nextStrip(cv))
    {
        cv.clearScreen();

        for (int i = 0; i < STAR_COUNT; i++)
        {
            const Star &s = stars[i];

            /* only the strips the trail can touch */
            const int loY = (s.py < s.sy) ? s.py : s.sy;
            const int hiY = (s.py < s.sy) ? s.sy : s.py;
            if (hiY < cv.clipY0() - 2 || loY > cv.clipY1() + 2) continue;

            /* brightness grows as the star comes closer */
            float br = (1.0f - s.z) * 1.28f;
            if (br > 1.0f) br = 1.0f;
            if (br < 0.0f) br = 0.0f;
            const uint8_t b8 = (uint8_t)(60 + br * 195.0f);

            /* the streak: the same colour at half brightness */
            if (s.px != s.sx || s.py != s.sy)
            {
                cv.addLine(s.px, s.py, s.sx, s.sy,
                           amoledScale(s.col, b8 / 2, 255));
            }
            if (s.z < 0.42f) cv.addGlow(s.sx, s.sy, 7, s.col, 200);
            cv.addPixelScaled(s.sx, s.sy, s.col, b8);
        }

        cv.drawTextCentered(CX, 46, "STAR WARP", AMOLED_WHITE, 2);
        cv.drawTextCentered(CX, H - 60, speed > 1.0f ? "HYPERJUMP" : "HOLD TO JUMP",
                            speed > 1.0f ? AMOLED_ORANGE : AMOLED_DARKGREY, 1);

        amoled.pushStrip();
    }
    amoled.endFrame();
}
