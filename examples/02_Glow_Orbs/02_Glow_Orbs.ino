/*
 * 02_Glow_Orbs - WaveshareAMOLED library
 * ============================================================================
 *  The streaming (strip) mode: no framebuffer, the frame is rendered and DMA'ed
 *  strip by strip, so rendering and the QSPI transfer overlap.
 *
 *  On screen: additive light blobs floating on mathematically exact #000000.
 *  A black AMOLED pixel is a pixel that is off, so the contrast is infinite -
 *  glowing on black is what this display does best.
 *
 *  Touch: the orbs are pulled towards your finger.
 * ============================================================================
 */
#include <WaveshareAMOLED.h>

AMOLED amoled;

#define ORB_COUNT 6

struct Orb
{
    float ax, ay;          /* motion amplitude  */
    float fx, fy;          /* motion frequency  */
    float phase;
    float radius;
    float hue;             /* 0..1024 palette position */
};

static Orb   orbs[ORB_COUNT];
static int   width, height;

void setup()
{
    Serial.begin(115200);
    amoled.begin();
    amoled.setBrightness(255);

    width  = amoled.width();
    height = amoled.height();

    /* random, but always the same "look" - tweak the values if you like */
    randomSeed(esp_random());
    for (int i = 0; i < ORB_COUNT; i++)
    {
        orbs[i].ax     = 55.0f + random(0, 90);
        orbs[i].ay     = 55.0f + random(0, 90);
        orbs[i].fx     = 0.18f + random(0, 55) / 100.0f;
        orbs[i].fy     = 0.18f + random(0, 55) / 100.0f;
        orbs[i].phase  = random(0, 628) / 100.0f;
        orbs[i].radius = 58.0f + random(0, 62);
        orbs[i].hue    = random(0, 1024);
    }
}

void loop()
{
    /* ---- one frame, strip by strip ---------------------------------- */
    AMOLED_Canvas &cv = amoled.canvas();
    const uint32_t now = millis();
    const float t = now * 0.001f;

    amoled.beginFrame();
    while (amoled.nextStrip(cv))
    {
        cv.clearScreen();                     /* true black background */

        for (int i = 0; i < ORB_COUNT; i++)
        {
            const Orb &o = orbs[i];
            float cx = width  * 0.5f + cosf(t * o.fx + o.phase) * o.ax;
            float cy = height * 0.5f + sinf(t * o.fy + o.phase * 1.7f) * o.ay;

            if (amoled.touch().isDown())      /* orbs follow the finger */
            {
                cx += ((float)amoled.touch().x() - cx) * 0.7f;
                cy += ((float)amoled.touch().y() - cy) * 0.7f;
            }

            const uint32_t col = amoled.palette((uint16_t)(int)(o.hue + t * 52.0f * (1.0f + 0.18f * i)));

            cv.addGlow((int)cx, (int)cy, (int)o.radius, col, 255);          /* the colour halo */
            cv.addGlow((int)cx, (int)cy, (int)(o.radius * 0.25f),
                       AMOLED_WHITE, 200);                                   /* the hot core    */

            /* a few twinkling sparkles inside the halo */
            for (int k = 0; k < 7; k++)
            {
                const float a  = t * (0.5f + 0.21f * k) + k * 1.31f + o.phase;
                const float rr = o.radius * (0.35f + 0.22f * (k % 3));
                cv.addPixelScaled((int)(cx + cosf(a * 1.7f) * rr),
                                  (int)(cy + sinf(a * 1.3f) * rr),
                                  AMOLED_WHITE,
                                  (uint8_t)(110 + 145 * (0.5f + 0.5f * sinf(a * 4.0f))));
            }
        }

        /* a little HUD, centred so it stays inside the round visible area */
        cv.drawTextCentered(width / 2, 40, "GLOW ORBS", AMOLED_WHITE, 2);
        cv.drawTextCentered(width / 2, height - 60, "TOUCH TO PULL", AMOLED_DARKGREY, 1);

        amoled.pushStrip();                   /* async: overlaps with the next strip */
    }
    amoled.endFrame();

    amoled.touch().update();                  /* poll the touch panel */
    delay(1);
}
