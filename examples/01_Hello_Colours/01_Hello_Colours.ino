/*
 * 01_Hello_Colours - WaveshareAMOLED library
 * ============================================================================
 *  The "hello world" example, using the STREAMING mode: the frame is drawn
 *  strip by strip and each strip is pushed asynchronously, so drawing and the
 *  QSPI transfer overlap.  Nothing has to be remembered between frames - every
 *  strip is drawn from scratch, which makes this mode the fastest and the
 *  easiest to reason about for animation.
 *
 *  On screen: named colours, a 24 bit grey ramp, the 1024 entry rainbow, some
 *  shapes, centred text and a soft glow wandering over the picture.
 *
 *  The pattern to remember:
 *
 *      amoled.beginFrame();
 *      while (amoled.nextStrip(cv)) {
 *          ... draw into cv (it is clipped to the current strip) ...
 *          amoled.pushStrip();
 *      }
 *      amoled.endFrame();
 * ============================================================================
 */
#include <WaveshareAMOLED.h>

AMOLED amoled;

static int W, H, CX, CY;

static const uint32_t primary[8] =
{
    AMOLED_RED, AMOLED_ORANGE, AMOLED_YELLOW, AMOLED_GREEN,
    AMOLED_CYAN, AMOLED_BLUE, AMOLED_PURPLE, AMOLED_MAGENTA
};

/* draws the whole picture - called once per strip, everything outside the
 * current strip is clipped away automatically */
static void drawPicture(AMOLED_Canvas &cv, uint32_t t)
{
    /* ---- eight saturated primaries --------------------------------- */
    for (int i = 0; i < 8; i++)
    {
        cv.fillRect(70 + i * 42, 40, 70 + i * 42 + 34, 100, primary[i]);
    }

    /* ---- 24 bit grey ramp: 466 real steps, one per column ----------- */
    for (int x = 0; x < W; x++)
    {
        const uint8_t g = (uint8_t)((x * 255) / (W - 1));
        cv.drawVLine(x, 120, 160, amoledRGB(g, g, g));
    }

    /* ---- the 1024 entry rainbow ------------------------------------ */
    for (int x = 0; x < W; x++)
    {
        cv.drawVLine(x, 180, 220, amoled.palette((uint16_t)((x * 1024L) / W)));
    }

    /* ---- shapes ----------------------------------------------------- */
    cv.drawRect(40, 250, 200, 330, AMOLED_WHITE);
    cv.fillCircle(120, 290, 30, AMOLED_DODGERBLUE);
    cv.drawCircle(120, 290, 36, AMOLED_WHITE);
    cv.drawLine(220, 250, 420, 330, AMOLED_GOLD);
    cv.fillRect(240, 250, 400, 330, amoledHSV(200.0f, 1.0f, 0.35f));

    /* ---- text, centred because the visible area is a circle --------- */
    cv.drawTextCentered(CX, 20, "WAVESHARE AMOLED", AMOLED_WHITE, 1);
    cv.drawTextCentered(CX, 355, "24 BIT COLOUR", AMOLED_WHITE, 2);
    cv.drawTextCentered(CX, 385, "RGB888 16.7M", AMOLED_LIGHTGREY, 1);

    /* ---- the animated part: a soft glow wandering over the picture ---
     * addGlow() only *adds* light, so wherever it passes the colours get
     * brighter (up to white) - the AMOLED party trick. */
    const int gx = CX + (int)(cosf(t * 0.0007f) * 150.0f);
    const int gy = CY + (int)(sinf(t * 0.0011f) * 150.0f);
    cv.addGlow(gx, gy, 90, amoled.palette((uint16_t)(t / 8)), 90);
}

void setup()
{
    Serial.begin(115200);

    if (!amoled.begin())
    {
        Serial.println("panel init failed");
        while (true) delay(1000);
    }
    amoled.setBrightness(255);

    W  = amoled.width();
    H  = amoled.height();
    CX = W / 2;
    CY = H / 2;

    Serial.printf("controller %s, %dx%d, %d bpp, rotation %d\n",
                  amoled.controllerName(), W, H, amoled.colorDepth(), amoled.getRotation());
}

void loop()
{
    AMOLED_Canvas &cv = amoled.canvas();
    const uint32_t t = millis();

    amoled.beginFrame();
    while (amoled.nextStrip(cv))
    {
        cv.clearScreen();                 /* true black: the pixels are off */
        drawPicture(cv, t);
        amoled.pushStrip();               /* async: the DMA of this strip runs
                                           * while the next one is drawn      */
    }
    amoled.endFrame();

    delay(1);
}
