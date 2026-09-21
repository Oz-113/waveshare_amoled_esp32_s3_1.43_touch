/*
 * 01_Hello_Colours - WaveshareAMOLED library
 * ============================================================================
 *  The "hello world" of the library, using the framebuffer mode:
 *  draw everything you like, then push it to the panel.
 *
 *  Shows: names colours, RGB888 gradients, shapes, text and the colour wheel,
 *  then animates a soft glow around the screen.
 *
 *  Tested on: Waveshare ESP32-S3-Touch-AMOLED-1.43 (466x466, SH8601/CO5300)
 * ============================================================================
 */
#include <WaveshareAMOLED.h>

AMOLED amoled;

void setup()
{
    Serial.begin(115200);

    if (!amoled.begin())
    {
        Serial.println("panel init failed");
        while (true) delay(1000);
    }
    amoled.setBrightness(255);

    /* framebuffer mode: 651 kB of PSRAM, everything is drawn in RAM first */
    if (!amoled.beginFramebuffer())
    {
        Serial.println("no PSRAM - use the streaming examples instead");
        while (true) delay(1000);
    }

    AMOLED_Canvas &fb = amoled.canvas();
    fb.clearScreen();                                  /* #000000, pixels off */

    /* ---- eight saturated primaries --------------------------------- */
    static const uint32_t primary[8] =
    {
        AMOLED_RED, AMOLED_ORANGE, AMOLED_YELLOW, AMOLED_GREEN,
        AMOLED_CYAN, AMOLED_BLUE, AMOLED_PURPLE, AMOLED_MAGENTA
    };
    for (int i = 0; i < 8; i++)
    {
        fb.fillRect(70 + i * 42, 40, 70 + i * 42 + 34, 100, primary[i]);
    }

    /* ---- a 24 bit grey ramp (466 real steps, one per column) -------- */
    for (int x = 0; x < amoled.width(); x++)
    {
        const uint8_t g = (uint8_t)((x * 255) / (amoled.width() - 1));
        fb.drawVLine(x, 120, 160, amoled.rgb(g, g, g));
    }

    /* ---- the 1024 entry rainbow, one colour per column -------------- */
    for (int x = 0; x < amoled.width(); x++)
    {
        fb.drawVLine(x, 180, 220, amoled.palette((uint16_t)((x * 1024L) / amoled.width())));
    }

    /* ---- shapes ----------------------------------------------------- */
    fb.drawRect(40, 250, 200, 330, AMOLED_WHITE);
    fb.fillCircle(120, 290, 30, AMOLED_DODGERBLUE);
    fb.drawCircle(120, 290, 36, AMOLED_WHITE);
    fb.drawLine(220, 250, 420, 330, AMOLED_GOLD);
    fb.fillRect(240, 250, 400, 330, amoled.hsv(200, 1.0f, 0.35f));

    /* ---- text: centred, because the visible area is a circle -------- */
    fb.drawTextCentered(amoled.width() / 2, 20, "WAVESHARE AMOLED", AMOLED_WHITE, 1);
    fb.drawTextCentered(amoled.width() / 2, 355, "24 BIT COLOUR", AMOLED_WHITE, 2);
    fb.drawTextCentered(amoled.width() / 2, 385, "RGB888 16.7M", AMOLED_LIGHTGREY, 1);

    amoled.push();                                     /* blit it to the panel */
    Serial.printf("controller %s, %d bpp\n", amoled.controllerName(), amoled.colorDepth());
}

void loop()
{
    AMOLED_Canvas &fb = amoled.canvas();
    const int cx = amoled.width() / 2;
    const int cy = amoled.height() / 2;
    const uint32_t t = millis();

    /* the animated part: a dim, slowly rotating glow over the static image.
     * addGlow() only *adds* light - the more you overlap them, the brighter
     * the area gets, up to white. */
    fb.addGlow(cx + (int)(cos((float)t * 0.0007f) * 150.0f),
               cy + (int)(sin((float)t * 0.0011f) * 150.0f),
               90, amoled.palette((uint16_t)(t / 8)), 90);

    /* only the region we touched is sent to the panel */
    amoled.pushRect(cx - 240, cy - 240, cx + 240, cy + 240);
    delay(20);
}
