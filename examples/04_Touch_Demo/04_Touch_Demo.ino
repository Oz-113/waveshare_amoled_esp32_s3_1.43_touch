/*
 * 04_Touch_Demo - WaveshareAMOLED library
 * ============================================================================
 *  Everything the FT3168 touch panel can do, with an on screen reaction:
 *
 *    raw      - touch().read(x, y) gives the raw position
 *    state    - touch().isDown(), x(), y(), pressTime()
 *    tap      - touch().tapped()     short press, fires once on release
 *    hold     - touch().held(ms)     true while it has been held that long
 *    one shot - touch().heldOnce(ms) fires exactly once per press
 *
 *  Drawing strategy: one static background is pushed once, after that only the
 *  small status panel is redrawn and sent with pushRect() - that is how you do
 *  cheap UI updates on this panel.
 * ============================================================================
 */
#include <WaveshareAMOLED.h>

AMOLED amoled;

static int      W, H, CX, CY;
static uint32_t taps = 0, holds = 0;
static int      lastX = -1, lastY = -1;
static bool     lastDown = false;

#define PANEL_X0 (CX - 130)
#define PANEL_Y0 (CY - 70)
#define PANEL_X1 (CX + 130)
#define PANEL_Y1 (CY + 70)

static inline int clampInt(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void drawPanel(void)
{
    AMOLED_Canvas &fb = amoled.canvas();
    char buf[40];

    fb.fillRect(PANEL_X0, PANEL_Y0, PANEL_X1, PANEL_Y1, AMOLED_BLACK);
    fb.drawRect(PANEL_X0, PANEL_Y0, PANEL_X1, PANEL_Y1, AMOLED_DARKGREY);

    snprintf(buf, sizeof(buf), "TAPS %u", (unsigned)taps);
    fb.drawTextCentered(CX, PANEL_Y0 + 10, buf, AMOLED_WHITE, 2);

    snprintf(buf, sizeof(buf), "HOLDS %u", (unsigned)holds);
    fb.drawTextCentered(CX, PANEL_Y0 + 34, buf, AMOLED_LIGHTGREY, 1);

    if (amoled.touch().isDown())
    {
        const int x = amoled.touch().x();
        const int y = amoled.touch().y();
        const int bx = PANEL_X0 + 12;
        const int bw = (PANEL_X1 - 12) - bx;

        snprintf(buf, sizeof(buf), "X %3d  Y %3d", x, y);
        fb.drawTextCentered(CX, PANEL_Y0 + 50, buf, AMOLED_CYAN, 1);

        /* two bars that show where the finger is */
        fb.fillRect(bx, PANEL_Y0 + 66, bx + bw, PANEL_Y0 + 71, AMOLED_DARKGREY);
        fb.fillRect(bx, PANEL_Y0 + 66, bx + (x * bw) / W, PANEL_Y0 + 71, AMOLED_CYAN);
        fb.fillRect(bx, PANEL_Y0 + 78, bx + bw, PANEL_Y0 + 83, AMOLED_DARKGREY);
        fb.fillRect(bx, PANEL_Y0 + 78, bx + (y * bw) / H, PANEL_Y0 + 83, AMOLED_ORANGE);

        /* the ripple: clamped into the panel, so the next repaint of the
         * panel erases it again.  (An additive glow drawn outside the panel
         * would pile up in the framebuffer for ever - that is the one trap of
         * the framebuffer mode.) */
        const int gx = clampInt(x, PANEL_X0 + 34, PANEL_X1 - 34);
        const int gy = clampInt(y, PANEL_Y0 + 34, PANEL_Y1 - 34);
        fb.addGlow(gx, gy, 26, AMOLED_WHITE, 90);
        fb.addArc(gx, gy, 22, 0, 360, 2, AMOLED_WHITE, 110);
    }
    else
    {
        fb.drawTextCentered(CX, PANEL_Y0 + 60, "TOUCH ME", AMOLED_CYAN, 1);
        fb.drawTextCentered(CX, PANEL_Y0 + 90, "TAP = COUNT   HOLD = FLASH", AMOLED_DARKGREY, 1);
    }

    /* only the panel is sent to the display - one DMA transfer per row */
    amoled.pushRect(PANEL_X0, PANEL_Y0, PANEL_X1, PANEL_Y1);
}

void setup()
{
    Serial.begin(115200);
    if (!amoled.begin()) while (true) delay(1000);
    amoled.setBrightness(220);

    W = amoled.width();
    H = amoled.height();
    CX = W / 2;
    CY = H / 2;

    if (!amoled.beginFramebuffer())
    {
        Serial.println("no PSRAM - this example uses framebuffer mode");
        while (true) delay(1000);
    }

    /* static background: concentric rings, all inside the visible circle */
    AMOLED_Canvas &fb = amoled.canvas();
    fb.clearScreen();
    for (int r = 40; r < 220; r += 30)
    {
        fb.addCircle(CX, CY, r, amoled.palette((uint16_t)(r * 11)), 70);
    }
    fb.drawTextCentered(CX, 46, "TOUCH DEMO", AMOLED_WHITE, 2);
    fb.drawTextCentered(CX, H - 66, "FT3168 VIA I2C", AMOLED_DARKGREY, 1);
    amoled.push();
    amoled.waitIdle();

    amoled.touch().begin();
    drawPanel();
}

void loop()
{
    amoled.touch().update();

    /* tap: one counter step */
    if (amoled.touch().tapped())
    {
        taps++;
        Serial.printf("tap  #%u at %u/%u\n", (unsigned)taps,
                      amoled.touch().x(), amoled.touch().y());
        drawPanel();
    }

    /* held longer than 700 ms: one step per press, with a brightness flash */
    if (amoled.touch().heldOnce(700))
    {
        holds++;
        Serial.printf("hold #%u\n", (unsigned)holds);
        amoled.fadeBrightness(90, 80);
        amoled.fadeBrightness(220, 80);
        drawPanel();
    }

    /* redraw the indicator only when something actually changed */
    const bool down = amoled.touch().isDown();
    if (down && (amoled.touch().x() != lastX || amoled.touch().y() != lastY))
    {
        lastX = amoled.touch().x();
        lastY = amoled.touch().y();
        drawPanel();
    }
    else if (down != lastDown)
    {
        drawPanel();
    }
    lastDown = down;

    delay(5);
}
