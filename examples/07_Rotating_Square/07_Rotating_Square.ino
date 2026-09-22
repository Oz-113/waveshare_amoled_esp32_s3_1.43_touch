/*
 * 07_Rotating_Square - WaveshareAMOLED library
 * ============================================================================
 *  The simplest animation there is: one square, rotating, its colour walking
 *  around the rainbow.  The frame rate is printed on the serial port (115200
 *  baud) once a second, and drawn on the screen as well.
 *
 *  Shows: streaming mode (beginFrame / nextStrip / pushStrip), additive
 *  drawing, and how to draw a *rotated*, *filled* shape with nothing but
 *  sin/cos: rotate the four corners, then walk the scanlines of their
 *  bounding box - for every scanline the outermost two crossings of the four
 *  edges are the span to fill.  (06_Rotating_Cube fills its faces the same
 *  way.)
 *
 *  No PSRAM needed: one strip at a time is drawn in internal DMA RAM, so this
 *  runs on any ESP32-S3 build.  A full RGB888 frame is 651 kB and the QSPI bus
 *  sends ~40 Mbit/s per MHz of clock, which is where the ~30 fps ceiling comes
 *  from at the default 40 MHz.  For more speed see the dirty rectangle notes in
 *  04_Touch_Demo and 06_Rotating_Cube, or raise AMOLED_QSPI_CLOCK_HZ in the
 *  library's AMOLED_Config.h.
 *
 *  SQUARE_TRAIL adds two dimmer copies of the square a fraction of a turn
 *  behind it.  They read as motion blur, and they also hide the seam the panel
 *  shows while a frame is being sent - the panel is written while it scans out,
 *  and on a hard bright edge that seam is easy to see.  Set SQUARE_TRAIL to 0
 *  for one hard edged square.
 *
 *  WHAT THE SERIAL PORT PRINTS
 *      fps  29.9 | frame 33.44 ms | strips 15
 * ============================================================================
 */
#include <WaveshareAMOLED.h>

/* ================================================================== */
/*  Knobs                                                             */
/* ================================================================== */
#define SQUARE_HALF     120.0f     /* half of one side, in pixels            */
#define SQUARE_SPIN     36.0f      /* degrees per second                     */
#define SQUARE_FILL     0.55f      /* fill brightness, 0..1                  */
#define SQUARE_EDGE     1.0f       /* edge brightness, 0..1                  */
#define SQUARE_TRAIL    2          /* ghost copies behind the square, 0..2   */
#define SQUARE_FPS_TEXT 1          /* frame rate on the display, too         */

/* ================================================================== */
/*  Objects, and where they come from                                 */
/*                                                                    */
/*    AMOLED         - the display object   (WaveshareAMOLED lib)     */
/*    AMOLED_Canvas  - the drawing surface  (WaveshareAMOLED lib)     */
/*    Serial         - USB/UART console     (Arduino core)            */
/*    millis()       - milliseconds         (Arduino core)            */
/*    DEG_TO_RAD     - degrees -> radians   (Arduino core)            */
/*    cosf/sinf      - math                 (C standard library)      */
/* ================================================================== */
AMOLED amoled;

static int   W, H, CX, CY;         /* screen size and centre                 */
static float angleDeg = 0.0f;      /* the square's rotation                  */

/* frame rate measurement */
static uint32_t fpsFrames  = 0;
static uint32_t fpsStartMs = 0;
static uint32_t lastMs     = 0;
static float    fps        = 0.0f;

/* ------------------------------------------------------------------ */
/*  The four corners of the square, rotated, centred on the screen     */
/*                                                                    */
/*  A corner lives at (+/-SQUARE_HALF, +/-SQUARE_HALF) in the square's */
/*  own space.  Rotating it by an angle is one 2x2 rotation:           */
/*                                                                    */
/*      x' = x * cos(a) - y * sin(a)                                  */
/*      y' = x * sin(a) + y * cos(a)                                  */
/*                                                                    */
/*  and then it is moved to the middle of the screen.  Screen y grows  */
/*  downwards, so a positive angle turns clockwise - which is what you */
/*  expect an angle on a screen to do.                                 */
/* ------------------------------------------------------------------ */
static inline int iround(float v) { return (int)(v < 0.0f ? v - 0.5f : v + 0.5f); }

static void squareCorners(int *xs, int *ys, float angDeg)
{
    static const float kLocal[4][2] = { {-1, -1}, {+1, -1}, {+1, +1}, {-1, +1} };

    const float a = angDeg * DEG_TO_RAD;
    const float c = cosf(a);
    const float s = sinf(a);

    for (int i = 0; i < 4; i++)
    {
        const float lx = kLocal[i][0] * SQUARE_HALF;
        const float ly = kLocal[i][1] * SQUARE_HALF;
        xs[i] = CX + iround(lx * c - ly * s);
        ys[i] = CY + iround(lx * s + ly * c);
    }
}

/* ------------------------------------------------------------------ */
/*  Fill a convex quad, additively                                    */
/*                                                                    */
/*  For every scanline of the bounding box the four edges are tested   */
/*  for a crossing; the leftmost and the rightmost one are the span to */
/*  draw.  addHLine() *adds* light instead of replacing the pixel, so  */
/*  shapes that overlap brighten each other on a black AMOLED.         */
/* ------------------------------------------------------------------ */
static void fillQuad(AMOLED_Canvas &cv, const int *xs, const int *ys, uint32_t col)
{
    int minX = xs[0], maxX = xs[0], minY = ys[0], maxY = ys[0];
    for (int i = 1; i < 4; i++)
    {
        if (xs[i] < minX) minX = xs[i];
        if (xs[i] > maxX) maxX = xs[i];
        if (ys[i] < minY) minY = ys[i];
        if (ys[i] > maxY) maxY = ys[i];
    }

    for (int y = minY; y <= maxY; y++)
    {
        int xMin = maxX, xMax = minX;             /* "nothing found yet" */
        for (int i = 0; i < 4; i++)
        {
            const int j  = (i + 1) & 3;
            const int ya = ys[i], yb = ys[j];
            /* half open test: a corner is never counted twice */
            if ((ya <= y && yb > y) || (yb <= y && ya > y))
            {
                const int xa = xs[i], xb = xs[j];
                int x = xa + (int)(((int32_t)(xb - xa) * (y - ya)) / (yb - ya));
                if (x < minX) x = minX;           /* stay inside the quad */
                if (x > maxX) x = maxX;
                if (x < xMin) xMin = x;
                if (x > xMax) xMax = x;
            }
        }
        if (xMax >= xMin) cv.addHLine(y, xMin, xMax, col);
    }
}

/* the four edges of one square, at one angle */
static void drawSquareEdges(AMOLED_Canvas &cv, float angDeg, uint32_t col)
{
    int xs[4], ys[4];
    squareCorners(xs, ys, angDeg);
    cv.addLine(xs[0], ys[0], xs[1], ys[1], col);
    cv.addLine(xs[1], ys[1], xs[2], ys[2], col);
    cv.addLine(xs[2], ys[2], xs[3], ys[3], col);
    cv.addLine(xs[3], ys[3], xs[0], ys[0], col);
}

/* ================================================================== */
/*  setup() - runs once                                               */
/* ================================================================== */
void setup()
{
    /* With "USB CDC On Boot" Serial is the native USB port: if no host has it
     * open a write would block for up to two seconds, so the timeout is set
     * to zero and every print is guarded with "if (Serial)". */
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);

    if (!amoled.begin())                      /* panel + touch + lookup tables */
    {
        if (Serial) Serial.println("panel init failed");
        while (true) delay(1000);
    }
    amoled.setBrightness(255);

    W  = amoled.width();
    H  = amoled.height();
    CX = W / 2;
    CY = H / 2;

    lastMs     = millis();
    fpsStartMs = lastMs;

    if (Serial)
    {
        Serial.printf("\n=== rotating square ===\n");
        Serial.printf("%s  %dx%d  %d bpp  QSPI %d MHz\n",
                      amoled.controllerName(), W, H, amoled.colorDepth(),
                      (int)(AMOLED_QSPI_CLOCK_HZ / 1000000UL));
    }
}

/* ================================================================== */
/*  loop() - one frame per call, as fast as the panel allows          */
/* ================================================================== */
void loop()
{
    const uint32_t now = millis();
    float dt = (now - lastMs) * 0.001f;
    lastMs = now;
    if (dt <= 0.0f) dt = 0.001f;
    if (dt > 0.2f)  dt = 0.2f;                /* after a pause: no jump */

    angleDeg += SQUARE_SPIN * dt;
    if (angleDeg >= 360.0f) angleDeg -= 360.0f;

    /* the square, and its colour - a full rainbow every ten seconds */
    int xs[4], ys[4];
    squareCorners(xs, ys, angleDeg);

    const float hue = angleDeg * 3.0f;
    const uint32_t fillCol = amoledScale(amoledHSV(hue, 1.0f, 1.0f),
                                         (uint16_t)(SQUARE_FILL * 255.0f), 255);
    const uint32_t edgeCol = amoledScale(amoledHSV(hue + 25.0f, 1.0f, 1.0f),
                                         (uint16_t)(SQUARE_EDGE * 255.0f), 255);

    /* ---- draw, strip by strip -------------------------------------- */
    AMOLED_Canvas &cv = amoled.canvas();
    amoled.beginFrame();
    while (amoled.nextStrip(cv))
    {
        cv.clearScreen();                     /* true black, pixels off */

#if SQUARE_TRAIL
        /* two dimmer copies a little way behind: motion blur.  They are drawn
         * first so the fill and the edges of the real square shine through
         * them (everything here is additive). */
        for (int g = 1; g <= SQUARE_TRAIL; g++)
        {
            drawSquareEdges(cv, angleDeg - g * 9.0f,
                            amoledScale(edgeCol, (uint8_t)(70 / g), 255));
        }
#endif

        fillQuad(cv, xs, ys, fillCol);

        for (int i = 0; i < 4; i++)
        {
            const int j = (i + 1) & 3;
            cv.addLine(xs[i], ys[i], xs[j], ys[j], edgeCol);
        }

#if SQUARE_FPS_TEXT
        /* The frame rate.  Streaming mode draws the whole frame anyway, so
         * an on screen counter costs nothing extra here. */
        char buf[24];
        snprintf(buf, sizeof(buf), "%2d FPS", (int)(fps + 0.5f));
        cv.drawTextCentered(CX, 26, buf, AMOLED_DARKGREY, 2);
#else
        (void)fps;
#endif

        amoled.pushStrip();                   /* async DMA of this strip */
    }
    amoled.endFrame();

    /* ---- measure and report the frame rate ------------------------ */
    fpsFrames++;
    const uint32_t now2 = millis();
    if ((now2 - fpsStartMs) >= 1000)
    {
        fps = fpsFrames * 1000.0f / (float)(now2 - fpsStartMs);
        fpsFrames  = 0;
        fpsStartMs = now2;

        if (Serial)
        {
            Serial.printf("fps %5.1f | frame %5.2f ms | strips %d\n",
                          fps,
                          1000.0f / (fps > 0.01f ? fps : 0.01f),
                          (int)((H + AMOLED_STRIP_LINES - 1) / AMOLED_STRIP_LINES));
        }
    }
}
