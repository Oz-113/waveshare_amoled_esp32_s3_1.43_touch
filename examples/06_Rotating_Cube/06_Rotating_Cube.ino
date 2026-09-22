/*
 * 06_Rotating_Cube - WaveshareAMOLED library
 * ============================================================================
 *  A neon cube spinning around two axes: twelve glowing edges, six translucent
 *  faces that add light through each other, and a colour that walks the 24 bit
 *  rainbow.  The frame rate is measured and reported on the serial port.
 *
 *  Shows: framebuffer mode + a dirty rectangle redraw + pushRect() + setClip().
 *
 *  This is the *fast* way to animate a small object: the cube only needs a box
 *  of roughly 330 x 330 pixels, so about 300-450 kB per frame go to the panel
 *  instead of the 651 kB of a full frame, and each band of that box is one big
 *  DMA transfer instead of one per row.
 *
 *  WHAT THE SERIAL PORT PRINTS   (115200 baud, once per second)
 *      fps  59.8 | frame 16.72 ms | pushed 447 kB | full width bands
 *
 *  MADE FOR A BOARD WITH PSRAM - the framebuffer needs 651 kB.  Enable
 *  "OPI PSRAM" in the Arduino IDE (Tools -> PSRAM) or the sketch stops with a
 *  message in the serial monitor.  Without PSRAM use 05_Starfield, which draws
 *  a strip at a time and needs no framebuffer.
 *
 *  IF THE EDGES LOOK BROKEN, DOUBLED OR "CUT INTO PARTS"
 *  -----------------------------------------------------
 *  The panel is written while it scans out, so the seam between the part of
 *  the picture that is already new and the part that is still the previous
 *  frame is visible on hard, bright edges like these.  When the loop runs
 *  faster than the panel refreshes, several of those seams are on screen at
 *  once and the cube looks like it is copied into a few horizontal bands.
 *  CUBE_FPS_CAP (60 by default) keeps the update rate at or below the panel's
 *  refresh rate, so only one seam can be on screen at a time.
 *
 *  To tell tearing and rendering bugs apart:
 *      set CUBE_SPIN_X and CUBE_SPIN_Y to 0.0f (a frozen cube)
 *      - the artifacts disappear  -> they were tearing (frame rate)
 *      - they stay on a frozen cube -> something in the drawing is wrong
 * ============================================================================
 */
#include <WaveshareAMOLED.h>

/* ================================================================== */
/*  Knobs                                                             */
/* ================================================================== */
#define CUBE_DIRTY_RECT   1        /* 0 = push the whole frame every time    */
#define CUBE_FULL_WIDTH   1        /* 1 = push full width bands (the shape   */
                                   /*     the streaming mode uses)           */
                                   /* 0 = push only the cube's own box       */
#define CUBE_FPS_CAP      60       /* updates per second, 0 = as fast as it  */
                                   /* gets (more tearing, bigger number)     */
#define CUBE_FACES        1        /* 1 = translucent additive faces         */
#define CUBE_EDGE_GLOW    1        /* 1 = soft halo around every edge        */
#define CUBE_SHOW_FPS     1        /* frame rate on the display              */
#define CUBE_SERIAL_FPS   1        /* frame rate on the serial port          */

/* Geometry / animation */
#define CUBE_CAM_DIST     3.6f     /* camera distance, in cube units         */
#define CUBE_FOCAL        270.0f   /* "lens": pixels per unit at distance 1  */
#define CUBE_SPIN_X       0.75f    /* pitch speed, radians per second        */
#define CUBE_SPIN_Y       1.15f    /* yaw speed, radians per second          */
#define CUBE_HUE_SPEED    55.0f    /* degrees of hue per second              */
#define CUBE_FOG          0.35f    /* how much the colour fades with depth   */
#define CUBE_GLOW_GAIN    90       /* brightness of the edge halo, 0..255    */
#define CUBE_MIN_FACE_AREA 30      /* skip faces thinner than this (pixels^2)*/

/* ================================================================== */
/*  Objects, and where they come from                                 */
/*                                                                    */
/*    AMOLED         - the display object      (WaveshareAMOLED lib)  */
/*    AMOLED_Canvas  - the drawing surface     (WaveshareAMOLED lib)  */
/*    Serial         - USB/UART console        (Arduino core)         */
/*    millis()       - milliseconds since boot (Arduino core)         */
/*    cosf/sinf/fmodf- math                    (C standard library)   */
/* ================================================================== */
AMOLED amoled;

static int W, H, CX, CY;          /* screen size and centre                 */


/* ================================================================== */
/*  The cube                                                          */
/*                                                                    */
/*  Eight corners of a cube with an edge length of 2, so every        */
/*  coordinate is -1 or +1.  The six faces list their four corners in */
/*  order (so each face can be filled as a quad), and the twelve      */
/*  edges are pairs of corner indices.                                */
/* ================================================================== */
static const float kCorner[8][3] =
{
    {-1, -1, -1}, {+1, -1, -1}, {+1, +1, -1}, {-1, +1, -1},   /* z = -1 */
    {-1, -1, +1}, {+1, -1, +1}, {+1, +1, +1}, {-1, +1, +1}    /* z = +1 */
};

static const uint8_t kFace[6][4] =
{
    {0, 3, 2, 1},   /* z = -1 */
    {4, 5, 6, 7},   /* z = +1 */
    {0, 4, 7, 3},   /* x = -1 */
    {1, 2, 6, 5},   /* x = +1 */
    {0, 1, 5, 4},   /* y = -1 */
    {3, 7, 6, 2}    /* y = +1 */
};

/* outward normal of each face - it tells how much light the face catches */
static const float kFaceNormal[6][3] =
{
    {0, 0, -1}, {0, 0, 1}, {-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}
};

static const uint8_t kEdge[12][2] =
{
    {0, 1}, {1, 2}, {2, 3}, {3, 0},   /* the z = -1 face */
    {4, 5}, {5, 6}, {6, 7}, {7, 4},   /* the z = +1 face */
    {0, 4}, {1, 5}, {2, 6}, {3, 7}    /* the four uprights */
};

/* direction the "sun" comes from (normalised).  +y is "up" on screen, so
 * the top face of the cube is the bright one. */
static const float kLight[3] = {0.42f, 0.71f, 0.57f};

/* a 3D vector - C++ has none built in, so here is a small one */
struct V3
{
    float x, y, z;
};

static V3    rotated[8];      /* the corners after rotation                */
static V3    normal[6];       /* the face normals after rotation           */
static int   scrX[8];         /* projected screen coordinates              */
static int   scrY[8];
static float depth[8];        /* distance from the camera, for the fog     */

/* the region pushed last frame, so it can be erased again exactly */
static int  lastX0 = 0, lastY0 = 0, lastX1 = 0, lastY1 = 0;
static bool lastValid = false;

/* frame rate measurement */
static uint32_t fpsFrames    = 0;
static uint32_t fpsStartMs   = 0;
static uint32_t frameStartMs = 0;
static float    fps          = 0.0f;
static int      fpsShown     = -1;
static uint32_t pushedBytes  = 0;   /* bytes per frame, for the serial report */

/* ================================================================== */
/*  STEP 1 - rotate every corner                                      */
/*                                                                    */
/*  Two rotations, one after the other - that is all a rotating cube   */
/*  needs, and both are plain 2x2 rotations, no matrix library:        */
/*                                                                    */
/*    yaw   around Y :  x'  =  x * cos(a) + z * sin(a)                 */
/*                      z'  = -x * sin(a) + z * cos(a)                 */
/*    pitch around X :  y'' =  y * cos(b) - z' * sin(b)                */
/*                      z'' =  y * sin(b) + z' * cos(b)                */
/* ================================================================== */
static void rotatePoint(float yaw, float pitch, const float *v, V3 &out)
{
    const float cy = cosf(yaw),   sy = sinf(yaw);
    const float cx = cosf(pitch), sx = sinf(pitch);

    const float x1 =  v[0] * cy + v[2] * sy;
    const float z1 = -v[0] * sy + v[2] * cy;

    out.x = x1;
    out.y =  v[1] * cx - z1 * sx;
    out.z =  v[1] * sx + z1 * cx;
}

/* ------------------------------------------------------------------ */
/*  STEP 2 - project a rotated point onto the screen                   */
/*                                                                    */
/*  Plain perspective: the further away (bigger z), the smaller.  The  */
/*  camera sits at z = -CUBE_CAM_DIST looking along +z, so a point at  */
/*  the middle of the cube is CUBE_CAM_DIST units away.  Screen y      */
/*  grows downwards, hence the minus.                                  */
/* ------------------------------------------------------------------ */
static inline int iround(float v) { return (int)(v < 0.0f ? v - 0.5f : v + 0.5f); }

static void project(const V3 &v, int &sx, int &sy, float &dist)
{
    float d = v.z + CUBE_CAM_DIST;
    if (d < 0.15f) d = 0.15f;                 /* never divide by ~zero */

    const float s = CUBE_FOCAL / d;
    sx   = CX + iround(v.x * s);
    sy   = CY - iround(v.y * s);
    dist = d;
}

/* fog: 0 = close and bright, 1 = far and dim */
static float depthFactor(float d)
{
    float f = (d - (CUBE_CAM_DIST - 1.8f)) / 3.6f;
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    return 1.0f - CUBE_FOG * f;
}

/* a rainbow colour: hue in degrees, brightness 0..1  (my library's
 * amoledHSV() for the hue plus amoledScale() for the brightness) */
static uint32_t hueColour(float hueDeg, float bright)
{
    if (bright < 0.0f) bright = 0.0f;
    if (bright > 1.0f) bright = 1.0f;
    const uint32_t c = amoledHSV(fmodf(hueDeg, 360.0f), 1.0f, 1.0f);
    return amoledScale(c, (uint16_t)(bright * 255.0f), 255);
}

/* ------------------------------------------------------------------ */
/*  Fill one cube face, additively                                    */
/*                                                                    */
/*  A projected face is always a convex quad, so for every scanline of */
/*  its bounding box the four edges are tested for crossings and the   */
/*  outermost two crossings are the span to draw.  addHLine() *adds*   */
/*  light instead of replacing the pixel, so faces shine through each  */
/*  other - the AMOLED speciality.                                    */
/*                                                                    */
/*  Two safeties matter for faces that are nearly edge on ("slivers"): */
/*  the interpolated x is clamped to the quad's own bounding box, and  */
/*  slivers below CUBE_MIN_FACE_AREA are skipped.  A face that thin    */
/*  cannot be seen, but the pixels an unclamped crossing would put on  */
/*  the screen would be very visible.                                  */
/* ------------------------------------------------------------------ */
#if CUBE_FACES
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

    /* twice the signed area of the quad (shoelace formula) */
    int32_t area2 = 0;
    for (int i = 0; i < 4; i++)
    {
        const int j = (i + 1) & 3;
        area2 += (int32_t)xs[i] * (int32_t)ys[j] - (int32_t)xs[j] * (int32_t)ys[i];
    }
    if (area2 < 0) area2 = -area2;
    if (area2 < 2 * CUBE_MIN_FACE_AREA) return;

    for (int y = minY; y <= maxY; y++)
    {
        int xMin = maxX, xMax = minX;             /* "nothing found" */
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
#endif  /* CUBE_FACES */

/* ------------------------------------------------------------------ */
/*  One glowing edge                                                   */
/*                                                                    */
/*  A bright one pixel core plus a dim halo on both sides.  The halo   */
/*  is offset *across* the line - up/down for a shallow line, left/    */
/*  right for a steep one - so an edge is a thin ribbon of light and   */
/*  never a set of parallel copies of itself.                          */
/* ------------------------------------------------------------------ */
static void drawEdge(AMOLED_Canvas &cv, int x0, int y0, int x1, int y1, uint32_t col)
{
    const int  dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    const int  dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
    const int  sx = (x0 < x1) ? 1 : -1;
    const int  sy = (y0 < y1) ? 1 : -1;
    const bool shallow = (dx >= dy);
    int        err = dx - dy;

    for (;;)
    {
        cv.drawPixel(x0, y0, col);

#if CUBE_EDGE_GLOW
        if (shallow)
        {
            cv.addPixelScaled(x0, y0 - 1, col, CUBE_GLOW_GAIN);
            cv.addPixelScaled(x0, y0 + 1, col, CUBE_GLOW_GAIN);
        }
        else
        {
            cv.addPixelScaled(x0 - 1, y0, col, CUBE_GLOW_GAIN);
            cv.addPixelScaled(x0 + 1, y0, col, CUBE_GLOW_GAIN);
        }
#endif
        if (x0 == x1 && y0 == y1) break;

        const int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

/* ------------------------------------------------------------------ */
/*  The on screen frame rate counter (drawn where the cube never goes) */
/* ------------------------------------------------------------------ */
static int  fpsTextX0 = 0, fpsTextY0 = 0, fpsTextX1 = 0, fpsTextY1 = 0;
static bool fpsTextValid = false;

static void drawFpsText(AMOLED_Canvas &cv, float value)
{
    char buf[24];
    snprintf(buf, sizeof(buf), "%3d FPS", (int)(value + 0.5f));   /* fixed width,
                                                                  * so the text
                                                                  * never shifts */

    const int w = AMOLED_Canvas::textWidth(buf, 2);
    const int x = CX - w / 2;
    const int y = 26;

    /* erase whatever was there before (the string length and therefore the
     * position of the first character can change) */
    if (fpsTextValid)
    {
        cv.fillRect(fpsTextX0, fpsTextY0, fpsTextX1, fpsTextY1, AMOLED_BLACK);
    }

    cv.drawText(x, y, buf, AMOLED_WHITE, 2);

    fpsTextX0 = x - 4;          fpsTextY0 = y - 4;
    fpsTextX1 = x + w + 4;      fpsTextY1 = y + 20;
    fpsTextValid = true;
}


/* ================================================================== */
/*  setup() - runs once                                               */
/* ================================================================== */
void setup()
{
    /* Serial belongs to the Arduino core.  With "USB CDC On Boot" it is the
     * native USB port: if no host has it open, a write would block for up to
     * two seconds - which would ruin the frame rate - so the timeout is set
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

    if (!amoled.beginFramebuffer())           /* 651 kB of PSRAM (24 bpp) */
    {
        if (Serial) Serial.println("no PSRAM framebuffer - Tools -> PSRAM -> OPI PSRAM");
        while (true) delay(1000);
    }

    /* ---- the static parts of the picture, drawn once --------------- */
    AMOLED_Canvas &fb = amoled.canvas();
    fb.clearScreen();

#if !CUBE_DIRTY_RECT
    fb.drawTextCentered(CX, H - 44, "FULL FRAME", AMOLED_DARKGREY, 1);
#elif CUBE_FULL_WIDTH
    fb.drawTextCentered(CX, H - 44, "FULL WIDTH BANDS", AMOLED_DARKGREY, 1);
#else
    fb.drawTextCentered(CX, H - 44, "DIRTY BOX", AMOLED_DARKGREY, 1);
#endif

#if CUBE_FPS_CAP
    char cap[24];
    snprintf(cap, sizeof(cap), "FPS CAP %d", (int)CUBE_FPS_CAP);
    fb.drawTextCentered(CX, H - 26, cap, AMOLED_DARKGREY, 1);
#else
    fb.drawTextCentered(CX, H - 26, "FPS UNCAPPED", AMOLED_DARKGREY, 1);
#endif

    fpsShown = -1;
    drawFpsText(fb, 0.0f);
    amoled.push();                            /* whole frame, once */

    lastValid = false;
    fpsStartMs = frameStartMs = millis();

    if (Serial)
    {
        Serial.printf("\n=== rotating rainbow cube ===\n");
        Serial.printf("%s  %dx%d  %d bpp  rotation %d  QSPI %d MHz\n",
                      amoled.controllerName(), W, H, amoled.colorDepth(),
                      amoled.getRotation(), (int)(AMOLED_QSPI_CLOCK_HZ / 1000000UL));
        Serial.printf("push %s, faces %d, edge glow %d, fps cap %d\n",
                      CUBE_DIRTY_RECT ? (CUBE_FULL_WIDTH ? "full width bands" : "dirty box")
                                      : "full frame",
                      (int)CUBE_FACES, (int)CUBE_EDGE_GLOW, (int)CUBE_FPS_CAP);
    }
}

/* ================================================================== */
/*  loop() - one frame per call, as fast as the panel allows          */
/* ================================================================== */
void loop()
{
    /* ---- 0. keep the update rate at or below the panel's rate ------ */
#if CUBE_FPS_CAP
    while ((uint32_t)(millis() - frameStartMs) < (1000u / CUBE_FPS_CAP)) delay(1);
#endif
    frameStartMs = millis();
    const float t = frameStartMs * 0.001f;     /* seconds since boot */

    /* ---- 1. rotate the cube: 8 corners and 6 face normals ---------- */
    const float yaw   = t * CUBE_SPIN_Y;
    const float pitch = t * CUBE_SPIN_X;
    for (int i = 0; i < 8; i++) rotatePoint(yaw, pitch, kCorner[i], rotated[i]);
    for (int f = 0; f < 6; f++) rotatePoint(yaw, pitch, kFaceNormal[f], normal[f]);

    /* ---- 2. project them and find the bounding box ---------------- */
    int minX = W, minY = H, maxX = -1, maxY = -1;
    for (int i = 0; i < 8; i++)
    {
        project(rotated[i], scrX[i], scrY[i], depth[i]);
        if (scrX[i] < minX) minX = scrX[i];
        if (scrX[i] > maxX) maxX = scrX[i];
        if (scrY[i] < minY) minY = scrY[i];
        if (scrY[i] > maxY) maxY = scrY[i];
    }

    /* a margin for the glowing edges and corners */
    const int M = 18;
    int boxX0 = minX - M, boxY0 = minY - M, boxX1 = maxX + M, boxY1 = maxY + M;
    if (boxX0 < 0)     boxX0 = 0;
    if (boxY0 < 0)     boxY0 = 0;
    if (boxX1 > W - 1) boxX1 = W - 1;
    if (boxY1 > H - 1) boxY1 = H - 1;

    /* the region this frame will touch: the new box plus last frame's, so
     * every pixel the cube has been drawn on is cleared again */
    int ux0 = boxX0, uy0 = boxY0, ux1 = boxX1, uy1 = boxY1;
    if (lastValid)
    {
        if (lastX0 < ux0) ux0 = lastX0;
        if (lastY0 < uy0) uy0 = lastY0;
        if (lastX1 > ux1) ux1 = lastX1;
        if (lastY1 > uy1) uy1 = lastY1;
    }

    /* ---- 3. erase it, and stop anything from drawing outside it ---- */
    AMOLED_Canvas &fb = amoled.canvas();
    fb.setClip(ux0, uy0, ux1, uy1);
    fb.fillRect(ux0, uy0, ux1, uy1, AMOLED_BLACK);        /* true black */

    /* ---- 4. draw the cube ---------------------------------------- */
    const float hueBase = fmodf(t * CUBE_HUE_SPEED, 360.0f);   /* the rainbow */

#if CUBE_FACES
    for (int f = 0; f < 6; f++)
    {
        /* the camera looks along +z, so only faces whose normal points
         * towards it (z < 0) are visible - the classic back face cull */
        if (normal[f].z >= 0.0f) continue;

        const uint8_t *q  = kFace[f];
        const int xs[4] = { scrX[q[0]], scrX[q[1]], scrX[q[2]], scrX[q[3]] };
        const int ys[4] = { scrY[q[0]], scrY[q[1]], scrY[q[2]], scrY[q[3]] };

        /* lambert-ish shading: how squarely the face points at the sun */
        float lit = normal[f].x * kLight[0] + normal[f].y * kLight[1] + normal[f].z * kLight[2];
        if (lit < 0.0f) lit = 0.0f;

        const float d = 0.25f * (depth[q[0]] + depth[q[1]] + depth[q[2]] + depth[q[3]]);
        fillQuad(fb, xs, ys,
                 hueColour(hueBase + f * 20.0f, (0.22f + 0.55f * lit) * depthFactor(d)));
    }
#endif

    /* the twelve edges: neon lines, a step along the rainbow per edge */
    for (int e = 0; e < 12; e++)
    {
        const int   a = kEdge[e][0], b = kEdge[e][1];
        const float d = 0.5f * (depth[a] + depth[b]);
        drawEdge(fb, scrX[a], scrY[a], scrX[b], scrY[b],
                 hueColour(hueBase + e * 13.0f + 12.0f, 0.45f + 0.55f * depthFactor(d)));
    }

    /* the eight corners glow like neon junctions */
    for (int i = 0; i < 8; i++)
    {
        fb.addGlow(scrX[i], scrY[i], 13,
                   hueColour(hueBase + i * 17.0f, 0.85f * depthFactor(depth[i])), 200);
    }

    fb.resetClip();                 /* back to the whole framebuffer */

    /* ---- 5. send the changed region to the panel ------------------ */
#if !CUBE_DIRTY_RECT
    amoled.push();                            /* whole frame: the reference */
    pushedBytes = (uint32_t)W * H * (uint32_t)amoled.bytesPerPixel();
#elif CUBE_FULL_WIDTH
    /* Full width bands that line up with the strip grid: every transfer is
     * then exactly the shape the streaming mode uses (466 x 32 pixels), and
     * packing one is a single contiguous memcpy. */
    int py0 = (uy0 / AMOLED_STRIP_LINES) * AMOLED_STRIP_LINES;
    int py1 = ((uy1 / AMOLED_STRIP_LINES) + 1) * AMOLED_STRIP_LINES - 1;
    if (py1 > H - 1) py1 = H - 1;
    amoled.pushRect(0, py0, W - 1, py1);
    pushedBytes = (uint32_t)W * (uint32_t)(py1 - py0 + 1) * (uint32_t)amoled.bytesPerPixel();
#else
    amoled.pushRect(ux0, uy0, ux1, uy1);      /* only what changed: least data */
    pushedBytes = (uint32_t)(ux1 - ux0 + 1) * (uint32_t)(uy1 - uy0 + 1) *
                  (uint32_t)amoled.bytesPerPixel();
#endif

    lastX0 = boxX0;  lastY0 = boxY0;
    lastX1 = boxX1;  lastY1 = boxY1;
    lastValid = true;

    /* ---- 6. measure and report the frame rate --------------------- */
    fpsFrames++;
    const uint32_t now2 = millis();
    if ((now2 - fpsStartMs) >= 1000)          /* report once per second */
    {
        fps = fpsFrames * 1000.0f / (float)(now2 - fpsStartMs);
        fpsFrames  = 0;
        fpsStartMs = now2;

#if CUBE_SHOW_FPS
        const int shown = (int)(fps + 0.5f);
        if (shown != fpsShown)                /* only redraw when it changes */
        {
            fpsShown = shown;
            const int ox0 = fpsTextX0, oy0 = fpsTextY0;      /* old text box */
            const int ox1 = fpsTextX1, oy1 = fpsTextY1;
            drawFpsText(fb, fps);
            /* push the union of the old and the new text box */
            amoled.pushRect(ox0 < fpsTextX0 ? ox0 : fpsTextX0,
                            oy0 < fpsTextY0 ? oy0 : fpsTextY0,
                            ox1 > fpsTextX1 ? ox1 : fpsTextX1,
                            oy1 > fpsTextY1 ? oy1 : fpsTextY1);
        }
#endif

#if CUBE_SERIAL_FPS
        if (Serial)                           /* never block on the USB CDC */
        {
            Serial.printf("fps %6.1f | frame %5.2f ms | pushed %4u kB | %s%s\n",
                          fps,
                          1000.0f / (fps > 0.01f ? fps : 0.01f),
                          (unsigned)(pushedBytes / 1024),
#if !CUBE_DIRTY_RECT
                          "full frame",
#elif CUBE_FULL_WIDTH
                          "full width bands",
#else
                          "dirty box",
#endif
                          CUBE_FPS_CAP ? " (capped)" : "");
        }
#endif
    }
}

/* ================================================================== */
/*  NEXT STEP - let the on board IMU hold the cube up                 */
/*                                                                    */
/*  The plan (not enabled yet): the board carries a QMI8658 6 axis IMU */
/*  on the same I2C bus the touch controller uses, so it costs no      */
/*  extra pins.  Read the accelerometer while the cube is at rest and  */
/*  it reports the gravity vector g (in the board's own frame).        */
/*                                                                    */
/*  1. read accel:  Wire.beginTransmission(0x6B); ... reg 0x35 (AX_L)  */
/*                 6 bytes, int16 each, sensitivity from CTRL2 (0x03)  */
/*  2. normalise g, and low pass it (the cube should tilt smoothly,    */
/*     not shake):  g = g * 0.85 + gNew * 0.15                         */
/*  3. build the rotation that takes g onto (0, +1, 0) - the "up" of   */
/*     the cube's own space.  With v = (0,1,0) x g, c = (0,1,0) . g    */
/*     and s2 = v . v, Rodrigues gives the whole matrix in one line:   */
/*         R = I + [v]x + [v]x^2 * (1 - c) / s2                        */
/*  4. apply R *after* the yaw/pitch rotation, so the cube spins in    */
/*     board space but always keeps one corner pointing at the sky.    */
/*                                                                    */
/*  That is a handful of lines in rotatePoint() plus a Wire read -     */
/*  worth doing once the rendering is confirmed clean on the panel.    */
/* ================================================================== */
