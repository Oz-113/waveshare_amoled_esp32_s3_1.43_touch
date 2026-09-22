/*
 * 08_IMU_Cube - WaveshareAMOLED library
 * ============================================================================
 *  A neon wireframe cube that hangs from the real world's gravity.  The on
 *  board QMI8658 accelerometer tells the sketch where "down" is, the cube is
 *  rotated so its top face always points at the sky, and it keeps spinning
 *  around the *world* vertical - so the cube looks like a solid object standing
 *  still in the room while the screen turns around it.
 *
 *  Shows: framebuffer mode + a dirty rectangle + setClip()/pushRect(), and how
 *  to read the on board IMU over I2C (the same bus the touch panel uses).
 *
 *  Touch it:
 *      tap                 freeze / unfreeze the spin
 *      hold for 0.7 s      flip through the four sensor -> display axis
 *                          candidates (the one that makes the cube sit level is
 *                          the right one - see below)
 *
 *  THE ONE THING THAT MAY NEED YOUR HELP
 *  -------------------------------------
 *  How the accelerometer's axes are placed inside the case is not something a
 *  driver can know, and Waveshare's own IMU example does not say either, so this
 *  sketch starts with a documented guess and lets you cycle the four sensible
 *  signs with a long press.  Two checks tell you instantly whether the guess is
 *  right (the mapped gravity vector is printed on the panel and on the serial
 *  port).  Remember that an accelerometer at rest measures the *upward* reaction
 *  to gravity, so the readout points at the sky:
 *
 *      display flat on the table, screen up  ->  g should be (0, 0, -1)
 *                                                and the cube lies flat
 *      display held upright in front of you  ->  g should be (0, -1, 0)
 *                                                and the cube is still level
 *
 *  If it is not, hold the screen until the readout reads SIGN 1, 2, 3 or 0 and
 *  pick the one that makes both checks true.  Put that number into
 *  IMU_SIGN_VARIANT below so it survives a reset.
 *
 *  Without an IMU (or with CUBE_IMU_ENABLE 0) the sketch falls back to the
 *  automatic two axis spin of 06_Rotating_Cube.
 *
 *  IF AN EDGE LOOKS CUT OR DOUBLED
 *  -------------------------------
 *  That is the panel's tear seam, not the drawing: the display is written while
 *  it scans out and this board has no tear-effect pin wired.  CUBE_FPS_CAP keeps
 *  the update rate at or below the panel's refresh rate, so only one seam can be
 *  on screen at a time.  Full explanation: GUIDE.md, Part A6.
 *
 *  THIS SKETCH NEEDS PSRAM (the 651 kB framebuffer): Tools -> PSRAM -> OPI PSRAM.
 * ============================================================================
 */
#include <WaveshareAMOLED.h>
#include <Wire.h>

/* ================================================================== */
/*  Display knobs                                                     */
/* ================================================================== */
#define CUBE_DIRTY_RECT   1        /* 0 = push the whole frame every time    */
#define CUBE_FULL_WIDTH   1        /* 1 = push full width bands (the shape   */
                                   /*     streaming mode uses); 0 = tight box*/
#define CUBE_FPS_CAP      60       /* updates per second, 0 = as fast as it  */
                                   /* gets (more tearing, bigger number)     */
#define CUBE_FACES        0        /* 0 = wireframe (the good looking one)   */
                                   /* 1 = translucent additive faces too     */
#define CUBE_EDGE_GLOW    1        /* 1 = soft halo around every edge        */
#define CUBE_SHOW_FPS     1        /* frame rate on the display              */
#define CUBE_SERIAL_FPS   1        /* frame rate on the serial port          */
#define CUBE_SHOW_IMU     1        /* mapped gravity + sign variant readout  */

/* ================================================================== */
/*  Cube / animation knobs                                            */
/* ================================================================== */
#define CUBE_CAM_DIST     3.6f     /* camera distance, in cube units         */
#define CUBE_FOCAL        270.0f   /* "lens": pixels per unit at distance 1  */
#define CUBE_SPIN_Y       0.90f    /* spin, radians per second. With the IMU */
                                   /* this is about the world vertical;      */
                                   /* without it, the yaw                     */
#define CUBE_SPIN_X       0.45f    /* pitch speed - only used when there is  */
                                   /* no IMU (with one, the tilt comes from  */
                                   /* gravity itself)                        */
#define CUBE_HUE_SPEED    55.0f    /* degrees of hue per second              */
#define CUBE_FOG          0.35f    /* how much the colour fades with depth    */
#define CUBE_GLOW_GAIN    90       /* brightness of the edge halo, 0..255    */
#define CUBE_MIN_FACE_AREA 30      /* skip faces thinner than this (pixels^2)*/
#define CUBE_SPIN_START   1        /* 1 = start spinning, 0 = start frozen   */

/* ================================================================== */
/*  IMU knobs (QMI8658, same I2C bus as the touch panel)              */
/* ================================================================== */
#define CUBE_IMU_ENABLE   1        /* 0 = no IMU, use the two axis spin      */
#define IMU_SIGN_VARIANT  1        /* start value, see kImuSigns[] below.    */
                                   /* Long press cycles it at run time.      */
#define IMU_FILTER        0.15f    /* weight of each new sample, 0..1        */
#define IMU_SHAKE_G       0.25f    /* ignore samples whose magnitude is off  */
                                   /* 1 g by more than this (being moved)    */
#define IMU_TEXT_MS       250      /* how often the readout text is redrawn  */

/* ================================================================== */
/*  Objects, and where they come from                                 */
/*                                                                    */
/*    AMOLED         - the display object   (WaveshareAMOLED lib)     */
/*    AMOLED_Canvas  - the drawing surface  (WaveshareAMOLED lib)     */
/*    Wire           - the I2C bus          (Arduino core)            */
/*    Serial         - USB/UART console     (Arduino core)            */
/*    millis()       - milliseconds         (Arduino core)            */
/*    cosf/sinf/sqrtf- math                 (C standard library)      */
/* ================================================================== */
AMOLED amoled;

static int W, H, CX, CY;           /* screen size and centre                 */

/* ================================================================== */
/*  The cube                                                          */
/*                                                                    */
/*  Eight corners of a cube with an edge length of 2, so every        */
/*  coordinate is -1 or +1.  The six faces list their four corners in */
/*  order (so a face can be filled as a quad), and the twelve edges   */
/*  are pairs of corner indices.                                      */
/*                                                                    */
/*  Cube space -> screen: +x is right, +y is up (the top face), +z is */
/*  into the screen; the camera sits at z = -CUBE_CAM_DIST looking    */
/*  along +z.  With the IMU the whole cube is rotated by a matrix     */
/*  built from the measured gravity, so +y ends up pointing at the    */
/*  sky.                                                              */
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
    {0, 0, -1}, {0, 0, +1}, {-1, 0, 0}, {+1, 0, 0}, {0, -1, 0}, {0, +1, 0}
};

static const uint8_t kEdge[12][2] =
{
    {0, 1}, {1, 2}, {2, 3}, {3, 0},   /* the z = -1 face */
    {4, 5}, {5, 6}, {6, 7}, {7, 4},   /* the z = +1 face */
    {0, 4}, {1, 5}, {2, 6}, {3, 7}    /* the four uprights */
};

/* the "sun", in world space: with the IMU it stays fixed in the room while you
 * turn the display, which is what makes the cube look like a real object */
static const float kLight[3] = {0.42f, 0.71f, 0.57f};

/* ================================================================== */
/*  Small 3D helpers                                                  */
/*                                                                    */
/*  A cube needs a vector, a 3x3 matrix and nothing else - so here    */
/*  they are, instead of pulling in a maths library.                  */
/* ================================================================== */
struct V3
{
    float x, y, z;
};

/* a 3x3 matrix, stored as its three rows: row i multiplied by v gives
 * component i of the result */
struct M3
{
    V3 rx, ry, rz;
};

/* A rectangle of text that remembers where it was drawn, so that a readout can
 * be repainted (and re-sent) on its own.  Declared up here because the Arduino
 * build inserts its generated function prototypes at the top of the sketch, and
 * those prototypes have to see the type. */
struct TextBox
{
    int  x0, y0, x1, y1;
    bool valid;
};

static inline float dot(const V3 &a, const V3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

static inline V3 cross(const V3 &a, const V3 &b)
{
    V3 r;
    r.x = a.y * b.z - a.z * b.y;
    r.y = a.z * b.x - a.x * b.z;
    r.z = a.x * b.y - a.y * b.x;
    return r;
}

static inline void normalize(V3 &v)
{
    const float m = sqrtf(dot(v, v));
    if (m > 1e-6f) { v.x /= m; v.y /= m; v.z /= m; }
}

static inline V3 apply(const M3 &m, const V3 &v)
{
    V3 r;
    r.x = dot(m.rx, v);
    r.y = dot(m.ry, v);
    r.z = dot(m.rz, v);
    return r;
}

static M3 mul(const M3 &a, const M3 &b)
{
    /* both matrices are orthonormal, so row * column is just a dot product */
    M3 r;
    const V3 bx = {b.rx.x, b.ry.x, b.rz.x};   /* the columns of b */
    const V3 by = {b.rx.y, b.ry.y, b.rz.y};
    const V3 bz = {b.rx.z, b.ry.z, b.rz.z};
    r.rx = {dot(a.rx, bx), dot(a.rx, by), dot(a.rx, bz)};
    r.ry = {dot(a.ry, bx), dot(a.ry, by), dot(a.ry, bz)};
    r.rz = {dot(a.rz, bx), dot(a.rz, by), dot(a.rz, bz)};
    return r;
}

static M3 identity(void)
{
    M3 m;
    m.rx = {1, 0, 0};
    m.ry = {0, 1, 0};
    m.rz = {0, 0, 1};
    return m;
}

/* the spin of the animation.  With an IMU this is a rotation about the world
 * vertical; without one it is the classic yaw + pitch. */
static M3 spinMatrix(float yaw, float pitch, bool imuOk)
{
    const float cy = cosf(yaw), sy = sinf(yaw);

    if (imuOk)
    {
        /* rotation about +y (in world coordinates) */
        M3 m;
        m.rx = { cy, 0, sy};
        m.ry = {  0, 1,  0};
        m.rz = {-sy, 0, cy};
        return m;
    }

    /* R = Rx(pitch) * Ry(yaw) - exactly the maths of 06_Rotating_Cube */
    const float cx = cosf(pitch), sx = sinf(pitch);
    M3 m;
    m.rx = {        cy,   0,        sy};
    m.ry = {    sx * sy,  cx, -sx * cy};
    m.rz = { -cx * sy,   sx,  cx * cy};
    return m;
}

/* The frame the cube lives in: +y is the measured world vertical, and the
 * display's forward axis (+z) is used as the horizontal reference, so a
 * display held upright gives the identity - the cube looks exactly like in
 * 06_Rotating_Cube until you tilt it. */
static M3 worldMatrix(const V3 &up)
{
    V3 wy = up;
    normalize(wy);

    V3 ref = {0, 0, 1};                       /* the display's forward axis */
    if (fabsf(wy.z) > 0.9f)                    /* flat on the table: any */
    {                                          /* horizontal will do     */
        ref.x = 0; ref.y = 1; ref.z = 0;
    }

    V3 wx = cross(wy, ref);
    if (dot(wx, wx) < 1e-6f)                   /* parallel after all: try the */
    {                                          /* other reference             */
        ref.x = 0; ref.y = 1; ref.z = 0;
        wx = cross(wy, ref);
    }
    normalize(wx);
    const V3 wz = cross(wx, wy);

    /* Q = [wx wy wz] as columns: it maps world coordinates to screen ones */
    M3 q;
    q.rx = {wx.x, wy.x, wz.x};
    q.ry = {wx.y, wy.y, wz.y};
    q.rz = {wx.z, wy.z, wz.z};
    return q;
}

/* ================================================================== */
/*  Per frame state                                                   */
/* ================================================================== */
static V3    rotated[8];      /* the corners after the rotation              */
static V3    normal[6];       /* the face normals after the rotation         */
static V3    lightView;       /* the sun, expressed in screen coordinates    */
static int   scrX[8];         /* projected screen coordinates                */
static int   scrY[8];
static float depth[8];        /* distance from the camera, for the fog        */

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
static bool     spinOn       = CUBE_SPIN_START ? true : false;

/* ================================================================== */
/*  The on board IMU: a QMI8658 on the touch panel's I2C bus          */
/*                                                                    */
/*  Only the accelerometer is used.  Its registers (from the QMI8658  */
/*  datasheet, and from Waveshare's own 03_I2C_QMI8658 example):      */
/*                                                                    */
/*      0x00  WHO_AM_I   must read 0x05                              */
/*      0x02  CTRL1      bit6 = address auto increment - needed so    */
/*                        one burst read gives all six bytes         */
/*      0x03  CTRL2      bits 6:4 = accel range, 3:0 = output rate    */
/*      0x08  CTRL7      bit0 = accelerometer enable                  */
/*      0x2E  STATUS0    bit0 = a fresh accel sample is ready         */
/*      0x35  AX_L      ax, ay, az as little endian int16, 4096/g at  */
/*                        the 8 g range                              */
/*                                                                    */
/*  The chip answers at 0x6B or 0x6A depending on its SA0 pin, so we  */
/*  try both.                                                         */
/*                                                                    */
/*  What an accelerometer measures at rest is the *upward* reaction to */
/*  gravity, so the value in g points at the sky.  Once it is mapped  */
/*  onto the display's axes it is the world vertical, which is exactly */
/*  what the cube needs.                                              */
/* ================================================================== */
#define IMU_ADDR_H          0x6B
#define IMU_ADDR_L          0x6A
#define IMU_REG_WHO_AM_I    0x00
#define IMU_REG_CTRL1       0x02
#define IMU_REG_CTRL2       0x03
#define IMU_REG_CTRL7       0x08
#define IMU_REG_STATUS0     0x2E
#define IMU_REG_AX_L        0x35
#define IMU_WHO_AM_I_VALUE  0x05
#define IMU_CTRL2_8G_250HZ  0x25       /* 8 g (bits 6:4 = 2) | 250 Hz (5)    */
#define IMU_LSB_PER_G       4096.0f    /* 1 << 12, the sensitivity at 8 g    */

/*  Which sensor axis feeds which display axis.  On these boards the sensor is
 *  almost certainly mounted square with the panel (same x, y, z order), so only
 *  the signs are in doubt - and there are four sensible combinations.  A long
 *  press cycles through them, and the readout shows which one is active.
 *
 *      +x  display right           +y  display down (screen y grows down)
 *      +z  into the screen
 *
 *  The variant is applied as display = (sx * gx, sy * gy, sz * gz).  The
 *  product of the three signs is +1, so the mapping stays a proper rotation.
 */
static const int8_t kImuSigns[4][3] =
{
    { +1, +1, +1 },
    { +1, -1, -1 },   /* IMU_SIGN_VARIANT 1: the default guess */
    { -1, +1, -1 },
    { -1, -1, +1 }
};

static uint8_t imuAddr    = 0;
static bool    imuOk      = false;
static int     imuSignIdx = IMU_SIGN_VARIANT & 3;

/* world "up" in display coordinates: x right, y down, z into the screen */
static V3   gravityDisp = {0, 1, 0};
static bool gravityOk   = false;

static bool imuRead(uint8_t reg, uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(imuAddr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;      /* no stop yet */
    if (Wire.requestFrom((int)imuAddr, (int)len) != (int)len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)Wire.read();
    return true;
}

static bool imuWrite(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(imuAddr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool imuBegin(void)
{
    const uint8_t candidates[2] = {IMU_ADDR_H, IMU_ADDR_L};
    uint8_t id = 0;

    for (int i = 0; i < 2; i++)
    {
        imuAddr = candidates[i];
        id = 0;
        if (imuRead(IMU_REG_WHO_AM_I, &id, 1) && id == IMU_WHO_AM_I_VALUE) break;
    }
    if (id != IMU_WHO_AM_I_VALUE)
    {
        if (Serial) Serial.printf("no IMU answer at 0x%02X / 0x%02X\n", IMU_ADDR_H, IMU_ADDR_L);
        return false;
    }

    imuWrite(IMU_REG_CTRL7, 0x00);            /* configure with sensors off */
    imuWrite(IMU_REG_CTRL1, 0x60);            /* address auto increment     */
    imuWrite(IMU_REG_CTRL2, IMU_CTRL2_8G_250HZ);
    imuWrite(IMU_REG_CTRL7, 0x01);            /* accelerometer on           */
    delay(30);                                /* let a few samples settle   */
    return true;
}

/* one fresh sample, still in the sensor's own axes, in g */
static bool imuReadRaw(float g[3])
{
    uint8_t status = 0;
    uint8_t b[6];

    if (!imuRead(IMU_REG_STATUS0, &status, 1)) return false;
    if (!(status & 0x01)) return false;                  /* nothing new yet */
    if (!imuRead(IMU_REG_AX_L, b, 6)) return false;

    g[0] = (float)(int16_t)((uint16_t)b[1] << 8 | b[0]) / IMU_LSB_PER_G;
    g[1] = (float)(int16_t)((uint16_t)b[3] << 8 | b[2]) / IMU_LSB_PER_G;
    g[2] = (float)(int16_t)((uint16_t)b[5] << 8 | b[4]) / IMU_LSB_PER_G;
    return true;
}

/* Map the sample onto the display axes and low pass it: the cube should follow
 * the device smoothly, not shake with every hand movement.  A sample that is
 * not close to 1 g means the device is being moved, so it is ignored. */
static void imuUpdate(void)
{
    float raw[3];

    if (!imuOk || !imuReadRaw(raw)) return;

    const int8_t *s = kImuSigns[imuSignIdx];
    V3 d = { s[0] * raw[0], s[1] * raw[1], s[2] * raw[2] };

    const float m = sqrtf(dot(d, d));
    if (m < 1.0f - IMU_SHAKE_G || m > 1.0f + IMU_SHAKE_G) return;
    normalize(d);

    if (!gravityOk)                          /* first sample (or a new sign  */
    {                                        /* variant): jump, do not ramp  */
        gravityDisp = d;
        gravityOk   = true;
        return;
    }

    gravityDisp.x += (d.x - gravityDisp.x) * IMU_FILTER;
    gravityDisp.y += (d.y - gravityDisp.y) * IMU_FILTER;
    gravityDisp.z += (d.z - gravityDisp.z) * IMU_FILTER;
    normalize(gravityDisp);                  /* the filter shortens the vector */
}

/* ================================================================== */
/*  Projection                                                        */
/*                                                                    */
/*  Plain perspective: the further away (bigger z), the smaller.  The  */
/*  camera sits at z = -CUBE_CAM_DIST looking along +z, so a point in  */
/*  the middle of the cube is CUBE_CAM_DIST away.  Screen y grows      */
/*  downwards, hence the minus.                                        */
/* ================================================================== */
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

/* a rainbow colour: hue in degrees, brightness 0..1 */
static uint32_t hueColour(float hueDeg, float bright)
{
    if (bright < 0.0f) bright = 0.0f;
    if (bright > 1.0f) bright = 1.0f;
    const uint32_t c = amoledHSV(fmodf(hueDeg, 360.0f), 1.0f, 1.0f);
    return amoledScale(c, (uint16_t)(bright * 255.0f), 255);
}

#if CUBE_FACES
/* ------------------------------------------------------------------ */
/*  Fill one cube face, additively (only when CUBE_FACES is 1)         */
/*                                                                    */
/*  A projected face is always a convex quad, so for every scanline of */
/*  its bounding box the four edges are tested for crossings and the   */
/*  outermost two are the span to draw.  The interpolated x is clamped */
/*  to the quad's own bounding box and quads thinner than               */
/*  CUBE_MIN_FACE_AREA are skipped: an edge-on face cannot be seen,    */
/*  but the pixels a degenerate crossing would place could be.         */
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
/*  The two text readouts                                             */
/*                                                                    */
/*  Both sit outside the cube's dirty rectangle, so they are pushed    */
/*  separately - and only when they change, because a push costs the   */
/*  same whether it carries one line or a whole band.                  */
/*                                                                    */
/*  (TextBox itself is declared up with the other helper types, because */
/*  the Arduino build inserts generated prototypes at the top of the    */
/*  sketch and they have to see the type.)                             */
/* ------------------------------------------------------------------ */
static TextBox fpsBox = {0, 0, 0, 0, false};
static TextBox imuBox = {0, 0, 0, 0, false};

/* push the union of the old and the new box (the string can change width) */
static void pushTextBox(const TextBox &oldBox, const TextBox &newBox)
{
    if (!newBox.valid) return;

    int x0 = newBox.x0, y0 = newBox.y0, x1 = newBox.x1, y1 = newBox.y1;
    if (oldBox.valid)
    {
        if (oldBox.x0 < x0) x0 = oldBox.x0;
        if (oldBox.y0 < y0) y0 = oldBox.y0;
        if (oldBox.x1 > x1) x1 = oldBox.x1;
        if (oldBox.y1 > y1) y1 = oldBox.y1;
    }
    amoled.pushRect(x0, y0, x1, y1);
}

static void drawFpsText(AMOLED_Canvas &cv, float value, TextBox &box)
{
    char buf[24];
    snprintf(buf, sizeof(buf), "%3d FPS", (int)(value + 0.5f));   /* fixed width,
                                                                  * so it never
                                                                  * shifts */
    const int w = AMOLED_Canvas::textWidth(buf, 2);
    const int x = CX - w / 2;
    const int y = 26;

    if (box.valid) cv.fillRect(box.x0, box.y0, box.x1, box.y1, AMOLED_BLACK);
    cv.drawText(x, y, buf, AMOLED_WHITE, 2);

    box.x0 = x - 4;          box.y0 = y - 4;
    box.x1 = x + w + 4;      box.y1 = y + 20;
    box.valid = true;
}

/* The IMU readout: the measured gravity in display coordinates, which axis
 * signs are in use, and whether the spin is frozen.  This is what you look at
 * to decide whether IMU_SIGN_VARIANT is right - see the header. */
static void drawImuText(AMOLED_Canvas &cv, TextBox &box)
{
#if CUBE_SHOW_IMU
    char buf[72];
    const int8_t *s = kImuSigns[imuSignIdx];

    if (imuOk && gravityOk)
    {
        snprintf(buf, sizeof(buf), "G %+.2f %+.2f %+.2f   SIGN %d  %c %c %c   %s",
                 gravityDisp.x, gravityDisp.y, gravityDisp.z, imuSignIdx,
                 s[0] > 0 ? '+' : '-', s[1] > 0 ? '+' : '-', s[2] > 0 ? '+' : '-',
                 spinOn ? "SPIN" : "HOLD");
    }
    else
    {
        snprintf(buf, sizeof(buf), "NO IMU - two axis spin   %s", spinOn ? "SPIN" : "HOLD");
    }

    const int w = AMOLED_Canvas::textWidth(buf, 1);
    const int x = CX - w / 2;
    const int y = H - 18;

    if (box.valid) cv.fillRect(box.x0, box.y0, box.x1, box.y1, AMOLED_BLACK);
    cv.drawText(x, y, buf, AMOLED_DARKGREY, 1);

    box.x0 = x - 3;          box.y0 = y - 3;
    box.x1 = x + w + 3;      box.y1 = y + 9;
    box.valid = true;
#else
    (void)cv;
    (void)box;
#endif
}

/* ================================================================== */
/*  setup() - runs once                                               */
/* ================================================================== */
void setup()
{
    /* With "USB CDC On Boot" Serial is the native USB port: if no host has it
     * open a write would block for up to two seconds, so the timeout is set to
     * zero and every print is guarded with "if (Serial)". */
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

    /* ---- the IMU --------------------------------------------------- */
#if CUBE_IMU_ENABLE
    /* amoled.begin() has already started Wire on the touch pins (SDA 47,
     * SCL 48) and the IMU is on that same bus, so there is nothing to set up
     * beyond the chip's own registers. */
    imuOk = imuBegin();
    if (imuOk)
    {
        for (int i = 0; i < 30 && !gravityOk; i++)    /* first samples */
        {
            imuUpdate();
            delay(5);
        }
    }
#endif
    if (!imuOk && Serial) Serial.println("no IMU: falling back to the two axis spin");

    /* ---- the static parts of the picture, drawn once --------------- */
    AMOLED_Canvas &fb = amoled.canvas();
    fb.clearScreen();
    fb.drawTextCentered(CX, H - 44, imuOk ? "IMU CUBE - WIREFRAME" : "CUBE - NO IMU",
                        AMOLED_DARKGREY, 1);
    fpsShown = -1;
    drawFpsText(fb, 0.0f, fpsBox);
    drawImuText(fb, imuBox);
    amoled.push();                            /* whole frame, once */

    lastValid = false;
    fpsStartMs = frameStartMs = millis();

    if (Serial)
    {
        Serial.printf("\n=== IMU cube ===\n");
        Serial.printf("%s  %dx%d  %d bpp  QSPI %d MHz\n",
                      amoled.controllerName(), W, H, amoled.colorDepth(),
                      (int)(AMOLED_QSPI_CLOCK_HZ / 1000000UL));
        if (imuOk) Serial.printf("QMI8658 at 0x%02X, 8 g / 250 Hz\n", imuAddr);
        Serial.printf("push %s, faces %d, fps cap %d, sign variant %d (%c %c %c)\n",
                      CUBE_DIRTY_RECT ? (CUBE_FULL_WIDTH ? "full width bands" : "dirty box")
                                      : "full frame",
                      (int)CUBE_FACES, (int)CUBE_FPS_CAP, imuSignIdx,
                      kImuSigns[imuSignIdx][0] > 0 ? '+' : '-',
                      kImuSigns[imuSignIdx][1] > 0 ? '+' : '-',
                      kImuSigns[imuSignIdx][2] > 0 ? '+' : '-');
        Serial.println("tap = freeze the spin, hold 0.7 s = next sign variant");
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
    static uint32_t lastFrameMs = 0;
    frameStartMs = millis();

    float dt = (frameStartMs - lastFrameMs) * 0.001f;
    lastFrameMs = frameStartMs;
    if (dt <= 0.0f) dt = 0.001f;
    if (dt > 0.2f)  dt = 0.2f;
    const float t = frameStartMs * 0.001f;        /* seconds since boot */

    /* ---- 1. touch gestures, then the IMU -------------------------- */
    amoled.touch().update();

    if (amoled.touch().tapped())                  /* tap: freeze / unfreeze */
    {
        spinOn = !spinOn;
        if (Serial) Serial.printf("spin %s\n", spinOn ? "on" : "frozen");
    }

    if (amoled.touch().heldOnce(700))             /* hold: next axis signs */
    {
        imuSignIdx = (imuSignIdx + 1) & 3;
        gravityOk  = false;                       /* re-seed, do not ramp */
        const int8_t *s = kImuSigns[imuSignIdx];
        if (Serial) Serial.printf("sign variant %d (%c %c %c)\n", imuSignIdx,
                                  s[0] > 0 ? '+' : '-', s[1] > 0 ? '+' : '-',
                                  s[2] > 0 ? '+' : '-');
    }

#if CUBE_IMU_ENABLE
    imuUpdate();
#endif

    /* ---- 2. the rotation: align with gravity, then spin ----------- */
    M3   q       = identity();                    /* world -> screen      */
    bool aligned = false;                         /* is gravity driving it? */

#if CUBE_IMU_ENABLE
    if (imuOk && gravityOk)
    {
        /* the renderer's +y is up on the screen, the display's +y is down */
        V3 up = { gravityDisp.x, -gravityDisp.y, gravityDisp.z };
        q = worldMatrix(up);
        aligned = true;
    }
#endif

    static float spinAngle = 0.0f;
    if (spinOn) spinAngle += CUBE_SPIN_Y * dt;    /* frozen: hold the angle */

    const M3 s = spinMatrix(spinAngle, t * CUBE_SPIN_X, aligned);
    const M3 m = mul(q, s);

    /* The sun is a direction in the world, so it goes through the alignment as
     * well.  Without an IMU q is the identity and it stays fixed to the screen,
     * exactly like in 06_Rotating_Cube. */
    const V3 lightWorld = { kLight[0], kLight[1], kLight[2] };
    lightView = apply(q, lightWorld);

    /* ---- 3. rotate the cube -------------------------------------- */
    for (int i = 0; i < 8; i++)
    {
        const V3 c = { kCorner[i][0], kCorner[i][1], kCorner[i][2] };
        rotated[i] = apply(m, c);
    }
#if CUBE_FACES
    for (int f = 0; f < 6; f++)
    {
        const V3 n = { kFaceNormal[f][0], kFaceNormal[f][1], kFaceNormal[f][2] };
        normal[f] = apply(m, n);
    }
#endif

    /* ---- 4. project them, and find the box that changes ----------- */
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

    /* the region this frame will touch: the new box plus last frame's, so every
     * pixel the cube has been drawn on is cleared again */
    int ux0 = boxX0, uy0 = boxY0, ux1 = boxX1, uy1 = boxY1;
    if (lastValid)
    {
        if (lastX0 < ux0) ux0 = lastX0;
        if (lastY0 < uy0) uy0 = lastY0;
        if (lastX1 > ux1) ux1 = lastX1;
        if (lastY1 > uy1) uy1 = lastY1;
    }

    /* ---- 5. erase it, and stop anything from drawing outside it --- */
    AMOLED_Canvas &fb = amoled.canvas();
    fb.setClip(ux0, uy0, ux1, uy1);
    fb.fillRect(ux0, uy0, ux1, uy1, AMOLED_BLACK);        /* true black */

    /* ---- 6. draw the cube ---------------------------------------- */
    const float hueBase = fmodf(t * CUBE_HUE_SPEED, 360.0f);   /* the rainbow */

#if CUBE_FACES
    for (int f = 0; f < 6; f++)
    {
        /* our camera looks along +z, so only faces whose normal points at it
         * (z < 0) are visible - the classic back face cull */
        if (normal[f].z >= 0.0f) continue;

        const uint8_t *qn = kFace[f];
        const int xs[4] = { scrX[qn[0]], scrX[qn[1]], scrX[qn[2]], scrX[qn[3]] };
        const int ys[4] = { scrY[qn[0]], scrY[qn[1]], scrY[qn[2]], scrY[qn[3]] };

        /* lambert-ish shading: how squarely the face points at the sun */
        float lit = dot(normal[f], lightView);
        if (lit < 0.0f) lit = 0.0f;

        const float d = 0.25f * (depth[qn[0]] + depth[qn[1]] + depth[qn[2]] + depth[qn[3]]);
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

    /* If the cube reached a readout, repaint it now: the box we are about to
     * push covers that area, so the text has to be in the framebuffer before it
     * goes out. */
    if (fpsBox.valid &&
        !(fpsBox.x1 < ux0 || fpsBox.x0 > ux1 || fpsBox.y1 < uy0 || fpsBox.y0 > uy1))
    {
        drawFpsText(fb, fps, fpsBox);
    }
    if (imuBox.valid &&
        !(imuBox.x1 < ux0 || imuBox.x0 > ux1 || imuBox.y1 < uy0 || imuBox.y0 > uy1))
    {
        drawImuText(fb, imuBox);
    }

    /* ---- 7. send the changed region to the panel ------------------ */
#if !CUBE_DIRTY_RECT
    amoled.push();                            /* whole frame: the reference */
    pushedBytes = (uint32_t)W * H * (uint32_t)amoled.bytesPerPixel();
#elif CUBE_FULL_WIDTH
    /* full width bands that line up with the strip grid: every transfer is the
     * shape the streaming mode uses, and packing one is a single memcpy */
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

    /* ---- 8. the readouts ----------------------------------------- */
    const uint32_t now2 = millis();
    fpsFrames++;

    if ((now2 - fpsStartMs) >= 1000)              /* report once per second */
    {
        fps = fpsFrames * 1000.0f / (float)(now2 - fpsStartMs);
        fpsFrames  = 0;
        fpsStartMs = now2;

#if CUBE_SHOW_FPS
        const int shown = (int)(fps + 0.5f);
        if (shown != fpsShown)                    /* only redraw when it changes */
        {
            fpsShown = shown;
            const TextBox old = fpsBox;
            drawFpsText(fb, fps, fpsBox);
            pushTextBox(old, fpsBox);
        }
#endif

#if CUBE_SERIAL_FPS
        if (Serial)                               /* never block on the USB CDC */
        {
            Serial.printf("fps %6.1f | frame %5.2f ms | pushed %4u kB | g %+.2f %+.2f %+.2f | %s%s\n",
                          fps,
                          1000.0f / (fps > 0.01f ? fps : 0.01f),
                          (unsigned)(pushedBytes / 1024),
                          gravityDisp.x, gravityDisp.y, gravityDisp.z,
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

    /* the IMU readout, a few times a second - it does not need 60 */
    static uint32_t imuTextMs = 0;
    if ((now2 - imuTextMs) >= IMU_TEXT_MS)
    {
        imuTextMs = now2;
        const TextBox old = imuBox;
        drawImuText(fb, imuBox);
        pushTextBox(old, imuBox);
    }
}
