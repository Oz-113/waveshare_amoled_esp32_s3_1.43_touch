/*
 * 09_IMU_Fluid - WaveshareAMOLED library
 * ============================================================================
 *  A puddle of neon liquid that always runs to the low side of the screen.
 *
 *  The on board QMI8658 accelerometer is the only input:
 *
 *      tilt the display      the liquid runs downhill and settles against the
 *                            lowest part of the round wall
 *      shake the display     it splashes, then calms down again
 *      lay it flat, screen up
 *                            there is no "down" left on the screen (gravity
 *                            pulls straight into the panel), so the droplets
 *                            stop falling and drift about as one blob
 *
 *  The liquid is FLUID_COUNT additive light blobs.  Each one is a radial glow,
 *  and light adds up where glows overlap, so a loose group of droplets *looks*
 *  like one connected body of liquid - without a single pixel of field maths,
 *  marching squares or per pixel metaball evaluation.
 *
 *  Shows: framebuffer mode + a dirty rectangle + setClip()/pushRect(), a small
 *  N-body simulation with real gravity, and how to get *both* the direction and
 *  the *violence* of the movement out of one accelerometer:
 *
 *      the direction of the reading   ->  where "down" is on the screen
 *      | reading | - 1 g              ->  how hard the device is being moved
 *
 *  Touch it:
 *      tap                 stir the liquid (a splash)
 *      hold for 0.7 s      flip through the four sensor -> display axis
 *                          candidates (the one that makes the liquid fall the
 *                          way gravity really pulls is the right one - below)
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
 *                                                and the liquid floats in the
 *                                                middle instead of falling
 *      display held upright in front of you  ->  g should be (0, -1, 0)
 *                                                and the liquid sits along the
 *                                                bottom of the screen
 *
 *  If it is not, hold the screen until the readout reads SIGN 1, 2, 3 or 0 and
 *  pick the one that makes both checks true.  Put that number into
 *  IMU_SIGN_VARIANT below so it survives a reset.
 *
 *  TUNING - the three knobs that change the look most
 *  --------------------------------------------------
 *      FLUID_COUNT      how much liquid there is (22 droplets by default).
 *                       More droplets = a fuller puddle *and* fewer frames per
 *                       second, because every droplet is another glow.
 *      FLUID_RADIUS     how big one droplet's light is.  Bigger = a fat, slow,
 *                       viscous liquid; smaller = a fine spray.
 *      FLUID_INTENSITY  how hard a shake throws the droplets around (0 ignores
 *                       movement altogether, 2 is an explosion).  FLUID_GRAVITY
 *                       is the same thing for tilt, FLUID_DAMPING for how
 *                       quickly it calms down again.
 *  Every knob is a plain define at the top of the code, and the ones that cost
 *  real time (FLUID_COUNT, FLUID_RADIUS, FLUID_DETAIL) are marked as such.
 *
 *  Without an IMU (or with FLUID_IMU_ENABLE 0) a synthetic gravity well walks
 *  slowly around the screen, so the sketch still does something.
 *
 *  IF AN EDGE LOOKS CUT OR DOUBLED
 *  -------------------------------
 *  That is the panel's tear seam, not the drawing: the display is written while
 *  it scans out and this board has no tear-effect pin wired.  FLUID_FPS_CAP
 *  keeps the update rate at or below the panel's refresh rate, so only one seam
 *  can be on screen at a time.  Full explanation: GUIDE.md, Part A6.
 *
 *  THIS SKETCH NEEDS PSRAM (the 651 kB framebuffer): Tools -> PSRAM -> OPI PSRAM.
 * ============================================================================
 */
#include <WaveshareAMOLED.h>
#include <Wire.h>

/* ================================================================== */
/*  Display knobs                                                     */
/* ================================================================== */
#define FLUID_DIRTY_RECT  1        /* 0 = push the whole frame every time    */
#define FLUID_FULL_WIDTH  1        /* 1 = push full width bands (the shape   */
                                   /*     streaming mode uses); 0 = tight box*/
#define FLUID_FPS_CAP     60       /* updates per second, 0 = as fast as it  */
                                   /* gets (more tearing, bigger number)     */
#define FLUID_SHOW_RING   1        /* 1 = draw the arena the liquid runs in  */
#define FLUID_SHOW_FPS    1        /* frame rate on the display              */
#define FLUID_SERIAL_FPS  1        /* frame rate on the serial port          */
#define FLUID_SHOW_IMU    1        /* mapped gravity + sign variant readout  */

/* ================================================================== */
/*  Fluid knobs - these are the ones to play with                     */
/*                                                                    */
/*  Everything is in screen pixels and seconds, so the numbers can be */
/*  read as what they are: FLUID_GRAVITY is "pixels per second per    */
/*  second, per g of tilt", and at 1 g the liquid falls like a stone. */
/* ================================================================== */
#define FLUID_COUNT       22       /* droplets. ^ cost: one glow each, per   */
                                   /* frame (22 of them is ~2 ms)            */
#define FLUID_RADIUS      54.0f    /* the light radius of ONE droplet, px ^  */
                                   /* cost grows with its square             */
#define FLUID_BODY        0.34f    /* its physical (collision) radius, as a  */
                                   /* fraction of FLUID_RADIUS               */
#define FLUID_CORE        0.40f    /* the bright centre of the glow, fraction*/
#define FLUID_HALO_GAIN   62       /* brightness of the wide halo, 0..255    */
#define FLUID_CORE_GAIN   200      /* brightness of the bright centre        */
#define FLUID_DETAIL      1        /* 1 = halo + centre, 0 = centre only     */
                                   /* (about 40 % faster, a harder look)     */
#define FLUID_GRAVITY     620.0f   /* px/s^2 per g of tilt                   */
#define FLUID_INTENSITY   1.00f    /* how hard a shake throws it around      */
#define FLUID_DAMPING     1.30f    /* viscosity, per second                  */
#define FLUID_COHESION    2600.0f  /* how strongly droplets pull each other  */
#define FLUID_SPACING     1.8f     /* where that pull stops, in units of the */
                                   /* distance at which the bodies touch     */
#define FLUID_WALL_BOUNCE 0.42f    /* how much of a wall hit comes back      */
#define FLUID_MAX_SPEED   1600.0f  /* px/s, the stability clamp              */
#define FLUID_SUBSTEPS    2        /* physics steps per frame                */
#define FLUID_POOL_MIN    9.0f     /* below this speed a droplet is asleep   */
#define FLUID_ARENA_PAD   12       /* gap between the liquid and the ring    */
#define FLUID_HUE_SPEED   22.0f    /* degrees of hue per second              */
#define FLUID_HUE_SPREAD  74.0f    /* hue difference between first and last  */
#define FLUID_HUE_TWIST   0.10f    /* hue per pixel across the screen        */

/* ================================================================== */
/*  IMU knobs (QMI8658, same I2C bus as the touch panel)              */
/* ================================================================== */
#define FLUID_IMU_ENABLE  1        /* 0 = no IMU, use the synthetic gravity  */
#define IMU_SIGN_VARIANT  1        /* start value, see kImuSigns[] below.    */
                                   /* Long press cycles it at run time.      */
#define IMU_FILTER        0.15f    /* weight of each new sample, 0..1        */
#define IMU_SHAKE_G       0.25f    /* samples further than this from 1 g do  */
                                   /* not move "down" - they shake instead   */
#define FLUID_SHAKE_MAX   1.20f    /* a |g| deviation of this counts as a    */
                                   /* full shake (0.6 g off = 1.0)           */
#define FLUID_KICK        1500.0f  /* px/s^2 per unit of shake level         */
#define IMU_TEXT_MS       250      /* how often the readout text is redrawn  */

/* ================================================================== */
/*  Objects, and where they come from                                 */
/*                                                                    */
/*    AMOLED         - the display object   (WaveshareAMOLED lib)     */
/*    AMOLED_Canvas  - the drawing surface  (WaveshareAMOLED lib)     */
/*    Wire           - the I2C bus          (Arduino core)            */
/*    Serial         - USB/UART console     (Arduino core)            */
/* ================================================================== */
AMOLED amoled;

static int W, H, CX, CY;           /* screen size and centre                 */

/* ------------------------------------------------------------------ */
/*  The liquid                                                        */
/*                                                                    */
/*  A droplet is a point with a velocity and a physical radius; what  */
/*  you *see* is its light (FLUID_RADIUS), which is much bigger than  */
/*  its body - that is why droplets merge into a puddle instead of    */
/*  stacking, and why the rendering is nothing but a few glows.       */
/*                                                                    */
/*  +y is down here (screen coordinates), which is also the direction */
/*  gravity pulls towards, so the measured gravity needs no flipping  */
/*  at all: "down" on the screen is simply -gravityDisp.              */
/* ------------------------------------------------------------------ */
struct Drop
{
    float x, y;        /* position, screen pixels                          */
    float vx, vy;      /* velocity, pixels per second                      */
    float r;           /* physical radius (what it collides with), px      */
    float bright;      /* 0..1, how much light it is giving off right now   */
    uint8_t id;        /* 0..FLUID_COUNT-1: its place in the hue ramp      */
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

static Drop drops[FLUID_COUNT];

static int arenaR = 0;                 /* the round wall's radius, px          */
static int bodyR  = 0;                 /* physical radius of one droplet, px   */
static int glowR  = 0;                 /* its light radius, px                 */

/* frame rate measurement */
static uint32_t fpsFrames    = 0;
static uint32_t fpsStartMs   = 0;
static uint32_t frameStartMs = 0;
static float    fps          = 0.0f;
static int      fpsShown     = -1;
static uint32_t pushedBytes  = 0;      /* bytes per frame, for the report      */
static float    shakeLevel   = 0.0f;   /* low passed |g| - 1, 0..1             */

/* the region pushed last frame, so it can be erased again exactly */
static int  lastX0 = 0, lastY0 = 0, lastX1 = 0, lastY1 = 0;
static bool lastValid = false;

/* ================================================================== */
/*  Small helpers                                                     */
/* ================================================================== */
static inline int iround(float v) { return (int)(v < 0.0f ? v - 0.5f : v + 0.5f); }

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* -1 .. +1, from the Arduino PRNG */
static inline float frand(void) { return (float)random(-1000, 1001) * 0.001f; }

/* a rainbow colour: hue in degrees (amoledHSV wraps it), brightness 0..1.
 * The bloom of overlapping glows saturates towards white on purpose - that is
 * what a hot spot in a liquid looks like. */
static uint32_t hueColour(float hueDeg, float bright)
{
    const uint32_t c = amoledHSV(hueDeg, 1.0f, 1.0f);
    return amoledScale(c, (uint16_t)(clampf(bright, 0.0f, 1.0f) * 255.0f), 255);
}

/* ================================================================== */
/*  The on board IMU: a QMI8658 on the touch panel's I2C bus          */
/*                                                                    */
/*  Only the accelerometer is used - and it is read for *two* things: */
/*                                                                    */
/*    1. the direction of the reading, which is the world vertical    */
/*       (an accelerometer at rest measures the upward reaction to    */
/*       gravity, i.e. it points at the sky).  That is where "down"   */
/*       is on the screen.                                            */
/*                                                                    */
/*    2. the *length* of the reading.  At rest it is 1 g, and nothing */
/*       else the board can tell you about how hard it is being moved */
/*       comes for free like this: |g| - 1 is exactly the violence of */
/*       the movement, which is what throws the liquid around.        */
/*                                                                    */
/*  The registers (from the QMI8658 datasheet and from Waveshare's own*/
/*  03_I2C_QMI8658 example):                                          */
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

/* the world vertical, in display coordinates: x right, y down, z into the
 * screen.  At rest this points at the sky (1 g), which is what an
 * accelerometer reports - so "down" on the screen is simply the negative of it.
 * The initial value is "held upright in front of you". */
struct V3
{
    float x, y, z;
};

static V3   gravityDisp = {0.0f, -1.0f, 0.0f};
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

/* ------------------------------------------------------------------ */
/*  From one sample to the two things the simulation wants            */
/*                                                                    */
/*     direction : low passed, and only trusted while the device is    */
/*                 essentially at rest (|g| within IMU_SHAKE_G of 1 g) */
/*                 - while you shake it, the "down" it reports is the  */
/*                 shake itself, not gravity.                          */
/*     violence  : |g| - 1, low passed with a *fast attack and a slow  */
/*                 release*, so a flick shows up immediately and then  */
/*                 decays over a second or so: that is a splash.       */
/* ------------------------------------------------------------------ */
static void imuUpdate(void)
{
    float raw[3];

    if (!imuOk || !imuReadRaw(raw)) return;

    const int8_t *s = kImuSigns[imuSignIdx];
    V3 d = { s[0] * raw[0], s[1] * raw[1], s[2] * raw[2] };

    const float m = sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);

    /* ---- how hard is the device being moved? --------------------- */
    const float target = clampf(fabsf(m - 1.0f) / FLUID_SHAKE_MAX, 0.0f, 1.0f);
    const float rate   = (target > shakeLevel) ? 0.50f : 0.06f;
    shakeLevel += (target - shakeLevel) * rate;

    /* ---- where is down? (only believable near 1 g) --------------- */
    if (m < 1.0f - IMU_SHAKE_G || m > 1.0f + IMU_SHAKE_G) return;
    if (m < 1e-6f) return;

    d.x /= m;  d.y /= m;  d.z /= m;

    if (!gravityOk)                          /* first sample (or a new sign  */
    {                                        /* variant): jump, do not ramp  */
        gravityDisp = d;
        gravityOk   = true;
        return;
    }

    gravityDisp.x += (d.x - gravityDisp.x) * IMU_FILTER;
    gravityDisp.y += (d.y - gravityDisp.y) * IMU_FILTER;
    gravityDisp.z += (d.z - gravityDisp.z) * IMU_FILTER;

    /* the filter shortens the vector; the simulation only cares about the
     * direction, so put it back on the unit sphere */
    const float n = sqrtf(gravityDisp.x * gravityDisp.x +
                          gravityDisp.y * gravityDisp.y +
                          gravityDisp.z * gravityDisp.z);
    if (n > 1e-6f)
    {
        gravityDisp.x /= n;  gravityDisp.y /= n;  gravityDisp.z /= n;
    }
}

/* ================================================================== */
/*  The liquid: where it starts, and how it moves                     */
/* ================================================================== */

/* Scatter the droplets over a disc in the middle of the arena.  The golden
 * angle (2.399963 rad) makes a sunflower spiral, which fills a disc evenly
 * instead of leaving the clumps a random or a grid layout would. */
static void fluidReset(void)
{
    for (int i = 0; i < FLUID_COUNT; i++)
    {
        Drop &d = drops[i];
        const float a = (float)i * 2.3999632f;
        const float r = (float)arenaR * 0.45f *
                        sqrtf((float)(i + 1) / (float)FLUID_COUNT);

        d.x = (float)CX + cosf(a) * r;
        d.y = (float)CY + sinf(a) * r;
        d.vx = frand() * 60.0f;
        d.vy = frand() * 60.0f;
        d.r  = (float)bodyR;
        d.bright = 0.5f;
        d.id = (uint8_t)i;
    }
}

/* ------------------------------------------------------------------ */
/*  One physics step                                                  */
/*                                                                    */
/*  gx, gy is the *in plane* pull of gravity, already scaled to       */
/*  px/s^2 - what is left of the measured gravity once the part that  */
/*  points into the screen is dropped.  When the display lies flat on */
/*  the table that vector is zero: gravity pulls straight into the    */
/*  panel, the droplets stop falling, and FLUID_COHESION alone folds  */
/*  them into one slowly drifting blob.  That is not a special case   */
/*  in the code, it is simply what the numbers do.                    */
/*                                                                    */
/*  Forces -> velocity, damping, walls, then position.  Nothing needs */
/*  a matrix, an integral or a pairwise neighbourhood search - which  */
/*  is the point: a puddle of FLUID_COUNT droplets costs a few        */
/*  hundred microseconds per frame.                                   */
/* ------------------------------------------------------------------ */
static void fluidStep(float dt, float gx, float gy, float kick)
{
    /* ---- 1. gravity, and the slap of a shake ---------------------- */
    for (int i = 0; i < FLUID_COUNT; i++)
    {
        Drop &d = drops[i];
        d.vx += gx * dt;
        d.vy += gy * dt;

        if (kick > 0.0f)                       /* being moved: turbulence */
        {
            d.vx += frand() * kick * dt;
            d.vy += frand() * kick * dt;
        }
    }

    /* ---- 2. the droplets against each other ----------------------- */
    for (int i = 0; i < FLUID_COUNT; i++)
    {
        for (int j = i + 1; j < FLUID_COUNT; j++)
        {
            const float dx = drops[j].x - drops[i].x;
            const float dy = drops[j].y - drops[i].y;
            const float d2 = dx * dx + dy * dy;
            if (d2 < 1e-3f) continue;                          /* same spot */

            const float touch = drops[i].r + drops[j].r;       /* bodies meet */
            const float rest  = touch * FLUID_SPACING;         /* pull ends   */
            if (d2 > rest * rest) continue;                    /* far apart   */

            const float d  = sqrtf(d2);
            const float nx = dx / d, ny = dy / d;              /* i -> j */

            /* One smooth force curve: they fall towards each other until
             * their bodies touch, then they push apart again.  That soft
             * body is what keeps the puddle from collapsing to a point,
             * and it is the only "fluid" behaviour in the whole sketch. */
            float acc;
            if (d < touch) acc = -FLUID_COHESION * 2.0f * (1.0f - d / touch);
            else           acc =  FLUID_COHESION * (1.0f - d / rest);

            const float a = acc * dt * 0.5f;                   /* both move */
            drops[i].vx -= nx * a;  drops[i].vy -= ny * a;
            drops[j].vx += nx * a;  drops[j].vy += ny * a;
        }
    }

    /* ---- 3. viscosity, the speed clamp, and going to sleep -------- */
    const float drag = 1.0f / (1.0f + FLUID_DAMPING * dt);
    for (int i = 0; i < FLUID_COUNT; i++)
    {
        Drop &d = drops[i];
        d.vx *= drag;
        d.vy *= drag;

        const float sp2 = d.vx * d.vx + d.vy * d.vy;
        if (sp2 > FLUID_MAX_SPEED * FLUID_MAX_SPEED)
        {
            const float k = FLUID_MAX_SPEED / sqrtf(sp2);      /* never let  */
            d.vx *= k;                                    /* one runaway    */
            d.vy *= k;                                    /* frame explode  */
        }
        else if (sp2 < FLUID_POOL_MIN * FLUID_POOL_MIN)
        {
            /* slow enough to settle: a little extra drag so the puddle
             * really does come to rest instead of shivering forever */
            d.vx *= 0.90f;
            d.vy *= 0.90f;
        }
    }

    /* ---- 4. the round wall ---------------------------------------- */
    for (int i = 0; i < FLUID_COUNT; i++)
    {
        Drop &d = drops[i];
        const float dx = d.x - (float)CX;
        const float dy = d.y - (float)CY;
        const float lim = (float)arenaR - d.r;
        const float d2 = dx * dx + dy * dy;
        if (d2 <= lim * lim) continue;                 /* still inside  */

        const float dist = sqrtf(d2 > 1e-6f ? d2 : 1e-6f);
        const float nx = dx / dist, ny = dy / dist;
        d.x = (float)CX + nx * lim;                    /* put it back   */
        d.y = (float)CY + ny * lim;

        const float vn = d.vx * nx + d.vy * ny;        /* radial speed  */
        if (vn > 0.0f)                                 /* heading out   */
        {
            d.vx -= (1.0f + FLUID_WALL_BOUNCE) * vn * nx;
            d.vy -= (1.0f + FLUID_WALL_BOUNCE) * vn * ny;
        }
        d.vx *= 0.94f;                                 /* wall friction  */
        d.vy *= 0.94f;
    }

    /* ---- 5. move, and work out how bright each droplet is --------- */
    for (int i = 0; i < FLUID_COUNT; i++)
    {
        Drop &d = drops[i];
        d.x += d.vx * dt;
        d.y += d.vy * dt;

        /* fast droplets shine, slow ones dim down: the motion *is* the
         * light, which is what makes a splash read as a splash */
        const float sp = sqrtf(d.vx * d.vx + d.vy * d.vy) / FLUID_MAX_SPEED;
        const float want = 0.34f + 0.66f * clampf(sp, 0.0f, 1.0f);
        d.bright += (want - d.bright) * 0.25f;
    }
}

/* ================================================================== */
/*  Drawing                                                           */
/* ================================================================== */

/* The wall the liquid runs along.  It never moves - but it is painted every
 * frame *inside the clip*, which is what keeps it visible where the eraser has
 * just cleared the framebuffer back to black. */
static void drawRing(AMOLED_Canvas &cv)
{
#if FLUID_SHOW_RING
    cv.addArc(CX, CY, arenaR, 0, 360, 2, amoledRGB(38, 56, 92), 220);
    cv.addArc(CX, CY, arenaR + 4, 0, 360, 1, amoledRGB(14, 22, 40), 255);
#else
    (void)cv;
#endif
}

/* ------------------------------------------------------------------ */
/*  The liquid itself                                                 */
/*                                                                    */
/*  One wide, dim radial glow per droplet, plus a tight bright centre */
/*  - and that is the whole renderer.  Because the panel adds light,  */
/*  a dozen overlapping glows *are* a metaball surface: the puddle    */
/*  has a single bright body where they merge and separate droplets   */
/*  where they do not, with no field to evaluate and no threshold at  */
/*  any resolution.  Each glow costs one multiply, one shift and one  */
/*  table lookup per pixel inside its bounding box.                   */
/*                                                                    */
/*  The hue of a droplet is its place in the ramp plus where it is on */
/*  the screen, so a settled puddle shows a smooth sweep of colour    */
/*  sideways instead of one flat tint.  (kHueStep also guards the     */
/*  FLUID_COUNT 1 case - every knob here is meant to be edited.)      */
/* ------------------------------------------------------------------ */
static const float kHueStep = FLUID_HUE_SPREAD /
                              (float)((FLUID_COUNT > 1) ? (FLUID_COUNT - 1) : 1);

static void drawFluid(AMOLED_Canvas &cv, float hueBase)
{
    for (int i = 0; i < FLUID_COUNT; i++)
    {
        const Drop &d = drops[i];

        const float hue = hueBase
                        + kHueStep * ((float)d.id - 0.5f * (float)(FLUID_COUNT - 1))
                        + (d.x - (float)CX) * FLUID_HUE_TWIST
                        + (d.y - (float)CY) * FLUID_HUE_TWIST * 0.6f;

        const int ix = iround(d.x);
        const int iy = iround(d.y);

#if FLUID_DETAIL
        cv.addGlow(ix, iy, glowR,
                   hueColour(hue + 26.0f, 0.22f + 0.30f * d.bright), FLUID_HALO_GAIN);
#endif
        cv.addGlow(ix, iy, (int)((float)glowR * FLUID_CORE),
                   hueColour(hue, 0.40f + 0.60f * d.bright), FLUID_CORE_GAIN);
    }
}

/* ------------------------------------------------------------------ */
/*  The three readouts                                                */
/*                                                                    */
/*  They sit outside the liquid's dirty rectangle most of the time,   */
/*  so they are pushed separately - and only when they change, because*/
/*  a push costs the same whether it carries one line or a whole band.*/
/*                                                                    */
/*  (TextBox itself is declared up with the other helper types,       */
/*  because the Arduino build inserts generated prototypes at the top */
/*  of the sketch and they have to see the type.)                     */
/* ------------------------------------------------------------------ */
static TextBox fpsBox  = {0, 0, 0, 0, false};
static TextBox imuBox  = {0, 0, 0, 0, false};
static TextBox hintBox = {0, 0, 0, 0, false};

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

/* does the rectangle about to be pushed overlap this readout? */
static bool overlaps(int x0, int y0, int x1, int y1, const TextBox &b)
{
    return b.valid && !(b.x1 < x0 || b.x0 > x1 || b.y1 < y0 || b.y0 > y1);
}

/* the frame rate, in a fixed width box so it never shifts */
static void drawFpsText(AMOLED_Canvas &cv, float value, TextBox &box)
{
    char buf[24];
    snprintf(buf, sizeof(buf), "%3d FPS", (int)(value + 0.5f));

    const int w = AMOLED_Canvas::textWidth(buf, 2);
    const int x = CX - w / 2;
    const int y = 24;

    if (box.valid) cv.fillRect(box.x0, box.y0, box.x1, box.y1, AMOLED_BLACK);
    cv.drawText(x, y, buf, AMOLED_WHITE, 2);

    box.x0 = x - 4;          box.y0 = y - 4;
    box.x1 = x + w + 4;      box.y1 = y + 20;
    box.valid = true;
}

/* what the screen can do - drawn once, but re-drawn whenever the liquid
 * reaches into it (the eraser has no idea text was there) */
static void drawHintText(AMOLED_Canvas &cv, TextBox &box)
{
    const char *s = "TAP = STIR   HOLD = AXIS SIGNS";
    const int w = AMOLED_Canvas::textWidth(s, 1);
    const int x = CX - w / 2;
    const int y = 52;

    if (box.valid) cv.fillRect(box.x0, box.y0, box.x1, box.y1, AMOLED_BLACK);
    cv.drawText(x, y, s, AMOLED_DARKGREY, 1);

    box.x0 = x - 3;          box.y0 = y - 3;
    box.x1 = x + w + 3;      box.y1 = y + 9;
    box.valid = true;
}

/* The IMU readout: the measured gravity in display coordinates, which axis signs
 * are in use, and how hard the device is being moved.  This is what you look at
 * to decide whether IMU_SIGN_VARIANT is right - see the header. */
static void drawImuText(AMOLED_Canvas &cv, TextBox &box)
{
#if FLUID_SHOW_IMU
    char buf[80];
    const int8_t *s = kImuSigns[imuSignIdx];

    if (imuOk && gravityOk)
    {
        snprintf(buf, sizeof(buf), "G %+.2f %+.2f %+.2f  SIGN %d  %c %c %c  SHAKE %d%%",
                 gravityDisp.x, gravityDisp.y, gravityDisp.z, imuSignIdx,
                 s[0] > 0 ? '+' : '-', s[1] > 0 ? '+' : '-', s[2] > 0 ? '+' : '-',
                 (int)(shakeLevel * 100.0f + 0.5f));
    }
    else
    {
        snprintf(buf, sizeof(buf), "NO IMU - synthetic gravity");
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

    /* ---- the arena, and the droplets ------------------------------ */
    arenaR = W / 2 - FLUID_ARENA_PAD;
    if (arenaR > AMOLED_SAFE_RADIUS - FLUID_ARENA_PAD)
    {
        arenaR = AMOLED_SAFE_RADIUS - FLUID_ARENA_PAD;    /* stay inside the   */
    }                                                    /* visible circle    */
    glowR = (int)(FLUID_RADIUS + 0.5f);
    bodyR = (int)(FLUID_RADIUS * FLUID_BODY + 0.5f);
    if (bodyR < 2)  bodyR = 2;
    if (bodyR > 40) bodyR = 40;
    fluidReset();

    /* ---- the IMU --------------------------------------------------- */
#if FLUID_IMU_ENABLE
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
    if (!imuOk && Serial) Serial.println("no IMU: using the synthetic gravity");

    /* ---- the static parts of the picture, drawn once --------------- */
    AMOLED_Canvas &fb = amoled.canvas();
    fb.clearScreen();
    drawRing(fb);
    fpsShown = -1;
    drawFpsText(fb, 0.0f, fpsBox);
    drawHintText(fb, hintBox);
    drawImuText(fb, imuBox);
    amoled.push();                            /* whole frame, once */

    lastX0 = lastY0 = 0;
    lastX1 = lastY1 = 0;
    lastValid  = false;
    fpsStartMs = frameStartMs = millis();

    if (Serial)
    {
        Serial.printf("\n=== IMU fluid ===\n");
        Serial.printf("%s  %dx%d  %d bpp  QSPI %d MHz\n",
                      amoled.controllerName(), W, H, amoled.colorDepth(),
                      (int)(AMOLED_QSPI_CLOCK_HZ / 1000000UL));
        if (imuOk) Serial.printf("QMI8658 at 0x%02X, 8 g / 250 Hz\n", imuAddr);
        Serial.printf("%d droplets, light radius %d px, body %d px, arena %d px\n",
                      (int)FLUID_COUNT, glowR, bodyR, arenaR);
        Serial.printf("gravity %.0f px/s^2/g, intensity %.2f, damping %.2f, substeps %d\n",
                      (double)FLUID_GRAVITY, (double)FLUID_INTENSITY,
                      (double)FLUID_DAMPING, (int)FLUID_SUBSTEPS);
        Serial.printf("push %s, fps cap %d, sign variant %d (%c %c %c)\n",
                      FLUID_DIRTY_RECT ? (FLUID_FULL_WIDTH ? "full width bands" : "dirty box")
                                       : "full frame",
                      (int)FLUID_FPS_CAP, imuSignIdx,
                      kImuSigns[imuSignIdx][0] > 0 ? '+' : '-',
                      kImuSigns[imuSignIdx][1] > 0 ? '+' : '-',
                      kImuSigns[imuSignIdx][2] > 0 ? '+' : '-');
        Serial.println("tap = stir, hold 0.7 s = next sign variant");
    }
}

/* ================================================================== */
/*  loop() - one frame per call, as fast as the panel allows          */
/* ================================================================== */
void loop()
{
    /* ---- 0. keep the update rate at or below the panel's rate ------ */
#if FLUID_FPS_CAP
    while ((uint32_t)(millis() - frameStartMs) < (1000u / FLUID_FPS_CAP)) delay(1);
#endif
    static uint32_t lastFrameMs = 0;
    frameStartMs = millis();

    float dt = (frameStartMs - lastFrameMs) * 0.001f;
    lastFrameMs = frameStartMs;
    if (dt <= 0.0f) dt = 0.001f;
    if (dt > 0.05f) dt = 0.05f;              /* a physics clamp, not a frame cap */
    const float t = frameStartMs * 0.001f;   /* seconds since boot               */

    /* ---- 1. touch gestures, then the IMU -------------------------- */
    amoled.touch().update();

    if (amoled.touch().tapped())                  /* tap: stir the liquid */
    {
        for (int i = 0; i < FLUID_COUNT; i++)
        {
            drops[i].vx += frand() * 900.0f;
            drops[i].vy += frand() * 900.0f;
        }
        if (Serial) Serial.println("stir");
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

#if FLUID_IMU_ENABLE
    imuUpdate();
#endif

    /* ---- 2. where does gravity pull, in the plane of the screen? -- */
    float gx, gy;

#if FLUID_IMU_ENABLE
    if (imuOk && gravityOk)
    {
        /* gravityDisp points at the sky, so "down" is the negative of it - and
         * it is already in screen coordinates (+x right, +y down), so the y
         * component does not have to be flipped here either.  Lying flat on the
         * table leaves nothing in the plane: gx and gy are ~0 and the liquid
         * simply stops falling. */
        gx = -gravityDisp.x * FLUID_GRAVITY;
        gy = -gravityDisp.y * FLUID_GRAVITY;
    }
    else
#endif
    {
        /* No IMU: a synthetic gravity well that walks slowly around the arena,
         * so the sketch still does something worth watching. */
        gx = sinf(t * 0.55f) * 0.45f * FLUID_GRAVITY;
        gy = (0.80f + 0.20f * cosf(t * 0.37f)) * FLUID_GRAVITY;
    }

    const float kick = shakeLevel * FLUID_INTENSITY * FLUID_KICK;

    /* ---- 3. the physics ------------------------------------------ */
    for (int step = 0; step < FLUID_SUBSTEPS; step++)
    {
        fluidStep(dt / (float)FLUID_SUBSTEPS, gx, gy, kick);
    }

    /* ---- 4. the box of screen the liquid occupies ------------------ */
    int minX = W, minY = H, maxX = -1, maxY = -1;
    for (int i = 0; i < FLUID_COUNT; i++)
    {
        const int x = iround(drops[i].x), y = iround(drops[i].y);
        if (x - glowR < minX) minX = x - glowR;
        if (y - glowR < minY) minY = y - glowR;
        if (x + glowR > maxX) maxX = x + glowR;
        if (y + glowR > maxY) maxY = y + glowR;
    }

    int boxX0 = minX, boxY0 = minY, boxX1 = maxX, boxY1 = maxY;
    if (boxX0 < 0)     boxX0 = 0;
    if (boxY0 < 0)     boxY0 = 0;
    if (boxX1 > W - 1) boxX1 = W - 1;
    if (boxY1 > H - 1) boxY1 = H - 1;

    /* the region this frame will touch: the new box plus last frame's, so every
     * pixel the liquid has been drawn on is cleared again */
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

    /* ---- 6. draw the liquid (and the wall it runs along) ---------- */
    drawRing(fb);
    drawFluid(fb, fmodf(t * FLUID_HUE_SPEED, 360.0f));

    fb.resetClip();                 /* back to the whole framebuffer */

    /* If the liquid reached a readout, repaint it now: the box we are about to
     * push covers that area, so the text has to be in the framebuffer before it
     * goes out. */
    if (overlaps(ux0, uy0, ux1, uy1, hintBox)) drawHintText(fb, hintBox);
    if (overlaps(ux0, uy0, ux1, uy1, fpsBox))  drawFpsText(fb, fps, fpsBox);
    if (overlaps(ux0, uy0, ux1, uy1, imuBox))  drawImuText(fb, imuBox);

    /* ---- 7. send the changed region to the panel ------------------ */
#if !FLUID_DIRTY_RECT
    amoled.push();                            /* whole frame: the reference */
    pushedBytes = (uint32_t)W * H * (uint32_t)amoled.bytesPerPixel();
#elif FLUID_FULL_WIDTH
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

#if FLUID_SHOW_FPS
        const int shown = (int)(fps + 0.5f);
        if (shown != fpsShown)                    /* only redraw when it changes */
        {
            fpsShown = shown;
            const TextBox old = fpsBox;
            drawFpsText(fb, fps, fpsBox);
            pushTextBox(old, fpsBox);
        }
#endif

#if FLUID_SERIAL_FPS
        if (Serial)                               /* never block on the USB CDC */
        {
            Serial.printf("fps %6.1f | frame %5.2f ms | pushed %4u kB | "
                          "g %+.2f %+.2f %+.2f | shake %3d%% | %s%s\n",
                          fps,
                          1000.0f / (fps > 0.01f ? fps : 0.01f),
                          (unsigned)(pushedBytes / 1024),
                          gravityDisp.x, gravityDisp.y, gravityDisp.z,
                          (int)(shakeLevel * 100.0f + 0.5f),
#if !FLUID_DIRTY_RECT
                          "full frame",
#elif FLUID_FULL_WIDTH
                          "full width bands",
#else
                          "dirty box",
#endif
                          FLUID_FPS_CAP ? " (capped)" : "");
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











