/*
 * 09_IMU_Fluid - WaveshareAMOLED library
 * ============================================================================
 *  A tank of neon liquid inside the screen that always runs to the low side of
 *  the display.
 *
 *  This is a real grid simulation, not a pile of glowing blobs.  The screen is
 *  cut into FLUID_GRID_COLS x FLUID_GRID_COLS cells, every cell stores an amount
 *  of liquid ("mass"), and once per frame the liquid is redistributed between
 *  neighbouring cells:
 *
 *      a cell gives liquid to a neighbour when it holds more than the neighbour
 *      (that height difference is what a real liquid calls pressure), and the
 *      direction of gravity decides how much easier it is to give in that
 *      direction than in the others.
 *
 *  That single rule, run a few times per frame, produces everything you see: a
 *  level surface, a puddle that runs to the low side when you tilt, waves that
 *  slosh back and forth, spray when you shake it, and a still flat layer when
 *  the display lies flat on the table (then there is no in-plane gravity left,
 *  so only the pressure term acts and the liquid spreads out evenly).
 *
 *  WHY IT LOOKS LIKE A FLUID AND NOT LIKE BLOCKS
 *  ---------------------------------------------
 *  The simulation is on a grid, the *picture* is not.  For every line of the
 *  grid across the direction of gravity the sketch adds up the mass of that line
 *  and draws one solid run of pixels from the floor of the tank up to that
 *  height - so the visible surface is smooth to the pixel, while the physics
 *  stays a cheap cellular automaton.  Cells holding liquid above that surface
 *  (droplets thrown up by a shake) are drawn separately as small squares, which
 *  is where the spray comes from.
 *
 *  THE ONE INPUT: the QMI8658 accelerometer
 *  ----------------------------------------
 *      direction of the reading   ->  where "down" is on the screen
 *      | reading | - 1 g          ->  how hard the device is being moved
 *
 *  The second one is not just cosmetic: while you shake, the acceleration
 *  dominates the reading, so the gravity vector the simulation follows is the
 *  *measured* one - which is what the liquid inside really feels.  That is why a
 *  hard shake throws it against the far wall instead of merely wobbling it.
 *
 *  Touch it:
 *      drag                push the liquid around (your finger is a local
 *                          gravity well, and it slowly paints new liquid)
 *      tap                 cycle the palette: constant colour / RGB by depth /
 *                          thermal
 *      hold for 0.7 s      cycle the four sensor -> display axis signs (see
 *                          IMU_SIGN_VARIANT below - the one thing that may need
 *                          your help)
 *
 *  SPEED, AND WHAT THE NUMBERS MEAN
 *  --------------------------------
 *  A full frame is W * H * 3 bytes over the QSPI bus (466 x 466 x 3 = 651 kB),
 *  which at the library's 80 MHz is about 16 ms - that, and not the simulation,
 *  is the ceiling: roughly 55-60 fps.  At 40 MHz QSPI (see AMOLED_QSPI_CLOCK_HZ
 *  in AMOLED_Config.h) it is about 33 ms, i.e. roughly 30 fps.
 *
 *  Two things keep this sketch near that ceiling instead of far below it:
 *
 *    1. streaming mode.  The scene is drawn into 32 line strips of internal DMA
 *       RAM and each strip is sent while the next one is drawn, so the pixels
 *       never leave the chip and no PSRAM is involved at all (this sketch does
 *       not need the 651 kB PSRAM framebuffer the other examples use).
 *    2. the fluid is drawn as a handful of rectangles per line, not as per pixel
 *       glow maths: filling 466 x 466 pixels costs about a millisecond, and the
 *       cellular automaton about the same.
 *
 *  The readout at the top of the screen (and the serial line once a second)
 *  prints where the time actually goes:
 *
 *      FPS   frames per second, measured over the last second
 *      PHY   cellular automaton, all substeps of the frame (ms)
 *      PLN   turning the grid into the rectangle list (ms)
 *      DRW   drawing the strips on the CPU (ms)
 *      DMA   the rest of the frame: the QSPI transfers, the part that overlaps
 *            the drawing, and waiting for a free strip buffer (ms)
 *      MB/s  W * H * 3 bytes / frame time - the achieved bus bandwidth.  This is
 *            the number to look at first: ~33 MB/s means the panel really is on
 *            an 80 MHz QSPI bus, ~16 MB/s means 40 MHz, and much less than that
 *            means something else (a slower clone of the panel, USB serial
 *            traffic, an unstable clock) is in the way.
 *
 *  TUNING - the four knobs that change the look most
 *  ------------------------------------------------
 *      FLUID_GRID_COLS  the resolution of the simulation.  40 cells across
 *                       means 12 px cells; 64 is a finer, more detailed liquid
 *                       (and 2.5x the simulation cost, which is still small);
 *                       24 is a coarse one, good for watching the grid work.
 *      FLUID_FILL_PCT   how much liquid there is, in percent of the tank.  20
 *                       is a thin layer at the bottom, 60 sloshes against the
 *                       top of the round wall.  It can also be changed at
 *                       runtime (see the serial commands below).
 *      FLUID_FLOW_RATE  how fast the liquid moves (0.6 is water-ish, 0.25 is
 *                       thick like syrup, 1.0 is chaos)
 *      FLUID_GRAVITY_BIAS
 *                       how strongly the tilt beats the levelling term.  At 0
 *                       the liquid stays level even when you tilt; at 6 a slight
 *                       tilt sends it all to one side.
 *
 *  SERIAL COMMANDS (type them in the serial monitor, 115200 baud)
 *      +  /  -            5 % more / less liquid
 *      r                  refill with the configured amount
 *      p                  next palette
 *      s                  next sensor axis sign variant
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
 *                                                and the liquid spreads out
 *                                                evenly instead of running to
 *                                                one side
 *      display held upright in front of you  ->  g should be (0, -1, 0)
 *                                                and the liquid pools along the
 *                                                bottom of the screen
 *
 *  If it is not, hold the screen until the readout reads SIGN 1, 2, 3 or 0 and
 *  pick the one that makes both checks true.  Put that number into
 *  IMU_SIGN_VARIANT below so it survives a reset.
 *
 *  WITHOUT AN IMU (or with FLUID_IMU_ENABLE 0) a synthetic gravity well walks
 *  slowly around the screen, so the sketch still does something.
 *
 *  IF AN EDGE LOOKS CUT OR DOUBLED
 *  -------------------------------
 *  That is the panel's tear seam, not the drawing: the display is written while
 *  it scans out and this board has no tear-effect pin wired.  FLUID_FPS_CAP
 *  keeps the update rate at or below the panel's refresh rate, so only one seam
 *  can be on screen at a time.  Full explanation: GUIDE.md, Part A6.
 * ============================================================================
 */
#include <WaveshareAMOLED.h>
#include <Wire.h>
#include <string.h>
#include <math.h>

static AMOLED amoled;

static int W = 0, H = 0, CX = 0, CY = 0;

/* ================================================================== */
/*  1. What is computed and pushed - in the order it costs time        */
/* ================================================================== */
#define FLUID_FPS_CAP        60     /* 0 = no cap (see the tear seam note) */
#define FLUID_SHOW_FPS       1
#define FLUID_SHOW_PROFILE   1      /* PHY / PLN / DRW / DMA / MB/s readout */
#define FLUID_SHOW_HINT      1
#define FLUID_SHOW_IMU       1
#define FLUID_SHOW_RING      1      /* the line around the round tank      */
#define FLUID_SHOW_GRID      0      /* 1 = show the simulation cells       */
#define FLUID_SERIAL_FPS     1
#define FLUID_SERIAL_CMDS    1      /* '+' '-' 'r' 'p' 's' in the monitor  */

/* ================================================================== */
/*  2. The grid - the resolution of the simulation                     */
/* ================================================================== */
#define FLUID_GRID_COLS      40     /* cells across the screen (16 .. 72)  */
#define FLUID_GRID_MAX       72     /* do not raise: the arrays are sized  */
#define FLUID_FILL_PCT       45.0f  /* how much liquid, in % of the tank   */
#define FLUID_FILL_FROM_TOP  0      /* 1 = fill from the top instead       */
#define FLUID_TANK_ROUND     1      /* 1 = the visible disc, 0 = rectangle */
#define FLUID_TANK_RADIUS    (AMOLED_SAFE_RADIUS - 2)

/* ================================================================== */
/*  3. The physics of the cellular automaton                           */
/* ================================================================== */
/*  A cell gives f = (its own mass - the neighbour's mass) * rate away.          */
/*  The rate of one direction is its weight divided by the sum of all four       */
/*  weights, and the weights are: a constant levelling term (a liquid wants to   */
/*  be flat) plus the gravity bias along that direction.                         */
#define FLUID_LEVELING       0.22f  /* constant part of a direction's weight */
#define FLUID_GRAVITY_BIAS   3.0f   /* how much tilt beats levelling         */
#define FLUID_FLOW_RATE      0.60f  /* fraction of a height difference moved */
#define FLUID_MAX_FLOW       0.30f  /* cells of liquid a cell may give away
                                     * in one substep - this is the "viscosity"
                                     * that keeps the surface from exploding   */
#define FLUID_VISCOSITY      0.05f  /* extra smoothing of the surface         */
#define FLUID_CELL_MAX       12.0f  /* safety clamp per cell                  */
#define FLUID_CONNECT_MIN    0.35f  /* a cell this empty breaks the pool: what
                                     * is beyond it is drawn as spray         */
#define FLUID_PHYS_MS        16.6f  /* one substep represents this much time  */
#define FLUID_MAX_SUBSTEPS   3      /* ... and a frame runs at most this many */
#define FLUID_GRAVITY        1.0f   /* 1 = the tilt as measured, 0 = ignore   */
#define FLUID_SHAKE_DIR      0.85f  /* how much of a shake turns into "down"  */
#define FLUID_SHAKE_BOOST    1.20f  /* extra pull while being shaken         */
#define FLUID_SPLASH_MIN     0.25f  /* shake needed before anything flies    */

/* ================================================================== */
/*  4. The look                                                        */
/* ================================================================== */
#define FLUID_PALETTE        1      /* 0 constant colour, 1 RGB by depth,
                                     * 2 thermal - 'tap' cycles it too       */
#define FLUID_R              0      /* used by palettes 0 and 2              */
#define FLUID_G              170
#define FLUID_B              255
#define FLUID_SHADE_BANDS    2      /* vertical bands per line: 1 = flat
                                     * (fastest), 2..6 = a depth gradient     */
#define FLUID_SHADE_FLOOR    0.30f  /* brightness of the deepest band         */
#define FLUID_DEPTH_FALLOFF  0.45f  /* how much a shallow line is dimmed      */
#define FLUID_HUE_DEPTH      130.0f /* hue travel from the floor to the surface */
#define FLUID_HUE_SPEED      10.0f  /* degrees per second of hue drift        */
#define FLUID_SURFACE_GAIN   140    /* extra light on the top line, 0 = off   */
#define FLUID_SURFACE_PX     2      /* thickness of that line, px             */
#define FLUID_SPRAY_MIN      0.06f  /* mass below this is not drawn at all    */
#define FLUID_BG             0x000000u
#define FLUID_MAX_RECTS      900    /* the rectangle list per frame           */

/* ================================================================== */
/*  5. Touch                                                           */
/* ================================================================== */
#define FLUID_TOUCH_ENABLE   1
#define FLUID_TOUCH_R        70     /* how far the finger's "gravity" reaches  */
#define FLUID_TOUCH_PUSH     3.0f   /* how hard it pushes away from the finger */
#define FLUID_TOUCH_PAINT    0.20f  /* liquid painted per second, 0 = no paint */

/* ================================================================== */
/*  6. IMU (QMI8658 accelerometer)                                     */
/* ================================================================== */
#define FLUID_IMU_ENABLE     1
#define IMU_SIGN_VARIANT     1      /* 0..3 - see the header                */
#define IMU_FILTER           0.25f  /* low pass on the gravity direction    */
#define IMU_SHAKE_G          0.25f  /* |g| within this of 1 g = "at rest"   */
#define FLUID_SHAKE_MAX      1.20f  /* |g|-1 that counts as a full shake    */
#define IMU_TEXT_MS          250    /* readout refresh, ms                  */

/* ================================================================== */
/*  7. The grid itself, and the frame's rectangle list                 */
/*                                                                    */
/*  gridMass/gridDelta are padded by one cell on every side, so the    */
/*  neighbour lookups in the hot loop never need a bounds test: a      */
/*  border cell simply counts as "outside" and is never written.       */
/* ================================================================== */
#define GRID_STRIDE      (FLUID_GRID_MAX + 2)
#define GRID_CELLS       (GRID_STRIDE * GRID_STRIDE)

static float    gridMass[GRID_CELLS];     /* cells of liquid (1.0 = full)  */
static float    gridDelta[GRID_CELLS];    /* one substep of net flow       */
static uint8_t  gridInside[GRID_CELLS];   /* 1 = this cell is in the tank  */

/* cell (x, y) in the padded array; only x/y in 0..gridCols-1 / 0..gridRows-1 are
 * ever written, everything outside stays 0 = outside the tank */
static inline int gidx(int x, int y) { return (y + 1) * GRID_STRIDE + (x + 1); }

static int      gridCols = 0, gridRows = 0, cellPx = 0;

/* where the tank is, per line of the grid: the first and last cell that is
 * inside, and the same as a pixel span - both directions are needed because the
 * sweep runs along the gravity axis (a display lying on its side pools against
 * the left or right wall) */
static int16_t  lineTop[FLUID_GRID_MAX],   lineBot[FLUID_GRID_MAX];
static int16_t  lineLeft[FLUID_GRID_MAX],  lineRight[FLUID_GRID_MAX];
static int16_t  colY0[FLUID_GRID_MAX],     colY1[FLUID_GRID_MAX];
static int16_t  rowX0[FLUID_GRID_MAX],     rowX1[FLUID_GRID_MAX];

static float    lineHeight[FLUID_GRID_MAX];    /* contiguous liquid, in cells */
static float    lineHeightMax = 1.0f;          /* the deepest line this frame */

static float    fluidFillPct = FLUID_FILL_PCT;
static float    fluidMass    = 0.0f;           /* total, for the readout     */

/* One rectangle of the frame.  "add" ones are drawn additively (light on top of
 * what is there), which is how the meniscus line on the surface is made. */
struct FluidRect
{
    int16_t  x0, y0, x1, y1;
    uint32_t c;
    uint8_t  add;
};

static FluidRect rects[FLUID_MAX_RECTS];
static int       rectCount = 0;
static int       rectDropped = 0;

/* ================================================================== */
/*  8. Frame state, timing and the readouts                            */
/* ================================================================== */
static float    fluidDirX = 0.0f, fluidDirY = 1.0f;  /* where it falls, unit */
static float    fluidMag  = 1.0f;                    /* how hard             */
static float    shakeLevel = 0.0f;                   /* 0..1, low passed     */
static float    hueBase   = 0.0f;
static uint8_t  paletteMode = FLUID_PALETTE;

static float    synthAngle = 90.0f;       /* degrees; no-IMU fallback */

static bool     touchOn = false;
static int      touchX = 0, touchY = 0;

static uint32_t fpsFrames = 0, fpsStartMs = 0, lastFrameMs = 0;
static float    fps = 0.0f;
static float    msPhys = 0.0f, msPlan = 0.0f, msDraw = 0.0f, msDma = 0.0f,
                msFrame = 0.0f, mbPerSec = 0.0f;

static char     fpsLine[32]     = "";
static char     profileLine[48] = "";
static char     imuLine[80]     = "";

/* ================================================================== */
/*  9. Small helpers                                                   */
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

/* colour for one band of the liquid in one line.
 *   t    0 = the floor of this line, 1 = its surface
 *   rel  this line's height compared to the deepest line (0..1) */
static uint32_t fluidColour(float t, float rel, bool surface)
{
    const float depth = FLUID_SHADE_FLOOR + (1.0f - FLUID_SHADE_FLOOR) * t;
    const float dim   = 1.0f - FLUID_DEPTH_FALLOFF * (1.0f - rel);
    float       v     = clampf(depth * dim, 0.0f, 1.0f);

    if (surface && FLUID_SURFACE_GAIN)
    {
        v = clampf(v * (1.0f + (float)FLUID_SURFACE_GAIN / 255.0f), 0.0f, 1.0f);
    }

    switch (paletteMode)
    {
        case 0:     /* one constant colour, only its brightness follows depth */
            return amoledScale(amoledRGB(FLUID_R, FLUID_G, FLUID_B),
                               (uint16_t)(v * 255.0f), 255);

        case 2:     /* thermal: the same base colour, but the surface and the
                     * spray go white hot */
        {
            const uint32_t base = amoledScale(amoledRGB(FLUID_R, FLUID_G, FLUID_B),
                                              (uint16_t)(v * 255.0f), 255);
            const uint8_t  k    = (uint8_t)(clampf(t * t, 0.0f, 1.0f) * (surface ? 230.0f : 150.0f));
            return amoledMix(base, AMOLED_WHITE, k);
        }

        default:    /* RGB: hue runs from the colour of the deep liquid to the
                     * colour at the surface, and drifts slowly with time */
        {
            const float hue = hueBase + FLUID_HUE_DEPTH * t;
            return amoledHSV(hue, 1.0f, clampf(0.22f + 0.78f * v, 0.0f, 1.0f));
        }
    }
}

/* ================================================================== */
/* 10. The on board IMU: a QMI8658 on the touch panel's I2C bus        */
/*                                                                    */
/*  Only the accelerometer is used, and it is read for *two* things:   */
/*                                                                    */
/*    1. the direction of the reading, which is the world vertical (an */
/*       accelerometer at rest measures the upward reaction to gravity,*/
/*       i.e. it points at the sky).  The liquid falls the other way.  */
/*    2. the *length* of the reading.  At rest it is 1 g, and nothing  */
/*       else the board can tell you about how hard it is being moved  */
/*       comes for free like this: |g| - 1 is exactly the violence of  */
/*       the movement, and while the board is being moved the reading  */
/*       IS the acceleration - so the liquid gets thrown by the very   */
/*       vector the sensor reports.                                    */
/*                                                                    */
/*  Registers (QMI8658 datasheet / Waveshare's 03_I2C_QMI8658 example):*/
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
 *  The variant is applied as display = (sx * gx, sy * gy, sz * gz).  The product
 *  of the three signs is +1, so the mapping stays a proper rotation.
 */
static const int8_t kImuSigns[4][3] =
{
    { +1, +1, +1 },
    { +1, -1, -1 },   /* IMU_SIGN_VARIANT 1: the default guess */
    { -1, +1, -1 },
    { -1, -1, +1 }
};

struct V3 { float x, y, z; };

static uint8_t imuAddr    = 0;
static bool    imuOk      = false;
static int     imuSignIdx = IMU_SIGN_VARIANT & 3;

/* the world vertical in display coordinates: x right, y down, z into the screen.
 * At rest this points at the sky (1 g) - so the liquid runs *away* from it. */
static V3   gravityDisp = { 0.0f, -1.0f, 0.0f };   /* low passed, gated    */
static V3   shakeDir    = { 0.0f, -1.0f, 0.0f };   /* raw, always current  */
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
    const uint8_t candidates[2] = { IMU_ADDR_H, IMU_ADDR_L };
    uint8_t id = 0;

    for (int i = 0; i < 2; i++)
    {
        imuAddr = candidates[i];
        id = 0;
        if (imuRead(IMU_REG_WHO_AM_I, &id, 1) && id == IMU_WHO_AM_I_VALUE) break;
    }
    if (id != IMU_WHO_AM_I_VALUE)
    {
        if (Serial) Serial.printf("no IMU answer at 0x%02X / 0x%02X\n",
                                  IMU_ADDR_H, IMU_ADDR_L);
        return false;
    }

    imuWrite(IMU_REG_CTRL7, 0x00);            /* configure with sensors off */
    imuWrite(IMU_REG_CTRL1, 0x60);            /* address auto increment     */
    imuWrite(IMU_REG_CTRL2, IMU_CTRL2_8G_250HZ);
    imuWrite(IMU_REG_CTRL7, 0x01);            /* accelerometer on           */
    delay(30);                                /* let a few samples settle   */
    return true;
}

/* one fresh sample, in the sensor's own axes, in g */
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
/*  From one sample to the two things the simulation wants             */
/*                                                                    */
/*     direction : low passed, and only trusted while the device is    */
/*                 essentially at rest (|g| within IMU_SHAKE_G of 1 g) */
/*                 - while you shake it, the "up" the sensor reports is */
/*                 the shake itself, not gravity.                      */
/*     violence  : |g| - 1, low passed with a *fast attack and a slow   */
/*                 release*, so a flick shows up immediately and then   */
/*                 decays over about a second: that is a splash.        */
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

    if (m < 1e-6f) return;
    shakeDir.x = d.x / m;  shakeDir.y = d.y / m;  shakeDir.z = d.z / m;

    /* ---- where is up? (only believable near 1 g) ------------------ */
    if (m < 1.0f - IMU_SHAKE_G || m > 1.0f + IMU_SHAKE_G) return;

    if (!gravityOk)                          /* first sample (or a new sign  */
    {                                        /* variant): jump, do not ramp  */
        gravityDisp = shakeDir;
        gravityOk   = true;
        return;
    }

    gravityDisp.x += (shakeDir.x - gravityDisp.x) * IMU_FILTER;
    gravityDisp.y += (shakeDir.y - gravityDisp.y) * IMU_FILTER;
    gravityDisp.z += (shakeDir.z - gravityDisp.z) * IMU_FILTER;

    /* the filter shortens the vector; only its direction is used, so put it
     * back on the unit sphere */
    const float n = sqrtf(gravityDisp.x * gravityDisp.x +
                          gravityDisp.y * gravityDisp.y +
                          gravityDisp.z * gravityDisp.z);
    if (n > 1e-6f)
    {
        gravityDisp.x /= n;  gravityDisp.y /= n;  gravityDisp.z /= n;
    }
}

/* ------------------------------------------------------------------ */
/*  The gravity the cellular automaton follows                         */
/*                                                                    */
/*  It is the *opposite* of the measured up vector (the liquid runs    */
/*  away from the sky), projected into the screen plane: only the x    */
/*  and y part can push liquid around inside the display.              */
/*                                                                    */
/*  While the board is being shaken, the reading is dominated by the   */
/*  acceleration, and physically that is exactly what the liquid feels,*/
/*  so the raw vector is mixed in and the pull is scaled up.  Lay the  */
/*  display flat and the in-plane part is zero: fluidMag 0, the whole  */
/*  simulation is then the levelling term, and the liquid spreads out  */
/*  evenly - which is what a liquid on a table does.                   */
/* ------------------------------------------------------------------ */
static void fluidGravityUpdate(float dt)
{
    (void)dt;

#if FLUID_IMU_ENABLE
    V3 up;

    if (imuOk && gravityOk)
    {
        const float k = clampf(shakeLevel * FLUID_SHAKE_DIR, 0.0f, 0.9f);
        up.x = gravityDisp.x * (1.0f - k) + shakeDir.x * k;
        up.y = gravityDisp.y * (1.0f - k) + shakeDir.y * k;
        up.z = gravityDisp.z * (1.0f - k) + shakeDir.z * k;

        const float n = sqrtf(up.x * up.x + up.y * up.y + up.z * up.z);
        if (n > 1e-6f) { up.x /= n; up.y /= n; up.z /= n; }

        float mag = FLUID_GRAVITY * (1.0f + shakeLevel * FLUID_SHAKE_BOOST);
        const float ip = sqrtf(up.x * up.x + up.y * up.y);

        if (ip > 1e-3f)
        {
            fluidDirX = -up.x / ip;
            fluidDirY = -up.y / ip;
            fluidMag  = clampf(mag * ip, 0.0f, 3.0f);
        }
        else                                 /* screen up: nothing to fall to */
        {
            fluidDirX = 0.0f;
            fluidDirY = 1.0f;
            fluidMag  = 0.0f;
        }
        return;
    }
#endif

    /* no IMU (or none answering): a gravity well that walks around the screen */
    synthAngle += dt * 24.0f;
    if (synthAngle >= 360.0f) synthAngle -= 360.0f;
    const float a = synthAngle * 0.0174532925f;
    fluidDirX = cosf(a);
    fluidDirY = sinf(a);
    fluidMag  = FLUID_GRAVITY;
}

/* ================================================================== */
/* 11. Building the tank, and filling it                               */
/*                                                                    */
/*  Cell size is chosen so the grid covers the screen: 466 px / 40     */
/*  columns = 12 px cells (rounded up), and 40 rows of 12 px is 480 px */
/*  so the last row hangs over the bottom edge by 14 px - harmless,    */
/*  the canvas clips and the round tank ends before that anyway.       */
/* ================================================================== */
static void fluidGridBuild(void)
{
    gridCols = FLUID_GRID_COLS;
    if (gridCols < 16) gridCols = 16;
    if (gridCols > FLUID_GRID_MAX) gridCols = FLUID_GRID_MAX;

    cellPx = (W + gridCols - 1) / gridCols;
    if (cellPx < 2) cellPx = 2;

    gridRows = (H + cellPx - 1) / cellPx;
    if (gridRows > FLUID_GRID_MAX) gridRows = FLUID_GRID_MAX;

    memset(gridMass,   0, sizeof(gridMass));
    memset(gridInside, 0, sizeof(gridInside));

    const float rad  = (float)FLUID_TANK_RADIUS;
    const float rad2 = rad * rad;

    /* ---- which cells hold liquid at all ---------------- */
    for (int y = 0; y < gridRows; y++)
    {
        for (int x = 0; x < gridCols; x++)
        {
            const float px = (float)(x * cellPx) + cellPx * 0.5f - (float)CX;
            const float py = (float)(y * cellPx) + cellPx * 0.5f - (float)CY;
#if FLUID_TANK_ROUND
            const bool in = (px * px + py * py) <= rad2;
#else
            const bool in = (px >= -rad && px <= rad && py >= -rad && py <= rad);
#endif
            if (in) gridInside[gidx(x, y)] = 1;
        }
    }

    /* ---- the tank, as seen from both sweep directions --- */
    for (int x = 0; x < gridCols; x++)
    {
        int top = gridRows, bot = -1;
        for (int y = 0; y < gridRows; y++)
        {
            if (!gridInside[gidx(x, y)]) continue;
            if (y < top) top = y;
            if (y > bot) bot = y;
        }
        lineTop[x] = (int16_t)top;
        lineBot[x] = (int16_t)bot;
        if (bot >= 0)
        {
            colY0[x] = (int16_t)(top * cellPx);
            const int y1 = bot * cellPx + cellPx - 1;
            colY1[x] = (int16_t)(y1 < H - 1 ? y1 : H - 1);
        }
        else { colY0[x] = 0; colY1[x] = -1; }
    }

    for (int y = 0; y < gridRows; y++)
    {
        int left = gridCols, right = -1;
        for (int x = 0; x < gridCols; x++)
        {
            if (!gridInside[gidx(x, y)]) continue;
            if (x < left)  left  = x;
            if (x > right) right = x;
        }
        lineLeft[y]  = (int16_t)left;
        lineRight[y] = (int16_t)right;
        if (right >= 0)
        {
            rowX0[y] = (int16_t)(left * cellPx);
            const int x1 = right * cellPx + cellPx - 1;
            rowX1[y] = (int16_t)(x1 < W - 1 ? x1 : W - 1);
        }
        else { rowX0[y] = 0; rowX1[y] = -1; }
    }
}

/* How many cells the tank has - the unit the fill percentage is measured in. */
static int fluidTankCells(void)
{
    int n = 0;
    for (int y = 0; y < gridRows; y++)
        for (int x = 0; x < gridCols; x++)
            if (gridInside[gidx(x, y)]) n++;
    return n;
}

/* Pour the liquid in.  Whole rows from the bottom, spread evenly along the row,
 * so that a fill below one cell per row is a thin film and not a stripe. */
static void fluidReset(float pct)
{
    fluidFillPct = clampf(pct, 0.0f, 100.0f);
    memset(gridMass, 0, sizeof(gridMass));

    const int tank = fluidTankCells();
    float target = (float)tank * fluidFillPct / 100.0f;

    for (int r = 0; r < gridRows && target > 0.0f; r++)
    {
        const int y = FLUID_FILL_FROM_TOP ? r : (gridRows - 1 - r);

        int n = 0;
        for (int x = 0; x < gridCols; x++) if (gridInside[gidx(x, y)]) n++;
        if (n == 0) continue;

        const float take = (target >= (float)n) ? (float)n : target;
        const float per  = take / (float)n;
        for (int x = 0; x < gridCols; x++)
        {
            const int i = gidx(x, y);
            if (gridInside[i]) gridMass[i] = per;
        }
        target -= take;
    }

    fluidMass = (float)tank * fluidFillPct / 100.0f;
}

/* Total liquid in the tank, in cells - for the readout. */
static float fluidMassTotal(void)
{
    double sum = 0.0;
    for (int y = 0; y < gridRows; y++)
        for (int x = 0; x < gridCols; x++)
        {
            const int i = gidx(x, y);
            if (gridInside[i]) sum += gridMass[i];
        }
    return (float)sum;
}

/* ================================================================== */
/* 12. The simulation: one substep of the cellular automaton           */
/*                                                                    */
/*  Every cell with liquid in it gives some of it to each of its four  */
/*  neighbours:                                                        */
/*                                                                    */
/*      flow = (my mass - the neighbour's mass) * weight * rate        */
/*                                                                    */
/*  and the weight of a direction is the levelling term (a liquid      */
/*  wants to be flat, and that is what makes it flow uphill sideways   */
/*  into a hollow) plus the gravity bias along that direction.         */
/*                                                                    */
/*  Only *downhill* differences are transported, a cell never hands    */
/*  out more than it holds, and every flow is applied as one           */
/*  subtraction plus one addition of the same number - so the total    */
/*  amount of liquid is conserved exactly, whatever order the sweep    */
/*  runs in.  There is no matrix to solve and no velocity field: the   */
/*  height difference *is* the pressure, which is why a few thousand   */
/*  cells per frame is all this costs.                                 */
/* ================================================================== */
static const int8_t kDirDX[4] = { +1, -1,  0,  0 };   /* right, left, down, up */
static const int8_t kDirDY[4] = {  0,  0, +1, -1 };

/* add the accumulated flows to the mass field */
static void fluidApplyDelta(void)
{
    for (int y = 0; y < gridRows; y++)
    {
        for (int x = 0; x < gridCols; x++)
        {
            const int i = gidx(x, y);
            if (!gridInside[i]) continue;

            float v = gridMass[i] + gridDelta[i];
            if (v < 0.0f)           v = 0.0f;
            if (v > FLUID_CELL_MAX) v = FLUID_CELL_MAX;
            gridMass[i] = v;
        }
    }
}

static void fluidStep(float ux, float uy, float mag)
{
    /* how hard gravity pulls, compared with the levelling term */
    const float gb = FLUID_GRAVITY_BIAS * 0.5f * clampf(mag, 0.0f, 2.5f);

    float w[4];
    w[0] = FLUID_LEVELING + gb * (ux > 0.0f ?  ux : 0.0f);   /* right */
    w[1] = FLUID_LEVELING + gb * (ux < 0.0f ? -ux : 0.0f);   /* left  */
    w[2] = FLUID_LEVELING + gb * (uy > 0.0f ?  uy : 0.0f);   /* down  */
    w[3] = FLUID_LEVELING + gb * (uy < 0.0f ? -uy : 0.0f);   /* up    */

    /* Steepest direction first: if a cell runs out of liquid while it is handing
     * something out, it is the direction gravity favours that gets it. */
    int ord[4] = { 0, 1, 2, 3 };
    for (int a = 0; a < 3; a++)
    {
        for (int b = a + 1; b < 4; b++)
        {
            if (w[ord[b]] > w[ord[a]])
            {
                const int t = ord[a]; ord[a] = ord[b]; ord[b] = t;
            }
        }
    }

    const float kBase = FLUID_FLOW_RATE / (w[0] + w[1] + w[2] + w[3]);

#if FLUID_TOUCH_ENABLE
    const bool finger = touchOn;
#else
    const bool finger = false;
#endif

    (void)finger;
    memset(gridDelta, 0, sizeof(gridDelta));

    for (int y = 0; y < gridRows; y++)
    {
        for (int x = 0; x < gridCols; x++)
        {
            const int i = gidx(x, y);
            if (!gridInside[i]) continue;

            const float m = gridMass[i];
            if (m < 0.002f) continue;

            float wl[4] = { w[0], w[1], w[2], w[3] };
            float kk    = kBase;

#if FLUID_TOUCH_ENABLE
            if (finger)
            {
                /* Your finger is a gravity well pointing away from itself: the
                 * closer a cell is to the finger, the harder it is pushed. */
                const float fx = (float)(x * cellPx + cellPx / 2) - (float)touchX;
                const float fy = (float)(y * cellPx + cellPx / 2) - (float)touchY;
                const float d2 = fx * fx + fy * fy;

                if (d2 < (float)(FLUID_TOUCH_R * FLUID_TOUCH_R))
                {
                    const float d   = sqrtf(d2) + 1.0f;
                    const float amp = gb * FLUID_TOUCH_PUSH *
                                      (1.0f - d / (float)(FLUID_TOUCH_R + 1));
                    if (fx > 0.0f) wl[0] += amp * ( fx / d);
                    else           wl[1] += amp * (-fx / d);
                    if (fy > 0.0f) wl[2] += amp * ( fy / d);
                    else           wl[3] += amp * (-fy / d);

                    const float sum = wl[0] + wl[1] + wl[2] + wl[3];
                    if (sum > 1e-6f) kk = FLUID_FLOW_RATE / sum;
                }
            }
#endif

            /* Hand out what is above the neighbours, never more than we hold. */
            float avail = m;
            for (int o = 0; o < 4; o++)
            {
                const int dir = ord[o];
                const int n   = i + kDirDX[dir] + kDirDY[dir] * GRID_STRIDE;
                if (!gridInside[n]) continue;

                const float head = m - gridMass[n];
                if (head <= 0.0f) continue;

                float f = head * wl[dir] * kk;
                if (f > FLUID_MAX_FLOW) f = FLUID_MAX_FLOW;
                if (f > avail)          f = avail;
                if (f <= 0.0f)          break;

                avail -= f;
                gridDelta[i] -= f;
                gridDelta[n] += f;
            }
        }
    }

    fluidApplyDelta();
}

/* A light diffusion of the mass field: it rounds off the staircase the grid puts
 * into the surface, and it makes the liquid a little thicker.  Each pair of cells
 * exchanges in one direction only, so the total stays exact. */
static void fluidRelax(void)
{
    if (FLUID_VISCOSITY <= 0.0f) return;

    memset(gridDelta, 0, sizeof(gridDelta));

    for (int y = 0; y < gridRows; y++)
    {
        for (int x = 0; x < gridCols; x++)
        {
            const int i = gidx(x, y);
            if (!gridInside[i]) continue;

            const float m = gridMass[i];

            for (int dir = 0; dir < 4; dir++)
            {
                const int n = i + kDirDX[dir] + kDirDY[dir] * GRID_STRIDE;
                if (!gridInside[n]) continue;

                const float head = m - gridMass[n];
                if (head <= 0.0f) continue;

                const float f = head * FLUID_VISCOSITY;
                gridDelta[i] -= f;
                gridDelta[n] += f;
            }
        }
    }

    fluidApplyDelta();
}

/* Liquid under the finger, while it is down: this is how you change the amount
 * without a recompile, and it is the same knob as FLUID_FILL_PCT. */
static void fluidPaint(float dt)
{
#if FLUID_TOUCH_ENABLE
    if (!touchOn || FLUID_TOUCH_PAINT <= 0.0f) return;

    static const int8_t pdx[5] = { 0, -1, +1,  0,  0 };
    static const int8_t pdy[5] = { 0,  0,  0, -1, +1 };
    const int cx = touchX / cellPx, cy = touchY / cellPx;
    const int rad = FLUID_TOUCH_R / (2 * cellPx) + 1;

    for (int k = 0; k < 5; k++)
    {
        const int x = cx + pdx[k] * rad, y = cy + pdy[k] * rad;
        if (x < 0 || y < 0 || x >= gridCols || y >= gridRows) continue;
        const int i = gidx(x, y);
        if (!gridInside[i]) continue;
        gridMass[i] += FLUID_TOUCH_PAINT * dt * 0.25f;
        if (gridMass[i] > FLUID_CELL_MAX) gridMass[i] = FLUID_CELL_MAX;
    }
#else
    (void)dt;
#endif
}

/* ================================================================== */
/* 13. From the grid to the picture                                    */
/*                                                                    */
/*  The simulation is a grid, the picture is not - and this is where   */
/*  the difference is made.  Per frame:                                */
/*                                                                    */
/*    1. for every line of the grid across the gravity axis, add up     */
/*       the mass of the cells that are connected to the floor.  That   */
/*       sum is the height of the liquid in that line.                  */
/*    2. cut that height into FLUID_SHADE_BANDS bands and turn each one */
/*       into one rectangle (a rectangle for the whole line, not one    */
/*       per cell - which is why a 40 x 40 grid costs about 130 fills   */
/*       per frame instead of 1600).                                    */
/*    3. everything that holds liquid but is NOT connected to the floor */
/*       - spray thrown up by a shake - becomes a small square per cell. */
/*                                                                    */
/*  The rectangle list is then drawn once per strip; each strip only    */
/*  touches the rectangles that fall into it, so the list is built once */
/*  and used fifteen times.                                            */
/* ================================================================== */
static void emitRectPx(int x0, int y0, int x1, int y1, uint32_t c, uint8_t add)
{
    if (x1 < x0 || y1 < y0) return;
    if (rectCount >= FLUID_MAX_RECTS) { rectDropped++; return; }

    FluidRect &r = rects[rectCount++];
    r.x0 = (int16_t)x0;  r.y0 = (int16_t)y0;
    r.x1 = (int16_t)x1;  r.y1 = (int16_t)y1;
    r.c  = c;
    r.add = add;
}

/* the pixel rectangle of one grid cell; "vertical" = the sweep runs down a
 * column of the grid, so the line index is x and the cell index is y */
static inline void cellPxRange(bool vertical, int line, int idx,
                               int &x0, int &y0, int &x1, int &y1)
{
    const int a0 = line * cellPx, a1 = a0 + cellPx - 1;
    const int b0 = idx  * cellPx, b1 = b0 + cellPx - 1;

    if (vertical) { x0 = a0; x1 = a1; y0 = b0; y1 = b1; }
    else          { y0 = a0; y1 = a1; x0 = b0; x1 = b1; }
}

/* one run of liquid, from the floor of a line towards its surface:
 * s0..s1 are along the sweep axis, perp0..perp1 across it */
static inline void emitSweepRun(bool vertical, int perp0, int perp1,
                                int s0, int s1, uint32_t c, uint8_t add)
{
    if (vertical) emitRectPx(perp0, s0, perp1, s1, c, add);
    else          emitRectPx(s0, perp0, s1, perp1, c, add);
}

static void fluidPlan(void)
{
    rectCount   = 0;
    rectDropped = 0;

    /* Which way do we sweep?  Along gravity: a display standing on its side
     * pools against the left or right wall, and then the lines of the grid run
     * horizontally, so the same code renders both cases. */
    const bool  vertical = fabsf(fluidDirY) >= fabsf(fluidDirX);
    const int   lines    = vertical ? gridCols : gridRows;
    const float cellF    = (float)cellPx;
    const int   idxMax   = (vertical ? gridRows : gridCols) - 1;

    /* ---- pass 1: how deep the liquid is in every line ---------------- */
    lineHeightMax = 0.01f;

    for (int p = 0; p < lines; p++)
    {
        const bool down  = vertical ? (fluidDirY >= 0.0f) : (fluidDirX >= 0.0f);
        const int  first = vertical ? (down ? lineBot[p] : lineTop[p])
                                    : (down ? lineRight[p] : lineLeft[p]);
        const int  last  = vertical ? (down ? lineTop[p] : lineBot[p])
                                    : (down ? lineLeft[p] : lineRight[p]);
        float h = 0.0f;

        if (first >= 0 && last >= 0)
        {
            const int step = (first <= last) ? +1 : -1;
            for (int idx = first; ; idx += step)
            {
                const float m = gridMass[vertical ? gidx(p, idx) : gidx(idx, p)];
                if (m < FLUID_CONNECT_MIN) break;      /* the pool stops here */
                h += m;
                if (idx == last) break;
            }
        }

        lineHeight[p] = h;
        if (h > lineHeightMax) lineHeightMax = h;
    }

    /* ---- pass 2: one line at a time --------------------------------- */
    for (int p = 0; p < lines; p++)
    {
        const float h = lineHeight[p];
        if (h * cellF < 1.0f) continue;                /* thinner than a pixel */

        const bool down  = vertical ? (fluidDirY >= 0.0f) : (fluidDirX >= 0.0f);
        const int  first = vertical ? (down ? lineBot[p] : lineTop[p])
                                    : (down ? lineRight[p] : lineLeft[p]);
        const int  last  = vertical ? (down ? lineTop[p] : lineBot[p])
                                    : (down ? lineLeft[p] : lineRight[p]);
        if (first < 0 || last < 0) continue;

        const int step = (first <= last) ? +1 : -1;

        /* the floor edge in pixels along the sweep axis, the tank's span on that
         * axis, and the line's pixel range across it */
        int floorPx, spanLo, spanHi, perp0, perp1;
        if (vertical)
        {
            perp0 = p * cellPx;
            perp1 = (perp0 + cellPx - 1 < W - 1) ? perp0 + cellPx - 1 : W - 1;
            spanLo = colY0[p];
            spanHi = colY1[p];
        }
        else
        {
            perp0 = p * cellPx;
            perp1 = (perp0 + cellPx - 1 < H - 1) ? perp0 + cellPx - 1 : H - 1;
            spanLo = rowX0[p];
            spanHi = rowX1[p];
        }
        if (spanHi < spanLo) continue;

        floorPx = (step < 0) ? (first + 1) * cellPx : first * cellPx;

        /* the surface, clamped to the wall of the tank */
        float surf = (float)floorPx + (float)step * h * cellF;
        if (step < 0) { if (surf < (float)spanLo) surf = (float)spanLo; }
        else          { if (surf > (float)spanHi) surf = (float)spanHi; }

        const int   runLo  = iround((surf < (float)floorPx) ? surf : (float)floorPx);
        const int   runHi  = iround((surf > (float)floorPx) ? surf : (float)floorPx);
        const float runLen = (float)(runHi - runLo + 1);
        if (runLen < 1.0f) continue;

        float rel = h / lineHeightMax;
        if (rel > 1.0f) rel = 1.0f;

        /* ---- the body of the liquid, in bands from the floor up ------ */
        for (int b = 0; b < FLUID_SHADE_BANDS; b++)
        {
            const float f0 = (float)b / (float)FLUID_SHADE_BANDS;
            const float f1 = (float)(b + 1) / (float)FLUID_SHADE_BANDS;

            int e0, e1;
            if (step < 0)                      /* floor = the bottom edge  */
            {
                e0 = iround((float)floorPx - runLen * f1);
                e1 = iround((float)floorPx - runLen * f0);
            }
            else                               /* floor = the top edge     */
            {
                e0 = iround((float)floorPx + runLen * f0);
                e1 = iround((float)floorPx + runLen * f1);
            }
            if (e1 < e0) { const int t = e0; e0 = e1; e1 = t; }

            emitSweepRun(vertical, perp0, perp1, e0, e1 - 1,
                         fluidColour((f0 + f1) * 0.5f, rel, false), 0);
        }

        /* ---- the bright line where the liquid meets the air --------- */
        if (FLUID_SURFACE_GAIN > 0)
        {
            int t0, t1;
            if (step < 0) { t0 = runLo; t1 = t0 + FLUID_SURFACE_PX - 1; }
            else          { t1 = runHi; t0 = t1 - FLUID_SURFACE_PX + 1; }
            emitSweepRun(vertical, perp0, perp1, t0, t1,
                         fluidColour(1.0f, rel, true), 1);
        }

        /* ---- spray: mass that is no longer part of the pool ---------- */
        int  idx = first;
        bool gap = false;

        for (;; idx += step)
        {
            const float m = gridMass[vertical ? gidx(p, idx) : gidx(idx, p)];
            if (m < FLUID_CONNECT_MIN) { gap = true; break; }
            if (idx == last) { idx += step; break; }
        }
        if (gap) idx += step;                    /* skip the surface cell */

        for (; idx >= 0 && idx <= idxMax; idx += step)
        {
            const float m = gridMass[vertical ? gidx(p, idx) : gidx(idx, p)];
            if (m < FLUID_SPRAY_MIN) continue;

            int x0, y0, x1, y1;
            cellPxRange(vertical, p, idx, x0, y0, x1, y1);

            int w = iround(cellF * (m < 1.0f ? m : 1.0f));
            if (w < 2)      w = 2;
            if (w > cellPx) w = cellPx;

            const int cx2 = (x0 + x1) / 2, cy2 = (y0 + y1) / 2;
            emitRectPx(cx2 - w / 2, cy2 - w / 2, cx2 + w / 2, cy2 + w / 2,
                       fluidColour(1.0f, rel, true), 0);
        }
    }
}
/* ================================================================== */
/* 14. Input: the touch panel                                          */
/*                                                                    */
/*      drag             the finger pushes the liquid away from itself  */
/*                       and slowly paints new liquid (FLUID_TOUCH_PAINT)*/
/*      tap              next palette                                  */
/*      hold for 0.7 s   next sensor axis sign variant                 */
/* ================================================================== */
static void touchUpdate(void)
{
#if FLUID_TOUCH_ENABLE
    AMOLED_Touch &t = amoled.touch();

    t.update();

    touchOn = t.isDown();
    touchX  = (int)t.x();
    touchY  = (int)t.y();

    if (t.tapped())                       /* short tap: next palette */
    {
        paletteMode = (uint8_t)((paletteMode + 1) % 3);
    }

    if (t.heldOnce(700))                  /* long press: next axis signs */
    {
        imuSignIdx = (imuSignIdx + 1) & 3;
        gravityOk  = false;               /* snap to the new orientation, */
                                          /* do not glide into it         */
    }
#endif
}

/* ================================================================== */
/* 15. Drawing one strip                                               */
/*                                                                    */
/*  In streaming mode the driver hands the canvas one horizontal strip  */
/*  of 32 lines at a time, in internal DMA RAM, and sends the previous  */
/*  strip while we draw this one.  Screen coordinates are what we draw  */
/*  in - the canvas translates them into the strip and clips everything */
/*  that falls outside it, which is why the whole scene can simply be   */
/*  drawn again for every strip.                                       */
/* ================================================================== */
static void hudText(AMOLED_Canvas &cv, int x, int y, const char *s, uint32_t c)
{
    /* a dark shadow first, so the readout stays readable over the liquid */
    cv.drawTextBlend(x + 1, y + 1, s, AMOLED_BLACK, 1, 200);
    cv.drawTextBlend(x, y, s, c, 1, 255);
}

static void hudTextCentered(AMOLED_Canvas &cv, int y, const char *s, uint32_t c)
{
    if (!s || !s[0]) return;
    hudText(cv, CX - AMOLED_Canvas::textWidth(s, 1) / 2, y, s, c);
}

static void fluidDrawStrip(AMOLED_Canvas &cv)
{
    /* the empty tank: black, or very dark if you want to see the glass */
    cv.fillScreen(FLUID_BG);

#if FLUID_SHOW_GRID
    /* the cells the simulation really uses - the quickest way to see what
     * FLUID_GRID_COLS does */
    for (int x = cellPx; x < W; x += cellPx)
        cv.fillRect(x - 1, 0, x - 1, H - 1, 0x1A1A1A);
    for (int y = cellPx; y < H; y += cellPx)
        cv.fillRect(0, y - 1, W - 1, y - 1, 0x1A1A1A);
#endif

    /* the liquid: one rectangle per band of every line of the grid */
    for (int i = 0; i < rectCount; i++)
    {
        const FluidRect &r = rects[i];
        if (r.y1 < cv.windowY0() || r.y0 > cv.windowY1()) continue;   /* other strip */

        if (r.add) cv.addRect(r.x0, r.y0, r.x1, r.y1, r.c);
        else       cv.fillRect(r.x0, r.y0, r.x1, r.y1, r.c);
    }

#if FLUID_SHOW_RING
    cv.addCircle(CX, CY, FLUID_TANK_RADIUS, 0x203040, 200);   /* the tank wall */
#endif

    /* ---- the readouts ---- */
#if FLUID_SHOW_FPS
    hudTextCentered(cv, 12, fpsLine, AMOLED_WHITE);
#endif
#if FLUID_SHOW_PROFILE
    hudTextCentered(cv, 26, profileLine, AMOLED_GREY);
#endif
#if FLUID_SHOW_HINT
    hudTextCentered(cv, 42, "DRAG PUSH  TAP COLOUR  HOLD SIGN", AMOLED_DARKGREY);
#endif
#if FLUID_SHOW_IMU
    hudTextCentered(cv, 58, imuLine, AMOLED_GREY);
#endif
}
/* ================================================================== */
/* 16. The readout strings                                             */
/* ================================================================== */
static void fluidBuildFpsLine(void)
{
    if (!FLUID_SHOW_FPS) { fpsLine[0] = 0; return; }
    snprintf(fpsLine, sizeof(fpsLine), "FPS %2d    %2.0f MB/s", iround(fps), mbPerSec);
}

static void fluidBuildProfileLine(void)
{
    if (!FLUID_SHOW_PROFILE) { profileLine[0] = 0; return; }
    snprintf(profileLine, sizeof(profileLine),
             "PHY %.1f PLN %.1f DRW %.1f DMA %.1f", msPhys, msPlan, msDraw, msDma);
}

static void fluidBuildImuLine(void)
{
    if (!FLUID_SHOW_IMU) { imuLine[0] = 0; return; }

    if (!imuOk || !gravityOk)
    {
        snprintf(imuLine, sizeof(imuLine), "NO IMU - synthetic gravity");
        return;
    }

    const int8_t *s = kImuSigns[imuSignIdx];
    snprintf(imuLine, sizeof(imuLine),
             "G %+.2f %+.2f %+.2f SIGN %d %c %c %c SHAKE %d%%",
             gravityDisp.x, gravityDisp.y, gravityDisp.z, imuSignIdx,
             s[0] > 0 ? '+' : '-', s[1] > 0 ? '+' : '-', s[2] > 0 ? '+' : '-',
             iround(shakeLevel * 100.0f));
}

/* ================================================================== */
/* 17. setup() - runs once                                             */
/* ================================================================== */
void setup()
{
    /* With "USB CDC On Boot" Serial is the native USB port: if no host has it
     * open, a write would block for up to two seconds, so the tx timeout is set
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

    randomSeed(micros());

    /* ---- the tank, and the liquid --------------------------------- */
    fluidGridBuild();
    fluidReset(FLUID_FILL_PCT);

    /* ---- the IMU -------------------------------------------------- */
#if FLUID_IMU_ENABLE
    /* amoled.begin() has already started Wire on the touch pins (SDA 47,
     * SCL 48) and the IMU is on that same bus, so there is nothing to set up
     * beyond the chip's own registers. */
    imuOk = imuBegin();
    if (imuOk)
    {
        for (int i = 0; i < 40 && !gravityOk; i++)     /* first samples */
        {
            imuUpdate();
            delay(5);
        }
    }
#endif

    fluidGravityUpdate(0.0f);

    if (Serial)
    {
        Serial.printf("WaveshareAMOLED 09_IMU_Fluid\n");
        Serial.printf("panel %s, %d bpp, QSPI %lu MHz\n",
                      amoled.controllerName(), amoled.colorDepth(),
                      (unsigned long)(AMOLED_QSPI_CLOCK_HZ / 1000000UL));
        Serial.printf("grid %d x %d, cell %d px, %d cells in the tank, %.0f%% full\n",
                      gridCols, gridRows, cellPx, fluidTankCells(), fluidFillPct);
        Serial.printf("IMU %s\n", imuOk ? "found" : "NOT found - synthetic gravity");
        Serial.printf("one frame is %d kB over the bus; commands: + - r p s\n",
                      (int)((long)W * H * 3 / 1024));
    }

    /* a first frame, so the panel never shows whatever came out of reset */
    fluidPlan();
    fluidBuildFpsLine();
    fluidBuildProfileLine();
    fluidBuildImuLine();

    AMOLED_Canvas &cv = amoled.canvas();
    amoled.beginFrame();
    while (amoled.nextStrip(cv))
    {
        fluidDrawStrip(cv);
        amoled.pushStrip();
    }
    amoled.endFrame();
    amoled.waitIdle();

    fpsStartMs  = millis();
    lastFrameMs = millis();
}

/* ================================================================== */
/* 18. loop() - the whole sketch is one frame of simulation, one frame */
/*     of drawing, and the numbers that say where the time went        */
/* ================================================================== */
void loop()
{
    const uint32_t frameStart = micros();
    const uint32_t nowMs      = millis();

    float dt = (float)(nowMs - lastFrameMs) * 0.001f;
    lastFrameMs = nowMs;
    dt = clampf(dt, 0.0f, 0.10f);      /* after a stall, do not simulate a year */

    /* ---- input ---------------------------------------------------- */
    imuUpdate();                       /* one sample, if the sensor has one */
    touchUpdate();
    fluidGravityUpdate(dt);            /* -> fluidDirX / fluidDirY / fluidMag */

    /* ---- simulate -------------------------------------------------
     * One substep is FLUID_PHYS_MS of simulated time and the number of substeps
     * follows the real frame time, so the liquid moves at the same speed whether
     * the sketch runs at 20 or at 60 fps. */
    int steps = iround(dt * 1000.0f / FLUID_PHYS_MS);
    if (steps < 1)                    steps = 1;
    if (steps > FLUID_MAX_SUBSTEPS)   steps = FLUID_MAX_SUBSTEPS;

    for (int s = 0; s < steps; s++)
    {
        fluidStep(fluidDirX, fluidDirY, fluidMag);
        fluidRelax();
    }
    fluidPaint(dt);

    const uint32_t afterPhys = micros();

    /* ---- the picture ---------------------------------------------- */
    fluidPlan();                       /* grid -> rectangle list */

    hueBase += FLUID_HUE_SPEED * dt;
    if (hueBase >= 360.0f) hueBase -= 360.0f;

    const uint32_t afterPlan = micros();
    uint32_t       drawUs    = 0;

    AMOLED_Canvas &cv = amoled.canvas();
    amoled.beginFrame();
    while (amoled.nextStrip(cv))
    {
        const uint32_t d0 = micros();
        fluidDrawStrip(cv);
        drawUs += micros() - d0;
        amoled.pushStrip();            /* async: the DMA runs while we draw on */
    }
    amoled.endFrame();

    const uint32_t afterDraw = micros();

    /* ---- where did the time go? ----------------------------------- */
    msPhys  = (float)(afterPhys - frameStart) * 0.001f;
    msPlan  = (float)(afterPlan - afterPhys) * 0.001f;
    msDraw  = (float)drawUs * 0.001f;
    msFrame = (float)(afterDraw - frameStart) * 0.001f;
    msDma   = msFrame - msPhys - msPlan - msDraw;
    if (msDma < 0.0f) msDma = 0.0f;

    /* every frame is W * H * 3 bytes over the bus, so this is the bandwidth the
     * panel is really getting */
    mbPerSec = (float)((double)W * (double)H * 3.0 * 0.001 /
                       (double)(msFrame > 0.01f ? msFrame : 0.01f));

    fluidMass = fluidMassTotal();

    fluidBuildProfileLine();

    /* ---- once a second: the numbers, and the serial line ---------- */
    fpsFrames++;
    if ((nowMs - fpsStartMs) >= 1000)
    {
        fps        = fpsFrames * 1000.0f / (float)(nowMs - fpsStartMs);
        fpsFrames  = 0;
        fpsStartMs = nowMs;

        fluidBuildFpsLine();

#if FLUID_SERIAL_FPS
        if (Serial)
        {
            const int tank = fluidTankCells();
            Serial.printf("fps %5.1f | frame %6.2f ms | phys %4.1f plan %3.1f draw %4.1f dma %5.1f"
                          " | %4.1f MB/s | grid %dx%d %d px | liquid %5.1f/%d %3.0f%%"
                          " | rects %3d%s | g %+.2f %+.2f %+.2f | shake %3d%% | pal %d sign %d\n",
                          fps, msFrame, msPhys, msPlan, msDraw, msDma, mbPerSec,
                          gridCols, gridRows, cellPx,
                          fluidMass, tank, 100.0f * fluidMass / (float)tank,
                          rectCount, rectDropped ? " (list full)" : "",
                          gravityDisp.x, gravityDisp.y, gravityDisp.z,
                          iround(shakeLevel * 100.0f), paletteMode, imuSignIdx);
        }
#endif
    }

    /* the IMU readout a few times a second - it does not need 60 */
    static uint32_t imuTextMs = 0;
    if ((nowMs - imuTextMs) >= IMU_TEXT_MS)
    {
        imuTextMs = nowMs;
        fluidBuildImuLine();
    }

    /* ---- serial commands ------------------------------------------ */
#if FLUID_SERIAL_CMDS
    if (Serial)
    {
        while (Serial.available() > 0)
        {
            const int c = Serial.read();
            if      (c == '+') fluidReset(fluidFillPct + 5.0f);
            else if (c == '-') fluidReset(fluidFillPct - 5.0f);
            else if (c == 'r') fluidReset(fluidFillPct);
            else if (c == 'p') paletteMode = (uint8_t)((paletteMode + 1) % 3);
            else if (c == 's') { imuSignIdx = (imuSignIdx + 1) & 3; gravityOk = false; }
        }
    }
#endif

    /* ---- the frame rate cap ---------------------------------------
     * The panel is written while it scans out and this board has no tear-effect
     * pin, so staying at or below the refresh rate keeps the seam to one line
     * (GUIDE.md, Part A6). */
#if FLUID_FPS_CAP > 0
    {
        const uint32_t targetUs = 1000000UL / (uint32_t)FLUID_FPS_CAP;
        const uint32_t spentUs  = micros() - frameStart;
        if (spentUs < targetUs) delay((targetUs - spentUs + 999) / 1000);
    }
#endif
}




