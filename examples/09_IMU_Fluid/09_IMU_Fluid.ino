/*
 * 09_IMU_Fluid - WaveshareAMOLED library
 * ============================================================================
 *  A tank of neon liquid inside the screen that always runs to the low side of
 *  the display, and sloshes while you move it.
 *
 *  This is a real liquid solver on a grid - not a pile of glowing blobs, and
 *  not a "hand the mass to the neighbour" relaxation.  The screen is cut into
 *  FLUID_GRID_COLS x FLUID_GRID_COLS cells; every cell holds a *depth* of
 *  liquid in pixels, and every edge between two cells holds a *flow*: the
 *  amount of liquid that crossed that edge during the last step.
 *
 *  One step of the solver is the classic pipe model of water in a sandbox:
 *
 *      flux  = flux * FLUID_DAMP
 *            + FLUID_FLOW * ( potential(this cell) - potential(neighbour) )
 *      depth = depth - what left the cell + what arrived
 *
 *  with
 *
 *      potential = depth + how far down the gravity vector that cell sits
 *
 *  The potential is the whole trick, and it is why this behaves like a liquid
 *  in *every* direction instead of only along the four grid axes:
 *
 *    - the depth difference is pressure, and it flattens the surface,
 *    - the gravity term is a *continuous* function of the tilt, so at any angle
 *      the liquid runs to the low side and the surface settles perpendicular to
 *      g: a straight waterline at 17 degrees, a wave that rolls when you turn
 *      the display over, all from the same two lines of code,
 *    - the flow remembers what it did last step, which is the liquid's
 *      momentum.  That is what makes it *slosh*: it overshoots the level, comes
 *      back, and rings itself out - the thing a "give the mass to the
 *      neighbour" model can never do, because diffusion has no inertia at all.
 *
 *  A speed limit on every flow, a "never pour out more than you hold" limiter
 *  and a touch of surface tension keep that stable and keep the exact amount of
 *  liquid in the tank: nothing is created, lost, or left inside the wall.
 *
 *  WHY IT LOOKS LIKE A LIQUID AND NOT LIKE BLOCKS
 *  ---------------------------------------------
 *  The simulation is a grid, the picture is not - and the renderer does not know
 *  about gravity at all.  Per frame:
 *
 *    1. the depth of every cell (and of the empty border around the tank, which
 *       is zero) becomes one byte per cell,
 *    2. every pixel of the screen is a *bilinear* sample of that field, so a
 *       12 pixel cell becomes a smoothly curving surface, and the waterline is
 *       simply where the field crosses zero - a straight line at whatever angle
 *       the display is held at,
 *    3. the sample is turned into a colour by a 64 entry table which is built
 *       once per frame: dark where the liquid is deep, bright where it is thin,
 *       and hot along the very edge - which is what draws the meniscus along
 *       the waterline and keeps a thin film visible instead of fading out in
 *       steps.
 *
 *  Nothing is drawn as a rectangle and no cell is ever visible.  The only
 *  geometry that is not the field is a droplet (see FLUID_DROPS): what a hard
 *  shake throws out of the pool, drawn as a small filled circle.
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
 *      drag                stir the liquid: the finger pushes it away from
 *                          itself and slowly paints new liquid
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
 *  Three things keep this sketch near that ceiling instead of far below it:
 *
 *    1. streaming mode.  The scene is drawn into 32 line strips of internal DMA
 *       RAM and each strip is sent while the next one is drawn, so the pixels
 *       never leave the chip and no PSRAM is involved at all (this sketch does
 *       not need the 651 kB PSRAM framebuffer the other examples use).
 *    2. the solver is a byte sized field of 40 x 40 cells plus two arrays of
 *       flows - a few microseconds per step, so it can run several steps per
 *       frame and still be invisible next to the bus.
 *    3. the pixels are written straight into the strip buffer through
 *       canvas.ptr() with a precomputed bilinear filter and a colour table:
 *       217,000 pixels cost a couple of milliseconds and no drawing calls at
 *       all.
 *
 *  The readout at the top of the screen (and the serial line once a second)
 *  prints where the time actually goes:
 *
 *      FPS   frames per second, measured over the last second
 *      PHY   the solver: every step of the frame (ms)
 *      FLD   turning the grid into the depth field and the colour table (ms)
 *      DRW   writing the strips on the CPU (ms)
 *      DMA   the rest of the frame: the QSPI transfers, the part that overlaps
 *            the drawing, and waiting for a free strip buffer (ms)
 *      MB/s  W * H * 3 bytes / frame time - the achieved bus bandwidth.  This is
 *            the number to look at first: ~33 MB/s means the panel really is on
 *            an 80 MHz QSPI bus, ~16 MB/s means 40 MHz, and much less than that
 *            means something else (a slower clone of the panel, USB serial
 *            traffic, an unstable clock) is in the way.
 *
 *  TUNING - the knobs that change the look and the feel most
 *  ---------------------------------------------------------
 *      FLUID_GRID_COLS  the resolution of the simulation.  40 cells across
 *                       means 12 px cells; 64 is a finer liquid (2.5x the
 *                       solver cost, which is still small); 24 is coarse in the
 *                       *physics* - the picture stays smooth either way.
 *      FLUID_FILL_PCT   how much liquid there is, in percent of the tank.  20
 *                       is a thin layer at the bottom, 60 sloshes against the
 *                       top of the round wall.  It can also be changed at
 *                       runtime (see the serial commands below).
 *      FLUID_FLOW       how fast the liquid answers a height difference - the
 *                       speed of a wave in it.  0.15 is syrup, 0.5 is water,
 *                       0.8 and up makes the surface ring.
 *      FLUID_DAMP       the friction: 0.996 keeps it sloshing for seconds,
 *                       0.95 dies in half a second, 0.999 rings for a long time.
 *      FLUID_SHAKE_KICK how violently a shake throws the liquid around.
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
#define FLUID_SHOW_PROFILE   1      /* PHY / FLD / DRW / DMA / MB/s readout */
#define FLUID_SHOW_HINT      1
#define FLUID_SHOW_IMU       1
#define FLUID_SHOW_RING      0     /* the line around the round tank      */
#define FLUID_SHOW_GRID      0      /* 1 = show the simulation cells       */
#define FLUID_SERIAL_FPS     1
#define FLUID_SERIAL_CMDS    1      /* '+' '-' 'r' 'p' 's' in the monitor  */

/* ================================================================== */
/*  2. The grid - the resolution of the simulation                     */
/* ================================================================== */
#define FLUID_GRID_COLS      20     /* cells across the screen (16 .. 72)  */
#define FLUID_GRID_MAX       36     /* do not raise: the arrays are sized  */
#define FLUID_FILL_PCT       60.0f  /* how much liquid, in % of the tank   */
#define FLUID_FILL_FROM_TOP  1      /* 1 = fill from the top instead       */
#define FLUID_TANK_ROUND     1      /* 1 = the visible disc, 0 = rectangle */
#define FLUID_TANK_RADIUS    (AMOLED_SAFE_RADIUS-2)

/* ================================================================== */
/*  3. The physics of the solver                                       */
/* ================================================================== */
/*  The classic "pipe model" of water in a sandbox: every cell holds a   */
/*  depth, every edge between two cells holds a flow, and one step is    */
/*                                                                    */
/*    1. every flow is accelerated by the difference in potential across */
/*       its edge and keeps a fraction of itself from the step before,   */
/*    2. every cell then loses what leaves it and gains what arrives.   */
/*                                                                    */
/*  The potential is "depth + how far down the gravity vector that cell */
/*  sits", and that is the whole trick: the depth part is pressure (it  */
/*  flattens the surface) and the gravity part is a *continuous*        */
/*  function of the tilt, so the liquid runs to the low side at any     */
/*  angle and not only along the four grid axes.  The flow remembering  */
/*  its previous value is the water's *momentum* - which is what makes  */
/*  it slosh, overshoot and ring out; a plain "hand the mass to the     */
/*  neighbours" rule can never do that, it only diffuses.               */
/*                                                                    */
/*  Everything is in pixels of liquid, so the numbers are easy to       */
/*  reason about: one edge, one cell, and a fully tilted display puts   */
/*  one cell of head into every edge.                                  */
/* ================================================================== */
#define FLUID_FLOW           0.50f  /* how fast a head difference turns into
                                     * a flow - this is the speed of a wave
                                     * in the liquid.  0.15 is thick like
                                     * honey, 0.5 is water, above 0.8 the
                                     * surface starts to ring badly.       */
#define FLUID_DAMP           0.996f /* flow kept from one step to the next:
                                     * 1.0 = no friction at all, 0.9 = the
                                     * sloshing dies within a second       */
#define FLUID_MAX_FLUX       0.60f  /* speed limit: one edge may not pass
                                     * more than this fraction of a cell per
                                     * step, whatever the head says       */
#define FLUID_TENSION        0.030f /* surface tension - it rounds off the
                                     * grid staircase and damps the grid
                                     * scale ripples. 0 = off, 0.1 is
                                     * noticeably thicker                 */
#define FLUID_DEPTH_MAX      600.0f /* runaway net, in pixels of head - it
                                     * has to sit above a *full* tank or it
                                     * silently eats the liquid that arrives in
                                     * the deepest cells and the pool never
                                     * settles: a full 466 px tank is 466 px of
                                     * head on the floor cell, so this is that
                                     * plus room for a shake.  It is not a
                                     * physics limit, nothing sane reaches it */
#define FLUID_PHYS_MS        16.6f  /* one step is this much simulated time */
#define FLUID_MAX_STEPS      3      /* ... and a frame runs at most this many */
#define FLUID_GRAVITY        1.0f   /* 1 = the tilt as measured, 0 = ignore   */
#define FLUID_SHAKE_DIR      0.85f  /* how much of a shake turns into "down"  */
#define FLUID_SHAKE_BOOST    1.20f  /* extra pull while being shaken         */
#define FLUID_SHAKE_KICK     0.55f  /* a shake also throws the liquid up and
                                     * sideways, in units of the maximum
                                     * flow: 0 = a shake only tilts the tank */
#define FLUID_SPLASH_MIN     0.22f  /* shake needed before anything flies    */

/* ================================================================== */
/*  4. The look                                                        */
/* ================================================================== */
/*  Every frame the sketch builds one 64 entry colour table out of the   */
/*  depth of the liquid: entry 0 is the empty screen and the last entry  */
/*  is the deepest liquid there is, so the table *is* the picture and    */
/*  the pixel loop is a table lookup.                                    */
#define FLUID_PALETTE        1      /* 0 constant colour, 1 RGB by depth,
                                     * 2 thermal - 'tap' cycles it too       */
#define FLUID_R              0      /* used by palettes 0 and 2              */
#define FLUID_G              170
#define FLUID_B              255
#define FLUID_SHADE_FLOOR    0.32f  /* brightness of the deepest liquid:
                                     * below that a pool stops reading as
                                     * one body                            */
#define FLUID_DEPTH_FULL     120.0f /* depth, in pixels, that counts as "as
                                     * deep as it gets".  The shading follows
                                     * the deepest cell in the tank until it
                                     * reaches this, so a shallow puddle still
                                     * uses the whole table                 */
#define FLUID_RIM            0.18f  /* where the meniscus highlight sits, as
                                     * a fraction of the table - it is what
                                     * draws the bright line along the water-
                                     * line and the edge of a thin film    */
#define FLUID_RIM_GAIN       190    /* how much white is added there, 0 = off */
#define FLUID_EDGE_AA        3      /* how many table entries the surface
                                     * fades in over: 0 = a hard pixel edge,
                                     * 4 = a soft one                      */
#define FLUID_HUE_DEPTH      130.0f /* how far the hue travels between a
                                     * shallow and a deep cell, in degrees  */
#define FLUID_HUE_SPEED      10.0f  /* degrees per second of hue drift     */
#define FLUID_LUT_SHIFT      2      /* the table has 256 >> this entries   */
#define FLUID_BG             0x000000u /* what the empty screen is         */

/* ---- droplets: what a hard shake throws out of the pool ------------ */
#define FLUID_DROPS          1      /* 1 = a real shake really sprays      */
#define FLUID_DROP_MAX       48     /* never more than this many at once   */
#define FLUID_DROP_RATE      0.75f  /* droplets per frame and unit of shake */
#define FLUID_DROP_MASS      14.0f  /* liquid per droplet, in pixels of depth */
#define FLUID_DROP_MIN       6.0f   /* a cell needs at least this much liquid
                                     * before it may give a droplet away   */
#define FLUID_DROP_LIFE      4.0f   /* seconds before a stranded one gives up */
#define FLUID_DROP_LAUNCH    170.0f /* px/s of "up" a new droplet starts with */
#define FLUID_DROP_SPREAD    110.0f /* px/s of randomness on top of that   */
#define FLUID_DROP_G         900.0f /* px/s^2 of acceleration, at 1 g      */

/* ================================================================== */
/*  5. Touch                                                           */
/* ================================================================== */
#define FLUID_TOUCH_ENABLE   1
#define FLUID_TOUCH_R        70     /* how far the finger's "gravity" reaches  */
#define FLUID_TOUCH_PUSH     3.0f   /* how hard it pushes away from the finger */
#define FLUID_TOUCH_PAINT    30.0f  /* pixels of depth painted per second, with
                                     * the finger as the tap: the same knob as
                                     * FLUID_FILL_PCT, live.  0 = the finger
                                     * only ever pushes, never adds          */

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
/*  7. The grid, the field, the colour table and the frame state       */
/*                                                                    */
/*  Every grid array is padded by one cell on each side, so the         */
/*  neighbour lookups in the hot loops never need a bounds test: a       */
/*  border cell is simply never "inside", and every flow that tries to   */
/*  cross it stays zero - which is exactly what a glass wall does.       */
/*                                                                    */
/*      gridDepth   the liquid column above the cell, in pixels         */
/*      gridFluxX   the flow through the cell's LEFT edge, + = right    */
/*      gridFluxY   the flow through the cell's TOP  edge, + = down     */
/*      gridInside  1 = the cell is inside the tank                     */
/*      gridField   the depth as one byte - what the renderer samples   */
/* ================================================================== */
#define GRID_STRIDE      (FLUID_GRID_MAX + 2)
#define GRID_CELLS       (GRID_STRIDE * GRID_STRIDE)

static float    gridDepth[GRID_CELLS];    /* liquid in the cell, in pixels */
static float    gridFluxX[GRID_CELLS];    /* left edge, + = to the right  */
static float    gridFluxY[GRID_CELLS];    /* top  edge, + = downwards     */
static uint8_t  gridField[GRID_CELLS];    /* depth as 0..255              */
static uint8_t  gridInside[GRID_CELLS];   /* 1 = this cell is in the tank */

/* cell (x, y) in the padded array; only x/y in 0..gridCols-1 / 0..gridRows-1
 * are ever used, everything outside stays 0 = outside the tank */
static inline int gidx(int x, int y) { return (y + 1) * GRID_STRIDE + (x + 1); }

static int      gridCols = 0, gridRows = 0, cellPx = 0;

/* Where a pixel of the screen samples the field: the cell to its left/top and
 * the 0..255 position between that cell's centre and the next.  That is the
 * whole bilinear filter, precomputed once per grid build so that the inner loop
 * of the renderer is one multiply and one shift per pixel. */
static uint8_t  pxCellX[AMOLED_WIDTH],  pxFracX[AMOLED_WIDTH];
static uint8_t  pxCellY[AMOLED_HEIGHT], pxFracY[AMOLED_HEIGHT];

/* The colour table: entry 0 is the empty screen, the last entry is the deepest
 * liquid there is.  It is rebuilt once per frame (fluidBuildLut), which is why
 * a shallow puddle still gets the full range of shades and nothing jumps. */
#define FLUID_LUT_N     (256 >> FLUID_LUT_SHIFT)
static uint8_t  lutR[FLUID_LUT_N], lutG[FLUID_LUT_N], lutB[FLUID_LUT_N];

/* A droplet a shake threw out of the pool - see section 12b. */
struct FluidDrop { float x, y, vx, vy, mass, life; };
static FluidDrop drops[FLUID_DROP_MAX];
static int       dropCount = 0;
static int       dropLive  = 0;           /* for the readout                */

static float    fieldRef     = 90.0f;     /* depth that "deep" means        */
static float    fluidFillPct = FLUID_FILL_PCT;
static float    fluidMass    = 0.0f;      /* total liquid, px of head       */
static float    fluidWetPct  = 0.0f;      /* how much of the glass it covers */
static float    fluidSpeed   = 0.0f;      /* mean flow, px per step         */

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
static float    msPhys = 0.0f, msField = 0.0f, msDraw = 0.0f, msDma = 0.0f,
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

/* The colour table for this frame: FLUID_LUT_N entries from "nothing at all" to
 * "as deep as this tank gets".  The last few steps towards zero are the
 * meniscus - the bright edge along the waterline - and they are what keeps a
 * thin film and a receding shoreline visible instead of letting them pop out of
 * existence the moment the depth drops below one cell. */
static void fluidBuildLut(void)
{
    for (int k = 0; k < FLUID_LUT_N; k++)
    {
        if (k == 0)                        /* entry 0 = the empty screen */
        {
            lutR[0] = (uint8_t)amoledR(FLUID_BG);
            lutG[0] = (uint8_t)amoledG(FLUID_BG);
            lutB[0] = (uint8_t)amoledB(FLUID_BG);
            continue;
        }

        const float v     = (float)k / (float)(FLUID_LUT_N - 1);   /* 0 thin .. 1 deep */
        const float shade = 1.0f - (1.0f - FLUID_SHADE_FLOOR) * v;
        uint32_t    c;

        switch (paletteMode)
        {
            case 0:   /* one constant colour, its brightness follows the depth */
                c = amoledScale(amoledRGB(FLUID_R, FLUID_G, FLUID_B),
                                (uint16_t)(shade * 255.0f), 255);
                break;

            case 2:   /* thermal: the same base colour, white hot at the edge */
            {
                const uint32_t base = amoledScale(amoledRGB(FLUID_R, FLUID_G, FLUID_B),
                                                  (uint16_t)(shade * 255.0f), 255);
                const uint8_t  hot  = (uint8_t)(clampf((1.0f - v) * (1.0f - v), 0.0f, 1.0f) * 150.0f);
                c = amoledMix(base, AMOLED_WHITE, hot);
                break;
            }

            default:  /* RGB: the hue runs from the deep liquid to the thin edge
                       * of the water and drifts slowly with time */
                c = amoledHSV(hueBase + FLUID_HUE_DEPTH * (1.0f - v), 1.0f,
                              clampf(0.22f + 0.78f * shade, 0.0f, 1.0f));
                break;
        }

        if (FLUID_RIM_GAIN > 0 && FLUID_RIM > 0.0f && v < FLUID_RIM)
        {
            const float rim = 1.0f - v / FLUID_RIM;
            c = amoledMix(c, AMOLED_WHITE, (uint8_t)(rim * (float)FLUID_RIM_GAIN));
        }

        if (FLUID_EDGE_AA > 0)             /* fade the first entries in, so the
                                            * surface has no hard pixel edge */
        {
            const float aa = clampf((float)k / (float)FLUID_EDGE_AA, 0.0f, 1.0f);
            if (aa < 1.0f) c = amoledScale(c, (uint16_t)(aa * 255.0f), 255);
        }

        lutR[k] = (uint8_t)amoledR(c);
        lutG[k] = (uint8_t)amoledG(c);
        lutB[k] = (uint8_t)amoledB(c);
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
/* ================================================================== */
/* One axis of the pixel -> cell map.  A pixel samples between the centres of
 * two cells: p samples the cell at (p + 0.5) / cellPx - 0.5 cell units, split
 * into the cell index and a 0..255 fraction.  Outside the grid the index is
 * clamped onto the empty border cell, which is what makes the edge of the tank
 * a smooth fade to the background instead of a hard cut. */
static void fluidBuildPxMap(int p, int cell, int cells, uint8_t *idx, uint8_t *frac)
{
    const int v256 = ((2 * p + 1) * 128) / cell - 128;     /* in 1/256 cells */
    int c, f;

    if (v256 <= 0) { c = 0; f = 0; }
    else
    {
        c = v256 >> 8;
        f = v256 & 0xFF;
        if (c >= cells) { c = cells - 1; f = 255; }
    }
    idx[p]  = (uint8_t)c;
    frac[p] = (uint8_t)f;
}

static void fluidGridBuild(void)
{
    gridCols = FLUID_GRID_COLS;
    if (gridCols < 4)               gridCols = 4;
    if (gridCols > FLUID_GRID_MAX)  gridCols = FLUID_GRID_MAX;
    cellPx   = (W + gridCols - 1) / gridCols;
    gridRows = (H + cellPx - 1) / cellPx;

    memset(gridInside, 0, sizeof(gridInside));
    memset(gridDepth,  0, sizeof(gridDepth));
    memset(gridFluxX,  0, sizeof(gridFluxX));
    memset(gridFluxY,  0, sizeof(gridFluxY));
    memset(gridField,  0, sizeof(gridField));

    /* which cells hold liquid at all - the round glass, in other words */
    for (int y = 0; y < gridRows; y++)
    for (int x = 0; x < gridCols; x++)
    {
        const float cx = ((float)x + 0.5f) * (float)cellPx - 0.5f;
        const float cy = ((float)y + 0.5f) * (float)cellPx - 0.5f;
        bool inside = (cx >= 0.0f && cy >= 0.0f &&
                       cx <= (float)(W - 1) && cy <= (float)(H - 1));
#if FLUID_TANK_ROUND
        const float dx = cx - (float)CX, dy = cy - (float)CY;
        const float r  = (float)FLUID_TANK_RADIUS;
        if (inside && dx * dx + dy * dy > r * r) inside = false;
#endif
        if (inside) gridInside[gidx(x, y)] = 1;
    }

    for (int x = 0; x < W; x++) fluidBuildPxMap(x, cellPx, gridCols, pxCellX, pxFracX);
    for (int y = 0; y < H; y++) fluidBuildPxMap(y, cellPx, gridRows, pxCellY, pxFracY);
}

static int fluidTankCells(void)
{
    int n = 0;
    for (int i = 0; i < GRID_CELLS; i++) if (gridInside[i]) n++;
    return n;
}

/* Fill the tank with `pct` percent of its volume - a slab of liquid standing on
 * the floor and `h` pixels tall, so the number means the same thing as
 * FLUID_FILL_PCT and the waterline lands where the top of the slab would be.
 * The depth is a head, so the slab is written as the height of the liquid
 * column above each cell: 0 at the surface, `h` down at the floor.  That is the
 * shape the solver rests in, which is why a fresh tank starts out still. */
static void fluidReset(float pct)
{
    memset(gridDepth, 0, sizeof(gridDepth));
    memset(gridFluxX, 0, sizeof(gridFluxX));
    memset(gridFluxY, 0, sizeof(gridFluxY));
    memset(gridField, 0, sizeof(gridField));
    dropCount = 0;
    dropLive  = 0;

    fluidFillPct = clampf(pct, 0.0f, 100.0f);

    const float h = fluidFillPct * 0.01f * (float)H;

    for (int y = 0; y < gridRows; y++)
    for (int x = 0; x < gridCols; x++)
    {
        const int i = gidx(x, y);
        if (!gridInside[i]) continue;

        /* a slab of liquid h pixels tall, standing on the floor of the tank
         * (FLUID_FILL_FROM_TOP: hanging from its ceiling instead).
         *
         * gridDepth is a head, not a thickness: the solver only sits still when
         * the depth grows by one cell per row downwards, because that is what
         * cancels the gravity term across an edge - so the number in a cell is
         * the height of the liquid column above it.  The surface of a slab h
         * tall standing on the floor is therefore at y = H - h, and the depth
         * runs from 0 there to h down at the floor, which is also what makes
         * FLUID_FILL_PCT mean "the waterline is (100 - pct) % down the tank".
         * Filling from the ceiling is the same slab the other way up. */
        const float cy = ((float)y + 0.5f) * (float)cellPx - 0.5f;
        const float d  = FLUID_FILL_FROM_TOP ? cy : (cy - ((float)H - h));

        gridDepth[i] = clampf(d, 0.0f, h);
    }
}

/* How much liquid there is, and how much of the glass it covers - the second
 * number is the one that reads like a fill level.  gridDepth is a head, not a
 * thickness: the solver only rests when the depth grows by a cell per row
 * downwards, so the value in a cell is the height of the liquid column above
 * it, and the sum of those is not a volume.  Counting the cells that actually
 * hold liquid (which is the area under the waterline, so it matches
 * FLUID_FILL_PCT on a still, upright tank) is honest and costs one comparison
 * in the loop that was there anyway. */
static float fluidMeasure(float *wetPct)
{
    float s   = 0.0f;
    int   wet = 0, live = 0;

    for (int i = 0; i < GRID_CELLS; i++)
    {
        if (!gridInside[i]) continue;
        live++;
        s += gridDepth[i];
        if (gridDepth[i] > 0.01f) wet++;
    }

    if (wetPct) *wetPct = live ? 100.0f * (float)wet / (float)live : 0.0f;
    return s;
}

/* ================================================================== */
/* 12. The simulation: one step of the solver                          */
/* ================================================================== */
/*  Two passes, and between them they are the whole physics:           */
/*                                                                    */
/*    1. FLUX.  Every edge looks at the head across it - the difference */
/*       in depth, plus the difference in how far down the gravity      */
/*       vector the two cells sit - and accelerates its flow by that,   */
/*       keeping FLUID_DAMP of what it had.  That memory is the water's */
/*       inertia: without it the liquid can only diffuse, with it, it   */
/*       sloshes.                                                       */
/*    2. FLOW.  Every cell adds up what enters and what leaves over its */
/*       four edges, limiting the outgoing flows so that no cell can    */
/*       pour out more than it holds - which is what lets a nearly     */
/*       empty cell coast instead of going negative.                    */
/*                                                                    */
/*  The gravity term is where the tilt enters, and it is continuous: at */
/*  17 degrees the head across one cell is gx * cellPx, so the surface  */
/*  settles perpendicular to g - at any angle, not only along the axes. */
/*                                                                    */
/*  One step is FLUID_PHYS_MS of simulated time whatever the frame rate */
/*  is, so the liquid moves at the same speed at 20 and at 60 fps.      */
/* ================================================================== */
static void fluidStep(float gx, float gy, float kick)
{
    const float headX = gx * (float)cellPx;      /* head across one cell, px */
    const float headY = gy * (float)cellPx;
    const float fmax  = FLUID_MAX_FLUX * (float)cellPx;

    /* ---- pass 1: accelerate every flow ---------------------------- */
    for (int y = 0; y < gridRows; y++)
    for (int x = 0; x < gridCols; x++)
    {
        const int i = gidx(x, y);
        if (!gridInside[i]) continue;

        /* the left edge: the flow from the cell on the left into this one */
        if (gridInside[i - 1])
        {
            const float f = gridFluxX[i] * FLUID_DAMP +
                            FLUID_FLOW * ((gridDepth[i - 1] - gridDepth[i]) + headX);
            gridFluxX[i] = clampf(f, -fmax, fmax);
        }
        else gridFluxX[i] = 0.0f;                /* the glass wall */

        /* the top edge: the flow from the cell above into this one */
        if (gridInside[i - GRID_STRIDE])
        {
            const float f = gridFluxY[i] * FLUID_DAMP +
                            FLUID_FLOW * ((gridDepth[i - GRID_STRIDE] - gridDepth[i]) + headY);
            gridFluxY[i] = clampf(f, -fmax, fmax);
        }
        else gridFluxY[i] = 0.0f;

        /* ---- a shake throws the liquid around --------------------- */
        if (FLUID_SHAKE_KICK > 0.0f && kick > 0.0f)
        {
            gridFluxX[i] = clampf(gridFluxX[i] + frand() * kick, -fmax, fmax);
            gridFluxY[i] = clampf(gridFluxY[i] + (frand() * 0.5f - 0.9f) * kick, -fmax, fmax);
        }

        /* ---- the finger stirs it --------------------------------- */
#if FLUID_TOUCH_ENABLE
        if (touchOn && FLUID_TOUCH_PUSH > 0.0f)
        {
            const float cx = ((float)x + 0.5f) * (float)cellPx;
            const float cy = ((float)y + 0.5f) * (float)cellPx;
            const float dx = cx - (float)touchX, dy = cy - (float)touchY;
            const float d2 = dx * dx + dy * dy;
            const float r  = (float)FLUID_TOUCH_R;

            if (d2 < r * r)
            {
                const float d   = sqrtf(d2) + 1.0f;
                const float amp = FLUID_TOUCH_PUSH * (1.0f - d / r) * fmax * 0.5f;
                gridFluxX[i] = clampf(gridFluxX[i] + amp * dx / d, -fmax, fmax);
                gridFluxY[i] = clampf(gridFluxY[i] + amp * dy / d, -fmax, fmax);
            }
        }
#endif
    }

    /* ---- pass 2: move the liquid ---------------------------------- */
    float speedSum = 0.0f;
    int   speedN   = 0;

    for (int y = 0; y < gridRows; y++)
    for (int x = 0; x < gridCols; x++)
    {
        const int i = gidx(x, y);
        if (!gridInside[i]) continue;

        float inX  = gridFluxX[i];                   /* from the left  */
        float outX = gridFluxX[i + 1];               /* to the right   */
        float inY  = gridFluxY[i];                   /* from above     */
        float outY = gridFluxY[i + GRID_STRIDE];     /* to below       */

        /* never pour out more than the cell actually holds */
        float out = 0.0f;
        if (outX > 0.0f) out += outX;
        if (outY > 0.0f) out += outY;
        if (inX  < 0.0f) out -= inX;
        if (inY  < 0.0f) out -= inY;

        const float d = gridDepth[i];

        if (out > d && out > 1e-6f)
        {
            const float k = d / out;                 /* the whole cell coasts now */
            if (outX > 0.0f) { outX *= k; gridFluxX[i + 1]           = outX; }
            if (outY > 0.0f) { outY *= k; gridFluxY[i + GRID_STRIDE] = outY; }
            if (inX  < 0.0f) { inX  *= k; gridFluxX[i]               = inX;  }
            if (inY  < 0.0f) { inY  *= k; gridFluxY[i]               = inY;  }
        }

        gridDepth[i] = d + (inX - outX) + (inY - outY);
        gridDepth[i] = clampf(gridDepth[i], 0.0f, FLUID_DEPTH_MAX);

        /* how fast the liquid moves, for the readout: if this is zero the tank
         * is at rest, whatever the picture happens to look like */
        speedSum += fabsf(inX) + fabsf(outX) + fabsf(inY) + fabsf(outY);
        speedN++;
    }

    if (speedN > 0) fluidSpeed = speedSum / (float)(speedN * 4);
}

/* Surface tension, done as a plain exchange between neighbours: a little liquid
 * moves from the fuller cell to the emptier one.  One multiply per edge, exactly
 * conservative, and it is what rounds off the grid staircase and damps the cell
 * sized ripples - the difference between a surface that reads as water and one
 * that reads as a chessboard with waves on it. */
static void fluidTension(void)
{
    if (FLUID_TENSION <= 0.0f) return;       /* the switch is off */
    for (int y = 0; y < gridRows; y++)
    for (int x = 0; x < gridCols; x++)
    {
        const int i = gidx(x, y);
        if (!gridInside[i]) continue;

        for (int k = 0; k < 2; k++)              /* right, then below */
        {
            const int n = k ? (i + GRID_STRIDE) : (i + 1);
            if (!gridInside[n]) continue;

            float f = (gridDepth[i] - gridDepth[n]) * FLUID_TENSION * 0.25f;
            if (f > 0.0f)
            {
                if (f > gridDepth[i]) f = gridDepth[i];   /* never go negative */
                gridDepth[i] -= f;
                gridDepth[n] += f;
            }
            else if (f < 0.0f)
            {
                if (-f > gridDepth[n]) f = -gridDepth[n];
                gridDepth[i] -= f;
                gridDepth[n] += f;
            }
        }
    }
}

/* ================================================================== */
/* 12c. Droplets - what a hard shake throws out of the pool            */
/*                                                                    */
/*  A droplet is a little liquid that leaves the grid for a moment: it  */
/*  takes its mass out of the cell it starts in, flies as a ballistic   */
/*  body under the *same* gravity the tank feels, bounces off the glass */
/*  and gives the mass back to whatever cell it lands in.  The total    */
/*  amount of liquid therefore never changes - it is only in the air    */
/*  for a moment, which is exactly what a splash is.                    */
/* ================================================================== */
static void fluidDropAdd(int cell, float mass)
{
    gridDepth[cell] += mass;
    if (gridDepth[cell] > FLUID_DEPTH_MAX) gridDepth[cell] = FLUID_DEPTH_MAX;

    /* landing makes a ripple: a small kick on the edge it came through */
    gridFluxY[cell] = clampf(gridFluxY[cell] - mass * 0.5f,
                             -FLUID_MAX_FLUX * (float)cellPx,
                              FLUID_MAX_FLUX * (float)cellPx);
}

static void fluidDropSpawn(float gx, float gy, float shake)
{
#if FLUID_DROPS
    static float pending = 0.0f;

    pending += FLUID_DROP_RATE * shake * shake;

    while (pending >= 1.0f && dropCount < FLUID_DROP_MAX)
    {
        pending -= 1.0f;

        /* look for a shallow cell to take the droplet from: a few random tries
         * is plenty, and it keeps the drops spread over the whole surface */
        int best = 0;
        for (int t = 0; t < 8; t++)
        {
            const int x = random(0, gridCols);
            const int y = random(0, gridRows);
            const int c = gidx(x, y);
            if (gridInside[c] && gridDepth[c] >= FLUID_DROP_MIN) { best = c; break; }
        }
        if (!best) break;

        /* where on the screen is that cell? (undo gidx) */
        int x, y;
        {
            const int off = best - GRID_STRIDE - 1;
            x = off % GRID_STRIDE;
            y = off / GRID_STRIDE;
        }

        FluidDrop &d = drops[dropCount++];
        const float take = (gridDepth[best] < FLUID_DROP_MASS) ? gridDepth[best]
                                                              : FLUID_DROP_MASS;

        d.x    = ((float)x + 0.5f) * (float)cellPx;
        d.y    = ((float)y + 0.5f) * (float)cellPx;
        d.vx   = -gx * FLUID_DROP_LAUNCH * 0.6f + frand() * FLUID_DROP_SPREAD * 0.5f;
        d.vy   = -gy * FLUID_DROP_LAUNCH
                 + frand() * FLUID_DROP_SPREAD * 0.5f
                 - 0.35f * FLUID_DROP_LAUNCH;
        d.mass = take;
        d.life = FLUID_DROP_LIFE;

        gridDepth[best] -= take;                /* the pool pays for it */
    }
#else
    (void)gx; (void)gy; (void)shake;
#endif
}

static void fluidDrops(float dt, float gx, float gy)
{
#if FLUID_DROPS
    const float lim = (float)FLUID_TANK_RADIUS;

    for (int i = 0; i < dropCount; )
    {
        FluidDrop &d = drops[i];

        d.life -= dt;
        d.vx += gx * FLUID_DROP_G * dt;
        d.vy += gy * FLUID_DROP_G * dt;
        d.x  += d.vx * dt;
        d.y  += d.vy * dt;

        /* -- the glass: bounce off the round wall and off the edges --- */
        const float rx = d.x - (float)CX, ry = d.y - (float)CY;
        const float r2 = rx * rx + ry * ry;
        bool outside = (d.x < 1.0f || d.x > (float)(W - 2) ||
                        d.y < 1.0f || d.y > (float)(H - 2));

        if (FLUID_TANK_ROUND && r2 > lim * lim) outside = true;

        if (outside)
        {
            if (FLUID_TANK_ROUND && r2 > lim * lim && r2 > 1e-3f)
            {
                const float r  = sqrtf(r2);
                const float nx = rx / r, ny = ry / r;
                const float vn = d.vx * nx + d.vy * ny;
                d.vx -= 1.6f * vn * nx;         /* reflect, lose a little */
                d.vy -= 1.6f * vn * ny;
                d.x   = (float)CX + nx * (lim - 1.0f);
                d.y   = (float)CY + ny * (lim - 1.0f);
            }
            if      (d.x < 1.0f)             { d.x = 1.0f;             d.vx = -d.vx * 0.6f; }
            else if (d.x > (float)(W - 2))   { d.x = (float)(W - 2);   d.vx = -d.vx * 0.6f; }
            if      (d.y < 1.0f)             { d.y = 1.0f;             d.vy = -d.vy * 0.6f; }
            else if (d.y > (float)(H - 2))   { d.y = (float)(H - 2);   d.vy = -d.vy * 0.6f; }
        }

        /* -- is it back in the liquid? -------------------------------- */
        int cx = (int)d.x / cellPx, cy = (int)d.y / cellPx;
        if (cx < 0) cx = 0;  if (cy < 0) cy = 0;
        if (cx >= gridCols) cx = gridCols - 1;
        if (cy >= gridRows) cy = gridRows - 1;

        const int  c    = gidx(cx, cy);
        const bool land = gridInside[c] && gridDepth[c] >= 1.5f;
        bool       gone = (d.life <= 0.0f) || land;

        if (!gone) { i++; continue; }

        /* give the mass back where it is, so nothing is ever lost */
        if (gridInside[c])
        {
            fluidDropAdd(c, d.mass);
        }
        else if (cx == 0 && cy == 0)
        {
            /* it died outside the glass entirely: hand the mass to any cell of
             * the tank, so the total never drifts */
            for (int k = 0; k < GRID_CELLS; k++)
                if (gridInside[k]) { fluidDropAdd(k, d.mass); break; }
        }

        drops[i] = drops[--dropCount];          /* swap-remove */
    }
#endif
    dropLive = dropCount;
}

/* Liquid under the finger, while it is down: how you change the amount without a
 * recompile, and the same knob as FLUID_FILL_PCT.  This is the only place in the
 * sketch that creates liquid - everything else is exact bookkeeping. */
static void fluidPaint(float dt)
{
#if FLUID_TOUCH_ENABLE
    if (!touchOn || FLUID_TOUCH_PAINT <= 0.0f) return;

    static const int8_t pdx[5] = { 0, -1, +1,  0,  0 };
    static const int8_t pdy[5] = { 0,  0,  0, -1, +1 };
    const int cx  = touchX / cellPx, cy = touchY / cellPx;
    const int rad = FLUID_TOUCH_R / (2 * cellPx) + 1;

    for (int k = 0; k < 5; k++)
    {
        const int x = cx + pdx[k] * rad, y = cy + pdy[k] * rad;
        if (x < 0 || y < 0 || x >= gridCols || y >= gridRows) continue;

        const int i = gidx(x, y);
        if (!gridInside[i]) continue;

        gridDepth[i] += FLUID_TOUCH_PAINT * dt * 0.25f;
        if (gridDepth[i] > FLUID_DEPTH_MAX) gridDepth[i] = FLUID_DEPTH_MAX;
    }
#else
    (void)dt;
#endif
}

/* ================================================================== */
/* 13. From the grid to the picture                                    */
/* ================================================================== */
/*  Every cell's depth becomes one byte of the field, and the field is  */
/*  what the renderer samples: per pixel, bilinearly, straight into a   */
/*  colour table entry.  The scale is picked so that the deepest cell of */
/*  the frame uses the whole table, which is why even a shallow puddle   */
/*  shows the full range of shades; that reference is smoothed over time */
/*  so the picture does not flicker when a wave passes under one cell.    */
/* ================================================================== */
static void fluidField(void)
{
    float deepest = 0.0f;
    for (int i = 0; i < GRID_CELLS; i++)
        if (gridInside[i] && gridDepth[i] > deepest) deepest = gridDepth[i];

    if (deepest > FLUID_DEPTH_FULL) deepest = FLUID_DEPTH_FULL;

    fieldRef += (deepest - fieldRef) * 0.05f;
    if (fieldRef < 12.0f) fieldRef = 12.0f;

    const float k = 255.0f / fieldRef;

    memset(gridField, 0, sizeof(gridField));

    for (int y = 0; y < gridRows; y++)
    for (int x = 0; x < gridCols; x++)
    {
        const int i = gidx(x, y);
        if (!gridInside[i]) continue;

        const int v = (int)(gridDepth[i] * k + 0.5f);
        gridField[i] = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
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
    /* the cells the solver really uses - the quickest way to see what
     * FLUID_GRID_COLS does */
    for (int x = cellPx; x < W; x += cellPx)
        cv.fillRect(x - 1, 0, x - 1, H - 1, 0x1A1A1A);
    for (int y = cellPx; y < H; y += cellPx)
        cv.fillRect(0, y - 1, W - 1, y - 1, 0x1A1A1A);
#endif

    /* ---- the liquid: one bilinear sample of the field per pixel ----
     * No rectangles and no cell edges: the depth is read between four cells and
     * blended, so the surface is smooth to the pixel and moves by fractions of a
     * cell.  The colour is a table lookup - entry 0 is the background, so an
     * empty pixel is not even written. */
    int x0 = cv.windowX0(), x1 = cv.windowX1();
    int y0 = cv.windowY0(), y1 = cv.windowY1();
    if (x0 < 0)     x0 = 0;
    if (y0 < 0)     y0 = 0;
    if (x1 > W - 1) x1 = W - 1;
    if (y1 > H - 1) y1 = H - 1;

    for (int y = y0; y <= y1; y++)
    {
        /* the two cell rows this pixel sits between, already offset into the
         * padded array */
        const uint8_t *rowA = gridField + (pxCellY[y] + 1) * GRID_STRIDE + 1;
        const uint8_t *rowB = rowA + GRID_STRIDE;
        const int      fy   = pxFracY[y];
        const int      iy   = 256 - fy;

        uint8_t *p = cv.ptr(x0, y);

        for (int x = x0; x <= x1; x++)
        {
            const int cx = pxCellX[x];
            const int fx = pxFracX[x];
            const int ix = 256 - fx;

            /* bilinear: along the row first, then between the rows.  Both are
             * fixed point with 8 fractional bits, hence the two shifts. */
            const int a = rowA[cx] * ix + rowA[cx + 1] * fx;   /* 0 .. 65280 */
            const int b = rowB[cx] * ix + rowB[cx + 1] * fx;
            const int k = ((a * iy + b * fy) >> 16) >> FLUID_LUT_SHIFT;

            if (k) amoledPxSet(p, lutR[k], lutG[k], lutB[k]);

            p += cv.strideX();
        }
    }

    /* ---- the droplets a shake threw into the air ------------------ */
#if FLUID_DROPS
    {
        int rim = (int)(FLUID_RIM * (float)(FLUID_LUT_N - 1));
        if (rim < 1) rim = 1;
        if (rim > FLUID_LUT_N - 1) rim = FLUID_LUT_N - 1;

        const uint32_t dropCol = amoledRGB(lutR[rim], lutG[rim], lutB[rim]);

        for (int i = 0; i < dropCount; i++)
        {
            const FluidDrop &d = drops[i];
            if (d.y < (float)y0 - 3.0f || d.y > (float)y1 + 3.0f) continue;
            if (d.x < (float)x0 - 3.0f || d.x > (float)x1 + 3.0f) continue;

            cv.fillCircle(iround(d.x), iround(d.y),
                          (d.mass > FLUID_DROP_MASS * 0.5f) ? 3 : 2, dropCol);
        }
    }
#endif

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
             "PHY %.1f FLD %.1f DRW %.1f DMA %.1f", msPhys, msField, msDraw, msDma);
}

static void fluidBuildImuLine(void)
{
    if (!FLUID_SHOW_IMU) { imuLine[0] = 0; return; }

    if (!imuOk || !gravityOk)
    {
        snprintf(imuLine, sizeof(imuLine), "NO IMU - SYNTHETIC GRAVITY");
        return;
    }

    const int8_t *s = kImuSigns[imuSignIdx];
    snprintf(imuLine, sizeof(imuLine),
             "G %+.2f %+.2f %+.2f  SIGN %d %c%c%c  SLOSH %.1f  SHAKE %d%%",
             gravityDisp.x, gravityDisp.y, gravityDisp.z, imuSignIdx,
             s[0] > 0 ? '+' : '-', s[1] > 0 ? '+' : '-', s[2] > 0 ? '+' : '-',
             fluidSpeed, iround(shakeLevel * 100.0f));
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
        Serial.printf("solver: flow %.2f damp %.3f flux %.2f tension %.3f, %d step(s) per frame\n",
                      (double)FLUID_FLOW, (double)FLUID_DAMP, (double)FLUID_MAX_FLUX,
                      (double)FLUID_TENSION, (int)FLUID_MAX_STEPS);
        Serial.printf("renderer: bilinear field %d x %d, %d entry colour table\n",
                      gridCols, gridRows, (int)FLUID_LUT_N);
        Serial.printf("IMU %s\n", imuOk ? "found" : "NOT found - synthetic gravity");
        Serial.printf("one frame is %d kB over the bus; commands: + - r p s\n",
                      (int)((long)W * H * 3 / 1024));
    }

    /* a first frame, so the panel never shows whatever came out of reset */
    fluidField();
    fluidBuildLut();
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
    fluidGravityUpdate(dt);            /* -> fluidDirX / fluidDirY / fluidMag
                                        *    and shakeLevel                   */

    /* ---- simulate -------------------------------------------------
     * One step is FLUID_PHYS_MS of simulated time and the number of steps follows
     * the real frame time, so the liquid moves at the same speed whether the
     * sketch runs at 20 or at 60 fps. */
    int steps = iround(dt * 1000.0f / FLUID_PHYS_MS);
    if (steps < 1)                  steps = 1;
    if (steps > FLUID_MAX_STEPS)    steps = FLUID_MAX_STEPS;

    /* the gravity the solver sees, in pixels of head per cell: the fall
     * direction times how hard it is pulling (1.0 = the board upright) */
    const float gx = fluidDirX * fluidMag;
    const float gy = fluidDirY * fluidMag;

    /* a shake is a violent, changing acceleration: it kicks the flows about, and
     * past FLUID_SPLASH_MIN it also starts throwing droplets around */
    float kick = 0.0f;
    if (FLUID_SHAKE_KICK > 0.0f && shakeLevel > FLUID_SPLASH_MIN)
    {
        kick = FLUID_SHAKE_KICK * (shakeLevel - FLUID_SPLASH_MIN) /
               (1.0f - FLUID_SPLASH_MIN) * FLUID_MAX_FLUX * (float)cellPx;
    }

    for (int s = 0; s < steps; s++)
    {
        fluidStep(gx, gy, kick);
        fluidTension();
    }

    fluidPaint(dt);                    /* the finger adds liquid, if enabled */
    fluidDropSpawn(fluidDirX, fluidDirY, shakeLevel);
    fluidDrops(dt, gx, gy);

    const uint32_t afterPhys = micros();

    /* ---- the picture ---------------------------------------------- */
    hueBase += FLUID_HUE_SPEED * dt;
    if (hueBase >= 360.0f) hueBase -= 360.0f;

    fluidField();                      /* grid -> the byte field the renderer reads */
    fluidBuildLut();                   /* this frame's colours                     */

    const uint32_t afterField = micros();
    uint32_t       drawUs     = 0;

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
    msField = (float)(afterField - afterPhys) * 0.001f;
    msDraw  = (float)drawUs * 0.001f;
    msFrame = (float)(afterDraw - frameStart) * 0.001f;
    msDma   = msFrame - msPhys - msField - msDraw;
    if (msDma < 0.0f) msDma = 0.0f;

    /* every frame is W * H * 3 bytes over the bus, so this is the bandwidth the
     * panel is really getting */
    mbPerSec = (float)((double)W * (double)H * 3.0 * 0.001 /
                       (double)(msFrame > 0.01f ? msFrame : 0.01f));

    fluidMass = fluidMeasure(&fluidWetPct);

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
            const int   tank = fluidTankCells();
            const float fill = fluidWetPct;

            Serial.printf("fps %5.1f | frame %6.2f ms | phys %4.1f field %4.1f draw %4.1f dma %5.1f"
                          " | %4.1f MB/s | grid %dx%d %d px | wet %3.0f%% of %d cells"
                          " | liquid %6.0f px | flow %4.1f px"
                          " | drops %2d | g %+.2f %+.2f %+.2f | shake %3d%% | pal %d sign %d\n",
                          fps, msFrame, msPhys, msField, msDraw, msDma, mbPerSec,
                          gridCols, gridRows, cellPx, fill, tank, fluidMass, fluidSpeed,
                          dropLive,
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
