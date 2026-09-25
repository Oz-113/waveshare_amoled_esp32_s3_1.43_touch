# WaveshareAMOLED — the guide

Everything about this library and about the cube demos (examples
`06_Rotating_Cube` / `08_IMU_Cube`, the fluid of `09_IMU_Fluid`, and the
standalone copies in `F:\skeces\amoled_rotating_cube` / `amoled_imu_cube` /
`amoled_imu_fluid`): the maths, the drawing, what each component is, which library
or API it comes from, and what happens on which line when a frame is rendered and
pushed to the panel.

* **Part A** — the rotating cube project: the maths and the components.
* **Part B** — how this library works, from your sketch down to the QSPI wires.
* **Part C** — the IMU: how the on board QMI8658 is used to hold the cube upright.
* **Part E** — the same IMU feeding a liquid: a grid cellular automaton, a
  heightfield renderer, and where a full-screen sketch really spends its time.
* **Part D** — where to find the API reference (`README.md`), the examples and
  what to build next.

> Line numbers refer to **library version 1.0.4**. They are here to help you
> read the code, not as a permanent contract — if you edit a file they move.


---

# Part A — the rotating cube

## A0. The players

| Thing | Where it lives | What it is |
|---|---|---|
| `AMOLED amoled;` | the sketch, near the top | one object that owns the panel: `begin()`, `pushRect()`, `canvas()`, `touch()` |
| `AMOLED_Canvas` | `AMOLED_Canvas.h` / `.cpp` | the drawing surface: pixels, lines, circles, text, glow, arc. It clips everything and handles the rotation |
| `AMOLED_Touch` | `AMOLED_Touch.h` | the FT3168 touch panel (not used by the cube) |
| `fillQuad()`, `drawEdge()`, `project()` … | the sketch itself | the 3D maths — that is *your* code, not the library's |

The cube needs four things from the library and nothing else:

```cpp
amoled.begin();              // panel + touch + lookup tables
amoled.beginFramebuffer();   // 651 kB of PSRAM to draw into, once
amoled.canvas()              // the drawing surface (framebuffer)
fb.setClip(x0,y0,x1,y1)      // "nothing may be drawn outside this box"
amoled.pushRect(x0,y0,x1,y1) // send the changed rectangle to the panel
```

## A1. What has to happen for every frame

The whole animation is this seven step loop (`loop()`):

| Step | Code | Cost |
|---|---|---|
| 0 | hold the loop at `CUBE_FPS_CAP` (60) updates per second | nothing (a `delay(1)` wait) |
| 1 | `rotatePoint()` × 14 — 8 corners + 6 face normals | 8 sin/cos calls, ~50 flops — nothing |
| 2 | `project()` × 8 — perspective divide, bounding box | nothing |
| 3 | `setClip()` + one `fillRect()` over the old + new box in true black | ~1 ms (only if the box is large) |
| 4 | the visible faces (`fillQuad` — only with `CUBE_FACES 1`), 12 edges (`drawEdge`), 8 glows (`addGlow`) | ~3 ms with faces, ~0.5 ms without |
| 5 | `resetClip()` + `amoled.pushRect()` — only the box crosses the QSPI bus | **5–15 ms — this is the bottleneck** |
| 6 | frame counter → fps, serial + on screen | nothing |

Everything else in the sketch serves those seven steps.

## A2. The maths

### The cube as data

A cube with an edge length of 2 has all its coordinates at −1 or +1, so the
eight corners can be written down once (sketch line 69):

```cpp
static const float kCorner[8][3] = { {-1,-1,-1}, {+1,-1,-1}, ... };
```

Two more tables describe how those corners are connected:

* `kFace[6][4]` — the six faces, each as four corner indices **in order**, so
  the face can be filled as a quad (line 75). The order matters: going around
  the face is what makes the edge-crossing test in `fillQuad()` work.
* `kEdge[12][2]` — the twelve edges as corner pairs (line 91), used for the
  neon lines.

Because the cube's faces are axis aligned, their outward normals are simply
`(0,0,±1)`, `(±1,0,0)`, `(0,±1,0)` (line 86). No need to compute them from the
face geometry — although rotating them is still necessary, see below.

### Rotation — two 2×2 matrices

A point in 3D is rotated by multiplying it with a rotation matrix. For a cube
you only need two of the three possible rotations, and both are cheap:

```
yaw around Y :  x'  =  x*cos(a) + z*sin(a)      z'  = -x*sin(a) + z*cos(a)
pitch around X: y'' =  y*cos(b) - z'*sin(b)     z'' =  y*sin(b) + z'*cos(b)
```

`rotatePoint()` (sketch line 139) applies exactly that, first yaw then pitch.
Two multiplications per axis per point — the ESP32-S3 does this for all
14 points (8 corners + 6 normals) in a few microseconds.

Angles come straight from the clock so the animation is independent of the
frame rate:

```cpp
const float yaw   = t * CUBE_SPIN_Y;     // t = seconds since power on
const float pitch = t * CUBE_SPIN_X;
```

That is why the cube spins at the same speed whether the panel manages 30 or
100 frames per second (a *step per frame* would look twice as fast at 60 fps).

### Perspective projection — why it looks 3D

`project()` (sketch line 160) divides by the distance from the camera:

```cpp
float d = v.z + CUBE_CAM_DIST;   // the camera sits at z = -CUBE_CAM_DIST
const float s = CUBE_FOCAL / d;  // pixels per world unit at that distance
sx = CX + (int)(v.x * s + 0.5f);
sy = CY - (int)(v.y * s + 0.5f); // y is flipped: screen y grows downwards
```

`CUBE_FOCAL` plays the role of a lens: a bigger value zooms in. `d` is stored
per corner in `depth[]` and is reused twice: for the depth fog and for deciding
whether a face points at us or away from us.

### Back face culling

The camera looks along +z (larger z is farther). A face is visible when its
outward normal has a **negative z component**, i.e. it points back at the
camera. That single test (sketch line 364) throws away the three faces you
would never see — half the fill work:

```cpp
if (normal[f].z >= 0.0f) continue;
```

This is why the face *normals* are rotated as well as the corners.

### Lambert shading and depth fog

How bright a face is depends on the angle between its normal and the light
direction (sketch line 371):

```cpp
float lit = normal.x*kLight[0] + normal.y*kLight[1] + normal.z*kLight[2];  // dot product
```

`kLight` is a unit vector pointing at the "sun" (line 100). `lit` is 1 when the
face looks straight into the light and 0 when it is edge-on. Add a small
ambient term so nothing is pure black, then multiply by the keep-it-subtle
depth fade `depthFactor()` (line 171), which is 1.0 at the nearest corner and
`1 - CUBE_FOG` at the farthest.

### The rainbow

The colour is **hue**, not RGB, so it can sweep smoothly through the wheel:

```cpp
const float hueBase = fmodf(t * CUBE_HUE_SPEED, 360.0f);   // 0..360, wraps
```

`hueColour()` (sketch line ~181) turns a hue plus a brightness into a 24 bit
colour using two library helpers:

```cpp
const uint32_t c = amoledHSV(fmodf(hueDeg, 360.0f), 1.0f, 1.0f);  // HSV -> 0xRRGGBB
return amoledScale(c, (uint16_t)(bright * 255.0f), 255);          // apply brightness
```

`amoledHSV()` lives in `AMOLED_Colors.cpp:9` (full saturation and value = a
"pure" colour) and `amoledScale()` is a three-channel multiply in
`AMOLED_Colors.h:66`. The hue is offset a little per face (`f*20`) and per edge
(`e*13+12`), so the cube is not one flat colour — the rotation makes those
offsets wander across it.

### Filling a convex quad (`fillQuad`, sketch line ~198)

A projected cube face is always a **convex** quad, which makes filling it easy:

1. find the top and bottom scanline of the quad's bounding box;
2. for each scanline, walk the four edges and find where they cross it;
3. because the shape is convex, the leftmost and rightmost crossings are the
   span of that scanline — draw it with one `addHLine()`.

The crossing is a linear interpolation (all integer, no floats in the loop):

```cpp
if ((ya <= y && yb > y) || (yb <= y && ya > y))          // half open: corners once
    x = xa + (int)(((int32_t)(xb - xa) * (y - ya)) / (yb - ya));
```

`addHLine()` is an **additive** primitive: it adds light instead of replacing
the pixel (`AMOLED_Canvas.cpp:359` → `amoledPxAdd`). Additive drawing is what
makes an AMOLED look like it emits light — overlapping faces get brighter while
the black background stays black (alpha blending would need an opaque backdrop).
If `CUBE_FACES` is 0 the whole function is compiled out.

Two safeguards were added after the first hardware test, both for faces that are
almost edge on (a "sliver" of a few pixels):

* the interpolated `x` is clamped to the quad's **own bounding box**, so even a
  degenerate crossing can only ever land inside the quad;
* a quad whose area (shoelace formula, `area2 = 2·area`) is under
  `2 × CUBE_MIN_FACE_AREA` is skipped entirely. A face that thin cannot be seen,
  but the pixels a wild crossing would put on the screen would be very visible —
  and a stray pixel outside the dirty rectangle would never be erased again.

### The dirty rectangle — the frame rate trick

The cube only covers a small part of the 466 × 466 screen, so sending the whole
frame would waste the bus:

```
full frame            466 * 466 * 3 B = 651 468 B  ->  32.6 ms at 40 MHz QSPI
tight box   ~330x330  330 * 330 * 3 B = 326 700 B  ->  16.3 ms
full width bands      466 * 352 * 3 B = 492 096 B  ->  24.6 ms
```

So the sketch remembers where the cube was last frame, erases the **union** of
the old and the new box with one `fillRect()`, draws the cube, and then sends:

```cpp
fb.setClip(ux0, uy0, ux1, uy1);          // nothing may escape the box
fb.fillRect(ux0, uy0, ux1, uy1, AMOLED_BLACK);
... draw the cube ...
fb.resetClip();
amoled.pushRect(0, py0, W - 1, py1);     // library AMOLED.cpp:387
```

`pushRect()` splits the region into bands of `AMOLED_STRIP_LINES` (32) rows,
packs each band into a strip buffer and sends it as **one** QSPI transfer with a
window of exactly `(x0..x1, band)` — so a 330-row box needs 11 transfers instead
of 330. Two details in the current implementation are worth knowing:

* **every transfer is a whole number of 32 bit words.** The row count of a band
  is rounded down to a multiple of four (`rows -= rows % 4`), and if fewer than
  four rows are left the window is slid up so it still ends on `y1`. Rows are
  then sent twice, which costs a little time but cannot change a pixel: the
  extra rows hold exactly the same framebuffer content. A partial 32 bit word is
  precisely the kind of thing that makes a QSPI/DMA path append stray bytes to
  the panel's window;
* **a full width region packs with a single `memcpy` per band** (`AMOLED.cpp`),
  because the band is then one contiguous run of the framebuffer. That is why
  `CUBE_FULL_WIDTH` is 1 by default: the transfers have the same shape as in
  streaming mode and the packing is about ten times cheaper than 32 narrow
  `memcpy`s. `CUBE_FULL_WIDTH 0` sends ~25 % fewer bytes (the tight box) if you
  want the last drop of frame rate.

## A3. Where every piece comes from

| Component | Comes from | Notes |
|---|---|---|
| `AMOLED`, `AMOLED_Canvas`, `AMOLED_Touch`, `amoledRGB/HSV/Scale` | **this library** | the whole drawing and panel layer |
| `Serial`, `Serial.printf`, `millis()`, `delay()` | Arduino core | the sketch sets `Serial.setTxTimeoutMs(0)` and guards prints with `if (Serial)` so nothing ever blocks |
| `cosf`, `sinf`, `fmodf`, `snprintf` | C standard library (`<math.h>`, `<stdio.h>`) | pulled in by the Arduino core |
| `struct V3`, all the maths, the `#define` knobs | plain C++ in the sketch | **no external vector or 3D library is used** |
| `esp_lcd_panel_*`, `spi_bus_initialize`, `heap_caps_malloc` | ESP-IDF, shipped inside the Arduino core | used by the library, never by the sketch |
| `gpio_set_level`, `esp_rom_delay_us` | ESP-IDF | only for the panel ID probe |
| `xSemaphoreTake/Give`, `portMAX_DELAY` | FreeRTOS, via the Arduino core | the strip-buffer hand-shake |
| `Wire` (I2C) | Arduino core | the touch panel (`AMOLED_Touch.cpp`) |
| `esp_lcd_sh8601.c/.h` | Espressif, Apache-2.0 | the SH8601 command sequence, shipped inside this library |

Nothing has to be installed for any of that — the Arduino-ESP32 core already
contains the ESP-IDF parts.

## A4. The sketch, line by line

**Knobs (top of the sketch)** — everything you may want to change: how the box
is pushed (`CUBE_DIRTY_RECT`, `CUBE_FULL_WIDTH`), the update rate
(`CUBE_FPS_CAP`), what is drawn (`CUBE_FACES`, `CUBE_EDGE_GLOW`,
`CUBE_SHOW_FPS`/`CUBE_SERIAL_FPS`), the two rotation speeds, the hue speed, the
camera (`CUBE_CAM_DIST`, `CUBE_FOCAL`), the fog strength, the halo gain and the
sliver cut-off (`CUBE_MIN_FACE_AREA`).

**setup() (line ~259)**

| Line area | What happens |
|---|---|
| ~262 | `Serial.begin(115200)` + `setTxTimeoutMs(0)` — never block on the USB |
| ~266 | `amoled.begin()` — panel, buses, lookup tables, touch |
| ~271 | `amoled.setBrightness(255)` — a register write, not PWM |
| ~278 | `amoled.beginFramebuffer()` — 651 kB of PSRAM as the drawing surface |
| ~285–291 | the static labels and the first frame-rate text |
| ~292 | `amoled.push()` — send that first picture once |
| ~297 | the banner on the serial port |

**loop()**

| Step | What happens |
|---|---|
| 0 | `while (millis() - frameStartMs < 1000/CUBE_FPS_CAP) delay(1)` — the anti-tearing wait, skipped when `CUBE_FPS_CAP` is 0 |
| 1 | yaw/pitch from `millis()`, then 8 corners + 6 normals through `rotatePoint()` |
| 2 | `project()` every corner and grow the bounding box from the results |
| 3 | add the glow margin (18 px) and clamp the box to the screen |
| 3 | union with last frame's box (the cube moved!) |
| 3 | `setClip(box)` then `fillRect(box, AMOLED_BLACK)` — erase; black on an AMOLED means the pixels are off |
| 4 | `hueBase` for this frame |
| 4 | the three visible faces (`CUBE_FACES 1` only): cull, shade, `fillQuad()` |
| 4 | the twelve edges: `drawEdge()` with a per-edge hue |
| 4 | the eight corners: `addGlow()` (a radial light blob each) |
| 4 | `resetClip()` — back to the whole framebuffer |
| 5 | `amoled.pushRect(...)` and count the bytes |
| 5 | remember the box for the next frame |
| 6 | count frames; once per second: fps, frame time, kB per frame → screen + serial |

## A5. The frame rate budget

```
data per frame / QSPI bandwidth = transfer time

  651 468 B / 20 MB/s  (40 MHz QSPI, 4 lanes) = 32.6 ms  -> 30 fps ceiling
  326 700 B / 20 MB/s  (tight dirty box)      = 16.3 ms  -> 61 fps ceiling
  492 096 B / 40 MB/s  (full width, 80 MHz)   = 12.3 ms  -> 81 fps ceiling
  326 700 B / 40 MB/s  (tight box, 80 MHz)    =  8.2 ms  -> 122 fps ceiling
```

Add the render cost (≈ 1 ms erase + 3 ms faces + 0.5 ms edges and glows) and you
get the numbers below. Two consequences:

* the ESP32-S3 is **not** the limit, the QSPI link is. The only big win left is
  `AMOLED_QSPI_CLOCK_HZ 80000000UL` (or fewer bytes per frame: a smaller box,
  fewer pixels, or RGB565);
* every millisecond of data you save is a millisecond of frame time, which is
  exactly why `pushRect()` exists.

| Configuration | Data per frame | Frame time (calculated) | fps |
|---|---|---|---|
| full width bands, faces on, 80 MHz | 492 kB | ~13 ms | ~75 (capped at 60 by default) |
| tight box, faces on, 80 MHz | 327 kB | ~10 ms | ~100 (capped at 60) |
| tight box, `CUBE_FACES 0`, 80 MHz | 327 kB | ~7 ms | ~140 (capped at 60) |
| full frame, `CUBE_DIRTY_RECT 0`, 80 MHz | 651 kB | ~21 ms | ~47 |

The sketch prints the real numbers once per second, so you can check these
estimates against your own board. **The numbers in the README/GUIDE are
calculated from the bus arithmetic above, not measured** — the sketch's own
serial output is the truth.

## A6. Tearing — what the panel is doing while you push

This is the honest part of the project, and it answers "why do the edges look
cut/ doubled/ copied into a few bands?".

The panel has a **scan out**: the controller reads its GRAM row by row, top to
bottom, non stop (60 Hz means a full pass every 16.6 ms). There is no
tear-effect (TE) pin wired on this board, so **there is no way to tell the
controller "wait, I am still sending"**. Whatever you push lands in GRAM while
the current pass is already half way down the screen. The result is one visible
**seam** per frame: above it the picture is already the new one, below it the
picture is still the previous frame.

Two things make that seam much more obvious on this sketch than on a normal
demo:

* the cube is **thin, bright and on pure black**. A gradient scene hides the
  seam completely (the 5 original demo scenes never showed it), a 1 px neon edge
  on #000000 makes it a visible step in the line;
* if the loop runs **faster than the panel refreshes**, more than one frame is
  in flight and more than one seam is on screen. At 200 fps that is three or
  four generations of the cube stacked in horizontal bands, each one a step
  further round — which is exactly the "the line breaks into a few parallel
  parts" symptom. It looks like four equal bands because each generation is
  whatever the last 16.6 ms of pushes produced.

What actually helps, best first:

1. **`CUBE_FPS_CAP 60`** (the default). One update per panel pass means at most
   one seam can exist, and the eye reads a single moving seam as ordinary motion
   blur rather than as a broken shape. Set it to 0 to see the raw frame rate the
   ESP32 can do, and to see why you would not want to.
2. **A faster bus** (`AMOLED_QSPI_CLOCK_HZ 80000000UL`, the default in this
   library): the seam is written in half the time, so it moves through the
   picture twice as fast and is twice as easy to miss.
3. **Fewer bytes per frame** — `CUBE_FULL_WIDTH 0`, `CUBE_FACES 0`, RGB565.
4. **A motion trail** (`SQUARE_TRAIL` in 07, or a dim redraw instead of
   `fillRect(BLACK)`) turns the seam into an intentional looking blur.

### Telling tearing and drawing bugs apart

Set `CUBE_SPIN_X` and `CUBE_SPIN_Y` to `0.0f` so the cube stands still:

* if the artifacts **vanish** they were tearing (a timing effect, not a bug in
  the picture);
* if they **stay on a frozen cube**, the drawing is wrong — and that is a real
  bug worth chasing.

That is also the test that separated the two problems in this project: the
earlier "thick line" drew three parallel strokes of the same edge (which reads
as a doubled/ tripled line), and the near-edge-on faces could interpolate a
crossing outside the quad. Both are fixed now (`drawEdge()` draws one core plus
a *perpendicular* halo, `fillQuad()` clamps and skips slivers), and what is left
on a hard edge at high frame rates is the panel's seam.

---

# Part B — how this library works

## B1. The layers

```
        your sketch  (amoled_rotating_cube.ino)
             |
             |  AMOLED amoled;  amoled.begin();  amoled.pushRect(...)
             v
    +--------------------------------------------------------------+
    |  WaveshareAMOLED library                                     |
    |    AMOLED         AMOLED.cpp/.h   panel, QSPI bus, brightness,|
    |                                   strip pipeline, framebuffer |
    |    AMOLED_Canvas  ..._Canvas.*    pixels, lines, circles,     |
    |                                   text, glow, arc, rotation    |
    |    AMOLED_Touch   ..._Touch.*     FT3168 over I2C             |
    |    AMOLED_Colors  ..._Colors.*    colours, HSV, mixing        |
    |    AMOLED_Probe   ..._Probe.*     bit-banged panel ID         |
    |    esp_lcd_sh8601 .c/.h           the SH8601 command sequence |
    +--------------------------------------------------------------+
             |
             |  ESP-IDF (inside the Arduino core) + FreeRTOS + Arduino Wire
             v
    esp_lcd_panel_io_spi -> driver/spi_master -> SPI2 peripheral + GDMA
                                                     |
                                                     v
                                             the 466x466 AMOLED
```

A sketch only ever calls `amoled.*` and `cv.*` — no ESP-IDF call is needed in
user code.

## B2. `amoled.begin()` — what happens at boot

`AMOLED.cpp:122`, in this order:

| Line | Call | What it does |
|---|---|---|
| 128 | `amoledProbePanelId()` | bit-bangs `0xDA` ("Read ID1") over the four QSPI lines **before** the SPI peripheral owns them: `0x86` = SH8601, `0xFF` = CO5300 (`AMOLED_Probe.cpp:107`). The CO5300 later needs a 6-pixel GRAM offset. |
| 135 / 138 | `heap_caps_malloc()` | two strip buffers of `466 × AMOLED_STRIP_LINES × bpp` bytes in internal DMA RAM (PSRAM as fallback). |
| 152 | `spi_bus_initialize()` | the QSPI bus: 4 data lines (GPIO 11–14), clock (10), DMA, `max_transfer_sz` = one full strip. |
| 166 | `esp_lcd_new_panel_io_spi()` | the panel IO: **32-bit command + 8-bit parameters + `quad_mode`**. The 32 bits are the QSPI trick — first byte = opcode (`0x02` for a command, `0x32` for pixel data), the rest is the LCD command byte. The completion callback `amoledOnColorDone` is registered here. |
| 195-197 | `esp_lcd_new_panel_sh8601()`, `esp_lcd_panel_reset()`, `esp_lcd_panel_init()` | creates the panel and runs the init table from line 36 (SH8601) or 47 (CO5300): sleep out, tearing effect, brightness control, **`COLMOD 0x3A` = `0x77` = 24 bit/pixel**, display on, brightness to max. |
| 199 | `esp_lcd_panel_set_gap()` | CO5300 only: shift the drawing window 6 columns. |
| 203 | `xSemaphoreCreateCounting(2, 2)` | two "strip buffer is free" tokens — the heart of the pipeline. |
| 207 | `AMOLED_Canvas::initTables()` | builds the 1024-entry rainbow, the 256-entry glow curve and the 512 arc direction steps (`AMOLED_Canvas.cpp:23`). |
| 209 | `_canvas.beginStrip(strip0, 0, 32)` | gives the canvas a sane mapping before the first frame. |
| 211 / 213 | `_touch.begin()`, `setBrightness(0xFF)` | I2C touch, then the panel brightness register. |

## B3. The strip pipeline — what gets rendered and pushed, when

This is the part that makes the library fast. The code lives in
`AMOLED.cpp:99–117` (buffer helpers) and `AMOLED.cpp:283–333` (the frame API).

**Once**, in `begin()`: two strip buffers and two semaphore tokens.
With the default `AMOLED_STRIP_BUFFERS 2` / `AMOLED_STRIP_LINES 32` each buffer
holds `466 × 32 × 3 = 44 736` bytes — one band of 32 screen rows at RGB888.

**Every frame** the sketch runs the documented pattern:

```cpp
amoled.beginFrame();
while (amoled.nextStrip(cv)) {      // hands you a canvas on the next strip
    ...draw...                      // writes into internal RAM, bus untouched
    amoled.pushStrip();             // queues that strip onto the QSPI bus
}
amoled.endFrame();
```

| Call | Line | What really happens |
|---|---|---|
| `beginFrame()` | 283 | resets the row counter `_stripY` to 0. A strip left over from an aborted frame is pushed first, so a buffer token can never be lost. |
| `nextStrip(cv)` | 291 | 1. pushes the strip you drew last time if you did not call `pushStrip()`; 2. `amoledTakeStripBuffer()` (line 300) → `xSemaphoreTake` (line 102) — **blocks until a buffer is DMA-free**; 3. remembers the row in `_pendingY`; 4. `canvas.beginStrip(buf, _pendingY, lines)` (line 308) points the canvas at that buffer and sets the clip rectangle; 5. advances `_stripY += 32` (line 309) and returns `true`. |
| *(your drawing)* | — | every primitive writes into `buf` through the canvas: `fillRect`, `addGlow`, `addHLine`, `drawText`… **nothing touches the panel here.** |
| `pushStrip()` | 314 | `amoledQueueStrip()` (line 107) → `esp_lcd_panel_draw_bitmap(s_panel, 0, y, 466, y+lines, buf)` (line 111). **This is the line where pixels start moving**: the panel driver sends `CASET`/`RASET` (the window) and then `RAMWR` with the 44 736 bytes. It returns immediately — the transfer is asynchronous. |
| *(next iteration)* | 291 | takes the *other* buffer, so the CPU draws strip N+1 while the DMA is still shifting out strip N. |
| `endFrame()` | 321 | nothing to wait for; it also puts the canvas back on the framebuffer if one exists. |
| `waitIdle()` | 328 | takes all tokens and gives them back — the "everything has been transmitted" barrier. Use it before changing the brightness, sleeping or re-configuring anything. |

**The hand-shake, and why a buffer is never overwritten early**

```
   task (your loop)                              SPI / DMA / ISR
   ----------------                              ---------------
   take token   count 2 -> 1
   draw into buf A
   push A  ------------------------------------>  queue: [window][44 736 B]
   take token   count 1 -> 0                       ... DMA shifting out buf A ...
   draw into buf B
   push B  ------------------------------------>  queue: next transfer
   take token   count 0 -> BLOCKS                  DMA of buf A finishes
                                                   post_cb -> amoledOnColorDone()  :73
                                                   xSemaphoreGiveFromISR()         :80
                count 0 -> 1: buf A is free again, keep drawing
```

The SPI driver processes its queue in order and the buffers are used in a strict
rotation, so the token that comes back always belongs to the buffer that was
pushed two strips ago. That is the whole trick: **rendering and transmission
overlap, and a buffer is only reused after its DMA has finished**. One strip
costs `max(render, transfer)` instead of the sum of both.

**Why strips are always full width**: the panel's window command is cheap, but
the *number of transfers* is what costs. One full-width strip = one transfer.
For partial regions there is `pushRect()` — see B4.

## B4. Framebuffer mode, `push()` and `pushRect()`

| Function | Line | What happens |
|---|---|---|
| `beginFramebuffer()` | 350 | `heap_caps_malloc(466*466*3, MALLOC_CAP_SPIRAM)` (line 358) = 651 468 bytes, then `_canvas.beginFull(_fb)` (line 364). `beginFull()` is literally `beginStrip(fb, 0, AMOLED_HEIGHT)`: the "strip" *is* the whole image. |
| `push()` | 368 | loops over the strips: take a strip buffer → `memcpy` the strip out of PSRAM (line 377) → queue it. The copies overlap the DMA of the previous strip. Ends with `waitIdle()` (line 384) so the framebuffer is safe to draw into again. |
| `pushRect(x0,y0,x1,y1)` | 387 | clamps the rectangle, then works in bands of `AMOLED_STRIP_LINES`: the band's rows are packed into a strip buffer and `esp_lcd_panel_draw_bitmap(s_panel, x0, y, x1+1, y+rows, buf)` sends them as one transfer with a **partial window**. 11 transfers for a 330-row box instead of 330, which is exactly why the cube uses it. Two refinements: the row count of a band is rounded to a multiple of four (`rows -= rows % 4`, so every transfer is a whole number of 32 bit words, with the window slid up if fewer than four rows are left), and a **full width** region is packed with a single contiguous `memcpy` instead of one per row. `waitIdle()` at the end. |

Both end with `waitIdle()`, so after `push()`/`pushRect()` returns you can draw
into the framebuffer again without tearing.

## B5. The canvas: one affine mapping does everything

Every function in `AMOLED_Canvas` eventually resolves a logical pixel `(x, y)`
to a byte address with the same formula (`AMOLED_Canvas.cpp:276`, `ptr()`):

```
address = _fb + ( _idx0 + (x - _lx0) * _stepX + (y - _ly0) * _stepY ) * AMOLED_BPP
```

`beginStrip()` (line 98) only fills in those five numbers:

| Rotation | `_idx0` | `_stepX` | `_stepY` | Clip rectangle | Effect |
|---|---|---|---|---|---|
| 0° | 0 | 1 | W | x 0…465, y = the strip | normal |
| 90° | W−1 | W | −1 | a **column** band of the image | image turned 90° |
| 180° | n·W−1 | −1 | −W | the mirrored band | upside down |
| 270° | (n−1)·W | −W | 1 | a column band | turned the other way |

That is the whole rotation implementation: **no second buffer, no extra
transfer, one add per pixel**. It is also why a "strip" is not always a band of
*screen* rows — at 90°/270° it is a band of image columns that ends up in a band
of screen rows.

Consequences worth knowing:

* `width()`/`height()` return the *logical* size of the **window** this canvas
  can draw (the strip, or the whole framebuffer), and `clipX0()…clipY1()` is
  what every primitive clamps to. A scene function can therefore be written as
  if it painted the whole screen — the canvas throws away whatever is outside
  the strip.
* `setClip(x0,y0,x1,y1)` narrows the clip to a rectangle inside the window and
  `resetClip()` puts it back. `setClip()` re-derives `_idx0` from the new clip
  origin (`AMOLED_Canvas.cpp`, `_widx0 + (x0-_wx0)*_stepX + (y0-_wy0)*_stepY`),
  so the same one-add addressing keeps working. `width()`/`height()` deliberately
  keep describing the window, so layout code does not change when a clip is set.
  This is the "nothing may be drawn outside the rectangle I am going to push"
  guarantee that a dirty rectangle renderer needs.
* `strideX()`/`strideY()` return the byte step for +1 logical x/y (negative
  under rotation). Fast hand-written loops use
  `p = cv.ptr(x, y); … p += cv.strideX();` (the demo and `03_Contrast_Test` do).
* `clearScreen()` (line 137) is a `memset` of `width()*height()*BPP` bytes,
  which is exactly the strip buffer whatever the rotation — and on an AMOLED a
  zeroed pixel is a pixel that is **off**.

The interesting primitives:

| Primitive | Line | Note |
|---|---|---|
| `amoledPxSet/Add/AddScaled/Mul/Blend` | `AMOLED_Canvas.h:50–88` | the only place where RGB888 vs RGB565 matters (`#if AMOLED_BPP == 3`) |
| `addGlow()` | `AMOLED_Canvas.cpp:444` | per pixel: the squared distance is projected onto a 256-entry curve — one multiply, one shift, one lookup; no sqrt and no division inside the loop (line 460 precomputes the reciprocal) |
| `addArc()` | 492 | 512 pre-computed direction steps, two stamps each so the arc is continuous |
| `drawText*()` | 592–612 | 5×7 glyphs from `AMOLED_Font5x7.h`, size 1…4, opaque / additive / alpha |
| the palette | `initTables()`, line 23 | 1024 rainbow entries, `palette(i)` |

## B6. The colour path, from `0xRRGGBB` to the panel

```
uint32_t colour = amoledRGB(255,128,0);        // 0xRRGGBB
        |
        |  amoledPxSet(p, r, g, b)             // AMOLED_Canvas.h:50
        v
buffer: R G B  R G B  R G B ...                // 3 bytes per pixel (RGB888)
        |
        |  esp_lcd_panel_draw_bitmap(..., len = w * h * bpp / 8)
        v
esp_lcd_panel_io_spi: 32-bit command (opcode 0x32 + RAMWR 0x2C), then the
bytes, four bits per clock over the four data lines
        |
        v
SH8601 / CO5300 with COLMOD 0x77 ("24 bit/pixel") -> the pixel lights up
```

Nothing converts anything on the way, which is why the library can promise the
full 16.7 million colours. With `AMOLED_COLOR_DEPTH 16` the same calls are packed
into RGB565 (`amoledPxSet565`, `AMOLED_Canvas.h:106`) and `COLMOD` becomes
`0x55` — the drawing API does not change at all.

## B7. Touch

`AMOLED_Touch` (`AMOLED_Touch.cpp`) talks to the FT3168 with Arduino's `Wire`
(I2C, SDA 47 / SCL 48 — the same bus as the QMI8658 IMU and the PCF85063 RTC):

| Function | Line | What it does |
|---|---|---|
| `begin()` | 35 | `Wire.begin()`, wake the controller (`reg 0x00 = 0`) |
| `read(x, y)` | 59 | two transfers: status register `0x02`, then 4 bytes from `0x03` (12-bit X and Y) |
| `update()` | 81 | polls at most every `AMOLED_TOUCH_POLL_MS` and tracks press/release |
| `tapped()`, `held(ms)`, `heldOnce(ms)` | 113 / 120 / 125 | gesture helpers, so a sketch does not have to debounce anything itself |

## B8. Where Arduino ends and ESP-IDF begins

| Layer | Provides | Used by |
|---|---|---|
| **your sketch** | the maths, the animation, the decision what to draw | — |
| **this library** | `amoled.*`, `cv.*`, colours, touch | your sketch |
| **Arduino core** | `Serial`, `millis()`, `delay()`, `Wire`, `Print`, the `setup()`/`loop()` runtime | sketch and library |
| **ESP-IDF** (inside the core) | `esp_lcd_panel_io_*`, `esp_lcd_panel_*`, `driver/spi_master`, `driver/gpio`, `heap_caps_*`, FreeRTOS semaphores, `esp_rom_delay_us` | `AMOLED.cpp`, `AMOLED_Probe.cpp` |
| **Espressif's SH8601 driver** (shipped in this library) | the controller command sequence, `esp_lcd_new_panel_sh8601()` | `AMOLED.cpp:195` |
| **hardware** | SPI2 + GDMA move the bytes without the CPU | after `esp_lcd_panel_draw_bitmap()` |

In short: C++ and the C standard library in the sketch, the Arduino API for time,
serial and I2C, and ESP-IDF only *below* the library — the same level TFT_eSPI
sits on for a normal SPI TFT, just with the modern `esp_lcd` layer instead of
hand-written register pokes.

## B9. Tuning checklist (fastest wins first)

| # | Change | Where | Why |
|---|---|---|---|
| 1 | `AMOLED_QSPI_CLOCK_HZ 80000000UL` | library `AMOLED_Config.h` | doubles the bandwidth of the only real bottleneck |
| 2 | send fewer bytes: `pushRect()` instead of `push()` | sketch | 330 kB instead of 651 kB per frame |
| 3 | `AMOLED_COLOR_DEPTH 16` | library | halves the bytes per pixel again (65k colours) |
| 4 | `CUBE_FPS_CAP 60` | sketch | not a speed-up but the biggest *quality* win: one panel pass per update, so only one tear seam exists |
| 5 | `CUBE_FULL_WIDTH 0` | sketch | drops the full width padding of the pushed bands (~25 % fewer bytes) |
| 6 | `CUBE_FACES 0`, `CUBE_EDGE_GLOW 0`, `CUBE_SHOW_FPS 0` | sketch | saves render time |
| 7 | `AMOLED_STRIP_LINES 24…48` | library | only matters for full-frame pushes; smaller = less RAM, more transfers |
| 8 | make sure nothing blocks | sketch | no `delay()`, serial prints guarded, `Serial.setTxTimeoutMs(0)` |

---

# Part C — holding the cube upright with the IMU (example 08_IMU_Cube)

## C0. What it does

`08_IMU_Cube` is `06_Rotating_Cube` with one addition: the on board **QMI8658
accelerometer** tells the sketch where "down" is, so the cube

* keeps its **top face pointing at the sky** however you turn the display,
* keeps **spinning about the world vertical** (not about a screen axis),
* and is lit by a sun that is fixed in the **world**, so turning the device
  around shows a different side of it.

So the cube behaves like an object standing still in the room while the screen
turns around it. Touching it:

| gesture | effect |
|---|---|
| tap | freeze / unfreeze the spin |
| hold 0.7 s | next of the four sensor → display sign variants (see C4) |

If no IMU answers (or `CUBE_IMU_ENABLE 0`), the sketch falls back to the
automatic two axis spin of 06 and says so on the panel.

## C1. The chip, and how it is read

The QMI8658 sits on the **same I2C bus as the touch panel** (SDA 47 / SCL 48),
which `amoled.begin()` → `AMOLED_Touch::begin()` already started
(`Wire.begin(...)`, 400 kHz) — the sketch adds no bus setup at all.

| register | value written | why |
|---|---|---|
| `0x00` WHO_AM_I | – | must read `0x05`; the chip answers at `0x6B` or `0x6A` (SA0), so both are tried |
| `0x02` CTRL1 | `0x60` | bit 6 = address auto increment, so one burst read can return all six data bytes |
| `0x03` CTRL2 | `0x25` | bits 6:4 = accel range `2` = 8 g, bits 3:0 = output rate `5` = 250 Hz |
| `0x08` CTRL7 | `0x01` | bit 0 = accelerometer enable (written last, after `0x00`) |
| `0x2E` STATUS0 | – | bit 0 = a fresh accelerometer sample is ready |
| `0x35…0x3A` | – | ax, ay, az as little endian `int16`, **4096 LSB per g** at 8 g |

The register map and the sequence are the same ones Waveshare's own
`Arduino/examples/03_I2C_QMI8658` uses (that example is where this list came
from — its `qmi8658c.cpp` `qmi8658_config_acc()` / `read_sensor_data()` are the
reference). Their driver enables gyro and interrupts as well; the cube needs
neither, so `imuBegin()` writes four registers and is done.

Reading is two short I2C transfers per frame (`imuRead()`), which costs a few
tens of microseconds and is invisible in the frame budget.

## C2. The maths: from gravity to a rotation

**1) The accelerometer measures "up", not "down".** At rest it reports the
*reaction* to gravity — a phone lying flat, screen up, reports `+1 g` out of the
screen. So the reading, in g, is the world vertical pointing at the sky.

**2) Map it into the renderer's coordinates.** The display frame is x right,
y down, z into the screen; the cube code uses x right, **y up**, z into the
screen (screen y grows downwards). Hence

```cpp
V3 up = { gravityDisp.x, -gravityDisp.y, gravityDisp.z };   /* +y is up here */
```

**3) Align with it, by the shortest path.** `alignMatrix()` returns the rotation
that takes the cube's own +y (the normal of its top face) onto `up` using the
**shortest arc** — Rodrigues' formula for the axis `v = y × up`:

```
c  = y · up                  /* cos of the angle between them */
v  = y × up
s2 = v · v
R  = I + [v]× + [v]×² · (1 − c) / s2
```

Written out, the entries are (this is exactly the code):

```
R00 = 1 + k(vx² − s2)   R01 = −vz + k·vx·vy   R02 =  vy + k·vx·vz
R10 =      vz + k·vx·vy R11 = 1 + k(vy² − s2) R12 = −vx + k·vy·vz
R20 =     −vy + k·vx·vz R21 =  vx + k·vy·vz   R22 = 1 + k(vz² − s2)
```

with `k = (1 − c)/s2`. Held upright, `up = (0,1,0)` gives `v = 0`, `s2 = 0` and
the identity, so the cube looks exactly like in 06. Roll the display by φ and
this reduces to exactly `Rz(−φ)` — **the opposite rotation**, which is what keeps
the cube standing still in the room rather than turning with the device.

> **Why "shortest" matters — the bug this fixes.** The first version built a
> whole world frame from `up` plus the *current* screen-forward axis as a
> horizontal reference. That looks tidy, but it makes the "world" frame ride
> along with the display: rolling the device moved the reference with it, so the
> cube turned *with* your hand instead of against it. Gravity alone cannot give
> you a horizontal reference at all — so you must not invent one. Using only the
> vertical, via the shortest rotation, leaves the one genuinely undetermined
> degree of freedom (a spin about the vertical) where it belongs: to the
> animation.

Two limitations that come with using gravity only (both are inherent, not bugs):

* **A turn about the vertical is invisible.** Rotating the display like a
  turntable (keeping its tilt) does not change the gravity vector at all, so the
  cube cannot know it happened and stays as it was on screen. An accelerometer
  simply cannot see that axis — a **gyro** can, which is why "use the gyro too"
  is first on the next-steps list in Part D.
* **Held exactly upside down** the shortest rotation has no unique axis (any
  half-turn about a horizontal axis does the same job), so the cube's roll can
  snap as you pass through that pose. Same cure: the gyro.

**4) Spin about the vertical.** The animation is a rotation about the cube's own
+y — which the alignment has just put along the world vertical:

```
S = rotation about +y by the accumulated spin angle
m = a · S            /* a = the alignment, applied to the 8 corners */
```

That is the whole trick: gravity only fixes the cube's orientation *up to a spin
about the vertical*, and that is precisely the free parameter an animation
wants. Both "hang correctly" and "keep turning" come out of the same matrix.

**5) The light.** A direction that stays put in the *room* — but gravity fixes
only the vertical and not the horizontal, so the "room" is defined once, from the
pose the device was in when the first sample arrived (`worldRef`, captured in the
loop). After that `lightView = worldRef · kLight` is constant, so the cube spins
under a fixed sun. Without an IMU there is no `worldRef` and the light stays
fixed to the screen, exactly as in 06. (It only affects the optional faces.)


**6) Noise.** `imuUpdate()` normalises every sample, ignores the ones whose
magnitude is more than `IMU_SHAKE_G` away from 1 g (that means the device is
being moved, and the reading is acceleration, not gravity), and low passes the
direction with `IMU_FILTER` (0.15 per sample) before it is used.

## C3. Where each piece lives in the sketch

| Piece | In `08_IMU_Cube` | Notes |
|---|---|---|
| the I2C helpers | `imuRead()`, `imuWrite()` | plain `Wire` calls, no library needed |
| chip bring-up | `imuBegin()` | WHO_AM_I probe at `0x6B`/`0x6A`, then four register writes |
| one sample | `imuReadRaw()` | STATUS0 bit 0, then a 6 byte burst; converts to g |
| mapping + filter | `imuUpdate()` | the only place that knows about `kImuSigns[]` |
| the alignment | `alignMatrix()` | gravity → the shortest rotation that stands the cube up (and therefore the *opposite* of how the display moved) |
| the animation | `spinMatrix()`, `mul()` | `S` (yaw + pitch, or the world spin), then `m = align · S` |
| the cube | `apply(m, …)`, `project()` | 8 corners and 6 normals go through `m` |
| drawing | `drawEdge()`, `fillQuad()` (faces off by default), `addGlow()` | the same code as 06 |
| the readouts | `drawFpsText()`, `drawImuText()`, `pushTextBox()` | each one pushes only its own little box when it changes |
| the loop | steps 0…8 | frame cap, touch, IMU, rotation, projection, dirty box, erase + `setClip`, draw, push, readouts |

Library API used, and nothing else: `begin()`, `setBrightness()`,
`beginFramebuffer()`, `canvas()`, `setClip()`/`resetClip()`, `fillRect()`,
`drawPixel()`, `addPixelScaled()`, `addGlow()`, `addHLine()`, `drawText()`,
`drawTextCentered()`, `textWidth()`, `pushRect()`, `push()`,
`touch().update()/tapped()/heldOnce()`.

## C4. The axis mapping — the one thing that may need your eyes

The image has to be built in the *display's* axes, but the accelerometer lives in
its *own* axes, and how the sensor sits inside the case is not something a driver
can know — Waveshare's IMU example does not document it either (it leaves its
`qmi8658_axis_convert(acc, gyro, 0)` at the identity layout). So the sketch

* starts from a documented guess: `IMU_SIGN_VARIANT 1`, i.e. the sensor's x is
  the display's x, and y and z are negated (the usual arrangement for a sensor
  mounted behind a panel),
* lets you **cycle the four sign variants with a long press** (hold ~0.7 s), and
* always shows what it believes, on the panel and on the serial port:

```
G +0.02 +0.98 -0.01   SIGN 1  + - -   SPIN
```

Two checks settle it — with the right variant both of these are true (the
readout is what an accelerometer reports at rest: the *upward* reaction to
gravity, so it points at the sky):

| hold the display… | …and the mapped gravity reads |
|---|---|
| flat on the table, screen up | `(0, 0, -1)` — the cube lies flat, its top face pointing into the screen, away from you |
| upright in front of you | `(0, -1, 0)` — the cube is still level |

Put the variant number that passes both into `IMU_SIGN_VARIANT` so it is the
default after the next reset. The four variants are `+ + +`, `+ - -`, `- + -`,
`- - +` (the sign combinations with a positive determinant, i.e. proper
rotations). Should a board ever place the sensor rotated by 90° rather than
mirrored, the fix is the single line in `imuUpdate()` where the three components
are taken from `raw[]` — swap the indices there.

---

# Part E — a grid liquid that obeys gravity (example 09_IMU_Fluid)

## E0. What it does

`09_IMU_Fluid` uses the **same accelerometer reading** as Part C, for something
that has nothing to do with a cube: a tank of neon liquid inside the screen that

* **runs to the low side** when you tilt the display, and levels out again when
  you lay it down,
* **sloshes and sprays** when you shake it, then settles over about a second,
* **spreads out evenly** when the display lies flat, because then the in-plane part
  of gravity is zero and only the levelling term is left,
* and can be **stirred with a finger**, which also paints new liquid.

The liquid is a **grid** simulation - a cellular automaton over
`FLUID_GRID_COLS × FLUID_GRID_COLS` cells - drawn as a heightfield, so what you see
is a continuous fluid with a pixel-smooth surface: not a field of squares, and no
longer blobs. (The additive-glow "metaball" rendering this example used before is
still worth reading about — it is what `02_Glow_Orbs` and `05_Starfield` do, and
`addGlow()` is documented in README section 7.)

| gesture | effect |
|---|---|
| drag | push the liquid away from your finger, and paint a little new liquid |
| tap | next palette (constant colour / RGB by depth / thermal) |
| hold 0.7 s | next of the four sensor → display sign variants (C4) |

The simulation runs in **streaming mode**: the scene is drawn into 32 line strips
of internal DMA RAM and each strip is sent while the next is drawn. No framebuffer,
no PSRAM — example 09 is the one sketch in the set that does not call
`beginFramebuffer()`.

## E1. One accelerometer, two pieces of information

Part C throws one thing away: any sample whose magnitude is not ~1 g is ignored,
because it is movement and not gravity. This example keeps that information, and
it turns out to be a gift:

| what you read | what it means | what the sketch does with it |
|---|---|---|
| the **direction** of the vector | where "down" is (at rest it points at the sky) | the direction the automaton transports liquid in |
| the **length** — `\|g\| - 1` | how hard the device is being moved, in g | mixes the raw vector into that direction, and scales the pull up |

The direction is low passed (`IMU_FILTER` 0.25 per sample) and only updates while
`|g|` is within `IMU_SHAKE_G` (0.25) of 1 g, exactly as in Part C — shaking a
device does not tell you anything about gravity. The *violence* is low passed the
other way round: a **fast attack (0.5) and a slow release (0.06)**, so a flick
appears at once and decays over about a second. That asymmetry is the whole
"splash" effect, and it costs four lines.

There is a physical subtlety here, and this example uses it: while you shake the
board the reading *is* the acceleration, and an accelerometer measures `g - a`,
which is exactly the vector a liquid inside would be pushed by. So the raw reading
is mixed into the gravity direction in proportion to the shake (`FLUID_SHAKE_DIR`)
and the pull is scaled up (`FLUID_SHAKE_BOOST`). A hard shake therefore throws the
liquid at the far wall instead of only wobbling it - and it is not a special
effect, it is what the sensor reports.

The display's coordinate system is x right, **y down**, z into the screen, and
`gravityDisp` is the *upward* vector in that same frame, so liquid falls along
`-gravityDisp` — no axis juggling, no rotation matrix. Only the x and y parts can
push liquid around inside a flat panel, so what is used is the *in-plane* part of
that vector, and its length becomes `fluidMag`:

```
flat on the table, screen up:   gravityDisp = (0, 0, -1)
                                in-plane part 0   -> fluidMag 0: the levelling
                                                     term alone spreads the
                                                     liquid out evenly
upright in front of you:        gravityDisp = (0, -1, 0)
                                pull (0, +1)      -> the liquid pools at the
                                                     bottom of the tank
```

## E2. The simulation, in one function

`fluidStep(ux, uy, mag)` — a single sweep over the grid. No matrix, no velocity
field, no pressure solve:

```
    flow = (my mass - the neighbour's mass) * weight * rate
```

* **mass** is what a cell holds, measured in cells (1.0 = a full cell). The height
  of the liquid in a cell *is* its mass, so a height difference between two
  neighbours is a pressure difference - that is the entire reason a rule this
  small behaves like a liquid.
* **weight** of a direction is `FLUID_LEVELING` (a constant: a liquid wants to be
  flat, and this is what lets it flow sideways into a hollow) plus
  `FLUID_GRAVITY_BIAS · max(0, u · dir)`, i.e. gravity only ever makes a direction
  *easier*. Each rate is the direction's weight divided by the sum of all four, so
  the weights are relative, never absolute.
* **rate** is `FLUID_FLOW_RATE` of the difference, clamped twice: to
  `FLUID_MAX_FLOW` cells per substep in one direction (the viscosity that keeps a
  splash from exploding), and to what the cell actually holds.
* **conservation is exact.** Every flow is applied as one subtraction and one
  addition of the same number into a delta array, and that array is applied to the
  mass field afterwards - so the total amount of liquid cannot drift, whatever
  order the sweep runs in. The total is on the serial line once a second
  (`liquid …`) and it stays put.
* **`FLUID_VISCOSITY`** runs the same head-difference rule once more with pure
  levelling weights, which rounds off the grid's staircase and makes the liquid a
  little thicker.
* **time**: one substep is `FLUID_PHYS_MS` (16.6 ms) of simulated time and the
  number of substeps follows the measured frame time (up to `FLUID_MAX_SUBSTEPS`),
  so the liquid moves at the same speed at 20 fps and at 60.

The tank is a disc (`FLUID_TANK_ROUND`, radius `FLUID_TANK_RADIUS`): cells whose
centre is outside it count as "outside" and neither give nor receive liquid, which
is what makes the liquid pool in a bowl instead of a box. `fluidReset()` pours in
`FLUID_FILL_PCT` percent of the tank's cells, spread evenly along each row, so a
fill below one cell per row becomes a thin film rather than a stripe.

## E3. The renderer: a heightfield out of a grid

The simulation is a grid, the picture is not, and that is entirely the renderer's
doing. Once per frame, `fluidPlan()`:

1. picks the axis gravity pulls along, then walks every line of the grid **across**
   it and adds up the mass of the cells connected to the floor. That sum is the
   height of the liquid in that line;
2. cuts that height into `FLUID_SHADE_BANDS` bands and turns each band into **one
   rectangle for the whole line**, not one per cell - which is why a 40 × 40 grid
   costs about 130 rectangle fills per frame instead of 1600;
3. draws everything that holds mass but is *not* connected to the floor - spray
   thrown up by a shake - as a small square per cell, sized by how much it holds;
4. adds the meniscus: a two pixel additive line where the liquid meets the air,
   which is what makes the surface read as a surface.

Because the sweep follows gravity, a display lying on its side pools against the
left or right wall using the very same code, and every run is clamped to the tank's
pixel span in that direction, so the liquid never leaves the circle.

Colour comes from one function, `fluidColour(t, rel, surface)`, with three
palettes: `0` a constant colour whose brightness follows depth, `1` RGB (the hue
travels `FLUID_HUE_DEPTH` degrees from the floor to the surface and drifts with
time), `2` thermal (the base colour, white hot at the surface and in the spray).
`FLUID_SHADE_BANDS 1` collapses the whole body to one flat rectangle per line, if
you want the cheapest possible look.

## E4. Why this is fast, and what the numbers mean

Two decisions do all the work:

* **streaming mode.** The scene goes into 32 line strips of internal DMA RAM and
  each strip is sent while the next one is drawn. There is no 651 kB framebuffer in
  PSRAM to draw into and to copy out again - for C-style per-pixel code that copy
  is usually a bigger cost than the maths itself.
* **rectangle fills instead of per-pixel maths.** 130-200 `fillRect()` calls paint
  the liquid (about 1 ms for a full screen worth of pixels), and the automaton
  itself is about 0.4 ms per substep at 40 × 40.

The sketch measures itself: `FLUID_SHOW_PROFILE` puts the numbers on the screen and
the same line goes to the serial port once a second.

| field | meaning |
|---|---|
| `FPS` | frames per second over the last second |
| `PHY` | the automaton, all substeps of this frame (ms) |
| `PLN` | grid → rectangle list (ms) |
| `DRW` | drawing the strips on the CPU (ms) |
| `DMA` | the rest of the frame: QSPI, the overlap, and waiting for a strip buffer |
| `MB/s` | `W · H · 3 bytes / frame time` — the bandwidth the panel is really getting |

The last one is the honest one, and it is the answer to "can this be faster". A
full frame is 466 · 466 · 3 = 651 kB, so at the library's 80 MHz QSPI
(`AMOLED_QSPI_CLOCK_HZ`) the transfer time alone is ~16 ms: **55-60 fps is the hard
ceiling of a full-screen update**, and at 40 MHz it is ~33 ms, i.e. about 30 fps.
If `MB/s` is near 33 the bus is healthy and `FPS` is telling the truth about it; if
it is far below that, the panel is running slower than configured, or something
else is holding the CPU (a blocked USB CDC write, for example).

A sketch that pushes a full screen every frame cannot be sped up by making the
simulation cheaper - only by not pushing the screen. That is exactly why the
*previous* version of this example (22 additive glows written into a PSRAM
framebuffer, then pushed in bands) measured far lower than the bus allows: per-pixel
float maths in PSRAM, plus a PSRAM → internal RAM copy on every push, dwarf the
QSPI transfer they are meant to feed.

## E5. Where each piece lives in the sketch

| Piece | In `09_IMU_Fluid` | Notes |
|---|---|---|
| the I2C helpers | `imuRead()`, `imuWrite()` | identical to Part C — plain `Wire` |
| chip bring-up | `imuBegin()` | WHO_AM_I at `0x6B`/`0x6A`, four register writes |
| one sample | `imuReadRaw()` | STATUS0 bit 0, then a 6 byte burst, converted to g |
| direction **and** violence | `imuUpdate()` | fast attack / slow release, and the shake gate for the direction |
| gravity → the automaton | `fluidGravityUpdate()` | in-plane part of `-gravityDisp`, shake mixed in by `FLUID_SHAKE_DIR` |
| the grid and the tank | `fluidGridBuild()` | cell size, round mask, per line cell and pixel spans |
| pouring the liquid in | `fluidReset()` | `FLUID_FILL_PCT` of the tank's cells, spread along each row |
| the simulation | `fluidStep()`, `fluidRelax()`, `fluidApplyDelta()` | the rule of E2, `FLUID_MAX_SUBSTEPS` times per frame |
| painting with a finger | `fluidPaint()` | adds mass under the touch |
| grid → rectangles | `fluidPlan()` | the heightfield of E3, bands, meniscus, spray |
| the picture | `fluidDrawStrip()`, `hudText()`, `hudTextCentered()` | one strip per call, screen coordinates, clipped by the canvas |
| the readouts | `fluidBuildFpsLine()`, `fluidBuildProfileLine()`, `fluidBuildImuLine()` | rebuilt when they change, drawn inside every strip |
| the loop | the sections of `loop()` | input, simulate, plan, draw, timing, readout, serial commands, frame cap |

Library API used, and nothing else: `begin()`, `setBrightness()`,
`beginFrame()`/`nextStrip()`/`pushStrip()`/`endFrame()`/`waitIdle()`, `canvas()`,
`fillScreen()`, `fillRect()`, `addRect()`, `addCircle()`, `windowY0()`,
`windowY1()`, `drawTextBlend()`, `textWidth()`,
`touch().update()/tapped()/heldOnce()/isDown()/x()/y()`, plus the colour helpers
`amoledHSV()`, `amoledScale()`, `amoledMix()` and `amoledRGB()`.


# Part D — where to look next

* **`README.md`** (this folder) — the installed API: every class, every method,
  the config defines, the examples, the tearing notes, troubleshooting.
* **`examples/`** — 01 Hello Colours (static test card), 02 Glow Orbs, 03
  Contrast Test (hand written pixel loops), 04 Touch Demo, 05 Starfield
  (streaming), 06 Rotating Cube (3D wireframe + dirty rectangle), 07 Rotating
  Square (smallest possible animation with a frame rate counter), 08 IMU Cube
  (the cube held upright by gravity), 09 IMU Fluid (a grid liquid that pools,
  splashes and spreads, simulated in a cellular automaton and drawn as a
  heightfield in streaming mode).
* **`F:\skeces\amoled_rotating_cube`**, **`F:\skeces\amoled_imu_cube`** and
  **`F:\skeces\amoled_imu_fluid`** — the same sketches as standalone projects with
  their own READMEs, so you can hack on them without touching the library.
* **`src/AMOLED_Config.h`** — the one file to edit: pins, colour depth, QSPI
  clock, rotation, strip size, touch, framebuffer.
* **Ideas that would be next**, roughly in order of payoff:
  1. use the **gyro** as well — the gyro's angular rate plus the accelerometer
     direction is a proper attitude filter (a complementary filter is a dozen
     lines), which makes the cube follow quick rotations without lag;
  2. **throw gestures**: integrate the gyro on a flick and let the cube spin on,
     decaying over a second or two;
  3. carry that gyro rate into **`09_IMU_Fluid`**: with the angular velocity known
     the liquid could carry a real inertia (it would run to the wall as you turn
     the display and slosh *back* when you stop), instead of only reacting to the
     tilt it can currently see;
  4. a **spirit level** or bubble level screen — the same `gravityDisp` vector,
     a completely different use, and about thirty lines of canvas code;
  5. the shape itself: the same maths drives any set of vertices, so a second
     example could swap the cube for a tetrahedron or a pyramid with two tables
     changed.






