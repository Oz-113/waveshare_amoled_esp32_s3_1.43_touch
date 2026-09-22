# WaveshareAMOLED — the guide

Everything about this library and about the rotating cube demo (examples
`06_Rotating_Cube` and `F:\skeces\amoled_rotating_cube`): the maths, the
drawing, what each component is, which library or API it comes from, and what
happens on which line when a frame is rendered and pushed to the panel.

* **Part A** — the rotating cube project: the maths and the components.
* **Part B** — how this library works, from your sketch down to the QSPI wires.
* **Part C** — where to find the API reference (`README.md`), the examples and
  what to build next.

> Line numbers refer to **library version 1.0.2**. They are here to help you
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
| 4 | 3 faces (`fillQuad`), 12 edges (`drawEdge`), 8 glows (`addGlow`) | ~3 ms with faces, ~0.5 ms without |
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
| 4 | the three visible faces: cull, shade, `fillQuad()` |
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

# Part C — where to look next

* **`README.md`** (this folder) — the installed API: every class, every method,
  the config defines, the examples, the tearing notes, troubleshooting.
* **`examples/`** — 01 Hello Colours (static test card), 02 Glow Orbs, 03
  Contrast Test (hand written pixel loops), 04 Touch Demo, 05 Starfield
  (streaming), 06 Rotating Cube (3D + dirty rectangle), 07 Rotating Square
  (smallest possible animation with a frame rate counter).
* **`F:\skeces\amoled_rotating_cube`** — the same cube as a standalone sketch,
  with its own README, so you can hack on it without touching the library.
* **`src/AMOLED_Config.h`** — the one file to edit: pins, colour depth, QSPI
  clock, rotation, strip size, touch, framebuffer.

## C1. Next step: let the IMU hold the cube up

The board carries a QMI8658 6 axis IMU on the same I2C bus as the touch
controller, so it needs no extra wiring. The plan, in four steps:

1. **Read the accelerometer.** `Wire` on the same pins as `AMOLED_Touch`
   (SDA 47 / SCL 48), IMU address `0x6B`. Enable the accelerometer, then read six
   bytes from the data register: accel X, Y, Z as little endian `int16`
   (register map: `CTRL1` 0x02, `CTRL2` 0x03 for the accel range, `CTRL7` 0x08 to
   enable, data from 0x35). *Verify the addresses against Waveshare's own IMU
   example for this board before trusting them.*
2. **Low pass it.** `g = g * 0.85 + gNew * 0.15` — the cube should tilt
   smoothly, not shake with every hand movement.
3. **Build the rotation that puts `g` onto `(0, +1, 0)`**, i.e. the "up" of the
   cube's space. With `a` = the direction you want gravity to end up in
   (`(0, +1, 0)`), `b = normalised g`, `v = a × b`, `c = a · b`, `s² = v · v`,
   Rodrigues' formula gives the whole matrix in one line:
   `R = I + [v]× + [v]×² · (1 − c) / s²`
   (`[v]×` is the 3 × 3 skew-symmetric matrix of `v`).
4. **Apply `R` after yaw/pitch** in `rotatePoint()`, so the cube spins in board
   space but always keeps one corner pointing at the sky.

Where it goes: a `#define CUBE_USE_IMU` block in `06_Rotating_Cube`, an
`imuUpdate()` that runs once per frame, and `rotatePoint()` growing a second
rotation. Everything else (the dirty rectangle, the push, the fps counter) stays
exactly as it is — the IMU only changes the 8 corners and 6 normals that go in,
which is the point of keeping the maths at the top of the sketch.






