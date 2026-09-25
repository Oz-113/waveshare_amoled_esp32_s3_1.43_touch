# WaveshareAMOLED

A **TFT_eSPI style Arduino library** for the
**Waveshare ESP32-S3-Touch-AMOLED-1.43** round AMOLED module
(466 × 466, SH8601 **or** CO5300 controller, FT3168 touch, ESP32-S3R8).

```cpp
#include <WaveshareAMOLED.h>
AMOLED amoled;

void setup() {
    amoled.begin();                     // panel + touch + lookup tables
    amoled.setBrightness(255);
    amoled.canvas().drawTextCentered(233, 220, "HELLO", AMOLED_WHITE, 3);
    amoled.push();                      // blit to the panel
}
void loop() { }
```

* **True 24-bit colour (RGB888, 16.7 M)** over QSPI - 3 bytes per pixel,
  `COLMOD 0x77`. RGB565 is a one-line fallback in `AMOLED_Config.h`.
* **Nothing to install** besides this library: it uses only the ESP-IDF
  `esp_lcd` layer that ships inside the Arduino-ESP32 core plus `Wire`.
* **Double buffered strip pipeline**: the CPU renders strip *N+1* while the QSPI
  DMA still sends strip *N* (`beginFrame()` / `nextStrip()`). That removes the
  flicker of drawing straight to the panel; the seam you may still see on a hard
  bright edge is explained in section 2.
* **Optional full framebuffer in PSRAM** for classic "draw, then `push()`"
  programming (651 kB, the board has 8 MB).
* Drawing API with the usual names: `fillScreen`, `fillRect`, `drawRect`,
  `drawLine`, `drawCircle`, `fillCircle`, `drawPixel`, `drawText`,
  `drawTextCentered` … plus the AMOLED specials `addGlow`, `addArc`,
  `addPixelScaled`, `blendPixel` (light *added* on top of black).
* Software **rotation** 0/90/180/270 at run time, **brightness** with a
  flicker-free hardware fade, **touch** with tap/hold helpers.
* A single config file to edit, exactly like TFT_eSPI's `User_Setup.h`.

> **`GUIDE.md`** in this folder is the long-form documentation: the maths of the
> `amoled_rotating_cube` example, where every component comes from (Arduino core
> / ESP-IDF / C++ / this library) and a walkthrough of the internals with line
> numbers — what gets rendered when, and on which line the pixels are pushed to
> the panel.

All **nine** examples are verified to compile with arduino-cli + esp32 core
3.3.11 for `esp32:esp32:esp32s3` (16 MB flash, OPI PSRAM, USB CDC on boot), in
both 24 bpp (RGB888) and 16 bpp (RGB565). The frame rate numbers in this README
and in `GUIDE.md` are calculated from the QSPI byte budgets; the sketches print
the measured values on their serial port.

---

## 1. Install

### Arduino IDE 2.x
1. Copy the folder **`WaveshareAMOLED`** (this folder, renamed from
   `waveshare_amoled_lib`) into your sketchbook libraries directory:

   ```
   Documents\Arduino\libraries\WaveshareAMOLED\
   ```

   On Windows the quick way: **Sketch → Include Library → Add .ZIP Library…**
   after zipping this folder, or just copy it as shown above and restart the IDE.
2. Check `src/AMOLED_Config.h` once - that is the file with the pins, the
   colour depth and the QSPI clock (see section 5).
3. Open one of the examples: **File → Examples → WaveshareAMOLED**.

### Board settings (Tools menu)
| Option | Value |
|---|---|
| Board | **ESP32S3 Dev Module** |
| USB CDC On Boot | **Enabled** (so `Serial` is the USB port) |
| CPU Frequency | 240 MHz (WiFi) |
| Flash Mode / Size | QIO 80 MHz / **16MB (128Mb)** |
| Partition Scheme | 16M Flash (3MB APP/9.9MB FATFS) |
| PSRAM | **OPI PSRAM** (needed for framebuffer mode) |

No other library has to be installed.

### The examples

| Example | PSRAM | What it shows |
|---|---|---|
| `01_Hello_Colours` | no | the colour space, text, glow and arcs — a static test card |
| `02_Glow_Orbs` | no | additive light: orbs that brighten where they overlap |
| `03_Contrast_Test` | no | hand written pixel loops with `ptr()` / `strideX()` |
| `04_Touch_Demo` | **yes** | touch + framebuffer + `pushRect()` (only the HUD is re-sent) |
| `05_Starfield` | no | streaming mode, 240 points of light, hold the screen to hyperjump |
| `06_Rotating_Cube` | **yes** | 3D wireframe cube (set `CUBE_FACES 1` for the translucent faces), dirty rectangle updates, fps counter |
| `07_Rotating_Square` | no | the smallest animation there is: a rotated, filled square with a serial fps counter |
| `08_IMU_Cube` | **yes** | the same cube held upright by the on board QMI8658 accelerometer: it hangs from real gravity and spins about the world vertical (tap = freeze, hold = axis signs) |
| `09_IMU_Fluid` | n | a **grid** liquid: a cellular automaton over a 40 x 40 cell tank, drawn as a heightfield, driven by the same accelerometer — it pools at the low side, splashes when you shake it and spreads out when you lay it flat (drag = push, tap = palette, hold = axis signs). Streaming mode, so no framebuffer |

### arduino-cli
```bash
arduino-cli compile --libraries /path/to/libraries \
  --fqbn "esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc,CPUFreq=240" \
  examples/01_Hello_Colours
```

---

## 2. Two ways to draw

### a) Streaming strips — fastest, for full screen animation

The frame is split into horizontal strips; each strip is drawn in internal DMA
RAM and pushed asynchronously, so rendering and the QSPI transfer overlap.

```cpp
AMOLED_Canvas &cv = amoled.canvas();

amoled.beginFrame();
while (amoled.nextStrip(cv)) {
    cv.clearScreen();                          // black, pixels off
    cv.addGlow(233, 233, 120, amoled.palette(millis() / 8));   // soft light
    cv.drawTextCentered(233, 60, "STREAMING", AMOLED_WHITE, 2);
    amoled.pushStrip();                        // queue this strip (async)
}
amoled.endFrame();                             // does not wait
```

`nextStrip()` blocks until one of the two strip buffers is DMA-free, so a
buffer is never overwritten while it is still being transmitted. Nothing has
to be flicker-free by yourself: the panel is written strip by strip from top to
bottom, exactly like a normal LCD refresh.

### b) Framebuffer — classic "draw, then push"

```cpp
amoled.beginFramebuffer();                     // 651 kB in PSRAM, once
AMOLED_Canvas &fb = amoled.canvas();

fb.fillScreen(AMOLED_BLACK);
fb.fillRect(20, 20, 120, 90, AMOLED_RED);
fb.drawCircle(233, 233, 100, AMOLED_CYAN);
fb.drawTextCentered(233, 233, "FRAMEBUFFER", AMOLED_WHITE, 2);

amoled.push();                                 // whole frame (async strips)
amoled.pushRect(20, 20, 120, 90);              // or just one region
```

`push()` copies the framebuffer through the strip buffers (so it is safe on
any board; the strips overlap internally, ~33 ms for a full frame at 40 MHz).
`pushRect()` packs the region into the strip buffers in bands of
`AMOLED_STRIP_LINES` rows and sends each band as **one** transfer, so a 300-row
box needs 10 transfers instead of 300; a full width region (what 06 does) packs
with a single `memcpy` per band. Every transfer is a whole number of 32 bit
words. Both calls return when the framebuffer may be drawn into again.

### Dirty rectangles, `setClip()` and tearing

A partial update is only safe if **nothing draws outside the rectangle you are
going to send** — a stray pixel that never gets pushed stays on the panel
forever. `setClip(x0, y0, x1, y1)` is the belt-and-braces way to guarantee that:

```cpp
fb.setClip(ux0, uy0, ux1, uy1);   // nothing can escape this box now
fb.fillRect(ux0, uy0, ux1, uy1, AMOLED_BLACK);
drawTheMovingThing(fb);
fb.resetClip();                   // back to the whole window
amoled.pushRect(ux0, uy0, ux1, uy1);
```

Every primitive — including `addGlow()`, `drawText()` and hand written loops
using `ptr()` — clamps to that rectangle. `width()`/`height()` keep describing
the whole window, not the clip, so your layout code does not change.

**Tearing.** The panel is written while it scans out, and this board has no
tear-effect (TE) pin wired, so the seam between the new and the old part of the
picture is always somewhere on screen. Smooth gradients hide it completely;
hard, bright edges (a wireframe cube, a huge square) show it as a line that
looks *cut* — and if the loop runs faster than the panel refreshes, several
frames are in flight at once and the shape looks copied into a few horizontal
bands.

What helps, in order:

1. **Keep the update rate at or below the panel's refresh rate.** At 60 updates
   per second only one seam can be on screen; at 200 you see three or four.
   `06_Rotating_Cube` has `CUBE_FPS_CAP` for exactly this.
2. **Send less per frame** — the dirty rectangle (section 2b) sends a box
   instead of 651 kB.
3. **A faster bus** — `AMOLED_QSPI_CLOCK_HZ` 80 MHz instead of 40 halves the
   time the panel is being written mid-scan.
4. **A motion trail** — drawing two dimmer copies a few degrees behind the shape
   turns the seam into blur (`SQUARE_TRAIL` in `07_Rotating_Square`).

To tell tearing and rendering bugs apart: freeze the animation (`CUBE_SPIN_X`
and `CUBE_SPIN_Y` to `0.0f`). Artifacts that disappear were tearing; artifacts
that stay are in the drawing code.


> Do not mix `push()`/`pushRect()` with an active streaming frame - call
> `amoled.endFrame()` first.

### Two pitfalls worth knowing

* **Every strip you drew must be pushed.** `pushStrip()` is what queues it;
  if you forget it, the next `nextStrip()` pushes it for you, so nothing is
  lost - but if you `break` out of the loop in the middle of a frame, that
  strip never reaches the panel.
* **Additive drawing in framebuffer mode accumulates.** `addGlow()`,
  `addPixel()`, `addArc()` … only add light, and the framebuffer is not cleared
  for you, so a glow drawn every frame gets brighter and brighter until the
  whole screen is washed out. Two ways out: use streaming mode (each strip is
  drawn from scratch every frame), or repaint the affected rectangle before you
  draw into it (see `04_Touch_Demo`, where the status panel is repainted
  opaquely and the glow is kept inside it).

---

## 3. API reference — `AMOLED` (the display object)

| Method | What it does |
|---|---|
| `bool begin()` | Full init: panel detection (SH8601/CO5300), QSPI bus, controller init, strip buffers, lookup tables, touch. Sets the brightness to max. Returns `false` if the panel did not come up. Call once from `setup()`. |
| `bool ready()` | `true` after a successful `begin()`. |
| `uint8_t controllerId()` | Raw ID byte: `0x86` SH8601, `0xFF` CO5300. |
| `const char *controllerName()` | `"SH8601"` / `"CO5300"` / `"unknown"`. |
| `int width()`, `int height()` | Panel geometry (466 × 466). |
| `int bytesPerPixel()` | 3 for RGB888, 2 for RGB565. |
| `uint8_t colorDepth()` | 24 or 16, from `AMOLED_Config.h`. |
| `void setBrightness(uint8_t level)` | 0…255. Register based, so it is instant and flicker free. |
| `uint8_t getBrightness()` | Last value that was set. |
| `void fadeBrightness(uint8_t to, uint16_t ms)` | Blocking fade - looks like a real brightness ramp (10 ms per step). |
| `void setDisplayOn(bool on)` | Controller commands `0x29` / `0x28`. |
| `void setRotation(uint8_t deg)` | 0 / 90 / 180 / 270, at run time. Software rotation by strided writes (one add per pixel, no extra buffer). |
| `uint8_t getRotation()` | Current rotation. |
| `void beginFrame()` | Start a streaming frame at the top strip. |
| `bool nextStrip(AMOLED_Canvas &cv)` | Hand out the next strip; `false` when the frame is finished. |
| `void pushStrip()` | Queue the strip you have just drawn (asynchronous DMA). |
| `void endFrame()` | Ends the frame (does not wait). |
| `void waitIdle()` | Block until every queued strip has been transmitted. |
| `void drawFrame(FrameCallback cb, uint32_t tMs = 0)` | Convenience: runs `cb(canvas, tMs)` once per strip for one whole frame. |
| `bool beginFramebuffer()` | Allocates 466 × 466 × bpp bytes in PSRAM and points the canvas at it. |
| `bool framebufferReady()` | Is there a framebuffer? |
| `AMOLED_Canvas &canvas()` | The canvas: framebuffer in framebuffer mode, current strip in streaming mode. |
| `void push()` | Send the whole framebuffer (asynchronous strips). |
| `void pushRect(x0, y0, x1, y1)` | Send one region: the rows are packed into the strip buffers in bands and each band goes out as one DMA transfer, always a whole number of 32 bit words (a 300-row box needs 10 transfers, not 300; a full width region packs with one `memcpy` per band). Returns when the framebuffer is free again. |
| `uint8_t *framebufferMemory()` | Raw pointer to the framebuffer. |
| `AMOLED_Touch &touch()` | The touch object (see section 5). |
| `static uint32_t rgb(r,g,b)` | `amoledRGB()` shortcut. |
| `static uint32_t hsv(h,s,v)` / `hue(h)` | Colour wheel shortcuts. |
| `static uint32_t palette(uint16_t i)` | One of the 1024 rainbow colours (perfectly smooth 24 bit hues). |

---

## 4. API reference — `AMOLED_Canvas` (the drawing engine)

**Colours** are always `0xRRGGBB`. **Coordinates** are logical (rotated) screen
coordinates; everything is clipped to the current window automatically.
The colour argument is called `c` below.

**Window / geometry**

| Method | What it does |
|---|---|
| `void beginStrip(uint8_t *buf, int stripY, int lines)` | Point the canvas at one strip of screen lines (used by the driver). |
| `void beginFull(uint8_t *buf)` | Point the canvas at a full framebuffer. |
| `void setClip(x0, y0, x1, y1)` | Narrow the drawing area to this rectangle (intersected with the window). Everything outside is silently ignored - the safe way to do a partial update. |
| `void resetClip()` | Back to the whole window. |
| `static void setRotation(uint8_t deg)` | 0/90/180/270 - library wide. |
| `static void initTables()` | Build palette/glow/arc tables (`begin()` does it for you). |
| `int width()`, `height()` | Size of the *window* (the strip / framebuffer), unaffected by `setClip()`. |
| `int clipX0()`, `clipY0()`, `clipX1()`, `clipY1()` | The active clipping rectangle (the window while no clip is set). |
| `uint8_t *ptr(int x, int y)` | Address of a pixel - for your own fast loops. |
| `int strideX()`, `strideY()` | Byte step per +1 logical x / y (negative under rotation). |

**Opaque drawing (replaces the pixel)**

| Method | What it does |
|---|---|
| `void clearScreen()` | Whole window to `#000000` (memset - very fast, and on an AMOLED those pixels are then *off*). |
| `void fillScreen(uint32_t c)` | Whole window to `c`. |
| `void fillRect(x0, y0, x1, y1, c)` | Filled rectangle (inclusive coordinates). |
| `void drawRect(x0, y0, x1, y1, c)` | Rectangle outline. |
| `void drawPixel(x, y, c)` | Single pixel. |
| `void drawHLine(y, x0, x1, c)` / `drawVLine(x, y0, y1, c)` | Horizontal / vertical line. |
| `void drawLine(x0, y0, x1, y1, c)` | Bresenham line. |
| `void drawCircle(cx, cy, r, c)` | Circle outline (midpoint algorithm). |
| `void fillCircle(cx, cy, r, c)` | Filled circle. |
| `void shadeRect(x0, y0, x1, y1, c, alpha)` | Translucent rectangle (`alpha` 0…255) - HUD backgrounds. |

**Additive drawing (adds light, saturates at 255) — the AMOLED specials**

| Method | What it does |
|---|---|
| `void addPixel(x, y, c)` | Add one pixel of light. |
| `void addPixelScaled(x, y, c, gain)` | Add light with a 0…255 gain (fading sparkles). |
| `void blendPixel(x, y, c, alpha)` | Alpha blend instead of adding. |
| `void addRect(x0, y0, x1, y1, c)` | Add a filled block of light. |
| `void addHLine(...)`, `addVLine(...)`, `addLine(...)` | Additive lines. |
| `void addCircle(cx, cy, r, c, gain = 255)` | Additive ring. |
| `void addGlow(cx, cy, radius, c, gain = 255)` | **Soft radial light blob.** The radius is the visible radius in pixels; the falloff comes from a 256 entry curve, so a whole blob costs one multiply, one shift and one table lookup per channel. On pure black this looks like real light emission. |
| `void addArc(cx, cy, radius, startDeg, endDeg, thickness, c, gain = 255)` | Additive arc / ring. Angles in degrees, `0` = 3 o'clock, clockwise, so `-90` = 12 o'clock. The arc is continuous (two stamps per direction step). |

**Text (built in 5×7 font)**

| Method | What it does |
|---|---|
| `static int textWidth(const char *s, int size = 1)` | Width in pixels. `size` is the integer scale 1…4, character pitch is `6 * size`. |
| `static int textHeight(int size = 1)` | `7 * size`. |
| `void drawText(x, y, s, c, size = 1)` | Opaque text (`y` is the top edge). |
| `void drawTextAdd(x, y, s, c, size = 1)` | Additive text - glows over the animation. |
| `void drawTextBlend(x, y, s, c, size = 1, alpha = 255)` | Semi transparent text. |
| `void drawTextCentered(cx, y, s, c, size = 1)` | Centred on `cx` - what you want on a round display. |
| `void drawTextCenteredBlend(cx, y, s, c, size, alpha)` | Centred + alpha. |

**Shared lookup tables**

| Method | What it does |
|---|---|
| `static uint32_t palette(uint16_t i)` | One of 1024 rainbow colours (24 bit, saturation boosted). |
| `static int cosStep(int i)`, `sinStep(int i)` | `cos/sin * 256` of the 512 direction steps (`i` = angle × 512/360). |
| `static uint8_t glowCurve(uint8_t i)` | The glow falloff curve (`i = 256 × d²/R²`). |
| `static void initTables()` | Builds all three (called by `amoled.begin()`). |

Almost all primitives have a `static inline` twin for hand written loops:
`amoledPxSet(ptr, r, g, b)`, `amoledPxAdd`, `amoledPxAddScaled`,
`amoledPxMul`, `amoledPxBlend` work on a raw `uint8_t *` from `ptr()`.

---

## 5. API reference — `AMOLED_Touch`

| Method | What it does |
|---|---|
| `bool begin()` | Starts `Wire` on SDA 47 / SCL 48 and wakes the FT3168. Called by `amoled.begin()`. Returns false if it does not answer. |
| `bool available()` | Is the controller alive? |
| `bool read(uint16_t &x, uint16_t &y)` | Raw read: returns true while the panel is touched, fills x/y (0…465). |
| `void update()` | Polls the controller (at most every `AMOLED_TOUCH_POLL_MS`, default 8 ms) and tracks the gestures. Call it often, e.g. once per `loop()`. |
| `bool isDown()`, `x()`, `y()` | Current state / last position. |
| `bool tapped()` | `true` once for a short (< 500 ms) press; the flag is cleared by reading it. |
| `bool held(ms)` | `true` while the current press has lasted at least `ms` ms (continuously true). |
| `bool heldOnce(ms)` | `true` exactly once per press, when it reaches `ms` ms - the one to use for "hold = do something". |
| `uint32_t pressTime()` | Length of the current press in ms (0 if not pressed). |

---

## 6. Colours

Every colour is a 24 bit `0xRRGGBB` value - exactly what the panel takes.

```cpp
amoledRGB(255, 128, 0)          // build one from its channels
amoledHSV(200.0f, 1.0f, 1.0f)   // colour wheel (h 0..360, s/v 0..1)
amoledHue(0.5f)                 // hue only, h 0..1
amoledScale(AMOLED_RED, 1, 2)   // half brightness red
amoledMix(a, b, 128)            // linear mix (128 = half way)
amoledTo565(c)                  // 0xRRGGBB -> RGB565
amoledFrom565(v)                // RGB565 -> 0xRRGGBB
AMOLED_Canvas::palette(i)       // 1024 entry rainbow, i & 1023
amoledR(c), amoledG(c), amoledB(c)
```

Named colours: `AMOLED_BLACK`, `WHITE`, `RED`, `GREEN`, `BLUE`, `CYAN`,
`MAGENTA`, `YELLOW`, `ORANGE`, `PURPLE`, `PINK`, `LIME`, `TEAL`, `NAVY`,
`MAROON`, `OLIVE`, `GREY`, `DARKGREY`, `LIGHTGREY`, `GOLD`, `SKYBLUE`,
`DEEPPINK`, `SPRINGGREEN`, `ORANGERED`, `DODGERBLUE`, `TURQUOISE`, `CRIMSON`,
`CHOCOLATE`, `INDIGO`, `SALMON`.

---

## 7. Configuration — `src/AMOLED_Config.h`

Like TFT_eSPI's `User_Setup.h`: the only file you normally edit.

| Define | Default | Meaning |
|---|---|---|
| `AMOLED_WIDTH`, `AMOLED_HEIGHT` | `466`, `466` | Panel geometry. |
| `AMOLED_PIN_CS` … `AMOLED_PIN_RST` | 9, 10, 11, 12, 13, 14, 21 | QSPI pins of the module. |
| `AMOLED_QSPI_CLOCK_HZ` | `40000000UL` | QSPI clock. `80000000UL` is the ESP32-S3 maximum - try it, the picture usually stays clean and you nearly double the frame rate. |
| `AMOLED_COLOR_DEPTH` | `24` | `24` = RGB888 (16.7 M colours, 3 bytes/pixel), `16` = RGB565 fallback. |
| `AMOLED_FORCE_PANEL_ID` | `0x00` | `0x00` = auto detect, `0x86` = force SH8601, `0xFF` = force CO5300. |
| `AMOLED_DEFAULT_ROTATION` | `0` | Start up rotation (0/90/180/270), changeable at run time. |
| `AMOLED_USE_BGR_ORDER` | `0` | Set to 1 if red and blue are swapped. |
| `AMOLED_RGB565_BYTE_SWAP` | `1` | Byte order of RGB565 words (only with depth 16). |
| `AMOLED_STRIP_LINES` | `32` | Strip height. Keep it **even** at 24 bpp (a 466 px RGB888 line is 1398 bytes). |
| `AMOLED_STRIP_BUFFERS` | `2` | Two is what makes rendering and DMA overlap; RAM cost = `buffers × lines × 1398 B`. |
| `AMOLED_TOUCH_ENABLE` | `1` | `0` = do not touch the I2C bus at all. |
| `AMOLED_TOUCH_PIN_SDA/SCL` | `47`, `48` | Touch I2C pins (shared with the IMU and the RTC). |
| `AMOLED_TOUCH_I2C_ADDR` | `0x38` | FT3168 address. |
| `AMOLED_TOUCH_I2C_HZ` | `400000` | I2C clock. |
| `AMOLED_TOUCH_POLL_MS` | `8` | Gesture polling period. |
| `AMOLED_FRAMEBUFFER_PSRAM` | `1` | Allocate the framebuffer in PSRAM (`0` = try internal RAM). |
| `AMOLED_SERIAL_LOG` | `1` | Print the panel info in `begin()` - only while a host has the USB port open. |
| `AMOLED_SAFE_RADIUS` | `226` | Radius of the circle the panel really shows (233 = the whole panel); used by the examples for their HUD layout. |

---

## 8. How it works / performance

```
one RGB888 frame = 466 × 466 × 3 = 651 468 bytes
QSPI at 40 MHz  = 4 lines × 40 Mbit/s = 20 MB/s   ->  ~33 ms per frame

beginFrame() / nextStrip() / pushStrip() pipeline:
    CPU draws strip N+1 into buffer B   while   DMA sends buffer A
    -> frame time = max(rendering, transfer), not the sum
    -> ~25-30 fps for a full screen animation at 40 MHz
    -> ~40-55 fps if AMOLED_QSPI_CLOCK_HZ is raised to 80 MHz
```

* The two strip buffers live in internal DMA-capable RAM. `nextStrip()` takes
  one through a counting semaphore whose tokens are returned by the QSPI DMA
  completion callback, so a buffer can never be overwritten while the DMA is
  still reading it (transfers complete in order, buffers are used in a strict
  rotation).
* Framebuffer mode copies each strip from PSRAM through the strip buffers, so
  it works on any board; it costs one extra memcpy per frame (~6 ms).
* Software rotation is just a different (strided) pixel mapping - no extra
  buffer, no extra transfer, about one add per pixel.
* Brightness is a controller register (`0x51`), not PWM: it reacts instantly,
  which is why `fadeBrightness()` looks like a real fade.
* A glow costs one multiply, one shift and one table lookup per pixel of its
  bounding box, and an AMOLED *adds* light, so overlapping glows merge into one
  bright body. `02_Glow_Orbs` and `05_Starfield` are built out of that;
  `addGlow()`'s two knobs are its radius (the area is what costs) and its gain.

---

## 9. Examples

| Example | What it shows |
|---|---|
| `01_Hello_Colours` | Framebuffer mode: named colours, 24 bit grey ramp, hue sweep, shapes, centred text, animated glow pushed with `pushRect()`. Start here. |
| `02_Glow_Orbs` | Streaming mode: six additive light blobs on true black, sparkles, touch pulls them to your finger. |
| `03_Contrast_Test` | The full test card: grey ramp, darkest/brightest 16 levels, moving 1-pixel checkerboard, frequency sweep, 24 bit hue sweep. |
| `04_Touch_Demo` | Touch: raw read, tap and hold gestures, an on-screen status panel updated with `pushRect()` only. |
| `05_Starfield` | 240 additive point lights with streaks, `held()` as a warp lever. |
| `06_Rotating_Cube` | Framebuffer mode + dirty rectangle + `setClip()`/`pushRect()`: a 3D wireframe cube with Lambert shading, depth fog and a rainbow, and an fps counter. Set `CUBE_FACES 1` for the translucent additive faces. |
| `07_Rotating_Square` | The smallest animation there is: one rotated, filled square, streaming vs framebuffer, and a 1-pixel tear/trail experiment (`SQUARE_TRAIL`). |
| `08_IMU_Cube` | The QMI8658 accelerometer read directly over I2C: the cube is rotated onto the measured gravity by the *shortest* arc (Rodrigues), so it keeps its top face at the sky and stands still in the room while the screen turns around it. |
| `09_IMU_Fluid` | The same accelerometer, a completely different use: a **grid liquid**. A cellular automaton moves mass between neighbouring cells of a 40 x 40 tank, gravity biases the flow, and a heightfield renderer turns that grid into ~150 rectangles per frame — so it *looks* like a fluid while the physics stays a few hundred microseconds. Streaming mode (no framebuffer, no PSRAM), with a measured breakdown of where the frame time actually goes, which is the honest way to tune a full-screen sketch. Drag to push the liquid, tap for the palette, hold for the axis signs. |

---

## 10. Troubleshooting

| Symptom | Fix |
|---|---|
| `begin()` returns false / nothing on screen | Check the pins in `AMOLED_Config.h`, then set `AMOLED_FORCE_PANEL_ID` to `0x86` (SH8601) or `0xFF` (CO5300). |
| Image shifted horizontally | Force the other panel ID - only the CO5300 needs the 6 px GRAM offset. |
| Garbage / stripes in 24 bit mode | Some panel batches only take 16 bpp over QSPI: set `AMOLED_COLOR_DEPTH 16` (and `AMOLED_RGB565_BYTE_SWAP` if colours look swapped). |
| Red and blue swapped | `AMOLED_USE_BGR_ORDER 1`. |
| Text is sideways | `amoled.setRotation(90 / 180 / 270)`. |
| Everything is slow and the sketch stalls | Something is blocking: do not write to `Serial` when no host is attached (the library guards its own prints, and `AMOLED_SERIAL_LOG 0` silences them). |
| HUD elements land in the dark corners | The visible area is a circle - keep everything inside `AMOLED_SAFE_RADIUS` (226) of the centre, or use `drawTextCentered()`. |
| Touch does not react | The touch IC shares I2C with the IMU/RTC; check `amoled.touch().available()` and the pins. |
| `beginFramebuffer()` returns false | PSRAM not enabled - set **PSRAM: OPI PSRAM** in the Tools menu (this board has 8 MB). |

---

## 11. Credits & licence

* `esp_lcd_sh8601.c/.h` - Espressif's SH8601 panel driver (Apache-2.0), taken
  unchanged from Waveshare's `ESP32-S3-AMOLED-1.43-Demo` package, including the
  CO5300 init sequence and the 6 pixel offset handling.
* `AMOLED_Probe.cpp` - bit-banged ID read, adapted from Waveshare's
  `read_lcd_id_bsp.c`.
* Everything else (panel layer, canvas, text, touch wrapper, examples) was
  written for this library. Use it however you like.
* Product documentation: <https://docs.waveshare.com/ESP32-S3-Touch-AMOLED-1.43>
