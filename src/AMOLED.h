/*
 * AMOLED.h
 * ============================================================================
 *  The display object of the WaveshareAMOLED library.
 *
 *      AMOLED amoled;                  // one display, like TFT_eSPI's tft
 *
 *      void setup() {
 *          amoled.begin();             // panel + touch + tables
 *          amoled.setBrightness(255);
 *      }
 *
 *  Two ways to draw:
 *
 *  1) STREAMING (fastest, for full screen animation)
 *         amoled.beginFrame();
 *         while (amoled.nextStrip(cv)) {
 *             cv.clearScreen();
 *             cv.addGlow(233, 233, 120, AMOLED_ORANGE);
 *             amoled.pushStrip();     // async DMA, overlaps with the next strip
 *         }
 *         amoled.endFrame();
 *     The canvas gives you one horizontal strip at a time, drawn in internal
 *     DMA RAM - no framebuffer needed, ~25-30 fps for a full RGB888 frame at
 *     40 MHz QSPI.
 *
 *  2) FRAMEBUFFER (classic TFT style: draw anywhere, then push)
 *         amoled.beginFramebuffer();          // 651 kB in PSRAM
 *         AMOLED_Canvas &fb = amoled.canvas();
 *         fb.fillRect(10, 10, 100, 60, AMOLED_RED);
 *         fb.drawTextCentered(233, 100, "HELLO", AMOLED_WHITE, 2);
 *         amoled.push();                      // or pushRect() for a region
 *
 *  Everything else (rotation, brightness, touch, colours) is documented in
 *  AMOLED_Canvas.h / AMOLED_Touch.h / AMOLED_Colors.h.
 *
 *  NOTE: like TFT_eSPI this driver supports ONE panel instance - the low
 *  level state lives in AMOLED.cpp, the class only holds the canvas, the
 *  touch object and the pointers, so this header stays free of ESP-IDF
 *  includes.
 * ============================================================================
 */
#pragma once

#include <stdint.h>
#include <Arduino.h>

#include "AMOLED_Config.h"
#include "AMOLED_Colors.h"
#include "AMOLED_Canvas.h"
#include "AMOLED_Touch.h"

class AMOLED
{
public:
    /* ================================================================
     *  Life cycle
     * ================================================================ */

    /* Initialise everything: panel detection, QSPI bus, controller, strip
     * buffers, lookup tables and touch, and set the brightness to maximum.
     * Returns false if the panel could not be brought up (then check the
     * wiring or AMOLED_FORCE_PANEL_ID in AMOLED_Config.h). */
    bool begin(void);
    bool ready(void) const { return _ready; }

    /* ================================================================
     *  Information
     * ================================================================ */
    uint8_t     controllerId(void) const   { return _panelId; }     /* 0x86 / 0xFF  */
    const char *controllerName(void) const;                         /* "SH8601" ... */
    int         width(void) const          { return AMOLED_WIDTH; }
    int         height(void) const         { return AMOLED_HEIGHT; }
    int         bytesPerPixel(void) const  { return AMOLED_BPP; }
    uint8_t     colorDepth(void) const     { return AMOLED_COLOR_DEPTH; }

    /* ================================================================
     *  Panel control
     * ================================================================ */

    /* 0 .. 255.  There is no backlight pin - brightness is a register inside
     * the controller, so this reacts instantly and without flicker. */
    void    setBrightness(uint8_t level);
    uint8_t getBrightness(void) const { return _brightness; }

    /* display on/off (controller commands 0x29 / 0x28) */
    void    setDisplayOn(bool on);

    /* 0, 90, 180 or 270 degrees - software rotation by strided writes */
    void    setRotation(uint16_t degrees) { AMOLED_Canvas::setRotation(degrees); }
    uint16_t getRotation(void) const { return AMOLED_Canvas::rotation(); }

    /* blocking fade of the brightness register - looks like a real fade */
    void    fadeBrightness(uint8_t to, uint16_t ms);

    /* ================================================================
     *  Streaming mode - render one frame, strip by strip
     * ================================================================ */

    /* start a frame; the first nextStrip() then hands you the top strip */
    void beginFrame(void);

    /* Hand the canvas the next strip to draw into.  Returns false once the
     * frame is complete.  After drawing, call pushStrip() - it queues the
     * strip asynchronously so the CPU draws the next one while the DMA is
     * still sending this one. */
    bool nextStrip(AMOLED_Canvas &canvas);
    void pushStrip(void);

    /* end of frame - does NOT wait, the DMA keeps running in the background,
     * so the next beginFrame() can start straight away */
    void endFrame(void);

    /* block until every queued strip has been sent (call it before changing
     * the brightness, powering down, or anything time critical) */
    void waitIdle(void);

    /* convenience: draw a whole frame with a single call.  The callback runs
     * once per strip, exactly like the loop above. */
    typedef void (*FrameCallback)(AMOLED_Canvas &canvas, uint32_t tMs);
    void drawFrame(FrameCallback callback, uint32_t tMs = 0);

    /* ================================================================
     *  Framebuffer mode - draw anywhere, push when you are done
     * ================================================================ */

    /* allocate the full framebuffer in PSRAM (651 kB at 24 bpp) and point the
     * canvas at it.  Returns false when there is no PSRAM or no memory. */
    bool beginFramebuffer(void);
    bool framebufferReady(void) const { return _fb != nullptr; }

    /* the canvas: draws into the framebuffer in framebuffer mode, or into the
     * current strip while a streaming frame is running */
    AMOLED_Canvas &canvas(void) { return _canvas; }

    /* send the whole framebuffer to the panel, strip by strip.  The strips
     * overlap internally; when the call returns the framebuffer can be drawn
     * into again safely. */
    void push(void);

    /* Send just one rectangle of the framebuffer (panel coordinates).
     * The region is packed in bands of AMOLED_STRIP_LINES rows and each band
     * goes out as one transfer, and every transfer is a whole number of 32
     * bit words, so it is safe for a QSPI/DMA panel.  When the call returns
     * the framebuffer can be drawn into again. */
    void pushRect(int x0, int y0, int x1, int y1);

    /* raw access to the framebuffer memory (panel order: row * width * bpp) */
    uint8_t *framebufferMemory(void) const { return _fb; }

    /* ================================================================
     *  Touch
     * ================================================================ */
    AMOLED_Touch &touch(void) { return _touch; }

    /* ================================================================
     *  Colour shortcuts
     * ================================================================ */
    static uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) { return amoledRGB(r, g, b); }
    static uint32_t hsv(float h, float s, float v)       { return amoledHSV(h, s, v); }
    static uint32_t hue(float h)                         { return amoledHue(h); }
    static uint32_t palette(uint16_t i)                  { return AMOLED_Canvas::palette(i); }

private:
    AMOLED_Canvas _canvas;
    AMOLED_Touch  _touch;
    bool          _ready         = false;
    uint8_t       _panelId       = 0;
    uint8_t       _brightness    = 0xFF;
    uint8_t      *_fb            = nullptr;
    int           _stripY        = 0;   /* next strip to hand out         */
    int           _pendingY      = 0;   /* screen line of the drawn strip  */
    bool          _pending       = false;
    uint8_t      *_stripBuf      = nullptr;
};
