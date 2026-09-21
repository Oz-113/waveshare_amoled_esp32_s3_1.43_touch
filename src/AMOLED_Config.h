/*
 * AMOLED_Config.h
 * ============================================================================
 *  THE FILE YOU EDIT.  Exactly like TFT_eSPI's User_Setup.h: everything the
 *  driver needs to know about your board lives here, nothing else in the
 *  library has to be touched.
 *
 *  Board this library was written for:
 *      Waveshare ESP32-S3-Touch-AMOLED-1.43   (and the 1.43-B metal case one)
 *      466 x 466 AMOLED, SH8601 or CO5300 controller, FT3168 touch,
 *      ESP32-S3R8 (16 MB flash, 8 MB OPI PSRAM)
 * ============================================================================
 */
#pragma once

/* ==================================================================== */
/*  1. Panel geometry (leave as is for the 1.43" module)                 */
/* ==================================================================== */
#define AMOLED_WIDTH                466
#define AMOLED_HEIGHT               466

/* ==================================================================== */
/*  2. QSPI pins (ESP32-S3 GPIO numbers, from the Waveshare schematic)   */
/* ==================================================================== */
#define AMOLED_PIN_CS               9
#define AMOLED_PIN_PCLK             10
#define AMOLED_PIN_D0               11
#define AMOLED_PIN_D1               12
#define AMOLED_PIN_D2               13
#define AMOLED_PIN_D3               14
#define AMOLED_PIN_RST              21

/* ==================================================================== */
/*  3. QSPI clock                                                        */
/*                                                                       */
/*  40 MHz is what Waveshare's own examples use and what is guaranteed   */
/*  to work.  The ESP32-S3 SPI peripheral is happy with 80 MHz, so try   */
/*  80000000UL - if the picture stays clean you nearly double the frame  */
/*  rate (651 kB have to cross the bus for every RGB888 frame).          */
/* ==================================================================== */
#define AMOLED_QSPI_CLOCK_HZ        80000000UL

/* ==================================================================== */
/*  4. Colour depth                                                      */
/*                                                                       */
/*   24 = RGB888 : 16.7 million colours, 3 bytes/pixel  (TRUE 24-bit)    */
/*   16 = RGB565 : 65 thousand colours,  2 bytes/pixel  (fallback)       */
/*                                                                       */
/*  If your panel shows garbage in 24 bit mode (a few early batches only */
/*  accept 16 bpp over QSPI) switch to 16 and, if red and blue are then  */
/*  swapped, flip AMOLED_RGB565_BYTE_SWAP below.                         */
/* ==================================================================== */
#define AMOLED_COLOR_DEPTH          24

/* ==================================================================== */
/*  5. Panel detection                                                   */
/*                                                                       */
/*  The driver bit-bangs the controller's "Read ID1" (0xDA) command to   */
/*  find out whether an SH8601 (0x86) or a CO5300 (0xFF) is fitted - the  */
/*  CO5300 stores its 466 visible columns 6 pixels further into its GRAM. */
/*                                                                       */
/*     0x00 = automatic (recommended)                                    */
/*     0x86 = force SH8601                                               */
/*     0xFF = force CO5300                                               */
/* ==================================================================== */
#define AMOLED_FORCE_PANEL_ID       0x00

/* ==================================================================== */
/*  6. Default rotation: 0, 90, 180 or 270 degrees                       */
/*                                                                       */
/*  Can also be changed at run time with amoled.setRotation(deg).        */
/*  Software rotation: it costs one extra add per pixel, the controller  */
/*  itself cannot rotate.                                                */
/* ==================================================================== */
#define AMOLED_DEFAULT_ROTATION     0

/* ==================================================================== */
/*  7. Set to 1 if red and blue are swapped on your panel                */
/*     (flips the MADCTL BGR bit of the controller)                      */
/* ==================================================================== */
#define AMOLED_USE_BGR_ORDER        0

/* ==================================================================== */
/*  8. Only for AMOLED_COLOR_DEPTH 16: byte order of the RGB565 words    */
/*     1 = high byte first (works for the Waveshare AMOLED boards)       */
/* ==================================================================== */
#define AMOLED_RGB565_BYTE_SWAP     1

/* ==================================================================== */
/*  9. Tear free strip pipeline (streaming mode)                         */
/*                                                                       */
/*  The frame is rendered and DMA'ed in horizontal strips.  Two buffers  */
/*  let the CPU render strip N+1 while the QSPI DMA still pushes strip N. */
/*                                                                       */
/*  Keep AMOLED_STRIP_LINES even when AMOLED_COLOR_DEPTH is 24: a 466    */
/*  pixel RGB888 line is 1398 bytes, so an even line count keeps every   */
/*  transfer a whole number of 32 bit words.                             */
/*                                                                       */
/*  RAM: AMOLED_STRIP_BUFFERS * AMOLED_STRIP_LINES * 1398 bytes          */
/*       (2 * 32 = 89.5 kB with the defaults, from internal DMA RAM)     */
/* ==================================================================== */
#define AMOLED_STRIP_LINES          32
#define AMOLED_STRIP_BUFFERS        2

/* ==================================================================== */
/* 10. Touch panel (FT3168, I2C - shared with the IMU and the RTC)       */
/* ==================================================================== */
#define AMOLED_TOUCH_ENABLE         1
#define AMOLED_TOUCH_PIN_SDA        47
#define AMOLED_TOUCH_PIN_SCL        48
#define AMOLED_TOUCH_I2C_ADDR       0x38
#define AMOLED_TOUCH_I2C_HZ         400000
#define AMOLED_TOUCH_POLL_MS        8         /* gesture polling period        */

/* ==================================================================== */
/* 11. Full framebuffer mode (optional convenience mode)                 */
/*                                                                       */
/*  beginFramebuffer() allocates AMOLED_WIDTH * AMOLED_HEIGHT * bytes    */
/*  per pixel (651 kB at 24 bpp) in PSRAM and lets you draw like on a    */
/*  classic TFT (drawPixel/drawLine/fillRect/print...), followed by      */
/*  push() or pushRect().  Needs PSRAM, which this board has.            */
/* ==================================================================== */
#define AMOLED_FRAMEBUFFER_PSRAM    1

/* ==================================================================== */
/* 12. Serial logging during begin() (set to 0 to keep the port silent)  */
/*                                                                       */
/*  Note: the library only prints when a host has the USB port open -    */
/*  a USB CDC write with nobody listening blocks for ~2 s in the ESP32   */
/*  core, which would stall the frame loop.                              */
/* ==================================================================== */
#define AMOLED_SERIAL_LOG           1

/* ==================================================================== */
/* 13. HUD helpers used by the examples (round display safe radius)      */
/*                                                                       */
/*  The visible area of this panel is a circle.  The examples keep their  */
/*  overlays inside AMOLED_SAFE_RADIUS pixels of the centre so nothing   */
/*  lands in the cut off corners.  233 = the whole panel.                */
/* ==================================================================== */
#define AMOLED_SAFE_RADIUS          226

