/*
 * WaveshareAMOLED.h
 * ============================================================================
 *  Umbrella header of the WaveshareAMOLED library - include this one file and
 *  you are done:
 *
 *      #include <WaveshareAMOLED.h>
 *      AMOLED amoled;
 *
 *  It pulls in everything:
 *
 *      AMOLED_Config.h   the ONE file you edit (like TFT_eSPI's User_Setup.h)
 *      AMOLED_Colors.h   colours, amoledRGB(), amoledHSV(), named colours
 *      AMOLED_Canvas.h   the drawing engine (pixels, lines, rects, circles,
 *                        glow, arcs, text) and the low level pixel helpers
 *      AMOLED_Touch.h    FT3168 touch with tap / hold helpers
 *      AMOLED.h          the AMOLED object: begin(), streaming mode,
 *                        framebuffer mode, brightness, rotation
 *
 *  Minimum sketch:
 *
 *      #include <WaveshareAMOLED.h>
 *      AMOLED amoled;
 *
 *      void setup() {
 *          amoled.begin();
 *          amoled.setBrightness(255);
 *          amoled.canvas().fillScreen(AMOLED_BLACK);      // needs framebuffer
 *      }
 * ============================================================================
 */
#pragma once

#include "AMOLED_Config.h"
#include "AMOLED_Colors.h"
#include "AMOLED_Canvas.h"
#include "AMOLED_Touch.h"
#include "AMOLED.h"
