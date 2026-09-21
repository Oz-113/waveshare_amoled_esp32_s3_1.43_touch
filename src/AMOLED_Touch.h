/*
 * AMOLED_Touch.h
 * ============================================================================
 *  The capacitive touch panel of the Waveshare ESP32-S3-Touch-AMOLED-1.43
 *  (FT3168, I2C address 0x38, same bus as the QMI8658 IMU and the PCF85063
 *  RTC: SDA = GPIO47, SCL = GPIO48).
 *
 *  Two levels of API:
 *
 *    raw      - read() gives you the current finger position
 *    gestures - update() + tapped() + held() implement the usual
 *               "tap to switch", "hold for something else" handling
 *
 *  Usage:
 *      amoled.touch().begin();               (already done by amoled.begin())
 *      amoled.touch().update();              call this often, e.g. every loop
 *      if (amoled.touch().tapped())  { ... }
 *      if (amoled.touch().held(700)) { ... }
 *      uint16_t x, y; amoled.touch().read(x, y);
 * ============================================================================
 */
#pragma once

#include <stdint.h>
#include "AMOLED_Config.h"

class AMOLED_Touch
{
public:
    /* start the I2C bus and wake the controller up; returns false if it
     * does not answer (the rest of the library keeps working) */
    bool begin(void);
    bool available(void) const { return _ok; }

    /* current finger position; returns true while the panel is touched */
    bool read(uint16_t &x, uint16_t &y);

    /* last state seen by update() */
    bool     isDown(void) const { return _down; }
    uint16_t x(void) const { return _x; }
    uint16_t y(void) const { return _y; }

    /* ---- gesture helper ---------------------------------------------
     * update() polls the controller (at most every AMOLED_TOUCH_POLL_MS)
     * and remembers what it saw. */
    void update(void);

    /* true once for a short tap (< 500 ms); the flag is cleared by reading */
    bool tapped(void);
    /* true while a finger is held for at least ms milliseconds */
    bool held(uint32_t ms);
    /* true exactly once per press, when the press reaches ms milliseconds -
     * this is what you want for "long press = do something" */
    bool heldOnce(uint32_t ms);
    /* how long the current press has lasted (0 if not pressed) */
    uint32_t pressTime(void) const;

    uint32_t pollIntervalMs(void) const { return AMOLED_TOUCH_POLL_MS; }

private:
    bool     _ok        = false;
    bool     _down      = false;
    uint16_t _x         = 0;
    uint16_t _y         = 0;
    bool     _rawDown   = false;
    bool     _tapFlag   = false;
    bool     _onceFired = false;
    uint32_t _downAt    = 0;
    uint32_t _lastPoll  = 0;
};
