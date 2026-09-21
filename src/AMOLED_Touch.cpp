/*
 * AMOLED_Touch.cpp
 * ============================================================================
 *  FT3168 touch controller.
 *
 *  Registers used:
 *    0x00  mode       (0x00 = normal, 0x01 = monitor, 0x02 = sleep)
 *    0x02  status     (bit7..4 = number of touch points, bit3..0 = 1)
 *    0x03  touch 1 X  (XH, XL) 12 bit
 *    0x05  touch 1 Y  (YH, YL) 12 bit
 * ============================================================================
 */
#include "AMOLED_Touch.h"

#include <Arduino.h>
#include <Wire.h>

/* short tap threshold used by tapped() */
#define AMOLED_TOUCH_TAP_MS  500

static bool touchReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission((uint8_t)AMOLED_TOUCH_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)AMOLED_TOUCH_I2C_ADDR, (int)len) != (int)len) return false;
    for (uint8_t i = 0; i < len; i++)
    {
        if (!Wire.available()) return false;
        buf[i] = (uint8_t)Wire.read();
    }
    return true;
}

bool AMOLED_Touch::begin(void)
{
#if AMOLED_TOUCH_ENABLE
    Wire.begin(AMOLED_TOUCH_PIN_SDA, AMOLED_TOUCH_PIN_SCL, AMOLED_TOUCH_I2C_HZ);
    Wire.setTimeOut(50);

    /* put the controller into normal mode */
    Wire.beginTransmission((uint8_t)AMOLED_TOUCH_I2C_ADDR);
    Wire.write((uint8_t)0x00);
    Wire.write((uint8_t)0x00);
    _ok = (Wire.endTransmission() == 0);

    uint8_t id = 0;
    if (_ok) touchReadRegs(0xA8, &id, 1);    /* chip ID, informational only */
    (void)id;
    _down = _rawDown = _tapFlag = _onceFired = false;
    _lastPoll = 0;
    _downAt = 0;
#else
    _ok = false;
#endif
    return _ok;
}

bool AMOLED_Touch::read(uint16_t &x, uint16_t &y)
{
    uint8_t status = 0;
    uint8_t buf[4];

    if (!_ok) return false;
    if (!touchReadRegs(0x02, &status, 1)) return false;
    if ((status & 0x0F) == 0) return false;
    if (!touchReadRegs(0x03, buf, 4)) return false;

    uint16_t tx = (uint16_t)((((uint16_t)buf[0] & 0x0F) << 8) | buf[1]);
    uint16_t ty = (uint16_t)((((uint16_t)buf[2] & 0x0F) << 8) | buf[3]);

    /* the panel is square, so rotation does not change the touch mapping */
    if (tx >= AMOLED_WIDTH)  tx = AMOLED_WIDTH - 1;
    if (ty >= AMOLED_HEIGHT) ty = AMOLED_HEIGHT - 1;

    x = tx;
    y = ty;
    return true;
}

void AMOLED_Touch::update(void)
{
    const uint32_t now = millis();
    if ((now - _lastPoll) < AMOLED_TOUCH_POLL_MS) return;
    _lastPoll = now;

    uint16_t tx = 0, ty = 0;
    const bool down = read(tx, ty);

    if (down)
    {
        _down = true;
        _x = tx;
        _y = ty;
        if (!_rawDown)
        {
            _rawDown   = true;
            _downAt    = now;
            _onceFired = false;
        }
    }
    else
    {
        _down = false;
        if (_rawDown)
        {
            _rawDown = false;
            if ((now - _downAt) <= AMOLED_TOUCH_TAP_MS) _tapFlag = true;
        }
    }
}

bool AMOLED_Touch::tapped(void)
{
    const bool t = _tapFlag;
    _tapFlag = false;
    return t;
}

bool AMOLED_Touch::held(uint32_t ms)
{
    return _rawDown && ((millis() - _downAt) >= ms);
}

bool AMOLED_Touch::heldOnce(uint32_t ms)
{
    if (!_rawDown || _onceFired) return false;
    if ((millis() - _downAt) >= ms)
    {
        _onceFired = true;
        return true;
    }
    return false;
}

uint32_t AMOLED_Touch::pressTime(void) const
{
    return _rawDown ? (millis() - _downAt) : 0;
}
