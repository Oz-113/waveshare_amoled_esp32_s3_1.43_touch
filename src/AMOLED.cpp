/*
 * AMOLED.cpp
 * ============================================================================
 *  Panel layer of the WaveshareAMOLED library: QSPI bus, SH8601 / CO5300
 *  initialisation, brightness, the double buffered strip pipeline and the
 *  optional PSRAM framebuffer.
 *
 *  Written directly on the ESP-IDF esp_lcd layer that ships inside the
 *  Arduino-ESP32 core, so the library needs nothing else installed.
 *  esp_lcd_sh8601.c/.h is Espressif's SH8601 panel driver (taken from the
 *  Waveshare demo package), extended with the CO5300 init sequence.
 * ============================================================================
 */
#include "AMOLED.h"
#include "AMOLED_Probe.h"
#include "esp_lcd_sh8601.h"

#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* QSPI opcodes of the SH8601/CO5300 command protocol */
#define AMOLED_OPCODE_WRITE_CMD   0x02u
#define AMOLED_OPCODE_WRITE_COLOR 0x32u

#define AMOLED_SPI_HOST           SPI2_HOST

/* ------------------------------------------------------------------ */
/*  Controller init sequences (from the Waveshare demo package)        */
/* ------------------------------------------------------------------ */
static const sh8601_lcd_init_cmd_t amoledSh8601InitCmds[] =
{
    {0x11, (uint8_t []){0x00}, 0, 120},     /* sleep out                    */
    {0x44, (uint8_t []){0x01, 0xD1}, 2, 0}, /* tear scan line               */
    {0x35, (uint8_t []){0x00}, 1, 0},       /* tearing effect on            */
    {0x53, (uint8_t []){0x20}, 1, 10},      /* brightness ctrl + dimming    */
    {0x51, (uint8_t []){0x00}, 1, 10},      /* brightness = 0               */
    {0x29, (uint8_t []){0x00}, 0, 10},      /* display on                   */
    {0x51, (uint8_t []){0xFF}, 1, 0},       /* brightness = max             */
};

static const sh8601_lcd_init_cmd_t amoledCo5300InitCmds[] =
{
    {0x11, (uint8_t []){0x00}, 0, 80},      /* sleep out                    */
    {0xC4, (uint8_t []){0x80}, 1, 0},       /* SPI mode control             */
    {0x53, (uint8_t []){0x20}, 1, 1},       /* brightness ctrl + dimming    */
    {0x63, (uint8_t []){0xFF}, 1, 1},       /* high brightness mode         */
    {0x51, (uint8_t []){0x00}, 1, 1},       /* brightness = 0               */
    {0x29, (uint8_t []){0x00}, 0, 10},      /* display on                   */
    {0x51, (uint8_t []){0xFF}, 1, 0},       /* brightness = max             */
};

/* ------------------------------------------------------------------ */
/*  Low level state (one panel - like TFT_eSPI)                        */
/* ------------------------------------------------------------------ */
static bool                      s_hwReady    = false;
static esp_lcd_panel_handle_t    s_panel       = NULL;
static esp_lcd_panel_io_handle_t s_io          = NULL;
static uint8_t                  *s_strip[AMOLED_STRIP_BUFFERS] = {NULL};
static uint8_t                   s_stripIndex  = 0;
static size_t                    s_stripBytes  = 0;
static SemaphoreHandle_t         s_freeStrips  = NULL;
static uint32_t                  s_stripCount  = 0;

/* ------------------------------------------------------------------ */
/*  DMA completion callback (ISR): hand one buffer token back          */
/* ------------------------------------------------------------------ */
static bool amoledOnColorDone(esp_lcd_panel_io_handle_t io,
                              esp_lcd_panel_io_event_data_t *edata,
                              void *user_ctx)
{
    (void)io; (void)edata; (void)user_ctx;
    BaseType_t woken = pdFALSE;
    s_stripCount++;
    if (s_freeStrips) xSemaphoreGiveFromISR(s_freeStrips, &woken);
    return woken == pdTRUE;
}

/* ------------------------------------------------------------------ */
/*  Helpers                                                            */
/* ------------------------------------------------------------------ */
static int amoledQspiCmd(uint8_t cmd)
{
    return (int)(((uint32_t)AMOLED_OPCODE_WRITE_CMD << 24) | ((uint32_t)cmd << 8));
}

static inline int amoledClampI(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* take a strip buffer for the next strip - blocks until the DMA of the
 * buffer used two strips ago has finished */
static uint8_t *amoledTakeStripBuffer(void)
{
    if (!s_hwReady) return NULL;
    if (xSemaphoreTake(s_freeStrips, portMAX_DELAY) != pdTRUE) return NULL;
    s_stripIndex = (uint8_t)((s_stripIndex + 1) % AMOLED_STRIP_BUFFERS);
    return s_strip[s_stripIndex];
}

static void amoledQueueStrip(const uint8_t *buffer, int y, int lines)
{
    if (!s_hwReady || !buffer || lines <= 0) return;
    if (y + lines > AMOLED_HEIGHT) lines = AMOLED_HEIGHT - y;
    const esp_err_t err = esp_lcd_panel_draw_bitmap(s_panel, 0, y, AMOLED_WIDTH, y + lines, buffer);
    if (err != ESP_OK)
    {
        /* nothing was queued, so nobody will hand the buffer back */
        xSemaphoreGive(s_freeStrips);
    }
}

/* ==================================================================== */
/*  Life cycle                                                          */
/* ==================================================================== */
bool AMOLED::begin(void)
{
    if (_ready) return true;

    /* ---- 1. which controller is fitted? --------------------------- */
    const uint8_t forced = (uint8_t)AMOLED_FORCE_PANEL_ID;
    _panelId = forced ? forced : amoledProbePanelId();
    if (_panelId != AMOLED_PANEL_ID_SH8601) _panelId = AMOLED_PANEL_ID_CO5300;

    /* ---- 2. strip buffers in internal DMA RAM --------------------- */
    s_stripBytes = (size_t)AMOLED_WIDTH * AMOLED_STRIP_LINES * AMOLED_BPP;
    for (int i = 0; i < AMOLED_STRIP_BUFFERS; i++)
    {
        s_strip[i] = (uint8_t *)heap_caps_malloc(s_stripBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!s_strip[i])     /* fall back to PSRAM, slower but it works */
        {
            s_strip[i] = (uint8_t *)heap_caps_malloc(s_stripBytes, MALLOC_CAP_SPIRAM);
        }
        if (!s_strip[i]) return false;
        memset(s_strip[i], 0, s_stripBytes);
    }

    /* ---- 3. QSPI bus (4 data lines + DMA) ------------------------- */
    spi_bus_config_t buscfg = {};
    buscfg.data0_io_num    = AMOLED_PIN_D0;
    buscfg.data1_io_num    = AMOLED_PIN_D1;
    buscfg.data2_io_num    = AMOLED_PIN_D2;
    buscfg.data3_io_num    = AMOLED_PIN_D3;
    buscfg.sclk_io_num     = AMOLED_PIN_PCLK;
    buscfg.max_transfer_sz = (int)s_stripBytes + 16;
    if (spi_bus_initialize(AMOLED_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO) != ESP_OK) return false;

    /* ---- 4. panel IO: 32 bit command, 8 bit parameters, quad mode - */
    esp_lcd_panel_io_spi_config_t ioCfg = {};
    ioCfg.cs_gpio_num         = AMOLED_PIN_CS;
    ioCfg.dc_gpio_num         = -1;
    ioCfg.spi_mode            = 0;
    ioCfg.pclk_hz             = AMOLED_QSPI_CLOCK_HZ;
    ioCfg.trans_queue_depth   = 10;
    ioCfg.on_color_trans_done = amoledOnColorDone;
    ioCfg.user_ctx            = NULL;
    ioCfg.lcd_cmd_bits        = 32;
    ioCfg.lcd_param_bits      = 8;
    ioCfg.flags.quad_mode     = 1;
    if (esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)AMOLED_SPI_HOST, &ioCfg, &s_io) != ESP_OK)
    {
        return false;
    }

    /* ---- 5. the panel itself -------------------------------------- */
    sh8601_vendor_config_t vendor = {};
    vendor.flags.use_qspi_interface = 1;
    if (_panelId == AMOLED_PANEL_ID_SH8601)
    {
        vendor.init_cmds      = amoledSh8601InitCmds;
        vendor.init_cmds_size = sizeof(amoledSh8601InitCmds) / sizeof(amoledSh8601InitCmds[0]);
    }
    else
    {
        vendor.init_cmds      = amoledCo5300InitCmds;
        vendor.init_cmds_size = sizeof(amoledCo5300InitCmds) / sizeof(amoledCo5300InitCmds[0]);
    }

    esp_lcd_panel_dev_config_t panelCfg = {};
    panelCfg.reset_gpio_num = AMOLED_PIN_RST;
#if AMOLED_USE_BGR_ORDER
    panelCfg.rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_BGR;
#else
    panelCfg.rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB;
#endif
    panelCfg.bits_per_pixel = AMOLED_COLOR_DEPTH;
    panelCfg.vendor_config  = &vendor;

    if (esp_lcd_new_panel_sh8601(s_io, &panelCfg, &s_panel) != ESP_OK) return false;
    if (esp_lcd_panel_reset(s_panel) != ESP_OK) return false;
    if (esp_lcd_panel_init(s_panel) != ESP_OK) return false;
    /* the CO5300 keeps its 466 visible columns 6 pixels inside its GRAM */
    if (_panelId == AMOLED_PANEL_ID_CO5300) esp_lcd_panel_set_gap(s_panel, 6, 0);
    esp_lcd_panel_disp_on_off(s_panel, true);

    /* ---- 6. hand shake, tables, touch ---------------------------- */
    s_freeStrips = xSemaphoreCreateCounting(AMOLED_STRIP_BUFFERS, AMOLED_STRIP_BUFFERS);
    if (!s_freeStrips) return false;
    s_hwReady = true;

    AMOLED_Canvas::initTables();
    AMOLED_Canvas::setRotation(AMOLED_DEFAULT_ROTATION);
    _canvas.beginStrip(s_strip[0], 0, AMOLED_STRIP_LINES);

    _touch.begin();
    _ready = true;
    setBrightness(0xFF);

#if AMOLED_SERIAL_LOG
    /* only print while a host has the USB port open - a blocked CDC write
     * would otherwise stall the frame loop for seconds */
    if (Serial)
    {
        Serial.printf("[amoled] %s detected, %dx%d, %d bpp, QSPI %d MHz\n",
                      controllerName(), (int)AMOLED_WIDTH, (int)AMOLED_HEIGHT,
                      (int)AMOLED_COLOR_DEPTH, (int)(AMOLED_QSPI_CLOCK_HZ / 1000000UL));
        Serial.printf("[amoled] %d strip buffers x %u bytes, touch %s\n",
                      (int)AMOLED_STRIP_BUFFERS, (unsigned)s_stripBytes,
                      _touch.available() ? "ready" : "not answering");
    }
#endif
    return true;
}

const char *AMOLED::controllerName(void) const
{
    if (_panelId == AMOLED_PANEL_ID_SH8601) return "SH8601";
    if (_panelId == AMOLED_PANEL_ID_CO5300) return "CO5300";
    return "unknown";
}

/* ==================================================================== */
/*  Brightness / power                                                  */
/* ==================================================================== */
void AMOLED::setBrightness(uint8_t level)
{
    if (!s_hwReady) return;
    _brightness = level;
    /* 0x51 = "Write Display Brightness Value in Normal Mode"; the command has
     * to be wrapped into the QSPI opcode byte exactly like the panel driver
     * does it internally */
    esp_lcd_panel_io_tx_param(s_io, amoledQspiCmd(0x51), &level, 1);
}

void AMOLED::setDisplayOn(bool on)
{
    if (!s_hwReady) return;
    esp_lcd_panel_io_tx_param(s_io, amoledQspiCmd(on ? 0x29 : 0x28), NULL, 0);
}

void AMOLED::fadeBrightness(uint8_t to, uint16_t ms)
{
    const int from  = (int)_brightness;
    int steps = (int)(ms / 10);                       /* one step every 10 ms */
    if (steps < 1) steps = 1;

    for (int i = 1; i <= steps; i++)
    {
        int v = from + (((int)to - from) * i) / steps;
        if (v < 0) v = 0;
        if (v > 255) v = 255;
        setBrightness((uint8_t)v);
        delay(10);
    }
    setBrightness(to);
}

/* ==================================================================== */
/*  Streaming mode                                                      */
/*                                                                      */
/*  The strip buffers are handed out through a counting semaphore whose  */
/*  tokens are returned by the QSPI DMA completion callback.  Because    */
/*  transfers complete in the order they were queued and the buffers are */
/*  taken in a strict rotation, a buffer can never be reused while the   */
/*  DMA is still reading it - that is what makes the overlap safe.       */
/* ==================================================================== */
void AMOLED::beginFrame(void)
{
    /* a strip that was handed out but never pushed would leak its buffer
     * token, so make sure it is queued */
    if (_pending) pushStrip();
    _stripY = 0;
}

bool AMOLED::nextStrip(AMOLED_Canvas &canvas)
{
    if (!_ready) return false;

    /* if the caller did not push the strip he drew, push it now */
    if (_pending) pushStrip();

    if (_stripY >= AMOLED_HEIGHT) return false;      /* frame complete */

    _stripBuf = amoledTakeStripBuffer();             /* waits for a free buffer */
    if (!_stripBuf) return false;

    /* remember where this strip belongs and advance to the next one - the
     * row counter must move here, not when the push happens, otherwise a
     * frame would render the same strip over and over */
    _pendingY = _stripY;
    const int lines = amoledClampI(AMOLED_HEIGHT - _stripY, 1, AMOLED_STRIP_LINES);
    canvas.beginStrip(_stripBuf, _pendingY, lines);
    _stripY += AMOLED_STRIP_LINES;
    _pending = true;
    return true;
}

void AMOLED::pushStrip(void)
{
    if (!_pending) return;
    amoledQueueStrip(_stripBuf, _pendingY, amoledClampI(AMOLED_HEIGHT - _pendingY, 1, AMOLED_STRIP_LINES));
    _pending = false;
}

void AMOLED::endFrame(void)
{
    if (_pending) pushStrip();                       /* safety net */
    _stripY = 0;
    if (_fb) _canvas.beginFull(_fb);                 /* canvas back on the framebuffer */
}

void AMOLED::waitIdle(void)
{
    if (!s_hwReady) return;
    for (int i = 0; i < AMOLED_STRIP_BUFFERS; i++) xSemaphoreTake(s_freeStrips, portMAX_DELAY);
    for (int i = 0; i < AMOLED_STRIP_BUFFERS; i++) xSemaphoreGive(s_freeStrips);
}

void AMOLED::drawFrame(FrameCallback callback, uint32_t tMs)
{
    if (!callback) return;
    beginFrame();
    while (nextStrip(_canvas))
    {
        callback(_canvas, tMs);
        pushStrip();
    }
    endFrame();
}

/* ==================================================================== */
/*  Framebuffer mode                                                    */
/* ==================================================================== */
bool AMOLED::beginFramebuffer(void)
{
    if (_fb) return true;
    if (!_ready) return false;

    const size_t bytes = (size_t)AMOLED_WIDTH * AMOLED_HEIGHT * AMOLED_BPP;

#if AMOLED_FRAMEBUFFER_PSRAM
    _fb = (uint8_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
#endif
    if (!_fb) _fb = (uint8_t *)malloc(bytes);        /* internal RAM, usually too small */
    if (!_fb) return false;

    memset(_fb, 0, bytes);
    _canvas.beginFull(_fb);
    return true;
}

void AMOLED::push(void)
{
    if (!_fb || !_ready) return;

    for (int y = 0; y < AMOLED_HEIGHT; y += AMOLED_STRIP_LINES)
    {
        const int lines = amoledClampI(AMOLED_HEIGHT - y, 1, AMOLED_STRIP_LINES);
        uint8_t *buf = amoledTakeStripBuffer();
        if (!buf) return;
        memcpy(buf, _fb + (size_t)y * AMOLED_WIDTH * AMOLED_BPP,
               (size_t)lines * AMOLED_WIDTH * AMOLED_BPP);
        amoledQueueStrip(buf, y, lines);
    }

    /* the strips overlap internally while they are queued, but once this
     * returns the framebuffer may be drawn into again safely */
    waitIdle();
}

void AMOLED::pushRect(int x0, int y0, int x1, int y1)
{
    if (!_fb || !_ready) return;

    int t;
    if (x0 > x1) { t = x0; x0 = x1; x1 = t; }
    if (y0 > y1) { t = y0; y0 = y1; y1 = t; }
    x0 = amoledClampI(x0, 0, AMOLED_WIDTH - 1);
    x1 = amoledClampI(x1, 0, AMOLED_WIDTH - 1);
    y0 = amoledClampI(y0, 0, AMOLED_HEIGHT - 1);
    y1 = amoledClampI(y1, 0, AMOLED_HEIGHT - 1);
    if (x0 > x1 || y0 > y1) return;

    const size_t rowBytes = (size_t)(x1 - x0 + 1) * AMOLED_BPP;

    for (int y = y0; y <= y1; y++)
    {
        /* a strip buffer doubles as the packed row buffer: taking the next one
         * blocks until the DMA of the previous row has finished */
        uint8_t *buf = amoledTakeStripBuffer();
        if (!buf) return;
        memcpy(buf, _fb + ((size_t)y * AMOLED_WIDTH + x0) * AMOLED_BPP, rowBytes);
        if (esp_lcd_panel_draw_bitmap(s_panel, x0, y, x1 + 1, y + 1, buf) != ESP_OK)
        {
            xSemaphoreGive(s_freeStrips);
        }
    }

    /* as soon as this returns the framebuffer may be modified again */
    waitIdle();
}

