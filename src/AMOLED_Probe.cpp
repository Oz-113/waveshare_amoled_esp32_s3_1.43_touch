/*
 * AMOLED_Probe.cpp
 * ============================================================================
 *  Bit-banged panel identification (port of Waveshare's read_lcd_id_bsp.c).
 * ============================================================================
 */
#include "AMOLED_Probe.h"
#include "AMOLED_Config.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define AMOLED_BIT_MASK ((uint64_t)0x01)

#define LCD_CS   ((gpio_num_t)AMOLED_PIN_CS)
#define LCD_CLK  ((gpio_num_t)AMOLED_PIN_PCLK)
#define LCD_D0   ((gpio_num_t)AMOLED_PIN_D0)
#define LCD_RST  ((gpio_num_t)AMOLED_PIN_RST)

#define cs_1   gpio_set_level(LCD_CS, 1)
#define cs_0   gpio_set_level(LCD_CS, 0)
#define clk_1  gpio_set_level(LCD_CLK, 1)
#define clk_0  gpio_set_level(LCD_CLK, 0)
#define d0_1   gpio_set_level(LCD_D0, 1)
#define d0_0   gpio_set_level(LCD_D0, 0)
#define rst_1  gpio_set_level(LCD_RST, 1)
#define rst_0  gpio_set_level(LCD_RST, 0)

#define read_d0 gpio_get_level(LCD_D0)

static void probe_gpio_init(void)
{
    gpio_config_t conf = {};
    conf.intr_type     = GPIO_INTR_DISABLE;
    conf.mode          = GPIO_MODE_OUTPUT;
    conf.pin_bit_mask  = (AMOLED_BIT_MASK << AMOLED_PIN_CS)   |
                         (AMOLED_BIT_MASK << AMOLED_PIN_PCLK) |
                         (AMOLED_BIT_MASK << AMOLED_PIN_D0)   |
                         (AMOLED_BIT_MASK << AMOLED_PIN_D1)   |
                         (AMOLED_BIT_MASK << AMOLED_PIN_D2)   |
                         (AMOLED_BIT_MASK << AMOLED_PIN_D3)   |
                         (AMOLED_BIT_MASK << AMOLED_PIN_RST);
    conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    conf.pull_up_en    = GPIO_PULLUP_ENABLE;
    gpio_config(&conf);
}

static void d0_as_input(void)
{
    gpio_config_t conf = {};
    conf.intr_type    = GPIO_INTR_DISABLE;
    conf.mode         = GPIO_MODE_INPUT;
    conf.pin_bit_mask = (AMOLED_BIT_MASK << AMOLED_PIN_D0);
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.pull_up_en   = GPIO_PULLUP_ENABLE;
    gpio_config(&conf);
}

static void d0_as_output(void)
{
    gpio_config_t conf = {};
    conf.intr_type    = GPIO_INTR_DISABLE;
    conf.mode         = GPIO_MODE_OUTPUT;
    conf.pin_bit_mask = (AMOLED_BIT_MASK << AMOLED_PIN_D0);
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.pull_up_en   = GPIO_PULLUP_ENABLE;
    gpio_config(&conf);
}

static void spi_1lane_send(uint8_t dat)
{
    for (int i = 0; i < 8; i++)
    {
        if (dat & 0x80) d0_1; else d0_0;
        dat <<= 1;
        clk_0;
        clk_1;
    }
}

static void spi_read_cmd(uint8_t reg)
{
    spi_1lane_send(0x03);        /* single lane read command */
    spi_1lane_send(0x00);
    spi_1lane_send(reg);
    spi_1lane_send(0x00);
}

static uint8_t spi_read_data(void)
{
    uint8_t dat = 0;
    for (int i = 0; i < 8; i++)
    {
        clk_0;
        d0_as_input();
        esp_rom_delay_us(1);
        dat = (uint8_t)((dat << 1) | read_d0);
        d0_as_output();
        clk_1;
        esp_rom_delay_us(1);
    }
    return dat;
}

uint8_t amoledProbePanelId(void)
{
    probe_gpio_init();

    /* hardware reset the controller, the same way the panel driver does */
    rst_1;
    vTaskDelay(pdMS_TO_TICKS(120));
    rst_0;
    vTaskDelay(pdMS_TO_TICKS(120));
    rst_1;
    vTaskDelay(pdMS_TO_TICKS(120));

    cs_0;
    spi_read_cmd(0xDA);          /* RDID1 */
    const uint8_t id = spi_read_data();
    cs_1;

    return id;
}
