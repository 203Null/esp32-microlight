#include <stdio.h>
#include "driver/rmt.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// #include "data.h"

static const char *TAG = "Microlight";

#define RMT_TX_CHANNEL RMT_CHANNEL_0

led_strip_t *strip;

void set_color(uint8_t index, uint32_t color, uint8_t brightness) //index是LED的位置， color是颜色
{
    //Color = 00000000RRRRRRRRGGGGGGGGBBBBBBB
    uint16_t R = (color & 0xFF0000) >> 16; //00000000 00000000 00000000 RRRRRRRR
    uint16_t G = (color & 0x00FF00) >> 8; //00000000 00000000 00000000 RRRRRRRR
    uint16_t B = color & 0x0000FF;
    if(brightness != 255)
    {
        R = (R * brightness) >> 8;
        G = (G * brightness) >> 8;
        B = (B * brightness) >> 8;
    }
    strip->set_pixel(strip, index, R, G, B);
}

void update_strip()
{
    strip->refresh(strip, 100);
}

void delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void setup_rmt(void) //配置WS2812驱动硬件（RMT)
{
     rmt_config_t config = RMT_DEFAULT_CONFIG_TX(14, RMT_TX_CHANNEL);
    // set counter clock to 40MHz
        // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(64, (led_strip_dev_t)config.channel);
    strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));
}

#define DELAY_MS 100
#define BRIGHTNESS 32
uint32_t colors[7] = {0xFFFFFF, 0xFF0000, 0xFFFF00, 0x00FF00, 0x00FFFF, 0x0000FF, 0xFF00FF};
void led_task(void)
{   
    uint32_t color_index = 0;
    while(true)
    {
        for(uint32_t i = 0; i < 64; i++)
        {
            set_color(i, 0x00FFFF, BRIGHTNESS);
            update_strip();
            delay(DELAY_MS);
        }
        color_index ++;
        for(uint32_t i = 0; i < 64; i++)
        {
            set_color(i, 0xFF00FF, BRIGHTNESS);
            update_strip();
            delay(DELAY_MS);
        }
        color_index ++;
    }
}

void app_main(void)
{
    setup_rmt(); //初始化LED
    led_task();
}

