#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "sdkconfig.h"

#include "esp_rom_gpio.h"
#include "hal/gpio_ll.h"
#include "hal/usb_hal.h"
#include "soc/usb_periph.h"

#include "driver/periph_ctrl.h"
#include "driver/rmt.h"

#include "led_strip.h"
#include "tusb.h"

#include "data.h"

static const char *TAG = "Microlight";

#define BRIGHTNESS 32

#define RMT_TX_CHANNEL RMT_CHANNEL_0

#define LED_GPIO GPIO_NUM_15

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

void setup_led(void) //配置WS2812驱动硬件（RMT)
{
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(LED_GPIO, RMT_TX_CHANNEL); //Change GPIO Here
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


static void configure_pins(usb_hal_context_t *usb)
{
  /* usb_periph_iopins currently configures USB_OTG as USB Device.
   * Introduce additional parameters in usb_hal_context_t when adding support
   * for USB Host.
   */
    for (const usb_iopin_dsc_t *iopin = usb_periph_iopins; iopin->pin != -1; ++iopin) {
    if ((usb->use_external_phy) || (iopin->ext_phy_only == 0)) {
      esp_rom_gpio_pad_select_gpio(iopin->pin);
      if (iopin->is_output) {
        esp_rom_gpio_connect_out_signal(iopin->pin, iopin->func, false, false);
      } else {
        esp_rom_gpio_connect_in_signal(iopin->pin, iopin->func, false);
        if ((iopin->pin != GPIO_FUNC_IN_LOW) && (iopin->pin != GPIO_FUNC_IN_HIGH)) {
          gpio_ll_input_enable(&GPIO, iopin->pin);
        }
      }
      esp_rom_gpio_pad_unhold(iopin->pin);
    }
  }
  if (!usb->use_external_phy) {
    gpio_set_drive_capability(USBPHY_DM_NUM, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(USBPHY_DP_NUM, GPIO_DRIVE_CAP_3);
  }
}

void setup_usb(void)
{   
    // USB Controller Hal init
    periph_module_reset(PERIPH_USB_MODULE);
    periph_module_enable(PERIPH_USB_MODULE);

    usb_hal_context_t hal = {
        .use_external_phy = false // use built-in PHY
    };
    usb_hal_init(&hal);
    configure_pins(&hal);
}

// Create a task for tinyusb device stack
#define USBD_STACK_SIZE     (3*configMINIMAL_STACK_SIZE)
StackType_t  usb_device_stack[USBD_STACK_SIZE];
StaticTask_t usb_device_taskdef;

// Create a task for midi
#define MIDI_STACK_SIZE     3*configMINIMAL_STACK_SIZE
StackType_t  midi_stack[MIDI_STACK_SIZE];
StaticTask_t midi_taskdef;


// USB Device Driver task
// This top level thread process all usb events and invoke callbacks
void usb_device_task()
{
  // (void) param;

  // This should be called after scheduler/kernel is started.
  // Otherwise it could cause kernel issue since USB IRQ handler does use RTOS queue API.
  tusb_init();

  // RTOS forever loop
  while (1)
  {
    // tinyusb device task
    ESP_LOGD(TAG, "TUSB Task");
    tud_task();
  }
}

uint32_t colors[7] = {0xFFFFFF, 0xFF0000, 0xFFFF00, 0x00FF00, 0x00FFFF, 0x0000FF, 0xFF00FF};
int color_index = 0;
void midi_task(void* param)
{
  (void) param;
  uint8_t packet[4];
  while (1)
  {
    while ( tud_midi_available() ) 
    {
      ESP_LOGD(TAG, "Midi Recived");
      tud_midi_packet_read(packet); 
      switch (packet[0]) 
      {
        case 0x08: //CIN_NOTE_OFF
        case 0x09: //CIN_NOTE_ON
        {
          uint8_t channel = packet[1] & 0x0F;
          uint8_t note = packet[2];
          uint8_t velocity = packet[3];

          if (note > 35 && note < 100)
          {   
            uint8_t index = user1_keymap_optimized[note - 36];
            set_color(index, palette[channel % 2][velocity], BRIGHTNESS);
            // ESP_LOGI(TAG, "Set Color");
          }
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static inline uint32_t board_millis(void)
{
  return ( ( ((uint64_t) xTaskGetTickCount()) * 1000) / configTICK_RATE_HZ );
}

// static timer
StaticTimer_t led_tmdef;
TimerHandle_t led_tm;

void app_main(void)
{
    setup_usb();
    setup_led();

    // Create a task for tinyusb device stack
    (void) xTaskCreateStatic( usb_device_task, "usbd", USBD_STACK_SIZE, NULL, configMAX_PRIORITIES-1, usb_device_stack, &usb_device_taskdef);

    // Create Midi task
    (void) xTaskCreateStatic( midi_task, "midi", MIDI_STACK_SIZE, NULL, configMAX_PRIORITIES-2, midi_stack, &midi_taskdef);

    // soft timer for led update
    led_tm = xTimerCreateStatic(NULL, pdMS_TO_TICKS(10), true, NULL, update_strip, &led_tmdef);
    xTimerStart(led_tm, 0);

    // usb_device_task();
}
