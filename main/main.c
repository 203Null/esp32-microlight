#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "esp_rom_gpio.h"
#include "hal/gpio_ll.h"
#include "hal/usb_hal.h"
#include "soc/usb_periph.h"

#include "driver/periph_ctrl.h"
#include "driver/rmt.h"

#include "led_strip.h"
#include "tusb.h"

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

void setup_led(void) //配置WS2812驱动硬件（RMT)
{
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(48, RMT_TX_CHANNEL);
    // set counter clock to 40MHz
        // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(1, (led_strip_dev_t)config.channel);
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
#define USBD_STACK_SIZE     (3*configMINIMAL_STACK_SIZE/2)
StackType_t  usb_device_stack[USBD_STACK_SIZE];
StaticTask_t usb_device_taskdef;

// Create a task for midi
#define MIDI_STACK_SIZE     configMINIMAL_STACK_SIZE
StackType_t  midi_stack[MIDI_STACK_SIZE];
StaticTask_t midi_taskdef;


// USB Device Driver task
// This top level thread process all usb events and invoke callbacks
void usb_device_task(void* param)
{
  (void) param;

  // This should be called after scheduler/kernel is started.
  // Otherwise it could cause kernel issue since USB IRQ handler does use RTOS queue API.
  tusb_init();

  // RTOS forever loop
  while (1)
  {
    // tinyusb device task
    tud_task();
  }
}

uint32_t colors[7] = {0xFFFFFF, 0xFF0000, 0xFFFF00, 0x00FF00, 0x00FFFF, 0x0000FF, 0xFF00FF};
int color_index = 0;
void midi_task(void* param)
{
  (void) param;
  // static uint32_t start_ms = 0;

  // uint8_t const cable_num = 0; // MIDI jack associated with USB endpoint
  // uint8_t const channel   = 0; // 0 for channel 1

  // The MIDI interface always creates input and output port/jack descriptors
  // regardless of these being used or not. Therefore incoming traffic should be read
  // (possibly just discarded) to avoid the sender blocking in IO
  uint8_t packet[4];
  while ( tud_midi_available() ) 
  {
    tud_midi_packet_read(packet); 
    set_color(0, colors[color_index % 7], 32);
    update_strip();
    color_index++;
    tud_midi_packet_write(packet); 
  }
}

static inline uint32_t board_millis(void)
{
  return ( ( ((uint64_t) xTaskGetTickCount()) * 1000) / configTICK_RATE_HZ );
}


// // Variable that holds the current position in the sequence.
// uint32_t note_pos = 0;

// // Store example melody as an array of note values
// uint8_t note_sequence[] =
// {
//   74,78,81,86,90,93,98,102,57,61,66,69,73,78,81,85,88,92,97,100,97,92,88,85,81,78,
//   74,69,66,62,57,62,66,69,74,78,81,86,90,93,97,102,97,93,90,85,81,78,73,68,64,61,
//   56,61,64,68,74,78,81,86,90,93,98,102
// };

// void midi_task(void* param)
// {
//   (void) param;

//   static uint32_t start_ms = 0;

//   uint8_t const cable_num = 0; // MIDI jack associated with USB endpoint
//   uint8_t const channel   = 0; // 0 for channel 1

//   // The MIDI interface always creates input and output port/jack descriptors
//   // regardless of these being used or not. Therefore incoming traffic should be read
//   // (possibly just discarded) to avoid the sender blocking in IO
//   uint8_t packet[4];
//   while ( tud_midi_available() ) tud_midi_packet_read(packet);

//   // send note periodically
//   if (board_millis() - start_ms < 286) return; // not enough time
//   start_ms += 286;

//   // Previous positions in the note sequence.
//   int previous = note_pos - 1;

//   // If we currently are at position 0, set the
//   // previous position to the last note in the sequence.
//   if (previous < 0) previous = sizeof(note_sequence) - 1;

//   // Send Note On for current position at full velocity (127) on channel 1.
//   uint8_t note_on[3] = { 0x90 | channel, note_sequence[note_pos], 127 };
//   tud_midi_stream_write(cable_num, note_on, 3);

//   // Send Note Off for previous note.
//   uint8_t note_off[3] = { 0x80 | channel, note_sequence[previous], 0};
//   tud_midi_stream_write(cable_num, note_off, 3);

//   set_color(0, colors[note_pos % 7], 32);
//   update_strip();

//   // Increment position
//   note_pos++;

//   // If we are at the end of the sequence, start over.
//   if (note_pos >= sizeof(note_sequence)) note_pos = 0;
// }

void app_main(void)
{
    setup_usb();
    setup_led();
    tusb_init();

    set_color(0, 0x00FFFF, 128);
    update_strip();

    // while (1)
    // {
    //   // tinyusb device task
    //   // tud_task();
    //   // midi_task();
    // }

    // usb_device_task(0);
    // Create a task for tinyusb device stack
    (void) xTaskCreateStatic( usb_device_task, "usbd", USBD_STACK_SIZE, NULL, configMAX_PRIORITIES-1, usb_device_stack, &usb_device_taskdef);

    // Create LED task
    // led_task(0);
    // (void) xTaskCreate( led_task, "led", LED_STACK_SZIE, NULL, configMAX_PRIORITIES-2, &led_taskdef);

    // Create Midi task
    // (void) xTaskCreateStatic( midi_task, "midi", MIDI_STACK_SIZE, NULL, configMAX_PRIORITIES-2, midi_stack, &midi_taskdef);
    while (1)
    {
      // tinyusb device task
      // tud_task();
      midi_task(0);
    }

}
