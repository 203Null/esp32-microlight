idf_component_register(SRCS "main.c"  "usb_descriptors.c"
                    INCLUDE_DIRS ".")

target_include_directories(${COMPONENT_LIB} PRIVATE .)

target_compile_options(${COMPONENT_TARGET} PUBLIC
    "-DCFG_TUSB_MCU=OPT_MCU_ESP32S3"
    "-DCFG_TUSB_OS=OPT_OS_FREERTOS"
)

set(TINYUSB "${CMAKE_SOURCE_DIR}/lib/tinyusb")
message(TINYUSB: ${TINYUSB})

target_include_directories(${COMPONENT_TARGET} PUBLIC
  "$ENV{IDF_PATH}/common_components/led_strip/include"
  "$ENV{IDF_PATH}/components/freertos/include/freertos"
  "${TINYUSB}/hw"
  "${TINYUSB}/src"
)

target_sources(${COMPONENT_TARGET} PUBLIC
  "${TINYUSB}/src/tusb.c"
  "${TINYUSB}/src/common/tusb_fifo.c"
  "${TINYUSB}/src/device/usbd.c"
  "${TINYUSB}/src/device/usbd_control.c"
  "${TINYUSB}/src/class/cdc/cdc_device.c"
  "${TINYUSB}/src/class/dfu/dfu_rt_device.c"
  "${TINYUSB}/src/class/hid/hid_device.c"
  "${TINYUSB}/src/class/midi/midi_device.c"
  "${TINYUSB}/src/class/msc/msc_device.c"
  "${TINYUSB}/src/class/net/net_device.c"
  "${TINYUSB}/src/class/usbtmc/usbtmc_device.c"
  "${TINYUSB}/src/class/vendor/vendor_device.c"
  "${TINYUSB}/src/portable/espressif/esp32sx/dcd_esp32sx.c"
)