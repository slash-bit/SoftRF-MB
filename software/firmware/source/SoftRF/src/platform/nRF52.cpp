/*
/*
 * Platform_nRF52.cpp
 * Copyright (C) 2020-2022 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// This is nRF52.h from SoftRF mainline v1.2, with a few comments added, and with
// the changes for Arduino JSON library v6 (document rather than buffer, etc).

#if defined(ARDUINO_ARCH_NRF52)

#include <SPI.h>
#include <Wire.h>
#include <pcf8563.h>
#include <Adafruit_SPIFlash.h>
#include "Adafruit_TinyUSB.h"
#include <Adafruit_SleepyDog.h>
#if defined(USE_JSON_SETTINGS)
#include <ArduinoJson.h>
#endif
#include "nrf_wdt.h"

#include "../system/SoC.h"
#include "../driver/RF.h"
#include "../driver/Settings.h"
#include "../driver/Filesys.h"
#include "../driver/GNSS.h"
#include "../driver/Baro.h"
#include "../driver/LED.h"
#include "../driver/Bluetooth.h"
#include "../driver/EPD.h"
#include "../driver/Battery.h"
#include "../driver/Buzzer.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"
#include "../protocol/data/IGC.h"
#if defined(USE_JSON_SETTINGS)
#include "../protocol/data/JSON.h"
#endif
#include "../system/Time.h"

#include "uCDB.hpp"

#if defined(USE_BLE_MIDI)
#include <bluefruit.h>
#endif /* USE_BLE_MIDI */

#if defined(USE_BLE_MIDI) || defined(USE_USB_MIDI)
#include <MIDI.h>
#endif /* USE_BLE_MIDI || USE_USB_MIDI */

typedef volatile uint32_t REG32;
#define pREG32 (REG32 *)

#define DEVICE_ID_HIGH    (*(pREG32 (0x10000060)))
#define DEVICE_ID_LOW     (*(pREG32 (0x10000064)))

#define EPD_STACK_SZ      (256*3)

// RFM95W pin mapping
lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .txe = LMIC_UNUSED_PIN,
    .rxe = LMIC_UNUSED_PIN,
    .rst = SOC_GPIO_PIN_PCA10059_RST,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
    .busy = SOC_GPIO_PIN_BUSY,
    .tcxo = LMIC_UNUSED_PIN,
};

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIX_NUM, SOC_GPIO_PIN_LED,
                              NEO_GRB + NEO_KHZ800);

//char UDPpacketBuffer[4]; // Dummy definition to satisfy build sequence

static uint32_t prev_tx_packets_counter = 0;
static uint32_t prev_rx_packets_counter = 0;
extern uint32_t tx_packets_counter, rx_packets_counter;

struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static uint32_t bootCount __attribute__ ((section (".noinit")));

/* Default board - will be updated by nRF52_board_detect() in setup */
/* NOTE: For Seeed T1000E/CARD without proper bootloader marker, set to NRF52_SEEED_T1000E */
static nRF52_board_id nRF52_board = NRF52_SEEED_T1000E; /* Changed from NRF52_LILYGO_TECHO_REV_2 for CARD support */
static nRF52_display_id nRF52_display = EP_UNKNOWN;

const char *nRF52_Device_Manufacturer = SOFTRF_IDENT;
const char *nRF52_Device_Model = "Badge Edition";
const uint16_t nRF52_Device_Version = SOFTRF_USB_FW_VERSION;
static uint16_t nRF52_USB_VID = 0x239A; /* Adafruit Industries */
static uint16_t nRF52_USB_PID = 0x8029; /* Feather nRF52840 Express */

const char *Hardware_Rev[] = {
  [0] = "2020-8-6",
  [1] = "2020-12-12",
  [2] = "2021-3-26",
  [3] = "Unknown"
};

const prototype_entry_t techo_prototype_boards[] = {
  { 0x684f99bd2d5c7fae, NRF52_LILYGO_TECHO_REV_0, EP_GDEP015OC1,  0 }, /* orange */
  { 0xf353e11726ea8220, NRF52_LILYGO_TECHO_REV_0, EP_GDEH0154D67, 0 }, /* blue   */
  { 0xf4e0f04ded1892da, NRF52_LILYGO_TECHO_REV_1, EP_GDEH0154D67, 0 }, /* green  */
  { 0x65ab5994ea2c9094, NRF52_LILYGO_TECHO_REV_1, EP_GDEH0154D67, 0 }, /* blue   */
};

PCF8563_Class *rtc = nullptr;
I2CBus        *i2c = nullptr;

static bool nRF52_has_rtc      = false;
static bool nRF52_has_spiflash = false;
static bool RTC_sync           = false;
static bool ADB_is_open        = false;
static bool screen_saver       = false;

bool FATFS_is_mounted = false;

RTC_Date fw_build_date_time = RTC_Date(__DATE__, __TIME__);

static TaskHandle_t EPD_Task_Handle = NULL;

#if !defined(ARDUINO_NRF52840_PCA10056)
#error "This nRF52 build variant is not supported!"
#endif

#if SPI_32MHZ_INTERFACE == 0
#define _SPI_DEV    NRF_SPIM3 // 32 Mhz
#define _SPI1_DEV   NRF_SPIM2
#elif SPI_32MHZ_INTERFACE == 1
#define _SPI_DEV    NRF_SPIM2
#define _SPI1_DEV   NRF_SPIM3 // 32 Mhz
#else
  #error "not supported yet"
#endif

#if defined(USE_EPAPER)

#if SPI_INTERFACES_COUNT == 1
SPIClass SPI1(_SPI1_DEV,
              SOC_GPIO_PIN_EPD_MISO,
              SOC_GPIO_PIN_EPD_SCK,
              SOC_GPIO_PIN_EPD_MOSI);
#endif

GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> epd_d67(GxEPD2_154_D67(
                                                          SOC_GPIO_PIN_EPD_SS,
                                                          SOC_GPIO_PIN_EPD_DC,
                                                          SOC_GPIO_PIN_EPD_RST,
                                                          SOC_GPIO_PIN_EPD_BUSY));
GxEPD2_BW<GxEPD2_154, GxEPD2_154::HEIGHT>         epd_c1 (GxEPD2_154(
                                                          SOC_GPIO_PIN_EPD_SS,
                                                          SOC_GPIO_PIN_EPD_DC,
                                                          SOC_GPIO_PIN_EPD_RST,
                                                          SOC_GPIO_PIN_EPD_BUSY));
GxEPD2_BW<GxEPD2_150_BN, GxEPD2_150_BN::HEIGHT>   epd_bn (GxEPD2_150_BN(
                                                          SOC_GPIO_PIN_EPD_SS,
                                                          SOC_GPIO_PIN_EPD_DC,
                                                          SOC_GPIO_PIN_EPD_RST,
                                                          SOC_GPIO_PIN_EPD_BUSY));

GxEPD2_GFX *display;
#endif /* USE_EPAPER */

// Adafruit_FlashTransport_QSPI HWFlashTransport(SOC_GPIO_PIN_SFL_SCK,
//                                               SOC_GPIO_PIN_SFL_SS,
//                                               SOC_GPIO_PIN_SFL_MOSI,
//                                               SOC_GPIO_PIN_SFL_MISO,
//                                               SOC_GPIO_PIN_SFL_WP,
//                                               SOC_GPIO_PIN_SFL_HOLD);

// Adafruit_SPIFlash QSPIFlash (&HWFlashTransport);

static Adafruit_FlashTransport_QSPI *FlashTrans = NULL;
static Adafruit_SPIFlash *SPIFlash = NULL;

#define SFLASH_CMD_READ_CONFIG  0x15

static uint32_t spiflash_id = 0;
static uint8_t mx25_status_config[3] = {0x00, 0x00, 0x00};

/// Flash device list count
enum {
  MX25R1635F_INDEX,
  ZD25WQ16B_INDEX,
  GD25Q64C_INDEX,
  EXTERNAL_FLASH_DEVICE_COUNT
};

/// List of all possible flash devices used by nRF52840 boards
static SPIFlash_Device_t possible_devices[] = {
  // LilyGO T-Echo
  [MX25R1635F_INDEX] = MX25R1635F,
  [ZD25WQ16B_INDEX]  = ZD25WQ16B,
  // Seeed T1000-E
  [GD25Q64C_INDEX]   = GD25Q64C,
};

// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;

// file system object from SdFat
FatFileSystem fatfs;

#if defined(USE_JSON_SETTINGS)
#define NRF52_JSON_BUFFER_SIZE  1024
StaticJsonDocument<NRF52_JSON_BUFFER_SIZE> nRF52_jsonDoc;
#endif

#if defined(USE_WEBUSB_SERIAL) || defined(USE_WEBUSB_SETTINGS)
// USB WebUSB object
Adafruit_USBD_WebUSB usb_web;
#endif

#if defined(USE_WEBUSB_SERIAL)
// Landing Page: scheme (0: http, 1: https), url
WEBUSB_URL_DEF(landingPage, 1 /*https*/, "adafruit.github.io/Adafruit_TinyUSB_Arduino/examples/webusb-serial/index.html");
#endif /* USE_WEBUSB_SERIAL */

#if defined(USE_WEBUSB_SETTINGS)
// Landing Page: scheme (0: http, 1: https), url
WEBUSB_URL_DEF(landingPage, 1 /*https*/, "lyusupov.github.io/SoftRF/settings.html");
#endif /* USE_WEBUSB_SETTINGS */

#if defined(USE_USB_MIDI)
// USB MIDI object
Adafruit_USBD_MIDI usb_midi;

// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI_USB);
#endif /* USE_USB_MIDI */

ui_settings_t ui_settings;
/* = {
#if defined(DEFAULT_REGION_US)
    .units        = UNITS_IMPERIAL,
#else
    .units        = UNITS_METRIC,
#endif
    .zoom         = ZOOM_MEDIUM,
    .protocol     = PROTOCOL_NMEA,
    .rotate       = ROTATE_0,
    .orientation  = DIRECTION_TRACK_UP,
    .adb          = DB_NONE,
    .idpref       = ID_TYPE,
    .vmode        = VIEW_MODE_STATUS,
    .voice        = VOICE_OFF,
    .aghost       = ANTI_GHOSTING_OFF,
    .filter       = TRAFFIC_FILTER_OFF,
    .team         = 0
}; */

//ui_settings_t *ui;
uCDB<FatFileSystem, File> ucdb(fatfs);

#if !defined(EXCLUDE_IMU)
#define IMU_UPDATE_INTERVAL 500 /* ms */

#include <MPU9250.h>
MPU9250 imu;

static bool nRF52_has_imu = false;
static unsigned long IMU_Time_Marker = 0;

#if defined(USE_EPAPER)
extern float IMU_g;
#endif
#endif /* EXCLUDE_IMU */

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
static int32_t nRF52_msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return SPIFlash->readBlocks(lba, (uint8_t*) buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
static int32_t nRF52_msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize)
{
  ledOn(SOC_GPIO_LED_USBMSC);

  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return SPIFlash->writeBlocks(lba, buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
static void nRF52_msc_flush_cb (void)
{
  // sync with flash
  SPIFlash->syncBlocks();

  // clear file system's cache to force refresh
  fatfs.cacheClear();

  ledOff(SOC_GPIO_LED_USBMSC);
}

static bool nRF52_bl_check(const char* signature)
{
  int  i, j;
  bool match    = false;
  uint8_t* data = (uint8_t*) 0x000F4000UL;
  int length    = 0xFE000 - 0xF4000 - 2048 ; /* 38 KB */
  int str_len   = strlen(signature);

  for (i = 0; i < length - str_len; i++) {
    if (data[i] == signature[0]) {
      match = true;
      for (j = 1; j < str_len; j++) {
        if (data[i + j] != signature[j]) {
            match = false;
            break;
        }
      }
      if (match) {
        return true;
      }
    }
  }
  return false;
}

static void nRF52_system_off()
{

#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
  uint8_t sd_en;
  (void) sd_softdevice_is_enabled(&sd_en);

  // Enter System OFF state
  if ( sd_en ) {
    sd_power_system_off();
  } else {
    __disable_irq();
    NRF_POWER->SYSTEMOFF = 1;
  }

  // Should never reach here after SYSTEMOFF
  while(1);
#else
  NRF_POWER->SYSTEMOFF = 1;
  while(1);
#endif /* ARDUINO_ARCH_MBED */
}

  static nRF52_board_id nRF52_board_detect() {
  /* Detect T1000E by bootloader marker */
  if (nRF52_bl_check("T1000-E")) {
    Serial.println("[BOARD] Detected Seeed Studio T1000-E board - LR1110 enabled");
    return NRF52_SEEED_T1000E;
  }
  if (nRF52_bl_check("TECHOBOOT")) {
    Serial.println("[BOARD] Detected LILYGO T-Echo board");
    return NRF52_LILYGO_TECHO_REV_2;
  }
  /* Detect ELECROW boards by bootloader marker */
  if (nRF52_bl_check("ELECROWBOOT")) {
    Serial.println("[BOARD] Detected Elecrow ThinkNode M3 board - LR1110 enabled");
    /* Differentiate M3 from M1 by checking for AHT20 sensor */
    Wire.setPins(SOC_GPIO_PIN_M3_SDA, SOC_GPIO_PIN_M3_SCL);
    Wire.begin();
    Wire.beginTransmission(0x38);  /* AHT20 I2C address */
    if (Wire.endTransmission() == 0) {
      return NRF52_ELECROW_TN_M3;  /* M3 has AHT20 */
    }
    /* M1 would be here, but we only target M3 */
  }
  Serial.println("[BOARD] No bootloader marker found - defaulting to LILYGO T-Echo (WRONG for T1000E!)");
  Serial.println("[BOARD] WARNING: If this is a T1000E/CARD, battery will read wrong pin!");
  return NRF52_LILYGO_TECHO_REV_2;
  }

/* Forward declaration for SPI initialization */
static void nRF52_SPI_begin();

static void nRF52_setup()
{
  // ui = &ui_settings;
  uint32_t reset_reason = readResetReason();

  if      (reset_reason & POWER_RESETREAS_RESETPIN_Msk)
  {
      reset_info.reason = REASON_EXT_SYS_RST;
  }
  else if (reset_reason & POWER_RESETREAS_DOG_Msk)
  {
      reset_info.reason = REASON_WDT_RST;
  }
  else if (reset_reason & POWER_RESETREAS_SREQ_Msk)
  {
      reset_info.reason = REASON_SOFT_RESTART;
  }
  else if (reset_reason & POWER_RESETREAS_LOCKUP_Msk)
  {
      reset_info.reason = REASON_SOFT_WDT_RST;
  }
  else if (reset_reason & POWER_RESETREAS_OFF_Msk)
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
  }
  else if (reset_reason & POWER_RESETREAS_LPCOMP_Msk)
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
  }
  else if (reset_reason & POWER_RESETREAS_DIF_Msk)
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
  }
  else if (reset_reason & POWER_RESETREAS_NFC_Msk)
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
  }
  else if (reset_reason & POWER_RESETREAS_VBUS_Msk)
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
  }

  /* inactivate initVariant() of PCA10056 */
  pinMode(PIN_LED1, INPUT);
  pinMode(PIN_LED2, INPUT);
  pinMode(PIN_LED3, INPUT);
  pinMode(PIN_LED4, INPUT);

  static nRF52_board_id nRF52_board = nRF52_board_detect();

  /* Set hardware model based on detected board */
  switch (nRF52_board) {
    case NRF52_SEEED_T1000E:
      hw_info.model = SOFTRF_MODEL_CARD;
      Serial.println("[SETUP] T1000-E board detected and configured");
      break;
    case NRF52_ELECROW_TN_M3:
      hw_info.model = SOFTRF_MODEL_POCKET;
      Serial.println("[SETUP] Elecrow M3 board detected and configured");
      break;
    case NRF52_LILYGO_TECHO_REV_2:
      hw_info.model = SOFTRF_MODEL_BADGE;
      Serial.println("[SETUP] LilyGO T-Echo board detected and configured");
      break;
    default:
      hw_info.model = SOFTRF_MODEL_BADGE;  /* fallback */
      Serial.println("[SETUP] Unknown board - defaulting to BADGE");
      break;
  }
  USBDevice.setManufacturerDescriptor(nRF52_Device_Manufacturer);
  USBDevice.setProductDescriptor(nRF52_Device_Model);
  USBDevice.setDeviceVersion(nRF52_Device_Version);

  switch (hw_info.model) {
    case SOFTRF_MODEL_BADGE:
#if defined(USE_TINYUSB)
  Serial1.setPins(SOC_GPIO_PIN_CONS_RX, SOC_GPIO_PIN_CONS_TX);
  Serial1.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
#endif

  digitalWrite(SOC_GPIO_PIN_IO_PWR,  HIGH);
  pinMode(SOC_GPIO_PIN_IO_PWR,  OUTPUT);  /* VDD_POWR is ON */
  digitalWrite(SOC_GPIO_PIN_3V3_PWR, INPUT);

  delay(200);

  Wire.setPins(SOC_GPIO_PIN_SDA, SOC_GPIO_PIN_SCL);
  Wire.begin();

  Wire.beginTransmission(PCF8563_SLAVE_ADDRESS);
  nRF52_has_rtc = (Wire.endTransmission() == 0);
  if (!nRF52_has_rtc) {
    delay(200);
    Wire.beginTransmission(PCF8563_SLAVE_ADDRESS);
    nRF52_has_rtc = (Wire.endTransmission() == 0);
    if (!nRF52_has_rtc) {
      delay(200);
      Wire.beginTransmission(PCF8563_SLAVE_ADDRESS);
      nRF52_has_rtc = (Wire.endTransmission() == 0);
    }
  }

#if !defined(EXCLUDE_IMU)
  Wire.beginTransmission(MPU9250_ADDRESS);
  nRF52_has_imu = (Wire.endTransmission() == 0);
#endif /* EXCLUDE_IMU */

  //Wire.end();   // <<< leave active, for baro chip
  break;
  default:
    break;
  }
  for (int i=0; i < sizeof(techo_prototype_boards) / sizeof(prototype_entry_t); i++) {
    if (techo_prototype_boards[i].id == ((uint64_t) DEVICE_ID_HIGH << 32 | (uint64_t) DEVICE_ID_LOW)) {
      nRF52_board   = techo_prototype_boards[i].rev;
      nRF52_display = techo_prototype_boards[i].panel;
      break;
    }
  }

  /* GPIO pins init */
  switch (nRF52_board)
  {
    case NRF52_LILYGO_TECHO_REV_0:
      /* Wake up Air530 GNSS */
      digitalWrite(SOC_GPIO_PIN_GNSS_WKE, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_WKE, OUTPUT);

      pinMode(SOC_GPIO_LED_TECHO_REV_0_GREEN, OUTPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_0_RED,   OUTPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_0_BLUE,  OUTPUT);

      ledOn (SOC_GPIO_LED_TECHO_REV_0_GREEN);
      ledOff(SOC_GPIO_LED_TECHO_REV_0_RED);
      ledOff(SOC_GPIO_LED_TECHO_REV_0_BLUE);

      lmic_pins.rst = SOC_GPIO_PIN_TECHO_REV_0_RST;
      hw_info.revision = 0;
      break;

    case NRF52_LILYGO_TECHO_REV_1:
      /* Wake up Air530 GNSS */
      digitalWrite(SOC_GPIO_PIN_GNSS_WKE, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_WKE, OUTPUT);

      pinMode(SOC_GPIO_LED_TECHO_REV_1_GREEN, OUTPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_1_RED,   OUTPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_1_BLUE,  OUTPUT);

      ledOn (SOC_GPIO_LED_TECHO_REV_1_GREEN);
      ledOff(SOC_GPIO_LED_TECHO_REV_1_RED);
      ledOff(SOC_GPIO_LED_TECHO_REV_1_BLUE);

      lmic_pins.rst = SOC_GPIO_PIN_TECHO_REV_1_RST;
      hw_info.revision = 1;
      break;

    case NRF52_LILYGO_TECHO_REV_2:
      /* Wake up Quectel L76K GNSS */
      digitalWrite(SOC_GPIO_PIN_GNSS_RST, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_RST, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_GNSS_WKE, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_WKE, OUTPUT);

      pinMode(SOC_GPIO_LED_TECHO_REV_2_GREEN, OUTPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_2_RED,   OUTPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_2_BLUE,  OUTPUT);

      ledOn (SOC_GPIO_LED_TECHO_REV_2_GREEN);
      ledOff(SOC_GPIO_LED_TECHO_REV_2_RED);
      ledOff(SOC_GPIO_LED_TECHO_REV_2_BLUE);

      lmic_pins.rst = SOC_GPIO_PIN_TECHO_REV_2_RST;
      hw_info.revision = 2;
      break;

    case NRF52_SEEED_T1000E:
      hw_info.model      = SOFTRF_MODEL_CARD;
      hw_info.display    = DISPLAY_NONE;
      nRF52_Device_Model = "Card Edition";
      nRF52_USB_VID      = 0x2886; /* Seeed Technology */
      nRF52_USB_PID      = 0x0057; /* SenseCAP T1000-E */
      Serial.println("[SETUP] Initializing T1000-E GPIO pins...");

      if (reset_reason & POWER_RESETREAS_VBUS_Msk ||
          reset_reason & POWER_RESETREAS_RESETPIN_Msk) {
        NRF_POWER->GPREGRET = DFU_MAGIC_SKIP;
        pinMode(SOC_GPIO_PIN_IO_PWR, INPUT);
        pinMode(SOC_GPIO_PIN_T1000_BUTTON, INPUT_PULLDOWN_SENSE /* INPUT_SENSE_HIGH */);

        nRF52_system_off();
      }

      pinMode(SOC_GPIO_PIN_SFL_T1000_EN, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_SFL_T1000_EN, HIGH);
      delay(100);  // Wait for flash chip to power up
      Serial.println("[SETUP] SPI Flash enabled (P1.13)");

  #if defined(USE_TINYUSB)
      Serial1.setPins(SOC_GPIO_PIN_CONS_T1000_RX, SOC_GPIO_PIN_CONS_T1000_TX);
  #endif /* USE_TINYUSB */
      pinMode(SOC_GPIO_PIN_T1000_3V3_EN, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_T1000_3V3_EN, HIGH);
      Serial.println("[SETUP] 3.3V sensor power enabled (P1.6)");

      pinMode(SOC_GPIO_PIN_T1000_BUZZER_EN, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_T1000_BUZZER_EN, HIGH);
      // tone(SOC_GPIO_PIN_BUZZER, 840, 200);
      // delay(100);
      // noTone(SOC_GPIO_PIN_BUZZER);
      Serial.println("[SETUP] Buzzer enabled (P1.5)");

      pinMode(SOC_GPIO_PIN_GNSS_T1000_EN, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_GNSS_T1000_EN, HIGH);
      Serial.println("[SETUP] GNSS module enabled (P1.11)");

      pinMode(SOC_GPIO_PIN_GNSS_T1000_VRTC, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_GNSS_T1000_VRTC, HIGH);
      Serial.println("[SETUP] GNSS VRTC enabled (P0.8)");

      pinMode(SOC_GPIO_PIN_GNSS_T1000_RST, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_GNSS_T1000_RST, LOW);
      Serial.println("[SETUP] GNSS reset initialized (P1.15)");

      pinMode(SOC_GPIO_PIN_GNSS_T1000_SINT, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_GNSS_T1000_SINT, HIGH);
      Serial.println("[SETUP] GNSS sleep interrupt initialized (P1.12)");

      pinMode(SOC_GPIO_PIN_GNSS_T1000_RINT, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_GNSS_T1000_RINT, LOW);
      Serial.println("[SETUP] GNSS RTC interrupt initialized (P0.15)");

      pinMode(SOC_GPIO_LED_T1000_GREEN, OUTPUT);
      digitalWrite(SOC_GPIO_LED_T1000_GREEN, LED_STATE_ON);
      Serial.println("[SETUP] Status LED ON (P0.24)");

      lmic_pins.nss  = SOC_GPIO_PIN_T1000_SS;
      lmic_pins.rst  = SOC_GPIO_PIN_T1000_RST;
      lmic_pins.busy = SOC_GPIO_PIN_T1000_BUSY;
      lmic_pins.dio[0] = SOC_GPIO_PIN_T1000_DIO9; /* LR1110 */


      hw_info.revision = 3; /* Unknown */

      break;

    case NRF52_ELECROW_TN_M3:
      hw_info.model      = SOFTRF_MODEL_POCKET;
      nRF52_Device_Model = "Pocket Edition";
      nRF52_USB_VID      = 0x2E8A; /* Elecrow Electronics */
      nRF52_USB_PID      = 0x0006; /* ThinkNode M3 */

      pinMode(SOC_GPIO_PIN_M3_TEMP_EN,    INPUT);
      pinMode(SOC_GPIO_PIN_M3_EEPROM_EN,  INPUT);

      if (reset_reason & POWER_RESETREAS_VBUS_Msk ||
          reset_reason & POWER_RESETREAS_RESETPIN_Msk) {
        NRF_POWER->GPREGRET = DFU_MAGIC_SKIP;
        pinMode(SOC_GPIO_PIN_IO_PWR, INPUT);
        pinMode(SOC_GPIO_PIN_M3_BUTTON, INPUT_PULLDOWN_SENSE /* INPUT_SENSE_HIGH */);

        nRF52_system_off();
      }

      pinMode(SOC_GPIO_PIN_M3_TEMP_EN,      INPUT_PULLUP);
      pinMode(SOC_GPIO_PIN_M3_EEPROM_EN,    INPUT_PULLUP);
  #if defined(USE_TINYUSB)
      Serial1.setPins(SOC_GPIO_PIN_CONS_M3_RX, SOC_GPIO_PIN_CONS_M3_TX);
  #endif /* USE_TINYUSB */
      digitalWrite(SOC_GPIO_PIN_M3_ADC_EN, HIGH);
      pinMode(SOC_GPIO_PIN_M3_ADC_EN, OUTPUT);

      digitalWrite(SOC_GPIO_PIN_M3_EN1, HIGH);
      pinMode(SOC_GPIO_PIN_M3_EN1, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_M3_EN2, HIGH);
      pinMode(SOC_GPIO_PIN_M3_EN2, OUTPUT);

      pinMode(SOC_GPIO_PIN_M3_BUT_EN, INPUT_PULLUP);

      digitalWrite(SOC_GPIO_PIN_GNSS_M3_EN, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_M3_EN, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_GNSS_M3_RST, LOW);
      pinMode(SOC_GPIO_PIN_GNSS_M3_RST, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_GNSS_M3_WKE, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_M3_WKE, OUTPUT);

      digitalWrite(SOC_GPIO_LED_M3_RGB_PWR, HIGH);
      pinMode(SOC_GPIO_LED_M3_RGB_PWR,      OUTPUT);

      pinMode(SOC_GPIO_LED_M3_RED,   OUTPUT);
      pinMode(SOC_GPIO_LED_M3_GREEN, OUTPUT);
      pinMode(SOC_GPIO_LED_M3_BLUE,  OUTPUT);

      digitalWrite(SOC_GPIO_LED_M3_RED,   1-LED_STATE_ON);
      digitalWrite(SOC_GPIO_LED_M3_GREEN, LED_STATE_ON);
      digitalWrite(SOC_GPIO_LED_M3_BLUE,  1-LED_STATE_ON);

      lmic_pins.nss  = SOC_GPIO_PIN_M3_SS;
      lmic_pins.rst  = SOC_GPIO_PIN_M3_RST;
      lmic_pins.busy = SOC_GPIO_PIN_M3_BUSY;
#if defined(USE_RADIOLIB)
      lmic_pins.dio[0] = SOC_GPIO_PIN_M3_DIO9; /* LR1110 */
#endif /* USE_RADIOLIB */

      hw_info.revision = 3; /* Unknown */

      break;

    case NRF52_NORDIC_PCA10059:
    default:
      pinMode(SOC_GPIO_LED_PCA10059_STATUS, OUTPUT);
      pinMode(SOC_GPIO_LED_PCA10059_GREEN,  OUTPUT);
      pinMode(SOC_GPIO_LED_PCA10059_RED,    OUTPUT);
      pinMode(SOC_GPIO_LED_PCA10059_BLUE,   OUTPUT);

      ledOn (SOC_GPIO_LED_PCA10059_GREEN);
      ledOff(SOC_GPIO_LED_PCA10059_RED);
      ledOff(SOC_GPIO_LED_PCA10059_BLUE);
      ledOff(SOC_GPIO_LED_PCA10059_STATUS);

      hw_info.revision = 3; /* Unknown */
      break;
  }

  i2c = new I2CBus(Wire);

  if (nRF52_has_rtc && (i2c != nullptr)) {
    rtc = new PCF8563_Class(*i2c);

    pinMode(SOC_GPIO_PIN_R_INT, INPUT);
    hw_info.rtc = RTC_PCF8563;
  }

#if !defined(EXCLUDE_IMU)
  if (nRF52_has_imu && imu.setup(MPU9250_ADDRESS)) {
    imu.verbose(false);
    if (imu.isSleeping()) {
      imu.sleep(false);
    }
    hw_info.imu = IMU_MPU9250;
    hw_info.mag = MAG_AK8963;
    IMU_Time_Marker = millis();
  }
#endif /* EXCLUDE_IMU */

  /* (Q)SPI flash init */
  switch (nRF52_board)
  {
    case NRF52_LILYGO_TECHO_REV_0:
      possible_devices[MX25R1635F_INDEX].max_clock_speed_mhz = 33;
      possible_devices[MX25R1635F_INDEX].supports_qspi = false;
      possible_devices[MX25R1635F_INDEX].supports_qspi_writes = false;
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
      FlashTrans = new Adafruit_FlashTransport_QSPI(SOC_GPIO_PIN_SFL_SCK,
                                                    SOC_GPIO_PIN_SFL_SS,
                                                    SOC_GPIO_PIN_SFL_MOSI,
                                                    SOC_GPIO_PIN_SFL_MISO,
                                                    SOC_GPIO_PIN_SFL_WP,
                                                    SOC_GPIO_PIN_SFL_HOLD);
      break;
    case NRF52_SEEED_T1000E:
      // Initialize QSPI control pins
      pinMode(SOC_GPIO_PIN_SFL_T1000_WP, OUTPUT);
      pinMode(SOC_GPIO_PIN_SFL_T1000_HOLD, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_SFL_T1000_WP, HIGH);    // WP inactive (HIGH)
      digitalWrite(SOC_GPIO_PIN_SFL_T1000_HOLD, HIGH);  // HOLD inactive (HIGH)

      FlashTrans = new Adafruit_FlashTransport_QSPI(SOC_GPIO_PIN_SFL_T1000_SCK,
                                              SOC_GPIO_PIN_SFL_T1000_SS,
                                              SOC_GPIO_PIN_SFL_T1000_MOSI,
                                              SOC_GPIO_PIN_SFL_T1000_MISO,
                                              SOC_GPIO_PIN_SFL_T1000_WP,
                                              SOC_GPIO_PIN_SFL_T1000_HOLD);
      break;
    case NRF52_NORDIC_PCA10059:
    case NRF52_ELECROW_TN_M3:
    default:
      break;
  }

  /* Configure SPI pins BEFORE accessing the flash */
  nRF52_SPI_begin();

  if (FlashTrans != NULL) {
    Serial.println("[SETUP] SPI Flash transport object created");
    FlashTrans->begin();
    Serial.println("[SETUP] SPI Flash transport started");
    FlashTrans->runCommand(0xAB); /* RDP/RES */
    FlashTrans->end();
    Serial.println("[SETUP] SPI Flash wake-up command sent");

    SPIFlash = new Adafruit_SPIFlash(FlashTrans);
    Serial.println("[SETUP] SPI Flash object created");
    nRF52_has_spiflash = SPIFlash->begin(possible_devices,
                                         EXTERNAL_FLASH_DEVICE_COUNT);
    if (nRF52_has_spiflash) {
      Serial.println("[SETUP] SPI Flash initialized successfully");
    } else {
      Serial.println("[SETUP] WARNING: SPI Flash initialization failed");
    }
  }

  hw_info.storage = nRF52_has_spiflash ? STORAGE_FLASH : STORAGE_NONE;

  if (nRF52_has_spiflash) {
    spiflash_id = SPIFlash->getJEDECID();
    Serial.print("[SETUP] SPI Flash JEDEC ID: 0x");
    Serial.println(spiflash_id, HEX);

    //mx25_status_config[0] = SPIFlash->readStatus();
    //HWFlashTransport.readCommand(SFLASH_CMD_READ_CONFIG,   mx25_status_config + 1, 2);
    //mx25_status_config[2] |= 0x2;       /* High performance mode */
    //SPIFlash->writeEnable();
    //HWFlashTransport.writeCommand(SFLASH_CMD_WRITE_STATUS, mx25_status_config,     3);
    //SPIFlash->writeDisable();
    //SPIFlash->waitUntilReady();

    //uint32_t const wr_speed = min(80 * 1000000U, (uint32_t)F_CPU);
    //uint32_t rd_speed = wr_speed;
    //HWFlashTransport.setClockSpeed(wr_speed, rd_speed);

    // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
    usb_msc.setID(nRF52_Device_Manufacturer, "External Flash", "1.0");

    // Set callback
    usb_msc.setReadWriteCallback(nRF52_msc_read_cb,
                                 nRF52_msc_write_cb,
                                 nRF52_msc_flush_cb);

    // Set disk size, block size should be 512 regardless of spi flash page size
    usb_msc.setCapacity(SPIFlash->size()/512, 512);

    // MSC is ready for read/write
    usb_msc.setUnitReady(true);

    usb_msc.begin();

    FATFS_is_mounted = fatfs.begin(SPIFlash);
  }

#if defined(USE_USB_MIDI)
  // Initialize MIDI with no any input channels
  // This will also call usb_midi's begin()
  MIDI_USB.begin(MIDI_CHANNEL_OFF);
#endif /* USE_USB_MIDI */

  Serial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);

#if defined(USE_TINYUSB) && defined(USBCON)
  for (int i=0; i < 20; i++) {if (Serial) break; else delay(100);}
#endif
}

static void print_dest(int dest)
{
  switch (dest)
  {
    case DEST_UART       :  Serial.println(F("UART"));          break;
    case DEST_USB        :  Serial.println(F("USB CDC"));       break;
    case DEST_BLUETOOTH  :  Serial.println(F("Bluetooth LE"));  break;
    case DEST_NONE       :
    default              :  Serial.println(F("NULL"));          break;
  }
}

static void nRF52_post_init()
{
  if (nRF52_board == NRF52_LILYGO_TECHO_REV_0 ||
      nRF52_board == NRF52_LILYGO_TECHO_REV_1 ||
      nRF52_board == NRF52_LILYGO_TECHO_REV_2) {

#if 0
    char strbuf[32];
    Serial.println();
    Serial.print  (F("64-bit Device ID: "));
    snprintf(strbuf, sizeof(strbuf),"0x%08x%08x", DEVICE_ID_HIGH, DEVICE_ID_LOW);
    Serial.println(strbuf);
#endif

#if 0
    Serial.println();
    Serial.print  (F("SPI FLASH JEDEC ID: "));
    Serial.print  (spiflash_id, HEX);           Serial.print(" ");
    Serial.print  (F("STATUS/CONFIG: "));
    Serial.print  (mx25_status_config[0], HEX); Serial.print(" ");
    Serial.print  (mx25_status_config[1], HEX); Serial.print(" ");
    Serial.print  (mx25_status_config[2], HEX); Serial.println();
#endif

    Serial.println();
    Serial.print  (hw_info.model == SOFTRF_MODEL_BADGE  ? F("Board: LILYGO T-Echo ") :
                   hw_info.model == SOFTRF_MODEL_CARD   ? F("Board: Seeed T1000-E ") :
                   hw_info.model == SOFTRF_MODEL_POCKET ? F("Board: Elecrow ThinkNode M3 ") :
                                                          F("Board: Unknown Model "));
    Serial.print  (hw_info.revision);
    Serial.print  (":");
    Serial.print  (hw_info.revision > 2 ?
                   Hardware_Rev[3] : Hardware_Rev[hw_info.revision]);
    Serial.println(F(") Power-on Self Test"));
    Serial.println();
    Serial.flush();

    Serial.println(F("Built-in components:"));

    Serial.print(F("RADIO   : "));
    Serial.println(hw_info.rf      == RF_IC_SX1262 ||
                   hw_info.rf      == RF_IC_SX1276     ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("GNSS    : "));
    Serial.println(hw_info.gnss == ((hw_info.revision == 0 || hw_info.revision == 1) ?
                                   GNSS_MODULE_GOKE :
                                   hw_info.gnss == GNSS_MODULE_AG33) ?
                                   F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("DISPLAY : "));
    Serial.println(hw_info.display == DISPLAY_EPD_1_54 ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("RTC     : "));
    Serial.println(hw_info.rtc     == RTC_PCF8563      ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("FLASH   : "));
    Serial.println(hw_info.storage == STORAGE_FLASH    ? F("PASS") : F("FAIL"));
    Serial.flush();

    if (nRF52_board == NRF52_LILYGO_TECHO_REV_1 ||
        nRF52_board == NRF52_LILYGO_TECHO_REV_2) {
      Serial.print(F("BMx280  : "));
      Serial.println(hw_info.baro == BARO_MODULE_BMP280 ? F("PASS") : F("N/A"));
      Serial.flush();
    }

#if !defined(EXCLUDE_IMU)
    Serial.println();
    Serial.println(F("External components:"));
    Serial.print(F("IMU     : "));
    Serial.println(hw_info.imu    == IMU_MPU9250       ? F("PASS") : F("N/A"));
    Serial.flush();
#endif /* EXCLUDE_IMU */

    Serial.println();
    Serial.println(F("Power-on Self Test is complete."));
    Serial.println();
    Serial.flush();
  } else if (nRF52_board == NRF52_NORDIC_PCA10059) {
    Serial.println();
    Serial.println(F("Board: Nordic PCA10059 USB Dongle"));
    Serial.println();
    Serial.flush();
  }

  if (FATFS_is_mounted)
      Serial.printf("FATFS mounted, free space: %d kB\n", FILESYS_free_kb());
  else
      Serial.println("Failed to mount FATFS");
  Serial.println();

  Serial.println(F("Data output device(s):"));

  Serial.print(F("NMEA   - "));
  print_dest(settings->nmea_out);

  Serial.print(F("NMEA2  - "));
  print_dest(settings->nmea_out2);

  Serial.print(F("GDL90  - "));
  print_dest(settings->gdl90);

  Serial.print(F("D1090  - "));
  print_dest(settings->d1090);

  Serial.println();

  Serial.flush();

#if defined(USE_EPAPER)
  if (nRF52_board == NRF52_LILYGO_TECHO_REV_0 ||
      nRF52_board == NRF52_LILYGO_TECHO_REV_1 ||
      nRF52_board == NRF52_LILYGO_TECHO_REV_2) {
    /* EPD back light on */
    digitalWrite(SOC_GPIO_PIN_EPD_BLGT, HIGH);

    EPD_info1();

    /* EPD back light off */
    digitalWrite(SOC_GPIO_PIN_EPD_BLGT, LOW);

    char key[8];
    char out[64];
    uint8_t tokens[3] = { 0 };
    cdbResult rt;
    int c, i = 0, token_cnt = 0;

    int acfts;
    char *reg, *mam, *cn;
    reg = mam = cn = NULL;

    if (ADB_is_open) {
      acfts = ucdb.recordsNumber();

      snprintf(key, sizeof(key),"%06X", ThisAircraft.addr);

      rt = ucdb.findKey(key, strlen(key));

      switch (rt) {
        case KEY_FOUND:
          while ((c = ucdb.readValue()) != -1 && i < (sizeof(out) - 1)) {
            if (c == '|') {
              if (token_cnt < (sizeof(tokens) - 1)) {
                token_cnt++;
                tokens[token_cnt] = i+1;
              }
              c = 0;
            }
            out[i++] = (char) c;
          }
          out[i] = 0;

          reg = out + tokens[1];
          mam = out + tokens[0];
          cn  = out + tokens[2];

          break;

        case KEY_NOT_FOUND:
        default:
          break;
      }

      reg = (reg != NULL) && strlen(reg) ? reg : (char *) "REG: N/A";
      mam = (mam != NULL) && strlen(mam) ? mam : (char *) "M&M: N/A";
      cn  = (cn  != NULL) && strlen(cn)  ? cn  : (char *) "N/A";

    } else {
      acfts = -1;
    }

    EPD_info2(acfts, reg, mam, cn);
  }
#endif /* USE_EPAPER */
}

static void nRF52_loop()
{
  // Reload the watchdog
  if (nrf_wdt_started(NRF_WDT)) {
    Watchdog.reset();
  }

  if (!RTC_sync) {
    if (rtc &&
        gnss.date.isValid()                         &&
        gnss.time.isValid()                         &&
        gnss.date.year() >= fw_build_date_time.year &&
        gnss.date.year() <  fw_build_date_time.year + 15 ) {
      rtc->setDateTime(gnss.date.year(),   gnss.date.month(),
                       gnss.date.day(),    gnss.time.hour(),
                       gnss.time.minute(), gnss.time.second());
      RTC_sync = true;
    }
  }

#if defined(USE_WEBUSB_SETTINGS) && !defined(USE_WEBUSB_SERIAL)

  if (USBDevice.mounted() && usb_web.connected() && usb_web.available()) {

#if defined(USE_JSON_SETTINGS)

    deserializeJson(nRF52_jsonDoc, usb_web);
    JsonObject root = nRF52_jsonDoc.as<JsonObject>();

    if (root.success()) {
      JsonVariant msg_class = root["class"];

      if (msg_class.success()) {
        const char *msg_class_s = msg_class.as<char*>();

        if (!strcmp(msg_class_s,"SOFTRF")) {
          parseSettings  (root);
          parseUISettings(root);

          SoC->WDT_fini();
          if (SoC->Bluetooth_ops) { SoC->Bluetooth_ops->fini(); }
          Serial.println();
          Serial.println(F("Restart is in progress. Please, wait..."));
          Serial.println();
          Serial.flush();
          EEPROM_store();
          RF_Shutdown();
          SoC->reset();
        }
      }
    }
  }

#endif /* USE_JSON_SETTINGS */

#endif /* USE_WEBUSB_SETTINGS */

#if !defined(EXCLUDE_IMU)
  if (hw_info.imu == IMU_MPU9250 &&
      (millis() - IMU_Time_Marker) > IMU_UPDATE_INTERVAL) {
    if (imu.update()) {
      float a_x = imu.getAccX();
      float a_y = imu.getAccY();
      float a_z = imu.getAccZ();

#if defined(USE_EPAPER)
      IMU_g = sqrtf(a_x*a_x + a_y*a_y + a_z*a_z);
#endif
    }
    IMU_Time_Marker = millis();
  }
#endif /* EXCLUDE_IMU */
}

static void nRF52_fini(int reason)
{
  uint8_t sd_en;

  if (nRF52_has_spiflash) {
    usb_msc.setUnitReady(false);
//  usb_msc.end(); /* N/A */
  }

  if (SPIFlash != NULL) {
    FlashTrans->runCommand(0xB9); /* DP */
    SPIFlash->end();
  }

#if !defined(EXCLUDE_IMU)
  if (hw_info.imu == IMU_MPU9250) {
    imu.sleep(true);
  }
#endif /* EXCLUDE_IMU */

  switch (nRF52_board)
  {
    case NRF52_LILYGO_TECHO_REV_0:
#if 0
      /* Air530 GNSS ultra-low power tracking mode */
      digitalWrite(SOC_GPIO_PIN_GNSS_WKE, LOW);
      pinMode(SOC_GPIO_PIN_GNSS_WKE, OUTPUT);

    // Serial_GNSS_Out.write("$PGKC105,4*33\r\n");
#else
      pinMode(SOC_GPIO_PIN_GNSS_WKE, INPUT);

      // Serial_GNSS_Out.write("$PGKC051,0*37\r\n");
      // Serial_GNSS_Out.write("$PGKC051,1*36\r\n");
#endif
      // Serial_GNSS_Out.flush(); delay(250);

      ledOff(SOC_GPIO_LED_TECHO_REV_0_GREEN);
      ledOff(SOC_GPIO_LED_TECHO_REV_0_RED);
      ledOff(SOC_GPIO_LED_TECHO_REV_0_BLUE);

      pinMode(SOC_GPIO_LED_TECHO_REV_0_GREEN, INPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_0_RED,   INPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_0_BLUE,  INPUT);

      pinMode(SOC_GPIO_PIN_IO_PWR, INPUT);
      pinMode(SOC_GPIO_PIN_SFL_SS, INPUT);
      break;

    case NRF52_LILYGO_TECHO_REV_1:
#if 0
      /* Air530 GNSS ultra-low power tracking mode */
      digitalWrite(SOC_GPIO_PIN_GNSS_WKE, LOW);
      pinMode(SOC_GPIO_PIN_GNSS_WKE, OUTPUT);

    // Serial_GNSS_Out.write("$PGKC105,4*33\r\n");
#else
      pinMode(SOC_GPIO_PIN_GNSS_WKE, INPUT);

      // Serial_GNSS_Out.write("$PGKC051,0*37\r\n");
      // Serial_GNSS_Out.write("$PGKC051,1*36\r\n");
#endif
      // Serial_GNSS_Out.flush(); delay(250);

      ledOff(SOC_GPIO_LED_TECHO_REV_1_GREEN);
      ledOff(SOC_GPIO_LED_TECHO_REV_1_RED);
      ledOff(SOC_GPIO_LED_TECHO_REV_1_BLUE);

      pinMode(SOC_GPIO_LED_TECHO_REV_1_GREEN, INPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_1_RED,   INPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_1_BLUE,  INPUT);

      pinMode(SOC_GPIO_PIN_IO_PWR,    INPUT);
      pinMode(SOC_GPIO_PIN_SFL_WP,    INPUT);
      pinMode(SOC_GPIO_PIN_SFL_HOLD,  INPUT);
      pinMode(SOC_GPIO_PIN_SFL_SS,    INPUT);
      break;

    case NRF52_LILYGO_TECHO_REV_2:
      ledOff(SOC_GPIO_LED_TECHO_REV_2_GREEN);
      ledOff(SOC_GPIO_LED_TECHO_REV_2_RED);
      ledOff(SOC_GPIO_LED_TECHO_REV_2_BLUE);

      pinMode(SOC_GPIO_LED_TECHO_REV_2_GREEN, INPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_2_RED,   INPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_2_BLUE,  INPUT);

      pinMode(SOC_GPIO_PIN_IO_PWR,    INPUT);
      pinMode(SOC_GPIO_PIN_SFL_HOLD,  INPUT);
      pinMode(SOC_GPIO_PIN_SFL_WP,    INPUT);
      pinMode(SOC_GPIO_PIN_SFL_SS,    INPUT);
      pinMode(SOC_GPIO_PIN_GNSS_WKE,  INPUT);
      pinMode(SOC_GPIO_PIN_GNSS_RST,  INPUT);
      /* Cut 3.3V power off on REV_2 board */
      pinMode(SOC_GPIO_PIN_3V3_PWR, INPUT_PULLDOWN);
      break;
    case NRF52_SEEED_T1000E:
      pinMode(SOC_GPIO_PIN_GNSS_T1000_RINT, INPUT_PULLDOWN);
      pinMode(SOC_GPIO_PIN_GNSS_T1000_SINT, INPUT_PULLDOWN);
      pinMode(SOC_GPIO_PIN_GNSS_T1000_RST,  INPUT_PULLDOWN);
      pinMode(SOC_GPIO_PIN_GNSS_T1000_VRTC, INPUT_PULLUP);
      pinMode(SOC_GPIO_PIN_GNSS_T1000_EN,   INPUT_PULLDOWN);

#if !defined(EXCLUDE_IMU)
      pinMode(SOC_GPIO_PIN_T1000_ACC_EN,    INPUT_PULLDOWN);
#endif /* EXCLUDE_IMU */
      pinMode(SOC_GPIO_PIN_T1000_BUZZER_EN, INPUT_PULLDOWN);
      pinMode(SOC_GPIO_PIN_T1000_3V3_EN,    INPUT_PULLDOWN);

      pinMode(SOC_GPIO_PIN_T1000_SS,        INPUT_PULLUP);

      digitalWrite(SOC_GPIO_LED_T1000_GREEN, 1-LED_STATE_ON);
      pinMode(SOC_GPIO_PIN_SFL_T1000_EN,    INPUT);
      pinMode(SOC_GPIO_LED_T1000_GREEN,     INPUT);
      break;
    case NRF52_ELECROW_TN_M3:
      digitalWrite(SOC_GPIO_PIN_GNSS_M3_WKE, LOW);
      digitalWrite(SOC_GPIO_LED_M3_RGB_PWR,  LOW);

      digitalWrite(SOC_GPIO_LED_M3_RED,  1-LED_STATE_ON);
      digitalWrite(SOC_GPIO_LED_M3_GREEN,1-LED_STATE_ON);
      digitalWrite(SOC_GPIO_LED_M3_BLUE, 1-LED_STATE_ON);

      pinMode(SOC_GPIO_LED_M3_RED,       INPUT);
      pinMode(SOC_GPIO_LED_M3_GREEN,     INPUT);
      pinMode(SOC_GPIO_LED_M3_BLUE,      INPUT);
      pinMode(SOC_GPIO_LED_M3_RGB_PWR,   INPUT);

//      pinMode(SOC_GPIO_PIN_GNSS_M3_WKE,  INPUT);
      pinMode(SOC_GPIO_PIN_GNSS_M3_RST,  INPUT);
      pinMode(SOC_GPIO_PIN_GNSS_M3_EN,   INPUT);
      pinMode(SOC_GPIO_PIN_M3_EEPROM_EN, INPUT);
      pinMode(SOC_GPIO_PIN_M3_EN1,       INPUT);
      pinMode(SOC_GPIO_PIN_M3_EN2,       INPUT);
      pinMode(SOC_GPIO_PIN_M3_ADC_EN,    INPUT);
      pinMode(SOC_GPIO_PIN_M3_ACC_EN,    INPUT);
      pinMode(SOC_GPIO_PIN_M3_TEMP_EN,   INPUT);
      break;
    case NRF52_NORDIC_PCA10059:
    default:
//      ledOff(SOC_GPIO_LED_PCA10059_GREEN);
      ledOff(SOC_GPIO_LED_PCA10059_RED);
      ledOff(SOC_GPIO_LED_PCA10059_BLUE);
      ledOff(SOC_GPIO_LED_PCA10059_STATUS);

//      pinMode(SOC_GPIO_LED_PCA10059_GREEN,  INPUT);
      pinMode(SOC_GPIO_LED_PCA10059_RED,    INPUT);
      pinMode(SOC_GPIO_LED_PCA10059_BLUE,   INPUT);
      pinMode(SOC_GPIO_LED_PCA10059_STATUS, INPUT);
      break;
  }

  // Reset watchdog during cleanup to prevent timeout
#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
  if (nrf_wdt_started(NRF_WDT)) {
    Watchdog.reset();
  }
#endif

  Serial_GNSS_In.end();

  // pinMode(SOC_GPIO_PIN_GNSS_RX, INPUT);
  // pinMode(SOC_GPIO_PIN_GNSS_TX, INPUT);

  // pinMode(SOC_GPIO_PIN_BATTERY, INPUT);

  if (i2c != nullptr) Wire.end();

  pinMode(SOC_GPIO_PIN_SDA,  INPUT);
  pinMode(SOC_GPIO_PIN_SCL,  INPUT);

  // pinMode(SOC_GPIO_PIN_MOSI, INPUT);
  // pinMode(SOC_GPIO_PIN_MISO, INPUT);
  // pinMode(SOC_GPIO_PIN_SCK,  INPUT);
  if (nRF52_board != NRF52_SEEED_T1000E &&
      nRF52_board != NRF52_ELECROW_TN_M3) {
    pinMode(SOC_GPIO_PIN_SS, INPUT_PULLUP);
  }
  // pinMode(SOC_GPIO_PIN_BUSY, INPUT);
  pinMode(lmic_pins.rst,  INPUT);

  int mode_button_pin;
  // pinMode(SOC_GPIO_PIN_PAD,    INPUT);
    switch (nRF52_board)
  {
      case NRF52_SEEED_T1000E:
        mode_button_pin = SOC_GPIO_PIN_T1000_BUTTON;
        break;
      case NRF52_LILYGO_TECHO_REV_0:
      case NRF52_LILYGO_TECHO_REV_1:
      case NRF52_LILYGO_TECHO_REV_2:
      case NRF52_NORDIC_PCA10059:
      default:
        mode_button_pin = SOC_GPIO_PIN_BUTTON;
        break;
  }
  pinMode(mode_button_pin, nRF52_board == NRF52_LILYGO_TECHO_REV_1 ? INPUT_PULLUP   :
                           nRF52_board == NRF52_SEEED_T1000E       ? INPUT_PULLDOWN :
                           INPUT);

  while (digitalRead(mode_button_pin) == (nRF52_board == NRF52_SEEED_T1000E ? HIGH : LOW));
  delay(100);

#if defined(USE_TINYUSB)
  if (nRF52_board != NRF52_SEEED_T1000E) {
    Serial1.end();
  }

  // pinMode(SOC_GPIO_PIN_CONS_RX, INPUT);
  // pinMode(SOC_GPIO_PIN_CONS_TX, INPUT);
#endif

  // Reset watchdog again before final cleanup
#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
  if (nrf_wdt_started(NRF_WDT)) {
    Watchdog.reset();
  }
#endif

  // setup wake-up pins
  switch (reason)
  {
  case SOFTRF_SHUTDOWN_BUTTON:
  case SOFTRF_SHUTDOWN_LOWBAT:
    NRF_POWER->GPREGRET = DFU_MAGIC_SKIP;
    pinMode(mode_button_pin, nRF52_board == NRF52_SEEED_T1000E ?
                             INPUT_PULLDOWN_SENSE /* INPUT_SENSE_HIGH */ :
                             INPUT_PULLUP_SENSE   /* INPUT_SENSE_LOW  */);
    break;
#if defined(USE_SERIAL_DEEP_SLEEP)
  case SOFTRF_SHUTDOWN_NMEA:
    pinMode(SOC_GPIO_PIN_CONS_RX, INPUT_PULLUP_SENSE /* INPUT_SENSE_LOW */);
    break;
#endif
  default:
    break;
  }

  Serial.end();

#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
  // Reset watchdog one more time before entering system off
  if (nrf_wdt_started(NRF_WDT)) {
    Watchdog.reset();
  }
#endif

  nRF52_system_off();
}

static void nRF52_reset()
{
  if (nrf_wdt_started(NRF_WDT)) {
    // When WDT is active - CRV, RREN and CONFIG are blocked
    // There is no way to stop/disable watchdog using source code
    // It can only be reset by WDT timeout, Pin reset, Power reset
#if defined(USE_EPAPER)
    if (hw_info.display == DISPLAY_EPD_1_54) {

#if defined(USE_EPD_TASK)
      while (EPD_update_in_progress != EPD_UPDATE_NONE) { delay(100); }
//    while (!SoC->Display_lock()) { delay(10); }
#endif

      EPD_Message("PLEASE,", "WAIT..");
    }
#endif /* USE_EPAPER */
    while (true) { delay(100); }
  } else {
    NVIC_SystemReset();
  }
}

static uint32_t nRF52_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
  uint32_t id = DEVICE_ID_LOW;

  return DevID_Mapper(id);
#else
  return (SOFTRF_ADDRESS & 0xFFFFFFFFU );
#endif
}

static void* nRF52_getResetInfoPtr()
{
  return (void *) &reset_info;
}

static String nRF52_getResetInfo()
{
  switch (reset_info.reason)
  {
    default                     : return F("No reset information available");
  }
}

static String nRF52_getResetReason()
{
  switch (reset_info.reason)
  {
    case REASON_DEFAULT_RST       : return F("DEFAULT");
    case REASON_WDT_RST           : return F("WDT");
    case REASON_EXCEPTION_RST     : return F("EXCEPTION");
    case REASON_SOFT_WDT_RST      : return F("SOFT_WDT");
    case REASON_SOFT_RESTART      : return F("SOFT_RESTART");
    case REASON_DEEP_SLEEP_AWAKE  : return F("DEEP_SLEEP_AWAKE");
    case REASON_EXT_SYS_RST       : return F("EXT_SYS");
    default                       : return F("NO_MEAN");
  }
}

static uint32_t nRF52_getFreeHeap()
{
  return dbgHeapTotal() - dbgHeapUsed();
}

static long nRF52_random(long howsmall, long howBig)
{
  return random(howsmall, howBig);
}

#if defined(USE_USB_MIDI)
byte note_sequence[] = {62,65,69,65,67,67,65,64,69,69,67,67,62,62};
#endif /* USE_USB_MIDI */

static void nRF52_Buzzer_test(int var)
{
#if defined(USE_USB_MIDI)
  if (USBDevice.mounted() && settings->volume != BUZZER_OFF) {
    unsigned int position = 0;
    unsigned int current  = 0;

    for (; position <= sizeof(note_sequence); position++) {
      // Setup variables for the current and previous
      // positions in the note sequence.
      current = position;
      // If we currently are at position 0, set the
      // previous position to the last note in the sequence.
      unsigned int previous = (current == 0) ? (sizeof(note_sequence)-1) : current - 1;

      // Send Note On for current position at full velocity (127) on channel 1.
      MIDI_USB.sendNoteOn(note_sequence[current], 127, MIDI_CHANNEL_TRAFFIC);

      // Send Note Off for previous note.
      MIDI_USB.sendNoteOff(note_sequence[previous], 0, MIDI_CHANNEL_TRAFFIC);

      delay(286);
    }

    MIDI_USB.sendNoteOff(note_sequence[current], 0, MIDI_CHANNEL_TRAFFIC);
  }
#endif /* USE_USB_MIDI */

#if defined(USE_PWM_SOUND)
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && settings->volume != BUZZER_OFF) {
    tone(SOC_GPIO_PIN_BUZZER, 440,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 640,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 840,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 1040, 500); delay(600);
  }
#endif /* USE_PWM_SOUND */
}

#if defined(USE_BLE_MIDI)
extern BLEMidi blemidi;
extern midi::MidiInterface<BLEMidi> MIDI_BLE;
#endif /* USE_BLE_MIDI */

static void nRF52_RFModeBeeps(int mode) {
  #if defined(USE_PWM_SOUND)
  uint8_t v = settings->volume;
  switch (mode) {
    case RF_PROTOCOL_FANET:
      SoC->Buzzer_tone(840, v); delay(100); SoC->Buzzer_tone(0, v); delay(100);
      SoC->Buzzer_tone(840, v); delay(100); SoC->Buzzer_tone(0, v); delay(100);
     
      break;
    case RF_PROTOCOL_LATEST:
      SoC->Buzzer_tone(840, v); delay(400); SoC->Buzzer_tone(0, v); delay(100);
      SoC->Buzzer_tone(840, v); delay(400); SoC->Buzzer_tone(0, v); delay(100);
       break;
    case RF_PROTOCOL_ADSL:
      SoC->Buzzer_tone(840, v); delay(100); SoC->Buzzer_tone(0, v); delay(100);
      SoC->Buzzer_tone(840, v); delay(100); SoC->Buzzer_tone(0, v); delay(100);
      SoC->Buzzer_tone(840, v); delay(100); SoC->Buzzer_tone(0, v); delay(100);
       break;
    default:
      tone(SOC_GPIO_PIN_BUZZER, 1000,  500); delay(300);
      break;
  }
  #endif /* USE_PWM_SOUND */
}


static void nRF52_Batt_beeps(int var, float batt)
{
#if defined(USE_PWM_SOUND)
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && settings->volume != BUZZER_OFF) {
    uint8_t v = settings->volume;
    uint8_t beeps = 0;
    //Check battery voltage and play n number of tones based on voltage
    if (batt > 4.00) {
      beeps = 5; //95% or more
    }
    else if (batt > 3.80 && batt <= 4.00) {beeps = 4;} //75% or more
    
    else if (batt > 3.70 && batt <= 3.80) {beeps = 3;} //55% or more
    
    else if (batt > 3.60 && batt <= 3.70) {beeps = 2;} //35% or more
    
    else if (batt > 3.40 && batt <= 3.60) {beeps = 1;} //15% or ore
    
    else {beeps = 0;}
    //Play beeps
    if (beeps == 0) {
      SoC->Buzzer_tone(640, v); delay(400); SoC->Buzzer_tone(0, v); delay(100);
    }
    else {
    for (int i = 0; i < beeps; i++) {
      SoC->Buzzer_tone(1000, v); delay(100); SoC->Buzzer_tone(0, v); delay(100);
    }
  }
    delay(1000);
    //Beeps confirming RF protocol
    nRF52_RFModeBeeps(settings->rf_protocol);
    
  }
#endif /* USE_PWM_SOUND */
}  
static void nRF52_BuzzerVolume(uint8_t state)
{
  #if defined(USE_PWM_SOUND)
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN) {
  switch (state)
  {
    case BUZZER_VOLUME_FULL:
      tone(SOC_GPIO_PIN_BUZZER, 640,  500); delay(500);
      tone(SOC_GPIO_PIN_BUZZER, 840,  500); delay(500);
      break;
    case BUZZER_VOLUME_LOW:
      tone(SOC_GPIO_PIN_BUZZER, 740,  500); delay(700);
      tone(SOC_GPIO_PIN_BUZZER, 740,  600); delay(500);
      break;
    case BUZZER_OFF:
      tone(SOC_GPIO_PIN_BUZZER, 840,  500); delay(500);
      tone(SOC_GPIO_PIN_BUZZER, 640,  500); delay(500);
      break;
    default:

      break;
  }
  }
#endif /* USE_PWM_SOUND */
}

static void nRF52_Buzzer_tone(int hz, uint8_t volume)
{
#if defined(USE_PWM_SOUND)
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && volume != BUZZER_OFF) {
    if (hz > 0) {
      tone(SOC_GPIO_PIN_BUZZER, hz, ALARM_TONE_MS);
    } else {
      noTone(SOC_GPIO_PIN_BUZZER);
    }
  }
#endif /* USE_PWM_SOUND */

#if defined(USE_USB_MIDI)
  if (USBDevice.mounted() && volume != BUZZER_OFF) {
    if (hz > 0) {
      MIDI_USB.sendNoteOn (60, 127, MIDI_CHANNEL_TRAFFIC); // 60 == middle C
    } else {
      MIDI_USB.sendNoteOff(60,   0, MIDI_CHANNEL_TRAFFIC);
    }
  }
#endif /* USE_USB_MIDI */

#if defined(USE_BLE_MIDI)
  if (volume != BUZZER_OFF  &&
      Bluefruit.connected() &&
      blemidi.notifyEnabled()) {
    if (hz > 0) {
      MIDI_BLE.sendNoteOn (60, 127, MIDI_CHANNEL_TRAFFIC); // 60 == middle C
    } else {
      MIDI_BLE.sendNoteOff(60,   0, MIDI_CHANNEL_TRAFFIC);
    }
  }
#endif /* USE_BLE_MIDI */
}

static void nRF52_Buzzer_GPSfix() {
      uint8_t v = settings->volume;

    SoC->Buzzer_tone(1046, v); delay(200); SoC->Buzzer_tone(0, v);
    SoC->Buzzer_tone(1175, v); delay(200); SoC->Buzzer_tone(0, v);
    SoC->Buzzer_tone(1318, v); delay(200); SoC->Buzzer_tone(0, v);
    SoC->Buzzer_tone(1397, v); delay(400); SoC->Buzzer_tone(0, v);
                              delay(200);
    SoC->Buzzer_tone(1046, v); delay(200); SoC->Buzzer_tone(0, v);
    SoC->Buzzer_tone(1397, v); delay(400); SoC->Buzzer_tone(0, v);
}

static void nRF52_WiFi_set_param(int ndx, int value)
{
  /* NONE */
}

static void nRF52_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* NONE */
}

static bool nRF52_EEPROM_begin(size_t size)
{
  if (size > EEPROM.length()) {
    return false;
  }

  EEPROM.begin();

  return true;
}

static void nRF52_EEPROM_extension(int cmd)
{
  uint8_t *raw = (uint8_t *) &ui_settings;

  switch (cmd)
  {
    case EEPROM_EXT_STORE:
      for (int i=0; i<sizeof(ui_settings_t); i++) {
        EEPROM.write(sizeof(eeprom_t) + i, raw[i]);
      }
      return;
    case EEPROM_EXT_DEFAULTS:
      //ui_settings.adapter      = 0;
      //ui_settings.connection   = 0;
#if defined(DEFAULT_REGION_US)
      ui_settings.units        = UNITS_IMPERIAL;
#else
      ui_settings.units        = UNITS_METRIC;
#endif
      ui_settings.zoom         = ZOOM_MEDIUM;
      //ui_settings.protocol     = PROTOCOL_NMEA;
      //ui_settings.baudrate     = 0;
      //strcpy(ui->server, "");
      //strcpy(ui->key,    "");
      ui_settings.rotate       = ROTATE_0;
      ui_settings.orientation  = DIRECTION_TRACK_UP;
      ui_settings.adb          = DB_NONE;
      ui_settings.epdidpref    = ID_TYPE;
      ui_settings.viewmode     = VIEW_MODE_STATUS;
      ui_settings.voice        = VOICE_OFF;
      ui_settings.antighost    = ANTI_GHOSTING_OFF;
      //ui_settings.filter       = TRAFFIC_FILTER_OFF;
      ui_settings.power_save   = 0;
      ui_settings.team         = 0;
      break;
    case EEPROM_EXT_LOAD:
    default:
      for (int i=0; i<sizeof(ui_settings_t); i++) {
        raw[i] = EEPROM.read(sizeof(eeprom_t) + i);
      }

#if defined(USE_JSON_SETTINGS)

      if ( nRF52_has_spiflash && FATFS_is_mounted ) {
        File file = fatfs.open("/settings.json", FILE_READ);

        if (file) {

          DeserializationError error = deserializeJson(nRF52_jsonDoc, file);
          if (! error) {

            JsonObject root = nRF52_jsonDoc.as<JsonObject>();

            if (root.containsKey("class")) {
              JsonVariant msg_class = root["class"];
              const char *msg_class_s = msg_class.as<char*>();
              if (!strcmp(msg_class_s,"SOFTRF")) {
                parseSettings  (root);
                parseUISettings(root);
              }
            }
          }
          file.close();
        }
      }

#endif // USE_JSON_SETTINGS

#if defined(USE_WEBUSB_SETTINGS) && !defined(USE_WEBUSB_SERIAL)

      usb_web.setLandingPage(&landingPage);
      usb_web.begin();

#endif /* USE_WEBUSB_SETTINGS */

      break;
  }
}

static void nRF52_SPI_begin()
{
  switch (nRF52_board)
  {
    case NRF52_SEEED_T1000E:
      SPI.setPins(SOC_GPIO_PIN_T1000_MISO,
                  SOC_GPIO_PIN_T1000_SCK,
                  SOC_GPIO_PIN_T1000_MOSI);
      break;
    case NRF52_ELECROW_TN_M3:
      SPI.setPins(SOC_GPIO_PIN_M3_MISO,
                  SOC_GPIO_PIN_M3_SCK,
                  SOC_GPIO_PIN_M3_MOSI);
      break;
    case NRF52_NORDIC_PCA10059:
      SPI.setPins(SOC_GPIO_PIN_PCA10059_MISO,
                  SOC_GPIO_PIN_PCA10059_SCK,
                  SOC_GPIO_PIN_PCA10059_MOSI);
      break;
    case NRF52_LILYGO_TECHO_REV_0:
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    default:
      SPI.setPins(SOC_GPIO_PIN_TECHO_REV_0_MISO,
                  SOC_GPIO_PIN_TECHO_REV_0_SCK,
                  SOC_GPIO_PIN_TECHO_REV_0_MOSI);
      break;
  }

  SPI.begin();
}

static void nRF52_swSer_begin(unsigned long baud)
{
  Serial_GNSS_In.flush();
  while (Serial_GNSS_In.available() > 0) { Serial_GNSS_In.read(); }

  switch (nRF52_board)
  {
    case NRF52_SEEED_T1000E:
      Serial_GNSS_In.setPins(SOC_GPIO_PIN_GNSS_T1000_RX, SOC_GPIO_PIN_GNSS_T1000_TX);
      break;
    case NRF52_ELECROW_TN_M3:
      Serial_GNSS_In.setPins(SOC_GPIO_PIN_GNSS_M3_RX, SOC_GPIO_PIN_GNSS_M3_TX);
      break;
    case NRF52_LILYGO_TECHO_REV_0:
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_NORDIC_PCA10059:
    default:
      Serial_GNSS_In.setPins(SOC_GPIO_PIN_GNSS_RX, SOC_GPIO_PIN_GNSS_TX);
      break;
  }
  Serial_GNSS_In.begin(baud);
}

static void nRF52_swSer_enableRx(boolean arg)
{
  /* NONE */
}

SemaphoreHandle_t Display_Semaphore;
unsigned long TaskInfoTime;

#if defined(USE_EPAPER)

#include <SoftSPI.h>
SoftSPI swSPI(SOC_GPIO_PIN_EPD_MOSI,
              SOC_GPIO_PIN_EPD_MOSI, /* half duplex */
              SOC_GPIO_PIN_EPD_SCK);

static nRF52_display_id nRF52_EPD_ident()
{
  nRF52_display_id rval = EP_GDEH0154D67; /* default */

  digitalWrite(SOC_GPIO_PIN_EPD_SS, HIGH);
  pinMode(SOC_GPIO_PIN_EPD_SS, OUTPUT);
  digitalWrite(SOC_GPIO_PIN_EPD_DC, HIGH);
  pinMode(SOC_GPIO_PIN_EPD_DC, OUTPUT);

  digitalWrite(SOC_GPIO_PIN_EPD_RST, LOW);
  pinMode(SOC_GPIO_PIN_EPD_RST, OUTPUT);
  delay(20);
  pinMode(SOC_GPIO_PIN_EPD_RST, INPUT_PULLUP);
  delay(200);
  pinMode(SOC_GPIO_PIN_EPD_BUSY, INPUT);

  swSPI.begin();

  uint8_t buf[11];

  taskENTER_CRITICAL();

  digitalWrite(SOC_GPIO_PIN_EPD_DC, LOW);
  digitalWrite(SOC_GPIO_PIN_EPD_SS, LOW);

  swSPI.transfer_out(0x2D /* 0x2E */);

  pinMode(SOC_GPIO_PIN_EPD_MOSI, INPUT);
  digitalWrite(SOC_GPIO_PIN_EPD_DC, HIGH);

  for (int i=0; i<10; i++) {
    buf[i] = swSPI.transfer_in();
  }

  digitalWrite(SOC_GPIO_PIN_EPD_SCK, LOW);
  digitalWrite(SOC_GPIO_PIN_EPD_DC,  LOW);
  digitalWrite(SOC_GPIO_PIN_EPD_SS,  HIGH);

  taskEXIT_CRITICAL();

  swSPI.end();

#if 0
  for (int i=0; i<10; i++) {
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

/*
 *  0x2D:
 *  FF FF FF FF FF FF FF FF FF FF FF - C1
 *  00 00 00 00 00 FF 40 00 00 00 01 - D67 SYX 1942
 *  00 00 00 FF 00 00 40 01 00 00 00 - D67 SYX 2118
 *
 *  0x2E:
 *  00 00 00 00 00 00 00 00 00 00    - C1
 *  00 00 00 00 00 00 00 00 00 00    - D67 SYX 1942
 *  00 05 00 9A 00 55 35 37 14 0C    - D67 SYX 2118
 */
#endif

  bool is_ff = true;
  for (int i=0; i<10; i++) {
    if (buf[i] != 0xFF) {is_ff = false; break;}
  }

  bool is_00 = true;
  for (int i=0; i<10; i++) {
    if (buf[i] != 0x00) {is_00 = false; break;}
  }

  if (is_ff || is_00) {
//    rval = EP_DEPG0150BN; /* TBD */
  }

  return rval;
}

#endif /* USE_EPAPER */

static byte nRF52_Display_setup()
{
  byte rval = DISPLAY_NONE;

#if defined(USE_EPAPER)

#if SPI_INTERFACES_COUNT >= 2
  SPI1.setPins(SOC_GPIO_PIN_EPD_MISO,
               SOC_GPIO_PIN_EPD_SCK,
               SOC_GPIO_PIN_EPD_MOSI);
#endif

  if (nRF52_display == EP_UNKNOWN) {
    nRF52_display = nRF52_EPD_ident();
  }

  switch (nRF52_display)
  {
  case EP_GDEP015OC1:
    display = &epd_c1;
    break;
  case EP_DEPG0150BN:
    display = &epd_bn;
    break;
  case EP_GDEH0154D67:
  default:
    display = &epd_d67;
    break;
  }

  if (EPD_setup(true)) {

#if defined(USE_EPD_TASK)
    Display_Semaphore = xSemaphoreCreateBinary();

    if( Display_Semaphore != NULL ) {
      xSemaphoreGive( Display_Semaphore );
    }

    xTaskCreate(EPD_Task, "EPD", EPD_STACK_SZ, NULL, /* TASK_PRIO_HIGH */ TASK_PRIO_LOW , &EPD_Task_Handle);

    TaskInfoTime = millis();
#endif
    rval = DISPLAY_EPD_1_54;
  }

  /* EPD back light off */
  pinMode(SOC_GPIO_PIN_EPD_BLGT, OUTPUT);
  digitalWrite(SOC_GPIO_PIN_EPD_BLGT, LOW);

#endif /* USE_EPAPER */

  return rval;
}

static void nRF52_Display_loop()
{
#if defined(USE_EPAPER)
  EPD_loop();

#if 0
  if (millis() - TaskInfoTime > 5000) {
    char pcWriteBuffer[512];
    vTaskList(pcWriteBuffer );
    Serial.println(pcWriteBuffer); Serial.flush();
    TaskInfoTime = millis();
  }
#endif
#endif /* USE_EPAPER */
}

static void nRF52_Display_fini(int reason)
{
#if defined(USE_EPAPER)

  EPD_fini(reason, screen_saver);

#if defined(USE_EPD_TASK)
  if( EPD_Task_Handle != NULL )
  {
    vTaskDelete( EPD_Task_Handle );
  }

  if( Display_Semaphore != NULL )
  {
    vSemaphoreDelete( Display_Semaphore );
  }
#endif

  SPI1.end();

  // pinMode(SOC_GPIO_PIN_EPD_MISO, INPUT);
  // pinMode(SOC_GPIO_PIN_EPD_MOSI, INPUT);
  // pinMode(SOC_GPIO_PIN_EPD_SCK,  INPUT);
  pinMode(SOC_GPIO_PIN_EPD_SS,   INPUT);
  pinMode(SOC_GPIO_PIN_EPD_DC,   INPUT);
  pinMode(SOC_GPIO_PIN_EPD_RST,  INPUT);
  // pinMode(SOC_GPIO_PIN_EPD_BUSY, INPUT);
  pinMode(SOC_GPIO_PIN_EPD_BLGT, INPUT);

#endif /* USE_EPAPER */
}

static bool nRF52_Display_lock()
{
  bool rval = false;

  if ( Display_Semaphore != NULL ) {
    rval = (xSemaphoreTake( Display_Semaphore, ( TickType_t ) 0 ) == pdTRUE);
  }
//Serial.print("Display_lock: "); Serial.println(rval); Serial.flush();
  return rval;
}

static bool nRF52_Display_unlock()
{
  bool rval = false;

  if ( Display_Semaphore != NULL ) {
    rval = (xSemaphoreGive( Display_Semaphore ) == pdTRUE);
  }
//Serial.print("Display_unlock: "); Serial.println(rval); Serial.flush();
  return rval;
}

static void nRF52_Battery_setup()
{

}

static float nRF52_Battery_param(uint8_t param)
{
  uint32_t bat_adc_pin;
  float rval, voltage, mult;

  switch (param)
  {
  case BATTERY_PARAM_THRESHOLD:
    rval = hw_info.model == SOFTRF_MODEL_BADGE ? BATTERY_THRESHOLD_LIPO   :
        hw_info.model == SOFTRF_MODEL_CARD     ? BATTERY_THRESHOLD_LIPO   :
        hw_info.model == SOFTRF_MODEL_POCKET   ? BATTERY_THRESHOLD_LIPO   :
                                                 BATTERY_THRESHOLD_NIMHX2;
    break;

  case BATTERY_PARAM_CUTOFF:
    rval = hw_info.model == SOFTRF_MODEL_BADGE ? BATTERY_CUTOFF_LIPO   :
    hw_info.model == SOFTRF_MODEL_CARD     ? BATTERY_CUTOFF_LIPO   :
    hw_info.model == SOFTRF_MODEL_POCKET   ? BATTERY_CUTOFF_LIPO   :
    hw_info.model == SOFTRF_MODEL_POCKET   ? BATTERY_CUTOFF_LIPO   :
                                                 BATTERY_CUTOFF_NIMHX2;
    break;

  case BATTERY_PARAM_CHARGE:
    // assume a LiPo battery, for which full=4.2, threshold=3.5 and cutoff=3.2
    voltage = Battery_voltage();
    if (voltage < BATTERY_CUTOFF_LIPO)
      return 0;
    if (voltage > BATTERY_FULL_LIPO)
      return 100.0;
    if (voltage < BATTERY_THRESHOLD_LIPO) {
      return ((voltage - BATTERY_CUTOFF_LIPO)
          * (10.0 / (BATTERY_THRESHOLD_LIPO - BATTERY_CUTOFF_LIPO)));   // 0 to 10% over 0.3V
    }
    return (10.0 + (voltage - BATTERY_THRESHOLD_LIPO)
                 * (90.0 / (BATTERY_FULL_LIPO - BATTERY_THRESHOLD_LIPO)));   // 10 to 100% over 0.7V
    break;

  case BATTERY_PARAM_VOLTAGE:
  default:

    // Set the analog reference to 3.0V (default = 3.6V)
    analogReference(AR_INTERNAL_3_0);

    // Set the resolution to 12-bit (0..4095)
    analogReadResolution(12); // Can be 8, 10, 12 or 14

    // Let the ADC settle
    delay(1);
    switch (nRF52_board)
    {
      case NRF52_SEEED_T1000E:
        bat_adc_pin = SOC_GPIO_PIN_T1000_BATTERY;
        mult        = SOC_ADC_T1000_VOLTAGE_DIV;
        break;
      case NRF52_ELECROW_TN_M3:
        bat_adc_pin = SOC_GPIO_PIN_M3_BATTERY;
        mult        = SOC_ADC_M3_VOLTAGE_DIV;
        break;
      case NRF52_LILYGO_TECHO_REV_0:
      case NRF52_LILYGO_TECHO_REV_1:
      case NRF52_LILYGO_TECHO_REV_2:
      case NRF52_NORDIC_PCA10059:
      default:
        bat_adc_pin = SOC_GPIO_PIN_BATTERY;
        mult        = SOC_ADC_VOLTAGE_DIV;
        break;
    }
    // Get the raw 12-bit, 0..3000mV ADC value
    // DEBUG: Log ADC read attempt
    // Serial.printf("[ADC_DEBUG] About to read battery ADC pin %d\r\n", bat_adc_pin);
    // Serial.flush();
    uint32_t adc_start = millis();
    voltage = analogRead(bat_adc_pin);
    uint32_t adc_elapsed = millis() - adc_start;
    // Serial.printf("[ADC_DEBUG] ADC read completed in %lu ms, raw value: %f\r\n", adc_elapsed, voltage);
    // Serial.flush();

    // Set the ADC back to the default settings
    analogReference(AR_DEFAULT);
    analogReadResolution(10);

    // Convert the raw value to compensated mv, taking the resistor-
    // divider into account (providing the actual LIPO voltage)
    // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
    voltage *= (mult * VBAT_MV_PER_LSB);
    rval = voltage  * 0.001;
    // Serial.printf("[ADC_DEBUG] Final voltage: %f V\r\n", rval);
    break;
  }

  return rval;
}

void nRF52_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long nRF52_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool nRF52_Baro_setup() {
    //return true;
    // Baro_probe() no longer called from Baro_setup() so need to call it here:
    if (Baro_probe()) {                  // found baro sensor on Wire
        Serial.println(F("BMP found"));
        return true;
    }
    return false;
}

static void nRF52_UATSerial_begin(unsigned long baud)
{

}

static void nRF52_UATModule_restart()
{

}

static void nRF52_WDT_setup()
{
  Watchdog.enable(12000);
}

static void nRF52_WDT_fini()
{
  // cannot disable nRF's WDT
  if (nrf_wdt_started(NRF_WDT)) {
    Watchdog.reset();
  }
}

#include <AceButton.h>
using namespace ace_button;

AceButton button_1(SOC_GPIO_PIN_BUTTON);
AceButton button_2(SOC_GPIO_PIN_PAD);

// The event handler for the button.
void handleEvent(AceButton* button, uint8_t eventType,
    uint8_t buttonState) {

  switch (eventType) {

    case AceButton::kEventClicked:
    // case AceButton::kEventReleased:

      if (button == &button_1) {
#if defined(SOFTRF_MODEL_T1000E) || defined(SOFTRF_MODEL_M3)
        if (eventType == AceButton::kEventClicked) {
          if (nRF52_board == NRF52_SEEED_T1000E ) {
            if (settings->volume == BUZZER_OFF) {
              nRF52_BuzzerVolume(BUZZER_VOLUME_FULL);
              settings->volume = BUZZER_VOLUME_FULL;
            } else if (settings->volume == BUZZER_VOLUME_FULL) {
              nRF52_BuzzerVolume(BUZZER_VOLUME_LOW);
              settings->volume = BUZZER_VOLUME_LOW;
            } else {
              nRF52_BuzzerVolume(BUZZER_OFF);
              settings->volume = BUZZER_OFF;
            }
          }
        } else if (eventType == AceButton::kEventReleased) {
          Serial.println(F("kEventReleased."));
        }
#elif defined(USE_EPAPER)
        if (FlightLogOpen)
            completeFlightLog();   // manually ensure complete flight log
        EPD_Mode(screen_saver);    // if screen_saver then redraws current page/mode
        screen_saver = false;
#endif
      } else if (button == &button_2) {    // touch
#if defined(USE_EPAPER)
        if (! screen_saver)
            EPD_Up();
#endif
      }
      break;

    case AceButton::kEventDoubleClicked:
      if (button == &button_1) {

#if defined(USE_EPAPER)
        if (settings->mode == SOFTRF_MODE_GPSBRIDGE) {
            settings->mode = SOFTRF_MODE_NORMAL;
            Serial.println(F("Switching to normal mode..."));
            EPD_Message("NORMAL", "MODE");
            delay(600);
        } else if (EPD_view_mode == VIEW_MODE_CONF) {
            EPD_view_mode = VIEW_CHANGE_SETTINGS;
            //Serial.println(F("Switching to change-settings screen..."));

        } else if (EPD_view_mode == VIEW_CHANGE_SETTINGS) {
            EPD_view_mode = VIEW_SAVE_SETTINGS;
            //Serial.println(F("Switching to save-settings screen..."));

        } else {   // view_modes other than CONF: toggle backlight
//        Serial.println(F("kEventDoubleClicked."));
        digitalWrite(SOC_GPIO_PIN_EPD_BLGT,
                     digitalRead(SOC_GPIO_PIN_EPD_BLGT) == LOW);
        }
      } else if (button == &button_2) {    // touch
        if (! screen_saver)
            EPD_Down();
#else
        if (eventType == AceButton::kEventDoubleClicked) {
          fanet_distress = !fanet_distress;
          if (fanet_distress) {
            fanet_sos_last_ms = 0;  /* send SOS message immediately */
            fanet_sos_count = 0;
            Serial.println(F("FANET SOS+DISTRESS mode ON"));
          } else {
            Serial.println(F("FANET SOS+DISTRESS mode OFF"));
          }
        }
#endif
      }
      break;

    case AceButton::kEventLongPressed:
      if (button == &button_1) {

#if defined(USE_EPAPER)
            if (digitalRead(SOC_GPIO_PIN_PAD) == LOW)     // touch while long-press
                screen_saver = true;
            else
                screen_saver = false;
#endif
            shutdown(SOFTRF_SHUTDOWN_BUTTON);
            Serial.println(F("This too will never be printed."));

#if defined(USE_EPAPER)
      } else if (button == &button_2) {    // long touch
          if (digitalRead(SOC_GPIO_PIN_BUTTON) != LOW) {    // long-touch without push
            EPD_Message("SCREEN", "SAVER");
            delay (1500);
            screen_saver = true;             // ignore touch until mode button pressed
            EPD_Message(NULL, NULL);         // blank the screen
          }
#endif
      }
      break;
  }
}

/* Callbacks for push button interrupt */
void onModeButtonEvent() {
  button_1.check();
}

/* Callbacks for touch button interrupt */
void onUpButtonEvent() {
  button_2.check();
}

static void nRF52_Button_setup()
{
  int mode_button_pin;
  int up_button_pin   = -1;

  switch (nRF52_board)
  {
    case NRF52_SEEED_T1000E:
      mode_button_pin = SOC_GPIO_PIN_T1000_BUTTON;
      break;
    case NRF52_ELECROW_TN_M3:
      mode_button_pin = SOC_GPIO_PIN_M3_BUTTON;
      break;
    case NRF52_LILYGO_TECHO_REV_0:
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_NORDIC_PCA10059:
      up_button_pin   = SOC_GPIO_PIN_PAD;
    default:
      mode_button_pin = SOC_GPIO_PIN_BUTTON;
      break;
  }
  // Button(s) uses external pull up resistor.
  pinMode(mode_button_pin, nRF52_board == NRF52_LILYGO_TECHO_REV_1 ? INPUT_PULLUP : 
                          nRF52_board == NRF52_SEEED_T1000E       ? INPUT_PULLDOWN :
                          INPUT);
  if (up_button_pin >= 0) { pinMode(up_button_pin, INPUT); }

   button_1.init(mode_button_pin, nRF52_board == NRF52_SEEED_T1000E ? LOW : HIGH);
  if (up_button_pin >= 0) { button_2.init(up_button_pin); }

  // Configure the ButtonConfig with the event handler, and enable all higher
  // level events.
  ButtonConfig* ModeButtonConfig = button_1.getButtonConfig();
  ModeButtonConfig->setEventHandler(handleEvent);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
  ModeButtonConfig->setFeature(
                    ButtonConfig::kFeatureSuppressClickBeforeDoubleClick);
//  ModeButtonConfig->setDebounceDelay(15);
  //ModeButtonConfig->setClickDelay(600);
  //ModeButtonConfig->setDoubleClickDelay(1500);
  ModeButtonConfig->setClickDelay(300);
  ModeButtonConfig->setDoubleClickDelay(600);
  ModeButtonConfig->setLongPressDelay(2000);

  ButtonConfig* UpButtonConfig = button_2.getButtonConfig();
  UpButtonConfig->setEventHandler(handleEvent);
  UpButtonConfig->setFeature(ButtonConfig::kFeatureClick);
  UpButtonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
  UpButtonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  UpButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
  UpButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
//  UpButtonConfig->setDebounceDelay(15);
  //UpButtonConfig->setClickDelay(600);
  //UpButtonConfig->setDoubleClickDelay(1500);
  UpButtonConfig->setClickDelay(300);
  UpButtonConfig->setDoubleClickDelay(600);
  UpButtonConfig->setLongPressDelay(2000);

//  attachInterrupt(digitalPinToInterrupt(mode_button_pin), onModeButtonEvent, CHANGE );
  attachInterrupt(digitalPinToInterrupt(up_button_pin),   onUpButtonEvent,   CHANGE );
}

static void nRF52_Button_loop()
{
  button_1.check();
  switch (nRF52_board)
  {
    case NRF52_LILYGO_TECHO_REV_0:
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_NORDIC_PCA10059:
      button_2.check();
      break;
    default:
      break;
  }
}

static void nRF52_Button_fini()
{
//  detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_BUTTON));
   switch (nRF52_board)
  {
    case NRF52_LILYGO_TECHO_REV_0:
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_NORDIC_PCA10059:
      detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_PAD));
      break;

    default:
      detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_PAD));
      break;
  }
}

#if defined(USE_WEBUSB_SERIAL) && !defined(USE_WEBUSB_SETTINGS)
void line_state_callback(bool connected)
{
  if ( connected ) usb_web.println("WebUSB Serial example");
}
#endif /* USE_WEBUSB_SERIAL */

static void nRF52_USB_setup()
{
  if (USBSerial && USBSerial != Serial) {
    USBSerial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
  }

#if defined(USE_WEBUSB_SERIAL) && !defined(USE_WEBUSB_SETTINGS)

  usb_web.setLandingPage(&landingPage);
  usb_web.setLineStateCallback(line_state_callback);
  //usb_web.setStringDescriptor("TinyUSB WebUSB");
  usb_web.begin();

#endif /* USE_WEBUSB_SERIAL */
}

static void nRF52_USB_loop()
{

}

static void nRF52_USB_fini()
{
  if (USBSerial && USBSerial != Serial) {
    USBSerial.end();
  }

#if defined(USE_WEBUSB_SERIAL) && !defined(USE_WEBUSB_SETTINGS)

  /* TBD */

#endif /* USE_WEBUSB_SERIAL */
}

static int nRF52_USB_available()
{
  int rval = 0;

  if (USBSerial) {
    rval = USBSerial.available();
  }

#if defined(USE_WEBUSB_SERIAL) && !defined(USE_WEBUSB_SETTINGS)

  if ((rval == 0) && USBDevice.mounted() && usb_web.connected()) {
    rval = usb_web.available();
  }

#endif /* USE_WEBUSB_SERIAL */

  return rval;
}

static int nRF52_USB_read()
{
  int rval = -1;

  if (USBSerial) {
    rval = USBSerial.read();
  }

#if defined(USE_WEBUSB_SERIAL) && !defined(USE_WEBUSB_SETTINGS)

  if ((rval == -1) && USBDevice.mounted() && usb_web.connected()) {
    rval = usb_web.read();
  }

#endif /* USE_WEBUSB_SERIAL */

  return rval;
}

static size_t nRF52_USB_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  if (USBSerial && (size < USBSerial.availableForWrite())) {
    rval = USBSerial.write(buffer, size);
  }

#if defined(USE_WEBUSB_SERIAL) && !defined(USE_WEBUSB_SETTINGS)

  size_t rval_webusb = size;

  if (USBDevice.mounted() && usb_web.connected()) {
    rval_webusb = usb_web.write(buffer, size);
  }

//  rval = min(rval, rval_webusb);

#endif /* USE_WEBUSB_SERIAL */

  return rval;
}

IODev_ops_t nRF52_USBSerial_ops = {
  "nRF52 USBSerial",
  nRF52_USB_setup,
  nRF52_USB_loop,
  nRF52_USB_fini,
  nRF52_USB_available,
  nRF52_USB_read,
  nRF52_USB_write
};

static bool nRF52_ADB_setup()
{
  if (FATFS_is_mounted) {
    const char fileName[] = "/Aircrafts/ogn.cdb";

    if (ucdb.open(fileName) != CDB_OK) {
      Serial.print("Invalid CDB: ");
      Serial.println(fileName);
    } else {
      ADB_is_open = true;
    }
  }

  return ADB_is_open;
}

static bool nRF52_ADB_fini()
{
  if (ADB_is_open) {
    ucdb.close();
    ADB_is_open = false;
  }

  return !ADB_is_open;
}

/*
 * One aircraft CDB (20000+ records) query takes:
 * 1)     FOUND : 5-7 milliseconds
 * 2) NOT FOUND :   3 milliseconds
 */
static bool nRF52_ADB_query(uint8_t type, uint32_t id, char *buf, size_t size)
{
  char key[8];
  char out[64];
  uint8_t tokens[3] = { 0 };
  cdbResult rt;
  int c, i = 0, token_cnt = 0;
  bool rval = false;

  if (!ADB_is_open) {
    return rval;
  }

  snprintf(key, sizeof(key),"%06X", id);

  rt = ucdb.findKey(key, strlen(key));

  switch (rt) {
    case KEY_FOUND:
      while ((c = ucdb.readValue()) != -1 && i < (sizeof(out) - 1)) {
        if (c == '|') {
          if (token_cnt < (sizeof(tokens) - 1)) {
            token_cnt++;
            tokens[token_cnt] = i+1;
          }
          c = 0;
        }
        out[i++] = (char) c;
      }
      out[i] = 0;

      switch (ui->epdidpref)
      {
      case ID_TAIL:
        snprintf(buf, size, "CN: %s",
          strlen(out + tokens[2]) ? out + tokens[2] : "N/A");
        break;
      case ID_MAM:
        snprintf(buf, size, "%s",
          strlen(out + tokens[0]) ? out + tokens[0] : "Unknown");
        break;
      case ID_REG:
      default:
        snprintf(buf, size, "%s",
          strlen(out + tokens[1]) ? out + tokens[1] : "REG: N/A");
        break;
      }

      rval = true;
      break;

    case KEY_NOT_FOUND:
    default:
      break;
  }

  return rval;
}

DB_ops_t nRF52_ADB_ops = {
  nRF52_ADB_setup,
  nRF52_ADB_fini,
  nRF52_ADB_query
};

const SoC_ops_t nRF52_ops = {
  SOC_NRF52,
  "nRF52",
  nRF52_setup,
  nRF52_post_init,
  nRF52_loop,
  nRF52_fini,
  nRF52_reset,
  nRF52_getChipId,
  nRF52_getResetInfoPtr,
  nRF52_getResetInfo,
  nRF52_getResetReason,
  nRF52_getFreeHeap,
  nRF52_random,
  nRF52_Batt_beeps,
  nRF52_Buzzer_test,
  nRF52_Buzzer_tone,
  nRF52_Buzzer_GPSfix,
  NULL,
  nRF52_WiFi_set_param,
  nRF52_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  nRF52_EEPROM_begin,
  nRF52_EEPROM_extension,
  nRF52_SPI_begin,
  nRF52_swSer_begin,
  nRF52_swSer_enableRx,
  &nRF52_Bluetooth_ops,
  &nRF52_USBSerial_ops,
  NULL,
  nRF52_Display_setup,
  nRF52_Display_loop,
  nRF52_Display_fini,
#if 0
  nRF52_Display_lock,
  nRF52_Display_unlock,
#endif
  nRF52_Battery_setup,
  nRF52_Battery_param,
  nRF52_GNSS_PPS_Interrupt_handler,
  nRF52_get_PPS_TimeMarker,
  nRF52_Baro_setup,
  nRF52_UATSerial_begin,
  nRF52_UATModule_restart,
  nRF52_WDT_setup,
  nRF52_WDT_fini,
  nRF52_Button_setup,
  nRF52_Button_loop,
  nRF52_Button_fini,
  &nRF52_ADB_ops
};

#endif /* ARDUINO_ARCH_NRF52 */
