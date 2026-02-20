/*
 * RFHelper.cpp
 * Copyright (C) 2016-2021 Linar Yusupov
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

#if defined(ARDUINO)
#include <SPI.h>
#endif /* ARDUINO */

#include "RF.h"
#include "../system/Time.h"
#include "../system/SoC.h"
#include "../TrafficHelper.h"
#include "Settings.h"
#include "Battery.h"

#include "../ui/Web.h"
#if !defined(EXCLUDE_MAVLINK)
#include "../protocol/data/MAVLink.h"
#endif /* EXCLUDE_MAVLINK */
#include <fec.h>
#if defined(USE_RADIOLIB)
#ifndef RadioSPI
#define RadioSPI        SPI
#endif
#endif /* USE_RADIOLIB */
#if LOGGER_IS_ENABLED
#include "../system/Log.h"
#endif /* LOGGER_IS_ENABLED */
#define DEBUG_RADIO_BASIC
// #define DEBUG
#define USE_LR1110

byte RxBuffer[MAX_PKT_SIZE] __attribute__((aligned(sizeof(uint32_t))));

time_t RF_time;
uint8_t RF_current_slot = 0;
uint8_t RF_current_chan = 0;
uint32_t RF_OK_from   = 0;
uint32_t RF_OK_until  = 0;
uint32_t TxTimeMarker = 0;
uint32_t TxTimeMarker2 = 0;
uint32_t TxEndMarker  = 0;
byte TxBuffer[MAX_PKT_SIZE] __attribute__((aligned(sizeof(uint32_t))));

uint32_t tx_packets_counter = 0;
uint32_t rx_packets_counter = 0;

int8_t which_rx_try = 0;
int8_t RF_last_rssi = 0;
uint32_t RF_last_crc = 0;
uint8_t RF_last_protocol = 0;
uint8_t current_RX_protocol;
uint8_t current_TX_protocol;
uint8_t dual_protocol = RF_SINGLE_PROTOCOL;
bool flr_adsl = false;

#if defined(USE_LR1110)
static const rf_proto_desc_t *lr11xx_current_protocol_ptr = NULL;
#endif

FreqPlan RF_FreqPlan;
static bool RF_ready = false;

static size_t RF_tx_size = 0;

const rfchip_ops_t *rf_chip = NULL;
bool RF_SX12XX_RST_is_connected = true;

const char *Protocol_ID[] = {
  [RF_PROTOCOL_NONE]      = "---",  // 0
  [RF_PROTOCOL_OGNTP]     = "OGN",
  [RF_PROTOCOL_P3I]       = "P3I",
  [RF_PROTOCOL_ADSB_1090] = "ADS",
  [RF_PROTOCOL_ADSB_UAT]  = "UAT",
  [RF_PROTOCOL_FANET]     = "FAN",
  [RF_PROTOCOL_LEGACY]    = "LEG",
  [RF_PROTOCOL_LATEST]    = "LAT",
  [RF_PROTOCOL_ADSL]      = "ADL",  // 8
  [RF_PROTOCOL_GDL90]     = "GDL"   // 9 - not an RF protocol
};

struct protocol_duo
{
    const uint8_t main;
    const uint8_t alt;
    const char *label;
};

protocol_duo protocol_duos[] = {
  {RF_PROTOCOL_LEGACY, RF_PROTOCOL_LATEST, "L+T"},
  {RF_PROTOCOL_LEGACY, RF_PROTOCOL_OGNTP,  "L+O"},
  {RF_PROTOCOL_LEGACY, RF_PROTOCOL_ADSL,   "L+A"},
  {RF_PROTOCOL_LATEST, RF_PROTOCOL_LEGACY, "T+L"},
  {RF_PROTOCOL_LATEST, RF_PROTOCOL_OGNTP,  "T+O"},
  {RF_PROTOCOL_LATEST, RF_PROTOCOL_ADSL,   "T+A"},
  {RF_PROTOCOL_LATEST, RF_PROTOCOL_FANET,  "TFa"},
  {RF_PROTOCOL_LATEST, RF_PROTOCOL_P3I,    "T+P"},
  {RF_PROTOCOL_OGNTP,  RF_PROTOCOL_LEGACY, "O+L"},
  {RF_PROTOCOL_OGNTP,  RF_PROTOCOL_LATEST, "O+T"},
  {RF_PROTOCOL_OGNTP,  RF_PROTOCOL_ADSL,   "O+A"},
  {RF_PROTOCOL_ADSL,   RF_PROTOCOL_OGNTP,  "A+O"},
  {RF_PROTOCOL_ADSL,   RF_PROTOCOL_LEGACY, "A+L"},
  {RF_PROTOCOL_ADSL,   RF_PROTOCOL_LATEST, "A+T"},
  {RF_PROTOCOL_FANET,  RF_PROTOCOL_LATEST, "F+T"},
  {RF_PROTOCOL_FANET,  RF_PROTOCOL_OGNTP,  "F+O"},
  {RF_PROTOCOL_FANET,  RF_PROTOCOL_ADSL,   "F+A"},
  {RF_PROTOCOL_P3I,    RF_PROTOCOL_LATEST, "P+T"},
  {RF_PROTOCOL_P3I,    RF_PROTOCOL_OGNTP,  "P+O"},
  {RF_PROTOCOL_P3I,    RF_PROTOCOL_ADSL,   "P+A"},
  {RF_PROTOCOL_NONE, RF_PROTOCOL_NONE, NULL}
};

const char *protocol_lbl(uint8_t main, uint8_t alt)
{
    if (alt == RF_PROTOCOL_NONE)
        return Protocol_ID[main];
    for (int i=0; ; i++) {
        if (protocol_duos[i].label == NULL)
            return "???";
        if (protocol_duos[i].main == main && protocol_duos[i].alt == alt)
            return protocol_duos[i].label;
    }
}

const char *dual_protocol_lbl[] = {
  [RF_SINGLE_PROTOCOL] = "PROTOCOL",
  [RF_FLR_ADSL]        = "FLR_ADSL",
  [RF_FLR_FANET]       = "FLR_FANET",
  [RF_FLR_P3I]         = "FLR_P3I"
};

size_t (*protocol_encode)(void *, container_t *);
size_t (*mainprotocol_encode)(void *, container_t *);
size_t (*altprotocol_encode)(void *, container_t *);
bool   (*protocol_decode)(void *, container_t *, ufo_t *);
bool   (*mainprotocol_decode)(void *, container_t *, ufo_t *);
bool   (*altprotocol_decode)(void *, container_t *, ufo_t *);
const rf_proto_desc_t  *curr_rx_protocol_ptr;
const rf_proto_desc_t  *curr_tx_protocol_ptr;
const rf_proto_desc_t  *mainprotocol_ptr;
const rf_proto_desc_t  *altprotocol_ptr;

/* Helper function to get protocol descriptor from protocol enum value */
static const rf_proto_desc_t* get_protocol_descriptor(uint8_t protocol_id) {
    switch (protocol_id) {
        case RF_PROTOCOL_ADSL:
            return &adsl_proto_desc;
        case RF_PROTOCOL_OGNTP:
            return &ogntp_proto_desc;
        case RF_PROTOCOL_P3I:
            return &p3i_proto_desc;
        case RF_PROTOCOL_FANET:
            return &fanet_proto_desc;
        case RF_PROTOCOL_LEGACY:
            return &legacy_proto_desc;
        case RF_PROTOCOL_LATEST:
        default:
            return &latest_proto_desc;
    }
}

/* Helper functions to get encode/decode functions from protocol descriptor */
static size_t (*get_protocol_encode_fn(const rf_proto_desc_t *proto))(void *, container_t *) {
    if (proto == NULL) return NULL;

    switch (proto->type) {
        case RF_PROTOCOL_OGNTP:
            return ogntp_encode;
        case RF_PROTOCOL_P3I:
            return p3i_encode;
        case RF_PROTOCOL_FANET:
            return fanet_encode;
        case RF_PROTOCOL_ADSL:
            return adsl_encode;
        case RF_PROTOCOL_LEGACY:
            return legacy_encode;
        case RF_PROTOCOL_LATEST:
            return legacy_encode;
        default:
            return NULL;
    }
}

static bool (*get_protocol_decode_fn(const rf_proto_desc_t *proto))(void *, container_t *, ufo_t *) {
    if (proto == NULL) return NULL;

    switch (proto->type) {
        case RF_PROTOCOL_OGNTP:
            return ogntp_decode;
        case RF_PROTOCOL_P3I:
            return p3i_decode;
        case RF_PROTOCOL_FANET:
            return fanet_decode;
        case RF_PROTOCOL_ADSL:
            return adsl_decode;
        case RF_PROTOCOL_LEGACY:
            return legacy_decode;
        case RF_PROTOCOL_LATEST:
            return legacy_decode;
        default:
            return NULL;
    }
}

static Slots_descr_t Time_Slots, *ts;
static uint8_t       RF_timing = RF_TIMING_INTERVAL;

extern const gnss_chip_ops_t *gnss_chip;

#define RF_CHANNEL_NONE 0xFF

static bool memeqzero(const uint8_t *data, size_t length)
{
	const uint8_t *p = data;
	size_t len;

	/* Check first 16 bytes manually */
	for (len = 0; len < 16; len++) {
		if (!length)
			return true;
		if (*p)
			return false;
		p++;
		length--;
	}

	/* Now we know that's zero, memcmp with self. */
	return memcmp((void *) data, (void *) p, length) == 0;
}

static bool nrf905_probe(void);
static void nrf905_setup(void);
static void nrf905_channel(uint8_t);
static bool nrf905_receive(void);
static void nrf905_transmit(void);
static void nrf905_shutdown(void);

static bool sx1276_probe(void);
static bool sx1262_probe(void);
static void sx12xx_setup(void);
static void sx12xx_channel(uint8_t);
static bool sx12xx_receive(void);
static void sx12xx_transmit(void);
static void sx1276_shutdown(void);
static void sx1262_shutdown(void);

static bool uatm_probe(void);
static void uatm_setup(void);
static void uatm_channel(uint8_t);
static bool uatm_receive(void);
static void uatm_transmit(void);
static void uatm_shutdown(void);

static bool cc13xx_probe(void);
static void cc13xx_setup(void);
static void cc13xx_channel(uint8_t);
static bool cc13xx_receive(void);
static void cc13xx_transmit(void);
static void cc13xx_shutdown(void);

static bool ognrf_probe(void);
static void ognrf_setup(void);
static void ognrf_channel(uint8_t);
static bool ognrf_receive(void);
static void ognrf_transmit(void);
static void ognrf_shutdown(void);

/* LR1110 radio driver declarations */
static bool lr11xx_probe();
static void lr11xx_setup();
static void lr11xx_channel(uint8_t);
static bool lr11xx_receive();
static void lr11xx_transmit();
static void lr11xx_resetup(void);
static void lr11xx_shutdown();
#if !defined(EXCLUDE_NRF905)
const rfchip_ops_t nrf905_ops = {
  RF_IC_NRF905,
  "NRF905",
  nrf905_probe,
  nrf905_setup,
  nrf905_channel,
  nrf905_receive,
  nrf905_transmit,
  nrf905_shutdown
};
#endif
#if !defined(EXCLUDE_SX12XX)
const rfchip_ops_t sx1276_ops = {
  RF_IC_SX1276,
  "SX127x",
  sx1276_probe,
  sx12xx_setup,
  sx12xx_channel,
  sx12xx_receive,
  sx12xx_transmit,
  sx1276_shutdown
};
#if defined(USE_BASICMAC)
const rfchip_ops_t sx1262_ops = {
  RF_IC_SX1262,
  "SX126x",
  sx1262_probe,
  sx12xx_setup,
  sx12xx_channel,
  sx12xx_receive,
  sx12xx_transmit,
  sx1262_shutdown
};
#endif /* USE_BASICMAC */
#endif /*EXCLUDE_SX12XX */
#if !defined(EXCLUDE_UATM)
const rfchip_ops_t uatm_ops = {
  RF_IC_UATM,
  "UATM",
  uatm_probe,
  uatm_setup,
  uatm_channel,
  uatm_receive,
  uatm_transmit,
  uatm_shutdown
};
#endif /* EXCLUDE_UATM */
#if !defined(EXCLUDE_CC13XX)
const rfchip_ops_t cc13xx_ops = {
  RF_IC_CC13XX,
  "CC13XX",
  cc13xx_probe,
  cc13xx_setup,
  cc13xx_channel,
  cc13xx_receive,
  cc13xx_transmit,
  cc13xx_shutdown
};
#endif /* EXCLUDE_CC13XX */

#if defined(USE_LR1110)
const rfchip_ops_t lr1110_ops = {
  RF_IC_LR1110,
  "LR1110",
  lr11xx_probe,
  lr11xx_setup,
  lr11xx_channel,
  lr11xx_receive,
  lr11xx_transmit,
  lr11xx_shutdown
};
#endif /* USE_LR1110 */

#if defined(USE_OGN_RF_DRIVER)

#define vTaskDelay  delay

#if defined(WITH_SI4X32)
#include <rf/si4x32/rfm.h>
#else
#include <rf/combo/rfm.h>
#endif /* WITH_SI4X32 */

const rfchip_ops_t ognrf_ops = {
  RF_DRV_OGN,
  "OGNDRV",
  ognrf_probe,
  ognrf_setup,
  ognrf_channel,
  ognrf_receive,
  ognrf_transmit,
  ognrf_shutdown
};
#endif /* USE_OGN_RF_DRIVER */

String Bin2Hex(byte *buffer, size_t size)
{
  String str = "";
  for (int i=0; i < size; i++) {
    byte c = buffer[i];
    str += (c < 0x10 ? "0" : "") + String(c, HEX);
  }
  return str;
}

uint8_t parity(uint32_t x) {
    uint8_t parity=0;
    while (x > 0) {
      if (x & 0x1) {
          parity++;
      }
      x >>= 1;
    }
    // return (parity % 2);
    return (parity & 0x01);
}

#if !defined(EXCLUDE_NRF905)
/*
 * NRF905-specific code
 *
 *
 */

static uint8_t nrf905_channel_prev = RF_CHANNEL_NONE;
static bool nrf905_receive_active  = false;

static bool nrf905_probe()
{
  uint8_t addr[4];
  uint8_t ref[] = TXADDR;

  digitalWrite(CS_N, HIGH);
  pinMode(CS_N, OUTPUT);

  SoC->SPI_begin();

#if defined(ARDUINO) && !defined(RASPBERRY_PI)
  SPI.setClockDivider(SPI_CLOCK_DIV2);
#endif /* ARDUINO */

  digitalWrite(CS_N, LOW);

  SPI.transfer(NRF905_CMD_R_TX_ADDRESS);
  for(uint8_t i=4;i--;) {
    addr[i] = SPI.transfer(NRF905_CMD_NOP);
  }

  digitalWrite(CS_N, HIGH);
  pinMode(CS_N, INPUT);

  SPI.end();

#if 0
  delay(3000);
  Serial.print("NRF905 probe: ");
  Serial.print(addr[0], HEX); Serial.print(" ");
  Serial.print(addr[1], HEX); Serial.print(" ");
  Serial.print(addr[2], HEX); Serial.print(" ");
  Serial.print(addr[3], HEX); Serial.print(" ");
  Serial.println();
#endif

  /* Cold start state */
  if ((addr[0] == 0xE7) && (addr[1] == 0xE7) && (addr[2] == 0xE7) && (addr[3] == 0xE7)) {
    return true;
  }

  /* Warm restart state */
  if ((addr[0] == 0xE7) && (addr[1] == ref[0]) && (addr[2] == ref[1]) && (addr[3] == ref[2])) {
    return true;
  }

  return false;
}

static void nrf905_channel(uint8_t channel)
{
  if (channel != nrf905_channel_prev) {

    uint32_t frequency;
    nRF905_band_t band;

    frequency = RF_FreqPlan.getChanFrequency(channel);
    band = (frequency >= 844800000UL ? NRF905_BAND_868 : NRF905_BAND_433);

    nRF905_setFrequency(band , frequency);

    nrf905_channel_prev = channel;
    /* restart Rx upon a channel switch */
    nrf905_receive_active = false;
  }
}

static void nrf905_setup()
{
  SoC->SPI_begin();

  // Start up
  nRF905_init();

  /* Channel selection is now part of RF_loop() */
//  nrf905_channel(channel);

  //nRF905_setTransmitPower(NRF905_PWR_10);
  //nRF905_setTransmitPower(NRF905_PWR_n10);

  switch(settings->txpower)
  {
  case RF_TX_POWER_FULL:

    /*
     * NRF905 is unable to give more than 10 dBm
     * 10 dBm is legal everywhere in the world
     */

    nRF905_setTransmitPower((nRF905_pwr_t)NRF905_PWR_10);
    break;
  case RF_TX_POWER_OFF:
  case RF_TX_POWER_LOW:
  default:
    nRF905_setTransmitPower((nRF905_pwr_t)NRF905_PWR_n10);
    break;
  }

  nRF905_setCRC(NRF905_CRC_16);
  //nRF905_setCRC(NRF905_CRC_DISABLE);

  // Set address of this device
  byte addr[] = RXADDR;
  nRF905_setRXAddress(addr);

  /* Enforce radio settings to follow "Legacy" protocol's RF specs */
  if (settings->rf_protocol != RF_PROTOCOL_LATEST)
      settings->rf_protocol = RF_PROTOCOL_LEGACY;

  /* Enforce encoder and decoder to process "Legacy" frames only */
  protocol_encode = &legacy_encode;
  protocol_decode = &legacy_decode;

  /* Put IC into receive mode */
  nRF905_receive();
}

static bool nrf905_receive()
{
  bool success = false;

  // Put into receive mode
  if (!nrf905_receive_active) {
    nRF905_receive();
    nrf905_receive_active = true;
  }

  success = nRF905_getData(RxBuffer, LEGACY_PAYLOAD_SIZE);
  if (success) { // Got data
    rx_packets_counter++;
  }

  return success;
}

static void nrf905_transmit()
{
    nrf905_receive_active = false;

    // Set address of device to send to
    byte addr[] = TXADDR;
    nRF905_setTXAddress(addr);

    // Set payload data
    nRF905_setData(&TxBuffer[0], LEGACY_PAYLOAD_SIZE );

    // Send payload (send fails if other transmissions are going on, keep trying until success)
    while (!nRF905_send()) {
      yield();
    } ;
}

static void nrf905_shutdown()
{
  nRF905_powerDown();
  SPI.end();
}

#endif /* EXCLUDE_NRF905 */

#if !defined(EXCLUDE_SX12XX)
/*
 * SX12XX-specific code
 *
 *
 */

osjob_t sx12xx_txjob;
osjob_t sx12xx_timeoutjob;

static void sx12xx_tx_func (osjob_t* job);
static void sx12xx_rx_func (osjob_t* job);
static void sx12xx_rx(osjobcb_t func);

static bool sx12xx_receive_complete = false;
bool sx12xx_receive_active = false;
static bool sx12xx_transmit_complete = false;

static uint8_t sx12xx_channel_prev = RF_CHANNEL_NONE;

#if defined(USE_BASICMAC)
void os_getDevEui (u1_t* buf) { }
u1_t os_getRegion (void) { return REGCODE_EU868; }
#else
#if !defined(DISABLE_INVERT_IQ_ON_RX)
#error This example requires DISABLE_INVERT_IQ_ON_RX to be set. Update \
       config.h in the lmic library to set it.
#endif
#endif

#define SX1276_RegVersion          0x42 // common

static u1_t sx1276_readReg (u1_t addr) {
#if defined(USE_BASICMAC)
    hal_spi_select(1);
#else
    hal_pin_nss(0);
#endif
    hal_spi(addr & 0x7F);
    u1_t val = hal_spi(0x00);
#if defined(USE_BASICMAC)
    hal_spi_select(0);
#else
    hal_pin_nss(1);
#endif
    return val;
}

static bool sx1276_probe()
{
  u1_t v, v_reset;

  SoC->SPI_begin();

  hal_init_softrf (nullptr);

  // manually reset radio
  hal_pin_rst(0); // drive RST pin low
  hal_waitUntil(os_getTime()+ms2osticks(1)); // wait >100us

  v_reset = sx1276_readReg(SX1276_RegVersion);

  hal_pin_rst(2); // configure RST pin floating!
  hal_waitUntil(os_getTime()+ms2osticks(5)); // wait 5ms

  v = sx1276_readReg(SX1276_RegVersion);

  pinMode(lmic_pins.nss, INPUT);
  SPI.end();

  if (v == 0x12 || v == 0x13) {

    if (v_reset == 0x12 || v_reset == 0x13) {
      RF_SX12XX_RST_is_connected = false;
    }

    return true;
  } else {
    return false;
  }
}

#if defined(USE_BASICMAC)

#define CMD_READREGISTER		        0x1D
#define REG_LORASYNCWORDLSB	        0x0741
#define SX126X_DEF_LORASYNCWORDLSB  0x24

static void sx1262_ReadRegs (uint16_t addr, uint8_t* data, uint8_t len) {
    hal_spi_select(1);
    hal_pin_busy_wait();
    hal_spi(CMD_READREGISTER);
    hal_spi(addr >> 8);
    hal_spi(addr);
    hal_spi(0x00); // NOP
    for (uint8_t i = 0; i < len; i++) {
        data[i] = hal_spi(0x00);
    }
    hal_spi_select(0);
}

static uint8_t sx1262_ReadReg (uint16_t addr) {
    uint8_t val;
    sx1262_ReadRegs(addr, &val, 1);
    return val;
}

#if defined(USE_LR1110)
static void lr11xx_GetVersion (uint8_t* hw, uint8_t* device,
                               uint8_t* major, uint8_t* minor) {
    uint8_t buf[4] = { 0 };

    hal_pin_busy_wait();
    hal_spi_select(1);

    hal_spi((uint8_t)((RADIOLIB_LR11X0_CMD_GET_VERSION & 0xFF00) >> 8));
    hal_spi((uint8_t) (RADIOLIB_LR11X0_CMD_GET_VERSION & 0x00FF));
    hal_spi_select(0);

    hal_pin_busy_wait();
    hal_spi_select(1);

    hal_spi(RADIOLIB_LR11X0_CMD_NOP);
    for (uint8_t i = 0; i < sizeof(buf); i++) {
        buf[i] = hal_spi(0x00);
    }
    hal_spi_select(0);

    if (hw)     { *hw     = buf[0]; }
    if (device) { *device = buf[1]; }
    if (major)  { *major  = buf[2]; }
    if (minor)  { *minor  = buf[3]; }
#if 0
    Serial.print("hw     = "); Serial.println(*hw, HEX);
    Serial.print("device = "); Serial.println(*device, HEX);
    Serial.print("major  = "); Serial.println(*major, HEX);
    Serial.print("minor  = "); Serial.println(*minor, HEX);
#endif
}
#endif /* USE_LR1110 */

static bool sx1262_probe()
{
  u1_t v, v_reset;

  SoC->SPI_begin();

  hal_init_softrf (nullptr);

  // manually reset radio
  hal_pin_rst(0); // drive RST pin low
  hal_waitUntil(os_getTime()+ms2osticks(1)); // wait >100us

  v_reset = sx1262_ReadReg(REG_LORASYNCWORDLSB);

  hal_pin_rst(2); // configure RST pin floating!
  hal_waitUntil(os_getTime()+ms2osticks(5)); // wait 5ms

  v = sx1262_ReadReg(REG_LORASYNCWORDLSB);

  pinMode(lmic_pins.nss, INPUT);
  SPI.end();

  u1_t fanet_sw_lsb = ((fanet_proto_desc.syncword[0]  & 0x0F) << 4) | 0x04;
  if (v == SX126X_DEF_LORASYNCWORDLSB || v == fanet_sw_lsb) {

    if (v_reset == SX126X_DEF_LORASYNCWORDLSB || v == fanet_sw_lsb) {
      RF_SX12XX_RST_is_connected = false;
    }

    return true;
  } else {
    return false;
  }
}
#endif

static void sx12xx_channel(uint8_t channel)
{
  if (channel != sx12xx_channel_prev) {

    uint32_t frequency = RF_FreqPlan.getChanFrequency(channel);

    //Serial.print("frequency: "); Serial.println(frequency);

    if (sx12xx_receive_active) {
      os_radio(RADIO_RST);
      sx12xx_receive_active = false;
    }

    /* correction of not more than 30 kHz is allowed */
    int8_t fc = settings->freq_corr;
    //if (rf_chip->type == RF_IC_SX1276) {
      /* Most of SX1262 designs use TCXO */
      // but allow frequency correction on it anyway
    if (fc != 0) {
      if (fc > 30) {
        fc = 30;
      } else if (fc < -30) {
        fc = -30;
      };
      LMIC.freq = frequency + (fc * 1000);
    } else {
      LMIC.freq = frequency;
    }
    /* Actual RF chip's channel registers will be updated before each Tx or Rx session */

    sx12xx_channel_prev = channel;

//Serial.println("sx12xx_channel() set freq");
//  } else {
//Serial.println("sx12xx_channel() skipped setting freq");
  }
}

static uint8_t sx12xx_txpower()
{
  uint8_t power = 2;   /* 2 dBm is minimum for RFM95W on PA_BOOST pin */

  if (settings->txpower == RF_TX_POWER_FULL) {

    /* Load regional max. EIRP at first */
    power = RF_FreqPlan.MaxTxPower;

    if (rf_chip->type == RF_IC_SX1262) {
      /* SX1262 is unable to give more than 22 dBm */
      //if (LMIC.txpow > 22)
      //  LMIC.txpow = 22;
      // The sx1262 has internal protection against antenna mismatch.
      // And yet the T-Echo instructions warn against using without an antenna.
      // But keep is a bit lower ayway
      if (power > 19)
          power = 19;
    } else {
      /* SX1276 is unable to give more than 20 dBm */
      //if (LMIC.txpow > 20)
      //  LMIC.txpow = 20;
    // Most T-Beams have an sx1276, it can give 20 dBm but only safe with a good antenna.
    // Note that the regional legal limit RF_FreqPlan.MaxTxPower also applies,
    //   it is only 14 dBm in EU, but 30 in Americas, India & Australia.
    //if (hw_info.model != SOFTRF_MODEL_PRIME_MK2) {
        /*
         * Enforce Tx power limit until confirmation
         * that RFM95W is doing well
         * when antenna is not connected
         */
        if (power > 17)
            power = 17;
    //}
    }
    if (settings->relay >= RELAY_ONLY)
        power = 8;
  }

  return power;
}

static void set_lmic_protocol(uint8_t protocol)   // only used during RF_setup()
{
  switch (protocol)
  {
  case RF_PROTOCOL_ADSL:
    LMIC.protocol = &adsl_proto_desc;
    protocol_encode = &adsl_encode;
    protocol_decode = &adsl_decode;
    break;
  case RF_PROTOCOL_OGNTP:
    LMIC.protocol = &ogntp_proto_desc;
    protocol_encode = &ogntp_encode;
    protocol_decode = &ogntp_decode;
    break;
  case RF_PROTOCOL_P3I:
    LMIC.protocol = &p3i_proto_desc;
    protocol_encode = &p3i_encode;
    protocol_decode = &p3i_decode;
    break;
  case RF_PROTOCOL_FANET:
    LMIC.protocol = &fanet_proto_desc;
    protocol_encode = &fanet_encode;
    protocol_decode = &fanet_decode;
    break;
  case RF_PROTOCOL_LEGACY:
    LMIC.protocol = &legacy_proto_desc;
    protocol_encode = &legacy_encode;     // encodes both LEGACY and LATEST
    protocol_decode = &legacy_decode;     // decodes both LEGACY and LATEST
    break;
  case RF_PROTOCOL_LATEST:
  default:
    LMIC.protocol = &latest_proto_desc;
    protocol_encode = &legacy_encode;     // encodes both LEGACY and LATEST
    protocol_decode = &legacy_decode;     // decodes both LEGACY and LATEST
    break;
  }
}

static void sx12xx_resetup()
{
  // initialize runtime env
  os_init (nullptr);

  // Reset the MAC state. Session and pending data transfers will be discarded.
  // - this clears the LMIC structure, so need to save and restore the protocol.
  const rf_proto_desc_t  *p = LMIC.protocol;
  LMIC_reset();   // >>> or maybe just do:  os_radio(RADIO_RST)  - if that
  delay(1);
  LMIC.protocol = p;

  sx12xx_channel_prev = RF_CHANNEL_NONE;
       // force channel setting on next call - even if channel has not changed

  LMIC.txpow = sx12xx_txpower();
}

static void sx12xx_setup()
{
  SoC->SPI_begin();

  sx12xx_resetup();
}

static void sx12xx_setvars()
{
  if (LMIC.protocol && LMIC.protocol->modulation_type == RF_MODULATION_TYPE_LORA) {
    LMIC.datarate = LMIC.protocol->bitrate;
    LMIC.syncword = LMIC.protocol->syncword[0];
  } else {
    LMIC.datarate = DR_FSK;
  }

#if defined(USE_BASICMAC)

#define updr2rps  LMIC_updr2rps

  // LMIC.rps = MAKERPS(sf, BW250, CR_4_5, 0, 0);

  LMIC.noRXIQinversion = true;
  LMIC.rxsyms = 100;

#endif /* USE_BASICMAC */

  // This sets CR 4/5, BW125 (except for DR_SF7B, which uses BW250)
  LMIC.rps = updr2rps(LMIC.datarate);


  if (LMIC.protocol && LMIC.protocol->type == RF_PROTOCOL_FANET) {
    /* for only a few nodes around, increase the coding rate to ensure a more robust transmission */
    LMIC.rps = setCr(LMIC.rps, CR_4_8);
  }
}

static bool sx12xx_receive()
{
  bool success = false;
  sx12xx_receive_complete = false;

  //LMIC.protocol = curr_rx_protocol_ptr;  done in set_protocol_for_slot

  // this section is redundant since already done in set_protocol_for_slot
  if (flr_adsl) {
      LMIC.protocol = &flr_adsl_proto_desc;
      protocol_decode = &flr_adsl_decode;
  }

  if (!sx12xx_receive_active) {
    if (settings->power_save & POWER_SAVE_NORECEIVE) {
      LMIC_shutdown();
    } else {
      sx12xx_setvars();
      sx12xx_rx(sx12xx_rx_func);
    }
    sx12xx_receive_active = true;
  }

  if (sx12xx_receive_complete == false) {
    // execute scheduled jobs and events
    os_runstep();
  };

  if (sx12xx_receive_complete == true) {
    RF_last_rssi = LMIC.rssi;
    rx_packets_counter++;
    success = true;
  }

  return success;
}

static void sx12xx_transmit()
{
    sx12xx_transmit_complete = false;
    sx12xx_receive_active = false;

    sx12xx_setvars();
    os_setCallback(&sx12xx_txjob, sx12xx_tx_func);

    unsigned long tx_timeout = LMIC.protocol ? (LMIC.protocol->air_time + 25) : 60;
    unsigned long tx_start   = millis();

    while (sx12xx_transmit_complete == false) {

      if ((millis() - tx_start) > tx_timeout) {   // timeout code from v1.2
        os_radio(RADIO_RST);
        //Serial.println("TX timeout");
        RF_tx_size = 0;
        break;
      }

      // execute scheduled jobs and events
      os_runstep();

      yield();
    };
}

static void sx1276_shutdown()
{
  LMIC_shutdown();

  SPI.end();
}

#if defined(USE_BASICMAC)
static void sx1262_shutdown()
{
  os_init (nullptr);
  sx126x_ll_ops.radio_sleep();
  delay(1);

  SPI.end();
}
#endif /* USE_BASICMAC */

// Enable rx mode and call func when a packet is received
static void sx12xx_rx(osjobcb_t func) {
  LMIC.osjob.func = func;
  LMIC.rxtime = os_getTime(); // RX _now_
  // Enable "continuous" RX for LoRa only (e.g. without a timeout,
  // still stops after receiving a packet)
if (LMIC.protocol==NULL)
Serial.println("sx12xx_rx(): LMIC.protocol==NULL");
  os_radio(LMIC.protocol &&
           LMIC.protocol->modulation_type == RF_MODULATION_TYPE_LORA ?
          RADIO_RXON : RADIO_RX);
  //Serial.println("RX");
}

static void sx12xx_rx_func(osjob_t* job) {

  u1_t crc8, pkt_crc8;
  u2_t crc16, pkt_crc16;
  u1_t i;

//Serial.printf("sx12xx_rx_func() called, datalen=%d\r\n", LMIC.dataLen);

  // SX1276 is in SLEEP after IRQ handler, Force it to enter RX mode
  sx12xx_receive_active = false;

  /* FANET (LoRa) LMIC IRQ handler may deliver empty packets here when CRC is invalid. */
  if (LMIC.dataLen == 0) {
    sx12xx_receive_complete = false;
    return;
  }

  u1_t size = LMIC.dataLen;     // include the CRC bytes in the data copy/shift

  unsigned crc_type = LMIC.protocol->crc_type;
  if (flr_adsl) {
      // examine 2 later bytes in the sync word to identify the protocol
      // - that was 4 bytes before Manchester decoding
      if (LMIC.frame[0]==FLR_ID_BYTE_1 && LMIC.frame[1]==FLR_ID_BYTE_2) {
          RF_last_protocol = RF_PROTOCOL_LEGACY;
          crc_type = RF_CHECKSUM_TYPE_CCITT_FFFF;
      } else if (LMIC.frame[0]==ADSL_ID_BYTE_1 && LMIC.frame[1]==ADSL_ID_BYTE_2) {
          RF_last_protocol = RF_PROTOCOL_ADSL;
          crc_type = RF_CHECKSUM_TYPE_CRC_MODES;
          size -= 2;        // packet 3 bytes shorter but CRC one byte longer than Legacy
      } else {
          RF_last_protocol = RF_PROTOCOL_NONE;
          sx12xx_receive_complete = false;
//Serial.printf("Unidentified packet protocol 0x%02x 0x%02x\r\n", LMIC.frame[0], LMIC.frame[1]);
          return;
      }
  }

  if (size > sizeof(RxBuffer))
      size = sizeof(RxBuffer);

//Serial.print("size=");
//Serial.println(size);
//Serial.println(Bin2Hex((byte *) LMIC.frame, size));

  u1_t offset = LMIC.protocol->payload_offset;    // 0 for FLR & ADSL & OGNTP

  if (flr_adsl) {
      // skip the unused sync bytes
      size -= 3;
      // shift the payload bits as needed
      byte *p = &LMIC.frame[2];
      byte *q = &LMIC.frame[3];
      byte *r = &RxBuffer[0];
      for (u1_t i=0; i < size; i++) {
          *r++ = (*p << 7) | (*q >> 1);
          ++p;
          ++q;
      }
  } else {
      // single protocol, no bit-shifting needed, just copy by bytes
      size -= offset;
      for (u1_t i=0; i < size; i++) {
         RxBuffer[i] = LMIC.frame[offset+i];
      }
      RF_last_protocol = current_RX_protocol;
  }

//Serial.println(Bin2Hex((byte *) RxBuffer, size));

  // now can compute and check the CRC

  switch (crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_CRC_MODES:
  case RF_CHECKSUM_TYPE_NONE:
     /* crc16 left not initialized */
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    crc8 = 0x71;     /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_0000:
    crc16 = 0x0000;  /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:    // includes FLR packet in FLR_ADSL dual mode
  default:
    crc16 = 0xffff;  /* seed value */
    break;
  }

  if (crc_type != RF_CHECKSUM_TYPE_CRC_MODES) {  // not ADS-L packet

    switch (LMIC.protocol->type)
    {
    case RF_PROTOCOL_LEGACY:    // includes FLR_ADSL dual mode
    case RF_PROTOCOL_LATEST:
      /* take in account NRF905/FLARM "address" bytes */
      crc16 = update_crc_ccitt(crc16, 0x31);
      crc16 = update_crc_ccitt(crc16, 0xFA);
      crc16 = update_crc_ccitt(crc16, 0xB6);
      break;
    case RF_PROTOCOL_P3I:
    case RF_PROTOCOL_OGNTP:
    case RF_PROTOCOL_ADSL:
    default:
      break;
    }


    for (i = 0; i < size - LMIC.protocol->crc_size; i++)
    {

      switch (crc_type)
      {
      case RF_CHECKSUM_TYPE_GALLAGER:
      case RF_CHECKSUM_TYPE_CRC_MODES:
      case RF_CHECKSUM_TYPE_NONE:
        break;
      case RF_CHECKSUM_TYPE_CRC8_107:
        update_crc8(&crc8, (u1_t)(RxBuffer[i]));
        break;
      case RF_CHECKSUM_TYPE_CCITT_FFFF:    // includes FLR packet in FLR_ADSL dual mode
      case RF_CHECKSUM_TYPE_CCITT_0000:
      default:
        crc16 = update_crc_ccitt(crc16, (u1_t)(RxBuffer[i]));
        break;
      }

      switch (LMIC.protocol->whitening)
      {
      case RF_WHITENING_NICERF:
        RxBuffer[i] ^= pgm_read_byte(&whitening_pattern[i]);
        break;
      case RF_WHITENING_MANCHESTER:
      case RF_WHITENING_NONE:
      default:
        break;
      }

    }

  }

  switch (crc_type)
  {
  case RF_CHECKSUM_TYPE_NONE:
    sx12xx_receive_complete = true;
    break;
  case RF_CHECKSUM_TYPE_GALLAGER:
    if (LDPC_Check((uint8_t  *) RxBuffer)) {
      sx12xx_receive_complete = false;
    } else {
      sx12xx_receive_complete = true;
    }
    break;
  case RF_CHECKSUM_TYPE_CRC_MODES:    // includes ADSL packet in FLR_ADSL dual mode
    if (ADSL_Packet::checkPI((uint8_t  *) RxBuffer, size)) {
      sx12xx_receive_complete = false;
Serial.println("ADS-L CRC wrong");
    } else {
      RF_last_crc = (RxBuffer[size-3] << 16 | RxBuffer[size-2] << 8 | RxBuffer[size-1]);
      sx12xx_receive_complete = true;
    }
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    pkt_crc8 = RxBuffer[i];
    if (crc8 == pkt_crc8) {
      RF_last_crc = crc8;
      sx12xx_receive_complete = true;
    } else {
      sx12xx_receive_complete = false;
    }
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:    // includes FLR packet in FLR_ADSL dual mode
  case RF_CHECKSUM_TYPE_CCITT_0000:
  default:
    pkt_crc16 = (RxBuffer[size-2] << 8 | RxBuffer[size-1]);
    if (crc16 == pkt_crc16) {
      RF_last_crc = crc16;
      sx12xx_receive_complete = true;
    } else {
      sx12xx_receive_complete = false;
Serial.println("FLR CRC wrong");
    }
    break;
  }
}

// Transmit the given string and call the given function afterwards
static void sx12xx_tx(unsigned char *buf, size_t size, osjobcb_t func) {

  u1_t crc8;
  u2_t crc16;

  switch (LMIC.protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_NONE:
     /* crc16 left not initialized */
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    crc8 = 0x71;     /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_0000:
    crc16 = 0x0000;  /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  default:
    crc16 = 0xffff;  /* seed value */
    break;
  }
  
  os_radio(RADIO_RST); // Stop RX first
  delay(1); // Wait a bit, without this os_radio below asserts, apparently because the state hasn't changed yet

  LMIC.dataLen = 0;

  switch (LMIC.protocol->type)
  {
  case RF_PROTOCOL_LEGACY:
  case RF_PROTOCOL_LATEST:
    /* take in account NRF905/FLARM "address" bytes */
    crc16 = update_crc_ccitt(crc16, 0x31);
    crc16 = update_crc_ccitt(crc16, 0xFA);
    crc16 = update_crc_ccitt(crc16, 0xB6);
    break;
  case RF_PROTOCOL_P3I:
    /* insert Net ID */
    LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >> 24) & 0x000000FF);
    LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >> 16) & 0x000000FF);
    LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >>  8) & 0x000000FF);
    LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >>  0) & 0x000000FF);
    /* insert byte with payload size */
    LMIC.frame[LMIC.dataLen++] = LMIC.protocol->payload_size;

    /* insert byte with CRC-8 seed value when necessary */
    if (LMIC.protocol->crc_type == RF_CHECKSUM_TYPE_CRC8_107) {
      LMIC.frame[LMIC.dataLen++] = crc8;
    }

    break;
  case RF_PROTOCOL_OGNTP:
  case RF_PROTOCOL_ADSL:
  default:
    break;
  }

  for (u1_t i=0; i < size; i++) {

    switch (LMIC.protocol->whitening)
    {
    case RF_WHITENING_NICERF:
      LMIC.frame[LMIC.dataLen] = buf[i] ^ pgm_read_byte(&whitening_pattern[i]);
      break;
    case RF_WHITENING_MANCHESTER:
    case RF_WHITENING_NONE:
    default:
      LMIC.frame[LMIC.dataLen] = buf[i];
      break;
    }

    switch (LMIC.protocol->crc_type)
    {
    case RF_CHECKSUM_TYPE_GALLAGER:
    case RF_CHECKSUM_TYPE_CRC_MODES:
    case RF_CHECKSUM_TYPE_NONE:
      break;
    case RF_CHECKSUM_TYPE_CRC8_107:
      update_crc8(&crc8, (u1_t)(LMIC.frame[LMIC.dataLen]));
      break;
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    case RF_CHECKSUM_TYPE_CCITT_0000:
    default:
      crc16 = update_crc_ccitt(crc16, (u1_t)(LMIC.frame[LMIC.dataLen]));
      break;
    }

    LMIC.dataLen++;
  }

  switch (LMIC.protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_CRC_MODES:
  case RF_CHECKSUM_TYPE_NONE:
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    LMIC.frame[LMIC.dataLen++] = crc8;
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  case RF_CHECKSUM_TYPE_CCITT_0000:
  default:
    LMIC.frame[LMIC.dataLen++] = (crc16 >>  8) & 0xFF;
    LMIC.frame[LMIC.dataLen++] = (crc16      ) & 0xFF;
    break;
  }

if (LMIC.protocol==NULL)
Serial.println("sx12xx_tx(): LMIC.protocol==NULL");

  LMIC.osjob.func = func;
  os_radio(RADIO_TX);
  //Serial.println("TX");
}

static void sx12xx_txdone_func (osjob_t* job) {
  sx12xx_transmit_complete = true;
}

static void sx12xx_tx_func (osjob_t* job) {

  if (RF_tx_size > 0) {
    sx12xx_tx((unsigned char *) &TxBuffer[0], RF_tx_size, sx12xx_txdone_func);
  }
}
#endif /* EXCLUDE_SX12XX */

#if !defined(EXCLUDE_UATM)
/*
 * UATM-specific code
 *
 *
 */

#include <uat.h>

#define UAT_RINGBUF_SIZE  (sizeof(Stratux_frame_t) * 2)

static unsigned char uat_ringbuf[UAT_RINGBUF_SIZE];
static unsigned int uatbuf_head = 0;
Stratux_frame_t uatradio_frame;

const char UAT_ident[] PROGMEM = SOFTRF_IDENT;

static bool uatm_probe()
{
  bool success = false;
  uint32_t startTime;
  unsigned int uatbuf_tail;
  u1_t keylen = strlen_P(UAT_ident);
  u1_t i=0;

  /* Do not probe on itself and ESP8266 */
  if (SoC->id == SOC_CC13X0 ||
      SoC->id == SOC_CC13X2 ||
      SoC->id == SOC_ESP8266) {
    return success;
  }

  SoC->UATSerial_begin(UAT_RECEIVER_BR);

  SoC->UATModule_restart();

  startTime = millis();

  // Timeout if no valid response in 1 second
  while (millis() - startTime < 1000) {

    if (UATSerial.available() > 0) {
      unsigned char c = UATSerial.read();
#if DEBUG
      Serial.println(c, HEX);
#endif
      uat_ringbuf[uatbuf_head % UAT_RINGBUF_SIZE] = c;

      uatbuf_tail = uatbuf_head - keylen;
      uatbuf_head++;

      for (i=0; i < keylen; i++) {
        if (pgm_read_byte(&UAT_ident[i]) != uat_ringbuf[(uatbuf_tail + i) % UAT_RINGBUF_SIZE]) {
          break;
        }
      }

      if (i >= keylen) {
        success = true;
        break;
      }
    }
  }

  /* cleanup UAT data buffer */
  uatbuf_head = 0;
  memset(uat_ringbuf, 0, sizeof(uat_ringbuf));

  /* Current ESP32 Core has a bug with Serial2.end()+Serial2.begin() cycle */
  if (SoC->id != SOC_ESP32) {
    UATSerial.end();
  }

  return success;
}

static void uatm_channel(uint8_t channel)
{
  /* Nothing to do */
}

static void uatm_setup()
{
  /* Current ESP32 Core has a bug with Serial2.end()+Serial2.begin() cycle */
  if (SoC->id != SOC_ESP32) {
    SoC->UATSerial_begin(UAT_RECEIVER_BR);
  }

  init_fec();

  /* Enforce radio settings to follow UAT978 protocol's RF specs */
  settings->rf_protocol = RF_PROTOCOL_ADSB_UAT;

  protocol_encode = &uat978_encode;
  protocol_decode = &uat978_decode;

}

static bool uatm_receive()
{
  bool success = false;
  unsigned int uatbuf_tail;
  int rs_errors;

  while (UATSerial.available()) {
    unsigned char c = UATSerial.read();

    uat_ringbuf[uatbuf_head % UAT_RINGBUF_SIZE] = c;

    uatbuf_tail = uatbuf_head - sizeof(Stratux_frame_t);
    uatbuf_head++;

    if (uat_ringbuf[ uatbuf_tail      % UAT_RINGBUF_SIZE] == STRATUX_UATRADIO_MAGIC_1 &&
        uat_ringbuf[(uatbuf_tail + 1) % UAT_RINGBUF_SIZE] == STRATUX_UATRADIO_MAGIC_2 &&
        uat_ringbuf[(uatbuf_tail + 2) % UAT_RINGBUF_SIZE] == STRATUX_UATRADIO_MAGIC_3 &&
        uat_ringbuf[(uatbuf_tail + 3) % UAT_RINGBUF_SIZE] == STRATUX_UATRADIO_MAGIC_4) {

      unsigned char *pre_fec_buf = (unsigned char *) &uatradio_frame;
      for (u1_t i=0; i < sizeof(Stratux_frame_t); i++) {
          pre_fec_buf[i] = uat_ringbuf[(uatbuf_tail + i) % UAT_RINGBUF_SIZE];
      }

      int frame_type = correct_adsb_frame(uatradio_frame.data, &rs_errors);

      if (frame_type == -1) {
        continue;
      }

      u1_t size = 0;

      if (frame_type == 1) {
        size = SHORT_FRAME_DATA_BYTES;
      } else if (frame_type == 2) {
        size = LONG_FRAME_DATA_BYTES;
      }

      if (size > sizeof(RxBuffer)) {
        size = sizeof(RxBuffer);
      }

      if (size > 0) {
        memcpy(RxBuffer, uatradio_frame.data, size);

        RF_last_rssi = uatradio_frame.rssi;
        rx_packets_counter++;
        success = true;

        break;
      }
    }
  }

  return success;
}

static void uatm_transmit()
{
  /* Nothing to do */
}

static void uatm_shutdown()
{
  /* Nothing to do */
}
#endif /* EXCLUDE_UATM */
const rf_proto_desc_t  *cc13xx_protocol = &uat978_proto_desc;

#if !defined(EXCLUDE_CC13XX)
/*
 * CC13XX-specific code
 *
 *
 */

#include "EasyLink.h"

#include <uat.h>
#include <fec/char.h>
#include <fec.h>
#include <uat_decode.h>
#include <manchester.h>

#define MAX_SYNCWORD_SIZE       4



EasyLink myLink;
#if !defined(EXCLUDE_OGLEP3)
EasyLink_TxPacket txPacket;
#endif /* EXCLUDE_OGLEP3 */

static uint8_t cc13xx_channel_prev = RF_CHANNEL_NONE;

static bool cc13xx_receive_complete  = false;
static bool cc13xx_receive_active    = false;
static bool cc13xx_transmit_complete = false;

void cc13xx_Receive_callback(EasyLink_RxPacket *rxPacket_ptr, EasyLink_Status status)
{
  cc13xx_receive_active = false;
  bool success = false;

  if (status == EasyLink_Status_Success) {

    size_t size = 0;
    uint8_t offset;

    u1_t crc8, pkt_crc8;
    u2_t crc16, pkt_crc16;

#if !defined(EXCLUDE_OGLEP3)
    switch (cc13xx_protocol->crc_type)
    {
    case RF_CHECKSUM_TYPE_GALLAGER:
    case RF_CHECKSUM_TYPE_NONE:
       /* crc16 left not initialized */
      break;
    case RF_CHECKSUM_TYPE_CRC8_107:
      crc8 = 0x71;     /* seed value */
      break;
    case RF_CHECKSUM_TYPE_CCITT_0000:
      crc16 = 0x0000;  /* seed value */
      break;
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    default:
      crc16 = 0xffff;  /* seed value */
      break;
    }

    switch (cc13xx_protocol->type)
    {
    case RF_PROTOCOL_LEGACY:
    case RF_PROTOCOL_LATEST:
      /* take in account NRF905/FLARM "address" bytes */
      crc16 = update_crc_ccitt(crc16, 0x31);
      crc16 = update_crc_ccitt(crc16, 0xFA);
      crc16 = update_crc_ccitt(crc16, 0xB6);
      break;
    case RF_PROTOCOL_P3I:
    case RF_PROTOCOL_OGNTP:
    case RF_PROTOCOL_ADSL:
    default:
      break;
    }
#endif /* EXCLUDE_OGLEP3 */

    switch (cc13xx_protocol->type)
    {
#if !defined(EXCLUDE_OGLEP3)
    case RF_PROTOCOL_P3I:
      uint8_t i;
      offset = cc13xx_protocol->payload_offset;
      for (i = 0; i < cc13xx_protocol->payload_size; i++)
      {
        update_crc8(&crc8, (u1_t)(rxPacket_ptr->payload[i + offset]));
        if (i < sizeof(RxBuffer)) {
          RxBuffer[i] = rxPacket_ptr->payload[i + offset] ^
                        pgm_read_byte(&whitening_pattern[i]);
        }
      }

      pkt_crc8 = rxPacket_ptr->payload[i + offset];

      if (crc8 == pkt_crc8) {

        success = true;
      }
      break;
    case RF_PROTOCOL_LATEST:
    case RF_PROTOCOL_LEGACY:
    case RF_PROTOCOL_OGNTP:
    case RF_PROTOCOL_ADSL:
      offset = cc13xx_protocol->syncword_size - 4;
      size =  cc13xx_protocol->payload_offset +
              cc13xx_protocol->payload_size +
              cc13xx_protocol->payload_size +
              cc13xx_protocol->crc_size +
              cc13xx_protocol->crc_size;
      if (rxPacket_ptr->len >= size + offset &&
          rxPacket_ptr->payload[0] == cc13xx_protocol->syncword[4] &&
          rxPacket_ptr->payload[1] == cc13xx_protocol->syncword[5] &&
          rxPacket_ptr->payload[2] == cc13xx_protocol->syncword[6] &&
          (offset > 3 ? (rxPacket_ptr->payload[3] == cc13xx_protocol->syncword[7]) : true)) {

        uint8_t i, val1, val2;
        for (i = 0; i < size; i++) {
          val1 = pgm_read_byte(&ManchesterDecode[rxPacket_ptr->payload[i + offset]]);
          i++;
          val2 = pgm_read_byte(&ManchesterDecode[rxPacket_ptr->payload[i + offset]]);
          if ((i>>1) < sizeof(RxBuffer)) {
            RxBuffer[i>>1] = ((val1 & 0x0F) << 4) | (val2 & 0x0F);

            if (i < size - (cc13xx_protocol->crc_size + cc13xx_protocol->crc_size)) {
              switch (cc13xx_protocol->crc_type)
              {
              case RF_CHECKSUM_TYPE_GALLAGER:
              case RF_CHECKSUM_TYPE_CRC_MODES:
              case RF_CHECKSUM_TYPE_NONE:
                break;
              case RF_CHECKSUM_TYPE_CCITT_FFFF:
              case RF_CHECKSUM_TYPE_CCITT_0000:
              default:
                crc16 = update_crc_ccitt(crc16, (u1_t)(RxBuffer[i>>1]));
                break;
              }
            }
          }
        }

        switch (cc13xx_protocol->crc_type)
        {
        case RF_CHECKSUM_TYPE_GALLAGER:
          if (LDPC_Check((uint8_t  *) &RxBuffer[0]) == 0) {

            success = true;
          }
          break;
        case RF_CHECKSUM_TYPE_CCITT_FFFF:
        case RF_CHECKSUM_TYPE_CCITT_0000:
          offset = cc13xx_protocol->payload_offset + cc13xx_protocol->payload_size;
          if (offset + 1 < sizeof(RxBuffer)) {
            pkt_crc16 = (RxBuffer[offset] << 8 | RxBuffer[offset+1]);
            if (crc16 == pkt_crc16) {
              RF_last_crc = crc16;
              success = true;
            }
          }
          break;
        default:
          break;
        }
      }
      break;
#endif /* EXCLUDE_OGLEP3 */
    case RF_PROTOCOL_ADSB_UAT:
    default:
      int rs_errors;
      int frame_type;
      frame_type = correct_adsb_frame(rxPacket_ptr->payload, &rs_errors);

      if (frame_type != -1) {

        if (frame_type == 1) {
          size = SHORT_FRAME_DATA_BYTES;
        } else if (frame_type == 2) {
          size = LONG_FRAME_DATA_BYTES;
        }

        if (size > sizeof(RxBuffer)) {
          size = sizeof(RxBuffer);
        }

        if (size > 0) {
          memcpy(RxBuffer, rxPacket_ptr->payload, size);

          success = true;
        }
      }
      break;
    }

    if (success) {
      RF_last_rssi = rxPacket_ptr->rssi;
      rx_packets_counter++;

      cc13xx_receive_complete  = true;
    }
  }
}

void cc13xx_Transmit_callback(EasyLink_Status status)
{
  if (status == EasyLink_Status_Success) {
    cc13xx_transmit_complete = true;
  }
}

static bool cc13xx_Receive_Async()
{
  bool success = false;
  EasyLink_Status status;

  if (!cc13xx_receive_active) {
    status = myLink.receive(&cc13xx_Receive_callback);

    if (status == EasyLink_Status_Success) {
      cc13xx_receive_active = true;
    }
  }

  if (cc13xx_receive_complete == true) {
    success = true;
    cc13xx_receive_complete = false;
  }

  return success;
}

static bool cc13xx_probe()
{
  bool success = false;

  if (SoC->id == SOC_CC13X0 || SoC->id == SOC_CC13X2) {
    success = true;
  }

  return success;
}

static void cc13xx_channel(uint8_t channel)
{
#if !defined(EXCLUDE_OGLEP3)
  if (settings->rf_protocol != RF_PROTOCOL_ADSB_UAT &&
      channel != cc13xx_channel_prev) {
    uint32_t frequency = RF_FreqPlan.getChanFrequency(channel);

    if (cc13xx_receive_active) {
      /* restart Rx upon a channel switch */
      EasyLink_abort();
      cc13xx_receive_active = false;
    }

    EasyLink_setFrequency(frequency);

    cc13xx_channel_prev = channel;
  }
#endif /* EXCLUDE_OGLEP3 */
}

static void cc13xx_setup()
{
  switch (settings->rf_protocol)
  {
#if !defined(EXCLUDE_OGLEP3)
  case RF_PROTOCOL_ADSL:
    cc13xx_protocol = &adsl_proto_desc;
    protocol_encode = &adsl_encode;
    protocol_decode = &adsl_decode;

    myLink.begin(EasyLink_Phy_100kbps2gfsk_ogntp);         // <<< needs work
    break;
  case RF_PROTOCOL_OGNTP:
    cc13xx_protocol = &ogntp_proto_desc;
    protocol_encode = &ogntp_encode;
    protocol_decode = &ogntp_decode;

    myLink.begin(EasyLink_Phy_100kbps2gfsk_ogntp);
    break;
  case RF_PROTOCOL_P3I:
    cc13xx_protocol = &p3i_proto_desc;
    protocol_encode = &p3i_encode;
    protocol_decode = &p3i_decode;

    myLink.begin(EasyLink_Phy_38400bps2gfsk_p3i);
    break;
  case RF_PROTOCOL_LEGACY:
    cc13xx_protocol = &legacy_proto_desc;
    protocol_encode = &legacy_encode;
    protocol_decode = &legacy_decode;

    myLink.begin(EasyLink_Phy_100kbps2gfsk_legacy);
    break;
  case RF_PROTOCOL_LATEST:
    cc13xx_protocol = &latest_proto_desc;
    protocol_encode = &legacy_encode;
    protocol_decode = &legacy_decode;

    myLink.begin(EasyLink_Phy_100kbps2gfsk_legacy);
    break;
#endif /* EXCLUDE_OGLEP3 */
  case RF_PROTOCOL_ADSB_UAT:
  default:
    cc13xx_protocol = &uat978_proto_desc;
    protocol_encode = &uat978_encode;
    protocol_decode = &uat978_decode;
    /*
     * Enforce UAT protocol setting
     * if other value (FANET) left in EEPROM from other (SX12XX) radio
     */
    settings->rf_protocol = RF_PROTOCOL_ADSB_UAT;

    init_fec();
    myLink.begin(EasyLink_Phy_Custom);
    break;
  }

  /* -10 dBm is a minumum for CC1310 ; CC1352 can operate down to -20 dBm */
  int8_t TxPower = -10;

  switch(settings->txpower)
  {
  case RF_TX_POWER_FULL:

    if (settings->rf_protocol != RF_PROTOCOL_ADSB_UAT) {
      /* Load regional max. EIRP at first */
      TxPower = RF_FreqPlan.MaxTxPower;
    }

    if (TxPower > 14)
      TxPower = 14; /* 'high power' CC13XXP (up to 20 dBm) is not supported yet */

    break;
  case RF_TX_POWER_OFF:
  case RF_TX_POWER_LOW:
  default:
    break;
  }

  EasyLink_setRfPwr(TxPower);
}

static bool cc13xx_receive()
{
  return cc13xx_Receive_Async();
}

static void cc13xx_transmit()
{
#if !defined(EXCLUDE_OGLEP3)
  EasyLink_Status status;

  u1_t crc8;
  u2_t crc16;
  u1_t i;

  if (RF_tx_size == 0) {
    return;
  }

  if (cc13xx_protocol->type == RF_PROTOCOL_ADSB_UAT) {
    return; /* no transmit on UAT */
  }

  EasyLink_abort();

  cc13xx_receive_active = false;
  cc13xx_transmit_complete = false;

  size_t PayloadLen = 0;

  switch (cc13xx_protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_NONE:
     /* crc16 left not initialized */
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    crc8 = 0x71;     /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_0000:
    crc16 = 0x0000;  /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  default:
    crc16 = 0xffff;  /* seed value */
    break;
  }

  for (i = MAX_SYNCWORD_SIZE; i < cc13xx_protocol->syncword_size; i++)
  {
    txPacket.payload[PayloadLen++] = cc13xx_protocol->syncword[i];
  }

  switch (cc13xx_protocol->type)
  {
  case RF_PROTOCOL_LEGACY:
  case RF_PROTOCOL_LATEST:
    /* take in account NRF905/FLARM "address" bytes */
    crc16 = update_crc_ccitt(crc16, 0x31);
    crc16 = update_crc_ccitt(crc16, 0xFA);
    crc16 = update_crc_ccitt(crc16, 0xB6);
    break;
  case RF_PROTOCOL_P3I:
    /* insert Net ID */
    txPacket.payload[PayloadLen++] = (u1_t) ((cc13xx_protocol->net_id >> 24) & 0x000000FF);
    txPacket.payload[PayloadLen++] = (u1_t) ((cc13xx_protocol->net_id >> 16) & 0x000000FF);
    txPacket.payload[PayloadLen++] = (u1_t) ((cc13xx_protocol->net_id >>  8) & 0x000000FF);
    txPacket.payload[PayloadLen++] = (u1_t) ((cc13xx_protocol->net_id >>  0) & 0x000000FF);
    /* insert byte with payload size */
    txPacket.payload[PayloadLen++] = cc13xx_protocol->payload_size;

    /* insert byte with CRC-8 seed value when necessary */
    if (cc13xx_protocol->crc_type == RF_CHECKSUM_TYPE_CRC8_107) {
      txPacket.payload[PayloadLen++] = crc8;
    }

    break;
  case RF_PROTOCOL_OGNTP:
  case RF_PROTOCOL_ADSL:
  default:
    break;
  }

  for (i=0; i < RF_tx_size; i++) {

    switch (cc13xx_protocol->whitening)
    {
    case RF_WHITENING_NICERF:
      txPacket.payload[PayloadLen] = TxBuffer[i] ^ pgm_read_byte(&whitening_pattern[i]);
      break;
    case RF_WHITENING_MANCHESTER:
      txPacket.payload[PayloadLen] = pgm_read_byte(&ManchesterEncode[(TxBuffer[i] >> 4) & 0x0F]);
      PayloadLen++;
      txPacket.payload[PayloadLen] = pgm_read_byte(&ManchesterEncode[(TxBuffer[i]     ) & 0x0F]);
      break;
    case RF_WHITENING_NONE:
    default:
      txPacket.payload[PayloadLen] = TxBuffer[i];
      break;
    }

    switch (cc13xx_protocol->crc_type)
    {
    case RF_CHECKSUM_TYPE_GALLAGER:
    case RF_CHECKSUM_TYPE_NONE:
      break;
    case RF_CHECKSUM_TYPE_CRC8_107:
      update_crc8(&crc8, (u1_t)(txPacket.payload[PayloadLen]));
      break;
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    case RF_CHECKSUM_TYPE_CCITT_0000:
    default:
      if (cc13xx_protocol->whitening == RF_WHITENING_MANCHESTER) {
        crc16 = update_crc_ccitt(crc16, (u1_t)(TxBuffer[i]));
      } else {
        crc16 = update_crc_ccitt(crc16, (u1_t)(txPacket.payload[PayloadLen]));
      }
      break;
    }

    PayloadLen++;
  }

  switch (cc13xx_protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_NONE:
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    txPacket.payload[PayloadLen++] = crc8;
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  case RF_CHECKSUM_TYPE_CCITT_0000:
  default:
    if (cc13xx_protocol->whitening == RF_WHITENING_MANCHESTER) {
      txPacket.payload[PayloadLen++] = pgm_read_byte(&ManchesterEncode[(((crc16 >>  8) & 0xFF) >> 4) & 0x0F]);
      txPacket.payload[PayloadLen++] = pgm_read_byte(&ManchesterEncode[(((crc16 >>  8) & 0xFF)     ) & 0x0F]);
      txPacket.payload[PayloadLen++] = pgm_read_byte(&ManchesterEncode[(((crc16      ) & 0xFF) >> 4) & 0x0F]);
      txPacket.payload[PayloadLen++] = pgm_read_byte(&ManchesterEncode[(((crc16      ) & 0xFF)     ) & 0x0F]);
      PayloadLen++;
    } else {
      txPacket.payload[PayloadLen++] = (crc16 >>  8) & 0xFF;
      txPacket.payload[PayloadLen++] = (crc16      ) & 0xFF;
    }
    break;
  }

  txPacket.len = PayloadLen;
  // Transmit immediately
  txPacket.absTime = EasyLink_ms_To_RadioTime(0);

#if 0
  status = myLink.transmit(&txPacket, &cc13xx_Transmit_callback);

  if (status == EasyLink_Status_Success) {
    while (cc13xx_transmit_complete == false) {
      yield();
    };
  }
#else
  myLink.transmit(&txPacket);
#endif

#endif /* EXCLUDE_OGLEP3 */
}

static void cc13xx_shutdown()
{
  EasyLink_abort();
}
#endif /* EXCLUDE_CC13XX */

#if defined(USE_OGN_RF_DRIVER)
/*
 * OGN driver specific code
 *
 *
 */

static RFM_TRX  TRX;

static uint8_t ognrf_channel_prev  = RF_CHANNEL_NONE;
static bool ognrf_receive_active   = false;

void RFM_Select  (void)                 { hal_pin_nss(0); }
void RFM_Deselect(void)                 { hal_pin_nss(1); }
uint8_t RFM_TransferByte(uint8_t Byte)  { return hal_spi(Byte); }

bool RFM_IRQ_isOn(void)   { return lmic_pins.dio[0] == LMIC_UNUSED_PIN ? \
                                  false : digitalRead(lmic_pins.dio[0]); }

#ifdef WITH_RFM95                     // RESET is active LOW
void RFM_RESET(uint8_t On)
{ if(On) hal_pin_rst(0);
    else hal_pin_rst(1); }
#endif

#if defined(WITH_RFM69) || defined(WITH_SX1272) // RESET is active HIGH
void RFM_RESET(uint8_t On)
{ if(On) hal_pin_rst(1);
    else hal_pin_rst(0); }
#endif

static bool ognrf_probe()
{
  bool success = false;

  TRX.Select       = RFM_Select;
  TRX.Deselect     = RFM_Deselect;
  TRX.TransferByte = RFM_TransferByte;
  TRX.RESET        = RFM_RESET;

  SoC->SPI_begin();
  hal_init_softrf (nullptr);

  TRX.RESET(1);                      // RESET active
  vTaskDelay(10);                    // wait 10ms
  TRX.RESET(0);                      // RESET released
  vTaskDelay(10);                    // wait 10ms

  uint8_t ChipVersion = TRX.ReadVersion();

  pinMode(lmic_pins.nss, INPUT);
  SPI.end();

#if defined(WITH_RFM95)
  if (ChipVersion == 0x12 || ChipVersion == 0x13) success = true;
#endif /* WITH_RFM95 */
#if defined(WITH_RFM69)
  if (ChipVersion == 0x24) success = true;
#endif /* WITH_RFM69 */
#if defined(WITH_SX1272)
  if (ChipVersion == 0x22) success = true;
#endif /* WITH_SX1272 */
#if defined(WITH_SI4X32)
  if (ChipVersion == 0x06 /* 4032 */ ||
      ChipVersion == 0x08 /* 4432 */ ) success = true;
#endif /* WITH_SI4X32 */

  return success;
}

static void ognrf_channel(uint8_t channel)
{
  if (channel != ognrf_channel_prev) {

    if (ognrf_receive_active) {

      TRX.WriteMode(RF_OPMODE_STANDBY);
      vTaskDelay(1);

      /* restart Rx upon a channel switch */
      ognrf_receive_active = false;
    }

    TRX.setChannel(channel & 0x7F);

    ognrf_channel_prev = channel;
  }
}

static void ognrf_setup()
{
  uint8_t TxPower = 0;

  /* Enforce radio settings to follow OGNTP protocol's RF specs */
  settings->rf_protocol = RF_PROTOCOL_OGNTP;

  LMIC.protocol = &ogntp_proto_desc;

  protocol_encode = &ogntp_encode;
  protocol_decode = &ogntp_decode;

  TRX.Select       = RFM_Select;
  TRX.Deselect     = RFM_Deselect;
  TRX.TransferByte = RFM_TransferByte;
  TRX.DIO0_isOn    = RFM_IRQ_isOn;
  TRX.RESET        = RFM_RESET;

  SoC->SPI_begin();
  hal_init_softrf (nullptr);

  TRX.RESET(1);                      // RESET active
  vTaskDelay(10);                    // wait 10ms
  TRX.RESET(0);                      // RESET released
  vTaskDelay(10);                    // wait 10ms

  // set TRX base frequency and channel separation
  TRX.setBaseFrequency(RF_FreqPlan.BaseFreq);
  TRX.setChannelSpacing(RF_FreqPlan.ChanSepar);
  TRX.setFrequencyCorrection(0);

  TRX.Configure(0, ogntp_proto_desc.syncword);  // setup RF chip parameters and set to channel #0
  TRX.WriteMode(RF_OPMODE_STANDBY);             // set RF chip mode to STANDBY

  switch(settings->txpower)
  {
  case RF_TX_POWER_FULL:

    /* Load regional max. EIRP at first */
    TxPower = RF_FreqPlan.MaxTxPower;

    if (TxPower > 20)
      TxPower = 20;
#if 1
    if (TxPower > 17)
      TxPower = 17;
#endif

#ifdef WITH_RFM69
    TRX.WriteTxPower(TxPower, RFM69_POWER_RATING == 1 ? true : false);
#else
    TRX.WriteTxPower(TxPower);
#endif /* WITH_RFM69 */
    break;
  case RF_TX_POWER_OFF:
  case RF_TX_POWER_LOW:
  default:
    TRX.WriteTxPowerMin();
    break;
  }

  /* Leave IC in standby mode */
}

static bool ognrf_receive()
{
  bool success = false;

#if !defined(WITH_SI4X32)

  uint8_t RxRSSI = 0;
  uint8_t Err [OGNTP_PAYLOAD_SIZE + OGNTP_CRC_SIZE];

  // Put into receive mode
  if (!ognrf_receive_active) {

//    TRX.ClearIrqFlags();

    TRX.WriteSYNC(7, 7, ogntp_proto_desc.syncword); // Shorter SYNC for RX
    TRX.WriteMode(RF_OPMODE_RECEIVER);
    vTaskDelay(1);

    ognrf_receive_active = true;
  }

  if(TRX.DIO0_isOn()) {
    RxRSSI = TRX.ReadRSSI();

    TRX.ReadPacket(RxBuffer, Err);
    if (LDPC_Check((uint8_t  *) RxBuffer) == 0) {
      success = true;
    }
  }

  if (success) {
    RF_last_rssi = RxRSSI;
    rx_packets_counter++;
  }

#endif /* WITH_SI4X32 */

  return success;
}

static void ognrf_transmit()
{
  ognrf_receive_active = false;

#if defined(WITH_SI4X32)

  TRX.WritePacket((uint8_t *) &TxBuffer[0]);
  TRX.Transmit();
  vTaskDelay(6);

#else

  TRX.WriteMode(RF_OPMODE_STANDBY);
  vTaskDelay(1);

  TRX.WriteSYNC(8, 7, ogntp_proto_desc.syncword);            // Full SYNC for TX

  TRX.ClearIrqFlags();
  TRX.WritePacket((uint8_t *) &TxBuffer[0]);

  TRX.WriteMode(RF_OPMODE_TRANSMITTER);
  vTaskDelay(5);

  uint8_t Break=0;
  for(uint16_t Wait=400; Wait; Wait--)        // wait for transmission to end
  {
    uint16_t Flags=TRX.ReadIrqFlags();
    if(Flags&RF_IRQ_PacketSent) Break++;
    if(Break>=2) break;
  }

  TRX.WriteMode(RF_OPMODE_STANDBY);

#endif /* WITH_SI4X32 */
}

static void ognrf_shutdown()
{
  TRX.WriteMode(RF_OPMODE_STANDBY);
  SPI.end();

  pinMode(lmic_pins.nss, INPUT);
}

#endif /* USE_OGN_RF_DRIVER */


// The wrapper code common to all the RF chips:

// these protocols share the frequency plan and time slots
bool in_family(uint8_t protocol)
{
    if (protocol == RF_PROTOCOL_LATEST)
        return true;
    if (protocol == RF_PROTOCOL_LEGACY)
        return true;
    if (protocol == RF_PROTOCOL_OGNTP)
        return true;
    if (protocol == RF_PROTOCOL_ADSL)
        return true;
    return false;
}

uint8_t useOGNfreq(uint8_t protocol)
{
    if (protocol == RF_PROTOCOL_OGNTP)
        return 1;
    //if (protocol == RF_PROTOCOL_ADSL)
    //    return 1;
    // - switched to using FLARM frequency for ADS-L
    return 0;
}

byte RF_setup(void)
{

  if (rf_chip == NULL) {
#if !defined(USE_OGN_RF_DRIVER)
#if defined(USE_LR1110)
    /* LR1110-only configuration */
    Serial.println(F("[RF_setup] Attempting LR1110 probe..."));
    if (lr1110_ops.probe()) {
      rf_chip = &lr1110_ops;
      Serial.println(F("[RF_setup] LR1110 probe SUCCESS"));
      Serial.flush();
    } else {
      Serial.println(F("[RF_setup] LR1110 probe FAILED"));
    }
#else
    /* Standard multi-radio probe sequence */
#if !defined(EXCLUDE_SX12XX)
#if !defined(EXCLUDE_SX1276)
    if (sx1276_ops.probe()) {
      rf_chip = &sx1276_ops;
#else
    if (false) {
#endif
#if defined(USE_BASICMAC)
#if !defined(EXCLUDE_SX1276)
      SX12XX_LL = &sx127x_ll_ops;
#endif
    } else if (sx1262_ops.probe()) {
      rf_chip = &sx1262_ops;
      SX12XX_LL = &sx126x_ll_ops;
#endif /* USE_BASICMAC */
#else
    if (false) {
#endif /* EXCLUDE_SX12XX */
#if !defined(EXCLUDE_NRF905)
    } else if (nrf905_ops.probe()) {
      rf_chip = &nrf905_ops;
#endif /* EXCLUDE_NRF905 */
#if !defined(EXCLUDE_UATM)
    } else if (uatm_ops.probe()) {
      rf_chip = &uatm_ops;
#endif /* EXCLUDE_UATM */
#if !defined(EXCLUDE_CC13XX)
    } else if (cc13xx_ops.probe()) {
      rf_chip = &cc13xx_ops;
#endif /* EXCLUDE_CC13XX */
    }
    if (rf_chip && rf_chip->name) {
      Serial.print(rf_chip->name);
      Serial.println(F(" RFIC is detected."));
    } else {
      Serial.println(F("WARNING! None of supported RFICs is detected!"));
    }
#endif /* USE_LR1110 */
#else /* USE_OGN_RF_DRIVER */
    if (ognrf_ops.probe()) {
      rf_chip = &ognrf_ops;
      Serial.println(F("OGN_DRV: RFIC is detected."));
    } else {
      Serial.println(F("WARNING! RFIC is NOT detected."));
    }
#endif /* USE_OGN_RF_DRIVER */
  }

  /* "AUTO" and "UK" freqs now mapped to EU */
  if (settings->band == RF_BAND_AUTO)
      settings->band == RF_BAND_EU;
  if (settings->band == RF_BAND_UK)
      settings->band == RF_BAND_EU;
  /* Supersede EU plan with UK when PAW is selected */
    if (rf_chip                &&
#if !defined(EXCLUDE_NRF905)
        rf_chip != &nrf905_ops &&
#endif
        settings->band == RF_BAND_EU
            && (settings->rf_protocol == RF_PROTOCOL_P3I || settings->altprotocol == RF_PROTOCOL_P3I))
      settings->band == RF_BAND_UK;

  if (settings->altprotocol == settings->rf_protocol
        //|| ! in_family(settings->rf_protocol)
        //|| ! in_family(settings->altprotocol)
        || (rf_chip != &sx1276_ops && rf_chip != &sx1262_ops && rf_chip != &lr1110_ops)) {
      settings->altprotocol = RF_PROTOCOL_NONE;
  }

  const char *p = protocol_lbl(settings->rf_protocol, settings->altprotocol);
  if (*p == '?')   // not a listed combination
      settings->altprotocol = RF_PROTOCOL_NONE;

  /* Initialize protocol pointers - works for both LMIC and LR1110 */
  uint8_t alt_proto = (settings->altprotocol == RF_PROTOCOL_NONE) ? settings->rf_protocol : settings->altprotocol;
  altprotocol_ptr = get_protocol_descriptor(alt_proto);
  altprotocol_encode = get_protocol_encode_fn(altprotocol_ptr);
  altprotocol_decode = get_protocol_decode_fn(altprotocol_ptr);

  current_RX_protocol = settings->rf_protocol;
  current_TX_protocol = settings->rf_protocol;
  mainprotocol_ptr = get_protocol_descriptor(settings->rf_protocol);
  curr_rx_protocol_ptr = mainprotocol_ptr;
  curr_tx_protocol_ptr = mainprotocol_ptr;
  mainprotocol_encode = get_protocol_encode_fn(mainprotocol_ptr);
  mainprotocol_decode = get_protocol_decode_fn(mainprotocol_ptr);

  /* Also set LMIC.protocol for LMIC-based chips */
  LMIC.protocol = mainprotocol_ptr;
  protocol_encode = mainprotocol_encode;
  protocol_decode = mainprotocol_decode;

Serial.printf("Main protocol: %s(%d)\r\n", mainprotocol_ptr->name, mainprotocol_ptr->type);
Serial.printf(" Alt protocol: %s(%d)\r\n", altprotocol_ptr->name, altprotocol_ptr->type);

  RF_FreqPlan.setPlan(settings->band, current_RX_protocol);

  if (settings->rf_protocol==RF_PROTOCOL_LATEST && settings->altprotocol==RF_PROTOCOL_ADSL) {
       if (settings->flr_adsl) {     // use dual-protocol reception trick
           dual_protocol = RF_FLR_ADSL;
           Serial.println("set up FLR_ADSL rx, FLR tx + some ADSL tx");
       } else {
           Serial.println("set up FLR rx & tx + some ADSL tx");
       }
  }
  if (settings->rf_protocol==RF_PROTOCOL_ADSL && settings->altprotocol==RF_PROTOCOL_LATEST) {
       if (settings->flr_adsl) {     // use dual-protocol reception trick
           dual_protocol = RF_FLR_ADSL;
           Serial.println("set up FLR_ADSL rx, ADSL tx + some FLR tx");
       } else {
           Serial.println("set up ADSL rx & tx + some FLR tx");
       }
  }
  if (settings->rf_protocol==RF_PROTOCOL_LATEST && settings->altprotocol==RF_PROTOCOL_OGNTP) {
       if (settings->flr_adsl) {
           dual_protocol = RF_FLR_ADSL;
           Serial.println("set up FLR_ADSL rx + some OGNTP tx");
       } else {
           Serial.println("set up FLR rx & tx + some OGNTP tx");
       }
  }
  if ((settings->rf_protocol==RF_PROTOCOL_LATEST && settings->altprotocol==RF_PROTOCOL_FANET)
  ||  (settings->rf_protocol==RF_PROTOCOL_FANET  && settings->altprotocol==RF_PROTOCOL_LATEST)) {
       dual_protocol = RF_FLR_FANET;
       if (settings->flr_adsl)
           Serial.println("set up FLR_FANET time slicing, FLR_ADSL rx + some ADSL tx");
       else
           Serial.println("set up FLR_FANET time slicing");
  }
  if (settings->rf_protocol==RF_PROTOCOL_FANET && settings->altprotocol==RF_PROTOCOL_ADSL) {
       dual_protocol = RF_FLR_FANET;
       if (settings->flr_adsl)
           Serial.println("set up FANET+ADSL time slicing, FLR_ADSL rx + ADSL tx");
       else
           Serial.println("set up FANET+ADSL time slicing");
  }
  if (settings->rf_protocol==RF_PROTOCOL_FANET && settings->altprotocol==RF_PROTOCOL_OGNTP) {
       dual_protocol = RF_FLR_FANET;
       Serial.println("set up FANET_OGNTP time slicing");
       if (settings->flr_adsl)
           Serial.println("set up FANET+OGNTP time slicing, FLR_ADSL rx, some ADSL tx");
       else
           Serial.println("set up FANET+OGNTP time slicing");
  }
  if ((settings->rf_protocol==RF_PROTOCOL_LATEST && settings->altprotocol==RF_PROTOCOL_P3I)
  ||  (settings->rf_protocol==RF_PROTOCOL_P3I    && settings->altprotocol==RF_PROTOCOL_LATEST)) {
       dual_protocol = RF_FLR_P3I;
       if (settings->flr_adsl)
           Serial.println("set up FLR_FP3I time slicing, FLR_ADSL rx + some ADSL tx");
       else
           Serial.println("set up FLR_P3I time slicing");

  }
  if (settings->rf_protocol==RF_PROTOCOL_P3I && settings->altprotocol==RF_PROTOCOL_ADSL) {
       dual_protocol = RF_FLR_P3I;
       if (settings->flr_adsl)
           Serial.println("set up P3I+ADSL time slicing, FLR_ADSL rx + ADSL tx");
       else
           Serial.println("set up P3I+ADSL time slicing");
  }
  if (settings->rf_protocol==RF_PROTOCOL_P3I && settings->altprotocol==RF_PROTOCOL_OGNTP) {
       dual_protocol = RF_FLR_P3I;
       Serial.println("set up P3I_OGNTP time slicing");
       if (settings->flr_adsl)
           Serial.println("set up P3I+OGNTP time slicing, FLR_ADSL rx, some ADSL tx");
       else
           Serial.println("set up P3I+OGNTP time slicing");
  }

  if (rf_chip) {

    rf_chip->setup();

    const rf_proto_desc_t *p = mainprotocol_ptr;

    RF_timing         = p->tm_type;

    ts                = &Time_Slots;
    ts->air_time      = p->air_time;
    ts->interval_min  = p->tx_interval_min;
    ts->interval_max  = p->tx_interval_max;
    ts->interval_mid  = (p->tx_interval_max + p->tx_interval_min) / 2;
    ts->s0.begin      = p->slot0.begin;
    ts->s1.begin      = p->slot1.begin;
    ts->s0.duration   = p->slot0.end - p->slot0.begin;
    ts->s1.duration   = p->slot1.end - p->slot1.begin;

    uint16_t duration = ts->s0.duration + ts->s1.duration;
    ts->adj = duration > ts->interval_mid ? 0 : (ts->interval_mid - duration) / 2;

    return rf_chip->type;

  }

  return RF_IC_NONE;
}

void RF_chip_channel(uint8_t protocol)
{
    uint8_t OGN = useOGNfreq(protocol);
    RF_current_chan = RF_FreqPlan.getChannel((time_t)RF_time, RF_current_slot, OGN);
    if (rf_chip)
        rf_chip->channel(RF_current_chan);
}

void RF_chip_reset(uint8_t protocol)
{
Serial.printf("[RF] chip_reset called, protocol = %d\r\n", protocol);

#if defined(USE_BASICMAC)
#if !defined(EXCLUDE_SX12XX)
    if (rf_chip == &sx1276_ops || rf_chip == &sx1262_ops)
        sx12xx_resetup();
#endif
#endif
#if defined(USE_RADIOLIB)
#if defined(USE_LR1110)
    if (rf_chip == &lr1110_ops || rf_chip == &lr1110_ops)
        lr11xx_resetup();
#endif
#endif
    RF_chip_channel(protocol);
Serial.printf("reset to Prot %d at millis %d, tx ok %d - %d, gd to %d\r\n",
current_RX_protocol, millis(), TxTimeMarker, TxEndMarker, RF_OK_until);
}

/* original code, now only called for protocols other than Legacy: */
void RF_SetChannel(void)
{
  tmElements_t  tm;
  time_t        Time;
  uint8_t       Slot;
  uint32_t now_ms, pps_btime_ms, time_corr_neg;

  switch (settings->mode)
  {
  case SOFTRF_MODE_TXRX_TEST:
    Time = OurTime;
    RF_timing = RF_timing == RF_TIMING_2SLOTS_PPS_SYNC ?
                RF_TIMING_INTERVAL : RF_timing;
    break;
#if !defined(EXCLUDE_MAVLINK)
  case SOFTRF_MODE_UAV:
    Time = the_aircraft.location.gps_time_stamp / 1000000;
    RF_timing = RF_timing == RF_TIMING_2SLOTS_PPS_SYNC ?
                RF_TIMING_INTERVAL : RF_timing;
    break;
#endif /* EXCLUDE_MAVLINK */

  case SOFTRF_MODE_NORMAL:
  default:

    now_ms = millis();
    pps_btime_ms = SoC->get_PPS_TimeMarker();

    if (pps_btime_ms) {
      if (now_ms > pps_btime_ms + 1010)
        pps_btime_ms += 1000;
      uint32_t last_Commit_Time = now_ms - gnss.time.age();
      if (pps_btime_ms <= last_Commit_Time) {
        time_corr_neg = (last_Commit_Time - pps_btime_ms) % 1000;
      } else {
        time_corr_neg = 1000 - ((pps_btime_ms - last_Commit_Time) % 1000);
      }
      ref_time_ms = pps_btime_ms;
    } else {
      uint32_t last_RMC_Commit = now_ms - gnss.date.age();
      time_corr_neg = 100;
      if (gnss_chip)
          time_corr_neg = gnss_chip->rmc_ms;
      ref_time_ms = last_RMC_Commit - time_corr_neg;
    }

    int yr    = gnss.date.year();
    if( yr > 99)
        yr    = yr - 1970;
    else
        yr    += 30;
    tm.Year   = yr;
    tm.Month  = gnss.date.month();
    tm.Day    = gnss.date.day();
    tm.Hour   = gnss.time.hour();
    tm.Minute = gnss.time.minute();
    tm.Second = gnss.time.second();

//  Time = makeTime(tm) + (gnss.time.age() - time_corr_neg) / 1000;
    Time = makeTime(tm) + (gnss.time.age() + time_corr_neg) / 1000;
    OurTime = Time;

    break;
  }

  switch (RF_timing)
  {
  case RF_TIMING_2SLOTS_PPS_SYNC:
    if ((now_ms - ts->s0.tmarker) >= ts->interval_mid) {
      ts->s0.tmarker = ref_time_ms + ts->s0.begin - ts->adj;
      ts->current = 0;
    }
    if ((now_ms - ts->s1.tmarker) >= ts->interval_mid) {
      ts->s1.tmarker = ref_time_ms + ts->s1.begin;
      ts->current = 1;
    }
    Slot = ts->current;
    break;
  case RF_TIMING_INTERVAL:
  default:
    Slot = 0;
    break;
  }

  uint8_t OGN = useOGNfreq(settings->rf_protocol);
  uint8_t chan = RF_FreqPlan.getChannel(Time, Slot, OGN);

#if DEBUG
  int("Plan: "); Serial.println(RF_FreqPlan.Plan);
  Serial.print("Slot: "); Serial.println(Slot);
  Serial.print("OGN: "); Serial.println(OGN);
  Serial.print("Channel: "); Serial.println(chan);
#endif

  if (RF_ready && rf_chip) {
    rf_chip->channel(chan);
  }
}

void set_protocol_for_slot()
{
  //uint8_t prev_protocol = current_RX_protocol;

  // Transmit one packet in alt protocol once every 4 seconds:
  // In time Slot 0 for ADS-L & FLR, and in Slot 1 for OGNTP.
  // If alt protocol is OGNTP transmit in third protocol in seconds 3,11
  // This arrangement is not used for time-slicing with FANET or P3I

  bool sec_3_7_11_15 = ((RF_time & 0x03) == 0x03);
  bool sec_3_11      = ((RF_time & 0x07) == 0x03);

  if (RF_current_slot == 0) {

    if (dual_protocol == RF_FLR_FANET || dual_protocol == RF_FLR_P3I) {
        if (sec_3_11 && settings->flr_adsl) {
            curr_rx_protocol_ptr = &flr_adsl_proto_desc;
            protocol_decode = &flr_adsl_decode;     // <<< this gets re-done in receive()
            if (settings->altprotocol == RF_PROTOCOL_ADSL) {
                curr_tx_protocol_ptr = &latest_proto_desc;
                protocol_encode = &legacy_encode;
            } else {     // altprotocol is Latest or OGNTP
                curr_tx_protocol_ptr = &adsl_proto_desc;
                protocol_encode = &adsl_encode;
            }
        } else if (settings->rf_protocol != RF_PROTOCOL_FANET && settings->rf_protocol != RF_PROTOCOL_P3I) {
            curr_tx_protocol_ptr = mainprotocol_ptr;
            protocol_encode = mainprotocol_encode;
            if (settings->flr_adsl) {
                curr_rx_protocol_ptr = &flr_adsl_proto_desc;
                protocol_decode = &flr_adsl_decode;           // <<< this gets re-done in receive()
            } else {
                curr_rx_protocol_ptr = mainprotocol_ptr;
                protocol_decode = mainprotocol_decode;
            }
        } else if (settings->altprotocol != RF_PROTOCOL_FANET && settings->altprotocol != RF_PROTOCOL_P3I) {
            curr_tx_protocol_ptr = altprotocol_ptr;
            protocol_encode = altprotocol_encode;
            if (settings->flr_adsl && settings->altprotocol != RF_PROTOCOL_OGNTP) {
                curr_rx_protocol_ptr = &flr_adsl_proto_desc;
                protocol_decode = &flr_adsl_decode;           // <<< this gets re-done in receive()
            } else {
                curr_rx_protocol_ptr = altprotocol_ptr;
                protocol_decode = altprotocol_decode;
            }
        }
    } else if (sec_3_7_11_15 && settings->altprotocol != RF_PROTOCOL_NONE) {
        if (settings->altprotocol == RF_PROTOCOL_OGNTP) {
            if (sec_3_11 && settings->flr_adsl && settings->rf_protocol != RF_PROTOCOL_ADSL) {
                curr_rx_protocol_ptr = &flr_adsl_proto_desc;
                curr_tx_protocol_ptr = &adsl_proto_desc;
                protocol_decode = &flr_adsl_decode;   // <<< this gets re-done in receive()
                protocol_encode = &adsl_encode;
            } else {
                // stay in main protocol
                // - will transmit in OGNTP in Slot 1
                curr_rx_protocol_ptr = mainprotocol_ptr;
                curr_tx_protocol_ptr = mainprotocol_ptr;
                protocol_decode = mainprotocol_decode;
                protocol_encode = mainprotocol_encode;
            }
        } else {    // Latest+ADSL, or ADSL+Latest
            if (settings->flr_adsl
             && (settings->altprotocol == RF_PROTOCOL_LATEST
              || settings->altprotocol == RF_PROTOCOL_ADSL)) {
                curr_rx_protocol_ptr = &flr_adsl_proto_desc;
                protocol_decode = &flr_adsl_decode;   // <<< this gets re-done in receive()
            } else {
                curr_rx_protocol_ptr = mainprotocol_ptr;
                protocol_decode = mainprotocol_decode;
            }
            curr_tx_protocol_ptr = altprotocol_ptr;
            protocol_encode = altprotocol_encode;
        }
    } else {    // single protocol
        if (settings->flr_adsl
         && (settings->rf_protocol == RF_PROTOCOL_LATEST || settings->rf_protocol == RF_PROTOCOL_ADSL)) {
            curr_rx_protocol_ptr = &flr_adsl_proto_desc;
            protocol_decode = &flr_adsl_decode;   // <<< this gets re-done in receive()
        } else if (sec_3_7_11_15 && settings->flr_adsl && settings->rf_protocol == RF_PROTOCOL_OGNTP) {
            curr_rx_protocol_ptr = &flr_adsl_proto_desc;
            protocol_decode = &flr_adsl_decode;
        } else {
            curr_rx_protocol_ptr = mainprotocol_ptr;
            protocol_decode = mainprotocol_decode;
        }
        curr_tx_protocol_ptr = mainprotocol_ptr;
        protocol_encode = mainprotocol_encode;
    }

  } else {  // slot 1

    if (dual_protocol == RF_FLR_FANET) {
        curr_rx_protocol_ptr = &fanet_proto_desc;
        curr_tx_protocol_ptr = &fanet_proto_desc;
        protocol_decode = &fanet_decode;
        protocol_encode = &fanet_encode;
    } else if (dual_protocol == RF_FLR_P3I) {
        curr_rx_protocol_ptr = &p3i_proto_desc;
        curr_tx_protocol_ptr = &p3i_proto_desc;
        protocol_decode = &p3i_decode;
        protocol_encode = &p3i_encode;
    } else if (sec_3_7_11_15 && settings->altprotocol == RF_PROTOCOL_OGNTP) {
        if (sec_3_11 && settings->flr_adsl && settings->rf_protocol != RF_PROTOCOL_ADSL) {
            // stay in main protocol - transmitted ADSL in slot 0
            curr_rx_protocol_ptr = mainprotocol_ptr;
            curr_tx_protocol_ptr = mainprotocol_ptr;
            protocol_decode = mainprotocol_decode;
            protocol_encode = mainprotocol_encode;
        } else {
            curr_rx_protocol_ptr = mainprotocol_ptr;
            curr_tx_protocol_ptr = &ogntp_proto_desc;
            protocol_decode = mainprotocol_decode;
            protocol_encode = &ogntp_encode;
        }
    } else {
        curr_rx_protocol_ptr = mainprotocol_ptr;
        curr_tx_protocol_ptr = mainprotocol_ptr;
        protocol_decode = mainprotocol_decode;
        protocol_encode = mainprotocol_encode;
    }
  }

  current_RX_protocol = curr_rx_protocol_ptr->type;
  current_TX_protocol = curr_tx_protocol_ptr->type;
  flr_adsl = (dual_protocol == RF_FLR_ADSL  // <<< does not happen unless settings->flr_adsl
              || (settings->flr_adsl
                  && (current_RX_protocol==RF_PROTOCOL_LATEST || current_RX_protocol==RF_PROTOCOL_ADSL)));

  Serial.printf("[set_protocol_for_slot] Slot %d: RX=%s(%d), TX=%s(%d), ptr_equal=%s\r\n",
    RF_current_slot,
    curr_rx_protocol_ptr ? curr_rx_protocol_ptr->name : "NULL", current_RX_protocol,
    curr_tx_protocol_ptr ? curr_tx_protocol_ptr->name : "NULL", current_TX_protocol,
    (lr11xx_current_protocol_ptr == curr_rx_protocol_ptr) ? "YES" : "NO");

  //if (current_RX_protocol != prev_protocol)
  //    RF_FreqPlan.setPlan(settings->band, current_RX_protocol);

  /* Protocol switching */
#if defined(USE_LR1110)
  if (rf_chip && rf_chip->type == RF_IC_LR1110) {
    /* LR1110 uses RadioLib, not LMIC */
    if (lr11xx_current_protocol_ptr != curr_rx_protocol_ptr) {
      /* Protocol has changed, need full reconfiguration */
      // Serial.print("[Protocol Switch] LR1110: ");
      // Serial.print(lr11xx_current_protocol_ptr ? lr11xx_current_protocol_ptr->name : "NULL");
      // Serial.print(" -> ");
      // Serial.println(curr_rx_protocol_ptr ? curr_rx_protocol_ptr->name : "NULL");
      RF_FreqPlan.setPlan(settings->band, current_RX_protocol);
      lr11xx_current_protocol_ptr = curr_rx_protocol_ptr;
      lr11xx_resetup();
    } else {
      /* Protocol unchanged, just update frequency/channel */
      RF_chip_channel(current_RX_protocol);
    }
  } else
#endif
  {
    /* LMIC-based chips (SX1276, SX1262, etc.) */
    if (LMIC.protocol != curr_rx_protocol_ptr) {
        Serial.print("[Protocol Switch] LMIC: ");
        Serial.print(LMIC.protocol ? LMIC.protocol->name : "NULL");
        Serial.print(" -> ");
        Serial.println(curr_rx_protocol_ptr ? curr_rx_protocol_ptr->name : "NULL");
        RF_FreqPlan.setPlan(settings->band, current_RX_protocol);
        LMIC.protocol = curr_rx_protocol_ptr;    // tx will switch to curr_tx_protocol_ptr
        RF_chip_reset(current_RX_protocol);
    } else {
        RF_chip_channel(current_RX_protocol);
    }
  }

  if (dual_protocol == RF_SINGLE_PROTOCOL && current_TX_protocol != settings->rf_protocol) {
      Serial.print("set up to tx one time slot in protocol ");
      Serial.print(current_TX_protocol);
      Serial.print(" rather than ");
      Serial.println(settings->rf_protocol);
  }

}

void RF_loop()
{
  if (!RF_ready) {
    if (RF_FreqPlan.Plan == RF_BAND_AUTO) {   // never happens, since setup() overrode AUTO
      if (ThisAircraft.latitude || ThisAircraft.longitude) {
        settings->band = RF_FreqPlan.calcPlan((int32_t)(ThisAircraft.latitude  * 600000),
                                              (int32_t)(ThisAircraft.longitude * 600000));
        RF_FreqPlan.setPlan(settings->band, current_RX_protocol);
        RF_ready = true;
      }
    } else {
      RF_ready = true;
    }
  }

//  if (dual_protocol == RF_SINGLE_PROTOCOL
//   && current_RX_protocol == settings->rf_protocol
//   && in_family(settings->rf_protocol) == false) {

  if (settings->altprotocol == RF_PROTOCOL_NONE
   && in_family(settings->rf_protocol) == false) {
      RF_SetChannel();    /* use original code */
      return;
  }

  /* Experimental code by Moshe Braner, specific to Legacy Protocol (and related protocols) */
  /* More correct on frequency hopping & time slots, and uses less CPU time */
  /* - requires OurTime to be set to UTC time in seconds - can do in Time_loop() */
  /* - also needs time since PPS, it is stored in ref_time_ms */

  if (ref_time_ms == 0)    /* no GNSS time yet */
    return;

  uint32_t now_ms = millis();

  // no need to adjust RF_time until setting up a new Time Slot
  if (now_ms < RF_OK_until)      /* channel already computed */
      return;

  RF_time = OurTime;      // OurTime may have been updated in Time_loop since last RF_loop

  if (now_ms < ref_time_ms) {   /* should not happen */
    --OurTime;
    ref_time_ms -= 1000;
    return;
  }

  uint32_t ms_since_pps = now_ms - ref_time_ms;
  if (ms_since_pps >= 1300) {   // should not happen since Time_loop takes care of this
    ++OurTime;
    ++RF_time;
    ref_time_ms += 1000;
    ms_since_pps -= 1000;
  }

  uint32_t slot_base_ms = ref_time_ms;
  if (ms_since_pps < 300) {  /* does not happen often? */
    /* channel does _NOT_ change at PPS rollover in middle of slot 1 */
    /* - therefore change the reference second to the previous one: */
    --RF_time;
    slot_base_ms -= 1000;
    ms_since_pps += 1000;
  }

  bool sec_15 = ((RF_time & 0x0F) == 0x0F);

  if (ms_since_pps >= 380 && ms_since_pps < 800) {

    if (RF_current_slot != 0) {
      RF_current_slot = 0;
      set_protocol_for_slot();
    }
    RF_OK_from  = slot_base_ms + 405;
    RF_OK_until = slot_base_ms + 800;
    TxEndMarker = slot_base_ms + 795;
    if (relay_next) {
        TxTimeMarker = TxEndMarker;     // prevent transmission (relay bypasses this)
        relay_next = false;
    } else if (current_TX_protocol == RF_PROTOCOL_ADSL) {  // ADS-L slot starts at 450
        TxTimeMarker = slot_base_ms + 455 + SoC->random(0, 335);
    } else {
        TxTimeMarker = slot_base_ms + 405 + SoC->random(0, 385);
    }

  } else if (ms_since_pps >= 800 && ms_since_pps < 1300) {

    if (RF_current_slot != 1) {
      RF_current_slot = 1;
      set_protocol_for_slot();
    }
    /* channel does _NOT_ change at PPS rollover in middle of slot 1 */
    RF_OK_from  = slot_base_ms + 805;
    RF_OK_until = slot_base_ms + 1380;
    if (dual_protocol == RF_FLR_FANET || dual_protocol == RF_FLR_P3I) {
        TxTimeMarker = RF_OK_until;          /* transmit only in FANET/P3I */
        TxEndMarker = slot_base_ms + 1370;
        if (TxTimeMarker2 == 0) {   // tx happened in previous Slot 1
            uint32_t interval;
            if (dual_protocol == RF_FLR_FANET)
                interval = 3805;
            else              // RF_FLR_P3I
                interval = 1805;
            uint32_t when = SoC->random(0, (565+565));
            if (when < 565)       // 50% chance
                when += 1000;     // tx any time in slot 1, 3 (or 4) seconds in the future
            else if (when > 848)  // 25% chance
                when -= 565;      // tx in second half of slot 1 but a second earlier
            else
                when += (2000 - 565);   // first half of slot 1, a second later
            TxTimeMarker2 = slot_base_ms + interval + when;
            // average interval 3 sec for P3I and 5 sec for FANET
        }
    } else if (current_TX_protocol == RF_PROTOCOL_ADSL) {
        // For ADS-L the official time slot ends at 1000,
        // and also not supposed to transmit fix more than 500 ms old:
        // Only transmit ADS-L in Slot 0, except for relaying:
        TxEndMarker = slot_base_ms + 995;
        TxTimeMarker = TxEndMarker;         // prevent transmission (relay bypasses this)
    } else if (sec_15 && current_TX_protocol == RF_PROTOCOL_LATEST) {
        // Some other receivers may mis-decrypt packets sent after the next PPS
        // so limit the transmissions to the pre-PPS half of the slot.
        TxEndMarker = slot_base_ms + 995;
        //TxTimeMarker = slot_base_ms + 805 + SoC->random(0, 185);
        TxTimeMarker = slot_base_ms + 805 + SoC->random(0, 385);
        if (TxTimeMarker > TxEndMarker)     // in 50% of the cases no tx in slot 1 in sec 15:
            TxTimeMarker = TxEndMarker;     // prevent transmission (relay bypasses this)
    } else {   // Legacy, OGNTP
        TxEndMarker = slot_base_ms + 1195;
        TxTimeMarker = slot_base_ms + 805 + SoC->random(0, 385);
    }

  } else { /* now 300-380 ms and somehow have not set up Slot 1 */

//Serial.println("<380 ms and somehow...");
    RF_current_slot = 1;
    set_protocol_for_slot();
    RF_OK_until = slot_base_ms + 380;
    RF_OK_from   = RF_OK_until;
    TxTimeMarker = RF_OK_until;          /* do not transmit for now */
    TxEndMarker  = RF_OK_until;
    return;
  }

  if (alt_relay_next) {                   // only happens when altprotocol == RF_PROTOCOL_OGNTP
      if (sec_15) {                       // second 15, slot 1
          TxTimeMarker = TxEndMarker;     // prevent transmission (relay bypasses this)
          alt_relay_next = false;
      }
  }

/*
  if (settings->debug_flags & DEBUG_DEEPER) {
      Serial.print("New freq at: ");
      Serial.print(now_ms - slot_base_ms);                   
      Serial.print(" ms after PPS, OK until ");
      Serial.println(RF_OK_until - slot_base_ms);
  }
*/
Serial.printf("Prot %d/%d, Slot %d set for sec %d at PPS+%d ms, PPS %d, tx ok %d - %d, gd to %d\r\n",
current_RX_protocol, current_TX_protocol, RF_current_slot, (RF_time & 0x0F), ms_since_pps, slot_base_ms,
   (current_TX_protocol==RF_PROTOCOL_FANET? TxTimeMarker2 : TxTimeMarker), TxEndMarker, RF_OK_until);
}

bool RF_Transmit_Happened()
{
    if (dual_protocol == RF_FLR_FANET && current_TX_protocol == RF_PROTOCOL_FANET)
        return (TxTimeMarker2 == 0);
    if (dual_protocol == RF_FLR_P3I && current_TX_protocol == RF_PROTOCOL_P3I)
        return (TxTimeMarker2 == 0);
    if (! TxEndMarker)
        return (TxTimeMarker > millis());   // for protocols handled by the original code
    return (TxTimeMarker >= RF_OK_until);
}

bool RF_Transmit_Ready(bool wait)
{
    //if (RF_Transmit_Happened())
    //    return false;
    uint32_t now_ms = millis();
    if ((dual_protocol == RF_FLR_FANET && current_TX_protocol == RF_PROTOCOL_FANET)
    ||  (dual_protocol == RF_FLR_P3I   && current_TX_protocol == RF_PROTOCOL_P3I))
        return (TxTimeMarker2 != 0 && now_ms > TxTimeMarker2 && now_ms < TxEndMarker);
    if (! TxEndMarker)                     // for protocols handled by the original code
        return (now_ms > TxTimeMarker);
    return (now_ms >= (wait? TxTimeMarker : RF_OK_from) && now_ms < TxEndMarker);
}

size_t RF_Encode(container_t *fop, bool wait)
{
  if (settings->txpower == RF_TX_POWER_OFF)
      return 0;

  if (! RF_ready)
      return 0;

  size_t size = 0;

  if (protocol_encode) {

    /* sanity checks: don't send bad data */
    const char *p = NULL;
    if (fop->latitude == 0.0)
        p = "position";
    else if (fop->altitude > 30000.0)   // meters
        p = "altitude";
    else if (fop->speed > (300.0 / _GPS_MPS_PER_KNOT))   // 300 m/s or about 600 knots
        p = "speed";
    else if (fabs(fop->vs) > (20.0 * (_GPS_FEET_PER_METER * 60.0))
             && fop->aircraft_type != AIRCRAFT_TYPE_JET && fop->aircraft_type != AIRCRAFT_TYPE_UFO)
        p = "vs";
    else if (fabs(fop->turnrate) > 100.0)
        p = "turnrate";
    if (p) {
        Serial.print("skipping sending bad ");
        Serial.println(p);
        return 0;
    }

    // encode in the current tx protocol
    if (RF_Transmit_Ready(wait)) 
        size = (*protocol_encode)((void *) &TxBuffer[0], fop);

  }

  return size;
}

bool RF_Transmit(size_t size, bool wait)   // only called with no-wait for air-relay
{
  if (RF_ready && rf_chip && (size > 0)) {

    RF_tx_size = size;

    if (in_family(current_TX_protocol)
     || current_TX_protocol != settings->rf_protocol
     || dual_protocol != RF_SINGLE_PROTOCOL) {

      /* Experimental code by Moshe Braner */

      if (RF_Transmit_Ready(wait)) {

        if (curr_tx_protocol_ptr != curr_rx_protocol_ptr) {
            // (otherwise LMIC.protocol was set in set_protocol_for_slot())
            RF_FreqPlan.setPlan(settings->band, current_TX_protocol);
            LMIC.protocol = curr_tx_protocol_ptr;
            RF_chip_reset(current_TX_protocol);
//Serial.println("transmitting in altprotocol...");
        }

        if (settings->txpower != RF_TX_POWER_OFF)
            rf_chip->transmit();

// Serial.printf("TX in protocol %d size=%d\r\n", current_TX_protocol, RF_tx_size);

        if (RF_tx_size == 0)   // tx timed out
            return false;
        RF_tx_size = 0;

//Serial.printf("TX at millis %d\r\n", millis());

        if (settings->txpower != RF_TX_POWER_OFF)
            tx_packets_counter++;

        if (dual_protocol == RF_FLR_FANET) {
if (current_TX_protocol == RF_PROTOCOL_FANET)
Serial.println("setting TxTimeMarker2 to 0");
            if (current_TX_protocol == RF_PROTOCOL_FANET)
                TxTimeMarker2 = 0;
            else
                TxTimeMarker = RF_OK_until;
        } else if (dual_protocol == RF_FLR_P3I) {
            if (current_TX_protocol == RF_PROTOCOL_P3I)
                TxTimeMarker2 = 0;
            else
                TxTimeMarker = RF_OK_until;
        } else {
            TxTimeMarker = RF_OK_until;  // do not transmit again (even relay) until next slot
            /* do not set next transmit time here - it is done in RF_loop() */
            if (curr_tx_protocol_ptr != curr_rx_protocol_ptr) {
                // go back to rx mode ASAP
                //delay(20);    // <<< can this delay be safely shortened?
                delay(10);      // maybe even less, if "sx12xx_transmit_complete" means what it says?
                RF_FreqPlan.setPlan(settings->band, current_RX_protocol);
                LMIC.protocol = curr_rx_protocol_ptr;
                RF_chip_reset(current_RX_protocol);     // <<< may want to condition this?
                //Serial.println("returned to normal protocol...");
            }
        }

//Serial.println(">");
//Serial.printf("> tx at %d s + %d ms\r\n", OurTime, millis()-ref_time_ms);
#if 0
if ((settings->debug_flags & (DEBUG_DEEPER | DEBUG_LEGACY)) == (DEBUG_DEEPER | DEBUG_LEGACY)) {
uint32_t ms = millis();
if (ms < ref_time_ms)  ms = 0;
else   ms -= ref_time_ms;
if (ms > 999)  ms = 999;
Serial.printf("> tx %d s %3d ms (%02d:%02d) timebits %2d chan %2d\r\n",
OurTime, ms, (int)gnss.time.minute(), (int)gnss.time.second(), (RF_time & 0x0F), RF_current_chan);
}
#endif
        return true;
      }

      return false;
    }

    /* original code for other protocols: */

    if (!wait || millis() > TxTimeMarker) {

      time_t timestamp = OurTime;

      if (settings->txpower != RF_TX_POWER_OFF) {
        rf_chip->transmit();
        tx_packets_counter++;
      }

      if (settings->nmea_p) {
        StdOut.print(F("$PSRFO,"));
        StdOut.print((uint32_t) timestamp);
        StdOut.print(F(","));
        StdOut.println(Bin2Hex((byte *) &TxBuffer[0],
                               RF_Payload_Size(current_TX_protocol)));
      }
      RF_tx_size = 0;

      Slot_descr_t *next;
      uint32_t adj;

      switch (RF_timing)
      {
      case RF_TIMING_2SLOTS_PPS_SYNC:
        next = RF_FreqPlan.Channels == 1 ? &(ts->s0) :
               ts->current          == 1 ? &(ts->s0) : &(ts->s1);
        adj  = ts->current ? ts->adj   : 0;
        TxTimeMarker = next->tmarker    +
                       ts->interval_mid +
                       SoC->random(adj, next->duration - ts->air_time);
        break;
      case RF_TIMING_INTERVAL:
      default:
        TxTimeMarker = millis() + SoC->random(ts->interval_min, ts->interval_max) - ts->air_time;
        break;
      }

      return true;
    }
  }
  return false;
}

bool RF_Receive(void)
{
  bool rval = false;

  if (RF_ready && rf_chip) {
    rval = rf_chip->receive();
  }

//if (rval)
//Serial.printf("rx at %d s + %d ms\r\n", OurTime, millis()-ref_time_ms);

  return rval;
}

void RF_Shutdown(void)
{
  if (rf_chip) {
    rf_chip->shutdown();
  }
}

#if defined(USE_LR1110)
/*
 * LR1110 radio driver implementation
 * Semtech LR1110 LoRa/GFSK transceiver
 */

/* LR11XX transceiver state variables */
// static Module *lr11xx_mod = nullptr;
// static LR1110 *lr11xx_radio = nullptr;

Module *mod;
LR11x0  *lr11xx_radio;
static int8_t lr11xx_channel_prev    = (int8_t) -1;

static volatile bool lr11xx_receive_complete = false;
static bool lr11xx_receive_active = false;
static bool lr11xx_transmit_complete = false;
static uint32_t lr11xx_last_check = 0;

/* Packet buffer for ISR */
static uint8_t lr11xx_isr_pkt_data[MAX_PKT_SIZE];
static volatile size_t lr11xx_isr_pkt_len = 0;

// static uint64_t lr11xx_eui_be = 0xdeadbeefdeadbeef;

/* setDioAsRfSwitch(0x0f, 0x0, 0x09, 0x0B, 0x0A, 0x0, 0x4, 0x0) */
static const uint32_t rfswitch_dio_pins_seeed[] = {
    RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6,
    RADIOLIB_LR11X0_DIO7, RADIOLIB_LR11X0_DIO8,
    RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table_seeed[] = {
    // mode                  DIO5  DIO6  DIO7  DIO8
    { LR11x0::MODE_STBY,   { LOW,  LOW,  LOW,  LOW  } },
    // SKY13373
    { LR11x0::MODE_RX,     { HIGH, LOW,  LOW,  HIGH } },
    { LR11x0::MODE_TX,     { HIGH, HIGH, LOW,  HIGH } },
    { LR11x0::MODE_TX_HP,  { LOW,  HIGH, LOW,  HIGH } },
    { LR11x0::MODE_TX_HF,  { LOW,  LOW,  LOW,  LOW  } },
    // BGA524N6
    { LR11x0::MODE_GNSS,   { LOW,  LOW,  HIGH, LOW  } },
    // LC
    { LR11x0::MODE_WIFI,   { LOW,  LOW,  LOW,  LOW  } },
    END_OF_MODE_TABLE,
};

static const uint32_t rfswitch_dio_pins_elecrow[] = {
    RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6,
    RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table_elecrow[] = {
    // mode                  DIO5  DIO6
    { LR11x0::MODE_STBY,   { LOW,  LOW  } },
    { LR11x0::MODE_RX,     { HIGH, LOW  } },
    { LR11x0::MODE_TX,     { HIGH, HIGH } },
    { LR11x0::MODE_TX_HP,  { LOW,  HIGH } },
    { LR11x0::MODE_TX_HF,  { LOW,  LOW  } },
    { LR11x0::MODE_GNSS,   { LOW,  LOW  } },
    { LR11x0::MODE_WIFI,   { LOW,  LOW  } },
    END_OF_MODE_TABLE,
};

void lr11xx_receive_handler(void) {
  /* ISR - keep it minimal, no Serial calls */
  lr11xx_receive_complete = true;
}

static bool lr11xx_probe()
{

  u1_t device, device_reset;
  u1_t hw, major, minor;

  SoC->SPI_begin();

  hal_init_softrf (nullptr);

  // manually reset radio
  hal_pin_rst(0); // drive RST pin low
  hal_waitUntil(os_getTime()+ms2osticks(1)); // wait >100us

  lr11xx_GetVersion(&hw, &device_reset, &major, &minor);

  hal_pin_rst(2); // configure RST pin floating!
  hal_waitUntil(os_getTime()+ms2osticks(300)); // wait 300 ms

  lr11xx_GetVersion(&hw, &device, &major, &minor);

  pinMode(lmic_pins.nss, INPUT);
  RadioSPI.end();

  if (device == RADIOLIB_LR11X0_DEVICE_LR1110) {

    if (device_reset == RADIOLIB_LR11X0_DEVICE_LR1110) {
      RF_SX12XX_RST_is_connected = false;
    }
#if 1
    char buf[8];
    snprintf(buf, sizeof(buf), "%d.%d", major, minor);
    Serial.print("INFO: LR1110 base FW version ");
    Serial.println(buf);
#endif
    return true;
  } else {
    return false;
  }

}

static void lr11xx_channel(uint8_t channel)
{
  if (channel != -1 && channel != lr11xx_channel_prev) {
    uint32_t frequency = RF_FreqPlan.getChanFrequency((uint8_t) channel);
    int8_t fc = settings->freq_corr;

    if (settings->rf_protocol == RF_PROTOCOL_LEGACY) {
      nRF905_band_t nrf_band;
      uint32_t nrf_freq_resolution;

      nrf_band = (frequency >= 844800000UL ? NRF905_BAND_868 : NRF905_BAND_433);
      nrf_freq_resolution = (nrf_band == NRF905_BAND_433 ? 100000UL : 200000UL);
      frequency -= (frequency % nrf_freq_resolution);
    }

    if (fc > 30) {
      fc = 30;
    } else if (fc < -30) {
      fc = -30;
    };

    int state = lr11xx_radio->setFrequency((frequency + (fc * 1000)) / 1000000.0);

#if RADIOLIB_DEBUG_BASIC
    if (state == RADIOLIB_ERR_INVALID_FREQUENCY) {
      Serial.println(F("[LR11XX] Selected frequency is invalid for this module!"));
      while (true) { delay(10); }
    } esle {
      Serial.print("[LR11XX] Selected frequency: ");
      Serial.print(frequency / 1000000.0);
    }
#endif

    lr11xx_channel_prev = channel;
    /* restart Rx upon a channel switch */
    lr11xx_receive_active = false;
  }
}

static void lr11xx_setup()
{
  /* Initialize SPI and LR1110 module */
  int state;
  SoC->SPI_begin();

  /* Ensure module is created */
  if (mod == nullptr) {
    Serial.println(lmic_pins.dio[0]);
    Serial.println(lmic_pins.busy);
    uint32_t irq = lmic_pins.dio[0] == LMIC_UNUSED_PIN ?
                   RADIOLIB_NC : lmic_pins.dio[0];
    uint32_t busy = lmic_pins.busy == LMIC_UNUSED_PIN ?
                    RADIOLIB_NC : lmic_pins.busy;

    mod = new Module(
      lmic_pins.nss,
      irq,
      lmic_pins.rst,
      busy,
      RadioSPI
    );
  }

  /* Ensure radio instance is created */
  if (lr11xx_radio == nullptr) {
    lr11xx_radio = new LR1110(mod);
  }

  Serial.print(F("[LR1110] Configuring protocol: "));
  Serial.println(current_RX_protocol);

  /* Set protocol-specific parameters */
  switch (current_RX_protocol) {
    case RF_PROTOCOL_OGNTP:
      Serial.println(F("[LR1110] Protocol: OGNTP"));
      curr_rx_protocol_ptr = &ogntp_proto_desc;
      protocol_encode = &ogntp_encode;
      protocol_decode = &ogntp_decode;
      break;
    case RF_PROTOCOL_P3I:
      Serial.println(F("[LR1110] Protocol: P3I"));
      curr_rx_protocol_ptr = &p3i_proto_desc;
      protocol_encode = &p3i_encode;
      protocol_decode = &p3i_decode;
      break;
    case RF_PROTOCOL_FANET:
      Serial.println(F("[LR1110] Protocol: FANET"));
      curr_rx_protocol_ptr = &fanet_proto_desc;
      protocol_encode = &fanet_encode;
      protocol_decode = &fanet_decode;
      break;
    case RF_PROTOCOL_ADSL:
      Serial.println(F("[LR1110] Protocol: ADSL"));
      curr_rx_protocol_ptr = &adsl_proto_desc;
      protocol_encode = &adsl_encode;
      protocol_decode = &adsl_decode;
      break;
    case RF_PROTOCOL_LEGACY:
      Serial.println(F("[LR1110] Protocol: LEGACY"));
      curr_rx_protocol_ptr = &legacy_proto_desc;
      protocol_encode = &legacy_encode;
      protocol_decode = &legacy_decode;
      settings->rf_protocol = RF_PROTOCOL_LEGACY;
      break;
    case RF_PROTOCOL_LATEST:
    default:
      Serial.println(F("[LR1110] Protocol: LATEST"));
      curr_rx_protocol_ptr = &legacy_proto_desc;
      protocol_encode = &legacy_encode;
      protocol_decode = &legacy_decode;
      settings->rf_protocol = RF_PROTOCOL_LATEST;
      break;
  }

  if (dual_protocol == RF_FLR_ADSL) {
    protocol_decode = &flr_adsl_decode;
  }



  RF_FreqPlan.setPlan(settings->band, settings->rf_protocol);

  /* Configure TCXO voltage based on hardware model */
  float Vtcxo = 1.6;  /* Default for Seeed T1000E */
  if (hw_info.model == SOFTRF_MODEL_POCKET) {
    Vtcxo = 3.3;  /* Elecrow TN-M3 */
  }
  Serial.print(F("[LR1110] TCXO Voltage: "));
  Serial.println(Vtcxo);

  uint32_t frequency = RF_FreqPlan.getChanFrequency(0);
  bool high = (frequency > 1000000000); /* above 1GHz */

  /* Initialize based on modulation type from protocol descriptor */
  Serial.print(F("[LR1110] Modulation type: "));
  Serial.println(curr_rx_protocol_ptr->modulation_type == RF_MODULATION_TYPE_LORA ? F("LoRa") : F("2-FSK"));

  float br, fdev, bw;
  switch (curr_rx_protocol_ptr->modulation_type) 
  {
  case RF_MODULATION_TYPE_LORA:
    /* Initialize LoRa mode */
    Serial.println(F("[LR1110] Initializing LoRa mode..."));

    /* Switch to LoRa packet type */
    state = lr11xx_radio->begin(125.0, 9, 7,
                                 RADIOLIB_LR11X0_LORA_SYNC_WORD_PRIVATE,
                                 8, Vtcxo);
#if 1
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("success!"));
    } else {
      Serial.print(F("failed, code "));
      Serial.println((int16_t) state);
      while (true) { delay(10); }
    }
#endif
    /* Get bandwidth from frequency plan */
    float bw;
    switch (RF_FreqPlan.Bandwidth)
    {
    case RF_RX_BANDWIDTH_SS_62KHZ:
      bw = high ? 203.125 : 125.0; /* BW_125 */
      break;
    case RF_RX_BANDWIDTH_SS_250KHZ:
      bw = high ? 812.5   : 500.0; /* BW_500 */
      break;
    case RF_RX_BANDWIDTH_SS_125KHZ:
    default:
      bw = high ? 406.25  : 250.0; /* BW_250 */
      break;
    }

    state = lr11xx_radio->setBandwidth(bw, high);
#if RADIOLIB_DEBUG_BASIC
    if (state == RADIOLIB_ERR_INVALID_BANDWIDTH) {
      Serial.println(F("[LR11XX] Selected bandwidth is invalid for this module!"));
      while (true) { delay(10); }
    } else {
      Serial.print(F("[LR11XX] setBandwidth("));
      Serial.print(bw);
      Serial.println((int16_t)state);
    }
#endif
    /* Configure LoRa modulation parameters from protocol descriptor */
    state = lr11xx_radio->setSpreadingFactor(7);
#if RADIOLIB_DEBUG_BASIC
    Serial.print(F("[LR11XX] setSpreadingFactor(7): "));
    Serial.println((int16_t)state);
#endif
    state = lr11xx_radio->setCodingRate(5);
    Serial.print(F("[LR11XX] setCodingRate(5): "));
    Serial.println((int16_t)state);

    state = lr11xx_radio->setSyncWord((uint8_t) curr_rx_protocol_ptr->syncword[0]);
    Serial.print(F("[LR11XX] setSyncWord(0x"));
    Serial.print(curr_rx_protocol_ptr->syncword[0], HEX);
    Serial.print(F("): "));
    Serial.println((int16_t)state);

    state = lr11xx_radio->setPreambleLength(8);

    state = lr11xx_radio->explicitHeader();
    state = lr11xx_radio->setCRC(true);
    break;

    //FSK Configuration
  case RF_MODULATION_TYPE_2FSK:
  case RF_MODULATION_TYPE_PPM: /* TBD */
  default:
    /* Initialize FSK mode */
    Serial.println(F("[LR11XX] Initializing FSK mode..."));
    state = lr11xx_radio->beginGFSK(4.8, 5.0, 156.2, 16, Vtcxo);
    Serial.print(F("[LR11XX] FSK begin(): "));
    Serial.println(state);

    switch (curr_rx_protocol_ptr->bitrate)
    {
    case RF_BITRATE_38400:
      br = high ? 125.0 :  38.4; /* SX128x minimum is 125 kbps */
      break;
    case RF_BITRATE_100KBPS:
    default:
      br = high ? 125.0 : 100.0; /* SX128x minimum is 125 kbps */
      break;
    }
    state = lr11xx_radio->setBitRate(br);
#if RADIOLIB_DEBUG_BASIC
  if (state == RADIOLIB_ERR_INVALID_BIT_RATE) {
    Serial.println(F("[LR11XX] Selected bit rate is invalid for this module!"));
    while (true) { delay(10); }
  } else if (state == RADIOLIB_ERR_INVALID_BIT_RATE_BW_RATIO) {
    Serial.println(F("[LR11XX] Selected bit rate to bandwidth ratio is invalid!"));
    Serial.println(F("[LR11XX] Increase receiver bandwidth to set this bit rate."));
    while (true) { delay(10); }
  }
#endif
    fdev = high ? 62.5 : 50.0;
    state = lr11xx_radio->setFrequencyDeviation(fdev);

#if RADIOLIB_DEBUG_BASIC
  if (state == RADIOLIB_ERR_INVALID_FREQUENCY_DEVIATION) {
    Serial.println(F("[LR11XX] Selected frequency deviation is invalid for this module!"));
    while (true) { delay(10); }
  }
#endif

    switch (curr_rx_protocol_ptr->bandwidth)
    {
    case RF_RX_BANDWIDTH_SS_50KHZ:
      bw = 117.3;
      break;
    case RF_RX_BANDWIDTH_SS_62KHZ:
      bw = 156.2;
      break;
    case RF_RX_BANDWIDTH_SS_100KHZ:
      bw = 234.3;
      break;
    case RF_RX_BANDWIDTH_SS_166KHZ:
      bw = 312.0;
      break;
    case RF_RX_BANDWIDTH_SS_200KHZ:
    case RF_RX_BANDWIDTH_SS_250KHZ:  /* TBD */
    case RF_RX_BANDWIDTH_SS_1567KHZ: /* TBD */
      bw = 467.0;
      break;
    case RF_RX_BANDWIDTH_SS_125KHZ:
    default:
      bw = 234.3;
      break;
    }
    state = lr11xx_radio->setRxBandwidth(bw);

    state = lr11xx_radio->setPreambleLength(curr_rx_protocol_ptr->preamble_size * 8);
    state = lr11xx_radio->setDataShaping(RADIOLIB_SHAPING_0_5);
    state = lr11xx_radio->setCRC(0, 0);


    size_t pkt_size = curr_rx_protocol_ptr->payload_offset + curr_rx_protocol_ptr->payload_size +
                  curr_rx_protocol_ptr->crc_size;

    switch (curr_rx_protocol_ptr->whitening)
    {
    case RF_WHITENING_MANCHESTER:
      pkt_size += pkt_size;
      break;
    case RF_WHITENING_PN9:
    case RF_WHITENING_NONE:
    case RF_WHITENING_NICERF:
    default:
      break;
    }

    state = lr11xx_radio->setWhitening(false, 0x0001 /* default SX128x value */);

    state = lr11xx_radio->fixedPacketLengthMode(pkt_size);

    state = lr11xx_radio->disableAddressFiltering();

    /* Work around premature P3I syncword detection */
    if (curr_rx_protocol_ptr->syncword_size == 2) {
      uint8_t preamble = curr_rx_protocol_ptr->preamble_type == RF_PREAMBLE_TYPE_AA ?
                         0xAA : 0x55;
      uint8_t sword[4] = { preamble,
                           preamble,
                           curr_rx_protocol_ptr->syncword[0],
                           curr_rx_protocol_ptr->syncword[1]
                         };
      state = lr11xx_radio->setSyncWord(sword, 4);
    } else {
      state = lr11xx_radio->setSyncWord((uint8_t *) curr_rx_protocol_ptr->syncword,
                                         (size_t)    curr_rx_protocol_ptr->syncword_size);
    }
    break; /*End of GFSK config*/
  }


    if (high) {
    state = lr11xx_radio->setFrequency(frequency / 1000000.0);

#if RADIOLIB_DEBUG_BASIC
    if (state == RADIOLIB_ERR_INVALID_FREQUENCY) {
      Serial.println(F("[LR11XX] Selected frequency is invalid for this module!"));
      while (true) { delay(10); }
    }
#endif 
  }
  /* Configure RF switch and output power based on hardware */
  Serial.println(F("[LR11XX] Configuring RF switch and power..."));

  switch (hw_info.model) {
    case SOFTRF_MODEL_CARD:
      /* Seeed T1000E Card */
#if RADIOLIB_VERSION_MAJOR >= 7 && RADIOLIB_VERSION_MINOR > 1
      lr11xx_radio->setRfSwitchTable(rfswitch_dio_pins_seeed, rfswitch_table_seeed);
      state = lr11xx_radio->setOutputPower(22);
#else
      lr11xx_radio->setDioAsRfSwitch(0x0f, 0x0, 0x09, 0x0B, 0x0A, 0x0, 0x4, 0x0);
      state = lr11xx_radio->setOutputPower(22, false);
#endif
      break;

    case SOFTRF_MODEL_POCKET:
      /* Elecrow ThinkNode M3 */
      // lr11xx_radio->setRfSwitchTable(rfswitch_dio_pins_elecrow, rfswitch_table_elecrow);
#if RADIOLIB_VERSION_MAJOR >= 7 && RADIOLIB_VERSION_MINOR > 1
      state = lr11xx_radio->setOutputPower(22);
#else
      state = lr11xx_radio->setOutputPower(22, false);
#endif
      break;

    default:
      state = lr11xx_radio->setOutputPower(22);
      break;
  }

  /* Enable boosted RX gain and set packet received handler */
  state = lr11xx_radio->setRxBoostedGainMode(true);

  lr11xx_radio->setPacketReceivedAction(lr11xx_receive_handler);

  Serial.println(F("[LR1110] Setup COMPLETE!"));
}


static void lr11xx_transmit()
{
  u1_t crc8;
  u2_t crc16;
  u1_t i;

  bool success = false;

  // Serial.print("[LR1110] lr11xx_transmit called, RF_tx_size = ");
  // Serial.println(RF_tx_size);




  if (RF_tx_size <= 0) {
    // return success;
    Serial.println("[LR1110] TX TF_TX = 0 exiting TX");
    return;
  }

  lr11xx_receive_active = false;
  lr11xx_transmit_complete = false;

  size_t PayloadLen = 0;

  switch (curr_rx_protocol_ptr->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_CRC_MODES:
  case RF_CHECKSUM_TYPE_NONE:
     /* crc16 left not initialized */
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    crc8 = 0x71;     /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_0000:
    crc16 = 0x0000;  /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  default:
    crc16 = 0xffff;  /* seed value */
    break;
  }

  switch (curr_rx_protocol_ptr->type)
  {
  case RF_PROTOCOL_LEGACY:
    /* take in account NRF905/FLARM "address" bytes */
    crc16 = update_crc_ccitt(crc16, 0x31);
    crc16 = update_crc_ccitt(crc16, 0xFA);
    crc16 = update_crc_ccitt(crc16, 0xB6);
    break;
  case RF_PROTOCOL_P3I:
    /* insert Net ID */
    RL_txPacket.payload[PayloadLen++] = (u1_t) ((curr_rx_protocol_ptr->net_id >> 24) & 0x000000FF);
    RL_txPacket.payload[PayloadLen++] = (u1_t) ((curr_rx_protocol_ptr->net_id >> 16) & 0x000000FF);
    RL_txPacket.payload[PayloadLen++] = (u1_t) ((curr_rx_protocol_ptr->net_id >>  8) & 0x000000FF);
    RL_txPacket.payload[PayloadLen++] = (u1_t) ((curr_rx_protocol_ptr->net_id >>  0) & 0x000000FF);
    /* insert byte with payload size */
    RL_txPacket.payload[PayloadLen++] = curr_rx_protocol_ptr->payload_size;

    /* insert byte with CRC-8 seed value when necessary */
    if (curr_rx_protocol_ptr->crc_type == RF_CHECKSUM_TYPE_CRC8_107) {
      RL_txPacket.payload[PayloadLen++] = crc8;
    }

    break;
  case RF_PROTOCOL_OGNTP:
  case RF_PROTOCOL_ADSL:
  default:
    break;
  }

  for (i=0; i < RF_tx_size; i++) {

    switch (curr_rx_protocol_ptr->whitening)
    {
    case RF_WHITENING_NICERF:
      RL_txPacket.payload[PayloadLen] = TxBuffer[i] ^ pgm_read_byte(&whitening_pattern[i]);
      break;
    case RF_WHITENING_MANCHESTER:
      RL_txPacket.payload[PayloadLen] = pgm_read_byte(&ManchesterEncode[(TxBuffer[i] >> 4) & 0x0F]);
      PayloadLen++;
      RL_txPacket.payload[PayloadLen] = pgm_read_byte(&ManchesterEncode[(TxBuffer[i]     ) & 0x0F]);
      break;
    case RF_WHITENING_NONE:
    default:
      RL_txPacket.payload[PayloadLen] = TxBuffer[i];
      break;
    }

    switch (curr_rx_protocol_ptr->crc_type)
    {
    case RF_CHECKSUM_TYPE_GALLAGER:
    case RF_CHECKSUM_TYPE_CRC_MODES:
    case RF_CHECKSUM_TYPE_NONE:
      break;
    case RF_CHECKSUM_TYPE_CRC8_107:
      update_crc8(&crc8, (u1_t)(RL_txPacket.payload[PayloadLen]));
      break;
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    case RF_CHECKSUM_TYPE_CCITT_0000:
    default:
      if (curr_rx_protocol_ptr->whitening == RF_WHITENING_MANCHESTER) {
        crc16 = update_crc_ccitt(crc16, (u1_t)(TxBuffer[i]));
      } else {
        crc16 = update_crc_ccitt(crc16, (u1_t)(RL_txPacket.payload[PayloadLen]));
      }
      break;
    }

    PayloadLen++;
  }

  switch (curr_rx_protocol_ptr->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_CRC_MODES:
  case RF_CHECKSUM_TYPE_NONE:
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    RL_txPacket.payload[PayloadLen++] = crc8;
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  case RF_CHECKSUM_TYPE_CCITT_0000:
  default:
    if (curr_rx_protocol_ptr->whitening == RF_WHITENING_MANCHESTER) {
      RL_txPacket.payload[PayloadLen++] = pgm_read_byte(&ManchesterEncode[(((crc16 >>  8) & 0xFF) >> 4) & 0x0F]);
      RL_txPacket.payload[PayloadLen++] = pgm_read_byte(&ManchesterEncode[(((crc16 >>  8) & 0xFF)     ) & 0x0F]);
      RL_txPacket.payload[PayloadLen++] = pgm_read_byte(&ManchesterEncode[(((crc16      ) & 0xFF) >> 4) & 0x0F]);
      RL_txPacket.payload[PayloadLen++] = pgm_read_byte(&ManchesterEncode[(((crc16      ) & 0xFF)     ) & 0x0F]);
      PayloadLen++;
    } else {
      RL_txPacket.payload[PayloadLen++] = (crc16 >>  8) & 0xFF;
      RL_txPacket.payload[PayloadLen++] = (crc16      ) & 0xFF;
    }
    break;
  }

  RL_txPacket.len = PayloadLen;

  // Serial.print("[LR1110] TX: RL_txPacket.len = ");
  // Serial.println(RL_txPacket.len);

  int state = lr11xx_radio->transmit((uint8_t *) &RL_txPacket.payload, (size_t) RL_txPacket.len);
  // Serial.print("[LR1110] transmit() returned state = ");
  // Serial.println(state);

  if (state == RADIOLIB_ERR_NONE) {

    success = true;
    memset(RL_txPacket.payload, 0, sizeof(RL_txPacket.payload));

  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("too long!"));

  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    // timeout occured while transmitting packet
    Serial.println(F("timeout!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println((int16_t) state);
  }


  // return success;
  // Serial.println(F("[LR1110] End of lr11xx_transmit!"));
}

static void lr11xx_resetup()
{
  int state;
  unsigned long t0, t1;
  t0 = millis();
  // Serial.print("[LR11XX] Reseting for new protocol: ");
  // Serial.println(curr_rx_protocol_ptr ? curr_rx_protocol_ptr->name : "NULL");
  // Switch to Standby RC mode first
  // t1 = millis();
  state = lr11xx_radio->standby(RADIOLIB_LR11X0_STANDBY_RC);
  // Serial.print("[LR11XX] → CChip Stanby called: ");
  // Serial.print(millis() - t1);
  // Serial.println(" ms");
  // ===== UPDATE PROTOCOL DESCRIPTORS =====
  /* Use curr_rx_protocol_ptr to set up protocol configuration */
  if (curr_rx_protocol_ptr) {
    // Serial.print(F("[LR1110] Protocol: "));
    // Serial.println(curr_rx_protocol_ptr->name);
    protocol_encode = get_protocol_encode_fn(curr_rx_protocol_ptr);
    protocol_decode = get_protocol_decode_fn(curr_rx_protocol_ptr);
  }

  // Update frequency plan
  RF_FreqPlan.setPlan(settings->band, settings->rf_protocol);

  // Get TCXO voltage (needed for begin() calls)
  float Vtcxo;
  // uint64_t eui_le = __builtin_bswap64(lr11xx_eui_be);

  switch (hw_info.model)
  {
  case SOFTRF_MODEL_BADGE:
  case SOFTRF_MODEL_PRIME_MK3:
    Vtcxo = 3.0;
    break;
  case SOFTRF_MODEL_CARD:
  default:
    Vtcxo = 1.6;
    break;
  }
  // Serial.println("[RadioLib] Get channel frequency");
  uint32_t frequency = RF_FreqPlan.getChanFrequency(0);
  bool high = (frequency > 1000000000);

  // ===== CONFIGURE MODULATION TYPE AND PROTOCOL-SPECIFIC PARAMETERS =====
  float bw;
  
  switch (curr_rx_protocol_ptr->modulation_type)
  {
  case RF_MODULATION_TYPE_LORA:
    // Serial.println("[LR11XX] → LoRa mode");
    t1 = millis();
    // // Switch to LoRa mode
    // state = lr11xx_radio->begin(125.0, 9, 7,
    //                              RADIOLIB_LR11X0_LORA_SYNC_WORD_PRIVATE,
    //                              8, Vtcxo);
    state = lr11xx_radio->setPacketType(RADIOLIB_LR11X0_PACKET_TYPE_LORA);
    // Serial.print("[LR11XX] Changed to LoRa mode: ");
    // Serial.print(millis() - t1);
    // Serial.println(" ms");
    // Set bandwidth (from RF_FreqPlan for LoRa)
    switch (RF_FreqPlan.Bandwidth)
    {
    case RF_RX_BANDWIDTH_SS_62KHZ:
      bw = high ? 203.125 : 125.0;
      break;
    case RF_RX_BANDWIDTH_SS_250KHZ:
      bw = high ? 812.5   : 500.0;
      break;
    case RF_RX_BANDWIDTH_SS_125KHZ:
    default:
      bw = high ? 406.25  : 250.0;
      break;
    }
    state = lr11xx_radio->setBandwidth(bw, high);
    
    // LoRa-specific parameters
    state = lr11xx_radio->setSpreadingFactor(7);
    state = lr11xx_radio->setCodingRate(5);
    state = lr11xx_radio->setSyncWord((uint8_t) curr_rx_protocol_ptr->syncword[0]);

    state = lr11xx_radio->setPreambleLength(8);
    state = lr11xx_radio->explicitHeader();
    state = lr11xx_radio->setCRC(true);
    
    break;
    
  case RF_MODULATION_TYPE_2FSK:
  default:

    // Serial.println("[LR11XX] → Set packet to FSK mode");
    // Switch to FSK mode
    t1 = millis();
    // state = lr11xx_radio->beginGFSK(4.8, 5.0, 156.2, 16, Vtcxo); // this task take 300ms
    state = lr11xx_radio->setPacketType(RADIOLIB_LR11X0_PACKET_TYPE_GFSK);
    // Serial.print("[LR11XX] Changed to FSK mode: ");
    // Serial.print(millis() - t1);
    // Serial.println(" ms");
    // Set bitrate
    float br = high ? 125.0 : (curr_rx_protocol_ptr->bitrate == RF_BITRATE_38400 ? 38.4 : 100.0);
    state = lr11xx_radio->setBitRate(br);
    // Serial.println("[LR11XX] → set bitrate");
    // Set frequency deviation
    float fdev;
    switch (curr_rx_protocol_ptr->deviation)
    {
    // case RF_FREQUENCY_DEVIATION_10KHZ:
    //   fdev = high ? 62.5 : 10.0;
    //   break;
    // case RF_FREQUENCY_DEVIATION_19_2KHZ:
    //   fdev = 19.2;
    //   break;
    // case RF_FREQUENCY_DEVIATION_25KHZ:
    //   fdev = 25.0;
    //   break;
    case RF_FREQUENCY_DEVIATION_50KHZ:
    default:
      fdev = high ? 62.5 : 50.0;
      break;
    }
    state = lr11xx_radio->setFrequencyDeviation(fdev);
    // Serial.println("[LR11XX] set freq deviation");
    // Set RX bandwidth
    switch (curr_rx_protocol_ptr->bandwidth)
    {
    case RF_RX_BANDWIDTH_SS_50KHZ:
      bw = 117.3;
      break;
    case RF_RX_BANDWIDTH_SS_62KHZ:
      bw = 156.2;
      break;
    case RF_RX_BANDWIDTH_SS_100KHZ:
      bw = 234.3;
      break;
    case RF_RX_BANDWIDTH_SS_166KHZ:
      bw = 312.0;
      break;
    case RF_RX_BANDWIDTH_SS_200KHZ:
    case RF_RX_BANDWIDTH_SS_250KHZ:
    case RF_RX_BANDWIDTH_SS_1567KHZ:
      bw = 467.0;
      break;
    case RF_RX_BANDWIDTH_SS_125KHZ:
    default:
      bw = 234.3;
      break;
    }
    state = lr11xx_radio->setRxBandwidth(bw);
    // Serial.println("[LR11XX] → Set Bandwidth");
    // Set preamble
    state = lr11xx_radio->setPreambleLength(curr_rx_protocol_ptr->preamble_size * 8);
    // Serial.println("[LR11XX] → Set Preamble Length");
    // Data shaping
    state = lr11xx_radio->setDataShaping(RADIOLIB_SHAPING_0_5);
    
    // Serial.println("[LR11XX] → Set Data Shaping");
    // CRC (handled by software for Legacy)
    state = lr11xx_radio->setCRC(0, 0);
    
    // Calculate packet size (Manchester doubles it)
    size_t pkt_size = curr_rx_protocol_ptr->payload_offset + curr_rx_protocol_ptr->payload_size + curr_rx_protocol_ptr->crc_size;
    if (curr_rx_protocol_ptr->whitening == RF_WHITENING_MANCHESTER) {
      pkt_size += pkt_size;
    }
    
    // Whitening
    state = lr11xx_radio->setWhitening(false, 0x0001);
    // Serial.println("[LR11XX] → Set Whitening");
  
    // Packet mode
    state = lr11xx_radio->fixedPacketLengthMode(pkt_size);
    state = lr11xx_radio->disableAddressFiltering();
    // Serial.println("[LR11XX] → Set Packet Mode");
    // Syncword (with preamble workaround)
    if (curr_rx_protocol_ptr->syncword_size == 2) {
      uint8_t preamble = (curr_rx_protocol_ptr->preamble_type == RF_PREAMBLE_TYPE_AA) ? 0xAA : 0x55;
      uint8_t sword[4] = { preamble, preamble, curr_rx_protocol_ptr->syncword[0], curr_rx_protocol_ptr->syncword[1] };
      state = lr11xx_radio->setSyncWord(sword, 4);
    } else {
      state = lr11xx_radio->setSyncWord((uint8_t *) curr_rx_protocol_ptr->syncword,
                                         (size_t) curr_rx_protocol_ptr->syncword_size);
    }
    
    break;
  }
  state = rf_chip->receive();
  // Serial.print("[LR11XX] Protocol reconfiguration complete: ");
  // Serial.print(millis() - t0);
  // Serial.println(" ms");
}

static void lr11xx_standby()
{
#if USE_LR11XX
  int state = lr11xx_radio->standby(RADIOLIB_LR11X0_STANDBY_RC);
#endif
  // DO NOT call RadioSPI.end() here!
}

static void lr11xx_shutdown()
{
  /* Shutdown LR1110 radio */
  if (lr11xx_radio != nullptr) {
    /* Enter standby/sleep mode */
    lr11xx_radio->standby();
    lr11xx_radio->sleep();
  }

  RadioSPI.end();
}

static bool lr11xx_receive()
{
  /* Check for incoming packets from LR1110 */
  bool success = false;
  int state;

  // static uint32_t last_call_print = 0;
  // if (millis() - last_call_print > 2000) {
  //   Serial.println(F("[LR1110] lr11xx_receive() called"));
  //   last_call_print = millis();
  // }

  if (lr11xx_radio == nullptr) {
    return success;
  }

  /* Skip if power saving is enabled */
  if (settings->power_save & POWER_SAVE_NORECEIVE) {
    return success;
  }

  /* Start receive if not already active */
  if (!lr11xx_receive_active) {
    state = lr11xx_radio->startReceive();
    if (state == RADIOLIB_ERR_NONE) {
      lr11xx_receive_active = true;
    } else {
      Serial.print(F("[LR11XX] startReceive() FAILED: "));
      Serial.println(state);
    }
  }

  /* Poll DIO9 pin as fallback if interrupt doesn't work */
  // if (lmic_pins.dio[0] != LMIC_UNUSED_PIN && !lr11xx_receive_complete) {
  //   if (digitalRead(lmic_pins.dio[0]) == HIGH) {
  //     /* Manually trigger if pin goes high */
  //     lr11xx_receive_complete = true;
  //   }
  // }

  // /* Periodic check every 5 seconds to detect packets if interrupt fails */
  // if (!lr11xx_receive_complete && (millis() - lr11xx_last_check > 1000)) {
  //   lr11xx_last_check = millis();

  //   size_t pkt_len = lr11xx_radio->getPacketLength();
  //   if (pkt_len > 0) {
  //     Serial.print(F("[LR1110] Timeout poll detected packet, length: "));
  //     Serial.println(pkt_len);
  //     lr11xx_receive_complete = true;
  //   }
  // }

  // static uint32_t last_print = 0;
  // if (millis() - last_print > 1000) {  // Print every second
  //   Serial.print("[LR1110] lr11xx_receive_complete = ");
  //   Serial.print(lr11xx_receive_complete ? "true" : "false");
  //   Serial.print(", DIO9_pin_state = ");
  //   if (lmic_pins.dio[0] != LMIC_UNUSED_PIN) {
  //     Serial.print(digitalRead(lmic_pins.dio[0]));
  //   } else {
  //     Serial.print("UNUSED");
  //   }
  //   Serial.print(", lr11xx_receive_active = ");
  //   Serial.println(lr11xx_receive_active);
  //   last_print = millis();
  // }

if (lr11xx_receive_complete == true) {

    RL_rxPacket.len = lr11xx_radio->getPacketLength();
    // Serial.print("[LR1110] Received packet length: ");
    // Serial.println(RL_rxPacket.len);

    if (RL_rxPacket.len > 0) {

      if (RL_rxPacket.len > sizeof(RL_rxPacket.payload)) {
        RL_rxPacket.len = sizeof(RL_rxPacket.payload);
      }

      state = lr11xx_radio->readData(RL_rxPacket.payload, RL_rxPacket.len);
      lr11xx_receive_active = false;

      if (state == RADIOLIB_ERR_NONE &&
         !memeqzero(RL_rxPacket.payload, RL_rxPacket.len)) {
        size_t size = 0;
        uint8_t offset;

        u1_t crc8, pkt_crc8;
        u2_t crc16, pkt_crc16;

        RadioLib_DataPacket *RL_rxPacket_ptr = &RL_rxPacket;

        switch (curr_rx_protocol_ptr->crc_type)
        {
        case RF_CHECKSUM_TYPE_GALLAGER:
        case RF_CHECKSUM_TYPE_CRC_MODES:
        case RF_CHECKSUM_TYPE_NONE:
           /* crc16 left not initialized */
          break;
        case RF_CHECKSUM_TYPE_CRC8_107:
          crc8 = 0x71;     /* seed value */
          break;
        case RF_CHECKSUM_TYPE_CCITT_0000:
          crc16 = 0x0000;  /* seed value */
          break;
        case RF_CHECKSUM_TYPE_CCITT_FFFF:
        default:
          crc16 = 0xffff;  /* seed value */
          break;
        }

        switch (curr_rx_protocol_ptr->type)
        {
        case RF_PROTOCOL_LEGACY:
          /* take in account NRF905/FLARM "address" bytes */
          crc16 = update_crc_ccitt(crc16, 0x31);
          crc16 = update_crc_ccitt(crc16, 0xFA);
          crc16 = update_crc_ccitt(crc16, 0xB6);
          break;
        case RF_PROTOCOL_P3I:
        case RF_PROTOCOL_OGNTP:
        case RF_PROTOCOL_ADSL:
        default:
          break;
        }

        uint8_t i;

        switch (curr_rx_protocol_ptr->type)
        {
        case RF_PROTOCOL_P3I:
          offset = curr_rx_protocol_ptr->payload_offset;
          for (i = 0; i < curr_rx_protocol_ptr->payload_size; i++)
          {
            update_crc8(&crc8, (u1_t)(RL_rxPacket_ptr->payload[i + offset]));
            if (i < sizeof(RxBuffer)) {
              RxBuffer[i] = RL_rxPacket_ptr->payload[i + offset] ^
                            pgm_read_byte(&whitening_pattern[i]);
            }
          }

          pkt_crc8 = RL_rxPacket_ptr->payload[i + offset];

          if (crc8 == pkt_crc8) {
            success = true;
          }
          break;
        case RF_PROTOCOL_FANET:
          offset = curr_rx_protocol_ptr->payload_offset;
          size   = curr_rx_protocol_ptr->payload_size + curr_rx_protocol_ptr->crc_size;
          for (i = 0; i < size; i++)
          {
            if (i < sizeof(RxBuffer)) {
              RxBuffer[i] = RL_rxPacket_ptr->payload[i + offset];
            }
          }
          success = true;
          break;
        case RF_PROTOCOL_OGNTP:
        case RF_PROTOCOL_ADSL:
        case RF_PROTOCOL_LEGACY:
        default:
          Serial.print("[LR1110] LEGACY RX: actual RL_rxPacket.len=");
          Serial.print(RL_rxPacket_ptr->len);
          Serial.print(", calculated size=");
          Serial.println(curr_rx_protocol_ptr->payload_offset +
                          curr_rx_protocol_ptr->payload_size +
                          curr_rx_protocol_ptr->payload_size +
                          curr_rx_protocol_ptr->crc_size +
                          curr_rx_protocol_ptr->crc_size);
          offset = 0;
          size   = curr_rx_protocol_ptr->payload_offset +
                   curr_rx_protocol_ptr->payload_size +
                   curr_rx_protocol_ptr->payload_size +
                   curr_rx_protocol_ptr->crc_size +
                   curr_rx_protocol_ptr->crc_size;
          if (RL_rxPacket_ptr->len >= (size + offset)) {
            uint8_t val1, val2;
            for (i = 0; i < size; i++) {
              val1 = pgm_read_byte(&ManchesterDecode[RL_rxPacket_ptr->payload[i + offset]]);
              i++;
              val2 = pgm_read_byte(&ManchesterDecode[RL_rxPacket_ptr->payload[i + offset]]);
              if ((i>>1) < sizeof(RxBuffer)) {
                RxBuffer[i>>1] = ((val1 & 0x0F) << 4) | (val2 & 0x0F);

                if (i < size - (curr_rx_protocol_ptr->crc_size + curr_rx_protocol_ptr->crc_size)) {
                  switch (curr_rx_protocol_ptr->crc_type)
                  {
                  case RF_CHECKSUM_TYPE_GALLAGER:
                  case RF_CHECKSUM_TYPE_CRC_MODES:
                  case RF_CHECKSUM_TYPE_NONE:
                    break;
                  case RF_CHECKSUM_TYPE_CCITT_FFFF:
                  case RF_CHECKSUM_TYPE_CCITT_0000:
                  default:
                    crc16 = update_crc_ccitt(crc16, (u1_t)(RxBuffer[i>>1]));
                    break;
                  }
                }
              }
            }

            size = size>>1;

            switch (curr_rx_protocol_ptr->crc_type)
            {
            case RF_CHECKSUM_TYPE_GALLAGER:
              if (LDPC_Check((uint8_t  *) &RxBuffer[0]) == 0) {
                success = true;
              }
              break;
            case RF_CHECKSUM_TYPE_CRC_MODES:
#if defined(ENABLE_ADSL)
              if (ADSL_Packet::checkPI((uint8_t  *) &RxBuffer[0], size) == 0) {
                success = true;
              }
#endif /* ENABLE_ADSL */
              break;
            case RF_CHECKSUM_TYPE_CCITT_FFFF:
            case RF_CHECKSUM_TYPE_CCITT_0000:
              offset = curr_rx_protocol_ptr->payload_offset + curr_rx_protocol_ptr->payload_size;
              if (offset + 1 < sizeof(RxBuffer)) {
                pkt_crc16 = (RxBuffer[offset] << 8 | RxBuffer[offset+1]);
                if (crc16 == pkt_crc16) {

                  success = true;
                }
              }
              break;
            default:
              break;
            }
          }
          break;
        }

        if (success) {
          RF_last_rssi = lr11xx_radio->getRSSI();
          RF_last_protocol = current_RX_protocol;
          rx_packets_counter++;
        }
      }

      memset(RL_rxPacket.payload, 0, sizeof(RL_rxPacket.payload));
#if USE_SX1262 && (RADIOLIB_GODMODE || RADIOLIB_LOW_LEVEL)
      lr11xx_radio->writeBuffer(RL_rxPacket.payload, RL_rxPacket.len);
      lr11xx_radio->setBufferBaseAddress();
#endif
      RL_rxPacket.len = 0;
    }

    lr11xx_receive_complete = false;
  }

  return success;
}

#endif /* USE_LR1110 */

uint8_t RF_Payload_Size(uint8_t protocol)
{
  switch (protocol)
  {
    case RF_PROTOCOL_LATEST:    return latest_proto_desc.payload_size;
    case RF_PROTOCOL_LEGACY:    return legacy_proto_desc.payload_size;
    case RF_PROTOCOL_OGNTP:     return ogntp_proto_desc.payload_size;
    case RF_PROTOCOL_ADSL:      return adsl_proto_desc.payload_size;
    case RF_PROTOCOL_P3I:       return p3i_proto_desc.payload_size;
    case RF_PROTOCOL_FANET:     return fanet_proto_desc.payload_size;
#if !defined(EXCLUDE_UAT978)
    case RF_PROTOCOL_ADSB_UAT:  return uat978_proto_desc.payload_size;
#endif
    default:                    return 0;
  }
}
