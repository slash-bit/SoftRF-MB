/*
 * radiolib_lr1110.cpp - Semtech LR1110 LoRa transceiver driver
 * Uses RadioLib library for SPI communication and packet handling
 */

#include <RadioLib.h>
#include "../RF.h"

#include <manchester.h>


#ifndef RadioSPI
#define RadioSPI        SPI
#endif

/* Forward declarations */
static Module *mod;
// static LR1110 *radio = nullptr;

const rf_proto_desc_t *rl_protocol = &ogntp_proto_desc;

#define RADIOLIB_MAX_DATA_LENGTH    128

typedef struct
{
  uint8_t len;
  uint8_t payload[RADIOLIB_MAX_DATA_LENGTH];
} RadioLib_DataPacket;


static RadioLib_DataPacket RL_txPacket;
static RadioLib_DataPacket RL_rxPacket;

extern size_t RF_tx_size;

#if !defined(USE_BASICMAC)
static const SPISettings probe_settings(1000000UL, MSBFIRST, SPI_MODE0);

static void hal_spi_select (int on) {

#if defined(SPI_HAS_TRANSACTION)
    if (on)
        RadioSPI.beginTransaction(probe_settings);
    else
        RadioSPI.endTransaction();
#endif

    //Serial.println(val?">>":"<<");
    digitalWrite(lmic_pins.nss, !on ? HIGH : LOW);
}

// Datasheet defins typical times until busy goes low. Most are < 200us,
// except when waking up from sleep, which typically takes 3500us. Since
// we cannot know here if we are in sleep, we'll have to assume we are.
// Since 3500 is typical, not maximum, wait a bit more than that.
static unsigned long MAX_BUSY_TIME = 5000;

static void hal_pin_busy_wait (void) {
    if (lmic_pins.busy == LMIC_UNUSED_PIN) {
        // TODO: We could probably keep some state so we know the chip
        // is in sleep, since otherwise the delay can be much shorter.
        // Also, all delays after commands (rather than waking up from
        // sleep) are measured from the *end* of the previous SPI
        // transaction, so we could wait shorter if we remember when
        // that was.
        delayMicroseconds(MAX_BUSY_TIME);
    } else {
        unsigned long start = micros();

        while((micros() - start) < MAX_BUSY_TIME && digitalRead(lmic_pins.busy)) /* wait */;
    }
}
#endif /* USE_BASICMAC */

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

/* LR1110 transceiver state */
static volatile bool lr1110_receive_flag = false;
static uint8_t lr1110_packet_length = 0;

bool lr1110_probe() {
  /* Attempt to initialize LR1110 module */
  /* Check for device presence on SPI bus */

  /* Test Module instantiation */
  if (!module) {
    module = new Module(
      SOC_GPIO_PIN_SS,
      SOC_GPIO_PIN_BUSY,
      LMIC_UNUSED_PIN,
      LMIC_UNUSED_PIN,
      SPI
    );
  }

  /* Create LR1110 instance and attempt initialization */
  if (!radio) {
    radio = new LR1110(module);
    int16_t state = radio->begin(868.0);  /* Test with 868 MHz */

    if (state != RADIOLIB_ERR_NONE) {
      /* Probe failed - cleanup */
      if (radio) {
        delete radio;
        radio = nullptr;
      }
      if (module) {
        delete module;
        module = nullptr;
      }
      return false;
    }
  }

  return (radio != nullptr);
}

void lr1110_setup() {
  /* Initialize SPI and LR1110 module */
  /* Configure frequency, bandwidth, spreading factor, etc. */

  if (!module) {
    module = new Module(
      SOC_GPIO_PIN_SS,
      SOC_GPIO_PIN_BUSY,
      LMIC_UNUSED_PIN,
      LMIC_UNUSED_PIN,
      SPI
    );
  }

  if (!radio) {
    radio = new LR1110(module);
  }

  /* Begin with regional frequency (adjust as needed) */
  int16_t state = radio->begin(868.0);  /* 868 MHz for EU */

  if (state == RADIOLIB_ERR_NONE) {
    /* Configure basic LoRa parameters */
    radio->setBandwidth(125.0);           /* 125 kHz bandwidth */
    radio->setSpreadingFactor(11);        /* SF11 for range */
    radio->setCodingRate(8);              /* CR 4/8 */
    radio->setSyncWord(0x12);             /* LoRa sync word */
    radio->setPreambleLength(8);          /* 8 symbol preamble */
    radio->startReceive();                /* Start continuous RX */
  }
}

void lr1110_channel(uint8_t channel) {
  /* Set frequency for given channel */
  /* Implementation depends on regional settings */

  if (!radio) return;

  /* Basic channel mapping: channel * 25kHz from base 868 MHz */
  /* Adjust based on actual SoftRF channel plan */
  float frequency = 868.0 + (channel * 0.025);

  radio->setFrequency(frequency);
}

bool lr1110_receive() {
  /* Check for incoming data on LR1110 */
  /* Read FIFO and process packet */

  if (!radio) return false;

  /* Check for BUSY pin state (should be LOW when packet ready) */
  int state = radio->readData(RxBuffer, sizeof(RxBuffer));

  if (state == RADIOLIB_ERR_NONE) {
    /* Packet received successfully */
    lr1110_packet_length = radio->getPacketLength();
    RF_last_rssi = radio->getRSSI();
    RF_last_crc = 0;  /* LR1110 has hardware CRC - assume valid if we got here */

    /* Restart receive mode */
    radio->startReceive();
    return true;
  } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    /* No packet received - restart RX */
    radio->startReceive();
  }

  return false;
}

void lr1110_transmit() {
  /* Transmit packet from buffer */
  /* Write to FIFO and trigger transmission */

  if (!radio) return;

  /* Transmit data from TxBuffer with proper length handling */
  int16_t state = radio->transmit(TxBuffer, TxBuffer[0] + 1);  /* Length field + data */

  if (state == RADIOLIB_ERR_NONE) {
    /* Transmission successful */
    TxEndMarker = millis();
  } else {
    /* Transmission failed - restart RX mode */
    radio->startReceive();
  }
}

void lr1110_shutdown() {
  /* Put LR1110 into sleep/low-power mode */

  if (radio) {
    radio->sleep();  /* Deep sleep */
  }
}

/* Interrupt handler for DIO1 */
void lr1110_setFlagISR() {
  lr1110_receive_flag = true;
}