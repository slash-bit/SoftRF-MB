/*
 * Protocol_P3I.cpp
 * Encoder and decoder for PilotAware P3I radio protocol
 * Copyright (C) 2017-2021 Linar Yusupov
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

#include <stdint.h>

#include <protocol.h>

#include "../../../SoftRF.h"
#include "../../driver/RF.h"
#include "../../driver/Settings.h"
#include "ADSL.h"


const rf_proto_desc_t p3i_proto_desc = {
  "P3I",
  .type             = RF_PROTOCOL_P3I,
  .modulation_type  = RF_MODULATION_TYPE_2FSK,
  .preamble_type    = P3I_PREAMBLE_TYPE,
  .preamble_size    = P3I_PREAMBLE_SIZE,
  .syncword         = P3I_SYNCWORD,
  .syncword_size    = P3I_SYNCWORD_SIZE,
  .syncword_skip    = 0,
  .net_id           = P3I_NET_ID,
  .payload_type     = RF_PAYLOAD_DIRECT,
  .payload_size     = P3I_PAYLOAD_SIZE,
  .payload_offset   = P3I_PAYLOAD_OFFSET,
  .crc_type         = P3I_CRC_TYPE,
  .crc_size         = P3I_CRC_SIZE,

  .bitrate          = RF_BITRATE_38400,
  .deviation        = P3I_FDEV,
  .whitening        = RF_WHITENING_NONE,   // new PAW uses ADS-L payload, no NiceRF whitening
  .bandwidth        = P3I_BANDWIDTH,

  .air_time         = P3I_AIR_TIME,

  .tm_type          = RF_TIMING_INTERVAL,
  .tx_interval_min  = P3I_TX_INTERVAL_MIN,
  .tx_interval_max  = P3I_TX_INTERVAL_MAX,
  .slot0            = {0, 0},
  .slot1            = {0, 0}
};

// whitening_pattern kept for reference; no longer used with new PAW protocol
const uint8_t whitening_pattern[] PROGMEM = { 0x05, 0xb4, 0x05, 0xae, 0x14, 0xda,
  0xbf, 0x83, 0xc4, 0x04, 0xb2, 0x04, 0xd6, 0x4d, 0x87, 0xe2, 0x01, 0xa3, 0x26,
  0xac, 0xbb, 0x63, 0xf1, 0x01, 0xca, 0x07, 0xbd, 0xaf, 0x60, 0xc8, 0x12, 0xed,
  0x04, 0xbc, 0xf6, 0x12, 0x2c, 0x01, 0xd9, 0x04, 0xb1, 0xd5, 0x03, 0xab, 0x06,
  0xcf, 0x08, 0xe6, 0xf2, 0x07, 0xd0, 0x12, 0xc2, 0x09, 0x34, 0x20 };

// New PAW protocol: outer layer is old P3I frame (CRC8), payload is ADS-L (21 bytes + 3-byte CRC24).
// RF.cpp already verified the CRC8; here we check the inner ADS-L CRC24 then delegate to adsl_decode().
bool p3i_decode(void *pkt, container_t *this_aircraft, ufo_t *fop) {

  if (ADSL_Packet::checkPI((uint8_t *) pkt, (uint8_t) P3I_PAYLOAD_SIZE)) {
    Serial.println("PAW internal CRC24 wrong");
    return false;
  }

  ++rx_packets_counter;

  if (adsl_decode(pkt, this_aircraft, fop) == false)
    return false;

  fop->protocol = RF_PROTOCOL_P3I;
  return true;
}

size_t p3i_encode(void *pkt, container_t *aircraft) {

  size_t size = adsl_encode(pkt, aircraft);
  if (size != ADSL_PAYLOAD_SIZE + ADSL_CRC_SIZE  // 24
   || size != P3I_PAYLOAD_SIZE) {                // 24
    Serial.print("p3i_encode() error: adsl_encode() returned ");
    Serial.println(size);
    return 0;
  }
  return P3I_PAYLOAD_SIZE;   // 24
}
