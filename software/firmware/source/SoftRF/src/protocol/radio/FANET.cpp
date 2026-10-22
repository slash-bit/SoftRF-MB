/*
 * Protocol_FANET.cpp
 *
 * Encoder and decoder for open FANET radio protocol
 *
 * URL:
 *    Development -  https://github.com/3s1d/fanet-stm32
 *    Deprecated  -  https://github.com/3s1d/fanet
 *
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

#include <math.h>
#include <stdint.h>
#include <string.h>

#include <protocol.h>
#include <TimeLib.h>

#include "../../../SoftRF.h"
#include "../../TrafficHelper.h"
#include "../../driver/GNSS.h"
#include "../../driver/RF.h"
#include "FANET.h"
#include "../../driver/Settings.h"
#include "../../protocol/data/NMEA.h"
#include "../../driver/Bluetooth.h"

const rf_proto_desc_t fanet_proto_desc = {
  "FANET",
  .type             = RF_PROTOCOL_FANET,
  .modulation_type  = RF_MODULATION_TYPE_LORA,
  .preamble_type    = 0 /* INVALID FOR LORA */,
  .preamble_size    = 0 /* INVALID FOR LORA */,
#if defined(FANET_DEPRECATED)
  .syncword         = { 0x12 },  // sx127x default value, valid for FANET
#else
  .syncword         = { 0xF1 },  // FANET+
#endif
  .syncword_size    = 1,
  .syncword_skip    = 0,
  .net_id           = 0x0000, /* not in use */
  .payload_type     = RF_PAYLOAD_DIRECT,
  .payload_size     = FANET_PAYLOAD_SIZE,
  .payload_offset   = 0,
  .crc_type         = RF_CHECKSUM_TYPE_NONE, /* LoRa packet has built-in CRC */
  .crc_size         = 0 /* INVALID FOR LORA */,
  .bitrate          = DR_SF7B /* CR_5 BW_250 SF_7 */,

  .deviation        = 0 /* INVALID FOR LORA */,
  .whitening        = RF_WHITENING_NONE,
  .bandwidth        = 0, /* INVALID FOR LORA */
     // later upstream code says: RF_RX_BANDWIDTH_SS_125KHZ, /* RF_RX_BANDWIDTH_SS_250KHZ */
     // although the .bandwidth field is not used in FANET

  .air_time         = FANET_AIR_TIME,

  .tm_type          = RF_TIMING_INTERVAL,
  .tx_interval_min  = FANET_TX_INTERVAL_MIN,
  .tx_interval_max  = FANET_TX_INTERVAL_MAX,
  .slot0            = {0, 0},
  .slot1            = {0, 0}
};

const uint8_t aircraft_type_to_fanet[] PROGMEM = {
	FANET_AIRCRAFT_TYPE_OTHER,
	FANET_AIRCRAFT_TYPE_GLIDER,
	FANET_AIRCRAFT_TYPE_POWERED,
	FANET_AIRCRAFT_TYPE_HELICOPTER,
	FANET_AIRCRAFT_TYPE_PARAGLIDER,   // changed from OTHER, following later upstream code
	FANET_AIRCRAFT_TYPE_POWERED,
	FANET_AIRCRAFT_TYPE_HANGGLIDER,
	FANET_AIRCRAFT_TYPE_PARAGLIDER,
	FANET_AIRCRAFT_TYPE_POWERED,
	FANET_AIRCRAFT_TYPE_POWERED,
	FANET_AIRCRAFT_TYPE_OTHER,
	FANET_AIRCRAFT_TYPE_BALLOON,
	FANET_AIRCRAFT_TYPE_BALLOON,
	FANET_AIRCRAFT_TYPE_UAV,
	FANET_AIRCRAFT_TYPE_OTHER,
	FANET_AIRCRAFT_TYPE_OTHER,
	FANET_AIRCRAFT_TYPE_OTHER    // added for "winch"
};

const uint8_t aircraft_type_from_fanet[] PROGMEM = {
	AIRCRAFT_TYPE_UNKNOWN,
	AIRCRAFT_TYPE_PARAGLIDER,
	AIRCRAFT_TYPE_HANGGLIDER,
	AIRCRAFT_TYPE_BALLOON,
	AIRCRAFT_TYPE_GLIDER,
	AIRCRAFT_TYPE_POWERED,
	AIRCRAFT_TYPE_HELICOPTER,
	AIRCRAFT_TYPE_UAV
};

/* AT_TO_FANET and AT_FROM_FANET macros are in FANET.h */

/* FANET name table: stores pilot names received via Type 2 packets */
fanet_name_entry_t fanet_name_table[FANET_NAME_TABLE_SIZE];

static void fanet_name_store(uint32_t addr, const char *name)
{
    int oldest = 0;
    for (int i = 0; i < FANET_NAME_TABLE_SIZE; i++) {
        if (fanet_name_table[i].addr == addr || fanet_name_table[i].addr == 0) {
            oldest = i;
            break;
        }
        /* reuse first slot if table is full (simple replacement) */
        oldest = i;
    }
    fanet_name_table[oldest].addr = addr;
    strncpy(fanet_name_table[oldest].name, name, FANET_NAME_MAX_LEN - 1);
    fanet_name_table[oldest].name[FANET_NAME_MAX_LEN - 1] = '\0';
}

const char *fanet_name_lookup(uint32_t addr)
{
    for (int i = 0; i < FANET_NAME_TABLE_SIZE; i++) {
        if (fanet_name_table[i].addr == addr)
            return fanet_name_table[i].name;
    }
    return NULL;
}

void fanet_name_set_type(uint32_t addr, uint8_t type_status)
{
    for (int i = 0; i < FANET_NAME_TABLE_SIZE; i++) {
        if (fanet_name_table[i].addr == addr) {
            fanet_name_table[i].type_status = type_status;
            return;
        }
    }
}

#if defined(FANET_DEPRECATED)
/* ------------------------------------------------------------------------- */
/*
 *
 *  Created on: 30 Sep 2016
 *      Author: sid
 */
static uint16_t coord2payload_compressed(float deg)
{
	float deg_round =  roundf(deg);
	bool deg_odd = ((int)deg_round) & 1;
	const float decimal = deg-deg_round;
	const int dec_int = constrain((int)(decimal*32767), -16383, 16383);

	return ((dec_int&0x7FFF) | (!!deg_odd<<15));
}
/* ------------------------------------------------------------------------- */
/*
 *  Created on: 06 Dec 2017
 *      Author: Linar Yusupov
 */
static float payload_compressed2coord(uint16_t payload, float ref_deg)
{
  float deg;
  bool deg_odd = (payload >> 15) & 0x1;
  unsigned int dec_uint = payload & 0x7FFF;

  float ref_deg_round =  roundf(ref_deg);
  bool ref_deg_odd = ((int)ref_deg_round) & 1;

  int dec_int = 0;
  if (dec_uint <= 0x3FFF) {
      dec_int = (int) dec_uint;
  } else if (dec_uint >= 0x4001) {
    dec_int = (int)-1 - (int) (dec_uint ^ 0x7FFF);
  }

  float decimal = (float) dec_int / 32767.0;

  if (deg_odd == ref_deg_odd) {
    deg = ref_deg_round + decimal;
  } else {
    if (decimal < 0) {
      deg = ref_deg_round + 1 + decimal;
    } else {
      deg = ref_deg_round - 1 + decimal;
    }
  }

  return deg;
}
/* ------------------------------------------------------------------------- */
#else

/* ------------------------------------------------------------------------- */
/*
 *
 *  Created on: 30 Sep 2016
 *      Author: sid
 */
static void coord2payload_absolut(float lat, float lon, uint8_t *buf)
{
	if (buf == NULL)
		return;

	int32_t lat_i = roundf(lat * 93206.0f);
	int32_t lon_i = roundf(lon * 46603.0f);

	buf[0] = ((uint8_t*)&lat_i)[0];
	buf[1] = ((uint8_t*)&lat_i)[1];
	buf[2] = ((uint8_t*)&lat_i)[2];

	buf[3] = ((uint8_t*)&lon_i)[0];
	buf[4] = ((uint8_t*)&lon_i)[1];
	buf[5] = ((uint8_t*)&lon_i)[2];
}
/* ------------------------------------------------------------------------- */
/*
 *  Created on: 06 Dec 2017
 *      Author: Linar Yusupov
 */
static void payload_absolut2coord(float *lat, float *lon, uint8_t *buf)
{
  int32_t lat_i = 0;
  int32_t lon_i = 0;

  if (buf == NULL || lat == NULL || lon == NULL)
    return;

  ((uint8_t*)&lat_i)[0] = buf[0];
  ((uint8_t*)&lat_i)[1] = buf[1];
  ((uint8_t*)&lat_i)[2] = buf[2];
  ((uint8_t*)&lat_i)[3] = buf[2] & 0x80 ? 0xFF : 0x00;

  ((uint8_t*)&lon_i)[0] = buf[3];
  ((uint8_t*)&lon_i)[1] = buf[4];
  ((uint8_t*)&lon_i)[2] = buf[5];
  ((uint8_t*)&lon_i)[3] = buf[5] & 0x80 ? 0xFF : 0x00;

  *lat = (float) lat_i / 93206.0f;
  *lon = (float) lon_i / 46603.0f;
}

/* ------------------------------------------------------------------------- */
#endif

bool fanet_decode(void *fanet_pkt, container_t *this_aircraft, ufo_t *fop) {

  /* RFMODE bit 0: FANET RX — skip if disabled by XCGuide */
  if (!(fnf_rfmode & FNF_RFMODE_FANET_RX))
    return false;

  fanet_packet_t *pkt = (fanet_packet_t *) fanet_pkt;
  unsigned int altitude;
  uint8_t speed_byte, climb_byte, offset_byte;
  int speed_int, climb_int, offset_int;
  bool rval = false;

  /* Extended header: check unicast destination address.
   * Byte 4 bit 5 (Cast) = 1 means unicast; bytes 5-7 carry destination address.
   * Spec: byte5=dest_manufacturer, bytes 6-7=dest_unique_id (little-endian).
   * Drop the packet if it is not addressed to us. */
  if (pkt->ext_header) {
    const uint8_t *raw = (const uint8_t *) fanet_pkt;
    uint8_t ext_flags = raw[4];
    bool is_unicast = (ext_flags >> 5) & 0x01;
    if (is_unicast) {
      uint32_t dest = ((uint32_t)raw[5] << 16) | ((uint32_t)raw[7] << 8) | raw[6];
      if (dest != this_aircraft->addr)
        return false;
    }
  }

  if (pkt->ext_header == 0 && pkt->type == 1 ) {  /* Tracking  */

    fop->addr     = (pkt->vendor << 16) | pkt->address;

    /* ignore this device own (relayed) packets */
    //if (pkt->vendor  == SOFRF_FANET_VENDOR_ID &&
    //    pkt->address == (this_aircraft->addr & 0xFFFF) /* && */
    //    /* pkt->forward == 1 */) {
    //  return rval;
    //}
    if (fop->addr == this_aircraft->addr)
        return false;

    fop->protocol = RF_PROTOCOL_FANET;
    fop->addr_type = ADDR_TYPE_FLARM;    // i.e., device - was ADDR_TYPE_FANET
    fop->timestamp = this_aircraft->timestamp;
    fop->gnsstime_ms = millis();

#if defined(FANET_DEPRECATED)
    fop->latitude  = payload_compressed2coord(pkt->latitude, this_aircraft->latitude);
    fop->longitude = payload_compressed2coord(pkt->longitude, this_aircraft->longitude);
#else
    payload_absolut2coord(&(fop->latitude), &(fop->longitude),
      ((uint8_t *) pkt) + FANET_HEADER_SIZE);
#endif

    altitude = ((pkt->altitude_msb << 8) | pkt->altitude_lsb);
    if (pkt->altitude_scale) {
      altitude = altitude * 4 /* -2 */;
    }
    fop->altitude = (float) altitude;

    fop->aircraft_type = AT_FROM_FANET(pkt->aircraft_type);

    fop->course = (float) pkt->heading * 360.0 / 256.0;

    speed_byte = pkt->speed;
    speed_int = (int) (speed_byte | (speed_byte & (1<<6) ? 0xFFFFFF80U : 0));

    if (pkt->speed_scale) {
      speed_int *= 5 /* -2 */;
    }
    fop->speed = ((float) speed_int) / (2 * _GPS_KMPH_PER_KNOT);

    climb_byte = pkt->climb;
    climb_int = (int) (climb_byte | (climb_byte & (1<<6) ? 0xFFFFFF80U : 0));

    if (pkt->climb_scale) {
      climb_int *= 5 /* +-2 */;
    }
    fop->vs = ((float)climb_int) * (_GPS_FEET_PER_METER * 6.0);

#if defined(FANET_NEXT)
    offset_byte = pkt->qne_offset;
    offset_int = (int) (offset_byte | (offset_byte & (1<<6) ? 0xFFFFFF80U : 0));

    if (pkt->qne_scale) {
      offset_int *= 4;
    }

    fop->pressure_altitude = fop->altitude + (float) offset_int;
#endif

    fop->stealth = 0;
    fop->no_track = !(pkt->track_online);
    if (settings->debug_flags & DEBUG_RELAY)  fop->no_track = 0;   // for testing

#if 0
    Serial.print(fop->addr, HEX);
    Serial.print(',');
    Serial.print(fop->aircraft_type, HEX);
    Serial.print(',');
    Serial.print(fop->latitude, 6);
    Serial.print(',');
    Serial.print(fop->longitude, 6);
    Serial.print(',');
    Serial.print(fop->altitude);
    Serial.print(',');
    Serial.print(fop->speed);
    Serial.print(',');
    Serial.print(fop->course);
    Serial.println();
    Serial.flush();
#endif
    fanet_name_set_type(fop->addr, pkt->aircraft_type);  /* 0-7 airborne */
    rval = true;

  } else if (pkt->ext_header == 0 && pkt->type == 2 ) {  /* Name */

    uint32_t addr = (pkt->vendor << 16) | pkt->address;
    if (addr == this_aircraft->addr)
        return false;

    uint8_t *body = ((uint8_t *) fanet_pkt) + FANET_HEADER_SIZE;
    size_t body_len = (RF_last_rx_len > FANET_HEADER_SIZE) ?
                       RF_last_rx_len - FANET_HEADER_SIZE : 0;
    if (body_len > 0) {
      if (body_len > FANET_NAME_MAX_LEN - 1)
        body_len = FANET_NAME_MAX_LEN - 1;
      char name[FANET_NAME_MAX_LEN];
      memcpy(name, body, body_len);
      name[body_len] = '\0';
      fanet_name_store(addr, name);
    }

    return false;  /* name packet is not traffic */

  } else if (pkt->ext_header == 0 && pkt->type == 7 ) {  /* Ground Tracking */

    /* use the same pkt pointer for header fields (shared layout with Type 1) */
    fop->addr     = (pkt->vendor << 16) | pkt->address;

    if (fop->addr == this_aircraft->addr)
        return false;

    fop->protocol = RF_PROTOCOL_FANET;
    fop->addr_type = ADDR_TYPE_FLARM;
    fop->timestamp = this_aircraft->timestamp;
    fop->gnsstime_ms = millis();

    payload_absolut2coord(&(fop->latitude), &(fop->longitude),
      ((uint8_t *) fanet_pkt) + FANET_HEADER_SIZE);

    uint8_t status = ((uint8_t *) fanet_pkt)[FANET_HEADER_SIZE + 6];

    fop->altitude = this_aircraft->altitude;  /* ground station, use own alt as estimate */
    fop->aircraft_type = AIRCRAFT_TYPE_UNKNOWN;
    fop->course = 0;
    fop->speed = 0;
    fop->vs = 0;
    fop->stealth = 0;
    fop->no_track = !(status & 0x01);
    if (settings->debug_flags & DEBUG_RELAY)  fop->no_track = 0;

    fanet_name_set_type(fop->addr, ((status >> 4) & 0x0F) + 10);  /* ground: 10-25 */
    rval = true;
  }

  return rval;
}

static uint32_t fanet_name_last_ms = 0;
uint32_t fanet_sos_last_ms  = 0;
uint8_t  fanet_sos_count    = 0;
static bool fanet_sos_cancel_pending = false;

#define FANET_SOS_INTERVAL_MS  30000  /* broadcast SOS message every 30 seconds */
#define FANET_SOS_MAX_COUNT    3      /* send SOS message this many times, then stop */

/* Call when leaving FANET_SOS_DISTRESS for any other state, so a one-shot
 * "Cancelled SOS" broadcast goes out - otherwise anyone who received the
 * distress message has no way to know it was a false alarm / resolved,
 * short of the tracking/ground packets simply stopping being distress. */
void fanet_sos_cancel(void)
{
  if (fanet_sos_state == FANET_SOS_DISTRESS)
    fanet_sos_cancel_pending = true;
}

static size_t fanet_type2_encode(void *fanet_pkt, container_t *this_aircraft) {

  const char *name = fnf_session_name[0] ? fnf_session_name : settings->fanet_name;
  if (name[0] == '\0')
    return 0;

  uint8_t *buf = (uint8_t *) fanet_pkt;
  uint32_t id = this_aircraft->addr;

  /* header (4 bytes) */
  buf[0] = (2 & 0x3F) | (1 << 6) | (0 << 7);  /* type=2, forward=1, ext_header=0 */
  buf[1] = (id >> 16) & 0xFF;                    /* vendor */
  buf[2] = id & 0xFF;                            /* address low */
  buf[3] = (id >> 8) & 0xFF;                     /* address high */

  /* name string */
  size_t len = strlen(name);
  if (len > MAX_PKT_SIZE - FANET_HEADER_SIZE)
    len = MAX_PKT_SIZE - FANET_HEADER_SIZE;
  memcpy(&buf[FANET_HEADER_SIZE], name, len);

  return FANET_HEADER_SIZE + len;
}

static size_t fanet_type3_encode(void *fanet_pkt, container_t *this_aircraft,
                                  const char *msg) {

  uint8_t *buf = (uint8_t *) fanet_pkt;
  uint32_t id = this_aircraft->addr;

  /* header (4 bytes) — broadcast, no extended header */
  buf[0] = (3 & 0x3F) | (1 << 6) | (0 << 7);  /* type=3, forward=1, ext_header=0 */
  buf[1] = (id >> 16) & 0xFF;
  buf[2] = id & 0xFF;
  buf[3] = (id >> 8) & 0xFF;

  /* body: subtype(1) + message string */
  buf[FANET_HEADER_SIZE] = 0x00;  /* subtype 0 = normal message */

  size_t len = strlen(msg);
  size_t max_msg = MAX_PKT_SIZE - FANET_HEADER_SIZE - 1;  /* -1 for subtype byte */
  if (len > max_msg) len = max_msg;
  memcpy(&buf[FANET_HEADER_SIZE + 1], msg, len);

  return FANET_HEADER_SIZE + 1 + len;
}

static size_t fanet_type7_encode(void *fanet_pkt, container_t *this_aircraft) {

  uint8_t *buf = (uint8_t *) fanet_pkt;
  uint32_t id = this_aircraft->addr;

  /* header (4 bytes) — same layout as Type 1 */
  buf[0] = (7 & 0x3F) | (1 << 6) | (0 << 7);  /* type=7, forward=1, ext_header=0 */
  buf[1] = (id >> 16) & 0xFF;                    /* vendor */
  buf[2] = id & 0xFF;                            /* address low */
  buf[3] = (id >> 8) & 0xFF;                     /* address high */

  /* body: lat(3) + lon(3) + status(1) */
  coord2payload_absolut(this_aircraft->latitude, this_aircraft->longitude,
    &buf[FANET_HEADER_SIZE]);

  uint8_t online = (this_aircraft->no_track ? 0 : 1);
  uint8_t gtype;
  if (fanet_ground_type != 0xFF)
    gtype = fanet_ground_type;           /* set by #FNG command from XCGuide */
  else
    gtype = (fanet_sos_state == FANET_SOS_DISTRESS) ? FANET_GROUND_TYPE_DISTRESS : FANET_GROUND_TYPE_LANDED_OK;
  buf[FANET_HEADER_SIZE + 6] = (gtype << 4) | online;

  return FANET_HEADER_SIZE + FANET_GROUND_BODY_SIZE;
}

static size_t fanet_type1_encode(void *fanet_pkt, container_t *this_aircraft) {

  uint32_t id = this_aircraft->addr;
  float lat = this_aircraft->latitude;
  float lon = this_aircraft->longitude;
  int16_t alt = (int16_t) this_aircraft->altitude;
  unsigned int aircraft_type =  this_aircraft->aircraft_type;
  float speed = this_aircraft->speed * _GPS_KMPH_PER_KNOT;
  float climb = this_aircraft->vs / (_GPS_FEET_PER_METER * 60.0);
  float heading = this_aircraft->course;
  float turnrate = 0;
  int16_t alt_diff = this_aircraft->pressure_altitude == 0 ? 0 :
          (int16_t) (this_aircraft->pressure_altitude - this_aircraft->altitude);

  fanet_packet_t *pkt = (fanet_packet_t *) fanet_pkt;

  pkt->ext_header     = 0;
  pkt->forward        = 1;
  pkt->type           = 1;  /* Tracking  */

  //pkt->vendor         = SOFRF_FANET_VENDOR_ID;
  pkt->vendor         = (id >> 16) & 0x00FF;   // will not be a constant
  pkt->address        = id & 0x00FFFF;

#if defined(FANET_DEPRECATED)
  pkt->latitude       = coord2payload_compressed(lat);
  pkt->longitude      = coord2payload_compressed(lon);
#else
  coord2payload_absolut(lat, lon, ((uint8_t *) pkt) + FANET_HEADER_SIZE);
#endif

  pkt->track_online   = (this_aircraft->no_track ? 0 : 1);
  pkt->aircraft_type  = AT_TO_FANET(aircraft_type);

  int altitude = constrain(alt, 0, 8190);
  pkt->altitude_scale = altitude > 2047 ? (altitude = (altitude + 2) / 4, 1) : 0;
  pkt->altitude_msb   = (altitude & 0x700) >> 8;
  pkt->altitude_lsb   = (altitude & 0x0FF);

  int speed2          = constrain((int)roundf(speed * 2.0f), 0, 635);
  if(speed2 > 127) {
    pkt->speed_scale  = 1;
    pkt->speed        = ((speed2 + 2) / 5);
  } else {
    pkt->speed_scale  = 0;
    pkt->speed        = speed2 & 0x7F;
  }

  int climb10         = this_aircraft->stealth ?
                        0 : constrain((int)roundf(climb * 10.0f), -315, 315);
  if(climb10 > 63) {
    pkt->climb_scale  = 1;
    pkt->climb        = ((climb10 + (climb10 >= 0 ? 2 : -2)) / 5);
  } else {
    pkt->climb_scale  = 0;
    pkt->climb        = climb10 & 0x7F;
  }

  pkt->heading        = constrain((int)roundf(heading * 256.0f)/360.0f, 0, 255);

  int turnr4          = constrain((int)roundf(turnrate * 4.0f), -255, 255);
  if(abs(turnr4) > 63) {
    pkt->turn_scale   = 1;
    pkt->turn_rate    = ((turnr4 + (turnr4 >= 0 ? 2 : -2)) / 4);
  } else {
    pkt->turn_scale   = 0;
    pkt->turn_rate    = turnr4 & 0x7F;
  }

#if defined(FANET_NEXT)
  int16_t offset      = constrain(alt_diff, -254, 254);
  if(abs(offset) > 63) {
    pkt->qne_scale    = 1;
    pkt->qne_offset   = ((offset + (offset >= 0 ? 2 : -2)) / 4);
  } else {
    pkt->qne_scale    = 0;
    pkt->qne_offset   = offset & 0x7F;
  }
#endif

  return sizeof(fanet_packet_t);
}

size_t fanet_encode(void *fanet_pkt, container_t *this_aircraft) {

  /* RFMODE bit 1: FANET TX — skip if disabled by XCGuide */
  if (!(fnf_rfmode & FNF_RFMODE_FANET_TX))
    return 0;

  uint32_t now = millis();

  /* One-shot "Cancelled SOS" broadcast, takes priority over everything else
   * so it goes out on the very next TX slot after distress is cleared. */
  if (fanet_sos_cancel_pending) {
    fanet_sos_cancel_pending = false;
    char msg[40];
    snprintf(msg, sizeof(msg), "Cancelled SOS %.5f,%.5f",
             this_aircraft->latitude, this_aircraft->longitude);
    return fanet_type3_encode(fanet_pkt, this_aircraft, msg);
  }

  /* Name packet (Type 2) is no longer sent instead of tracking -
   * it goes out on the priority lane, see fanet_name_tx() */

  /* Distress mode: alternate SOS message and DISTRESS tracking */
  if (fanet_sos_state == FANET_SOS_DISTRESS) {
    if (fanet_sos_count < FANET_SOS_MAX_COUNT &&
        (fanet_sos_last_ms == 0 ||
         (now - fanet_sos_last_ms) >= FANET_SOS_INTERVAL_MS)) {
      fanet_sos_last_ms = now;
      fanet_sos_count++;
      char sos_msg[72];
      /* Altitude included to help rescue in high-gradient terrain, where
       * lat/lon alone leave a large search area on a steep slope. */
      snprintf(sos_msg, sizeof(sos_msg), "SOS %s %.5f,%.5f,%.0fm",
               (fanet_ground_type == FANET_GROUND_TYPE_NEED_MED) ?
                   "need medical help" : "Pilot in DISTRESS",
               this_aircraft->latitude, this_aircraft->longitude,
               this_aircraft->altitude);
      return fanet_type3_encode(fanet_pkt, this_aircraft, sos_msg);
    }
    return fanet_type7_encode(fanet_pkt, this_aircraft);
  }

  /* If not airborne: only send Ground Tracking if confirmed landed */
  if (!this_aircraft->airborne) {
    if (fanet_sos_state == FANET_SOS_LANDED_OK)
      return fanet_type7_encode(fanet_pkt, this_aircraft);
    /* AIRBORNE or COUNTDOWN state → Type 1 */
    return fanet_type1_encode(fanet_pkt, this_aircraft);
  }

  /* Otherwise send normal Tracking (Type 1) */
  return fanet_type1_encode(fanet_pkt, this_aircraft);
}

/*
 * FANET neighbor table: every FANET source heard recently (any packet type).
 * Used to scale the tracking interval and to gate forwarding.
 */
#define FANET_NEIGHBOR_TABLE_SIZE  32
#define FANET_NEIGHBOR_TIMEOUT_MS  120000

typedef struct {
  uint32_t addr;
  uint32_t last_rx_ms;
  uint32_t last_fwd_ms;   /* when we last forwarded a position packet from this source */
  int16_t  alt_m;         /* altitude from its last Type 1 (Type 7 carries none) */
  bool     alt_valid;
} fanet_neighbor_t;

static fanet_neighbor_t fanet_neighbors[FANET_NEIGHBOR_TABLE_SIZE];

static fanet_neighbor_t *fanet_neighbor_find(uint32_t addr)
{
  uint32_t now = millis();
  for (int i = 0; i < FANET_NEIGHBOR_TABLE_SIZE; i++) {
    fanet_neighbor_t *nb = &fanet_neighbors[i];
    if (nb->addr == addr && (now - nb->last_rx_ms) < FANET_NEIGHBOR_TIMEOUT_MS)
      return nb;
  }
  return NULL;
}

static fanet_neighbor_t *fanet_neighbor_seen(uint32_t addr)
{
  uint32_t now = millis();
  int oldest = 0;
  uint32_t oldest_age = 0;
  for (int i = 0; i < FANET_NEIGHBOR_TABLE_SIZE; i++) {
    fanet_neighbor_t *nb = &fanet_neighbors[i];
    if (nb->addr == addr) {
      nb->last_rx_ms = now;
      return nb;
    }
    uint32_t age = (nb->addr == 0) ? 0xFFFFFFFF : (now - nb->last_rx_ms);
    if (age >= oldest_age) {
      oldest_age = age;
      oldest = i;
    }
  }
  fanet_neighbor_t *nb = &fanet_neighbors[oldest];
  nb->addr        = addr;
  nb->last_rx_ms  = now;
  nb->last_fwd_ms = 0;
  nb->alt_valid   = false;
  return nb;
}

uint8_t fanet_neighbor_count(void)
{
  uint32_t now = millis();
  uint8_t n = 0;
  for (int i = 0; i < FANET_NEIGHBOR_TABLE_SIZE; i++) {
    if (fanet_neighbors[i].addr != 0
        && (now - fanet_neighbors[i].last_rx_ms) < FANET_NEIGHBOR_TIMEOUT_MS)
      n++;
  }
  return n;
}

/* Tracking interval per FANET author guidance: floor((neighbors/10 + 1) * 5s) */
uint32_t fanet_tx_interval_ms(void)
{
  uint32_t n = fanet_neighbor_count();
  return ((5 * (n + 10)) / 10) * 1000;
}

/*
 * FANET forwarding (FANET-only mode), per FANET author guidance + Issue #7
 * rules. "Below" means how far the sender is below us; a relay is only
 * useful when it is higher than the sender.
 *  - only packets with the forward bit set; relay 1:1 with forward bit
 *    cleared (hop limit 2). Exception: a Type 0 ACK between a message pair
 *    we relayed (its sender clears the bit)
 *  - cancel if another node's relay of the same packet is heard first
 *  - Type 1: only if > 10 km away OR >= 1000 m below us; none above 10
 *    neighbors; with 5+ neighbors once per minute per source; otherwise
 *    once every 10s, with a per-source lockout while other sources compete
 *  - Type 7: distress (13-15) if >= 100 m below, need ride / landed OK (8, 9)
 *    if >= 300 m below, any distance; once per minute per source; others not
 *    forwarded. Type 7 has no altitude - the sender's last Type 1 altitude is
 *    used, unknown counts as a pass
 *  - Type 3: unicast only, to a known neighbor, sender >= 500 m below (unknown
 *    counts as a pass). The pair is then remembered for 5 minutes, and its
 *    replies and ACKs are forwarded regardless of height / neighbor
 *  - messages, ACKs and distress take the slot from a pending position forward
 *  - other types (e.g. Type 2 Name, broadcast messages) are not forwarded
 */
#define FANET_FWD_MAX_NEIGHBORS     10      /* Type 1: none above this */
#define FANET_FWD_BUSY_NEIGHBORS    5       /* Type 1: from here once/min/source */
#define FANET_FWD_MIN_INTERVAL_MS   10000
#define FANET_FWD_BUSY_INTERVAL_MS  60000
#define FANET_FWD_GROUND_INTERVAL_MS 60000
#define FANET_FWD_MSG_INTERVAL_MS   2000
#define FANET_FWD_SRC_LOCKOUT_MS    30000
#define FANET_FWD_AIR_DIST_M        10000
#define FANET_FWD_AIR_BELOW_M       1000
#define FANET_FWD_DISTRESS_BELOW_M  100
#define FANET_FWD_LANDED_BELOW_M    300
#define FANET_FWD_MSG_BELOW_M       500
#define FANET_FWD_PAIRS             4
#define FANET_FWD_PAIR_TIMEOUT_MS   300000
#define FANET_FWD_ACK_DEDUP_MS      5000
/* forward delay = DELAY_MIN + up to RANK + up to JITTER ms of usable TX time
 * (in FANET-only slotted mode only Slot 1 time counts, ~530 ms per second).
 * The better placed relay (higher above / farther from the sender) gets less
 * of RANK, so it usually goes first and the others hear it and cancel. */
#define FANET_FWD_DELAY_MIN_MS      100
#define FANET_FWD_RANK_MS           700
#define FANET_FWD_JITTER_MS         400
#define FANET_FWD_RANK_BELOW_M      2000    /* this far below = best rank */
#define FANET_FWD_RANK_DIST_M       20000   /* Type 1: this far away = best rank */
#define FANET_FWD_STALE_MS          3500

/* message pairs we relayed a Type 3 between (either direction) */
typedef struct {
  uint32_t a, b;
  uint32_t last_ms;       /* last relayed message, 0 = free */
  uint32_t ack_fwd_ms;    /* last relayed (or overheard relayed) ACK */
} fanet_fwd_pair_t;

static fanet_fwd_pair_t fwd_pairs[FANET_FWD_PAIRS];

static fanet_fwd_pair_t *fanet_pair_find(uint32_t x, uint32_t y)
{
  uint32_t now = millis();
  for (int i = 0; i < FANET_FWD_PAIRS; i++) {
    fanet_fwd_pair_t *p = &fwd_pairs[i];
    if (p->last_ms != 0 && (now - p->last_ms) < FANET_FWD_PAIR_TIMEOUT_MS
        && ((p->a == x && p->b == y) || (p->a == y && p->b == x)))
      return p;
  }
  return NULL;
}

static void fanet_pair_add(uint32_t x, uint32_t y)
{
  uint32_t now = millis();
  fanet_fwd_pair_t *p = fanet_pair_find(x, y);
  if (p == NULL) {
    int oldest = 0;
    uint32_t oldest_age = 0;
    for (int i = 0; i < FANET_FWD_PAIRS; i++) {
      uint32_t age = (fwd_pairs[i].last_ms == 0) ? 0xFFFFFFFF : (now - fwd_pairs[i].last_ms);
      if (age >= oldest_age) {
        oldest_age = age;
        oldest = i;
      }
    }
    p = &fwd_pairs[oldest];
    p->a = x;
    p->b = y;
    p->ack_fwd_ms = 0;
  }
  p->last_ms = now;
}

/* how far the source is below us (m); false if its or our altitude is unknown */
static bool fanet_fwd_below(const fanet_neighbor_t *nb, int32_t *below)
{
  if (! nb->alt_valid || ! isValidFix())
    return false;
  *below = (int32_t) ThisAircraft.altitude - nb->alt_m;
  return true;
}

static uint8_t  fwd_buf[MAX_PKT_SIZE];
static size_t   fwd_len       = 0;
static uint32_t fwd_src       = 0;
static uint32_t fwd_dest      = 0;      /* unicast destination, 0 = broadcast */
static bool     fwd_urgent    = false;  /* message / ACK / distress */
static uint32_t fwd_queued_ms = 0;
static uint32_t fwd_due_ms    = 0;
static uint32_t fwd_last_ms   = 0;      /* last position (Type 1/7) forward */
static uint32_t fwd_msg_last_ms = 0;    /* last message (Type 3) forward */
static uint32_t fwd_contention_ms = 0;  /* last time a different source lost out */

/* same packet as the pending forward, ignoring the forward bit */
static bool fanet_fwd_same(const uint8_t *raw, size_t len)
{
  return (len == fwd_len
       && (raw[0] & 0xBF) == (fwd_buf[0] & 0xBF)
       && memcmp(raw + 1, fwd_buf + 1, len - 1) == 0);
}

/* Called for every received FANET packet (raw bytes, actual length) */
void fanet_rx_forward(const uint8_t *raw, size_t len, int8_t rssi)
{
  if (len < FANET_HEADER_SIZE || len > MAX_PKT_SIZE)
    return;

  uint32_t src = ((uint32_t)raw[1] << 16) | ((uint32_t)raw[3] << 8) | raw[2];
  if (src == ThisAircraft.addr)
    return;

  fanet_neighbor_t *nb = fanet_neighbor_seen(src);
  bool    fwd_bit = (raw[0] >> 6) & 0x01;
  bool    ext     = (raw[0] & 0x80);
  uint8_t type    = raw[0] & 0x3F;

  /* remember the altitude of every Type 1 source - Type 3 and Type 7
   * carry none, so their height checks use this */
  if (type == 1 && ! ext && len >= FANET_HEADER_SIZE + 8) {
    const uint8_t *p = raw + FANET_HEADER_SIZE;
    int alt = ((p[7] & 0x07) << 8) | p[6];
    if (p[7] & 0x08)
      alt *= 4;
    nb->alt_m     = (int16_t) alt;
    nb->alt_valid = true;
  }

  /* unicast destination, if any */
  bool     unicast = ext && len >= 8 && (raw[4] & 0x20);
  uint32_t dest    = unicast ?
           (((uint32_t)raw[5] << 16) | ((uint32_t)raw[7] << 8) | raw[6]) : 0;

  if (fwd_len > 0 && fanet_fwd_same(raw, len)) {
    if (! fwd_bit) {
      Serial.printf("FWD: dropped (overheard) type %d from %06X\r\n",
                    (int) type, (unsigned) src);
      fwd_len = 0;
      if (type == 0) {              /* that ACK is relayed: don't relay it again */
        fanet_fwd_pair_t *pair = fanet_pair_find(src, dest);
        if (pair)
          pair->ack_fwd_ms = millis();
      }
    }
    return;    /* or a repeat of the original we already queued */
  }

  /* forwarding only in FANET-only mode */
  if (dual_protocol != RF_SINGLE_PROTOCOL || settings->rf_protocol != RF_PROTOCOL_FANET)
    return;
  if (!(fnf_rfmode & FNF_RFMODE_FANET_TX) || settings->txpower == RF_TX_POWER_OFF)
    return;

  if (unicast && dest == ThisAircraft.addr)
    return;                         /* addressed to us */
  fanet_fwd_pair_t *pair = unicast ? fanet_pair_find(src, dest) : NULL;

  /* forward bit required - except for an ACK of a message pair we relayed,
   * since its sender always clears the bit */
  if (type == 0 ? (pair == NULL) : ! fwd_bit)
    return;

  uint32_t now = millis();
  uint8_t  n   = fanet_neighbor_count();
  int32_t  below = 0;
  bool     below_known = fanet_fwd_below(nb, &below);
  const char *why = NULL;
  bool lost_out = false;     /* eligible, but the relay is taken */
  bool urgent   = false;     /* may take the slot from a pending position */
  float rank    = 0.5f;      /* 0..1, 1 = best placed relay; unknown = middle */
  if (below_known)
    rank = (float) below / FANET_FWD_RANK_BELOW_M;

  switch (type) {
  case 0:                           /* ACK between a relayed message pair */
    urgent = true;
    if (pair->ack_fwd_ms != 0 && (now - pair->ack_fwd_ms) < FANET_FWD_ACK_DEDUP_MS)
      why = "ack-dup";
    break;

  case 3:                           /* message */
    urgent = true;
    if (! unicast)
      why = "broadcast";
    else if (pair == NULL && fanet_neighbor_find(dest) == NULL)
      why = "dest not neighbor";
    else if (pair == NULL && below_known && below < FANET_FWD_MSG_BELOW_M)
      why = "height";
    else if (fwd_msg_last_ms != 0 && (now - fwd_msg_last_ms) < FANET_FWD_MSG_INTERVAL_MS)
      why = "msg-rate";
    break;

  case 7: {                         /* ground tracking */
    if (ext || len < FANET_HEADER_SIZE + FANET_GROUND_BODY_SIZE) {
      why = "format";
      break;
    }
    uint8_t state = raw[FANET_HEADER_SIZE + 6] >> 4;
    bool distress = (state >= FANET_GROUND_TYPE_NEED_MED);
    urgent = distress;
    if (distress) {
      if (below_known && below < FANET_FWD_DISTRESS_BELOW_M)
        why = "height";
    } else if (state == FANET_GROUND_TYPE_NEED_RIDE || state == FANET_GROUND_TYPE_LANDED_OK) {
      if (below_known && below < FANET_FWD_LANDED_BELOW_M)
        why = "height";
    } else {
      why = "ground state";
    }
    if (why == NULL && nb->last_fwd_ms != 0
        && (now - nb->last_fwd_ms) < FANET_FWD_GROUND_INTERVAL_MS)
      why = "src-rate";
    else if (why == NULL && ! distress && fwd_last_ms != 0
             && (now - fwd_last_ms) < FANET_FWD_MIN_INTERVAL_MS) {
      why = "rate-limit";
      lost_out = true;
    }
    break;
  }

  case 1: {                         /* airborne tracking */
    if (ext || len < FANET_HEADER_SIZE + 8) {
      why = "format";
      break;
    }
    if (n > FANET_FWD_MAX_NEIGHBORS) {
      why = "neighbors";
      break;
    }
    if (! below_known) {            /* we have no fix */
      why = "no fix";
      break;
    }
    float lat, lon;
    payload_absolut2coord(&lat, &lon, (uint8_t *) raw + FANET_HEADER_SIZE);
    float y = 111300.0f * (lat - ThisAircraft.latitude);
    float x = 111300.0f * (lon - ThisAircraft.longitude) * CosLat();
    float dist = hypotf(x, y);
    if (dist <= FANET_FWD_AIR_DIST_M && below < FANET_FWD_AIR_BELOW_M) {
      why = "near";
      break;
    }
    if (dist / FANET_FWD_RANK_DIST_M > rank)
      rank = dist / FANET_FWD_RANK_DIST_M;
    if (n >= FANET_FWD_BUSY_NEIGHBORS) {
      if (nb->last_fwd_ms != 0 && (now - nb->last_fwd_ms) < FANET_FWD_BUSY_INTERVAL_MS)
        why = "src-rate";
    } else if (fwd_contention_ms != 0 && (now - fwd_contention_ms) < FANET_FWD_SRC_LOCKOUT_MS
          && nb->last_fwd_ms != 0 && (now - nb->last_fwd_ms) < FANET_FWD_SRC_LOCKOUT_MS) {
      why = "src-lockout";   /* only while other sources compete for the relay */
    }
    if (why == NULL && fwd_last_ms != 0 && (now - fwd_last_ms) < FANET_FWD_MIN_INTERVAL_MS) {
      why = "rate-limit";
      lost_out = true;
    }
    break;
  }

  default:
    return;                         /* other types are not forwarded */
  }

  /* one pending slot: an urgent packet takes it from a pending position */
  if (why == NULL && fwd_len > 0) {
    if (urgent && ! fwd_urgent) {
      Serial.printf("FWD: replaced pending type %d from %06X by type %d\r\n",
                    (int) (fwd_buf[0] & 0x3F), (unsigned) fwd_src, (int) type);
    } else {
      why = "busy";
      lost_out = ! urgent;
    }
  }

  if (why) {
    /* a different source wanted the relay but lost out: from now on take turns */
    if (lost_out && src != fwd_src)
      fwd_contention_ms = now;
    Serial.printf("FWD: dropped (%s) type %d from %06X rssi %d nb %d below %d%s\r\n",
                  why, (int) type, (unsigned) src, (int) rssi, (int) n,
                  (int) below, below_known ? "" : "?");
    return;
  }

  memcpy(fwd_buf, raw, len);
  fwd_buf[0]   &= ~0x40;            /* clear forward bit: hop limit 2 */
  fwd_len       = len;
  fwd_src       = src;
  fwd_dest      = dest;
  fwd_urgent    = urgent;
  fwd_queued_ms = now;
  if (rank < 0.0f) rank = 0.0f;
  if (rank > 1.0f) rank = 1.0f;
  uint32_t usable = (uint32_t) ((1.0f - rank) * FANET_FWD_RANK_MS)
                  + SoC->random(0, FANET_FWD_JITTER_MS);
  fwd_due_ms    = RF_Priority_Spread(FANET_FWD_DELAY_MIN_MS, usable);
  Serial.printf("FWD: queued type %d from %06X rssi %d nb %d below %d%s rank %d%% in %d ms\r\n",
                (int) type, (unsigned) src, (int) rssi, (int) n,
                (int) below, below_known ? "" : "?",
                (int) (rank * 100), (int) (fwd_due_ms - now));
}

/* Priority lane: transmit the pending forward when due. Returns true if sent. */
bool fanet_forward_tx(void)
{
  if (fwd_len == 0)
    return false;
  if ((millis() - fwd_queued_ms) > FANET_FWD_STALE_MS) {
    Serial.printf("FWD: dropped (stale) from %06X\r\n", (unsigned) fwd_src);
    fwd_len = 0;
    return false;
  }
  if (! RF_TIME_REACHED(fwd_due_ms) || ! RF_Priority_Ready(true))
    return false;

  memcpy(TxBuffer, fwd_buf, fwd_len);
  size_t len = fwd_len;
  fwd_len = 0;                      /* clear before transmit so re-entry is safe */
  RF_Transmit_Priority(len, true);  /* exact: due time already spread */
  uint32_t now  = millis();
  uint8_t  type = fwd_buf[0] & 0x3F;
  if (type == 3) {
    fwd_msg_last_ms = now;          /* messages do not use up the position limits */
    fanet_pair_add(fwd_src, fwd_dest);  /* let replies and ACKs back for 5 min */
  } else if (type == 0) {
    fanet_fwd_pair_t *pair = fanet_pair_find(fwd_src, fwd_dest);
    if (pair)
      pair->ack_fwd_ms = now;
  } else {
    fwd_last_ms = now;
    fanet_neighbor_t *nb = fanet_neighbor_find(fwd_src);
    if (nb)
      nb->last_fwd_ms = now;
  }
  Serial.printf("FWD: sent type %d from %06X len %d, %d ms after RX\r\n",
                (int) type, (unsigned) fwd_src, (int) len,
                (int) (now - fwd_queued_ms));
  return true;
}

/* Priority lane: Name (Type 2) every FANET_NAME_INTERVAL_MS. Returns true if sent. */
bool fanet_name_tx(void)
{
  if (!(fnf_rfmode & FNF_RFMODE_FANET_TX))
    return false;
  if (!fnf_session_name[0] && settings->fanet_name[0] == '\0')
    return false;
  if ((millis() - fanet_name_last_ms) < FANET_NAME_INTERVAL_MS)
    return false;
  if (! RF_Priority_Ready())
    return false;

  size_t s = fanet_type2_encode((void *) TxBuffer, &ThisAircraft);
  if (s == 0)
    return false;
  fanet_name_last_ms = millis();
  RF_Transmit_Priority(s);
  Serial.println("FN_TX: transmitted Type 2 name");
  return true;
}
 