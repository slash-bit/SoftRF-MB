# SoftRF Under the Hood

*Advanced Documentation for Moshe Braner's version of SoftRF*

**By Moshe Braner**

**Last updated:** January 16, 2026 (software version MB173)

**Latest version:** https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/documentation

---

## Table of Contents

- [Advanced Settings](#advanced-settings)
- [Add-Ons & Hardware Mods](#add-ons)
- [Tips and Tricks](#tips-and-tricks)
- [Details of Operation](#details-of-operation)
- [Major Changes](#description-of-the-major-changes-made-in-this-version)
- [Known Issues](#known-issues)
- [Compiling SoftRF](#compiling-softrf)

---

## Advanced Settings

### id_method - Address Type Declaration

Declares the 6-hex-digit transmitted ID to be either:

- **"1"** = Aircraft ICAO ID (preferred when available)
- **"2"** = Device ID (now always starts with "8")
- **"0"** = Random (changes every few minutes)
- **"3"** = Anonymous (random but fixed for flight)
- **"7"** = Override - transmit entered Aircraft ID marked as device ID

#### When to Use Each

**Best practice:** Use ICAO ID (option "1") as not all aircraft have ICAO ID, and if device ID is duplicate of someone else's registered ID, you can:
1. Enter your old (or any) device ID in "Aircraft ID" setting
2. Select "7" (override) as id_method
3. Transmit that ID marked as device ID rather than ICAO

**OGNTP Protocol note:** If selecting "2" (device ID) or "7" (override) with OGNTP main protocol, the transmitted "address type" becomes "3" to fit protocol.

**Important:** Do not choose "3" in id_method setting - will transmit random "anonymous" ID instead of registered device ID.

### ignore_id - Aircraft ID to Ignore

Enter aircraft (or device) ID you do **NOT** want included in traffic data and collision warnings.

**Example use cases:**
- Towplane with FLARM (avoid unnecessary collision warnings during tow)
- Leave as 000000 to disable this feature

**Auto-ignore rule:** Signals from aircraft ID identical to this aircraft's ID always ignored - no need to enter here.

**Multi-device same aircraft:**
- Set both SoftRF and/or FLARM devices to same ICAO ID
- SoftRF ignores the other device
- (Should probably also set one device to not transmit)

**ADS-B receiver consideration:** If aircraft has transponder transmitting same ID as SoftRF, SoftRF ignores own transponder.

### follow_id - Aircraft ID to Follow

Enter aircraft (or device) ID to prioritize in traffic reports.

**Example:** Flying buddy in another aircraft

**Feature behavior:**
- Leave as 000000 to disable
- SoftRF tracks up to 8 aircraft, reports up to 6
- Normally selects closest aircraft
- Followed aircraft shown even if not among closest
- **Collision danger exception:** Aircraft triggering collision warning takes priority

### tx_power - Transmission Power

Only accessible via editing settings.txt file.

**Values:**
- **2** = Full transmission power (default) - 100 mW or less depending on regional legal limit and radio chip safety, 8 mW for relay-only mode
- **1** = Low power (2 mW) - for short-range testing
- **0** = Receive-only (no transmissions) - ground stations or second device in same aircraft

#### When to Use

- **Full power (2):** Normal operation. Activate "Stealth mode" if want limit visibility while retaining collision avoidance.
- **Low power (1):** Short-range testing
- **Receive-only (0):** Ground station operation or second device in same aircraft

**Stealth mode note:** To avoid being tracked by ground stations, activate "no_track" setting.

**Backward compatibility:** Old settings.txt files may have "txpower" (different from "tx_power") with obsolete coding (2=off, 0=full power).

### hrange and vrange

- **hrange** = Horizontal reporting range in km
- **vrange** = Vertical reporting range in hundreds of meters

### pointer

LED ring direction for display such as "FLARMview":
- **1** = Track up
- **2** = North up

### relay

Relaying mode for other aircraft data:
- **0** = Off
- **1** = Relay landed-out (opt-in)
- **2** = Relay "all" traffic
- **3** = Relay-only mode

See "Air Relay" section below for details.

### pflaa_cs

PFLAA sentence configuration:
- **1** (default) = Include "callsign" after hex ID
- **0** = Exclude callsign

### expire

Number of seconds to keep reporting traffic not heard from:
- **Range:** 1-30 seconds
- **Default:** 5 seconds
- **Note:** Display device may keep reporting longer (e.g., XCsoar for 5+ more seconds)

### altprotocol

Alternative transmission protocol once every 4 seconds:
- **255** (default) = None
- **0** = Legacy
- **1** = OGNTP
- **7** = Latest
- **8** = ADS-L

#### Conditions
- Only possible if main protocol is also 0, 1, 7, or 8
- **Motivation:** Better Forward Error Correction in OGNTP for effective long-distance ground station tracking, or add ADS-L e-conspicuity

#### Special Cases (MB172+)
- **altprotocol=1 with relay enabled:** Landed-out traffic relayed in both protocols, taking priority over self-reporting once per 16 seconds

**Dual-protocol reception (MB172+):**
- If main protocol is "Latest" and altprotocol is ADS-L (or vice versa): Dual-protocol reception - receive both FLARM and ADS-L simultaneously
- If altprotocol is OGNTP: Same FLARM+ADS-L dual reception, altprotocol transmissions alternate between ADS-L and OGNTP ("two and a half protocols")
- **No OGNTP reception** unless it's the main protocol (then no FLARM or ADS-L reception)

### nmea_out and nmea_out2

NMEA output destination codes:
- **0** = Off
- **1** = Serial
- **2** = UDP
- **3** = TCP
- **4** = USB
- **5** = Bluetooth
- **6** = Secondary serial

**Device limitations:** Not all destinations available on all devices (e.g., T-Echo has BT but not UDP/TCP/secondary serial).

### nmea_g and nmea2_g - GNSS Sentences

Bitfield (two hexadecimal digits):
- **1** = Basic (GGA+RMC)
- **2** = GSA
- **3** = Basic + GSA
- **4** = GST
- **8** = GSV
- **F** = All

### nmea_s and nmea2_s - Sensor Sentences

Bitfield:
- **1** = Basic (PGRMZ)
- **2** = LK8EX1
- **3** = Both (basic + LK8EX1)

### nmea_t and nmea2_t - Traffic Sentences

Bitfield:
- **1** = Basic (PFLAU + PFLAA)
- **8** = PFLAJ
- **9** = Both (basic + PFLAJ)

### nmea_e and nmea2_e - External Sentences

Bitfield:
- **1** = "Tunnel" from connected devices
- **2** = Generated (strobe, alarm)

### nmea_d and nmea2_d - Debug Output

24-bit bitfield - see values in Settings.h

### nmea_p and nmea2_p

Debug output from original SoftRF. This version added nmea_d.

### baud_rate

Baud rate code for main serial/USB port:
- **0** = Default (38400)
- **1** = 4800
- **2** = 9600
- **3** = 19200
- **4** = 38400
- **5** = 57600
- **6** = 115200

### gdl90

GDL90 output (different local data transfer protocol):
- Usually leave Off (0)
- If used: should not choose same destination as NMEA output

### d1090

Dump1090 output (another local data protocol):
- Usually leave Off (0)
- If used: should not choose same destination as NMEA output

### gdl90_in

GDL90 input (import data from external ADS-B receiver):
- Usually leave Off
- GDL90 uses UDP port 4000, NMEA uses 10110
- Cannot use same port for both input and output

**Values:** Same codes as nmea_out (0-6)

### logflight - Flight Logging

Options:
- **0** = Off
- **1** = Always
- **2** = When airborne
- **3** = When airborne and also report near traffic

### loginterval

Interval (seconds) between flight log position records:
- **Range:** 1-155 seconds

### compflash - Flash Compression

- **T-Beam with SD card:** Setting ignored
- **T-Beam without SD card:**
  - **0** = Log to RAM only
  - **1** = Also compress to IGZ file in flash
- **T-Echo:**
  - **0** = Log into uncompressed IGC file
  - **1** = Log into compressed IGZ file

### alarmlog - Alarms Log

If enabled (1), every collision alarm writes a line to alarmlog.txt file (in flash):
- **Content:** Date, time, lat/lon, alarm level (1-3), aircraft ID, relative heading, horizontal and vertical separation (meters)
- **T-Beam download:** "Alarm Log: [Download]" button on web interface status page
- **Existing file:** Normally appended to
- **Low space:** File erased and new one created
- **Manual clear:** "Alarm Log: [Clear]" button on status page
- **Flight logging:** Similar data also inserted as LPLT comment if flight logging enabled

### log_nmea

Option to log all NMEA output to SD card (T-Beam only):
- Logging starts when GNSS fix attained
- Uses date for file naming (similar to IGC flight log)
- Example: 52B1NMEA.txt

### gn_to_gp

Default = 0

Set to 1 to convert NMEA output GNSS sentences from $GN/$GA/$GL to $GP. Some instruments receiving these only process $GP___.

Only accessible via editing settings.txt file.

### leapsecs - Leap Seconds

Only accessible via editing settings.txt file.

- Used if GNSS module says "have a fix, but leap seconds not known"
- **Default:** 18 (correct for 2025)
- **Auto-update:** Automatically set to what GNSS module reports after receiving from satellite
- **Should never need manual edit**
- **Relevance:** Currently only relevant to Ublox GNSS modules (T-Beam)

### geoid - Geoid Separation

Only accessible via editing settings.txt file.

Most users can ignore this. Some GNSS modules don't report geoid separation (e.g., old T-Beam v0.7).

**If missing geoid data:**
- Status web page shows "Altitude above ellipsoid - MSL n.a."
- No effect on SoftRF "Latest" protocol (ellipsoid-based altitude)
- Affects MSL GNSS altitude to connected glide computers and other protocol transmissions

#### Solutions

**Option 1 - Manual entry:**
- Enter integer between -104 and +84 (meters)
- If 0 is correct, enter 1

**Option 2 - Geoid file:**
- Leave setting as zero
- Upload "egm96s.dem" file using button in status page
- SoftRF looks up needed number by location
- File available: https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/source/libraries/Geoid/egm96s.dem

---

## On the T-Beam Only

### altpin0 - Serial Port Input Pin

Default (altpin0,0) uses "RX" pin connected to USB interface.

**Alternatives:**
- (altpin0,1) = Use "VP" pin instead, disassociating serial input from USB
- (altpin0,25) = Any available pin, e.g., pin 25

Or use secondary serial port instead.

### baudrate2 - Secondary Serial Port Baud Rate

Baud rate code for secondary serial port (pins 39 "VN" & 4):
- **0** = Off
- **2-6** = Same codes as main baud_rate

### invert2 - Aux/Secondary Serial Port Logic

Auxiliary/Secondary Serial Port Logic:
- **"Normal"** = Connect to TTL devices or RS232 conversion chip
- **"Inverted"** = Connect to RS232 devices without conversion chip (output voltage may be insufficient - see data bridging section in user guide)

### power_save

- **1** = Turn WiFi off after 10 minutes if no client connected (saves battery)
- **0** (default) = Keep WiFi on (e.g., sending data throughout flight)

### power_ext - External Power Shutdown

- **1** = Automatically shut down when conditions met (see below)
- **0** (default) = Disabled

**Shutdown conditions (all must be met):**
1. External power was disconnected
2. Battery voltage below 3.9V
3. Device operating for at least 1 hour
4. Aircraft not airborne

**Device support:** Currently T-Beam v1.x only

**Side effect:** Prevents T-Beam from entering "charging mode" instead of fully booting when both USB and battery power present.

**Relevance:** Only relevant if internal battery installed. Irrelevant otherwise.

### rx1090 - Internal ADS-B Receiver

Activate use of GNS5892 ADS-B receiver module if installed.

See user guide section about connecting ADS-B receiver.

### mode_s - Mode S Messages (from GNS5892)

GNS5892 can receive Mode S (but not Mode C) transponder messages:
- Report altitude only, not position
- Non-directional alarm based on signal strength possible

**Values:** Nonzero to activate processing

**Traffic area consideration:** In high traffic areas may interfere with 868/915 MHz radio processing. Keep off ("0") unless area has lots of Mode S traffic lacking ADS-B output.

#### Auto-Calibration (MB149+)

Signal strength thresholds auto-calibrate based on ADS-B traffic:
- Requires 100 ADS-B position samples under 1km distance to collect enough data
- Until sufficient data collected: default thresholds used
- To reset data and restart collection: delete "rssidist.txt" file (e.g., after antenna change)

#### Threshold Adjustment (MB153+)

Value (1-9) adjusts default thresholds:
- Higher value = alarms for weaker signals
- **Try "3" first** unless very poor antenna
- Does not change actual circuit gain - only RSSI number interpretation
- Defaults have no effect once sufficient auto-calibrate data collected

### rx1090x - GNS5892 Comparator Offset

Default = 100

Can affect weak signal reception. Default fine for nearby traffic awareness.

### hrange1090 and vrange1090

Horizontal and vertical reporting ranges for ADS-B traffic:
- **hrange1090** = Horizontal in km
- **vrange1090** = Vertical in hundreds of meters

### strobe - LED/Strobe Control

LED or strobe connection to T-Beam - choose mode:
- **0** = Strobe off - never flashes
- **1** = Strobe alarm - flashes only during collision alarm
- **2** = Strobe airborne - flashes whenever airborne, more frequent if alarm
- **3** = Strobe always - flashes even if not airborne

### voice - Voice Warnings

Voice Warnings output:
- **0** = Off
- **1** = Internal DAC (analog output via pin 25)
- **2** = External I2S (pins 14, 15, 25)

**Note:** External I2S precludes buzzer use on pins 14 and 15.

### ssid - External WiFi Network

Connect to external WiFi network after booting:
- **If specified:** SoftRF tries to connect
- **If unsuccessful (10 seconds):** Creates own WiFi network
- **If SSID blank:** Skip 10-second delay, create internal network immediately (password ignored)
- **If password blank but SSID not:** Use entered SSID as custom name for SoftRF network (same as "myssid" setting)

**External network IP:** Chosen by router, seen in third OLED page (press button to cycle).

### psk - External WiFi Password

Password for external WiFi network (displays as "******" - doesn't show true password length).

### myssid - WiFi Network Name

SSID for WiFi network created by SoftRF:
- Also used as Bluetooth name
- If blank: Based on chip ID
- For internal network: IP always 192.168.1.1, password always 12345678

### tcpmode - TCP Mode

- **0** = Server (default) - expected by XCsoar, Tophat
- **1** = Client - for XCvario connection

### host_ip - Host IP (TCP Client)

IP address for TCP client connection (e.g., 192.168.4.1).

Only affects TCP client connection.

### tcpport - Host Port (TCP Client)

Port for TCP client connection:
- **2000** = Default
- **8880** = Option for stock XCvario firmware

**Current version (MB09r+):** Both TCP and UDP simultaneously as primary/secondary NMEA destinations.

**Purpose:** Wireless connection to XCvario (and XCsoar simultaneously).

### alt_udp - UDP Port for NMEA Output

Choose port:
- **10110** = Default
- **10111** = Alternative

### rfc - Radio Frequency Correction

Correction ±up to 30 KHz for sx1276 only.

Only accessible via editing settings.txt file.

Only use if reliable information (e.g., from OGN ground stations) indicates device is slightly off-frequency.

### gnss_pins - GNSS Module Connection

Which pins GNSS module connected to:
- **0** = Internal GNSS (default)
- **1** = Pins 39 & 4 (blocks auxiliary serial)
- **2** = Pins 13 & 2 (blocks BMP/OLED on those pins; use 21,22 instead)
- **3** = Pins 15 & 14 (blocks buzzer; 25 on T-Beam v0.7)

See section about connecting other hardware for details.

### ppswire - Added PPS Wire

Normally leave as 0 (none).

Two purposes:

1. **External GNSS module:** If connected including PPS signal wire to T-Beam "VP" pin:
   - ppswire,1 - use "VP" pin for PPS
   - ppswire,25 - use pin 25 for PPS
   - Enables more precise timing

2. **T-Beam v0.7 internal GNSS:** Unlike v1.x, v0.7 doesn't have built-in PPS connection:
   - ppswire,1 indicates hardware modification added PPS connection

### sd_card - SD Card Adapter Configuration

If SD adapter added to T-Beam, change from 0 to code matching pin connection:
- **0** = No SD card
- **1** = Pins 13,25,2,0
- **2** = Pins 13,VP,2,0
- **3** = Pins 5,19,27,0 (radio module pins + 0 for CS)

---

## On T-Echo Only

### epd_units

Display units:
- **0** = Metric
- **1** = Imperial
- **2** = Mixed

### epd_zoom

Default zoom level

### epd_rotate

Display direction (upside down, sideways, etc.)

### epd_orient

Track up or North up

### epd_adb

Aircraft database type

### epd_idpref

Which field(s) in database to display in text page:
- **0** = Registration
- **1** = Tail
- **2** = Model
- **3** = Type

### epd_vmode

Default view mode:
- **0** = Status
- **1** = Radar
- **2** = Text

### epd_aghost - Automatic Anti-Ghosting

Screen-clearing frequency:
- **0** = Off
- **1** = Auto
- **2** = 2 minutes
- **3** = 5 minutes

### epd_team

See Linar's T-Echo documentation

---

## Add-Ons

Some external connection combinations impossible due to limited I/O pins on T-Beam board.

**See the "Interacting_Options" table** (.xlsx and .pdf) in documentation folder:
https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/documentation

### Adding an SD Card Adapter

#### SPI Connection Requirements

SD adapters connect via "SPI" requiring 4 GPIO pins (plus power/ground).

**Pin choices:**
- 13,25,2,0
- 13,VP,2,0
- 5,19,27,0 (radio module pins)

Pin order: **SCK, MISO, MOSI, SS/CS**

(Pins differ for older v0.7 T-Beam)

See "Interacting_Options" file for pin interaction matrix.

#### Unbuffered vs. Buffered Adapters

##### Unbuffered Adapters

- SD card pins directly connected to ESP32 pins
- Exposes ESP32 to static electricity and hazards
- **Safe if:** SD card and adapter inside T-Beam case, card rarely removed (not while running)

**Example:** [Amazon unbuffered adapter](https://www.amazon.com/UMLIFE-Interface-Conversion-Compatible-Raspberry/dp/B0989SM146)

**Advantages:**
- Very small, can glue to GNSS module (metal shield to shield)
- Can share radio module SPI pins (most practical option)

**Disadvantages:**
- SD card slot should face up or card works loose from vibrations
- Sticks out farther than expected
- **Boot issue on T-Beam v1.x:** CS line (pin 0) prevents boot when powered
  - **Solution:** Add diode in CS line (cathode toward pin 0), or 3.3K resistor

**Photo:** [unbuffered_SD_attached.jpg](https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/documentation/unbuffered_SD_attached.jpg)

##### Buffered Adapters

- Interface chip between SD card and ESP32
- Larger but nicer deeper push-to-release SD holder
- Typically include voltage regulator (5V to 3.3V)

**Example:** [Amazon buffered adapter](https://www.amazon.com/HiLetgo-Adater-Interface-Conversion-Arduino/dp/B07BJ2P6X6)

**Issue:** T-Beam "5V" pin is really battery voltage (~3.7V):
- **Solution:** Modify adapter to bypass regulator
  - Connect 3.3V pin from T-Beam directly to buffer chip
  - Cut off three regulator pins (don't leave ground connected, can leave wide tab connected to board)
- Typically have built-in 3.3K resistor in pin 0 line - no boot problem

**Image:** [SD_adapter_bypass_regulator.jpg](https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/documentation/SD_adapter_bypass_regulator.jpg)

#### Shared Radio Module SPI Option

Best option without clashing other peripheral pins - share SPI with radio:
- Carefully solder wires to radio chip pins
- Besides MOSI, MISO, SCK: SD adapter also needs SS (CS) connected to "pin 0"
- Ground and 3.3V from radio module pins (8 for ground, 12 for Vcc)

**Photo:** [LORA_SPI_pins.jpg](https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/documentation/LORA_SPI_pins.jpg)

#### MISO Buffering Consideration

**Potential issue:** MISO pin may interfere when SD not accessed but radio active.

**Unbuffered adapters:** Not a problem - SD card disconnects MISO when not addressed.

**Buffered adapters - two common chips:**

1. **Adafruit (4050 chip):** MISO directly connected, not through 4050, thus not fully buffered but OK.

2. **74HC125 chip:** Tri-state, can leave outputs disconnected. But commonly available adapters have EN pin permanently grounded - outputs never disconnect.

**Solutions for 74HC125 adapters:**
- Connect EN pin to CS pin (hard - tiny wires)
- Disconnect MISO on ESP32 side, connect directly to SD card on other side of 125 (like 4050 design)
- Insert ~3.3K series resistor in MISO connection (not tested)

**If unsure:** Use unbuffered adapter - easier, fits case better.

### Adding a GNSS Module

#### Overview

Alternative to built-in GNSS for better performance or replacement of failing module.

Connected via UART built into ESP32, so just initialize with different I/O pins.

**Limited pins** require compromise on desired features.

#### Module Requirements

- TTL-level serial interface
- Ideally: PPS output wire
- Compatible with Ublox configuration commands
- **Example modules:** UBX-G7020-KT chips (VK2828U7 type)

**Recommendation:** [Amazon VK2828U7](https://www.amazon.com/G28U7FTTL-UBX-G7020-KT-Monitoring-Navigation-DIYmall/dp/B015R62YHI)

**Advantages:**
- Available, cheap, work well
- Much better GNSS reception than Ublox NEO6 (in most T-Beams)

#### Pin Choices

Three choices (in "gnss_pins" setting). Each precludes some functionality:

**1. Pins VN, 4 (blocks auxiliary serial, e.g., ADS-B module)**
- On T-Beam v1.x only

**2. Pins 13, 2 (blocks BMP/OLED on those pins)**
- Use 21, 22 instead for barometric sensor/OLED
- On T-Beam v1.x only

**3. Pins 15, 14 (blocks buzzer)**
- Pin 25 instead of 15 on T-Beam v0.7
- On both v0.7 and v1.x

#### Connection

- Module **Ground** → T-Beam ground pin
- Module **VCC** → T-Beam "3.3V" pin
- Module **TX** → Chosen RX pin ("VN", "13", or "15")
- Module **RX** → Chosen TX pin ("4", "2", or "14")

**Optional but helpful:**
- Module **PPS** → T-Beam "VP" pin
- Settings: ppswire=1 ("present")
- Precludes alternative RX pin for main serial

**Note on v0.7:** "VN" pin not usable, 15 not available. Use 13,2 or 25,14. Optional PPS on VP or pin 25. Alternative: VP for input from GNSS, 4 for output, optionally PPS on pin 25.

#### Wiring Diagrams

Diagrams available (T-Beam v1.x):
- T-beam_wiring_ext_GNSS_VN_4.jpg
- T-beam_wiring_ext_GNSS_13_2.jpg
- T-beam_wiring_ext_GNSS_15_14.jpg

https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/documentation

### Adding an ADS-B Receiver Module

#### Data Flow

Data flows into auxiliary serial port of T-Beam.

ADS-B traffic appears in SoftRF output (e.g., to XCsoar) same as FLARM traffic.

Collision warnings should also generate for ADS-B traffic (not thoroughly tested).

#### Processing

- **Only:** ES1090 "data frames" 17 and 18
- **Excluded:** Plain transponders, UAT978 transmitters
- **Filtering:** Only aircraft within 18 nm horizontally and 2000 m vertically processed

#### Code Impact

- Additional code ~20 KB binary size
- Efficiency improvements: common intermediate pre-computed, floating point avoided
- Version 120 onward: 160 MHz CPU clock (slightly higher power)
- Module power: ~40 mA at 3.3V (T-Beam supplied)
- **Total power increase:** ~40%
- **Battery life:** Still all day on single 18650 cell

#### Mode-S Messages

Only convey altitude, not position. Aircraft ID overlaid on CRC bits, recovered assuming no bit errors.

Rough distance estimate from signal strength (RSSI) 22-42:
- Estimate tailored to installation
- Based on data collected from ADS-B messages (known distance <6km)
- Data counted in 4 distance categories
- If most ADS-B at RSSI 32 from >1km away, then Mode-S at RSSI 32 assumed >1km (no alarm)
- Sample biased toward farther distances (proportional to distance²)
- **Not corrected:** Real-time signals similarly biased

#### RSSI Thresholds

Defaults not used until 100+ ADS-B messages received from <1km away (may require multiple flights):
- Until then: default thresholds
- Data file: "rssidist.txt" in SPIFFS
- To reset: Delete file (after antenna changes, etc.)

#### ADS-B Traffic Relaying

ADS-B traffic (and external GDL90 source) can be included in "air relay" feature:
- Re-transmit (but not re-re-transmit) ADS-B aircraft positions
- Protocol: "Legacy" radio protocol or ADS-L
- **Purpose:** Make ADS-B visible to other nearby SoftRF devices without ADS-B receiver

#### Data Bridging

Version MB110 on T-Beam (v1.0, 1.1, 1.2) added:
- Secondary serial port use
- Forward data from 1-3 input ports to 1-2 output ports
- **Benefit:** Eliminates need for IOIO box or Bluetooth dongle
- GNSS and FLARM messages NOT forwarded
- Other messages forwarded to designated routes
- Only complete NMEA sentences ($ through * + checksum) forwarded
- **Note:** Checksum not validated

**Important:** T-Beam design: same data sent via both USB interface and primary serial (UART) pins - cannot be separated.

Debugging messages may appear on primary serial, also connected to USB. Secondary serial more efficient for external instruments.

#### I/O Structure Revision

Originally: GNSS.cpp polled all input ports, pulled GNSS from wherever it arrived.

**Issues:** Could clash with data from internal GNSS module. Character polling could mix data from multiple sources, garbling NMEA sentences.

**Current design:**
- "Serial" = UART0 (main, also on USB)
- "Serial1" = UART1 (GNSS module communication)
- "Serial2" = UART2 (auxiliary general-purpose, formerly UAT; UAT functionality excluded via #define EXCLUDE_UATM in ESP32.h)
- **Key change:** GNSS data taken *only* from internal module
- External NMEA sentences polled in NMEA.cpp, handled separately
- Characters from each source collected into separate buffers
- Processed when full NMEA sentence accumulated
- $PSRF* sentences still interpreted for SoftRF config

#### UART2 Pin Selection

Few remaining available pins. Currently mapped to pins 39 ("VN") (rx) and 4 (tx).

**Note:** Pin 4 also drives red LED on T-Beam board.

**Attempted:** Pin 0 for RX - issues with USB power attached.

**Other options:** Pin 13 (reserved for I2C), or pins used for strobe/buzzer.

**Alternative RX pin for UART0:** Tentatively pin 36 ("VP").

Needed because when USB powered, normal RX pulled high. Serial data from USB contends with non-USB serial input on same pin.

If using alt pin: Serial input from USB disabled. Alternative: Add Schottky diode conducting from "VP" to "RX".

**Note on v0.7:** May be better to avoid GPIO 36, 39 for serial (small capacitor between GPIO36/37, and 38/39). Serial to GPIO 36 may interfere with optional hardware mod connecting GNSS PPS to GPIO37. Can recompile to use GPIO 13 instead of 36.

#### GDL90 Traffic Merging

Option to input GDL90 traffic messages and merge into traffic table:
- Decoding: GDL90.cpp, called from NMEA_loop() in NMEA.cpp
- Separated code: AddTraffic() from ParseData() in TrafficHelper.cpp
- GDL90 traffic marked with different "protocol"
- If same ICAO ID already in table under different protocol: GDL90 report ignored
- Duplicates still possible if other protocol of same aircraft not using ICAO ID

---

## Tips and Tricks

### Air Relay

Optional relaying by airborne aircraft of radio packets from other aircraft.

**Key rule:** Relayed packets not relayed a second time.

#### Relay Modes

- **Setting "1" ("Landed"):** Gliders that "landed out" (activated via web interface or aircraft type set to zero) seen from farther away, especially if airborne SoftRF traffic nearby will relay data.

- **Setting "2" ("all"):** Relays aircraft received via:
  - ADS-B (if receiver module installed)
  - FLARM traffic (if no ADS-B module)
  - Relayed in ADS-L protocol (if main protocol Latest and altprotocol ADS-L or OGNTP)
  - Plus landed-out gliders

- **Setting "3" ("relay only"):** SoftRF (only if airborne) relays ADS-B traffic more often at lower power (8 mW if full power set), but doesn't transmit own position
  - Use this to spread ADS-B benefit in SoftRF to Classic FLARMs in same/nearby aircraft

#### Relaying Rules

- **Frequency:** No more than once every 5 seconds total, once every 7+ seconds per aircraft
- **FLARM traffic:** Only over 10km away relayed (closer if lower)
- **ADS-B traffic:** Less than 8km away (10km helicopters, 16km jets)

#### Protocol Considerations

- **Dual-protocol mode + relay-all:** Relays ADS-B and FLARM traffic in ADS-L (invisible to FLARMs, visible to SoftRF dual-protocol reception)
- **Non-dual-protocol:** FLARM traffic not relayed, ADS-B in "Legacy" protocol
- **Relay-only mode:** ADS-B in "Latest" new protocol (visible to FLARMs and other SoftRF)

#### Marking Relayed Packets

- **Latest/Legacy protocols:** Third bit in "address type" field set (seems ignored by OGN stations)
- **ADS-L protocol:** "Relayed" bit set
- Ground testing: Enter "test mode" to test relaying without being airborne

---

### Test Mode

Some compile-time behaviors in test mode:
- Currently: Air-relay happens even if not airborne

#### Toggling Test Mode

- **T-Beam:** Double-click middle button, or use URL `.../testmode`, or send NMEA command $PSRFT
- **Querying test mode:** $PSRFT,?*50 with GNS5892 active saves and shows (in USB-serial output) "RSSI zone stats"

---

### How to Upload and Download Files

#### File Management in Web Interface

**SPIFFS file list:** Click "manage files" button

**Upload options:**
- "Upload egm96s.dem" button
- Settings "Upload" button
- Can actually upload any file into SPIFFS

**Download from SPIFFS:**
- Click file name in "manage files" page, OR
- Use URL 192.168.1.1/filename (substitute correct IP)

#### SD Card File Management

**Flight logs list:** Dedicated page shows .igc and .txt files

**View all SD files:** Use URL `.../listsdall`

**Upload to SD /logs folder:** Use `.../logupload`

---

### How to Monitor RSSI of Received Signals

SoftRF stores current RSSI of each tracked aircraft:
- **Legacy, Latest, OGNTP:** Negative number (e.g., -54)
- **ADS-B:** RSSI from GNS5892 (~22 to 40)

#### Maximum RSSI

"Maximum RSSI" = highest RSSI of all non-ADS-B currently-tracked aircraft (max over aircraft, not time).

**Use case:** Check transmission strength of another device in various directions from short range (that aircraft hopefully gives maximum RSSI).

#### Where to View Maximum RSSI

- **T-Beam OLED:** Second page (turn ADS-B reception off)
- **T-Beam web page:** Status page
- **T-Echo EPD:** Bottom of Time page
- **T-Echo traffic details:** Top, e.g., "1/1 RSSI -54"
- **Either device:** End of $PSRFH "heartbeat" NMEA sentence (every 10 seconds via USB, WiFi, Bluetooth)

**Advantage of web/BT view:** Can be distance away without affecting reception.

---

### Settings via NMEA

#### HTML Settings Files

Open HTML files from: https://github.com/moshe-braner/SoftRF/tree/master/software/app/Settings
- **settings1:** Basic settings
- **settings2:** Additional settings

In these open HTML pages:
1. Select desired settings
2. **Important:** Click outside input field last changed
3. Cryptic $PSRF... line changes at bottom
4. Copy line, paste into terminal program
5. Send to T-Echo or T-Beam
6. Device reboots with new settings

#### Additional Settings via NMEA (MB148+)

Code added for accessing additional settings via $PSRFD and $PSRFF sentences - useful for devices without web UI.

**Version field (first in NMEA):** Indicates whether to "save and reboot":
- $PSRFC,1,... = Work as before (including with BT app)
- $PSRFC,0,... = Just change variables, no save/reboot
- $PSRFC,SAV*3C = Save and reboot

**Reply format:** Shows correct checksum if wrong

Moved from GNSS.cpp to NMEA.cpp (following mainline).

#### MB152+ Settings Configuration

New $PSRFS for one-setting-at-a-time configuration:

- $PSRFS,?*57 = List all settings and current values (settings.txt format)
- $PSRFS,0,label,? = Query one setting
- $PSRFS,0,label,value = Change one setting (later $PSRFC,SAV*3C to save/reboot)
- $PSRFS,1,label,value = Change and save/reboot immediately

**Checksum requirement:** Each sentence ends "*XX". Can send with any two characters, SoftRF replies with correct checksum. Must have correct checksum before processing (NMEA config mechanism intended for automated setup by connected device).

---

## Details of Operation

### Real-Time Embedded Software

Multi-tasking built into hardware platform and utilized by some libraries:
- CPU multitasks
- Other modules operate independently (radio receives potential packets, software checks later; serial periodically polled)

### Main Loop Order

Main SoftRF loop operates normally() with this operation order:

1. **Baro_loop()** - Computes vertical speed if sensor available

2. **GNSS_loop()** - Gets new GPS fix if available

3. **If *new* GPS fix NOT available** - Skip most following

4. **Compute timestamps**

5. **Discretize GPS fixes** - Into ~2-second intervals for turn rate, climb rate, etc. computation

6. **Estimate_Climbrate()** - Computes vertical speed using GPS data (if no baro)

7. **Estimate_Wind()**
   - Only recomputes if 666 ms since last
   - Calls project(ThisAircraft) estimate future path
     - Only recomputes if 400 ms since last

8. **RF_Transmit()** - Only actually transmits at preset time intervals
   - Calls protocol_encode()
     - Only if/when time to transmit
     - Legacy_encode() calls project(ThisAircraft)
       - Only recomputes if 400 ms since last

9. **RF_Receive()** - Check for new received data (usually none)

10. **If received new data by radio:**
    - Call ParseData()
      - Calls protocol_decode() on packet
      - Calls Traffic_Update() for new/refreshed traffic
        - Calls one of collision alarm algorithms

11. **Traffic_loop()** - Check collision dangers at 2-sec intervals
    - Calls Traffic_Update() for each known traffic (unless updated in last 2 sec)
      - Calls one of collision alarm algorithms
        - Alarm_Latest() calls project(other_Aircraft)
    - Calls Sound_Notify(max_alarm_level)

12. **Sound_loop()**

13. **NMEA_Export()**

---

## Description of the Major Changes Made in This Version

Modified ~50 source files. Many git commits combine unrelated modifications.

### New Files

#### Wind.cpp

Completely new significant module:
- Estimates ambient wind when aircraft circling (downwind drift + ground speed variation)
- Updates wind gradually (not influenced by noisy momentary pilot maneuvers)
- Determines if aircraft airborne
- Includes code to project future path (this aircraft and others)
- Estimates climb rate from GNSS when no barometric sensor

**Needed because:** Latest collision algorithm assumes perfect circles at constant airspeed. GNSS gives ground path. With wind, not a circle. Need wind drift to convert ground path ↔ air path.

#### ApproxMath Module

Fast trig function approximations (only as accurate as needed):
- atan2_approx()
- approxHypotenuse()
- Set up for calling efficiency (no degree↔radian conversion)
- Also: simple distance/bearing approximation in TrafficHelper
- Pre-compute and store distance/bearing for reuse

#### Sound Separation

Old Sound.cpp now two separate files:
- Buzzer.cpp
- Voice.cpp
- Plus Waves.cpp for voice file handling

#### GNS5892.cpp

Interpret ADS-B data from receiver module

#### Filesys.cpp

On-board and SD-card flash file system operations

#### IGC.cpp

IGC flight log operations

#### Settings.cpp (formerly EEPROM.cpp)

Renamed upon move from EEPROM emulation to settings.txt config file

### Corrected UTC Time Determination

Fixed bug in time computation. Code now says "+ time_corr_neg". Handles possible small PPS jitter.

Exact time computed in system/Time.cpp - Time_loop().

Changed globally to variable OurTime for UTC second (from GNSS), isolating from possible code that adjusts "system" time (e.g., GNSSTimeSync() from GNSS_loop()).

### Rewrote Frequency Hopping Time Slots

New code computes FLARM-compatible frequency hopping time slots (also OGNTP, different frequency):
- **Original code:** Buggy and incomprehensible
- **Problem:** Time slot 800-1200 ms spans next second tick (PPS). Original switches frequency at 1000 ms using new Time (seconds UTC), which is incorrect. Full "Slot 1" should use previous second's frequency.
- **Solution:** New Time adopted later, around 1300 ms, during 200 ms dead time between Slot 1 and Slot 0
- **Efficiency:** Compute slot/channel once for 400 ms duration of Slot, recomputed as needed
- **Variables:** Track when new computation needed
- **Random time:** Computed in advance at beginning of slot

RF_time holds timestamp (increments 300 ms after UTC PPS, which increments OurTime) on which frequency hopping/encryption key based.

For other protocols (P3I, FANET): original timing code still in place.

### Audio Alarms

When collision warning given (three urgency levels like FLARM):
- Send warning as NMEA sentences
- Use piezo buzzer (if present) produce beeps:
  - One for level 1 warning
  - Two higher-pitch for level 2
  - Five beeps for urgent level 3

**Files affected:** driver/Buzzer, platform/ESP32

**Added:** Voice output option (see below)

### Vertical Separation Accounting

Both in alarm level calculations and old traffic purging in favor of closer new traffic:
- Vertical separation × constant (5) treated as additional distance
- Relative vertical speed (if converging in altitude) included
- GNSS-based altitude not very accurate
- **Goal:** Avoid unnecessary warnings about well-separated-by-altitude traffic while not skip warnings about traffic without true vertical separation

**File affected:** TrafficHelper

### Hysteresis in Collision Alarms

When warning given about aircraft, try not to repeat too often.

Example: Distance method - aircraft moves in/out of threshold, triggering new warning each time.

**Solution:** Require 2-step change in alarm level for given aircraft before threshold reset.

Example:
- Alarm at LOW level given
- New alert only if same aircraft becomes URGENT (skip IMPORTANT level)
- If aircraft moves away to CLOSE level, threshold reduced
- Next time IMPORTANT level reached, new alert triggered

**File affected:** TrafficHelper - Traffic_Update() and Traffic_loop()

### Graduated Collision Cone (Vector Method)

If relative velocity vector slightly outside angles threshold for triggering alarm, still issue alarm but at lower urgency.

**File affected:** TrafficHelper - Alarm_Vector()

### New "Latest" Collision Prediction Method

Applies to circling aircraft - complicated by wind effects:
- FLARM sends projected future path in format - with wind - neither relative to air nor ground
- Solution: Project velocities in 3-second intervals, paths in 1-second intervals based on those velocities

**Files affected:**
- Wind.cpp (compute path projections for ThisAircraft and other Aircraft)
- TrafficHelper - Alarm_Latest()
- Outgoing radio packets modified (protocol/Legacy legacy_encode()) to send path projection in FLARM-expected format

### Improved FLARM Compatibility (Pre-2024 "Legacy" Protocol)

**Outgoing:**
- Send lat/lon computed 2 seconds into future
- Send projected velocity vectors for future time points dependent on aircraft type, for gliders on circling establishment
- Projected velocities based on current ground speed/turn rate, ignore wind
- Set undocumented _unk2 field to mimic FLARM

**Incoming:**
- Convert location to current time
- Convert projected velocities to airmass frame of reference
- Use _unk2 field and aircraft type to interpret projected time points

### Modified Traffic Data Replacement Decision

Besides distance (altitude-separation-adjusted basis), introduced:
- Concept of aircraft to "follow" (e.g., "buddy")
- Followed aircraft tracked preferentially over closer traffic (unless closer triggers collision warning)
- Remember previous data from same aircraft (help estimate climb/turn rates)

**File affected:** TrafficHelper - ParseData()

### Settable Aircraft IDs

- Ability to ignore one aircraft ID (e.g., another device in same aircraft, towplane)
- Incoming packets with same ID as this aircraft always ignored
- Also allow setting this aircraft ID (e.g., to ICAO)
- Also allow setting "follow" ID

**Files affected:** TrafficHelper, protocol/radio/Legacy legacy_decode(), driver/EEPROM, ui/Web

### Modified Lat/Lon Computation (Legacy Protocol)

For more accuracy and efficiency. Similar to, but not same as, mainline recent implementation.

**Affected:** protocol/radio/Legacy legacy_decode() and legacy_encode()

### Additional Settings

Code to choose aircraft ID type, aircraft IDs, baud rate, external power, debugging flags, etc.

**Files affected:** driver/EEPROM, ui/Web

### Shutdown When No External Power

Some want device mounted in inaccessible place, external USB power. When master power/aircraft battery turned off, SoftRF also turns off (if no device battery). But T-Beam takes long time to re-acquire GPS fix after sitting unpowered.

**Solution (Prime Mark II supported only):** Battery in T-Beam, auto-turn-off when:
1. Battery voltage below 3.9V
2. Device operating ≥1 hour
3. External power removed
4. Aircraft not airborne

Idea: After hour with external power, battery voltage back over 3.9V.

"Power source" setting needs "External" for this.

**File affected:** driver/Battery (plus settings adjustment files)

### Spurious FLARM Alarms from Stationary SoftRF

Multiple rounds of changes to "airborne" status computation.

Now in Wind.cpp

### Spurious FLARM Alarms from Towed FLARM by SoftRF Towplane

Old protocol: Transmit lat/lon computed 2 seconds into future (like FLARM does).

### Hide IGC Encryption Key

If compiled with OGNTP encryption enabled, key divided into 4 sections (8 hex digits each):
- If zero: Shows "00000000"
- Otherwise: Shows "88888888" (masking true key)
- Can overwrite "88888888" with something else (including "00000000")
- Left as "88888888": Current key intact
- Set to all zeros: No encryption

**Allows** contestant inspect/change settings without losing key or seeing key (set earlier by contest official).

### Support "Badge Edition" (T-Echo, nRF52840)

Incorporated mainline v1.2 EPD code.

Created new display screen showing some settings. Moved protocol/aircraft ID here from status screen.

Replaced in status screen with satellite count and current collision alarm level.

Settings not changeable via SoftRF Tool app changeable via USB + "SoftRF settings tool 2.html".

"SoftRF settings tool 1.html" offers "Latest" alarm method choice (now default). Or better: edit settings.txt.

**New file:** Conf_EPD.cpp

### T-Echo Settings via E-Paper Screen

Added ability adjust some settings within device using EPD & buttons.

**New file:** Change_Settings_EPD.cpp

**Also modified:** Conf_EPD.cpp, nRF52.cpp, etc.

### Second NMEA Output Destination

Allow two NMEA output routes simultaneously (e.g., BT & USB, or UDP & TCP).

Also allow selection of sentence types per route independently.

Default routes depend on platform.

**Files affected:** NMEA.cpp, Web.cpp

### NMEA Processing (in NMEA.cpp)

- **"Private" sentences:** From Linar's debug code (left alone, ignored)
- **"Debug" sentences:** Own debug bits
- **"Traffic" sentences:** FLARM-like output
- **"GNSS" sentences:** $GPGGA, etc.
- **"Sensor" sentences:** Internal sensors (mainly baro chip)
- **"External" sentences:** From other devices, passed through
- Output of each can be enabled/disabled per route
- Incoming external sentences examined before passing through
- External GNSS sentences ignored (not passed through)
- External FLARM sentences ignored (not passed through)
- External sentence not passed to same port from which it came
- External config sentences ($PSRF...) processed
- Config sentence replies only to same port of origin

### Visual Alarms and Strobe Driver

Added "strobe control" module:
- LED on T-Beam case or cockpit elsewhere
- Flash on collision alarm, or trigger high-intensity strobe in canopy front
- Periodic flashes, more frequent on collision alarm
- Currently needs wired connection to pin 33 on T-Beam
- SoftRF sends special $PSKSF NMEA message whenever flash happens (or could happen)
- Future: Separate device (SkyStrobe) possibly embedded in strobe unit, receive from SoftRF/FLARM (wired/wireless), control strobe

**New files:** driver/Strobe.h and .cpp

### Improved Audio Alarms

Incorporated ToneAC library for beeping signal in opposite phases on 2 GPIO pins (14&15):
- Passive piezo buzzer between pins = higher volume than one pin to ground
- Added settings option for DC (+3V) output pin 14 to trigger external active buzzer (via transistor, higher voltage)

### Stealth Mode

Masked additional data fields to fit FLARM specifications

### WiFi Client Mode and TCP Client Mode

Revived option connecting to external WiFi network (SSID & PSK via settings):
- After boot, try connect to specified WiFi
- Not found after 10 seconds: Create own network
- IP address visible in third OLED screen
- Added TCP client ability: host IP, port (2000 or 8880) selection
- Both TCP and UDP simultaneously (primary/secondary NMEA destinations)
- Purpose: Wireless XCvario connection (and potentially other devices creating own WiFi)
- Output via UDP may tunnel through external network, reach other devices (e.g., XCsoar via XCvario WiFi)

### Voice Output

New module using WAV files (8000 Hz sample, 8-bit, ~70 KB total) stored in SPIFFS single file waves.tar:
- Sent as I2S data stream
- Options: External I2S decoder/amplifier (NOT YET TESTED), internal DAC on pin 25 (tested)
- Exact I2S configuration critical (Voice.cpp)
- Works in Arduino ESP32 Core v2.0.3 (counter to rumors)
- Methods to upload tar file via web interface
- **Files:** driver/Voice.cpp and .h, Waves.cpp, UI/Web.cpp modifications

### Memory Use

Nearly all heap space used once library objects created. Web server and Bluetooth can't be simultaneously active.

**Workaround:** Upon web server access, SoftRF checks available RAM. If necessary, turns Bluetooth off until reboot.

**Space freeing:** Blocks >1 KB (not default 4 KB) now allocated in PSRAM (T-Beam).

### Altitude Handling

Revised in version MB146:
- **SoftRF internal:** WGS84 ellipsoid-relative (what FLARMs transmit, IGC files)
- **GGA sentences:** MSL with geoid separation (standard)
- **OGNTP:** MSL
- **ADS-B:** Pressure altitude
- **P3I/FANET:** Unclear

### The Latest Protocol

Version receives and transmits new FLARM protocol (V7, March 2024):
- Protocol setting only chooses transmission format
- If "legacy" or "latest" selected: receives both
- **MB133:** Avoids transmission in second half time slot 1 (1000-1200 ms after PPS) when "epoch" is 1 less than multiple of 16 (some OGN ground stations decrypt incorrectly otherwise)

### Other Changes in Version 120

Added code in arduino/basicmac/src/lmic/radio/sx127x.c to clear FIFO before transmission:
- Prevents possible re-transmission of received packets (long-standing SoftRF bug)
- Credit: Alessandro Faillace, Nick Bonniere

Added received data check in main loop just *before* transmission:
- Prevents loss of received data upon radio chip FIFO clearing for transmission

### Efforts to Reduce Binary Size

Flash space on T-Beam almost full, as is RAM used. Before adding features, reduce memory use.

**Version 124:**
- PMU chips (AXP192 and AXP2101) on T-Beam (v1.1, 1.2) handled by same library (XpowersLib)
- "jQuery" script (percent progress during OTA firmware update) removed
- **Reduced:** 60 KB binary

**Another 16 KB shaved:** Removed EGM96 table from binary
- See "geoid separation" setting to restore when needed

### Revision of Data Structures

Revised data structures at SoftRF heart: "container[]" array:
- Each element grown to hundreds of bytes, copied over/over for each packet
- **Solution:** Two structure types:
  - Short ufo_t: only needed fields from incoming packets (57 bytes)
  - Long container_t: for array
  - Third small adsfo_t: only fields for ADS-B message processing

### GNSS Baud Rate and Bridge Mode

Internal processor-GNSS connection uses independent baud rate (normally 9600) from external serial connection rate.

If GNSS unreachable at 9600, may be configured (prior firmware) to other baud rate or only UBX binary (not NMEA).

#### MB156+: Automatic Baud Rate Detection

Tries detect GNSS NMEA sentences at other baud rates. Also tries UBX query at each rate (if NMEA not outputting). If either succeeds: use that rate. If only UBX: reset module to factory defaults.

#### Reset GNSS (Web Interface)

Click "reset GNSS" button: GNSS returns to factory default (usually 9600, except Ublox Neo-8 possibly 115200). SoftRF reboots, baud rate detection happens.

#### GNSS Bridge Mode

If SoftRF fails to connect despite auto baud rate selection: Use "GNSS bridge mode" + U-Center Windows app (Ublox) for diagnostics/repair.

From MB156: Internal GNSS connection uses same baud rate as external (main) serial port when in bridge mode. (No auto-adjustment in bridge mode.)

To use:
1. Change main serial port baud rate to GNSS module rate (normally 9600)
2. If unreachable at 9600: Try other rates (19200, 38400, 57600, 115200) until U-Center connects
3. Change U-Center and SoftRF settings each try, reboot
4. Use U-Center restore factory defaults
5. GNSS back to default baud rate (usually 9600, except Ublox Neo-8 possibly 115200)
6. Change U-Center to that rate, then SoftRF to that rate (reboot) for final bridge mode test
7. Change SoftRF to normal mode and default (38400) baud rate
8. Exit GNSS bridge mode: Web interface to normal mode, or temporarily via middle button press (T-Beam)

### Reception Range Analysis

From version MB155: SoftRF collects summary data about distance/RSSI of aircraft entering/leaving reception range.

**Storage:** "range.txt" file in flash (SPIFFS T-Beam, FATFS T-Echo)

**Format:** Plain text, 14 lines:
```
1               # version number
9.4,3.226,1234  # range (km), log2(range), sample count - for "oclock=0" (straight ahead)
...             # similarly for "oclock" 1-11
-91.4,2.58      # average RSSI, mean square deviation
```

**Process:**
- Read file
- New data appended during flight
- New version written after landing

**Conditions:** Only if airborne, other aircraft airborne, ≥1km away, not too high/low vertical angle.

ADS-B not sampled.

**Analysis:** Log-scale averaging used (plain km appear first).

**After enough data:** If range much lower some directions = antenna placement not ideal.

**RSSI considerations:**
- If average much higher (less negative) than -100: Radio noise in cockpit (e.g., USB power converter) reduces sensitivity
- If mean square deviation large: Noise intermittent (device sometimes carried/turned on)

**Reset data:** Delete range.txt file

### Dual-Protocol Reception

From version MB172: Can receive FLARM and ADS-L simultaneously.

Following Pawel Jalocha's method (OGN Tracker):
- Sync word: just 16 bits, starting 2 bits into "0x55 0x99" part of sync word (mimic nRF905 preamble), ending 2 bits into rest of sync word
- Same 16 bits in both protocols
- If packet arrives: check next 32 bits sync word (2 bytes after Manchester decoding)
- If matches part of sync word for FLARM or ADS-L: process packet
  - Shift received bits to recover payload
  - Compute and compare CRC
  - Accept if CRCs match
- **To enable:** ADS-L sent/received on same frequency as FLARM (standard for future ADS-L use outside EU)
- **OGNTP:** Always different frequency - cannot receive simultaneously with these protocols

---

## Known Issues

### Packet Transmit Count

Packets-transmitted counter on OLED sometimes increments by dozens/hundreds at once - reason unknown.

### Received Packet Loopback (T-Echo)

Packets-received occasionally increments while no traffic in range. Code reports "RF loopback" (exact replica of last transmitted packet appears as received). Probably sx1262 register programming bug. Appears harmless.

### Reception Improvement Possibility

FLARM signal reception can be further improved by shifting our clock (for frequency hopping) ~50 ms. Reason unclear.

---

## Compiling SoftRF

Current compilation: Ubuntu 20.04, Arduino IDE version 1.8.16.

**T-Beam:** ESP32 board support version 2.0.3
- Later versions make binaries too large
- Expect library incompatibilities with other versions

**Compile settings:**
- 160 MHz processor
- 80 MHz DIO flash
- Minimal SPIFFS
- PSRAM enabled

**T-Echo:** Adafruit nRF52 board support version 1.2.0

---

**For more information and updates:**
- GitHub: https://github.com/moshe-braner/SoftRF
- Discussion Group: https://groups.google.com/g/softrf_community
