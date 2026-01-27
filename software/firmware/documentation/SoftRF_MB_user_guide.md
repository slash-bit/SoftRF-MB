# SoftRF User Guide

*Basic Documentation for Moshe Braner's version of SoftRF*

**By Moshe Braner**

**Last updated:** January 11, 2026 (software version MB172)

**Latest version:** https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/documentation

**Discussion group:** https://groups.google.com/g/softrf_community

---

## Table of Contents

- [What is SoftRF](#what-is-softrf)
- [History](#history)
- [How to Use SoftRF](#how-to-use-softrf)
- [Hardware Choices](#hardware-choices)
- [Powering the Device](#how-to-power-the-softrf-device)
- [Turning On and Off](#turning-the-device-on-and-off)
- [Installing Firmware](#installing-firmware-for-the-first-time)
- [Web Interface](#the-web-interface-t-beam-only)
- [Settings](#choosing-the-operational-settings)
- [Collision Alarms](#collision-alarms)
- [Special Modes](#winch-mode)
- [Voice Files](#uploading-voice-files-t-beam-only)
- [External Devices](#how-to-connect-external-devices)
- [SD Card](#connecting-an-sd-card-adapter)
- [Flight Logging](#flight-logging)
- [Data Bridging](#data-bridging)
- [ADS-B Receiver](#connecting-an-ads-b-receiver)
- [XCvario Connection](#connecting-to-xcvario)
- [Firmware Updates](#updating-the-firmware)

---

## What is SoftRF

SoftRF is a do-it-yourself, multifunctional, compatible, radio-based proximity awareness system for general aviation. It is free, open-source software that runs on inexpensive off-the-shelf hardware.

### What SoftRF Does

When installed on suitable hardware and carried in an aircraft, SoftRF:
- Transmits your aircraft position
- Receives position data from other compatible aircraft
- Sends information about nearby traffic in standard formats to visualization devices and navigation software
- Supports several radio communications protocols, primarily FLARM-compatible "Latest" mode

### Features

Using SoftRF in your aircraft (when configured for "Latest" protocol) will:
- Make you visible to other FLARMs
- Make you visible to OGN ground stations (viewable on websites like https://glidertracker.org/)
- Make other FLARMs visible on your devices such as glide computers
- Generate collision warnings, alone or in conjunction with other devices

### What Can SoftRF Do That FLARM Cannot?

- Send traffic data via WiFi or Bluetooth (older FLARM models cannot)
- Operate using alternative protocols: OGNTP, ADS-L, P3I, or FANET
- Run all day on a small internal battery
- **This version specifically:**
  - Receives and transmits new FLARM protocol (V7, March 2024)
  - Fixes radio protocol bugs for better FLARM interoperability
  - Correctly transmits curved paths for circling aircraft
  - Receives and interprets curved paths from other aircraft
  - Offers built-in audible collision warnings in 3 urgency levels
  - Supports aircraft ID assignment
  - Offers IGC flight logging
  - Can relay traffic via ADS-L
  - Can receive ADS-B signals (with optional hardware module)
  - Can receive ADS-L signals simultaneously with FLARM signals (MB172+)

### What Can FLARM Do That SoftRF Cannot?

- PowerFLARM units can listen to signals via two antennas
- FLARM is available off-the-shelf from established manufacturers
- Requires less DIY construction (though T-Echo is more ready-to-use)

### What Can the Mainline Version Do That This Version Cannot?

The main branch of SoftRF offers binaries for a wide variety of hardware platforms. This version is compiled only for:
- **T-Beam** (ESP32-based)
- **T-Echo** (nRF52-based)

*Note: Only the plain T-Beam is supported, not the T-Beam Supreme.*

---

## History

**FLARM** is a traffic awareness system adopted by most glider pilots in Europe and operates in 866-928 MHz range (470 MHz in China). It transmits low power (25 mW) in short bursts, once or twice per second.

**The Open Glider Network (OGN)** has encouraged the establishment of ground stations throughout Europe that listen to FLARM and other devices and feed data to internet servers.

**The Internet of Things (IoT)** community has developed inexpensive low-power hardware with integrated computing power, memory, and communications capabilities (WiFi, Bluetooth, GNSS, and various radio protocols).

**Linar Yusupov** developed [SoftRF](https://github.com/lyusupov/SoftRF), which sends, receives, and interprets FLARM-compatible signals on IoT hardware.

**Moshe Braner** further developed SoftRF, filling in gaps to make it a more complete functional equivalent to FLARM. [View the fork here](https://github.com/moshe-braner/SoftRF).

The software runs on bare hardware without an operating system, compiles to less than 2 megabytes, and is flashed into the device via USB cable or WiFi.

---

## How to Use SoftRF

### Hardware Choices

This version runs on two specific devices:

#### T-Beam (Prime Mark II)
- **LilyGo (TTGO) "T-Beam"** device
- Features: integrated battery holder, GNSS module, 900 MHz radio, WiFi, Bluetooth, flash memory, PSRAM
- **Purchase from:**
  - [LilyGo directly](https://www.lilygo.cc/products/t-beam-v1-1-esp32-lora-module?variant=42204035186869)
  - Amazon or banggood.com (via LilyGo store)

#### T-Echo (Badge Edition)
- Based on nRF52840 CPU
- Features: 1.5-inch e-paper display (more self-contained)
- **Limitations:** No WiFi, lower transmission power, weaker stock antenna, harder to attach external devices

**Important:** Select the correct frequency band for your region:
- **Europe:** 868 MHz
- **USA/Canada:** 915 MHz
- **China:** 433/470 MHz

### Optional Accessories

#### Piezo Buzzer
For audible collision warnings:
- **Model:** [Murata Electronics PKM22EPPH4001-B0](https://www.digikey.com/en/products/detail/murata-electronics/PKM22EPPH4001-B0/1219323)
- **Alternative:** "22x4mm AC With Lead Passive Piezo Electronic Alarm Buzzer 2204"
- **Connection:** Between GPIO pins 14 and 15 in series with 100-ohm resistor
- **Note:** On T-Beam v0.7, use pin 25 instead of 15 (if voice output not enabled)

#### Voice Warnings
- **Output:** Pin 25 (line-level audio, requires external amplifier)
- **Circuit example:** [Audio output circuit diagram](https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/documentation/audio_output_circuit_.jpg)
- **Amplifier options:** Standalone device or aircraft radio AUX input
- **Alternative:** External I2S decoder/amplifier (Max98357 chip)

#### LED/Strobe
- **Connection:** Pin 25 through 100-ohm resistor (pin 15 if voice output enabled)
- **Function:** Flashes on collision alarm or periodically when airborne
- **NMEA output:** $PSRSF sentence for integration with other devices

#### OLED Display
- **Optional** (device works without it)
- **Shows:** firmware version, device ID, protocol, band, aircraft type, tracked aircraft count, packet counts, GNSS status, battery voltage, IP address
- **Connection:** Pins 21 (SDA) and 22 (SCL), or pins 13 (SDA) and 2 (SCL)
- **Note:** Very fragile - protect well in case

#### Case
- **3D-printed cases available** with GoPro mount compatibility
- [Modified case design with GoPro mount](https://drive.google.com/drive/folders/1hdEJ-nRVz4D_sYihwv1Re2S-p8_nsYaM?usp=sharing)
- [Larger case with rear mounting space](https://github.com/moshe-braner/SoftRF/blob/master/case/T-Beam_case.zip)
- [Case for T-Beam + ADS-B receiver](https://github.com/moshe-braner/SoftRF/blob/master/case/T-Beam%20%2B%20ADS-B%20case.zip)
- **Material:** ABS works better than PLA
- **Printing service:** [Aero3DS](https://aero3ds.com/3d-printing) - reasonable prices

#### Antenna
- Better antenna than included whip may improve performance
- **Type:** True dipole antenna (no ground plane needed)
- **Mounting:** Far forward (nose) or rearward (wing spar/fuselage) performs better than instrument panel
- **Examples:**
  - [Data-Alliance 900MHz dipole](https://www.data-alliance.net/antenna-900mhz-3g-gsm-omni-directional-rp-sma-or-sma/) (select SMA male)
  - [Data-Alliance 824-960 MHz narrow band](https://www.data-alliance.net/antenna-824-960-1710-2170mhz-sma-connector/)
- **Orientation:** Vertical if possible

#### GNSS Antenna
- **T-Beam mounting:** With antenna connector facing up (built-in GNSS needs horizontal top position)
- **External option:** UFL connector for better reception
- **Example:** [Data-Alliance GPS LNA 25x25mm with 4-inch cable](https://www.data-alliance.net/antenna-gps-lna-embedded-u-fl-sma/)

**Important:** SoftRF only transmits when a GNSS fix is acquired (takes seconds to 30 minutes after cold start). Leap second information acquisition may take up to 12 more minutes for correct time.

**Stock GPS antenna limitations:**
- Won't work indoors
- Needs clear outdoor area
- Performance varies between units
- Some slower than others
- **Solution:** Add 25x25mm active antenna for much better performance

**Reset GNSS (T-Beam only):** Button in web interface allows factory reset and cold start.

### How to Power the SoftRF Device

Three main options:

#### 1. Battery Only
- Install 18650 lithium-ion cell (FLAT TOP, not BUTTON TOP)
- Sufficient for long flights
- Charge via USB port
- On T-Beam v1.x with PMU: blue LED indicates charging
- **Note:** Old T-Beam v0.7 has no charging indication

#### 2. USB Power Only
- No battery needed
- Any USB power source works
- Can use 12V-to-5V DC-DC converter with aircraft power
- **Current needed:** ~120 mA normally, ~240 mA during transmission bursts
- **Disadvantage:** Long GNSS acquisition time after extended power-off
- **Advantage:** Can mount permanently in aircraft

#### 3. Hybrid System
- Combines external USB power and on-board battery
- USB charges battery while in use
- Battery provides backup if external power fails
- **Unique to this version:** "External power" setting automatically shuts down when:
  1. Device has run for at least 1 hour
  2. Aircraft is not airborne
  3. External power is removed
  4. Battery voltage drops below 3.9V
- **Storage concern:** Hot summer trailer temperatures with lithium battery

#### Alternative Power Without Battery
- Supply +5V directly (not via USB jack)
- **Method 1:** Feed to battery holder + tab through standard silicon diode (reduces to ~4.3V) + 2-5 kΩ parallel resistor
- **Method 2:** Use 3.7V power converter (DD4012SA)
- **Advantage:** No need for space under device for USB plug

**Long storage note:** After weeks in "off" mode, installed battery may deplete. Always recharge before long flights after storage exceeding 2 weeks.

### Turning the Device On and Off

#### T-Beam v1.x
- **Turn ON:** Press and hold button closest to USB jack (~2 seconds) until LEDs/OLED light
- **Turn OFF:** Press and hold until blue LED turns off, OLED says "OFF", or buzzer makes high-low sound
- **Shutdown time:** Few seconds
- **Complete:** When dim red LED turns off
- **Middle button:** Not normally used (but see special modes)
- **Third button:** Reset

#### T-Beam v0.7
- Use slide switch to turn on/off

#### T-Echo
- **Press:** Reset button or Mode button to turn on
- **Turn OFF:** Long-press Mode button
- **Page cycling:** Brief Mode button press
- **Screen saver:** Touch and hold touchpad for 2 seconds (device still operating, click Mode to reactivate)
- **Screen saver + OFF:** Long-press Mode while touching pad

#### Alternative Boot Methods
If battery installed and USB power connected, normal boot may not occur (charging mode instead):

**Method 1:** Boot on battery first, then connect USB power

**Method 2:** Press middle button before/while OLED displays "CHARGE MODE", hold until "NORMAL BOOT..." appears

**Method 3:** Enable "external power" setting (mentioned above)

#### Power Management Unit (PMU) (T-Beam v1.x)
- Always active
- First pushbutton sends instructions to PMU
- PMU controls power to individual board components
- Can turn on/off CPU, GNSS, etc.

#### LED Indicators (T-Beam v1.x)
- **Blue LED (near PMU):** Controlled by PMU
- **Red LED (near PMU):** CPU is powered
- **Red LED (near GNSS):** GNSS fix indicator (blinking = fix)

#### Startup Behavior
- SoftRF waits for GNSS fix before transmitting
- **T-Beam v1.x LED sequence:**
  - Blue LED blinks rapidly (4/sec) until fix
  - GNSS red LED blinks slowly (1/sec) when fix achieved
  - Check OLED for satellite count
  - After fix: blue LED off if transmissions off, stays lit if on (blinks slowly if low battery)
- **Ground testing:** Gives warnings and sends signals pretending airborne for 1 minute after boot with GNSS fix
- **Strobe:** Fires first 9 seconds despite not airborne (for testing)

---

### Installing Firmware for the First Time

#### Initial State
T-Beam often arrives with SoftRF pre-installed, enabling WiFi firmware updates. See firmware update section below.

#### If Board Has Different Firmware
If board arrived with other software (e.g., Meshtastic) or after failed WiFi update, use USB installation:

**Instructions:** https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/binaries/README.md#esp32

**Files needed:** Use desired .bin file instead of SoftRF.ino.bin. ZIP with other 3 required files:
https://github.com/moshe-braner/SoftRF/tree/master/software/app/Flashing

#### Requirements
- PC running Windows with USB port
- USB driver (Windows 10 usually auto-recognizes)
- USB shows as COM port in Device Manager
- Expressif ESP tool software download and installation
- **Critical:** Set memory addresses as shown in instructions

#### Flashing Tools

**Expressif ESP Tool:**
- Official tool from ESP32 makers
- Not very intuitive - follow written instructions and image carefully
- If it goes wrong, repeated attempts usually recover the device

**Web-based alternatives:**
- [esp.huhn.me](https://esp.huhn.me/) - Works with Chrome, Edge, Opera
- [Espressif ESP-Launchpad](https://espressif.github.io/esp-launchpad/) - Chrome only

#### Troubleshooting
- **Pin 2 issues:** Disconnect anything on pin 2 or ground it via 100-ohm resistor during flashing
- **Strange behavior:** Fully erase flash memory before re-installing firmware
- **Connection issues:** Momentarily ground pin 2 via 100-ohm resistor while flashing tool connects

#### T-Echo
Usually includes bootloader for easy updates - see firmware update section below.

---

### The Web Interface (T-Beam Only)

#### Connecting
After startup, SoftRF tries to connect to WiFi network specified in settings (if SSID and password provided).

**If connection unsuccessful (10 seconds):** Creates own WiFi network
- **Network name:** SoftRF-xxxxxx (where xxxxxx is device ID)
- **Alternative:** Use "myssid" setting for custom name
- **Password:** 12345678
- **Device ID:** Check 4th or 5th OLED page for SSID

**Using external WiFi network:** IP address assigned by router - check OLED page 4-5 or click button to cycle through pages

**Browser:** Use IP address **192.168.1.1** (if using SoftRF network)
- Mini web server built into SoftRF
- View status, restore user settings, change settings, update firmware

**T-Echo note:** Web interface NOT available (no WiFi support)

#### Status Page
Home page displays:
- SoftRF status information
- Buttons for various operations:
  - View/change settings
  - Firmware update
  - Download flight logs
  - Manage other functions

**Revision (MB153):** Less cluttered, more intuitive layout

#### Landed-Out Mode
Button on home page activates "landed out" mode.

**Alternative:** Long-press middle button on T-Beam v1.x

See "under the hood" document for landed-out mode and relaying details.

#### Memory Issues
When Bluetooth is active, web interface may not function (insufficient memory).

**Workaround:** Low memory triggers automatic Bluetooth shutdown (until reboot), with notice at top of web pages

**Recovery if stuck:** USB/BT terminal + NMEA config (see "under the hood" document)

---

### Choosing the Operational Settings

#### T-Beam Settings Access
1. Connect to SoftRF status web page (see above)
2. Click "Basic Settings" button (lower left)
3. View and change settings
4. Scroll to bottom, click "Save and Reboot"
5. Reconnect to WiFi, verify settings applied

#### T-Beam Advanced Settings (MB153+)
- **Short "basic settings" page:** Common settings
- **Auto-generated "advanced settings" page:** Nearly all settings
  - Enter raw code for each setting
  - Click "Save and Reboot"
  - Comments on some lines explain values

#### Settings File
- **Location:** "settings.txt" in flash memory file system
- **Format:** Text file resembling advanced settings web page
- **Functions:**
  - Download for backup elsewhere
  - Upload (possibly after manual editing)
  - Backup copy within flash memory
  - Swap current file with backup copy

**Important:** Clicking "save and reboot" replaces existing backup with current settings before writing new file. After save/reboot, "Restore/Swap" button reverts to previous settings.

#### T-Echo Settings
More difficult to adjust than T-Beam:

**Best method:** Edit "settings.txt" file after USB connection to computer

**Setup steps:**
1. Connect T-Echo to computer via USB (opens as "drive")
2. May need to format as "FAT" first (if space not prepared)
3. Edit settings.txt using text editor
4. Each line: setting label, comma, setting value
5. Comments explain values
6. Save and reboot T-Echo

**Older methods:**
- Within-device settings adjustment (see "settings displayed within T-Echo" section)
- Android SoftRF Configuration Tool (limited features)
- USB/Bluetooth terminal app with NMEA config

---

### Description of Basic Settings

#### Mode
- **Normal:** Standard operation
- **GNSS Bridge Mode:** Direct GNSS module communication for diagnostics (e.g., U-Center program)
  - Uses selected USB baud rate for internal GNSS connection
  - T-Echo: Enter via NMEA config, escape via double-click
  - T-Beam: Exit via middle button click (switches to Normal)
  - T-Beam v0.7: Long-press first button

#### Device ID
- Unique to each device - **cannot be changed**
- First hex digit now always "8" (avoids FLARM duplication)
- **Update note:** If registered with OGN using device ID, update that registration after version upgrade

#### Aircraft ID
- 6 hex digits
- If aircraft registered, enter ICAO ID (unique to aircraft)
- **USA:** [FAA Aircraft Inquiry](https://registry.faa.gov/aircraftinquiry/Search/NNumberInquiry) - look for "Mode S Code (Base 16 / Hex)"
- **Other countries:** Contact local aviation authorities
- **Multi-device same aircraft:** Set all to same ID to avoid duplicate OGN reporting
- **OGN Registration:** Register at [ddb.glidernet.org](http://ddb.glidernet.org/)

#### ID Type to Use
**Choices:**
- **ICAO:** Use Aircraft ID entered above
- **Device:** Use fixed device ID
- **Anonymous:** Random but fixed for flight

**Important:** Transmitted ID depends on **TWO settings**. Example: entering ICAO in Aircraft ID field but leaving ID Type as "device" still transmits device ID.

**Registration:** Whether ICAO or device ID, register on [ddb.glidernet.org](http://ddb.glidernet.org/) so OGN viewers display your registration/contest ID.

#### Protocol
- **Latest:** New (2024) FLARM radio protocol - **RECOMMENDED** for FLARM compatibility
- **Legacy:** Older FLARM protocol - visible to SoftRF and OGN stations, NOT to FLARMs. (FLARMs see both Latest and Legacy)
- **OGNTP:** Visible to OGN stations, invisible to FLARMs
- **P3I:** Light plane protocol (common in UK)
- **FANET:** Paraglider protocol
- **ADS-L:** Alternate protocol (new in MB166)

**Towplane note:** If device triggers collision alerts on tow, configure as OGNTP or Legacy for FLARM alerting prevention while remaining visible to OGN.

**Alternate protocol:** Can also choose to transmit alternate protocol once every 16 seconds (see "under the hood" document)

#### Region (Band)
**Must choose correct region or radio fails:**
- **EU:** 868 MHz (Europe)
- **US:** 915 MHz (USA/Canada)
- **AU:** 921 MHz
- **NZ:** 869.250 MHz
- **RU:** 868.8 MHz
- **CN:** 470 MHz (China)
- **UK:** Treated as EU unless FANET
- **IN:** 866.0 MHz
- **IL:** 916.2 MHz
- **KR:** 920.9 MHz

#### Aircraft Type
- Essential for collision avoidance and situational awareness
- Important for OGN tracking
- **Options:** Glider, powered plane, helicopter, etc.
- **New (this version):** "Winch" - see Winch Mode section below

#### Alarm Trigger
Chooses collision warning algorithm:

- **Latest (Recommended):** Unique to this version. Mimics FLARM, predicts near-future paths of circling aircraft. Reverts to Vector method if neither aircraft circling, then to Distance method if movement undetermined. **Thus ALL algorithms used as appropriate.**
- **Vector:** Extrapolates straight lines (reverts to Distance if movement not determined)
- **Distance:** Simple distance-based warnings
- **None:** No collision warnings

#### Volume
- **Off:** No buzzer
- **Low:** Quiet buzzer output
- **Full:** Loud buzzer (still not loud enough)
- **External:** +3V DC output on pin 14 to trigger external 12V active buzzer via transistor

**When to use Low/Off:** If getting audible warnings via connected device (e.g., XCsoar)

#### Built-in Bluetooth
- **Off:** Recommended default
- **SPP:** Classic Bluetooth (T-Beam only)
- **LE:** Bluetooth Low Energy (T-Echo can only use LE)

Use when connecting to external device (e.g., XCsoar) via Bluetooth.

#### NMEA Output
Selects port for data transmission:
- **Off:** No data sent
- **Serial:** On-board serial interface (also appears on USB)
- **Auxiliary Serial:** T-Beam only
- **UDP:** WiFi method
- **TCP:** Alternate WiFi method
- **Bluetooth:** Wireless method

#### NMEA Sentences
Turn on/off sentence types:
- **GNSS:** Position data (RMC, GGA in basic settings)
- **Sensors:** Barometric sensor, etc.
- **Traffic:** Traffic data in FLARM format
- **External:** Data from external devices forwarded (see "data bridging")

**Advanced settings (MB153+):** Enter value "3" (instead of "1") for GNSS sentences to add GSA (needed by Skydemon). Similarly for other sentence types.

#### NMEA Secondary Output
Two output ports allowed simultaneously:
- Example: Bluetooth and USB, or UDP and TCP
- **Caution:** Bluetooth + WiFi not recommended (share same radio)

**Default:** Both ports receive GNSS, sensors, and traffic data (GGA, RMC, RMZ, PFLAU, PFLAA)

#### Serial Port Baud Rate
- **Default:** 38400
- **Choices:** 4800, 9600, 19200, 38400, 57600, 115200
- **Note:** Lower rates may not accept all data - pare down output

#### Stealth
- **On:** Aircraft not shown on others' displays, vice versa (except close range and collision danger)
- Opts out of long-range "FLARM RADAR" mutual visibility

#### No Track
- Similar to Stealth but for ground (OGN) stations
- Tells ground stations not to report your position

#### Flight Logging
Options:
- **Airborne:** Start on takeoff, stop after landing
- **Traffic:** Airborne + output nearby aircraft as comments
- **Interval:** Seconds between recorded positions (default 4)
- See Flight Logging section for details

#### IGC Encryption Key
*Shown only if using OGNTP protocol*

- 4 sections of 8 hexadecimal digits each
- Display coding: "00000000" if zero, "88888888" if non-zero (masks true key)
- **To preserve existing key:** Leave as "88888888"
- **To change/set new key:** Overwrite with new value
- **To disable encryption:** Set to all zeros

---

### The Settings Displayed and Adjusted Within the T-Echo

Last display screen shows settings like:

```
Normal  Glider  _      < normal mode, aircraft type glider, relay landed-out aircraft
US TX P:T+A A:LAT      < US region, full power, Latest protocol + ADS-L alt-protocol, Latest alarm method
Device: 60ABCD         < device ID (internal, cannot be changed)
Aircft: A23456 >>      < aircraft ID - and this is transmitted (">>")
--A34567 ++A45678      < ignore ID and follow ID
NMEA1:BLT GT           < NMEA output to Bluetooth, sends GNSS & Traffic data
NMEA2:USB SD           < NMEA output to USB, sends Sensors and Debug data
```

#### Changing Settings
Double-click Mode button on settings screen to enter change-settings mode:

- Shows one setting at a time
- **Touch pad:** Change current setting (double-touch to move in opposite direction)
- **Short Mode press:** Move to next setting (~2 second delay between settings)

#### Exit Change-Settings Mode

**Option 1:** Single-click through all settings until "what to do next: cancel" screen
- Use touch button to select: cancel, review, or save
- Single-click Mode button to confirm

**Option 2:** Double-click Mode button on any settings page
- Saves new settings and reboots

**Option 3:** Press Reset button
- Reboots without changing settings

#### Accessible Settings
- **Aircraft Type:** Glider, Towplane, Helicopter, Powered, Hangglider, Paraglider, Dropplane, Parachute, Balloon, UAV, Static
- **RF Protocol:** LATEST, LEGACY, OGNTP, P3I, FANET, ADS-L
- **Alternate Protocol:** LATEST, LEGACY, OGNTP, ADS-L (P3I/FANET listed but not valid)
- **RF Band (Region):** EU, US, UK, AU, NZ, RU, CN, IN, IL, KR (UK only for P3I)
- **Collision Prediction Algorithm:** Latest, Vector, Distance, None
- **Relay Mode:** None, Landed, All
- **Display Units:** Metric, Imperial, Mixed
- **Display Orientation:** Track Up, North Up
- **Aircraft ID:** (tedious to enter)
- **ID Type**

**Other settings:** Edit settings.txt file. Or use Bluetooth/USB terminal app with HTML files mentioned above.

---

### About the Settings File

#### Version Changes
From version MB151, settings stored in text file within flash memory instead of binary "EEPROM" block.

#### First Boot of New Version
- Reads existing EEPROM settings if available (otherwise defaults)
- Check settings in web interface, click "Save"
- Settings then retrieved from file, EEPROM ignored
- Invalid files auto-deleted (check serial output for invalid line)

#### Backup Best Practice
Click "download" button to save settings.txt copy on phone/computer. Can then restore or edited version via "upload".

#### File Management
- **Upload:** File must be named "settings.txt"
- **Note:** Upload doesn't change current settings until reboot
- **Example workflow:**
  1. Click "backup" (copies settings.txt to settingb.txt)
  2. Upload alternative settings.txt
  3. Click "swap" (reboots with previous settings)
  4. Another "swap" reboots with alternative settings

#### File Format
Plain text, label,value pairs, one per line:
- **Labels:** Must match generated exactly (capitalization matters)
- **Comments:** Anything after space (except text fields like pilot name) is comment
- **Required line:** "SoftRF,1" (other settings skip defaults to defaults)
- **Comment syntax:** "band,2  # US/CA"

#### Common Codes

**Protocol and Alt-Protocol (P3I/FANET not valid as alternates):**
```
0 = Legacy    /* Air V6 */
1 = OGNTP
2 = P3I       /* PilotAware */
5 = FANET     /* Skytraxx */
7 = Latest    /* new 2024 V7 protocol */
8 = ADS-L
```

**Band:**
```
1 = EU     /* 868.2 MHz band */
2 = US     /* 915 MHz band */
3 = AU     /* 921 MHz band */
4 = NZ     /* 869.250 MHz band */
5 = RU     /* 868.8 MHz band */
6 = CN     /* 470 MHz band */
7 = UK     /* treated as EU, unless FANET */
8 = IN     /* 866.0 MHz band */
9 = IL     /* 916.2 MHz band */
10 = KR    /* 920.9 MHz band */
```

**Aircraft Type:**
```
0 = unknown       // marks as landed-out glider for relay
1 = glider
2 = towplane
3 = helicopter
4 = parachute
5 = dropplane
6 = hangglider
7 = paraglider
8 = powered
15 = static
16 = winch        // transmitted as static with variable altitude
```

**Alarm:**
```
0 = none
1 = distance
2 = vector
3 = latest
```

**Volume (Buzzer):**
```
0 = off
1 = low
2 = full
3 = external (active buzzer)
```

**NMEA Output Routes:**
```
0 = none
1 = main serial
2 = UDP
3 = TCP
4 = USB (same as main serial)
5 = Bluetooth
6 = aux serial
```

**Baud Rate:**
```
0 = default (38400 for main serial, OFF for aux serial)
1 = 4800
2 = 9600
3 = 19200
4 = 38400
5 = 57600
6 = 115200
```

**Bluetooth:**
```
0 = off
1 = classic (SPP)   // (T-Beam only)
2 = BLE             // (T-Echo can only use BLE)
```

**Flight Logging:**
```
0 = none
1 = always
2 = airborne
3 = also record close traffic
```

**Boolean settings:** 0 for "no", 1 for "yes" (stealth, no_track, alarmlog, compflash, etc.)

See "under the hood" document for more advanced settings details.

---

### Collision Alarms

#### Alarm Types
When nearby aircraft is possibly on collision course, SoftRF generates:
- **NMEA sentences** (PFLAA, PFLAU) sent to connected devices in standard format
- **Audio indications** (buzzer or voice on T-Beam)
- **Visual indications** (strobe)

#### Demo/Test Mode
Short-press middle button on T-Beam v1.x for simple audio/visual alarm demo (9 seconds):
- Activates demo and test of audio/visual alarms
- PFLAU output sentences pretend aircraft is airborne with collision alarm
- Helps test attached devices (FLARMviewer, XCsoar, canopy flasher)
- OLED shows "ALARM DEMO"

*T-Beam v0.7:* Long-press first button (between slide switch and reset) for same function

#### Three Alarm Levels
- **Level 3 (High urgency):** Collision possible within 9 seconds
- **Level 2 (Medium):** Collision possible within 13 seconds
- **Level 1 (Low):** Collision possible within 19 seconds (~one typical thermal turn)

#### Hysteresis
To avoid distraction, buzzer/voice doesn't repeat immediately for same aircraft unless alarm level increases. After 9 seconds may repeat.

#### Alarm Indications Summary

**Buzzer:**
- **Level 1:** Single beep
- **Level 2:** 2 faster beeps (higher pitch)
- **Level 3:** 5 quick beeps
- **Multiple threats:** Beeps "doubled up" (1 or 2 beeps) or extended to 7 beeps
- Tone increases in pitch with alarm level (passive type)

**Voice:**
- **Levels 1 & 2:** "traffic, 2 o'clock high"
- **Level 3:** "danger, ahead low"
- Multiple threats: "traffic" appended at end

**Strobe:**
- **Any alarm level:** Triple flash every 750 ms
  - 50 ms on, 50 ms off, repeat twice more
- **Periodic (if setting chosen):** Triple flash every 2400 ms
  - 40 ms on, 50 ms off, repeat twice more

---

### Winch Mode

#### Purpose
"Winch" aircraft type intended for device mounted on ground (e.g., at winch):
- **Transmitted type:** "static"
- **Transmitted altitude:** Alternates between 100, 200, 300, 400 meters AGL (1 second each, repeat)

#### Operation
Middle button click on T-Beam v1.x toggles between:
- **Full power transmissions** (active)
- **No transmissions** (paused)

**Purpose:** Turn on warnings before launches, pause during launch pauses

**Indicators:**
- OLED display shows transmissions active/inactive
- Blue LED shows active/inactive

**Note:** Middle button demo function NOT available in winch mode

*T-Beam v0.7:* Long-press first button for same function

---

### Uploading Voice Files (T-Beam Only)

#### Voice Warnings Activation
If voice warnings enabled but audio not uploaded, warnings say "traffic" only.

#### Getting Full Voice Messages
1. Download waves.tar file: https://github.com/moshe-braner/SoftRF/tree/master/software/data/Audio
2. Boot SoftRF and access web interface
3. Click "upload" button (bottom of status page)
4. Browse to downloaded waves.tar file, click "upload"
5. Status page shows "17 WAV files found"

#### File Storage
- **Location:** SPIFFS (flash file-system area) within T-Beam flash memory
- **Persistence:** Not erased during firmware updates
- **Clearing:** Use "clear" button in web interface if needed

#### Creating Custom Voice Files

**File requirements:**
- 17 files needed with standard names (from supplied template)
- **Format:** PCM, mono, 8-bit, 8000 Hz sample rate
- **Tool example:** Free PC app "Wavosaur" for format conversion
- **Total size:** No more than ~150 KB
- **Compression:** Use file utility (7ZIP) to combine into TAR file (NOT ZIP)
- **TAR file:** File order doesn't matter, name doesn't matter (always saved as waves.tar)

---

### How to Connect External Devices

#### Overview
SoftRF can send data to other devices in several formats, primarily **NMEA sentences**.

#### Example NMEA Sentence
```
$PFLAU,1,0,2,1,0,14,2,0,57,A8B031*7C
```
Line of text describing nearby aircraft in FLARM format.

GNSS data sent in NMEA sentences along with traffic data.

#### External Device Types

**Dedicated Traffic Display:**
- [SkyView EZ](https://github.com/lyusupov/SoftRF/wiki/SkyView-EZ)
- [Improved SkyView firmware](https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/binaries/ESP32/SkyView)

**Glide Computers/E-readers:**
- ILEC SN10
- E-readers, phones, tablets running:
  - XCsoar
  - Tophat
  - Other navigation software

**Data types:** Some devices benefit from SoftRF also sending GNSS position data (e.g., e-reader without GPS)

#### Connection Methods

**Wireless (convenient):**
- Serial cable not needed
- Select relevant SoftRF settings (above)
- **WiFi UDP:** [XCsoar quick start](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition-MkII.-Quick-start)
  - Traffic visible on moving map in XCsoar
  - FLARM RADAR screen available
  - Spoken collision warnings: "Traffic, 2 O'Clock, High"
- **WiFi TCP:** Similar to UDP
- **Bluetooth:** Classic or LE

**Wired (various hardware needed):**
- USB cable: PC terminal software (e.g., "Termite") displays data - useful for debugging
- Serial ports: TTL-level pins RX/TX on T-Beam (but RX not usable, use VP for input)
- Secondary serial: Pins 39 ("VN") RX, pin 4 TX (VP on T-Beam v0.7)

#### Voltage Level Considerations
T-Beam pins operate at **TTL levels** (0V and ~+3V)

Many devices use **RS232 levels** (several volts negative and positive)

**Level conversion needed:** MAX232 chip or similar interface

**Considerations:**
1. Limited case space for conversion board
2. Cheap MAX232 variants may be unstable/oscillate, excessive current draw
3. **Protection recommendations:**
   - Unused TTL _inputs_ connected to +3.3V via ~22kΩ resistor (don't leave open)
   - MAX232 power via ~39Ω resistor to limit current if misbehaves

**Alternative:** Try T-Beam secondary serial with "invert" setting first (see Data Bridging section)

#### Connection Setup
Set **baud rate on external device to match SoftRF** (38400 default) or vice versa

**Note:** T-Beam sends data at 115,200 baud during startup, then switches to selected rate. Devices with auto-detection may get confused - set specific baud rate.

#### Barometric Sensor
**BMP280 or BMP180 type can connect via I2C:**
- Pins 21 (SDA) and 22 (SCL), OR
- Pins 13 (SDA) and 2 (SCL)
- **Note:** Pin 2 should be easily disconnectable (see USB flashing section)
- **Sharing:** Can share pins with OLED display and AXP PMU
- **Data output:** Sent as NMEA sentences if "sensors" output enabled

#### Pinout Diagram
https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/documentation/T-beam_wiring.jpg

---

### Connecting an SD Card Adapter

#### SD Card Preparation
1. Micro-SD card: **32 GB or smaller** (1GB plenty)
2. **Format:** FAT32
3. **Allocation size:** Small (1024, 512, or 2048 bytes) to reduce wear from re-writes
4. **Folders:** Create "firmware" and "logs" at top level
   - **firmware folder:** Place firmware update file renamed as "SoftRF.bin"
   - **logs folder:** Flight logs stored here
5. **Firmware updates:** SoftRF boot looks for firmware file, updates and deletes it

#### After Card Installation
Normally no physical access needed:
- Web interface allows listing, downloading, clearing flight logs
- **File list:** View files in "logs" folder via web interface
- **All files:** Use URL `.../listsdall` to see all files in /logs folder
- **Upload files:** Use `.../logupload` URL

#### Web Interface Functions
- **List flight logs:** View all logs
- **Download:** Get latest log or any from list
- **Clear:** Move to "old" subfolder
- **Empty trash:** Delete all in "old" subfolder

For detailed connection instructions, see "under the hood" document.

---

### Flight Logging

#### T-Beam Flash Memory Limitations
- Very little space available for flight logging
- Flash memory write cycles limited
- After ~2000 hours logging at 4-second intervals: some flash failure possibility
- **For most users:** Not a problem

#### T-Beam Logging Options

**To RAM (PSRAM):**
- Volatile (volatile means lost on power-off)
- Limited to 3.5 megabytes
- Single combined log between SoftRF startup/shutdown
- **Must download before power-off**

**To Flash (compressed):**
- Non-volatile (survives power-off)
- Optionally archive in compressed format
- Enable via "compflash" setting = "1"
- Persist even powered off
- Space available: ~12 hours at 4-second intervals (depends on other stored data)

**To SD Card:**
- See SD card section
- Best option for long-term storage and multiple flights

#### T-Beam Logging Destination
Automatic based on SD file system mount status:
- **SD present:** Logs to SD
- **No SD:** Logs to RAM/flash

Status page shows appropriate message and buttons:
- **RAM/Flash:** "download current", "List Archived", "Clear Archive"
- **SD Card:** "Download Latest", "List Files", "Clear Files"

#### T-Echo Logging
Non-volatile flash memory only (insufficient RAM):
- Default: Uncompressed
- Optional: Compressed format (enable "compflash" = "1")
- **Space limit:** ~2 megabytes (~12 hours at 1 fix/second)
- **Free space requirement:** Need 3+ hours logging space or log won't start
- Best practice: Select "airborne" + less frequent interval, download/clear periodically
- **Aircraft database trade-off:** Space shared with optional aircraft database

#### Pre-Flight Setup
Enter settings before creating logs:
- **igc_pilot:** Pilot name
- **igc_aircraft_type:** Aircraft model
- **igc_aircraft_reg:** Aircraft registration ID
- **igc_callsign:** Contest/tail ID

**Method:** Download settings.txt, edit, upload

Data embedded in IGC flight log files.

#### Logging Modes

**Airborne:**
- File starts on takeoff detection
- Finalizes on landing
- New file created on next takeoff

**Always:**
- Continuous data collection
- Single file throughout

**Traffic:**
- Same as Airborne
- Plus outputs nearby aircraft as comments

**Alarms:** If "log alarms" enabled, alarm traffic logged even in non-traffic mode

#### Compression
Simple algorithm designed to make position ("B") records ~5x smaller:
- Other IGC file lines not compressed
- If logging nearby traffic as comments with lots of traffic: almost as large as uncompressed

#### Downloading Flight Logs

**T-Beam:**
- Use "download" or "list" buttons in flight logs web section
- Compressed logs automatically decompressed on download

**T-Echo:**
- Connect to computer via USB (files window opens)
- Download by dragging files out

**T-Echo Compressed Logs (2 options):**

1. **Drag and decompress on computer:**
   - Windows program "igz2igc.exe" available at https://github.com/moshe-braner/SoftRF/tree/master/software/app
   - Command-line program (drag-and-drop file onto icon decompresses)
   - **Method:** Copy .IGZ file to computer hard drive, then drag onto program icon
   - Output: .IGC file in same folder

2. **Have SoftRF decompress:**
   - Connect T-Echo, open files window
   - Delete old logs no longer needed
   - Note: File date stamps on T-Echo incorrect
   - Identify flight date from first 3 filename characters: year digit, month, day (1-9 and A-V for 10-31)
   - Copy .IGZ files to computer for safety
   - Delete T-Echo files if needed for space
   - **Free space requirement:** 6x the file size
   - Rename xxxxxxxx.IGZ to xxxxxxxx.IGX
   - "Safely disconnect" from computer
   - **Reboot T-Echo** (press reset button)
   - SoftRF looks for .IGX files and decompresses
   - Display shows "DECOMP 5A9_1" during decompression
   - When complete: "DECOMP DONE"
   - .IGX files deleted after decompression
   - Reconnect to computer and retrieve .IGC files
   - If space insufficient to decompress all: some may remain
   - If any .IGX remain (or drag back): repeat from "safely disconnect"

#### Log Finalization
**T-Beam:**
- Long-press pushbutton shutdown (v1.x), or alarm demo activation (v0.7)
- Web interface button clicks
- Finalizes active flight log

**T-Echo:**
- Long-press Mode button shutdown
- Finalizes and retains in flash

**Simple power disconnection:**
- Flight log likely valid but may be missing last few minutes

#### Flight Log Security
- Files have G-records appended
- Accepted as valid on Weglide and OLC
- **NOT IGC-certified:** Cannot use for badges and records

---

### Data Bridging

#### Overview
Feature for T-Beam versions 0.7-1.2 allows NMEA sentences from external devices to be copied to NMEA destination devices.

#### Forwarding Rules
- **Forwarded:** External device NMEA sentences (if "external" output enabled in settings)
- **NOT echoed back:** Same port they came from
- **Never forwarded:**
  - GNSS (GPS) position sentences (originate from SoftRF too - redundant)
  - FLARM-format traffic messages (same reason, may echo)
- **Processed:** $PSRF... config sentences

#### Example Use Case
Connect S7 vario via bidirectional serial cable:
1. SoftRF configured to send position and traffic sentences
2. S7 echoes them back with added vario data
3. XCsoar (on phone) receives via WiFi/Bluetooth/other serial
4. S7 vario data forwarded to XCsoar by SoftRF
5. XCsoar commands forwarded back to S7
6. **Result:** Eliminates need for Bluetooth dongle

#### Main Serial Port Enhancement
**Old limitation:** USB jack "RX" pin works without USB power, but not with it. Can't have two input sources on same pin.

**Solution:** New setting allows using pin 36 ("VP") instead of "RX"
- **Effect:** Serial input from USB disabled
- **Note:** May not work well at high baud rates on T-Beam v0.7 (capacitor between GPIO36/37)
- **Serial output:** USB and TX pin unaffected

#### Secondary Serial Port
Pins 39 ("VN") RX and pin 4 TX (VP on T-Beam v0.7):
- **Shared with:** Red LED on T-Beam v1.x
- **LED behavior:** Turns off when secondary serial active, flickers during transmit

#### Data Bridging Capabilities
Two serial ports enable:
- Forward data one-to-other
- Combine data from both + send to wireless stream
- **Result:** Eliminates need for IOIO box

#### Secondary Serial Logic Inversion
**On secondary port only:** Can invert logic
- TTL voltage level (not RS232)
- With inversion: Can connect to devices claiming "RS232" but accepting 0V as "low" and 3V as "high"
- **Advantage:** No MAX232 conversion chip needed
- **Disadvantage:** Need RS232 input protection

**Protection circuit (if input needed):**
- 3.3V zener diode (pin to ground)
- Schottky diode in parallel
- 1.2K resistor (pin to external wire)
- **Output:** Should protect too (may be accidentally connected to RS232)
- **Input:** Needs pull-up resistor, 10kΩ (pin to +3.3V)

**Example protection circuit:** https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/documentation/rs232_wiring.jpg

**Inversion indicator:**
- Normal: Red LED on, flickers off during transmit
- Inverted: Red LED mostly on, flickers dimmer during transmit

#### TCP/UDP Input
- **TCP:** Only active if TCP set up as NMEA destination
- **UDP:** Port 10110 for NMEA output
  - If UDP chosen for output: output on 10110, input on 4352

---

### Connecting an ADS-B Receiver

#### Option 1: External ADS-B Device (GDL90 Format)
SoftRF accepts traffic data in **GDL90 format** from external device:
- Input ports: Serial, WiFi/UDP, WiFi/TCP
- **Not Bluetooth** (protocol incompatibility)
- **Processing:** Only other-ship traffic messages
- **Filtering:** Ignores traffic >15 nm away or ±2000m altitude difference
- **NMEA:** Not sent to GDL90 input source

#### Option 2: Hardwired GNS5892 Module (T-Beam v1.x)
Small ADS-B receiver module fits inside T-Beam case:

**Connection:**
- Auxiliary serial port
- Wiring diagram: https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/documentation/GNS5892_wiring.jpg
- (T-Beam v0.7: use "VP" instead of "VN")

**Actual installation photos:**
- https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/documentation/GNS5892_T_Beam_1_.jpg
- https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/documentation/GNS5892_T_Beam_2_.jpg

**Antenna options:**
- Vertical wire: ~65 mm long
- Short coaxial cable from module to case jack (e.g., SMA)
- Short whip antenna (900 MHz tuned or 800-1200 MHz range)
- ADS-B signals ~1000x stronger than FLARM - poor antenna acceptable

**Range limitations:** Software ignores traffic >15 nm away or ±2000m altitude

**Side effect:** Auxiliary serial port becomes unavailable
- Appears "disabled" in baud-rate setting but actually active at 921600 baud
- Keep wires short

#### ADS-B Data Output
For situational awareness:
- Collision alarms NOT generated for TIS-B traffic (imprecise location)
- Vector algorithm (not "latest") for ADS-R traffic collision prediction (time delays)

---

### Connecting to XCvario

#### Wired Connection
Custom-built serial cable:
- XCvario claims to work at TTL levels
- **No RS232 conversion chip needed**
- XCvario can forward data (with added vario data) to other devices
- Wired or wireless capability

#### Wireless Connection
More complex - requires firmware coordination:

**SoftRF Configuration:**
- WiFi client: SSID "XCVario-XXXX", password "xcvario-21"
- TCP Client: IP 192.168.4.1, port 8880
- Primary NMEA output: TCP
- Secondary NMEA output: UDP

**XCvario Configuration:**
- Wireless: WiFi (Master or Standalone, not Bluetooth)
- Wireless data route: Connected to "vario"

**XCsoar Configuration:**
- Connect to same XCvario WiFi network
- Device 1: TCP client, port 8880, XCvario protocol
- Device 2: UDP listener, port 10110, FLARM protocol

**Result:** Works with both XCvario and XCsoar simultaneously!

**Future:** Custom XCvario firmware (and possibly official version) may accept data on port 2000, freeing 8880.

---

### Updating the Firmware on the T-Beam

#### Easy Method: Over-the-Air (OTA) via WiFi
Easiest way to install newer firmware versions.

**Important Note:** If T-Beam has "RC9" version installed, **DO NOT update via WiFi** - there's a bug. Use USB method instead. After RC9 update, WiFi updates will work.

#### Step-by-Step OTA Update

1. **Download firmware:**
   - https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/binaries/ESP32/SoftRF
   - Unzip - actual firmware file has .bin suffix, ~1.7 MB

2. **Fully charge** T-Beam battery

3. **Boot SoftRF**

4. **Connect to WiFi Access Point:**
   - SSID: SoftRF-XXXXXX
   - Key: 12345678
   - *(Or connect to external WiFi if SoftRF configured as client)*

5. **Open web browser:** 192.168.1.1
   - *(If external network: press button to find IP on OLED screen 4-5)*

6. **Check status page:** Note firmware version listed

7. **Go to Settings page:** Click button at lower-left of Status page

8. **Record settings:** Write down or screenshot current settings. Return to status page.

9. **Click "Firmware update" button**

10. **Select firmware:** Click "Choose file", select .bin file

11. **Update:** Click "Update" button

12. **Wait:** "UPDATE DONE, REBOOTING" message appears (may take 1+ minute)

13. **Monitor progress:**
    - Blue LED flashes 1/sec during update
    - Stays on several seconds when done
    - **If flashes 4/sec:** Update failed (before reboot)
    - OLED shows incrementing counter (if equipped)

14. **Post-update wait:** Don't touch for another minute to avoid "bricked" device

15. **Automatic reboot:** Usually reboots by itself
    - If not after 1-2 minutes: Press reset (third) button

16. **Verify update:**
    - Reconnect to WiFi
    - Check status page for new firmware version
    - Also shown on OLED with settings preservation status

17. **Check settings:** Restore if necessary
    - Some, all, or none may persist depending on version difference

#### Potential WiFi Issues
If strong external network with internet available:
- PC/tablet may switch networks during update
- Update won't complete (OLED count stops incrementing)
- **Solutions:**
  - Try different device or more distance from external network
  - Turn off external network
  - Change SoftRF settings to connect to external network instead

#### Failed Update
If OTA fails (always fails from RC9 version):
- Use USB method described in "Installing firmware for the first time" section
- USB method usually succeeds even if T-Beam won't boot

#### With Screenshots
https://github.com/lyusupov/SoftRF/wiki/Firmware-update-%28Web-method%29#esp32

---

### Updating the Firmware on the T-Echo

T-Echo usually includes bootloader for easy updates:

1. **Download firmware:**
   - https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/binaries/nRF52840/SoftRF/MassStorage
   - Unzip to get .uf2 file

2. **Connect T-Echo to PC:** USB Type-A <-> Type-C cable

3. **Open as mass storage device** (flash drive)

4. **Delete large files:** Flight logs, etc. (may need space)

5. **Eject drive:** Finalize deletion (keep cable connected)

6. **Double-click RESET button:** Two clicks within 0.5 seconds

7. **Wait for bootloader:** Virtual mass storage device labeled NRF52BOOT (or TECHOBOOT) appears

8. **Drag firmware file:** Drag .uf2 file into bootloader window

9. **Wait for transfer:** ~30 seconds. Window closes, T-Echo reboots.

#### Reference
https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#nrf52840

---

## PART 2: Under the Hood

See separate document: [SoftRF_MB_under_the_hood.md](SoftRF_MB_under_the_hood.md)

---

**For more information and updates:**
- GitHub: https://github.com/moshe-braner/SoftRF
- Discussion Group: https://groups.google.com/g/softrf_community
