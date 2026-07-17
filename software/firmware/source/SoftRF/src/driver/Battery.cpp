/*
 * BatteryHelper.cpp
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
#include <Arduino.h>
#endif

#include "../system/SoC.h"
#include "Settings.h"
#include "Battery.h"
#include "GNSS.h"
#include "Filesys.h"

unsigned long Battery_TimeMarker        = 0;

static float Battery_voltage_cache      = 0;
static int Battery_cutoff_count         = 0;

void Battery_setup()
{
  SoC->Battery_setup();

  Battery_voltage_cache = SoC->Battery_param(BATTERY_PARAM_VOLTAGE);
  Battery_TimeMarker = millis();
}

float Battery_voltage()
{
  return Battery_voltage_cache;
}

/* low battery voltage threshold */
float Battery_threshold()
{
  return SoC->Battery_param(BATTERY_PARAM_THRESHOLD);
}

/* Battery is empty */
float Battery_cutoff()
{
  return SoC->Battery_param(BATTERY_PARAM_CUTOFF);
}

/* Battery charge level (in %) */
uint8_t Battery_charge() {
  return (uint8_t) SoC->Battery_param(BATTERY_PARAM_CHARGE);
}

/*
 * When set to run on external power but with a battery installed, allow running
 * on the battery as long as still airborne.  Shut down after at least an hour
 * of operation, once external power is turned off, and battery voltage is
 * somewhat down.  For now only implemented for T-Beam (and partially for T-Echo).
 */
static bool had_ext_power = false;
static bool follow_ext_power_shutoff(float voltage)
{
    if (! settings->power_ext)
        return false;
//  if (hw_info.model != SOFTRF_MODEL_PRIME_MK2)
//      return false;
#if defined(ESP32)
    if (ESP32_onExternalPower())
#else
    if (voltage >= 4.1)
    // not great since after full charge it might be >4.1 witout ext power
    // - but if settings->power_ext is off then no problem
#endif
    {
        had_ext_power = true;
        return false;
    }
    if (had_ext_power == false)
        return false;
    if (ThisAircraft.airborne)
        return false;
    if (voltage >= 3.9)
        return false;
    if (millis() < 3600000)
        return false;
    return true;
}

void Battery_loop()
{
  if (isTimeToBattery()) {
    float voltage = SoC->Battery_param(BATTERY_PARAM_VOLTAGE);

    int reason = SOFTRF_SHUTDOWN_NONE;
    if (voltage < Battery_cutoff())
        reason = SOFTRF_SHUTDOWN_LOWBAT;
    else if (follow_ext_power_shutoff(voltage))
        reason = SOFTRF_SHUTDOWN_EXTPWR;
    if (voltage > BATTERY_THRESHOLD_INVALID && reason != SOFTRF_SHUTDOWN_NONE) {
      if (Battery_cutoff_count > 3) {
        shutdown(reason);
      } else {
        Battery_cutoff_count++;
      }
    } else {
      Battery_cutoff_count = 0;
    }

    Battery_voltage_cache = voltage;
    Battery_TimeMarker = millis();
  }
}

/* ---- Battery log ---- */
#if defined(FILESYS)
#define BATTERYLOG_FILE    "/batterylog.txt"
#define BATTERYLOG_MAXLINES 20

static bool BatteryLog_PowerOn_done = false;

void BatteryLog_write(uint8_t on_off)
{
    if (!FS_is_mounted) return;

    char line[56];
    snprintf(line, sizeof(line), "%04d-%02d-%02d %02d:%02d:%02d,%d,%.2f,%d\r\n",
        gnss.date.year(), gnss.date.month(), gnss.date.day(),
        gnss.time.hour(), gnss.time.minute(), gnss.time.second(),
        (int)Battery_charge(), Battery_voltage(), on_off);

    /* --- read existing lines --- */
    char lines[BATTERYLOG_MAXLINES][56];
    int count = 0;
    if (FILESYS.exists(BATTERYLOG_FILE)) {
        File f = FILESYS.open(BATTERYLOG_FILE, FILE_READ);
        if (f) {
            while (count < BATTERYLOG_MAXLINES && f.available()) {
                int len = 0;
                char c;
                while (f.available() && len < 55) {
                    c = f.read();
                    if (c == '\n') break;
                    if (c != '\r')
                        lines[count][len++] = c;
                }
                lines[count][len] = '\0';
                if (len > 0) count++;
            }
            f.close();
        }
    }

    /* --- trim: keep last (MAXLINES-1) entries, then append new --- */
    int start = 0;
    if (count >= BATTERYLOG_MAXLINES)
        start = count - (BATTERYLOG_MAXLINES - 1);

    FILESYS.remove(BATTERYLOG_FILE);
    File f = FILESYS.open(BATTERYLOG_FILE, (O_WRITE | O_CREAT));
    if (f) {
        for (int i = start; i < count; i++) {
            f.write((const uint8_t *)lines[i], strlen(lines[i]));
            f.write((const uint8_t *)"\r\n", 2);
        }
        f.write((const uint8_t *)line, strlen(line));
        f.close();
        Serial.print(F("Battery log: "));
        Serial.print(line);
    }
}

void BatteryLog_PowerOn()
{
    if (!BatteryLog_PowerOn_done) {
        BatteryLog_write(1);
        BatteryLog_PowerOn_done = true;
    }
}

void BatteryLog_PowerOff()
{
    if (GNSSTimeMarker != 0)   /* only log if we had a GPS fix (valid datetime) */
        BatteryLog_write(0);
}

#endif /* FILESYS */
