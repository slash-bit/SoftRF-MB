# iPhone & iOS - Web App Guide

## iOS Browser Requirements

Web Bluetooth requires **iOS 14.6 or newer**. The built-in Safari browser supports it natively — no third-party apps needed on modern iOS.

For older iOS (13.x and below), **Bluefy** (App Store) provides a WebView with BLE bridging.

---

## Installing as a Home Screen App (Safari)

1. Open `index.html` in **Safari**
2. Tap the **Share** button (bottom centre)
3. Select **"Add to Home Screen"**
4. Tap **"Add"**

The icon appears on your home screen and opens full-screen with BLE working normally.

---

## Using Bluefy (iOS 13 or older)

1. Install **Bluefy** from the App Store
2. Open Bluefy and paste the URL to `index.html`
3. BLE works the same as Chrome on Android

Bluefy is only needed if your iOS version is older than 14.6.

---

## Firmware Update via BLE (T1000E)

The webapp supports triggering a firmware update from the Status tab when connected via BLE.

### Prepare the firmware package

Convert the `.uf2` build output to a Nordic DFU `.zip` using `adafruit-nrfutil`:

```sh
pip install adafruit-nrfutil
adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application firmware.hex firmware_dfu.zip
```

To get `.hex` from `.uf2`:
```sh
python uf2conv.py firmware.uf2 -c -D
```

### Trigger DFU from the webapp

1. Connect to the device via BLE
2. Go to the **Status** tab → **Device Control**
3. Tap **"Update Firmware"** → **"Enter DFU Mode"**
4. The device disconnects and restarts into the bootloader

### Upload firmware on iPhone

**Option A — USB mass storage** (simplest): connect the device via USB — a drive appears, drag the `.uf2` onto it.

**Option B — BLE OTA with nRF Toolbox**:
1. Install **nRF Toolbox** from the App Store
2. Open nRF Toolbox → tap **DFU**
3. Select the `DfuTarg` device
4. Select the `firmware_dfu.zip` package
5. Upload starts automatically

**Option C — nRF Connect**:
1. Open **nRF Connect** → scan → connect to `DfuTarg`
2. Tap the DFU icon → select `firmware_dfu.zip`

---

## Troubleshooting

**BLE not working**
Check iOS version is 14.6 or newer: Settings → General → About → iOS Version. If older, use Bluefy.

**"Add to Home Screen" not visible**
Must be in Safari. Tap the Share button (bottom centre), scroll the share sheet to find it.

**Device doesn't enter DFU after tapping "Enter DFU Mode"**
Firmware must be built and flashed with the `PSRFC,DFU` handler (added in this branch). The current production firmware does not support this command — flash the new firmware via USB drag-drop first, then BLE DFU is available for future updates.
