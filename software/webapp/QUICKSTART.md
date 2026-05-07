# Quick Start

## Connect to the Device

Open `index.html` in a browser, then tap/click **BLE** to connect via Bluetooth, or **USB** to connect via USB Serial (Web Serial API).

**Browser requirements:**
- Chrome / Edge on Android or desktop: full support
- Safari on iOS 14.6+: BLE supported
- Firefox: Web Bluetooth not supported

---

## Install as a Home Screen App

**Android (Chrome):** tap ⋮ → "Add to Home Screen" or "Install app"

**iPhone (Safari):** tap Share → "Add to Home Screen"

The app caches itself for offline use. BLE works without internet.

---

## Update Firmware (T1000E)

From the **Status** tab when connected via BLE:
1. Tap **"Update Firmware"** → **"Enter DFU Mode"**
2. Device restarts into bootloader
3. Upload firmware via USB drag-drop (`.uf2`) or via **nRF Toolbox** (`.zip`)

See [IPHONE_BLUEFY_GUIDE.md](IPHONE_BLUEFY_GUIDE.md) for details on preparing the firmware package and using nRF Toolbox.

---

## Further Reading

- [IPHONE_BLUEFY_GUIDE.md](IPHONE_BLUEFY_GUIDE.md) — iOS setup and firmware update details
- [TECHNICAL_DETAILS.md](TECHNICAL_DETAILS.md) — developer notes
