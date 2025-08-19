# Relio
Relio : A Matter-Enabled Smart Controller for AC Appliances 
**Version:** v1  

## 📖 Project Overview

This project is a **Matter-enabled smart controller** built around the ESP32, with support for **relay control, environment sensing, energy monitoring, and occupancy detection**.  
It combines **hardware (KiCAD PCB design)** and **firmware (Arduino/C++ code)** into a single, ready-to-build system.

---

## 📂 Repository Structure

├── hardware/ # KiCAD PCB design files (schematics & layout)
└── src/ # Source code (Arduino / ESP-IDF compatible)


## ⚙️ Features

- **3x On/Off Lights** 💡  
  Controlled through a TCA9554 I²C expander.  
  - Always start **OFF** when powered up (safe startup).  
  - Exposed as **separate Matter endpoints**.  

- **Environmental Sensing (BME688)** 🌡️💧  
  - Temperature & Humidity exposed to Matter.  
  - Pressure & Gas Resistance logged to Serial for debugging.  

- **Energy Monitoring (BL0942)** ⚡  
  - Measures Voltage, Current, Power, Energy, Frequency.  
  - Data shown in Serial logs (Watts/Current can later be exposed to Matter).  

- **Occupancy Detection** 🚶‍♂️  
  - Uses TCA9554 pin 0 as input.  
  - HIGH = Occupied, LOW = Not Occupied.  
  - Reflected in Matter occupancy endpoint.  

- **Commissioning & Control** 🔐  
  - Commission into Matter fabric with QR/Manual code.  
  - BOOT button short press = toggle Light 1.  
  - BOOT button long press (5s) = decommission node.  

---

## 🧑‍💻 How the Code Works

- **Relay Control:**  
  `SWITCH1/2/3` booleans track relay states. Callbacks ensure Matter state and hardware (TCA9554 relays) stay in sync.  

- **BME688:**  
  Reads real temperature/humidity every 5s → updates Matter.  
  Pressure/gas logged for debugging but not exposed to Matter (yet).  

- **BL0942:**  
  Runs on `Serial1` (pins RX=4, TX=5, 4800 baud). Provides live power metrics to Serial monitor.  

- **Occupancy:**  
  Directly read from **TCA9554 pin 0**. When state changes, it’s pushed to the Matter controller and printed to Serial.  

---

## 🚀 Getting Started

1. Clone this repo:  
   ```bash
   git clone https://github.com/ElektorLabs/Relio.git
2. Open the src/ folder in Arduino IDE or PlatformIO.

3. Install dependencies:

   * [Espressif Matter Arduino](https://github.com/espressif/arduino-esp32)
   * [TCA9554 library](https://github.com/RobTillaart/TCA9554)
   * [BL0942 library](https://github.com/santerilindfors/BL0942)
   * [BME688 library](https://github.com/styropyr0/BME688)

4. Upload the code to your ESP32.

5. Use your Matter controller app to commission the device.

🛠️ Future Work

- Expose BL0942 Power/Current as native Matter attributes.

- Add Matter endpoints for Pressure & Gas Resistance.

- Implement OTA updates & web-based config.

- Add configurable boot behavior (restore last state vs always OFF).

## ⚠️ Disclaimer
This project is provided as-is, without warranty of any kind.

## ⚡ Caution: This project involves mains-level voltages (relays, energy metering).
Only experienced users should build or test this hardware. Always follow electrical safety best practices when working with AC-powered circuits.

The author is not responsible for any damage, injury, or loss caused by misuse of this code or hardware.

