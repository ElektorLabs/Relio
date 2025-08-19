/*
  Project: Relio : Multi-Endpoint Matter Device with Environmental & Energy Monitoring
  Version: v1
  Author : Saad Imtiaz
  Date   : 16 August 2025

  ---------------------------------------------------------------------------
  Functionality:
  - Implements a Matter node with multiple endpoints:
      • 3x On/Off switches mapped to relays via TCA9554 I²C expander
        → always forced OFF at startup (hardware + Matter state)
      • Temperature & Humidity via BME688 (I²C @ 0x76)
        → real sensor data exposed to Matter, pressure & gas printed to Serial
      • Occupancy sensor controlled by a boolean variable (OCCUPANCY)
      • Energy monitoring via BL0942 (UART on pins RX=4, TX=5)
        → Voltage, Current, Power, Energy, Frequency printed to Serial
  - Supports commissioning / decommissioning into a Matter fabric
    • Short press on BOOT button toggles Light 1
    • Long press (5s) decommissions the node

  ---------------------------------------------------------------------------
  Features Implemented:
  ✔ 3x Relay control with Matter synchronization
  ✔ Safe startup (all relays OFF, state reflected in controller)
  ✔ Real-time Temperature & Humidity from BME688
  ✔ Pressure & Gas resistance (BME688) logged to Serial
  ✔ Occupancy endpoint.
  ✔ BL0942 energy meter integration (Serial monitoring)
  ✔ Commissioning status & QR/manual pairing codes over Serial

  ---------------------------------------------------------------------------
  Features To Be Added (future roadmap):
  - Expose BL0942 power/current data as official Matter endpoint(s)
  - Add optional Matter endpoint for BME688 Pressure & Gas Resistance
  - User-configurable boot policy (restore last state vs always-off)
  - Error handling / fallback if BME688 or BL0942 not detected
    ---------------------------------------------------------------------------
*/

#include <Matter.h>
#if !CONFIG_ENABLE_CHIPOBLE
#include <WiFi.h>
#endif
#include <Preferences.h>
#include <Wire.h>
#include "TCA9554.h"
#include <BME688.h>


// ====== BL0942 Energy Meter ======
#include <BL0942.h>
#define BL0942_RX 4
#define BL0942_TX 5
bl0942::BL0942 blSensor(Serial1);

// === Compile-time option: expose Watts (or Current) in Matter ===
// Set to 1 to enable, and make sure your SDK provides a suitable endpoint.
// Common names you might have in recent Arduino Matter builds include something like
// MatterElectricalMeasurement / MatterPowerMeter / MatterPowerSensor.
//
// If you enable, adjust the include/class/methods in the MARKED SECTION below.
#define EXPOSE_WATTS_IN_MATTER 0   // 0 = Serial only (safe default), 1 = publish to Matter (needs endpoint support)
#define PUBLISH_WATTS           1   // 1 = publish watts, 0 = publish current (amps) if endpoint supports it


// WiFi credentials if not using CHIP over BLE
#if !CONFIG_ENABLE_CHIPOBLE
const char *ssid = "xxx";
const char *password = "xxx";
#endif


// ---------- Matter Endpoints ----------
MatterOnOffLight        OnOffLight1;
MatterOnOffLight        OnOffLight2;
MatterOnOffLight        OnOffLight3;
MatterTemperatureSensor TempSensor;
MatterHumiditySensor    HumSensor;
MatterOccupancySensor   Occupancy;
// ---------- Logical states ----------
bool SWITCH1 = false;
bool SWITCH2 = false;
bool SWITCH3 = false;
bool OCCUPANCY = false;  // set true/false in your app logic

// ---------- Persistence ----------
Preferences matterPref;
static constexpr const char *kNS        = "MatterPrefs";
static constexpr const char *kOnOffKey1 = "OnOff1";
static constexpr const char *kOnOffKey2 = "OnOff2";
static constexpr const char *kOnOffKey3 = "OnOff3";

// ---------- Button / Decommission ----------
const uint8_t buttonPin = BOOT_PIN;
uint32_t button_time_stamp = 0;
bool button_state = false;
const uint32_t debouceTime = 250;              // ms
const uint32_t decommissioningTimeout = 5000;  // ms

// ---------- TCA9554 (I²C expander) ----------
TCA9554 TCA(0x20);                    // change I2C address if needed
const bool RELAY_ACTIVE_HIGH = true;  // set false if your relays are active-LOW

// Map switches to expander pins (0..7)
const uint8_t RELAY1_PIN = 1;
const uint8_t RELAY2_PIN = 2;
const uint8_t RELAY3_PIN = 3;

// ---------- BME688 (Temp/Humidity/Pressure/Gas) ----------
BME688 bme;        // I²C sensor at 0x76

// Store last BL0942 readings & print timing
struct {
  float voltage = NAN;
  float current = NAN;
  float powerW  = NAN;
  float energy  = NAN;
  float freqHz  = NAN;
  uint32_t lastPrintMs = 0;
} em;

void bl0942DataReceived(bl0942::SensorData &data) {
  em.voltage = data.voltage;
  em.current = data.current;
  em.powerW  = data.watt;
  em.energy  = data.energy;
  em.freqHz  = data.frequency;
}

// ---------- Relay helpers ----------
inline uint8_t relayLevelFor(bool on) {
  if (RELAY_ACTIVE_HIGH) return on ? HIGH : LOW;
  else                   return on ? LOW  : HIGH;
}

void driveRelay(uint8_t pin, bool on) {
  if (pin > 7) return;
  TCA.write1(pin, relayLevelFor(on));
}

void applyAllRelaysFromState() {
  driveRelay(RELAY1_PIN, SWITCH1);
  driveRelay(RELAY2_PIN, SWITCH2);
  driveRelay(RELAY3_PIN, SWITCH3);
}

// ---------- Hard "All OFF on Boot" ----------
void forceAllOffOnBoot() {
  // Logical variables
  SWITCH1 = false;
  SWITCH2 = false;
  SWITCH3 = false;

  // Persist OFF so any later reads are consistent (we ignore prefs at boot)
  matterPref.putBool(kOnOffKey1, false);
  matterPref.putBool(kOnOffKey2, false);
  matterPref.putBool(kOnOffKey3, false);

  // Hardware OFF immediately (before Matter starts)
  driveRelay(RELAY1_PIN, false);
  driveRelay(RELAY2_PIN, false);
  driveRelay(RELAY3_PIN, false);
}

// ---------- On/Off callbacks (drive relays + persist) ----------
bool setLight1(bool state) {
  Serial.printf("Light 1 -> %s\r\n", state ? "ON" : "OFF");
  SWITCH1 = state;
  matterPref.putBool(kOnOffKey1, state);
  driveRelay(RELAY1_PIN, state);
  return true;
}
bool setLight2(bool state) {
  Serial.printf("Light 2 -> %s\r\n", state ? "ON" : "OFF");
  SWITCH2 = state;
  matterPref.putBool(kOnOffKey2, state);
  driveRelay(RELAY2_PIN, state);
  return true;
}
bool setLight3(bool state) {
  Serial.printf("Light 3 -> %s\r\n", state ? "ON" : "OFF");
  SWITCH3 = state;
  matterPref.putBool(kOnOffKey3, state);
  driveRelay(RELAY3_PIN, state);
  return true;
}

// ---------- Commissioning helper ----------
static void waitForCommissioningIfNeeded() {
  if (Matter.isDeviceCommissioned()) return;

  Serial.println();
  Serial.println("Matter Node is not commissioned yet.");
  Serial.println("Open your Matter controller to add this device.");
  Serial.printf("Manual pairing code: %s\r\n", Matter.getManualPairingCode().c_str());
  Serial.printf("QR code URL: %s\r\n", Matter.getOnboardingQRCodeUrl().c_str());

  uint32_t tick = 0;
  while (!Matter.isDeviceCommissioned()) {
    delay(100);
    if ((tick++ % 50) == 0) Serial.println("Waiting for commissioning...");
  }
  Serial.println("Commissioned and connected. Ready for use.");
}

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);

#if !CONFIG_ENABLE_CHIPOBLE
  Serial.printf("Connecting to %s\r\n", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\r\nWiFi connected. IP: %s\r\n", WiFi.localIP().toString().c_str());
#endif

  // ---- I²C / TCA9554 init ----
  Wire.begin();         // SDA/SCL defaults
  Wire.setClock(50000); // conservative; raise if stable (100k/400k)

  if (!TCA.begin()) {
    Serial.println("TCA9554 not found or init failed!");
  } else {
    Serial.println("TCA9554 initialized.");
  }
  // Configure all pins as outputs and default OFF
  for (int i = 0; i < 8; i++) {
 if (i == 0) {
    TCA.pinMode1(i, INPUT);   // pin 0 reserved for occupancy sensor
  } else {
    TCA.pinMode1(i, OUTPUT);
    TCA.write1(i, relayLevelFor(false));
  }
  }

  // ---- Preferences ----
  matterPref.begin(kNS, false);

  // ---- BME688 init @ 0x76 ----
  if (!bme.begin(0x76)) {
    Serial.println("Failed to initialize BME688 @0x76!");
    // If you want to continue without BME, comment out the next line:
    // while (1) delay(10);
  } else {
    Serial.println("BME688 initialized successfully!");
  }

  // Enforce "all OFF on boot" BEFORE Matter starts
  forceAllOffOnBoot();

  // ---- Matter endpoints (start explicitly OFF) ----
  OnOffLight1.begin(false);
  OnOffLight2.begin(false);
  OnOffLight3.begin(false);
  OnOffLight1.onChange(setLight1);
  OnOffLight2.onChange(setLight2);
  OnOffLight3.onChange(setLight3);

  // Temp/Humidity endpoints (start 0, updated from BME688 in loop)
  TempSensor.begin(0.0f);
  HumSensor.begin(0.0f);

  // Occupancy endpoint (boolean-based)
  Occupancy.begin();
  Occupancy.setOccupancy(OCCUPANCY);

  // ---- Start Matter ----
  Matter.begin();

  // ---- Commissioning ----
  waitForCommissioningIfNeeded();

  // Make sure controllers reflect OFF immediately
  OnOffLight1.setOnOff(false);
  OnOffLight2.setOnOff(false);
  OnOffLight3.setOnOff(false);
  OnOffLight1.updateAccessory();
  OnOffLight2.updateAccessory();
  OnOffLight3.updateAccessory();

  // Ensure hardware matches (already OFF, but keep for symmetry)
  applyAllRelaysFromState();

  // ---- BL0942 serial init ----
  Serial1.begin(4800, SERIAL_8N1, BL0942_RX, BL0942_TX);
  blSensor.setup();                       // default ModeConfig
  blSensor.onDataReceived(bl0942DataReceived);
  Serial.println("BL0942 initialized. Waiting for data...");
}

void loop() {
  // Re-block if decommissioned during runtime
  if (!Matter.isDeviceCommissioned()) {
    waitForCommissioningIfNeeded();
    OnOffLight1.setOnOff(false);
    OnOffLight2.setOnOff(false);
    OnOffLight3.setOnOff(false);
    OnOffLight1.updateAccessory();
    OnOffLight2.updateAccessory();
    OnOffLight3.updateAccessory();
    Occupancy.setOccupancy(OCCUPANCY);
    applyAllRelaysFromState();

  }

  const uint32_t now = millis();

  // ---- BME688 update every 5s ----
  static uint32_t lastBmeMs = 0;
  if (now - lastBmeMs >= 5000) {
    lastBmeMs = now;

    float tempC = bme.readTemperature();
    float humPct = bme.readHumidity();
    float press  = bme.readPressure();
    float gas    = bme.readGas(0);   // IAQ profile 0

    // Print all for debugging
    Serial.printf("BME688  T=%.2f°C  H=%.2f%%  P=%.2f Pa  Gas=%.2f Ω\r\n",
                  tempC, humPct, press, gas);

    // Update Matter endpoints
    if (!isnan(tempC))  TempSensor.setTemperature(tempC);
    if (!isnan(humPct)) HumSensor.setHumidity(humPct);
  }

  // ---- BL0942 processing ----
  blSensor.update();
  blSensor.loop();

  // Print to Serial every ~3s if new data present
  if (now - em.lastPrintMs >= 3000) {
    em.lastPrintMs = now;
    if (!isnan(em.voltage)) {
      Serial.printf("BL0942  V=%.1f V  I=%.3f A  P=%.1f W  E=%.2f Wh  f=%.1f Hz\r\n",
                    em.voltage, em.current, em.powerW, em.energy, em.freqHz);
    } else {
      Serial.println("BL0942  Waiting for data...");
    }
  }

// ---- Occupancy update (from TCA9554 pin 0) ----
static bool lastOccSent = false;
bool occNow = (TCA.read1(0) == HIGH);   // HIGH = occupied, LOW = no one

if (occNow != lastOccSent) {
  lastOccSent = occNow;
  Serial.printf("Occupancy -> %s\r\n", occNow ? "OCCUPIED" : "UN-OCCUPIED");
  Occupancy.setOccupancy(occNow);
}

  // ---- Button handling ----
  if (digitalRead(buttonPin) == LOW && !button_state) {
    button_time_stamp = now;
    button_state = true;
  }
  uint32_t held = now - button_time_stamp;

  // Short press example: toggle Light 1 (optional)
  if (button_state && held > debouceTime && digitalRead(buttonPin) == HIGH) {
    button_state = false;
    Serial.println("Button released: toggling Light 1 (example)...");
    OnOffLight1.toggle();  // updates SWITCH1 via callback, which drives relay 1
  }

  // Long press: decommission node
  if (button_state && held > decommissioningTimeout) {
    Serial.println("Decommissioning the Matter Node...");
    Matter.decommission();
    button_time_stamp = now; // prevent re-trigger
  }
}