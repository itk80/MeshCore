#include <Arduino.h>
#include <WiFi.h>
#include <esp_random.h>
#include "target.h"

#ifndef MODEM_DEFAULT_HOST
  #define MODEM_DEFAULT_HOST "192.168.5.11"
#endif
#ifndef MODEM_DEFAULT_PORT
  #define MODEM_DEFAULT_PORT 5055
#endif
#ifndef WIFI_SSID
  #error "WIFI_SSID must be defined as a build flag (or via platformio.local.ini)"
#endif
#ifndef WIFI_PWD
  #error "WIFI_PWD must be defined as a build flag (or via platformio.local.ini)"
#endif
#ifndef WIFI_CONNECT_TIMEOUT_MS
  #define WIFI_CONNECT_TIMEOUT_MS 30000
#endif

HeltecV3Board board;

WRAPPER_CLASS radio_driver(MODEM_DEFAULT_HOST, MODEM_DEFAULT_PORT);

ESP32RTCClock fallback_clock;
AutoDiscoverRTCClock rtc_clock(fallback_clock);
EnvironmentSensorManager sensors;

#ifdef DISPLAY_CLASS
  DISPLAY_CLASS display;
  MomentaryButton user_btn(PIN_USER_BTN, 1000, true);
#endif

// esp_random() pulls from the ESP32 hardware RNG (TRNG when Wi-Fi/BT are up,
// PRNG otherwise — both adequate for seeding an Ed25519 identity).
class EspRng : public mesh::RNG {
public:
  void random(uint8_t* dest, size_t sz) override {
    size_t i = 0;
    while (i < sz) {
      uint32_t r = esp_random();
      size_t n = (sz - i < 4) ? sz - i : 4;
      memcpy(dest + i, &r, n);
      i += n;
    }
  }
};

bool radio_init() {
  fallback_clock.begin();
  rtc_clock.begin(Wire);

  Serial.printf("[TCPRadio init] connecting Wi-Fi to '%s'...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PWD);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("[TCPRadio init] Wi-Fi FAILED (status=%d)\n", WiFi.status());
    return false;
  }
  Serial.printf("[TCPRadio init] Wi-Fi up, IP %s, RSSI %ddBm\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());

  // LoRa params we'll push to the modem in CMD_SET_CONFIG during begin().
  // Defaults match `arduino_base` build flags (MeshCore EU narrow). Override
  // with -D LORA_FREQ / -D LORA_BW / -D LORA_SF / -D LORA_CR.
  uint8_t cr = 5;       // 4/5 — stock MeshCore default
#ifdef LORA_CR
  cr = LORA_CR;
#endif
  radio_driver.setLoRaParams(
    (float)LORA_FREQ,   // MHz
    (float)LORA_BW,     // kHz
    (uint8_t)LORA_SF,
    cr,
    22,                 // tx power dBm — Heltec V3 stock max
    0x0012,             // private sync word
    16                  // preamble symbols
  );
  return true;
}

mesh::LocalIdentity radio_new_identity() {
  EspRng rng;
  return mesh::LocalIdentity(&rng);
}
