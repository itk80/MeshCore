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

ESP32Board board;

WRAPPER_CLASS radio_driver(MODEM_DEFAULT_HOST, MODEM_DEFAULT_PORT);

ESP32RTCClock fallback_clock;
AutoDiscoverRTCClock rtc_clock(fallback_clock);
EnvironmentSensorManager sensors;

#ifdef DISPLAY_CLASS
  DISPLAY_CLASS display;
  MomentaryButton user_btn(PIN_USER_BTN, 1000, true);
#endif

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

  uint8_t cr = 5;
#ifdef LORA_CR
  cr = LORA_CR;
#endif
  radio_driver.setLoRaParams(
    (float)LORA_FREQ,
    (float)LORA_BW,
    (uint8_t)LORA_SF,
    cr,
    22,                 // tx power dBm (modem can apply less per region)
    0x0012,             // private sync word
    16                  // preamble symbols
  );
  return true;
}

mesh::LocalIdentity radio_new_identity() {
  EspRng rng;
  return mesh::LocalIdentity(&rng);
}
