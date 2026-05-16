#pragma once

// Generic ESP32 TCPRadio variant — works on any ESP32 family chip with Wi-Fi.
// Uses mesh::ESP32Board directly (no board-specific battery/VEXT logic);
// optional OLED via DISPLAY_CLASS build flag.

#include <Mesh.h>
#include <helpers/tcpradio/CustomTCPRadioWrapper.h>
#include <helpers/ESP32Board.h>
#include <helpers/AutoDiscoverRTCClock.h>
#include <helpers/SensorManager.h>
#include <helpers/sensors/EnvironmentSensorManager.h>
#ifdef DISPLAY_CLASS
  #include <helpers/ui/SSD1306Display.h>
  #include <helpers/ui/MomentaryButton.h>
#endif

#ifndef WRAPPER_CLASS
  #define WRAPPER_CLASS CustomTCPRadioWrapper
#endif

extern ESP32Board board;
extern WRAPPER_CLASS radio_driver;
extern AutoDiscoverRTCClock rtc_clock;
extern EnvironmentSensorManager sensors;

#ifdef DISPLAY_CLASS
  extern DISPLAY_CLASS display;
  extern MomentaryButton user_btn;
#endif

bool radio_init();
mesh::LocalIdentity radio_new_identity();
