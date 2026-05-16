#pragma once

// Heltec V3 + TCPRadio variant. The on-board SX1262 is ignored — the MCU only
// drives Wi-Fi and tunnels Radio operations to a remote pymc_usb modem.

#include <Mesh.h>
#include <helpers/tcpradio/CustomTCPRadioWrapper.h>
#include <HeltecV3Board.h>
#include <helpers/AutoDiscoverRTCClock.h>
#include <helpers/SensorManager.h>
#include <helpers/sensors/EnvironmentSensorManager.h>
#ifdef DISPLAY_CLASS
  #include <helpers/ui/SSD1306Display.h>
  #include <helpers/ui/MomentaryButton.h>
#endif

// MeshCore main.cpp / variants reference these via macros — keep the names
// stable even though there's no RadioLib RADIO_CLASS underneath.
#ifndef WRAPPER_CLASS
  #define WRAPPER_CLASS CustomTCPRadioWrapper
#endif

extern HeltecV3Board board;
extern WRAPPER_CLASS radio_driver;
extern AutoDiscoverRTCClock rtc_clock;
extern EnvironmentSensorManager sensors;

#ifdef DISPLAY_CLASS
  extern DISPLAY_CLASS display;
  extern MomentaryButton user_btn;
#endif

bool radio_init();
mesh::LocalIdentity radio_new_identity();
