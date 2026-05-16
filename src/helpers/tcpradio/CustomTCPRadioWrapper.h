#pragma once

// CustomTCPRadioWrapper — bridges mesh::Radio to a remote "dumb modem"
// (itk80/pymc_usb) speaking wire protocol v0.7 over TCP/IP.
//
// Layered design: protocol/framing lives in pymc_proto.h (Arduino-free,
// unit-testable via [env:native]); this header is the Arduino-side glue that
// implements mesh::Radio and owns the WiFiClient. The .cpp ties them together.
//
// This file is OPT-IN: only compiled when a variant adds
// +<helpers/tcpradio/*.cpp> to build_src_filter. Stock builds untouched.

#include <Mesh.h>
#include "pymc_proto.h"

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  #include <WiFi.h>
  #include <WiFiClient.h>
#endif

/**
 * \brief  Tunnels mesh::Radio operations to a remote pymc_usb modem over TCP.
 *
 * Lifecycle:
 *   1. Constructor stores host/port; nothing happens on the network.
 *   2. Caller may set auth token / LoRa params before begin().
 *   3. begin() connects, authenticates (if token set), configures, enables
 *      auto-CAD (Variant A — pymc_usb ≥v0.7), starts RX. After this
 *      isInRecvMode() is true once the modem ACKs EVT_RX_STARTED.
 *   4. Dispatcher::loop() repeatedly calls loop() (frame pump + heartbeat),
 *      recvRaw() (drains the RX ring), startSendRaw()/isSendComplete()
 *      (TX state machine).
 */
class CustomTCPRadioWrapper : public mesh::Radio {
public:
  CustomTCPRadioWrapper(const char* host, uint16_t port);

  // ---- Pre-begin configuration ----

  /** \brief Optional auth token sent in CMD_AUTH right after TCP connect. */
  void setAuthToken(const uint8_t* token, size_t len);

  /** \brief LoRa parameters that will be sent in CMD_SET_CONFIG. */
  void setLoRaParams(float freq_mhz, float bw_khz, uint8_t sf, uint8_t cr,
                     int8_t power_dbm, uint16_t syncword, uint8_t preamble_len);

  // ---- mesh::Radio interface ----
  void     begin() override;
  int      recvRaw(uint8_t* bytes, int sz) override;
  uint32_t getEstAirtimeFor(int len_bytes) override;
  float    packetScore(float snr, int packet_len) override;
  bool     startSendRaw(const uint8_t* bytes, int len) override;
  bool     isSendComplete() override;
  void     onSendFinished() override;
  void     loop() override;
  int      getNoiseFloor() const override;
  void     triggerNoiseFloorCalibrate(int threshold) override;
  void     resetAGC() override;
  bool     isInRecvMode() const override;
  bool     isReceiving() override;        // Variant A: always false (modem does auto-CAD)
  float    getLastRSSI() const override;
  float    getLastSNR()  const override;

  // ---- Diagnostics (not part of the Radio interface) ----
  bool     isConnected() const;
  uint32_t getReconnectCount() const   { return _reconnect_count; }
  void     forceReconnect();
  uint32_t getRxCount() const          { return _n_rx; }
  uint32_t getTxCount() const          { return _n_tx; }
  uint32_t getCrcErrors() const        { return _n_crc_err; }
  uint32_t getPongCount() const        { return _n_pong; }
  uint32_t getMsSinceLastPong() const;
  const char* getModemHost() const     { return _host; }
  uint16_t getModemPort() const        { return _port; }
  bool     isAuthenticated() const     { return _authenticated; }
  bool     isHandshakeComplete() const { return _authenticated && _rx_started; }
  // Sends a one-off PING; used by CLI `modem.ping`. Returns true if write queued.
  bool     sendPing();

  // Runtime endpoint reconfig (CLI). If host/port differ from current,
  // tears down the connection so the next loop() iteration reconnects to the
  // new endpoint with a fresh handshake.
  void     setEndpoint(const char* host, uint16_t port);

  // ---- RadioLibWrapper-compatible API ----
  // MeshCore's variant-agnostic main.cpp, MyMesh.cpp and StatsFormatHelper.h
  // call these names. We provide aliases (and forwarding behaviour for
  // setParams/setTxPower so runtime CLI reconfig still pushes a fresh
  // SET_CONFIG to the modem). Boosted-gain mode is a chip register on
  // SX1262; the modem owns it, so we no-op.
  uint32_t getRngSeed();
  uint32_t getPacketsRecv() const        { return _n_rx; }
  uint32_t getPacketsSent() const        { return _n_tx; }
  uint32_t getPacketsRecvErrors() const  { return _n_crc_err; }
  void resetStats() { _n_rx = _n_tx = _n_crc_err = _n_pong = 0; }
  void setParams(float freq_mhz, float bw_khz, uint8_t sf, uint8_t cr);
  void setTxPower(int8_t dbm);
  void setRxBoostedGainMode(bool /*en*/) { /* modem owns RX frontend */ }
  bool getRxBoostedGainMode() const { return false; }

  // Configurable knobs (default heartbeat = 5 s, watchdog = 30 s).
  void setHeartbeatInterval(uint32_t ms) { _heartbeat_interval_ms = ms; }
  void setWatchdogTimeout(uint32_t ms)   { _watchdog_timeout_ms = ms; }

private:
  // ---- TCP transport ----
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  WiFiClient   _client;
#endif
  char         _host[64];
  uint16_t     _port;
  uint8_t      _token[32];
  uint8_t      _token_len;
  bool         _authenticated;
  bool         _rx_started;
  bool         _config_acked;       // EVT_CONFIG_RESP seen
  bool         _auto_cad_acked;     // EVT_SET_AUTO_CAD_RESP seen, status OK
  uint32_t     _last_reconnect_ms;
  uint32_t     _reconnect_count;
  uint8_t      _reconnect_backoff_step;  // 0..N, exp backoff: 5s,10s,20s,40s,60s (cap)

  // ---- Frame parser (one byte at a time from loop()) ----
  pymc_proto::FrameParser _parser;

  // ---- Heartbeat / watchdog (step 1.4) ----
  uint32_t _last_ping_sent_ms;
  uint32_t _last_pong_recv_ms;
  uint32_t _heartbeat_interval_ms;
  uint32_t _watchdog_timeout_ms;

  // ---- RX ring buffer (filled in loop() from EVT_RX_PACKET, drained by recvRaw()) ----
  struct RxFrame {
    uint8_t  data[pymc_proto::MAX_LORA_PAYLOAD];
    uint8_t  len;
    int16_t  rssi_dbm;     // per modem firmware: dBm directly
    int16_t  snr_x10;      // dB × 10
    int16_t  sig_rssi_dbm;
    uint32_t recv_ms;
  };
  static constexpr size_t RX_RING = 8;
  RxFrame  _rx_ring[RX_RING];
  volatile uint8_t _rx_head, _rx_tail;   // empty when equal; lossy when full

  // ---- TX state machine ----
  enum TxState : uint8_t { TX_IDLE, TX_PENDING, TX_DONE, TX_FAIL };
  volatile TxState _tx_state;
  uint32_t _tx_started_ms;
  uint32_t _last_tx_airtime_us;          // from EVT_TX_DONE payload

  // ---- Cached metadata ----
  float _last_rssi;
  float _last_snr;
  int   _noise_floor_dbm;

  // ---- LoRa params snapshot (for getEstAirtimeFor / packetScore) ----
  float    _freq_mhz;
  float    _bw_khz;
  uint8_t  _sf;
  uint8_t  _cr;
  int8_t   _power_dbm;
  uint16_t _syncword;
  uint8_t  _preamble_len;

  // ---- Frame I/O ----
  /** Build a frame and write it to the socket in a single send. */
  bool sendFrame(uint8_t cmd, const uint8_t* payload, uint16_t len);

  /** Drain bytes available on the socket through _parser; dispatch full frames. */
  void pumpRx();

  // ---- Per-event handlers ----
  void handleRxPacket(const uint8_t* payload, uint16_t len);
  void handleTxDone(const uint8_t* payload, uint16_t len);
  void handleTxFail(const uint8_t* payload, uint16_t len);
  void handleNoiseResp(const uint8_t* payload, uint16_t len);
  void handleLogMsg(const uint8_t* payload, uint16_t len);
  void handleErrorEvt(const uint8_t* payload, uint16_t len);
  void handlePong();
  void handleConfigResp(const uint8_t* payload, uint16_t len);
  void handleAutoCadResp(const uint8_t* payload, uint16_t len);
  void handleRxStarted(const uint8_t* payload, uint16_t len);
  void handleAuthOk(const uint8_t* payload, uint16_t len);

  // ---- Connection-management helpers ----
  bool tcpConnect();
  bool sendAuth();
  bool sendConfig();
  bool sendAutoCad(bool enable);
  bool sendRxStart();
  void onDisconnect();

  // ---- begin() handshake helper: pump rx until `flag` flips, or timeout. ----
  bool awaitFlag(bool& flag, uint32_t timeout_ms);
  bool connectAndHandshake();

  // ---- RX ring helpers ----
  bool ringPush(const pymc_proto::RxPacketMeta& meta, const uint8_t* data, uint8_t data_len);
  bool ringPop(uint8_t* dst, int max_len, int& out_len);  // out_len = bytes copied
  bool ringEmpty() const { return _rx_head == _rx_tail; }

  // ---- Stats ----
  uint32_t _n_rx, _n_tx, _n_crc_err, _n_pong;
};
