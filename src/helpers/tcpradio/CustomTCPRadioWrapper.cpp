// CustomTCPRadioWrapper — steps 1.4 (framing+heartbeat) + 1.5 (config+RX) + 1.6 (TX, Variant A).
// Helpers (getEstAirtimeFor / packetScore) come in step 1.7.

#include "CustomTCPRadioWrapper.h"

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  #include <Arduino.h>      // millis(), delay(), Serial
  #include <esp_random.h>   // esp_random()
  #include <lwip/sockets.h> // setsockopt(SO_LINGER) for RST-on-close
#else
  static unsigned long millis() { return 0; }
  static void delay(unsigned long) { }
#endif

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
// Force a TCP RST on close (instead of graceful FIN) so the modem — which
// only accepts a single client — releases our slot immediately. Without
// this, a stale half-closed session can linger 30–60 s on the modem side,
// causing the next attempt to TCP-connect but get no SET_CONFIG ack.
static void closeWithReset(WiFiClient& c) {
  int fd = c.fd();
  if (fd >= 0) {
    struct linger so_linger = {1, 0};   // l_onoff=1, l_linger=0 → RST on close
    setsockopt(fd, SOL_SOCKET, SO_LINGER, &so_linger, sizeof(so_linger));
  }
  c.stop();
}
#endif

using namespace pymc_proto;

CustomTCPRadioWrapper::CustomTCPRadioWrapper(const char* host, uint16_t port)
  : _port(port),
    _token_len(0),
    _authenticated(false),
    _rx_started(false),
    _config_acked(false),
    _auto_cad_acked(false),
    _last_reconnect_ms(0),
    _reconnect_count(0),
    _reconnect_backoff_step(0),
    _last_ping_sent_ms(0),
    _last_pong_recv_ms(0),
    _heartbeat_interval_ms(5000),
    _watchdog_timeout_ms(30000),
    _rx_head(0),
    _rx_tail(0),
    _tx_state(TX_IDLE),
    _tx_started_ms(0),
    _last_tx_airtime_us(0),
    _last_rssi(0.0f),
    _last_snr(0.0f),
    _noise_floor_dbm(0),
    _freq_mhz(0.0f),
    _bw_khz(0.0f),
    _sf(0),
    _cr(0),
    _power_dbm(0),
    _syncword(0),
    _preamble_len(0),
    _n_rx(0),
    _n_tx(0),
    _n_crc_err(0),
    _n_pong(0)
{
  if (host) {
    strncpy(_host, host, sizeof(_host) - 1);
    _host[sizeof(_host) - 1] = '\0';
  } else {
    _host[0] = '\0';
  }
  memset(_token, 0, sizeof(_token));
  memset(_rx_ring, 0, sizeof(_rx_ring));
}

// ─── Pre-begin configuration ───────────────────────────────────────────

void CustomTCPRadioWrapper::setAuthToken(const uint8_t* token, size_t len) {
  if (len > sizeof(_token)) len = sizeof(_token);
  if (token && len > 0) {
    memcpy(_token, token, len);
    _token_len = (uint8_t)len;
  } else {
    _token_len = 0;
  }
}

void CustomTCPRadioWrapper::setLoRaParams(float freq_mhz, float bw_khz, uint8_t sf, uint8_t cr,
                                          int8_t power_dbm, uint16_t syncword, uint8_t preamble_len) {
  _freq_mhz     = freq_mhz;
  _bw_khz       = bw_khz;
  _sf           = sf;
  _cr           = cr;
  _power_dbm    = power_dbm;
  _syncword     = syncword;
  _preamble_len = preamble_len;
}

// ─── mesh::Radio interface ────────────────────────────────────────────

void CustomTCPRadioWrapper::begin() {
  connectAndHandshake();
  _last_pong_recv_ms = millis();   // grace period before watchdog
}

int CustomTCPRadioWrapper::recvRaw(uint8_t* bytes, int sz) {
  int out_len = 0;
  if (ringPop(bytes, sz, out_len)) return out_len;
  return 0;
}

uint32_t CustomTCPRadioWrapper::getEstAirtimeFor(int len_bytes) {
  return estimate_airtime_ms(len_bytes, _sf, (uint32_t)(_bw_khz * 1000.0f),
                             _cr, _preamble_len);
}

float CustomTCPRadioWrapper::packetScore(float snr, int packet_len) {
  return packet_score(snr, _sf, packet_len);
}

bool CustomTCPRadioWrapper::startSendRaw(const uint8_t* bytes, int len) {
  if (len <= 0 || len > (int)MAX_LORA_PAYLOAD) return false;
  if (_tx_state == TX_PENDING) return false;   // previous TX not finished
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  if (!_client.connected() || !_rx_started) return false;
#endif
  if (!sendFrame(CMD_TX_REQUEST, bytes, (uint16_t)len)) return false;
  _tx_state      = TX_PENDING;
  _tx_started_ms = millis();
  return true;
}

bool CustomTCPRadioWrapper::isSendComplete() {
  return _tx_state == TX_DONE || _tx_state == TX_FAIL;
}

void CustomTCPRadioWrapper::onSendFinished() {
  _tx_state = TX_IDLE;
}

void CustomTCPRadioWrapper::loop() {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  uint32_t now = millis();

  pumpRx();

  if (_client.connected()) {
    // Signed-diff compare so that timestamps slightly *ahead* of `now`
    // (e.g. when pumpRx() above updated _last_pong_recv_ms after we cached
    // `now`) read as "recent", not as UINT32_MAX wrap-around.
    int32_t since_ping = (int32_t)(now - _last_ping_sent_ms);
    if (since_ping >= (int32_t)_heartbeat_interval_ms) {
      sendFrame(CMD_PING, nullptr, 0);
      _last_ping_sent_ms = now;
    }
    int32_t since_pong = (int32_t)(now - _last_pong_recv_ms);
    if (since_pong > (int32_t)_watchdog_timeout_ms) {
      Serial.printf("[TCPRadio] watchdog tripped (no frame for %ldms), reconnecting\n",
                    (long)since_pong);
      closeWithReset(_client);
      onDisconnect();
    }
  } else {
    // Exponential backoff: 5s, 10s, 20s, 40s, 60s (cap). Reset to 5s on
    // any successful handshake; bump one step on every failed attempt.
    uint32_t step = _reconnect_backoff_step;
    if (step > 4) step = 4;
    uint32_t interval = (uint32_t)5000 << step;
    if (interval > 60000) interval = 60000;
    int32_t since_reconnect = (int32_t)(now - _last_reconnect_ms);
    if (since_reconnect >= (int32_t)interval) {
      connectAndHandshake();
    }
  }
#endif
}

int  CustomTCPRadioWrapper::getNoiseFloor() const { return _noise_floor_dbm; }

void CustomTCPRadioWrapper::triggerNoiseFloorCalibrate(int /*threshold*/) {
  // Fire-and-forget; response handled in handleNoiseResp.
  sendFrame(CMD_NOISE_REQ, nullptr, 0);
}

void CustomTCPRadioWrapper::resetAGC() { /* nop — modem owns AGC */ }

bool CustomTCPRadioWrapper::isInRecvMode() const {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  return _authenticated && _rx_started &&
         const_cast<WiFiClient&>(_client).connected();
#else
  return _authenticated && _rx_started;
#endif
}

bool CustomTCPRadioWrapper::isReceiving() { return false; }  // Variant A: modem auto-CAD

float CustomTCPRadioWrapper::getLastRSSI() const { return _last_rssi; }
float CustomTCPRadioWrapper::getLastSNR()  const { return _last_snr;  }

// ─── Diagnostics ──────────────────────────────────────────────────────

bool CustomTCPRadioWrapper::isConnected() const {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  return const_cast<WiFiClient&>(_client).connected();
#else
  return false;
#endif
}

void CustomTCPRadioWrapper::forceReconnect() {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  closeWithReset(_client);
#endif
  onDisconnect();
}

uint32_t CustomTCPRadioWrapper::getMsSinceLastPong() const {
  return (uint32_t)(millis() - _last_pong_recv_ms);
}

uint32_t CustomTCPRadioWrapper::getRngSeed() {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  // ESP32 hardware RNG (TRNG when Wi-Fi is up, PRNG otherwise — both fine
  // as a seed for the host-side StdRNG).
  return esp_random();
#else
  return 0;
#endif
}

void CustomTCPRadioWrapper::setParams(float freq_mhz, float bw_khz, uint8_t sf, uint8_t cr) {
  _freq_mhz = freq_mhz;
  _bw_khz   = bw_khz;
  _sf       = sf;
  _cr       = cr;
  // Push the new config to the modem so it re-tunes the SX1262.
  // EVT_CONFIG_RESP will arrive asynchronously; runtime CLI doesn't await it.
  if (isConnected()) sendConfig();
}

void CustomTCPRadioWrapper::setTxPower(int8_t dbm) {
  _power_dbm = dbm;
  if (isConnected()) sendConfig();
}

void CustomTCPRadioWrapper::setEndpoint(const char* host, uint16_t port) {
  if (host == nullptr || host[0] == '\0' || port == 0) return;  // ignore empty
  if (strncmp(_host, host, sizeof(_host)) == 0 && _port == port) return;  // no change
  strncpy(_host, host, sizeof(_host) - 1);
  _host[sizeof(_host) - 1] = '\0';
  _port = port;
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  Serial.printf("[TCPRadio] endpoint changed to %s:%u — forcing reconnect\n",
                _host, (unsigned)_port);
#endif
  forceReconnect();
}

bool CustomTCPRadioWrapper::sendPing() {
  return sendFrame(CMD_PING, nullptr, 0);
}

// ─── Frame I/O ────────────────────────────────────────────────────────

bool CustomTCPRadioWrapper::sendFrame(uint8_t cmd, const uint8_t* payload, uint16_t len) {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  if (!_client.connected()) return false;
  static uint8_t out[MAX_FRAME_SIZE];
  size_t n = build_frame(cmd, payload, len, out);
  if (n == 0) return false;
  size_t written = _client.write(out, n);
  return written == n;
#else
  (void)cmd; (void)payload; (void)len;
  return false;
#endif
}

void CustomTCPRadioWrapper::pumpRx() {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  while (_client.connected() && _client.available()) {
    int v = _client.read();
    if (v < 0) break;
    uint8_t b = (uint8_t)v;
    if (_parser.feed(b)) {
      _last_pong_recv_ms = millis();
      switch (_parser.cmd) {
        case EVT_PONG:                handlePong();                                                  break;
        case EVT_RX_PACKET:           handleRxPacket    (_parser.payload, _parser.payload_len);      break;
        case EVT_TX_DONE:             handleTxDone      (_parser.payload, _parser.payload_len);      break;
        case EVT_TX_FAIL:             handleTxFail      (_parser.payload, _parser.payload_len);      break;
        case EVT_NOISE_RESP:          handleNoiseResp   (_parser.payload, _parser.payload_len);      break;
        case EVT_LOG_MSG:             handleLogMsg      (_parser.payload, _parser.payload_len);      break;
        case EVT_ERROR:               handleErrorEvt    (_parser.payload, _parser.payload_len);      break;
        case EVT_CONFIG_RESP:         handleConfigResp  (_parser.payload, _parser.payload_len);      break;
        case EVT_SET_AUTO_CAD_RESP:   handleAutoCadResp (_parser.payload, _parser.payload_len);      break;
        case EVT_RX_STARTED:          handleRxStarted   (_parser.payload, _parser.payload_len);      break;
        case EVT_AUTH_OK:             handleAuthOk      (_parser.payload, _parser.payload_len);      break;
        default: break;  // ignore unknowns (forward-compat with v0.7+)
      }
      _parser.reset();
    } else if (_parser.crc_failed) {
      _n_crc_err++;
    }
  }
#endif
}

// ─── Per-event handlers ───────────────────────────────────────────────

void CustomTCPRadioWrapper::handleRxPacket(const uint8_t* payload, uint16_t len) {
  RxPacketMeta meta;
  const uint8_t* data = nullptr;
  int data_len = parse_rx_packet(payload, len, meta, data);
  if (data_len < 0) return;
  if (data_len > (int)MAX_LORA_PAYLOAD) data_len = MAX_LORA_PAYLOAD;
  ringPush(meta, data, (uint8_t)data_len);
  _n_rx++;
}

void CustomTCPRadioWrapper::handleTxDone(const uint8_t* payload, uint16_t len) {
  parse_tx_done(payload, len, _last_tx_airtime_us);
  _tx_state = TX_DONE;
  _n_tx++;
}

void CustomTCPRadioWrapper::handleTxFail(const uint8_t* payload, uint16_t len) {
  (void)payload; (void)len;
  _tx_state = TX_FAIL;
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  Serial.println("[TCPRadio] EVT_TX_FAIL");
#endif
}

void CustomTCPRadioWrapper::handleNoiseResp(const uint8_t* payload, uint16_t len) {
  int n = 0;
  if (parse_noise_resp(payload, len, n)) _noise_floor_dbm = n;
}

void CustomTCPRadioWrapper::handleLogMsg(const uint8_t* payload, uint16_t len) {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  if (len < 1) return;
  Serial.printf("[modem lvl=%u] ", (unsigned)payload[0]);
  Serial.write(payload + 1, len - 1);
  Serial.println();
#else
  (void)payload; (void)len;
#endif
}

void CustomTCPRadioWrapper::handleErrorEvt(const uint8_t* payload, uint16_t len) {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  if (len >= 1) Serial.printf("[TCPRadio] EVT_ERROR code=0x%02X\n", payload[0]);
#else
  (void)payload;
#endif
  // If we had a TX in flight, ANY error from the modem ends it. Without this,
  // the Dispatcher would wait for outbound_expiry (1.5× est airtime) before
  // giving up — wasting hundreds of ms per failed TX.
  if (_tx_state == TX_PENDING) {
    _tx_state = TX_FAIL;
  }
  (void)len;
}

void CustomTCPRadioWrapper::handlePong() {
  _n_pong++;
}

void CustomTCPRadioWrapper::handleConfigResp(const uint8_t* payload, uint16_t len) {
  // EVT_CONFIG_RESP echoes the applied config (14B); we just care that it arrived.
  (void)payload; (void)len;
  _config_acked = true;
}

void CustomTCPRadioWrapper::handleAutoCadResp(const uint8_t* payload, uint16_t len) {
  // 1B status (0 = OK).
  if (len >= 1 && payload[0] == 0) _auto_cad_acked = true;
}

void CustomTCPRadioWrapper::handleRxStarted(const uint8_t* payload, uint16_t len) {
  (void)payload; (void)len;
  _rx_started = true;
}

void CustomTCPRadioWrapper::handleAuthOk(const uint8_t* payload, uint16_t len) {
  (void)payload; (void)len;
  _authenticated = true;
}

// ─── Connection helpers ───────────────────────────────────────────────

bool CustomTCPRadioWrapper::tcpConnect() {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  _last_reconnect_ms = millis();
  if (_host[0] == '\0' || _port == 0) return false;
  if (_client.connected()) return true;
  closeWithReset(_client);
  _parser.reset();
  bool ok = _client.connect(_host, _port);
  if (ok) {
    _client.setNoDelay(true);
    _reconnect_count++;
    _last_pong_recv_ms = millis();
    Serial.printf("[TCPRadio] connected to %s:%u (attempt #%lu)\n",
                  _host, (unsigned)_port, (unsigned long)_reconnect_count);
  } else {
    Serial.printf("[TCPRadio] connect to %s:%u failed\n", _host, (unsigned)_port);
  }
  return ok;
#else
  return false;
#endif
}

bool CustomTCPRadioWrapper::sendAuth() {
  if (_token_len == 0) return false;
  return sendFrame(CMD_AUTH, _token, _token_len);
}

bool CustomTCPRadioWrapper::sendConfig() {
  RadioConfig cfg{};
  cfg.freq_hz      = (uint32_t)(_freq_mhz * 1e6f);
  cfg.bandwidth_hz = (uint32_t)(_bw_khz * 1000.0f);
  cfg.sf           = _sf;
  cfg.cr           = _cr;
  cfg.power_dbm    = _power_dbm;
  cfg.syncword     = _syncword;
  cfg.preamble_len = _preamble_len;

  uint8_t wire[14];
  serialize_radio_config(cfg, wire);
  return sendFrame(CMD_SET_CONFIG, wire, sizeof(wire));
}

bool CustomTCPRadioWrapper::sendAutoCad(bool enable) {
  uint8_t v = enable ? 1 : 0;
  return sendFrame(CMD_SET_AUTO_CAD, &v, 1);
}

bool CustomTCPRadioWrapper::sendRxStart() {
  return sendFrame(CMD_RX_START, nullptr, 0);
}

void CustomTCPRadioWrapper::onDisconnect() {
  _authenticated  = false;
  _rx_started     = false;
  _config_acked   = false;
  _auto_cad_acked = false;
  _tx_state       = TX_IDLE;
  _parser.reset();
}

// ─── Handshake / await helpers ────────────────────────────────────────

bool CustomTCPRadioWrapper::awaitFlag(bool& flag, uint32_t timeout_ms) {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  uint32_t deadline = millis() + timeout_ms;
  while ((int32_t)(deadline - millis()) > 0) {
    pumpRx();
    if (flag) return true;
    delay(1);  // yield to OS
  }
  return flag;
#else
  (void)flag; (void)timeout_ms;
  return false;
#endif
}

bool CustomTCPRadioWrapper::connectAndHandshake() {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  if (!tcpConnect()) {
    if (_reconnect_backoff_step < 4) _reconnect_backoff_step++;
    return false;
  }

  // 1. AUTH (only if token configured)
  if (_token_len > 0) {
    _authenticated = false;
    if (!sendAuth() || !awaitFlag(_authenticated, 2000)) {
      Serial.println("[TCPRadio] auth failed or timed out");
      closeWithReset(_client); onDisconnect();
      return false;
    }
  } else {
    _authenticated = true;  // server doesn't enforce auth without configured token
  }

  // 2. CONFIG
  _config_acked = false;
  if (!sendConfig() || !awaitFlag(_config_acked, 2000)) {
    Serial.println("[TCPRadio] SET_CONFIG ack timeout");
    closeWithReset(_client); onDisconnect();
    if (_reconnect_backoff_step < 4) _reconnect_backoff_step++;
    return false;
  }

  // 3. AUTO_CAD on (Variant A LBT). Non-critical: a v0.6 modem won't ack
  //    but the rest of the radio still works (caveat: no LBT).
  _auto_cad_acked = false;
  if (!sendAutoCad(true) || !awaitFlag(_auto_cad_acked, 1000)) {
    Serial.println("[TCPRadio] AUTO_CAD not acked — assuming v0.6 modem, continuing without LBT");
  }

  // 4. RX_START
  _rx_started = false;
  if (!sendRxStart() || !awaitFlag(_rx_started, 2000)) {
    Serial.println("[TCPRadio] RX_START ack timeout");
    closeWithReset(_client); onDisconnect();
    if (_reconnect_backoff_step < 4) _reconnect_backoff_step++;
    return false;
  }

  Serial.printf("[TCPRadio] handshake complete; auth=%d cfg=%d auto_cad=%d rx=%d\n",
                _authenticated, _config_acked, _auto_cad_acked, _rx_started);
  _reconnect_backoff_step = 0;   // reset exp backoff on success
  return true;
#else
  return false;
#endif
}

// ─── RX ring ──────────────────────────────────────────────────────────

bool CustomTCPRadioWrapper::ringPush(const pymc_proto::RxPacketMeta& meta,
                                     const uint8_t* data, uint8_t data_len) {
  uint8_t next = (uint8_t)((_rx_head + 1) % RX_RING);
  if (next == _rx_tail) {
    // Full — drop oldest to make room (lossy ring as designed).
    _rx_tail = (uint8_t)((_rx_tail + 1) % RX_RING);
  }
  RxFrame& f = _rx_ring[_rx_head];
  f.len          = data_len;
  f.rssi_dbm     = meta.rssi_dbm;
  f.snr_x10      = meta.snr_x10;
  f.sig_rssi_dbm = meta.sig_rssi_dbm;
  f.recv_ms      = (uint32_t)millis();
  if (data && data_len) memcpy(f.data, data, data_len);
  _rx_head = next;
  return true;
}

bool CustomTCPRadioWrapper::ringPop(uint8_t* dst, int max_len, int& out_len) {
  if (_rx_head == _rx_tail) { out_len = 0; return false; }
  RxFrame& f = _rx_ring[_rx_tail];
  int n = f.len;
  if (n > max_len) n = max_len;
  if (dst && n > 0) memcpy(dst, f.data, n);
  // Update last RSSI/SNR cache so Dispatcher's call site reads correct metadata.
  _last_rssi = (float)f.rssi_dbm;
  _last_snr  = (float)f.snr_x10 / 10.0f;
  _rx_tail   = (uint8_t)((_rx_tail + 1) % RX_RING);
  out_len    = n;
  return true;
}
