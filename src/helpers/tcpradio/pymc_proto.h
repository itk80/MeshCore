#pragma once

// pymc_proto.h — pymc_usb wire protocol v0.7 constants, packed structures,
// CRC, frame builder, and a byte-by-byte frame parser state machine.
//
// Intentionally Arduino-free and MeshCore-free so the protocol layer can be
// unit-tested under the [env:native] gtest target. The Arduino-side wrapper
// (CustomTCPRadioWrapper.h) is the only consumer that pulls in WiFi etc.
//
// Numbers verified against pymc_usb/firmware/include/protocol.h v0.7 AND
// against a live modem (lilygo-t3s3 v0.7.0 at 192.168.5.11:5055).

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

namespace pymc_proto {

// ─── Frame sync ────────────────────────────────────────────────────────
static constexpr uint8_t SYNC = 0xAA;

// ─── Host → Modem commands ─────────────────────────────────────────────
static constexpr uint8_t CMD_TX_REQUEST        = 0x01;
static constexpr uint8_t CMD_SET_CONFIG        = 0x10;
static constexpr uint8_t CMD_GET_CONFIG        = 0x11;
static constexpr uint8_t CMD_STATUS_REQ        = 0x20;
static constexpr uint8_t CMD_NOISE_REQ         = 0x22;
static constexpr uint8_t CMD_CAD_REQUEST       = 0x30;
static constexpr uint8_t CMD_RX_START          = 0x31;
static constexpr uint8_t CMD_SET_CAD_PARAMS    = 0x34;
static constexpr uint8_t CMD_RADIO_STANDBY     = 0x40;
static constexpr uint8_t CMD_SET_WIFI          = 0x41;
static constexpr uint8_t CMD_RADIO_RESUME      = 0x42;
static constexpr uint8_t CMD_SET_DISPLAY_NAME  = 0x48;
static constexpr uint8_t CMD_SET_AUTO_CAD      = 0x4A;
static constexpr uint8_t CMD_AUTH              = 0x50;
static constexpr uint8_t CMD_WIFI_RESET        = 0x60;
static constexpr uint8_t CMD_GET_WIFI          = 0x61;
static constexpr uint8_t CMD_GET_VERSION       = 0x70;
static constexpr uint8_t CMD_GET_DEBUG         = 0x72;
static constexpr uint8_t CMD_ENTER_BOOTLOADER  = 0x74;
static constexpr uint8_t CMD_PING              = 0xFF;

// ─── Modem → Host events (numbered CMD_* in protocol.h) ────────────────
static constexpr uint8_t EVT_TX_DONE              = 0x02;
static constexpr uint8_t EVT_TX_FAIL              = 0x03;
static constexpr uint8_t EVT_RX_PACKET            = 0x04;
static constexpr uint8_t EVT_CONFIG_RESP          = 0x12;
static constexpr uint8_t EVT_STATUS_RESP          = 0x21;
static constexpr uint8_t EVT_NOISE_RESP           = 0x23;
static constexpr uint8_t EVT_CAD_RESP             = 0x32;
static constexpr uint8_t EVT_RX_STARTED           = 0x33;
static constexpr uint8_t EVT_CAD_PARAMS_RESP      = 0x35;
static constexpr uint8_t EVT_RADIO_STANDBY_RESP   = 0x44;
static constexpr uint8_t EVT_RADIO_RESUME_RESP    = 0x46;
static constexpr uint8_t EVT_SET_DISPLAY_NAME_RESP = 0x49;
static constexpr uint8_t EVT_SET_AUTO_CAD_RESP    = 0x4B;
static constexpr uint8_t EVT_AUTH_OK              = 0x51;
static constexpr uint8_t EVT_WIFI_STATUS          = 0x62;
static constexpr uint8_t EVT_VERSION_RESP         = 0x71;
static constexpr uint8_t EVT_DEBUG_RESP           = 0x73;
static constexpr uint8_t EVT_LOG_MSG              = 0x80;
static constexpr uint8_t EVT_ERROR                = 0xFE;
static constexpr uint8_t EVT_PONG                 = 0xFF;

// ─── Error codes (carried in EVT_ERROR payload byte 0) ─────────────────
static constexpr uint8_t ERR_CRC_MISMATCH    = 0x01;
static constexpr uint8_t ERR_INVALID_CMD     = 0x02;
static constexpr uint8_t ERR_RADIO_BUSY      = 0x03;
static constexpr uint8_t ERR_TX_TIMEOUT      = 0x04;
static constexpr uint8_t ERR_PAYLOAD_TOO_BIG = 0x05;
static constexpr uint8_t ERR_INVALID_CONFIG  = 0x06;
static constexpr uint8_t ERR_CAD_FAILED      = 0x07;
static constexpr uint8_t ERR_RADIO_INIT      = 0x08;
static constexpr uint8_t ERR_UNAUTHORIZED    = 0x09;
static constexpr uint8_t ERR_INVALID_WIFI    = 0x0A;
static constexpr uint8_t ERR_NO_RADIO        = 0x0B;
static constexpr uint8_t ERR_OTA_UNSUPPORTED = 0x0C;
static constexpr uint8_t ERR_OTA_NO_BUFFER   = 0x0D;
static constexpr uint8_t ERR_CHANNEL_BUSY    = 0x0E;

// ─── Sizes ─────────────────────────────────────────────────────────────
static constexpr uint16_t MAX_LORA_PAYLOAD = 255;
// Worst-case modem→host frame carries EVT_RX_PACKET = 6B metadata + 255B data.
static constexpr uint16_t MAX_PAYLOAD      = 6 + MAX_LORA_PAYLOAD;
static constexpr uint16_t FRAME_OVERHEAD   = 1 /*SYNC*/ + 1 /*CMD*/ + 2 /*LEN*/ + 2 /*CRC*/;
static constexpr uint16_t MAX_FRAME_SIZE   = FRAME_OVERHEAD + MAX_PAYLOAD;  // 267 + 4 = 271 incl. SYNC

// 14-byte packed config (matches RadioConfig in protocol.h v0.7).
// Note: PLAN.md / CLAUDE.md describe this with `syncword`/`preamble_len`
// widths swapped and a non-existent `flags` byte. Wire reality wins.
struct __attribute__((packed)) RadioConfig {
  uint32_t freq_hz;
  uint32_t bandwidth_hz;
  uint8_t  sf;
  uint8_t  cr;
  int8_t   power_dbm;
  uint16_t syncword;
  uint8_t  preamble_len;
};
static_assert(sizeof(RadioConfig) == 14, "RadioConfig must be 14 bytes on the wire");

// ─── CRC-16/CCITT (poly 0x1021, init 0xFFFF) ───────────────────────────
// Bit-by-bit, identical implementation to crc16_ccitt() in protocol.h so
// results are guaranteed bit-identical to the modem firmware. Performance
// is fine for our throughput (≤ a few thousand bytes/sec).
static inline uint16_t crc16_ccitt(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
      else              crc = (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// ─── Frame builder ─────────────────────────────────────────────────────
/**
 * Serialise SYNC | CMD | LEN(2LE) | PAYLOAD | CRC(2LE) into `out`.
 * `out` must be at least FRAME_OVERHEAD + payload_len bytes (≤ MAX_FRAME_SIZE).
 * Returns total bytes written, or 0 if payload_len > MAX_PAYLOAD.
 * payload may be nullptr when payload_len == 0.
 */
static inline size_t build_frame(uint8_t cmd, const uint8_t* payload, uint16_t payload_len, uint8_t* out) {
  if (payload_len > MAX_PAYLOAD) return 0;
  size_t i = 0;
  out[i++] = SYNC;
  out[i++] = cmd;
  out[i++] = (uint8_t)(payload_len & 0xFF);
  out[i++] = (uint8_t)((payload_len >> 8) & 0xFF);
  if (payload_len) {
    memcpy(&out[i], payload, payload_len);
    i += payload_len;
  }
  // CRC covers CMD + LEN(2) + PAYLOAD (i.e. everything after SYNC, before CRC bytes).
  uint16_t crc = crc16_ccitt(&out[1], (uint16_t)(1 + 2 + payload_len));
  out[i++] = (uint8_t)(crc & 0xFF);
  out[i++] = (uint8_t)((crc >> 8) & 0xFF);
  return i;
}

// ─── Payload parsers ──────────────────────────────────────────────────

/**
 * EVT_RX_PACKET metadata (the first 6 bytes of the payload).
 * Units determined empirically from live modem (lilygo-t3s3 v0.7.0 at .5.11)
 * cross-referenced with StatusResp where the same fields appear:
 *   - rssi_dbm     : dBm directly (e.g. -51 means -51 dBm)
 *   - snr_x10      : dB × 10 (e.g. 120 means +12.0 dB)
 *   - sig_rssi_dbm : dBm directly (signal-only RSSI, distinct from noise floor)
 */
struct RxPacketMeta {
  int16_t rssi_dbm;
  int16_t snr_x10;
  int16_t sig_rssi_dbm;
};

/**
 * Parse EVT_RX_PACKET payload: 6B metadata + N B LoRa data.
 * Returns LoRa data length (≥ 0), or −1 if payload is too short.
 * `data_out` is set to point at the LoRa bytes inside `payload`.
 */
static inline int parse_rx_packet(const uint8_t* payload, uint16_t len,
                                  RxPacketMeta& meta, const uint8_t*& data_out) {
  if (len < 6) return -1;
  meta.rssi_dbm     = (int16_t)((uint16_t)payload[0] | ((uint16_t)payload[1] << 8));
  meta.snr_x10      = (int16_t)((uint16_t)payload[2] | ((uint16_t)payload[3] << 8));
  meta.sig_rssi_dbm = (int16_t)((uint16_t)payload[4] | ((uint16_t)payload[5] << 8));
  data_out = payload + 6;
  return (int)len - 6;
}

/**
 * Parse EVT_TX_DONE payload: 4B airtime_us LE.
 * Returns true if payload is well-formed.
 */
static inline bool parse_tx_done(const uint8_t* payload, uint16_t len, uint32_t& airtime_us) {
  if (len < 4) return false;
  airtime_us = (uint32_t)payload[0]
             | ((uint32_t)payload[1] << 8)
             | ((uint32_t)payload[2] << 16)
             | ((uint32_t)payload[3] << 24);
  return true;
}

/**
 * Parse EVT_NOISE_RESP payload: 2B int16 LE in dBm × 10.
 * Returns true if payload is well-formed; `noise_dbm` is the rounded dBm value.
 */
static inline bool parse_noise_resp(const uint8_t* payload, uint16_t len, int& noise_dbm) {
  if (len < 2) return false;
  int16_t x10 = (int16_t)((uint16_t)payload[0] | ((uint16_t)payload[1] << 8));
  // Round toward zero — fine for noise floor reporting (values ~ −100 .. −120).
  noise_dbm = x10 / 10;
  return true;
}

/**
 * Serialise RadioConfig to wire bytes (14 B, little-endian). The struct itself
 * is already packed and LE on our targets (ESP32 little-endian, packed attr),
 * so this is `memcpy` + a static assertion. Provided as a helper so callers
 * don't accidentally pass an unpacked layout.
 */
static inline void serialize_radio_config(const RadioConfig& cfg, uint8_t out[14]) {
  static_assert(sizeof(RadioConfig) == 14, "RadioConfig must be 14B for memcpy serialisation");
  memcpy(out, &cfg, 14);
}

// ─── LoRa time-on-air + packet score (host-side math) ─────────────────

/**
 * LoRa time-on-air estimate in milliseconds. Ported from RadioLib's
 * SX127x::calculateTimeOnAir() (which SX126x delegates to the same formula
 * via PhysicalLayer::getTimeOnAir). Source: SX127x.cpp lines 1142–1182.
 *
 * Assumptions matching MeshCore's RadioLibWrapper:
 *   - explicit header (ih=0)
 *   - CRC enabled (crc=1)
 *   - LDR optimisation auto-enabled when symbol duration ≥ 16 ms
 *     (Semtech-recommended threshold; what most stacks do.)
 *
 * `cr` is the coding-rate denominator in 5..8 form (5=4/5 ... 8=4/8) — the
 * same value the modem reports in CONFIG_RESP / accepts in SET_CONFIG.
 *
 * Returns 0 for invalid args (sf<5 or >12, bw==0, cr<5 or >8, len<0).
 */
static inline uint32_t estimate_airtime_ms(int payload_bytes, uint8_t sf,
                                           uint32_t bw_hz, uint8_t cr,
                                           uint16_t preamble_len) {
  if (sf < 5 || sf > 12 || bw_hz == 0 || cr < 5 || cr > 8 || payload_bytes < 0) {
    return 0;
  }
  double sym_us = (double)((uint32_t)1 << sf) * 1e6 / (double)bw_hz;
  double de  = (sym_us >= 16000.0) ? 1.0 : 0.0;
  double ih  = 0.0;
  double crc = 1.0;
  double num = 8.0 * (double)payload_bytes - 4.0 * (double)sf + 28.0 + 16.0 * crc - 20.0 * ih;
  double den = 4.0 * (double)sf - 8.0 * de;
  double blocks = (num > 0.0) ? ceil(num / den) : 0.0;
  double n_pay = 8.0 + blocks * (double)cr;
  if (n_pay < 8.0) n_pay = 8.0;
  double n_sym = (double)preamble_len + n_pay + 4.25;
  double airtime_us = ceil(sym_us * n_sym);
  return (uint32_t)(airtime_us / 1000.0);
}

/**
 * MeshCore packet score (0..1). Direct port of
 * RadioLibWrapper::packetScoreInt — threshold table is per-SF SNR floor
 * from the Semtech SX126x datasheet; collision penalty linear in length.
 */
static inline float packet_score(float snr, uint8_t sf, int packet_len) {
  // SNR thresholds for SF7..SF12 (dB).
  static const float snr_threshold[6] = { -7.5f, -10.0f, -12.5f, -15.0f, -17.5f, -20.0f };
  if (sf < 7 || sf > 12 || packet_len < 0) return 0.0f;
  float threshold = snr_threshold[sf - 7];
  if (snr < threshold) return 0.0f;
  float success_rate = (snr - threshold) / 10.0f;
  float collision_penalty = 1.0f - ((float)packet_len / 256.0f);
  float score = success_rate * collision_penalty;
  if (score < 0.0f) return 0.0f;
  if (score > 1.0f) return 1.0f;
  return score;
}

// ─── Frame parser (byte-by-byte state machine) ─────────────────────────
/**
 * Feed bytes from a non-blocking socket one at a time via feed().
 *
 *   FrameParser p;
 *   while (client.available()) {
 *     if (p.feed((uint8_t)client.read())) {
 *       dispatch(p.cmd, p.payload, p.payload_len);
 *       p.reset();
 *     } else if (p.crc_failed) {
 *       // bump crc error counter; parser already reset to SYNC
 *     }
 *   }
 *
 * Garbage before SYNC is silently skipped — the parser hunts for the next
 * SYNC byte. A CRC mismatch sets crc_failed and resets to SYNC so the next
 * good frame is found cleanly. There is no buffering of pre-SYNC garbage,
 * so a runaway peer cannot exhaust memory.
 */
struct FrameParser {
  enum class State : uint8_t {
    WAIT_SYNC,
    READ_CMD,
    READ_LEN_LO,
    READ_LEN_HI,
    READ_PAYLOAD,
    READ_CRC_LO,
    READ_CRC_HI,
  };

  State    state       = State::WAIT_SYNC;
  uint8_t  cmd         = 0;
  uint16_t payload_len = 0;
  uint16_t got         = 0;
  uint16_t crc_rx      = 0;
  bool     crc_failed  = false;
  uint8_t  payload[MAX_PAYLOAD] = {};

  void reset() {
    state       = State::WAIT_SYNC;
    cmd         = 0;
    payload_len = 0;
    got         = 0;
    crc_rx      = 0;
    crc_failed  = false;
  }

  /** Feed one byte. Returns true iff a complete frame is now parsed and
   *  validated; caller should consume {cmd, payload, payload_len} then reset().
   *  After a CRC mismatch, returns false and crc_failed is set until the next
   *  feed() call clears it. */
  bool feed(uint8_t b) {
    crc_failed = false;
    switch (state) {
      case State::WAIT_SYNC:
        if (b == SYNC) state = State::READ_CMD;
        return false;
      case State::READ_CMD:
        cmd   = b;
        state = State::READ_LEN_LO;
        return false;
      case State::READ_LEN_LO:
        payload_len = b;
        state       = State::READ_LEN_HI;
        return false;
      case State::READ_LEN_HI:
        payload_len |= (uint16_t)b << 8;
        if (payload_len > MAX_PAYLOAD) {
          // Oversize claim — abandon, hunt for next SYNC. Don't use reset()
          // because it clobbers crc_failed; do it inline.
          state = State::WAIT_SYNC;
          cmd = 0; payload_len = 0; got = 0; crc_rx = 0;
          return false;
        }
        got   = 0;
        state = (payload_len > 0) ? State::READ_PAYLOAD : State::READ_CRC_LO;
        return false;
      case State::READ_PAYLOAD:
        payload[got++] = b;
        if (got >= payload_len) state = State::READ_CRC_LO;
        return false;
      case State::READ_CRC_LO:
        crc_rx = b;
        state  = State::READ_CRC_HI;
        return false;
      case State::READ_CRC_HI: {
        crc_rx |= (uint16_t)b << 8;
        // CRC covers CMD + LEN(2) + PAYLOAD. Compute over a small header
        // then chain over payload bytes (bit-by-bit, no extra buffer).
        uint8_t hdr[3] = { cmd,
                           (uint8_t)(payload_len & 0xFF),
                           (uint8_t)((payload_len >> 8) & 0xFF) };
        uint16_t crc_calc = crc16_ccitt(hdr, 3);
        for (uint16_t i = 0; i < payload_len; i++) {
          crc_calc ^= (uint16_t)payload[i] << 8;
          for (uint8_t j = 0; j < 8; j++) {
            if (crc_calc & 0x8000) crc_calc = (uint16_t)((crc_calc << 1) ^ 0x1021);
            else                   crc_calc = (uint16_t)(crc_calc << 1);
          }
        }
        bool ok = (crc_calc == crc_rx);
        // Either way, the frame ends here: rearm to hunt for next SYNC.
        // We clear parsing state but preserve crc_failed (set below if !ok)
        // so the caller can observe the rejection on this very call.
        state = State::WAIT_SYNC;
        // Don't clear cmd/payload/payload_len/got on success: caller still
        // needs to read them. On failure, leaving them dangling is fine —
        // they'll be overwritten by the next frame.
        if (!ok) {
          crc_failed  = true;
          got         = 0;
          // Leave payload contents in place — caller doesn't get a "true"
          // return on this byte so won't try to consume them.
        }
        crc_rx = 0;
        return ok;
      }
    }
    return false;
  }
};

} // namespace pymc_proto
