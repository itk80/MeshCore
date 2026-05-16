# mesh::Radio interface → pymc_usb wire protocol v0.7 mapping

Source: `src/Dispatcher.h` (class Radio, lines 22–79), `src/Dispatcher.cpp` (call sites),
`src/helpers/radiolib/RadioLibWrappers.{h,cpp}` (base RadioLibWrapper),
`src/helpers/radiolib/CustomSX1262Wrapper.h` (concrete SPI wrapper — pattern to copy).

`CustomTCPRadioWrapper` will inherit directly from `mesh::Radio` (not from
`RadioLibWrapper`, because there is no `PhysicalLayer*` underneath — the
modem on the other side of TCP owns the RadioLib instance).

---

## Exact virtual signatures (must match)

```cpp
class Radio {
public:
  virtual void begin() { }                                    // non-pure
  virtual int recvRaw(uint8_t* bytes, int sz) = 0;
  virtual uint32_t getEstAirtimeFor(int len_bytes) = 0;
  virtual float packetScore(float snr, int packet_len) = 0;
  virtual bool startSendRaw(const uint8_t* bytes, int len) = 0;
  virtual bool isSendComplete() = 0;
  virtual void onSendFinished() = 0;
  virtual void loop() { }                                     // non-pure
  virtual int getNoiseFloor() const { return 0; }             // const!
  virtual void triggerNoiseFloorCalibrate(int threshold) { }
  virtual void resetAGC() { }
  virtual bool isInRecvMode() const = 0;                      // const!
  virtual bool isReceiving() { return false; }                // NOT const
  virtual float getLastRSSI() const { return 0; }             // const!
  virtual float getLastSNR() const { return 0; }              // const!
};
```

Notes on `const`-correctness:
- `getNoiseFloor`, `isInRecvMode`, `getLastRSSI`, `getLastSNR` are **`const`**.
  Mutable cache fields in TCPRadio for these must be either `mutable` or
  updated only via the non-const handlers (preferred — they live in `loop()`).
- `isReceiving()` is **not** `const` (in case a subclass wants to issue a CAD
  on the fly). For Variant A (auto-CAD modem-side) we return `false` and don't
  need it mutable.

---

## How Dispatcher uses each method (semantics from Dispatcher.cpp)

| Method | When called | Expected semantics | Sync/async OK? |
|---|---|---|---|
| `begin()` | `Dispatcher::begin()` once at boot, before main loop. After this `isInRecvMode()` is queried once and stored as `prev_isrecv_mode`. | Initialise hardware, enter RX mode if possible. May block briefly. | Sync (boot-time blocking acceptable). |
| `loop()` | Every `Dispatcher::loop()` iteration (very frequent). | Background pump — poll any async sources, fill ring buffers, update cached metadata. Must return quickly. | **Must be non-blocking.** |
| `triggerNoiseFloorCalibrate(threshold)` | Every `NOISE_FLOOR_CALIB_INTERVAL` (2 s) from `Dispatcher::loop()`. Threshold = `getInterferenceThreshold()` (0 by default → disabled). | Reset the floor sampler; subsequent samples get averaged. Threshold value is what `isChannelActive()` would compare against. | Async (fire-and-forget). |
| `isInRecvMode()` | Every `Dispatcher::loop()`. If false for >8 s, sets `ERR_EVENT_STARTRX_TIMEOUT`. | True iff the radio is currently listening for packets (not idle, not TX). | Sync, return cached state. |
| `recvRaw(buf, sz)` | Every `Dispatcher::loop()` from `checkRecv()`. | Return 0 if nothing pending. Return packet length and fill `buf` if a packet arrived since last call. | Sync, must be O(1). |
| `getLastRSSI() / getLastSNR()` | Immediately after a successful `recvRaw()` (logRxRaw + `pkt->_snr = snr*4`). Also in `MESH_PACKET_LOGGING` block. | Metadata of the most recently returned packet. Must be valid AT LEAST until the next `recvRaw()` call. | Sync, cached. |
| `packetScore(snr, len)` | Right after `recvRaw()` returns a parsed packet. Result steers RX delay calc and queueing. | Pure function of snr+len+sf. Same formula as `RadioLibWrapper::packetScoreInt(snr, sf, len)`. | Sync, pure math. |
| `getEstAirtimeFor(len)` | After `recvRaw()` (`rx_air_time += ...`) AND in `checkSend()` for budget reservation AND for `outbound_expiry`. Called per packet. | Time-on-air in ms for `len` bytes given current LoRa params (freq/bw/sf/cr/preamble). | Sync, pure math. |
| `isReceiving()` | Just before each TX in `checkSend()`. If true, Dispatcher backs off `getCADFailRetryDelay()` ms and retries. | True iff host-side LBT should currently block transmit. **Variant A: always return false** (modem does auto-CAD). | Sync, must be O(1). |
| `startSendRaw(buf, len)` | From `checkSend()` when budget allows and channel is clear. | Begin transmit. Return true if accepted, false on error. Do NOT block until TX done. | Async kickoff (return immediately). |
| `isSendComplete()` | Every `Dispatcher::loop()` while `outbound != NULL`. Latches `outbound_start → total_air_time` on true. | True once the previous `startSendRaw()` finished (success or fail). After it returns true the Dispatcher calls `onSendFinished()` and releases the packet. | Sync, poll cached state. |
| `onSendFinished()` | Right after `isSendComplete()` returns true, OR on `outbound_expiry` timeout. | TX cleanup. RadioLibWrapper does `_radio->finishTransmit()` then back to idle (next `recvRaw()` re-arms RX). For TCPRadio: modem returns to RX itself; here just reset our TX state. | Sync, lightweight. |
| `getNoiseFloor()` | Not called by Dispatcher directly; exposed to apps (CLI `get noise.floor`). | Last sampled noise floor in dBm (negative integer ~-100..-120). | Sync, return cached. |
| `resetAGC()` | Every `getAGCResetInterval()` ms (0 by default → disabled). | Soft-reset RX frontend. RadioLibWrapper does warm-sleep + re-arm. | Async or nop. |

Critical timing constraints:
- `loop()` runs at ~kHz. **No blocking I/O.** Socket reads must be non-blocking.
- `recvRaw()` and `isSendComplete()` are polled hot. They must consult local
  state set by `loop()`'s frame parser, not issue TCP requests.
- `startSendRaw()` returning `true` only means "queued for transmit"; the
  Dispatcher then waits for `isSendComplete()` for up to `1.5 × est_airtime`.
  TCP RTT (~5 ms LAN) + air time (300–2500 ms typical for MeshCore at
  SF8/BW62.5) → 1.5× margin is comfortable, no special handling needed.

---

## Mapping table: Radio method → wire protocol v0.7

| Radio method | Wire command(s) | Direction & timing |
|---|---|---|
| `begin()` | TCP connect → `CMD_AUTH` (0x50, if token configured) → `CMD_SET_CONFIG` (0x10) → `CMD_SET_AUTO_CAD` (0x4A, value=1) → `CMD_RX_START` (0x31) | Sequential, await each response. Boot-time blocking ≤2 s acceptable. |
| `loop()` | RX side: read TCP non-blocking, parse `SYNC/CMD/LEN/PAYLOAD/CRC` frames, dispatch by CMD: `EVT_RX_PACKET` (0x04) → push to `_rx_ring`; `EVT_TX_DONE` (0x02) / `EVT_TX_FAIL` (0x03) → update `_tx_state`; `EVT_NOISE_RESP` (0x23) → update `_noise_floor`; `EVT_LOG_MSG` (0x80) → forward to Serial; `EVT_PONG` (0xFF) → mark watchdog alive. TX side: drain any pending host→modem frames buffered. Also: reconnect logic on `!_client.connected()`. | Async pump. |
| `recvRaw(dst, sz)` | None (consumer of `_rx_ring`). | Sync, O(1). |
| `startSendRaw(src, len)` | `CMD_TX_REQUEST` (0x01), payload = raw bytes (len ≤ 255). Set `_tx_state = TX_PENDING`. | Async kickoff. |
| `isSendComplete()` | None (returns `_tx_state != TX_PENDING`). | Sync, O(1). |
| `onSendFinished()` | None (modem auto-returns to RX). Just `_tx_state = TX_IDLE`. | Sync. |
| `isInRecvMode()` | None (returns cached `_rx_started && _connected && _authenticated`). | Sync. |
| `isReceiving()` | None — **Variant A: always returns false**, modem does CAD. | Sync. |
| `getEstAirtimeFor(len)` | None — pure math host-side using `_freq/_bw/_sf/_cr/_preamble` snapshot from `sendConfig()`. Formula from RadioLib `getTimeOnAir(len)/1000`. | Sync, pure math. |
| `packetScore(snr, len)` | None — copy `RadioLibWrapper::packetScoreInt(snr, _sf, len)`. | Sync, pure math. |
| `getLastRSSI() / getLastSNR()` | None — read `_last_rssi / _last_snr` set by `handleRxPacket()`. | Sync, cached. |
| `triggerNoiseFloorCalibrate(threshold)` | `CMD_NOISE_REQ` (0x22). Response `EVT_NOISE_RESP` (0x23) handled in `loop()`. Throttle: skip if already pending or last result <2 s old. | Async (fire-and-forget). |
| `getNoiseFloor()` | None (returns cached `_noise_floor`). | Sync. |
| `resetAGC()` | **nop** for v1. Add `CMD_RESET_AGC` to protocol later if needed (R1 in PLAN.md). | nop. |

Methods/commands NOT used in MVP but worth knowing:
- `CMD_CAD_REQUEST` (0x30) / `EVT_CAD_RESP` (0x32) — Variant C fallback only.
- `CMD_GET_CONFIG` (0x11) / `EVT_CONFIG_RESP` (0x12) — used as ACK of SET_CONFIG too.
- `CMD_STATUS_REQ` (0x20) / `EVT_STATUS_RESP` (0x21) — diagnostics CLI.
- `CMD_RADIO_STANDBY` (0x40) / `CMD_RADIO_RESUME` (0x42) — power management.
- `CMD_PING` (0xFF) / `EVT_PONG` (0xFF) — watchdog every 5–10 s.
- `CMD_GET_VERSION` (0x70) / `EVT_VERSION_RESP` (0x71) — log on connect, compat check.

---

## Corrections vs CLAUDE.md / PLAN.md (from real protocol.h)

CLAUDE.md and PLAN.md describe `RadioConfig` as 14 bytes with a `flags` byte.
The actual `firmware/include/protocol.h` in pymc_usb v0.7 defines it as **13 bytes**
with different widths for two fields:

| Field | CLAUDE.md / PLAN.md | Real protocol.h |
|---|---|---|
| `frequency_hz` | uint32 | uint32 ✓ |
| `bandwidth_hz` | uint32 | uint32 ✓ |
| `sf` | uint8 | uint8 ✓ |
| `cr` | uint8 | uint8 ✓ |
| `tx_power_dbm` | int8 | int8 (named `power_dbm`) ✓ |
| `sync_word` | **uint8** | **uint16** ⚠ |
| `preamble_len` | **uint16** | **uint8** ⚠ |
| `flags` | uint8 | **does not exist** ⚠ |
| **Total** | **14 B** | **13 B** |

Other naming corrections: the response/event opcodes are named `CMD_*` in
both directions in protocol.h (e.g. `CMD_TX_DONE`, `CMD_RX_PACKET`,
`CMD_PONG`), not `EVT_*` as PLAN.md hints. We can keep a logical `Evt`
prefix in our `pymc_proto` namespace for readability, but the numeric values
must come from protocol.h verbatim.

Extra opcodes present in v0.7 that PLAN.md / CLAUDE.md don't list:
- `CMD_RADIO_STANDBY_RESP` 0x44, `CMD_RADIO_RESUME_RESP` 0x46
- `CMD_SET_DISPLAY_NAME_RESP` 0x49, `CMD_SET_AUTO_CAD_RESP` 0x4B
- `CMD_CAD_PARAMS_RESP` 0x35, `CMD_SET_CAD_PARAMS` 0x34
- `CMD_ENTER_BOOTLOADER` 0x74
- OTA family: `CMD_OTA_BEGIN/CHUNK/VERIFY/APPLY/ABORT` + responses

Error codes are an enum in protocol.h (ERR_CRC_MISMATCH=0x01 … ERR_CHANNEL_BUSY=0x0E).

## Open questions for Jarek

1. **`CMD_SET_AUTO_CAD` (0x4A) on ESP32?** README of pymc_usb suggests T114-only.
   If not supported on the modem's ESP32 build, we either:
   - extend firmware to support it (preferred), or
   - fall back to Variant C (synchronous `CMD_CAD_REQUEST` in `isReceiving()`)
     with ~20–50 ms LAN latency penalty per TX attempt.
2. **Auth flow:** is `CMD_AUTH` mandatory when the modem has no token configured,
   or skip silently? Need to check `firmware/include/protocol.h`.
3. **`EVT_RX_PACKET` payload layout:** confirm exact byte order (RSSI / SNR
   as int16 LE? Big-endian? scaled ×10?). Spec in PLAN.md says "dBm × 10" —
   verify in pymc_usb firmware before consuming.
4. **Max payload for `CMD_TX_REQUEST`:** MeshCore `MAX_TRANS_UNIT` is the
   cap. We need to confirm modem accepts the full size (256 with header?).
