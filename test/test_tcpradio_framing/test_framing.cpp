// Unit tests for pymc_proto framing/CRC. Runs under [env:native] gtest.
// Fixtures captured from live modem at 192.168.5.11 (lilygo-t3s3 v0.7.0).

#include <gtest/gtest.h>
#include "helpers/tcpradio/pymc_proto.h"

using namespace pymc_proto;

// ── CRC-16/CCITT against vectors confirmed by live modem ───────────────

TEST(Crc, Ping) {
  // CMD=0xFF LEN=0x0000 — no payload.
  uint8_t hdr[] = {0xFF, 0x00, 0x00};
  EXPECT_EQ(0x03FF, crc16_ccitt(hdr, sizeof(hdr)));
}

TEST(Crc, GetVersion) {
  uint8_t hdr[] = {CMD_GET_VERSION, 0x00, 0x00};
  EXPECT_EQ(0x1494, crc16_ccitt(hdr, sizeof(hdr)));
}

TEST(Crc, SetAutoCadOn) {
  // CMD=0x4A LEN=0x0001 PAYLOAD=0x01
  uint8_t buf[] = {CMD_SET_AUTO_CAD, 0x01, 0x00, 0x01};
  EXPECT_EQ(0xA5E6, crc16_ccitt(buf, sizeof(buf)));
}

TEST(Crc, EmptyInput) {
  // 0xFFFF is the initial value; on empty input it must come back unchanged.
  EXPECT_EQ(0xFFFF, crc16_ccitt(nullptr, 0));
}

// ── RadioConfig wire layout ────────────────────────────────────────────

TEST(RadioConfigStruct, IsExactly14Bytes) {
  EXPECT_EQ(14u, sizeof(RadioConfig));
}

TEST(RadioConfigStruct, FieldOffsetsMatchProtocolH) {
  // Match the order in pymc_usb/firmware/include/protocol.h.
  RadioConfig c{};
  uint8_t* base = reinterpret_cast<uint8_t*>(&c);
  EXPECT_EQ(0,  reinterpret_cast<uint8_t*>(&c.freq_hz)      - base);
  EXPECT_EQ(4,  reinterpret_cast<uint8_t*>(&c.bandwidth_hz) - base);
  EXPECT_EQ(8,  reinterpret_cast<uint8_t*>(&c.sf)           - base);
  EXPECT_EQ(9,  reinterpret_cast<uint8_t*>(&c.cr)           - base);
  EXPECT_EQ(10, reinterpret_cast<uint8_t*>(&c.power_dbm)    - base);
  EXPECT_EQ(11, reinterpret_cast<uint8_t*>(&c.syncword)     - base);
  EXPECT_EQ(13, reinterpret_cast<uint8_t*>(&c.preamble_len) - base);
}

// ── build_frame: structural correctness ────────────────────────────────

TEST(BuildFrame, PingNoPayload) {
  uint8_t out[MAX_FRAME_SIZE];
  size_t n = build_frame(CMD_PING, nullptr, 0, out);
  ASSERT_EQ(6u, n);                 // SYNC + CMD + LEN(2) + CRC(2)
  EXPECT_EQ(SYNC,    out[0]);
  EXPECT_EQ(CMD_PING, out[1]);
  EXPECT_EQ(0x00,    out[2]);       // LEN lo
  EXPECT_EQ(0x00,    out[3]);       // LEN hi
  EXPECT_EQ(0xFF,    out[4]);       // CRC lo (0x03FF)
  EXPECT_EQ(0x03,    out[5]);       // CRC hi
}

TEST(BuildFrame, SetAutoCadOnPayload) {
  uint8_t payload[] = {0x01};
  uint8_t out[MAX_FRAME_SIZE];
  size_t n = build_frame(CMD_SET_AUTO_CAD, payload, sizeof(payload), out);
  ASSERT_EQ(7u, n);
  EXPECT_EQ(SYNC,             out[0]);
  EXPECT_EQ(CMD_SET_AUTO_CAD, out[1]);
  EXPECT_EQ(0x01,             out[2]);
  EXPECT_EQ(0x00,             out[3]);
  EXPECT_EQ(0x01,             out[4]);
  EXPECT_EQ(0xE6,             out[5]);   // CRC lo (0xA5E6)
  EXPECT_EQ(0xA5,             out[6]);   // CRC hi
}

TEST(BuildFrame, OversizePayloadRejected) {
  uint8_t out[MAX_FRAME_SIZE + 10];
  uint8_t big[MAX_PAYLOAD + 1] = {};
  EXPECT_EQ(0u, build_frame(0x99, big, sizeof(big), out));
}

// ── FrameParser: happy path ────────────────────────────────────────────

TEST(FrameParser, ParsesPingRoundtrip) {
  uint8_t out[MAX_FRAME_SIZE];
  size_t n = build_frame(CMD_PING, nullptr, 0, out);

  FrameParser p;
  bool done = false;
  for (size_t i = 0; i < n; i++) {
    if (p.feed(out[i])) { done = true; EXPECT_EQ(i, n - 1); break; }
    EXPECT_FALSE(p.crc_failed);
  }
  ASSERT_TRUE(done);
  EXPECT_EQ(CMD_PING, p.cmd);
  EXPECT_EQ(0u, p.payload_len);
}

TEST(FrameParser, ParsesWithPayload) {
  uint8_t payload[5] = {0xDE, 0xAD, 0xBE, 0xEF, 0x42};
  uint8_t out[MAX_FRAME_SIZE];
  size_t n = build_frame(EVT_NOISE_RESP, payload, sizeof(payload), out);

  FrameParser p;
  bool done = false;
  for (size_t i = 0; i < n; i++) {
    if (p.feed(out[i])) done = true;
  }
  ASSERT_TRUE(done);
  EXPECT_EQ(EVT_NOISE_RESP, p.cmd);
  ASSERT_EQ(sizeof(payload), p.payload_len);
  EXPECT_EQ(0, memcmp(payload, p.payload, sizeof(payload)));
}

TEST(FrameParser, ParsesMaxPayload) {
  uint8_t payload[MAX_PAYLOAD];
  for (size_t i = 0; i < MAX_PAYLOAD; i++) payload[i] = (uint8_t)(i & 0xFF);

  uint8_t out[MAX_FRAME_SIZE];
  size_t n = build_frame(EVT_RX_PACKET, payload, MAX_PAYLOAD, out);
  ASSERT_GT(n, 0u);

  FrameParser p;
  bool done = false;
  for (size_t i = 0; i < n; i++) {
    if (p.feed(out[i])) done = true;
  }
  ASSERT_TRUE(done);
  EXPECT_EQ(EVT_RX_PACKET, p.cmd);
  ASSERT_EQ(MAX_PAYLOAD, p.payload_len);
  EXPECT_EQ(0, memcmp(payload, p.payload, MAX_PAYLOAD));
}

// ── FrameParser: error paths ───────────────────────────────────────────

TEST(FrameParser, SkipsLeadingGarbage) {
  uint8_t out[MAX_FRAME_SIZE];
  size_t n = build_frame(CMD_PING, nullptr, 0, out);

  FrameParser p;
  // 4 bytes of garbage before the frame
  EXPECT_FALSE(p.feed(0x00));
  EXPECT_FALSE(p.feed(0x55));
  EXPECT_FALSE(p.feed(0xA9));
  EXPECT_FALSE(p.feed(0x11));
  bool done = false;
  for (size_t i = 0; i < n; i++) {
    if (p.feed(out[i])) done = true;
  }
  EXPECT_TRUE(done);
  EXPECT_EQ(CMD_PING, p.cmd);
}

TEST(FrameParser, RejectsBadCrc) {
  uint8_t out[MAX_FRAME_SIZE];
  size_t n = build_frame(CMD_PING, nullptr, 0, out);
  out[n - 1] ^= 0x01;   // flip a bit in CRC hi

  FrameParser p;
  bool done = false;
  for (size_t i = 0; i < n; i++) {
    if (p.feed(out[i])) done = true;
  }
  EXPECT_FALSE(done);
  EXPECT_TRUE(p.crc_failed);
}

TEST(FrameParser, RecoversAfterBadCrc) {
  uint8_t out_bad[MAX_FRAME_SIZE], out_good[MAX_FRAME_SIZE];
  size_t nb = build_frame(CMD_PING, nullptr, 0, out_bad);
  size_t ng = build_frame(CMD_GET_VERSION, nullptr, 0, out_good);
  out_bad[nb - 1] ^= 0xFF;

  FrameParser p;
  for (size_t i = 0; i < nb; i++) p.feed(out_bad[i]);
  ASSERT_TRUE(p.crc_failed);
  // After bad CRC the parser should be hunting for SYNC again. Feed a clean frame:
  bool done = false;
  for (size_t i = 0; i < ng; i++) {
    if (p.feed(out_good[i])) done = true;
  }
  EXPECT_TRUE(done);
  EXPECT_EQ(CMD_GET_VERSION, p.cmd);
}

TEST(FrameParser, RejectsOversizeLength) {
  // SYNC, CMD, LEN=0xFFFF (> MAX_PAYLOAD)
  FrameParser p;
  EXPECT_FALSE(p.feed(SYNC));
  EXPECT_FALSE(p.feed(0x99));
  EXPECT_FALSE(p.feed(0xFF));
  EXPECT_FALSE(p.feed(0xFF));   // would reset to WAIT_SYNC
  EXPECT_EQ(FrameParser::State::WAIT_SYNC, p.state);
}

// ── parse_rx_packet (step 1.5) ─────────────────────────────────────────

TEST(ParseRxPacket, ValidPayloadWithLoraData) {
  // RSSI = -51 dBm (0xFFCD LE), SNR = +120 (0x0078 LE),
  // SIG_RSSI = -55 dBm (0xFFC9 LE), then 3 bytes LoRa data.
  uint8_t pl[] = { 0xCD, 0xFF,   0x78, 0x00,   0xC9, 0xFF,   0xAA, 0xBB, 0xCC };
  RxPacketMeta meta;
  const uint8_t* data = nullptr;
  int n = parse_rx_packet(pl, sizeof(pl), meta, data);
  ASSERT_EQ(3, n);
  EXPECT_EQ(-51,  meta.rssi_dbm);
  EXPECT_EQ(120,  meta.snr_x10);
  EXPECT_EQ(-55,  meta.sig_rssi_dbm);
  ASSERT_NE(nullptr, data);
  EXPECT_EQ(0xAA, data[0]);
  EXPECT_EQ(0xBB, data[1]);
  EXPECT_EQ(0xCC, data[2]);
}

TEST(ParseRxPacket, EmptyLoraData) {
  uint8_t pl[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  RxPacketMeta meta;
  const uint8_t* data = nullptr;
  int n = parse_rx_packet(pl, sizeof(pl), meta, data);
  EXPECT_EQ(0, n);
  EXPECT_EQ(data, pl + 6);
}

TEST(ParseRxPacket, TooShortReturnsMinusOne) {
  uint8_t pl[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
  RxPacketMeta meta;
  const uint8_t* data = nullptr;
  EXPECT_EQ(-1, parse_rx_packet(pl, sizeof(pl), meta, data));
}

// ── parse_tx_done (step 1.6) ───────────────────────────────────────────

TEST(ParseTxDone, ValidPayload) {
  // 4B LE: 0x12345678 = 305 419 896 microseconds
  uint8_t pl[] = { 0x78, 0x56, 0x34, 0x12 };
  uint32_t airtime = 0;
  ASSERT_TRUE(parse_tx_done(pl, sizeof(pl), airtime));
  EXPECT_EQ(0x12345678u, airtime);
}

TEST(ParseTxDone, TooShort) {
  uint8_t pl[] = { 0x01, 0x02, 0x03 };
  uint32_t airtime = 0xDEAD;
  EXPECT_FALSE(parse_tx_done(pl, sizeof(pl), airtime));
  EXPECT_EQ(0xDEADu, airtime);  // unchanged on failure
}

// ── parse_noise_resp ───────────────────────────────────────────────────

TEST(ParseNoiseResp, MinusOneFourteenDbm) {
  // Live modem reported: int16=-1140 → -114.0 dBm.
  uint8_t pl[] = { 0x8C, 0xFB };  // 0xFB8C = -1140 little-endian
  int dbm = 0;
  ASSERT_TRUE(parse_noise_resp(pl, sizeof(pl), dbm));
  EXPECT_EQ(-114, dbm);
}

TEST(ParseNoiseResp, TooShort) {
  uint8_t pl[] = { 0x42 };
  int dbm = 0;
  EXPECT_FALSE(parse_noise_resp(pl, sizeof(pl), dbm));
}

// ── RadioConfig serialisation ──────────────────────────────────────────

TEST(SerializeRadioConfig, MatchesLiveModemSnapshot) {
  // Snapshot from .5.11 GET_CONFIG: 869.618 MHz, 62.5 kHz, SF8, CR 4/8,
  // +22 dBm, sw 0x0012, preamble 16.
  RadioConfig c{};
  c.freq_hz      = 869618000;
  c.bandwidth_hz = 62500;
  c.sf           = 8;
  c.cr           = 8;
  c.power_dbm    = 22;
  c.syncword     = 0x0012;
  c.preamble_len = 16;

  uint8_t out[14];
  serialize_radio_config(c, out);

  // freq_hz (4B LE) — 869'618'000 = 0x33D55150
  EXPECT_EQ(0x50, out[0]);
  EXPECT_EQ(0x51, out[1]);
  EXPECT_EQ(0xD5, out[2]);
  EXPECT_EQ(0x33, out[3]);
  // bandwidth_hz (4B LE) — 62500 = 0x0000F424
  EXPECT_EQ(0x24, out[4]);
  EXPECT_EQ(0xF4, out[5]);
  EXPECT_EQ(0x00, out[6]);
  EXPECT_EQ(0x00, out[7]);
  EXPECT_EQ(8,    out[8]);
  EXPECT_EQ(8,    out[9]);
  EXPECT_EQ(22,   (int8_t)out[10]);
  // syncword (2B LE)
  EXPECT_EQ(0x12, out[11]);
  EXPECT_EQ(0x00, out[12]);
  EXPECT_EQ(16,   out[13]);
}

// ── estimate_airtime_ms (step 1.7) ─────────────────────────────────────
// Fixture values computed in Python from the same Semtech formula; matches
// RadioLib SX127x::calculateTimeOnAir() / 1000.

TEST(EstimateAirtime, MeshCoreEuNarrowStandard) {
  // SF=8, BW=62.5 kHz, CR=4/8, preamble=16 — the .5.11 modem's live config.
  EXPECT_EQ(148u,  estimate_airtime_ms(  1, 8, 62500, 8, 16));
  EXPECT_EQ(214u,  estimate_airtime_ms( 10, 8, 62500, 8, 16));
  EXPECT_EQ(541u,  estimate_airtime_ms( 50, 8, 62500, 8, 16));
  EXPECT_EQ(967u,  estimate_airtime_ms(100, 8, 62500, 8, 16));
  EXPECT_EQ(1786u, estimate_airtime_ms(200, 8, 62500, 8, 16));
  EXPECT_EQ(2245u, estimate_airtime_ms(255, 8, 62500, 8, 16));
}

TEST(EstimateAirtime, Sf10Bw125Khz) {
  // BW=125kHz, SF=10, CR=4/5, preamble=8 — common LoRaWAN-ish preset.
  EXPECT_EQ(288u,  estimate_airtime_ms( 10, 10, 125000, 5, 8));
  EXPECT_EQ(1026u, estimate_airtime_ms(100, 10, 125000, 5, 8));
  EXPECT_EQ(1845u, estimate_airtime_ms(200, 10, 125000, 5, 8));
}

TEST(EstimateAirtime, LdrOptimizationKicksInForSlowSymbols) {
  // SF=12, BW=62.5kHz → sym = 65.5 ms ≫ 16 ms → LDR on.
  EXPECT_EQ(2899u, estimate_airtime_ms(10, 12, 62500, 8, 16));
}

TEST(EstimateAirtime, RejectsInvalidArgs) {
  EXPECT_EQ(0u, estimate_airtime_ms(10,  4, 62500, 8, 16));  // sf<5
  EXPECT_EQ(0u, estimate_airtime_ms(10, 13, 62500, 8, 16));  // sf>12
  EXPECT_EQ(0u, estimate_airtime_ms(10,  8,     0, 8, 16));  // bw=0
  EXPECT_EQ(0u, estimate_airtime_ms(10,  8, 62500, 4, 16));  // cr<5
  EXPECT_EQ(0u, estimate_airtime_ms(10,  8, 62500, 9, 16));  // cr>8
  EXPECT_EQ(0u, estimate_airtime_ms(-1,  8, 62500, 8, 16));  // negative len
}

// ── packet_score (step 1.7) ────────────────────────────────────────────

TEST(PacketScore, BelowThresholdReturnsZero) {
  // SF=8 threshold = -10 dB. SNR -11 is below.
  EXPECT_FLOAT_EQ(0.0f, packet_score(-11.0f, 8, 50));
}

TEST(PacketScore, ExcellentSnrCapsAtOne) {
  // SF=8, SNR=+20 → success_rate = 30/10 = 3.0, very large; collision_penalty
  // for len=10 ≈ 0.961; product ≈ 2.88 → capped to 1.0.
  EXPECT_FLOAT_EQ(1.0f, packet_score(20.0f, 8, 10));
}

TEST(PacketScore, ModerateSnrAndLength) {
  // SF=8 threshold -10. SNR=-5 → success_rate = 5/10 = 0.5.
  // len=50 → collision_penalty = 1 - 50/256 ≈ 0.8047.
  // expected ≈ 0.4023.
  float s = packet_score(-5.0f, 8, 50);
  EXPECT_NEAR(0.40234f, s, 0.0005f);
}

TEST(PacketScore, MaxLengthGoesToZero) {
  // len=256 → collision_penalty = 0 → score=0 regardless of SNR.
  EXPECT_FLOAT_EQ(0.0f, packet_score(20.0f, 8, 256));
}

TEST(PacketScore, RejectsUnknownSf) {
  EXPECT_FLOAT_EQ(0.0f, packet_score(20.0f, 6, 50));   // SF<7
  EXPECT_FLOAT_EQ(0.0f, packet_score(20.0f, 13, 50));  // SF>12
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
