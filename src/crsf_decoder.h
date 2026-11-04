// Hardware-independent CRSF decoder for the RC channels frame. Tested on the host by
// test/test_crsf.cpp.
#pragma once
#include <stdint.h>

// Frame: [address, length, type, payload..., CRC8]. length counts type + payload + CRC.
// The CRC (DVB-S2, polynomial 0xD5) covers type and payload.
// RC channels (type 0x16): 16 channels x 11 bits, LSB first. OpenI6X adds one status byte for
// ELRS arming, so its length is 25 instead of 24. Value 992 = 1500 us, 1 unit = 0.625 us.
inline uint8_t crsfCrc8(const uint8_t* d, int n) {
  uint8_t c = 0;
  for (int i = 0; i < n; i++) {
    c ^= d[i];
    for (int k = 0; k < 8; k++) c = (c & 0x80) ? (uint8_t)(c << 1) ^ 0xD5 : (uint8_t)(c << 1);
  }
  return c;
}

// Builds [0xC8, length, type, payload, CRC] into f (n + 4 bytes). Returns the frame size.
// The radio accepts 0xC8 (sync) as the first byte of a telemetry frame.
inline int crsfFrame(uint8_t* f, uint8_t type, const uint8_t* payload, uint8_t n) {
  f[0] = 0xC8;
  f[1] = n + 2;  // type + payload + CRC
  f[2] = type;
  for (int i = 0; i < n; i++) f[3 + i] = payload[i];
  f[3 + n] = crsfCrc8(f + 2, n + 1);
  return n + 4;
}

class CrsfDecoder {
 public:
  static constexpr int kChannels = 16;

  // Call for every received byte. Returns the type of a frame that ends with a valid CRC (0x16 = RC
  // channels, which updates channels(); 0x28 = device ping), or 0.
  uint8_t byte(uint8_t b);
  const uint16_t* channels() const { return us_; }
  // The frame that byte() just reported: [address, length, type, payload..., CRC]. Valid until the
  // next call.
  const uint8_t* frame() const { return buf_; }

 private:
  uint8_t buf_[64];
  uint8_t n_ = 0;
  uint16_t us_[kChannels] = {};
};

inline uint8_t CrsfDecoder::byte(uint8_t b) {
  if (n_ == 0 && b != 0xEE && b != 0xC8) return 0;  // module address or sync byte
  buf_[n_++] = b;
  if (n_ == 2 && (b < 2 || b > 62)) n_ = 0;  // impossible length: wait for the next address
  if (n_ < 2 || n_ < buf_[1] + 2) return 0;

  uint8_t len = buf_[1];
  n_ = 0;
  if (crsfCrc8(buf_ + 2, len - 1) != buf_[len + 1]) return 0;
  if (buf_[2] != 0x16) return buf_[2];
  if (len < 24) return 0;

  const uint8_t* p = buf_ + 3;
  uint32_t bits = 0;
  int avail = 0;
  for (int i = 0; i < kChannels; i++) {
    while (avail < 11) {
      bits |= (uint32_t)*p++ << avail;
      avail += 8;
    }
    int v = bits & 0x7FF;
    bits >>= 11;
    avail -= 11;
    us_[i] = 1500 + (v - 992) * 5 / 8;
  }
  return 0x16;
}
