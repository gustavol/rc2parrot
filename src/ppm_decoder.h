// Hardware-independent PPM decoder. Tested on the host by test/test_ppm.cpp.
#pragma once
#include <stdint.h>
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// Measures rising edge to rising edge, so it works with positive and inverted PPM.
// A pulse outside 800..2200 us invalidates the frame (same rule as DIY-Multiprotocol PPM_decode).
// A frame is also rejected if its channel count differs from the previous frame, so a noise
// spike cannot shift the channels and trigger the kill switch.
class PpmDecoder {
 public:
  static constexpr int kMaxChannels = 16;

  // Call on every rising edge. Returns true when a valid frame ends.
  bool edge(uint32_t nowUs);
  const uint16_t* channels() const { return frame_; }
  uint8_t count() const { return count_; }

 private:
  uint32_t last_ = 0;
  uint16_t buf_[kMaxChannels] = {};
  uint16_t frame_[kMaxChannels] = {};
  uint8_t ch_ = 0, prevCh_ = 0, count_ = 0;
  bool bad_ = true;  // the first frame starts mid-way
};

inline bool IRAM_ATTR PpmDecoder::edge(uint32_t nowUs) {
  uint32_t w = nowUs - last_;
  last_ = nowUs;
  if (w > 3000) {  // sync gap = end of frame
    bool ok = !bad_ && ch_ >= 6 && ch_ == prevCh_;
    if (ok) {
      for (int i = 0; i < ch_; i++) frame_[i] = buf_[i];
      count_ = ch_;
    }
    prevCh_ = bad_ ? 0 : ch_;
    ch_ = 0;
    bad_ = false;
    return ok;
  }
  if (w < 800 || w > 2200 || ch_ >= kMaxChannels) bad_ = true;
  else buf_[ch_++] = w;
  return false;
}
