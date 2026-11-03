// Host test for the PPM decoder:
//   g++ -std=c++17 -Wall -Isrc test/test_ppm.cpp -o /tmp/test_ppm && /tmp/test_ppm
#include <cassert>
#include <cstdio>
#include <vector>

#include "ppm_decoder.h"

static PpmDecoder dec;
static uint32_t t = 1000000;  // us
static int frames = 0;        // valid frames

static void edge() { frames += dec.edge(t); }

// One edge at the end of each channel, then the sync gap and the edge that closes the frame.
static void frame(const std::vector<uint32_t>& widths) {
  for (auto w : widths) { t += w; edge(); }
  t += 10000;
  edge();
}

int main() {
  const std::vector<uint32_t> good = {1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800};
  edge();  // opens the first frame

  frame(good);
  assert(frames == 0 && "the first frame is only a reference");
  frame(good);
  assert(frames == 1 && dec.count() == 8);
  for (int i = 0; i < 8; i++) assert(dec.channels()[i] == good[i]);

  // Noise: an extra edge splits CH3 into 300 + 1000. The frame is dropped.
  frame({1500, 1500, 300, 1000, 1500, 1500, 1500, 1500, 1500});
  assert(frames == 1 && dec.channels()[2] == 1300 && "channels must not shift");

  // Different channel count (9 valid channels): dropped.
  frame({1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500});
  assert(frames == 1 && dec.channels()[0] == 1100);

  // Pulse too long (2500 us): dropped.
  frame({2500, 1500, 1500, 1500, 1500, 1500, 1500, 1500});
  assert(frames == 1);

  // After noise, the second good frame is accepted again.
  const std::vector<uint32_t> back = {2000, 1000, 1500, 1500, 1500, 1500, 1500, 1500};
  frame(back);
  assert(frames == 1);
  frame(back);
  assert(frames == 2 && dec.channels()[0] == 2000 && dec.channels()[1] == 1000);

  puts("ok");
}
