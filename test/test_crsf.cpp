// Host test for the CRSF decoder:
// g++ -std=c++17 -Wall -Isrc test/test_crsf.cpp -o /tmp/test_crsf && /tmp/test_crsf
#include <cassert>
#include <cstdio>
#include <vector>

#include "crsf_decoder.h"
#include "crsf_menu.h"

// Builds an RC channels frame like OpenI6X pulses/crossfire.cpp (extra = status byte).
static std::vector<uint8_t> frame(const int* v, bool extra) {
  std::vector<uint8_t> f = {0xEE, (uint8_t)(extra ? 25 : 24), 0x16};
  uint32_t bits = 0;
  int avail = 0;
  for (int i = 0; i < 16; i++) {
    bits |= (uint32_t)v[i] << avail;
    avail += 11;
    while (avail >= 8) {
      f.push_back(bits);
      bits >>= 8;
      avail -= 8;
    }
  }
  if (extra) f.push_back(0x02);
  f.push_back(crsfCrc8(f.data() + 2, f.size() - 2));
  return f;
}

static int feed(CrsfDecoder& d, const std::vector<uint8_t>& bytes) {
  int frames = 0;
  for (uint8_t b : bytes) frames += d.byte(b) == 0x16;
  return frames;
}

int main() {
  int v[16];
  for (int i = 0; i < 16; i++) v[i] = 992;
  v[0] = 172;   // -100 %
  v[1] = 1811;  // +100 %
  v[2] = 992 + 800;

  for (bool extra : {false, true}) {
    CrsfDecoder d;
    assert(feed(d, frame(v, extra)) == 1);
    assert(d.channels()[0] == 988);
    assert(d.channels()[1] == 2011);
    assert(d.channels()[2] == 2000);
    assert(d.channels()[15] == 1500);
  }

  // Garbage before the frame, and a frame cut in the middle: the next whole frame still decodes.
  CrsfDecoder d;
  std::vector<uint8_t> f = frame(v, true);
  std::vector<uint8_t> s = {0x00, 0x13, 0xEE, 0xFF};  // 0xEE then an impossible length
  s.insert(s.end(), f.begin(), f.begin() + 10);
  assert(feed(d, s) == 0);
  // The cut frame swallows bytes of the next one until its length is reached, so send two.
  assert(feed(d, f) + feed(d, f) >= 1);

  // A corrupted byte fails the CRC.
  CrsfDecoder e;
  std::vector<uint8_t> bad = frame(v, true);
  bad[5] ^= 0x10;
  assert(feed(e, bad) == 0);
  assert(feed(e, frame(v, true)) == 1);

  // The radio's device ping (OpenI6X sends it until a module answers) is reported, not decoded.
  CrsfDecoder g;
  const uint8_t ping[] = {0xC8, 0x04, 0x28, 0x00, 0xEA, 0x54};  // captured from the radio
  uint8_t type = 0;
  for (uint8_t b : ping)
    if (uint8_t t = g.byte(b)) type = t;
  assert(type == 0x28);
  assert(feed(g, frame(v, false)) == 1);

  // Telemetry frame: sync, length, type, payload, CRC over type + payload.
  uint8_t tf[64];
  const uint8_t text[] = {'h', 'i', 0};
  assert(crsfFrame(tf, 0x21, text, 3) == 7);
  assert(tf[0] == 0xC8 && tf[1] == 5 && tf[2] == 0x21 && tf[5] == 0);
  assert(tf[6] == crsfCrc8(tf + 2, 4));

  // Menu entries, parsed the way OpenI6X tools/elrs.cpp does: after [id, chunks], paramData starts
  // at the parent; the type data starts after the name; the unit follows value, min, max, default.
  uint8_t entry[64];
  int n = menuEntry(1, entry);  // "Max tilt", UINT8 10 (5..25) "deg"
  const uint8_t* pd = entry + 2;
  assert(pd[0] == 0 && pd[1] == PARAM_UINT8 && !strcmp((const char*)pd + 2, "Max tilt"));
  int off = 2 + strlen("Max tilt") + 1;
  assert(pd[off] == 10 && pd[off + 1] == 5 && pd[off + 2] == 25);
  assert(!strcmp((const char*)pd + off + 4, "deg") && n == 2 + off + 4 + 4);
  n = menuEntry(7, entry);  // "Accessory", SELECT 3
  pd = entry + 2;
  off = 2 + strlen("Accessory") + 1;
  int optLen = strlen((const char*)pd + off) + 1;
  assert(pd[1] == PARAM_SELECT && pd[off + optLen] == 3 && pd[off + optLen + 4] == 0);
  assert(menuEntry(0, entry) == 0 && menuEntry(kMenuCount + 1, entry) == 0);
  assert(menuWrite(1, 20) && settings.maxTilt == 20 && settingsChanged);
  assert(!menuWrite(1, 30) && !menuWrite(kMenuCount, 1));  // out of range; INFO is read-only

  printf("crsf decoder: ok\n");
}
