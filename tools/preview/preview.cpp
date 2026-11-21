// Draws every clock face to PNG files, to check how they look without flashing the screen.
//   g++ -O2 -I../../src preview.cpp ../../src/clock.cpp -o preview && ./preview out-dir
// ./preview out-dir RRGGBB glow% unlit%   draws the digital faces with that colour (make_previews.py uses it)
#include "clock.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <chrono>

static void writePPM(const char *path, const uint16_t *b) {
  FILE *f = fopen(path, "wb");
  fprintf(f, "P6 %d %d 255\n", clk::W, clk::H);
  for (int i = 0; i < clk::W * clk::H; i++) {
    uint16_t v = b[i];
    unsigned char p[3] = {(unsigned char)(((v >> 11) & 31) * 255 / 31), (unsigned char)(((v >> 5) & 63) * 255 / 63),
                          (unsigned char)((v & 31) * 255 / 31)};
    fwrite(p, 1, 3, f);
  }
  fclose(f);
}

int main(int argc, char **argv) {
  const char *dir = argc > 1 ? argv[1] : ".";
  if (argc > 4)
    for (int f = 0; f < clk::FACE_COUNT; f++) {
      clk::setDigitColor(f, strtoul(argv[2], nullptr, 16));
      clk::setDigitLevels(f, atoi(argv[3]), atoi(argv[4]));
    }
  std::vector<uint16_t> dial(clk::W * clk::H), out(clk::W * clk::H);
  const float phases[clk::FACE_COUNT] = {0.18f, 0.5f, 0.62f, 0.85f, 0.5f, 0.1f, 0.7f, 0.3f, 0.45f, 0.9f, 0.62f};
  for (int f = 0; f < clk::FACE_COUNT; f++) {
    for (int v = 0; v < 2; v++) {               // with and without the extras
      clk::Dial d;
      d.face = f;
      d.moon = v == 0;
      d.moonPhase = phases[f];
      d.temps = v == 0;
      strcpy(d.in, "71");
      strcpy(d.out, f == 3 ? "-4" : "48");
      d.month = 12; d.mday = 9; d.wday = 3; d.daysToXmas = 16;
      if (v == 1 && f == 6) { d.month = 11; d.mday = 18; d.daysToXmas = 37; }
      if (v == 0) { strcpy(d.inHum, "55"); strcpy(d.outHum, "74"); strcpy(d.pres, f == 1 ? "29.03 INHG" : "983 HPA"); strcpy(d.presWord, f == 1 ? "CHANGE" : "FAIR"); d.presTrend = f % 3 - 1; }
      auto t0 = std::chrono::steady_clock::now();
      clk::renderDial(dial.data(), d);
      auto t1 = std::chrono::steady_clock::now();
      out = dial;
      clk::drawMinuteLayer(out.data(), f, 10, 9);
      auto t15 = std::chrono::steady_clock::now();
      clk::drawSecondLayer(out.data(), f, 10 + 9 / 60.0f, 9 + 36 / 60.0f, 36, true);
      auto t2 = std::chrono::steady_clock::now();
      char p[256];
      snprintf(p, sizeof p, "%s/%s%s.ppm", dir, clk::faceName(f), v ? "-plain" : "");
      writePPM(p, out.data());
      printf("%-12s dial %5.1f ms, minute layer %5.1f ms, second layer %4.1f ms\n", clk::faceName(f),
             std::chrono::duration<double, std::milli>(t1 - t0).count(),
             std::chrono::duration<double, std::milli>(t15 - t1).count(),
             std::chrono::duration<double, std::milli>(t2 - t15).count());
    }
  }
  return 0;
}
