// Draws every clock face to PNG files, to check how they look without flashing the screen.
//   g++ -O2 -I../../src preview.cpp ../../src/clock.cpp -o preview && ./preview out-dir
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
  std::vector<uint16_t> dial(clk::W * clk::H), out(clk::W * clk::H);
  const float phases[clk::FACE_COUNT] = {0.18f, 0.5f, 0.62f, 0.85f};
  for (int f = 0; f < clk::FACE_COUNT; f++) {
    for (int v = 0; v < 2; v++) {               // with and without the extras
      clk::Dial d;
      d.face = f;
      d.moon = v == 0;
      d.moonPhase = phases[f];
      d.temps = v == 0;
      strcpy(d.in, "71");
      strcpy(d.out, f == 3 ? "-4" : "48");
      if (v == 0) { strcpy(d.inHum, "55"); strcpy(d.outHum, "74"); strcpy(d.pres, f == 1 ? "29.03 INHG" : "983 HPA"); }
      auto t0 = std::chrono::steady_clock::now();
      clk::renderDial(dial.data(), d);
      auto t1 = std::chrono::steady_clock::now();
      out = dial;
      clk::drawHands(out.data(), f, 10 + 9 / 60.0f, 9 + 36 / 60.0f, 36, true);
      auto t2 = std::chrono::steady_clock::now();
      char p[256];
      snprintf(p, sizeof p, "%s/%s%s.ppm", dir, clk::faceName(f), v ? "-plain" : "");
      writePPM(p, out.data());
      printf("%-10s dial %.1f ms, hands %.1f ms\n", clk::faceName(f),
             std::chrono::duration<double, std::milli>(t1 - t0).count(),
             std::chrono::duration<double, std::milli>(t2 - t1).count());
    }
  }
  return 0;
}
