#pragma once
// Art deco analog clock faces for the 480x480 screen. Plain C++ (no Arduino), so the same code draws on the
// board and in tools/preview (renders PNGs of every face on a PC).
//
// Two steps: renderDial() draws the still parts (face, numerals, moon, temperatures) into a buffer once;
// drawHands() draws the hands on a copy of it every second.
#include <stdint.h>
#include <time.h>

namespace clk {

static const int W = 480, H = 480;
static const int FACE_COUNT = 9;

const char *faceName(int face);          // Gatsby, Skyline, Ivory, Midnight, Pup, Grandfather, Christmas, Advent, Lit Up

struct Dial {
  int face = 0;
  bool moon = false;
  float moonPhase = 0;                   // 0 = new, 0.25 first quarter, 0.5 full, 0.75 last quarter
  bool temps = false;
  char in[8] = "--", out[8] = "--";      // e.g. "71", "-3" or "--"; a degree sign is added after numbers
  char inHum[8] = "", outHum[8] = "";    // e.g. "54" (a % sign is added), "--", or "" = not shown
  char pres[16] = "";                    // e.g. "983 HPA", "29.03 INHG", or "" = no pressure window
  char presWord[12] = "";                // barometer word: "STORMY", "RAIN", "CHANGE", "FAIR", "VERY DRY"
  int presTrend = 0;                     // 1 rising, -1 falling, 0 steady (an arrow after the number)
  int month = 0, mday = 0;               // today (1-12, 1-31): the Advent calendar opens its doors by it
  int daysToXmas = -1;                   // 0 on Christmas Day; the Christmas and Advent faces count down
};

void renderDial(uint16_t *buf, const Dial &d);    // W*H RGB565, every pixel written
// Hands every second (the Advent face shows the time as digits instead).
void drawHands(uint16_t *buf, int face, float hour, float minute, float second, bool showSeconds);

void init();                             // build the font once, before drawing from two tasks at a time

float moonPhase(time_t utc);             // 0..1
const char *moonName(float phase);

}  // namespace clk
