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
static const int FACE_COUNT = 11;

const char *faceName(int face);          // Gatsby, Skyline, Ivory, Midnight, Grandfather, Christmas, Advent,
                                         // Red LED, VFD, Nixie, Flip
bool faceHasSeconds(int face);           // false: the face never shows seconds (Advent)

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
  int month = 0, mday = 0, wday = 0;     // today (1-12, 1-31, 0 = Sunday): Advent opens doors by it, digital shows it
  int daysToXmas = -1;                   // 0 on Christmas Day; the Christmas and Advent faces count down
};

void renderDial(uint16_t *buf, const Dial &d);    // W*H RGB565, every pixel written
// The same in two steps, so the slow part can be kept: renderBase() draws what never changes for a face
// (background, numerals; every pixel written); renderExtras() adds the windows (temperatures, moon, pressure,
// countdown) on a copy of it. baseKey() tells when a kept base no longer fits the dial.
void renderBase(uint16_t *buf, const Dial &d);
void renderExtras(uint16_t *buf, const Dial &d);
void baseKey(const Dial &d, char *out, int n);
// The time over the dial, in two layers. Faces with digits (Advent, Red LED, VFD, Nixie, Flip) draw their big
// digits in the minute layer, which is slow and only changes once a minute; everything else (hands, colon,
// seconds, AM/PM) is the second layer, drawn every tick on a copy of dial + minute layer.
bool faceHasMinuteLayer(int face);
void drawMinuteLayer(uint16_t *buf, int face, int hour24, int minute);
void drawSecondLayer(uint16_t *buf, int face, float hour, float minute, float second, bool showSeconds);
void drawHands(uint16_t *buf, int face, float hour, float minute, float second, bool showSeconds);   // both

void init();                             // build the font once, before drawing from two tasks at a time
void setYield(void (*fn)());             // called now and then during long drawing (lets other tasks run)

float moonPhase(time_t utc);             // 0..1
const char *moonName(float phase);

}  // namespace clk
