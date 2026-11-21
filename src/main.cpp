// Matrixinator: shows an LED matrix on the Guition ESP32-S3-4848S040 touch screen.
//
// It looks like WLED to Signinator: /json/info reports a 2D matrix (32x32 by default), /json/state takes on/bri,
// and DDP on UDP 4048 brings the pixels. Each LED is drawn as a round dot; the backlight follows the brightness.
// Also: Improv Wi-Fi over USB (set Wi-Fi from the Lickety Split flasher), a fallback hotspot, a settings page,
// OTA updates at POST /update (same path as WLED), and tap-to-show song / status.
// "Auto (with clock)": when nothing is sent, an art deco analog clock (clock.cpp) during set hours, with the
// moon phase and inside / outside temperatures from Home Assistant if wanted.
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Update.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <esp32s3/rom/cache.h>
#include <lwip/sockets.h>
#include <JPEGDEC.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include "board.h"
#include "clock.h"
#include "face_previews.h"
static_assert(sizeof FACE_PREVIEWS / sizeof FACE_PREVIEWS[0] == clk::FACE_COUNT, "run tools/preview/make_previews.py");
#include "panel_init.h"
#include "improv.h"

#ifndef FW_VERSION
#define FW_VERSION ""
#endif
static const char *VERSION = sizeof(FW_VERSION) > 1 ? FW_VERSION : "dev";
static const int MAX_W = 240, MAX_H = 240;   // 240 = 2 screen pixels per LED
static const uint32_t LIVE_TIMEOUT_MS = 2500;

// ---------- settings ----------
struct Settings {
  String ssid, pass, name, host;
  int w = 32, h = 32;
  bool unlit = true;        // draw unlit LEDs as dark grey dots
  bool follow = true;       // backlight follows the brightness the sender sets
  int blMax = 255, blMin = 70;
  // clock
  int mode = 1;             // 0 = Auto (dark when nothing's sent), 1 = Auto (with clock), 2 = Clock only
  int face = 0;
  bool secs = true, moon = true;
  int clkOn = 5 * 60, clkOff = 22 * 60 + 30;   // shown from 5:00 to 22:30 (minutes after midnight)
  int clkBri = 200;
  int night = 0;            // outside the clock's hours: 0 = screen off, 1 = the clock, dimmed
  int nightBri = 40;
  int bootSecs = 20;        // power-on: the Matrix rain plays this long before pictures or the clock
  String tz = "PST8PDT,M3.2.0,M11.1.0";
  // Home Assistant temperatures
  bool ha = false;
  String haUrl = "http://homeassistant.local:8123", haToken;
  String haEnt[5];
  bool haSea = true;        // the pressure sensor reads station pressure: correct it with HA's elevation          // entity ids: inside temp, outside temp, inside humidity, outside humidity, pressure
} S;
static Preferences prefs;
// Home Assistant readings, in this order everywhere
enum { HA_IN_T, HA_OUT_T, HA_IN_H, HA_OUT_H, HA_PRES, HA_N };
static const char *const HA_KEYS[HA_N] = {"hain", "haout", "hainh", "haouth", "hapres"};
static const char *const HA_LABELS[HA_N] = {"Inside temperature", "Outside temperature", "Inside humidity",
                                            "Outside humidity", "Air pressure"};
static const char *const HA_HINTS[HA_N] = {"sensor.living_room_temperature", "sensor.outdoor_temperature",
                                           "sensor.living_room_humidity", "sensor.outdoor_humidity",
                                           "sensor.outdoor_pressure"};

static void saveSettings();
static void loadSettings() {
  prefs.begin("mx", true);
  S.ssid = prefs.getString("ssid", "");
  S.pass = prefs.getString("pass", "");
  S.name = prefs.getString("name", "Matrixinator");
  S.host = prefs.getString("host", "matrixinator");
  S.w = constrain(prefs.getInt("w", 32), 1, MAX_W);
  S.h = constrain(prefs.getInt("h", 32), 1, MAX_H);
  S.unlit = prefs.getBool("unlit", true);
  S.follow = prefs.getBool("follow", true);
  S.blMax = constrain(prefs.getInt("blmax", 255), 10, 255);
  S.blMin = constrain(prefs.getInt("blmin", 70), 0, 255);
  S.mode = constrain(prefs.getInt("mode", 1), 0, 2);
  S.face = constrain(prefs.getInt("face", 0), 0, clk::FACE_COUNT - 1);
  S.secs = prefs.getBool("secs", true);
  S.moon = prefs.getBool("moon", true);
  S.clkOn = constrain(prefs.getInt("con", 5 * 60), 0, 1439);
  S.clkOff = constrain(prefs.getInt("coff", 22 * 60 + 30), 0, 1439);
  S.clkBri = constrain(prefs.getInt("cbri", 200), 10, 255);
  S.night = constrain(prefs.getInt("night", 0), 0, 1);
  S.nightBri = constrain(prefs.getInt("nbri", 40), 5, 255);
  S.bootSecs = constrain(prefs.getInt("boot", 20), 0, 120);
  S.tz = prefs.getString("tz", "PST8PDT,M3.2.0,M11.1.0");
  S.ha = prefs.getBool("ha", false);
  S.haUrl = prefs.getString("haurl", "http://homeassistant.local:8123");
  S.haToken = prefs.getString("hatok", "");
  for (int i = 0; i < 5; i++) S.haEnt[i] = prefs.getString(HA_KEYS[i], "");
  S.haSea = prefs.getBool("hasea", true);
  // Face list v2 (holiday faces; Red LED and VFD became one colourable Digital face): move the saved face
  // and the digital colours from the old numbers to the new ones.
  int oldOf[clk::FACE_COUNT];                     // new face -> where its settings were saved before
  for (int f = 0; f < clk::FACE_COUNT; f++) oldOf[f] = f;
  if (prefs.getInt("fv", 1) < 2) {
    static const int MAP[11] = {0, 1, 2, 3, 4, 12, 13, 14, 14, 15, 16};   // old face -> new face
    int old = prefs.getInt("face", 0);
    S.face = old >= 0 && old < 11 ? MAP[old] : 0;
    oldOf[14] = old == 7 ? 7 : 8;                 // Digital: the Red LED's colours if that was the face, else VFD's
    oldOf[15] = 9;
    oldOf[16] = 10;
  }
  for (int f = 0; f < clk::FACE_COUNT; f++) {   // digital faces: colour, glow, unlit level
    if (!clk::defaultDigitColor(f)) continue;
    String k = String(oldOf[f]);
    clk::setDigitColor(f, prefs.getUInt(("dc" + k).c_str(), clk::defaultDigitColor(f)));
    clk::setDigitLevels(f, prefs.getInt(("dg" + k).c_str(), 100), prefs.getInt(("du" + k).c_str(), 100));
  }
  bool migrate = prefs.getInt("fv", 1) < 2;
  prefs.end();
  if (migrate) saveSettings();
}
static void saveSettings() {
  prefs.begin("mx", false);
  prefs.putInt("fv", 2);
  prefs.putString("ssid", S.ssid);
  prefs.putString("pass", S.pass);
  prefs.putString("name", S.name);
  prefs.putString("host", S.host);
  prefs.putInt("w", S.w);
  prefs.putInt("h", S.h);
  prefs.putBool("unlit", S.unlit);
  prefs.putBool("follow", S.follow);
  prefs.putInt("blmax", S.blMax);
  prefs.putInt("blmin", S.blMin);
  prefs.putInt("mode", S.mode);
  prefs.putInt("face", S.face);
  prefs.putBool("secs", S.secs);
  prefs.putBool("moon", S.moon);
  prefs.putInt("con", S.clkOn);
  prefs.putInt("coff", S.clkOff);
  prefs.putInt("cbri", S.clkBri);
  prefs.putInt("night", S.night);
  prefs.putInt("nbri", S.nightBri);
  prefs.putInt("boot", S.bootSecs);
  prefs.putString("tz", S.tz);
  prefs.putBool("ha", S.ha);
  prefs.putString("haurl", S.haUrl);
  prefs.putString("hatok", S.haToken);
  for (int i = 0; i < 5; i++) prefs.putString(HA_KEYS[i], S.haEnt[i]);
  prefs.putBool("hasea", S.haSea);
  for (int f = 0; f < clk::FACE_COUNT; f++) {
    if (!clk::defaultDigitColor(f)) continue;
    prefs.putUInt(("dc" + String(f)).c_str(), clk::digitColor(f));
    prefs.putInt(("dg" + String(f)).c_str(), clk::digitGlowPct(f));
    prefs.putInt(("du" + String(f)).c_str(), clk::digitUnlitPct(f));
  }
  prefs.end();
}

// ---------- display ----------
static Arduino_DataBus *bus;
static Arduino_ESP32RGBPanel *rgbpanel;
static Arduino_RGB_Display *gfx;
static uint16_t *fb;
static bool gfxOk = false;
static uint8_t touchAddr = 0;
static int blDuty = -1;                // brightness level applied to the picture (0-255)
static uint8_t dimLut[256];            // colour level -> dimmed level
static int dimFor = -1;
// What's being shown: the saved matrix size as round "LED" dots, or (sent by Signinator for its "full resolution"
// look) a bigger picture drawn as plain square pixels. Three receive buffers: the receiver task writes one,
// one waits to be drawn, one is being drawn.
static int curW = 32, curH = 32;
static bool pixelLook = false;       // draw square pixels instead of round LED dots
static bool customSize = false;      // size set by the sender (Signinator's full-resolution look), not the saved one
static uint8_t *bufs[3] = {nullptr, nullptr, nullptr}, *shown = nullptr;
static int wIdx = 0, readyIdx = -1, drawIdx = 2;
static SemaphoreHandle_t bufLock;
static volatile uint32_t lastPushMs = 0, rxFrom = 0, rxDropped = 0, rxBad = 0;
static int cell, dot, ox, oy;        // layout of the dots on the screen
static uint8_t *mask;                // cell*cell: 1 = inside the round dot
static int16_t *spanS, *spanL;       // per line of a cell: where the dot starts and how wide it is
static bool forceRedraw = true;
static bool idleShown = false;     // the idle screen (Matrix rain) is showing
static bool asleep = false;        // nothing to show: backlight off until pixels/photos arrive or a tap
static bool clockShown = false;    // the clock is on the screen
static bool clockForce = false;    // redraw the clock now (something drew over it)
static uint32_t idleSince = 0, peekUntil = 0;
static const uint32_t IDLE_SLEEP_MS = 15000;
static uint32_t bootRainUntil = 0;   // power-on: the rain plays until then (0 = done)
static bool bootRain() { if (bootRainUntil && (int32_t)(millis() - bootRainUntil) >= 0) bootRainUntil = 0; return bootRainUntil; }
static String npTitle, npArtist;        // the song (from Signinator), shown on a tap
static void goDark();
static void wakeAfterDraw(bool wasAsleep);
static void mrStep();
static int overlayTop = TFT_H;       // rows from here down belong to the overlay while it shows

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

static void layout() {
  cell = min(TFT_W / curW, TFT_H / curH);
  if (cell < 1) cell = 1;
  // dots at every size: a dark gap between LEDs (1 pixel when they're small)
  int gap = pixelLook ? 0 : cell >= 6 ? max(1, cell / 7) : cell >= 2 ? 1 : 0;
  dot = cell - gap;
  ox = (TFT_W - curW * cell) / 2;
  oy = (TFT_H - curH * cell) / 2;
  free(mask);
  mask = (uint8_t *)malloc(cell * cell);
  float r = dot / 2.0f, c = (cell - 1) / 2.0f;
  for (int y = 0; y < cell; y++)
    for (int x = 0; x < cell; x++) {
      float dx = x - c, dy = y - c;
      mask[y * cell + x] = pixelLook ? 1 : (dot >= 4) ? (dx * dx + dy * dy <= r * r) : (x < dot && y < dot);
    }
  free(spanS); free(spanL);
  spanS = (int16_t *)malloc(cell * sizeof(int16_t));
  spanL = (int16_t *)malloc(cell * sizeof(int16_t));
  for (int y = 0; y < cell; y++) {         // a round dot is one run of pixels per line
    int a = -1, b = -1;
    for (int x = 0; x < cell; x++) if (mask[y * cell + x]) { if (a < 0) a = x; b = x; }
    spanS[y] = a < 0 ? 0 : a;
    spanL[y] = a < 0 ? 0 : b - a + 1;
  }
  size_t n = (size_t)MAX_W * MAX_H * 3;   // allocated once at the largest size
  for (auto &b : bufs) if (!b) b = (uint8_t *)ps_calloc(n, 1);
  if (!shown) shown = (uint8_t *)ps_calloc(n, 1);
  gfx->fillScreen(BLACK);
  forceRedraw = true;
  clockForce = true;
}

// Draw the LEDs that changed. Returns quickly when nothing did.
static uint16_t lineBuf[TFT_W];        // one screen line (internal RAM), copied down for each row of a pixel

static void render(const uint8_t *rx) {
  int yMin = TFT_H, yMax = -1;
  {                                      // one row of LEDs at a time: build each screen line, then copy it
    const size_t rowBytes = (size_t)curW * 3;
    const size_t n = (size_t)curW * cell * 2;
    static uint16_t colors[MAX_W];
    for (int ly = 0; ly < curH; ly++) {
      int py = oy + ly * cell;
      if (py + cell > overlayTop) break;
      const uint8_t *src = rx + ly * rowBytes;
      uint8_t *old = shown + ly * rowBytes;
      if (!forceRedraw && memcmp(src, old, rowBytes) == 0) continue;
      memcpy(old, src, rowBytes);
      for (int lx = 0; lx < curW; lx++) {
        const uint8_t *p = src + lx * 3;
        colors[lx] = (!pixelLook && S.unlit && p[0] < 10 && p[1] < 10 && p[2] < 10)
                         ? rgb565(24, 24, 24) : rgb565(dimLut[p[0]], dimLut[p[1]], dimLut[p[2]]);
      }
      if (pixelLook) {                     // squares: build one line, copy it down
        uint16_t *ln = lineBuf + ox;
        for (int lx = 0; lx < curW; lx++) { uint16_t c = colors[lx]; for (int k = 0; k < cell; k++) *ln++ = c; }
        for (int y = 0; y < cell; y++) memcpy(fb + (py + y) * TFT_W + ox, lineBuf + ox, n);
      } else if (cell < 12) {              // small dots: build each line (dots + gaps) and copy it whole
        for (int y = 0; y < cell; y++) {
          const uint8_t *m = mask + y * cell;
          uint16_t *ln = lineBuf + ox;
          for (int lx = 0; lx < curW; lx++) { uint16_t c = colors[lx]; for (int k = 0; k < cell; k++) *ln++ = m[k] ? c : BLACK; }
          memcpy(fb + (py + y) * TFT_W + ox, lineBuf + ox, n);
        }
      } else {                             // big dots: write only the dot's run on each line (gaps stay black)
        for (int y = 0; y < cell; y++) {
          int len = spanL[y];
          if (!len) continue;
          uint16_t *line = fb + (py + y) * TFT_W + ox + spanS[y];
          for (int lx = 0; lx < curW; lx++) {
            uint16_t c = colors[lx];
            uint16_t *d = line + lx * cell;
            for (int k = 0; k < len; k++) d[k] = c;
          }
        }
      }
      yMin = min(yMin, py);
      yMax = max(yMax, py + cell - 1);
    }
    forceRedraw = false;
    if (yMax >= yMin) Cache_WriteBack_Addr((uint32_t)(fb + yMin * TFT_W), (yMax - yMin + 1) * TFT_W * 2);
    return;
  }

}

// ---------- backlight ----------
static bool on = true;
static int bri = 128;                 // last brightness the sender asked for (WLED "bri")
// This board's backlight doesn't follow PWM dimming (it stays dark), so the backlight is simply on or off and the
// brightness is applied to the picture itself (square-root curve: a dim LED sign still gives a readable screen).
static void backlight() {
  int v = 0;
  if (on) v = S.follow ? S.blMin + (int)((S.blMax - S.blMin) * sqrtf(bri / 255.0f)) : S.blMax;
  blDuty = constrain(v, 0, 255);
  // the clock keeps its own light: Signinator's on/off is about its pictures
  digitalWrite(TFT_BCKL, ((clockShown || (on && blDuty > 0)) && !asleep) ? HIGH : LOW);
  if (blDuty != dimFor) {
    for (int i = 0; i < 256; i++) dimLut[i] = (uint8_t)((i * blDuty + 127) / 255);
    dimFor = blDuty;
    forceRedraw = true;          // redraw every LED at the new level
  }
}

// ---------- network ----------
static WebServer web(80);
static DNSServer dns;
static bool apOn = false;
static uint32_t wifiLostAt = 0;
static String apName;

static String url() { return WiFi.status() == WL_CONNECTED ? "http://" + WiFi.localIP().toString() + "/" : ""; }

static bool joinWifi(const String &ssid, const String &pass, bool save) {
  WiFi.mode(apOn ? WIFI_AP_STA : WIFI_STA);
  WiFi.setHostname(S.host.c_str());
  WiFi.begin(ssid.c_str(), pass.c_str());
  uint32_t t0 = millis();
  while (millis() - t0 < 15000 && WiFi.status() != WL_CONNECTED) {
    if (idleShown) mrStep();
    delay(10);
  }
  if (WiFi.status() != WL_CONNECTED) return false;
  if (save) { S.ssid = ssid; S.pass = pass; saveSettings(); }
  return true;
}

static void startAp() {
  if (apOn) return;
  apOn = true;
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(apName.c_str());
  dns.start(53, "*", WiFi.softAPIP());   // captive portal: every name -> this page
}
static void stopAp() {
  if (!apOn) return;
  dns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  apOn = false;
}

// ---------- live data ----------
static uint32_t lastFrameAt = 0, frames = 0, fpsAt = 0;
static int fps = 0;
static IPAddress liveFrom;
static bool photoShown = false;    // a photo (album art etc.) is on the screen, not LED dots

// The pixels arrive on their own task (core 0) so no packet is lost while the main loop draws or serves pages.
static void rxTask(void *) {
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in a = {};
  a.sin_family = AF_INET;
  a.sin_port = htons(4048);
  a.sin_addr.s_addr = htonl(INADDR_ANY);
  bind(sock, (sockaddr *)&a, sizeof a);
  int rcv = 64 * 1024;
  setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcv, sizeof rcv);
  static uint8_t pkt[1500];
  // A frame is shown only if every byte of it arrived and it fits the current size. Otherwise (a lost
  // packet, or frames of another size right after a look change) it's dropped and the last good frame
  // stays: no mix of new and leftover pixels.
  uint32_t got = 0;
  bool bad = false;
  for (;;) {
    sockaddr_in from;
    socklen_t fl = sizeof from;
    int n = recvfrom(sock, pkt, sizeof pkt, 0, (sockaddr *)&from, &fl);
    if (n < 10 || (pkt[0] & 0xC0) != 0x40) continue;          // DDP version 1 only
    int hdr = (pkt[0] & 0x10) ? 14 : 10;                        // timecode adds 4 bytes
    uint32_t off = ((uint32_t)pkt[4] << 24) | (pkt[5] << 16) | (pkt[6] << 8) | pkt[7];
    uint32_t len = (pkt[8] << 8) | pkt[9];
    if (pkt[3] == 251 || pkt[3] == 254 || hdr + (int)len > n) continue;  // config/status queries: ignore
    xSemaphoreTake(bufLock, portMAX_DELAY);
    uint32_t total = (uint32_t)curW * curH * 3;
    if (off + len > total) bad = true;                           // another size: not for this look
    else { memcpy(bufs[wIdx] + off, pkt + hdr, len); got += len; }
    if (pkt[0] & 0x01) {                                         // PUSH: end of a frame
      if (!bad && got == total) {
        if (readyIdx >= 0) { int t = readyIdx; readyIdx = wIdx; wIdx = t; rxDropped++; }   // replace an undrawn frame
        else { readyIdx = wIdx; wIdx = 3 - readyIdx - drawIdx; }
        lastPushMs = millis();
        rxFrom = from.sin_addr.s_addr;
      } else {
        rxBad++;                                                 // incomplete or wrong size: skip it
      }
      got = 0;
      bad = false;
    }
    xSemaphoreGive(bufLock);
  }
}

// Change what's being shown (size and look). Not saved: Signinator sets it again after a restart.
static void setLook(int w, int h, bool squares, bool custom) {
  w = constrain(w, 1, MAX_W);
  h = constrain(h, 1, MAX_H);
  customSize = custom;
  if (w == curW && h == curH && squares == pixelLook) return;
  xSemaphoreTake(bufLock, portMAX_DELAY);
  curW = w; curH = h; pixelLook = squares;
  for (auto &b : bufs) memset(b, 0, (size_t)MAX_W * MAX_H * 3);
  readyIdx = -1;
  xSemaphoreGive(bufLock);
  layout();
}

static void drawNewFrame() {
  int idx = -1;
  xSemaphoreTake(bufLock, portMAX_DELAY);
  if (readyIdx >= 0) { drawIdx = readyIdx; readyIdx = -1; idx = drawIdx; }
  xSemaphoreGive(bufLock);
  if (idx < 0) return;
  lastFrameAt = millis();
  liveFrom = IPAddress(rxFrom);
  frames++;
  if (!on || S.mode == 2 || bootRain()) return;   // off, "Clock only" or the power-on rain: not drawn
  bool wasAsleep = asleep;
  idleSince = 0; peekUntil = 0;
  if (idleShown || photoShown || clockShown) {
    gfx->fillScreen(BLACK); forceRedraw = true; idleShown = false; photoShown = false; clockShown = false;
  }
  render(bufs[idx]);
  wakeAfterDraw(wasAsleep);                // the picture first, then the light
}

static bool live() { return lastFrameAt && millis() - lastFrameAt < LIVE_TIMEOUT_MS; }

// ---------- photos (album art, GIFs, pictures from Signinator: full resolution, no dots) ----------
// Two ways in:
//  * TCP port 4049 (preferred): a connection that stays open. Each frame is a 12-byte header
//    "MXF1", flags (bit 0: draw at 2x), 3 reserved bytes, uint32 little-endian length, then a JPEG.
//    A task receives it; the main loop draws it; then one byte 'K' goes back. The sender waits for the
//    'K' before the next frame, so it sends exactly as fast as the screen can draw.
//  * POST /api/photo (JPEG as a multipart file upload), for single pictures.
// Photos are drawn centered (2x for small frames, e.g. 240x240 GIF frames) and dimmed like the rest.
// LED pixels arriving afterwards clear the photo and go back to dots.
static const size_t PHOTO_MAX = 400 * 1024;
static const uint16_t PHOTO_PORT = 4049;
static uint8_t *photoBuf = nullptr;          // HTTP uploads
static size_t photoLen = 0;
static bool photoOk = false;
static uint8_t *streamBuf = nullptr;         // TCP frames
static volatile size_t streamLen = 0;
static volatile uint8_t streamFlags = 0;
static volatile bool streamPending = false;
static SemaphoreHandle_t streamDrawn;
static JPEGDEC jpeg;
static uint8_t dim5[32], dim6[64];
static int dimTabFor = -1;
static int photoScale = 1, photoOx = 0, photoOy = 0;

static inline uint16_t dim565(uint16_t c) {
  return (dim5[c >> 11] << 11) | (dim6[(c >> 5) & 63] << 5) | dim5[c & 31];
}

static int photoDraw(JPEGDRAW *d) {
  if (dimTabFor != blDuty) {
    for (int i = 0; i < 32; i++) dim5[i] = (i * blDuty + 127) / 255;
    for (int i = 0; i < 64; i++) dim6[i] = (i * blDuty + 127) / 255;
    dimTabFor = blDuty;
  }
  const bool dimIt = blDuty < 255;
  const int sc = photoScale;
  for (int y = 0; y < d->iHeight; y++) {
    const uint16_t *src = d->pPixels + y * d->iWidth;
    int x0 = photoOx + d->x * sc;
    int w = min(d->iWidthUsed, (TFT_W - x0) / sc);
    if (w <= 0) continue;
    int sy = photoOy + (d->y + y) * sc;
    if (sy < 0 || sy + sc > TFT_H || sy + sc > overlayTop) continue;
    uint16_t *dst = fb + sy * TFT_W + x0;
    if (sc == 1) {
      if (!dimIt) { memcpy(dst, src, w * 2); continue; }
      for (int x = 0; x < w; x++) dst[x] = dim565(src[x]);
      continue;
    }
    for (int x = 0; x < w; x++) {               // 2x: each pixel twice across, the line twice down
      uint16_t c = dimIt ? dim565(src[x]) : src[x];
      dst[2 * x] = c; dst[2 * x + 1] = c;
    }
    memcpy(dst + TFT_W, dst, w * 2 * 2);
  }
  return 1;
}

static void showPhoto(uint8_t *data, size_t len, int scale) {
  if (!data || !len) return;
  if (bootRain()) { lastFrameAt = millis(); frames++; return; }   // the power-on rain is still playing
  if (!on || S.mode == 2) return;
  if (!jpeg.openRAM(data, len, photoDraw)) return;
  jpeg.setPixelType(RGB565_LITTLE_ENDIAN);
  int w = jpeg.getWidth(), h = jpeg.getHeight();
  int opt = 0;
  photoScale = (scale == 2 && w * 2 <= TFT_W && h * 2 <= TFT_H) ? 2 : 1;
  if (photoScale == 1) {                       // bigger than the screen: let the decoder scale down
    if (w > TFT_W * 4 || h > TFT_H * 4) { opt = JPEG_SCALE_EIGHTH; w /= 8; h /= 8; }
    else if (w > TFT_W * 2 || h > TFT_H * 2) { opt = JPEG_SCALE_QUARTER; w /= 4; h /= 4; }
    else if (w > TFT_W || h > TFT_H) { opt = JPEG_SCALE_HALF; w /= 2; h /= 2; }
  }
  photoOx = max(0, (TFT_W - w * photoScale) / 2);
  photoOy = max(0, (TFT_H - h * photoScale) / 2);
  bool wasAsleep = asleep;
  idleSince = 0; peekUntil = 0;
  if (!photoShown || idleShown || clockShown) { gfx->fillScreen(BLACK); idleShown = false; clockShown = false; }
  jpeg.decode(0, 0, opt);
  jpeg.close();
  Cache_WriteBack_Addr((uint32_t)fb, TFT_W * TFT_H * 2);
  wakeAfterDraw(wasAsleep);                          // the photo first, then the light
  photoShown = true;
  lastFrameAt = millis();
  frames++;
}

static bool readAll(int sock, uint8_t *p, size_t n) {
  while (n) {
    int r = recv(sock, p, n, 0);
    if (r <= 0) return false;
    p += r; n -= r;
  }
  return true;
}

static void photoTask(void *) {
  int ls = socket(AF_INET, SOCK_STREAM, 0);
  int one = 1;
  setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  sockaddr_in a = {};
  a.sin_family = AF_INET;
  a.sin_port = htons(PHOTO_PORT);
  a.sin_addr.s_addr = htonl(INADDR_ANY);
  bind(ls, (sockaddr *)&a, sizeof a);
  listen(ls, 1);
  for (;;) {
    sockaddr_in from;
    socklen_t fl = sizeof from;
    int c = accept(ls, (sockaddr *)&from, &fl);
    if (c < 0) { delay(100); continue; }
    timeval tv = {30, 0};                      // a sender that goes quiet for 30 s is dropped
    setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    uint8_t h[12];
    while (readAll(c, h, sizeof h)) {
      if (memcmp(h, "MXF1", 4) != 0) break;
      uint32_t len = h[8] | (h[9] << 8) | (h[10] << 16) | ((uint32_t)h[11] << 24);
      if (len == 0 || len > PHOTO_MAX || !streamBuf) break;
      if (!readAll(c, streamBuf, len)) break;
      streamLen = len;
      streamFlags = h[4];
      rxFrom = from.sin_addr.s_addr;
      xSemaphoreTake(streamDrawn, 0);          // clear a stale signal
      streamPending = true;
      if (xSemaphoreTake(streamDrawn, pdMS_TO_TICKS(3000)) != pdTRUE) streamPending = false;
      if (send(c, "K", 1, 0) != 1) break;
    }
    close(c);
  }
}

static void drawStreamPhoto() {
  if (!streamPending) return;
  liveFrom = IPAddress(rxFrom);
  showPhoto(streamBuf, streamLen, (streamFlags & 1) ? 2 : 1);
  streamPending = false;
  xSemaphoreGive(streamDrawn);
}

static void centered(const String &s, int y, int size, uint16_t color) {
  gfx->setTextSize(size);
  gfx->setTextColor(color);
  int w = s.length() * 6 * size;
  gfx->setCursor(max(0, (TFT_W - w) / 2), y);
  gfx->print(s);
}

// ---------- Matrix rain: boot, idle (connecting / IP address) and firmware updates ----------
// Full-screen falling green characters: a bright head per column, a trail fading from bright to dark green,
// characters flickering as they change. A black panel in the middle shows the name and the status; the
// rain never draws over it. Only cells whose colour changes are redrawn, so it's cheap enough to run
// while connecting to Wi-Fi and while an update is written.
static const int MR_CW = 12, MR_CH = 16, MR_COLS = TFT_W / MR_CW, MR_ROWS = TFT_H / MR_CH;   // 40 x 30
static uint8_t mrChar[MR_COLS * MR_ROWS];
static float mrHead[MR_COLS], mrSpeed[MR_COLS];
static uint8_t mrLen[MR_COLS];
static int mrLastRow[MR_COLS];
static int boxX0 = 0, boxY0 = 0, boxX1 = 0, boxY1 = 0;      // the panel; no rain in here
static uint32_t mrAt = 0;
static String panelKey;
static volatile bool updating = false;
static size_t upTotal = 0, upDone = 0;
static const char MR_SET[] = "0123456789ABCDEFXZ:=+*<>|";

static uint8_t mrRandChar() {
  const int a = sizeof(MR_SET) - 1, n = a + 16;              // plus 16 Greek-looking glyphs (cp437 0xE0-0xEF)
  int r = esp_random() % n;
  return r < a ? MR_SET[r] : 0xE0 + (r - a);
}
static bool mrInBox(int c, int r) {
  int x = c * MR_CW, y = r * MR_CH;
  return x + MR_CW > boxX0 && x < boxX1 && y + MR_CH > boxY0 && y < boxY1;
}
static uint16_t mrColor(int d, int len) {
  if (d == 0) return rgb565(200, 255, 200);                  // the head
  if (d <= 2) return rgb565(60, 255, 90);
  if (d < len / 2) return rgb565(0, 190, 50);
  if (d < len) return rgb565(0, 95, 25);
  return BLACK;
}
static void mrCell(int c, int r, uint16_t color, bool newChar) {
  if (r < 0 || r >= MR_ROWS || mrInBox(c, r)) return;
  uint8_t &ch = mrChar[r * MR_COLS + c];
  if (newChar || !ch) ch = mrRandChar();
  gfx->setTextSize(2);
  gfx->drawChar(c * MR_CW, r * MR_CH, ch, color, BLACK);
}
static void mrReset(int c, bool anywhere) {
  mrHead[c] = anywhere ? -(float)(esp_random() % MR_ROWS) : -(float)(esp_random() % 12);
  mrSpeed[c] = 0.25f + (esp_random() % 60) / 100.0f;
  mrLen[c] = 8 + esp_random() % 16;
  mrLastRow[c] = -1000;
}
static void mrInit() {
  memset(mrChar, 0, sizeof mrChar);
  for (int c = 0; c < MR_COLS; c++) mrReset(c, true);
  boxX0 = boxY0 = boxX1 = boxY1 = 0;
  panelKey = "";
  mrAt = 0;
}
static void mrStep() {                                        // about 25 frames a second
  if (millis() - mrAt < 40) return;
  mrAt = millis();
  for (int c = 0; c < MR_COLS; c++) {
    mrHead[c] += mrSpeed[c];
    int hr = (int)floorf(mrHead[c]), len = mrLen[c];
    if (hr == mrLastRow[c]) continue;
    mrLastRow[c] = hr;
    // only cells whose colour changes when the head moves down one row
    const int ds[5] = {0, 1, 3, len / 2, len};
    for (int k = 0; k < 5; k++) mrCell(c, hr - ds[k], mrColor(ds[k], len), k == 0);
    if (hr - len > MR_ROWS) mrReset(c, false);
  }
  for (int k = 0; k < 6; k++) {                              // flicker: a few lit characters change
    int c = esp_random() % MR_COLS, r = esp_random() % MR_ROWS;
    int d = (int)floorf(mrHead[c]) - r;
    if (d > 0 && d < mrLen[c]) mrCell(c, r, mrColor(d, mrLen[c]), true);
  }
}
static void mrPanel(const String &title, uint16_t titleColor, const String &l2, uint16_t c2, const String &l3) {
  boxX0 = 36; boxX1 = TFT_W - 36; boxY0 = 168; boxY1 = 318;
  gfx->fillRect(boxX0, boxY0, boxX1 - boxX0, boxY1 - boxY0, BLACK);
  gfx->drawRect(boxX0 + 3, boxY0 + 3, boxX1 - boxX0 - 6, boxY1 - boxY0 - 6, rgb565(0, 90, 25));
  centered(title, boxY0 + 22, 4, titleColor);
  if (l2.length()) centered(l2, boxY0 + 74, 2, c2);
  if (l3.length()) centered(l3, boxY0 + 112, 1, rgb565(0, 150, 45));
}

static void updateProgress() {
  const int bw = 320, bx = (TFT_W - bw) / 2, by = boxY0 + 78, bh = 12;
  gfx->drawRect(bx - 2, by - 2, bw + 4, bh + 4, rgb565(0, 120, 35));
  int fill = upTotal ? (int)((uint64_t)bw * min(upDone, upTotal) / upTotal) : (int)((millis() / 8) % bw);
  gfx->fillRect(bx, by, fill, bh, rgb565(60, 255, 90));
  if (fill < bw) gfx->fillRect(bx + fill, by, bw - fill, bh, BLACK);
}
static uint32_t upBarAt = 0;
// A still rain picture for the firmware update screen (an animation stutters while flash is written).
static void mrStill() {
  for (int c = 0; c < MR_COLS; c++) {
    int head = esp_random() % (MR_ROWS + 10), len = mrLen[c];
    for (int d = 0; d < len; d++) mrCell(c, head - d, mrColor(d, len), true);
  }
}
static void updateAnimate() {           // called while the update is written: only the bar moves
  if (millis() - upBarAt >= 250) { upBarAt = millis(); updateProgress(); }
}

// The idle screen: the rain, with the name and what's happening in the panel. Called often; only redraws
// the panel when its text changes (and everything when coming from LED pixels or a photo).
// Nothing to show: dark (backlight off, black picture behind it, so waking never flashes old content).
static void goDark() {
  gfx->fillScreen(BLACK);
  idleShown = false; photoShown = false; clockShown = false; forceRedraw = true;
  idleSince = 0; peekUntil = 0;
  asleep = true;
  backlight();
}
// Draw first, light up after: a wake-up shows the new picture straight away.
static void wakeAfterDraw(bool wasAsleep) {
  if (wasAsleep) { asleep = false; backlight(); }
}

static void showIdle() {
  if (!idleShown) { gfx->fillScreen(BLACK); mrInit(); idleShown = true; }
  String l2, l3;
  uint16_t c2 = rgb565(60, 255, 90);
  bool peek = peekUntil && millis() < peekUntil;
  if (apOn && WiFi.status() != WL_CONNECTED) {            // setting up: the instructions are needed
    l2 = "Join Wi-Fi " + apName;
    c2 = rgb565(255, 220, 80);
    l3 = "then open http://192.168.4.1";
  } else if (peek) {                                       // a tap: the details
    l2 = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("No Wi-Fi yet");
    l3 = npTitle.length() ? (npTitle + (npArtist.length() ? " - " + npArtist : "")).substring(0, 60)
                          : String("Matrixinator v") + VERSION;
  }
  String key = l2 + "|" + l3;
  if (key == panelKey) return;
  panelKey = key;
  mrPanel("MATRIXINATOR", rgb565(60, 255, 90), l2, c2, l3);
}

// ---------- overlay (tap the screen) ----------
static uint32_t overlayUntil = 0;
static const int OVERLAY_H = 110;

static void showOverlay() {
  overlayTop = TFT_H - OVERLAY_H;
  gfx->fillRect(0, overlayTop, TFT_W, OVERLAY_H, rgb565(10, 10, 16));
  gfx->drawFastHLine(0, overlayTop, TFT_W, DARKGREY);
  if (npTitle.length()) {
    centered(npTitle.substring(0, 38), overlayTop + 14, 2, WHITE);
    centered(npArtist.substring(0, 38), overlayTop + 40, 2, LIGHTGREY);
  } else {
    centered(S.name, overlayTop + 22, 2, WHITE);
  }
  String st = WiFi.localIP().toString() + (live() ? "  |  " + String(fps) + " fps from " + liveFrom.toString() : "  |  idle");
  centered(st, overlayTop + 76, 1, DARKGREY);
  centered(String("Matrixinator v") + VERSION, overlayTop + 92, 1, DARKGREY);
  overlayUntil = millis() + 6000;
}
static void hideOverlay() {
  overlayUntil = 0;
  overlayTop = TFT_H;
  gfx->fillRect(0, TFT_H - OVERLAY_H, TFT_W, OVERLAY_H, BLACK);
  forceRedraw = true;
  if (clockShown) clockForce = true;
  else if (photoShown && live()) { gfx->fillRect(0, TFT_H - OVERLAY_H, TFT_W, OVERLAY_H, BLACK); }   // the next photo fills it
  else if (live()) render(bufs[drawIdx]);
  else if (idleShown) { /* boot / setup screen: the rain carries on */ }
  else goDark();
}

// ---------- clock ("Auto (with clock)" / "Clock only") ----------
// The still dial (face, numerals, moon, temperatures) is drawn once into dialBuf and again only when something on
// it changes. Each second: copy it, add the hands, copy that to the screen at the clock's brightness.
static uint16_t *dialBuf = nullptr, *clkBuf = nullptr;
static String dialKey;
// A new dial is drawn by its own task into dialBack (it takes a few seconds on this chip), then swapped in;
// meanwhile the hands keep going on the old one.
static uint16_t *dialBack = nullptr;
static uint16_t *baseBuf = nullptr;          // the face's still parts, kept: a new dial is a copy plus the windows
static char baseKeyNow[48] = "";
static clk::Dial dialReq;
static String dialReqKey;
static int dialReqH = 0, dialReqM = 0;      // the minute drawn in (faces with digits)
static volatile bool dialBusy = false, dialReady = false;
static int dialFace = -1;                    // the face dialBuf holds (-1: none yet)
static TaskHandle_t dialTaskH = nullptr;
static volatile uint32_t dialRenderMs = 0;
static void dialTask(void *) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    uint32_t t0 = millis();
    char k[48];
    clk::baseKey(dialReq, k, sizeof k);
    if (strcmp(k, baseKeyNow) != 0) {          // another face (or day, for Advent): the slow part
      clk::renderBase(baseBuf, dialReq);
      strlcpy(baseKeyNow, k, sizeof baseKeyNow);
    }
    memcpy(dialBack, baseBuf, (size_t)TFT_W * TFT_H * 2);
    clk::renderExtras(dialBack, dialReq);
    if (clk::faceHasMinuteLayer(dialReq.face)) clk::drawMinuteLayer(dialBack, dialReq.face, dialReqH, dialReqM);
    dialRenderMs = millis() - t0;
    dialReady = true;
  }
}
static int32_t clockStamp = -1;
static uint32_t clockPeekUntil = 0;          // a tap on the dark screen: the clock for a while, whatever the hours
static const uint32_t CLOCK_PEEK_MS = 30000;

// Home Assistant (its own task, so a slow reply never holds up the screen)
static SemaphoreHandle_t haLock;
static String haTxt[HA_N], haErr;        // ready to show: "71", "54", "983 HPA"
static String haPresWord;                // barometer word for the sea-level pressure
static int haPresTrend = 0;              // over the last 3 hours: 1 rising, -1 falling
static float haPresHpa = NAN, haPresSea = NAN, haElevM = NAN;
static const char *barometerWord(float seaHpa) {     // the words on an old aneroid barometer
  return seaHpa < 973 ? "STORMY" : seaHpa < 990 ? "RAIN" : seaHpa < 1007 ? "CHANGE" : seaHpa < 1024 ? "FAIR" : "VERY DRY";
}
static uint32_t haAt[HA_N] = {}, haTriedAt = 0;   // when each was last read (0 = never)
static volatile bool haKick = true;
static const uint32_t HA_EVERY_MS = 60000, HA_STALE_MS = 30 * 60000;

static bool timeOk() { return time(nullptr) > 1700000000; }
static void applyTz() { configTzTime(S.tz.c_str(), "pool.ntp.org", "time.google.com", "time.cloudflare.com"); }
static String hhmm(int m) { char b[6]; snprintf(b, sizeof b, "%02d:%02d", m / 60, m % 60); return b; }
static int parseHhmm(const String &s, int dflt) {
  int c = s.indexOf(':');
  if (c < 1) return dflt;
  int h = s.substring(0, c).toInt(), m = s.substring(c + 1).toInt();
  return (h >= 0 && h < 24 && m >= 0 && m < 60) ? h * 60 + m : dflt;
}
static bool inSchedule() {
  time_t now = time(nullptr);
  tm t;
  localtime_r(&now, &t);
  int m = t.tm_hour * 60 + t.tm_min;
  if (S.clkOn == S.clkOff) return true;                                   // same time: all day
  return S.clkOn < S.clkOff ? (m >= S.clkOn && m < S.clkOff) : (m >= S.clkOn || m < S.clkOff);
}
static bool clockWanted() {
  if (S.mode == 0 || !timeOk()) return false;
  if (clockPeekUntil || S.night == 1) return true;   // night "Dim": the clock stays, darker
  return inSchedule();
}
// Daytime brightness, or the night one outside the hours (a tap shows it at daytime brightness).
static int clockBri() { return (clockPeekUntil || inSchedule()) ? S.clkBri : S.nightBri; }

// Whole days from today to the next Christmas Day (0 on the day).
static int daysToChristmas(const tm &now) {
  tm a = {}, b = {};
  a.tm_year = now.tm_year; a.tm_mon = now.tm_mon; a.tm_mday = now.tm_mday; a.tm_hour = 12;
  b.tm_year = now.tm_year + ((now.tm_mon == 11 && now.tm_mday > 25) ? 1 : 0); b.tm_mon = 11; b.tm_mday = 25; b.tm_hour = 12;
  a.tm_isdst = b.tm_isdst = -1;
  return (int)lround(difftime(mktime(&b), mktime(&a)) / 86400.0);
}

// What's on the dial right now (for the screen, or the preview on the settings page). `key` changes whenever
// the dial would look different.
static clk::Dial makeDial(time_t now, const tm &t, int face, String *key) {
  clk::Dial d;
  d.face = face;
  d.moon = S.moon;
  int moonStep = (int)(clk::moonPhase(now) * 118);                 // the moon is redrawn about every 6 hours
  d.moonPhase = (moonStep + 0.5f) / 118;
  d.temps = S.ha;
  d.year = t.tm_year + 1900;
  d.month = t.tm_mon + 1;
  d.mday = t.tm_mday;
  d.wday = t.tm_wday;
  d.daysToXmas = daysToChristmas(t);
  if (S.ha) {
    xSemaphoreTake(haLock, portMAX_DELAY);
    uint32_t ms = millis();
    String v[HA_N];
    for (int i = 0; i < HA_N; i++)
      v[i] = !S.haEnt[i].length() ? String("") : haAt[i] && ms - haAt[i] < HA_STALE_MS ? haTxt[i] : String("--");
    xSemaphoreGive(haLock);
    strlcpy(d.in, v[HA_IN_T].length() ? v[HA_IN_T].c_str() : "--", sizeof d.in);
    strlcpy(d.out, v[HA_OUT_T].length() ? v[HA_OUT_T].c_str() : "--", sizeof d.out);
    strlcpy(d.inHum, v[HA_IN_H].c_str(), sizeof d.inHum);
    strlcpy(d.outHum, v[HA_OUT_H].c_str(), sizeof d.outHum);
    strlcpy(d.pres, v[HA_PRES].c_str(), sizeof d.pres);
    if (v[HA_PRES].length() && v[HA_PRES] != "--") {
      xSemaphoreTake(haLock, portMAX_DELAY);
      strlcpy(d.presWord, haPresWord.c_str(), sizeof d.presWord);
      d.presTrend = haPresTrend;
      xSemaphoreGive(haLock);
    }
    if (!d.inHum[0] && d.outHum[0]) strlcpy(d.inHum, "--", sizeof d.inHum);   // both windows the same height
    if (!d.outHum[0] && d.inHum[0]) strlcpy(d.outHum, "--", sizeof d.outHum);
  }
  if (key)
    *key = String(d.face) + "|" + d.moon + "|" + moonStep + "|" + d.temps + "|" + d.in + "|" + d.out + "|" + d.inHum +
           "|" + d.outHum + "|" + d.pres + "|" + d.presWord + d.presTrend + "|" + d.month + "/" + d.mday + "|" +
           String(clk::digitColor(face), HEX) + "/" + clk::digitGlowPct(face) + "/" + clk::digitUnlitPct(face);
  return d;
}
static void handTimes(const tm &t, float &h, float &m) {
  h = t.tm_hour + t.tm_min / 60.0f + t.tm_sec / 3600.0f;
  m = t.tm_min + t.tm_sec / 60.0f;
}

static void clockTick() {
  if (!dialBuf || !clkBuf) return;
  time_t now = time(nullptr);
  tm t;
  localtime_r(&now, &t);
  String key;
  clk::Dial d = makeDial(now, t, S.face, &key);
  // Faces with digits have the minute drawn into the dial (it's slow), so the dial changes every minute; from
  // :45 the next minute's is drawn ahead, and swapped in when the minute turns.
  const bool ml = clk::faceHasMinuteLayer(S.face);
  if (ml) key += "|" + hhmm(t.tm_hour * 60 + t.tm_min);
  String nextKey;
  clk::Dial dn;
  tm tn;
  if (ml && t.tm_sec >= 45) {
    time_t nx = now + 60;
    localtime_r(&nx, &tn);
    dn = makeDial(nx, tn, S.face, &nextKey);
    nextKey += "|" + hhmm(tn.tm_hour * 60 + tn.tm_min);
  }
  if (dialReady) {
    if (dialReqKey == key) {                           // a new dial is done and it's time for it: swap it in
      std::swap(dialBuf, dialBack);
      dialKey = dialReqKey;
      dialFace = dialReq.face;
      dialReady = false;
      dialBusy = false;
      clockForce = true;
    } else if (dialReqKey != nextKey) {                // out of date (settings changed meanwhile): drop it
      dialReady = false;
      dialBusy = false;
    }
  }
  if (!dialBusy && dialTaskH) {
    bool now_ = key != dialKey, ahead = !now_ && nextKey.length() && nextKey != dialReqKey;
    if (now_ || ahead) {
      dialReq = now_ ? d : dn;
      dialReqKey = now_ ? key : nextKey;
      dialReqH = now_ ? t.tm_hour : tn.tm_hour;
      dialReqM = now_ ? t.tm_min : tn.tm_min;
      dialBusy = true;
      xTaskNotifyGive(dialTaskH);
    }
  }
  if (dialFace < 0) return;                            // the first dial isn't ready yet
  const int face = dialFace;                                     // hands to match the dial on show
  const bool secs = S.secs && clk::faceHasSeconds(face);         // Advent shows minutes only
  int bri = clockBri();
  static int briShown = -1;
  if (bri != briShown) { briShown = bri; clockForce = true; }
  int32_t stamp = (t.tm_hour * 60 + t.tm_min) * 60 + (secs ? t.tm_sec : 0);
  if (stamp == clockStamp && !clockForce) return;
  clockStamp = stamp;
  clockForce = false;
  memcpy(clkBuf, dialBuf, (size_t)TFT_W * TFT_H * 2);
  float hh, mm;
  handTimes(t, hh, mm);
  clk::drawSecondLayer(clkBuf, face, hh, mm, t.tm_sec, secs);   // the minute layer is in the dial
  // to the screen at the clock's brightness, leaving the tap overlay alone
  static uint8_t d5[32], d6[64];
  static int dimAt = -1;
  if (dimAt != bri) {
    for (int i = 0; i < 32; i++) d5[i] = (i * bri + 127) / 255;
    for (int i = 0; i < 64; i++) d6[i] = (i * bri + 127) / 255;
    dimAt = bri;
  }
  int rows = min(overlayTop, TFT_H);
  size_t n = (size_t)rows * TFT_W;
  if (bri >= 255) memcpy(fb, clkBuf, n * 2);
  else
    for (size_t i = 0; i < n; i++) {
      uint16_t c = clkBuf[i];
      fb[i] = (d5[c >> 11] << 11) | (d6[(c >> 5) & 63] << 5) | d5[c & 31];
    }
  Cache_WriteBack_Addr((uint32_t)fb, n * 2);
  if (asleep) { asleep = false; backlight(); }        // the clock first, then the light
}

static void startClock() {
  idleShown = false; photoShown = false;
  idleSince = 0; peekUntil = 0;
  clockShown = true;
  clockForce = true;
  clockTick();
  backlight();
}

// GET a Home Assistant API path; the reply body in `body`.
static bool haGet(const String &base, const String &token, const String &path, String &body, String &err) {
  WiFiClient plain;                 // the connections first: HTTPClient's destructor still uses its client
  WiFiClientSecure tls;
  HTTPClient http;
  String u = base + path;
  bool begun;
  if (u.startsWith("https://")) { tls.setInsecure(); begun = http.begin(tls, u); }
  else begun = http.begin(plain, u);
  if (!begun) { err = "bad Home Assistant URL"; return false; }
  http.setConnectTimeout(3000);
  http.setTimeout(4000);
  http.addHeader("Authorization", "Bearer " + token);
  int code = http.GET();
  if (code != 200) {
    err = path + ": " + (code == 401 ? String("token refused (401)") : code == 404 ? String("not found (404)")
                                       : code < 0 ? String("can't reach Home Assistant") : "HTTP " + String(code));
    http.end();
    return false;
  }
  body = http.getString();
  http.end();
  return true;
}

// One sensor's state, ready to show: temperature and humidity in whole numbers, pressure with its unit
// ("983 HPA", "29.03 INHG"; *hpa gets it in hPa). "--" if the state isn't a number (e.g. unavailable).
static bool haFetch(const String &base, const String &token, const String &entity, int kind, String &val, String &err,
                    float *hpa = nullptr) {
  String body;
  if (!haGet(base, token, "/api/states/" + entity, body, err)) return false;
  JsonDocument filter;
  filter["state"] = true;
  filter["attributes"]["unit_of_measurement"] = true;
  JsonDocument doc;
  if (deserializeJson(doc, body, DeserializationOption::Filter(filter)) != DeserializationError::Ok) {
    err = entity + ": unreadable reply";
    return false;
  }
  const char *st = doc["state"] | "";
  char *end = nullptr;
  float v = strtof(st, &end);
  if (!st[0] || end == st) { err = entity + " is " + (st[0] ? st : "empty"); val = "--"; return true; }
  if (kind == HA_PRES) {
    String unit = doc["attributes"]["unit_of_measurement"] | "";
    unit.toUpperCase();
    val = fabsf(v) >= 100 ? String((int)lroundf(v)) : String(v, 2);   // 983 hPa, 29.03 inHg
    if (unit.length()) val += " " + unit.substring(0, 5);
    if (hpa) {
      float k = unit == "INHG" ? 33.8639f : unit == "KPA" ? 10 : unit == "PA" ? 0.01f : unit == "MMHG" ? 1.33322f
              : unit == "PSI" ? 68.9476f : 1;                                  // hPa, mbar
      *hpa = v * k;
    }
  } else {
    val = String((int)lroundf(v));
  }
  return true;
}

// Pressure every 10 minutes for 3 hours: is it rising or falling?
static float presHist[19];
static int presHistN = 0;
static uint32_t presHistAt = 0;
static int pressureTrend(float hpa) {
  if (!presHistN || millis() - presHistAt >= 10 * 60000) {
    if (presHistN == 19) memmove(presHist, presHist + 1, 18 * sizeof(float));
    else presHistN++;
    presHist[presHistN - 1] = hpa;
    presHistAt = millis();
  }
  if (presHistN < 7) return 0;                       // under an hour of readings: no arrow yet
  float d = hpa - presHist[0];
  return d >= 1.0f ? 1 : d <= -1.0f ? -1 : 0;
}

static void haTask(void *) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(500));
    if (!S.ha || WiFi.status() != WL_CONNECTED || updating) continue;
    if (!haKick && millis() - haTriedAt < HA_EVERY_MS) continue;
    haKick = false;
    haTriedAt = millis();
    xSemaphoreTake(haLock, portMAX_DELAY);
    String base = S.haUrl, token = S.haToken, ent[HA_N];
    for (int i = 0; i < HA_N; i++) ent[i] = S.haEnt[i];
    xSemaphoreGive(haLock);
    String err, val[HA_N];
    bool ok[HA_N] = {};
    float hpa = NAN;
    if (!token.length()) err = "no access token";
    else
      for (int i = 0; i < HA_N; i++)
        if (ent[i].length()) ok[i] = haFetch(base, token, ent[i], i, val[i], err, i == HA_PRES ? &hpa : nullptr);
    // Home Assistant's elevation (once an hour), to turn a station's pressure into sea-level pressure
    static uint32_t elevAt = 0;
    if (token.length() && S.haSea && ent[HA_PRES].length() && (isnan(haElevM) || millis() - elevAt > 3600000)) {
      String body, e2;
      if (haGet(base, token, "/api/config", body, e2)) {
        JsonDocument f, doc;
        f["elevation"] = true;
        if (deserializeJson(doc, body, DeserializationOption::Filter(f)) == DeserializationError::Ok &&
            doc["elevation"].is<float>()) { haElevM = doc["elevation"].as<float>(); elevAt = millis(); }
      }
    }
    String word;
    int trend = 0;
    float sea = NAN;
    if (ok[HA_PRES] && !isnan(hpa)) {
      float elev = S.haSea && !isnan(haElevM) ? haElevM : 0;
      sea = hpa / powf(1 - 2.25577e-5f * elev, 5.25588f);
      word = barometerWord(sea);
      trend = pressureTrend(hpa);
    }
    xSemaphoreTake(haLock, portMAX_DELAY);
    for (int i = 0; i < HA_N; i++) {
      if (ok[i]) { haTxt[i] = val[i]; haAt[i] = millis(); }
      if (!ent[i].length()) haAt[i] = 0;
    }
    if (ok[HA_PRES]) { haPresWord = word; haPresTrend = trend; haPresHpa = hpa; haPresSea = sea; }
    haErr = err;
    xSemaphoreGive(haLock);
  }
}

// ---------- touch (GT911) ----------
static void touchBegin() {
  Wire.begin(TOUCH_SDA, TOUCH_SCL, 400000);
  for (uint8_t a : {0x5D, 0x14}) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { touchAddr = a; break; }
  }
}
// 1 = finger down, 0 = up, -1 = no new reading (keep the last state)
static int touched() {
  if (!touchAddr) return 0;
  Wire.beginTransmission(touchAddr);
  Wire.write(0x81); Wire.write(0x4E);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(touchAddr, (uint8_t)1) != 1) return -1;
  uint8_t st = Wire.read();
  if (!(st & 0x80)) return -1;
  Wire.beginTransmission(touchAddr);                // clear the status for the next read
  Wire.write(0x81); Wire.write(0x4E); Wire.write(0);
  Wire.endTransmission();
  return (st & 0x0F) > 0 ? 1 : 0;
}

// ---------- web ----------
static String esc(const String &s) {
  String o;
  for (char c : s) {
    if (c == '<') o += "&lt;"; else if (c == '&') o += "&amp;"; else if (c == '"') o += "&quot;"; else o += c;
  }
  return o;
}

static void sendJson(JsonDocument &d, int code = 200) {
  String out;
  serializeJson(d, out);
  web.sendHeader("Access-Control-Allow-Origin", "*");
  web.send(code, "application/json", out);
}

static const char *const MODES[3] = {"Auto", "Auto (with clock)", "Clock only"};
static const char *const TZS[][2] = {
    {"Pacific (Los Angeles)", "PST8PDT,M3.2.0,M11.1.0"}, {"Mountain (Denver)", "MST7MDT,M3.2.0,M11.1.0"},
    {"Arizona", "MST7"}, {"Central (Chicago)", "CST6CDT,M3.2.0,M11.1.0"},
    {"Eastern (New York)", "EST5EDT,M3.2.0,M11.1.0"}, {"Alaska", "AKST9AKDT,M3.2.0,M11.1.0"},
    {"Hawaii", "HST10"}, {"UK (London)", "GMT0BST,M3.5.0/1,M10.5.0"},
    {"Central Europe", "CET-1CEST,M3.5.0,M10.5.0/3"}, {"UTC", "UTC0"}};

static String nowText() {
  if (!timeOk()) return "not set yet (waiting for internet time)";
  time_t now = time(nullptr);
  tm t;
  localtime_r(&now, &t);
  static const char *days[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
  char b[32];
  snprintf(b, sizeof b, "%s %d:%02d %s", days[t.tm_wday % 7], (t.tm_hour + 11) % 12 + 1, t.tm_min, t.tm_hour < 12 ? "AM" : "PM");
  return b;
}

static void fillClock(JsonObject c) {
  c["mode"] = MODES[S.mode];
  c["face"] = clk::faceName(S.face);
  c["faceIndex"] = S.face;
  if (clk::defaultDigitColor(S.face)) {
    char hex[8];
    snprintf(hex, sizeof hex, "#%06lx", (unsigned long)clk::digitColor(S.face));
    JsonObject dg = c["digits"].to<JsonObject>();
    dg["color"] = hex;
    dg["glow"] = clk::digitGlowPct(S.face);
    dg["unlit"] = clk::digitUnlitPct(S.face);
  }
  c["showing"] = clockShown;
  c["dialMs"] = dialRenderMs;             // how long the last dial took to draw
  c["time"] = nowText();
  c["timeSet"] = timeOk();
  c["on"] = hhmm(S.clkOn);
  c["off"] = hhmm(S.clkOff);
  c["inHours"] = timeOk() && inSchedule();
  c["seconds"] = S.secs;
  c["bri"] = S.clkBri;
  c["night"] = S.night ? "dim" : "off";
  c["nightBri"] = S.nightBri;
  c["tz"] = S.tz;
  if (S.moon) {
    float p = clk::moonPhase(time(nullptr));
    c["moon"] = clk::moonName(p);
    c["moonPhase"] = roundf(p * 1000) / 1000;
  }
  if (S.ha) {
    xSemaphoreTake(haLock, portMAX_DELAY);
    JsonObject h = c["ha"].to<JsonObject>();
    static const char *const names[HA_N] = {"in", "out", "inHumidity", "outHumidity", "pressure"};
    for (int i = 0; i < HA_N; i++) {
      if (!S.haEnt[i].length()) continue;
      JsonObject o = h[names[i]].to<JsonObject>();
      o["entity"] = S.haEnt[i];
      o["value"] = haAt[i] ? haTxt[i] : String("--");
      o["age"] = haAt[i] ? (int32_t)((millis() - haAt[i]) / 1000) : -1;
    }
    if (haPresWord.length()) {
      JsonObject b = h["barometer"].to<JsonObject>();
      b["word"] = haPresWord;
      b["seaLevelHpa"] = roundf(haPresSea * 10) / 10;
      b["trend"] = haPresTrend > 0 ? "rising" : haPresTrend < 0 ? "falling" : "steady";
      if (!isnan(haElevM)) b["elevationM"] = haElevM;
    }
    h["error"] = haErr;
    xSemaphoreGive(haLock);
  }
}

static void fillInfo(JsonObject i) {
  i["ver"] = String("0.16.0-matrixinator-") + VERSION;   // looks like a WLED version to tools that parse it
  i["vid"] = 2611090;
  i["name"] = S.name;
  i["brand"] = "Matrixinator";
  i["product"] = "Matrixinator 4848S040";
  i["arch"] = "esp32-s3";
  i["mac"] = WiFi.macAddress();
  i["ip"] = WiFi.localIP().toString();
  i["live"] = live();
  i["lm"] = live() ? "DDP" : "";
  i["lip"] = live() ? liveFrom.toString() : "";
  i["uptime"] = millis() / 1000;
  i["freeheap"] = ESP.getFreeHeap();
  JsonObject dg = i["diag"].to<JsonObject>();     // for troubleshooting the screen
  dg["gfx"] = gfxOk;
  dg["fb"] = fb != nullptr;
  dg["psram"] = ESP.getPsramSize();
  dg["psramFree"] = ESP.getFreePsram();
  dg["touch"] = touchAddr;
  dg["blDuty"] = blDuty;
  dg["idle"] = idleShown;
  dg["dropped"] = rxDropped;
  dg["badFrames"] = rxBad;                       // incomplete or wrong-size frames that were skipped
  dg["photo"] = photoShown;
  i["photoPort"] = PHOTO_PORT;                  // Signinator streams photos here
  JsonObject ck = i["clock"].to<JsonObject>();
  fillClock(ck);
  JsonObject mx = i["mx"].to<JsonObject>();     // what's shown right now (Signinator's "full resolution" look)
  mx["w"] = curW;
  mx["h"] = curH;
  mx["look"] = customSize ? "pixels" : "dots";
  mx["shape"] = pixelLook ? "squares" : "dots";
  JsonObject l = i["leds"].to<JsonObject>();
  l["count"] = S.w * S.h;
  l["fps"] = fps;
  l["maxpwr"] = 0;
  l["rgbw"] = false;
  JsonObject m = l["matrix"].to<JsonObject>();
  m["w"] = S.w;
  m["h"] = S.h;
  JsonObject w = i["wifi"].to<JsonObject>();
  w["rssi"] = WiFi.RSSI();
  w["signal"] = constrain(2 * (WiFi.RSSI() + 100), 0, 100);
}
static void fillState(JsonObject s) {
  s["on"] = on;
  s["bri"] = bri;
  s["live"] = live();
}

static const char PAGE_HEAD[] PROGMEM =
    "<!doctype html><html><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>Matrixinator</title><style>body{font:15px system-ui;background:#0a0e15;color:#dde;max-width:560px;margin:auto;padding:16px}"
    "h1{font-size:22px}fieldset{border:1px solid #334;border-radius:8px;margin:12px 0;padding:10px 12px}label{display:block;margin:8px 0}"
    "input[type=text],input[type=password],input[type=number]{width:100%;box-sizing:border-box;padding:7px;background:#141a24;color:#fff;border:1px solid #334;border-radius:6px}"
    "button{padding:8px 16px;border-radius:6px;border:0;background:#00b6ff;color:#012;font-weight:600}.note{color:#889;font-size:13px}</style></head><body>";

// Settings page: the digital faces' colour controls. The preview is tinted here in the browser from two
// built-in pictures (base + mask; see tools/preview/make_previews.py), so trying colours costs the screen nothing.
static const char DIGIT_JS[] = R"JS(<script>(function(){
var q=function(i){return document.getElementById(i)},F=document.querySelector('select[name=face]'),pv=q('pv'),cv=q('pvc'),L={};
if(!F||!cv)return;
function img(u){return new Promise(function(r){var i=new Image();i.onload=function(){r(i)};i.onerror=function(){r(null)};i.src=u})}
function px(i){var c=document.createElement('canvas');c.width=c.height=240;var x=c.getContext('2d');x.drawImage(i,0,0,240,240);return x.getImageData(0,0,240,240).data}
async function layers(f){if(L[f]===undefined){var a=await img('/api/clock.jpg?face='+f+'&layer=base&v='+PV),b=await img('/api/clock.jpg?face='+f+'&layer=mask&v='+PV);L[f]=a&&b?[px(a),px(b)]:0}return L[f]}
async function draw(){var f=+F.value,d=DIG[f];
 if(!d){cv.style.display='none';pv.style.display='block';return}
 var l=await layers(f);if(+F.value!=f)return;
 if(!l){cv.style.display='none';pv.style.display='block';return}
 var r=parseInt(d.c.substr(1,2),16),g=parseInt(d.c.substr(3,2),16),b=parseInt(d.c.substr(5,2),16),G=d.gl?d.g/100:0,U=d.un?d.u/100:0;
 var x=cv.getContext('2d'),o=x.createImageData(240,240),B=l[0],M=l[1],O=o.data;
 for(var i=0;i<O.length;i+=4){var k=(M[i]+G*M[i+1]+U*M[i+2])/255;O[i]=B[i]+k*r;O[i+1]=B[i+1]+k*g;O[i+2]=B[i+2]+k*b;O[i+3]=255}
 x.putImageData(o,0,0);cv.style.display='block';pv.style.display='none'}
function hsv(c){var r=parseInt(c.substr(1,2),16)/255,g=parseInt(c.substr(3,2),16)/255,b=parseInt(c.substr(5,2),16)/255,
 M=Math.max(r,g,b),m=Math.min(r,g,b),e=M-m,h=0;if(e){h=M==r?(g-b)/e%6:M==g?(b-r)/e+2:(r-g)/e+4;h*=60;if(h<0)h+=360}
 return [Math.round(h),Math.round(M?e/M*100:0),Math.round(M*100)]}
function hex(h,s,v){s/=100;v/=100;var f=function(n){var k=(n+h/60)%6;return Math.round(255*(v-v*s*Math.max(0,Math.min(k,4-k,1))))};
 return '#'+[f(5),f(3),f(1)].map(function(x){return ('0'+x.toString(16)).slice(-2)}).join('')}
function hsvShow(d,keep){if(!keep){var a=hsv(d.c);d.H=a[0];d.S=a[1];d.V=a[2]}
 q('dh').value=d.H;q('ds').value=d.S;q('dv').value=d.V;q('dhv').textContent=d.H+'\u00b0';q('dsv').textContent=d.S+'%';q('dvv').textContent=d.V+'%';
 if(document.activeElement!=q('dhex'))q('dhex').value=d.c}
function show(keep){var d=DIG[+F.value];q('dctl').style.display=d?'':'none';
 if(d){q('dcol').value=d.c;hsvShow(d,keep);q('dgl').value=d.g;q('dun').value=d.u;q('dgv').textContent=d.g+'%';q('duv').textContent=d.u+'%';
  q('dgll').style.display=d.gl?'':'none';q('dul').style.display=d.un?'':'none'}
 draw()}
function set(k,v,keep){var f=+F.value,d=DIG[f];if(!d)return;d[k]=v;q('dc'+f).value=d.c;q('dg'+f).value=d.g;q('du'+f).value=d.u;show(keep)}
function fine(){var d=DIG[+F.value];if(!d)return;d.H=+q('dh').value;d.S=+q('ds').value;d.V=+q('dv').value;set('c',hex(d.H,d.S,d.V),1)}
['dh','ds','dv'].forEach(function(i){q(i).oninput=fine});
document.querySelectorAll('[data-n]').forEach(function(b){b.onclick=function(){var t=q(b.dataset.n);
 t.value=Math.max(+t.min,Math.min(+t.max,+t.value+(+b.dataset.d)));fine()}});
q('dhex').oninput=function(){var v=this.value.trim();if(v[0]!='#')v='#'+v;if(/^#[0-9a-fA-F]{6}$/.test(v))set('c',v.toLowerCase())};
q('dcol').oninput=function(){set('c',this.value)};
q('dgl').oninput=function(){set('g',+this.value)};
q('dun').oninput=function(){set('u',+this.value)};
q('dreset').onclick=function(){var d=DIG[+F.value];if(!d)return;d.c=d.d;d.g=100;set('u',100)};
document.querySelectorAll('.sw [data-c]').forEach(function(b){b.onclick=function(){set('c',b.dataset.c)}});
F.addEventListener('change',show);show();
})();</script>)JS";

static void handleRoot() {
  String h = FPSTR(PAGE_HEAD);
  h += "<h1>" + esc(S.name) + "</h1><p class=note>Matrixinator v" + VERSION + " &middot; " + WiFi.localIP().toString() +
       " &middot; " + (live() ? String(fps) + " fps from " + liveFrom.toString() : "waiting for pixels") + "</p>";
  h += "<form method=post action=/settings><fieldset><legend>Matrix</legend>"
       "<label>Name<input type=text name=name value=\"" + esc(S.name) + "\"></label>"
       "<label>Width (LEDs)<input type=number min=1 max=240 name=w value=" + String(S.w) + "></label>"
       "<label>Height (LEDs)<input type=number min=1 max=240 name=h value=" + String(S.h) + "></label>"
       "<label><input type=checkbox name=unlit " + (S.unlit ? "checked" : "") + "> Show unlit LEDs as dark dots</label>"
       "<p class=note>Set the same size as the LED sign. Signinator reads it from /json/info like it does from WLED.</p></fieldset>"
       "<fieldset><legend>Brightness</legend>"
       "<label><input type=checkbox name=follow " + String(S.follow ? "checked" : "") + "> Follow the brightness Signinator sends</label>"
       "<label>Brightest (10-255)<input type=number min=10 max=255 name=blmax value=" + String(S.blMax) + "></label>"
       "<label>Dimmest (0-255)<input type=number min=0 max=255 name=blmin value=" + String(S.blMin) + "></label></fieldset>"
       "<fieldset><legend>Clock</legend>"
       "<label>Screen mode<select name=mode>";
  for (int m = 0; m < 3; m++)
    h += "<option value=" + String(m) + (S.mode == m ? " selected>" : ">") + MODES[m] + "</option>";
  h += "</select></label><p class=note>Auto: dark when Signinator sends nothing. Auto (with clock): the clock "
       "instead, during the hours below. Clock only: always the clock in those hours, pictures ignored.</p>"
       "<label>Face<select name=face onchange=\"pv.style.opacity=.35;pv.src='/api/clock.jpg?v='+PV+'&face='+this.value\">";
  for (int f = 0; f < clk::FACE_COUNT; f++) {          // grouped: Classic, Holidays, Digital
    if (!f || strcmp(clk::faceGroup(f), clk::faceGroup(f - 1)))
      h += String(f ? "</optgroup>" : "") + "<optgroup label=\"" + clk::faceGroup(f) + "\">";
    h += "<option value=" + String(f) + (S.face == f ? " selected>" : ">") + clk::faceName(f) + "</option>";
  }
  h += "</optgroup></select></label>"
       "<p class=note>The holiday faces count down to their day in the window under 12.</p>"
       "<img id=pv alt='Clock preview' src='/api/clock.jpg?v=" + String(VERSION) + "&face=" + String(S.face) +
       "' onload=\"this.style.opacity=1\" onerror=\"this.style.opacity=1\""
       " style='width:100%;max-width:320px;aspect-ratio:1;display:block;margin:6px auto;border-radius:10px;background:#000;transition:opacity .2s'>"
       "<canvas id=pvc width=240 height=240 style='display:none;width:100%;max-width:320px;aspect-ratio:1;margin:6px auto;border-radius:10px;background:#000'></canvas>"
       "<p class=note style='text-align:center'>Preview of the face (sample weather). Pick one, then Save.</p>"
       "<div id=dctl style='display:none'>"
       "<label>Digit colour<input type=color id=dcol style='height:38px'></label>"
       "<div class=sw>"
       "<button type=button data-c='#ff2c1c' style='background:#ff2c1c'>Red</button>"
       "<button type=button data-c='#ff9a1c' style='background:#ff9a1c'>Amber</button>"
       "<button type=button data-c='#3cff5a' style='background:#3cff5a'>Green</button>"
       "<button type=button data-c='#00f2c8' style='background:#00f2c8'>Stereo</button>"
       "<button type=button data-c='#00ffe0' style='background:#00ffe0'>Aqua</button>"
       "<button type=button data-c='#2a9dff' style='background:#2a9dff'>Blue</button>"
       "<button type=button data-c='#ffffff' style='background:#fff'>White</button></div>"
       "<div class=note>Stereo greens and blues</div><div class=sw>"
       "<button type=button data-c='#3cffa0' style='background:#3cffa0'>&nbsp;</button>"
       "<button type=button data-c='#14ffb4' style='background:#14ffb4'>&nbsp;</button>"
       "<button type=button data-c='#00fac0' style='background:#00fac0'>&nbsp;</button>"
       "<button type=button data-c='#00f2c8' style='background:#00f2c8'>&nbsp;</button>"
       "<button type=button data-c='#00ecd2' style='background:#00ecd2'>&nbsp;</button>"
       "<button type=button data-c='#00e6dc' style='background:#00e6dc'>&nbsp;</button>"
       "<button type=button data-c='#00dce8' style='background:#00dce8'>&nbsp;</button>"
       "<button type=button data-c='#00d0f4' style='background:#00d0f4'>&nbsp;</button>"
       "<button type=button data-c='#00c0ff' style='background:#00c0ff'>&nbsp;</button></div>"
       "<label>Fine tune<input type=text id=dhex maxlength=7 placeholder='#00f2c8' style='font-family:monospace'></label>"
       "<label>Hue <b id=dhv></b> <span class=nb><button type=button data-n=dh data-d=-1>&minus;</button>"
       "<button type=button data-n=dh data-d=1>+</button></span><input type=range id=dh min=0 max=359 step=1></label>"
       "<label>Saturation <b id=dsv></b> <span class=nb><button type=button data-n=ds data-d=-1>&minus;</button>"
       "<button type=button data-n=ds data-d=1>+</button></span><input type=range id=ds min=0 max=100 step=1></label>"
       "<label>Brightness <b id=dvv></b> <span class=nb><button type=button data-n=dv data-d=-1>&minus;</button>"
       "<button type=button data-n=dv data-d=1>+</button></span><input type=range id=dv min=0 max=100 step=1></label>"
       "<label id=dgll>Glow <b id=dgv></b><input type=range id=dgl min=0 max=200 step=5></label>"
       "<label id=dul>Unlit segments <b id=duv></b><input type=range id=dun min=0 max=300 step=5></label>"
       "<button type=button id=dreset>Default colour</button>"
       "<p class=note>Hue moves green (150\u00b0) towards blue (190\u00b0) one degree at a time. For a deeper colour on the backlit screen, keep saturation at 100%, "
       "turn the glow up and the unlit segments down.</p></div>";
  {
    String dj = "{";
    for (int f = 0; f < clk::FACE_COUNT; f++) {
      uint32_t def = clk::defaultDigitColor(f);
      if (!def) continue;
      char c[8], d[8];
      snprintf(c, sizeof c, "#%06lx", (unsigned long)clk::digitColor(f));
      snprintf(d, sizeof d, "#%06lx", (unsigned long)def);
      if (dj.length() > 1) dj += ",";
      // gl: has a glow (not Flip); un: has unlit segments (Digital)
      dj += String(f) + ":{c:'" + c + "',d:'" + d + "',g:" + clk::digitGlowPct(f) + ",u:" + clk::digitUnlitPct(f) +
            ",gl:" + (clk::digitHasGlow(f) ? "1" : "0") + ",un:" + (clk::digitHasUnlit(f) ? "1" : "0") + "}";
      h += "<input type=hidden name=dc" + String(f) + " id=dc" + f + " value='" + c + "'>"
           "<input type=hidden name=dg" + String(f) + " id=dg" + f + " value=" + clk::digitGlowPct(f) + ">"
           "<input type=hidden name=du" + String(f) + " id=du" + f + " value=" + clk::digitUnlitPct(f) + ">";
    }
    dj += "}";
    h += "<script>var PV='" + String(VERSION) + "';</script>";   // in the picture links: a new firmware's pictures aren't taken from the browser's cache
    h += "<style>.sw{display:flex;flex-wrap:wrap;gap:6px;margin:4px 0 8px}.sw button{padding:6px 9px;color:#000;border:0;"
         "border-radius:6px;font-size:13px}.nb button{padding:1px 9px;margin-left:4px;font-size:14px}input[type=range]{width:100%}</style><script>var DIG=" + dj + ";</script>";
  }
  h += "<label><input type=checkbox name=secs " + String(S.secs ? "checked" : "") + "> Second hand</label>"
       "<label><input type=checkbox name=moon " + String(S.moon ? "checked" : "") + "> Moon phase</label>"
       "<label>Day starts at<input type=time name=con value=" + hhmm(S.clkOn) + "></label>"
       "<label>Night starts at<input type=time name=coff value=" + hhmm(S.clkOff) + "></label>"
       "<label>Time zone<select name=tz>";
  bool tzKnown = false;
  for (auto &z : TZS) {
    bool sel = S.tz == z[1];
    tzKnown |= sel;
    h += "<option value=\"" + String(z[1]) + "\"" + (sel ? " selected>" : ">") + z[0] + "</option>";
  }
  if (!tzKnown) h += "<option value=\"" + esc(S.tz) + "\" selected>" + esc(S.tz) + "</option>";
  h += "</select></label>"
       "<label>Start-up animation (seconds, 0-120)<input type=number min=0 max=120 name=boot value=" + String(S.bootSecs) + "></label>"
       "<p class=note>After power-on the Matrix rain plays this long before the pictures or the clock. A tap skips it.</p>"
       "<label>Clock brightness (10-255)<input type=number min=10 max=255 name=cbri value=" + String(S.clkBri) + "></label>"
       "<label>At night<select name=night><option value=0" + (S.night == 0 ? " selected" : "") + ">Screen off</option>"
       "<option value=1" + (S.night == 1 ? " selected" : "") + ">Clock, dimmed</option></select></label>"
       "<label>Night brightness (5-255)<input type=number min=5 max=255 name=nbri value=" + String(S.nightBri) + "></label>"
       "<p class=note>Now " + nowText() + ". With the screen off at night, a tap shows the clock for 30 s. "
       "The backlight can't dim on this board, so a dimmed clock still has a faint glow behind black.</p>"
       "<button type=button onclick=\"fetch('/api/clock?preview=30',{method:'POST'})\">Show the clock for 30 s</button>"
       "</fieldset>"
       "<fieldset><legend>Home Assistant weather</legend>"
       "<label><input type=checkbox name=ha " + String(S.ha ? "checked" : "") + "> Show temperature, humidity and pressure on the clock</label>"
       "<label>Home Assistant URL<input type=text name=haurl value=\"" + esc(S.haUrl) + "\"></label>"
       "<label>Long-lived access token<input type=password name=hatok placeholder='" +
       (S.haToken.length() ? "saved; type to change" : "Profile > Security > Long-lived access tokens") + "'></label>"
       "<p class=note>Sensor entity ids. Leave one empty to leave it off the clock.</p>";
  for (int i = 0; i < HA_N; i++)
    h += String("<label>") + HA_LABELS[i] + "<input type=text name=" + HA_KEYS[i] + " placeholder=" + HA_HINTS[i] +
         " value=\"" + esc(S.haEnt[i]) + "\"></label>";
  h += "<label><input type=checkbox name=hasea " + String(S.haSea ? "checked" : "") +
       "> The pressure sensor reads local (station) pressure: correct it to sea level with Home Assistant's elevation "
       "for the barometer word (Stormy, Rain, Change, Fair, Very dry)</label>";
  if (S.ha) {
    static const char *const units[HA_N] = {"&deg;", "&deg;", "%", "%", ""};
    xSemaphoreTake(haLock, portMAX_DELAY);
    String st = "Now:";
    for (int i = 0; i < HA_N; i++)
      if (S.haEnt[i].length())
        st += String(i ? ", " : " ") + HA_LABELS[i] + " " + (haAt[i] ? haTxt[i] + units[i] : String("--"));
    if (haPresWord.length())
      st += " (" + haPresWord + ", " + String(haPresSea, 1) + " hPa at sea level" +
            (haPresTrend > 0 ? ", rising" : haPresTrend < 0 ? ", falling" : "") + ")";
    if (haErr.length()) st += ". Problem: " + esc(haErr);
    xSemaphoreGive(haLock);
    h += "<p class=note>" + st + "</p>";
  }
  h += "</fieldset>"
       "<fieldset><legend>Wi-Fi</legend>"
       "<label>Network<input type=text name=ssid value=\"" + esc(S.ssid) + "\"></label>"
       "<label>Password<input type=password name=pass placeholder='" + (S.pass.length() ? "saved; type to change" : "") + "'></label>"
       "<label>Host name<input type=text name=host value=\"" + esc(S.host) + "\"></label></fieldset>"
       "<button>Save</button></form>"
       "<form method=post action=/update enctype=multipart/form-data><fieldset><legend>Update firmware</legend>"
       "<input type=file name=update accept=.bin> <button>Upload</button>"
       "<p class=note>Or use Lickety Split to update over Wi-Fi.</p></fieldset></form>";
  h += DIGIT_JS;
  h += "</body></html>";
  web.send(200, "text/html", h);
}

static void handleSettings() {
  if (updating) { web.send(503, "text/plain", "Updating firmware"); return; }
  bool wifiChanged = false, sizeChanged = false;
  if (web.hasArg("name") && web.arg("name").length()) S.name = web.arg("name").substring(0, 40);
  int w = web.hasArg("w") ? constrain(web.arg("w").toInt(), 1, MAX_W) : S.w;
  int h = web.hasArg("h") ? constrain(web.arg("h").toInt(), 1, MAX_H) : S.h;
  sizeChanged = w != S.w || h != S.h;
  S.w = w; S.h = h;
  S.unlit = web.hasArg("unlit");
  S.follow = web.hasArg("follow");
  if (web.hasArg("blmax")) S.blMax = constrain(web.arg("blmax").toInt(), 10, 255);
  if (web.hasArg("blmin")) S.blMin = constrain(web.arg("blmin").toInt(), 0, 255);
  if (web.hasArg("ssid") && web.arg("ssid") != S.ssid) { S.ssid = web.arg("ssid"); wifiChanged = true; }
  if (web.hasArg("pass") && web.arg("pass").length()) { S.pass = web.arg("pass"); wifiChanged = true; }
  if (web.hasArg("host") && web.arg("host").length()) S.host = web.arg("host").substring(0, 32);
  // clock (only when the form has the clock fields, so an older form can't switch things off)
  bool tzChanged = false;
  if (web.hasArg("mode")) {
    S.mode = constrain(web.arg("mode").toInt(), 0, 2);
    S.face = constrain(web.arg("face").toInt(), 0, clk::FACE_COUNT - 1);
    for (int f = 0; f < clk::FACE_COUNT; f++) {
      if (!clk::defaultDigitColor(f) || !web.hasArg("dc" + String(f))) continue;
      String c = web.arg("dc" + String(f));
      c.replace("#", "");
      if (c.length() == 6) clk::setDigitColor(f, strtoul(c.c_str(), nullptr, 16));
      clk::setDigitLevels(f, web.arg("dg" + String(f)).toInt(), web.arg("du" + String(f)).toInt());
    }
    S.secs = web.hasArg("secs");
    S.moon = web.hasArg("moon");
    S.clkOn = parseHhmm(web.arg("con"), S.clkOn);
    S.clkOff = parseHhmm(web.arg("coff"), S.clkOff);
    if (web.hasArg("cbri")) S.clkBri = constrain(web.arg("cbri").toInt(), 10, 255);
    if (web.hasArg("night")) S.night = constrain(web.arg("night").toInt(), 0, 1);
    if (web.hasArg("boot")) S.bootSecs = constrain(web.arg("boot").toInt(), 0, 120);
    if (web.hasArg("nbri")) S.nightBri = constrain(web.arg("nbri").toInt(), 5, 255);
    if (web.hasArg("tz") && web.arg("tz").length() && web.arg("tz") != S.tz) { S.tz = web.arg("tz").substring(0, 60); tzChanged = true; }
    xSemaphoreTake(haLock, portMAX_DELAY);
    S.ha = web.hasArg("ha");
    String u = web.arg("haurl");
    u.trim();
    while (u.endsWith("/")) u.remove(u.length() - 1);
    if (u.length()) S.haUrl = u.substring(0, 120);
    if (web.arg("hatok").length()) { S.haToken = web.arg("hatok"); S.haToken.trim(); }
    S.haSea = web.hasArg("hasea");
    for (int i = 0; i < HA_N; i++) {
      S.haEnt[i] = web.arg(HA_KEYS[i]);
      S.haEnt[i].trim();
      haAt[i] = 0;
    }
    haErr = "";
    xSemaphoreGive(haLock);
    haKick = true;
    clockForce = true;
    dialKey = "";
  }
  saveSettings();
  if (tzChanged) applyTz();
  backlight();
  web.sendHeader("Location", "/");
  web.send(303);
  if (sizeChanged && !customSize) setLook(S.w, S.h, false, false);
  if (wifiChanged) { delay(300); ESP.restart(); }
}

static void handleUpdateDone() {
  bool ok = !Update.hasError();
  web.sendHeader("Access-Control-Allow-Origin", "*");
  web.send(ok ? 200 : 500, "text/plain", ok ? "Update done, restarting." : "Update failed.");
  if (ok) { mrPanel("RESTARTING", rgb565(60, 255, 90), "", 0, ""); delay(700); ESP.restart(); }
  mrPanel("UPDATE FAILED", RED, "", 0, "");
  delay(2500);
  updating = false;
  goDark();
}
static void handleUpdateUpload() {
  HTTPUpload &u = web.upload();
  if (u.status == UPLOAD_FILE_START) {
    updating = true;                         // from here on: taps, pixels, photos and settings are ignored
    overlayUntil = 0; overlayTop = TFT_H;
    clockShown = false;
    upTotal = web.clientContentLength() > 0 ? web.clientContentLength() : 0;
    upDone = 0;
    digitalWrite(TFT_BCKL, HIGH);
    gfx->fillScreen(BLACK);
    mrInit();
    idleShown = false;
    asleep = false;
    mrPanel("FIRMWARE UPDATE", rgb565(60, 255, 90), "", 0, "Don't unplug");
    mrStill();
    updateProgress();
    backlight();
    Update.begin(UPDATE_SIZE_UNKNOWN);
  } else if (u.status == UPLOAD_FILE_WRITE) {
    Update.write(u.buf, u.currentSize);
    upDone += u.currentSize;
    updateAnimate();
  } else if (u.status == UPLOAD_FILE_END) {
    Update.end(true);
  } else if (u.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
  }
}

static void handleState() {
  if (web.method() == HTTP_POST && updating) { web.send(503, "text/plain", "Updating firmware"); return; }
  if (web.method() == HTTP_POST) {
    JsonDocument in;
    if (deserializeJson(in, web.arg("plain")) == DeserializationError::Ok) {
      if (in["rb"] | false) {                    // restart, like WLED's {"rb":true}
        web.send(200, "application/json", "{\"success\":true}");
        delay(300);
        ESP.restart();
      }
      if (in["on"].is<bool>()) on = in["on"].as<bool>();
      else if (in["on"].is<const char *>() && String(in["on"].as<const char *>()) == "t") on = !on;
      if (in["bri"].is<int>()) bri = constrain(in["bri"].as<int>(), 0, 255);
      backlight();
      if (!on) { if (!clockShown) gfx->fillScreen(BLACK); idleShown = false; forceRedraw = true; }
      else if (!live() && !idleShown && !clockShown) goDark();
    }
  }
  JsonDocument d;
  fillState(d.to<JsonObject>());
  sendJson(d);
}

static void setupWeb() {
  web.on("/", HTTP_GET, handleRoot);
  web.on("/settings", HTTP_POST, handleSettings);
  web.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
  web.on("/json/info", HTTP_GET, [] { JsonDocument d; fillInfo(d.to<JsonObject>()); sendJson(d); });
  web.on("/json/state", handleState);
  web.on("/json/si", HTTP_GET, [] {
    JsonDocument d;
    fillState(d["state"].to<JsonObject>());
    fillInfo(d["info"].to<JsonObject>());
    sendJson(d);
  });
  web.on("/json", HTTP_GET, [] {
    JsonDocument d;
    fillState(d["state"].to<JsonObject>());
    fillInfo(d["info"].to<JsonObject>());
    sendJson(d);
  });
  web.on("/json/cfg", HTTP_GET, [] {   // the parts of WLED's config Signinator reads
    JsonDocument d;
    JsonObject live = d["if"]["live"].to<JsonObject>();
    live["en"] = true;
    live["rlm"] = true;
    live["no-gc"] = true;
    JsonObject led = d["hw"]["led"].to<JsonObject>();
    led["fps"] = 60;
    JsonObject o = led["ins"].to<JsonArray>().add<JsonObject>();
    o["start"] = 0;
    o["len"] = S.w * S.h;
    o["pin"].to<JsonArray>().add(-1);
    JsonObject p = led["matrix"]["panels"].to<JsonArray>().add<JsonObject>();
    p["x"] = 0; p["y"] = 0; p["w"] = S.w; p["h"] = S.h;
    sendJson(d);
  });
  web.on("/api/test", HTTP_GET, [] {      // troubleshooting: /api/test?color=ff0000&duty=255&freq=5000
    if (web.hasArg("pin")) digitalWrite(TFT_BCKL, web.arg("pin").toInt() ? HIGH : LOW);
    if (web.hasArg("color")) {
      uint32_t c = strtoul(web.arg("color").c_str(), nullptr, 16);
      gfx->fillScreen(rgb565(c >> 16, c >> 8, c));
      idleShown = true;   // the next pixels or idle check redraw over it
    }
    web.send(200, "text/plain", String("gfx=") + gfxOk + " fb=" + (fb != nullptr) + " level=" + blDuty);
  });
  // {"look":"pixels","w":120,"h":120,"shape":"dots"|"squares"} = the sender's size; {"look":"dots"} = the saved size
  web.on("/api/look", HTTP_POST, [] {
    if (updating) { web.send(503, "text/plain", "Updating firmware"); return; }
    JsonDocument in;
    if (deserializeJson(in, web.arg("plain")) != DeserializationError::Ok) { web.send(400, "text/plain", "bad JSON"); return; }
    bool px = String(in["look"] | "dots") == "pixels";
    if (px) setLook(in["w"] | 120, in["h"] | 120, String(in["shape"] | "squares") == "squares", true);
    else setLook(S.w, S.h, false, false);
    if (!live() && !idleShown && !clockShown) goDark();
    web.send(200, "application/json", "{\"ok\":true}");
  });
  web.on("/api/photo", HTTP_POST, [] {
    if (updating) { web.send(503, "text/plain", "Updating firmware"); return; }
    bool ok = photoOk;
    web.send(ok ? 200 : 400, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
    if (ok) { liveFrom = web.client().remoteIP(); showPhoto(photoBuf, photoLen, 1); }
  }, [] {
    HTTPUpload &u = web.upload();
    if (u.status == UPLOAD_FILE_START) {
      if (!photoBuf) photoBuf = (uint8_t *)ps_malloc(PHOTO_MAX);
      photoLen = 0;
      photoOk = photoBuf != nullptr;
    } else if (u.status == UPLOAD_FILE_WRITE && photoOk) {
      if (photoLen + u.currentSize > PHOTO_MAX) { photoOk = false; return; }
      memcpy(photoBuf + photoLen, u.buf, u.currentSize);
      photoLen += u.currentSize;
    } else if (u.status == UPLOAD_FILE_ABORTED) {
      photoOk = false;
    }
  });
  // GET: the clock's status. POST ?preview=30 (or JSON {"preview":30,"face":1,"mode":1}): show it now, change the
  // face or mode (face and mode are saved).
  web.on("/api/clock", [] {
    if (web.method() == HTTP_POST) {
      if (updating) { web.send(503, "text/plain", "Updating firmware"); return; }
      JsonDocument in;
      deserializeJson(in, web.arg("plain"));
      int pv = web.hasArg("preview") ? web.arg("preview").toInt() : (in["preview"] | 0);
      bool save = false;
      if (in["face"].is<int>()) { S.face = constrain(in["face"].as<int>(), 0, clk::FACE_COUNT - 1); save = true; }
      if (in["mode"].is<int>()) { S.mode = constrain(in["mode"].as<int>(), 0, 2); save = true; }
      if (clk::defaultDigitColor(S.face)) {           // the digital face's colour: "#rrggbb", glow %, unlit %
        if (in["color"].is<const char *>()) {
          String c = in["color"].as<const char *>();
          c.replace("#", "");
          if (c.length() == 6) { clk::setDigitColor(S.face, strtoul(c.c_str(), nullptr, 16)); save = true; }
        }
        if (in["glow"].is<int>() || in["unlit"].is<int>()) {
          clk::setDigitLevels(S.face, in["glow"] | clk::digitGlowPct(S.face), in["unlit"] | clk::digitUnlitPct(S.face));
          save = true;
        }
      }
      if (save) { saveSettings(); clockForce = true; }
      if (pv > 0) {
        if (!S.mode) { S.mode = 1; saveSettings(); }   // a preview needs a clock mode
        clockPeekUntil = millis() + (uint32_t)constrain(pv, 1, 3600) * 1000;
      }
    }
    JsonDocument d;
    fillClock(d.to<JsonObject>());
    sendJson(d);
  });
  // A picture of clock face N (default the saved one), drawn ahead of time on a PC (tools/preview/make_previews.py)
  // with sample weather: 240x240 JPEG. Drawing it here took seconds and slowed the screen.
  web.on("/api/clock.jpg", HTTP_GET, [] {
    int face = web.hasArg("face") ? constrain(web.arg("face").toInt(), 0, clk::FACE_COUNT - 1) : S.face;
    const FacePreview &p = FACE_PREVIEWS[face];
    // digital faces: layer=base / layer=mask, for the page to tint with any colour (see make_previews.py)
    String layer = web.arg("layer");
    const uint8_t *data = p.data;
    size_t len = p.len;
    if (layer == "base" || layer == "mask") {
      if (!p.base) { web.send(404, "text/plain", "Not a digital face"); return; }
      data = layer == "base" ? p.base : p.mask;
      len = layer == "base" ? p.baseLen : p.maskLen;
    }
    web.sendHeader("Cache-Control", "max-age=86400");
    web.send_P(200, "image/jpeg", (const char *)data, len);
  });
  web.on("/api/restart", HTTP_POST, [] {
    web.send(200, "application/json", "{\"ok\":true}");
    delay(300);
    ESP.restart();
  });
  web.on("/api/overlay", HTTP_POST, [] {   // Signinator sends the song so a tap can show it
    JsonDocument in;
    if (deserializeJson(in, web.arg("plain")) == DeserializationError::Ok) {
      npTitle = in["title"] | "";
      npArtist = in["artist"] | "";
    }
    web.send(200, "application/json", "{\"ok\":true}");
  });
  web.onNotFound([] {
    if (apOn) { web.sendHeader("Location", "http://192.168.4.1/"); web.send(302); return; }
    web.send(404, "text/plain", "Not found");
  });
  web.begin();
}

// ---------- main ----------
void setup() {
  Serial.begin(115200);
  loadSettings();
  pinMode(TFT_BCKL, OUTPUT);
  digitalWrite(TFT_BCKL, LOW);

  bus = new Arduino_SWSPI(GFX_NOT_DEFINED, TFT_CS, TFT_SCLK, TFT_MOSI, GFX_NOT_DEFINED);
  rgbpanel = new Arduino_ESP32RGBPanel(TFT_DE, TFT_VSYNC, TFT_HSYNC, TFT_PCLK, TFT_R0, TFT_R1, TFT_R2, TFT_R3, TFT_R4,
                                       TFT_G0, TFT_G1, TFT_G2, TFT_G3, TFT_G4, TFT_G5, TFT_B0, TFT_B1, TFT_B2, TFT_B3,
                                       TFT_B4, 1, 10, 8, 50, 1, 10, 8, 20);
  gfx = new Arduino_RGB_Display(TFT_W, TFT_H, rgbpanel, 0, true, bus, GFX_NOT_DEFINED, st7701_4848s040_init,
                                sizeof(st7701_4848s040_init));
  gfxOk = gfx->begin(GFX_NOT_DEFINED);
  fb = gfx->getFramebuffer();
  bufLock = xSemaphoreCreateMutex();
  haLock = xSemaphoreCreateMutex();
  dialBuf = (uint16_t *)ps_malloc((size_t)TFT_W * TFT_H * 2);
  dialBack = (uint16_t *)ps_malloc((size_t)TFT_W * TFT_H * 2);
  baseBuf = (uint16_t *)ps_malloc((size_t)TFT_W * TFT_H * 2);
  clk::init();
  clk::setYield([] {           // only the dial task gives way; drawing on the main loop (previews) runs straight
    static uint32_t at = 0;
    if (xTaskGetCurrentTaskHandle() == dialTaskH && millis() - at > 50) { at = millis(); vTaskDelay(1); }
  });
  clkBuf = (uint16_t *)ps_malloc((size_t)TFT_W * TFT_H * 2);
  curW = S.w; curH = S.h;
  Serial.printf("display begin=%d fb=%p psram=%u free=%u\n", gfxOk, fb, ESP.getPsramSize(), ESP.getFreePsram());
  layout();
  backlight();
  touchBegin();

  uint8_t mac[6];
  WiFi.macAddress(mac);
  char ap[32];
  snprintf(ap, sizeof ap, "Matrixinator-%02X%02X", mac[4], mac[5]);
  apName = ap;
  idleShown = false;
  showIdle();

  improv::begin({[](const String &s, const String &p) {
                   bool ok = joinWifi(s, p, true);
                   if (ok) { stopAp(); showIdle(); }
                   return ok;
                 },
                 url, "Matrixinator", VERSION, "ESP32-S3", "Matrixinator"});

  if (S.ssid.length()) joinWifi(S.ssid, S.pass, false);
  if (WiFi.status() != WL_CONNECTED) startAp();
  WiFi.setSleep(false);              // lowest latency for the pixel stream
  if (MDNS.begin(S.host.c_str())) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("wled", "tcp", 80);
  }
  xTaskCreatePinnedToCore(rxTask, "ddp", 6144, nullptr, 5, nullptr, 0);
  streamBuf = (uint8_t *)ps_malloc(PHOTO_MAX);
  streamDrawn = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(photoTask, "photo", 6144, nullptr, 4, nullptr, 0);
  xTaskCreatePinnedToCore(haTask, "ha", 10240, nullptr, 1, nullptr, 0);
  // drawing pauses for a tick now and then (setYield below), so the idle task and the watchdog are happy
  xTaskCreatePinnedToCore(dialTask, "dial", 8192, nullptr, 1, &dialTaskH, 0);
  applyTz();                          // internet time (keeps trying in the background)
  setupWeb();
  bootRainUntil = S.bootSecs ? max<uint32_t>(1, S.bootSecs * 1000UL) : 0;   // counted from power-on
  idleSince = 0;
  showIdle();
}

void loop() {
  if (updating) { web.handleClient(); delay(1); return; }   // firmware update: nothing else happens
  improv::loop();
  drawStreamPhoto();
  drawNewFrame();
  web.handleClient();
  if (apOn) dns.processNextRequest();

  uint32_t now = millis();
  if (now - fpsAt >= 1000) { fps = frames * 1000 / (now - fpsAt); frames = 0; fpsAt = now; }

  // Wi-Fi: keep trying; open the setup hotspot after 30 s without a connection
  if (WiFi.status() == WL_CONNECTED) {
    wifiLostAt = 0;
    if (apOn && WiFi.softAPgetStationNum() == 0) { stopAp(); if (!live()) goDark(); }
  } else if (S.ssid.length()) {
    if (!wifiLostAt) { wifiLostAt = now; WiFi.reconnect(); }
    else if (now - wifiLostAt > 30000 && !apOn) { startAp(); if (!live()) { bool w = asleep; showIdle(); wakeAfterDraw(w); } }
  }

  // The clock: when nothing is being shown (or always, in "Clock only") during its hours, or after a tap.
  if (clockPeekUntil && (int32_t)(now - clockPeekUntil) >= 0) clockPeekUntil = 0;
  {
    bool settingUp = apOn && WiFi.status() != WL_CONNECTED;
    bool picture = live() && on && S.mode != 2;
    if (!settingUp && !picture && !bootRain() && clockWanted()) {
      if (!clockShown) startClock();
      else clockTick();
    } else if (clockShown) {
      clockShown = false;
      if (!picture) goDark();
    }
  }

  if (on && !live() && lastFrameAt && !idleShown && !asleep && !bootRain() && !overlayUntil && !clockShown) goDark();   // stream stopped: dark
  if (idleShown && on && !overlayUntil && !asleep) {          // the rain keeps falling; panel follows Wi-Fi
    mrStep();
    static uint32_t panelAt = 0;
    if (now - panelAt > 1000) { panelAt = now; showIdle(); }
    // nothing to show for a while (and not setting up): go to sleep until pixels, a photo or a tap
    bool settingUp = apOn && WiFi.status() != WL_CONNECTED;
    if (!idleSince) idleSince = now;
    if (!settingUp && !bootRain() && now - idleSince > IDLE_SLEEP_MS) goDark();   // after power-on: dark until there's something
  }

  static uint32_t touchAt = 0;
  if (now - touchAt > 40) {
    touchAt = now;
    static bool wasDown = false;
    static uint32_t upAt = 0;
    int t = touched();
    if (t == 1 && !wasDown && now - upAt > 250) {
      if (bootRain() && !(apOn && WiFi.status() != WL_CONNECTED)) {
        bootRainUntil = 0;                                 // a tap skips the power-on rain
      } else if (asleep) {                                 // dark: the info strip (song, IP) on black
        if (S.mode && timeOk()) clockPeekUntil = now + CLOCK_PEEK_MS;   // and the clock for a while
        gfx->fillScreen(BLACK); asleep = false; backlight(); showOverlay();
      } else {
        bool lit = on || clockShown;
        if (lit) { digitalWrite(TFT_BCKL, LOW); delayMicroseconds(200); digitalWrite(TFT_BCKL, HIGH); }   // a tap re-arms the light too
        if (overlayUntil) hideOverlay(); else if (lit) showOverlay();
      }
    }
    if (t == 0 && wasDown) upAt = now;
    if (t >= 0) wasDown = t == 1;
  }
  if (overlayUntil && now > overlayUntil) hideOverlay();

  // The backlight driver sometimes shuts itself off (the picture keeps changing in the dark) and stays off
  // until its enable pin is switched off and on. Re-arm it every 30 s with a 0.2 ms blip (too short to see).
  static uint32_t rearmAt = 0;
  if ((clockShown || (on && blDuty > 0)) && !asleep && now - rearmAt >= 30000) {
    rearmAt = now;
    digitalWrite(TFT_BCKL, LOW);
    delayMicroseconds(200);
    digitalWrite(TFT_BCKL, HIGH);
  }
  delay(1);
}
