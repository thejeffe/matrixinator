// Matrixinator: shows an LED matrix on the Guition ESP32-S3-4848S040 touch screen.
//
// It looks like WLED to Signinator: /json/info reports a 2D matrix (32x32 by default), /json/state takes on/bri,
// and DDP on UDP 4048 brings the pixels. Each LED is drawn as a round dot; the backlight follows the brightness.
// Also: Improv Wi-Fi over USB (set Wi-Fi from the Lickety Split flasher), a fallback hotspot, a settings page,
// OTA updates at POST /update (same path as WLED), and tap-to-show song / status.
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
#include "board.h"
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
} S;
static Preferences prefs;

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
  prefs.end();
}
static void saveSettings() {
  prefs.begin("mx", false);
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
static uint32_t idleSince = 0, peekUntil = 0;
static const uint32_t IDLE_SLEEP_MS = 15000;
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
  digitalWrite(TFT_BCKL, (on && blDuty > 0 && !asleep) ? HIGH : LOW);
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
  bool wasAsleep = asleep;
  idleSince = 0; peekUntil = 0;
  if (idleShown || photoShown) { gfx->fillScreen(BLACK); forceRedraw = true; idleShown = false; photoShown = false; }
  lastFrameAt = millis();
  liveFrom = IPAddress(rxFrom);
  if (on) { render(bufs[idx]); wakeAfterDraw(wasAsleep); }   // the picture first, then the light
  frames++;
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
  if (!on || !data || !len) return;
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
  if (!photoShown || idleShown) { gfx->fillScreen(BLACK); idleShown = false; }
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
  idleShown = false; photoShown = false; forceRedraw = true;
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
  if (photoShown && live()) { gfx->fillRect(0, TFT_H - OVERLAY_H, TFT_W, OVERLAY_H, BLACK); }   // the next photo fills it
  else if (live()) render(bufs[drawIdx]);
  else if (idleShown) { /* boot / setup screen: the rain carries on */ }
  else goDark();
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
       "<label><input type=checkbox name=follow " + (S.follow ? "checked" : "") + "> Follow the brightness Signinator sends</label>"
       "<label>Brightest (10-255)<input type=number min=10 max=255 name=blmax value=" + String(S.blMax) + "></label>"
       "<label>Dimmest (0-255)<input type=number min=0 max=255 name=blmin value=" + String(S.blMin) + "></label></fieldset>"
       "<fieldset><legend>Wi-Fi</legend>"
       "<label>Network<input type=text name=ssid value=\"" + esc(S.ssid) + "\"></label>"
       "<label>Password<input type=password name=pass placeholder='" + (S.pass.length() ? "saved; type to change" : "") + "'></label>"
       "<label>Host name<input type=text name=host value=\"" + esc(S.host) + "\"></label></fieldset>"
       "<button>Save</button></form>"
       "<form method=post action=/update enctype=multipart/form-data><fieldset><legend>Update firmware</legend>"
       "<input type=file name=update accept=.bin> <button>Upload</button>"
       "<p class=note>Or use Lickety Split to update over Wi-Fi.</p></fieldset></form></body></html>";
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
  saveSettings();
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
      if (!on) { gfx->fillScreen(BLACK); idleShown = false; forceRedraw = true; }
      else if (!live() && !idleShown) goDark();
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
    if (!live() && !idleShown) goDark();
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
  setupWeb();
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

  if (on && !live() && lastFrameAt && !idleShown && !asleep && !overlayUntil) goDark();   // stream stopped: dark
  if (idleShown && on && !overlayUntil && !asleep) {          // the rain keeps falling; panel follows Wi-Fi
    mrStep();
    static uint32_t panelAt = 0;
    if (now - panelAt > 1000) { panelAt = now; showIdle(); }
    // nothing to show for a while (and not setting up): go to sleep until pixels, a photo or a tap
    bool settingUp = apOn && WiFi.status() != WL_CONNECTED;
    if (!idleSince) idleSince = now;
    if (!settingUp && now - idleSince > IDLE_SLEEP_MS) goDark();   // after power-on: dark until there's something
  }

  static uint32_t touchAt = 0;
  if (now - touchAt > 40) {
    touchAt = now;
    static bool wasDown = false;
    static uint32_t upAt = 0;
    int t = touched();
    if (t == 1 && !wasDown && now - upAt > 250) {
      if (asleep) {                                        // dark: the info strip (song, IP) on black
        gfx->fillScreen(BLACK); asleep = false; backlight(); showOverlay();
      } else {
        if (on) { digitalWrite(TFT_BCKL, LOW); delayMicroseconds(200); digitalWrite(TFT_BCKL, HIGH); }   // a tap re-arms the light too
        if (overlayUntil) hideOverlay(); else if (on) showOverlay();
      }
    }
    if (t == 0 && wasDown) upAt = now;
    if (t >= 0) wasDown = t == 1;
  }
  if (overlayUntil && now > overlayUntil) hideOverlay();

  // The backlight driver sometimes shuts itself off (the picture keeps changing in the dark) and stays off
  // until its enable pin is switched off and on. Re-arm it every 30 s with a 0.2 ms blip (too short to see).
  static uint32_t rearmAt = 0;
  if (on && blDuty > 0 && !asleep && now - rearmAt >= 30000) {
    rearmAt = now;
    digitalWrite(TFT_BCKL, LOW);
    delayMicroseconds(200);
    digitalWrite(TFT_BCKL, HIGH);
  }
  delay(1);
}
