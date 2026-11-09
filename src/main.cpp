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
#include "board.h"
#include "panel_init.h"
#include "improv.h"

#ifndef FW_VERSION
#define FW_VERSION ""
#endif
static const char *VERSION = sizeof(FW_VERSION) > 1 ? FW_VERSION : "dev";
static const int MAX_W = 128, MAX_H = 128;
static const uint32_t LIVE_TIMEOUT_MS = 2500;

// ---------- settings ----------
struct Settings {
  String ssid, pass, name, host;
  int w = 32, h = 32;
  bool unlit = true;        // draw unlit LEDs as dark grey dots
  bool follow = true;       // backlight follows the brightness the sender sets
  int blMax = 255, blMin = 25;
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
  S.blMin = constrain(prefs.getInt("blmin", 25), 0, 255);
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
static uint8_t *rx, *shown;          // received / drawn RGB, W*H*3
static int cell, dot, ox, oy;        // layout of the dots on the screen
static uint8_t *mask;                // cell*cell: 1 = inside the round dot
static bool forceRedraw = true;
static int overlayTop = TFT_H;       // rows from here down belong to the overlay while it shows

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

static void layout() {
  cell = min(TFT_W / S.w, TFT_H / S.h);
  if (cell < 1) cell = 1;
  int gap = cell >= 6 ? max(1, cell / 7) : 0;
  dot = cell - gap;
  ox = (TFT_W - S.w * cell) / 2;
  oy = (TFT_H - S.h * cell) / 2;
  free(mask);
  mask = (uint8_t *)malloc(cell * cell);
  float r = dot / 2.0f, c = (cell - 1) / 2.0f;
  for (int y = 0; y < cell; y++)
    for (int x = 0; x < cell; x++) {
      float dx = x - c, dy = y - c;
      mask[y * cell + x] = (dot >= 4) ? (dx * dx + dy * dy <= r * r) : (x < dot && y < dot);
    }
  size_t n = (size_t)S.w * S.h * 3;
  free(rx);
  free(shown);
  rx = (uint8_t *)ps_calloc(n, 1);
  shown = (uint8_t *)ps_calloc(n, 1);
  gfx->fillScreen(BLACK);
  forceRedraw = true;
}

// Draw the LEDs that changed. Returns quickly when nothing did.
static void render() {
  int yMin = TFT_H, yMax = -1;
  for (int ly = 0; ly < S.h; ly++) {
    int py = oy + ly * cell;
    if (py + cell > overlayTop) break;
    for (int lx = 0; lx < S.w; lx++) {
      int i = (ly * S.w + lx) * 3;
      uint8_t r = rx[i], g = rx[i + 1], b = rx[i + 2];
      if (!forceRedraw && r == shown[i] && g == shown[i + 1] && b == shown[i + 2]) continue;
      shown[i] = r; shown[i + 1] = g; shown[i + 2] = b;
      uint16_t c = (S.unlit && r < 10 && g < 10 && b < 10) ? rgb565(24, 24, 24) : rgb565(r, g, b);
      int px = ox + lx * cell;
      for (int y = 0; y < cell; y++) {
        uint16_t *row = fb + (py + y) * TFT_W + px;
        const uint8_t *m = mask + y * cell;
        for (int x = 0; x < cell; x++) row[x] = m[x] ? c : BLACK;
      }
      yMin = min(yMin, py);
      yMax = max(yMax, py + cell - 1);
    }
  }
  forceRedraw = false;
  if (yMax >= yMin) Cache_WriteBack_Addr((uint32_t)(fb + yMin * TFT_W), (yMax - yMin + 1) * TFT_W * 2);
}

// ---------- backlight ----------
static bool on = true;
static int bri = 128;                 // last brightness the sender asked for (WLED "bri")
static void backlight() {
  int v = 0;
  if (on) v = S.follow ? map(bri, 0, 255, S.blMin, S.blMax) : S.blMax;
  ledcWrite(0, constrain(v, 0, 255));
}

// ---------- network ----------
static WebServer web(80);
static DNSServer dns;
static WiFiUDP udp;
static bool apOn = false;
static uint32_t wifiLostAt = 0;
static String apName;

static String url() { return WiFi.status() == WL_CONNECTED ? "http://" + WiFi.localIP().toString() + "/" : ""; }

static bool joinWifi(const String &ssid, const String &pass, bool save) {
  WiFi.mode(apOn ? WIFI_AP_STA : WIFI_STA);
  WiFi.setHostname(S.host.c_str());
  WiFi.begin(ssid.c_str(), pass.c_str());
  for (int i = 0; i < 150 && WiFi.status() != WL_CONNECTED; i++) {
    delay(100);
    if (i % 10 == 0) gfx->fillCircle(TFT_W / 2 - 60 + (i / 10) * 8, TFT_H - 40, 3, DARKGREY);
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
static bool idleShown = false;

static void handleDdp() {
  static uint8_t pkt[1500];
  int n;
  while ((n = udp.parsePacket()) > 0) {
    if (n > (int)sizeof(pkt)) { udp.flush(); continue; }
    udp.read(pkt, n);
    if (n < 10 || (pkt[0] & 0xC0) != 0x40) continue;       // DDP version 1 only
    int hdr = (pkt[0] & 0x10) ? 14 : 10;                     // timecode adds 4 bytes
    uint32_t off = ((uint32_t)pkt[4] << 24) | (pkt[5] << 16) | (pkt[6] << 8) | pkt[7];
    uint32_t len = (pkt[8] << 8) | pkt[9];
    if (pkt[3] == 251 || pkt[3] == 254 || hdr + (int)len > n) continue;  // config/status queries: ignore
    uint32_t total = (uint32_t)S.w * S.h * 3;
    if (off < total) memcpy(rx + off, pkt + hdr, min(len, total - off));
    if (pkt[0] & 0x01) {                                      // PUSH: show the frame
      if (idleShown) { gfx->fillScreen(BLACK); forceRedraw = true; idleShown = false; }
      lastFrameAt = millis();
      liveFrom = udp.remoteIP();
      if (on) render();
      frames++;
    }
  }
}

static bool live() { return lastFrameAt && millis() - lastFrameAt < LIVE_TIMEOUT_MS; }

static void centered(const String &s, int y, int size, uint16_t color) {
  gfx->setTextSize(size);
  gfx->setTextColor(color);
  int w = s.length() * 6 * size;
  gfx->setCursor(max(0, (TFT_W - w) / 2), y);
  gfx->print(s);
}

static void showIdle() {
  gfx->fillScreen(BLACK);
  centered(S.name, 170, 4, WHITE);
  if (WiFi.status() == WL_CONNECTED) {
    centered(WiFi.localIP().toString(), 230, 3, CYAN);
    centered("Waiting for pixels (DDP, port 4048)", 280, 2, DARKGREY);
  } else if (apOn) {
    centered("Join Wi-Fi \"" + apName + "\"", 230, 2, YELLOW);
    centered("then open http://192.168.4.1", 260, 2, YELLOW);
  } else {
    centered("Connecting to Wi-Fi...", 230, 2, DARKGREY);
  }
  centered(String("v") + VERSION, 440, 1, DARKGREY);
  idleShown = true;
}

// ---------- overlay (tap the screen) ----------
static String npTitle, npArtist;
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
  if (live()) render(); else showIdle();
}

// ---------- touch (GT911) ----------
static uint8_t touchAddr = 0;
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
       "<label>Width (LEDs)<input type=number min=1 max=128 name=w value=" + String(S.w) + "></label>"
       "<label>Height (LEDs)<input type=number min=1 max=128 name=h value=" + String(S.h) + "></label>"
       "<label><input type=checkbox name=unlit " + (S.unlit ? "checked" : "") + "> Show unlit LEDs as dark dots</label>"
       "<p class=note>Set the same size as the LED sign. Signinator reads it from /json/info like it does from WLED.</p></fieldset>"
       "<fieldset><legend>Backlight</legend>"
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
  if (sizeChanged) { layout(); showIdle(); }
  if (wifiChanged) { delay(300); ESP.restart(); }
}

static void handleUpdateDone() {
  bool ok = !Update.hasError();
  web.sendHeader("Access-Control-Allow-Origin", "*");
  web.send(ok ? 200 : 500, "text/plain", ok ? "Update done, restarting." : "Update failed.");
  if (ok) { delay(500); ESP.restart(); }
}
static void handleUpdateUpload() {
  HTTPUpload &u = web.upload();
  if (u.status == UPLOAD_FILE_START) {
    gfx->fillScreen(BLACK);
    centered("Updating firmware...", 220, 3, YELLOW);
    Update.begin(UPDATE_SIZE_UNKNOWN);
  } else if (u.status == UPLOAD_FILE_WRITE) {
    Update.write(u.buf, u.currentSize);
  } else if (u.status == UPLOAD_FILE_END) {
    Update.end(true);
  } else if (u.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
  }
}

static void handleState() {
  if (web.method() == HTTP_POST) {
    JsonDocument in;
    if (deserializeJson(in, web.arg("plain")) == DeserializationError::Ok) {
      if (in["on"].is<bool>()) on = in["on"].as<bool>();
      else if (in["on"].is<const char *>() && String(in["on"].as<const char *>()) == "t") on = !on;
      if (in["bri"].is<int>()) bri = constrain(in["bri"].as<int>(), 0, 255);
      backlight();
      if (!on) { gfx->fillScreen(BLACK); idleShown = false; forceRedraw = true; }
      else if (!live()) showIdle();
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
  ledcSetup(0, 5000, 8);
  ledcAttachPin(TFT_BCKL, 0);
  ledcWrite(0, 0);

  bus = new Arduino_SWSPI(GFX_NOT_DEFINED, TFT_CS, TFT_SCLK, TFT_MOSI, GFX_NOT_DEFINED);
  rgbpanel = new Arduino_ESP32RGBPanel(TFT_DE, TFT_VSYNC, TFT_HSYNC, TFT_PCLK, TFT_R0, TFT_R1, TFT_R2, TFT_R3, TFT_R4,
                                       TFT_G0, TFT_G1, TFT_G2, TFT_G3, TFT_G4, TFT_G5, TFT_B0, TFT_B1, TFT_B2, TFT_B3,
                                       TFT_B4, 1, 10, 8, 50, 1, 10, 8, 20);
  gfx = new Arduino_RGB_Display(TFT_W, TFT_H, rgbpanel, 0, true, bus, GFX_NOT_DEFINED, st7701_4848s040_init,
                                sizeof(st7701_4848s040_init));
  gfx->begin(GFX_NOT_DEFINED);
  fb = gfx->getFramebuffer();
  layout();
  backlight();
  touchBegin();

  uint8_t mac[6];
  WiFi.macAddress(mac);
  char ap[32];
  snprintf(ap, sizeof ap, "Matrixinator-%02X%02X", mac[4], mac[5]);
  apName = ap;
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
  udp.begin(4048);
  setupWeb();
  showIdle();
}

void loop() {
  improv::loop();
  handleDdp();
  web.handleClient();
  if (apOn) dns.processNextRequest();

  uint32_t now = millis();
  if (now - fpsAt >= 1000) { fps = frames * 1000 / (now - fpsAt); frames = 0; fpsAt = now; }

  // Wi-Fi: keep trying; open the setup hotspot after 30 s without a connection
  if (WiFi.status() == WL_CONNECTED) {
    wifiLostAt = 0;
    if (apOn && WiFi.softAPgetStationNum() == 0) { stopAp(); if (!live()) showIdle(); }
  } else if (S.ssid.length()) {
    if (!wifiLostAt) { wifiLostAt = now; WiFi.reconnect(); }
    else if (now - wifiLostAt > 30000 && !apOn) { startAp(); if (!live()) showIdle(); }
  }

  if (on && !live() && lastFrameAt && !idleShown && !overlayUntil) showIdle();   // stream stopped

  static uint32_t touchAt = 0;
  if (now - touchAt > 40) {
    touchAt = now;
    static bool wasDown = false;
    static uint32_t upAt = 0;
    int t = touched();
    if (t == 1 && !wasDown && now - upAt > 250) { if (overlayUntil) hideOverlay(); else if (on) showOverlay(); }
    if (t == 0 && wasDown) upAt = now;
    if (t >= 0) wasDown = t == 1;
  }
  if (overlayUntil && now > overlayUntil) hideOverlay();
  delay(1);
}
