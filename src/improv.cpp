#include "improv.h"

namespace improv {

enum : uint8_t { T_STATE = 1, T_ERROR = 2, T_RPC = 3, T_RESULT = 4 };
enum : uint8_t { S_READY = 2, S_PROVISIONING = 3, S_PROVISIONED = 4 };
enum : uint8_t { E_NONE = 0, E_INVALID = 1, E_UNKNOWN_CMD = 2, E_NO_CONNECT = 3 };
enum : uint8_t { C_WIFI = 1, C_STATE = 2, C_INFO = 3, C_SCAN = 4 };

static Hooks H;
static uint8_t buf[256];
static size_t pos = 0;

static void send(uint8_t type, const uint8_t *data, uint8_t len) {
  uint8_t out[9 + 256 + 1];
  memcpy(out, "IMPROV", 6);
  out[6] = 1;  // protocol version
  out[7] = type;
  out[8] = len;
  memcpy(out + 9, data, len);
  uint8_t sum = 0;
  for (size_t i = 0; i < 9u + len; i++) sum += out[i];
  out[9 + len] = sum;
  Serial.write(out, 10 + len);
  Serial.write('\n');
}
static void state(uint8_t s) { send(T_STATE, &s, 1); }
static void error(uint8_t e) { send(T_ERROR, &e, 1); }

// RPC result: [command, total length, (len, string)...]
static void result(uint8_t cmd, std::initializer_list<String> strs) {
  uint8_t d[250];
  size_t n = 2;
  for (const String &s : strs) {
    size_t l = s.length() > 60 ? 60 : s.length();
    if (n + 1 + l > sizeof(d)) break;
    d[n++] = l;
    memcpy(d + n, s.c_str(), l);
    n += l;
  }
  d[0] = cmd;
  d[1] = n - 2;
  send(T_RESULT, d, n);
}

void begin(const Hooks &h) { H = h; }

static void handle(uint8_t type, const uint8_t *data, uint8_t len) {
  if (type != T_RPC || len < 2) return error(E_INVALID);
  uint8_t cmd = data[0];
  const uint8_t *p = data + 2;
  size_t left = data[1];
  if (left + 2 > len) return error(E_INVALID);
  if (cmd == C_STATE) {
    String u = H.url();
    state(u.length() ? S_PROVISIONED : S_READY);
    if (u.length()) result(C_STATE, {u});
  } else if (cmd == C_INFO) {
    result(C_INFO, {H.firmware, H.version, H.chip, H.name});
  } else if (cmd == C_SCAN) {
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; i++)
      result(C_SCAN, {WiFi.SSID(i), String(WiFi.RSSI(i)), WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "NO" : "YES"});
    result(C_SCAN, {});
    WiFi.scanDelete();
  } else if (cmd == C_WIFI) {
    if (left < 1 || p[0] + 1u > left) return error(E_INVALID);
    String ssid((const char *)p + 1, p[0]);
    size_t o = 1 + p[0];
    String pass;
    if (o < left) {
      uint8_t pl = p[o];
      if (o + 1 + pl > left) return error(E_INVALID);
      pass = String((const char *)p + o + 1, pl);
    }
    state(S_PROVISIONING);
    if (H.connect(ssid, pass)) {
      state(S_PROVISIONED);
      result(C_WIFI, {H.url()});
    } else {
      error(E_NO_CONNECT);
      state(S_READY);
    }
  } else {
    error(E_UNKNOWN_CMD);
  }
}

void loop() {
  while (Serial.available()) {
    uint8_t c = Serial.read();
    if (pos < 6) {                      // match the "IMPROV" header byte by byte
      if (c == (uint8_t)"IMPROV"[pos]) buf[pos++] = c;
      else pos = (c == 'I') ? (buf[0] = c, 1) : 0;
      continue;
    }
    buf[pos++] = c;
    if (pos >= 9) {
      uint8_t len = buf[8];
      if (pos == 10u + len) {
        uint8_t sum = 0;
        for (size_t i = 0; i < 9u + len; i++) sum += buf[i];
        if (sum == buf[9 + len] && buf[6] == 1) handle(buf[7], buf + 9, len);
        else error(E_INVALID);
        pos = 0;
      }
    }
    if (pos >= sizeof(buf)) pos = 0;
  }
}

}  // namespace improv
