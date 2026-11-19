// Art deco clock faces. Everything is drawn with signed distances, so edges are smooth (anti-aliased) and a shape
// made of several parts (a hand, a numeral) is blended once, with no seams where the parts overlap.
#include "clock.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <initializer_list>
#include <vector>

namespace clk {

static const float CX = W / 2.0f, CY = H / 2.0f;
static const float PI2 = 6.28318531f;

// ---------- colour ----------
struct C3 { float r, g, b; };
static inline C3 mix(C3 a, C3 b, float t) { return {a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t}; }
static inline C3 mul(C3 a, float k) { return {a.r * k, a.g * k, a.b * k}; }
static inline float clamp01(float v) { return v < 0 ? 0 : v > 1 ? 1 : v; }
static inline uint16_t pack(C3 c) {
  int r = (int)(c.r + 0.5f), g = (int)(c.g + 0.5f), b = (int)(c.b + 0.5f);
  r = r < 0 ? 0 : r > 255 ? 255 : r; g = g < 0 ? 0 : g > 255 ? 255 : g; b = b < 0 ? 0 : b > 255 ? 255 : b;
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
static inline C3 unpack(uint16_t v) {
  return {((v >> 11) & 31) * (255.0f / 31), ((v >> 5) & 63) * (255.0f / 63), (v & 31) * (255.0f / 31)};
}
static inline void blend(uint16_t &d, C3 c, float a) {
  if (a <= 0.002f) return;
  d = a >= 0.998f ? pack(c) : pack(mix(unpack(d), c, a));
}
// coverage of a pixel whose centre is `d` pixels outside an edge
static inline float cov(float d) { return clamp01(0.5f - d); }

// ---------- shapes: unions of capsules, discs, rounded boxes and convex polygons ----------
struct Seg { float ax, ay, bx, by, r; };
struct Disc { float x, y, r; };
struct Box { float x, y, hx, hy, rr; };
struct Poly { int n; float x[8], y[8], nx[8], ny[8], c[8]; };

struct Shape {
  std::vector<Seg> segs;
  std::vector<Disc> discs;
  std::vector<Box> boxes;
  std::vector<Poly> polys;
  float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
  void grow(float x, float y, float r) {
    if (x - r < x0) x0 = x - r;
    if (y - r < y0) y0 = y - r;
    if (x + r > x1) x1 = x + r;
    if (y + r > y1) y1 = y + r;
  }
  void seg(float ax, float ay, float bx, float by, float r) {
    segs.push_back({ax, ay, bx, by, r}); grow(ax, ay, r); grow(bx, by, r);
  }
  void disc(float x, float y, float r) { discs.push_back({x, y, r}); grow(x, y, r); }
  void box(float x, float y, float hx, float hy, float rr) {
    boxes.push_back({x, y, hx, hy, rr}); grow(x - hx, y - hy, 0); grow(x + hx, y + hy, 0);
  }
  void poly(const float *px, const float *py, int n) {   // convex, either winding
    Poly p;
    p.n = n;
    float mx = 0, my = 0;
    for (int i = 0; i < n; i++) { p.x[i] = px[i]; p.y[i] = py[i]; mx += px[i]; my += py[i]; grow(px[i], py[i], 0); }
    mx /= n; my /= n;
    for (int i = 0; i < n; i++) {
      int j = (i + 1) % n;
      float ex = p.x[j] - p.x[i], ey = p.y[j] - p.y[i], l = sqrtf(ex * ex + ey * ey);
      if (l < 1e-6f) l = 1e-6f;
      float nx = ey / l, ny = -ex / l;
      if ((mx - p.x[i]) * nx + (my - p.y[i]) * ny > 0) { nx = -nx; ny = -ny; }   // point outwards
      p.nx[i] = nx; p.ny[i] = ny; p.c[i] = p.x[i] * nx + p.y[i] * ny;
    }
    polys.push_back(p);
  }
  float sdf(float px, float py) const {
    float d = 1e9f;
    for (const Seg &s : segs) {
      float bax = s.bx - s.ax, bay = s.by - s.ay, pax = px - s.ax, pay = py - s.ay;
      float l2 = bax * bax + bay * bay;
      float h = l2 > 0 ? clamp01((pax * bax + pay * bay) / l2) : 0;
      float dx = pax - bax * h, dy = pay - bay * h;
      float v = sqrtf(dx * dx + dy * dy) - s.r;
      if (v < d) d = v;
    }
    for (const Disc &c : discs) {
      float dx = px - c.x, dy = py - c.y, v = sqrtf(dx * dx + dy * dy) - c.r;
      if (v < d) d = v;
    }
    for (const Box &b : boxes) {
      float qx = fabsf(px - b.x) - (b.hx - b.rr), qy = fabsf(py - b.y) - (b.hy - b.rr);
      float ox = qx > 0 ? qx : 0, oy = qy > 0 ? qy : 0;
      float in = qx > qy ? qx : qy;
      float v = sqrtf(ox * ox + oy * oy) + (in < 0 ? in : 0) - b.rr;
      if (v < d) d = v;
    }
    for (const Poly &p : polys) {
      float v = -1e9f;
      for (int i = 0; i < p.n; i++) {
        float e = p.nx[i] * px + p.ny[i] * py - p.c[i];
        if (e > v) v = e;
      }
      if (v < d) d = v;
    }
    return d;
  }
};

// grow > 0 makes the shape fatter (outlines); soft > 1 blurs the edge (shadows); ox/oy shift it.
static void fill(uint16_t *buf, const Shape &s, C3 c, float alpha = 1, float grow = 0, float soft = 1,
                 float ox = 0, float oy = 0) {
  if (s.x1 < s.x0) return;
  float pad = grow + soft + 1.5f;
  int xa = (int)floorf(s.x0 + ox - pad), xb = (int)ceilf(s.x1 + ox + pad);
  int ya = (int)floorf(s.y0 + oy - pad), yb = (int)ceilf(s.y1 + oy + pad);
  if (xa < 0) xa = 0;
  if (ya < 0) ya = 0;
  if (xb > W - 1) xb = W - 1;
  if (yb > H - 1) yb = H - 1;
  for (int y = ya; y <= yb; y++) {
    uint16_t *row = buf + y * W;
    float py = y + 0.5f - oy;
    for (int x = xa; x <= xb; x++) {
      float d = s.sdf(x + 0.5f - ox, py) - grow;
      float a = clamp01(0.5f - d / soft) * alpha;
      if (a > 0.002f) blend(row[x], c, a);
    }
  }
}

// ---------- positions on the dial (clock angle: 0 = 12 o'clock, clockwise, radians) ----------
static inline float px(float a, float r) { return CX + r * sinf(a); }
static inline float py(float a, float r) { return CY - r * cosf(a); }

// A part of a hand in its own coordinates: t along the hand (outwards), s across it.
struct HandBuilder {
  Shape sh;
  float sa, ca;
  explicit HandBuilder(float a) : sa(sinf(a)), ca(cosf(a)) {}
  float X(float t, float s) const { return CX + t * sa + s * ca; }
  float Y(float t, float s) const { return CY - t * ca + s * sa; }
  void poly(std::initializer_list<float> ts) {     // t0,s0, t1,s1, ...
    float x[8], y[8];
    int n = 0;
    const float *v = ts.begin();
    for (size_t i = 0; i + 1 < ts.size() && n < 8; i += 2, n++) { x[n] = X(v[i], v[i + 1]); y[n] = Y(v[i], v[i + 1]); }
    sh.poly(x, y, n);
  }
  void rect(float t0, float t1, float w) { poly({t0, -w / 2, t1, -w / 2, t1, w / 2, t0, w / 2}); }
  void disc(float t, float r) { sh.disc(X(t, 0), Y(t, 0), r); }
  void line(float t0, float t1, float r) { sh.seg(X(t0, 0), Y(t0, 0), X(t1, 0), Y(t1, 0), r); }
};

// ---------- the numeral font: tall, condensed, geometric strokes (10 units high) ----------
struct Glyph { float w; std::vector<std::vector<float>> strokes; };   // stroke = x0,y0,x1,y1,...
static std::vector<float> &arc(std::vector<float> &v, float cx, float cy, float rx, float ry, float a0, float a1) {
  int n = (int)(fabsf(a1 - a0) / 10) + 2;   // a point every <= 10 degrees
  for (int i = 0; i <= n; i++) {
    float a = (a0 + (a1 - a0) * i / n) * (PI2 / 360);
    v.push_back(cx + rx * cosf(a)); v.push_back(cy + ry * sinf(a));   // 0 = right, 90 = down
  }
  return v;
}
static const Glyph &glyph(char ch) {
  static Glyph g[128];
  static bool built = false;
  if (!built) {
    built = true;
    for (auto &x : g) x.w = 3;                                            // unknown / space
    {  // 0 and O: a stadium
      std::vector<float> s;
      arc(s, 2.5f, 2.5f, 2.5f, 2.5f, 180, 360);
      arc(s, 2.5f, 7.5f, 2.5f, 2.5f, 0, 180);
      s.push_back(s[0]); s.push_back(s[1]);
      g['0'] = {5, {s}};
      g['O'] = g['0'];
    }
    g['1'] = {3.2f, {{0.4f, 1.8f, 2.0f, 0, 2.0f, 10}}};
    { std::vector<float> s; arc(s, 2.5f, 2.6f, 2.5f, 2.5f, 190, 380); s.insert(s.end(), {0, 10, 5, 10}); g['2'] = {5, {s}}; }
    { std::vector<float> s = {0.3f, 0, 4.8f, 0, 2.5f, 4.45f}; arc(s, 2.5f, 7.2f, 2.75f, 2.75f, 270, 515); g['3'] = {5, {s}}; }
    g['4'] = {5, {{3.7f, 10, 3.7f, 0, 0, 6.9f, 5, 6.9f}}};
    { std::vector<float> s = {4.8f, 0, 0.7f, 0, 0.45f, 4.8f}; arc(s, 2.5f, 7.25f, 2.75f, 2.75f, 238, 515); g['5'] = {5, {s}}; }
    {
      std::vector<float> s;
      arc(s, 2.5f, 2.5f, 2.5f, 2.5f, 330, 180);
      s.insert(s.end(), {0, 7.5f});
      arc(s, 2.5f, 7.5f, 2.5f, 2.5f, 180, 540);
      g['6'] = {5, {s}};
      std::vector<float> n = s;                                           // 9 = 6 turned round
      for (size_t i = 0; i < n.size(); i += 2) { n[i] = 5 - n[i]; n[i + 1] = 10 - n[i + 1]; }
      g['9'] = {5, {n}};
    }
    g['7'] = {5, {{0, 0, 5, 0, 1.6f, 10}}};
    {
      std::vector<float> a, b;
      arc(a, 2.5f, 2.35f, 2.2f, 2.35f, 0, 360);
      arc(b, 2.5f, 7.35f, 2.5f, 2.65f, 0, 360);
      g['8'] = {5, {a, b}};
    }
    g['I'] = {0, {{0, 0, 0, 10}}};
    g['N'] = {5, {{0, 10, 0, 0, 5, 10, 5, 0}}};
    { std::vector<float> s = {0, 0, 0, 7.5f}; arc(s, 2.5f, 7.5f, 2.5f, 2.5f, 180, 0); s.insert(s.end(), {5, 0}); g['U'] = {5, {s}}; }
    g['T'] = {5, {{0, 0, 5, 0}, {2.5f, 0, 2.5f, 10}}};
    g['V'] = {5, {{0, 0, 2.5f, 10, 5, 0}}};
    g['X'] = {5, {{0, 0, 5, 10}, {5, 0, 0, 10}}};
    g['-'] = {3.6f, {{0.2f, 5.6f, 3.4f, 5.6f}}};
    { std::vector<float> s; arc(s, 1.3f, 1.3f, 1.3f, 1.3f, 0, 360); g['*'] = {2.6f, {s}}; }   // degree sign
  }
  return g[(unsigned char)ch & 127];
}
// Add a line of text, centred on (cx, cy): h = letter height in pixels, th = stroke width, gap = between letters (units).
static void text(Shape &s, const char *t, float cx, float cy, float h, float th, float gap = 2.2f) {
  float k = h / 10, wsum = 0;
  int n = (int)strlen(t);
  for (int i = 0; i < n; i++) wsum += glyph(t[i]).w + (i ? gap : 0);
  float x = cx - wsum * k / 2, y = cy - h / 2;
  for (int i = 0; i < n; i++) {
    const Glyph &g = glyph(t[i]);
    for (const auto &st : g.strokes)
      for (size_t j = 0; j + 3 < st.size(); j += 2)
        s.seg(x + st[j] * k, y + st[j + 1] * k, x + st[j + 2] * k, y + st[j + 3] * k, th / 2);
    x += (g.w + gap) * k;
  }
}

// ---------- faces ----------
struct Theme {
  C3 ink, label, window, windowEdge, moonLit, moonDark, hands, handEdge, second, hub, hubDot;
};
static const Theme THEMES[FACE_COUNT] = {
    // Gatsby: black and gold
    {{232, 196, 110}, {176, 140, 70}, {10, 9, 8}, {196, 160, 80}, {238, 226, 190}, {34, 30, 26},
     {228, 192, 104}, {70, 50, 14}, {200, 40, 36}, {232, 196, 110}, {30, 24, 14}},
    // Skyline: steel and silver
    {{222, 226, 234}, {150, 158, 172}, {20, 22, 26}, {170, 176, 188}, {236, 236, 228}, {40, 42, 48},
     {226, 230, 238}, {14, 15, 18}, {214, 48, 48}, {226, 230, 238}, {20, 22, 26}},
    // Ivory: cream, black and jade
    {{22, 22, 22}, {20, 120, 102}, {248, 242, 228}, {22, 22, 22}, {250, 244, 222}, {58, 60, 64},
     {20, 20, 20}, {20, 20, 20}, {18, 128, 108}, {20, 20, 20}, {18, 128, 108}},
    // Midnight: navy and silver, quiet for the night
    {{196, 204, 224}, {120, 134, 170}, {6, 10, 24}, {120, 134, 170}, {226, 228, 236}, {26, 32, 52},
     {200, 208, 226}, {8, 12, 30}, {214, 180, 96}, {200, 208, 226}, {10, 16, 40}},
};

static const C3 GOLD = {212, 172, 84}, SILVER = {196, 202, 214}, BRASS = {196, 156, 72};

static inline float band(float r, float r0, float r1) { return clamp01(fminf(r - r0, r1 - r) + 0.5f); }
// Polished metal ring: lighter towards the top left, a rounded profile across the ring.
static C3 metal(C3 base, float r, float r0, float r1, float a) {
  float m = (r - r0) / (r1 - r0) * 2 - 1;                 // -1 .. 1 across
  float light = 0.72f + 0.34f * cosf(a - 5.5f);           // 5.5 rad = towards 10 o'clock
  float prof = 1.08f - 0.32f * m * m;
  return mul(base, light * prof);
}
// Wedges round the dial: returns 0..1, how much of this pixel is in an "odd" wedge.
static float wedge(float a, float r, int n, float phase = 0) {
  float u = a / PI2 * n + phase;
  float f = u - floorf(u);
  int odd = ((int)floorf(u)) & 1;
  float edge = fminf(f, 1 - f) * PI2 * r / n;              // pixels to the nearest wedge edge
  float c = clamp01(0.5f + edge);
  return odd ? c : 1 - c;
}

static void background(uint16_t *buf, int face) {
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      float dx = x + 0.5f - CX, dy = y + 0.5f - CY;
      float r = sqrtf(dx * dx + dy * dy);
      float a = atan2f(dx, -dy);
      if (a < 0) a += PI2;
      C3 c = {0, 0, 0};
      switch (face) {
        case 0: {  // Gatsby: a black sunburst in a gold bezel
          C3 dark = {14, 12, 9}, light = {34, 27, 13};
          float rays = wedge(a, r, 48);
          float fade = clamp01((r - 18) / 40);
          c = mix(mix(dark, light, 0.5f), mix(dark, light, rays), fade);
          c = mul(c, 1.05f - 0.25f * (r / 220) * (r / 220));
          c = mix(c, GOLD, band(r, 118.5f, 120.5f));
          c = mix(c, mul(GOLD, 0.6f), band(r, 124, 125));
          c = mix(c, mul(GOLD, 0.95f), band(r, 218.5f, 221.5f));
          c = mix(c, {4, 4, 4}, band(r, 221.5f, 226));
          c = mix(c, metal(GOLD, r, 226, 238, a), band(r, 226, 238));
          c = mix(c, {3, 3, 3}, clamp01(r - 238 + 0.5f));
          break;
        }
        case 1: {  // Skyline: charcoal, faint rays and a crown of stepped silver triangles
          float t = r / 222;
          c = mix({46, 49, 56}, {22, 24, 28}, t * t);
          c = mix(c, mul(c, 1.12f), wedge(a, r, 120) * clamp01((r - 30) / 40));
          for (int tier = 0; tier < 2; tier++) {
            float r0 = tier ? 158 : 140, r1 = tier ? 177 : 159;
            if (r < r0 - 1 || r > r1 + 1) continue;
            int n = 24;
            float u = a / PI2 * n + (tier ? 0.5f : 0);
            float f = u - floorf(u) - 0.5f;
            float tt = clamp01((r - r0) / (r1 - r0));
            float halfw = 0.5f * (1 - tt);
            float dpx = (halfw - fabsf(f)) * PI2 * r / n;          // pixels inside the triangle
            float inside = clamp01(dpx + 0.5f) * band(r, r0, r1);
            C3 s = mul(SILVER, (tier ? 0.62f : 0.82f) + 0.25f * (1 - tt));
            c = mix(c, s, inside);
          }
          c = mix(c, mul(SILVER, 0.8f), band(r, 137, 138.5f));
          c = mix(c, mul(SILVER, 0.8f), band(r, 179, 180.5f));
          c = mix(c, mul(SILVER, 0.55f), band(r, 46, 47));
          c = mix(c, {12, 13, 15}, band(r, 222, 228));
          c = mix(c, metal(SILVER, r, 228, 238, a), band(r, 228, 238));
          c = mix(c, {3, 3, 3}, clamp01(r - 238 + 0.5f));
          break;
        }
        case 2: {  // Ivory: cream with a black chapter ring and a jade band, in brass
          float t = r / 200;
          c = mix({242, 235, 216}, {222, 212, 186}, t * t);
          c = mix(c, {24, 24, 24}, band(r, 123, 124.5f));
          c = mix(c, {24, 132, 112}, band(r, 126, 134));
          c = mix(c, {24, 24, 24}, band(r, 135.5f, 137));
          c = mix(c, {16, 16, 16}, band(r, 200, 228));
          c = mix(c, metal(BRASS, r, 228, 238, a), band(r, 228, 238));
          c = mix(c, {3, 3, 3}, clamp01(r - 238 + 0.5f));
          break;
        }
        default: {  // Midnight: navy, a fan of fine lines
          float t = r / 222;
          c = mix({20, 32, 70}, {6, 10, 28}, t);
          float u = a / PI2 * 12, f = u - floorf(u);
          float lineD = fminf(f, 1 - f) * PI2 * r / 12;              // pixels to the nearest fan line
          float fan = clamp01(0.9f - lineD) * band(r, 50, 196) * 0.55f;
          c = mix(c, {70, 90, 140}, fan);
          c = mix(c, {60, 78, 124}, band(r, 48.5f, 49.5f));
          c = mix(c, {70, 90, 140}, band(r, 196, 197.5f));
          c = mix(c, {4, 6, 14}, band(r, 222, 228));
          c = mix(c, metal(mul(SILVER, 0.8f), r, 228, 238, a), band(r, 228, 238));
          c = mix(c, {2, 2, 4}, clamp01(r - 238 + 0.5f));
          break;
        }
      }
      buf[y * W + x] = pack(c);
    }
  }
}

// Many small shapes: each is filled over its own small area (one shape spanning the dial would be tested at every
// pixel of the dial against every stroke).
typedef std::vector<Shape> Shapes;
static Shape &add(Shapes &v) { v.emplace_back(); return v.back(); }
static void fillAll(uint16_t *buf, const Shapes &v, C3 c, float alpha = 1, float grow = 0, float soft = 1) {
  for (const Shape &s : v) fill(buf, s, c, alpha, grow, soft);
}

static void markers(uint16_t *buf, int face, bool moon) {
  const float H12 = PI2 / 12, M60 = PI2 / 60;
  Shapes ticks, bars, nums;
  char t[4];
  switch (face) {
    case 0: {  // Gatsby: 12 3 6 9, tapered bars, fine minute ticks, diamonds at the quarters
      for (int m = 0; m < 60; m++) {
        float a = m * M60;
        if (m % 5) add(ticks).seg(px(a, 210), py(a, 210), px(a, 215), py(a, 215), 0.7f);
      }
      for (int h = 0; h < 12; h++) {
        float a = h * H12;
        HandBuilder d(a);
        if (h % 3) d.poly({186, -5, 215, -2.5f, 215, 2.5f, 186, 5});
        else d.poly({203, 0, 209, -5, 215, 0, 209, 5});
        bars.push_back(d.sh);
      }
      const char *q[4] = {"12", "3", "6", "9"};
      for (int i = 0; i < 4; i++) {
        float a = i * 3 * H12;
        text(add(nums), q[i], px(a, 168), py(a, 168), 44, 5.5f, 2.4f);
      }
      fillAll(buf, ticks, mul(GOLD, 0.8f));
      fillAll(buf, bars, {70, 50, 14}, 1, 1.2f);
      fillAll(buf, bars, GOLD);
      fillAll(buf, nums, {0, 0, 0}, 0.5f, 1.5f, 3);
      fillAll(buf, nums, {230, 194, 104});
      break;
    }
    case 1: {  // Skyline: all twelve numerals, minute ticks in the outer track
      for (int m = 0; m < 60; m++) {
        float a = m * M60;
        bool hr = m % 5 == 0;
        add(ticks).seg(px(a, hr ? 212 : 215), py(a, hr ? 212 : 215), px(a, 220), py(a, 220), hr ? 1.6f : 0.7f);
      }
      for (int h = 1; h <= 12; h++) {
        float a = h * H12;
        snprintf(t, sizeof t, "%d", h);
        text(add(nums), t, px(a, 196), py(a, 196), 25, 3.4f, 2.0f);
      }
      fillAll(buf, ticks, SILVER);
      fillAll(buf, nums, {0, 0, 0}, 0.6f, 1.2f, 2.5f);
      fillAll(buf, nums, {226, 230, 238});
      break;
    }
    case 2: {  // Ivory: cream ticks on the black chapter ring, black numerals
      for (int m = 0; m < 60; m++) {
        float a = m * M60;
        if (m % 5 == 0) {
          HandBuilder d(a);
          d.poly({203, -2.5f, 223, -1.5f, 223, 1.5f, 203, 2.5f});
          ticks.push_back(d.sh);
        } else {
          add(ticks).seg(px(a, 208), py(a, 208), px(a, 218), py(a, 218), 0.8f);
        }
      }
      for (int h = 1; h <= 12; h++) {
        float a = h * H12;
        snprintf(t, sizeof t, "%d", h);
        text(add(nums), t, px(a, 172), py(a, 172), 31, 4.6f, 2.0f);
      }
      fillAll(buf, ticks, {236, 228, 206});
      fillAll(buf, nums, {20, 20, 20});
      break;
    }
    default: {  // Midnight: Roman quarters, silver bars, minute dots
      for (int m = 0; m < 60; m++)
        if (m % 5) add(ticks).disc(px(m * M60, 212), py(m * M60, 212), 1.3f);
      for (int h = 0; h < 12; h++)
        if (h % 3) {
          HandBuilder d(h * H12);
          d.poly({192, -3, 216, -1.5f, 216, 1.5f, 192, 3});
          bars.push_back(d.sh);
        }
      const char *q[4] = {"XII", "III", "VI", "IX"};
      for (int i = 0; i < 4; i++) {
        float a = i * 3 * H12;
        if (i == 2 && moon) text(add(nums), "VI", px(a, 182), py(a, 182), 26, 3.2f, 1.6f);   // smaller, clear of the moon
        else text(add(nums), q[i], px(a, 176), py(a, 176), 34, 3.6f, 1.6f);
      }
      fillAll(buf, ticks, {150, 162, 192});
      fillAll(buf, bars, {200, 208, 226});
      fillAll(buf, nums, {200, 208, 226});
      break;
    }
  }
}

// ---------- complications ----------
static void tempWindow(uint16_t *buf, const Theme &t, float cx, float cy, const char *label, const char *val) {
  Shape win;
  win.box(cx, cy, 39, 30, 9);
  fill(buf, win, t.windowEdge, 1, 1.5f);
  fill(buf, win, t.window);
  Shape lab, num;
  text(lab, label, cx, cy - 15, 9, 1.7f, 1.8f);
  char v[12];
  bool isNum = strcmp(val, "--") != 0 && val[0];
  snprintf(v, sizeof v, "%s%s", val[0] ? val : "--", isNum ? "*" : "");
  text(num, v, cx + (isNum ? 3 : 0), cy + 8, 22, 3.2f, 1.9f);
  fill(buf, lab, t.label);
  fill(buf, num, t.ink);
}

static void moonWindow(uint16_t *buf, const Theme &t, float phase) {
  const float mx = CX, my = CY + 108, R = 25;
  Shape win;
  win.disc(mx, my, R + 5);
  fill(buf, win, t.windowEdge, 1, 1.5f);
  fill(buf, win, t.window);
  float k = cosf(phase * PI2);
  bool waxing = phase < 0.5f;
  static const float craters[][3] = {{-0.35f, -0.3f, 0.22f}, {0.3f, 0.25f, 0.16f}, {0.05f, -0.05f, 0.12f},
                                     {-0.1f, 0.5f, 0.14f},  {0.45f, -0.35f, 0.1f}};
  for (int y = (int)(my - R - 2); y <= (int)(my + R + 2); y++)
    for (int x = (int)(mx - R - 2); x <= (int)(mx + R + 2); x++) {
      float dx = x + 0.5f - mx, dy = y + 0.5f - my;
      float r = sqrtf(dx * dx + dy * dy);
      float edge = cov(r - R);
      if (edge <= 0) continue;
      float nx = dx / R, ny = dy / R;
      float hw = sqrtf(fmaxf(0, 1 - ny * ny));
      float lit = waxing ? (nx - k * hw) * R : (-k * hw - nx) * R;       // pixels into the sunlit side
      float l = clamp01(lit + 0.5f);
      C3 day = mul(t.moonLit, 1.0f - 0.18f * (nx * nx + ny * ny));       // a little darker at the rim
      for (auto &c : craters) {
        float cx = nx - c[0], cy = ny - c[1];
        float d = (sqrtf(cx * cx + cy * cy) - c[2]) * R;
        day = mix(day, mul(day, 0.84f), clamp01(0.5f - d));
      }
      blend(buf[y * W + x], mix(t.moonDark, day, l), edge);
    }
}

void renderDial(uint16_t *buf, const Dial &d) {
  int f = d.face < 0 || d.face >= FACE_COUNT ? 0 : d.face;
  const Theme &t = THEMES[f];
  background(buf, f);
  markers(buf, f, d.moon);
  if (d.temps) {
    tempWindow(buf, t, CX - 96, CY, "IN", d.in);
    tempWindow(buf, t, CX + 96, CY, "OUT", d.out);
  }
  if (d.moon) moonWindow(buf, t, d.moonPhase);
}

// ---------- hands ----------
static void handShapes(int face, int which, float a, Shape &out) {   // which: 0 hour, 1 minute, 2 second
  HandBuilder h(a);
  switch (face) {
    case 0:  // Gatsby: lances with a diamond
      if (which == 0) { h.poly({-24, -6, 84, -4.5f, 84, 4.5f, -24, 6}); h.poly({78, 0, 106, -13, 138, 0, 106, 13}); h.disc(-24, 7); }
      else if (which == 1) { h.poly({-30, -5, 168, -3, 168, 3, -30, 5}); h.poly({160, 0, 180, -7.5f, 208, 0, 180, 7.5f}); h.disc(-30, 6); }
      else { h.line(-44, 202, 1.3f); h.disc(-32, 6); }
      break;
    case 1:  // Skyline: stepped spires
      if (which == 0) { h.rect(-18, 62, 16); h.rect(60, 94, 11); h.rect(92, 118, 6); h.poly({116, -3, 136, 0, 116, 3}); }
      else if (which == 1) { h.rect(-24, 102, 12); h.rect(100, 152, 8); h.rect(150, 184, 4); h.poly({182, -2, 206, 0, 182, 2}); }
      else { h.line(-40, 206, 1.2f); h.disc(-30, 5); }
      break;
    case 2:  // Ivory: spade hour, lance minute, jade lollipop seconds
      if (which == 0) { h.poly({-20, -4.5f, 70, -3, 70, 3, -20, 4.5f}); h.poly({62, 0, 78, -11, 104, -8, 124, 0, 104, 8, 78, 11}); h.disc(-20, 6); }
      else if (which == 1) { h.poly({-26, -4, 172, -2.5f, 172, 2.5f, -26, 4}); h.poly({166, 0, 182, -5.5f, 204, 0, 182, 5.5f}); h.disc(-26, 5); }
      else { h.line(-40, 196, 1.2f); h.disc(150, 4.5f); h.disc(-30, 6); }
      break;
    default:  // Midnight: slim dauphine
      if (which == 0) { h.poly({-18, -7, 126, 0, -18, 7}); h.disc(-18, 7); }
      else if (which == 1) { h.poly({-24, -5.5f, 206, 0, -24, 5.5f}); h.disc(-24, 5.5f); }
      else { h.line(-38, 204, 1.1f); h.disc(-28, 4.5f); }
      break;
  }
  out = h.sh;
}

void drawHands(uint16_t *buf, int face, float hour, float minute, float second, bool showSeconds) {
  int f = face < 0 || face >= FACE_COUNT ? 0 : face;
  const Theme &t = THEMES[f];
  float ah = fmodf(hour, 12) / 12 * PI2, am = minute / 60 * PI2, as = second / 60 * PI2;
  Shape s;
  handShapes(f, 0, ah, s);
  fill(buf, s, {0, 0, 0}, 0.42f, 0, 7, 3, 5);
  fill(buf, s, t.handEdge, 1, 1.1f);
  fill(buf, s, t.hands);
  handShapes(f, 1, am, s);
  fill(buf, s, {0, 0, 0}, 0.42f, 0, 7, 4, 6);
  fill(buf, s, t.handEdge, 1, 1.1f);
  fill(buf, s, t.hands);
  if (showSeconds) {
    handShapes(f, 2, as, s);
    fill(buf, s, {0, 0, 0}, 0.35f, 0, 5, 4, 7);
    fill(buf, s, t.second);
  }
  Shape hub, dot;
  hub.disc(CX, CY, f == 3 ? 8 : 10);
  dot.disc(CX, CY, 3.5f);
  fill(buf, hub, showSeconds ? t.second : t.hub);
  fill(buf, hub, t.hub, 1, -2.5f);
  fill(buf, dot, t.hubDot);
}

// ---------- the moon ----------
float moonPhase(time_t utc) {
  const double synodic = 29.530588853, ref = 947182440.0;   // new moon 2000-01-06 18:14 UTC
  double p = fmod(((double)utc - ref) / 86400.0 / synodic, 1.0);
  if (p < 0) p += 1;
  return (float)p;
}
const char *moonName(float p) {
  static const char *n[8] = {"New moon", "Waxing crescent", "First quarter", "Waxing gibbous",
                             "Full moon", "Waning gibbous", "Last quarter", "Waning crescent"};
  return n[(int)floorf(p * 8 + 0.5f) % 8];
}

const char *faceName(int f) {
  static const char *n[FACE_COUNT] = {"Gatsby", "Skyline", "Ivory", "Midnight"};
  return f >= 0 && f < FACE_COUNT ? n[f] : n[0];
}

}  // namespace clk
