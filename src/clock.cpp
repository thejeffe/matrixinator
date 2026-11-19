// Art deco clock faces. Everything is drawn with signed distances, so edges are smooth (anti-aliased) and a shape
// made of several parts (a hand, a numeral) is blended once, with no seams where the parts overlap.
// Arduino builds for size (-Os); this file is all arithmetic, so it gets full speed.
#pragma GCC optimize("O2", "fast-math")
#include "clock.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <initializer_list>
#include <vector>

namespace clk {

static const float CX = W / 2.0f, CY = H / 2.0f;
// The faces, in the order the settings page lists them.
enum { F_GATSBY, F_SKYLINE, F_IVORY, F_MIDNIGHT, F_GRANDFATHER, F_CHRISTMAS, F_ADVENT, F_LED, F_VFD, F_NIXIE, F_FLIP };
static void (*yieldFn)() = nullptr;
void setYield(void (*fn)()) { yieldFn = fn; }
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
  if (yieldFn) yieldFn();
  float pad = grow + soft + 1.5f;
  int xa = (int)floorf(s.x0 + ox - pad), xb = (int)ceilf(s.x1 + ox + pad);
  int ya = (int)floorf(s.y0 + oy - pad), yb = (int)ceilf(s.y1 + oy + pad);
  if (xa < 0) xa = 0;
  if (ya < 0) ya = 0;
  if (xb > W - 1) xb = W - 1;
  if (yb > H - 1) yb = H - 1;
  // 8x8 blocks: the distance at a block's centre says whether the whole block is outside (skip it), inside
  // (fill it) or on the edge (work out each pixel). The distance changes by at most 1 per pixel moved.
  const int B = 8;
  const float reach = (B - 1) * 0.5f * 1.41422f, half = 0.5f * soft;
  const uint16_t solid = pack(c);
  for (int by = ya; by <= yb; by += B) {
    int ey = by + B - 1 < yb ? by + B - 1 : yb;
    for (int bx = xa; bx <= xb; bx += B) {
      int ex = bx + B - 1 < xb ? bx + B - 1 : xb;
      float d0 = s.sdf((bx + ex) * 0.5f + 0.5f - ox, (by + ey) * 0.5f + 0.5f - oy) - grow;
      if (d0 - reach >= half) continue;                        // all outside
      if (d0 + reach <= -half) {                               // all inside
        for (int y = by; y <= ey; y++) {
          uint16_t *row = buf + y * W;
          if (alpha >= 0.998f) for (int x = bx; x <= ex; x++) row[x] = solid;
          else for (int x = bx; x <= ex; x++) blend(row[x], c, alpha);
        }
        continue;
      }
      for (int y = by; y <= ey; y++) {
        uint16_t *row = buf + y * W;
        float py = y + 0.5f - oy;
        for (int x = bx; x <= ex; x++) {
          float d = s.sdf(x + 0.5f - ox, py) - grow;
          float a = clamp01(0.5f - d / soft) * alpha;
          if (a > 0.002f) blend(row[x], c, a);
        }
      }
    }
  }
}

// atan2 to about 0.00001 rad, several times quicker than the library's on the ESP32.
static inline float fastAtan2(float y, float x) {
  float ax = fabsf(x), ay = fabsf(y);
  if (ax < 1e-9f && ay < 1e-9f) return 0;
  bool swap = ay > ax;
  float z = swap ? ax / ay : ay / ax, z2 = z * z;
  float r = z * (0.99997726f + z2 * (-0.33262347f + z2 * (0.19354346f + z2 * (-0.11643287f + z2 * (0.05265332f + z2 * -0.01172120f)))));
  if (swap) r = 1.57079633f - r;
  if (x < 0) r = 3.14159265f - r;
  return y < 0 ? -r : r;
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
    g['A'] = {5, {{0, 10, 2.5f, 0, 5, 10}, {1.1f, 6.4f, 3.9f, 6.4f}}};
    {
      std::vector<float> a = {0, 10, 0, 0, 2.6f, 0}, b = {0, 4.7f, 2.7f, 4.7f};
      arc(a, 2.6f, 2.35f, 2.35f, 2.35f, 270, 450); a.insert(a.end(), {0, 4.7f});
      arc(b, 2.7f, 7.35f, 2.65f, 2.65f, 270, 450); b.insert(b.end(), {0, 10});
      g['B'] = {5.3f, {a, b}};
    }
    {
      std::vector<float> s;
      arc(s, 2.5f, 2.5f, 2.5f, 2.5f, 330, 180); s.insert(s.end(), {0, 7.5f});
      arc(s, 2.5f, 7.5f, 2.5f, 2.5f, 180, 0); s.insert(s.end(), {5, 5.4f, 2.8f, 5.4f});
      g['G'] = {5, {s}};
    }
    g['H'] = {5, {{0, 0, 0, 10}, {5, 0, 5, 10}, {0, 5, 5, 5}}};
    g['M'] = {5.4f, {{0, 10, 0, 0, 2.7f, 6.2f, 5.4f, 0, 5.4f, 10}}};
    {
      std::vector<float> s = {0, 10, 0, 0, 2.5f, 0};
      arc(s, 2.5f, 2.6f, 2.6f, 2.6f, 270, 450); s.insert(s.end(), {0, 5.2f});
      g['P'] = {5.1f, {s}};
      g['R'] = {5.1f, {s, {2.4f, 5.2f, 5.1f, 10}}};
    }
    {
      std::vector<float> a, b;
      arc(a, 1.2f, 1.8f, 1.2f, 1.8f, 0, 360);
      arc(b, 4.8f, 8.2f, 1.2f, 1.8f, 0, 360);
      g['%'] = {6, {{0.4f, 10, 5.6f, 0}, a, b}};
    }
    g['.'] = {0, {{0, 9.7f, 0, 9.7f}}};
    g['{'] = {4, {{2, 10, 2, 0.4f}, {0, 2.6f, 2, 0.4f, 4, 2.6f}}};     // arrow up
    g['}'] = {4, {{2, 0, 2, 9.6f}, {0, 7.4f, 2, 9.6f, 4, 7.4f}}};    // arrow down
    g[':'] = {0, {{0, 2.8f, 0, 2.8f}, {0, 8.2f, 0, 8.2f}}};
    g['!'] = {0, {{0, 0, 0, 6.6f}, {0, 9.7f, 0, 9.7f}}};
    {
      std::vector<float> s;
      arc(s, 2.5f, 2.5f, 2.5f, 2.5f, 330, 180); s.insert(s.end(), {0, 7.5f});
      arc(s, 2.5f, 7.5f, 2.5f, 2.5f, 180, 30);
      g['C'] = {5, {s}};
    }
    {
      std::vector<float> s = {0, 0, 2.2f, 0};
      arc(s, 2.2f, 2.8f, 2.8f, 2.8f, 270, 360); s.insert(s.end(), {5, 7.2f});
      arc(s, 2.2f, 7.2f, 2.8f, 2.8f, 0, 90); s.insert(s.end(), {0, 10, 0, 0});
      g['D'] = {5, {s}};
    }
    g['E'] = {4.6f, {{4.6f, 0, 0, 0, 0, 10, 4.6f, 10}, {0, 5, 3.8f, 5}}};
    g['F'] = {4.6f, {{4.6f, 0, 0, 0, 0, 10}, {0, 5, 3.8f, 5}}};
    { std::vector<float> s = {4, 0, 4, 7.6f}; arc(s, 2, 7.6f, 2, 2.4f, 0, 180); g['J'] = {4, {s}}; }
    g['K'] = {5, {{0, 0, 0, 10}, {5, 0, 0, 6.2f}, {1.7f, 4.4f, 5, 10}}};
    g['L'] = {4.4f, {{0, 0, 0, 10, 4.4f, 10}}};
    g['Q'] = g['0'];
    g['Q'].strokes.push_back({3.1f, 7.4f, 5.4f, 10.4f});
    {
      std::vector<float> s;
      arc(s, 2.5f, 2.6f, 2.4f, 2.6f, 340, 90);
      arc(s, 2.5f, 7.6f, 2.5f, 2.4f, 270, 520);
      g['S'] = {5, {s}};
    }
    g['W'] = {6.4f, {{0, 0, 1.4f, 10, 3.2f, 3.4f, 5, 10, 6.4f, 0}}};
    g['Y'] = {5, {{0, 0, 2.5f, 5.2f, 5, 0}, {2.5f, 5.2f, 2.5f, 10}}};
    g['Z'] = {5, {{0, 0, 5, 0, 0, 10, 5, 10}}};
  }
  if (ch >= 'a' && ch <= 'z') ch -= 32;   // lower case is drawn as capitals
  return g[(unsigned char)ch & 127];
}
// Add a line of text, centred on (cx, cy): h = letter height in pixels, th = stroke width, gap = between letters
// (units), rot = turned clockwise about its centre (radians).
typedef std::vector<Shape> Shapes;
static void textCore(Shape *one, Shapes *many, const char *t, float cx, float cy, float h, float th, float gap, float rot) {
  float k = h / 10, wsum = 0;
  int n = (int)strlen(t);
  for (int i = 0; i < n; i++) wsum += glyph(t[i]).w + (i ? gap : 0);
  float x = -wsum * k / 2, y = -h / 2;
  float cr = cosf(rot), sr = sinf(rot);
  auto X = [&](float u, float v) { return cx + u * cr - v * sr; };
  auto Y = [&](float u, float v) { return cy + u * sr + v * cr; };
  for (int i = 0; i < n; i++) {
    const Glyph &g = glyph(t[i]);
    if (many) many->emplace_back();
    Shape &s = many ? many->back() : *one;
    for (const auto &st : g.strokes)
      for (size_t j = 0; j + 3 < st.size(); j += 2) {
        float u0 = x + st[j] * k, v0 = y + st[j + 1] * k, u1 = x + st[j + 2] * k, v1 = y + st[j + 3] * k;
        s.seg(X(u0, v0), Y(u0, v0), X(u1, v1), Y(u1, v1), th / 2);
      }
    x += (g.w + gap) * k;
  }
}
static void text(Shape &s, const char *t, float cx, float cy, float h, float th, float gap = 2.2f, float rot = 0) {
  textCore(&s, nullptr, t, cx, cy, h, th, gap, rot);
}
static Shape &add(Shapes &v) { v.emplace_back(); return v.back(); }
static void fillAll(uint16_t *buf, const Shapes &v, C3 c, float alpha = 1, float grow = 0, float soft = 1) {
  for (const Shape &s : v) fill(buf, s, c, alpha, grow, soft);
}
// The same, one shape per letter: much quicker to fill for long or large text.
static void text(Shapes &v, const char *t, float cx, float cy, float h, float th, float gap = 2.2f, float rot = 0) {
  textCore(nullptr, &v, t, cx, cy, h, th, gap, rot);
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
    // Grandfather: silvered chapter ring on a brass plate, black hands
    {{30, 26, 22}, {96, 76, 44}, {238, 234, 222}, {40, 34, 28}, {245, 236, 205}, {40, 48, 80},
     {26, 23, 20}, {26, 23, 20}, {26, 23, 20}, {26, 23, 20}, {200, 160, 80}},
    // Christmas: green, candy-cane hands, ornaments
    {{244, 214, 120}, {236, 236, 236}, {10, 44, 24}, {222, 182, 92}, {250, 245, 220}, {20, 50, 35},
     {252, 252, 252}, {120, 20, 24}, {244, 204, 96}, {204, 30, 40}, {244, 204, 96}},
    // Advent: red, 24 doors, the time as digits
    {{244, 214, 120}, {250, 236, 210}, {60, 10, 14}, {222, 182, 92}, {250, 245, 220}, {40, 20, 20},
     {244, 214, 120}, {60, 10, 14}, {244, 214, 120}, {244, 214, 120}, {60, 10, 14}},
    // Red LED, VFD, Nixie, Flip: digital (only the moon colours are used)
    {{230, 46, 34}, {140, 26, 20}, {0, 0, 0}, {0, 0, 0}, {255, 120, 90}, {40, 8, 6}, {}, {}, {}, {}, {}},
    {{116, 236, 214}, {60, 140, 130}, {0, 0, 0}, {0, 0, 0}, {160, 255, 236}, {10, 34, 32}, {}, {}, {}, {}, {}},
    {{255, 164, 96}, {170, 110, 70}, {0, 0, 0}, {0, 0, 0}, {255, 190, 120}, {50, 32, 24}, {}, {}, {}, {}, {}},
    {{206, 206, 200}, {130, 130, 130}, {0, 0, 0}, {0, 0, 0}, {236, 232, 216}, {50, 50, 54}, {}, {}, {}, {}, {}},
};

static const C3 GOLD = {212, 172, 84}, SILVER = {196, 202, 214}, BRASS = {196, 156, 72};

// sin to about 0.001, much quicker than the library's on the ESP32 (for textures, not geometry).
static inline float fastSin(float x) {
  const float inv = 1 / PI2;
  x -= PI2 * floorf(x * inv + 0.5f);                  // -pi .. pi
  float y = 1.27323954f * x - 0.40528473f * x * fabsf(x);
  return 0.225f * (y * fabsf(y) - y) + y;
}

static inline float band(float r, float r0, float r1) { return clamp01(fminf(r - r0, r1 - r) + 0.5f); }
// Polished metal ring: lighter towards the top left, a rounded profile across the ring.
static C3 metal(C3 base, float r, float r0, float r1, float a) {
  float m = (r - r0) / (r1 - r0) * 2 - 1;                 // -1 .. 1 across
  float light = 0.72f + 0.34f * fastSin(a - 5.5f + 1.57079633f);   // cos; 5.5 rad = towards 10 o'clock
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
    if (yieldFn && (y & 15) == 0) yieldFn();
    for (int x = 0; x < W; x++) {
      float dx = x + 0.5f - CX, dy = y + 0.5f - CY;
      float r = sqrtf(dx * dx + dy * dy);
      float a = fastAtan2(dx, -dy);
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
          if (r > 225 && r < 239) c = mix(c, metal(GOLD, r, 226, 238, a), band(r, 226, 238));
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
          if (r > 227 && r < 239) c = mix(c, metal(SILVER, r, 228, 238, a), band(r, 228, 238));
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
          if (r > 227 && r < 239) c = mix(c, metal(BRASS, r, 228, 238, a), band(r, 228, 238));
          c = mix(c, {3, 3, 3}, clamp01(r - 238 + 0.5f));
          break;
        }
        case F_GRANDFATHER: {  // Grandfather: ring-turned centre, silvered chapter ring, brass plate in the corners
          C3 ring = {236, 234, 226};
          c = r < 151 ? mix(mul({214, 208, 192}, 0.95f + 0.05f * fastSin(r * 0.95f)), ring, clamp01(r - 150 + 0.5f)) : ring;
          C3 ink = {30, 26, 22};
          c = mix(c, ink, band(r, 150, 151.5f));
          c = mix(c, ink, band(r, 155, 156));
          c = mix(c, ink, band(r, 200, 201));
          c = mix(c, ink, band(r, 219, 220.5f));
          c = mix(c, mul({178, 138, 60}, 0.7f), band(r, 220.5f, 224));
          if (r > 223) {   // brushed brass
            C3 plate = mul({178, 138, 60}, 0.93f + 0.05f * fastSin((y + 0.5f) * 1.7f + fastSin(x * 0.045f) * 2) +
                                               0.04f * (1 - (dx + dy) / 680));
            c = mix(c, plate, clamp01(r - 224 + 0.5f));
          }
          // a dark edge round the plate
          float edge = fminf(fminf(x + 0.5f, W - x - 0.5f), fminf(y + 0.5f, H - y - 0.5f));
          c = mix(c, mul({178, 138, 60}, 0.45f), clamp01(4.5f - edge));
          break;
        }
        case F_CHRISTMAS: {  // Christmas: deep green, a candy-stripe bezel
          float t = r / 220;
          c = mix({24, 96, 54}, {8, 48, 28}, t * t);
          c = mix(c, {222, 182, 92}, band(r, 218, 222));
          if (r > 221) {
            float u = a / PI2 * 40 + (r - 222) / 16 * 0.6f;
            float f = u - floorf(u);
            float d = f < 0.5f ? fminf(f, 0.5f - f) : -fminf(f - 0.5f, 1 - f);   // >0 inside a red stripe
            float edgePx = d * PI2 * r / 40;
            float red = clamp01(edgePx + 0.5f);
            C3 stripe = mix({248, 248, 244}, {210, 30, 40}, red);
            if (r > 221 && r < 239) c = mix(c, metal(stripe, r, 222, 238, a + 0.8f), band(r, 222, 238));
          }
          c = mix(c, {3, 3, 3}, clamp01(r - 238 + 0.5f));
          break;
        }
        case F_ADVENT: {  // Advent: red with a gold frame
          float ex = fabsf(dx) / 240, ey = fabsf(dy) / 240;
          c = mul({156, 22, 32}, 1.0f - 0.3f * (ex * ex + ey * ey));
          float edge = fminf(fminf(x + 0.5f, W - x - 0.5f), fminf(y + 0.5f, H - y - 0.5f));
          c = mix(c, {222, 182, 92}, band(edge, 5, 8));
          c = mix(c, {222, 182, 92}, band(edge, 10, 11));
          break;
        }
        case F_LED:
        case F_VFD: {  // black plastic
          float t = (y + 0.5f) / H;
          c = mix({26, 24, 24}, {6, 6, 6}, t);
          break;
        }
        case F_NIXIE: {  // walnut
          float grain = 0.82f + 0.12f * fastSin((y + 0.5f) * 0.09f + fastSin(x * 0.012f) * 3) + 0.06f * fastSin(y * 0.6f);
          float vig = 1 - 0.35f * (dx * dx + dy * dy) / (240.0f * 240 * 2);
          c = mul({92, 52, 28}, grain * vig);
          break;
        }
        case F_FLIP: {
          float t = r / 340;
          c = mix({40, 40, 44}, {14, 14, 16}, t);
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
          if (r > 227 && r < 239) c = mix(c, metal(mul(SILVER, 0.8f), r, 228, 238, a), band(r, 228, 238));
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

// Brass scrollwork in one corner of the Grandfather face (right / bottom mirror the top-left one).
static void spandrel(uint16_t *buf, bool right, bool bottom) {
  auto X = [&](float v) { return right ? W - v : v; };
  auto Y = [&](float v) { return bottom ? H - v : v; };
  Shapes scrolls;                                  // one per edge: small areas to fill
  Shape leaves, rose, roseDot;
  auto path = [&](Shape &sc, const std::vector<float> &p, float r) {
    for (size_t i = 0; i + 3 < p.size(); i += 2) sc.seg(X(p[i]), Y(p[i + 1]), X(p[i + 2]), Y(p[i + 3]), r);
  };
  for (int swap = 0; swap < 2; swap++) {           // one scroll along each edge
    std::vector<float> p = {40, 26, 62, 18, 86, 15, 104, 19};
    arc(p, 108, 31, 12, 12, 250, 560);
    std::vector<float> q;
    arc(q, 108, 31, 6, 6, 200, 380);
    if (swap) {
      for (size_t i = 0; i < p.size(); i += 2) std::swap(p[i], p[i + 1]);
      for (size_t i = 0; i < q.size(); i += 2) std::swap(q[i], q[i + 1]);
    }
    Shape &sc = add(scrolls);
    path(sc, p, 2.4f);
    path(sc, q, 1.6f);
    float lv[][3] = {{58, 28, 4.5f}, {80, 25, 4}, {96, 28, 3.2f}};
    for (auto &l : lv) leaves.disc(X(swap ? l[1] : l[0]), Y(swap ? l[0] : l[1]), l[2]);
  }
  rose.disc(X(30), Y(30), 12);
  for (int i = 0; i < 6; i++) {
    float a = i * PI2 / 6 + 0.3f;
    rose.disc(X(30 + 14 * cosf(a)), Y(30 + 14 * sinf(a)), 5);
  }
  roseDot.disc(X(30), Y(30), 4.5f);
  C3 gold = {236, 196, 104}, dark = {92, 66, 22};
  for (auto &sc : scrolls) fill(buf, sc, {0, 0, 0}, 0.35f, 0, 4, 2, 3);
  fill(buf, rose, {0, 0, 0}, 0.35f, 0, 4, 2, 3);
  fillAll(buf, scrolls, dark, 1, 1);
  fillAll(buf, scrolls, gold);
  fill(buf, leaves, dark, 1, 1);
  fill(buf, leaves, gold);
  fill(buf, rose, dark, 1, 1);
  fill(buf, rose, gold);
  fill(buf, roseDot, dark);
}

// Two lines for the Christmas countdown.
static void xmasLines(int days, char *l1, char *l2, size_t n) {
  if (days == 0) { snprintf(l1, n, "MERRY"); snprintf(l2, n, "CHRISTMAS!"); }
  else if (days == 1) { snprintf(l1, n, "1 DAY"); snprintf(l2, n, "TO CHRISTMAS"); }
  else { snprintf(l1, n, "%d DAYS", days); snprintf(l2, n, "TO CHRISTMAS"); }
}

static void countdownWindow(uint16_t *buf, const Theme &t, int days) {
  char l1[24], l2[24];
  xmasLines(days, l1, l2, sizeof l1);
  const float x = CX, y = CY - 94;
  Shape win, a, b;
  win.box(x, y, 64, 24, 10);
  text(a, l1, x, y - 6, 17, 2.8f, 2.0f);
  text(b, l2, x, y + 14, 8, 1.6f, 1.8f);
  fill(buf, win, t.windowEdge, 1, 1.5f);
  fill(buf, win, t.window);
  fill(buf, a, t.ink);
  fill(buf, b, t.label);
}

// ---------- the Advent calendar ----------
// 24 doors in a fixed shuffled order. In December, doors up to today are open (today's glows); after the 24th
// they're all open; the rest of the year they're all shut. The banner at the top: the time (drawn by drawHands)
// and the countdown.
static const int ADVENT_ORDER[24] = {13, 5, 21, 9, 1, 17, 22, 10, 3, 15, 24, 7, 6, 19, 12, 23, 2, 16, 18, 4, 14, 8, 20, 11};
static void adventIcon(uint16_t *buf, int kind, float x, float y) {
  Shape a, b, c;
  switch (kind) {
    case 0: {  // star
      float px5[5], py5[5];
      for (int i = 0; i < 5; i++) { float an = i * PI2 / 5; px5[i] = x + 7 * sinf(an + PI2 / 10); py5[i] = y - 7 * cosf(an + PI2 / 10); }
      a.poly(px5, py5, 5);
      for (int i = 0; i < 5; i++) {
        float an = i * PI2 / 5;
        float tx[3] = {x + 18 * sinf(an), px5[(i + 4) % 5], px5[i]}, ty[3] = {y - 18 * cosf(an), py5[(i + 4) % 5], py5[i]};
        a.poly(tx, ty, 3);
      }
      fill(buf, a, {120, 80, 10}, 1, 1);
      fill(buf, a, {250, 210, 90});
      break;
    }
    case 1: {  // tree
      for (int i = 0; i < 3; i++) {
        float top = y - 20 + i * 9, w = 9 + i * 5;
        float tx[3] = {x, x - w, x + w}, ty[3] = {top, top + 15, top + 15};
        a.poly(tx, ty, 3);
      }
      b.box(x, y + 15, 3.5f, 4, 1);
      c.disc(x, y - 21, 3.5f);
      fill(buf, b, {110, 66, 30});
      fill(buf, a, {40, 150, 70});
      fill(buf, c, {250, 210, 90});
      break;
    }
    case 2: {  // ornament
      a.disc(x, y + 3, 15);
      b.box(x, y - 13, 5, 3.5f, 1);
      c.disc(x - 5, y - 2, 3.5f);
      fill(buf, b, {230, 188, 80});
      fill(buf, a, {214, 34, 44});
      fill(buf, c, {255, 255, 255}, 0.7f);
      break;
    }
    case 3: {  // present
      a.box(x, y + 4, 16, 13, 2);
      b.seg(x, y - 9, x, y + 17, 2.2f);
      b.seg(x - 16, y + 4, x + 16, y + 4, 2.2f);
      c.disc(x - 5, y - 12, 4.5f);
      c.disc(x + 5, y - 12, 4.5f);
      fill(buf, a, {50, 100, 200});
      fill(buf, b, {250, 210, 90});
      fill(buf, c, {250, 210, 90});
      break;
    }
    case 4: {  // candy cane: red and white in turns
      std::vector<float> p;
      arc(p, x + 4, y - 10, 7, 7, 180, 360);
      p.insert(p.begin(), {x - 3, y + 20});
      std::vector<float> q = {x - 3, y + 20, x - 3, y + 12, x - 3, y + 4, x - 3, y - 4, x - 3, y - 10};
      q.insert(q.end(), p.begin() + 4, p.end());
      for (size_t i = 0; i + 3 < q.size(); i += 2) (i / 2 % 2 ? b : a).seg(q[i], q[i + 1], q[i + 2], q[i + 3], 3.4f);
      fill(buf, a, {250, 250, 250});
      fill(buf, b, {214, 34, 44});
      break;
    }
    default: {  // snowflake
      for (int k = 0; k < 3; k++) {
        float an = k * PI2 / 6, dx = sinf(an) * 17, dy = cosf(an) * 17;
        a.seg(x - dx, y - dy, x + dx, y + dy, 1.8f);
      }
      for (int k = 0; k < 6; k++) {
        float an = k * PI2 / 6, mx = x + sinf(an) * 10, my = y - cosf(an) * 10;
        for (int sgn = -1; sgn <= 1; sgn += 2) {
          float b2 = an + sgn * 0.8f;
          a.seg(mx, my, mx + sinf(b2) * 6, my - cosf(b2) * 6, 1.4f);
        }
      }
      fill(buf, a, {220, 236, 255});
      break;
    }
  }
}
static void advent(uint16_t *buf, const Dial &d) {
  int open = d.month == 12 ? (d.mday > 24 ? 24 : d.mday) : 0;
  int today = (d.month == 12 && d.mday <= 24) ? d.mday : 0;
  char l1[24], l2[24], line[48];
  if (d.daysToXmas >= 0) {
    xmasLines(d.daysToXmas, l1, l2, sizeof l1);
    snprintf(line, sizeof line, "%s %s", l1, l2);
    Shapes t;
    text(t, line, CX, 104, 14, 2.4f, 2.0f);
    fillAll(buf, t, {250, 236, 210});
  }
  static const C3 doors[3] = {{26, 112, 62}, {236, 226, 200}, {34, 66, 128}};
  const float x0 = 20, y0 = 128, gw = 440, gh = 334, gap = 8;
  const float dw = (gw - 5 * gap) / 6, dh = (gh - 3 * gap) / 4;
  for (int i = 0; i < 24; i++) {
    int n = ADVENT_ORDER[i];
    float cx = x0 + (i % 6) * (dw + gap) + dw / 2, cy = y0 + (i / 6) * (dh + gap) + dh / 2;
    C3 col = doors[i % 3];
    C3 ink = i % 3 == 1 ? C3{150, 24, 34} : C3{244, 214, 120};
    Shape door;
    door.box(cx, cy, dw / 2, dh / 2, 6);
    char num[4];
    snprintf(num, sizeof num, "%d", n);
    if (n == today) fill(buf, door, {255, 214, 110}, 0.9f, 3, 10);        // today's door glows
    fill(buf, door, {0, 0, 0}, 0.35f, 0, 5, 2, 3);
    if (n <= open) {
      if (n == today) fill(buf, door, {255, 214, 110}, 1, 2.5f);            // a gold edge
      fill(buf, door, {48, 22, 14});                                        // inside
      Shape hinge, small;
      hinge.box(cx - dw / 2 + 4, cy, 4, dh / 2, 2);                        // the door, swung open
      fill(buf, hinge, col);
      text(small, num, cx + dw / 2 - 12, cy + dh / 2 - 10, 9, 1.6f, 1.6f);
      fill(buf, small, {244, 214, 120});
      adventIcon(buf, n % 6, cx + 3, cy - 2);
    } else {
      Shape inner, t;
      inner.box(cx, cy, dw / 2 - 5, dh / 2 - 5, 4);
      fill(buf, door, col);
      fill(buf, inner, mul(col, 1.12f), 1, 0, 1);
      fill(buf, inner, col, 1, -1.5f);
      text(t, num, cx, cy, 28, 4.4f, 2.0f);
      fill(buf, t, ink);
    }
  }
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
    case F_GRANDFATHER: {  // Grandfather: Roman numerals round the ring, minute track, brass corners with scrollwork
      for (int m = 0; m < 60; m++) {
        float a = m * M60;
        if (m % 5) add(ticks).seg(px(a, 203), py(a, 203), px(a, 217), py(a, 217), 0.6f);
        else {
          HandBuilder d(a);
          d.poly({201, -1.6f, 219, -1.6f, 219, 1.6f, 201, 1.6f});
          ticks.push_back(d.sh);
        }
      }
      static const char *R[12] = {"XII", "I", "II", "III", "IIII", "V", "VI", "VII", "VIII", "IX", "X", "XI"};
      for (int h = 0; h < 12; h++) {
        float a = h * H12;
        text(nums, R[h], px(a, 178), py(a, 178), 30, 3.2f, 2.1f, a);   // feet towards the centre
      }
      fillAll(buf, ticks, {30, 26, 22});
      fillAll(buf, nums, {30, 26, 22});
      for (int c = 0; c < 4; c++) spandrel(buf, c & 1, c >> 1);
      break;
    }
    case F_CHRISTMAS: {  // Christmas: gold 12 3 6 9, ornaments at the other hours, snowflakes
      static const float flakes[][3] = {{-150, -60, 16}, {140, -150, 12}, {-60, 160, 13}, {158, 70, 14},
                                        {-128, 120, 10}, {60, -176, 9},  {-170, 30, 8},  {110, 150, 9}};
      for (auto &f : flakes) {
        Shape &s = add(ticks);
        for (int k = 0; k < 3; k++) {
          float a = k * PI2 / 6;
          float dx = sinf(a) * f[2], dy = cosf(a) * f[2];
          s.seg(CX + f[0] - dx, CY + f[1] - dy, CX + f[0] + dx, CY + f[1] + dy, 1.2f);
        }
      }
      fillAll(buf, ticks, {255, 255, 255}, 0.22f);
      ticks.clear();
      for (int m = 0; m < 60; m++)
        if (m % 5) add(ticks).disc(px(m * M60, 211), py(m * M60, 211), 1.3f);
      fillAll(buf, ticks, {240, 240, 236});
      static const C3 balls[3] = {{214, 34, 44}, {230, 188, 80}, {70, 130, 210}};
      for (int h = 0; h < 12; h++) {
        if (h % 3 == 0) continue;
        float a = h * H12;
        Shape cap, ball, shine;
        HandBuilder b(a);
        b.poly({201, -3, 207, -3, 207, 3, 201, 3});
        cap = b.sh;
        ball.disc(px(a, 191), py(a, 191), 10);
        shine.disc(px(a, 191) - 3.5f, py(a, 191) - 3.5f, 2.6f);
        fill(buf, ball, {0, 0, 0}, 0.4f, 0, 4, 2, 3);
        fill(buf, cap, {230, 188, 80});
        fill(buf, ball, balls[h % 3]);
        fill(buf, shine, {255, 255, 255}, 0.75f);
      }
      const char *q[4] = {"12", "3", "6", "9"};
      for (int i = 0; i < 4; i++) {
        float a = i * 3 * H12;
        text(add(nums), q[i], px(a, 172), py(a, 172), 40, 5.5f, 2.4f);
      }
      fillAll(buf, nums, {0, 0, 0}, 0.5f, 1.5f, 3);
      fillAll(buf, nums, {244, 214, 120});
      break;
    }
    case F_ADVENT:    // Advent: drawn by advent()
      break;
    case 3:
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
static void tempWindow(uint16_t *buf, const Theme &t, float cx, float cy, const char *label, const char *val,
                       const char *hum) {
  bool tall = hum[0] != 0;                     // three lines: label, temperature, humidity
  Shape win;
  win.box(cx, cy, 40, tall ? 44 : 30, 9);
  fill(buf, win, t.windowEdge, 1, 1.5f);
  fill(buf, win, t.window);
  Shape lab, num;
  float top = tall ? cy - 29 : cy - 15;
  text(lab, label, cx, top, 9, 1.7f, 1.8f);
  char v[12];
  bool isNum = strcmp(val, "--") != 0 && val[0];
  snprintf(v, sizeof v, "%s%s", val[0] ? val : "--", isNum ? "*" : "");
  text(num, v, cx + (isNum ? 3 : 0), top + 23, 22, 3.2f, 1.9f);
  fill(buf, lab, t.label);
  fill(buf, num, t.ink);
  if (tall) {
    Shape h;
    bool hn = strcmp(hum, "--") != 0;
    snprintf(v, sizeof v, "%s%s", hum, hn ? "%" : "");
    text(h, v, cx, cy + 27, 13, 2.0f, 2.2f);
    fill(buf, h, t.ink);
  }
}

// Pressure below 12: the barometer word over the reading (with a rising / falling arrow).
static void pressureWindow(uint16_t *buf, const Theme &t, const Dial &d) {
  const float x = CX, y = CY - 96;
  char line[24];
  snprintf(line, sizeof line, "%s%s", d.pres, d.presTrend > 0 ? " {" : d.presTrend < 0 ? " }" : "");
  Shapes word, num;
  bool two = d.presWord[0] != 0;
  if (two) {
    text(word, d.presWord, x, y - 7, 13, 2.4f, 2.0f);
    text(num, line, x, y + 11, 8, 1.6f, 1.8f);
  } else {
    text(num, line, x, y, 12, 2.0f, 1.8f);
  }
  float x0 = 1e9f, x1 = -1e9f;
  for (auto *v : {&word, &num})
    for (auto &g : *v) { x0 = fminf(x0, g.x0); x1 = fmaxf(x1, g.x1); }
  float hw = fmaxf(36, (x1 - x0) / 2 + 12);
  Shape win;
  win.box(x, y, hw, two ? 21 : 14, 8);
  fill(buf, win, t.windowEdge, 1, 1.5f);
  fill(buf, win, t.window);
  fillAll(buf, word, t.ink);
  fillAll(buf, num, two ? t.label : t.ink);
}

static void moonDisc(uint16_t *buf, const Theme &t, float phase, float mx, float my, float R);
static void moonWindow(uint16_t *buf, const Theme &t, float phase) {
  const float mx = CX, my = CY + 108, R = 25;
  Shape win;
  win.disc(mx, my, R + 5);
  fill(buf, win, t.windowEdge, 1, 1.5f);
  fill(buf, win, t.window);
  moonDisc(buf, t, phase, mx, my, R);
}
static void moonDisc(uint16_t *buf, const Theme &t, float phase, float mx, float my, float R) {
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

// ---------- digital faces: Red LED, VFD, Nixie, Flip ----------
// Big HH:MM across the middle, the date above, the weather and the moon below. The still parts (panel, unlit
// segments, tubes, cards) are the base; the lit digits are drawn every tick by drawHands.
struct Digi { C3 on, glow, ghost, text, dim; };
static const Digi DIGI[4] = {
    {{255, 44, 28}, {255, 20, 10}, {44, 7, 6}, {230, 46, 34}, {140, 26, 20}},          // Red LED
    {{130, 255, 232}, {60, 255, 210}, {14, 42, 40}, {116, 236, 214}, {60, 140, 130}},  // VFD
    {{255, 150, 60}, {255, 90, 20}, {64, 42, 32}, {255, 164, 96}, {170, 110, 70}},     // Nixie
    {{238, 238, 232}, {0, 0, 0}, {0, 0, 0}, {206, 206, 200}, {130, 130, 130}},         // Flip
};
static const Digi &digi(int f) { return DIGI[f - F_LED]; }
static bool isDigital(int f) { return f >= F_LED && f <= F_FLIP; }

// The 4 big digit boxes and the colon between them
static const float DX[4] = {31, 133, 263, 365}, DY = 96, DW = 84, DH = 170;
static const float SX[2] = {377, 415}, SY = 290, SW = 32, SH = 52;          // seconds (7-segment faces)

// One 7-segment digit: lit segments to `on`, unlit to `off` (either may be null). digit < 0 = blank.
static void seg7(Shapes *on, Shapes *off, int digit, float x, float y, float w, float h, float t) {
  static const uint8_t bits[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
  uint8_t lit = digit >= 0 && digit <= 9 ? bits[digit] : 0;
  const float g = t * 0.12f + 1, slant = 0.1f;
  auto P = [&](float u, float v, float *px_, float *py_) { *px_ = x + u + (h / 2 - v) * slant; *py_ = y + v; };
  for (int sgm = 0; sgm < 7; sgm++) {
    float u[6], v[6];
    if (sgm == 0 || sgm == 3 || sgm == 6) {                 // a, d, g: across
      float vc = sgm == 0 ? t / 2 : sgm == 3 ? h - t / 2 : h / 2, u0 = t / 2 + g, u1 = w - t / 2 - g;
      float uu[6] = {u0, u0 + t / 2, u1 - t / 2, u1, u1 - t / 2, u0 + t / 2};
      float vv[6] = {vc, vc - t / 2, vc - t / 2, vc, vc + t / 2, vc + t / 2};
      memcpy(u, uu, sizeof u); memcpy(v, vv, sizeof v);
    } else {                                               // b, c, e, f: up and down
      bool right = sgm == 1 || sgm == 2, low = sgm == 2 || sgm == 4;
      float uc = right ? w - t / 2 : t / 2;
      float v0 = low ? h / 2 + g : t / 2 + g, v1 = low ? h - t / 2 - g : h / 2 - g;
      float uu[6] = {uc, uc + t / 2, uc + t / 2, uc, uc - t / 2, uc - t / 2};
      float vv[6] = {v0, v0 + t / 2, v1 - t / 2, v1, v1 - t / 2, v0 + t / 2};
      memcpy(u, uu, sizeof u); memcpy(v, vv, sizeof v);
    }
    // segment order a b c d e f g = bits 0..6
    static const int bitOf[7] = {0, 1, 2, 3, 4, 5, 6};
    float px6[6], py6[6];
    for (int i = 0; i < 6; i++) P(u[i], v[i], &px6[i], &py6[i]);
    Shapes *dst = (lit >> bitOf[sgm]) & 1 ? on : off;
    if (dst) add(*dst).poly(px6, py6, 6);
  }
}
// segment index -> which of a..g it is: the loop above uses 0 a, 1 b, 2 c, 3 d, 4 e, 5 f, 6 g

static void colonDots(Shapes &v, float cx, float y, float h, float r, float slant = 0.1f) {
  add(v).disc(cx + (h * 0.18f) * slant, y + h * 0.32f, r);
  add(v).disc(cx - (h * 0.18f) * slant, y + h * 0.68f, r);
}

static void glowFill(uint16_t *buf, const Shapes &v, const Digi &c, float soft) {
  fillAll(buf, v, c.glow, 0.55f, 3, soft);
  fillAll(buf, v, c.on);
}

static void digitalBase(uint16_t *buf, int f) {
  const Digi &c = digi(f);
  if (f == F_LED || f == F_VFD) {
    Shape panel;
    panel.box(CX, 214, 228, 136, 18);
    fill(buf, panel, {0, 0, 0}, 0.5f, 2, 8, 0, 4);
    fill(buf, panel, f == F_LED ? C3{24, 4, 4} : C3{4, 16, 18});
    Shape shine;
    shine.box(CX, 86, 214, 3, 3);
    fill(buf, shine, {255, 255, 255}, 0.06f);
    Shapes off;
    for (int i = 0; i < 4; i++) seg7(nullptr, &off, -1, DX[i], DY, DW, DH, 17);
    for (int i = 0; i < 2; i++) seg7(nullptr, &off, -1, SX[i], SY, SW, SH, 7);
    colonDots(off, CX, DY, DH, 8);
    fillAll(buf, off, c.ghost);
    Shapes ap;
    text(ap, "AM", 70, 300, 16, 2.8f, 1.8f);
    text(ap, "PM", 70, 330, 16, 2.8f, 1.8f);
    fillAll(buf, ap, c.ghost);
  } else if (f == F_NIXIE) {
    for (int i = 0; i < 4; i++) {
      Shape tube, rim, hl;
      tube.box(DX[i] + DW / 2, DY + DH / 2, DW / 2 + 8, DH / 2 + 18, 34);
      hl.box(DX[i] + 16, DY + DH / 2, 3, DH / 2 - 10, 3);
      fill(buf, tube, {0, 0, 0}, 0.5f, 2, 10, 3, 6);
      fill(buf, tube, {90, 80, 70}, 1, 1.5f);
      fill(buf, tube, {20, 15, 13});
      Shapes g;
      text(g, "8", DX[i] + DW / 2, DY + DH / 2, 150, 4.5f);
      text(g, "0", DX[i] + DW / 2, DY + DH / 2, 150, 4.5f);
      fillAll(buf, g, c.ghost);
      fill(buf, hl, {255, 255, 255}, 0.07f);
      Shape base;
      base.box(DX[i] + DW / 2, DY + DH + 26, DW / 2 + 4, 8, 3);
      fill(buf, base, {40, 36, 32});
    }
  } else {                                               // Flip: two cards
    for (int k = 0; k < 2; k++) {
      float x0 = k ? 246 : 28, x1 = k ? 452 : 234, y0 = 92, y1 = 308, ym = (y0 + y1) / 2;
      Shape card, top;
      card.box((x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2, (y1 - y0) / 2, 16);
      top.box((x0 + x1) / 2, (y0 + ym) / 2, (x1 - x0) / 2, (ym - y0) / 2, 16);
      fill(buf, card, {0, 0, 0}, 0.6f, 2, 12, 0, 6);
      fill(buf, card, {44, 44, 48});
      fill(buf, top, {54, 54, 58});
      (void)top;
    }
  }
}

// The time on a digital face, in two layers. Minute layer (slow, drawn once a minute): the big digits.
// Second layer (quick, every tick): blinking colon, seconds, AM/PM.
static void digitalMinute(uint16_t *buf, int f, int h24, int m) {
  const Digi &c = digi(f);
  int h12 = (h24 + 11) % 12 + 1;
  if (f == F_LED || f == F_VFD) {
    Shapes on;
    seg7(&on, nullptr, h12 >= 10 ? 1 : -1, DX[0], DY, DW, DH, 17);
    seg7(&on, nullptr, h12 % 10, DX[1], DY, DW, DH, 17);
    seg7(&on, nullptr, m / 10, DX[2], DY, DW, DH, 17);
    seg7(&on, nullptr, m % 10, DX[3], DY, DW, DH, 17);
    glowFill(buf, on, c, f == F_VFD ? 12 : 10);
  } else if (f == F_NIXIE) {
    char d[4] = {(char)('0' + h12 / 10), (char)('0' + h12 % 10), (char)('0' + m / 10), (char)('0' + m % 10)};
    for (int i = 0; i < 4; i++) {
      if (i == 0 && h12 < 10) continue;
      char s1[2] = {d[i], 0};
      Shapes g;
      text(g, s1, DX[i] + DW / 2, DY + DH / 2, 150, 7);
      fillAll(buf, g, c.glow, 0.6f, 3, 12);
      fillAll(buf, g, c.on);
      fillAll(buf, g, {255, 226, 180}, 1, -2.2f);
    }
  } else {                                               // Flip
    char hs[4], ms[4];
    snprintf(hs, sizeof hs, "%d", h12);
    snprintf(ms, sizeof ms, "%02d", m);
    Shapes dg;
    text(dg, hs, 131, 200, 136, 19, 2.6f);
    text(dg, ms, 349, 200, 136, 19, 2.6f);
    fillAll(buf, dg, c.on);
    for (int k = 0; k < 2; k++) {                          // the split across each card, over the digits
      float x0 = k ? 246 : 28, x1 = k ? 452 : 234;
      Shape line, n1, n2;
      line.box((x0 + x1) / 2, 200, (x1 - x0) / 2, 2, 0);
      n1.box(x0 + 3, 200, 4, 7, 2);
      n2.box(x1 - 3, 200, 4, 7, 2);
      fill(buf, line, {14, 14, 16});
      fill(buf, n1, {14, 14, 16});
      fill(buf, n2, {14, 14, 16});
    }
  }
}

static void digitalSecond(uint16_t *buf, int f, int h24, int sec, bool showSeconds) {
  const Digi &c = digi(f);
  bool colon = !showSeconds || sec % 2 == 0;               // the colon blinks with the seconds
  if (f == F_LED || f == F_VFD) {
    Shapes on;
    if (colon) colonDots(on, CX, DY, DH, 8);
    if (showSeconds) {
      seg7(&on, nullptr, sec / 10, SX[0], SY, SW, SH, 7);
      seg7(&on, nullptr, sec % 10, SX[1], SY, SW, SH, 7);
    }
    text(on, h24 < 12 ? "AM" : "PM", 70, h24 < 12 ? 300 : 330, 16, 2.8f, 1.8f);
    glowFill(buf, on, c, f == F_VFD ? 12 : 10);
  } else if (f == F_NIXIE) {
    Shapes dots;
    if (colon) colonDots(dots, CX, DY, DH, 6, 0);
    glowFill(buf, dots, c, 10);
    Shapes ap;
    char sx[12];
    if (showSeconds) snprintf(sx, sizeof sx, "%s  %02d", h24 < 12 ? "AM" : "PM", sec);
    else snprintf(sx, sizeof sx, "%s", h24 < 12 ? "AM" : "PM");
    text(ap, sx, CX, 322, 18, 3, 2.0f);
    glowFill(buf, ap, c, 8);
  } else {                                               // Flip
    Shapes small;
    text(small, h24 < 12 ? "AM" : "PM", 56, 290, 13, 2.4f, 1.8f);
    if (showSeconds) { char sx[4]; snprintf(sx, sizeof sx, "%02d", sec); text(small, sx, 424, 290, 13, 2.4f, 1.8f); }
    fillAll(buf, small, c.dim);
  }
}

static void moonDisc(uint16_t *buf, const Theme &t, float phase, float mx, float my, float R);

// Date above, weather and moon below.
static void digitalInfo(uint16_t *buf, int f, const Dial &d) {
  const Digi &c = digi(f);
  static const char *days[7] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
  static const char *mons[12] = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};
  char line[48];
  Shapes top;
  if (d.month >= 1 && d.month <= 12) {
    snprintf(line, sizeof line, "%s  %s %d", days[d.wday % 7], mons[d.month - 1], d.mday);
    text(top, line, CX, 46, 24, 3.4f, 2.2f);
  }
  Shapes info, info2;
  if (d.temps) {
    char a[24], b[24];
    snprintf(a, sizeof a, "IN %s%s%s%s", d.in, strcmp(d.in, "--") ? "*" : "", d.inHum[0] ? "  " : "", d.inHum);
    if (d.inHum[0] && strcmp(d.inHum, "--")) strcat(a, "%");
    snprintf(b, sizeof b, "OUT %s%s%s%s", d.out, strcmp(d.out, "--") ? "*" : "", d.outHum[0] ? "  " : "", d.outHum);
    if (d.outHum[0] && strcmp(d.outHum, "--")) strcat(b, "%");
    text(info, a, 128, 384, 20, 3, 2.0f);
    text(info, b, 352, 384, 20, 3, 2.0f);
  }
  if (d.pres[0]) {
    snprintf(line, sizeof line, "%s%s%s%s", d.presWord, d.presWord[0] ? "  " : "", d.pres,
             d.presTrend > 0 ? " {" : d.presTrend < 0 ? " }" : "");
    text(info2, line, d.moon ? 214 : CX, 428, 16, 2.4f, 1.9f);
  }
  if (f == F_FLIP) {
    fillAll(buf, top, c.text);
    fillAll(buf, info, c.text);
    fillAll(buf, info2, c.dim);
  } else {
    glowFill(buf, top, {c.text, c.glow, c.ghost, c.text, c.dim}, 8);
    glowFill(buf, info, {c.text, c.glow, c.ghost, c.text, c.dim}, 8);
    fillAll(buf, info2, c.dim);
  }
  if (d.moon) moonDisc(buf, THEMES[f], d.moonPhase, d.pres[0] ? 400 : CX, d.pres[0] ? 428 : 430, 16);
}

void baseKey(const Dial &d, char *out, int n) {
  int f = d.face < 0 || d.face >= FACE_COUNT ? 0 : d.face;
  if (f == F_ADVENT) snprintf(out, n, "%d|%d/%d|%d", f, d.month, d.mday, d.daysToXmas);   // Advent: changes daily
  else snprintf(out, n, "%d|%d", f, f == 3 ? (int)d.moon : 0);                   // Midnight's VI moves for the moon
}
void renderBase(uint16_t *buf, const Dial &d) {
  int f = d.face < 0 || d.face >= FACE_COUNT ? 0 : d.face;
  background(buf, f);
  if (f == F_ADVENT) advent(buf, d);                   // the Advent calendar fills the screen: no windows
  else if (isDigital(f)) digitalBase(buf, f);
  else markers(buf, f, d.moon);
}
void renderDial(uint16_t *buf, const Dial &d) {
  renderBase(buf, d);
  renderExtras(buf, d);
}
void renderExtras(uint16_t *buf, const Dial &d) {
  int f = d.face < 0 || d.face >= FACE_COUNT ? 0 : d.face;
  const Theme &t = THEMES[f];
  if (f == F_ADVENT) return;
  if (isDigital(f)) { digitalInfo(buf, f, d); return; }
  if (d.temps) {
    tempWindow(buf, t, CX - 96, CY, "IN", d.in, d.inHum);
    tempWindow(buf, t, CX + 96, CY, "OUT", d.out, d.outHum);
  }
  if (f == F_CHRISTMAS && d.daysToXmas >= 0) countdownWindow(buf, t, d.daysToXmas);   // in place of the pressure
  else if (d.pres[0]) pressureWindow(buf, t, d);
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
    case F_GRANDFATHER:  // Grandfather: spade hour, a slim minute with a bead, fine seconds
      if (which == 0) {
        h.rect(-22, 64, 6); h.disc(36, 7); h.disc(-22, 7);
        h.poly({58, 0, 76, -14, 98, 0, 76, 14}); h.disc(72, 8); h.poly({94, -4, 120, 0, 94, 4});
      } else if (which == 1) { h.rect(-28, 170, 4); h.disc(118, 5.5f); h.disc(-28, 6); h.poly({160, 0, 174, -6, 204, 0, 174, 6}); }
      else { h.line(-40, 190, 1.0f); h.disc(-30, 5); }
      break;
    case F_CHRISTMAS:  // Christmas: candy canes (stripes added in drawHands)
      if (which == 0) h.line(-16, 104, 6.5f);
      else if (which == 1) h.line(-20, 178, 5);
      else { h.line(-36, 196, 1.3f); h.disc(186, 5); }
      break;
    case 3:
    default:  // Midnight: slim dauphine
      if (which == 0) { h.poly({-18, -7, 126, 0, -18, 7}); h.disc(-18, 7); }
      else if (which == 1) { h.poly({-24, -5.5f, 206, 0, -24, 5.5f}); h.disc(-24, 5.5f); }
      else { h.line(-38, 204, 1.1f); h.disc(-28, 4.5f); }
      break;
  }
  out = h.sh;
}

// Red stripes across a candy-cane hand from t0 to t1, w wide.
static void caneStripes(uint16_t *buf, float a, float t0, float t1, float w) {
  for (float t = t0; t + 9 < t1; t += 13) {
    HandBuilder h(a);
    h.poly({t, -w / 2, t + 5, -w / 2, t + 9, w / 2, t + 4, w / 2});
    fill(buf, h.sh, {204, 30, 40});
  }
}

static void adventTime(uint16_t *buf, float hour, float minute) {
  int h = (int)hour % 24, m = (int)minute % 60;
  char t[12];
  snprintf(t, sizeof t, "%d:%02d", (h + 11) % 12 + 1, m);
  Shapes a, b;
  text(a, t, CX - 16, 54, 54, 7, 2.2f);
  float right = 0;
  for (auto &g : a) right = fmaxf(right, g.x1);
  text(b, h < 12 ? "AM" : "PM", right + 22, 70, 16, 2.6f, 1.8f);
  fillAll(buf, a, {0, 0, 0}, 0.4f, 1, 4);
  fillAll(buf, a, {244, 214, 120});
  fillAll(buf, b, {244, 214, 120});
}

bool faceHasMinuteLayer(int f) { return f == F_ADVENT || isDigital(f); }

void drawMinuteLayer(uint16_t *buf, int face, int h24, int minute) {
  int f = face < 0 || face >= FACE_COUNT ? 0 : face;
  if (f == F_ADVENT) adventTime(buf, h24, minute);
  else if (isDigital(f)) digitalMinute(buf, f, h24 % 24, minute % 60);
}

void drawHands(uint16_t *buf, int face, float hour, float minute, float second, bool showSeconds) {
  int f = face < 0 || face >= FACE_COUNT ? 0 : face;
  if (faceHasMinuteLayer(f)) drawMinuteLayer(buf, f, (int)hour, (int)minute);
  drawSecondLayer(buf, f, hour, minute, second, showSeconds);
}

void drawSecondLayer(uint16_t *buf, int face, float hour, float minute, float second, bool showSeconds) {
  int f = face < 0 || face >= FACE_COUNT ? 0 : face;
  const Theme &t = THEMES[f];
  if (f == F_ADVENT) return;
  if (isDigital(f)) { digitalSecond(buf, f, (int)hour % 24, (int)second % 60, showSeconds); return; }
  float ah = fmodf(hour, 12) / 12 * PI2, am = minute / 60 * PI2, as = second / 60 * PI2;
  Shape s;
  handShapes(f, 0, ah, s);
  fill(buf, s, {0, 0, 0}, 0.42f, 0, 7, 3, 5);
  fill(buf, s, t.handEdge, 1, 1.1f);
  fill(buf, s, t.hands);
  if (f == F_CHRISTMAS) caneStripes(buf, ah, -10, 100, 13);
  handShapes(f, 1, am, s);
  fill(buf, s, {0, 0, 0}, 0.42f, 0, 7, 4, 6);
  fill(buf, s, t.handEdge, 1, 1.1f);
  fill(buf, s, t.hands);
  if (f == F_CHRISTMAS) caneStripes(buf, am, -14, 176, 10);
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

void init() { glyph('0'); }
bool faceHasSeconds(int f) { return f != F_ADVENT; }

const char *faceName(int f) {
  static const char *n[FACE_COUNT] = {"Gatsby", "Skyline",   "Ivory", "Midnight", "Grandfather", "Christmas",
                                      "Advent", "Red LED", "VFD",   "Nixie",    "Flip"};
  return f >= 0 && f < FACE_COUNT ? n[f] : n[0];
}

}  // namespace clk
