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
enum { F_GATSBY, F_SKYLINE, F_IVORY, F_MIDNIGHT, F_GRANDFATHER,
       F_NEWYEAR, F_VALENTINE, F_PATRICK, F_EASTER, F_JULY4, F_HALLOWEEN, F_THANKS, F_CHRISTMAS, F_ADVENT,
       F_LED, F_NIXIE, F_FLIP };
static_assert(F_FLIP + 1 == FACE_COUNT, "clock.h FACE_COUNT");
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
struct Ell { float x, y, rx, ry, ca, sa; };   // turned by an angle (cos, sin)

struct Shape {
  std::vector<Seg> segs;
  std::vector<Disc> discs;
  std::vector<Box> boxes;
  std::vector<Poly> polys;
  std::vector<Ell> ells;
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
  void ell(float x, float y, float rx, float ry, float rot = 0) {
    ells.push_back({x, y, rx, ry, cosf(rot), sinf(rot)}); grow(x, y, rx > ry ? rx : ry);
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
    for (const Ell &e : ells) {          // close to the true distance near the edge (good enough for coverage)
      float dx = px - e.x, dy = py - e.y, u = dx * e.ca + dy * e.sa, w = -dx * e.sa + dy * e.ca;
      float ux = u / e.rx, wy = w / e.ry, k0 = sqrtf(ux * ux + wy * wy);
      float k1 = sqrtf(ux * ux / (e.rx * e.rx) + wy * wy / (e.ry * e.ry));
      float v = k1 > 1e-6f ? k0 * (k0 - 1) / k1 : -(e.rx < e.ry ? e.rx : e.ry);
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
    g['\''] = {0, {{0, 0, 0, 2.6f}}};
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
    // New Year: midnight and gold, fireworks
    {{240, 206, 120}, {190, 196, 214}, {8, 8, 18}, {212, 172, 84}, {238, 232, 210}, {26, 26, 40},
     {236, 204, 120}, {80, 60, 20}, {226, 80, 120}, {236, 204, 120}, {40, 30, 14}},
    // Valentine's: red and pink, hearts
    {{255, 226, 232}, {255, 170, 196}, {74, 6, 26}, {244, 170, 170}, {255, 240, 236}, {70, 20, 34},
     {252, 240, 242}, {110, 14, 38}, {246, 206, 110}, {252, 240, 242}, {150, 20, 50}},
    // St. Patrick's: green and gold, shamrocks
    {{244, 214, 120}, {200, 236, 190}, {6, 40, 18}, {212, 172, 84}, {244, 244, 220}, {18, 50, 30},
     {244, 214, 120}, {70, 50, 14}, {246, 146, 50}, {244, 214, 120}, {20, 60, 30}},
    // Easter: pastel lilac, eggs
    {{92, 52, 116}, {150, 96, 160}, {250, 246, 252}, {150, 110, 180}, {252, 248, 230}, {130, 120, 160},
     {96, 56, 120}, {96, 56, 120}, {232, 104, 156}, {96, 56, 120}, {250, 220, 120}},
    // July 4th: navy, red and white, stars
    {{248, 248, 250}, {180, 200, 255}, {8, 14, 44}, {236, 236, 240}, {244, 244, 236}, {24, 30, 64},
     {248, 248, 250}, {150, 20, 32}, {230, 40, 52}, {248, 248, 250}, {200, 30, 44}},
    // Halloween: purple dark, a pumpkin glow, bats
    {{255, 160, 50}, {200, 150, 230}, {16, 6, 22}, {255, 140, 30}, {252, 240, 200}, {40, 20, 40},
     {236, 228, 206}, {26, 10, 20}, {140, 230, 70}, {236, 228, 206}, {26, 10, 20}},
    // Thanksgiving: harvest brown, autumn leaves
    {{250, 232, 196}, {236, 176, 90}, {50, 24, 10}, {210, 140, 70}, {252, 240, 210}, {60, 34, 20},
     {250, 236, 210}, {60, 30, 10}, {236, 110, 34}, {250, 236, 210}, {120, 56, 20}},
    // Christmas: green, candy-cane hands, ornaments
    {{244, 214, 120}, {236, 236, 236}, {10, 44, 24}, {222, 182, 92}, {250, 245, 220}, {20, 50, 35},
     {252, 252, 252}, {120, 20, 24}, {244, 204, 96}, {204, 30, 40}, {244, 204, 96}},
    // Advent: red, 24 doors, the time as digits
    {{244, 214, 120}, {250, 236, 210}, {60, 10, 14}, {222, 182, 92}, {250, 245, 220}, {40, 20, 20},
     {244, 214, 120}, {60, 10, 14}, {244, 214, 120}, {244, 214, 120}, {60, 10, 14}},
    // Digital, Nixie, Flip (only the moon colours are used)
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
        case F_LED: {  // black plastic
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
        case F_NEWYEAR: {  // New Year: midnight blue-black, faint gold rays, a gold bezel
          float t = r / 222;
          c = mix({22, 20, 44}, {4, 4, 12}, t);
          c = mix(c, mul(c, 1.35f), wedge(a, r, 72) * clamp01((r - 30) / 60) * 0.6f);
          c = mix(c, GOLD, band(r, 218.5f, 221.5f));
          c = mix(c, {4, 4, 8}, band(r, 221.5f, 226));
          if (r > 225 && r < 239) c = mix(c, metal(GOLD, r, 226, 238, a), band(r, 226, 238));
          c = mix(c, {3, 3, 3}, clamp01(r - 238 + 0.5f));
          break;
        }
        case F_VALENTINE: {  // Valentine's: deep red, soft pink rays, rose gold
          float t = r / 222;
          c = mix({190, 32, 64}, {90, 8, 32}, t * t);
          c = mix(c, mix(c, {255, 130, 170}, 0.12f), wedge(a, r, 36) * clamp01((r - 30) / 50));
          c = mix(c, {255, 196, 210}, band(r, 218.5f, 221));
          c = mix(c, {50, 4, 18}, band(r, 221, 226));
          if (r > 225 && r < 239) c = mix(c, metal({226, 160, 150}, r, 226, 238, a), band(r, 226, 238));
          c = mix(c, {3, 3, 3}, clamp01(r - 238 + 0.5f));
          break;
        }
        case F_PATRICK: {  // St. Patrick's: emerald, faint rays, gold
          float t = r / 222;
          c = mix({34, 130, 66}, {6, 54, 26}, t * t);
          c = mix(c, mul(c, 1.14f), wedge(a, r, 48) * clamp01((r - 30) / 50));
          c = mix(c, GOLD, band(r, 218.5f, 221.5f));
          c = mix(c, {4, 24, 10}, band(r, 221.5f, 226));
          if (r > 225 && r < 239) c = mix(c, metal(GOLD, r, 226, 238, a), band(r, 226, 238));
          c = mix(c, {3, 3, 3}, clamp01(r - 238 + 0.5f));
          break;
        }
        case F_EASTER: {  // Easter: pale lilac, a pastel-striped bezel
          float t = r / 222;
          c = mix({246, 240, 252}, {206, 192, 236}, t * t);
          c = mix(c, {150, 110, 180}, band(r, 218, 220));
          if (r > 220) {
            static const C3 pas[4] = {{250, 170, 200}, {252, 228, 130}, {150, 226, 190}, {150, 196, 246}};
            float u = a / PI2 * 32 + (r - 222) / 16 * 0.5f, f = u - floorf(u);
            int i = ((int)floorf(u)) & 3;
            float e = fminf(f, 1 - f) * PI2 * r / 32;                    // pixels to the stripe edge
            C3 s = mix(mix(pas[i], pas[(i + (f < 0.5f ? 3 : 1)) & 3], 0.5f), pas[i], clamp01(e + 0.5f));
            if (r < 239) c = mix(c, metal(s, r, 222, 238, a + 0.8f), band(r, 222, 238));
          }
          c = mix(c, {16, 12, 22}, clamp01(r - 238 + 0.5f));
          break;
        }
        case F_JULY4: {  // July 4th: navy, a red and white striped bezel
          float t = r / 222;
          c = mix({30, 44, 110}, {8, 14, 50}, t * t);
          c = mix(c, {236, 236, 240}, band(r, 218, 221));
          if (r > 221) {
            float u = a / PI2 * 26, f = u - floorf(u);
            float dd = f < 0.5f ? fminf(f, 0.5f - f) : -fminf(f - 0.5f, 1 - f);   // >0 inside a red stripe
            C3 st = mix({246, 246, 244}, {204, 30, 44}, clamp01(dd * PI2 * r / 26 + 0.5f));
            if (r < 239) c = mix(c, metal(st, r, 222, 238, a), band(r, 222, 238));
          }
          c = mix(c, {3, 3, 3}, clamp01(r - 238 + 0.5f));
          break;
        }
        case F_HALLOWEEN: {  // Halloween: purple dark in the middle, a pumpkin-orange glow round the edge, iron
          float t = r / 216;
          C3 in = mix({22, 6, 32}, {58, 14, 70}, clamp01(t / 0.55f));
          float glow = clamp01((t - 0.5f) / 0.38f);
          c = mix(in, {226, 104, 18}, glow * glow * (3 - 2 * glow));
          c = mix(c, {40, 10, 16}, clamp01((r - 206) / 12));
          if (r > 222 && r < 239) c = mix(c, metal({110, 96, 118}, r, 224, 238, a), band(r, 224, 238));
          c = mix(c, {3, 2, 4}, clamp01(r - 238 + 0.5f));
          break;
        }
        case F_THANKS: {  // Thanksgiving: warm brown, harvest rays, copper
          float t = r / 222;
          c = mix({156, 90, 40}, {78, 36, 14}, t * t);
          c = mix(c, mul(c, 1.12f), wedge(a, r, 40) * clamp01((r - 30) / 50));
          c = mix(c, {240, 210, 150}, band(r, 218.5f, 221));
          c = mix(c, {40, 18, 6}, band(r, 221, 226));
          if (r > 225 && r < 239) c = mix(c, metal({200, 124, 66}, r, 226, 238, a), band(r, 226, 238));
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

// ---------- holidays: the dates, the countdown window and the decorations ----------
enum { H_NONE, H_NEWYEAR, H_VALENTINE, H_PATRICK, H_EASTER, H_JULY4, H_HALLOWEEN, H_THANKS, H_XMAS };
int holidayOf(int f) {
  switch (f) {
    case F_NEWYEAR: return H_NEWYEAR;
    case F_VALENTINE: return H_VALENTINE;
    case F_PATRICK: return H_PATRICK;
    case F_EASTER: return H_EASTER;
    case F_JULY4: return H_JULY4;
    case F_HALLOWEEN: return H_HALLOWEEN;
    case F_THANKS: return H_THANKS;
    case F_CHRISTMAS: case F_ADVENT: return H_XMAS;
    default: return H_NONE;
  }
}
static long dayNumber(int y, int m, int d) {          // days since 1970-01-01 (proleptic Gregorian)
  y -= m <= 2;
  long era = (y >= 0 ? y : y - 399) / 400;
  long yoe = y - era * 400, doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  return era * 146097 + yoe * 365 + yoe / 4 - yoe / 100 + doy - 719468;
}
static void holidayDate(int h, int y, int &m, int &d) {
  switch (h) {
    case H_NEWYEAR: m = 1; d = 1; break;
    case H_VALENTINE: m = 2; d = 14; break;
    case H_PATRICK: m = 3; d = 17; break;
    case H_EASTER: {                                   // the anonymous Gregorian computus
      int a = y % 19, b = y / 100, c = y % 100, e = (19 * a + b - b / 4 - (b - (8 * b + 13) / 25) + 15) % 30;
      int l = (32 + 2 * (b % 4) + 2 * (c / 4) - e - c % 4) % 7, k = (a + 11 * e + 19 * l) / 433;
      m = (e + l - 7 * k + 90) / 25; d = (e + l - 7 * k + 33 * m + 19) % 32;
      break;
    }
    case H_JULY4: m = 7; d = 4; break;
    case H_HALLOWEEN: m = 10; d = 31; break;
    case H_THANKS: {                                   // the fourth Thursday in November
      int w = (int)((dayNumber(y, 11, 1) % 7 + 11) % 7);   // 0 = Sunday (1970-01-01 was a Thursday)
      m = 11; d = 1 + (4 - w + 7) % 7 + 21;
      break;
    }
    default: m = 12; d = 25; break;
  }
}
int daysToHoliday(const Dial &d, int h) {
  if (h == H_NONE || d.month < 1) return -1;
  if (d.year < 1900) return h == H_XMAS ? d.daysToXmas : -1;
  long today = dayNumber(d.year, d.month, d.mday);
  for (int y = d.year; y <= d.year + 1; y++) {
    int m, dd;
    holidayDate(h, y, m, dd);
    long t = dayNumber(y, m, dd);
    if (t >= today) return (int)(t - today);
  }
  return -1;
}
// Two lines: "12 DAYS" / "TO HALLOWEEN", or on the day "HAPPY" / "HALLOWEEN!".
static void holidayLines(int h, int days, char *l1, char *l2, size_t n) {
  static const char *to[] = {"", "TO NEW YEAR", "TO VALENTINE'S", "TO ST. PATRICK'S", "TO EASTER", "TO JULY 4TH",
                             "TO HALLOWEEN", "TO THANKSGIVING", "TO CHRISTMAS"};
  static const char *day[] = {"", "NEW YEAR!", "VALENTINE'S DAY", "ST. PATRICK'S DAY", "EASTER!", "4TH OF JULY!",
                              "HALLOWEEN!", "THANKSGIVING!", "CHRISTMAS!"};
  if (days == 0) { snprintf(l1, n, h == H_XMAS ? "MERRY" : "HAPPY"); snprintf(l2, n, "%s", day[h]); }
  else if (days == 1) { snprintf(l1, n, "1 DAY"); snprintf(l2, n, "%s", to[h]); }
  else { snprintf(l1, n, "%d DAYS", days); snprintf(l2, n, "%s", to[h]); }
}
static void countdownWindow(uint16_t *buf, const Theme &t, int h, int days) {
  char l1[24], l2[24];
  holidayLines(h, days, l1, l2, sizeof l1);
  const float x = CX, y = CY - 94;
  Shapes a, b;
  text(a, l1, x, y - 6, 17, 2.8f, 2.0f);
  text(b, l2, x, y + 14, 8, 1.6f, 1.8f);
  float x0 = 1e9f, x1 = -1e9f;
  for (auto *v : {&a, &b})
    for (auto &g : *v) { x0 = fminf(x0, g.x0); x1 = fmaxf(x1, g.x1); }
  Shape win;
  win.box(x, y, fmaxf(64, (x1 - x0) / 2 + 12), 24, 10);
  fill(buf, win, t.windowEdge, 1, 1.5f);
  fill(buf, win, t.window);
  fillAll(buf, a, t.ink);
  fillAll(buf, b, t.label);
}

// Turn (u, v) by angle `rot` (clockwise) about (x, y): v is "down" when rot = 0.
struct Frame {
  float x, y, c, s, k;
  Frame(float x_, float y_, float rot, float size) : x(x_), y(y_), c(cosf(rot)), s(sinf(rot)), k(size) {}
  float X(float u, float v) const { return x + (u * c - v * s) * k; }
  float Y(float u, float v) const { return y + (u * s + v * c) * k; }
};
// A heart about 2 x 2 units, its point down (towards +v).
static void heart(Shape &sh, const Frame &f) {
  sh.disc(f.X(-0.5f, -0.3f), f.Y(-0.5f, -0.3f), 0.55f * f.k);
  sh.disc(f.X(0.5f, -0.3f), f.Y(0.5f, -0.3f), 0.55f * f.k);
  float x[4] = {f.X(-1.02f, -0.12f), f.X(0, -0.2f), f.X(1.02f, -0.12f), f.X(0, 1.0f)};
  float y[4] = {f.Y(-1.02f, -0.12f), f.Y(0, -0.2f), f.Y(1.02f, -0.12f), f.Y(0, 1.0f)};
  sh.poly(x, y, 4);
}
// A five-pointed star, radius 1 unit, a point up.
static void star(Shape &sh, const Frame &f) {
  float px5[5], py5[5];
  for (int i = 0; i < 5; i++) { float an = (i + 0.5f) * PI2 / 5; px5[i] = f.X(0.4f * sinf(an), -0.4f * cosf(an)); py5[i] = f.Y(0.4f * sinf(an), -0.4f * cosf(an)); }
  sh.poly(px5, py5, 5);
  for (int i = 0; i < 5; i++) {
    float an = i * PI2 / 5;
    float tx[3] = {f.X(sinf(an), -cosf(an)), px5[(i + 4) % 5], px5[i]}, ty[3] = {f.Y(sinf(an), -cosf(an)), py5[(i + 4) % 5], py5[i]};
    sh.poly(tx, ty, 3);
  }
}
// Three heart leaves and a stem.
static void shamrock(Shape &leaves, Shape &stem, const Frame &f) {
  for (int i = 0; i < 3; i++) {
    float an = i * PI2 / 3, ux = 0.58f * sinf(an), vy = -0.58f * cosf(an);
    heart(leaves, Frame(f.X(ux, vy), f.Y(ux, vy), atan2f(f.s, f.c) + an, f.k * 0.58f));
  }
  stem.seg(f.X(0, 0.1f), f.Y(0, 0.1f), f.X(0.25f, 0.75f), f.Y(0.25f, 0.75f), 0.11f * f.k);
  stem.seg(f.X(0.25f, 0.75f), f.Y(0.25f, 0.75f), f.X(0.5f, 1.15f), f.Y(0.5f, 1.15f), 0.1f * f.k);
}
// Fireworks: n sparks out from (x, y), r long, with a bright tip.
static void burst(uint16_t *buf, float x, float y, float r, int n, C3 col, float phase) {
  Shape lines, tips;
  for (int i = 0; i < n; i++) {
    float an = (i + phase) * PI2 / n, len = r * (i % 2 ? 0.78f : 1.0f);
    lines.seg(x + sinf(an) * r * 0.25f, y - cosf(an) * r * 0.25f, x + sinf(an) * len * 0.86f, y - cosf(an) * len * 0.86f, 1.0f);
    tips.disc(x + sinf(an) * len, y - cosf(an) * len, 1.8f);
  }
  fill(buf, tips, col, 0.5f, 2, 5);
  fill(buf, lines, col, 0.8f);
  fill(buf, tips, mix(col, {255, 255, 255}, 0.5f));
}
// A jack-o'-lantern, about 2 units wide.
static void pumpkin(uint16_t *buf, float x, float y, float k) {
  Shape body, ribs, stem, face;
  body.ell(x, y, 0.62f * k, 0.78f * k);
  body.ell(x - 0.42f * k, y + 0.03f * k, 0.55f * k, 0.72f * k);
  body.ell(x + 0.42f * k, y + 0.03f * k, 0.55f * k, 0.72f * k);
  ribs.ell(x - 0.32f * k, y + 0.02f * k, 0.08f * k, 0.66f * k);
  ribs.ell(x + 0.32f * k, y + 0.02f * k, 0.08f * k, 0.66f * k);
  stem.box(x + 0.04f * k, y - 0.84f * k, 0.11f * k, 0.2f * k, 0.05f * k);
  float ex[3] = {x - 0.48f * k, x - 0.14f * k, x - 0.31f * k}, ey[3] = {y - 0.1f * k, y - 0.1f * k, y - 0.4f * k};
  face.poly(ex, ey, 3);
  float fx[3] = {x + 0.14f * k, x + 0.48f * k, x + 0.31f * k};
  face.poly(fx, ey, 3);
  float mx[6] = {x - 0.6f * k, x + 0.6f * k, x + 0.46f * k, x + 0.2f * k, x - 0.2f * k, x - 0.46f * k};
  float my[6] = {y + 0.12f * k, y + 0.12f * k, y + 0.38f * k, y + 0.5f * k, y + 0.5f * k, y + 0.38f * k};
  face.poly(mx, my, 6);
  fill(buf, body, {0, 0, 0}, 0.45f, 0, 5, 2, 3);
  fill(buf, body, {120, 44, 6}, 1, 1.2f);
  fill(buf, body, {246, 128, 24});
  fill(buf, ribs, {206, 96, 14}, 0.8f);
  fill(buf, stem, {70, 90, 30});
  fill(buf, face, {255, 220, 90}, 0.5f, 1.5f, 3);
  fill(buf, face, {255, 236, 140});
}
// A bat, about 2 units wide.
static void bat(Shape &sh, float x, float y, float k) {
  sh.ell(x, y, 0.16f * k, 0.28f * k);
  sh.disc(x, y - 0.3f * k, 0.13f * k);
  for (int sd = -1; sd <= 1; sd += 2) {
    float ex[3] = {x + sd * 0.04f * k, x + sd * 0.12f * k, x + sd * 0.15f * k}, ey[3] = {y - 0.36f * k, y - 0.54f * k, y - 0.3f * k};
    sh.poly(ex, ey, 3);
    float wx[4] = {x, x + sd * 0.55f * k, x + sd * 0.62f * k, x + sd * 0.2f * k}, wy[4] = {y - 0.18f * k, y - 0.36f * k, y + 0.06f * k, y + 0.12f * k};
    sh.poly(wx, wy, 4);
    float vx[4] = {x + sd * 0.5f * k, x + sd * 1.0f * k, x + sd * 0.95f * k, x + sd * 0.58f * k}, vy[4] = {y - 0.36f * k, y - 0.2f * k, y + 0.16f * k, y + 0.06f * k};
    sh.poly(vx, vy, 4);
  }
}
// An autumn leaf: three lobes, veins and a stem; it points along `rot`.
static void leaf(uint16_t *buf, const Frame &f, C3 col) {
  Shape lf, vein;
  lf.ell(f.X(0, -0.2f), f.Y(0, -0.2f), 0.42f * f.k, 0.78f * f.k, atan2f(f.s, f.c));
  for (int sd = -1; sd <= 1; sd += 2) {
    float an = atan2f(f.s, f.c) + sd * 0.95f;
    lf.ell(f.X(sd * 0.42f, 0.02f), f.Y(sd * 0.42f, 0.02f), 0.3f * f.k, 0.55f * f.k, an);
  }
  vein.seg(f.X(0, 0.75f), f.Y(0, 0.75f), f.X(0, -0.82f), f.Y(0, -0.82f), 0.06f * f.k);
  vein.seg(f.X(0, 0.2f), f.Y(0, 0.2f), f.X(-0.62f, -0.22f), f.Y(-0.62f, -0.22f), 0.045f * f.k);
  vein.seg(f.X(0, 0.2f), f.Y(0, 0.2f), f.X(0.62f, -0.22f), f.Y(0.62f, -0.22f), 0.045f * f.k);
  fill(buf, lf, {0, 0, 0}, 0.4f, 0, 4, 2, 3);
  fill(buf, lf, mul(col, 0.55f), 1, 1);
  fill(buf, lf, col);
  fill(buf, vein, mul(col, 0.6f));
}
// A painted egg, 1.4 x 2 units, its top along `rot`; a zigzag band and dots.
static void egg(uint16_t *buf, const Frame &f, C3 col, C3 band) {
  Shape e, zz, dots;
  e.ell(f.X(0, 0.05f), f.Y(0, 0.05f), 0.7f * f.k, 0.95f * f.k, atan2f(f.s, f.c));
  for (int i = 0; i < 6; i++) {
    float u0 = -0.6f + i * 0.2f, u1 = u0 + 0.2f, v0 = i % 2 ? 0.1f : -0.1f, v1 = i % 2 ? -0.1f : 0.1f;
    zz.seg(f.X(u0, v0), f.Y(u0, v0), f.X(u1, v1), f.Y(u1, v1), 0.07f * f.k);
  }
  for (int i = -1; i <= 1; i++) dots.disc(f.X(i * 0.32f, -0.5f + (i ? 0.08f : 0)), f.Y(i * 0.32f, -0.5f + (i ? 0.08f : 0)), 0.08f * f.k);
  for (int i = -1; i <= 1; i += 2) dots.disc(f.X(i * 0.24f, 0.55f), f.Y(i * 0.24f, 0.55f), 0.08f * f.k);
  fill(buf, e, {60, 40, 80}, 0.3f, 0, 4, 2, 3);
  fill(buf, e, col);
  fill(buf, zz, band);
  fill(buf, dots, band);
  Shape shine;
  shine.ell(f.X(-0.3f, -0.35f), f.Y(-0.3f, -0.35f), 0.12f * f.k, 0.22f * f.k, atan2f(f.s, f.c) + 0.4f);
  fill(buf, shine, {255, 255, 255}, 0.45f);
}
// A daisy: white petals round a yellow middle.
static void daisy(uint16_t *buf, float x, float y, float k) {
  Shape p, m;
  for (int i = 0; i < 8; i++) { float an = i * PI2 / 8; p.ell(x + sinf(an) * 0.55f * k, y - cosf(an) * 0.55f * k, 0.2f * k, 0.42f * k, -an); }
  m.disc(x, y, 0.3f * k);
  fill(buf, p, {150, 120, 170}, 0.35f, 1, 2);
  fill(buf, p, {255, 255, 255});
  fill(buf, m, {250, 206, 70});
}
// Clear of the windows (countdown at the top, temperatures at the sides, moon below) and the 12 3 6 9.
static bool clearSpot(float x, float y, float pad) {
  float dx = x - CX, dy = y - CY;
  if (fabsf(dx) < 96 + pad && fabsf(dy + 94) < 26 + pad) return false;
  if (fabsf(fabsf(dx) - 96) < 42 + pad && fabsf(dy) < 46 + pad) return false;
  if (dx * dx + (dy - 108) * (dy - 108) < (32 + pad) * (32 + pad)) return false;
  for (int q = 0; q < 4; q++) {
    float a = q * PI2 / 4, qx = sinf(a) * 172, qy = -cosf(a) * 172;
    if ((dx - qx) * (dx - qx) + (dy - qy) * (dy - qy) < (26 + pad) * (26 + pad)) return false;
  }
  float r = sqrtf(dx * dx + dy * dy);
  return r > 40 + pad && r < 214 - pad && (r < 180 - pad || r > 204 + pad);   // not on the hour marks
}
// Spots spread round the dial, the same every time.
static int scatter(float *xs, float *ys, int want, float pad, unsigned seed) {
  int n = 0;
  for (int tries = 0; tries < 600 && n < want; tries++) {
    seed = seed * 1103515245u + 12345u;
    float a = (seed >> 8 & 0xFFFF) / 65536.0f * PI2;
    seed = seed * 1103515245u + 12345u;
    float r = 44 + (seed >> 8 & 0xFFFF) / 65536.0f * 168;
    float x = px(a, r), y = py(a, r);
    if (!clearSpot(x, y, pad)) continue;
    bool near = false;
    for (int i = 0; i < n && !near; i++) near = (xs[i] - x) * (xs[i] - x) + (ys[i] - y) * (ys[i] - y) < (pad * 2 + 20) * (pad * 2 + 20);
    if (near) continue;
    xs[n] = x; ys[n] = y; n++;
  }
  return n;
}

// The holiday faces: something at each hour that isn't 12 3 6 9, minute dots, the 12 3 6 9, and a few extras.
static void quarterNumerals(uint16_t *buf, C3 col, C3 edge, float edgeGrow) {
  Shapes nums;
  const char *q[4] = {"12", "3", "6", "9"};
  for (int i = 0; i < 4; i++) {
    float a = i * PI2 / 4;
    text(add(nums), q[i], px(a, 172), py(a, 172), 40, 5.5f, 2.4f);
  }
  fillAll(buf, nums, {0, 0, 0}, 0.45f, 1.5f, 3);
  if (edgeGrow > 0) fillAll(buf, nums, edge, 1, edgeGrow);
  fillAll(buf, nums, col);
}
static void minuteDots(uint16_t *buf, C3 col, float r = 211, float size = 1.3f) {
  Shapes ticks;
  for (int m = 0; m < 60; m++)
    if (m % 5) add(ticks).disc(px(m * PI2 / 60, r), py(m * PI2 / 60, r), size);
  fillAll(buf, ticks, col);
}
static void holidayMarkers(uint16_t *buf, int face) {
  const float H12 = PI2 / 12;
  float xs[24], ys[24];
  switch (face) {
    case F_NEWYEAR: {
      static const C3 cf[4] = {{240, 206, 120}, {210, 214, 230}, {240, 120, 170}, {120, 170, 250}};
      int n = scatter(xs, ys, 22, 3, 2027);
      for (int i = 0; i < n; i++) {                        // confetti
        Shape q;
        Frame f(xs[i], ys[i], i * 1.3f, 1);
        float x[4] = {f.X(-4, -1.6f), f.X(4, -1.6f), f.X(4, 1.6f), f.X(-4, 1.6f)}, y[4] = {f.Y(-4, -1.6f), f.Y(4, -1.6f), f.Y(4, 1.6f), f.Y(-4, 1.6f)};
        q.poly(x, y, 4);
        fill(buf, q, cf[i % 4], 0.8f);
      }
      static const C3 bc[4] = {{250, 210, 110}, {236, 110, 160}, {130, 180, 255}, {220, 224, 236}};
      for (int i = 0; i < 4; i++) {
        float a = (i * 3 + 1.5f) * H12;
        burst(buf, px(a, 150), py(a, 150), 22, 14, bc[i], i * 0.3f);
      }
      minuteDots(buf, mul(GOLD, 0.9f));
      for (int h = 0; h < 12; h++) {
        if (h % 3 == 0) continue;
        Shape s;
        star(s, Frame(px(h * H12, 194), py(h * H12, 194), 0, 10));
        fill(buf, s, {0, 0, 0}, 0.4f, 0, 4, 2, 3);
        fill(buf, s, {90, 66, 20}, 1, 1);
        fill(buf, s, {246, 214, 120});
      }
      quarterNumerals(buf, {244, 212, 124}, {}, 0);
      break;
    }
    case F_VALENTINE: {
      int n = scatter(xs, ys, 10, 12, 214);
      for (int i = 0; i < n; i++) {                        // faint big hearts
        Shape s;
        heart(s, Frame(xs[i], ys[i], (i % 3 - 1) * 0.35f, 10 + i % 3 * 3));
        fill(buf, s, {255, 190, 210}, 0.16f);
      }
      minuteDots(buf, {255, 200, 214});
      static const C3 hc[3] = {{255, 160, 190}, {252, 240, 242}, {255, 110, 150}};
      for (int h = 0; h < 12; h++) {
        if (h % 3 == 0) continue;
        Shape s;
        heart(s, Frame(px(h * H12, 193), py(h * H12, 193), 0, 11));
        fill(buf, s, {40, 0, 10}, 0.45f, 0, 4, 2, 3);
        fill(buf, s, {110, 10, 36}, 1, 1);
        fill(buf, s, hc[h % 3]);
      }
      quarterNumerals(buf, {255, 236, 240}, {}, 0);
      break;
    }
    case F_PATRICK: {
      int n = scatter(xs, ys, 8, 14, 317);
      for (int i = 0; i < n; i++) {                        // faint big shamrocks
        Shape l, st;
        shamrock(l, st, Frame(xs[i], ys[i], (i % 3 - 1) * 0.4f, 16 + i % 3 * 4));
        fill(buf, l, {150, 230, 140}, 0.13f);
        fill(buf, st, {150, 230, 140}, 0.13f);
      }
      n = scatter(xs, ys, 7, 5, 99);
      for (int i = 0; i < n; i++) {                        // gold coins
        Shape c, in;
        c.disc(xs[i], ys[i], 6);
        in.disc(xs[i], ys[i], 3.8f);
        fill(buf, c, {0, 0, 0}, 0.4f, 0, 3, 1, 2);
        fill(buf, c, {250, 214, 100});
        fill(buf, in, {200, 150, 50}, 1, 0.6f);
        fill(buf, in, {250, 214, 100}, 1, -0.6f);
      }
      minuteDots(buf, {226, 240, 210});
      for (int h = 0; h < 12; h++) {
        if (h % 3 == 0) continue;
        Shape l, st;
        shamrock(l, st, Frame(px(h * H12, 192), py(h * H12, 192), 0, 13));
        fill(buf, l, {0, 0, 0}, 0.45f, 0, 4, 2, 3);
        fill(buf, l, {10, 60, 20}, 1, 1);
        fill(buf, st, {60, 150, 60});
        fill(buf, l, {96, 206, 96});
      }
      quarterNumerals(buf, {244, 214, 120}, {}, 0);
      break;
    }
    case F_EASTER: {
      int n = scatter(xs, ys, 6, 12, 412);
      for (int i = 0; i < n; i++) daisy(buf, xs[i], ys[i], 11);
      minuteDots(buf, {150, 110, 180});
      static const C3 ec[4] = {{246, 150, 186}, {250, 216, 100}, {130, 214, 170}, {130, 180, 240}};
      static const C3 eb[4] = {{255, 255, 255}, {200, 110, 170}, {255, 255, 255}, {250, 230, 140}};
      for (int h = 0; h < 12; h++) {
        if (h % 3 == 0) continue;
        egg(buf, Frame(px(h * H12, 192), py(h * H12, 192), h * H12, 16), ec[h % 4], eb[h % 4]);
      }
      quarterNumerals(buf, {96, 56, 120}, {}, 0);
      break;
    }
    case F_JULY4: {
      static const C3 bc[4] = {{236, 50, 60}, {240, 240, 248}, {110, 160, 255}, {236, 50, 60}};
      for (int i = 0; i < 4; i++) {
        float a = (i * 3 + 1.5f) * H12;
        burst(buf, px(a, 148), py(a, 148), 26, 16, bc[i], i * 0.4f);
      }
      int n = scatter(xs, ys, 12, 4, 1776);
      for (int i = 0; i < n; i++) {                        // small sparkles
        Shape s;
        star(s, Frame(xs[i], ys[i], i * 0.5f, 3.5f));
        fill(buf, s, {255, 255, 255}, 0.6f);
      }
      minuteDots(buf, {236, 236, 244});
      for (int h = 0; h < 12; h++) {
        if (h % 3 == 0) continue;
        Shape s;
        star(s, Frame(px(h * H12, 194), py(h * H12, 194), h * H12, 11));
        fill(buf, s, {0, 0, 0}, 0.45f, 0, 4, 2, 3);
        fill(buf, s, {248, 248, 250});
      }
      quarterNumerals(buf, {226, 40, 52}, {248, 248, 250}, 2.2f);
      break;
    }
    case F_HALLOWEEN: {
      int n = scatter(xs, ys, 5, 18, 1031);
      for (int i = 0; i < n; i++) {                        // bats against the glow
        Shape b;
        bat(b, xs[i], ys[i], 14 + i % 3 * 4);
        fill(buf, b, {14, 4, 16});
      }
      minuteDots(buf, {30, 8, 18}, 211, 1.5f);
      for (int h = 0; h < 12; h++) {
        if (h % 3 == 0) continue;
        Shape b;
        bat(b, px(h * H12, 192), py(h * H12, 192), 13);
        fill(buf, b, {14, 4, 16});
      }
      for (int q = 0; q < 4; q++) pumpkin(buf, px(q * PI2 / 4, 176), py(q * PI2 / 4, 176), 19);
      break;
    }
    case F_THANKS: {
      static const C3 lc[4] = {{200, 50, 30}, {236, 130, 30}, {232, 182, 56}, {170, 80, 30}};
      int n = scatter(xs, ys, 9, 12, 1122);
      for (int i = 0; i < n; i++) {                        // falling leaves, faint
        Shape lf;
        lf.ell(xs[i], ys[i], 6, 11, i * 0.9f);
        fill(buf, lf, lc[i % 4], 0.22f);
      }
      minuteDots(buf, {246, 226, 190});
      for (int h = 0; h < 12; h++) {
        if (h % 3 == 0) continue;
        leaf(buf, Frame(px(h * H12, 192), py(h * H12, 192), h * H12, 17), lc[h % 4]);
      }
      quarterNumerals(buf, {250, 232, 196}, {}, 0);
      break;
    }
  }
}

// ---------- the Advent calendar ----------
// 24 doors in a fixed shuffled order. In December, doors up to today are open (today's glows); after the 24th
// they're all open; the rest of the year they're all shut. The banner at the top: the time (drawn by drawHands)
// and the countdown.
static const int ADVENT_ORDER[24] = {13, 5, 21, 9, 1, 17, 22, 10, 3, 15, 24, 7, 6, 19, 12, 23, 2, 16, 18, 4, 14, 8, 20, 11};
// What's behind each door (1..24): all different; the star on Christmas Eve, the tree the day before.
static const int DOOR_ICON[25] = {0, 5, 9, 4, 14, 2, 11, 16, 3, 13, 17, 6, 20, 8, 22, 10, 19, 12, 18, 21, 23, 7, 15, 1, 0};
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
    case 6: {  // bell
      Shape body, clap, bow;
      body.ell(x, y - 2, 11, 13);
      float bx[4] = {x - 15, x + 15, x + 9, x - 9}, by[4] = {y + 12, y + 12, y - 2, y - 2};
      body.poly(bx, by, 4);
      clap.disc(x, y + 15, 4);
      bow.ell(x - 6, y - 17, 6, 3.5f, -0.4f); bow.ell(x + 6, y - 17, 6, 3.5f, 0.4f); bow.disc(x, y - 16, 3);
      fill(buf, clap, {150, 110, 40});
      fill(buf, body, {120, 80, 20}, 1, 1);
      fill(buf, body, {244, 200, 80});
      fill(buf, bow, {214, 34, 44});
      break;
    }
    case 7: {  // candle
      a.box(x, y + 8, 7, 14, 2);
      b.ell(x, y - 13, 4.5f, 8);
      c.ell(x, y - 11, 2.2f, 4);
      Shape wick, holly;
      wick.seg(x, y - 6, x, y - 3, 1);
      holly.ell(x - 9, y + 21, 7, 3.5f, 0.3f); holly.ell(x + 9, y + 21, 7, 3.5f, -0.3f);
      fill(buf, b, {255, 200, 60}, 0.5f, 3, 6);
      fill(buf, a, {214, 34, 44});
      fill(buf, wick, {40, 30, 20});
      fill(buf, b, {255, 190, 60});
      fill(buf, c, {255, 250, 210});
      fill(buf, holly, {40, 150, 70});
      Shape berry; berry.disc(x, y + 20, 3); fill(buf, berry, {230, 40, 40});
      break;
    }
    case 8: {  // stocking
      a.box(x - 2, y - 2, 8, 14, 2);
      a.ell(x + 4, y + 13, 12, 8);
      b.box(x - 2, y - 16, 11, 5, 2.5f);
      fill(buf, a, {214, 34, 44});
      fill(buf, b, {250, 250, 250});
      Shape stripe; stripe.box(x - 2, y + 2, 8, 2, 0);
      fill(buf, stripe, {40, 150, 70});
      break;
    }
    case 9: {  // snowman
      a.disc(x, y + 10, 12);
      a.disc(x, y - 8, 8.5f);
      b.box(x, y - 19, 6, 5, 1); b.box(x, y - 14.5f, 9, 1.5f, 0.5f);
      float nx[3] = {x, x + 10, x}, ny[3] = {y - 9.5f, y - 7.5f, y - 5.5f};
      c.poly(nx, ny, 3);
      Shape dots; dots.disc(x - 3, y - 10, 1.4f); dots.disc(x + 3, y - 10, 1.4f);
      dots.disc(x, y + 5, 1.6f); dots.disc(x, y + 11, 1.6f);
      fill(buf, a, {250, 252, 255});
      fill(buf, b, {30, 30, 36});
      fill(buf, c, {250, 140, 30});
      fill(buf, dots, {30, 30, 36});
      break;
    }
    case 10: {  // gingerbread man
      a.disc(x, y - 13, 7.5f);
      a.ell(x, y + 3, 8, 10);
      a.seg(x - 6, y - 2, x - 15, y + 4, 3.6f); a.seg(x + 6, y - 2, x + 15, y + 4, 3.6f);
      a.seg(x - 4, y + 10, x - 9, y + 21, 4); a.seg(x + 4, y + 10, x + 9, y + 21, 4);
      Shape ice; ice.disc(x - 2.8f, y - 14, 1.3f); ice.disc(x + 2.8f, y - 14, 1.3f);
      ice.disc(x, y, 1.6f); ice.disc(x, y + 6, 1.6f);
      ice.seg(x - 3, y - 9.5f, x + 3, y - 9.5f, 0.9f);
      fill(buf, a, {176, 100, 40});
      fill(buf, ice, {255, 255, 255});
      break;
    }
    case 11: {  // holly
      a.ell(x - 9, y - 2, 13, 6, 0.5f);
      a.ell(x + 9, y - 2, 13, 6, -0.5f);
      a.ell(x, y + 10, 6, 12);
      b.disc(x - 4, y - 6, 4); b.disc(x + 4, y - 6, 4); b.disc(x, y - 12, 4);
      c.disc(x - 5, y - 8, 1.2f); c.disc(x + 3, y - 8, 1.2f); c.disc(x - 1, y - 14, 1.2f);
      fill(buf, a, {20, 90, 40}, 1, 1);
      fill(buf, a, {40, 150, 70});
      fill(buf, b, {214, 34, 44});
      fill(buf, c, {255, 255, 255}, 0.7f);
      break;
    }
    case 12: {  // wreath
      for (int i = 0; i < 12; i++) { float an = i * PI2 / 12; a.disc(x + 13 * sinf(an), y + 2 - 13 * cosf(an), 6.5f); }
      for (int i = 0; i < 5; i++) { float an = i * PI2 / 5 + 0.4f; c.disc(x + 13 * sinf(an), y + 2 - 13 * cosf(an), 2.2f); }
      b.ell(x - 6, y + 16, 6, 3.5f, -0.4f); b.ell(x + 6, y + 16, 6, 3.5f, 0.4f); b.disc(x, y + 16, 3);
      fill(buf, a, {24, 110, 50});
      fill(buf, c, {230, 40, 40});
      fill(buf, b, {214, 34, 44});
      break;
    }
    case 13: {  // mitten
      a.ell(x + 2, y - 2, 11, 14);
      a.ell(x - 10, y + 2, 4.5f, 8, -0.5f);
      b.box(x + 1, y + 15, 11, 5, 2);
      fill(buf, a, {60, 110, 210});
      fill(buf, b, {250, 250, 250});
      Shape flake;
      for (int k = 0; k < 3; k++) { float an = k * PI2 / 6; flake.seg(x + 2 - sinf(an) * 5, y - 3 - cosf(an) * 5, x + 2 + sinf(an) * 5, y - 3 + cosf(an) * 5, 0.9f); }
      fill(buf, flake, {255, 255, 255});
      break;
    }
    case 14: {  // hot cocoa
      a.box(x - 3, y + 6, 12, 13, 3);
      b.disc(x + 11, y + 6, 7);
      Shape hole; hole.disc(x + 11, y + 6, 3.6f);
      c.box(x - 3, y - 6, 10, 2, 1);
      Shape steam;
      for (int k = -1; k <= 1; k++) { steam.seg(x - 3 + k * 5, y - 11, x - 1 + k * 5, y - 16, 1.1f); steam.seg(x - 1 + k * 5, y - 16, x - 3 + k * 5, y - 21, 1.1f); }
      Shape mallow; mallow.box(x - 6, y - 7, 3, 2.5f, 1); mallow.box(x + 1, y - 7.5f, 3, 2.5f, 1);
      fill(buf, b, {214, 34, 44});
      fill(buf, hole, {48, 22, 14});
      fill(buf, a, {214, 34, 44});
      fill(buf, c, {110, 60, 30});
      fill(buf, mallow, {255, 255, 255});
      fill(buf, steam, {255, 255, 255}, 0.6f);
      break;
    }
    case 15: {  // Santa hat
      float hx[3] = {x - 14, x + 13, x + 10}, hy[3] = {y + 10, y + 10, y - 17};
      a.poly(hx, hy, 3);
      a.seg(x + 10, y - 17, x + 16, y - 6, 3);
      b.box(x, y + 13, 17, 5, 4);
      c.disc(x + 16, y - 5, 5);
      fill(buf, a, {214, 34, 44});
      fill(buf, b, {250, 250, 250});
      fill(buf, c, {250, 250, 250});
      break;
    }
    case 16: {  // reindeer
      Shape ant;
      for (int sd = -1; sd <= 1; sd += 2) {
        ant.seg(x + sd * 5, y - 10, x + sd * 12, y - 22, 1.8f);
        ant.seg(x + sd * 9, y - 16, x + sd * 16, y - 16, 1.6f);
        ant.seg(x + sd * 11, y - 20, x + sd * 7, y - 24, 1.5f);
        a.ell(x + sd * 11, y - 6, 5, 2.6f, sd * 0.5f);
      }
      a.ell(x, y + 2, 10, 13);
      b.disc(x, y + 13, 4.5f);
      c.disc(x - 4, y - 2, 1.6f); c.disc(x + 4, y - 2, 1.6f);
      fill(buf, ant, {170, 120, 60});
      fill(buf, a, {140, 84, 40});
      fill(buf, c, {20, 14, 10});
      fill(buf, b, {230, 30, 30}, 0.5f, 2, 4);
      fill(buf, b, {240, 40, 40});
      break;
    }
    case 17: {  // gingerbread house
      a.box(x, y + 9, 14, 11, 1);
      float rx[3] = {x - 19, x + 19, x}, ry[3] = {y, y, y - 18};
      b.poly(rx, ry, 3);
      c.box(x, y + 14, 4, 6, 2);
      Shape ice, drops;
      ice.seg(x - 19, y, x, y - 18, 1.6f); ice.seg(x, y - 18, x + 19, y, 1.6f);
      drops.disc(x - 8, y + 4, 2); drops.disc(x + 8, y + 4, 2); drops.disc(x, y - 7, 2.4f);
      fill(buf, a, {176, 100, 40});
      fill(buf, b, {150, 80, 30});
      fill(buf, ice, {255, 255, 255});
      fill(buf, c, {100, 50, 20});
      Shape win; win.box(x - 8, y + 6, 3, 3, 0.5f); win.box(x + 8, y + 6, 3, 3, 0.5f);
      fill(buf, win, {255, 220, 100});
      fill(buf, drops, {214, 34, 44});
      break;
    }
    case 18: {  // sugar cookie with sprinkles
      a.disc(x, y, 16);
      b.disc(x, y, 12.5f);
      static const C3 sp[4] = {{214, 34, 44}, {40, 150, 70}, {250, 210, 90}, {80, 140, 230}};
      for (int i = 0; i < 10; i++) {
        float an = i * 2.4f, r = 3 + (i * 37 % 9);
        Shape s1; float px1 = x + r * sinf(an), py1 = y - r * cosf(an);
        s1.seg(px1 - 1.8f * cosf(an * 1.7f), py1 - 1.8f * sinf(an * 1.7f), px1 + 1.8f * cosf(an * 1.7f), py1 + 1.8f * sinf(an * 1.7f), 0.9f);
        if (i == 0) { fill(buf, a, {210, 160, 90}); fill(buf, b, {250, 236, 240}); }
        fill(buf, s1, sp[i % 4]);
      }
      break;
    }
    case 19: {  // drum
      a.box(x, y + 5, 15, 11, 2);
      b.ell(x, y - 6, 15, 4.5f);
      Shape zz;
      for (int i = 0; i < 5; i++) zz.seg(x - 15 + i * 6, y + (i % 2 ? 13 : -3), x - 9 + i * 6, y + (i % 2 ? -3 : 13), 1);
      c.seg(x - 16, y - 20, x - 4, y - 8, 1.4f); c.seg(x + 16, y - 20, x + 4, y - 8, 1.4f);
      c.disc(x - 16, y - 20, 2.6f); c.disc(x + 16, y - 20, 2.6f);
      fill(buf, a, {214, 34, 44});
      fill(buf, zz, {250, 210, 90});
      fill(buf, b, {236, 230, 214});
      fill(buf, c, {200, 160, 100});
      break;
    }
    case 20: {  // teddy bear
      a.disc(x - 10, y - 11, 5); a.disc(x + 10, y - 11, 5);
      a.disc(x, y - 2, 13);
      b.ell(x, y + 4, 6.5f, 5);
      c.disc(x - 4.5f, y - 5, 1.6f); c.disc(x + 4.5f, y - 5, 1.6f); c.disc(x, y + 2, 2);
      Shape inner; inner.disc(x - 10, y - 11, 2.4f); inner.disc(x + 10, y - 11, 2.4f);
      Shape bow; bow.ell(x - 5, y + 14, 5, 3, -0.4f); bow.ell(x + 5, y + 14, 5, 3, 0.4f); bow.disc(x, y + 14, 2.2f);
      fill(buf, a, {160, 104, 56});
      fill(buf, inner, {220, 170, 120});
      fill(buf, b, {226, 186, 140});
      fill(buf, c, {30, 20, 14});
      fill(buf, bow, {214, 34, 44});
      break;
    }
    case 21: {  // penguin
      a.ell(x, y + 2, 12, 17);
      a.disc(x, y - 12, 8.5f);
      b.ell(x, y + 5, 7.5f, 12);
      b.ell(x, y - 11, 5.5f, 5);
      float kx[3] = {x - 3, x + 3, x}, ky[3] = {y - 9, y - 9, y - 5};
      c.poly(kx, ky, 3);
      c.ell(x - 5, y + 19, 4.5f, 2); c.ell(x + 5, y + 19, 4.5f, 2);
      Shape eyes; eyes.disc(x - 2.5f, y - 13, 1.2f); eyes.disc(x + 2.5f, y - 13, 1.2f);
      Shape scarf; scarf.box(x, y - 3.5f, 9, 2.2f, 1); scarf.box(x + 5, y + 1, 2, 5, 1);
      fill(buf, a, {150, 150, 170}, 1, 1.2f);
      fill(buf, a, {26, 26, 34});
      fill(buf, b, {250, 250, 250});
      fill(buf, eyes, {20, 20, 24});
      fill(buf, c, {250, 150, 30});
      fill(buf, scarf, {214, 34, 44});
      break;
    }
    case 22: {  // crescent moon and stars
      a.disc(x - 2, y, 15);
      b.disc(x + 5, y - 5, 13);
      fill(buf, a, {250, 230, 140});
      fill(buf, b, {48, 22, 14});
      for (int i = 0; i < 3; i++) {
        static const float st[3][3] = {{11, -12, 4}, {15, 6, 3}, {5, 15, 2.6f}};
        Shape s1; star(s1, Frame(x + st[i][0], y + st[i][1], 0, st[i][2]));
        fill(buf, s1, {255, 240, 170});
      }
      break;
    }
    case 23: {  // wrapped sweet
      a.ell(x, y, 11, 8);
      float lx[3] = {x - 9, x - 21, x - 21}, ly[3] = {y, y - 9, y + 9};
      float rx[3] = {x + 9, x + 21, x + 21}, ry[3] = {y, y - 9, y + 9};
      b.poly(lx, ly, 3); b.poly(rx, ry, 3);
      Shape st; st.seg(x - 5, y - 7, x - 1, y + 7, 1.6f); st.seg(x + 2, y - 7, x + 6, y + 7, 1.6f);
      fill(buf, b, {120, 60, 200});
      fill(buf, a, {170, 100, 240});
      fill(buf, st, {255, 255, 255}, 0.85f);
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
  int toXmas = daysToHoliday(d, H_XMAS);
  if (toXmas >= 0) {
    xmasLines(toXmas, l1, l2, sizeof l1);
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
      adventIcon(buf, DOOR_ICON[n], cx + 3, cy - 2);
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
    case F_NEWYEAR: case F_VALENTINE: case F_PATRICK: case F_EASTER: case F_JULY4: case F_HALLOWEEN: case F_THANKS:
      holidayMarkers(buf, face);
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

// ---------- digital faces: Digital (seven segments), Nixie, Flip ----------
// Big HH:MM across the middle, the date above, the weather and the moon below. The still parts (panel, unlit
// segments, tubes, cards) are the base; the lit digits are drawn every tick by drawHands.
struct Digi { C3 on, glow, ghost, text, dim; };
// The lit colour of each digital face (settable); everything else on the face is worked out from it, as
// k * colour + a fixed part, so the settings page can tint a preview without asking the screen.
static const uint32_t DIGIT_DEFAULT[3] = {0x00F2C8, 0xFF963C, 0xEEEEE8};   // Digital (stereo green-blue), Nixie, Flip
static uint32_t digitRGB[3] = {0x00F2C8, 0xFF963C, 0xEEEEE8};
static float digitGlow[3] = {1, 1, 1}, digitUnlit[3] = {1, 1, 1};   // 1 = as designed
static C3 digitC3(int f) {
  uint32_t v = digitRGB[f - F_LED];
  return {(float)((v >> 16) & 255), (float)((v >> 8) & 255), (float)(v & 255)};
}
static Digi digi(int f) {
  C3 on = digitC3(f);
  switch (f) {
    case F_NIXIE: return {on, on, {64, 42, 32}, mix(on, {255, 255, 255}, 0.15f), mul(on, 0.66f)};   // ghost: tube wire
    case F_FLIP: return {on, {0, 0, 0}, {0, 0, 0}, mul(on, 0.87f), mul(on, 0.55f)};
    default: return {on, on, mul(on, 0.17f * digitUnlit[f - F_LED]), mul(on, 0.91f), mul(on, 0.55f)};  // Digital
  }
}
static bool isDigital(int f) { return f >= F_LED && f <= F_FLIP; }
bool digitHasGlow(int f) { return isDigital(f) && f != F_FLIP; }
bool digitHasUnlit(int f) { return f == F_LED; }
uint32_t digitColor(int f) { return isDigital(f) ? digitRGB[f - F_LED] : 0; }
uint32_t defaultDigitColor(int f) { return isDigital(f) ? DIGIT_DEFAULT[f - F_LED] : 0; }
void setDigitColor(int f, uint32_t rgb) { if (isDigital(f)) digitRGB[f - F_LED] = rgb & 0xFFFFFF; }
void setDigitLevels(int f, int glowPct, int unlitPct) {
  if (!isDigital(f)) return;
  digitGlow[f - F_LED] = (glowPct < 0 ? 0 : glowPct > 200 ? 200 : glowPct) / 100.0f;
  digitUnlit[f - F_LED] = (unlitPct < 0 ? 0 : unlitPct > 300 ? 300 : unlitPct) / 100.0f;
}
int digitGlowPct(int f) { return isDigital(f) ? (int)lroundf(digitGlow[f - F_LED] * 100) : 0; }
int digitUnlitPct(int f) { return isDigital(f) ? (int)lroundf(digitUnlit[f - F_LED] * 100) : 0; }
static float glowOf(int f) { return isDigital(f) ? digitGlow[f - F_LED] : 1; }

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

static float glowNow = 1;                                  // the glow strength of the face being drawn
static void glowFill(uint16_t *buf, const Shapes &v, const Digi &c, float soft) {
  if (glowNow > 0) fillAll(buf, v, c.glow, fminf(1, 0.55f * glowNow), 3, soft);
  fillAll(buf, v, c.on);
}

static void digitalBase(uint16_t *buf, int f) {
  const Digi &c = digi(f);
  if (f == F_LED) {
    Shape panel;
    panel.box(CX, 214, 228, 136, 18);
    fill(buf, panel, {0, 0, 0}, 0.5f, 2, 8, 0, 4);
    fill(buf, panel, mul(c.on, 0.08f));
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
  glowNow = glowOf(f);
  int h12 = (h24 + 11) % 12 + 1;
  if (f == F_LED) {
    Shapes on;
    seg7(&on, nullptr, h12 >= 10 ? 1 : -1, DX[0], DY, DW, DH, 17);
    seg7(&on, nullptr, h12 % 10, DX[1], DY, DW, DH, 17);
    seg7(&on, nullptr, m / 10, DX[2], DY, DW, DH, 17);
    seg7(&on, nullptr, m % 10, DX[3], DY, DW, DH, 17);
    glowFill(buf, on, c, 12);
  } else if (f == F_NIXIE) {
    char d[4] = {(char)('0' + h12 / 10), (char)('0' + h12 % 10), (char)('0' + m / 10), (char)('0' + m % 10)};
    for (int i = 0; i < 4; i++) {
      if (i == 0 && h12 < 10) continue;
      char s1[2] = {d[i], 0};
      Shapes g;
      text(g, s1, DX[i] + DW / 2, DY + DH / 2, 150, 7);
      if (glowNow > 0) fillAll(buf, g, c.glow, fminf(1, 0.6f * glowNow), 3, 12);
      fillAll(buf, g, c.on);
      fillAll(buf, g, mix(c.on, {255, 255, 255}, 0.65f), 1, -2.2f);   // the hot core
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
  glowNow = glowOf(f);
  bool colon = !showSeconds || sec % 2 == 0;               // the colon blinks with the seconds
  if (f == F_LED) {
    Shapes on;
    if (colon) colonDots(on, CX, DY, DH, 8);
    if (showSeconds) {
      seg7(&on, nullptr, sec / 10, SX[0], SY, SW, SH, 7);
      seg7(&on, nullptr, sec % 10, SX[1], SY, SW, SH, 7);
    }
    text(on, h24 < 12 ? "AM" : "PM", 70, h24 < 12 ? 300 : 330, 16, 2.8f, 1.8f);
    glowFill(buf, on, c, 12);
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
  glowNow = glowOf(f);
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
    text(info, line, d.moon ? 216 : CX, 428, 20, 3, 2.0f);
  }
  (void)info2;
  if (f == F_FLIP) {
    fillAll(buf, top, c.text);
    fillAll(buf, info, c.text);
  } else {
    glowFill(buf, top, {c.text, c.glow, c.ghost, c.text, c.dim}, 8);
    glowFill(buf, info, {c.text, c.glow, c.ghost, c.text, c.dim}, 8);
  }
  if (d.moon) {
    float mx = d.pres[0] ? 416 : CX, my = d.pres[0] ? 428 : 430;
    float litPart = 0.5f - 0.5f * cosf(d.moonPhase * PI2);   // 0 new .. 1 full
    Shape halo;
    halo.disc(mx, my, 16);
    fill(buf, halo, THEMES[f].moonLit, 0.12f + 0.3f * litPart, 2, 11);   // a soft glow, brighter when fuller
    moonDisc(buf, THEMES[f], d.moonPhase, mx, my, 16);
  }
}

void baseKey(const Dial &d, char *out, int n) {
  int f = d.face < 0 || d.face >= FACE_COUNT ? 0 : d.face;
  if (f == F_ADVENT) snprintf(out, n, "%d|%d/%d|%d", f, d.month, d.mday, daysToHoliday(d, H_XMAS));   // Advent: changes daily
  else if (isDigital(f))                                                                       // its colours
    snprintf(out, n, "%d|%06lx|%d|%d", f, (unsigned long)digitColor(f), digitGlowPct(f), digitUnlitPct(f));
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
  int hol = holidayOf(f), days = hol ? daysToHoliday(d, hol) : -1;
  if (days >= 0) countdownWindow(buf, t, hol, days);                // in place of the pressure
  else if (d.pres[0]) pressureWindow(buf, t, d);
  if (d.moon) moonWindow(buf, t, d.moonPhase);
}

// ---------- hands ----------
static void handShapes(int face, int which, float a, Shape &out) {   // which: 0 hour, 1 minute, 2 second
  HandBuilder h(a);
  switch (face) {
    case 0:  // Gatsby (and New Year): lances with a diamond
    case F_NEWYEAR:
      if (which == 0) { h.poly({-24, -6, 84, -4.5f, 84, 4.5f, -24, 6}); h.poly({78, 0, 106, -13, 138, 0, 106, 13}); h.disc(-24, 7); }
      else if (which == 1) { h.poly({-30, -5, 168, -3, 168, 3, -30, 5}); h.poly({160, 0, 180, -7.5f, 208, 0, 180, 7.5f}); h.disc(-30, 6); }
      else { h.line(-44, 202, 1.3f); h.disc(-32, 6); }
      break;
    case 1:  // Skyline: stepped spires
      if (which == 0) { h.rect(-18, 62, 16); h.rect(60, 94, 11); h.rect(92, 118, 6); h.poly({116, -3, 136, 0, 116, 3}); }
      else if (which == 1) { h.rect(-24, 102, 12); h.rect(100, 152, 8); h.rect(150, 184, 4); h.poly({182, -2, 206, 0, 182, 2}); }
      else { h.line(-40, 206, 1.2f); h.disc(-30, 5); }
      break;
    case 2:  // Ivory (and Easter): spade hour, lance minute, lollipop seconds
    case F_EASTER:
      if (which == 0) { h.poly({-20, -4.5f, 70, -3, 70, 3, -20, 4.5f}); h.poly({62, 0, 78, -11, 104, -8, 124, 0, 104, 8, 78, 11}); h.disc(-20, 6); }
      else if (which == 1) { h.poly({-26, -4, 172, -2.5f, 172, 2.5f, -26, 4}); h.poly({166, 0, 182, -5.5f, 204, 0, 182, 5.5f}); h.disc(-26, 5); }
      else { h.line(-40, 196, 1.2f); h.disc(150, 4.5f); h.disc(-30, 6); }
      break;
    case F_GRANDFATHER:  // Grandfather (and Thanksgiving): spade hour, a slim minute with a bead, fine seconds
    case F_THANKS:
      if (which == 0) {
        h.rect(-22, 64, 6); h.disc(36, 7); h.disc(-22, 7);
        h.poly({58, 0, 76, -14, 98, 0, 76, 14}); h.disc(72, 8); h.poly({94, -4, 120, 0, 94, 4});
      } else if (which == 1) { h.rect(-28, 170, 4); h.disc(118, 5.5f); h.disc(-28, 6); h.poly({160, 0, 174, -6, 204, 0, 174, 6}); }
      else { h.line(-40, 190, 1.0f); h.disc(-30, 5); }
      break;
    case F_VALENTINE:  // Valentine's: slim, with a heart at the tip
      if (which == 0) { h.rect(-18, 92, 6); h.disc(-18, 6); heart(h.sh, Frame(h.X(106, 0), h.Y(106, 0), a, 15)); }
      else if (which == 1) { h.rect(-24, 168, 4.5f); h.disc(-24, 5); heart(h.sh, Frame(h.X(180, 0), h.Y(180, 0), a, 11)); }
      else { h.line(-38, 200, 1.2f); h.disc(-28, 5); }
      break;
    case F_HALLOWEEN:  // Halloween: bony, with a pointed tip
      if (which == 0) { h.poly({-20, -7, 96, -5, 96, 5, -20, 7}); h.poly({90, -11, 132, 0, 90, 11}); h.disc(-20, 7); }
      else if (which == 1) { h.poly({-26, -5, 168, -3.5f, 168, 3.5f, -26, 5}); h.poly({162, -8, 204, 0, 162, 8}); h.disc(-26, 6); }
      else { h.line(-38, 200, 1.3f); h.disc(-28, 5); }
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

bool isFlip(int f) { return f == F_FLIP; }
void flipCard(int k, int &x0, int &y0, int &x1, int &y1) { x0 = k ? 246 : 28; x1 = k ? 452 : 234; y0 = 92; y1 = 308; }
uint32_t chimeColor(int f) {
  if (f < 0 || f >= FACE_COUNT) f = 0;
  if (isDigital(f)) return digitColor(f);
  C3 c = THEMES[f].ink;
  return ((uint32_t)c.r << 16) | ((uint32_t)c.g << 8) | (uint32_t)c.b;
}

void init() { glyph('0'); }
bool faceHasSeconds(int f) { return f != F_ADVENT; }

const char *faceName(int f) {
  static const char *n[FACE_COUNT] = {"Gatsby", "Skyline", "Ivory", "Midnight", "Grandfather",
                                      "New Year", "Valentine's", "St. Patrick's", "Easter", "July 4th", "Halloween",
                                      "Thanksgiving", "Christmas", "Advent", "Digital", "Nixie", "Flip"};
  return f >= 0 && f < FACE_COUNT ? n[f] : n[0];
}
const char *faceGroup(int f) { return f < F_NEWYEAR ? "Classic" : f < F_LED ? "Holidays" : "Digital"; }

}  // namespace clk
