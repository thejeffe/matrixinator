#!/usr/bin/env python3
"""Bake the settings-page face previews into the firmware (src/face_previews.h).

The screen used to draw a preview on request, which took seconds and slowed it down. Instead every
face is drawn once here, with sample weather, as a 240x240 JPEG compiled in. Run after changing a face:

    cd tools/preview && g++ -O2 -I../../src preview.cpp ../../src/clock.cpp -o preview
    python3 make_previews.py          # needs Pillow

The digital faces also get two layers so the page can show any colour, glow and unlit level without
asking the screen: everything they draw is (fixed part) + (amount) x colour, so
    picture = base + (lit + glow% x glowMask + unlit% x unlitMask) x colour
with base = the face with a black colour, and lit / glow / unlit masks in the red / green / blue channels.
"""
import io, os, subprocess, sys, tempfile
from PIL import Image, ImageChops

here = os.path.dirname(os.path.abspath(__file__))
src = os.path.join(here, "..", "..", "src")
names = ["Gatsby", "Skyline", "Ivory", "Midnight", "Grandfather", "New Year", "Valentine's", "St. Patrick's",
         "Easter", "July 4th", "Halloween", "Thanksgiving", "Christmas", "Advent",
         "Digital", "Nixie", "Flip"]   # clock.cpp's face order
DIGITAL = range(14, 17)
GRAY = 128


def render(d, *args):
    os.makedirs(d, exist_ok=True)
    subprocess.run([os.path.join(here, "preview"), d, *args], check=True, stdout=subprocess.DEVNULL)
    return lambda n: Image.open(os.path.join(d, n + ".ppm")).convert("RGB").resize((240, 240), Image.LANCZOS)


def jpeg(img, q, sub=2):
    b = io.BytesIO()
    img.save(b, "JPEG", quality=q, optimize=True, subsampling=sub)
    return b.getvalue()


def mask(a, b):
    """(a - b) / GRAY averaged over the channels, 0..255 = 0..1."""
    pa, pb = a.load(), b.load()
    m = Image.new("L", a.size)
    pm = m.load()
    for y in range(a.size[1]):
        for x in range(a.size[0]):
            d = sum(pa[x, y]) - sum(pb[x, y])
            pm[x, y] = max(0, min(255, round(d / 3 / GRAY * 255)))
    return m


with tempfile.TemporaryDirectory() as d:
    std = render(d + "/std")
    g = "%02x%02x%02x" % (GRAY, GRAY, GRAY)
    blk = render(d + "/blk", "000000", "0", "0")
    lit = render(d + "/lit", g, "0", "0")
    glo = render(d + "/glo", g, "100", "0")
    unl = render(d + "/unl", g, "0", "100")
    out = ["// Made by tools/preview/make_previews.py: each clock face as a 240x240 JPEG for the settings page,",
           "// and for the digital faces a base and a mask (lit, glow, unlit in R, G, B) to tint it with any colour.",
           "#pragma once", "#include <stdint.h>", "#include <stddef.h>", ""]
    table = []

    def arr(name, data):
        out.append("static const uint8_t %s[] = {" % name)
        for j in range(0, len(data), 24):
            out.append("  " + ",".join(str(x) for x in data[j:j + 24]) + ",")
        out.append("};")

    for i, n in enumerate(names):
        arr("FACE_JPG_%d" % i, jpeg(std(n), 85))
        if i in DIGITAL:
            b, l = blk(n), lit(n)
            m = Image.merge("RGB", (mask(l, b), mask(glo(n), l), mask(unl(n), l)))
            arr("FACE_BASE_%d" % i, jpeg(b, 90))
            arr("FACE_MASK_%d" % i, jpeg(m, 95, sub=0))
            table.append("  {FACE_JPG_%d, sizeof FACE_JPG_%d, FACE_BASE_%d, sizeof FACE_BASE_%d, FACE_MASK_%d, sizeof FACE_MASK_%d},"
                         % ((i,) * 6))
        else:
            table.append("  {FACE_JPG_%d, sizeof FACE_JPG_%d, nullptr, 0, nullptr, 0}," % (i, i))
        print("%-12s done" % n, file=sys.stderr)
    out += ["", "struct FacePreview { const uint8_t *data; size_t len; const uint8_t *base; size_t baseLen;",
            "                      const uint8_t *mask; size_t maskLen; };",
            "static const FacePreview FACE_PREVIEWS[] = {"] + table + ["};", ""]
    with open(os.path.join(src, "face_previews.h"), "w") as f:
        f.write("\n".join(out))
