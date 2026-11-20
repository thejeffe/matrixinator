#!/usr/bin/env python3
"""Bake the settings-page face previews into the firmware (src/face_previews.h).

The screen used to draw a preview on request, which took seconds and slowed it down. Instead every
face is drawn once here, with sample weather, as a 240x240 JPEG compiled in. Run after changing a face:

    cd tools/preview && g++ -O2 -I../../src preview.cpp ../../src/clock.cpp -o preview
    python3 make_previews.py          # needs Pillow
"""
import io, os, subprocess, sys, tempfile
from PIL import Image

here = os.path.dirname(os.path.abspath(__file__))
src = os.path.join(here, "..", "..", "src")
names = ["Gatsby", "Skyline", "Ivory", "Midnight", "Grandfather", "Christmas", "Advent",
         "Red LED", "VFD", "Nixie", "Flip"]   # clock.cpp's face order

with tempfile.TemporaryDirectory() as d:
    subprocess.run([os.path.join(here, "preview"), d], check=True, stdout=subprocess.DEVNULL)
    out = ["// Made by tools/preview/make_previews.py: each clock face as a 240x240 JPEG for the settings page.",
           "#pragma once", "#include <stdint.h>", "#include <stddef.h>", ""]
    table = []
    for i, n in enumerate(names):
        img = Image.open(os.path.join(d, n + ".ppm")).convert("RGB").resize((240, 240), Image.LANCZOS)
        b = io.BytesIO()
        img.save(b, "JPEG", quality=85, optimize=True)
        data = b.getvalue()
        out.append("static const uint8_t FACE_JPG_%d[] = {" % i)
        for j in range(0, len(data), 20):
            out.append("  " + ",".join(str(x) for x in data[j:j + 20]) + ",")
        out.append("};")
        table.append("  {FACE_JPG_%d, sizeof FACE_JPG_%d}," % (i, i))
        print("%-12s %6d bytes" % (n, len(data)), file=sys.stderr)
    out += ["", "struct FacePreview { const uint8_t *data; size_t len; };",
            "static const FacePreview FACE_PREVIEWS[] = {"] + table + ["};", ""]
    with open(os.path.join(src, "face_previews.h"), "w") as f:
        f.write("\n".join(out))
