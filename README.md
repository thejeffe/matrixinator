# Matrixinator

Firmware that turns a **Guition ESP32-S3-4848S040** (4" 480×480 touch screen, the board openHASP calls
`esp32-s3-4848s040_16MB`) into a screen that shows an LED matrix.

It looks like WLED to Signinator: `/json/info` reports a 2D matrix (32×32 by default), `/json/state` takes `on`/`bri`,
and pixels arrive as **DDP on UDP 4048**. Each LED is drawn as a round dot; the backlight follows the brightness.

- **Wi-Fi setup**: right after flashing from Lickety Split (Improv over USB), or join the `Matrixinator-XXXX` hotspot and open http://192.168.4.1.
- **Settings page** at `http://<ip>/`: matrix size, unlit-LED dots, backlight range, Wi-Fi, host name.
- **Updates**: `POST /update` (multipart, same as WLED), from the settings page or Lickety Split.
- **Tap the screen**: shows the song Signinator is playing, the IP and the frame rate.
- Extra endpoint: `POST /api/overlay {"title","artist"}` (Signinator sends it).

## Clock

**Screen mode** on the settings page:

- **Auto**: Signinator's pictures; dark when it sends nothing.
- **Auto (with clock)** (default): Signinator's pictures; when it sends nothing, an art deco analog clock during the
  clock's hours (default 5:00 on, 22:30 off). Outside the hours the screen is dark; a tap shows the clock for 30 s.
- **Clock only**: the clock during its hours, pictures ignored.

Faces: **Gatsby** (black and gold sunburst), **Skyline** (steel, a crown of stepped triangles), **Ivory** (cream,
black chapter ring, jade), **Midnight** (navy and silver, quiet at night). Options: second hand, moon phase,
clock brightness, time zone (time comes from NTP).

**Home Assistant weather**: the URL, a long-lived access token (HA: Profile > Security) and up to five sensor
entity ids: inside and outside temperature, inside and outside humidity, air pressure (any can be left empty).
The screen reads `GET /api/states/<entity>` every minute. Temperatures and humidity sit in IN / OUT windows
either side of the centre; pressure (with HA's unit, e.g. 983 HPA or 29.03 INHG) in a window below 12.

`GET /api/clock` gives the clock's status; `POST /api/clock?preview=30` shows it for 30 s; `POST /api/clock`
with `{"face":1}` or `{"mode":1}` changes the face or mode.

`tools/preview` draws every face to image files on a PC (the clock code is plain C++):
`g++ -O2 -Isrc tools/preview/preview.cpp src/clock.cpp -o preview && ./preview /tmp`

Display setup (pins, panel timing, ST7701 init) comes from openHASP's working configuration for this board.

## Build

GitHub Actions builds every push (`.github/workflows/build.yml`). Tag `vX.Y.Z` to make a release with
`matrixinator-4848s040-factory.bin` (USB, offset 0) and `matrixinator-4848s040-ota.bin` (Wi-Fi update).

Locally: `pio run` (PlatformIO, espressif32 6.5.0 / Arduino core 2.0.x, Arduino_GFX 1.4.7).
