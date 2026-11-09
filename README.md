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

Display setup (pins, panel timing, ST7701 init) comes from openHASP's working configuration for this board.

## Build

GitHub Actions builds every push (`.github/workflows/build.yml`). Tag `vX.Y.Z` to make a release with
`matrixinator-4848s040-factory.bin` (USB, offset 0) and `matrixinator-4848s040-ota.bin` (Wi-Fi update).

Locally: `pio run` (PlatformIO, espressif32 6.5.0 / Arduino core 2.0.x, Arduino_GFX 1.4.7).
