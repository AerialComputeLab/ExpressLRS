# RP2040 temporary stubs

Libraries in this directory are visible only to the RP2040 envs (`lib_extra_dirs` in
`targets/rp2040-rx.ini`). They stand in for ESP-only libraries until the real port
lands, and are not intended for upstream.

- `WIFI_stub`: no-op `WIFI_device` and `setWifiUpdateMode()` so `rx_main.cpp` builds
  while `lib/WIFI` is in `lib_ignore`. Delete it when WiFi mode is ported (Phase 5).
- `hardware.json`: bench pin map (Pico W + Olimex LoRa915 Rev C). `python/rp2040_config_fallback.py`
  compiles it into the image as a fallback for when LittleFS has no `/hardware.json`. It is kept
  here rather than in `src/data/`, which is PlatformIO's global `data_dir` and would be uploaded
  by `uploadfs` on ESP envs too. Replaced by configurator / UF2 config support (Phase 6).
- `BENCH_crsf`: bench-only CRSF RC channel frames (50 Hz, channel 1 sweeping, throttle and AUX1
  low) sent to the flight controller while there is no link, enabled with
  `-DRP2040_BENCH_CRSF_FRAMES`. It checks the CRSF UART without a transmitter. Remove its hook in
  `rx_main.cpp` loop() before upstreaming.
