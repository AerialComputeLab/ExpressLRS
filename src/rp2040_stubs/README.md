# RP2040 temporary stubs

Libraries in this directory are visible only to the RP2040 envs (`lib_extra_dirs` in
`targets/rp2040-rx.ini`). They stand in for ESP-only libraries until the real port
lands, and are not intended for upstream.

- `WIFI_stub`: no-op `WIFI_device` and `setWifiUpdateMode()` so `rx_main.cpp` builds
  while `lib/WIFI` is in `lib_ignore`. Delete it when WiFi mode is ported (Phase 5).
