#pragma once

#include <Arduino.h>

// Bench-only (RP2040_BENCH_CRSF_FRAMES): send CRSF RC channel frames to the flight controller
// while there is no link, so the UART can be checked without a transmitter. Never upstreamed.
void benchCrsfFramesLoop(Stream &port, uint32_t nowMs);
