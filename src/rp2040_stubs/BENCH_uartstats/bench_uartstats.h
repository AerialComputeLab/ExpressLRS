#pragma once

#include <Arduino.h>
#include "RP2040Serial.h"

// Bench-only (RP2040_BENCH_UART_STATS): wraps the CRSF output port to count writes that
// the UART driver drops (plan finding #43), split by ISR (RC frames from tock)
// and main loop (queued data, written with interrupts off). Never upstreamed.

// The stream to give SerialCRSF as its output port
Stream &benchUartStatsStream(RP2040Serial &port);

// Call from loop(): prints the counters every 5 seconds (needs DEBUG_LOG)
void benchUartStatsLoop(uint32_t nowMs);
