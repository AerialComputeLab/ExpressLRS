#pragma once

#include <Arduino.h>

// Bench-only (RP2040_BENCH_CRASH): records where the firmware died, for crashes that take
// USB down with them. A HardFault, or loop() not running for 1s (a hang, even with
// interrupts disabled: the watchdog alarm is routed to NMI), saves the PC/LR/xPSR to RAM
// that survives a reset, then reboots. The record is logged at boot and every 10s.
// Map the PC to a line with addr2line on firmware.elf. Never upstreamed.

// Call at the end of setup(): logs any saved record and starts the loop watchdog
void benchCrashSetup();

// Call from loop(): pushes the watchdog deadline 1s ahead, and re-logs the record
void benchCrashLoop(uint32_t nowMs);
