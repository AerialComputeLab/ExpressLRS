#include "bench_crash.h"
#include "logging.h"

#include <hardware/irq.h>
#include <hardware/structs/syscfg.h>
#include <hardware/timer.h>
#include <hardware/watchdog.h>

namespace
{
constexpr uint32_t RECORD_MAGIC = 0xC0DEDEAD;
constexpr uint32_t HANG_TIMEOUT_US = 1000000;

enum : uint32_t
{
    KIND_HARDFAULT = 1,
    KIND_HANG = 2,
};

struct CrashRecord
{
    uint32_t magic;
    uint32_t kind;
    uint32_t pc;
    uint32_t lr;
    uint32_t xpsr; // low 6 bits: the exception that was running (0 = thread, 16 + n = IRQ n)
    uint32_t uptimeMs;
    uint32_t count; // crashes since power-on
};

int watchdogAlarm = -1;
}

// Not zeroed at boot, so it survives the watchdog reboot (not a power cycle)
static CrashRecord __uninitialized_ram(crashRecord);

// frame is the exception stack frame: r0, r1, r2, r3, r12, lr, pc, xpsr.
// Runs from RAM and writes fields one by one, in case the fault hit during a flash write.
extern "C" void __not_in_flash_func(benchCrashSave)(const uint32_t *frame, uint32_t kind)
{
    const uint32_t count = crashRecord.magic == RECORD_MAGIC ? crashRecord.count : 0;
    crashRecord.kind = kind;
    crashRecord.pc = frame[6];
    crashRecord.lr = frame[5];
    crashRecord.xpsr = frame[7];
    crashRecord.uptimeMs = timer_hw->timerawl / 1000;
    crashRecord.count = count + 1;
    crashRecord.magic = RECORD_MAGIC;

    // Reboot through the watchdog, which keeps RAM
    watchdog_hw->ctrl = WATCHDOG_CTRL_TRIGGER_BITS;
    while (true)
    {
    }
}

// Pass the stack frame (MSP or PSP, from EXC_RETURN bit 2) and the kind to benchCrashSave()
#define BENCH_CRASH_HANDLER(name, kind)          \
    extern "C" __attribute__((naked)) void name() \
    {                                             \
        asm volatile(                             \
            "movs r0, #4\n"                       \
            "mov r1, lr\n"                        \
            "tst r0, r1\n"                        \
            "beq 1f\n"                            \
            "mrs r0, psp\n"                       \
            "b 2f\n"                              \
            "1: mrs r0, msp\n"                    \
            "2: movs r1, #" #kind "\n"            \
            "ldr r2, =benchCrashSave\n"           \
            "bx r2\n"                             \
            ".ltorg\n");                          \
    }

BENCH_CRASH_HANDLER(isr_hardfault, 1)
BENCH_CRASH_HANDLER(isr_nmi, 2)

static void report()
{
    if (crashRecord.magic != RECORD_MAGIC)
    {
        return;
    }
    DBGLN("bench: crash #%u (%s) pc 0x%x lr 0x%x xpsr 0x%x (exception %u) at %u ms",
          crashRecord.count, crashRecord.kind == KIND_HARDFAULT ? "HardFault" : "hang, loop stopped",
          crashRecord.pc, crashRecord.lr, crashRecord.xpsr, crashRecord.xpsr & 0x3f, crashRecord.uptimeMs);
}

void benchCrashSetup()
{
    if (!watchdog_caused_reboot())
    {
        crashRecord.magic = 0; // power-on: RAM content is random
    }
    report();

#if defined(RP2040_BENCH_HANG)
    // Hang test for the hardware watchdog in rx_main.cpp: leave the recorder disarmed so
    // the watchdog, not the recorder, has to reset the chip
    DBGLN("bench: hang test, loop() stops 30 s after boot; recorder disarmed");
    return;
#endif
    watchdogAlarm = hardware_alarm_claim_unused(true);
    timer_hw->alarm[watchdogAlarm] = timer_hw->timerawl + HANG_TIMEOUT_US;
    hw_set_bits(&timer_hw->inte, 1u << watchdogAlarm);
    // The alarm drives NMI instead of its NVIC IRQ, so it fires even with interrupts disabled
    syscfg_hw->proc0_nmi_mask = 1u << (TIMER_IRQ_0 + watchdogAlarm);
    DBGLN("bench: crash recorder armed (alarm %d, %u ms)", watchdogAlarm, HANG_TIMEOUT_US / 1000);
}

void benchCrashLoop(uint32_t nowMs)
{
    static uint32_t lastReportMs;

#if defined(RP2040_BENCH_HANG)
    if (nowMs >= 30000)
    {
        DBGLN("bench: hanging loop() now (interrupts on)");
        Serial.flush();
        while (true)
        {
        }
    }
#endif
    if (watchdogAlarm < 0)
    {
        return;
    }
    // Writing the alarm re-arms it at the new deadline
    timer_hw->alarm[watchdogAlarm] = timer_hw->timerawl + HANG_TIMEOUT_US;

    if (nowMs - lastReportMs >= 10000)
    {
        lastReportMs = nowMs;
        report();
    }
}
