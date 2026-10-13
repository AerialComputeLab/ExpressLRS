#if defined(PLATFORM_RP2040)
#include "hwTimer.h"
#include "logging.h"

#include <hardware/irq.h>
#include <hardware/structs/timer.h>
#include <hardware/sync.h>
#include <hardware/timer.h>

void (*hwTimer::callbackTick)() = nullptr;
void (*hwTimer::callbackTock)() = nullptr;

volatile bool hwTimer::running = false;
volatile bool hwTimer::isTick = false;

volatile uint32_t hwTimer::HWtimerInterval = TimerIntervalUSDefault;
volatile int32_t hwTimer::PhaseShift = 0;
volatile int32_t hwTimer::FreqOffset = 0;

// Same resolution as the ESP timers, so FreqOffset steps are 0.2us
#define HWTIMER_TICKS_PER_US 5

// Shared by the timer IRQ and the GPIO IRQ, above PICO_DEFAULT_IRQ_PRIORITY (0x80)
#define HWTIMER_IRQ_PRIORITY 0x40

// The RP2040 system timer counts whole microseconds, and an alarm fires when the low 32 bits
// of the counter equal the alarm value. The next event is scheduled on an absolute time kept
// as whole microseconds plus a remainder in ticks (0-4), so the average rate is exact at 0.2us
// and nothing drifts from re-arming relative to "now".
static int alarmNum = -1;
static uint32_t alarmMask;
static uint32_t nextFireUs;
static uint32_t nextFireFrac;

// Exact n / 5 for any 32 bit n, using only shifts, adds and a 32 bit multiply.
// The SDK divider routines run from flash, so they can't be used from the ISR.
static __force_inline uint32_t divu5(uint32_t n)
{
    uint32_t q = (n >> 3) + (n >> 4);
    q = q + (q >> 4);
    q = q + (q >> 8);
    q = q + (q >> 16);
    uint32_t r = n - q * 5;
    return q + ((13 * r) >> 6);
}

static __force_inline uint32_t timeUs()
{
    return timer_hw->timerawl;
}

static __force_inline void fireNow()
{
    timer_hw->armed = alarmMask; // disarm
    hw_set_bits(&timer_hw->intf, alarmMask);
}

static __force_inline void armAt(uint32_t targetUs)
{
    timer_hw->alarm[alarmNum] = targetUs;
    // The alarm compares for equality, so a target that has already passed would not fire
    // until the counter wraps. If it's still armed and the target has passed, force it.
    if ((int32_t)(targetUs - timeUs()) <= 0 && (timer_hw->armed & alarmMask))
    {
        fireNow();
    }
}

// Advance the absolute schedule by `ticks` and arm the alarm for it.
// Called from three places, so the compiler won't inline it: keep it in RAM explicitly.
static void ICACHE_RAM_ATTR scheduleNext(uint32_t ticks)
{
    const uint32_t total = nextFireFrac + ticks;
    const uint32_t us = divu5(total);
    nextFireFrac = total - us * HWTIMER_TICKS_PER_US;
    nextFireUs += us;

    const uint32_t now = timeUs();
    if ((int32_t)(nextFireUs - now) <= 0)
    {
        // A whole half period was missed (e.g. a flash write stalled the CPU). Don't burst
        // through the missed events: rebase so the next one is a full step from now, and let
        // the PFD pull the phase back in.
        nextFireUs = now + us;
    }
    armAt(nextFireUs);
}

void hwTimer::init(void (*callbackTick)(), void (*callbackTock)())
{
    if (alarmNum < 0)
    {
        hwTimer::callbackTick = callbackTick;
        hwTimer::callbackTock = callbackTock;

        alarmNum = hardware_alarm_claim_unused(true);
        alarmMask = 1u << alarmNum;
        const uint irqNum = TIMER_IRQ_0 + alarmNum;
        irq_set_exclusive_handler(irqNum, hwTimer::callback);
        // The timer and the GPIO IRQ (radio DIO) must have the SAME priority: resume() is called
        // from inside the radio ISR and the forced tock must wait until that ISR has returned
        // (as on ESP), not preempt it. Both are raised above the default priority used by
        // arduino-pico's USB IRQs (USBCTRL and the tud_task() IRQ), which otherwise delay
        // timer edges by 10-20us while USB serial is busy.
        irq_set_priority(irqNum, HWTIMER_IRQ_PRIORITY);
        irq_set_priority(IO_IRQ_BANK0, HWTIMER_IRQ_PRIORITY);
        hw_set_bits(&timer_hw->inte, alarmMask);
        irq_set_enabled(irqNum, true);
        DBGLN("hwTimer Init, alarm %d", alarmNum);
    }
}

void ICACHE_RAM_ATTR hwTimer::stop()
{
    if (alarmNum >= 0 && running)
    {
        running = false;
        timer_hw->armed = alarmMask;
        hw_clear_bits(&timer_hw->intf, alarmMask);
        timer_hw->intr = alarmMask;
        DBGLN("hwTimer stop");
    }
}

void ICACHE_RAM_ATTR hwTimer::resume()
{
    if (alarmNum >= 0 && !running)
    {
        nextFireUs = timeUs();
        nextFireFrac = 0;
        running = true;
#if defined(TARGET_TX)
        scheduleNext(HWtimerInterval);
#else
        // tock() should always be the first event to maintain consistency, so fire it now.
        // When called from the radio ISR it runs as soon as that ISR returns.
        isTick = false;
        fireNow();
#endif
        DBGLN("hwTimer resume");
    }
}

void ICACHE_RAM_ATTR hwTimer::updateInterval(uint32_t time)
{
    // timer should not be running when updateInterval() is called
    HWtimerInterval = time * HWTIMER_TICKS_PER_US;
    DBGLN("hwTimer interval: %d", time);
}

void ICACHE_RAM_ATTR hwTimer::phaseShift(int32_t newPhaseShift)
{
    int32_t minTicks = -(HWtimerInterval >> 2);
    int32_t maxTicks = (HWtimerInterval >> 2);

    // Convert microseconds to ticks first, then clamp to the valid tick range
    int32_t newPhaseShiftTicks = newPhaseShift * HWTIMER_TICKS_PER_US;
    PhaseShift = constrain(newPhaseShiftTicks, minTicks, maxTicks);
}

void ICACHE_RAM_ATTR hwTimer::callback()
{
    // Acknowledge the alarm (and any forced interrupt) first
    hw_clear_bits(&timer_hw->intf, alarmMask);
    timer_hw->intr = alarmMask;

    if (!running)
    {
        return;
    }

#if defined(TARGET_TX)
    scheduleNext(HWtimerInterval);
    callbackTock();
#else
    uint32_t NextInterval = (HWtimerInterval >> 1) + FreqOffset;
    if (hwTimer::isTick)
    {
        scheduleNext(NextInterval);
        hwTimer::callbackTick();
    }
    else
    {
        NextInterval += PhaseShift;
        scheduleNext(NextInterval);
        PhaseShift = 0;
        hwTimer::callbackTock();
    }
    hwTimer::isTick = !hwTimer::isTick;
#endif
}

#endif
