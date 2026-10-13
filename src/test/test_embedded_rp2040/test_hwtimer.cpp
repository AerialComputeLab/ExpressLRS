// On-device tests for the RP2040 hwTimer (Step C bench test).
// Run with: pio test -e Pico_W_900_RX_via_UART
// The radio is not used. Tick/tock also toggle TEST_PIN_TICK/TEST_PIN_TOCK for a logic analyzer.
#include <Arduino.h>
#include <initializer_list>
#include <stdarg.h>
#include <unity.h>

#if defined(PLATFORM_RP2040)
#include <EEPROM.h>
#include <hardware/structs/timer.h>
#include "hwTimer.h"

#define TEST_PIN_TICK 14
#define TEST_PIN_TOCK 15

#define INTERVAL_US 5000
#define HALF_US (INTERVAL_US / 2)
// Individual edges are quantised to 1us and see a little IRQ latency
#define EDGE_TOLERANCE_US 3

typedef struct {
    uint32_t us;
    bool tick;
} event_t;

static constexpr uint32_t MAX_EVENTS = 1200;
static event_t events[MAX_EVENTS];
static volatile uint32_t eventCount;

static void __not_in_flash_func(record)(bool tick)
{
    const uint32_t now = timer_hw->timerawl;
    gpio_xor_mask(1u << (tick ? TEST_PIN_TICK : TEST_PIN_TOCK));
    if (eventCount < MAX_EVENTS)
    {
        events[eventCount].us = now;
        events[eventCount].tick = tick;
        eventCount = eventCount + 1;
    }
}

static void __not_in_flash_func(onTick)() { record(true); }
static void __not_in_flash_func(onTock)() { record(false); }

static void message(const char *fmt, ...)
{
    char buf[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    TEST_MESSAGE(buf);
}

// Run the timer until `count` events are recorded (or a timeout), then stop it
static void runEvents(uint32_t count)
{
    eventCount = 0;
    hwTimer::resume();
    const uint32_t timeoutMs = millis() + (count * HALF_US) / 1000 + 500;
    while (eventCount < count && (int32_t)(millis() - timeoutMs) < 0)
    {
        delay(1);
    }
    hwTimer::stop();
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(count, eventCount, "timed out waiting for timer events");
}

static int32_t halfInterval(uint32_t i)
{
    return (int32_t)(events[i + 1].us - events[i].us);
}

void setUp()
{
    hwTimer::stop();
    hwTimer::resetFreqOffset();
    hwTimer::phaseShift(0);
    hwTimer::updateInterval(INTERVAL_US);
}

void tearDown()
{
    hwTimer::stop();
}

void test_first_event_is_tock_and_immediate()
{
    const uint32_t start = timer_hw->timerawl;
    runEvents(3);
    TEST_ASSERT_FALSE_MESSAGE(events[0].tick, "first event must be tock");
    const int32_t delayUs = (int32_t)(events[0].us - start);
    message("resume() to first tock: %d us", delayUs);
    TEST_ASSERT_LESS_THAN_INT32(20, delayUs);
}

void test_events_alternate_at_half_interval()
{
    const uint32_t n = 401;
    runEvents(n);
    int32_t minHalf = INT32_MAX, maxHalf = INT32_MIN;
    for (uint32_t i = 0; i < n; i++)
    {
        TEST_ASSERT_EQUAL_MESSAGE(i % 2 == 1, events[i].tick, "tick/tock must alternate starting with tock");
        if (i + 1 < n)
        {
            const int32_t half = halfInterval(i);
            minHalf = min(minHalf, half);
            maxHalf = max(maxHalf, half);
        }
    }
    const uint32_t total = events[n - 1].us - events[0].us;
    message("half interval min %d max %d us, 200 periods took %u us (expect %u)", minHalf, maxHalf, total, 200 * INTERVAL_US);
    TEST_ASSERT_INT32_WITHIN(EDGE_TOLERANCE_US, HALF_US, minHalf);
    TEST_ASSERT_INT32_WITHIN(EDGE_TOLERANCE_US, HALF_US, maxHalf);
    TEST_ASSERT_UINT32_WITHIN(EDGE_TOLERANCE_US, 200 * INTERVAL_US, total);
}

// FreqOffset is in 0.2us ticks and applies to every half interval
static void checkFreqOffset(int32_t offsetTicks)
{
    for (int32_t i = 0; i < abs(offsetTicks); i++)
    {
        if (offsetTicks > 0)
            hwTimer::incFreqOffset();
        else
            hwTimer::decFreqOffset();
    }
    TEST_ASSERT_EQUAL_INT32(offsetTicks, hwTimer::getFreqOffset());

    const uint32_t halves = 1000;
    runEvents(halves + 1);
    const uint32_t total = events[halves].us - events[0].us;
    // halves * (2500us + offset * 0.2us)
    const int32_t expected = halves * HALF_US + (int32_t)halves * offsetTicks / 5;
    message("FreqOffset %d ticks: %u half intervals took %u us (expect %d)", offsetTicks, halves, total, expected);
    TEST_ASSERT_INT32_WITHIN(EDGE_TOLERANCE_US, expected, (int32_t)total);
}

void test_freq_offset_plus_one_tick_is_fractional() { checkFreqOffset(1); }
void test_freq_offset_plus_ten_ticks() { checkFreqOffset(10); }
void test_freq_offset_minus_seven_ticks() { checkFreqOffset(-7); }

// phaseShift() delays (or advances) the tick that follows the next tock, once
static void checkPhaseShift(int32_t shiftUs, int32_t expectedUs)
{
    eventCount = 0;
    hwTimer::resume();
    while (eventCount < 10)
    {
        delay(1);
    }
    hwTimer::phaseShift(shiftUs);
    while (eventCount < 30)
    {
        delay(1);
    }
    hwTimer::stop();

    uint32_t shifted = 0;
    for (uint32_t i = 0; i + 1 < eventCount; i++)
    {
        const int32_t half = halfInterval(i);
        if (abs(half - HALF_US) > EDGE_TOLERANCE_US)
        {
            ++shifted;
            message("phaseShift(%d): interval after %s = %d us (expect %d)", shiftUs, events[i].tick ? "tick" : "tock", half, HALF_US + expectedUs);
            TEST_ASSERT_FALSE_MESSAGE(events[i].tick, "phase shift must apply after a tock");
            TEST_ASSERT_INT32_WITHIN(EDGE_TOLERANCE_US, HALF_US + expectedUs, half);
        }
    }
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(1, shifted, "expected exactly one shifted interval");
}

void test_phase_shift_positive() { checkPhaseShift(600, 600); }
void test_phase_shift_negative() { checkPhaseShift(-600, -600); }
// Clamped to a quarter of the full interval
void test_phase_shift_clamped() { checkPhaseShift(2000, INTERVAL_US / 4); }

void test_stop_prevents_further_events()
{
    runEvents(5);
    const uint32_t countAtStop = eventCount;
    delay(20);
    TEST_ASSERT_EQUAL_UINT32(countAtStop, eventCount);
}

// Deviation of every edge from the ideal grid, while the main loop keeps USB serial busy
void test_latency_with_usb_logging()
{
    const uint32_t n = 1000;
    eventCount = 0;
    hwTimer::resume();
    uint32_t lines = 0;
    uint32_t lastLineUs = 0;
    while (eventCount < n)
    {
        if (micros() - lastLineUs >= 2000)
        {
            lastLineUs = micros();
            Serial.printf("usb load line %u ................................................\r\n", lines++);
        }
    }
    hwTimer::stop();

    int32_t worst = 0;
    for (uint32_t i = 0; i < n; i++)
    {
        const int32_t err = (int32_t)(events[i].us - (events[0].us + i * HALF_US));
        if (abs(err) > abs(worst))
            worst = err;
    }
    message("worst edge deviation from grid over %u edges with USB logging: %d us (%u lines printed)", n, worst, lines);
    TEST_ASSERT_LESS_THAN_INT32(10, abs(worst));
}

// A flash write disables interrupts. Measure the stall and check the timer recovers
// without a burst of back-to-back events. This overwrites the EEPROM emulation sector,
// so ELRS settings on this board reset to defaults.
void test_flash_write_stall_recovery()
{
    EEPROM.begin(1024);
    eventCount = 0;
    hwTimer::resume();
    while (eventCount < 20)
    {
        delay(1);
    }
    EEPROM.write(0, EEPROM.read(0) ^ 0xff);
    const uint32_t before = timer_hw->timerawl;
    EEPROM.commit();
    const uint32_t commitUs = timer_hw->timerawl - before;
    while (eventCount < 60)
    {
        delay(1);
    }
    hwTimer::stop();
    EEPROM.end();

    int32_t maxGap = 0, minGap = INT32_MAX;
    uint32_t afterStall = 0;
    for (uint32_t i = 0; i + 1 < eventCount; i++)
    {
        const int32_t half = halfInterval(i);
        maxGap = max(maxGap, half);
        minGap = min(minGap, half);
        if (half > HALF_US + EDGE_TOLERANCE_US)
            afterStall = i + 1;
    }
    message("EEPROM.commit() took %u us; longest gap %d us, shortest gap %d us", commitUs, maxGap, minGap);
    // No burst: no interval shorter than a normal half interval
    TEST_ASSERT_INT32_WITHIN(EDGE_TOLERANCE_US, HALF_US, minGap);
    // Regular again after the stall
    for (uint32_t i = afterStall; i + 1 < eventCount; i++)
    {
        TEST_ASSERT_INT32_WITHIN(EDGE_TOLERANCE_US, HALF_US, halfInterval(i));
    }
}

void setup()
{
    Serial.begin(115200);
    // Wait for the host to open the USB serial port, so no results are lost
    // (this also allows flashing by UF2 drag-and-drop and starting `pio test` afterwards)
    while (!Serial)
    {
        delay(10);
    }
    delay(1000);

    for (auto pin : {TEST_PIN_TICK, TEST_PIN_TOCK})
    {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
    }
    hwTimer::init(onTick, onTock);

    UNITY_BEGIN();
    RUN_TEST(test_first_event_is_tock_and_immediate);
    RUN_TEST(test_events_alternate_at_half_interval);
    RUN_TEST(test_freq_offset_plus_one_tick_is_fractional);
    RUN_TEST(test_freq_offset_plus_ten_ticks);
    RUN_TEST(test_freq_offset_minus_seven_ticks);
    RUN_TEST(test_phase_shift_positive);
    RUN_TEST(test_phase_shift_negative);
    RUN_TEST(test_phase_shift_clamped);
    RUN_TEST(test_stop_prevents_further_events);
    RUN_TEST(test_latency_with_usb_logging);
    RUN_TEST(test_flash_write_stall_recovery);
    UNITY_END();
}

void loop()
{
}

#else
// Only meaningful on RP2040
void setup()
{
    UNITY_BEGIN();
    UNITY_END();
}

void loop()
{
}
#endif
