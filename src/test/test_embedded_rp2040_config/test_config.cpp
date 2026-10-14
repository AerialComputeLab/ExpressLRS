// On-device tests for RX config storage on RP2040 (Step E bench test).
// Run with: pio test -e Pico_W_900_RX_via_UART -f test_embedded_rp2040_config
//
// Writes to the EEPROM emulation sector (the ELRS RX config). The original model ID and serial
// protocol are restored at the end. The radio is not used.
#include <Arduino.h>
#include <stdarg.h>
#include <unity.h>

#if defined(PLATFORM_RP2040)
#include <EEPROM.h>
#include <StreamString.h>
#include <hardware/structs/timer.h>
#include "common.h"
#include "config.h"
#include "elrs_eeprom.h"
#include "logging.h"

// Normally defined in src/common.cpp, which test builds don't compile
connectionState_e connectionState = disconnected;

static ELRS_EEPROM eeprom;
// The global that config.h declares (normally defined in rx_main.cpp)
RxConfig config;
static uint8_t originalModelId;
static eSerialProtocol originalProtocol;

static void message(const char *fmt, ...)
{
    char buf[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    TEST_MESSAGE(buf);
}

// Make arduino-pico's EEPROM drop its RAM copy and read the flash sector again
static void reloadEepromFromFlash()
{
    EEPROM.end();
    eeprom.Begin();
}

void setUp()
{
    connectionState = disconnected;
}

void tearDown()
{
    connectionState = disconnected;
}

void test_load_existing_config()
{
    config.SetStorageProvider(&eeprom);
    config.Load();
    originalModelId = config.GetModelId();
    originalProtocol = config.GetSerialProtocol();
    message("loaded config: bound=%d modelId=%u serialProtocol=%u", config.GetIsBound(), originalModelId, originalProtocol);
    // Load() either reads a valid config or writes defaults, so nothing is left pending
    TEST_ASSERT_FALSE(config.IsModified());
}

void test_commit_without_changes_skips_flash()
{
    const uint32_t start = timer_hw->timerawl;
    TEST_ASSERT_EQUAL_UINT32(0, config.Commit());
    const uint32_t elapsed = timer_hw->timerawl - start;
    message("Commit() with no changes took %u us", elapsed);
    TEST_ASSERT_LESS_THAN_UINT32(1000, elapsed);
}

void test_commit_roundtrip_through_flash()
{
    const uint8_t newModelId = (originalModelId + 1) % 64;
    const eSerialProtocol newProtocol = (originalProtocol == PROTOCOL_SBUS) ? PROTOCOL_CRSF : PROTOCOL_SBUS;
    config.SetModelId(newModelId);
    config.SetSerialProtocol(newProtocol);
    TEST_ASSERT_TRUE(config.IsModified());

    const uint32_t start = timer_hw->timerawl;
    TEST_ASSERT_NOT_EQUAL_UINT32(0, config.Commit());
    const uint32_t elapsed = timer_hw->timerawl - start;
    message("Commit() with changes took %u us (flash write, interrupts off)", elapsed);

    reloadEepromFromFlash();
    RxConfig reloaded;
    reloaded.SetStorageProvider(&eeprom);
    reloaded.Load();
    TEST_ASSERT_EQUAL_UINT8(newModelId, reloaded.GetModelId());
    TEST_ASSERT_EQUAL(newProtocol, reloaded.GetSerialProtocol());
}

// Returns the log output produced while committing one change
static String commitAndCaptureLog()
{
    StreamString captured;
    Stream *previous = BackpackOrLogStrm;
    BackpackOrLogStrm = &captured;
    config.SetModelId((config.GetModelId() + 1) % 64);
    config.Commit();
    BackpackOrLogStrm = previous;
    return captured;
}

void test_commit_while_connected_is_reported()
{
    connectionState = connected;
    const String log = commitAndCaptureLog();
    message("log while connected: %s", log.c_str());
    TEST_ASSERT_TRUE_MESSAGE(log.indexOf("while connected") >= 0, "expected a warning for a commit while connected");
}

void test_commit_while_disconnected_is_silent()
{
    connectionState = disconnected;
    const String log = commitAndCaptureLog();
    TEST_ASSERT_TRUE_MESSAGE(log.indexOf("while connected") < 0, "unexpected warning while disconnected");
}

// Expected UIDs from python/build_flags.py: md5('-DMY_BINDING_PHRASE="<phrase>"')[0:6]
static void checkBindPhrase(const char *phrase, const uint8_t (&expected)[UID_LEN])
{
    RxConfig scratch; // never committed
    scratch.SetBindPhrase((uint8_t *)phrase, strlen(phrase));
    const uint8_t *uid = scratch.GetUID();
    message("SetBindPhrase(\"%s\") -> %u,%u,%u,%u,%u,%u", phrase, uid[0], uid[1], uid[2], uid[3], uid[4], uid[5]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, uid, UID_LEN);
}

void test_bind_phrase_uid_matches_python()
{
    static const uint8_t expected[UID_LEN] = {20, 185, 220, 14, 41, 36};
    checkBindPhrase("expresslrs rp2040 test", expected);
}

void test_bind_phrase_single_char()
{
    static const uint8_t expected[UID_LEN] = {247, 201, 32, 98, 8, 24};
    checkBindPhrase("a", expected);
}

void test_empty_bind_phrase_is_unbound()
{
    static const uint8_t expected[UID_LEN] = {0, 0, 0, 0, 0, 0};
    checkBindPhrase("", expected);
}

void test_restore_original_config()
{
    config.SetModelId(originalModelId);
    config.SetSerialProtocol(originalProtocol);
    config.Commit();
    reloadEepromFromFlash();
    RxConfig reloaded;
    reloaded.SetStorageProvider(&eeprom);
    reloaded.Load();
    TEST_ASSERT_EQUAL_UINT8(originalModelId, reloaded.GetModelId());
    TEST_ASSERT_EQUAL(originalProtocol, reloaded.GetSerialProtocol());
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }
    delay(1000);

    // ELRS logging goes nowhere unless a test captures it
    static StreamString discard;
    BackpackOrLogStrm = &discard;
    eeprom.Begin();

    UNITY_BEGIN();
    RUN_TEST(test_load_existing_config);
    RUN_TEST(test_commit_without_changes_skips_flash);
    RUN_TEST(test_commit_roundtrip_through_flash);
    RUN_TEST(test_commit_while_connected_is_reported);
    RUN_TEST(test_commit_while_disconnected_is_silent);
    RUN_TEST(test_bind_phrase_uid_matches_python);
    RUN_TEST(test_bind_phrase_single_char);
    RUN_TEST(test_empty_bind_phrase_is_unbound);
    RUN_TEST(test_restore_original_config);
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
