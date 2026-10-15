// On-device tests for the RP2040 + LoRa915 power settings (Phase 1 prerequisite).
// Run with: pio test -e Pico_W_900_RX_via_UART -f test_embedded_rp2040_power
//
// Receive-safe: the radio is initialised and its PA config register is set through the real
// ELRS power code, then read back. The radio is never put into TX mode (checked after every
// step), and it is put to sleep at the end. Writes /hardware.json to LittleFS with the bench
// pin map (rp2040_stubs/hardware.json), so the firmware uses the same power fields.
#include <Arduino.h>
#include <stdarg.h>
#include <unity.h>

#if defined(PLATFORM_RP2040)
#include <LittleFS.h>
// Test-only access to SX127xDriver::CommitOutputPower(). In the firmware a power change is only
// written to the radio after a real transmission (TXnbISR), and these tests must not transmit.
// Included first so the header guards keep this the only definition seen in this file.
#define private public
#include "SX127xDriver.h"
#undef private
#include "FHSS.h"
#include "POWERMGNT.h"
#include "SX127xHal.h"
#include "SX127xRegs.h"
#include "options.h"

// The bench pin map, generated from rp2040_stubs/hardware.json by python/rp2040_config_fallback.py
#include "rp2040_config_fallback.h"
#if !defined(RP2040_HARDWARE_FALLBACK)
#error "custom_hardware_fallback is not set for this env"
#endif

// Normally defined in src/common.cpp, which test builds don't compile
SX127xDriver Radio;

// RegPaConfig with PaSelect = RFO (bit 7 clear) and MaxPower = 7 (Pmax = 15 dBm):
// Pout = OutputPower dBm at the RFO_HF pin
static constexpr uint8_t PA_CONFIG_10MW = 0x7A; // +10 dBm
static constexpr uint8_t PA_CONFIG_25MW = 0x7E; // +14 dBm

static void message(const char *fmt, ...)
{
    char buf[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    TEST_MESSAGE(buf);
}

static uint8_t readRegister(uint8_t reg)
{
    return SX127xHal::instance->readRegister(reg, SX12XX_Radio_1);
}

static uint8_t readPaConfig()
{
    return readRegister(SX127X_REG_PA_CONFIG);
}

// The radio must never have been switched to TX by these tests
static void assertNotTransmitting()
{
    const uint8_t mode = readRegister(SX127X_REG_OP_MODE) & 0x07;
    TEST_ASSERT_NOT_EQUAL_MESSAGE(SX127x_OPMODE_TX, mode, "radio is in TX mode");
}

static void setPowerAndCheck(PowerLevels_e requested, PowerLevels_e expectedLevel, uint8_t expectedReg, const char *nominal)
{
    POWERMGNT::setPower(requested);
    Radio.CommitOutputPower();
    const uint8_t reg = readPaConfig();
    message("setPower(%u) -> level %u, RegPaConfig 0x%02x (expect 0x%02x, %s)",
            requested, PowerLevelContainer::currPower(), reg, expectedReg, nominal);
    TEST_ASSERT_EQUAL_UINT8(expectedLevel, PowerLevelContainer::currPower());
    TEST_ASSERT_EQUAL_HEX8(expectedReg, reg);
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(0, reg & 0x80, "PaSelect must be RFO (bit 7 clear)");
    assertNotTransmitting();
}

void setUp() {}
void tearDown() {}

void test_power_fields_loaded()
{
    TEST_ASSERT_TRUE_MESSAGE(options_init(), "options_init() found no hardware config");
    TEST_ASSERT_TRUE(OPT_USE_SX1276_RFO_HF);
    TEST_ASSERT_EQUAL_INT(PWR_10mW, MinPower);
    TEST_ASSERT_EQUAL_INT(PWR_25mW, MaxPower);
    TEST_ASSERT_EQUAL_INT(PWR_10mW, DefaultPower);
    TEST_ASSERT_EQUAL_INT(0, hardware_int(HARDWARE_power_control));
    TEST_ASSERT_EQUAL_INT(2, POWER_OUTPUT_VALUES_COUNT);
    TEST_ASSERT_NOT_NULL(POWER_OUTPUT_VALUES);
    TEST_ASSERT_EQUAL_INT16(122, POWER_OUTPUT_VALUES[0]);
    TEST_ASSERT_EQUAL_INT16(126, POWER_OUTPUT_VALUES[1]);
}

void test_radio_begin_in_standby()
{
    // As rx_main's setupRadio(): the frequency must be set before Begin()
    Radio.currFreq = FREQ_HZ_TO_REG_VAL(915000000);
    TEST_ASSERT_TRUE_MESSAGE(Radio.Begin(FREQ_HZ_TO_REG_VAL(903500000), FREQ_HZ_TO_REG_VAL(926900000)), "radio not detected");
    const uint8_t reg = readPaConfig();
    message("after Begin(): RegPaConfig 0x%02x (RFO default)", reg);
    TEST_ASSERT_EQUAL_HEX8(0x00, reg);
    assertNotTransmitting();
}

void test_default_power_is_10mw()
{
    // As rx_main's setupRadio()
    POWERMGNT::init();
    Radio.CommitOutputPower();
    const uint8_t reg = readPaConfig();
    message("after POWERMGNT::init(): level %u, RegPaConfig 0x%02x", PowerLevelContainer::currPower(), reg);
    TEST_ASSERT_EQUAL_UINT8(PWR_10mW, PowerLevelContainer::currPower());
    TEST_ASSERT_EQUAL_HEX8(PA_CONFIG_10MW, reg);
    assertNotTransmitting();
}

void test_25mw() { setPowerAndCheck(PWR_25mW, PWR_25mW, PA_CONFIG_25MW, "+14 dBm"); }
void test_100mw_clamps_to_25mw() { setPowerAndCheck(PWR_100mW, PWR_25mW, PA_CONFIG_25MW, "+14 dBm"); }
void test_back_to_10mw() { setPowerAndCheck(PWR_10mW, PWR_10mW, PA_CONFIG_10MW, "+10 dBm"); }

void test_radio_to_sleep()
{
    // Check sleep before End(): End() also shuts SPI down, so the radio can't be read afterwards
    Radio.SetMode(SX127x_OPMODE_SLEEP, SX12XX_Radio_All);
    const uint8_t mode = readRegister(SX127X_REG_OP_MODE) & 0x07;
    TEST_ASSERT_EQUAL_UINT8(SX127x_OPMODE_SLEEP, mode);
    Radio.End();
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }
    delay(1000);

    LittleFS.begin();
    File f = LittleFS.open("/hardware.json", "w");
    f.print(RP2040_HARDWARE_FALLBACK_JSON);
    f.close();

    UNITY_BEGIN();
    RUN_TEST(test_power_fields_loaded);
    RUN_TEST(test_radio_begin_in_standby);
    RUN_TEST(test_default_power_is_10mw);
    RUN_TEST(test_25mw);
    RUN_TEST(test_100mw_clamps_to_25mw);
    RUN_TEST(test_back_to_10mw);
    RUN_TEST(test_radio_to_sleep);
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
