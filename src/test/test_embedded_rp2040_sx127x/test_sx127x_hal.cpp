// On-device tests for the RP2040 SPI / SX127x HAL (Step F bench test).
// Run with: pio test -e Pico_W_900_RX_via_UART -f test_embedded_rp2040_sx127x
//
// Receive-safe: only reset, register reads and register writes in standby. The radio is
// never put into TX mode. Writes /hardware.json to LittleFS with the bench pin map
// (rp2040_stubs/hardware.json).
#include <Arduino.h>
#include <stdarg.h>
#include <unity.h>

#if defined(PLATFORM_RP2040)
#include <LittleFS.h>
#include <hardware/spi.h>
#include "SX127xHal.h"
#include "SX127xRegs.h"
#include "options.h"

// The bench pin map, generated from rp2040_stubs/hardware.json by python/rp2040_config_fallback.py
#include "rp2040_config_fallback.h"
#if !defined(RP2040_HARDWARE_FALLBACK)
#error "custom_hardware_fallback is not set for this env"
#endif

static SX127xHal hal;
static volatile uint32_t dio0Count;

static void onDio0() { dio0Count = dio0Count + 1; }

static void message(const char *fmt, ...)
{
    char buf[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    TEST_MESSAGE(buf);
}

void setUp() {}
void tearDown() {}

void test_hardware_config_loaded()
{
    TEST_ASSERT_TRUE_MESSAGE(options_init(), "options_init() found no hardware config");
    TEST_ASSERT_EQUAL_INT(16, GPIO_PIN_MISO);
    TEST_ASSERT_EQUAL_INT(17, GPIO_PIN_NSS);
    TEST_ASSERT_EQUAL_INT(18, GPIO_PIN_SCK);
    TEST_ASSERT_EQUAL_INT(19, GPIO_PIN_MOSI);
    TEST_ASSERT_EQUAL_INT(20, GPIO_PIN_DIO0);
    TEST_ASSERT_EQUAL_INT(21, GPIO_PIN_RST);
    TEST_ASSERT_EQUAL_INT(UNDEF_PIN, GPIO_PIN_NSS_2);
}

void test_init_and_version_register()
{
    hal.IsrCallback_1 = onDio0;
    hal.IsrCallback_2 = nullptr;
    hal.init();
    hal.reset();
    // The SX1276 needs ~5ms after reset is released (the firmware's DetectChip() retries instead)
    delay(10);
    const uint8_t version = hal.readRegister(SX127X_REG_VERSION, SX12XX_Radio_1);
    message("RegVersion (0x42) = 0x%02x (expect 0x%02x)", version, SX127X_VERSION);
    TEST_ASSERT_EQUAL_HEX8(SX127X_VERSION, version);
}

void test_spi_clock_at_most_10mhz()
{
    const uint32_t baud = spi_get_baudrate(spi0);
    message("SPI0 clock %u Hz", baud);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(10000000, baud);
    TEST_ASSERT_GREATER_THAN_UINT32(5000000, baud);
}

// After reset the chip is in FSK standby. The frequency registers are a 3 byte burst.
void test_frequency_registers_burst_write_readback()
{
    WORD_ALIGNED_ATTR uint8_t frf[3] = {0xE4, 0xC0, 0x00}; // 915 MHz
    hal.writeRegister(SX127X_REG_FRF_MSB, frf, sizeof(frf), SX12XX_Radio_1);
    WORD_ALIGNED_ATTR uint8_t readBack[3] = {0};
    hal.readRegister(SX127X_REG_FRF_MSB, readBack, sizeof(readBack), SX12XX_Radio_1);
    message("RegFrf = %02x %02x %02x", readBack[0], readBack[1], readBack[2]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(frf, readBack, sizeof(frf));
}

// 8 FSK sync bytes plus the address byte is 9 bytes, more than the 8 entry SPI FIFO
#define REG_FSK_SYNC_VALUE1 0x28
void test_sync_value_9_byte_transfer()
{
    WORD_ALIGNED_ATTR uint8_t sync[8] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};
    hal.writeRegister(REG_FSK_SYNC_VALUE1, sync, sizeof(sync), SX12XX_Radio_1);
    WORD_ALIGNED_ATTR uint8_t readBack[8] = {0};
    hal.readRegister(REG_FSK_SYNC_VALUE1, readBack, sizeof(readBack), SX12XX_Radio_1);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(sync, readBack, sizeof(sync));
}

void test_single_register_write_readback()
{
    hal.writeRegister(REG_FSK_SYNC_VALUE1, 0x5A, SX12XX_Radio_1);
    TEST_ASSERT_EQUAL_HEX8(0x5A, hal.readRegister(REG_FSK_SYNC_VALUE1, SX12XX_Radio_1));
    hal.writeRegisterBits(REG_FSK_SYNC_VALUE1, 0x03, 0x0F, SX12XX_Radio_1);
    TEST_ASSERT_EQUAL_HEX8(0x53, hal.readRegister(REG_FSK_SYNC_VALUE1, SX12XX_Radio_1));
}

void test_reset_restores_defaults()
{
    hal.reset();
    delay(10);
    // RegFrf resets to 434 MHz (0x6C8000) on the SX1276
    WORD_ALIGNED_ATTR uint8_t frf[3] = {0};
    hal.readRegister(SX127X_REG_FRF_MSB, frf, sizeof(frf), SX12XX_Radio_1);
    message("RegFrf after reset = %02x %02x %02x", frf[0], frf[1], frf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x6C, frf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x80, frf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, frf[2]);
    message("DIO0 interrupts seen: %u (none expected)", dio0Count);
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
    RUN_TEST(test_hardware_config_loaded);
    RUN_TEST(test_init_and_version_register);
    RUN_TEST(test_spi_clock_at_most_10mhz);
    RUN_TEST(test_frequency_registers_burst_write_readback);
    RUN_TEST(test_sync_value_9_byte_transfer);
    RUN_TEST(test_single_register_write_readback);
    RUN_TEST(test_reset_restores_defaults);
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
