#include "targets.h"
#include <SPI.h>

#if defined(PLATFORM_RP2040)
// arduino-pico's SPIClass is the abstract HardwareSPI, the concrete class is SPIClassRP2040
#define SPIEX_BASE_CLASS SPIClassRP2040
#else
#define SPIEX_BASE_CLASS SPIClass
#endif

/**
 * @brief An extension to the platform SPI class that provides some performance enhancements.
 *
 * 1. The provided data buffer is expected to be word-aligned and word-padded, this allow the
 *    data to be copied to the SPI fifo in word-sized chunks.
 * 2. The wait for non-busy is removed from the end of the write call so the processor can
 *    continue to do work while the SPI module is pumping the data from the FIFO to the external
 *    device.
 */
class SPIExClass : public SPIEX_BASE_CLASS
{
public:
#if defined(PLATFORM_ESP32)
    explicit SPIExClass(uint8_t spi_bus=HSPI) : SPIClass(spi_bus) {}
#elif defined(PLATFORM_RP2040)
    explicit SPIExClass(spi_inst_t *spi=spi0)
        : SPIClassRP2040(spi, PIN_SPI0_MISO, PIN_SPI0_SS, PIN_SPI0_SCK, PIN_SPI0_MOSI), _spi(spi) {}

    /**
     * @brief Start the SPI bus on the given pins in 8-bit mode 0, MSB first.
     *
     * The RP2040 SPI block can't hold CS low across a multi-byte mode 0 transfer, so the
     * NSS pins are driven in software. nss2 may be UNDEF_PIN.
     */
    void begin(int8_t sck, int8_t miso, int8_t mosi, int8_t nss, int8_t nss2);
    using SPIClassRP2040::begin;
    void setFrequency(uint32_t freq);
#else
    explicit SPIExClass() : SPIClass() {}
#endif

    /**
     * @brief Perform an SPI read operation on the SPI bus.
     *
     * If the SPI bus is busy the processor waits for the current operation to complete before starting
     * this operation.
     * Once the SPI bus is free, the data is copied to the SPI fifo and the processor waits for the operation
     * to complete then copies the incoming data back into the provided data buffer.
     *
     * @param cs_mask mask of CS pins to enable for this operation
     * @param data word-aligned and padded data buffer
     * @param size the number of bytes to be read into in the data buffer
     */
    void inline ICACHE_RAM_ATTR read(uint8_t cs_mask, uint8_t *data, uint32_t size) { _transfer(cs_mask, data, size, true); }

    /**
     * @brief Perform an SPI write operation on the SPI bus.
     *
     * If the SPI bus is busy the processor waits for the current operation to complete before starting
     * this operation.
     * One the SPI bus is not busy, it copies the data to the SPI fifo then returns without waiting for
     * the operation to complete allowing the processor to do other work.
     *
     * @param cs_mask mask of CS pins to enable for this operation
     * @param data word-aligned and padded data buffer
     * @param size the number of bytes to be written to the SPI device
     */
    void inline ICACHE_RAM_ATTR write(uint8_t cs_mask, uint8_t * data, uint32_t size) { _transfer(cs_mask, data, size, false); }

private:
    void _transfer(uint8_t cs_mask, uint8_t *data, uint32_t size, bool reading);
#if defined(PLATFORM_RP2040)
    spi_inst_t *_spi;
    int8_t _nss[2] = {UNDEF_PIN, UNDEF_PIN};
#endif
};

extern SPIExClass SPIEx;
