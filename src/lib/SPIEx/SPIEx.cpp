#include "SPIEx.h"

#if defined(PLATFORM_ESP32)
#include <soc/spi_struct.h>
#endif

#if defined(PLATFORM_RP2040)
#include "logging.h"

void SPIExClass::begin(int8_t sck, int8_t miso, int8_t mosi, int8_t nss, int8_t nss2)
{
    if (!setSCK(sck) || !setRX(miso) || !setTX(mosi))
    {
        ERRLN("SCK %d / MISO %d / MOSI %d are not valid SPI%d pins", sck, miso, mosi, spi_get_index(_spi));
        return;
    }
    SPIClassRP2040::begin(false);
    spi_set_format(_spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    _nss[0] = nss;
    _nss[1] = nss2;
    for (auto pin : _nss)
    {
        if (pin != UNDEF_PIN)
        {
            gpio_init(pin);
            gpio_put(pin, true);
            gpio_set_dir(pin, GPIO_OUT);
        }
    }
}

void SPIExClass::setFrequency(uint32_t freq)
{
    spi_set_baudrate(_spi, freq);
}
#endif

void ICACHE_RAM_ATTR SPIExClass::_transfer(uint8_t cs_mask, uint8_t *data, uint32_t size, bool reading)
{
#if defined(PLATFORM_ESP32)
    spi_dev_t *spi = *(reinterpret_cast<spi_dev_t**>(bus()));
    // wait for SPI to become non-busy
    while(spi->cmd.usr) {}

    // Set the CS pins which we want controlled by the SPI module for this operation
    spiDisableSSPins(bus(), ~cs_mask);
    spiEnableSSPins(bus(), cs_mask);

#if defined(PLATFORM_ESP32_S3) || defined(PLATFORM_ESP32_C3)
    spi->ms_dlen.ms_data_bitlen = (size*8)-1;
#else
    spi->mosi_dlen.usr_mosi_dbitlen = ((size * 8) - 1);
    spi->miso_dlen.usr_miso_dbitlen = ((size * 8) - 1);
#endif

    // write the data to the SPI FIFO
    const uint32_t words = (size + 3) / 4;
    auto * const wordsBuf = reinterpret_cast<uint32_t *>(data);
    for(int i=0; i<words; i++)
    {
        spi->data_buf[i] = wordsBuf[i]; //copy buffer to spi fifo
    }

#if defined(PLATFORM_ESP32_S3) || defined(PLATFORM_ESP32_C3)
    spi->cmd.update = 1;
    while (spi->cmd.update) {}
#endif
    // start the SPI module
    spi->cmd.usr = 1;

    if (reading)
    {
        // wait for SPI write to complete
        while(spi->cmd.usr) {}

        for(int i=0; i<words; i++)
        {
            wordsBuf[i] = spi->data_buf[i]; //copy spi fifo to buffer
        }
    }
#elif defined(PLATFORM_RP2040)
    // Software CS, and a blocking transfer for writes too: CS can only be released once
    // the last byte has been clocked out. At 10MHz a 13 byte packet takes ~11us.
    // Only inline SDK register helpers are used here so this stays in RAM.
    for (int i = 0; i < 2; i++)
    {
        if ((cs_mask & (1 << i)) && _nss[i] != UNDEF_PIN)
            gpio_put(_nss[i], false);
    }

    spi_hw_t * const hw = spi_get_hw(_spi);
    uint32_t txCount = 0;
    uint32_t rxCount = 0;
    // Keep the 8 entry FIFO full, and drain RX as we go so it never overflows
    while (rxCount < size)
    {
        if (txCount < size && spi_is_writable(_spi) && (txCount - rxCount) < 8)
        {
            hw->dr = data[txCount++];
        }
        if (spi_is_readable(_spi))
        {
            const uint8_t in = (uint8_t)hw->dr;
            if (reading)
                data[rxCount] = in;
            ++rxCount;
        }
    }

    for (int i = 0; i < 2; i++)
    {
        if ((cs_mask & (1 << i)) && _nss[i] != UNDEF_PIN)
            gpio_put(_nss[i], true);
    }
#elif defined(PLATFORM_ESP8266)
    // we support only one hardware controlled CS pin, so theres nothing to do to configure it at this point

    // wait for SPI to become non-busy
    while(SPI1CMD & SPIBUSY) {}

    // Set in/out Bits to transfer
    const uint32_t mask = ~((SPIMMOSI << SPILMOSI) | (SPIMMISO << SPILMISO));
    const auto bits = (size * 8) - 1;
    SPI1U1 = ((SPI1U1 & mask) | ((bits << SPILMOSI) | (bits << SPILMISO)));

    // write the data to the SPI FIFO
    volatile uint32_t * const fifoPtr = &SPI1W0;
    const uint8_t outSize = ((size + 3) / 4);
    uint32_t * const dataPtr = (uint32_t*) data;
    for(int i=0; i<outSize; i++)
    {
        fifoPtr[i] = dataPtr[i];
    }

    // start the SPI module
    SPI1CMD |= SPIBUSY;

    if (reading)
    {
        // wait for SPI write to complete
        while(SPI1CMD & SPIBUSY) {}

        // read data from the FIFO back to the application buffer
        for(int i=0; i<outSize; i++)
        {
            dataPtr[i] = fifoPtr[i];
        }
    }
#endif
}

#if defined(PLATFORM_ESP32_S3) || defined(PLATFORM_ESP32_C3)
SPIExClass SPIEx(FSPI);
#elif defined(PLATFORM_ESP32)
SPIExClass SPIEx(VSPI);
#else
SPIExClass SPIEx;
#endif
