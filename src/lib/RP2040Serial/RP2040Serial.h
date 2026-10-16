#pragma once

#if defined(PLATFORM_RP2040)
#include <Arduino.h>
#include <hardware/uart.h>

/**
 * A UART Stream that can be written from an ISR.
 *
 * arduino-pico's SerialUART takes a per-port mutex in every call. ELRS writes CRSF RC
 * frames from the tock ISR, so when the ISR preempted loop() while it was reading the
 * same port, the mutex was held by the same core and write() returned 0: about 15% of
 * RC frames were silently dropped at 100Hz. This class has no lock, like the ESP cores.
 *
 * RX is filled from the UART IRQ into a 256 byte ring buffer (the ESP cores' default)
 * and read by loop(). Writes go straight into the 32 byte hardware FIFO and wait for
 * space when it is full. As on ESP, callers must not write from loop() and the ISR at
 * the same time: SerialIO::sendQueuedData() writes with interrupts disabled.
 */
class RP2040Serial : public Stream
{
public:
    explicit RP2040Serial(uart_inst_t *uart) : _uart(uart) {}

    // Pins must be UART function pins of this UART, or UNDEF_PIN. Call before begin().
    bool setPins(int8_t tx, int8_t rx);
    // Invert both TX and RX (e.g. SBUS). Call before begin().
    void setInvert(bool invert) { _invert = invert; }

    // config is an Arduino SERIAL_xxx value, e.g. SERIAL_8N1
    void begin(unsigned long baud, uint16_t config = SERIAL_8N1);
    void end();
    uint32_t getActualBaud() const { return _actualBaud; }
    // True if RX bytes were lost since the last call
    bool overflow();

    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    using Print::write;
    int availableForWrite() override;
    void flush() override;

    int available() override;
    int read() override;
    int peek() override;

private:
    static constexpr uint32_t RX_BUFFER_SIZE = 256; // power of two
    static void uart0IRQ();
    static void uart1IRQ();
    void handleIRQ();

    uart_inst_t * const _uart;
    int8_t _tx = -1;
    int8_t _rx = -1;
    bool _invert = false;
    volatile bool _running = false;
    volatile bool _overflow = false;
    uint32_t _actualBaud = 0;

    // Single producer (UART IRQ writes head) / single consumer (loop() writes tail)
    // volatile so the IRQ's byte store can't be reordered after its head store
    volatile uint8_t _rxBuffer[RX_BUFFER_SIZE];
    volatile uint32_t _rxHead = 0;
    volatile uint32_t _rxTail = 0;
};
#endif
