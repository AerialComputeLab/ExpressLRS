#if defined(PLATFORM_RP2040)
#include "RP2040Serial.h"
#include "targets.h"

#include <hardware/gpio.h>
#include <hardware/irq.h>

static RP2040Serial *instances[2];

// GPIO n is a UART pin of UART ((n + 4) / 8) % 2: TX when n % 4 == 0, RX when n % 4 == 1
static bool isUartPin(uart_inst_t *uart, int8_t pin, int function)
{
    return pin == UNDEF_PIN ||
        (pin >= 0 && pin < (int)NUM_BANK0_GPIOS && pin % 4 == function && ((pin + 4) / 8) % 2 == (int)uart_get_index(uart));
}

bool RP2040Serial::setPins(int8_t tx, int8_t rx)
{
    if (!isUartPin(_uart, tx, 0) || !isUartPin(_uart, rx, 1))
    {
        return false;
    }
    _tx = tx;
    _rx = rx;
    return true;
}

void RP2040Serial::begin(unsigned long baud, uint16_t config)
{
    end();

    _rxHead = _rxTail = 0;
    _overflow = false;
    _actualBaud = uart_init(_uart, baud);

    const uart_parity_t parity = (config & SERIAL_PARITY_MASK) == SERIAL_PARITY_EVEN ? UART_PARITY_EVEN
                               : (config & SERIAL_PARITY_MASK) == SERIAL_PARITY_ODD  ? UART_PARITY_ODD
                               : UART_PARITY_NONE;
    const uint stopBits = (config & SERIAL_STOP_BIT_MASK) == SERIAL_STOP_BIT_1 ? 1 : 2;
    uint dataBits = 8;
    switch (config & SERIAL_DATA_MASK)
    {
    case SERIAL_DATA_5: dataBits = 5; break;
    case SERIAL_DATA_6: dataBits = 6; break;
    case SERIAL_DATA_7: dataBits = 7; break;
    }
    uart_set_format(_uart, dataBits, stopBits, parity);
    uart_set_hw_flow(_uart, false, false);

    if (_tx != UNDEF_PIN)
    {
        gpio_set_function(_tx, GPIO_FUNC_UART);
        gpio_set_outover(_tx, _invert ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
    }
    if (_rx != UNDEF_PIN)
    {
        gpio_set_function(_rx, GPIO_FUNC_UART);
        gpio_set_inover(_rx, _invert ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
    }

    const uint irq = uart_get_index(_uart) == 0 ? UART0_IRQ : UART1_IRQ;
    instances[uart_get_index(_uart)] = this;
    irq_set_exclusive_handler(irq, uart_get_index(_uart) == 0 ? uart0IRQ : uart1IRQ);
    irq_set_enabled(irq, true);
    uart_set_irqs_enabled(_uart, _rx != UNDEF_PIN, false);

    _running = true;
}

void RP2040Serial::end()
{
    if (!_running)
    {
        return;
    }
    // Writes from the ISR return 0 from here on (end() is only called from loop())
    _running = false;

    const uint irq = uart_get_index(_uart) == 0 ? UART0_IRQ : UART1_IRQ;
    uart_set_irqs_enabled(_uart, false, false);
    irq_set_enabled(irq, false);
    irq_remove_handler(irq, uart_get_index(_uart) == 0 ? uart0IRQ : uart1IRQ);
    instances[uart_get_index(_uart)] = nullptr;
    uart_deinit(_uart);

    if (_tx != UNDEF_PIN)
    {
        gpio_set_outover(_tx, GPIO_OVERRIDE_NORMAL);
        gpio_set_function(_tx, GPIO_FUNC_NULL);
    }
    if (_rx != UNDEF_PIN)
    {
        gpio_set_inover(_rx, GPIO_OVERRIDE_NORMAL);
        gpio_set_function(_rx, GPIO_FUNC_NULL);
    }
}

bool RP2040Serial::overflow()
{
    const bool overflowed = _overflow;
    _overflow = false;
    return overflowed;
}

size_t ICACHE_RAM_ATTR RP2040Serial::write(uint8_t c)
{
    return write(&c, 1);
}

size_t ICACHE_RAM_ATTR RP2040Serial::write(const uint8_t *buffer, size_t size)
{
    if (!_running)
    {
        return 0;
    }
    for (size_t i = 0; i < size; i++)
    {
        uart_putc_raw(_uart, buffer[i]); // waits while the 32 byte TX FIFO is full
    }
    return size;
}

int RP2040Serial::availableForWrite()
{
    return _running && uart_is_writable(_uart) ? 1 : 0;
}

void RP2040Serial::flush()
{
    if (_running)
    {
        uart_tx_wait_blocking(_uart);
    }
}

int RP2040Serial::available()
{
    return (_rxHead - _rxTail) & (RX_BUFFER_SIZE - 1);
}

int RP2040Serial::read()
{
    const uint32_t tail = _rxTail;
    if (tail == _rxHead)
    {
        return -1;
    }
    const uint8_t c = _rxBuffer[tail];
    _rxTail = (tail + 1) & (RX_BUFFER_SIZE - 1);
    return c;
}

int RP2040Serial::peek()
{
    const uint32_t tail = _rxTail;
    return tail == _rxHead ? -1 : _rxBuffer[tail];
}

void ICACHE_RAM_ATTR RP2040Serial::handleIRQ()
{
    while (uart_is_readable(_uart))
    {
        const uint32_t dr = uart_get_hw(_uart)->dr;
        if (dr & UART_UARTDR_OE_BITS)
        {
            // The hardware FIFO overflowed before this byte, which is still valid
            _overflow = true;
        }
        if (dr & (UART_UARTDR_BE_BITS | UART_UARTDR_PE_BITS | UART_UARTDR_FE_BITS))
        {
            continue; // break, parity or framing error: drop the byte, as arduino-pico does
        }
        const uint32_t head = _rxHead;
        const uint32_t next = (head + 1) & (RX_BUFFER_SIZE - 1);
        if (next == _rxTail)
        {
            _overflow = true; // ring buffer full: drop the byte
            continue;
        }
        _rxBuffer[head] = (uint8_t)dr;
        _rxHead = next;
    }
}

void ICACHE_RAM_ATTR RP2040Serial::uart0IRQ()
{
    if (instances[0] != nullptr)
    {
        instances[0]->handleIRQ();
    }
}

void ICACHE_RAM_ATTR RP2040Serial::uart1IRQ()
{
    if (instances[1] != nullptr)
    {
        instances[1]->handleIRQ();
    }
}
#endif
