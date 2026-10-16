#include "bench_uartstats.h"
#include "device.h" // also lets the library finder add lib/DEVICE, which common.h needs
#include "common.h"
#include "logging.h"

namespace
{
struct WriteStats
{
    volatile uint32_t calls;
    volatile uint32_t shortWrites; // write() returned fewer bytes than asked for
    volatile uint32_t bytesLost;
    volatile uint32_t maxUs;       // longest single write() call
};

class BenchUartStatsStream : public Stream
{
public:
    explicit BenchUartStatsStream(RP2040Serial &port) : port(port) {}

    size_t write(uint8_t c) override { return write(&c, 1); }

    size_t write(const uint8_t *buffer, size_t size) override
    {
        // Non-zero inside an exception handler, i.e. the tock ISR's RC frames
        WriteStats &s = __get_current_exception() ? isr : loop;
        const uint32_t start = time_us_32();
        const size_t written = port.write(buffer, size);
        const uint32_t took = time_us_32() - start;

        s.calls++;
        if (written < size)
        {
            s.shortWrites++;
            s.bytesLost += size - written;
        }
        if (took > s.maxUs)
        {
            s.maxUs = took;
        }
        return written;
    }

    int available() override { return port.available(); }
    int read() override { return port.read(); }
    int peek() override { return port.peek(); }
    void flush() override { port.flush(); }

    RP2040Serial &port;
    WriteStats isr = {};
    WriteStats loop = {};
};

BenchUartStatsStream *stream;
}

Stream &benchUartStatsStream(RP2040Serial &port)
{
    if (stream == nullptr)
    {
        stream = new BenchUartStatsStream(port);
    }
    return *stream;
}

static void takeStats(WriteStats &from, WriteStats &to)
{
    noInterrupts();
    to = {from.calls, from.shortWrites, from.bytesLost, from.maxUs};
    from = {};
    interrupts();
}

void benchUartStatsLoop(uint32_t nowMs)
{
    static uint32_t lastReportMs;
    static uint32_t rxOverflows;

    if (stream == nullptr)
    {
        return;
    }
    // overflow() clears the flag, so poll it every pass to count each one
    if (stream->port.overflow())
    {
        rxOverflows++;
    }
    if (nowMs - lastReportMs < 5000)
    {
        return;
    }
    lastReportMs = nowMs;

    WriteStats isr, loop;
    takeStats(stream->isr, isr);
    takeStats(stream->loop, loop);
    DBGLN("bench: uart isr %u writes, %u short (%u bytes lost), max %uus | loop %u writes, %u short, max %uus | rx overflows %u | %s",
          isr.calls, isr.shortWrites, isr.bytesLost, isr.maxUs,
          loop.calls, loop.shortWrites, loop.maxUs, rxOverflows,
          connectionState == connected ? "connected" : "not connected");
    rxOverflows = 0;
}
