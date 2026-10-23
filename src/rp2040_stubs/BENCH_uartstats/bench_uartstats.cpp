#include "bench_uartstats.h"
#include "device.h" // also lets the library finder add lib/DEVICE, which common.h needs
#include "common.h"
#include "logging.h"
#include "FHSS.h"
#include "hwTimer.h"

extern uint8_t uplinkLQ;
#if defined(RADIO_SX127X)
#include "SX127xHal.h"
#include "SX127xRegs.h"

extern SX127xHal hal;
#endif
#if defined(RP2040_BENCH_MSP_FRAMES)
#include "CRSFRouter.h"
#include "RXOTAConnector.h"
extern RXOTAConnector otaConnector;
#endif

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

#if defined(RP2040_BENCH_MSP_FRAMES)
// Every 100ms, routes a 64 byte (the CRSF maximum) MSP_REQ frame to the FC as if it had
// arrived over the air, so it takes the real passthrough path: crsfRouter ->
// SerialCRSF::forwardMessage() -> sendQueuedData(), which writes it with interrupts off.
// The request is MSP_API_VERSION (read-only) padded with zeros; ArduPilot ignores MSP.
// Don't use it with a Betaflight/INAV FC: it answers every request, and 10 replies/s swamp
// the telemetry downlink (sensors go stale, the Betaflight Lua script stalls; finding #45).
static uint32_t mspFramesSent;

static void sendBenchMspFrame(uint32_t nowMs)
{
    static uint32_t lastMs;
    if (connectionState != connected || nowMs - lastMs < 100)
    {
        return;
    }
    lastMs = nowMs;

    constexpr uint8_t frameSize = CRSF_FRAME_SIZE_MAX - CRSF_FRAME_NOT_COUNTED_BYTES; // 62
    uint8_t frame[CRSF_FRAME_SIZE_MAX] = {};
    auto *ext = (crsf_ext_header_t *)frame;
    ext->payload[0] = 0x30; // MSP v1, start of packet, sequence 0
    ext->payload[1] = 0;    // MSP payload size
    ext->payload[2] = 1;    // MSP_API_VERSION
    crsfRouter.SetExtendedHeaderAndCrc(ext, CRSF_FRAMETYPE_MSP_REQ, frameSize,
                                       CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_ADDRESS_RADIO_TRANSMITTER);
    crsfRouter.deliverMessage(&otaConnector, (crsf_header_t *)ext);
    mspFramesSent++;
}
#endif

#if defined(RADIO_SX127X)
// While disconnected: what the radio sees, to tell a deaf radio (packet count flat), packets
// that ELRS rejects (count rising, DIO0 low) and a missed DIO0 edge (DIO0 stuck high, RxDone set)
static void reportRadio()
{
    noInterrupts(); // the RX-done ISR also uses SPI
    const uint8_t opMode = hal.readRegister(SX127X_REG_OP_MODE, SX12XX_Radio_1);
    const uint8_t irqFlags = hal.readRegister(SX127X_REG_IRQ_FLAGS, SX12XX_Radio_1);
    uint8_t cnt[2], frf[3];
    hal.readRegister(SX127X_REG_RX_PACKET_CNT_VALUE_MSB, cnt, 2, SX12XX_Radio_1);
    hal.readRegister(SX127X_REG_FRF_MSB, frf, 3, SX12XX_Radio_1);
    const int dio0 = digitalRead(GPIO_PIN_DIO0);
    interrupts();
    // The ELRS logger has no field widths, so values are plain hex
    DBGLN("bench: radio opmode 0x%x irqflags 0x%x dio0 %d rxpackets %u frf 0x%x rate %u",
          (uint32_t)opMode, (uint32_t)irqFlags, (int32_t)dio0, (uint32_t)((cnt[0] << 8) | cnt[1]),
          (uint32_t)((frf[0] << 16) | (frf[1] << 8) | frf[2]),
          (uint32_t)ExpressLRS_currAirRate_Modparams->index);
}
#endif

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
    // Count link drops since boot (connected -> anything else), for the soak test
    static bool wasConnected;
    static uint32_t linkDrops;
    const bool isConnected = connectionState == connected;
    if (wasConnected && !isConnected)
    {
        linkDrops++;
    }
    wasConnected = isConnected;
#if defined(RP2040_BENCH_MSP_FRAMES)
    sendBenchMspFrame(nowMs);
#endif
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
    // Drift and link quality: fc (SX127x frequency correction) and fo (timer frequency offset)
    // should stay bounded over a long run, see the Phase 4 soak test
    DBGLN("bench: link fc %d fo %d lq %u drops %u uptime %u s",
          FreqCorrection, hwTimer::getFreqOffset(), (uint32_t)uplinkLQ, linkDrops, nowMs / 1000);
#if defined(RADIO_SX127X)
    if (connectionState != connected)
    {
        reportRadio();
    }
#endif
#if defined(RP2040_BENCH_MSP_FRAMES)
    DBGLN("bench: %u MSP frames queued to the FC", mspFramesSent);
    mspFramesSent = 0;
#endif
}
