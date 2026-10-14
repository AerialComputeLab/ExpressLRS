#include "bench_crsf.h"
#include "common.h"
#include "CRSFRouter.h"
#include "crsf_protocol.h"
#include "logging.h"

static constexpr uint32_t FRAME_INTERVAL_MS = 20;  // 50Hz
static constexpr uint32_t SWEEP_PERIOD_MS = 4000;  // channel 1 goes min -> max -> min

void benchCrsfFramesLoop(Stream &port, uint32_t nowMs)
{
    static uint32_t lastFrameMs;
    static uint32_t lastReportMs;
    static uint32_t framesSent;

    // Real channel data flows once linked
    if (connectionState == connected || nowMs - lastFrameMs < FRAME_INTERVAL_MS)
    {
        return;
    }
    lastFrameMs = nowMs;

    // Triangle sweep across the standard range on channel 1 (roll)
    const uint32_t span = CRSF_CHANNEL_VALUE_STD_MAX - CRSF_CHANNEL_VALUE_STD_MIN;
    const uint32_t phase = nowMs % SWEEP_PERIOD_MS;
    const uint32_t half = SWEEP_PERIOD_MS / 2;
    const uint32_t sweep = CRSF_CHANNEL_VALUE_STD_MIN +
        ((phase < half ? phase : SWEEP_PERIOD_MS - phase) * span) / half;

    CRSF_MK_FRAME_T(crsf_channels_t) frame = {};
    frame.p.ch0 = sweep;
    frame.p.ch1 = CRSF_CHANNEL_VALUE_MID;
    frame.p.ch2 = CRSF_CHANNEL_VALUE_STD_MIN; // throttle low
    frame.p.ch3 = CRSF_CHANNEL_VALUE_MID;
    frame.p.ch4 = CRSF_CHANNEL_VALUE_STD_MIN; // AUX1 (usual arm switch) low
    frame.p.ch5 = CRSF_CHANNEL_VALUE_MID;
    frame.p.ch6 = CRSF_CHANNEL_VALUE_MID;
    frame.p.ch7 = CRSF_CHANNEL_VALUE_MID;
    frame.p.ch8 = CRSF_CHANNEL_VALUE_MID;
    frame.p.ch9 = CRSF_CHANNEL_VALUE_MID;
    frame.p.ch10 = CRSF_CHANNEL_VALUE_MID;
    frame.p.ch11 = CRSF_CHANNEL_VALUE_MID;
    frame.p.ch12 = CRSF_CHANNEL_VALUE_MID;
    frame.p.ch13 = CRSF_CHANNEL_VALUE_MID;
    frame.p.ch14 = CRSF_CHANNEL_VALUE_MID;
    frame.p.ch15 = CRSF_CHANNEL_VALUE_MID;
    crsfRouter.SetHeaderAndCrc(&frame.h, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, CRSF_FRAME_SIZE(sizeof(crsf_channels_t)));

    const size_t written = port.write((const uint8_t *)&frame, sizeof(frame));
    if (framesSent++ == 0)
    {
        DBGLN("bench: sending CRSF RC frames on UART0 (%u bytes each)", sizeof(frame));
    }
    if (written != sizeof(frame))
    {
        DBGLN("bench: short write %u of %u bytes", written, sizeof(frame));
    }
    if (nowMs - lastReportMs >= 5000)
    {
        lastReportMs = nowMs;
        DBGLN("bench: %u CRSF frames sent, channel 1 = %u", framesSent, sweep);
    }
}
