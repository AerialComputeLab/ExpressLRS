#include "PWM.h"

#if defined(PLATFORM_RP2040)
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>

#include "logging.h"

// Every GPIO maps to a fixed slice (0-7) and channel (A/B), so the channel handle is just
// the GPIO number. Both channels of a slice share one counter and therefore one frequency,
// and GPIO n and n+16 share the same slice channel.
#define PWM_SLICES 8

static struct
{
    uint32_t frequency;     // 0 when the slice is unused
    uint32_t tickHz;        // counter rate after the clock divider
    uint16_t wrap;          // counter TOP, the period is wrap+1 ticks
    int8_t pins[2];         // pin using channel A/B, or -1
} slices[PWM_SLICES];

static bool slicesInitialised = false;

static void initSlices()
{
    for (auto &slice : slices)
    {
        slice.frequency = 0;
        slice.pins[0] = -1;
        slice.pins[1] = -1;
    }
    slicesInitialised = true;
}

pwm_channel_t PWMController::allocate(uint8_t pin, uint32_t frequency)
{
    if (!slicesInitialised)
    {
        initSlices();
    }
    if (pin >= NUM_BANK0_GPIOS || frequency == 0)
    {
        ERRLN("Invalid PWM pin %d / frequency %d", pin, frequency);
        return -1;
    }

    const uint sliceNum = pwm_gpio_to_slice_num(pin);
    const uint chan = pwm_gpio_to_channel(pin);
    auto &slice = slices[sliceNum];

    if (slice.pins[chan] != -1)
    {
        DBGLN("PWM slice %d channel %d already used by pin %d", sliceNum, chan, slice.pins[chan]);
        return -1;
    }
    if (slice.frequency != 0 && slice.frequency != frequency)
    {
        DBGLN("PWM slice %d already running at %dHz, can't use %dHz on pin %d", sliceNum, slice.frequency, frequency, pin);
        return -1;
    }

    if (slice.frequency == 0)
    {
        // Smallest integer divider that fits one period in the 16 bit counter, for best resolution
        const uint32_t sysHz = clock_get_hz(clk_sys);
        uint32_t div = (uint32_t)(((uint64_t)sysHz + (uint64_t)frequency * 65536 - 1) / ((uint64_t)frequency * 65536));
        if (div < 1)
            div = 1;
        if (div > 255)
        {
            ERRLN("PWM frequency %dHz is too low", frequency);
            return -1;
        }

        slice.frequency = frequency;
        slice.tickHz = sysHz / div;
        slice.wrap = (uint16_t)(slice.tickHz / frequency - 1);

        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv_int(&cfg, div);
        pwm_config_set_wrap(&cfg, slice.wrap);
        pwm_init(sliceNum, &cfg, false);
        pwm_set_both_levels(sliceNum, 0, 0);
        pwm_set_enabled(sliceNum, true);
    }

    slice.pins[chan] = pin;
    pwm_set_chan_level(sliceNum, chan, 0);
    gpio_set_function(pin, GPIO_FUNC_PWM);

    DBGLN("allocate PWM slice %d channel %d on pin %d, %dHz, wrap %d", sliceNum, chan, pin, frequency, slice.wrap);
    return pin;
}

void PWMController::release(pwm_channel_t channel)
{
    if (!slicesInitialised || channel < 0 || channel >= NUM_BANK0_GPIOS)
    {
        ERRLN("Invalid PWM channel %x", channel);
        return;
    }

    const uint sliceNum = pwm_gpio_to_slice_num(channel);
    const uint chan = pwm_gpio_to_channel(channel);
    auto &slice = slices[sliceNum];
    if (slice.pins[chan] != channel)
    {
        ERRLN("Invalid PWM channel %x", channel);
        return;
    }

    pwm_set_chan_level(sliceNum, chan, 0);
    gpio_init(channel);
    slice.pins[chan] = -1;

    if (slice.pins[0] == -1 && slice.pins[1] == -1)
    {
        pwm_set_enabled(sliceNum, false);
        slice.frequency = 0;
    }
}

void PWMController::setDuty(pwm_channel_t channel, uint16_t duty)
{
    const uint sliceNum = pwm_gpio_to_slice_num(channel);
    const auto &slice = slices[sliceNum];
    // duty is in tenths of a percent, 0 gives a constant low output
    const uint32_t level = ((uint32_t)slice.wrap + 1) * duty / 1000;
    pwm_set_chan_level(sliceNum, pwm_gpio_to_channel(channel), level);
}

void PWMController::setMicroseconds(pwm_channel_t channel, uint16_t microseconds)
{
    const uint sliceNum = pwm_gpio_to_slice_num(channel);
    const auto &slice = slices[sliceNum];
    // 0us gives a constant low output (no pulses)
    const uint32_t level = (uint32_t)(((uint64_t)microseconds * slice.tickHz) / 1000000U);
    pwm_set_chan_level(sliceNum, pwm_gpio_to_channel(channel), level);
}

void PWMController::feedWatchdog()
{
}

#endif
