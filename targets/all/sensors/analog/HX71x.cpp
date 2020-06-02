/*
 * Copyright (c) 2020 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/analog/HX71x.cpp
 */

#include "HX71x.h"

#define MYDBG(...)  DBGCL("HX71x", __VA_ARGS__)

namespace sensors::analog
{

static ALWAYS_INLINE void clkdelay()
{
    // TODO: may need to be adjusted for other CPU frequencies
    __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP();
}

async(HX71x::Init, MeasurementType type)
async_def()
{
    this->type = type ? type : MeasurementType::Default;

    sck.ConfigureDigitalOutput(false);
    dout.ConfigureDigitalInput();
    refEna.ConfigureDigitalOutput(false);

    // wait for the initialization to complete
    await(dout.WaitFor, false);

    // configure the selected measurement type
    for (int i = 0; i < 24 + this->type; i++)
    {
        clkdelay();
        sck.Toggle();
        clkdelay();
        sck.Toggle();
    }
}
async_end

async(HX71x::Measure, MeasurementType nextType)
async_def()
{
    if (sck)
    {
        // power on, but make sure it is really powered down if it was done just a while ago
        MYDBG("waking up");
        mono_signed_t powerDownTicks = MonoFromMicroseconds(60) + 1;
        mono_signed_t pdRemain = powerDownAt + powerDownTicks - MONO_CLOCKS;
        if (pdRemain > 0 && pdRemain < powerDownTicks)
        {
            async_delay_until(powerDownTicks);
        }
        sck.Res();
        refEna.Set();
    }

    // wait for the measurement
    await(dout.WaitFor, false);

    // read out data
    int res = 0;
    auto sck = this->sck;
    auto dout = this->dout;
    for (int i = 0; i < 24; i++)
    {
        sck.Toggle();
        clkdelay();
        sck.Toggle();
        clkdelay();
        res = (res << 1) | dout.Get();
    }

    // convert into float value
    value = (res << 8 >> 8) * (float)(1.0 / BIT(23));

    if (nextType)
    {
        type = nextType;
    }
    for (int i = 0; i < type; i++)
    {
        sck.Toggle();
        clkdelay();
        sck.Toggle();
        clkdelay();
    }

    MYDBG("value = %06X, next = %d", res, nextType);
}
async_end

async(HX71x::PowerDown)
async_def_sync()
{
    sck.Set();
    powerDownAt = MONO_CLOCKS;
}
async_end

}
