/*
 * Copyright (c) 2019 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/environment/LPS22HB.cpp
 */

#include "LPS22HB.h"

namespace sensors::environment
{

async(LPS22HB::Init)
async_def(uint8_t id)
{
    MYDBG("Reading ID...");

    if (!await(ReadRegister, Register::ID, f.id))
    {
        async_return(false);
    }

    if ((IDValue)f.id != IDValue::Valid)
    {
        MYDBG("Invalid ID: %02X != %02X", f.id, IDValue::Valid);
        async_return(false);
    }

    MYDBG("Init complete, ID: %02X", f.id);
    async_return(init = true);
}
async_end

async(LPS22HB::Measure)
async_def(struct { Status status; uint32_t pressure : 24; int16_t temperature; } data;)
{
    if (!init && !await(Init))
    {
        async_return(false);
    }

    if (Rate() == Control1::RateOneShot)
    {
        if (!await(Trigger) || !await(WaitForDataMs, 1000))
        {
            async_return(false);
        }
    }

    if (!await(ReadRegister, Register::Status, f.data))
    {
        MYDBG("Failed to read data");
        async_return(false);
    }

    if ((f.data.status & (Status::PressureAvailable | Status::TemperatureAvailable)) != (Status::PressureAvailable | Status::TemperatureAvailable))
    {
        MYDBG("Data not available");
        async_return(false);
    }

    pressure = f.data.pressure * (1.0f / 4096);
    temperature = f.data.temperature * 0.01f;
    MYDBG("new data: P=%.3q, T=%.2q", int(pressure * 1000), int(temperature * 100));
    async_return(true);
}
async_end

async(LPS22HB::DataReady)
async_def(Status stat)
{
    async_return(await(ReadRegister, Register::Status, f.stat) &&
        (f.stat & (Status::PressureAvailable | Status::TemperatureAvailable)) == (Status::PressureAvailable | Status::TemperatureAvailable));
}
async_end

async(LPS22HB::WaitForDataTicks, mono_t ticksTimeout)
async_def(mono_t waitUntil)
{
    f.waitUntil = MONO_CLOCKS + ticksTimeout;

    while (!await(DataReady))
    {
        if (OVF_GE(MONO_CLOCKS, f.waitUntil))
        {
            MYDBG("Timeout while waiting for measurement");
            async_return(false);
        }
        async_delay_until(OVF_MIN(MONO_CLOCKS + MonoFromMilliseconds(10), f.waitUntil));
    }

    async_return(true);
}
async_end

}
