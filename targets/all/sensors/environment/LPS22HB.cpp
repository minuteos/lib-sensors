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

async(LPS22HB::InitImpl, InitConfig cfg)
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

    if (!await(WriteRegister, Register::Control2, Control2::Reset) ||
        !await(WriteRegister, Register::Control1, cfg.ctl1) ||
        !await(WriteRegister, Register::FifoControl, cfg.fifo) ||
        !await(WriteRegister, Register::Control2, cfg.ctl2))
    {
        async_return(false);
    }

    this->cfg = cfg;
    MYDBG("Init complete, ID: %02X, CTL1: %02X, CTL2: %02X, FIFO: %02X", f.id, cfg.ctl1, cfg.ctl2, cfg.fifo);
    async_return(init = true);
}
async_end

async(LPS22HB::Measure)
async_def(PACKED_UNALIGNED_STRUCT { Status status; Sample smp; } data;)
{
    if (!init && !await(Init))
    {
        async_return(false);
    }

    if (Rate() == Control1::RateOneShot)
    {
        if (!await(Trigger) || !await(WaitForData, Timeout::Seconds(1)))
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

    pressure = f.data.smp.Pressure();
    temperature = f.data.smp.Temperature();
    MYDBG("new data: P=%.3q, T=%.2q", int(pressure * 1000), int(temperature * 100));
    async_return(true);
}
async_end

async(LPS22HB::Trigger)
async_def(Control2 ctl2)
{
    f.ctl2 = cfg.ctl2 | Control2::Trigger;
    async_return(await(WriteRegister, Register::Control2, f.ctl2));
}
async_end

async(LPS22HB::DataReady)
async_def(
    FifoStatus stat;
)
{
    async_return(await(ReadRegister, Register::FifoStatus, f.stat) ? f.stat.count : 0);
}
async_end

async(LPS22HB::ReadFifo, Sample* buffer, size_t count)
async_def(
    FifoStatus stat;
    size_t count;
)
{
    if (count == 0 || !await(ReadRegister, Register::FifoStatus, f.stat) || f.stat.count == 0)
    {
        async_return(0);
    }

    f.count = std::min(count, (size_t)f.stat.count);
    if (!await(ReadRegister, Register::Data, Buffer(buffer, f.count * sizeof(Sample))))
    {
        async_return(0);
    }

    async_return(f.count);
}
async_end

async(LPS22HB::WaitForData, Timeout timeout)
async_def(
    Timeout timeout;
)
{
    f.timeout = timeout.MakeAbsolute();

    while (!await(DataReady))
    {
        if (timeout.Elapsed())
        {
            MYDBG("Timeout while waiting for measurement");
            async_return(false);
        }
        async_delay_timeout(std::min(timeout.Milliseconds(10).MakeAbsolute(), f.timeout));
    }

    async_return(true);
}
async_end

}
