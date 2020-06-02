/*
 * Copyright (c) 2020 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/position/LIS3DH.cpp
 */

#include "LIS3DH.h"

namespace sensors::position
{

async(LIS3DH::InitImpl, InitConfig cfg)
async_def(
    uint8_t id;
)
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

    if (!await(WriteRegister, Register::Control5, Control5::Reset) ||
        !await(WriteRegister, Register::Control5, cfg.ctl5) ||
        !await(WriteRegister, Register::Control4, cfg.ctl4) ||
        !await(WriteRegister, Register::FifoControl, cfg.fifo) ||
        !await(WriteRegister, Register::Control1, cfg.ctl1))
    {
        async_return(false);
    }

    // calculate multiplier by mg/digit at 10th bit
    this->cfg = cfg;
    auto scaleIndex = (unsigned(cfg.ctl4) >> 4) & 3;
    auto scale = BYTES(4, 8, 16, 48)[scaleIndex];
    mul = scale * float(0.001f/64);
    MYDBG("Init complete, scaleIndex: %d, scale: %d, ID: %02X, CTL1: %02X, CTL4: %02X, CTL5: %02X, FIFO: %02X", scaleIndex, scale, f.id, cfg.ctl1, cfg.ctl4, cfg.ctl5, cfg.fifo);
    async_return(init = true);
}
async_end

async(LIS3DH::ReadFifo, Sample* buffer, size_t count)
async_def(
    FifoStatus stat;
    size_t count;
)
{
    if (count == 0 || !await(ReadRegister, Register::FifoStatus, f.stat) || f.stat.count == 0)
    {
        async_return(0);
    }

    f.count = std::min(count, size_t(f.stat.count + f.stat.overrun));
    if (!await(ReadRegister, Register::Data, Buffer(buffer, f.count * sizeof(Sample))))
    {
        async_return(0);
    }

    async_return(f.count);
}
async_end

}
