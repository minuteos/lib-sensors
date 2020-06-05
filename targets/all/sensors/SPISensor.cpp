/*
 * Copyright (c) 2020 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/SPISensor.cpp
 */

#include "SPISensor.h"

namespace sensors
{

async(SPISensor::ReadRegisterImpl, RegAndLength arg, void* buf)
async_def(
   bus::SPI::Descriptor tx[2];
   uint8_t hdr;
)
{
    f.hdr = arg.reg | hdrRead;
    await(spi.Acquire, cs);
    f.tx[0].Transmit(f.hdr);
    f.tx[1].Receive(Buffer(buf, arg.length));
    await(spi.Transfer, f.tx);
    spi.Release();
    async_return(true);
}
async_end

async(SPISensor::WriteRegisterImpl, RegAndLength arg, const void* buf)
async_def(
    bus::SPI::Descriptor tx[2];
    uint8_t hdr;
)
{
    f.hdr = arg.reg | hdrWrite;
    await(spi.Acquire, cs);
    f.tx[0].Transmit(f.hdr);
    f.tx[1].Transmit(Span(buf, arg.length));
    await(spi.Transfer, f.tx);
    spi.Release();
    async_return(true);
}
async_end

}
