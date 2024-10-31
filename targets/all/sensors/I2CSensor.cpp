/*
 * Copyright (c) 2019 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/I2CSensor.cpp
 */

#include "I2CSensor.h"

namespace sensors
{

async(I2CSensor::ReadRegisterImpl, RegAndLength arg, void* buf)
async_def(
    uint8_t reg;
)
{
    // we don't want to be passing a stack value to Write
    f.reg = arg.reg;
    if (!await(Write, f.reg, arg.length ? Next::Restart : Next::Stop))
    {
        if (!arg.allowFail)
        {
            MYDBG("Failed to write register %02X address", arg.reg);
        }
        async_return(false);
    }

    if (arg.length)
    {
        if (!await(Read, Buffer(buf, arg.length)))
        {
            MYDBG("Failed to read register %02X value, error at %d/%d", arg.reg, Transferred(), arg.length);
            async_return(false);
        }
    }

    async_return(true);
}
async_end

async(I2CSensor::WriteRegisterImpl, RegAndLength arg, const void* buf)
async_def(
    uint8_t reg;
)
{
    // we don't want to be passing a stack value to Write
    f.reg = arg.reg;
    if (!await(Write, f.reg, arg.length ? Next::Continue : Next::Stop))
    {
        MYDBG("Failed to write register %02X address", arg.reg);
        async_return(false);
    }

    if (arg.length)
    {
        if (!await(Write, Span(buf, arg.length)))
        {
            MYDBG("Failed to write register %02X value, error at %d/%d", arg.reg, Transferred(), arg.length);
            async_return(false);
        }
    }

    async_return(true);
}
async_end

}
