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
async_def()
{
    if (await(i2c.Write, address, arg.reg, true, false) != 1)
    {
        if (!arg.allowFail)
        {
            MYDBG("Failed to write register %02X address", arg.reg);
        }
        async_return(false);
    }

    size_t len;
    len = await(i2c.Read, address, Buffer(buf, arg.length), false, true);
    if (len != arg.length)
    {
        MYDBG("Failed to read register %02X value, error at %d/%d", arg.reg, len, arg.length);
        async_return(false);
    }

    async_return(true);
}
async_end

async(I2CSensor::WriteRegisterImpl, RegAndLength arg, const void* buf)
async_def()
{
    if (await(i2c.Write, address, arg.reg, true, arg.length == 0) != 1)
    {
        MYDBG("Failed to write register %02X address", arg.reg);
        async_return(false);
    }

    if (arg.length)
    {
        size_t len;
        len = await(i2c.Write, Span(buf, arg.length), true);
        if (len != arg.length)
        {
            MYDBG("Failed to write register %02X value, error at %d/%d", arg.reg, len, arg.length);
            async_return(false);
        }
    }

    async_return(true);
}
async_end

}
