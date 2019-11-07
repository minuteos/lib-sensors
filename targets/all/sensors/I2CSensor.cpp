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

async(I2CSensor::ReadRegisterImpl, uint8_t reg, Buffer buf)
async_def()
{
    if (await(i2c.Write, address, reg, true, false) != 1)
    {
        MYDBG("Failed to write register %02X address", reg);
        async_return(false);
    }

    size_t len = await(i2c.Read, address, buf, false, true);
    if (len != buf.Length())
    {
        MYDBG("Failed to read register %02X value, error at %d/%d", reg, len, buf.Length());
        async_return(false);
    }

    async_return(true);
}
async_end

async(I2CSensor::WriteRegisterImpl, uint8_t reg, Span buf)
async_def()
{
    if (await(i2c.Write, address, reg, true, buf.Length() == 0) != 1)
    {
        MYDBG("Failed to write register %02X address", reg);
        async_return(false);
    }

    if (buf.Length())
    {
        size_t len = await(i2c.Write, buf, true);
        if (len != buf.Length())
        {
            MYDBG("Failed to write register %02X value, error at %d/%d", reg, len, buf.Length());
            async_return(false);
        }
    }

    async_return(true);
}
async_end

}
