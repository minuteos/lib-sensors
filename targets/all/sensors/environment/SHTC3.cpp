/*
 * Copyright (c) 2019 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/environment/SHTC3.cpp
 */

#include "SHTC3.h"

namespace sensors::environment
{

async(SHTC3::Init)
async_def(uint8_t id[3])
{
    MYDBG("Initializing...");

    // it's likely that the SHTC3 is sleeping
    await(WriteCommand, Command::Wake);
    async_delay_ms(1);

    if (!await(WriteCommand, Command::ReadID, false))
    {
        async_delay_ms(100);
        await(WriteCommand, Command::Wake);
        async_delay_ms(1);
        if (!await(WriteCommand, Command::ReadID, false))
        {
            async_return(false);
        }
    }

    MYDBG("Reading ID...");

    if (await(Read, f.id, false, true) != sizeof(f.id))
    {
        MYDBG("Failed to read ID");
        async_return(false);
    }

    await(WriteCommand, Command::Sleep);
    MYDBG("Init complete, ID: %H", Span(f.id));
    init = true;
    async_return(true);
}
async_end

async(SHTC3::WriteCommand, Command cmd, bool stop)
async_def(uint8_t cmd[2])
{
    f.cmd[0] = (unsigned)cmd >> 8;
    f.cmd[1] = (uint8_t)cmd;

    if (await(Write, f.cmd, true, stop) != 2)
    {
        MYDBG("Failed to send command %04X", cmd);
        async_return(false);
    }
    else
    {
        async_return(true);
    }
}
async_end

async(SHTC3::Measure)
async_def(uint8_t data[6]; bool success;)
{
    if (!init)
    {
        await(Init);
        if (!init)
        {
            async_return(false);
        }
    }

    await(WriteCommand, Command::Wake);
    async_delay_ms(1);

    if (await(WriteCommand, lowPower ? Command::MeasureLowPower : Command::Measure, false) &&
        await(Read, f.data, false, true) == sizeof(f.data))
    {
        uint16_t rawTemp = (f.data[0] << 8) | f.data[1];
        uint16_t rawHum = (f.data[3] << 8) | f.data[4];
        temp = -45 + rawTemp * (175 / 65536.0f);
        hum = rawHum / 65536.0f;
        MYDBG("new data: t=%.1q (%04X) H=%.1q%% (%04X)", int(temp * 10), rawTemp, int(hum * 1000), rawHum);
        f.success = true;
    }
    else
    {
        temp = hum = NAN;
        MYDBG("failed to read data");
        f.success = false;
    }

    await(WriteCommand, Command::Sleep);
    async_return(f.success);
}
async_end

}
