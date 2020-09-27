/*
 * Copyright (c) 2020 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/analog/FDC1004.cpp
 */

#include "FDC1004.h"

namespace sensors::analog
{

async(FDC1004::Init)
async_def(
    uint16_t value;
    FDCConfig fdcConf;
)
{
    init = false;
    if (!await(ReadRegister, Register::DEVICE_ID, f.value, true))
    {
        async_return(false);
    }

    if (f.value != DeviceID)
    {
        MYDBG("Unexpected ID: %04X, expected %04X", f.value, DeviceID);
        async_return(false);
    }

    f.fdcConf = FDCConfig::Reset;
    if (!await(WriteRegister, Register::FDC_CONF, f.fdcConf))
    {
        async_return(false);
    }

    MYDBG("Reset...");
    for (;;)
    {
        if (!await(ReadRegister, Register::FDC_CONF, f.fdcConf))
        {
            async_return(false);
        }

        if (!(f.fdcConf & FDCConfig::Reset))
        {
            break;
        }

        async_yield();
    }

    init = true;
    configuredChannels = 0;
    async_return(true);
}
async_end

async(FDC1004::Configure, unsigned channel, uint16_t cfg)
async_def()
{
    ASSERT(channel < countof(value));

    if (!await(WriteRegister, (uint8_t)Register::CONF_MEAS1 + channel, cfg))
    {
        async_return(false);
    }

    SETBIT(configuredChannels, channel);
    async_return(true);
}
async_end

async(FDC1004::SetCalibration, unsigned input, OffsetAndGain arg)
async_def()
{
    async_return(
        await(WriteRegister, (uint8_t)Register::OFFSET_CAL_CIN1 + input, arg.offset) &&
        await(WriteRegister, (uint8_t)Register::GAIN_CAL_CIN1 + input, arg.gain)
    );
}
async_end

async(FDC1004::Start, FDCConfig config)
async_def()
{
    async_return(await(WriteRegister, Register::FDC_CONF, config));
}
async_end

async(FDC1004::Wait, Timeout timeout)
async_def(
    Timeout timeout;
    FDCConfig config;
)
{
    f.timeout = timeout.MakeAbsolute();

    do
    {
        if (!await(ReadRegister, Register::FDC_CONF, f.config))
        {
            async_return(false);
        }
        if (!!(f.config & FDCConfig::DoneMask))
        {
            async_return(unsigned(f.config & FDCConfig::DoneMask) >> FDCConfigDoneOffset);
        }
        if (!(f.config & FDCConfig::EnableMask))
        {
            // no measurement is enabled, hence no measurement can complete
            async_return(false);
        }
    } while (!timeout.Elapsed());

    async_return(false);
}
async_end

async(FDC1004::Measure, Timeout timeout)
async_def(
    unsigned updated, i;
    union
    {
        struct { uint16_t msbBE, lsbBE; };
        uint32_t valueBE;
    };
)
{
    if ((f.updated = await(Wait, timeout)))
    {
        for (f.i = 0; f.i < countof(value); f.i++)
        {
            if (GETBIT(f.updated, f.i))
            {
                if (await(ReadRegister, unsigned(Register::MEAS1_MSB) + 2 * f.i, f.msbBE) &&
                    await(ReadRegister, unsigned(Register::MEAS1_MSB) + 2 * f.i, f.lsbBE))
                {
                    value[f.i] = TO_LE32(f.valueBE) * 1.0f/(1<<19);
                }
                else
                {
                    RESBIT(f.updated, f.i);
                }
            }
        }
    }

    async_return(f.updated);
}
async_end

}
