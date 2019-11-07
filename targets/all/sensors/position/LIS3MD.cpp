/*
 * Copyright (c) 2019 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/position/LIS3MD.cpp
 */

#include "LIS3MD.h"

namespace sensors::position
{

async(LIS3MD::Init)
async_def(IDValue id; Control2 ctl2;)
{
    MYDBG("Reading ID...");
    if (!await(ReadRegister, Register::ID, f.id))
    {
        async_return(false);
    }

    if (f.id != IDValue::Valid)
    {
        MYDBG("Unsupported ID %02X", f.id);
        async_return(false);
    }

    // reset the device to make sure we're in a defined state
    MYDBG("Resetting...");
    f.ctl2 = Control2::Reset;
    if (!await(WriteRegister, Register::Control2, f.ctl2))
    {
        MYDBG("Failed to reset device");
        async_return(false);
    }

    do
    {
        if (!await(ReadRegister, Register::Control2, f.ctl2))
        {
            async_return(false);
        }
    } while (!!(f.ctl2 & Control2::Reset));

    if (!await(ReadRegister, Register::Control1, cfgActual))
    {
        async_return(false);
    }

    if (!await(UpdateConfiguration))
    {
        async_return(false);
    }

    MYDBG("Init complete");
    async_return(init = true);
}
async_end

async(LIS3MD::Configure, Config cfg)
async_def()
{
    cfgDesired.ctl1 = (Control1)cfg;
    cfgDesired.ctl2 = (Control2)((uint32_t)cfg >> 8);
    cfgDesired.ctl4 = (Control4)((uint32_t)cfg >> 16);

    if (init && !await(UpdateConfiguration))
    {
        async_return(false);
    }

    async_return(true);
}
async_end

async(LIS3MD::UpdateConfiguration)
async_def()
{
    if (cfgActual.combined != cfgDesired.combined)
    {
        if (!await(WriteRegister, Register::Control1, cfgDesired))
        {
            // need re-init
            init = false;
            async_return(false);
        }

        cfgActual = cfgDesired;
    }

    mul = cfgActual.GetScale() / 32768.0f;

    async_return(true);
}
async_end

async(LIS3MD::Measure)
async_def(
    PACKED_UNALIGNED_STRUCT
    {
        Status status;
        int16_t x, y, z;
    } data;
)
{
    if (!init && !await(Init))
    {
        async_return(false);
    }

    if (cfgActual.IsPowerDown())
    {
        // do a single conversion
        cfgActual.ctl3 = (cfgActual.ctl3 & ~Control3::ModeMask) | Control3::ModeSingle;
        if (!await(WriteRegister, Register::Control3, cfgActual.ctl3))
        {
            init = true;
            async_return(false);
        }

        do
        {
            if (!await(ReadRegister, Register::Control3, cfgActual.ctl3))
            {
                init = true;
                async_return(false);
            }
        } while (!cfgActual.IsPowerDown());
    }

    if (!await(ReadRegister, Register::Status, f.data) || (f.data.status & Status::ReadyAll) != Status::ReadyAll)
    {
        MYDBG("no data available, status: %02X", f.data.status);
        async_return(false);
    }

    if (!!(f.data.status & Status::OverrunAny))
    {
        MYDBG("overrun, status: %02X", f.data.status);
    }

    x = int16_t(FROM_LE16(f.data.x)) * mul;
    y = int16_t(FROM_LE16(f.data.y)) * mul;
    z = int16_t(FROM_LE16(f.data.z)) * mul;
    MYDBG("new data: X=%.3q Y=%.3q Z=%.3q (%H)", int(x * 1000), int(y * 1000), int(z * 1000), Span(f.data));
    async_return(true);
}
async_end

}
