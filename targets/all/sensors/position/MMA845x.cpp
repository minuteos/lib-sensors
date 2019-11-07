/*
 * Copyright (c) 2019 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/position/MMA845x.cpp
 */

#include "MMA845x.h"

namespace sensors::position
{

async(MMA845x::Init)
async_def(Control2 ctl2)
{
    MYDBG("Reading ID...");
    if (!await(ReadRegister, Register::ID, id))
    {
        async_return(false);
    }

    switch (id)
    {
        case IDValue::MMA8451:
        case IDValue::MMA8452:
        case IDValue::MMA8453:
            MYDBG("MMA845%d detected", (uint8_t)id >> 4);
            break;

        default:
            MYDBG("Unsupported ID %02X", id);
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

    async_delay_ms(10);

    do
    {
        if (!await(ReadRegister, Register::Control2, f.ctl2))
        {
            async_return(false);
        }
    } while (!!(f.ctl2 & Control2::Reset));

    if (!await(ReadRegister, Register::DataConfig, cfgActual.dcfg) ||
        !await(ReadRegister, Register::Control1, cfgActual.ctl))
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

async(MMA845x::Configure, Config cfg)
async_def()
{
    cfgDesired.ctl.reg1 = (Control1)cfg;
    cfgDesired.ctl.reg2 = (Control2)((uint32_t)cfg >> 8);
    cfgDesired.dcfg = (DataConfig)((uint32_t)cfg >> 16);

    if (init && !await(UpdateConfiguration))
    {
        async_return(false);
    }

    async_return(true);
}
async_end

async(MMA845x::UpdateConfiguration)
async_def(bool wasActive)
{
    if (cfgActual.CompareValue() != cfgDesired.CompareValue())
    {
        f.wasActive = IsActive();
        if (f.wasActive && !await(Stop))
        {
            async_return(false);
        }

        if (!await(WriteRegister, Register::DataConfig, cfgDesired.dcfg) ||
            !await(WriteRegister, Register::Control1, cfgDesired.ctl))
        {
            // need re-init
            init = false;
            async_return(false);
        }

        cfgActual = cfgDesired;

        if (f.wasActive && !await(Start))
        {
            async_return(false);
        }
    }

    mul = GetScale() / 32768.0f;

    async_return(true);
}
async_end

async(MMA845x::Start)
async_def()
{
    if (!init && !await(Init))
    {
        async_return(false);
    }

    if (!IsActive())
    {
        cfgActual.ctl.reg1 = cfgActual.ctl.reg1 | Control1::Active;
        if (!await(WriteRegister, Register::Control1, cfgActual.ctl.reg1))
        {
            init = false;
            async_return(false);
        }

        MYDBG("started, DCFG = %02X, CTL1 = %02X, CTL2 = %02X", cfgActual.dcfg, cfgActual.ctl.reg1, cfgActual.ctl.reg2);
    }

    async_return(true);
}
async_end

async(MMA845x::Stop)
async_def()
{
    if (init && IsActive())
    {
        cfgActual.ctl.reg1 = cfgActual.ctl.reg1 & ~Control1::Active;
        if (!await(WriteRegister, Register::Control1, cfgActual.ctl.reg1))
        {
            init = false;
            async_return(false);
        }

        MYDBG("stopped");
    }

   async_return(true);
}
async_end

async(MMA845x::Measure)
async_def(
    PACKED_UNALIGNED_STRUCT
    {
        Status status;
        int16_t x, y, z;
    } data;
)
{
    if (!IsActive() && !await(Start))
    {
        // start measuring if not started
        async_return(false);
    }

    if (!await(ReadRegister, Register::Status, f.data) || (f.data.status & Status::ReadyAll) != Status::ReadyAll)
    {
        MYDBG("no data available, status: %02X, ctl1: %02X", f.data.status, cfgActual.ctl.reg1);
        async_return(false);
    }

    if (!!(f.data.status & Status::OverrunAny))
    {
        MYDBG("overrun, status: %02X", f.data.status);
    }

    x = int16_t(FROM_BE16(f.data.x)) * mul;
    y = int16_t(FROM_BE16(f.data.y)) * mul;
    z = int16_t(FROM_BE16(f.data.z)) * mul;
    MYDBG("new data: X=%.3q Y=%.3q Z=%.3q", int(x * 1000), int(y * 1000), int(z * 1000));
    async_return(true);
}
async_end

}
