/*
 * Copyright (c) 2019 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/position/TLE493D.cpp
 */

#include "TLE493D.h"

namespace sensors::position
{

async(TLE493D::Init)
async_def(
    union
    {
        struct
        {
            uint8_t _discard_data_regs[6];
            Diagnostics diag;
            uint8_t _discard_wake_regs[9];
            Config cfg;
            Mode1 mode1;
            uint8_t _reserved12;
            uint8_t _reserved13 : 5;
            uint8_t prd : 3;
            uint8_t _reserved14, _reserved15;
            uint8_t rev : 4;
            uint8_t feat: 2;
            uint8_t : 2;
        } data;
        struct
        {
            Register reg;
            Config cfg;
            Mode1 mode1;
        } init;
    };
)
{
    // we must start by configuring the protocol of the sensor, as it stars in a mode where reads are not I2C compatible
    MYDBG("Configuring...");
    f.init.reg = Register::Config;
    f.init.cfg = Config::TemperatureCompensationOff | Config::ReadTriggerBefore | Config::DisableTemperature;
    f.init.mode1 = Mode1::InterruptDisable | Mode1::PowerModeTrigger | Mode1::Protocol1Byte | Mode1Address();
    f.init.mode1 = f.init.mode1 | (!__builtin_parity((uint8_t)f.init.mode1) * Mode1::FuseParity);

    // double configuration to verify the device did not stop communicatings
    if (await(Write, f.init, true, true) != sizeof(f.init) ||
        await(Write, f.init, true, true) != sizeof(f.init))
    {
        MYDBG("Failed to configure sensor");
        async_return(false);
    }

    async_delay_ms(1);

    MYDBG("Reading registers...");
    if (await(Read, f.data, true, true) != sizeof(f.data))
    {
        MYDBG("Failed to read registers register");
        async_return(false);
    }

    if (!(f.data.diag & Diagnostics::FuseParity))
    {
        MYDBG("Fuse parity error, sensor not functional");
        async_return(false);
    }

    MYDBG("DIAG: %02X, PRD: %d, FEAT: %d, REV: %d", f.data.diag, f.data.prd, f.data.feat, f.data.rev);
    async_return(init = true);
}
async_end

async(TLE493D::Measure)
async_def(
    struct
    {
        int8_t bx, by, bz;
        uint8_t temp;
        uint8_t byl : 4;
        uint8_t bxl : 4;
        uint8_t bzl : 4;
        uint8_t id : 2;
        uint8_t templ : 2;
        Diagnostics diag;
    } data;
)
{
    if (!init && !await(Init))
    {
        async_return(false);
    }

    if (await(Read, f.data, true, true) != sizeof(f.data))
    {
        MYDBG("Failed to read measurement");
        async_return(false);
    }

    x = (int32_t(((f.data.bx << 4) | f.data.bxl) << 20) >> 20) * ValueMultiply;
    y = (int32_t(((f.data.by << 4) | f.data.byl) << 20) >> 20) * ValueMultiply;
    z = (int32_t(((f.data.bz << 4) | f.data.bzl) << 20) >> 20) * ValueMultiply;
    MYDBG("new data: X=%.1q Y=%.1q Z=%.1q", int(x * 10), int(y * 10), int(z * 10));
    async_return(true);
}
async_end

}
