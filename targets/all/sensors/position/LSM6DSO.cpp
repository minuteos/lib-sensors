/*
 * Copyright (c) 2024 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/position/LSM6DSO.cpp
 */

#include "LSM6DSO.h"

namespace sensors::position
{

async(LSM6DSO::Init)
async_def(IDValue id; uint8_t d;)
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
    f.d = 0x81;
    if (!await(WriteRegister, Register::Control3, f.d))
    {
        MYDBG("Failed to reset device");
        async_return(false);
    }

    do
    {
        if (!await(ReadRegister, Register::Control3, f.d))
        {
            async_return(false);
        }
    } while (f.d & 0x81);

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

async(LSM6DSO::UpdateConfiguration)
async_def()
{
    if (Span(cfgActual) != Span(cfgDesired))
    {
        MYDBG("Updating configuration: %H > %H",
            Span(cfgActual),
            Span(cfgDesired));
        if (!await(WriteRegister, Register::Control1, cfgDesired))
        {
            // need re-init
            init = false;
            async_return(false);
        }

        cfgActual = cfgDesired;
    }

    amul = cfgActual.GetAccelerationScale() * 0x1p-14f;
    gmul = cfgActual.GetAngularScale() * 0x1p-14f;

    async_return(true);
}
async_end

async(LSM6DSO::Measure)
async_def(
    PACKED_UNALIGNED_STRUCT
    {
        Status status;
        uint8_t resvd;
        int16_t temp;
        int16_t gx, gy, gz;
        int16_t ax, ay, az;
    } data;
)
{
    if (!init && !await(Init))
    {
        async_return(false);
    }

    if (!await(ReadRegister, Register::Status, f.data) || (f.data.status & Status::ReadyAll) != Status::ReadyAll)
    {
        MYDBG("no data available, status: %02X", f.data.status);
        async_return(false);
    }

    ax = int16_t(FROM_LE16(f.data.ax)) * amul;
    ay = int16_t(FROM_LE16(f.data.ay)) * amul;
    az = int16_t(FROM_LE16(f.data.az)) * amul;
    gx = int16_t(FROM_LE16(f.data.gx)) * gmul;
    gy = int16_t(FROM_LE16(f.data.gy)) * gmul;
    gz = int16_t(FROM_LE16(f.data.gz)) * gmul;
    MYTRACE("new data: aX=%.3q aY=%.3q aZ=%.3q gX=%.3q gY=%.3q gZ=%.3q (%H)",
        int(ax * 1000), int(ay * 1000), int(az * 1000),
        int(gx * 1000), int(gy * 1000), int(gz * 1000),
        Span(f.data));
    async_return(true);

}
async_end

}
