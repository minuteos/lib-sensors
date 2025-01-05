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

    if (!await(ReadRegister, Register::FifoCtrl1, fifoActual))
    {
        async_return(false);
    }

    if (!await(UpdateConfiguration))
    {
        async_return(false);
    }

    MYDBG("Init complete");
    lastFifoTag = 0;
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

    if (Span(fifoActual) != Span(fifoDesired))
    {
        MYDBG("Updating FIFO configuration: %H > %H",
            Span(fifoActual),
            Span(fifoDesired));
        if (!await(WriteRegister, Register::FifoCtrl1, fifoDesired))
        {
            // need re-init
            init = false;
            async_return(false);
        }

        fifoActual = fifoDesired;
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

    if (!await(ReadRegister, Register::Status, f.data))
    {
        init = false;
        async_return(false);
    }

    if ((f.data.status & Status::ReadyAll) != Status::ReadyAll)
    {
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

static bool parity(uint8_t val)
{
    val ^= val >> 4;
    val ^= val >> 2;
    val ^= val >> 1;
    return val & 1;
}

async(LSM6DSO::FifoRead)
async_def(
    PACKED_UNALIGNED_STRUCT
    {
        union
        {
            uint8_t rawtag;
            PACKED_UNALIGNED_STRUCT
            {
                bool tagParity : 1;
                uint8_t tagCnt : 2;
                FifoTag tag : 5;
            };
        };
        int16_t x, y, z;
    } data;
)
{
    if (!init && !await(Init))
    {
        async_return(0);
    }

    if (!await(ReadRegister, Register::FifoOutTag, f.data))
    {
        init = false;
        async_return(0);
    }

    bool change = f.data.rawtag != lastFifoTag;
    lastFifoTag = f.data.rawtag;
    if (!change)
    {
        // no new data
        async_return(0);
    }

    if (f.data.tag != FifoTag::NoData)
    {
        if (parity(f.data.rawtag))
        {
            MYDBG("Fifo parity error: %X", f.data.rawtag);
            async_return(0);
        }

        switch (f.data.tag)
        {
            case FifoTag::AccelNc:
                ax = int16_t(FROM_LE16(f.data.x)) * amul;
                ay = int16_t(FROM_LE16(f.data.y)) * amul;
                az = int16_t(FROM_LE16(f.data.z)) * amul;
                MYTRACE("new data: aX=%.3q aY=%.3q aZ=%.3q (%H)",
                    int(ax * 1000), int(ay * 1000), int(az * 1000),
                    Span(f.data));
                break;

            case FifoTag::GyroNc:
                gx = int16_t(FROM_LE16(f.data.x)) * gmul;
                gy = int16_t(FROM_LE16(f.data.y)) * gmul;
                gz = int16_t(FROM_LE16(f.data.z)) * gmul;
                MYTRACE("new data: gX=%.3q gY=%.3q gZ=%.3q (%H)",
                    int(gx * 1000), int(gy * 1000), int(gz * 1000),
                    Span(f.data));
                break;

            default:
                MYDBG("fifo?: %X %d %d %d %d %d", f.data.tag, f.data.tagCnt, f.data.tagParity, f.data.x, f.data.y, f.data.z);
                break;
        }
    }

    async_return(intptr_t(f.data.tag));
}
async_end

}
