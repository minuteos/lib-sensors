/*
 * Copyright (c) 2019 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/environment/CCS811.cpp
 */

#include "CCS811.h"

namespace sensors::environment
{

async(CCS811::SetMode, DriveMode mode)
async_def()
{
    if (this->mode != mode)
    {
        this->mode = mode;
        if (init)
        {
            // device initialized, try to change mode immediately
            await(Wake);
            msg.mode.raw = 0;
            msg.mode.driveMode = (uint8_t)mode;
            DBG("Changing mode to %d", mode);
            init = await(WriteRegister, Register::Mode, msg.mode);
            await(Sleep);
        }
    }
}
async_end

async(CCS811::Init)
async_def()
{
    MYDBG("Resetting...");
    async_yield();
    wakeCount++;
    wake.Res();
    reset.Res();
    async_delay_ms(1);
    reset.Set();
    async_delay_ms(1);

    MYDBG("Initializing...");

    if (!await(ReadRegister, Register::HWID, msg.hwId))
        goto done;

    if (msg.hwId != HwID::CCS811)
    {
        MYDBG("Unexpected HWID value: %02X, expected %02X", msg.hwId, HwID::CCS811);
        goto done;
    }

    for (;;)
    {
        if (!await(ReadRegister, Register::Status, msg.status))
            goto done;

        MYDBG("STATUS: %02X", msg.status.raw);
        if (msg.status.appRunning)
            break;

        MYDBG("Firmware in boot mode, starting application");
        if (!await(WriteRegister, Register::BootAppStart, Span()))
            goto done;
        async_delay_ms(10);
    }

    if (!await(ReadRegister, Register::Mode, msg.mode))
        goto done;

    MYDBG("MODE: %d %c%c", msg.mode.driveMode, msg.mode.intDataReady ? 'I' : '-', msg.mode.intThreshold ? 'T': '-');
    if (msg.mode.driveMode != (uint8_t)mode)
    {
        MYDBG("Setting mode %d", mode);
        msg.mode.driveMode = (uint8_t)mode;
        if (!await(WriteRegister, Register::Mode, msg.mode))
            goto done;
    }

#if TRACE
    if (!await(ReadRegister, Register::HWVersion, msg.hwVer))
        goto done;

    MYDBG("HWVER: %d.%d", msg.hwVer.major, msg.hwVer.build);

    if (!await(ReadRegister, Register::FWBootVersion, msg.bootVer))
        goto done;

    MYDBG("BOOT: %d.%d.%d", msg.bootVer.major, msg.bootVer.minor, msg.bootVer.trivial);

    if (!await(ReadRegister, Register::FWAppVersion, msg.appVer))
        goto done;

    MYDBG("APP: %d.%d.%d", msg.appVer.major, msg.appVer.minor, msg.appVer.trivial);

    if (!await(ReadRegister, Register::ErrorID, msg.error))
        goto done;

    MYDBG("ERROR: %d", msg.error);
#endif

    RequestUpdate();
    async_yield();
    init = true;
done:
    await(Sleep);
    async_return(init);
}
async_end

async(CCS811::Measure)
async_def(bool success; int i)
{
    if (init || await(Init))
    {
        await(Wake);

        if (mode == DriveMode::Idle)
        {
            // enable the fastest mode for one measurement
            msg.mode.raw = 0;
            msg.mode.driveMode = (uint8_t)DriveMode::ConstantPower_250ms;
            if (await(WriteRegister, Register::Mode, msg.mode))
            {
                // measurement takes at least one second
                async_delay_ms(980);
                // wait up to another second for the result
                for (f.i = 0; f.i < 50; f.i++)
                {
                    async_delay_ms(20);
                    if (!await(ReadRegister, Register::Result, msg.result))
                        continue;

                    if (msg.result.status.error)
                    {
                        MYDBG("ERROR %d", msg.result.error);
                        break;
                    }

                    if (msg.result.status.dataReady)
                    {
                        f.success = true;
                        co2 = FROM_BE16(msg.result.co2BE);
                        tvoc = FROM_BE16(msg.result.tvocBE);
                        MYDBG("new data: CO2=%dppm, TVOC=%dppb, RAW I=%duA, ADC=%d",
                            (int)co2, (int)tvoc, msg.result.raw.current, (msg.result.raw.adcHi) << 8 | msg.result.raw.adcLo);
                        break;
                    }
                }

                msg.mode.raw = 0;
                msg.mode.driveMode = (uint8_t)DriveMode::Idle;
                await(WriteRegister, Register::Mode, msg.mode);
            }
        }
        else if (await(ReadRegister, Register::Result, msg.result))
        {
            if (msg.result.status.error)
            {
                MYDBG("ERROR %d", msg.result.error);
            }
            else
            {
                f.success = true;
                co2 = FROM_BE16(msg.result.co2BE);
                tvoc = FROM_BE16(msg.result.tvocBE);
                raw = FROM_BE16(msg.result.raw.raw);
                MYDBG("%s data: CO2=%dppm, TVOC=%dppb, RAW I=%duA, ADC=%d",
                    msg.result.status.dataReady ? "new" : "old",
                    (int)co2, (int)tvoc, msg.result.raw.current, (msg.result.raw.adcHi) << 8 | msg.result.raw.adcLo);
            }
        }
    }

    if (!f.success)
    {
        MYDBG("Measurement FAILED");
        init = false;
        co2 = tvoc = NAN;
        raw = ~0;
        envSet = { ~0u };
    }

    await(Sleep);
    async_return(f.success && msg.result.status.dataReady);
}
async_end

void CCS811::RequestUpdate()
{
    if (!update && UpdateRequired())
    {
        update = true;
        kernel::Task::Run(this, &CCS811::Update);
    }
}

async(CCS811::Wake)
async_def()
{
    if (!wakeCount++)
    {
        wake.Res();
        async_delay_ms(1);
    }
}
async_end

async(CCS811::Sleep)
async_def_sync()
{
    ASSERT(wakeCount);
    if (!--wakeCount)
    {
        wake.Set();
    }
}
async_end

async(CCS811::Update)
async_def(
    EnvData env;
    bool wake;
)
{
    while (envSet.raw != envCfg.raw)
    {
        if (!f.wake)
        {
            f.wake = true;
            await(Wake);
        }

        f.env = envCfg;
        if (!await(WriteRegister, Register::EnvData, f.env))
        {
            break;
        }
        envSet = f.env;
    }

    if (f.wake)
    {
        await(Sleep);
    }

    update = false;
}
async_end

}

