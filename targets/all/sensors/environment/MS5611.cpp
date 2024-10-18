/*
 * Copyright (c) 2024 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/environment/MS5611.cpp
 *
 * Driver for MEAS/TE Connectivity MS5611 Barometric sensor
 *
 * The conversion algorithm has been adapted for floating point numbers
 * from the original datasheet.
 */

#include "MS5611.h"

namespace sensors::environment
{

async(MS5611::InitImpl, InitConfig config)
async_def(uint8_t u; uint16_t reg;)
{
    init = false;
    cfg = config;

    MYDBG("Initializing...");
    if (!await(ReadRegister, Command::Reset, Buffer())) { async_return(false); }

    // wait until the module recovers from reset - there is no signal for this, it just NAKs everything
    async_delay_ms(100);

    MYDBG("Reading calibration values");
    for (f.u = 0; f.u < countof(c); f.u++)
    {
        if (!await(ReadRegister, uint8_t(Command::ReadC1) + f.u * 2, Buffer(f.reg)))
        {
            MYDBG("Calibration value readout failed");
            async_return(false);
        }
        c[f.u] = FROM_BE16(f.reg);
    }

    MYDBG("Init complete");
    async_return(init = true);
}
async_end

async(MS5611::Measure)
async_def(
    unsigned i;
    uint32_t d[2];
)
{
    if (!init && !await(InitImpl, cfg))
    {
        async_return(false);
    }

    for (f.i = 0; f.i <= countof(f.d); f.i++)
    {
        if (!await(ReadRegister, f.i == 0 ? cfg.d1 : cfg.d2, Buffer()))
        {
            MYDBG("Error while triggering D%d", f.i);
            async_return(false);
        }
        async_delay_ms(10);
        if (!await(ReadRegister, Command::Read, Buffer(&f.d[f.i], 3)))
        {
            MYDBG("Error while reading D%d", f.i);
        }
    }

    float senst1 = c1 << 1,
          offt1 = c2 << 1,
          tcs = c3 << 2,
          tco = c4 << 2,
          tref = c5 * 0x1p-16f,
          tsens = c6 * 2e-2f;

    float d1 = FROM_BE24(f.d[0]) * 0x1p-22f;
    float d2 = FROM_BE24(f.d[1]) * 0x1p-24f;

    float dT = d2 - tref;
    temperature = 20 + dT * tsens;

    // second-order temperature correction
    // coefficients calculated by offsetting the original ones from the DS
    const float tCoef = 0x1p17 * 1e-2;
    const float offCoef1 = 5 * 0x1p-16 * 1e4;
    const float sensCoef1 = 5 * 0x1p-16 * 1e4;
    const float offCoef2 = 7 * 0x1p-15 * 1e4;
    const float sensCoef2 = 11 * 0x1p-15 * 1e4;

    float t2 = 0, off2 = 0, sens2 = 0;
    float tsub = temperature - 20;
    if (tsub < 0)
    {
        t2 = tCoef * (dT * dT);
        off2 = offCoef1 * (tsub * tsub);
        sens2 = sensCoef1 * (tsub * tsub);

        tsub = temperature + 15;
        if (tsub < 0)
        {
            off2 += offCoef2 * (tsub * tsub);
            sens2 += sensCoef2 * (tsub * tsub);
        }

        temperature -= t2;
    }

    float off = offt1 + tco * dT - off2;
    float sens = senst1 + tcs * dT - sens2;
    pressure = (d1 * sens - off) * 1e-2f;

    async_return(true);
}
async_end

}


