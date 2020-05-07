/*
 * Copyright (c) 2020 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/environment/MCP9600.cpp
 */

#include "MCP9600.h"

//#define MCP9600_TRACE   1

#if MCP9600_TRACE
#define MYTRACE(...) MYDBG(__VA_ARGS__)
#else
#define MYTRACE(...)
#endif

namespace sensors::environment
{

async(MCP9600::Init, SensorConfig sensorConfig, DeviceConfig deviceConfig)
async_def(
    struct
    {
        uint8_t id;
        uint8_t maj : 4;
        uint8_t min : 4;
    } info;
    union {
        SensorConfig sensor;
        DeviceConfig device;
    } config;
)
{
    if (!init)
    {
        MYDBG("Reading ID...");

        if (!await(ReadRegister, Register::DeviceID, f.info))
        {
            async_return(false);
        }

        if (f.info.id != ValidID)
        {
            MYDBG("Invalid ID: %02X != %02X", f.info.id, ValidID);
            async_return(false);
        }

        MYDBG("Init complete, ID: %02X rev %d.%d", f.info.id, f.info.maj, f.info.min);
    }

    if (!init || config.sensor != sensorConfig)
    {
        f.config.sensor = sensorConfig;
        if (!await(WriteRegister, Register::SensorConfig, f.config.sensor))
        {
            async_return(false);
        }
        config.sensor = f.config.sensor;
    }

    if (!init || config.device != deviceConfig || (deviceConfig & DeviceConfig::_ModeMask) == DeviceConfig::ModeBurst)
    {
        f.config.device = deviceConfig;
        if (!await(WriteRegister, Register::DeviceConfig, f.config.device))
        {
            async_return(false);
        }
        config.device = f.config.device;
    }

    async_return(init = true);
}
async_end

async(MCP9600::Measure)
async_def(
    PACKED_UNALIGNED_STRUCT
    {
        uint16_t tHot, tDelta, tCold;
        uint32_t adc : 24;
        union
        {
            uint8_t status;
            struct
            {
                uint8_t alert : 4;
                uint8_t range : 1;
                uint8_t : 1;
                uint8_t update : 1;
                uint8_t complete : 1;
            };
        };
        SensorConfig scfg;
        DeviceConfig dcfg;
    } data, data2;
    bool good;
    int errors;
)
{
    if (!init && !await(Init, config.sensor, config.device))
    {
        async_return(false);
    }

    while (f.errors < MaxErrors && !await(ReadRegister, Register::HotJunction, f.data))
    {
         f.errors++;
    }

    while (f.errors < MaxErrors && !f.data.complete)
    {
        if (await(ReadRegister, Register::HotJunction, f.data2))
        {
            if (Span(f.data) == Span(f.data2))
            {
                break;
            }

            MYDBG("\n%H\n%H", Span(f.data), Span(f.data2));
            f.data = f.data2;
        }
        f.errors++;
    }

    if (f.errors >= MaxErrors)
    {
        MYDBG("Failed to verify read data, reinint required");
        init = false;
        async_return(false);
    }

    if (f.errors)
    {
        MYTRACE("%d errors before successful data read", f.errors);
    }

    if (f.data.update || f.data.complete)
    {
        await(WriteRegister, Register::Status, BYTES(0));
        tempHot = int16_t(FROM_BE16(f.data.tHot)) * TEMP_MUL;
        tempCold = int16_t(FROM_BE16(f.data.tCold)) * TEMP_MUL;
        raw = int32_t(FROM_BE32(f.data.adc)) >> 8;
        MYTRACE("new data: Thot=%.3q, Tcold=%.3q, ADC=%d", int(tempHot * 1000), int(tempCold * 1000), raw);
    }

    if (f.data.scfg != config.sensor)
    {
        MYDBG("Sensor config reset, expected %02X, found %02X", config.sensor, f.data.scfg);
        await(WriteRegister, Register::SensorConfig, config.sensor);
    }

    if (f.data.dcfg != config.device)
    {
        if (!!((f.data.dcfg ^ config.device) & ~DeviceConfig::_ModeMask))
        {
            MYDBG("Device config reset, expected %02X, found %02X", config.device, f.data.dcfg);
        }
        await(WriteRegister, Register::DeviceConfig, config.device);
    }

    async_return(f.data.update);
}
async_end

}
