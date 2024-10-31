/*
 * Copyright (c) 2020 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/environment/MCP9600.cpp
 */

#include "MCP9600.h"

namespace sensors::environment
{

async(MCP9600::Init, SensorConfig sensorConfig, DeviceConfig deviceConfig)
async_def(
    struct
    {
        uint8_t id;
        uint8_t min : 4;
        uint8_t maj : 4;
    } info;
    union {
        uint8_t status;
        SensorConfig sensor;
        DeviceConfig device;
    } config;
)
{
    if (!init)
    {
#if DEBUG
        if (OutputFrequency() > 80000)
        {
            MYDBG("I2C frequency %d too high for MCP9600. Must be <= 80000", OutputFrequency());
            ASSERT(false);
        }
#endif

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

    if (!init ||
        config.sensor != sensorConfig ||
        config.device != deviceConfig ||
        (deviceConfig & DeviceConfig::_ModeMask) == DeviceConfig::ModeBurst)
    {
        // always fully reinitialize the sensor on change
        init = false;

        // shutdown first
        f.config.device = (deviceConfig & ~DeviceConfig::_ModeMask) | DeviceConfig::ModeShutdown;
        if (!await(WriteRegister, Register::DeviceConfig, f.config.sensor))
        {
            async_return(false);
        }

        // clear status
        f.config.status = 0;
        if (!await(WriteRegister, Register::Status, f.config.status))
        {
            async_return(false);
        }

        // configure sensor
        f.config.sensor = sensorConfig;
        if (!await(WriteRegister, Register::SensorConfig, f.config.sensor))
        {
            async_return(false);
        }

        // configure power mode
        if ((deviceConfig & DeviceConfig::_ModeMask) != DeviceConfig::ModeShutdown)
        {
            f.config.device = deviceConfig;
            if (!await(WriteRegister, Register::DeviceConfig, f.config.device))
            {
                async_return(false);
            }
        }

        config.sensor = sensorConfig;
        config.device = (deviceConfig & DeviceConfig::_ModeMask) == DeviceConfig::ModeBurst ?
            deviceConfig ^ (DeviceConfig::ModeBurst ^ DeviceConfig::ModeShutdown) : // replace ModeBurst with ModeShutdown for stored configuration
            deviceConfig;
    }

    async_return(init = true);
}
async_end

async(MCP9600::Trigger)
async_def()
{
    // force init to burst mode
    if (!await(Init, config.sensor, (config.device & ~DeviceConfig::_ModeMask) | DeviceConfig::ModeBurst))
    {
        async_return(false);
    }

    MYTRACE("TRIGGER");
    async_return(true);
}
async_end

async(MCP9600::Measure, Timeout timeout)
async_def(
    PACKED_UNALIGNED_STRUCT
    {
        uint16_t tHot, tDelta, tCold;
        uint32_t adc : 24;
    }
    data;
    PACKED_UNALIGNED_STRUCT
    {
        union
        {
            uint8_t raw;
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
    } status;
    Timeout timeout;
#if TRACE && SENSOR_TRACE
    int retry;
#endif
)
{
    f.timeout = timeout.MakeAbsolute();

    if (!init && !await(Init, config.sensor, config.device))
    {
        async_return(false);
    }
again:
    // check if data available and if sensor hasn't been reset
    if (!await(ReadRegister, Register::Status, f.status))
    {
        init = false;
        async_return(false);
    }

    if (f.status.scfg != config.sensor)
    {
        MYDBG("Sensor config reset, expected %02X, found %02X", config.sensor, f.status.scfg);
        init = false;
        async_return(false);
    }

    if (f.status.dcfg != config.device)
    {
        if (!!((f.status.dcfg ^ config.device) & ~DeviceConfig::_ModeMask))
        {
            MYDBG("Device config reset, expected %02X, found %02X", config.device, f.status.dcfg);
            init = false;
            async_return(false);
        }

        goto retry;
    }

    if (!(f.status.update || f.status.complete))
    {
        // conversion pending
        goto retry;
    }

    // reset status
    await(WriteRegister, Register::Status, BYTES(0));

    if (f.status.range)
    {
        tempHot = tempCold = NAN;
        raw = 0;
        MYTRACE("out of range");
        async_return(true);
    }

    // read current values
    if (!await(ReadRegister, Register::HotJunction, f.data))
    {
        init = false;
        async_return(false);
    }

    tempHot = int16_t(FROM_BE16(f.data.tHot)) * TEMP_MUL;
    tempCold = int16_t(FROM_BE16(f.data.tCold)) * TEMP_MUL;
    raw = int32_t(FROM_BE32(f.data.adc)) >> 8;
    MYTRACE("new data: Thot=%.3q, Tcold=%.3q, ADC=%d", int(tempHot * 1000), int(tempCold * 1000), raw);
    async_return(true);

retry:
    auto t = f.timeout.Relative();
    if (t <= 0)
    {
        async_return(false);
    }
#if TRACE && SENSOR_TRACE
    MYTRACE("retry %d (%d)", ++f.retry, t);
#endif
    async_delay_ticks(std::min(mono_signed_t(MonoFromMilliseconds(10)), t));
    goto again;
}
async_end

}
