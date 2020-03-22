/*
 * Copyright (c) 2020 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/environment/MCP9600.h
 *
 * Driver for Microchip MCP96x00 Thermocouple interface
 */

#pragma once

#include <sensors/I2CSensor.h>

namespace sensors::environment
{

class MCP9600 : I2CSensor
{
public:
    MCP9600(bus::I2C& i2c, uint8_t addr = 0)
        : I2CSensor(i2c, AddressBase | addr)
    {
    }

    enum struct SensorConfig : uint8_t
    {
        FilterOff = 0,
        Filter1 = 1,
        Filter2 = 2,
        Filter3 = 3,
        Filter4 = 4,
        Filter5 = 5,
        Filter6 = 6,
        Filter7 = 7,
        FilterMin = 1,
        FilterMax = 7,

        TypeK = 0 << 4,
        TypeJ = 1 << 4,
        TypeT = 2 << 4,
        TypeN = 3 << 4,
        TypeS = 4 << 4,
        TypeE = 5 << 4,
        TypeB = 6 << 4,
        TypeR = 7 << 4,

        _FilterMask = 7,
        _TypeMask = 3 << 4,

        _Default = FilterOff | TypeK,
    };

    enum struct DeviceConfig : uint8_t
    {
        ModeNormal = 0,
        ModeShutdown = 1,
        ModeBurst = 2,

        AdcRes18 = 0,
        AdcRes16 = 1 << 5,
        AdcRes14 = 2 << 5,
        AdcRes12 = 3 << 5,

        ColdResHigh = 0,
        ColdResLow = BIT(7),

        _ModeMask = 3,
        _AdcMask = 3 << 5,

        _Default = ModeBurst | AdcRes14 | ColdResLow,
    };

    DECLARE_FLAG_ENUM(MCP9600::SensorConfig);
    DECLARE_FLAG_ENUM(MCP9600::DeviceConfig);

    //! Initializes the sensor
    async(Init, SensorConfig sensorConfig = SensorConfig::_Default, DeviceConfig deviceConfig = DeviceConfig::_Default);
    //! Retrieves the last measurement result, return value indicates if the measured values have changed in the meantime
    async(Measure);

    //! Gets the last measured cold junction temperature in degrees celsius; NaN if not available
    float GetColdTemperature() const { return tempCold; }
    //! Gets the last measured temperature in degrees celsius; NaN if not available
    float GetTemperature() const { return tempHot; }

protected:
    const char* DebugComponent() const { return "MCP9600"; }

private:
    enum
    {
        AddressBase = 0x60,
        ValidID = 0x40,
        MaxErrors = 10,
    };

    enum struct Register
    {
        HotJunction = 0,
        JunctionDelta = 1,
        ColdJunction = 2,
        RawADC = 3,
        Status = 4,
        SensorConfig = 5,
        DeviceConfig = 6,
        DeviceID = 32,
    };

    static constexpr float TEMP_MUL = 1.0f / 16;

    bool init = false;
    struct
    {
        SensorConfig sensor = SensorConfig::_Default;
        DeviceConfig device = DeviceConfig::_Default;
    } config;
    float tempCold, tempHot;
    int32_t raw;
};

DEFINE_FLAG_ENUM(MCP9600::SensorConfig);
DEFINE_FLAG_ENUM(MCP9600::DeviceConfig);

}
