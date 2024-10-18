/*
 * Copyright (c) 2024 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/environment/MS5611.h
 *
 * Driver for MEAS/TE Connectivity MS5611 Barometric sensor
 */


#pragma once

#include <sensors/I2CSensor.h>

namespace sensors::environment
{

class MS5611 : I2CSensor
{
public:
    enum struct Address : uint8_t
    {
        Low = 0x76,
        High = 0x77,
    };

    enum OSR : uint8_t
    {
        Osr256 = 0,
        Osr512 = 2,
        Osr1024 = 4,
        Osr2048 = 6,
        Osr4096 = 8,
    };

    MS5611(bus::I2C i2c, Address address)
        : I2CSensor(i2c, (uint8_t)address)
    {
    }

    //! Initializes the sensor
    async(Init, OSR oversampling) { return async_forward(InitImpl, InitConfig(oversampling, oversampling)); }
    //! Initializes the sensor
    async(Init, OSR pressureOversampling, OSR temperatureOversampling) { return async_forward(InitImpl, InitConfig(pressureOversampling, temperatureOversampling)); }
    //! Performs a measurement cycle
    async(Measure);

    //! Checks if the sensor is initialized
    bool Initialized() const { return init; }
    //! Gets the last measured pressure in hPa; NaN if not available
    float GetPressure() const { return pressure; }
    //! Gets the last measured temperature in degrees celsius; NaN if not available
    float GetTemperature() const { return temperature; }

protected:
    const char* DebugComponent() const { return "MS5611"; }

private:
    enum struct Command : uint8_t
    {
        Reset = 0x1E,
        ConvertD1 = 0x40,
        ConvertD2 = 0x50,
        Read = 0x00,
        ReadC1 = 0xA2
    };

    struct InitConfig
    {
        constexpr InitConfig() {}
        constexpr InitConfig(OSR d1, OSR d2)
            : d1(uint8_t(Command::ConvertD1) | uint8_t(d1)), d2(uint8_t(Command::ConvertD2) | uint8_t(d2)) {}

        uint8_t d1 = 0, d2 = 0;
    };

    async(InitImpl, InitConfig cfg);
    async(Trigger);
    async(DataReady);
    async(WaitForData, Timeout timeout);

    bool init = false;
    InitConfig cfg;
    // calibration values
    union
    {
        uint16_t c[6];
        struct { uint16_t c1, c2, c3, c4, c5, c6; };
    };
    float pressure = NAN, temperature = NAN;
    float pressureI = NAN, temperatureI = NAN;

};

}
