/*
 * Copyright (c) 2019 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/environment/SHTC3.h
 */

#pragma once

#include <kernel/kernel.h>

#include <bus/I2C.h>

namespace sensors::environment
{

//! Driver for Sensirion SHTC3 Humidity/Temperature sensor
class SHTC3
{
public:
    enum
    {
        Address = 0x70,
    };

    SHTC3(bus::I2C& i2c, bool lowPower = false)
        : i2c(i2c)
    {
    }

    //! Initialize the sensor
    async(Init);
    //! Ask the sensor to perform a measurement
    async(Measure);

    //! Indicates if the low power measurement mode is enabled
    bool LowPower() const { return lowPower; }
    //! Enables or disables the low power measurement mode
    void LowPower(bool value) { this->lowPower = lowPower; }

    //! Gets the last measured temperature. NaN when there is no measurement available.
    float GetTemperature() const { return temp; }
    //! Gets the last measured relative humidity. NaN when there is no measurement available.
    float GetHumidity() const { return hum; }

private:
    bus::I2C& i2c;
    float temp = NAN, hum = NAN;
    bool init = false;
    bool lowPower = false;

    enum struct Command
    {
        Reset = 0x805D,
        ReadID = 0xEFC8,
        Sleep = 0xB098,
        Wake = 0x3517,

        Measure = 0x7CA2,
        MeasureLowPower = 0x6458,
    };

	async(WriteCommand, Command cmd, bool stop = true);
};

}
