/*
 * Copyright (c) 2020 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/analog/HX71x.h
 */

#pragma once

#include <kernel/kernel.h>

#include <hw/GPIO.h>

namespace sensors::analog
{

class HX71x
{
public:
    enum MeasurementType
    {
        NoChange = 0,
        Default = 1,

        HX710_ExternalX128 = 1,
        HX710_InternalX128 = 2,
        HX710A_Temperature = 2,
        HX710B_VoltageDifference = 2,
        HX710_ExternalX128Fast = 3,

        HX711_ChannelAX128 = 1,
        HX711_ChannelBX32 = 2,
        HX711_ChannelAX64 = 3,

        HX712_ExternalX128 = 1,
        HX712_InternalX128 = 2,
        HX712_ExternalX128Fast = 3,
        HX712_ExternalX256 = 4,
        HX712_ExternalX256Fast = 5,
    };

    HX71x(GPIOPin sck, GPIOPin dout, GPIOPin refEna = Px)
        : sck(sck), dout(dout), refEna(refEna) {}

    //! Initializes the sensor, configures the specified measurement type and powers the sensor down
    async(Init, MeasurementType type = MeasurementType::Default);
    //! Reads out a single measurement of the currently initialized type, the
    async(Measure, MeasurementType type = MeasurementType::NoChange);
    //! Powers down the device
    async(PowerDown);

    //! Indicates if the device is active (not in sleep mode)
    bool Active() const { return !sck; }
    //! Retrieves the last measured value
    float GetValue() const { return value; }
    //! Gets the currently configured measurement type
    MeasurementType GetType() const { return type; }

private:
    GPIOPin sck, dout, refEna;
    MeasurementType type;
    mono_t powerDownAt;
    float value = NAN;
};

}
