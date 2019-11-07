/*
 * Copyright (c) 2019 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/position/TLE493D.h
 */

#pragma once

#include <sensors/I2CSensor.h>

namespace sensors::position
{

class TLE493D : I2CSensor
{
public:
    enum struct Address : uint8_t
    {
        A0 = 0x35,  // 0b011 0101
        A1 = 0x22,  // 0b010 0010
        A2 = 0x78,  // 0b111 1000
        A3 = 0x44,  // 0b100 0100
    };

    TLE493D(bus::I2C& i2c, Address address)
        : I2CSensor(i2c, (uint8_t)address)
    {
    }

    //! Field intensity in X direction in mT
    float GetFieldX() const { return x; }
    //! Field intensity in Y direction in mT
    float GetFieldY() const { return y; }
    //! Field intensity in Z direction in mT
    float GetFieldZ() const { return z; }

    //! Initializes the sensor
    async(Init);
    //! Retrieves the last measurement result, return value indicates if the measured values have changed in the meantime
    async(Measure);

protected:
    const char* DebugComponent() const { return "TLE493D"; }

private:
    enum struct Register : uint8_t
    {
        Bx = 0, By = 1, Bz = 2,
        Temp = 3, Bx2 = 4, Temp2 = 5,
        Diagnostics = 6,

        WakeXLow = 7,
        WakeXHigh = 8,
        WakeYLow = 9,
        WakeYHigh = 0xA,
        WakeZLow = 0xB,
        WakeZHigh = 0xC,

        WakeXLSB = 0xD,
        WakeYLSB = 0xE,
        WakeZLSB = 0xF,

        Config = 0x10,
        Mode1 = 0x11,
        Mode2 = 0x13,
        Version = 0x16,
    };

    enum struct Diagnostics : uint8_t
    {
        FrameMask = 0x03,
        BxDone = 0x04,
        TempDone = 0x08,
        TestMode = 0x10,
        ConfigParity = 0x20,
        FuseParity = 0x40,
        BusParity = 0x80,
    };

    enum struct Config : uint8_t
    {
        Parity = 1,

        TemperatureCompensationOff = 0,
        TemperatureCompensation1 = 1 << 1,
        TemperatureCompensation2 = 2 << 1,
        TemperatureCompensation3 = 3 << 1,
        ShortRange = 8,

        ReadTriggerOff = 0,
        ReadTriggerBefore = 0x10,
        ReadTriggerAfter = 0x20,

        DisableZ = 0x40,
        DisableTemperature = 0x80,
    };

    enum struct Mode1 : uint8_t
    {
        PowerModeAuto = 0,
        PowerModeTrigger = 1,
        PowerModeContinuous = 3,

        InterruptDisable = 4,
        CollisionAvoidanceDisable = 8,

        Protocol1Byte = 0x10,

        Address0 = 0 << 5,
        Address1 = 1 << 5,
        Address2 = 2 << 5,
        Address3 = 3 << 5,

        FuseParity = 0x80,
    };

    DECLARE_FLAG_ENUM(Diagnostics);
    DECLARE_FLAG_ENUM(Config);
    DECLARE_FLAG_ENUM(Mode1);

    //! Returns the configred address in Mode1 register format
    Mode1 Mode1Address() const { return (Mode1::Address2 * GETBIT(BusAddress(), 6)) | (Mode1::Address1 * !GETBIT(BusAddress(), 4)); }

    bool init = false;
    float x = NAN, y = NAN, z = NAN;

    static constexpr float ValueMultiply = 200 / 2048.0;
};

DEFINE_FLAG_ENUM(TLE493D::Diagnostics);
DEFINE_FLAG_ENUM(TLE493D::Config);
DEFINE_FLAG_ENUM(TLE493D::Mode1);

}
