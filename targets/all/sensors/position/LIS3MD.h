/*
 * Copyright (c) 2019 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/position/LIS3MD.h
 */

#pragma once

#include <sensors/I2CSensor.h>

namespace sensors::position
{

class LIS3MD : I2CSensor
{
public:
    enum struct Address : uint8_t
    {
        Low = 0x1C,
        High = 0x1E,
    };

    enum struct Config : uint32_t
    {
        // values for CTRL_REG1
        Rate0p625Hz = 0 << 2,
        Rate1p25Hz = 1 << 2,
        Rate2p5Hz = 2 << 2,
        Rate5Hz = 3 << 2,
        Rate10Hz = 4 << 2,
        Rate20Hz = 5 << 2,
        Rate40Hz = 6 << 2,
        Rate80Hz = 7 << 2,

        PowerXYLow = 0 << 5,
        PowerXYMedium = 1 << 5,
        PowerXYHigh = 2 << 5,
        PowerXYUltra = 3 << 5,

        // values for CTRL_REG2
        Scale4gS = 0 << 5 << 8,
        Scale8gS = 1 << 5 << 8,
        Scale12gS = 2 << 5 << 8,
        Scale16gS = 3 << 5 << 8,

        // values for CTRL_REG4
        PowerZLow = 0 << 2 << 16,
        PowerZMedium = 1 << 2 << 16,
        PowerZHigh = 2 << 2 << 16,
        PowerZUltra = 3 << 2 << 16,

        // combined power values
        PowerLow = PowerXYLow | PowerZLow,
        PowerMedium = PowerXYMedium | PowerZMedium,
        PowerHigh = PowerXYHigh | PowerZHigh,
        PowerUltra = PowerXYUltra | PowerZUltra,
    };

    LIS3MD(bus::I2C& i2c, Address address)
        : I2CSensor(i2c, (uint8_t)address)
    {
    }

    //! Field intensity in X direction in gauss
    float GetFieldX() const { return x; }
    //! Field intensity in Y direction in gauss
    float GetFieldY() const { return y; }
    //! Field intensity in Z direction in gauss
    float GetFieldZ() const { return z; }

    //! Initializes the sensor
    async(Init);
    //! Updates sensor configuration
    async(Configure, Config cfg);
    //! Retrieves the last measurement result, return value indicates if the measured values have changed in the meantime
    async(Measure);

protected:
    const char* DebugComponent() const { return "LIS3MD"; }

private:
    enum struct Register : uint8_t
    {
        ID = 0xF,

        Control1 = 0x20,
        Control2 = 0x21,
        Control3 = 0x22,
        Control4 = 0x23,
        Control5 = 0x24,

        Status = 0x27,
        OutXL = 0x28, OutXH = 0x29,
        OutYL = 0x2A, OutYH = 0x2B,
        OutZL = 0x2C, OutZH = 0x2D,
    };

    enum struct IDValue : uint8_t
    {
        Valid = 0x3D,
    };

    enum struct Status : uint8_t
    {
        ReadyX = 1,
        ReadyY = 2,
        ReadyZ = 4,
        ReadyAll = ReadyX | ReadyY | ReadyZ,
        ReadyAny = 8,

        OverrunX = 0x10,
        OverrunY = 0x20,
        OverrunZ = 0x40,
        OverrunAll = OverrunX | OverrunY | OverrunZ,
        OverrunAny = 0x80,
    };

    enum struct Control1 : uint8_t
    {
    };

    enum struct Control2 : uint8_t
    {
        Reset = 4,
        Reboot = 8,

        Scale4gS = 0 << 5,
        Scale8gS = 1 << 5,
        Scale12gS = 2 << 5,
        Scale16gS = 3 << 5,
    };

    enum struct Control3 : uint8_t
    {
        ModeContinuous = 0,
        ModeSingle = 1,
        ModePowerDown = 2,
        ModeMask = 3,
    };

    enum struct Control4 : uint8_t
    {
    };

    DECLARE_FLAG_ENUM(Status);
    DECLARE_FLAG_ENUM(Control2);
    DECLARE_FLAG_ENUM(Control3);

    async(UpdateConfiguration);

    bool init = false;

    struct
    {
        union
        {
            struct
            {
                Control1 ctl1;
                Control2 ctl2;
                Control3 ctl3;
                Control4 ctl4;
            };
            uint32_t combined = FROM_BE32(0x10000300);  // initialize with default values of registers
        };

        float GetScale() const { return BYTES(4, 8, 12, 16)[(uint8_t(ctl2) >> 5) & 3]; }
        bool IsPowerDown() const { return !!(ctl3 & Control3::ModePowerDown); }
    } cfgActual, cfgDesired;

    float x = NAN, y = NAN, z = NAN;
    float mul;
};

DEFINE_FLAG_ENUM(LIS3MD::Config);
DEFINE_FLAG_ENUM(LIS3MD::Status);
DEFINE_FLAG_ENUM(LIS3MD::Control2);
DEFINE_FLAG_ENUM(LIS3MD::Control3);

}
