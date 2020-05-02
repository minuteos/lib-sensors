/*
 * Copyright (c) 2019 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/position/MMA845x.h
 *
 * Driver for the MMA845x-series accelerometers
 */

#pragma once

#include <sensors/I2CSensor.h>

namespace sensors::position
{

class MMA845x : I2CSensor
{
public:
    enum struct Address : uint8_t
    {
        Low = 0x1C,
        High = 0x1D,
    };

    enum struct Config : uint32_t
    {
        // values for CTRL_REG1
        LowNoise = 4,

        Rate800Hz = 0 << 3,
        Rate400Hz = 1 << 3,
        Rate200Hz = 2 << 3,
        Rate100Hz = 3 << 3,
        Rate50Hz = 4 << 3,
        Rate12p5Hz = 5 << 3,
        Rate6p25Hz = 6 << 3,
        Rate1p56Hz = 7 << 3,

        RateFastest = Rate800Hz,
        RateSlowest = Rate1p56Hz,

        // values for CTRL_REG2
        ModeNormal = 0 << 8,
        ModeLowNoiseLowPower = 1 << 8,
        ModeHighResolution = 2 << 8,
        ModeLowPower = 3 << 8,

        // values for XYZ_DATA_CG
        Scale2g = 0 << 16,
        Scale4g = 1 << 16,
        Scale8g = 2 << 16,
    };

    MMA845x(bus::I2C i2c, Address address)
        : I2CSensor(i2c, (uint8_t)address)
    {
    }

    //! Acceleration in X direction as a multiply of g (standard gravity)
    float GetAccelerationX() const { return x; }
    //! Acceleration in Y direction as a multiply of g (standard gravity)
    float GetAccelerationY() const { return y; }
    //! Acceleration in Z direction as a multiply of g (standard gravity)
    float GetAccelerationZ() const { return z; }
    //! Get full-scale range in g (standard gravity)
    float GetScale() const { return cfgDesired.GetScale(); }
    //! Checks if the measurements are running
    bool IsActive() const { return cfgActual.IsActive(); }

    //! Initializes the sensor
    async(Init);
    //! Updates sensor configuration
    async(Configure, Config cfg);
    //! Starts measuring
    async(Start);
    //! Stops measuring
    async(Stop);
    //! Retrieves the last measurement result, return value indicates if the measured values have changed in the meantime
    async(Measure);

protected:
    const char* DebugComponent() const { return "MMA845x"; }

private:
    enum struct Register : uint8_t
    {
        Status = 0,
        OutXH = 1, OutXL = 2,
        OutYH = 3, OutYL = 4,
        OutZH = 5, OutZL = 6,

        SysMode = 0x0B,
        IntSource = 0x0C,
        ID = 0x0D,
        DataConfig = 0x0E,
        HighPassCutoff = 0x0F,

        Control1 = 0x2A,
        Control2 = 0x2B,
        Control3 = 0x2C,
        Control4 = 0x2D,
        Control5 = 0x2E,

        OffsetX = 0x2F,
        OffsetY = 0x30,
        OffsetZ = 0x31,
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

    enum struct IDValue : uint8_t
    {
        MMA8451 = 0x1A,
        MMA8452 = 0x2A,
        MMA8453 = 0x3A,
    };

    enum struct DataConfig : uint8_t
    {
        _Default = 0,
    };

    enum struct Control1 : uint8_t
    {
        _Default = 0,

        Active = 1,
        FastRead = 2,
        LowNoise = 4,

        Rate800Hz = 0 << 3,
        Rate400Hz = 1 << 3,
        Rate200Hz = 2 << 3,
        Rate100Hz = 3 << 3,
        Rate50Hz = 4 << 3,
        Rate12p5Hz = 5 << 3,
        Rate6p25Hz = 6 << 3,
        Rate1p56Hz = 7 << 3,
    };

    enum struct Control2 : uint8_t
    {
        _Default = 0,

        ModeNormal = 0,
        ModeLowNoiseLowPower = 1,
        ModeHighResolution = 2,
        ModeLowPower = 3,

        Reset = 1 << 6,
        SelfTest = 1 << 7,
    };

    DECLARE_FLAG_ENUM(Status);
    DECLARE_FLAG_ENUM(Control1);
    DECLARE_FLAG_ENUM(Control2);

    async(UpdateConfiguration);

    bool init = false;
    IDValue id;

    struct
    {
        DataConfig dcfg = DataConfig::_Default;
        struct
        {
            Control1 reg1 = Control1::_Default;
            Control2 reg2 = Control2::_Default;
        } ctl;

        float GetScale() const { return BYTES(2, 4, 8, 12)[uint8_t(dcfg) & 3]; }
        bool IsActive() const { return !!(ctl.reg1 & Control1::Active); }

        //! Gets value for comparing actual vs. desired
        uint32_t CompareValue() const { return (uint8_t)dcfg | ((uint8_t)(ctl.reg1 & ~Control1::Active) << 8) | ((uint8_t)ctl.reg2 << 16); }
    } cfgActual, cfgDesired;

    float x = NAN, y = NAN, z = NAN;
    float mul;
};

DEFINE_FLAG_ENUM(MMA845x::Config);
DEFINE_FLAG_ENUM(MMA845x::Status);
DEFINE_FLAG_ENUM(MMA845x::Control1);
DEFINE_FLAG_ENUM(MMA845x::Control2);

}
