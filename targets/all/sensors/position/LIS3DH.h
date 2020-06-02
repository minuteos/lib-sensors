/*
 * Copyright (c) 2020 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/position/LIS3DH.h
 */

#pragma once

#include <sensors/I2CSensor.h>

#include <sensors/types.h>

namespace sensors::position
{

class LIS3DH : I2CSensor
{
public:
    enum struct Address : uint8_t
    {
        Low = 0x18,
        High = 0x19,
    };

    PACKED_UNALIGNED_STRUCT Sample
    {
        short x, y, z;

        XYZ ToXYZ(float mul) const { return { x * mul, y * mul, z * mul }; }
    };

    LIS3DH(bus::I2C i2c, Address address)
        : I2CSensor(i2c, (uint8_t)address)
    {
    }

    enum Rate
    {
        RateOneShot = 0,
        Rate1Hz = 0x10,
        Rate10Hz = 0x20,
        Rate25Hz = 0x30,
        Rate50Hz = 0x40,
        Rate100Hz = 0x50,
        Rate200Hz = 0x60,
        Rate400Hz = 0x70,
        RateLP1600Hz = 0x80,
        RateMaximum = 0x90,
    };

    enum Scale
    {
        Scale2g = 0,
        Scale4g = 0x10,
        Scale8g = 0x20,
        Scale16g = 0x30,
    };

    enum Resolution
    {
        Resolution8bit = 0,
        Resolution10bit = 1,
        Resolution12bit = 2,
    };

    //! Initializes the sensor
    async(Init, Rate rate, Scale scale, Resolution res = Resolution10bit)
    {
        return async_forward(InitImpl, InitConfig(
            Control1(rate) | Control1::DirectionAll | (res == Resolution8bit) * Control1::LowPower,
            Control4(scale) | (res == Resolution12bit) * Control4::HighResolution,
            (res != Resolution12bit) * Control5::FifoEnable,
            FifoControl::ModeStream));
    }

    //! Retrieves the last measurement result, return value indicates if the measured values have changed in the meantime
    async(Measure);
    //! Retrieves fifo contents
    template<size_t n> async(ReadFifo, Sample (&buffer)[n]) { return async_forward(ReadFifo, buffer, n); }
    //! Retrieves fifo contents
    async(ReadFifo, Sample* buffer, size_t count);

    //! Gets the last measured acceleration values
    XYZ GetAccelerationXYZ() const { return xyz; }
    //! Gets the multiplier used to convert raw values
    float GetRawMultiplier() const { return mul; }
    //! Converts a raw sample to standard acceleration values
    XYZ SampleToXYZ(const Sample& smp) const { return smp.ToXYZ(mul); }

protected:
    const char* DebugComponent() const { return "LIS3DH"; }

private:
    enum struct Register : uint8_t
    {
        ID = 0x0F,
        Control1 = 0x20,
        Control4 = 0x23,
        Control5 = 0x24,
        FifoControl = 0x2E,
        FifoStatus = 0x2F,

        Data = 0xA8,    // read with auto-increment
    };

    enum struct IDValue : uint8_t
    {
        Valid = 0x33,
    };

    enum struct Control1 : uint8_t
    {
        DirectionX = 0x01,
        DirectionY = 0x02,
        DirectionZ = 0x04,
        DirectionAll = 0x07,

        LowPower = 0x08,

        RateOneShot = 0,
        Rate1Hz = 0x10,
        Rate10Hz = 0x20,
        Rate25Hz = 0x30,
        Rate50Hz = 0x40,
        Rate100Hz = 0x50,
        Rate200Hz = 0x60,
        Rate400Hz = 0x70,
        RateLP1600Hz = 0x80,
        RateMaximum = 0x90,
    };

    enum struct Control4 : uint8_t
    {
        HighResolution = 0x08,

        Scale2g = 0,
        Scale4g = 0x10,
        Scale8g = 0x20,
        Scale16g = 0x30,

        _ScaleMask = 0x30,
        _ScaleOffset = 4,
    };

    enum struct Control5 : uint8_t
    {
        FifoEnable = 0x40,

        Reset = 0x80,
    };

    enum struct FifoControl : uint8_t
    {
        ModeStream = 0x80,
    };

    DECLARE_FLAG_ENUM(Control1);
    DECLARE_FLAG_ENUM(Control4);
    DECLARE_FLAG_ENUM(Control5);

    struct FifoStatus
    {
        uint8_t count : 5;
        bool empty : 1;
        bool overrun : 1;
        bool watermark : 1;
    };

    union InitConfig
    {
        constexpr InitConfig()
            : value(0) {}
        constexpr InitConfig(Control1 ctl1, Control4 ctl4, Control5 ctl5, FifoControl fifo)
            : ctl1(ctl1), ctl4(ctl4), ctl5(ctl5), fifo(fifo) {}

        uint32_t value;
        struct
        {
            Control1 ctl1;
            Control4 ctl4;
            Control5 ctl5;
            FifoControl fifo;
        };
    };

    async(InitImpl, InitConfig cfg);

    bool init = false;
    InitConfig cfg;
    float mul = NAN;
    XYZ xyz = { NAN, NAN, NAN };
};

DEFINE_FLAG_ENUM(LIS3DH::Control1);
DEFINE_FLAG_ENUM(LIS3DH::Control4);
DEFINE_FLAG_ENUM(LIS3DH::Control5);

}
