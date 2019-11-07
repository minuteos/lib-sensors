/*
 * Copyright (c) 2019 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/environment/LPS22HB.h
 *
 * Driver for STMicroelectronics LPS22HB Barometer
 */

#pragma once

#include <sensors/I2CSensor.h>

namespace sensors::environment
{

class LPS22HB : I2CSensor
{
public:
    enum struct Address : uint8_t
    {
        Low = 0x5C,
        High = 0x5D,
    };

    LPS22HB(bus::I2C& i2c, Address address = Address::Low)
        : I2CSensor(i2c, (uint8_t)address)
    {
    }

    //! Initializes the sensor
    async(Init);
    //! Retrieves the last measurement result, return value indicates if the measured values have changed in the meantime
    //! If current rate is @ref Control2::RateOneShot, a measurement is triggered and result is retrieved
    async(Measure);

    //! Gets the last measured pressure in hPa; NaN if not available
    float GetPressure() const { return pressure; }
    //! Gets the last measured temperature in degrees celsius; NaN if not available
    float GetTemperature() const { return temperature; }

protected:
    const char* DebugComponent() const { return "LPS22HB"; }

private:
    enum struct Register : uint8_t
    {
        ID = 0x0F,
        Control1 = 0x10,
        Control2 = 0x11,
        FifoControl = 0x14,
        Resolution = 0x1A,
        FifoStatus = 0x26,
        Status = 0x27,
        Data = 0x28,
    };

    enum struct IDValue : uint8_t
    {
        Valid = 0xB1,
    };

    enum struct Control1 : uint8_t
    {
        SPI4Write = 0,
        SPI3Wire = 1,

        ContinuousDataUpdate = 0,
        BlockDataUpdate = 2,

        LowPassFilterOff = 0,
        LowPassFilterWeak = 0x8,
        LowPassFilterStrong = 0xC,

        RateOneShot = 0,
        Rate1Hz = 0x10,
        Rate10Hz = 0x20,
        Rate25Hz = 0x30,
        Rate50Hz = 0x40,
        Rate75Hz = 0x50,
        RateMask = 0x70,
    };

    enum struct Control2 : uint8_t
    {
        Trigger = 1,
        Reset = 4,
        I2CDisable = 8,
        AutoAddrIncrement = 0x10,
        FifoWatermark = 0x20,
        FifoEnable = 0x40,
        MemReset = 0x80,
    };

    enum struct FifoControl : uint8_t
    {
        WatermarkMask = 0x1F,

        ModeBypass = 0 << 5,
        ModeFifo = 1 << 5,
        ModeStream = 2 << 5,
        ModeStreamToFifo = 3 << 5,
        ModeBypassToStream = 4 << 5,
        ModeDynamicStream = 6 << 5,
        ModeBypassToFifo = 7 << 5,
    };

    enum struct Status : uint8_t
    {
        PressureAvailable = 1,
        TemperatureAvailable = 2,
        PressureOverrun = 0x10,
        TemperatureOverrun = 0x20,
    };

    DECLARE_FLAG_ENUM(LPS22HB::Control1);
    DECLARE_FLAG_ENUM(LPS22HB::Control2);
    DECLARE_FLAG_ENUM(LPS22HB::Status);

    Control1 Rate() const { return ctl1 & Control1::RateMask; }

    async(Trigger) { return async_forward(WriteRegister, Register::Control2, (uint8_t)(ctl2 | Control2::Trigger)); }
    async(DataReady);
    async(WaitForDataTicks, mono_t ticksTimeout);
    async(WaitForDataMs, mono_t msTimeout) { return async_forward(WaitForDataTicks, MonoFromMilliseconds(msTimeout)); }

    bool init = false;
    Control1 ctl1 = Control1::SPI4Write | Control1::ContinuousDataUpdate | Control1::LowPassFilterOff | Control1::RateOneShot;
    Control2 ctl2 = Control2::AutoAddrIncrement;
    float pressure = NAN, temperature = NAN;
};

DEFINE_FLAG_ENUM(LPS22HB::Control1);
DEFINE_FLAG_ENUM(LPS22HB::Control2);
DEFINE_FLAG_ENUM(LPS22HB::Status);

}
