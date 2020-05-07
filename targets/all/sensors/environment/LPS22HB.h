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

    PACKED_UNALIGNED_STRUCT Sample
    {
        uint32_t pressureLE : 24;
        uint16_t tempLE;

        float Pressure() const { return FROM_LE24(pressureLE) * (1.0f / 4096); }
        float Temperature() const { return FROM_LE16(tempLE) * 0.01f; }
    };

    LPS22HB(bus::I2C i2c, Address address)
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
        Rate75Hz = 0x50,
    };

    enum Filter
    {
        FilterOff = 0,
        FilterWeak = 0x8,
        FilterStrong = 0xC,
    };

    //! Initializes the sensor
    async(Init, Rate rate = RateOneShot, Filter filter = FilterOff) { return async_forward(InitImpl, InitConfig(rate | filter, Control2::FifoEnable | Control2::AutoAddrIncrement, FifoControl::ModeDynamicStream)); }
    //! Retrieves the last measurement result, return value indicates if the measured values have changed in the meantime
    //! If current rate is @ref Control2::RateOneShot, a measurement is triggered and result is retrieved
    async(Measure);
    //! Retrieves fifo contents
    template<size_t n> async(ReadFifo, Sample (&buffer)[n]) { return async_forward(ReadFifo, buffer, n); }
    //! Retrieves fifo contents
    async(ReadFifo, Sample* buffer, size_t count);

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
        SPI4Wire = 0,
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

        _Default = 0,
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

        _Default = AutoAddrIncrement,
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

    struct FifoStatus
    {
        uint8_t count : 6;
        bool overrun : 1;
        bool watermark : 1;
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

    Control1 Rate() const { return cfg.ctl1 & Control1::RateMask; }

    union InitConfig
    {
        constexpr InitConfig()
            : value(0) {}
        constexpr InitConfig(int rateAndMode, Control2 ctl2, FifoControl fifo)
            : ctl1(Control1(rateAndMode)), ctl2(ctl2), fifo(fifo) {}

        uint32_t value;
        struct
        {
            Control1 ctl1;
            Control2 ctl2;
            FifoControl fifo;
        };
    };

    async(InitImpl, InitConfig cfg);
    async(Trigger);
    async(DataReady);
    async(WaitForData, mono_t timeout);

    bool init = false;
    InitConfig cfg;
    float pressure = NAN, temperature = NAN;
};

DEFINE_FLAG_ENUM(LPS22HB::Control1);
DEFINE_FLAG_ENUM(LPS22HB::Control2);
DEFINE_FLAG_ENUM(LPS22HB::Status);

}
