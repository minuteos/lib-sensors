/*
 * Copyright (c) 2020 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/analog/FDC1004.h
 */

#pragma once

#include <sensors/I2CSensor.h>

namespace sensors::analog
{

class FDC1004 : I2CSensor
{
public:
    FDC1004(bus::I2C i2c)
        : I2CSensor(i2c, Address)
    {
    }

    enum Input
    {
        CIN1, CIN2, CIN3, CIN4,
    };

    enum Rate
    {
        Rate100Sps = 1 << 10,
        Rate200Sps = 2 << 10,
        Rate400Sps = 3 << 10,
    };

    //! Initializes the sensor
    async(Init);
    //! Configures one single ended measurement
    async(Configure, Input pos, float capdacOffset = 0) { return async_forward(Configure, 0, pos, capdacOffset); }
    //! Configures one differential measurement
    async(Configure, Input pos, Input neg ) { return async_forward(Configure, 0, pos, neg); }
    //! Configures the specified single ended measurement
    async(Configure, unsigned channel, Input pos, float capdacOffset = 0) { return async_forward(Configure, channel, pos << ChannelConfigPositiveOffset | ChannelConfigNegativeDisabled | ConvertCAPDAC(capdacOffset)); }
    //! Configures the specified differential measurement
    async(Configure, unsigned channel, Input pos, Input neg) { return async_forward(Configure, channel, pos << ChannelConfigPositiveOffset | neg << ChannelConfigNegativeOffset); }
    //! Śets the calibration values for the specified input
    async(SetCalibration, Input input, float offset = 0, float gain = 1) { return async_forward(SetCalibration, unsigned(input), OffsetAndGain(ConvertOffset(offset), ConvertGain(gain))); }
    //! Starts single measurements for the specified channel at the specified rate
    async(StartSingle, Rate rate = Rate100Sps, unsigned index = 0) { return async_forward(Start, FDCConfig(rate | BIT(int(FDCConfigEnableOffset) + ChannelCount - 1 - index))); }
    //! Starts repeated measurements at the specified rate
    async(StartRepeat, Rate rate = Rate100Sps, unsigned mask = 1) { return async_forward(Start, FDCConfig::Repeat | FDCConfig(rate | (RevMask(mask) << FDCConfigEnableOffset))); }
    //! Stops repeated measurements
    async(Stop);
    //! Waits until at least one measurement is completed or the specified timeout elapses
    //! @returns a bitmask of measurements which are completed
    async(Wait, Timeout timeout = Timeout::Infinite);
    //! Retrieves data for all completed measurements
    //! @returns a bitmask of measurements which have been updated
    async(Measure, Timeout timeout = Timeout::Infinite);

    //! Retrieves the last measured capacitance for the specified channel
    float GetCapacitance(unsigned index = 0) const { return value[index]; }

protected:
    const char* DebugComponent() const { return "FDC1004"; }

private:
    enum struct Register : uint8_t
    {
        MEAS1_MSB = 0x00,
        MEAS1_LSB = 0x01,
        MEAS2_MSB = 0x02,
        MEAS2_LSB = 0x03,
        MEAS3_MSB = 0x04,
        MEAS3_LSB = 0x05,
        MEAS4_MSB = 0x06,
        MEAS4_LSB = 0x07,

        CONF_MEAS1 = 0x08,
        CONF_MEAS2 = 0x09,
        CONF_MEAS3 = 0x0A,
        CONF_MEAS4 = 0x0B,

        FDC_CONF = 0x0C,

        OFFSET_CAL_CIN1 = 0x0D,
        OFFSET_CAL_CIN2 = 0x0E,
        OFFSET_CAL_CIN3 = 0x0F,
        OFFSET_CAL_CIN4 = 0x10,

        GAIN_CAL_CIN1 = 0x11,
        GAIN_CAL_CIN2 = 0x12,
        GAIN_CAL_CIN3 = 0x13,
        GAIN_CAL_CIN4 = 0x14,

        MFG_ID = 0xFE,
        DEVICE_ID = 0xFF,
    };

    enum
    {
        DeviceID = 0x1004,          //< Device ID
        ManufacturerID = 0x5449,    //< Texas Instruments
        Address = 0x50,             //< Fixed device address

        ChannelCount = 4,

        ChannelConfigPositiveOffset = 13,
        ChannelConfigNegativeOffset = 10,
        ChannelConfigCAPDACOffset = 5,
        ChannelConfigNegativeDisabled = 7 << ChannelConfigNegativeOffset,

        FDCConfigDoneOffset = 0,
        FDCConfigEnableOffset = 4,
        FDCConfigRateOffset = 10,
    };

    enum struct FDCConfig : uint16_t
    {
        DoneMask = MASK(4) << FDCConfigDoneOffset,
        EnableMask = MASK(4) << FDCConfigEnableOffset,
        Repeat = BIT(8),

        RateMask = MASK(2) << FDCConfigRateOffset,

        Reset = BIT(15),
    };

    DECLARE_FLAG_ENUM(FDCConfig);

    struct OffsetAndGain
    {
        constexpr OffsetAndGain(uint16_t offset, uint16_t gain)
            : offset(offset), gain(gain) {}

        uint16_t offset;
        uint16_t gain;
    };

    async(Configure, unsigned channel, uint16_t cfg);
    async(SetCalibration, unsigned input, OffsetAndGain offsetAndGain);
    async(Start, FDCConfig cfg);

    static constexpr float ConvertValue(uint32_t val) { return val * (float)(1.0 / BIT(19)); }
    static constexpr uint16_t ConvertCAPDAC(float val) { return std::min(unsigned(val * (float)(1.0 / 3.125)), MASK(5)) << ChannelConfigCAPDACOffset; }
    static constexpr uint16_t ConvertOffset(float val) { return std::min(std::max(signed(val * (float)BIT(11)), -32768), 32767); }
    static constexpr uint16_t ConvertGain(float val) { return std::min(unsigned(val * (float)BIT(14)), MASK(16)); }
#ifdef __RBIT
    static ALWAYS_INLINE unsigned RevMask(unsigned mask) { return __RBIT(mask) >> 28; }
#else
    static ALWAYS_INLINE unsigned RevMask(unsigned mask) { return (mask & 1) << 3 | (mask & 2) << 1 | (mask & 4) >> 1 | (mask & 8) >> 3; }
#endif

    bool init = false;
    uint8_t configuredChannels = 0;
    float value[ChannelCount] = { NAN, NAN, NAN, NAN };

    template<typename T> static constexpr T swap16(T val) { return (T)FROM_BE16((uint16_t)val); }
};

DEFINE_FLAG_ENUM(FDC1004::FDCConfig);

}
