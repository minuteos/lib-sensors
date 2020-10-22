/*
 * Copyright (c) 2019 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/environment/CCS811.h
 */

#pragma once

#include <sensors/I2CSensor.h>

#include <hw/GPIO.h>

namespace sensors::environment
{

//! Driver for ams CCS811 CO2/VOC sensor
class CCS811 : I2CSensor
{
public:
    enum struct Address : uint8_t
    {
        Low = 0x5A,
        High = 0x5B,
    };

    enum struct DriveMode : uint8_t
    {
        Idle = 0,
        ConstantPower_1s = 1,
        PulseHeating_10s = 2,
        LowPowerPulse_60s = 3,
        ConstantPower_250ms = 4,
    };

    CCS811(bus::I2C i2c, Address address = Address::Low, GPIOPin wake = Px, GPIOPin reset = Px)
        : I2CSensor(i2c, (uint8_t)address), wake(wake), reset(reset)
    {
    }

    //! Initializes the sensors and configures the selected mode
    async(Init);
    //! Retrieves the last measurement result, return value indicates if the measured values have changed in the meantime
    //! If current mode is @ref Idle, the sensor is temporary switched to mode 4 and waits for one cycle to complete
    async(Measure);
    //! Sets the measurement mode. If the sensor is already initialized, change is applied immediately
    async(SetMode, DriveMode mode);
    //! Gets the currently requested measurement mode
    DriveMode GetMode() const { return mode; }

    //! Gets the last CO2 measurement in ppm; NaN if not available
    float GetCO2ppm() const { return co2; }
    //! Gets the last tVOC measurement in ppb; NaN if not available
    float GetTVOCppb() const { return tvoc; }
    //! Gets the raw data from last measurement
    uint16_t GetRaw() const { return raw; }

    //! Sets environment temperature (degrees Celsius)
    void EnvironmentTemperature(float temp) { envCfg.Temperature(temp); RequestUpdate(); }
    //! Gets the configured environment temperature
    float EnvironmentTemperature() const { return envCfg.Temperature(); }
    //! Sets environment relative humidity (1 == 100%)
    void EnvironmentHumidity(float rh) { envCfg.Humidity(rh); RequestUpdate(); }
    //! Gets the configured environment humidity
    float EnvironmentHumidity() const { return envCfg.Humidity(); }

protected:
    const char* DebugComponent() const { return "CCS811"; }

private:
    DriveMode mode = DriveMode::Idle;
    bool init = false;
    bool update = false;
    uint32_t wakeCount = 0;
    GPIOPin wake, reset;
    float co2 = NAN, tvoc = NAN;
    float envHum = NAN, envTemp = NAN;
    uint16_t raw;

    void RequestUpdate();
    bool UpdateRequired() const { return envSet.raw != envCfg.raw; }
    async(Wake);
    async(Sleep);
    async(Update);

    enum struct Register : uint8_t
    {
        Status = 0x00,
        Mode = 0x01,
        Result = 0x02,
        Raw = 0x03,
        EnvData = 0x05,
        Thresholds = 0x10,
        Baseline = 0x11,
        HWID = 0x20,
        HWVersion = 0x21,
        FWBootVersion = 0x23,
        FWAppVersion = 0x24,
        ErrorID = 0xE0,
        BootAppErase = 0xF1,
        BootAppData = 0xF2,
        BootAppVerify = 0xF3,
        BootAppStart = 0xF4,
        SoftReset = 0xFF,
    };

    enum struct HwID : uint8_t
    {
        CCS811 = 0x81,
    };

    enum struct Error : uint8_t
    {
        //! Attempt to write to an invalid register
        InvalidWrite,
        //! Attempt to read from an invalid register
        InvalidRead,
        //! Invalid mode value written
        InvalidMode,
        //! Measured value exceeded range
        MaxResistance,
        //! Heater failed
        HeaterFault,
        //! Heater supply failed
        HeaterSupply,
    };

    enum struct ResetKey : uint32_t
    {
        Boot = 0x8A72E511,
    };

    enum struct AppEraseKey : uint32_t
    {
        Key = 0x09E6A7E7,
    };

    enum struct AppVerifyKey : uint8_t
    {
        Key = 0,
    };

    union Status
    {
        uint8_t raw;
        struct
        {
            uint8_t error : 1;
            uint8_t : 2;
            uint8_t dataReady : 1;
            uint8_t appValid : 1;
            uint8_t appVerify : 1;
            uint8_t appErase : 1;
            uint8_t appRunning : 1;
        };
    };

    union Mode
    {
        uint8_t raw;
        struct
        {
            uint8_t : 2;
            uint8_t intThreshold : 1;
            uint8_t intDataReady : 1;
            uint8_t driveMode : 3;
            uint8_t : 1;
        };
    };

    union RawData
    {
        uint16_t raw;
        struct
        {
            //! Top two bits of raw ADC value
            uint8_t adcHi : 2;
            //! Current in microamperes
            uint8_t current : 6;
            //! Bottom eight bits of raw ADC value
            uint8_t adcLo;
        };
    };

    union EnvData
    {
        uint32_t raw;
        struct
        {
            //! Big endian relative humidity in 1/512% steps, [0,51200] => [0%,100%] RH
            uint16_t humBE;
            //! Big endian temperature in 1/512 degree steps, [0,65536) => [-25,102) deg. C
            uint16_t tempBE;
        };

        float Temperature() const { return tempBE == 0xFFFF ? NAN : FROM_BE16(tempBE) / 512.0f - 25; }
        void Temperature(float temp) { tempBE = TO_BE16(__USAT((temp + 25) * 512, 16)); }

        float Humidity() const { return humBE == 0xFFFF ? NAN : FROM_BE16(humBE) / 512.0f; }
        void Humidity(float rh) { humBE = TO_BE16(__USAT(rh * 51200, 16)); }
    } envSet = { ~0u }, envCfg = { ~0u };

    union Result
    {
        uint8_t data[8];
        struct
        {
            //! Big endian measured CO2 level in ppm
            uint16_t co2BE;
            //! Big endian measured TVOC level in ppb
            uint16_t tvocBE;
            Status status;
            Error error;
            RawData raw;
        };
    };

    struct Thresholds
    {
        //! Big endian threshold between low and medium CO2 concentration in ppm (default 1500)
        uint16_t lowMedBE;
        //! Big endian threshold between medium and high CO2 concentration in ppm (default 2500)
        uint16_t medHighBE;
    };

    struct HwVersion
    {
        //! Build variant
        uint8_t build : 4;
        //! Major hardware version, should be 1
        uint8_t major : 4;
    };

    struct FwVersion
    {
        //! Minor version
        uint8_t minor : 4;
        //! Major version
        uint8_t major : 4;
        //! Trivial version (revision)
        uint8_t trivial;
    };

    union
    {
        Status status;
        Mode mode;
        Result result;
        RawData raw;
        EnvData env;
        Thresholds thr;
        //! Calculation baseline with unspecified format
        //! Can be stored after calibration in clean air and restored later
        uint16_t baseline;
        //! Hardware ID
        HwID hwId;
        //! Hardware version
        HwVersion hwVer;
        //! Bootloader version
        FwVersion bootVer;
        //! Firmware version
        FwVersion appVer;
        Error error;
        ResetKey reset;
        AppEraseKey appErase;
        AppVerifyKey appVerify;
    } msg;
};

}
