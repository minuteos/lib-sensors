/*
 * Copyright (c) 2024 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/position/LSM6DSO.h
 */

#pragma once

#include <sensors/I2CSensor.h>
#include <math/Vector3.h>

namespace sensors::position
{

class LSM6DSO : I2CSensor
{
public:
    enum struct Address : uint8_t
    {
        Low = 0x6A,
        High = 0x6B,
    };

    LSM6DSO(bus::I2C i2c, Address address)
        : I2CSensor(i2c, (uint8_t)address)
    {
    }

    //! Accelerometer full-scale range
    enum struct AccelFs
    {
        Fs2g = 0b00,
        Fs16g = 0b01,
        Fs4g = 0b10,
        Fs8g = 0b11,
    };

    //! Gyroscope full-scale range
    enum struct GyroFs
    {
        Fs125dps = 0b001,
        Fs250dps = 0b000,
        Fs500dps = 0b010,
        Fs1000dps = 0b100,
        Fs2000dps = 0b110,
    };

    //! Output data rate
    enum struct Odr
    {
        Disabled = 0,
        Odr12p5Hz = 0b0001,
        Odr26Hz = 0b0010,
        Odr52Hz = 0b0011,
        Odr104Hz = 0b0100,
        Odr208Hz = 0b0101,
        Odr416Hz = 0b0110,
        Odr833Hz = 0b0111,
        Odr1k66Hz = 0b1000,
        Odr3k33Hz = 0b1001,
        Odr6k66Kz = 0b1010,
        Odr1p6Hz_AccelLpm = 0b1011,
    };

    //! Temperature output data rate
    enum struct TempOdr
    {
        Disabled = 0,
        Odr1p6Hz = 0b01,
        Odr12p5Hz = 0b10,
        Odr52Hz = 0b11,
    };

    //! Timestamp output rate
    enum struct TsRate
    {
        Disabled,
        Every1,
        Every8,
        Every32,
    };

    //! Fifo mode
    enum struct FifoMode
    {
        Bypass,
        Fifo,
        ContinuousToFifo = 3,
        BypassToContinuous,
        Continuous = 6,
        BypassToFifo,
    };

    //! Kind of data loaded from the FIFO
    enum struct FifoTag
    {
        NoData = 0,
        GyroNc = 1,
        AccelNc = 2,
        Temp = 3,
        Timestamp = 4,
        CfgChange = 5,
        AccelNcT2 = 6,
        AccelNcT1 = 7,
        Accel2C = 8,
        Accel3C = 9,
        GyroNcT2 = 0xA,
        GyroNcT1 = 0xB,
        Gyro2C = 0xC,
        Gyro3C = 0xD,
        Slave0 = 0xE,
        Slave1 = 0xF,
        Slave2 = 0x10,
        Slave3 = 0x11,
        StepCount = 0x12,
        Nack = 0x19,
    };

    //! Acceleration in X direction as a multiply of g (standard gravity)
    float GetAccelerationX() const { return ax; }
    //! Acceleration in Y direction as a multiply of g (standard gravity)
    float GetAccelerationY() const { return ay; }
    //! Acceleration in Z direction as a multiply of g (standard gravity)
    float GetAccelerationZ() const { return az; }
    //! Acceleration as a three-dimensional vector with elements as a multiply of g (standard gravity)
    Vector3 GetAcceleration() const { return { ax, ay, az }; }
    //! Angular velocity around X (pitch) axis in dps (degrees per second)
    float GetAngularX() const { return gx; }
    //! Angular velocity around Y (roll) axis in dps (degrees per second)
    float GetAngularY() const { return gy; }
    //! Angular velocity around Z (yaw) axis in dps (degrees per second)
    float GetAngularZ() const { return gz; }
    //! Angular velocity as a three-dimensional vector with elements in dps (degrees per second)
    Vector3 GetAngular() const { return { gx, gy, gz }; }

    //! Get full-scale range in g (standard gravity)
    float GetAccelerationScale() const { return cfgDesired.GetAccelerationScale(); }
    //! Get full-scale range in dps (degrees per second)
    float GetAngularScale() const { return cfgDesired.GetAngularScale(); }

    //! Configures the output data rate
    void Configure(Odr accel, Odr gyro) { cfgDesired.accelOdr = accel; cfgDesired.gyroOdr = gyro; }
    //! Configures the accelerometer full-scale range
    void Configure(AccelFs fs) { cfgDesired.accelFs = fs; }
    //! Configures the accelerometer full-scale range
    void Configure(GyroFs fs) { cfgDesired.gyroFs = fs; }
    //! Configures the accelerometer full-scale range
    void Configure(FifoMode mode, Odr accel, Odr gyro, TempOdr temp = TempOdr::Disabled, TsRate ts = TsRate::Disabled)
    {
        fifoDesired.fifoMode = mode;
        fifoDesired.accelOdr = accel;
        fifoDesired.gyroOdr = gyro;
        fifoDesired.tempOdr = temp;
        fifoDesired.tsRate = ts;
    }

    //! Initializes the sensor
    async(Init);
    //! Starts measuring
    async(Start);
    //! Stops measuring
    async(Stop);
    //! Retrieves the last measurement result, return value indicates if the measured values have changed in the meantime
    async(Measure);
    //! Reads the next entry from fifo, return value indicates the type of value read
    async(FifoRead);

protected:
    const char* DebugComponent() const { return "LSM6DSO"; }

private:
    enum struct Register : uint8_t
    {
        FuncCfgAddress = 0x01,
        PinCtrl = 0x02,
        FifoCtrl1 = 0x07,
        FifoCtrl2 = 0x08,
        FifoCtrl3 = 0x09,
        FifoCtrl4 = 0x0A,
        CounterBdr1 = 0x0B,
        CounterBdr2 = 0x0C,
        Int1Ctrl = 0x0D,
        Int2Ctrl = 0x0E,
        ID = 0xF,

        Control1 = 0x10,
        Control2 = 0x11,
        Control3 = 0x12,
        Control4 = 0x13,
        Control5 = 0x14,
        Control6 = 0x15,
        Control7 = 0x16,
        Control8 = 0x17,
        Control9 = 0x18,
        Control10 = 0x19,

        AllIntSrc = 0x1A,
        WakeUpSrc = 0x1B,
        TapSrc = 0x1C,
        D6DSrc = 0x1D,
        Status = 0x1E,

        OutTempL = 0x20, OutTempH = 0x21,
        OutGyroXL = 0x22, OutGyroXH = 0x23,
        OutGyroYL = 0x24, OutGyroYH = 0x25,
        OutGyroZL = 0x26, OutGyroZH = 0x27,
        OutAccXL = 0x28, OutAccXH = 0x29,
        OutAccYL = 0x2A, OutAccYH = 0x2B,
        OutAccZL = 0x2C, OutAccZH = 0x2D,

        EmbFuncStatus = 0x35,
        FsmStatusA = 0x36,
        FsmStatusB = 0x37,
        StatusMaster = 0x29,

        FifoStatus1 = 0x3A,
        FifoStatus2 = 0x3B,

        Timestamp0 = 0x40,
        Timestamp1 = 0x41,
        Timestamp2 = 0x42,
        Timestamp3 = 0x43,

        TapCfg0 = 0x56,
        TapCfg1 = 0x57,
        TapCfg2 = 0x58,
        TapThs6D = 0x59,

        IntDur2 = 0x5A,
        WakeUpThis = 0x5B,
        WakeUpDur = 0x5C,
        FreeFall = 0x5D,
        Md1Cfg = 0x5E,
        Md2Cfg = 0x5F,

        I3CBusAvb = 0x62,
        IntFreqFine = 0x63,
        OisInt = 0x6F,
        OisCtrl1 = 0x70,
        OisCtrl2 = 0x71,
        OisCtrl3 = 0x72,

        XOfsUsr = 0x73,
        YOfsUsr = 0x74,
        ZOfsUsr = 0x75,

        FifoOutTag = 0x78,
        FifoOutXL = 0x79, FifoOutXH = 0x7A,
        FifoOutYL = 0x7B, FifoOutYH = 0x7C,
        FifoOutZL = 0x7D, FifoOutZH = 0x7E,
    };

    enum struct IDValue : uint8_t
    {
        Valid = 0x6C,
    };

    enum struct Status : uint8_t
    {
        ReadyAccel = 1,
        ReadyGyro = 2,
        ReadyTemp = 4,
        ReadyAll = 7,
    };

    DECLARE_FLAG_ENUM(Status);

    enum struct DenMode
    {
        EdgeTrigger = 0b100,
        LevelTrigger = 0b010,
        LevelLatch = 0b011,
        LevelFifo = 0b110,
    };

    enum struct GyroHpf
    {
        Disabled = 0,
        Cut16mHz = 0b100,
        Cut65mHz = 0b101,
        Cut260mHz = 0b110,
        Cut1p04Hz = 0b111,
    };

    enum struct AccelFilterOdrDiv
    {
        OdrDiv4,
        OdrDiv10,
        OdrDiv20,
        OdrDiv45,
        OdrDiv100,
        OdrDiv200,
        OdrDiv400,
        OdrDiv800
    };

    enum struct FifoNoCompressRate
    {
        NoCompress0,
        NoCompress8,
        NoCompress16,
        NoCompress32,
    };

    async(UpdateConfiguration);

    struct FifoConfig
    {
        // FIFO_CTRL1+2
        uint16_t watermark : 9;
        FifoNoCompressRate compRate : 2;
        uint8_t : 1;
        bool odrChange : 1;
        uint8_t : 1;
        bool compress : 1;
        bool stopOnWtm : 1;

        // FIFO_CTRL3
        Odr accelOdr : 4;
        Odr gyroOdr : 4;

        // FIFO_CTRL4
        FifoMode fifoMode : 4;
        TempOdr tempOdr : 2;
        TsRate tsRate : 2;
    } fifoActual, fifoDesired = {};

    struct Config
    {
        // CTRL1, default 00
        uint8_t : 1;
        bool accelLpf2 : 1;
        AccelFs accelFs : 2;
        Odr accelOdr : 4;

        // CTRL2, default 00
        uint8_t : 1;
        GyroFs gyroFs : 3;
        Odr gyroOdr : 4;

        // CTRL3, default 04 (ifInc = 1)
        bool reset : 1;
        uint8_t : 1;
        bool ifInc : 1;
        bool spiMode : 1;
        bool intOpenDrain : 1;
        bool intActiveLow : 1;
        bool bdu : 1;
        bool boot : 1;

        // CTRL4, default 00
        uint8_t : 1;
        bool gyroLpf1 : 1;
        bool i2cDis : 1;
        bool drdyMask : 1;
        uint8_t : 1;
        bool intCombine : 1;
        bool gyroSleep : 1;
        uint8_t : 1;

        // CTRL5, default 00
        uint8_t accelSelfTest : 2;
        uint8_t gyroSelfTest : 2;
        uint8_t : 1;
        bool accelWrap : 1;
        bool gyroWrap : 1;
        bool accelUlp : 1;

        // CTRL6, default 00
        uint8_t gyroLpf : 3;    //< See datasheet for details
        bool accellUsrOffWeight : 1;
        bool accelHpm : 1;
        DenMode denMode : 3;

        // CTRL7, default 00
        bool oisOn : 1;
        bool accelUsrOffEnable : 1;
        bool oisOnEn : 1;
        uint8_t : 1;
        GyroHpf gyroHpf : 3;
        bool gyroHpm : 1;

        // CTRL8, default 00
        bool accel6dLpf : 1;
        bool accelOisFsMode : 1;
        bool accelSlopeFilter : 1;
        bool accelFastFilter : 1;
        bool accelHpfRefMode : 1;
        AccelFilterOdrDiv accelFilterOdrDiv : 3;

        // CTRL9, default E0 (denXYZ = 1)
        uint8_t : 1;
        bool i3cDisable : 1;
        bool denActiveHigh : 1;
        bool denAccel : 1;
        bool denStampAccel : 1;
        bool denZ : 1;
        bool denY : 1;
        bool denX : 1;

        // CTRL10, default 0
        uint8_t : 5;
        bool tsEn : 1;
        uint8_t : 2;

        float GetAccelerationScale() const { return BYTES(2, 16, 4, 8)[int(accelFs)]; }
        float GetAngularScale() const { return 125 << ((int(gyroFs) & 1) ? 0 : (int(gyroFs) >> 1)); }
    } cfgActual, cfgDesired = { .ifInc = 1 };

    bool init = false;
    uint8_t lastFifoTag = 0;
    float ax = NAN, ay = NAN, az = NAN;
    float gx = NAN, gy = NAN, gz = NAN;
    float amul, gmul;
};

DEFINE_FLAG_ENUM(LSM6DSO::Status);

}
