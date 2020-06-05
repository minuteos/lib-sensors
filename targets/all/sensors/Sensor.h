/*
 * Copyright (c) 2019 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/I2CSensor.h
 */

#pragma once

#include <kernel/kernel.h>

#include "I2CInterface.h"
#include "SPIInterface.h"

namespace sensors
{

#if SENSORS_NO_I2C

class Sensor : protected SPISensor
{
protected:
    Sensor(bus::SPI spi, GPIOPin cs, uint8_t hdrRead, uint8_t hdrWrite)
        : SPISensor(spi, cs, hdrRead, hdrWrite) { }
    Sensor(bus::I2C i2c, uint8_t address)
        : SPISensor(NULL, Px, 0, 0) { ASSERT(false); }
};

#elif SENSORS_NO_SPI

class Sensor : protected I2CSensor
{
protected:
    Sensor(bus::SPI spi, GPIOPin cs, uint8_t hdrRead, uint8_t hdrWrite)
        : I2CSensor(null, 0) { ASSERT(false); }
    Sensor(bus::I2C i2c, uint8_t address)
        : I2CSensor(i2c, address) { }
};

#else

class Sensor
{
#if TRACE
    void InitTrace() { interface._owner = this; }
#else
    void InitTrace() {}
#endif

protected:
    Sensor(bus::I2C i2c, uint8_t address)
        : interface(*new(MemPoolAlloc<I2CInterface>()) I2CInterface(i2c, address)) { InitTrace(); }
    Sensor(bus::SPI spi, GPIOPin cs, uint8_t hdrRead, uint8_t hdrWrite)
        : interface(*new(MemPoolAlloc<SPIInterface>()) SPIInterface(spi, cs, hdrRead, hdrWrite)) { InitTrace(); }

    //! Reads data from consecutive registers (register address is written before changing direction)
    template<typename T> async(ReadRegister, T reg, Buffer buf) { return async_forward(interface.ReadRegisterImpl, Interface::RegAndLength(uint8_t(reg), buf.Length()), buf.Pointer()); }
    //! Writes data to consecutive registers (register address is written as the first byte)
    template<typename T> async(WriteRegister, T reg, Span buf) { return async_forward(interface.WriteRegisterImpl, Interface::RegAndLength(uint8_t(reg), buf.Length()), buf.Pointer()); }

#if TRACE
    virtual const char* DebugComponent() const { return "Sensor"; }
    template<typename... Args> void MYDBG(Args... args) { interface._DebugHeader(); _DBG(args...); _DBGCHAR('\n'); }
#else
    template<typename... Args> void MYDBG(Args...) {}
#endif

private:
    Interface& interface;

    friend class Interface;
};

#if TRACE
ALWAYS_INLINE const char* Interface::OwnerDebugComponent() const { return _owner->DebugComponent(); }
#endif

#endif

}
