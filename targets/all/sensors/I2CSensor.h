/*
 * Copyright (c) 2019 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/I2CSensor.h
 */

#pragma once

#include <kernel/kernel.h>

#include <bus/I2C.h>

#include "Interface.h"

namespace sensors
{

class I2CSensor
{
protected:
    I2CSensor(bus::I2C i2c, uint8_t address)
        : dev(i2c.Master(address))
    {
    }

    //! Indicates the next operation on the device
    using Next = bus::I2C::Next;

    async(Read, Buffer data, Next next = Next::Stop) { return async_forward(dev.Read, data, next); }
    async(Write, Span data, Next next = Next::Stop) { return async_forward(dev.Write, data, next); }
    //! Reads data from consecutive registers (register address is written before changing direction)
    template<typename T> async(ReadRegister, T reg, Buffer buf, bool allowFail = false) { return async_forward(ReadRegisterImpl, RegAndLength(uint8_t(reg), buf.Length(), allowFail), buf.Pointer()); }
    //! Writes data to consecutive registers (register address is written as the first byte)
    template<typename T> async(WriteRegister, T reg, Span buf, bool allowFail = false) { return async_forward(WriteRegisterImpl, RegAndLength(uint8_t(reg), buf.Length(), allowFail), buf.Pointer()); }

    uint8_t BusAddress() const { return dev.Address(); }
    unsigned Transferred() const { return dev.Transferred(); }

    //! Gets the current bus frequency
    uint32_t OutputFrequency() const { return dev.Bus().OutputFrequency(); }
    //! Sets the current bus frequency
    void OutputFrequency(uint32_t freq) { dev.Bus().OutputFrequency(freq); }

#if TRACE
    virtual const char* DebugComponent() const { return "I2CSensor"; }
    void _DebugHeader() const { DBG("%s[%02X]: ", DebugComponent(), BusAddress()); }
    template<typename... Args> void MYDBG(Args... args) { _DebugHeader(); _DBG(args...); _DBGCHAR('\n'); }
#else
    template<typename... Args> void MYDBG(Args...) {}
#endif

#if TRACE && SENSOR_TRACE
    template<typename... Args> void MYTRACE(Args... args) { MYDBG(args...); }
#else
    template<typename... Args> void MYTRACE(Args...) {}
#endif

private:
    bus::I2C::Device dev;

    typedef Interface::RegAndLength RegAndLength;

    async(ReadRegisterImpl, RegAndLength arg, void* buf);
    async(WriteRegisterImpl, RegAndLength arg, const void* buf);

    friend class I2CInterface;
};

}
