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
        : i2c(i2c), address(address)
    {
    }

    //! Starts or restarts an I2C read transaction with optional stop
    async(Read, Buffer buf, bool start, bool stop) { return async_forward(i2c.Read, address, buf, start, stop); }
    //! Continues a running I2C read transaction with optional stop
    async(Read, Buffer buf, bool stop) { return async_forward(i2c.Read, buf, stop); }
    //! Starts or restarts an I2C write transaction with optional stop
    async(Write, Span buf, bool start, bool stop) { return async_forward(i2c.Write, address, buf, start, stop); }
    //! Continues a running I2C write transaction with optional stop
    async(Write, Span buf, bool stop) { return async_forward(i2c.Write, buf, stop); }
    //! Reads data from consecutive registers (register address is written before changing direction)
    template<typename T> async(ReadRegister, T reg, Buffer buf, bool allowFail = false) { return async_forward(ReadRegisterImpl, RegAndLength(uint8_t(reg), buf.Length(), allowFail), buf.Pointer()); }
    //! Writes data to consecutive registers (register address is written as the first byte)
    template<typename T> async(WriteRegister, T reg, Span buf, bool allowFail = false) { return async_forward(WriteRegisterImpl, RegAndLength(uint8_t(reg), buf.Length(), allowFail), buf.Pointer()); }

    uint8_t BusAddress() const { return address; }

    //! Gets the current bus frequency
    uint32_t OutputFrequency() const { return i2c.OutputFrequency(); }
    //! Sets the current bus frequency
    void OutputFrequency(uint32_t freq) { i2c.OutputFrequency(freq); }

#if TRACE
    virtual const char* DebugComponent() const { return "I2CSensor"; }
    void _DebugHeader() const { DBG("%s[%02X]: ", DebugComponent(), address); }
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
    bus::I2C i2c;
    uint8_t address;

    typedef Interface::RegAndLength RegAndLength;

    async(ReadRegisterImpl, RegAndLength arg, void* buf);
    async(WriteRegisterImpl, RegAndLength arg, const void* buf);

    friend class I2CInterface;
};

}
