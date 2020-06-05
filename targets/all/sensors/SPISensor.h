/*
 * Copyright (c) 2020 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/SPISensor.h
 */

#pragma once

#include <kernel/kernel.h>

#include <bus/SPI.h>

#include "Interface.h"

namespace sensors
{

class SPISensor
{
protected:
    SPISensor(bus::SPI spi, GPIOPin cs, uint8_t hdrRead, uint8_t hdrWrite)
        : spi(spi), cs(spi.GetChipSelect(cs)), hdrRead(hdrRead), hdrWrite(hdrWrite)
#if TRACE
            , pin(cs)
#endif
    {
    }

    //! Reads data from consecutive registers (register address is written before changing direction)
    template<typename T> async(ReadRegister, T reg, Buffer buf) { return async_forward(ReadRegisterImpl, RegAndLength(uint8_t(reg), buf.Length()), buf.Pointer()); }
    //! Writes data to consecutive registers (register address is written as the first byte)
    template<typename T> async(WriteRegister, T reg, Span buf) { return async_forward(WriteRegisterImpl, RegAndLength(uint8_t(reg), buf.Length()), buf.Pointer()); }

#if TRACE
    virtual const char* DebugComponent() const { return "SPISensor"; }
    void _DebugHeader() const { DBG("%s[%s]: ", DebugComponent(), pin.Name()); }
    template<typename... Args> void MYDBG(Args... args) { _DebugHeader(); _DBG(args...); _DBGCHAR('\n'); }
#else
    template<typename... Args> void MYDBG(Args...) {}
#endif

private:
    bus::SPI spi;
    bus::SPI::ChipSelect cs;
    uint8_t hdrRead, hdrWrite;
#if TRACE
    GPIOPin pin;
#endif

    using RegAndLength = Interface::RegAndLength;

    async(ReadRegisterImpl, RegAndLength arg, void* buf);
    async(WriteRegisterImpl, RegAndLength arg, const void* buf);

    friend class SPIInterface;
};

}
