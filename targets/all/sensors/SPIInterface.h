/*
 * Copyright (c) 2020 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/SPIInterface.h
 */

#pragma once

#include "Interface.h"

#include "SPISensor.h"

namespace sensors
{

class SPIInterface : public Interface, SPISensor
{
private:
    SPIInterface(bus::SPI bus, GPIOPin cs, uint8_t hdrRead, uint8_t hdrWrite)
        : SPISensor(bus, cs, hdrRead, hdrWrite) {}

protected:
    virtual async(ReadRegisterImpl, Interface::RegAndLength arg, void* buf) final override
        { return async_forward(SPISensor::ReadRegisterImpl, arg, buf); }
    virtual async(WriteRegisterImpl, Interface::RegAndLength arg, const void* buf) final override
        { return async_forward(SPISensor::WriteRegisterImpl, arg, buf); }

#if TRACE
    virtual const char* DebugComponent() const final override { return OwnerDebugComponent(); }
    virtual void _DebugHeader() const { SPISensor::_DebugHeader(); }
#endif

    friend class Sensor;
};

}
