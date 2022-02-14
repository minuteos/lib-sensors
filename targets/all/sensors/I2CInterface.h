/*
 * Copyright (c) 2020 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/I2CInterface.h
 */

#pragma once

#include "Interface.h"

#include "I2CSensor.h"

namespace sensors
{

class I2CInterface : public Interface, I2CSensor
{
private:
    I2CInterface(bus::I2C bus, uint8_t address)
        : I2CSensor(bus, address) {}

protected:
    virtual async(ReadRegisterImpl, Interface::RegAndLength arg, void* buf) final override
        { return async_forward(I2CSensor::ReadRegisterImpl, arg, buf); }
    virtual async(WriteRegisterImpl, Interface::RegAndLength arg, const void* buf) final override
        { return async_forward(I2CSensor::WriteRegisterImpl, arg, buf); }

#if TRACE
    virtual const char* DebugComponent() const final override { return OwnerDebugComponent(); }
    virtual void _DebugHeader() const final override { I2CSensor::_DebugHeader(); }
#endif

    friend class Sensor;
};

}
