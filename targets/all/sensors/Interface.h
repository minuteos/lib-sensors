/*
 * Copyright (c) 2020 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/Interface.h
 */

#pragma once

#include <kernel/kernel.h>

namespace sensors
{

class Interface
{
public:
    union RegAndLength
    {
        constexpr RegAndLength(uint8_t reg, uint16_t length)
            : value(reg << 16 | length){}

        uint32_t value;
        struct
        {
            uint16_t length;
            uint8_t reg;
        };
    };

    virtual async(ReadRegisterImpl, RegAndLength arg, void* buf) = 0;
    virtual async(WriteRegisterImpl, RegAndLength arg, const void* buf) = 0;

protected:
#if TRACE
    const class Sensor* _owner;
    const char* OwnerDebugComponent() const;
    virtual void _DebugHeader() const = 0;
    friend class Sensor;
#endif

};

}
