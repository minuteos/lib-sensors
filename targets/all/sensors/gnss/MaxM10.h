/*
 * Copyright (c) 2024 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/gnss/MaxM10.h
 */

#pragma once

#include <kernel/kernel.h>

#include "NmeaGnssDevice.h"

namespace sensors::gnss
{

class MaxM10 : public NmeaGnssDevice
{
public:
    MaxM10(io::DuplexPipe pipe)
        : NmeaGnssDevice(pipe)
    {
    }

    async(SetBaudRate, unsigned baudRate) { return async_forward(SendMessageF, "PUBX,41,1,3,3,%u,0", baudRate); }
};

}
