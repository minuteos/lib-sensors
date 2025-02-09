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

    const UbxData& ExtendedData() const { return stableData; }

protected:
    virtual void OnMessage(io::Pipe::Iterator& message);
    virtual void OnIdle();

    async(PollRequest) { return async_forward(SendMessage, "PUBX,00"); }

private:
    bool request00 = true;
    UbxData data = { NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, 0, FixType::Unknown, -1 }, stableData = data;

    FixType ReadFixType(io::Pipe::Iterator& message);
};

}
