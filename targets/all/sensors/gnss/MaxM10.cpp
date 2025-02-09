/*
 * Copyright (c) 2024 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/gnss/MaxM10.cpp
 */

#include "MaxM10.h"

namespace sensors::gnss
{


void MaxM10::OnMessage(io::Pipe::Iterator& message)
{
    if (request00)
    {
        request00 = false;
        kernel::Task::Run(this, &MaxM10::PollRequest);
    }

    if (!message.Matches("PUBX,00,"))
    {
        return NmeaGnssDevice::OnMessage(message);
    }

    message.Skip(8);
    ReadDecimal(message);  // time
    ReadDecimal(message);  // latitude
    ReadChar(message);  // N/S
    ReadDecimal(message);   // longitude
    ReadChar(message);  // E/W
    data.altitude = ReadFloat(message);
    data.fixType = ReadFixType(message);
    data.hAcc = ReadFloat(message);
    data.vAcc = ReadFloat(message);
    data.groundSpeedKm = ReadFloat(message);
    data.course = ReadFloat(message);
    data.vVel = ReadFloat(message);
    data.diffAge = ReadNum(message, 10, -1);
    data.hdop = ReadFloat(message);
    data.vdop = ReadFloat(message);
    data.tdop = ReadFloat(message);
    data.numSat = ReadNum(message);
}

void MaxM10::OnIdle()
{
    NmeaGnssDevice::OnIdle();
    stableData = data;
    request00 = true;
}

FixType MaxM10::ReadFixType(io::Pipe::Iterator& message)
{
    char id[2];
    message.Read(id);
    message.Consume(',');

    switch (ID(id))
    {
        case ID("NF"): return FixType::None;
        case ID("DR"): return FixType::DeadReckoning;
        case ID("G2"): return FixType::Std2D;
        case ID("G3"): return FixType::Std3D;
        case ID("D2"): return FixType::Diff2D;
        case ID("D3"): return FixType::Diff3D;
        case ID("RK"): return FixType::Combined;
        case ID("TT"): return FixType::TimeOnly;
        default: return FixType::Unknown;
    }
}

}
