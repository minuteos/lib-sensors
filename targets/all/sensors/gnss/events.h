/*
 * Copyright (c) 2024 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/gnss/Events.h
 */

#pragma once

#include <base/base.h>

#include "types.h"

namespace sensors::gnss
{

class NmeaGnssDevice;

struct LocationData
{
    const NmeaGnssDevice* source;

    Date date;
    Time time;
    float latitude, longitude;
    float groundSpeedKnots, groundSpeedKm;
    float course, magneticCourse;
    float magVariance;
    int quality, numSat, lockSat, trkSat, visSat, knownSat;
    float hdop, pdop, vdop;
    float altitude, separation;
    int diffAge, diffStation;
    char status, posMode, navStatus, opMode;
    uint8_t navMode, systemId;
};

struct SatelliteData
{
    mono_t stamp;
    uint32_t groupId;   // 0x00ttttss where tttt = 16-bit talker-id and ss = 8-bit signal-id
    uint8_t lockSat, visSat, knownSat;

    constexpr uint8_t SignalId() const { return groupId; }
    constexpr uint16_t TalkerId() const { return groupId >> 8; }
};

}
