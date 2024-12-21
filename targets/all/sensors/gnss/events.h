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
    int quality, numSat;
    float hdop, pdop, vdop;
    float altitude, separation;
    int diffAge, diffStation;
    char status, posMode, navStatus, opMode;
    uint8_t navMode, systemId;
};

}
