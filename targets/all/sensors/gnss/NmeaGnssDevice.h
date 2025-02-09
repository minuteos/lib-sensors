/*
 * Copyright (c) 2024 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/gnss/NmeaGnssDevice.h
 *
 * Base for GNSS devices communicating using the NMEA protocol
 */

#pragma once

#include <kernel/kernel.h>

#include "NmeaDevice.h"
#include "events.h"

namespace sensors::gnss
{

class NmeaGnssDevice : public NmeaDevice
{
public:
    NmeaGnssDevice(io::DuplexPipe pipe)
        : NmeaDevice(pipe) {}

    const LocationData& LastLocation() const { return stableData; }

protected:
    virtual void OnMessage(io::Pipe::Iterator& message);
    virtual void OnIdle();

private:
    enum {
        MaxGsvGroups = 10,
    };

    LocationData data = {
        .latitude = NAN, .longitude = NAN,
        .groundSpeedKnots = NAN, .groundSpeedKm = NAN,
        .course = NAN, .magneticCourse = NAN,
        .magVariance = NAN,
        .hdop = NAN, .pdop = NAN, .vdop = NAN,
        .altitude = NAN, .separation = NAN,
    };
    LocationData stableData = data;
    SatelliteData sdata[MaxGsvGroups];
    SatelliteData sdataPending = {};
    uint8_t sdataPendLast, sdataPendTotal;

    void Update(const LocationData& data);
    void SaveSatelliteData(const SatelliteData& data);
};

}
