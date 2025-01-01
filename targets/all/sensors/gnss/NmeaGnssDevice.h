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

    const LocationData& LastLocation() const { return data; }

protected:
    virtual void OnMessage(io::Pipe::Iterator& message);

private:
    enum {
        MaxGsvGroups = 10,
    };

    LocationData data = {};
    SatelliteData sdata[MaxGsvGroups];
    SatelliteData sdataPending = {};
    uint8_t sdataPendLast, sdataPendTotal;

    void Update(const LocationData& data);
    void SaveSatelliteData(const SatelliteData& data);
};

}
