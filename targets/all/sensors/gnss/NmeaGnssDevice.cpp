/*
 * Copyright (c) 2024 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/gnss/NmeaGnssDevice.cpp
 */

#include "NmeaGnssDevice.h"

#define MYDBG(...)      DBGCL("GNSS", __VA_ARGS__)

//#define GNSS_TRACE    1

#if GNSS_TRACE
#define MYTRACE(...)    MYDBG(__VA_ARGS__)
#else
#define MYTRACE(...)
#endif

namespace sensors::gnss
{

void NmeaGnssDevice::OnMessage(io::Pipe::Iterator& message)
{
#if TRACE && GNSS_TRACE
    DBGC("GNSS", "<< ");
    for (auto s: message.Spans()) { _DBG("%b", s); }
    _DBGCHAR('\n');
#endif
    char talker[2], command[3];
    message.Read(talker);
    message.Read(command);
    SkipFieldSeparator(message);

    switch (ID(talker))
    {
        case ID("GN"):
            switch (ID(command))
            {
                case ID("TXT"): // text message
                {
#if TRACE
                    int n = ReadNum(message);
                    int cnt = ReadNum(message);
                    int level = ReadNum(message);

                    DBGC("GNSS", "Message %d/%d [%d]: ", n, cnt, level);
                    for (auto s : message.Spans()) { _DBG("%b", s); }
                    _DBGCHAR('\n');
#endif
                    return;
                }

                case ID("RMC"): // recommended minimum data (basic location, etc.)
                {
                    auto data = this->data;
                    data.source = this;
                    data.time = ReadDecimal(message);
                    data.status = ReadChar(message);
                    data.latitude = ReadDeg(message) * (ReadChar(message) == 'S' ? -1 : 1);
                    data.longitude = ReadDeg(message) * (ReadChar(message) == 'W' ? -1 : 1);
                    data.groundSpeedKnots = ReadFloat(message);
                    data.course = ReadFloat(message);
                    data.date = ReadNum(message);
                    data.magVariance = ReadFloat(message) * (ReadChar(message) == 'W' ? -1 : 1);
                    data.posMode = ReadChar(message);
                    data.navStatus = ReadChar(message);
                    Update(data);
                    return;
                }

                case ID("VTG"):
                {
                    auto data = this->data;
                    data.course = ReadFloat(message);
                    ReadChar(message);  // fixed 'T'
                    data.magneticCourse = ReadFloat(message);
                    ReadChar(message);  // fixed 'M'
                    data.groundSpeedKnots = ReadFloat(message);
                    ReadChar(message);  // fixed 'N'
                    data.groundSpeedKm = ReadFloat(message);
                    ReadChar(message);  // fixed 'K'
                    data.posMode = ReadChar(message);
                    Update(data);
                    return;
                }

                case ID("GGA"): // fix data
                {
                    auto data = this->data;
                    data.time = ReadDecimal(message);
                    data.latitude = ReadDeg(message) * (ReadChar(message) == 'S' ? -1 : 1);
                    data.longitude = ReadDeg(message) * (ReadChar(message) == 'W' ? -1 : 1);
                    data.quality = ReadNum(message);
                    data.numSat = ReadNum(message);
                    data.hdop = ReadFloat(message);
                    data.altitude = ReadFloat(message);
                    ReadChar(message);  // fixed 'M'
                    data.separation = ReadFloat(message);
                    ReadChar(message);  // fixed 'M'
                    data.diffAge = ReadNum(message);
                    data.diffStation = ReadNum(message);
                    Update(data);
                    return;
                }

                case ID("GSA"): // satellite data
                {
                    auto data = this->data;
                    data.opMode = ReadChar(message);
                    data.navMode = ReadNum(message);
                    for (int i = 0; i < 12; i++) { ReadNum(message); }  // skip over satellite IDs
                    data.pdop = ReadFloat(message);
                    data.hdop = ReadFloat(message);
                    data.vdop = ReadFloat(message);
                    data.systemId = ReadNum(message, 16);
                    Update(data);
                    return;
                }

                case ID("GLL"): // location data
                    // don't care - RMC already contains everything in GLL
                    return;
            }
            break;
    }

    // handle commands that can come from any talker
    switch (ID(command))
    {
        case ID("GSV"): // satellites in view
        {
            int msgCnt = ReadNum(message);
            int msgNum = ReadNum(message);
            uint8_t numSat = ReadNum(message);
            int numTrack = 0;
            uint8_t sigId = ReadNum(message);
            while (message)
            {
                // this is actually a satellite record
                // we're only interested on whether it's being tracked
                // sigId was really svId, ignore it
                ReadDecimal(message);   // elevation
                ReadDecimal(message);   // azimuth
                if (ReadDecimal(message).divisor)
                {
                    numTrack++;
                }
                sigId = ReadNum(message);   // next candidate for signalId
            }
            if (msgNum > 1 && (
                msgNum != sdataPendLast + 1 ||
                msgCnt != sdataPendTotal ||
                sigId != sdataPending.signalId ||
                talker[1] != sdataPending.talkerId))
            {
                MYDBG("GSV out of order, expected %c:%d %d %d/%d, received %c:%d %d %d/%d",
                    sdataPending.talkerId, sdataPending.signalId, sdataPending.visSat, sdataPendLast + 1, sdataPendTotal,
                    talker[1], sigId, numSat, msgNum, msgCnt);
                break;
            }

            if (msgNum == 1)
            {
                if (sdataPending.talkerId)
                {
                    MYDBG("Dropping incomplete GSV data %c:%d %d %d/%d",
                        sdataPending.talkerId, sdataPending.signalId, sdataPending.visSat, sdataPendLast + 1, sdataPendTotal);
                }
                sdataPending = {
                    .talkerId = talker[1], .signalId = sigId,
                    .trkSat = 0, .visSat = numSat,
                };
            }
            sdataPending.stamp = MONO_CLOCKS;
            sdataPending.trkSat += numTrack;
            if (msgNum == msgCnt)
            {
                SaveSatelliteData(sdataPending);
                sdataPending = {};
                sdataPendLast = sdataPendTotal = 0;
            }
            else
            {
                sdataPendLast = msgNum;
                sdataPendTotal = msgCnt;
            }
            return;
        }
    }

    // only unknown messages get here
#if TRACE && !GNSS_TRACE
    DBGC("GNSS", "<? %b%b,", Span(talker, sizeof(talker)), Span(command, sizeof(command)));
    for (auto s: message.Spans()) { _DBG("%b", s); }
    _DBGCHAR('\n');
#endif

}

void NmeaGnssDevice::Update(const LocationData& data)
{
    if (memcmp(&this->data, &data, sizeof(LocationData)))
    {
        this->data = data;
        kernel::FireEvent(data);
    }
}

void NmeaGnssDevice::SaveSatelliteData(const SatelliteData& data)
{
    SatelliteData* oldest = NULL;
    SatelliteData* match = NULL;
    SatelliteData* free = NULL;
    for (auto& sd: sdata)
    {
        if (!free && !sd.talkerId) { free = &sd; }
        if (!match && sd.talkerId == data.talkerId && sd.signalId == data.signalId) { match = &sd; }
        if (!oldest || (data.stamp - sd.stamp > data.stamp - oldest->stamp)) { oldest = &sd; }
    }
    if (!match && !free)
    {
        MYDBG("Dropping oldest GSV data %c:%d %d/%d",
            oldest->talkerId, oldest->signalId, oldest->trkSat, oldest->visSat);
    }
    *(match ? match : free ? free : oldest) = data;
    kernel::FireEvent(data);

    // update the number of tracked sats in LocationData
    auto ld = this->data;
    ld.trkSat = ld.visSat = 0;
    for (auto& sd: sdata)
    {
        if (sd.talkerId)
        {
            ld.trkSat += sd.trkSat;
            ld.visSat += sd.visSat;
        }
    }
    Update(ld);
}

}
