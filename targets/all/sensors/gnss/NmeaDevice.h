/*
 * Copyright (c) 2024 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/gnss/NmeaDevice.h
 *
 * Base for devices communicating using the NMEA protocol
 */

#pragma once

#include <kernel/kernel.h>
#include <io/DuplexPipe.h>

#include "types.h"

namespace sensors::gnss
{

class NmeaDevice
{
public:
    NmeaDevice(io::DuplexPipe pipe)
        : rx(pipe), tx(pipe) {}

    //! Initializes the sensor
    async(Init);
    //! Waits for all data to be sent
    async(TxIdle, Timeout timeout = Timeout::Infinite) { return async_forward(tx.Empty, timeout); }

protected:
    async(SendMessageF, const char* format, ...) async_def_va(SendMessageFV, format, Timeout::Infinite, format);
    async(SendMessageFTimeout, Timeout timeout, const char* format, ...) async_def_va(SendMessageFV, format, timeout, format);
    async(SendMessageFV, Timeout timeout, const char* format, va_list va);
    virtual void OnMessage(io::Pipe::Iterator& message) {}

    #pragma region Message readout helpers

    static bool SkipFieldSeparator(io::Pipe::Iterator& message) { return message.Consume(','); }
    static int ReadNum(io::Pipe::Iterator& message, unsigned base = 10, int errorValue = INT32_MAX);
    static float ReadDeg(io::Pipe::Iterator& message);
    static float ReadFloat(io::Pipe::Iterator& message);
    static char ReadChar(io::Pipe::Iterator& message);
    static Decimal ReadDecimal(io::Pipe::Iterator& message, unsigned base = 10) { return unpack<Decimal>(ReadDecimalImpl(message, base)); };
    static Packed<Decimal> ReadDecimalImpl(io::Pipe::Iterator& message, unsigned base);

    #pragma endregion

private:
    io::PipeReader rx;
    io::PipeWriter tx;

    async(Receiver);

    enum struct Signal
    {
        TaskActive = 1,
    };

    Signal sig = {};

    DECLARE_FLAG_ENUM(Signal);
};

DEFINE_FLAG_ENUM(NmeaDevice::Signal);

}
