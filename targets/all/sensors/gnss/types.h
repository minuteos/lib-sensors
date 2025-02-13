/*
 * Copyright (c) 2024 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/gnss/types.h
 */

#pragma once

#include <base/base.h>

namespace sensors::gnss
{

struct Decimal
{
    int value;
    int divisor;
};

struct Date
{
    Date() {}
    ALWAYS_INLINE Date(int num)
    {
        y = num % 100;
        num /= 100;
        m = num % 100;
        num /= 100;
        d = num;
    }
    constexpr bool IsValid() const { return !!d; }

    uint8_t d, m, y;
};

struct Time
{
    Time() {}
    ALWAYS_INLINE Time(Decimal dec)
    {
        if (!dec.divisor)
        {
            h = m = s = hs = 0;
            return;
        }

        hs = dec.value % dec.divisor * 100 / dec.divisor;
        dec.value /= dec.divisor;
        s = dec.value % 100;
        dec.value /= 100;
        m = dec.value % 100;
        dec.value /= 100;
        h = dec.value;
    }

    uint8_t h, m, s, hs;
};

}
