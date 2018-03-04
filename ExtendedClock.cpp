/*
 * ExtendedClock.cpp
 *
 *  Created on: 4 janv. 2018
 *      Author: Ross
 */

// The us ticker is a wrapping uint32_t. We insert interrupts at
// 0, 0x40000000, 0x8000000, and 0xC0000000, rather than just 0 or just 0xFFFFFFFF because there is
// code that calls interrupts that are "very soon" immediately and we don't
// want that. Also because if we only use 0 and 0x80000000 then there is a chance it would
// be considered to be in the past and executed immediately.

#include <mbed.h>
#include <ExtendedClock.h>

static ExtendedClock _GlobalClock;

// Return the number of seconds since boot.
float clock_s()
{
    return _GlobalClock.read();
}

// Return the number of milliseconds since boot.
uint64_t clock_ms()
{
    return _GlobalClock.read_ms();
}

// Return the number of microseconds since boot.
uint64_t clock_us()
{
    return _GlobalClock.read_us();
}

