/*
 * ExtendedClock.h
 *
 *  Created on: 4 janv. 2018
 *      Author: Ross
 */

#ifndef EXTENDEDCLOCK_H_
#define EXTENDEDCLOCK_H_

#pragma once

#include <stdint.h>

class ExtendedClock: public TimerEvent
{
public:
	ExtendedClock()
	{
		// This also starts the us ticker.
		insert(0x40000000);
	}

	float read()
	{
		return read_us() / 1000000.0f;
	}

	uint64_t read_ms()
	{
		return read_us() / 1000;
	}

	uint64_t read_us()
	{
		return mTriggers * 0x40000000ull + (ticker_read(_ticker_data) & 0x3FFFFFFF);
	}

	private:
	    void handler() override
	    {
	        ++mTriggers;
	        // If this is the first time we've been called (at 0x4...)
	        // then mTriggers now equals 1 and we want to insert at 0x80000000.
	        insert((mTriggers+1) * 0x40000000);
	    }

	    // The number of times the us_ticker has rolled over.
	    uint32_t mTriggers = 0;
};

// Return the number of seconds since boot.
float clock_s();

// Return the number of milliseconds since boot.
uint64_t clock_ms();

// Return the number of microseconds since boot.
uint64_t clock_us();

#endif /* EXTENDEDCLOCK_H_ */
