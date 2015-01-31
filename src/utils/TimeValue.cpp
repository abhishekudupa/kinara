// TimeValue.cpp ---
//
// Filename: TimeValue.cpp
// Author: Abhishek Udupa
// Created: Wed Jan 15 14:53:54 2014 (-0500)
//
//
// Copyright (c) 2013, Abhishek Udupa, University of Pennsylvania
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by The University of Pennsylvania
// 4. Neither the name of the University of Pennsylvania nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//

// Code:

#include "TimeValue.hpp"

namespace ESMC {

void TimeValue::Initialize(const TimeValue& Other)
{
    Value = Other.Value;
}

TimeValue::TimeValue(const struct timespec& Value)
    : Value(Value)
{
    // Nothing here
}

TimeValue::TimeValue(time_t sec, long nsec)
{
    Value.tv_sec = sec;
    Value.tv_nsec = nsec;
}

TimeValue::TimeValue()
{
    Value.tv_sec = 0;
    Value.tv_nsec = 0;
}

TimeValue& TimeValue::operator = (const TimeValue& Other)
{
    if(&Other == this) {
        return *this;
    }
    Initialize(Other);
    return *this;
}

TimeValue TimeValue::operator - (const TimeValue& Other) const
{
    time_t sec;
    long nsec;
    struct timespec tv = Value;

    if (tv.tv_nsec < Other.Value.tv_nsec) {
        tv.tv_nsec += 1000000000LL;
        tv.tv_sec--;
    }

    nsec = tv.tv_nsec - Other.Value.tv_nsec;
    sec = tv.tv_sec - Other.Value.tv_sec;

    return TimeValue(sec, nsec);
}

TimeValue TimeValue::operator + (const TimeValue& Other) const
{
    time_t sec;
    long nsec;

    sec = 0;
    nsec = 0;

    nsec = this->Value.tv_nsec + Other.Value.tv_nsec;
    if(nsec > 1000000000LL) {
        nsec -= 1000000000LL;
        sec += 1;
    }
    sec += (this->Value.tv_sec + Other.Value.tv_sec);
    return TimeValue(sec, nsec);
}

TimeValue TimeValue::operator += (const TimeValue& Other)
{
    this->Value.tv_nsec += Other.Value.tv_nsec;
    if(this->Value.tv_nsec > 1000000000LL) {
        this->Value.tv_nsec -= 1000000000LL;
        this->Value.tv_sec += 1;
    }
    this->Value.tv_sec += Other.Value.tv_sec;
    return *this;
}

u64 TimeValue::InMicroSeconds() const
{
    return ((u64)Value.tv_sec * (u64)1000000 + ((u64)Value.tv_nsec) / 1000);
}

string TimeValue::ToString(u32 Verbosity) const
{
    ostringstream sstr;
    sstr << ((double)Value.tv_sec + ((double)Value.tv_nsec / 1000000000.0));
    return sstr.str();
}

TimeValue TimeValue::GetTimeValue(clockid_t ClockID)
{
    struct timespec tv;
#ifndef __APPLE__
    clock_gettime(ClockID, &tv);
#endif
    return TimeValue(tv);
}

TimeValue TimeValue::GetTimeValue()
{
#ifdef __APPLE__
    struct timespec tv;
    clock_serv_t cclock;
    mach_timespec_t mts;
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    tv.tv_sec = mts.tv_sec;
    tv.tv_nsec = mts.tv_nsec;
    return TimeValue(tv);
#else
    return TimeValue::GetTimeValue(CLOCK_THREAD_CPUTIME_ID);
#endif
}

ostream& operator << (ostream& str, const TimeValue& TV)
{
    str << TV.ToString();
    return str;
}

} /* End namespace ESMC */

//
// TimeValue.cpp ends here
