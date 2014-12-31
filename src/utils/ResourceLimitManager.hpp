// ResourceLimitManager.hpp ---
//
// Filename: ResourceLimitManager.hpp
// Author: Abhishek Udupa
// Created: Wed Jan 15 14:49:32 2014 (-0500)
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


#if !defined ESMC_RESOURCE_LIMIT_MANAGER_HPP_
#define ESMC_RESOURCE_LIMIT_MANAGER_HPP_

#include <functional>
#include <signal.h>
#include <time.h>
#include <setjmp.h>
#include <vector>

#include "../common/ESMCFwdDecls.hpp"

namespace ESMC {

    class ResourceLimitManager
    {
    public:
        // constants
        static const u64 MemLimitDefault;
        static const u64 CPULimitDefault;
        static const u64 TimerIntervalDefault;

    private:
        static u64 MemLimit;
        static u64 CPULimit;
        static u64 TimerInterval;
        static struct sigaction OldAction;
        static bool TimerHandlerInstalled;
        static bool TimerCreated;
        static timer_t TimerID;
        static bool TimeOut;
        static bool MemOut;

        static vector<function<void(bool)>> OnLimitHandlers;

        // Hide all constructors
        ResourceLimitManager();
        ResourceLimitManager(const ResourceLimitManager& Other);

        // A private timer handler
        static void TimerHandler(int, siginfo_t*, void*);
        static void RegisterTimerHandler();
        static void UnregisterTimerHandler();

    public:
        static void SetMemLimit(u64 MemLimit);
        static u64 GetMemLimit();
        static void SetCPULimit(u64 TimeLimit);
        static u64 GetCPULimit();
        static void SetTimerInterval(u64 TimerIntervalNS);
        static u64 GetTimerFrequency();
        static bool QueryStart();
        static void QueryEnd();
        static bool CheckTimeOut();
        static bool CheckMemOut();
        static void AddOnLimitHandler(const function<void(bool)>& Handler);
        static void ClearOnLimitHandlers();

        static void GetUsage(double& TotalTime, double& PeakMem);
    };

} /* End namespace ESMC */

#endif /* ESMC_RESOURCE_LIMIT_MANAGER_HPP_ */

//
// ResourceLimitManager.hpp ends here
