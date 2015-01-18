// LogManager.hpp ---
// Filename: LogManager.hpp
// Author: Abhishek Udupa
// Created: Sun Jan 18 16:14:58 2015 (-0500)
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

#if !defined ESMC_LOG_MANAGER_HPP_
#define ESMC_LOG_MANAGER_HPP_

#include <type_traits>

#include "../common/ESMCFwdDecls.hpp"

namespace ESMC {
    namespace Logging {

        class LogManager
        {
        private:
            static unordered_set<string> EnabledLogOptions;
            static ostream* LogStream;
            static bool IsInitialized;

            LogManager();

            static inline void AssertInitialized();

        public:
            LogManager(const LogManager& Other) = delete;
            LogManager(LogManager&& Other) = delete;

            static void Initialize(const string& LogStreamName = "", bool AppendMode = false);
            static void Finalize();

            static ostream& GetLogStream();
            static const unordered_set<string>& GetEnabledLogOptions();
            static bool IsOptionEnabled(const string& OptionName);
            template <typename T>
            static inline void Print(const T& Obj)
            {
            }
        };

    } /* end namespace Logging */
} /* end namespace ESMC */


// Inspired from Z3's tracing/logging mechanism

#define ESMC_LOG_CODE (CODE) { CODE } ((void)0)

#define ESMC_LOG_FULL (TAG, CODE) \
    EMSC_LOG_CODE(\
    if (ESMC::Logging::LogManager::IsOptionEnabled(TAG)) {\
        ostream& Out = ESMC::Logging::LogManager::GetLogStream();\
        Out <<

#endif /* ESMC_LOG_MANAGER_HPP_ */

//
// LogManager.hpp ends here
