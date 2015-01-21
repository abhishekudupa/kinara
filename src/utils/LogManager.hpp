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
#include <unordered_set>
#include <map>
#include <vector>

#include "../common/ESMCFwdDecls.hpp"

namespace ESMC {
    namespace Logging {

        class LogManager
        {
        private:
            static unordered_set<string> EnabledLogOptions;
            static ostream* LogStream;
            static bool IsInitialized;
            static const map<string, string> LogOptionDescriptions;
            static bool NoLoggingEnabled;

            LogManager();

            static inline void AssertInitialized();

        public:
            LogManager(const LogManager& Other) = delete;
            LogManager(LogManager&& Other) = delete;

            static void Initialize(const string& LogStreamName = "",
                                   bool CompressedStream = false,
                                   bool AppendMode = false);
            static void Finalize();

            static ostream& GetLogStream();
            static void EnableLogOption(const string& OptionName);
            static void EnableLogOptions(const vector<string>& OptionNames);
            template <typename ForwardIterator>
            static inline void EnableLogOptions(const ForwardIterator& First,
                                                const ForwardIterator& Last);
            static void DisableLogOption(const string& OptionName);
            static const unordered_set<string>& GetEnabledLogOptions();
            static bool IsOptionEnabled(const string& OptionName);
            static bool IsLoggingDisabled();
            static string GetLogOptions();
        };

        template <typename ForwardIterator>
        inline void LogManager::EnableLogOptions(const ForwardIterator& First,
                                                 const ForwardIterator& Last)
        {
            for (auto it = First; it != Last; ++it) {
                EnableLogOption(*it);
            }
        }

    } /* end namespace Logging */
} /* end namespace ESMC */


// Inspired from Z3's tracing/logging mechanism

#ifdef ESMC_ENABLE_TRACING_
#define ESMC_LOG_CODE(CODE_) { CODE_ } ((void)0)
#else
#define ESMC_LOG_CODE(CODE_) ((void)0)
#endif /* ESMC_ENABLE_TRACING_ */

#define ESMC_LOG_FULL(TAG_, CODE_) \
    ESMC_LOG_CODE(if (ESMC::Logging::LogManager::IsOptionEnabled(TAG_)) { \
        ostream& Out_ = ESMC::Logging::LogManager::GetLogStream();\
        Out_ << "------------- [" << TAG_ << "], at " << __FUNCTION__ << ", "\
             << __FILE__ << ":" << __LINE__ << " -------------" << endl; \
        CODE_ \
        Out_ << "-----------------------------------------------------------" \
             << "--------------------" << endl;                         \
        Out_.flush(); \
    })

#define ESMC_LOG_SHORT(TAG_, CODE_) \
    ESMC_LOG_CODE(\
    if (ESMC::Logging::LogManager::IsOptionEnabled(TAG_)) {\
        ostream& Out_ = ESMC::Logging::LogManager::GetLogStream();  \
        CODE_ \
        ESMC::Logging::LogManager::GetLogStream().flush(); \
    })

#define ESMC_LOG_COND_FULL(TAG_, CODE_, COND_) \
    ESMC_LOG_CODE(\
    if (ESMC::Logging::LogManager::IsOptionEnabled(TAG_) && (COND_)) {\
        ostream& Out_ = ESMC::Logging::LogManager::GetLogStream();\
        Out_ << "------------- [" << TAG_ << "], at " << __FUNCTION__ << ", "\
             << __FILE__ << ":" << __LINE__ << " -------------" << endl; \
        CODE_ \
        Out_ << "-----------------------------------------------------------" \
             << "--------------------" << endl;                         \
        Out_.flush(); \
    })

#define ESMC_LOG_COND_SHORT(TAG_, CODE_, COND_)      \
    ESMC_LOG_CODE(\
    if (ESMC::Logging::LogManager::IsOptionEnabled(TAG_) && (COND_)) {  \
        ostream& Out_ = ESMC::Logging::LogManager::GetLogStream();\
        CODE_ \
        Out_.flush(); \
    })

#define ESMC_LOG_MIN_FULL(CODE_) \
    if (!ESMC::Logging::LogManager::IsLoggingDisabled()) {\
        ostream& Out_ = ESMC::Logging::LogManager::GetLogStream();\
        Out_ << "------------- [" << TAG << "], at " << __FUNCTION__ << ", "\
             << __FILE__ << ":" << __LINE__ << " -------------" << endl; \
        CODE_ \
        Out_ << "-----------------------------------------------------------" \
             << "--------------------" << endl;                         \
        Out_.flush(); \
    }\
    ((void)0)

#define ESMC_LOG_MIN_SHORT(CODE_) \
    if (!ESMC::Logging::LogManager::IsLoggingDisabled()) {\
        ostream& Out_ = ESMC::Logging::LogManager::GetLogStream();\
        CODE_ \
        Out_.flush(); \
    }\
    ((void)0)


#endif /* ESMC_LOG_MANAGER_HPP_ */

//
// LogManager.hpp ends here
