// LogManager.cpp ---
// Filename: LogManager.cpp
// Author: Abhishek Udupa
// Created: Sun Jan 18 16:34:01 2015 (-0500)
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

#include <stdlib.h>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/bzip2.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "LogManager.hpp"

namespace ESMC {
    namespace Logging {

        const map<string, string> LogManager::LogOptionDescriptions =
            {
                {
                    "Solver.Traces",
                    "Print error traces for counterexamples obtained by the solver."
                },
                {
                    "Solver.Purification",
                    (string)"Print information about expression purification performed " +
                    "by the solver whenever applicable."
                },
                {
                    "Solver.CEXAssertions",
                    (string)"Print information about assertions obtained by " +
                    "analysis of counterexamples in the solver."
                },
                {
                    "Solver.Duplicates",
                    (string)"Print information about duplicate assertions when " +
                    "encountered in the solver."
                },
                {
                    "Solver.OtherAssertions",
                    (string)"Print information about determinism and symmetry " +
                    "assertions asserted by the solver."
                },
                {
                    "Solver.Models",
                    (string)"Print model obtained from Z3 in each iteration in the solver."
                },
                {
                    "IncompleteEFSM.Assertions",
                    (string)"Print assertions made in during processing of IncompleteEFSMs."
                },
                {
                    "TheoremProver.Unrolled",
                    (string)"Print unrolled assertions in the theorem prover as they are " +
                    "generated."
                },
                {
                    "TheoremProver.RandomSeed",
                    (string)"Print random seed used by the theorem prover (if any)."
                },
                {
                    "TheoremProver.Assertions",
                    (string)"Print assertions as they are asserted into the theorem prover."
                },
                {
                    "Simplifier.PerIteration",
                    "Print progress in each iteration of simplifier."
                },
                {
                    "Trace.Generation",
                    "Print unwinding process in trace generation."
                },
                {
                    "Checker.Fairness",
                    (string)"Print information from fairness checkers " +
                    "during the check for liveness properties."
                },
                {
                    "Checker.AQSDetailed",
                    (string)"Print detailed information during AQS construction " +
                    "in the model checker."
                },
                {
                    "Checker.Stats",
                    (string)"Print statistics from AQS construction."
                },
                {
                    "Checker.GuardedCommands",
                    (string)"Print the guarded commands that the checker starts out with."
                },
                {
                    "Analyses.Detailed",
                    (string)"Track detailed progress in weakest pre analyses."
                },
                {
                    "Canonicalizer.Detailed",
                    (string)"Print detailed information about canonicalization steps."
                },
                {
                    "Canonicalizer.Best",
                    (string)"Print info about canonicalized state for each state."
                },
                {
                    "Z3.PerIterationStats",
                    (string)"Print statistics from Z3 after each solve."
                },
                {
                    "ESMC.Minimal",
                    (string)"Bare minimal trace output from model checker and solver, " +
                    "serving only to indicate progress. Disabling this will cause NOTHING " +
                    "to ever be printed on the trace stream by the ESMC libraries. This " +
                    "is option is always enabled, unless disabled by ESMC.None below. " +
                    "Further, this option is enabled, even if the ESMC libraries have been " +
                    "built without -DESMC_ENABLE_TRACING_ set."
                },
                {
                    "ESMC.None",
                    (string)"Turns of ALL tracing options (including ESMC.Minimal)."
                },
                {
                    "ESMC.All",
                    (string)"Turns ON ALL tracing options."
                }
            };

        unordered_set<string>& LogManager::EnabledLogOptions()
        {
            static unordered_set<string> EnabledLogOptions_;
            return EnabledLogOptions_;
        }

        ostream*& LogManager::LogStream()
        {
            static ostream* LogStream_ = nullptr;
            return LogStream_;
        }

        bool& LogManager::NoLoggingEnabled()
        {
            static bool NoLoggingEnabled_ = false;
            return NoLoggingEnabled_;
        }

        LogManager::LogManager()
        {
            // Nothing here
        }

        void LogManager::Initialize(const string& LogStreamName,
                                    LogFileCompressionTechniqueT LogCompressionTechnique)
        {
            Finalize();

            if (LogStreamName == "") {
                LogStream() = &std::cout;
            } else {
                auto LogStreamFileName = LogStreamName;
                auto LocalLogStream = new boost::iostreams::filtering_ostream();
                if (LogCompressionTechnique == LogFileCompressionTechniqueT::COMPRESS_BZIP2) {
                    LocalLogStream->push(boost::iostreams::bzip2_compressor(9));
                    if (!boost::algorithm::ends_with(LogStreamFileName, ".bz2")) {
                        LogStreamFileName = LogStreamFileName + ".bz2";
                    }
                } else if (LogCompressionTechnique == LogFileCompressionTechniqueT::COMPRESS_GZIP) {
                    LocalLogStream->push(boost::iostreams::gzip_compressor(9));
                    if (!boost::algorithm::ends_with(LogStreamFileName, ".gz")) {
                        LogStreamFileName += ".gz";
                    }
                }

                auto OpenFlags = ios_base::out;
                if (LogCompressionTechnique != LogFileCompressionTechniqueT::COMPRESS_NONE) {
                    OpenFlags = OpenFlags | ios_base::binary;
                }

                LocalLogStream->push(boost::iostreams::file_sink(LogStreamFileName, OpenFlags));
                LogStream() = LocalLogStream;
            }
        }

        void LogManager::Finalize()
        {
            if (LogStream() != nullptr && LogStream() != &cout) {
                dynamic_cast<ofstream*>(LogStream())->close();
                delete LogStream();
                LogStream() = nullptr;
            } else if (LogStream() != nullptr) {
                LogStream()->flush();
            }
            LogStream() = nullptr;
            EnabledLogOptions().clear();
        }

        ostream& LogManager::GetLogStream()
        {
            return *(LogStream());
        }

        void LogManager::EnableLogOption(const string& OptionName)
        {
            if (LogOptionDescriptions.find(OptionName) ==
                LogOptionDescriptions.end()) {
                throw ESMCError((string)"Log option \"" + OptionName + "\" is not " +
                                "a recognized log option.\nIn call to " + __FUNCTION__ +
                                " at " + __FILE__ + ":" + to_string(__LINE__));
            }

            if (OptionName == "ESMC.All") {
                NoLoggingEnabled() = false;
                for (auto const& OptionDesc : LogOptionDescriptions) {
                    if (OptionDesc.first != "ESMC.None") {
                        EnabledLogOptions().insert(OptionName);
                    }
                }
            } else if (OptionName == "ESMC.None") {
                NoLoggingEnabled() = true;
                EnabledLogOptions().clear();
            } else {
                NoLoggingEnabled() = false;
                EnabledLogOptions().insert(OptionName);
            }
        }

        void LogManager::EnableLogOptions(const vector<string>& OptionNames)
        {
            EnableLogOptions(OptionNames.begin(), OptionNames.end());
        }

        void LogManager::DisableLogOption(const string& OptionName)
        {
            if (LogOptionDescriptions.find(OptionName) ==
                LogOptionDescriptions.end()) {
                throw ESMCError((string)"Log option \"" + OptionName + "\" is not " +
                                "a recognized log option.\nIn call to " + __FUNCTION__ +
                                " at " + __FILE__ + ":" + to_string(__LINE__));
            }
            EnabledLogOptions().erase(OptionName);
        }

        const unordered_set<string>& LogManager::GetEnabledLogOptions()
        {
            return EnabledLogOptions();
        }

        bool LogManager::IsOptionEnabled(const string& OptionName)
        {
            return (!NoLoggingEnabled() && (EnabledLogOptions().find(OptionName) !=
                                            EnabledLogOptions().end()));
        }

        bool LogManager::IsLoggingDisabled()
        {
            return NoLoggingEnabled();
        }

        string LogManager::GetLogOptions()
        {
            ostringstream sstr;
            sstr << "Available logging options:" << endl;
            for (auto const& Option : LogOptionDescriptions) {
                sstr << left << setw(24) << setfill(' ') << Option.first
                     << ": " << Option.second << endl;
            }
            return sstr.str();
        }

    } /* end namespace Logging */
} /* end namespace ESMC */

//
// LogManager.cpp ends here
