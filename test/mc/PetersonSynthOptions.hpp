// PetersonSynthOptions.hpp ---
// Filename: PetersonSynthOptions.hpp
// Author: Abhishek Udupa
// Created: Wed Dec 10 23:11:10 2014 (-0500)
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

#include <string>
#include <boost/program_options.hpp>

using namespace std;

#include "../../src/synth/Solver.hpp"

namespace po = boost::program_options;
using ESMC::Synth::GuardBoundingMethodT;
using ESMC::Synth::UpdateBoundingMethodT;
using ESMC::Synth::StateUpdateBoundingMethodT;
using ESMC::LogFileCompressionTechniqueT;
using ESMC::u64;
using ESMC::u32;

struct PetersonSynthOptionsT {
    GuardBoundingMethodT GBoundMethod;
    UpdateBoundingMethodT UBoundMethod;
    StateUpdateBoundingMethodT SBoundMethod;
    bool UnrollQuantifiers;
    ESMC::MC::BFSPrioMethodT BFSPrioMethod;
    u64 CPULimit;
    u64 MemLimit;
    float CoverageDesired;
    u32 BoundLimit;
    string LogFileName;
    vector<string> LogOptions;
    LogFileCompressionTechniqueT LogCompressionTechnique;
    bool RemoveL1L2FlagUpdate;
    bool RemoveCriticalL1FlagUpdate;
    bool RemoveL3CriticalGuard;
};

static inline void ParseOptions(int Argc, char* ArgV[], PetersonSynthOptionsT& Options)
{
    po::options_description Desc("Usage and Allowed Options");
    string GBoundMethodStr, UBoundMethodStr, SBoundMethodStr;
    u64 CPULimit;
    u64 MemLimit;
    float CoverageDesired;
    string BFSPrioMethodStr;
    u32 BoundLimit;
    auto&& LogOptionsDesc = ESMC::Logging::LogManager::GetLogOptions();
    vector<string> LogOptions;
    string LogFileName;
    string LogCompressionTechnique;

    Desc.add_options()
        ("help", "Produce this help message")
        ("no-state", "Fix the next control state during synthesis")
        ("gbound,g", po::value<string>(&GBoundMethodStr)->default_value("none"),
         "Method for bounding guards; one of: none, vardep, nonfalse, point")
        ("ubound,u", po::value<string>(&UBoundMethodStr)->default_value("none"),
         "Method for bounding updates; one of: none, vardep, nonid, point")
        ("sbound,s", po::value<string>(&SBoundMethodStr)->default_value("none"),
         "Method for bounding location updates; one of: none, allsame, vardep")
        ("narrow,n", "Use narrow domains for functions to be synthesized")
        ("quants,q", "Unroll Quantifiers before handing off to Z3")
        ("prioritization-method,p", po::value<string>(&BFSPrioMethodStr)->default_value("none"),
         "Prioritization method used in model checking, one of: none, simple, coverage")
        ("coverage,c", po::value<float>(&CoverageDesired)->default_value(1.0f),
         ((string)"Amount of coverage to give each tentative edge, if prioritization mode " +
          "is set to \"coverage\", otherwise, number of counterexamples to process in each run.").c_str())

        ("bound,b", po::value<u32>(&BoundLimit)->default_value(256),
         "Max limit on bound")
        ("cpu-limit,t", po::value<u64>(&CPULimit)->default_value(UINT64_MAX),
         "CPU Time limit in seconds")
        ("mem-limit,m", po::value<u64>(&MemLimit)->default_value(UINT64_MAX),
         "Memory limit in MB")
        ("log-file", po::value<string>(&LogFileName)->default_value(""),
         "Name of file to write logging info into, defaults to stdout")
        ("log-compression", po::value<string>(&LogCompressionTechnique)->default_value("none"),
         "Compression option for log file; one of: none, gzip, bzip2")
        ("log-opts", po::value<vector<string>>(&LogOptions)->multitoken(),
         ((string)"Logging Options to enable\n" + LogOptionsDesc).c_str())
        ("l1l2flag",
         "Remove update expression of flag[pid] from l1 to l2 transition")
        ("criticall1flag",
         "Remove update expression of flag[pid] from critical to l1 transition")
        ("l3criticalguard",
         "Remove guards from l3 to critical transition, l3 to l3, but set them to be one the negation of the other");

    po::variables_map vm;

    po::store(po::command_line_parser(Argc, ArgV).options(Desc).run(), vm);
    po::notify(vm);

    if (vm.count("help") > 0) {
        cout << Desc << endl;
        exit(1);
    }

    if (GBoundMethodStr == "vardep") {
        Options.GBoundMethod = GuardBoundingMethodT::VarDepBound;
    } else if (GBoundMethodStr == "nonfalse") {
        Options.GBoundMethod = GuardBoundingMethodT::NonFalseBound;
    } else if (GBoundMethodStr == "point") {
        Options.GBoundMethod = GuardBoundingMethodT::PointBound;
    } else if (GBoundMethodStr == "none") {
        Options.GBoundMethod = GuardBoundingMethodT::NoBounding;
    } else {
        cout << Desc << endl;
        exit(1);
    }

    if (UBoundMethodStr == "vardep") {
        Options.UBoundMethod = UpdateBoundingMethodT::VarDepBound;
    } else if (UBoundMethodStr == "nonid") {
        Options.UBoundMethod = UpdateBoundingMethodT::NonIdentityBound;
    } else if (UBoundMethodStr == "point") {
        Options.UBoundMethod = UpdateBoundingMethodT::PointBound;
    } else if (UBoundMethodStr == "none") {
        Options.UBoundMethod = UpdateBoundingMethodT::NoBounding;
    } else {
        cout << Desc << endl;
        exit(1);
    }

    if (SBoundMethodStr == "vardep") {
        Options.SBoundMethod = StateUpdateBoundingMethodT::VarDepBound;
    } else if (SBoundMethodStr == "allsame") {
        Options.SBoundMethod = StateUpdateBoundingMethodT::AllSame;
    } else if (SBoundMethodStr == "none") {
        Options.SBoundMethod = StateUpdateBoundingMethodT::NoBounding;
    } else {
        cout << Desc << endl;
        exit(1);
    }

    if (LogCompressionTechnique == "none") {
        Options.LogCompressionTechnique = LogFileCompressionTechniqueT::COMPRESS_NONE;
    } else if (LogCompressionTechnique == "bzip2") {
        Options.LogCompressionTechnique = LogFileCompressionTechniqueT::COMPRESS_BZIP2;
    } else if (LogCompressionTechnique == "gzip") {
        Options.LogCompressionTechnique = LogFileCompressionTechniqueT::COMPRESS_GZIP;
    } else {
        cout << "Invalid log file compression method" << endl;
        cout << Desc << endl;
        exit(1);
    }

    Options.UnrollQuantifiers = (vm.count("quants") > 0);
    Options.LogFileName = LogFileName;
    Options.LogOptions = LogOptions;
    Options.CPULimit = CPULimit;
    Options.MemLimit = MemLimit;
    Options.BoundLimit = BoundLimit;

    Options.RemoveL1L2FlagUpdate = (vm.count("l1l2flag") > 0);
    Options.RemoveCriticalL1FlagUpdate = (vm.count("criticall1flag") > 0);
    Options.RemoveL3CriticalGuard = (vm.count("l3criticalguard") > 0);

    return;
}

static inline void OptsToSolverOpts(const PetersonSynthOptionsT& Opts,
                                    ESMC::Synth::SolverOptionsT& SolverOpts)
{
    SolverOpts.GBoundMethod = Opts.GBoundMethod;
    SolverOpts.UBoundMethod = Opts.UBoundMethod;
    SolverOpts.SBoundMethod = Opts.SBoundMethod;
    SolverOpts.UnrollQuantifiers = Opts.UnrollQuantifiers;
    SolverOpts.CPULimitInSeconds = Opts.CPULimit;
    SolverOpts.MemLimitInMB = Opts.MemLimit;
    SolverOpts.DesiredCoverage = Opts.CoverageDesired;
    SolverOpts.BoundLimit = Opts.BoundLimit;
}

static inline void OptsToLibOpts(const PetersonSynthOptionsT& Opts,
                                 ESMC::ESMCLibOptionsT& LibOpts)
{
    LibOpts.LogFileName = Opts.LogFileName;
    LibOpts.LogCompressionTechnique = Opts.LogCompressionTechnique;
    LibOpts.LoggingOptions = set<string>(Opts.LogOptions.begin(),
                                         Opts.LogOptions.end());
}

//
// PetersonSynthOptions.hpp ends here
