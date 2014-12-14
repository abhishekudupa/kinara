// MSISynthOptions.hpp ---
// Filename: MSISynthOptions.hpp
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

// Common option handling code for MSISynth Benchmarks

#include <string>
#include <boost/program_options.hpp>

using namespace std;

#include "../../src/synth/Solver.hpp"

namespace po = boost::program_options;
using ESMC::Synth::GuardBoundingMethodT;
using ESMC::Synth::UpdateBoundingMethodT;
using ESMC::Synth::StateUpdateBoundingMethodT;
using ESMC::u64;
using ESMC::u32;

struct MSISynthOptionsT {
    GuardBoundingMethodT GBoundMethod;
    UpdateBoundingMethodT UBoundMethod;
    StateUpdateBoundingMethodT SBoundMethod;
    bool UnrollQuantifiers;
    bool NarrowDomains;
    u64 CPULimit;
    u64 MemLimit;
    u32 NumCExToProcess;
};

static inline void ParseOptions(int Argc, char* ArgV[], MSISynthOptionsT& Options)
{
    po::options_description Desc("Usage and Allowed Options");
    string GBoundMethodStr, UBoundMethodStr, SBoundMethodStr;
    u64 CPULimit;
    u64 MemLimit;
    u32 CExToProcess;

    Desc.add_options()
        ("help", "Produce this help message")
        ("gbound,g", po::value<string>(&GBoundMethodStr)->default_value("none"),
         "Method for bounding guards; one of: none, vardep, nonfalse, point")
        ("ubound,u", po::value<string>(&UBoundMethodStr)->default_value("none"),
         "Method for bounding updates; one of: none, vardep, nonid")
        ("sbound,s", po::value<string>(&SBoundMethodStr)->default_value("none"),
         "Method for bounding location updates; one of: none, allsame, vardep")
        ("narrow,n", "Use narrow domains for functions to be synthesized")
        ("quants,q", "Unroll Quantifiers before handing off to Z3")
        ("cex,c", po::value<u32>(&CExToProcess)->default_value(8),
         "Number of counterexamples to process on each model checking run")
        ("cpu-limit,t", po::value<u64>(&CPULimit)->default_value(UINT64_MAX),
         "CPU Time limit in seconds")
        ("mem-limit,m", po::value<u64>(&MemLimit)->default_value(UINT64_MAX),
         "Memory limit in MB");

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

    Options.UnrollQuantifiers = (vm.count("quants") > 0);
    Options.NarrowDomains = (vm.count("narrow") > 0);
    Options.CPULimit = CPULimit;
    Options.MemLimit = MemLimit;
    Options.NumCExToProcess = CExToProcess;

    return;
}

static inline void OptsToSolverOpts(const MSISynthOptionsT& Opts,
                                    ESMC::Synth::SolverOptionsT& SolverOpts)
{
    SolverOpts.GBoundMethod = Opts.GBoundMethod;
    SolverOpts.UBoundMethod = Opts.UBoundMethod;
    SolverOpts.SBoundMethod = Opts.SBoundMethod;
    SolverOpts.UnrollQuantifiers = Opts.UnrollQuantifiers;
    SolverOpts.CPULimitInSeconds = Opts.CPULimit;
    SolverOpts.MemLimitInMB = Opts.MemLimit;
    SolverOpts.NumCExToProcess = Opts.NumCExToProcess;
}

//
// MSISynthOptions.hpp ends here
