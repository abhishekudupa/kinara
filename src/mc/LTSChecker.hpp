// LTSChecker.hpp ---
//
// Filename: LTSChecker.hpp
// Author: Abhishek Udupa
// Created: Sun Aug 17 17:32:54 2014 (-0400)
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

#if !defined ESMC_LTS_CHECKER_HPP_
#define ESMC_LTS_CHECKER_HPP_

#include "../common/ESMCFwdDecls.hpp"

#include "AQStructure.hpp"

namespace ESMC {
namespace MC {

using namespace ESMC::LTS;
using ESMC::Symm::Canonicalizer;
using namespace Detail;

class LTSChecker
{
    friend class Detail::FairnessChecker;
    friend class TraceBase;
    friend class Synth::Solver;

public:
    typedef unordered_map<const StateVec*, ExpT,
                          Detail::StateVecPtrHasher,
                          Detail::StateVecPtrEquals> ErrorStateSetT;
    typedef vector<pair<const StateVec*, ExpT>> ErrorStateVecT;

private:
    LabelledTS* TheLTS;
    StateFactory* Factory;
    Canonicalizer* TheCanonicalizer;
    StateVec* ZeroState;
    StateVecPrinter* Printer;
    LTSCompiler* Compiler;
    AQStructure* AQS;
    ProductStructure* ThePS;
    vector<GCmdRef> GuardedCommands;
    u32 NumGuardedCmds;
    u32 NumTentativeCmds;
    // Total number of fairness objects
    u32 NumFairnessObjects;
    // Total number of fairness sets
    u32 NumFairnessSets;
    // The system index set
    SystemIndexSet* SysIdxSet;
    // Fairness Checkers by class id
    vector<vector<Detail::FairnessChecker*>> FairnessCheckers;
    map<string, BuchiAutomatonBase*> AllBuchiAutomata;
    map<string, StateBuchiAutomaton*> StateBuchiAutomata;
    map<string, MsgBuchiAutomaton*> MsgBuchiAutomata;
    vector<string> BuchiMonitorNames;

    // Lowered invariant from the LTS
    ExpT LoweredInvariant;
    ExpT DeadlockFreeInvariant;
    ExpT LoweredDLFInvariant;

    // The set of all error states, mapping to the
    // invariant expression that was blown
    ErrorStateSetT ErrorStateSet;
    ErrorStateVecT ErrorStates;
    // A more fine grained classification for coverage
    // based prioritization
    unordered_map<u32, ErrorStateVecT> ErrorStateSetsByCmdID;

    // A set of commands that need to be tested
    // these are all the commands that are "fully"
    // interpreted, i.e., have been "unlocked" and
    // have a model supporting them
    set<u32> InterpretedCommands;

    // Fields that are reset on each call to BuildAQS
    float DesiredCoverage;
    u32 MaxNumErrors;
    BFSPrioMethodT PrioMethod;
    AQSConstructionMethod AQSConstMethod;
    PathFPPrioritizer* Prioritizer;
    PathFingerprintSet* PathFPSet;

    // returns <Cmd, false> if successful
    // returns <null, false> if no more commands
    // returns <null, true> if exception
    inline const GCmdRef& GetNextEnabledCmd(StateVec* State, i64& LastFired,
                                            bool& Exception, ExpT& NEPred);

    inline bool RecordErrorState(const StateVec* ErrorState,
                                 const ExpT& BlownInvariant);

    inline void BFSRecordErrorState(const StateVec* ErrorState,
                                    const ExpT& BlownInvariant,
                                    u32 TaintLevel,
                                    const PathFingerprint* FP,
                                    bool ExistingError);

    // Utilities to avoid complex if-then-else structures in
    // DoBFS
    inline void BFSInitQueue(BFSQueueT& Queue, const vector<StateVec*>& Roots,
                             const PathFingerprint* ZeroFP);

    inline void BFSPushQueue(BFSQueueT& Queue,
                             StateVec* State, u32 TaintLevel,
                             const PathFingerprint* PathFP);

    inline StateVec* BFSPopQueue(BFSQueueT& Queue, u32& TaintLevel,
                                 const PathFingerprint*& PathFP);

    inline bool BFSCheckInvariant(const StateVec* State, u32 TaintLevel,
                                  const PathFingerprint* PathFP);

    inline StateVec* BFSExecuteCommand(const StateVec* CurState, u32 TaintLevel,
                                       const PathFingerprint* PathFP, u32 CmdID,
                                       bool& Exception);

    inline void BFSGetNextTaintLevelAndPathFP(u32 CmdID, u32 CurrentTaintLevel,
                                              u32& NextTaintLevel,
                                              const PathFingerprint* CurrentFP,
                                              const PathFingerprint*& NextFP);

    inline bool BFSIsDone(u32 TaintLevel, const PathFingerprint* PathFP);



    inline void DoDFS(StateVec* Root);
    inline void DoBFS(const vector<StateVec*>& Roots);

    inline void ConstructProduct(StateBuchiAutomaton* Monitor);
    inline vector<const ProductState*>
    GetAcceptingSCCsFromInitState(const ProductState* InitState,
                                  u32& LowestUnusedIndex,
                                  u32& LowestUnusedSCCID);
    inline vector<const ProductState*> GetAcceptingSCCs();

    inline void DoThreadedBFS(const ProductState* SCCRoot,
                              u32 IndexID);

    inline bool CheckSCCFairness(const ProductState* SCCRoot,
                                 vector<const ProductState*>& UnfairStates);


public:
    LTSChecker(LabelledTS* TheLTS);
    virtual ~LTSChecker();

    // Returns true if no errors encountered
    // false otherwise
    bool BuildAQS(AQSConstructionMethod Method =
                  AQSConstructionMethod::BreadthFirst,
                  u32 MaxErrors = UINT32_MAX,
                  BFSPrioMethodT PrioMethod = BFSPrioMethodT::None,
                  float DesiredCoverage = FLT_MAX);

    void ClearAQS();

    StateBuchiAutomaton* MakeStateBuchiMonitor(const string& Name,
                                               const vector<ExpT>& SymmIndices,
                                               const ExpT& Constraint);

    TraceBase* CheckLiveness(const string& BuchiMonitorName);

    LabelledTS* GetLTS() const;
    LTSCompiler* GetCompiler() const;
    AQStructure* GetAQS() const;
    ProductStructure* GetPS() const;

    const ErrorStateVecT& GetAllErrorStates() const;
    const unordered_map<u32, ErrorStateVecT>& GetErrorStatesByCmdID() const;
    TraceBase* MakeTraceToError(const StateVec* ErrorState);
    const vector<string>& GetBuchiMonitorNames() const;
};

// Returns false if exception encountered
extern bool ApplyUpdates(const vector<LTSAssignRef>& Updates,
                         const StateVec* InputState,
                         StateVec* OutputState,
                         ExpT& NEPred);

// Returns null if exception encountered
extern StateVec* ExecuteCommand(const GCmdRef& Cmd,
                                const StateVec* InputState,
                                ExpT& NEPred);

// returns <null, true> if command cannot be executed
// returns <statevec, true> if command CAN be executed
// returns <null, false> if command CAN be executed
// but results in an exception when actually executing it
// or if an error was encountered evaluating the guard of Cmd
extern StateVec* TryExecuteCommand(const GCmdRef& Cmd,
                                   const StateVec* InputState,
                                   bool& Exception,
                                   ExpT& NEPred);

} /* end namespace MC */
} /* end namespace ESMC */


#endif /* ESMC_LTS_CHECKER_HPP_ */

//
// LTSChecker.hpp ends here
