// Trace.cpp ---
//
// Filename: Trace.cpp
// Author: Abhishek Udupa
// Created: Tue Sep  9 13:34:49 2014 (-0400)
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

#include <algorithm>
#include <sparse_hash_set>

#include "../uflts/LTSTransitions.hpp"
#include "../uflts/LabelledTS.hpp"
#include "../uflts/LTSFairnessSet.hpp"
#include "../uflts/LTSAutomaton.hpp"
#include "../uflts/LTSEFSMBase.hpp"
#include "../uflts/LTSAssign.hpp"
#include "../uflts/LTSTransformers.hpp"
#include "../symmetry/SymmCanonicalizer.hpp"

#include "StateVec.hpp"
#include "StateVecPrinter.hpp"
#include "Trace.hpp"
#include "AQStructure.hpp"
#include "LTSChecker.hpp"
#include "StateVecPrinter.hpp"
#include "IndexSet.hpp"
#include "OmegaAutomaton.hpp"

namespace ESMC {
    namespace MC {

        using LTS::LabelledTS;
        using Symm::Canonicalizer;
        using LTS::ExpT;

        TraceBase::TraceBase(StateVecPrinter* Printer)
            : Printer(Printer)
        {
            // Nothing here
        }

        TraceBase::~TraceBase()
        {
            // Nothing here
        }

        StateVecPrinter* TraceBase::GetPrinter() const
        {
            return Printer;
        }

        inline const StateVec*
        TraceBase::UnwindPermPath(AQSPermPath* PermPath,
                                  LTSChecker* Checker,
                                  vector<TraceElemT>& PathElems,
                                  u32& InvPermAlongPathOut)
        {
            auto TheLTS = Checker->TheLTS;
            auto TheCanonicalizer = Checker->TheCanonicalizer;
            // auto Printer = Checker->Printer;

            auto const& GuardedCommands = TheLTS->GetGuardedCmds();
            auto CurUnwoundState = PermPath->GetOrigin()->Clone();
            auto UnwoundOrigin = CurUnwoundState;
            auto const& PPathElems = PermPath->GetPathElems();
            // identity permutation = 0
            u32 InvPermAlongPath = 0;

            auto PermSet = TheCanonicalizer->GetPermSet();

            for (auto Edge : PPathElems) {
                auto CurInvPermIt = PermSet->GetIteratorForInv(Edge->GetPermutation());
                auto InvPermAlongPathIt = PermSet->Compose(InvPermAlongPath,
                                                           CurInvPermIt.GetIndex());
                InvPermAlongPath = InvPermAlongPathIt.GetIndex();
                auto NextPermState = Edge->GetTarget();

                // cout << "Permuted State:" << endl;
                // cout << "-------------------------------------------" << endl;
                // Printer->PrintState(NextPermState, cout);
                // cout << "-------------------------------------------" << endl;

                auto NextUnwoundState = TheCanonicalizer->ApplyPermutation(NextPermState,
                                                                           InvPermAlongPath);
                u32 Dummy;
                TheCanonicalizer->Sort(NextUnwoundState);
                // cout << "Unwound State:" << endl;
                // cout << "-------------------------------------------" << endl;
                // Printer->PrintState(NextUnwoundState, cout);
                // cout << "-------------------------------------------" << endl;

                // Now find out which command takes us from the current
                // unwound state to the next unwound state
                StateVec* NextUnsortedState = nullptr;
                bool FoundCmd = false;
                for (auto const& Cmd : GuardedCommands) {

                    // Ignore the exception here
                    bool Exception = false;
                    ExpT NEPred;
                    auto CandidateState = TryExecuteCommand(Cmd, CurUnwoundState, Exception, NEPred);
                    if (CandidateState == nullptr) {
                        continue;
                    }
                    auto SortedCandidateState = TheCanonicalizer->SortChans(CandidateState,
                                                                            false, Dummy);

                    if (SortedCandidateState->Equals(*NextUnwoundState)) {
                        NextUnsortedState = CandidateState;
                        FoundCmd = true;
                        SortedCandidateState->Recycle();
                        PathElems.push_back(TraceElemT(Cmd, NextUnsortedState));
                        NextUnwoundState->Recycle();
                        break;
                    } else {
                        SortedCandidateState->Recycle();
                        CandidateState->Recycle();
                    }
                }
                if (!FoundCmd) {
                    throw InternalError((string)"Could not find a command to compute the " +
                                        "next unwound state\nAt: " + __FILE__ + ":" +
                                        to_string(__LINE__));
                }

                // Do not recycle the unwound state
                // it's part of the return value
                CurUnwoundState = NextUnsortedState;
            }

            InvPermAlongPathOut = InvPermAlongPath;
            return UnwoundOrigin;
        }

        inline const StateVec*
        TraceBase::UnwindPermPath(AQSPermPath* PermPath,
                                  LTSChecker* Checker,
                                  vector<TraceElemT>& PathElems)
        {
            u32 Unused = 0;
            return UnwindPermPath(PermPath, Checker, PathElems, Unused);
        }

        // InvPermAlongPathOut is only a return value
        inline const ProductState*
        TraceBase::UnwindPermPath(PSPermPath* PermPath, LTSChecker* Checker,
                                  vector<PSTraceElemT>& PathElems,
                                  u32& InvPermAlongPathOut)
        {
            auto TheCanonicalizer = Checker->TheCanonicalizer;
            auto const& GuardedCommands = Checker->GuardedCommands;
            auto Origin = PermPath->GetOrigin();
            auto ThePS = Checker->ThePS;
            auto Monitor = ThePS->GetMonitor();
            auto ProcIdxSet = Monitor->GetIndexSet();

            auto InvPermAlongPath = 0;
            auto CurUnwoundState = TheCanonicalizer->ApplyPermutation(Origin,
                                                                      InvPermAlongPath,
                                                                      ProcIdxSet);
            auto UnwoundOrigin = CurUnwoundState;
            auto const& PPathElems = PermPath->GetPathElems();
            auto PermSet = TheCanonicalizer->GetPermSet();

            for (auto Edge : PPathElems) {
                auto CurInvPermIt = PermSet->GetIteratorForInv(Edge->GetPermutation());
                auto InvPermAlongPathIt = PermSet->Compose(InvPermAlongPath,
                                                           CurInvPermIt.GetIndex());
                InvPermAlongPath = InvPermAlongPathIt.GetIndex();
                auto NextPermPS = Edge->GetTarget();

                auto NextUnwoundPS = TheCanonicalizer->ApplyPermutation(NextPermPS,
                                                                        InvPermAlongPath,
                                                                        ProcIdxSet);
                TheCanonicalizer->Sort(NextUnwoundPS);
                // find out which command takes us here
                bool FoundCmd = false;
                ProductState* NextUnsortedPS = nullptr;

                for (auto const& Cmd : GuardedCommands) {
                    // we ignore the exception here
                    bool Exception = false;
                    ExpT NEPred;
                    auto CandidateSV = TryExecuteCommand(Cmd, CurUnwoundState->GetSVPtr(),
                                                         Exception, NEPred);
                    if (CandidateSV == nullptr) {
                        continue;
                    }

                    u32 Dummy;
                    auto SortedCandidateSV = TheCanonicalizer->SortChans(CandidateSV, false,
                                                                         Dummy);

                    if (SortedCandidateSV->Equals(*(NextUnwoundPS->GetSVPtr()))) {
                        NextUnsortedPS = new ProductState(CandidateSV,
                                                          NextUnwoundPS->GetMonitorState(),
                                                          NextUnwoundPS->GetIndexID(), 0);
                        FoundCmd = true;
                        SortedCandidateSV->Recycle();
                        PathElems.push_back(PSTraceElemT(Cmd, NextUnsortedPS));
                        NextUnwoundPS->GetSVPtr()->Recycle();
                        delete NextUnwoundPS;
                        break;
                    } else {
                        SortedCandidateSV->Recycle();
                        CandidateSV->Recycle();
                    }
                }

                if (!FoundCmd) {
                    throw InternalError((string)"Unable to find a command to compute next " +
                                        "unwound product state.\nAt: " + __FILE__ + ":" +
                                        to_string(__LINE__));
                }

                CurUnwoundState = NextUnsortedPS;
            }

            InvPermAlongPathOut = InvPermAlongPath;
            return UnwoundOrigin;
        }

        // static methods for liveness traces
        inline unordered_set<const ProductState*>
        TraceBase::ExpandSCC(const ProductState* SCCRoot, LTSChecker* Checker)
        {
            auto ThePS = Checker->ThePS;

            // A simple BFS to get all the scc nodes
            unordered_set<const ProductState*> SCCNodes;
            deque<const ProductState*> BFSQueue;
            auto SCCID = SCCRoot->Status.InSCC;

            BFSQueue.push_back(SCCRoot);
            SCCNodes.insert(SCCRoot);

            while (BFSQueue.size() > 0) {
                auto CurNode = BFSQueue.front();
                BFSQueue.pop_front();

                auto const& Edges = ThePS->GetEdges(const_cast<ProductState*>(CurNode));

                for (auto Edge : Edges) {
                    auto Target = Edge->GetTarget();
                    if (Target->IsInSCC(SCCID) &&
                        SCCNodes.find(Target) == SCCNodes.end()) {
                        SCCNodes.insert(Target);
                        BFSQueue.push_back(Target);
                    }
                }
            }

            return SCCNodes;
        }

        inline const ProductState*
        TraceBase::DoUnwoundBFS(const ProductState* CanonicalRoot,
                                const LTSChecker* Checker,
                                u32& InvPermAlongPathOut,
                                u32& InvSortPermForRoot,
                                const function<bool(u32, const ProductState*)>& MatchPred,
                                vector<PSTraceElemT>& PathElems,
                                const unordered_set<const ProductState*>& Bounds)
        {
            auto ThePS = Checker->ThePS;
            auto TheCanonicalizer = Checker->TheCanonicalizer;
            auto PermSet = TheCanonicalizer->GetPermSet();
            auto SortPermSet = TheCanonicalizer->GetSortPermSet();
            auto Monitor = ThePS->GetMonitor();
            auto ProcIdxSet = Monitor->GetIndexSet();
            auto const& GuardedCmds = Checker->GuardedCommands;
            const u32 NumGuardedCmds = GuardedCmds.size();

            deque<Detail::BFSQueueEntryT> BFSQueue;
            Detail::PSPermSetT VisitedStates;
            Detail::UnwoundPredMapT PredMap;

            auto InvPermAlongPath = InvPermAlongPathOut;
            auto OriginPair = make_pair(CanonicalRoot, InvPermAlongPath);
            auto OriginTuple = make_tuple(CanonicalRoot, InvPermAlongPath, InvSortPermForRoot);
            VisitedStates.insert(OriginPair);
            BFSQueue.push_back(OriginTuple);

            bool ReachedTarget = false;
            Detail::BFSQueueEntryT TargetPSPerm;

            while (BFSQueue.size() > 0 && !ReachedTarget) {
                auto CurTuple = BFSQueue.front();
                BFSQueue.pop_front();

                auto CurPermPS = get<0>(CurTuple);
                auto CurPermIndex = get<1>(CurTuple);
                auto CurSortPermIdx = get<2>(CurTuple);

                auto CurUnsortedPS = TheCanonicalizer->ApplyPermutation(CurPermPS,
                                                                        CurPermIndex,
                                                                        ProcIdxSet);
                TheCanonicalizer->Sort(CurUnsortedPS);
                TheCanonicalizer->ApplySort(CurUnsortedPS, CurSortPermIdx);

                auto const& Edges = ThePS->GetEdges(const_cast<ProductState*>(CurPermPS));

                for (auto Edge : Edges) {
                    auto NextPermPS = Edge->GetTarget();

                    if (Bounds.find(NextPermPS) == Bounds.end()) {
                        // Not part of the scc, ignore
                        continue;
                    }

                    auto EdgePermIndex = Edge->GetPermutation();
                    auto EdgeInvPermIt = PermSet->GetIteratorForInv(EdgePermIndex);
                    auto NextPermIt = PermSet->Compose(CurPermIndex, EdgeInvPermIt.GetIndex());
                    auto NextPermIndex = NextPermIt.GetIndex();

                    auto NextPair = make_pair(NextPermPS, NextPermIndex);
                    if (VisitedStates.find(NextPair) != VisitedStates.end()) {
                        // Already visited
                        continue;
                    }

                    VisitedStates.insert(NextPair);
                    auto NextUnwoundPS = TheCanonicalizer->ApplyPermutation(NextPermPS,
                                                                            NextPermIndex,
                                                                            ProcIdxSet);
                    u32 Dummy;
                    auto NextSortedSV = TheCanonicalizer->SortChans(NextUnwoundPS->GetSVPtr(),
                                                                    false, Dummy);

                    // Find out which command takes us from the current
                    // unwound product state to the next unwound product
                    // state
                    bool FoundCmd = false;
                    for (u32 i = 0; i < NumGuardedCmds; ++i) {
                        auto const& Cmd = GuardedCmds[i];

                        // We ignore the exception here
                        bool Exception = false;
                        ExpT NEPred;
                        auto CandidateSV = TryExecuteCommand(Cmd, CurUnsortedPS->GetSVPtr(),
                                                             Exception, NEPred);
                        if (CandidateSV == nullptr) {
                            continue;
                        }

                        u32 SortPermIdx;
                        auto SortedCandidateSV = TheCanonicalizer->SortChans(CandidateSV, true,
                                                                             SortPermIdx);
                        if (SortedCandidateSV->Equals(*NextSortedSV)) {
                            FoundCmd = true;
                            // SortedCandidateSV->Recycle();
                            auto InvSortPermIdx =
                                SortPermSet->GetIteratorForInv(SortPermIdx).GetIndex();
                            // I need to apply the inverse permutation first,
                            // followed by a sort, followd by the inverse sort
                            // to get my true predecessor

                            auto NextUnsortedPS = new ProductState(CandidateSV,
                                                                   NextUnwoundPS->GetMonitorState(),
                                                                   NextUnwoundPS->GetIndexID(), 0);

                            SortedCandidateSV->Recycle();

                            auto NextTuple = make_tuple(NextPermPS, NextPermIndex,
                                                        InvSortPermIdx);
                            PredMap[NextTuple] = make_pair(i, CurTuple);
                            BFSQueue.push_back(NextTuple);

                            if (MatchPred(i, NextUnsortedPS)) {
                                ReachedTarget = true;
                                TargetPSPerm = NextTuple;
                            }

                            // delete the next unsorted ps
                            CandidateSV->Recycle();
                            delete NextUnsortedPS;
                            break;
                        } else {
                            SortedCandidateSV->Recycle();
                            CandidateSV->Recycle();
                        }
                    } // End iterating over commands

                    // Delete the unwound product state, we'll recreate it
                    // later anyway
                    NextUnwoundPS->GetSVPtr()->Recycle();
                    delete NextUnwoundPS;

                    if (!FoundCmd) {
                        throw InternalError((string)"Unable to find a command to compute next " +
                                            "unwound product state.\nAt: " + __FILE__ + ":" +
                                            to_string(__LINE__));
                    }

                    if (ReachedTarget) {
                        break;
                    }
                } // End iterating over edges

                // Delete the current unwound product state as well
                CurUnsortedPS->GetSVPtr()->Recycle();
                delete CurUnsortedPS;
            } // End BFS

            // We now need to assemble a path from the predecessors
            auto CurTuple = TargetPSPerm;
            auto it = PredMap.find(CurTuple);
            auto CurPermPS = get<0>(CurTuple);
            auto CurPerm = get<1>(CurTuple);
            auto CurSortPerm = get<2>(CurTuple);

            auto CurUnsortedPS = TheCanonicalizer->ApplyPermutation(CurPermPS, CurPerm, ProcIdxSet);
            TheCanonicalizer->Sort(CurUnsortedPS);
            TheCanonicalizer->ApplySort(CurUnsortedPS, CurSortPerm);

            InvPermAlongPathOut = CurPerm;
            InvSortPermForRoot = CurSortPerm;

            vector<PSTraceElemT> RTraceElems;
            while (it != PredMap.end()) {
                // Unwind the pred state
                auto UnwoundEdge = it->second;
                auto PredStatePerm = UnwoundEdge.second;
                auto CmdID = UnwoundEdge.first;
                auto PredPermPS = get<0>(PredStatePerm);
                auto PredPerm = get<1>(PredStatePerm);
                auto PredSortPerm = get<2>(PredStatePerm);

                auto UnsortedPredPS = TheCanonicalizer->ApplyPermutation(PredPermPS, PredPerm,
                                                                         ProcIdxSet);
                TheCanonicalizer->Sort(UnsortedPredPS);
                TheCanonicalizer->ApplySort(UnsortedPredPS, PredSortPerm);

                RTraceElems.push_back(PSTraceElemT(GuardedCmds[CmdID], CurUnsortedPS));
                it = PredMap.find(PredStatePerm);
                CurUnsortedPS = UnsortedPredPS;
            }

            // delete the origin
            CurUnsortedPS->GetSVPtr()->Recycle();
            delete CurUnsortedPS;

            PathElems.clear();
            PathElems.insert(PathElems.end(), RTraceElems.rbegin(), RTraceElems.rend());
            return get<0>(TargetPSPerm);
        }

        inline bool
        TraceBase::CheckFairnessSat(const vector<PSTraceElemT>& PathSoFar,
                                    const Detail::FairnessChecker* FChecker,
                                    const vector<GCmdRef>& GuardedCmds,
                                    u32 InstanceID)
        {
            if (FChecker->IsStrongFairness()) {
                if (!FChecker->IsEnabled(InstanceID)) {
                    return true;
                } else {
                    auto const& SatCmds = FChecker->GetCmdIDsToRespondTo(InstanceID);
                    for (auto const& TraceElem : PathSoFar) {
                        if (SatCmds.find(TraceElem.first->GetCmdID()) != SatCmds.end()) {
                            return true;
                        }
                    }
                    return false;
                }
            } else {
                if (!FChecker->IsDisabled(InstanceID)) {
                    auto const& SatCmds = FChecker->GetCmdIDsToRespondTo(InstanceID);
                    for (auto const& TraceElem : PathSoFar) {
                        if (SatCmds.find(TraceElem.first->GetCmdID()) != SatCmds.end()) {
                            return true;
                        }
                    }
                    return false;
                } else {
                    auto const& SatCmds = FChecker->GetCmdIDsToRespondTo(InstanceID);
                    for (auto const& TraceElem : PathSoFar) {
                        auto PS = TraceElem.second;
                        bool FoundOne = false;
                        for (auto SatCmd : SatCmds) {

                            // We ignore the exception here
                            bool Exception = false;
                            ExpT NEPred;
                            auto NS = TryExecuteCommand(GuardedCmds[SatCmd],
                                                        PS->GetSVPtr(), Exception, NEPred);
                            if (NS != nullptr) {
                                NS->Recycle();
                                FoundOne = true;
                                break;
                            }
                        }
                        if (!FoundOne) {
                            return true;
                        }
                    }

                    return false;
                }
            }
        }

        SafetyViolation* TraceBase::MakeSafetyViolation(const StateVec* ErrorState,
                                                        LTSChecker* Checker,
                                                        const ExpT& BlownInvariant)
        {
            auto TheAQS = Checker->AQS;
            auto PPath = TheAQS->FindPath(ErrorState);
            return MakeSafetyViolation(PPath, Checker, BlownInvariant);
        }

        SafetyViolation* TraceBase::MakeBoundsViolation(const StateVec* ErrorState,
                                                        LTSChecker* Checker)
        {
            auto TheAQS = Checker->AQS;
            auto PPath = TheAQS->FindPath(ErrorState);
            return MakeBoundsViolation(PPath, Checker);
        }

        DeadlockViolation* TraceBase::MakeDeadlockViolation(const StateVec* ErrorState,
                                                            LTSChecker* Checker)
        {
            auto TheAQS = Checker->AQS;
            auto PPath = TheAQS->FindPath(ErrorState);
            return MakeDeadlockViolation(PPath, Checker);
        }

        SafetyViolation* TraceBase::MakeSafetyViolation(AQSPermPath* PermPath,
                                                        LTSChecker* Checker,
                                                        const ExpT& BlownInvariant)
        {
            vector<TraceElemT> PathElems;
            auto UnwoundInitState = UnwindPermPath(PermPath, Checker, PathElems);
            delete PermPath;
            return new SafetyViolation(UnwoundInitState, PathElems,
                                       Checker->Printer, BlownInvariant);
        }

        SafetyViolation* TraceBase::MakeBoundsViolation(AQSPermPath* PermPath,
                                                        LTSChecker* Checker)
        {
            vector<TraceElemT> PathElems;
            u32 FinalInvPerm = 0;
            auto UnwoundInitState = UnwindPermPath(PermPath, Checker,
                                                   PathElems, FinalInvPerm);
            auto FinalState = PathElems.back().second;

            auto const& Commands = Checker->GuardedCommands;
            ExpT BlownInvariant = ExpT::NullPtr;
            // Find out which of the bounds invariants is blown
            for (auto const& Cmd : Commands) {
                ExpT NEPred;
                bool Exception = false;
                auto NS = TryExecuteCommand(Cmd, FinalState, Exception, NEPred);
                if (Exception) {
                    BlownInvariant = NEPred;
                    break;
                } else if (NS != nullptr) {
                    NS->Recycle();
                }
            }

            if (BlownInvariant == ExpT::NullPtr) {
                throw InternalError((string)"Could not find invariant that was " +
                                    "blown in the last state of the error trace " +
                                    "in call to TraceBase::MakeBoundsViolation().\n" +
                                    "At: " + __FILE__ + ":" + to_string(__LINE__));
            }

            return new SafetyViolation(UnwoundInitState, PathElems,
                                       Checker->Printer, BlownInvariant);
        }

        DeadlockViolation* TraceBase::MakeDeadlockViolation(AQSPermPath* PermPath,
                                                            LTSChecker* Checker)
        {
            vector<TraceElemT> PathElems;
            auto UnwoundInitState = UnwindPermPath(PermPath, Checker, PathElems);
            delete PermPath;
            return new DeadlockViolation(UnwoundInitState, PathElems,
                                         Checker->Printer,
                                         Checker->DeadlockFreeInvariant);
        }

        LivenessViolation* TraceBase::MakeLivenessViolation(const ProductState* SCCRoot,
                                                            LTSChecker* Checker)
        {
            auto ThePS = Checker->ThePS;
            auto const& GuardedCmds = Checker->GuardedCommands;
            auto TheCanonicalizer = Checker->TheCanonicalizer;

            auto&& SCCNodes = ExpandSCC(SCCRoot, Checker);
            // cout << "[Trace:]" << "ExpandedSCC contains " << SCCNodes.size()
            //      << " states" << endl << endl;
            u32 InvPermAlongPath = 0;

            // Find a path from the initial state to one of the SCC nodes
            auto StemPPath = ThePS->FindPath([&] (const ProductState* State) -> bool
                                             {
                                                 return (SCCNodes.find(State) !=
                                                         SCCNodes.end());
                                             });
            vector<PSTraceElemT> StemPath;
            auto InitState = UnwindPermPath(StemPPath, Checker, StemPath, InvPermAlongPath);

            auto StemPathBackPair = StemPath.back();
            auto StartOfLoop = StemPathBackPair.second;


            u32 StemPermID;
            auto SortedSOLSV = TheCanonicalizer->SortChans(StartOfLoop->GetSVPtr(), true,
                                                           StemPermID);
            auto StemPermUpdates = TheCanonicalizer->GetChanUpdatesForPermutation(StemPermID);

            auto SortedStartOfLoop = new ProductState(SortedSOLSV,
                                                      StartOfLoop->GetMonitorState(),
                                                      StartOfLoop->GetIndexID(), 0);
            StemPath.pop_back();
            StemPath.push_back(PSTraceElemT(StemPathBackPair.first, SortedStartOfLoop));
            StartOfLoop->GetSVPtr()->Recycle();
            delete StartOfLoop;
            StartOfLoop = SortedStartOfLoop;

            auto CurEndOfPath = StemPPath->GetPathElems().back()->GetTarget();
            u32 InvSortPermAlongPath = 0;

            delete StemPPath;

            auto const& AllFCheckers = Checker->FairnessCheckers;
            vector<PSTraceElemT> PathSoFar;

            for (auto const& FCheckers : AllFCheckers) {
                for (auto FChecker : FCheckers) {
                    const u32 NumInstances = FChecker->NumInstances;

                    for (u32 InstanceID = 0; InstanceID < NumInstances; ++InstanceID) {

                        // Check if this fairness is already satisfied
                        auto AlreadySat = CheckFairnessSat(PathSoFar, FChecker,
                                                           GuardedCmds, InstanceID);
                        if (AlreadySat) {
                            continue;
                        }

                        vector<PSTraceElemT> CurElems;
                        function<bool(u32, const ProductState*)> MatchPred;
                        auto const& SatCmds = FChecker->GetCmdIDsToRespondTo(InstanceID);
                        // extend this path
                        if (FChecker->IsStrongFairness()) {
                            if (FChecker->IsEnabled(InstanceID)) {
                                MatchPred =
                                    [&] (u32 CmdID, const ProductState* State) -> bool
                                    {
                                        return (SatCmds.find(CmdID) != SatCmds.end());
                                    };
                            }
                            // if not then, we're trivially satisfied
                        } else {
                            if (!FChecker->IsDisabled(InstanceID)) {
                                MatchPred =
                                    [&] (u32 CmdID, const ProductState* State) -> bool
                                    {
                                        return (SatCmds.find(CmdID) != SatCmds.end());
                                    };

                            } else {
                                MatchPred =
                                    [&] (u32 CmdID, const ProductState* State) -> bool
                                    {
                                        for (auto SatCmdID : SatCmds) {
                                            // we ignore exceptions here
                                            bool Exception = false;
                                            ExpT NEPred;
                                            auto NS = TryExecuteCommand(GuardedCmds[SatCmdID],
                                                                        State->GetSVPtr(),
                                                                        Exception, NEPred);
                                            if (NS != nullptr) {
                                                NS->Recycle();
                                                return false;
                                            }
                                        }
                                        return true;
                                    };
                            }
                        }

                        CurEndOfPath = DoUnwoundBFS(CurEndOfPath, Checker, InvPermAlongPath,
                                                    InvSortPermAlongPath, MatchPred,
                                                    CurElems, SCCNodes);

                        PathSoFar.insert(PathSoFar.end(), CurElems.begin(), CurElems.end());
                    }
                }
            }

            // are we done?
            if (PathSoFar.size() > 0) {
                auto LastPair = PathSoFar.back();
                auto LastPS = LastPair.second;
                u32 FinalPermID;
                auto SortedSV = TheCanonicalizer->SortChans(LastPS->GetSVPtr(), true, FinalPermID);
                if (SortedSV->Equals(*(StartOfLoop->GetSVPtr())) &&
                    StartOfLoop->GetMonitorState() == LastPS->GetMonitorState() &&
                    StartOfLoop->GetIndexID() == LastPS->GetIndexID()) {

                    auto&& LoopPermUpdates =
                        TheCanonicalizer->GetChanUpdatesForPermutation(FinalPermID);
                    PathSoFar.pop_back();
                    auto SortedPS = new ProductState(SortedSV, StartOfLoop->GetMonitorState(),
                                                     StartOfLoop->GetIndexID(), 0);
                    PathSoFar.push_back(PSTraceElemT(LastPair.first, SortedPS));
                    LastPS->GetSVPtr()->Recycle();
                    delete LastPS;

                    return (new LivenessViolation(InitState, StemPath, StemPermUpdates,
                                                  PathSoFar, LoopPermUpdates,
                                                  Checker->Printer, ThePS));
                }
            } if (PathSoFar.size() == 0 && SCCNodes.size() == 1) {
                // Single node SCC with self loop
                // Find the command
                auto const& Edges = ThePS->GetEdges(const_cast<ProductState*>(CurEndOfPath));
                for (auto const& Edge : Edges) {
                    if (Edge->GetTarget()->Equals(CurEndOfPath)) {
                        PSTraceElemT CurElem(GuardedCmds[Edge->GetGCmdIndex()],
                                             new ProductState(Edge->GetTarget()->GetSVPtr()->Clone(),
                                                              Edge->GetTarget()->GetMonitorState(),
                                                              Edge->GetTarget()->GetIndexID(), 1));
                        PathSoFar.push_back(CurElem);
                        return new LivenessViolation(InitState, StemPath, StemPermUpdates,
                                                     PathSoFar, {}, Checker->Printer, ThePS);
                    }
                }
                throw InternalError((string)"Could not complete cycle in single node SCC.\n" +
                                    "At: " + __FILE__ + ":" + to_string(__LINE__));
            } else if (PathSoFar.size() == 0) {
                // Transition to some other state first and then call
                // do unwound bfs
                vector<PSTraceElemT> TempPath;
                CurEndOfPath = DoUnwoundBFS(CurEndOfPath, Checker, InvPermAlongPath,
                                            InvSortPermAlongPath,
                                            [&] (u32 CmdID, const ProductState* State) -> bool
                                            {
                                                return (!State->Equals(StartOfLoop));
                                            }, TempPath, SCCNodes);

                PathSoFar.insert(PathSoFar.end(), TempPath.begin(), TempPath.end());
                // fall through and be a man! Do the right thing!
            }

            // That handles all the special cases!
            // Now connect this path back to the original state
            vector<PSTraceElemT> LoopBack;
            auto FinalPred =
                [&] (u32 CmdID, const ProductState* State) -> bool
                {
                    u32 Dummy;
                    auto SortedSV = TheCanonicalizer->SortChans(State->GetSVPtr(), false, Dummy);
                    auto Retval = SortedSV->Equals(*(StartOfLoop->GetSVPtr()));
                    Retval = Retval && State->GetMonitorState() == StartOfLoop->GetMonitorState();
                    Retval = Retval && State->GetIndexID() == StartOfLoop->GetIndexID();
                    SortedSV->Recycle();
                    return Retval;
                };

            DoUnwoundBFS(CurEndOfPath, Checker, InvPermAlongPath,
                         InvSortPermAlongPath,
                         FinalPred, LoopBack, SCCNodes);
            // Sort the final state and include the updates
            auto OldEndOfPathPair = LoopBack.back();
            auto OldEndOfPath = OldEndOfPathPair.second;
            LoopBack.pop_back();
            u32 FinalPermID;
            auto SortedSV = TheCanonicalizer->SortChans(OldEndOfPath->GetSVPtr(), true,
                                                        FinalPermID);
            auto SortedEndOfPath = new ProductState(SortedSV, OldEndOfPath->GetMonitorState(),
                                                    OldEndOfPath->GetIndexID(), 0);
            OldEndOfPath->GetSVPtr()->Recycle();
            delete OldEndOfPath;

            LoopBack.push_back(PSTraceElemT(OldEndOfPathPair.first, SortedEndOfPath));
            auto&& LoopPermUpdates = TheCanonicalizer->GetChanUpdatesForPermutation(FinalPermID);

            PathSoFar.insert(PathSoFar.end(), LoopBack.begin(), LoopBack.end());

            return (new LivenessViolation(InitState, StemPath, StemPermUpdates, PathSoFar,
                                          LoopPermUpdates, Checker->Printer, ThePS));
        }

        // Takes ownership of trace elems
        // and initial state
        SafetyViolation::SafetyViolation(const StateVec* InitialState,
                                         const vector<TraceElemT>& TraceElems,
                                         StateVecPrinter* Printer,
                                         const ExpT& BlownInvariant)
            : TraceBase(Printer), InitialState(InitialState),
              TraceElems(TraceElems), BlownInvariant(BlownInvariant)
        {
            // Nothing here
        }

        SafetyViolation::~SafetyViolation()
        {
            InitialState->Recycle();
            for (auto const& TraceElem : TraceElems) {
                TraceElem.second->Recycle();
            }
        }

        const StateVec* SafetyViolation::GetInitialState() const
        {
            return InitialState;
        }

        const vector<TraceElemT>& SafetyViolation::GetTraceElems() const
        {
            return TraceElems;
        }

        string SafetyViolation::ToString(u32 Verbosity) const
        {
            ostringstream sstr;
            if (this->Is<DeadlockViolation>()) {
                sstr << "Trace to deadlock with " << TraceElems.size()
                     << " steps:" << endl << endl;
            } else {
                sstr << "Trace to safety violation with " << TraceElems.size()
                     << " steps:" << endl << endl;
            }
            sstr << "Initial State (in full)" << endl;
            sstr << "-----------------------------------------------------" << endl;
            Printer->PrintState(InitialState, sstr);
            sstr << "-----------------------------------------------------" << endl << endl;

            auto PrevState = InitialState;
            for (auto const& TraceElem : TraceElems) {
                auto const& MsgType = TraceElem.first->GetMsgType();
                auto MsgTypeAsRec = MsgType->SAs<RecordType>();
                if (Verbosity < 1) {
                    sstr << "Fired Guarded Command with label: "
                         << (MsgTypeAsRec != nullptr ? MsgTypeAsRec->GetName() :
                             "(internal transition)") << endl;
                } else {
                    sstr << "Fired Guarded Command:" << endl;
                    sstr << TraceElem.first->ToString(Verbosity) << endl;
                }
                sstr << "Obtained next state (delta from previous state):" << endl;
                sstr << "-----------------------------------------------------" << endl;
                Printer->PrintState(TraceElem.second, PrevState, sstr);
                sstr << "-----------------------------------------------------" << endl << endl;
                PrevState = TraceElem.second;
            }

            sstr << "Last state of trace (in full):" << endl;
            sstr << "-----------------------------------------------------" << endl;
            Printer->PrintState(PrevState, sstr);
            sstr << "-----------------------------------------------------" << endl << endl;
            return sstr.str();
        }

        const ExpT& SafetyViolation::GetInvariantBlown() const
        {
            return BlownInvariant;
        }

        DeadlockViolation::DeadlockViolation(const StateVec* InitialState,
                                             const vector<TraceElemT>& TraceElems,
                                             StateVecPrinter* Printer,
                                             const ExpT& BlownInvariant)
            : SafetyViolation(InitialState, TraceElems, Printer, BlownInvariant)
        {
            // Nothing here
        }

        DeadlockViolation::~DeadlockViolation()
        {
            // Nothing here
        }

        // Implementation of LivenessViolation
        LivenessViolation::LivenessViolation(const ProductState* InitialState,
                                             const vector<PSTraceElemT>& StemElems,
                                             const vector<LTSAssignRef>& StemSortPermutation,
                                             const vector<PSTraceElemT>& LoopElems,
                                             const vector<LTSAssignRef>& LoopSortPermutation,
                                             StateVecPrinter* Printer,
                                             const ProductStructure* ThePS)
            : TraceBase(Printer), InitialState(InitialState), ThePS(ThePS),
              StemPath(StemElems), StemSortPermutation(StemSortPermutation),
              LoopPath(LoopElems), LoopSortPermutation(LoopSortPermutation)
        {
            // Nothing here
        }

        // We assume that all statevectors are cloned
        // and the product states are free to be deleted
        LivenessViolation::~LivenessViolation()
        {
            InitialState->GetSVPtr()->Recycle();
            delete InitialState;

            for (auto const& StemElem: StemPath) {
                StemElem.second->GetSVPtr()->Recycle();
                delete StemElem.second;
            }

            for (auto const& LoopElem : LoopPath) {
                LoopElem.second->GetSVPtr()->Recycle();
                delete LoopElem.second;
            }
        }

        const ProductState* LivenessViolation::GetInitialState() const
        {
            return InitialState;
        }

        const vector<PSTraceElemT>& LivenessViolation::GetStem() const
        {
            return StemPath;
        }

        const vector<PSTraceElemT>& LivenessViolation::GetLoop() const
        {
            return LoopPath;
        }

        string LivenessViolation::ToString(u32 Verbosity) const
        {
            ostringstream sstr;
            sstr << "Trace to liveness violation with " << StemPath.size()
                 << " steps in the stem and " << LoopPath.size()
                 << " steps in the loop:" << endl;
            sstr << "Initial State (in full)" << endl;
            sstr << "-----------------------------------------------------" << endl;
            Printer->PrintState(InitialState, ThePS, sstr);
            sstr << "-----------------------------------------------------" << endl << endl;

            auto PrevState = InitialState;
            const u32 NumStemSteps = StemPath.size();
            for (u32 i = 0; i < NumStemSteps; ++i) {
                auto const& TraceElem = StemPath[i];
                auto const& MsgType = TraceElem.first->GetMsgType();
                auto MsgTypeAsRec = MsgType->SAs<RecordType>();
                if (Verbosity < 1) {
                    sstr << "Fired Guarded Command with label: "
                         << (MsgTypeAsRec != nullptr ? MsgTypeAsRec->GetName() :
                             "(internal transition)") << endl;
                } else {
                    sstr << "Fired Guarded Command:" << endl;
                    sstr << TraceElem.first->ToString(Verbosity) << endl;
                }
                if (i != NumStemSteps - 1) {
                    sstr << "Obtained next state (delta from previous state):" << endl;
                    sstr << "-----------------------------------------------------" << endl;
                    Printer->PrintState(TraceElem.second, PrevState, ThePS, sstr);
                    sstr << "-----------------------------------------------------" << endl << endl;
                    PrevState = TraceElem.second;
                } else {
                    sstr << "Obtained first state of loop (delta from previous state):" << endl;
                    sstr << "-----------------------------------------------------" << endl;
                    Printer->PrintState(TraceElem.second, PrevState, ThePS, sstr);
                    sstr << "-----------------------------------------------------" << endl << endl;
                    PrevState = TraceElem.second;
                    sstr << "Note that the following permutation on the channels have "
                         << "been applied to get the first state of loop from the stem:" << endl;
                    sstr << "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-" << endl << endl;
                    for (auto const& Update : StemSortPermutation) {
                        sstr << Update->ToString(Verbosity) << endl;
                    }
                    sstr << "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-" << endl << endl;
                }
            }

            sstr << "First state of the loop (in full), which is same as last state of "
                 << "the stem printed above:" << endl;
            sstr << "-----------------------------------------------------" << endl;
            Printer->PrintState(PrevState, ThePS, sstr);
            sstr << "-----------------------------------------------------" << endl << endl;

            for (auto const& TraceElem : LoopPath) {
                auto const& MsgType = TraceElem.first->GetMsgType();
                auto MsgTypeAsRec = MsgType->SAs<RecordType>();
                if (Verbosity < 1) {
                    sstr << "Fired Guarded Command with label: "
                         << (MsgTypeAsRec != nullptr ? MsgTypeAsRec->GetName() :
                             "(internal transition)") << endl;
                } else {
                    sstr << "Fired Guarded Command:" << endl;
                    sstr << TraceElem.first->ToString(Verbosity) << endl;
                }
                sstr << "Obtained next state (delta from previous state):" << endl;
                sstr << "-----------------------------------------------------" << endl;
                Printer->PrintState(TraceElem.second, PrevState, ThePS, sstr);
                sstr << "-----------------------------------------------------" << endl << endl;
                PrevState = TraceElem.second;
            }

            sstr << "Loop back state, same as first state of loop above (in full):"
                 << endl;
            sstr << "-----------------------------------------------------" << endl;
            Printer->PrintState(PrevState, ThePS, sstr);
            sstr << "-----------------------------------------------------" << endl << endl;
            sstr << "Note that the following permutation on the channels have "
                 << "been applied to get to the loop back state from the previous "
                 << "state in the loop:" << endl;
            sstr << "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-" << endl << endl;
            for (auto const& Update : LoopSortPermutation) {
                sstr << Update->ToString(Verbosity) << endl;
            }
            sstr << "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-" << endl << endl;

            return sstr.str();
        }

        const vector<LTSAssignRef>& LivenessViolation::GetStemSortPermutation() const
        {
            return StemSortPermutation;
        }

        const vector<LTSAssignRef>& LivenessViolation::GetLoopSortPermutation() const
        {
            return LoopSortPermutation;
        }

    } /* end namespace MC */
} /* end namespace ESMC */

//
// Trace.cpp ends here
