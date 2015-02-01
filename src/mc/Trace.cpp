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
#include "../mc/Compiler.hpp"

#include "StateVec.hpp"
#include "StateVecPrinter.hpp"
#include "Trace.hpp"
#include "AQStructure.hpp"
#include "LTSChecker.hpp"
#include "LTSCheckerUtils.hpp"
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

inline vector<tuple<GCmdRef, StateVec*, u32>>
TraceBase::FindCommandsForTransition(const vector<GCmdRef>& Commands,
                                     Canonicalizer* TheCanonicalizer,
                                     const StateVec* UnwoundFromState,
                                     const StateVec* UnwoundToState,
                                     bool RememberSortPermIdx)
{
    vector<tuple<GCmdRef, StateVec*, u32>> Retval;
    for (auto const& Cmd : Commands) {
        bool Exception = false;
        ExpT NEPred;
        auto CandidateSV = TryExecuteCommand(Cmd, UnwoundFromState,
                                             Exception, NEPred);
        if (CandidateSV == nullptr) {
            continue;
        }

        u32 SortPermIdx = 0;
        auto SortedCandidateSV =
            TheCanonicalizer->SortChans(CandidateSV, RememberSortPermIdx, SortPermIdx);
        if (SortedCandidateSV->Equals(*UnwoundToState)) {
            SortedCandidateSV->Recycle();
            Retval.push_back(make_tuple(Cmd, CandidateSV, SortPermIdx));
        } else {
            SortedCandidateSV->Recycle();
            CandidateSV->Recycle();
        }
    }

    return Retval;
}

inline const StateVec*
TraceBase::UnwindPermPath(AQSPermPath* PermPath,
                          LTSChecker* Checker,
                          vector<TraceElemT>& PathElems,
                          u32& InvPermAlongPathOut)
{
    auto TheLTS = Checker->TheLTS;
    auto TheCanonicalizer = Checker->TheCanonicalizer;

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

        ESMC_LOG_FULL(
                      "Trace.Generation",
                      auto Printer = Checker->Printer;
                      Out_ << "Permuted State:" << endl;
                      Out_ << "-------------------------------------------" << endl;
                      Printer->PrintState(NextPermState, Out_);
                      Out_ << "-------------------------------------------" << endl;
                      );

        auto NextUnwoundState = TheCanonicalizer->ApplyPermutation(NextPermState,
                                                                   InvPermAlongPath);
        TheCanonicalizer->Sort(NextUnwoundState);

        ESMC_LOG_FULL(
                      "Trace.Generation",
                      auto Printer = Checker->Printer;
                      Out_ << "Unwound State:" << endl;
                      Out_ << "-------------------------------------------" << endl;
                      Printer->PrintState(NextUnwoundState, Out_);
                      Out_ << "-------------------------------------------" << endl;
                      );

        // Now find out which command takes us from the current
        // unwound state to the next unwound state
        auto&& CmdNextUnsortedStateList =
            FindCommandsForTransition(GuardedCommands, TheCanonicalizer,
                                      CurUnwoundState, NextUnwoundState);
        auto const& TransitionCmd = get<0>(CmdNextUnsortedStateList.front());
        auto NextUnsortedState = get<1>(CmdNextUnsortedStateList.front());

        if (TransitionCmd == GCmdRef::NullPtr) {
            ostringstream sstr;
            auto Printer = Checker->Printer;
            sstr << "Could not find a command to compute the next unwound state!"
                 << endl << "Current unwound state:" << endl;
            Printer->PrintState(CurUnwoundState, sstr);
            sstr << endl << "Next Unwound state:" << endl;
            Printer->PrintState(NextUnwoundState, sstr);
            sstr << endl;
            sstr << "At: " << __FUNCTION__ << ", " << __FILE__ << ":" << __LINE__ << endl;
            throw InternalError(sstr.str());
        } else {
            NextUnwoundState->Recycle();
            PathElems.push_back(TraceElemT(TransitionCmd, NextUnsortedState));
        }

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

        ProductState* NextUnsortedPS = nullptr;

        // find out which command takes us here
        auto&& CmdNextUnsortedSVList =
            FindCommandsForTransition(GuardedCommands, TheCanonicalizer,
                                      CurUnwoundState->GetSVPtr(),
                                      NextUnwoundPS->GetSVPtr());
        auto const& TransitionCmd = get<0>(CmdNextUnsortedSVList.front());
        auto NextUnsortedSV = get<1>(CmdNextUnsortedSVList.front());

        if (TransitionCmd == GCmdRef::NullPtr) {
            throw InternalError((string)"Unable to find a command to compute next " +
                                "unwound product state.\nAt: " + __FILE__ + ":" +
                                to_string(__LINE__));
        } else {
            NextUnsortedPS = new ProductState(NextUnsortedSV,
                                              NextUnwoundPS->GetMonitorState(),
                                              NextUnwoundPS->GetIndexID(), 0);
            PathElems.push_back(PSTraceElemT(TransitionCmd, NextUnsortedPS));
            NextUnwoundPS->GetSVPtr()->Recycle();
            delete NextUnwoundPS;
        }

        CurUnwoundState = NextUnsortedPS;
    }

    InvPermAlongPathOut = InvPermAlongPath;
    return UnwoundOrigin;
}

// static methods for liveness traces
inline const ProductState*
TraceBase::DoUnwoundBFS(const ProductState* CanonicalRoot,
                        const LTSChecker* Checker,
                        u32& InvPermAlongPathOut,
                        u32& InvSortPermForRoot,
                        FairnessChecker* FChecker,
                        u32 FairnessInstance,
                        const function<bool(u32, const ProductState*)>& MatchPred,
                        vector<PSTraceElemT>& PathElems)
{
    auto ThePS = Checker->ThePS;
    auto TheCanonicalizer = Checker->TheCanonicalizer;
    auto PermSet = TheCanonicalizer->GetPermSet();
    auto SortPermSet = TheCanonicalizer->GetSortPermSet();
    auto Monitor = ThePS->GetMonitor();
    auto ProcIdxSet = Monitor->GetIndexSet();
    auto const& GuardedCmds = Checker->GuardedCommands;
    auto TheSCCID = CanonicalRoot->GetSCCID();

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
    // Predecessor of the target. This is not in
    // the predecessor map, because the target could
    // have looped back into the structure to a
    // previously visited node
    pair<u32, BFSQueueEntryT> TargetPredecessor;

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
            bool RevisitingNode = false;

            if (NextPermPS->GetSCCID() != TheSCCID) {
                // Not part of the scc, ignore
                continue;
            }

            auto EdgePermIndex = Edge->GetPermutation();
            auto EdgeInvPermIt = PermSet->GetIteratorForInv(EdgePermIndex);
            auto NextPermIt = PermSet->Compose(CurPermIndex, EdgeInvPermIt.GetIndex());
            auto NextPermIndex = NextPermIt.GetIndex();
            auto NextPair = make_pair(NextPermPS, NextPermIndex);

            if (VisitedStates.find(NextPair) != VisitedStates.end()) {
                RevisitingNode = true;
            }

            VisitedStates.insert(NextPair);
            auto NextUnwoundPS = TheCanonicalizer->ApplyPermutation(NextPermPS,
                                                                    NextPermIndex,
                                                                    ProcIdxSet);
            u32 Unused = 0;
            auto NextSortedSV = TheCanonicalizer->SortChans(NextUnwoundPS->GetSVPtr(),
                                                            false, Unused);

            // Find out which commands take us from the current unwound
            // product state to the next unwound product state

            auto&& CmdNextSVSortPermList =
                FindCommandsForTransition(GuardedCmds, TheCanonicalizer,
                                          CurUnsortedPS->GetSVPtr(),
                                          NextSortedSV, true);

            for (auto const& CmdNextSVSortPerm : CmdNextSVSortPermList) {
                auto const& TransitionCmd = get<0>(CmdNextSVSortPerm);
                auto TransitionCmdID = TransitionCmd->GetCmdID();
                auto NextUnsortedSV = get<1>(CmdNextSVSortPerm);
                auto SortPermIdx = get<2>(CmdNextSVSortPerm);

                if (TransitionCmd == GCmdRef::NullPtr) {
                    throw InternalError((string)"Unable to find a command to compute next " +
                                        "unwound product state.\nAt: " + __FILE__ + ":" +
                                        to_string(__LINE__));
                }

                auto InvSortPermIdx =
                    SortPermSet->GetIteratorForInv(SortPermIdx).GetIndex();
                auto NextUnsortedPS = new ProductState(NextUnsortedSV,
                                                       NextUnwoundPS->GetMonitorState(),
                                                       NextUnwoundPS->GetIndexID(), 0);
                auto NextTuple = make_tuple(NextPermPS, NextPermIndex, InvSortPermIdx);

                if (!RevisitingNode) {
                    BFSQueue.push_back(NextTuple);
                    PredMap[NextTuple] = make_pair(TransitionCmdID, CurTuple);
                }

                if (FChecker != nullptr) {
                    auto FairnessSat =
                        FChecker->CheckInstanceSatisfaction(FairnessInstance,
                                                            TransitionCmdID,
                                                            NextUnsortedPS);
                    if (FairnessSat) {
                        ReachedTarget = true;
                        TargetPSPerm = NextTuple;
                        TargetPredecessor = make_pair(TransitionCmdID, CurTuple);
                    }
                } else {
                    // MatchPred must be something valid
                    if (MatchPred(TransitionCmdID, NextUnsortedPS)) {
                        ReachedTarget = true;
                        TargetPSPerm = NextTuple;
                        TargetPredecessor = make_pair(TransitionCmdID, CurTuple);
                    }
                }

                NextUnsortedPS->GetSVPtr()->Recycle();
                delete NextUnsortedPS;
                // don't iterate over rest of commands that take us to next
                // state if target already reached
                if (ReachedTarget) {
                    break;
                }
            } // end iterating over the commands that take us to the next state
            NextUnwoundPS->GetSVPtr()->Recycle();
            delete NextUnwoundPS;

            // don't iterate over the rest of the edges if target already reached
            if (ReachedTarget) {
                break;
            }
        } // End iterating over edges

        // Delete the current unwound product state as well
        CurUnsortedPS->GetSVPtr()->Recycle();
        delete CurUnsortedPS;
    } // End BFS

    if (!ReachedTarget) {
        return nullptr;
    }

    // We now need to assemble a path from the predecessors
    auto CurTuple = TargetPSPerm;
    auto CurPermPS = get<0>(CurTuple);
    auto CurPerm = get<1>(CurTuple);
    auto CurSortPerm = get<2>(CurTuple);

    auto CurUnsortedPS =
        TheCanonicalizer->ApplyPermutation(CurPermPS, CurPerm, ProcIdxSet);
    TheCanonicalizer->Sort(CurUnsortedPS);
    TheCanonicalizer->ApplySort(CurUnsortedPS, CurSortPerm);

    InvPermAlongPathOut = CurPerm;
    InvSortPermForRoot = CurSortPerm;

    vector<PSTraceElemT> RTraceElems;
    UnwoundPredMapT::const_iterator it;
    auto PredStatePerm = TargetPredecessor.second;
    auto CmdID = TargetPredecessor.first;
    auto PredPermPS = get<0>(PredStatePerm);
    auto PredPerm = get<1>(PredStatePerm);
    auto PredSortPerm = get<2>(PredStatePerm);

    auto UnsortedPredPS = TheCanonicalizer->ApplyPermutation(PredPermPS,
                                                             PredPerm,
                                                             ProcIdxSet);
    TheCanonicalizer->Sort(UnsortedPredPS);
    TheCanonicalizer->ApplySort(UnsortedPredPS, PredSortPerm);

    RTraceElems.push_back(PSTraceElemT(GuardedCmds[CmdID], CurUnsortedPS));
    it = PredMap.find(PredStatePerm);
    CurUnsortedPS = UnsortedPredPS;

    while (it != PredMap.end()) {
        // Unwind the pred state
        auto UnwoundEdge = it->second;
        PredStatePerm = UnwoundEdge.second;
        CmdID = UnwoundEdge.first;
        PredPermPS = get<0>(PredStatePerm);
        PredPerm = get<1>(PredStatePerm);
        PredSortPerm = get<2>(PredStatePerm);

        UnsortedPredPS = TheCanonicalizer->ApplyPermutation(PredPermPS,
                                                            PredPerm,
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

inline void
TraceBase::MarkFairnessesSatisfied(const vector<PSTraceElemT>& PathSegment,
                                   const LTSChecker* Checker)
{
    auto const& AllFCheckers = Checker->FairnessCheckers;

    for (auto const& FCheckers : AllFCheckers) {
        for (auto const& FChecker : FCheckers) {
            const u32 NumInstances = FChecker->NumInstances;

            for (u32 Instance = 0; Instance < NumInstances; ++Instance) {
                for (auto const& TraceElem : PathSegment) {
                    auto CmdID = TraceElem.first->GetCmdID();
                    auto PS = TraceElem.second;
                    if (FChecker->CheckInstanceSatisfaction(Instance, CmdID, PS)) {
                        ESMC_LOG_FULL("Trace.Generation",
                                      Out_ << "Marking fairness:" << endl
                                      << FChecker->ToString() << endl
                                      << "as satisfied by segment!" << endl;
                                      );
                        FChecker->SetInstanceSatisfiedInTrace(Instance);
                        // The instace is already satisfied, ignore the
                        // rest of the trace segment.
                        break;
                    }
                }
            }
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
    auto TheCanonicalizer = Checker->TheCanonicalizer;
    auto PermSet = TheCanonicalizer->GetPermSet();
    auto Mgr = Checker->TheLTS->GetMgr();
    u32 InvPermAlongPath = 0;

    // Find a path from the initial state to the SCC root
    // audupa, 01/25/2015: Changed this to be a path TO the root of the
    // SCC rather than ANY node in the SCC to eliminate a bug in the trace
    // generation.
    auto StemPPath = ThePS->FindPath([&] (const ProductState* State) -> bool
                                     {
                                         return (State == SCCRoot);
                                     });
    vector<PSTraceElemT> StemPath;
    auto InitState = UnwindPermPath(StemPPath, Checker, StemPath, InvPermAlongPath);

    auto InvPermAtEndOfStemIdx = InvPermAlongPath;
    auto InvPermAtEndOfStem = PermSet->GetIterator(InvPermAlongPath).GetPerm();

    ESMC_LOG_FULL("Trace.Generation",
                  Out_ << "Permutation at end of stem:" << endl
                  << Symm::PermToString(InvPermAtEndOfStem) << endl;
                  );

    auto StemPathBackPair = StemPath.back();
    auto StartOfLoop = StemPathBackPair.second;

    u32 StemPermID;
    auto SortedSOLSV = TheCanonicalizer->SortChans(StartOfLoop->GetSVPtr(), true,
                                                   StemPermID);
    auto StemPermUpdates = TheCanonicalizer->GetChanUpdatesForPermutation(StemPermID);
    StemPermUpdates = LTSCompiler::ArrayTransformAssignments(StemPermUpdates, Mgr);

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
    // Permute all the fairness checkers by the inverse perm at end of stem
    for (auto const& FCheckers : AllFCheckers) {
        for (auto const& FChecker : FCheckers) {
            FChecker->Permute(InvPermAtEndOfStem);
        }
    }

    vector<PSTraceElemT> PathSoFar;

    for (auto const& FCheckers : AllFCheckers) {
        for (auto FChecker : FCheckers) {
            const u32 NumInstances = FChecker->NumInstances;
            for (u32 Instance = 0; Instance < NumInstances; ++Instance) {
                if (FChecker->CheckInstanceSatisfaction(Instance)) {
                    FChecker->SetInstanceSatisfiedInTrace(Instance);
                    continue;
                }
                // Check if this fairness is already satisfied
                if (FChecker->IsInstanceSatisfiedInTrace(Instance)) {
                    continue;
                }

                vector<PSTraceElemT> CurElems;
                // extend this path
                auto NewEndOfPath = DoUnwoundBFS(CurEndOfPath, Checker, InvPermAlongPath,
                                                 InvSortPermAlongPath, FChecker,
                                                 Instance,
                                                 TruePred<u32, const ProductState*>,
                                                 CurElems);
                if (NewEndOfPath == nullptr) {
                    ostringstream sstr;
                    sstr << "Could not connect the end of path to a state satisfying "
                         << "the next fairness requirement." << endl
                         << "Current end of path:" << endl;
                    Checker->Printer->PrintState(CurEndOfPath, sstr);
                    sstr << endl << "Fairness requirement:" << endl;
                    sstr << FChecker->ToString() << " on instance: "
                         << Instance << endl;
                    sstr << "Permutation at end of stem: " << endl;
                    PermSet->Print(InvPermAtEndOfStemIdx, sstr);
                    sstr << endl;
                    sstr << "At: " << __FUNCTION__ << ", " << __FILE__ << ":"
                         << __LINE__ << endl;

                    // // audupa: debugging code, redo the DoUnwoundBFS,
                    // //         so we can catch it in the debugger, a poor man's
                    // //         replay, basically.
                    // cout << sstr.str();
                    // cout.flush();
                    // auto NewEndOfPath = DoUnwoundBFS(CurEndOfPath, Checker, InvPermAlongPath,
                    //                                  InvSortPermAlongPath, FChecker,
                    //                                  Instance,
                    //                                  TruePred<u32, const ProductState*>,
                    //                                  CurElems);

                    throw InternalError(sstr.str());
                } else {
                    CurEndOfPath = NewEndOfPath;
                }

                FChecker->SetInstanceSatisfiedInTrace(Instance);
                MarkFairnessesSatisfied(CurElems, Checker);

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
    } if (PathSoFar.size() == 0 && SCCRoot->IsSingular()) {
        // Single node SCC with self loop
        // Find the command

        vector<PSTraceElemT> TempPath;
        CurEndOfPath = DoUnwoundBFS(CurEndOfPath, Checker, InvPermAlongPath,
                                    InvSortPermAlongPath, nullptr, 0,
                                    [&] (u32 CmdID, const ProductState* State) -> bool
                                    {
                                        return State->Equals(StartOfLoop);
                                    }, TempPath);
        PathSoFar.insert(PathSoFar.end(), TempPath.begin(), TempPath.end());
    } else if (PathSoFar.size() == 0) {
        // Transition to some other state first and then call
        // do unwound bfs
        vector<PSTraceElemT> TempPath;
        CurEndOfPath = DoUnwoundBFS(CurEndOfPath, Checker, InvPermAlongPath,
                                    InvSortPermAlongPath,
                                    nullptr, 0,
                                    [&] (u32 CmdID, const ProductState* State) -> bool
                                    {
                                        return (!State->Equals(StartOfLoop));
                                    },
                                    TempPath);

        PathSoFar.insert(PathSoFar.end(), TempPath.begin(), TempPath.end());
        // fall through and be a man! Do the right thing!
    }

    // That handles all the special cases!
    // Now connect this path back to the original state
    // if necessary
    auto LastStateOnPath = PathSoFar.back().second;
    u32 LoopSortPerm;
    auto SortedLastStateSV = TheCanonicalizer->SortChans(LastStateOnPath->GetSVPtr(),
                                                         true, LoopSortPerm);
    if (SortedLastStateSV->Equals(*(StartOfLoop->GetSVPtr()))) {
        auto&& LoopPermUpdates =
            TheCanonicalizer->GetChanUpdatesForPermutation(LoopSortPerm);
        auto SortedLastStateOnPath = new ProductState(SortedLastStateSV,
                                                      LastStateOnPath->GetMonitorState(),
                                                      LastStateOnPath->GetIndexID(), 0);
        auto OldEndOfPathPair = PathSoFar.back();
        PathSoFar.pop_back();

        PathSoFar.push_back(PSTraceElemT(OldEndOfPathPair.first, SortedLastStateOnPath));
        OldEndOfPathPair.second->GetSVPtr()->Recycle();
        delete OldEndOfPathPair.second;
        return new LivenessViolation(InitState, StemPath, StemPermUpdates, PathSoFar,
                                     LoopPermUpdates, Checker->Printer, ThePS);
    }
    SortedLastStateSV->Recycle();

    // We do need to make a path back to the beginning of the loop
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
                 InvSortPermAlongPath, nullptr, 0, FinalPred, LoopBack);
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
    LoopPermUpdates = LTSCompiler::ArrayTransformAssignments(LoopPermUpdates, Mgr);

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
