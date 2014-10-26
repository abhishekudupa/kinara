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
                                  vector<TraceElemT>& PathElems)
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
                // cout << "Unwound State:" << endl;
                // cout << "-------------------------------------------" << endl;
                // Printer->PrintState(NextUnwoundState, cout);
                // cout << "-------------------------------------------" << endl;

                // Now find out which command takes us from the current
                // unwound state to the next unwound state
                bool FoundCmd = false;
                for (auto const& Cmd : GuardedCommands) {

                    // Ignore the exception here
                    bool Exception = false;
                    auto CandidateState = TryExecuteCommand(Cmd, CurUnwoundState, Exception);
                    if (CandidateState == nullptr) {
                        continue;
                    }
                    auto SortedCandidateState = TheCanonicalizer->SortChans(CandidateState);
                    CandidateState->Recycle();
                    if (SortedCandidateState->Equals(*NextUnwoundState)) {
                        FoundCmd = true;
                        SortedCandidateState->Recycle();
                        PathElems.push_back(TraceElemT(Cmd, NextUnwoundState));
                        break;
                    }
                }
                if (!FoundCmd) {
                    throw InternalError((string)"Unable to find a command to compute next " +
                                        "unwound state.\nAt: " + __FILE__ + ":" + 
                                        to_string(__LINE__));
                }

                // Do not recycle the unwound state
                // it's part of the return value
                CurUnwoundState = NextUnwoundState;
            }
            return UnwoundOrigin;
        }

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

            auto CurUnwoundState = TheCanonicalizer->ApplyPermutation(Origin, 
                                                                      InvPermAlongPathOut, 
                                                                      ProcIdxSet);
            auto UnwoundOrigin = CurUnwoundState;
            auto const& PPathElems = PermPath->GetPathElems();
            u32 InvPermAlongPath = InvPermAlongPathOut;
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
                // find out which command takes us here
                bool FoundCmd = false;
                for (auto const& Cmd : GuardedCommands) {
                    // we ignore the exception here
                    bool Exception = false;
                    auto CandidateSV = TryExecuteCommand(Cmd, CurUnwoundState->GetSVPtr(), 
                                                         Exception);
                    if (CandidateSV == nullptr) {
                        continue;
                    }
                    
                    auto SortedCandidateSV = TheCanonicalizer->SortChans(CandidateSV);
                    CandidateSV->Recycle();

                    if (SortedCandidateSV->Equals(*(NextUnwoundPS->GetSVPtr()))) {
                        FoundCmd = true;
                        SortedCandidateSV->Recycle();
                        PathElems.push_back(PSTraceElemT(Cmd, NextUnwoundPS));
                        break;
                    }
                }

                if (!FoundCmd) {
                    throw InternalError((string)"Unable to find a command to compute next " +
                                        "unwound product state.\nAt: " + __FILE__ + ":" + 
                                        to_string(__LINE__));
                }

                CurUnwoundState = NextUnwoundPS;
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

        inline pair<const ProductState*, const ProductState*>
        TraceBase::DoUnwoundBFS(const ProductState* Root, const LTSChecker* Checker, 
                                u32& InvPermAlongPathOut,
                                const function<bool(u32, const ProductState*)>& MatchPred,
                                vector<PSTraceElemT>& PathElems, 
                                const unordered_set<const ProductState*>& Bounds)
        {
            auto ThePS = Checker->ThePS;
            auto TheCanonicalizer = Checker->TheCanonicalizer;
            auto PermSet = TheCanonicalizer->GetPermSet();
            auto Monitor = ThePS->GetMonitor();
            auto ProcIdxSet = Monitor->GetIndexSet();
            auto const& GuardedCmds = Checker->GuardedCommands;
            const u32 NumGuardedCmds = GuardedCmds.size();

            deque<Detail::PSPermPairT> BFSQueue;
            Detail::PSPermSetT VisitedStates;
            Detail::UnwoundPredMapT PredMap;

            auto InvPermAlongPath = InvPermAlongPathOut;
            auto OriginPair = make_pair(Root, InvPermAlongPath);
            VisitedStates.insert(OriginPair);
            BFSQueue.push_back(OriginPair);

            bool ReachedTarget = false;
            Detail::PSPermPairT TargetPSPerm;

            while (BFSQueue.size() > 0 && !ReachedTarget) {
                auto CurPair = BFSQueue.front();
                BFSQueue.pop_front();

                auto CurPermPS = CurPair.first;
                auto CurPermIndex = CurPair.second;

                auto CurUnwoundPS = TheCanonicalizer->ApplyPermutation(CurPermPS, 
                                                                       CurPermIndex, 
                                                                       ProcIdxSet);
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
                    BFSQueue.push_back(NextPair);

                    auto NextUnwoundPS = TheCanonicalizer->ApplyPermutation(NextPermPS,
                                                                            NextPermIndex,
                                                                            ProcIdxSet);

                    // Find out which command takes us from the current 
                    // unwound product state to the next unwound product 
                    // state
                    bool FoundCmd = false;
                    for (u32 i = 0; i < NumGuardedCmds; ++i) {
                        auto const& Cmd = GuardedCmds[i];

                        // We ignore the exception here
                        bool Exception = false;
                        auto CandidateSV = TryExecuteCommand(Cmd, CurUnwoundPS->GetSVPtr(),
                                                             Exception);
                        if (CandidateSV == nullptr) {
                            continue;
                        }

                        auto SortedCandidateSV = TheCanonicalizer->SortChans(CandidateSV);
                        CandidateSV->Recycle();

                        if (SortedCandidateSV->Equals(*(NextUnwoundPS->GetSVPtr()))) {
                            FoundCmd = true;
                            SortedCandidateSV->Recycle();
                            PredMap[NextPair] = make_pair(i, CurPair);

                            if (MatchPred(i, NextUnwoundPS)) {
                                ReachedTarget = true;
                                TargetPSPerm = NextPair;
                            }
                            break;
                        }
                    }

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
                }

                // Delete the current unwound product state as well
                CurUnwoundPS->GetSVPtr()->Recycle();
                delete CurUnwoundPS;
            }

            // We now need to assemble a path from the predecessors
            auto CurPair = TargetPSPerm;
            auto it = PredMap.find(CurPair);
            auto CurPermPS = CurPair.first;
            auto CurPerm = CurPair.second;
            auto CurUnwoundPS = TheCanonicalizer->ApplyPermutation(CurPermPS, CurPerm,
                                                                   ProcIdxSet);
            InvPermAlongPathOut = CurPerm;

            vector<PSTraceElemT> RTraceElems;
            while (it != PredMap.end()) {
                // Unwind the pred state
                auto UnwoundEdge = it->second;
                auto PredStatePerm = UnwoundEdge.second;
                auto CmdID = UnwoundEdge.first;
                auto PredPermPS = PredStatePerm.first;
                auto PredPerm = PredStatePerm.second;

                auto UnwoundPredPS = TheCanonicalizer->ApplyPermutation(PredPermPS, PredPerm,
                                                                        ProcIdxSet);
                RTraceElems.push_back(PSTraceElemT(GuardedCmds[CmdID], CurUnwoundPS));
                it = PredMap.find(PredStatePerm);
                CurUnwoundPS = UnwoundPredPS;
            }

            PathElems.clear();
            PathElems.insert(PathElems.end(), RTraceElems.rbegin(), RTraceElems.rend());
            return (make_pair(CurUnwoundPS, TargetPSPerm.first));
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
                            auto NS = TryExecuteCommand(GuardedCmds[SatCmd], 
                                                        PS->GetSVPtr(), Exception);
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
            vector<TraceElemT> PathElems;
            auto TheAQS = Checker->AQS;
            auto PPath = TheAQS->FindShortestPath(ErrorState);
            auto UnwoundInitState = UnwindPermPath(PPath, Checker, PathElems);
            delete PPath;
            return new SafetyViolation(UnwoundInitState, PathElems, 
                                       Checker->Printer, BlownInvariant);
        }

        DeadlockViolation* TraceBase::MakeDeadlockViolation(const StateVec* ErrorState, 
                                                            LTSChecker* Checker)
        {
            vector<TraceElemT> PathElems;
            auto TheAQS = Checker->AQS;
            auto PPath = TheAQS->FindShortestPath(ErrorState);
            auto UnwoundInitState = UnwindPermPath(PPath, Checker, PathElems);
            delete PPath;
            auto const& BlownInvariant = Checker->DeadlockFreeInvariant;
            return new DeadlockViolation(UnwoundInitState, PathElems, 
                                         Checker->Printer, BlownInvariant);
        }

        LivenessViolation* TraceBase::MakeLivenessViolation(const ProductState* SCCRoot, 
                                                            LTSChecker* Checker)
        {
            auto ThePS = Checker->ThePS;
            auto const& GuardedCmds = Checker->GuardedCommands;

            auto&& SCCNodes = ExpandSCC(SCCRoot, Checker);
            u32 InvPermAlongPath = 0;
            
            // Find a path from the initial state to one of the SCC nodes
            auto StemPPath = ThePS->FindPath([&] (const ProductState* State) -> bool
                                             {
                                                 return (SCCNodes.find(State) != 
                                                         SCCNodes.end());
                                             });
            vector<PSTraceElemT> StemPath;
            auto InitState = UnwindPermPath(StemPPath, Checker, StemPath, InvPermAlongPath);

            auto StartOfLoop = StemPath.back().second;
            auto CurEndOfPath = StemPPath->GetPathElems().back()->GetTarget();

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
                                            auto NS = TryExecuteCommand(GuardedCmds[SatCmdID], 
                                                                        State->GetSVPtr(),
                                                                        Exception);
                                            if (NS != nullptr) {
                                                NS->Recycle();
                                                return false;
                                            }
                                        }
                                        return true;
                                    };
                            }
                        }

                        auto CurPair = DoUnwoundBFS(CurEndOfPath, Checker, InvPermAlongPath, 
                                                    MatchPred, CurElems, SCCNodes);

                        CurPair.first->GetSVPtr()->Recycle();
                        delete CurPair.first;

                        CurEndOfPath = CurPair.second;
                        PathSoFar.insert(PathSoFar.end(), CurElems.begin(), CurElems.end());
                    }
                }
            }

            // audupa: DEBUG
            
            // auto TempLV = new LivenessViolation(InitState, StemPath, PathSoFar,
            //                                     Checker->Printer, ThePS);
            // cout << TempLV->ToString() << endl << endl;

            // audupa: DEBUG END

            // Self loop??
            if (CurEndOfPath->Equals(StartOfLoop)) {
                return (new LivenessViolation(InitState, StemPath, PathSoFar,
                                              Checker->Printer, ThePS));
            }

            // Now connect this path back to the original state
            vector<PSTraceElemT> LoopBack;
            auto FinalPair = DoUnwoundBFS(CurEndOfPath, Checker, InvPermAlongPath,
                                          [&] (u32 CmdID, const ProductState* State) -> bool
                                          {
                                              return (State->Equals(StartOfLoop));
                                          },
                                          LoopBack, SCCNodes);

            FinalPair.first->GetSVPtr()->Recycle();
            delete FinalPair.first;

            PathSoFar.insert(PathSoFar.end(), LoopBack.begin(), LoopBack.end());
            
            return (new LivenessViolation(InitState, StemPath, PathSoFar,
                                          Checker->Printer, ThePS));
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
                auto MsgTypeAsRec = MsgType->SAs<Exprs::ExprRecordType>();
                if (Verbosity < 1) {
                    sstr << "Fired Guarded Command with label: " 
                         << (MsgTypeAsRec != nullptr ? MsgTypeAsRec->GetName() : 
                             "(internal transition)") << endl;
                } else {
                    sstr << "Fired Guarded Command:" << endl;
                    sstr << TraceElem.first->ToString() << endl;
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
                                             const vector<PSTraceElemT>& LoopElems,
                                             StateVecPrinter* Printer,
                                             const ProductStructure* ThePS)
            : TraceBase(Printer), InitialState(InitialState), ThePS(ThePS),
              StemPath(StemElems), LoopPath(LoopElems)
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
            for (auto const& TraceElem : StemPath) {
                auto const& MsgType = TraceElem.first->GetMsgType();
                auto MsgTypeAsRec = MsgType->SAs<Exprs::ExprRecordType>();
                if (Verbosity < 1) {
                    sstr << "Fired Guarded Command with label: " 
                         << (MsgTypeAsRec != nullptr ? MsgTypeAsRec->GetName() : 
                             "(internal transition)") << endl;
                } else {
                    sstr << "Fired Guarded Command:" << endl;
                    sstr << TraceElem.first->ToString() << endl;
                }
                sstr << "Obtained next state (delta from previous state):" << endl;
                sstr << "-----------------------------------------------------" << endl;
                Printer->PrintState(TraceElem.second, PrevState, ThePS, sstr);
                sstr << "-----------------------------------------------------" << endl << endl;
                PrevState = TraceElem.second;                
            }

            sstr << "First state of the loop (in full), which is same as last state of " 
                 << "the stem printed above:" << endl;
            sstr << "-----------------------------------------------------" << endl;
            Printer->PrintState(PrevState, ThePS, sstr);
            sstr << "-----------------------------------------------------" << endl << endl;
            
            for (auto const& TraceElem : LoopPath) {
                auto const& MsgType = TraceElem.first->GetMsgType();
                auto MsgTypeAsRec = MsgType->SAs<Exprs::ExprRecordType>();
                if (Verbosity < 1) {
                    sstr << "Fired Guarded Command with label: " 
                         << (MsgTypeAsRec != nullptr ? MsgTypeAsRec->GetName() : 
                             "(internal transition)") << endl;
                } else {
                    sstr << "Fired Guarded Command:" << endl;
                    sstr << TraceElem.first->ToString() << endl;
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
            return sstr.str();
        }

    } /* end namespace MC */
} /* end namespace ESMC */

// 
// Trace.cpp ends here
