// LTSChecker.cpp ---
//
// Filename: LTSChecker.cpp
// Author: Abhishek Udupa
// Created: Tue Aug 19 04:03:22 2014 (-0400)
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

#include <stack>
#include <deque>

#include "../uflts/LabelledTS.hpp"
#include "../symmetry/SymmCanonicalizer.hpp"
#include "../uflts/LTSAssign.hpp"
#include "../uflts/LTSTransitions.hpp"
#include "../uflts/LTSFairnessSet.hpp"
#include "../uflts/LTSEFSMBase.hpp"
#include "../uflts/LTSUtils.hpp"

#include "LTSChecker.hpp"
#include "Compiler.hpp"
#include "StateVec.hpp"
#include "StateVecPrinter.hpp"
#include "AQStructure.hpp"
#include "OmegaAutomaton.hpp"
#include "IndexSet.hpp"
#include "Trace.hpp"

namespace ESMC {
    namespace MC {

        using namespace ESMC::LTS;
        using ESMC::Symm::Canonicalizer;

        namespace Detail {

            DFSStackEntry::DFSStackEntry()
                : State(nullptr), LastFired(-1)
            {
                // Nothing here
            }

            DFSStackEntry::DFSStackEntry(StateVec* State)
                : State(State), LastFired(-1)
            {
                // Nothing here
            }

            DFSStackEntry::DFSStackEntry(StateVec* State, u32 LastFired)
                : State(State), LastFired(LastFired)
            {
                // Nothing here
            }

            DFSStackEntry::DFSStackEntry(const DFSStackEntry& Other)
                : State(Other.State), LastFired(Other.LastFired)
            {
                // Nothing here
            }

            DFSStackEntry::~DFSStackEntry()
            {
                // Nothing here
            }

            DFSStackEntry& DFSStackEntry::operator = (const DFSStackEntry& Other)
            {
                if (&Other == this) {
                    return *this;
                }
                State = Other.State;
                LastFired = Other.LastFired;
                return *this;
            }

            bool DFSStackEntry::operator == (const DFSStackEntry& Other) const
            {
                return (State == Other.State && LastFired == Other.LastFired);
            }

            i64& DFSStackEntry::GetLastFired()
            {
                return LastFired;
            }

            i64 DFSStackEntry::GetLastFired() const
            {
                return LastFired;
            }

            const StateVec* DFSStackEntry::GetState() const
            {
                return State;
            }

            StateVec* DFSStackEntry::GetState()
            {
                return State;
            }

            void DFSStackEntry::SetLastFired(i64 NewLastFired)
            {
                LastFired = NewLastFired;
            }

            // BFSQueueT implementation
            const u32 BFSQueueT::MaxNumBuckets = 32;

            BFSQueueT::BFSQueueT(bool UsePrioQueue)
                : BFSDeque(UsePrioQueue ? nullptr : new deque<StateVec*>()),
                  PrioQueues(UsePrioQueue ?
                             new vector<deque<StateVec*>*>(MaxNumBuckets, nullptr) :
                             nullptr),
                  NumElems(0)
            {
                // Nothing here
            }

            BFSQueueT::~BFSQueueT()
            {
                if (BFSDeque != nullptr) {
                    delete BFSDeque;
                }
                if (PrioQueues != nullptr) {
                    for (auto const& Queue : *PrioQueues) {
                        if (Queue != nullptr) {
                            delete Queue;
                        }
                    }
                    delete PrioQueues;
                }
            }

            void BFSQueueT::Push(StateVec* State, u32 TaintLevel)
            {
                if (BFSDeque != nullptr) {
                    BFSDeque->push_back(State);
                } else {
                    auto PrioIndex = min(MaxNumBuckets - 1, TaintLevel);
                    if ((*PrioQueues)[PrioIndex] == nullptr) {
                        (*PrioQueues)[PrioIndex] = new deque<StateVec*>();
                    }
                    (*PrioQueues)[PrioIndex]->push_back(State);
                }
                ++NumElems;
            }

            StateVec* BFSQueueT::Pop(u32& TaintLevel)
            {
                StateVec* Retval = nullptr;
                if (BFSDeque != nullptr) {
                    Retval = BFSDeque->front();
                    BFSDeque->pop_front();
                    TaintLevel = 0;
                } else {
                    // look for a node with taint level one first
                    if ((*PrioQueues)[1] != nullptr && (*PrioQueues)[1]->size() > 0) {
                        TaintLevel = 1;
                        Retval = (*PrioQueues)[1]->front();
                        (*PrioQueues)[1]->pop_front();
                        --NumElems;
                        return Retval;
                    }
                    // No? Okay, let's try to get a node with taint level 0
                    if ((*PrioQueues)[0] != nullptr && (*PrioQueues)[0]->size() > 0) {
                        TaintLevel = 0;
                        Retval = (*PrioQueues)[0]->front();
                        (*PrioQueues)[0]->pop_front();
                        --NumElems;
                        return Retval;
                    }
                    // No? Okay, traverse down the rest of the buckets
                    for (u32 i = 2; i < MaxNumBuckets; ++i) {
                        if ((*PrioQueues)[i] != nullptr && (*PrioQueues)[i]->size() > 0) {
                            Retval = (*PrioQueues)[i]->front();
                            (*PrioQueues)[i]->pop_front();
                            TaintLevel = i;
                            break;
                        }
                    }
                }
                --NumElems;
                return Retval;
            }

            u32 BFSQueueT::Size() const
            {
                return NumElems;
            }

            // Fairness Checker implementation
            FairnessChecker::FairnessChecker(const LTSFairSetRef& FairSet,
                                             SystemIndexSet* SysIdxSet,
                                             const vector<GCmdRef>& GuardedCommands,
                                             LTSChecker* Checker)
                : Enabled(false), Disabled(false), Executed(false),
                  FairSet(FairSet), NumInstances(FairSet->GetNumInstances()),
                  IsStrong(FairSet->GetFairnessType() == FairSetFairnessType::Strong),
                  SysIdxSet(SysIdxSet), ClassID(FairSet->GetClassID()),
                  GCmdsToRespondTo(NumInstances, BitSet(GuardedCommands.size(), false)),
                  GCmdIDsToRespondTo(NumInstances),
                  Checker(Checker),
                  EnabledPerInstance(NumInstances, false),
                  ExecutedPerInstance(NumInstances, false),
                  DisabledPerInstance(NumInstances, false),
                  SatisfiedInTrace(NumInstances, false)
            {
                for (u32 Instance = 0; Instance < NumInstances; ++Instance) {
                    const u32 NumGuardedCommands = GuardedCommands.size();
                    for (u32 i = 0; i < NumGuardedCommands; ++i) {
                        auto const& Cmd = GuardedCommands[i];
                        auto const& FairObjs = Cmd->GetFairnessObjsSatisfied();
                        for (auto const& FairObj : FairObjs) {
                            if (FairObj->GetFairnessSet() == FairSet &&
                                FairObj->GetInstanceNumber() == Instance) {
                                GCmdsToRespondTo[Instance].Set(i);
                                GCmdIDsToRespondTo[Instance].insert(i);
                            }
                        }
                    }
                }
            }

            FairnessChecker::~FairnessChecker()
            {
                // Nothing here
            }

            void FairnessChecker::InitializeForNewSCC()
            {
                InitializeForNewThread();
                EnabledPerInstance.Clear();
                ExecutedPerInstance.Clear();
                DisabledPerInstance.Clear();
            }

            void FairnessChecker::InitializeForNewThread()
            {
                Enabled = false;
                Disabled = false;
                Executed = false;
                EnabledStates.clear();
            }

            void FairnessChecker::Reset()
            {
                InitializeForNewSCC();
                ClearTraceSatisfactionBits();
            }

            void FairnessChecker::ClearTraceSatisfactionBits() const
            {
                SatisfiedInTrace.Clear();
            }

            bool FairnessChecker::IsInstanceSatisfiedInTrace(u32 Instance) const
            {
                return SatisfiedInTrace.Test(Instance);
            }

            void FairnessChecker::SetInstanceSatisfiedInTrace(u32 Instance) const
            {
                SatisfiedInTrace.Set(Instance);
            }

            bool FairnessChecker::CheckInstanceSatisfaction(u32 Instance) const
            {
                return (!EnabledPerInstance.Test(Instance));
            }

            bool FairnessChecker::CheckInstanceSatisfaction(u32 Instance,
                                                            u32 CmdID,
                                                            const ProductState* ReachedState) const
            {
                // Do we even need to look at the cmd and the state?
                if (CheckInstanceSatisfaction(Instance)) {
                    return true;
                }

                // Assumes that we're not trivially satisfied
                if (IsStrong || !DisabledPerInstance.Test(Instance)) {
                    return GCmdsToRespondTo[Instance].Test(CmdID);
                } else {
                    for (auto CmdID : GCmdIDsToRespondTo[Instance]) {
                        bool Exception;
                        ExpT NEPred;
                        auto NSVec = TryExecuteCommand(Checker->GuardedCommands[CmdID],
                                                       ReachedState->GetSVPtr(), Exception,
                                                       NEPred);
                        if (NSVec != nullptr) {
                            NSVec->Recycle();
                            return false;
                        }
                    }
                    return true;
                }
            }

            void FairnessChecker::AcceptSCCState(const ProductState* State,
                                                 const ProductEdgeSetT& Edges,
                                                 u32 CurrentTrackedIndex,
                                                 u32 OriginalTrackedIndex)
            {
                auto CurrentInstance =
                    SysIdxSet->GetIndexForClassID(CurrentTrackedIndex, ClassID);
                auto OriginalInstance =
                    SysIdxSet->GetIndexForClassID(OriginalTrackedIndex, ClassID);

                if (CurrentInstance < 0 || OriginalInstance < 0) {
                    return;
                }

                ESMC_LOG_FULL(
                              "Checker.Fairness",
                              Out_ << "Fairness Checker for fairness set: "
                                   << FairSet->GetName() << " on EFSM "
                                   << FairSet->GetEFSM()->GetName()
                                   << " with current tracked index "
                                   << CurrentTrackedIndex
                                   << " and original tracked index "
                                   << OriginalTrackedIndex << endl;
                              Out_ << "Considering state:" << endl;
                              Checker->Printer->PrintState(State, Out_);
                              Out_ << endl;
                              );

                auto const SCCID = State->Status.InSCC;

                bool AtLeastOneEnabled = false;
                for (auto const& Edge : Edges) {
                    auto NextState = Edge->GetTarget();
                    auto GCmdIndex = Edge->GetGCmdIndex();
                    if (!GCmdsToRespondTo[CurrentInstance].Test(GCmdIndex)) {
                        continue;
                    }

                    // This command causes this state to be marked enabled

                    ESMC_LOG_SHORT(
                                  "Checker.Fairness",
                                  Out_ << "Marking as Enabled, because command "
                                       << GCmdIndex << " is enabled." << endl;
                                  );

                    Enabled = true;
                    EnabledPerInstance.Set(OriginalInstance);
                    AtLeastOneEnabled = true;

                    if (NextState->IsInSCC(SCCID)) {

                        ESMC_LOG_SHORT(
                                      "Checker.Fairness",
                                      auto PermSet = Checker->TheCanonicalizer->GetPermSet();
                                      Out_ << "Marking as Executed, because, next state is in SCC"
                                           << endl;
                                      Out_ << "Next State:" << endl;
                                      Checker->Printer->PrintState(NextState, Out_);
                                      Out_ << endl << "With permutation:" << endl << endl;
                                      PermSet->Print(Edge->GetPermutation(), Out_);
                                      Out_ << endl;
                                      );

                        Executed = true;
                        ExecutedPerInstance.Set(OriginalInstance);
                        if (EnabledStates.size() > 0) {
                            EnabledStates.clear();
                        }
                    } else {
                        EnabledStates.insert(State);
                    }
                }

                // if no commands are enabled, then we're disabled!
                if (!AtLeastOneEnabled) {
                    Disabled = true;
                    DisabledPerInstance.Set(OriginalInstance);
                }
            }

            string FairnessChecker::ToString(u32 Verbosity) const
            {
                return
                    ((string)"Fairness Checker for fairness set: " +
                     FairSet->GetName() + " on EFSM " + FairSet->GetEFSM()->GetName() +
                     " with " + to_string(NumInstances) + " instances");
            }

            bool FairnessChecker::IsFair() const
            {
                if (IsStrong) {
                    return (!Enabled || Executed);
                } else {
                    return (Disabled || Executed);
                }
            }

            bool FairnessChecker::IsStrongFairness() const
            {
                return IsStrong;
            }

            bool FairnessChecker::IsEnabled() const
            {
                return Enabled;
            }

            bool FairnessChecker::IsExecuted() const
            {
                return Executed;
            }

            bool FairnessChecker::IsDisabled() const
            {
                return Disabled;
            }

            bool FairnessChecker::IsEnabled(u32 Instance) const
            {
                return EnabledPerInstance.Test(Instance);
            }

            bool FairnessChecker::IsExecuted(u32 Instance) const
            {
                return ExecutedPerInstance.Test(Instance);
            }

            bool FairnessChecker::IsDisabled(u32 Instance) const
            {
                return DisabledPerInstance.Test(Instance);
            }

            const unordered_set<const ProductState*>& FairnessChecker::GetEnabledStates() const
            {
                return EnabledStates;
            }

            void FairnessChecker::Permute(const vector<u08>& Permutation)
            {
                // Construct a local permutation vector for my instances
                vector<u32> InstancePermutation(NumInstances, 0);
                for (u32 Instance = 0; Instance < NumInstances; ++Instance) {
                    auto GlobalIndex =
                        SysIdxSet->GetIndexIDForClassIndex(Instance, ClassID);
                    auto PermutedGlobalIndex =
                        SysIdxSet->Permute(GlobalIndex, Permutation);
                    auto PermutedInstance =
                        SysIdxSet->GetIndexForClassID(PermutedGlobalIndex, ClassID);
                    InstancePermutation[Instance] = PermutedInstance;
                }

                // Apply the local permutation to all the per instance structures
                auto OriginalGCmdsToRespondTo = GCmdsToRespondTo;
                auto OriginalGCmdIDsToRespondTo = GCmdIDsToRespondTo;
                auto OriginalExecutedPerInstance = ExecutedPerInstance;
                auto OriginalEnabledPerInstance = EnabledPerInstance;
                auto OriginalDisabledPerInstance = DisabledPerInstance;

                for (u32 Instance = 0; Instance < NumInstances; ++Instance) {
                    GCmdsToRespondTo[Instance] =
                        OriginalGCmdsToRespondTo[InstancePermutation[Instance]];
                    GCmdIDsToRespondTo[Instance] =
                        OriginalGCmdIDsToRespondTo[InstancePermutation[Instance]];
                    ExecutedPerInstance[Instance] =
                        OriginalExecutedPerInstance[InstancePermutation[Instance]];
                    EnabledPerInstance[Instance] =
                        OriginalEnabledPerInstance[InstancePermutation[Instance]];
                    DisabledPerInstance[Instance] =
                        OriginalDisabledPerInstance[InstancePermutation[Instance]];
                }
            }

        } /* end namespace Detail */

        using namespace Detail;

        // Extern helper function definitions
        bool ApplyUpdates(const vector<LTSAssignRef>& Updates,
                          const StateVec* InputState,
                          StateVec* OutputState,
                          ExpT& NEPred)
        {
            for (auto const& Update : Updates) {
                auto const& LHS = Update->GetLHS();
                auto const& RHS = Update->GetRHS();
                auto const& LHSInterp = LHS->ExtensionData.Interp->SAs<LValueInterpreter>();
                auto const& RHSInterp = RHS->ExtensionData.Interp;
                auto Status = LHSInterp->Update(RHSInterp, InputState, OutputState, NEPred);
                if (Status == UpdateStatusT::BoundsViolation) {
                    NEPred = Update->GetLoweredBoundsConstraint();
                    return false;
                } else if (Status == UpdateStatusT::EvalException) {
                    return false;
                }
            }
            return true;
        }

        StateVec* ExecuteCommand(const GCmdRef& Cmd,
                                 const StateVec* InputState,
                                 ExpT& NEPred)
        {
            auto Retval = InputState->Clone();
            auto const& Updates = Cmd->GetUpdates();
            auto Status = ApplyUpdates(Updates, InputState, Retval, NEPred);
            if (!Status) {
                auto Mgr = NEPred->GetMgr();
                auto GuardInterp = Cmd->GetGuard()->ExtensionData.Interp;
                auto const& GuardNEPred = GuardInterp->GetNoExceptionPredicate();
                NEPred = Mgr->MakeExpr(LTSOps::OpIMPLIES,
                                       Mgr->MakeExpr(LTSOps::OpAND, GuardNEPred,
                                                     Cmd->GetLoweredGuard()),
                                       NEPred);
                Retval->Recycle();
                return nullptr;
            } else {
                return Retval;
            }
        }

        StateVec* TryExecuteCommand(const GCmdRef& Cmd,
                                    const StateVec* InputState,
                                    bool& Exception,
                                    ExpT& NEPred)
        {
            Exception = false;

            auto const& Guard = Cmd->GetGuard();
            auto GuardInterp = Guard->ExtensionData.Interp;
            auto Res = GuardInterp->Evaluate(InputState);
            if (Res == ExceptionValue) {
                Exception = true;
                NEPred = GuardInterp->GetNoExceptionPredicate();
                return nullptr;
            } else if (Res == 0) {
                return nullptr;
            } else {
                auto OutState = ExecuteCommand(Cmd, InputState, NEPred);
                if (OutState == nullptr) {
                    Exception = true;
                    return nullptr;
                }
                return OutState;
            }
        }

        inline bool LTSChecker::RecordErrorState(const StateVec* ErrorState,
                                                 const ExpT& BlownInvariant,
                                                 u32 MaxErrors)
        {
            if (ErrorStateSet.find(ErrorState) != ErrorStateSet.end()) {
                return true;
            } else {
                ErrorStateSet[ErrorState] = BlownInvariant;
                ErrorStates.push_back(make_pair(ErrorState, BlownInvariant));
                return (ErrorStates.size() < MaxErrors);
            }
        }

        LTSChecker::LTSChecker(LabelledTS* TheLTS)
            : TheLTS(TheLTS), AQS(nullptr), ThePS(nullptr)
        {
            // Freeze the LTS in any case
            TheLTS->Freeze();
            Compiler = new LTSCompiler();
            Compiler->CompileLTS(TheLTS);
            Factory = new StateFactory(TheLTS->StateVectorSize);
            Printer = new StateVecPrinter(TheLTS, Compiler);
            TheCanonicalizer = new Canonicalizer(TheLTS, Printer);
            GuardedCommands = TheLTS->GetGuardedCmds();
            ZeroState = Factory->MakeState();
            NumGuardedCmds = GuardedCommands.size();

            ESMC_LOG_FULL("Checker.GuardedCommands",
                          Out_ << "Guarded Commands:" << endl;
                          for (auto const& Cmd : GuardedCommands) {
                              Out_ << Cmd << endl;
                          });

            NumFairnessObjects = 0;
            NumFairnessSets = 0;

            for (auto const& NameEFSM : TheLTS->AllEFSMs) {
                auto EFSM = NameEFSM.second;
                auto const& FairnessGroup = EFSM->GetFairnessGroup();
                auto const& FairnessSetMap = FairnessGroup->GetAllFairnessSets();
                NumFairnessSets += FairnessSetMap.size();
            }

            vector<vector<vector<ExpT>>> FairnessSetInsts(NumFairnessSets);
            FairnessCheckers = vector<vector<FairnessChecker*>>(NumFairnessSets);

            for (auto const& NameEFSM : TheLTS->AllEFSMs) {
                auto EFSM = NameEFSM.second;
                auto const& FairnessGroup = EFSM->GetFairnessGroup();
                auto const& FairnessSetMap = FairnessGroup->GetAllFairnessSets();
                for (auto const& NameFairnessSet : FairnessSetMap) {
                    auto const& FairnessSet = NameFairnessSet.second;
                    FairnessSetInsts[FairnessSet->GetClassID()] = FairnessSet->GetAllInstances();
                    NumFairnessObjects += FairnessSet->GetNumInstances();
                }
            }

            SysIdxSet = new SystemIndexSet(FairnessSetInsts);

            for (auto const& NameEFSM : TheLTS->AllEFSMs) {
                auto EFSM = NameEFSM.second;
                auto const& FairnessGroup = EFSM->GetFairnessGroup();
                auto const& FairnessSetMap = FairnessGroup->GetAllFairnessSets();
                for (auto const& NameFairnessSet : FairnessSetMap) {
                    auto const& FairnessSet = NameFairnessSet.second;
                    auto ClassID = FairnessSet->GetClassID();
                    auto CurChecker = new FairnessChecker(FairnessSet, SysIdxSet,
                                                          GuardedCommands, this);
                    if (FairnessSet->GetFairnessType() == FairSetFairnessType::Strong) {
                        FairnessCheckers[ClassID].push_back(CurChecker);
                    } else {
                        FairnessCheckers[ClassID].insert(FairnessCheckers[ClassID].begin(),
                                                         CurChecker);
                    }
                }
            }

            auto Mgr = TheLTS->GetMgr();
            // Make the deadlock freedom invariant
            vector<ExpT> DLFDisjunctions;
            for (auto const& Cmd : GuardedCommands) {
                DLFDisjunctions.push_back(Cmd->GetGuard());
            }

            DeadlockFreeInvariant = MakeDisjunction(DLFDisjunctions, Mgr);
            DeadlockFreeInvariant = Mgr->Simplify(DeadlockFreeInvariant);

            LoweredInvariant =
                Mgr->ApplyTransform<LTS::Detail::ArrayRValueTransformer>(TheLTS->InvariantExp);
            LoweredDLFInvariant =
                Mgr->ApplyTransform<LTS::Detail::ArrayRValueTransformer>(DeadlockFreeInvariant);
            LoweredDLFInvariant = Mgr->SimplifyFP(LoweredDLFInvariant);
        }

        LTSChecker::~LTSChecker()
        {
            ZeroState->Recycle();
            delete TheLTS;
            delete Compiler;
            delete Factory;
            delete TheCanonicalizer;
            delete Printer;
            if (AQS != nullptr) {
                delete AQS;
            }
            if (ThePS != nullptr) {
                delete ThePS;
            }

            for (auto const& Aut : AllBuchiAutomata) {
                delete Aut.second;
            }
            delete SysIdxSet;
            for (auto const& FCheckList : FairnessCheckers) {
                for (auto FChecker : FCheckList) {
                    delete FChecker;
                }
            }
        }

        inline const GCmdRef& LTSChecker::GetNextEnabledCmd(StateVec* State, i64& LastFired,
                                                            bool& Exception, ExpT& NEPred)
        {
            Exception = false;
            NEPred = ExpT::NullPtr;

            set<u32>::iterator it;
            if (LastFired == -1) {
                it = InterpretedCommands.begin();
            } else {
                it = next(InterpretedCommands.find(LastFired));
            }
            while (it != InterpretedCommands.end()) {
                auto CurCmdID = *it;
                auto const& Guard = GuardedCommands[CurCmdID]->GetGuard();
                auto Interp = Guard->ExtensionData.Interp;
                auto GRes = Interp->Evaluate(State);
                if (GRes == ExceptionValue) {
                    Exception = true;
                    NEPred = Interp->GetNoExceptionPredicate();
                    return GCmdRef::NullPtr;
                } else if (GRes != 0) {
                    LastFired = CurCmdID;
                    return GuardedCommands[CurCmdID];
                }
                ++it;
            }
            return GCmdRef::NullPtr;
        }

        inline void LTSChecker::DoDFS(StateVec *Root, u32 NumErrors)
        {
            stack<DFSStackEntry> DFSStack;
            AQS->InsertInitState(Root);
            DFSStack.push(DFSStackEntry(Root));
            auto const& Invar = TheLTS->GetInvariant();

            while (DFSStack.size() > 0) {
                auto& CurEntry = DFSStack.top();
                auto State = CurEntry.GetState();

                ESMC_LOG_FULL(
                              "Checker.AQSDetailed",
                              Out_ << "[DFS] Considering State:" << endl;
                              Out_ << "--------------------------------------------------------"
                                   << endl;
                              Printer->PrintState(State, Out_);
                              Out_ << "--------------------------------------------------------"
                                   << endl;
                              );

                auto& LastFired = CurEntry.GetLastFired();
                bool Deadlocked = (LastFired == -1);
                StateVec* NextState = nullptr;

                bool Exception = false;
                ExpT NEPred = ExpT::NullPtr;
                auto const& Cmd = GetNextEnabledCmd(State, LastFired, Exception, NEPred);

                if (Exception) {
                    if (!RecordErrorState(State, NEPred, NumErrors)) {
                        return;
                    }
                } else if (Cmd == GCmdRef::NullPtr) {
                    if (Deadlocked) {
                        if (!RecordErrorState(State, LoweredDLFInvariant, NumErrors)) {
                            return;
                        }
                    }

                    ESMC_LOG_SHORT(
                                   "Checker.AQSDetailed",
                                   Out_ << "No more successors, popping from stack!" << endl;
                                   );

                    // Done exploring this state
                    DFSStack.pop();
                    continue;
                }

                // Successors remain to be explored
                NextState = ExecuteCommand(Cmd, State, NEPred);

                if (NextState == nullptr) {
                    // An exception was encountered firing this
                    // command on the State. Store it, and pop from
                    // stack, since only one error is remembered
                    // per state
                    if (!RecordErrorState(State, NEPred, NumErrors)) {
                        return;
                    }
                    DFSStack.pop();
                    continue;
                }

                ESMC_LOG_SHORT(
                               "Checker.AQSDetailed",
                               Out_ << "Firing guarded command:" << endl;
                               Out_ << Cmd->ToString() << endl;
                               Out_ << "Got Next State (Uncanonicalized):" << endl;
                               Out_ << "--------------------------------------------------------"
                                    << endl;
                               Printer->PrintState(NextState, Out_);
                               Out_ << "--------------------------------------------------------"
                                    << endl;
                               );


                u32 PermID;
                auto CanonState = TheCanonicalizer->Canonicalize(NextState, PermID, AQS);

                ESMC_LOG_SHORT(
                               "Checker.AQSDetailed",
                               Out_ << "Canonicalized Next State with Permutation ID "
                                    << PermID << ":" << endl;
                               Out_ << "--------------------------------------------------------"
                                    << endl;
                               if (CanonState->Equals(*NextState)) {
                                   Out_ << "Canonicalized state is the same as "
                                        << "uncanonicalized state!" << endl;
                               } else {
                                   Printer->PrintState(CanonState, Out_);
                               }
                               Out_ << "--------------------------------------------------------"
                                    << endl;
                               );

                NextState->Recycle();

                auto ExistingState = AQS->Find(CanonState);

                if (ExistingState == nullptr) {

                    AQS->Insert(CanonState);

                    ESMC_LOG_SHORT(
                                   "Checker.AQSDetailed",
                                   Out_ << "Pushed new successor onto stack." << endl;
                                   );

                    AQS->AddEdge(State, CanonState, PermID, LastFired);
                    DFSStack.push(DFSStackEntry(CanonState));

                    // This is a new state, check for error
                    auto Interp = Invar->ExtensionData.Interp;
                    auto InvarRes = Interp->Evaluate(CanonState);
                    if (InvarRes == ExceptionValue) {
                        if (!RecordErrorState(CanonState, Interp->GetNoExceptionPredicate(),
                                              NumErrors)) {
                            return;
                        }
                        DFSStack.pop();
                    } else if (InvarRes == 0) {
                        if (!RecordErrorState(CanonState, LoweredInvariant, NumErrors)) {
                            return;
                        }
                        DFSStack.pop();
                    }
                    continue;
                } else {
                    // Successor already explored, add edge
                    // and continue

                    ESMC_LOG_SHORT(
                                   "Checker.AQSDetailed",
                                   Out_ << "Successor has been previously encountered." << endl;
                                   );

                    AQS->AddEdge(State, ExistingState, PermID, LastFired);
                    CanonState->Recycle();
                    continue;
                }
            }
        }

        inline void LTSChecker::DoBFS(const vector<StateVec*>& Roots, u32 NumErrors,
                                      bool PrioritizeNonTentative)
        {
            BFSQueueT BFSQueue(PrioritizeNonTentative);
            BFSQueue.Push(Roots.begin(), Roots.end(), 0);

            auto const& Invar = TheLTS->GetInvariant();

            for (auto const& Root : Roots) {
                AQS->InsertInitState(Root);
            }

            while (BFSQueue.Size() > 0) {
                u32 CurrentTaintLevel;
                auto CurState = BFSQueue.Pop(CurrentTaintLevel);

                ESMC_LOG_FULL(
                              "Checker.AQSDetailed",
                              Out_ << "[BFS] Considering State:" << endl;
                              Out_ << "--------------------------------------------------------"
                              << endl;
                              Printer->PrintState(CurState, Out_);
                              Out_ << "--------------------------------------------------------"
                                   << endl;
                              );

                bool Deadlocked = true;

                for (auto i : InterpretedCommands) {

                    auto const& Cmd = GuardedCommands[i];

                    StateVec* NextState = nullptr;
                    bool Exception = false;
                    ExpT NEPred;

                    NextState = TryExecuteCommand(Cmd, CurState, Exception, NEPred);
                    if (Exception) {
                        Deadlocked = false;
                        if (!RecordErrorState(CurState, NEPred, NumErrors)) {
                            return;
                        }
                        // Already an error. Don't try any more commands
                        break;
                    } else if (NextState == nullptr) {
                        continue;
                    }

                    // valid next state

                    u32 PermID = 0;
                    Deadlocked = false;

                    ESMC_LOG_SHORT(
                                   "Checker.AQSDetailed",
                                   Out_ << "Got Next State (Uncanonicalized), by firing "
                                   << "guarded command:" << endl << Cmd->ToString() << endl;
                                   Out_ << "--------------------------------------------------------"
                                   << endl;
                                   Printer->PrintState(NextState, Out_);
                                   Out_ << "--------------------------------------------------------"
                                   << endl;
                                   );

                    auto CanonNextState = TheCanonicalizer->Canonicalize(NextState, PermID, AQS);

                    ESMC_LOG_SHORT(
                                   "Checker.AQSDetailed",
                                   Out_ << "Canonicalized Next State with Permutation ID "
                                   << PermID << ":" << endl;
                                   Out_ << "--------------------------------------------------------"
                                   << endl;
                                   if (CanonNextState->Equals(*NextState)) {
                                       Out_ << "Canonicalized state is the same as "
                                            << "uncanonicalized state!" << endl;
                                   } else {
                                       Printer->PrintState(CanonNextState, Out_);
                                   }
                                   Out_ << "--------------------------------------------------------"
                                   << endl;
                                   );

                    NextState->Recycle();

                    auto ExistingState = AQS->Find(CanonNextState);
                    if (ExistingState != nullptr) {
                        AQS->AddEdge(CurState, ExistingState, PermID, i);
                        CanonNextState->Recycle();
                        ESMC_LOG_SHORT(
                                       "Checker.AQSDetailed",
                                       Out_ << "Successor has been previously encountered."
                                       << endl;
                                       );
                    } else {

                        ESMC_LOG_SHORT(
                                       "Checker.AQSDetailed",
                                       Out_ << "Enqueueing new successor onto BFS queue."
                                       << endl;
                                       );

                        AQS->Insert(CanonNextState);
                        AQS->AddEdge(CurState, CanonNextState, PermID, i);

                        // New state, check for errors;
                        auto Interp = Invar->ExtensionData.Interp;
                        auto InvarRes = Interp->Evaluate(CanonNextState);
                        if (InvarRes == ExceptionValue) {
                            if (!RecordErrorState(CanonNextState,
                                                  LoweredInvariant,
                                                  NumErrors)) {
                                return;
                            }
                        } else if (InvarRes == 0) {
                            // Again, remember this state, but continue
                            // on with the AQS construction
                            if (!RecordErrorState(CanonNextState,
                                                  LoweredInvariant,
                                                  NumErrors)) {
                                return;
                            }
                        } else {
                            // We're good to consider successors of this state
                            BFSQueue.Push(CanonNextState,
                                          (Cmd->IsTentative() ?
                                           CurrentTaintLevel + 1 :
                                           CurrentTaintLevel));
                        }
                    } // end if (ExistingState != nullptr), else

                } // end iterating over commands

                if (Deadlocked) {
                    if (!RecordErrorState(CurState, LoweredDLFInvariant, NumErrors)) {
                        return;
                    }
                }
            }
        }

        bool LTSChecker::BuildAQS(AQSConstructionMethod Method,
                                  bool PrioritizeNonTentative,
                                  u32 NumErrors)
        {
            vector<TraceBase*> Retval;

            if (Method != AQSConstructionMethod::BreadthFirst &&
                PrioritizeNonTentative) {
                throw ESMCError((string)"Non tentative edges can be prioritized only " +
                                "if construction method is breadth-first");
            }

            if (AQS != nullptr) {
                return (ErrorStates.size() == 0);
            }

            // Gather the commands that are relevant
            InterpretedCommands.clear();

            for (auto const& Cmd : GuardedCommands) {
                if (Cmd->IsFullyInterpreted()) {
                    InterpretedCommands.insert(Cmd->GetCmdID());
                }
            }

            AQS = new AQStructure(TheLTS);

            auto const& ISGens = TheLTS->GetInitStateGenerators();
            vector<StateVec*> InitStates;

            for (auto const& ISGen : ISGens) {
                auto CurState = Factory->MakeState();
                ExpT NEPred;
                if (!ApplyUpdates(ISGen->GetUpdates(), ZeroState, CurState, NEPred)) {
                    throw ESMCError((string)"Could not construct initial state!");
                }

                ESMC_LOG_FULL(
                              "Checker.AQSDetailed",
                              Out_ << "Initial State (Uncanonicalized):" << endl;
                              Out_ << "--------------------------------------------------------"
                                   << endl;
                              Printer->PrintState(CurState, Out_);
                              Out_ << "--------------------------------------------------------"
                                   << endl;
                              );

                u32 CanonPerm = 0;
                auto CanonState = TheCanonicalizer->Canonicalize(CurState, CanonPerm);

                ESMC_LOG_FULL(
                              "Checker.AQSDetailed",
                              Out_ << "Initial State (Canonicalized):" << endl;
                              Out_ << "--------------------------------------------------------"
                                   << endl;
                              if (CanonState->Equals(*CurState)) {
                                  Out_ << "Same as uncanonicalized initial state." << endl;
                              } else {
                                  Printer->PrintState(CurState, Out_);
                              }
                              Out_ << "--------------------------------------------------------"
                                   << endl;
                              );

                CurState->Recycle();

                InitStates.push_back(CanonState);
            }

            if (Method == AQSConstructionMethod::DepthFirst) {
                for (auto InitState : InitStates) {
                    auto ExistingState = AQS->Find(InitState);
                    if (ExistingState == nullptr) {
                        DoDFS(InitState, NumErrors);
                    } else if (ExistingState != InitState) {
                        InitState->Recycle();
                    }
                }
            } else {
                DoBFS(InitStates, NumErrors, PrioritizeNonTentative);
            }

            ESMC_LOG_MIN_SHORT(
                               Out_ << "AQS Built, contains " << AQS->GetNumStates()
                                    << " states and " << AQS->GetNumEdges() << " edges. "
                                    << endl;
                               if (ErrorStates.size() > 0) {
                                   Out_ << "AQS contains one or more errors, and may "
                                        << "not be complete." << endl;
                               }
                               );

            if (ErrorStates.size() > 0) {
                return false;
            }
            return true;
        }

        const LTSChecker::ErrorStateVecT& LTSChecker::GetAllErrorStates() const
        {
            return ErrorStates;
        }

        AQStructure* LTSChecker::GetAQS() const
        {
            return AQS;
        }

        ProductStructure* LTSChecker::GetPS() const
        {
            return ThePS;
        }

        TraceBase* LTSChecker::MakeTraceToError(const StateVec* ErrorState)
        {
            auto it = ErrorStateSet.find(ErrorState);
            if (it == ErrorStateSet.end()) {
                throw ESMCError((string)"Argument to LTSChecker::MakeTraceToError() is " +
                                "not marked as an error state by the checker");
            }
            auto const& BlownInvariant = it->second;
            if (BlownInvariant == LoweredInvariant) {
                return TraceBase::MakeSafetyViolation(ErrorState, this, LoweredInvariant);
            } else if (BlownInvariant == LoweredDLFInvariant) {
                return TraceBase::MakeDeadlockViolation(ErrorState, this);
            } else {
                return TraceBase::MakeBoundsViolation(ErrorState, this);
            }
        }

        void LTSChecker::ClearAQS()
        {
            if (AQS != nullptr) {
                delete AQS;
                AQS = nullptr;
                ZeroState->Recycle();
                delete Factory;
                Factory = new StateFactory(TheLTS->StateVectorSize);
                ZeroState = Factory->MakeState();
                ErrorStates.clear();
                ErrorStateSet.clear();
            }
            if (ThePS != nullptr) {
                delete ThePS;
                ThePS = nullptr;
            }
        }

        StateBuchiAutomaton*
        LTSChecker::MakeStateBuchiMonitor(const string& Name,
                                          const vector<ExpT>& SymmIndices,
                                          const ExpT& Constraint)
        {
            if (AllBuchiAutomata.find(Name) != AllBuchiAutomata.end()) {
                throw ESMCError((string)"Monitor named \"" + Name + "\" already exists " +
                                "in the LTS Checker");
            }
            auto Retval = new StateBuchiAutomaton(TheLTS, Name, SymmIndices,
                                                  Constraint, Compiler);
            AllBuchiAutomata[Name] = Retval;
            StateBuchiAutomata[Name] = Retval;
            BuchiMonitorNames.push_back(Name);
            return Retval;
        }

        inline void LTSChecker::ConstructProduct(StateBuchiAutomaton *Monitor)
        {
            deque<ProductState*> BFSQueue;
            ThePS = new ProductStructure(NumFairnessObjects, Monitor);
            auto MonIndexSet = Monitor->GetIndexSet();
            auto PermSet = TheCanonicalizer->GetPermSet();

            for (u32 i = 0; i < MonIndexSet->GetNumIndexVectors(); ++i) {
                for (auto const& MonInitState : Monitor->GetInitialStates()) {
                    for (auto const& AQSInitState : AQS->GetInitStates()) {
                        ThePS->AddInitialState(AQSInitState, MonInitState, i);
                    }
                }
            }

            // A BFS over the initial states
            auto const& InitStates = ThePS->GetInitialStates();
            BFSQueue.insert(BFSQueue.begin(), InitStates.begin(), InitStates.end());

            while (BFSQueue.size() > 0) {
                auto CurProdState = BFSQueue.front();
                BFSQueue.pop_front();

                auto SVPtr = CurProdState->GetSVPtr();
                auto MonState = CurProdState->GetMonitorState();
                if (Monitor->IsAccepting(MonState)) {
                    CurProdState->MarkAccepting();
                }
                auto IndexID = CurProdState->GetIndexID();
                auto const& AQSEdges = AQS->GetEdges(SVPtr);
                auto&& MonitorNextStates = Monitor->GetNextStates(MonState, IndexID, SVPtr);
                for (auto const& Edge : AQSEdges) {
                    auto NextSVPtr = Edge->GetTarget();
                    auto PermID = Edge->GetPermutation();
                    auto PermIt = PermSet->GetIterator(Edge->GetPermutation());
                    auto NextIndexID = MonIndexSet->Permute(IndexID, PermIt.GetPerm());
                    for (auto const& NextMonState : MonitorNextStates) {
                        bool New;
                        auto NextPS =
                            ThePS->AddState(NextSVPtr, NextMonState, NextIndexID, New);
                        if (New) {
                            BFSQueue.push_back(NextPS);
                        }
                        ThePS->AddEdge(CurProdState, NextPS, PermID, Edge->GetGCmdIndex());
                    }
                }
            }

            ESMC_LOG_MIN_SHORT(
                               Out_ << "Product construction complete!" << endl;
                               Out_ << "Product Structure contains " << ThePS->GetNumStates()
                                    << " states and " << ThePS->GetNumEdges()
                                    << " edges." << endl;
                               );
        }

        inline void LTSChecker::DoThreadedBFS(const ProductState *SCCRoot, u32 TrackedIndex)
        {
            deque<pair<const ProductState*, u32>> BFSQueue;
            SCCRoot->MarkTracked(TrackedIndex);
            const u32 SCCID = SCCRoot->Status.InSCC;
            // We can set the class id here,
            // since the classes will not change
            const u32 ClassID = SysIdxSet->GetClassID(TrackedIndex);

            ESMC_LOG_FULL(
                          "Checker.AQSDetailed",
                          Out_ << "Doing threaded BFS on product state:"
                               << endl;
                          Printer->PrintState(SCCRoot, Out_);
                          Out_ << "Tracking thread index: " << TrackedIndex << endl;
                          );

            BFSQueue.push_back(make_pair(SCCRoot, TrackedIndex));
            auto PermSet = TheCanonicalizer->GetPermSet();

            while (BFSQueue.size() > 0) {
                auto const& CurEntry = BFSQueue.front();

                auto CurState = CurEntry.first;
                auto CurIndex = CurEntry.second;
                BFSQueue.pop_front();

                auto const& CurEdges = ThePS->GetEdges(const_cast<ProductState*>(CurState));

                // process this state for fairness
                for (auto Checker : FairnessCheckers[ClassID]) {
                    Checker->AcceptSCCState(CurState, CurEdges, CurIndex, TrackedIndex);
                }

                for (auto Edge : CurEdges) {
                    auto NextState = Edge->GetTarget();
                    auto Permutation = Edge->GetPermutation();
                    auto PermIt = PermSet->GetIterator(Permutation);

                    if (!NextState->IsInSCC(SCCID)) {
                        // Not in this scc
                        continue;
                    }

                    auto NextIndex = SysIdxSet->Permute(CurIndex, PermIt.GetPerm());
                    if (NextState->IsTracked(NextIndex)) {
                        continue;
                    }

                    NextState->MarkTracked(NextIndex);
                    BFSQueue.push_back(make_pair(NextState, NextIndex));
                }
            }

            ESMC_LOG_FULL(
                          "Checker.AQSDetailed",
                          Out_ << "Threaded BFS done!" << endl;
                          );
        }

        inline bool LTSChecker::CheckSCCFairness(const ProductState *SCCRoot,
                                                 vector<const ProductState*>& UnfairStates)
        {
            for (auto FCheckers : FairnessCheckers) {
                for (auto FChecker : FCheckers) {
                    FChecker->InitializeForNewSCC();
                }
            }

            const u32 MaxTrackedIndex = SysIdxSet->GetNumTrackedIndices();
            for (u32 TrackedIndex = 0; TrackedIndex < MaxTrackedIndex; ++TrackedIndex) {

                if (SCCRoot->IsTracked(TrackedIndex)) {
                    continue;
                }

                auto ClassID = SysIdxSet->GetClassID(TrackedIndex);

                // Reset all the fairness checkers in this class
                for (auto FChecker : FairnessCheckers[ClassID]) {
                    FChecker->InitializeForNewThread();
                }

                ESMC_LOG_FULL(
                              "Checker.AQSDetailed",
                              Out_ << "Checking fairness of SCC with tracked index "
                                   << TrackedIndex << endl;
                              );

                DoThreadedBFS(SCCRoot, TrackedIndex);
                // Are all the fairness requirements satisfied?
                for (auto FChecker : FairnessCheckers[ClassID]) {
                    if (!FChecker->IsFair()) {
                        if (FChecker->IsStrongFairness()) {
                            auto const& EnabledStates = FChecker->GetEnabledStates();
                            UnfairStates.clear();
                            UnfairStates.insert(UnfairStates.end(), EnabledStates.begin(),
                                                EnabledStates.end());
                        }
                        return false;
                    }
                }
            }
            return true;
        }

        inline vector<const ProductState*>
        LTSChecker::GetAcceptingSCCs()
        {
            vector<const ProductState*> Retval;
            u32 CurIndex = 0;
            u32 CurSCCID = 0;

            auto const& AllStates = ThePS->GetAllStates();
            for (auto it = AllStates.begin(); it != AllStates.end(); ++it) {
                auto const& PS = it->first;
                if (PS->DFSNum == -1 && !PS->IsDeleted()) {
                    auto&& CurSCCRoots = GetAcceptingSCCsFromInitState(PS, CurIndex, CurSCCID);
                    Retval.insert(Retval.end(), CurSCCRoots.begin(), CurSCCRoots.end());
                }
            }
            return Retval;
        }

        inline vector<const ProductState*>
        LTSChecker::GetAcceptingSCCsFromInitState(const ProductState* InitState,
                                                  u32& LowestUnusedIndex,
                                                  u32& LowestUnusedSCCID)
        {
            // Find the SCCs on the fly
            stack<pair<const ProductState*, u32>> DFSStack;
            stack<ProductState*> SCCStack;
            vector<const ProductState*> Retval;

            u32 CurIndex = LowestUnusedIndex;
            u32 CurSCCID = LowestUnusedSCCID;

            InitState->DFSNum = CurIndex;
            InitState->LowLink = CurIndex;
            InitState->MarkOnStack();
            DFSStack.push(make_pair(InitState, 0));
            SCCStack.push(const_cast<ProductState*>(InitState));
            ++CurIndex;

            while (DFSStack.size() > 0) {
                auto const& CurEntry = DFSStack.top();
                auto CurState = const_cast<ProductState*>(CurEntry.first);
                auto EdgeToExplore = CurEntry.second;
                auto const& Edges = ThePS->GetEdges(CurState);

                if (EdgeToExplore >= Edges.size()) {
                    // We're done with this state
                    DFSStack.pop();
                    // Update my ancestor's lowlink
                    // if I have an ancestor!
                    if (DFSStack.size() > 0) {
                        auto PrevState = DFSStack.top().first;
                        PrevState->LowLink = min(PrevState->LowLink, CurState->LowLink);
                    }
                } else {
                    ++DFSStack.top().second;
                    // Push successor onto stack
                    // if unexplored, else update lowlink
                    auto it = Edges.begin();
                    for (u32 i = 0; i < EdgeToExplore; ++i) {
                        ++it;
                    }
                    auto CurEdge = *(it);
                    auto NextState = const_cast<ProductState*>(CurEdge->GetTarget());
                    if (NextState->DFSNum == -1 && (!(NextState->IsDeleted()))) {
                        // unexplored and not deleted
                        NextState->DFSNum = CurIndex;
                        NextState->LowLink = CurIndex;
                        NextState->MarkOnStack();
                        DFSStack.push(make_pair(NextState, 0));
                        SCCStack.push(NextState);
                        ++CurIndex;
                    } else if (NextState->IsOnStack()) {
                        // explored
                        CurState->LowLink = min(CurState->LowLink, NextState->DFSNum);
                    }
                    continue;
                }

                // We only get here if we've popped a state
                // from the DFS stack.
                if (CurState->LowLink == CurState->DFSNum) {
                    u32 NumStatesInSCC = 0;
                    ProductState* SCCState = nullptr;
                    vector<ProductState*> SCCStateVec;
                    bool FoundAccepting = false;
                    do {
                        SCCState = SCCStack.top();
                        SCCState->MarkInSCC(CurSCCID);
                        SCCState->MarkNotOnStack();
                        SCCStack.pop();
                        if (SCCState->IsAccepting()) {
                            FoundAccepting = true;
                        }
                        ++NumStatesInSCC;
                        SCCStateVec.push_back(SCCState);
                    } while (SCCState != CurState);

                    bool SCCIsGood = FoundAccepting;

                    if (SCCIsGood && NumStatesInSCC == 1) {
                        auto const& Edges = ThePS->GetEdges(SCCState);
                        bool SelfLoop = false;
                        for (auto const& Edge : Edges) {
                            if (Edge->GetTarget() == SCCState) {
                                SelfLoop = true;
                                break;
                            }
                        }
                        if (!SelfLoop) {
                            SCCState->MarkNotInSCC();
                            SCCIsGood = false;
                        } else {
                            SCCState->MarkSingular();
                        }
                    } else {
                        SCCState->MarkNotSingular();
                    }

                    if (!SCCIsGood) {
                        for (auto SCCState : SCCStateVec) {
                            SCCState->MarkNotInSCC();
                        }
                    } else {
                        ESMC_LOG_FULL("Checker.Fairness",
                                      Out_ << "Accepting SCC " << CurSCCID << ":" << endl;
                                      for (auto const& SCCState : SCCStateVec) {
                                          Out_ << "--------------------------------" << endl;
                                          Printer->PrintState(SCCState, Out_);
                                          Out_ << "--------------------------------" << endl;
                                      }
                                      Out_ << "End of accepting SCC" << endl;
                                      );

                        Retval.push_back(CurState);
                        ++CurSCCID;
                    }
                }
            }

            LowestUnusedSCCID = CurSCCID;
            LowestUnusedIndex = CurIndex;
            return Retval;
        }

        TraceBase* LTSChecker::CheckLiveness(const string& BuchiMonitorName)
        {
            auto it = AllBuchiAutomata.find(BuchiMonitorName);
            if (it == AllBuchiAutomata.end()) {
                throw ESMCError((string)"Buchi Monitor with name \"" + BuchiMonitorName +
                                "\" was not found to check liveness property");
            }
            if (AQS == nullptr) {
                throw ESMCError((string)"AQS not built to check liveness property!");
            }

            if (ErrorStates.size() > 0) {
                throw ESMCError((string)"AQS contains one or more deadlocked and/or " +
                                "error states. Liveness checks aborted due to this!");
            }

            auto Monitor = it->second->As<StateBuchiAutomaton>();
            if (Monitor == nullptr) {
                throw InternalError((string)"Monitor \"" + BuchiMonitorName + "\" is not " +
                                    "a state based Buchi Monitor, other monitor types " +
                                    "are not supported just yet!");
            }

            Monitor->Freeze();

            if (ThePS != nullptr) {
                delete ThePS;
                ThePS = nullptr;
            }

            ESMC_LOG_MIN_SHORT(
                               Out_ << "Constructing Product..." << endl;
                               );

            ConstructProduct(Monitor);

            bool FixPoint = false;
            vector<const ProductState*> AllUnfairStates;

            do {
                FixPoint = true;
                ESMC_LOG_MIN_SHORT(
                                   Out_ << "Getting accepting SCCs... " << endl;
                                   );

                ThePS->ClearSCCMarkings();
                for (auto const& UnfairState : AllUnfairStates) {
                    UnfairState->MarkDeleted();
                }
                auto&& SCCRoots = GetAcceptingSCCs();

                ESMC_LOG_MIN_SHORT(
                                   Out_ << "Found " << SCCRoots.size()
                                        << " accepting SCCs" << endl;
                                   if (SCCRoots.size() > 0) {
                                       Out_ << "Checking Fairness of SCCs..." << endl;
                                   } else {
                                       Out_ << "No accepting SCCs!" << endl;
                                   });

                u32 SCCNum = 0;
                for (auto SCCRoot : SCCRoots) {
                    // Check if each of the SCCs are fair
                    vector<const ProductState*> UnfairStates;
                    auto IsFair = CheckSCCFairness(SCCRoot, UnfairStates);

                    if (!IsFair && UnfairStates.size() > 0) {
                        ESMC_LOG_FULL(
                                      "Checker.Fairness",
                                      Out_ << "SCC " << SCCNum << " is unfair." << endl;
                                      );
                        AllUnfairStates.insert(AllUnfairStates.end(),
                                               UnfairStates.begin(),
                                               UnfairStates.end());
                        FixPoint = false;
                    } else if (!IsFair) {
                        continue;
                    } else {
                        ESMC_LOG_MIN_SHORT(
                                           Out_ << "Found fair accepting SCC" << endl;
                                           );

                        auto Trace = TraceBase::MakeLivenessViolation(SCCRoot, this);
                        return Trace;
                    }
                    ++SCCNum;
                }
            } while (!FixPoint);

            ESMC_LOG_MIN_SHORT(
                               Out_ << "No liveness violations found!" << endl;
                               );

            return nullptr;
        }

        const vector<string>& LTSChecker::GetBuchiMonitorNames() const
        {
            return BuchiMonitorNames;
        }

        LabelledTS* LTSChecker::GetLTS() const
        {
            return TheLTS;
        }

        LTSCompiler* LTSChecker::GetCompiler() const
        {
            return Compiler;
        }

    } /* end namespace MC */
} /* end namespace ESMC */

//
// LTSChecker.cpp ends here
