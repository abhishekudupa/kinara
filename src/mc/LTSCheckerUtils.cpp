// LTSCheckerUtils.cpp ---
// Filename: LTSCheckerUtils.cpp
// Author: Abhishek Udupa
// Created: Sat Jan 31 14:14:06 2015 (-0500)
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

#include "../uflts/LTSTransitions.hpp"
#include "../uflts/LTSEFSMBase.hpp"

#include "LTSChecker.hpp"
#include "LTSCheckerUtils.hpp"
#include "IndexSet.hpp"

namespace ESMC {
namespace MC {
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
} /* end namespace MC */
} /* end namespace ESMC */

//
// LTSCheckerUtils.cpp ends here
