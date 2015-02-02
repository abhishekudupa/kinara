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
#include "../symmetry/SymmCanonicalizer.hpp"

#include "LTSChecker.hpp"
#include "LTSCheckerUtils.hpp"
#include "IndexSet.hpp"
#include "StateVecPrinter.hpp"

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

BFSQueueT::BFSQueueT(BFSPrioMethodT PrioMethod, const PathFPPrioritizer* Prioritizer)
    : BFSDeque(nullptr), SimplePrioQueues(nullptr), CoveragePrioQueues(nullptr),
      Prioritizer(nullptr), NumElems(0)
{
    if (PrioMethod == BFSPrioMethodT::None) {
        BFSDeque = new deque<StateVec*>();
    } else if (PrioMethod == BFSPrioMethodT::Simple) {
        SimplePrioQueues = new vector<deque<StateVec*>*>(MaxNumBuckets, nullptr);
    } else if (PrioMethod == BFSPrioMethodT::Coverage) {
        if (Prioritizer == nullptr) {
            throw InternalError((string)"Attempted to create a coverage based " +
                                "BFS priority queue without a prioritizer.\nAt: " +
                                __FUNCTION__ + ", " + __FILE__ + ":" +
                                to_string(__LINE__));
        }
        this->Prioritizer = Prioritizer;
        CoveragePrioQueues = new CoveragePrioQueueT();
    }
}

BFSQueueT::~BFSQueueT()
{
    if (BFSDeque != nullptr) {
        delete BFSDeque;
    } else if (SimplePrioQueues != nullptr) {
        for (auto Queue : *SimplePrioQueues) {
            if (Queue != nullptr) {
                delete Queue;
            }
        }
        delete SimplePrioQueues;
    } else if (CoveragePrioQueues != nullptr) {
        for (auto const& FPQueuePair : *CoveragePrioQueues) {
            if (FPQueuePair.second != nullptr) {
                delete FPQueuePair.second;
            }
        }
        delete CoveragePrioQueues;
    }
}

void BFSQueueT::Push(StateVec* State)
{
    BFSDeque->push_back(State);
    ++NumElems;
}

void BFSQueueT::Push(StateVec* State, u32 TaintLevel)
{
    u32 PrioIndex = min(TaintLevel, MaxNumBuckets - 1);
    auto TheQueue = (*SimplePrioQueues)[PrioIndex];
    if (TheQueue == nullptr) {
        TheQueue = new deque<StateVec*>();
        (*SimplePrioQueues)[PrioIndex] = TheQueue;
    }
    TheQueue->push_back(State);
    ++NumElems;
}

void BFSQueueT::Push(StateVec* State, const PathFingerprint* FP)
{
    deque<StateVec*>* TheQueue = nullptr;
    auto it = CoveragePrioQueues->find(FP);

    if (it == CoveragePrioQueues->end()) {
        TheQueue = new deque<StateVec*>();
        (*CoveragePrioQueues)[FP] = TheQueue;
    } else {
        TheQueue = it->second;
    }
    TheQueue->push_back(State);
    ++NumElems;
}

StateVec* BFSQueueT::Pop()
{
    StateVec* Retval = BFSDeque->front();
    BFSDeque->pop_front();
    --NumElems;
    return Retval;
}

StateVec* BFSQueueT::Pop(u32& TaintLevel)
{
    StateVec* Retval = nullptr;
    for (u32 i = 0; i < MaxNumBuckets; ++i) {
        if ((*SimplePrioQueues)[i] != nullptr &&
            (*SimplePrioQueues)[i]->size() > 0) {
            Retval = (*SimplePrioQueues)[i]->front();
            (*SimplePrioQueues)[i]->pop_front();
            TaintLevel = i;
            break;
        }
    }
    --NumElems;
    return Retval;
}

StateVec* BFSQueueT::Pop(const PathFingerprint*& RetFP)
{
    StateVec* Retval = nullptr;
    RetFP = nullptr;
    auto const& FPPriorities = Prioritizer->GetPriorities();
    for (auto const& FP : FPPriorities) {
        auto it = CoveragePrioQueues->find(FP);
        if (it != CoveragePrioQueues->end() && it->second != nullptr &&
            it->second->size() > 0) {
            Retval = it->second->front();
            RetFP = FP;
            it->second->pop_front();
            break;
        }
    }
    --NumElems;
    return Retval;
}

u32 BFSQueueT::Size() const
{
    return NumElems;
}

// PathFingerprint implementation
PathFingerprint::PathFingerprint()
    : PathFPSet(nullptr), HashCode(0)
{
    // Nothing here
}

PathFingerprint::~PathFingerprint()
{
    // Nothing here
}

inline i32 PathFingerprint::Compare(const PathFingerprint& Other) const
{
    auto Diff = CommandsAlongPathVec.size() - Other.CommandsAlongPathVec.size();
    if (Diff != 0) {
        return Diff;
    }
    for (u32 i = 0; i < CommandsAlongPathVec.size(); ++i) {
        Diff = CommandsAlongPathVec[i] - Other.CommandsAlongPathVec[i];
        if (Diff != 0) {
            return Diff;
        }
    }
    return 0;
}

PathFingerprint::PathFingerprint(const PathFingerprint& Other)
    : CommandsAlongPath(Other.CommandsAlongPath),
      CommandsAlongPathVec(Other.CommandsAlongPathVec),
      CommandsAlongPathBS(Other.CommandsAlongPathBS),
      PathFPSet(Other.PathFPSet), HashCode(Other.HashCode)
{
    // Nothing here
}

PathFingerprint::PathFingerprint(PathFingerprint&& Other)
    : CommandsAlongPath(move(Other.CommandsAlongPath)),
      CommandsAlongPathVec(move(Other.CommandsAlongPathVec)),
      CommandsAlongPathBS(move(Other.CommandsAlongPathBS)),
      PathFPSet(Other.PathFPSet), HashCode(Other.HashCode)
{
    // Nothing here
}

PathFingerprint& PathFingerprint::operator = (const PathFingerprint& Other)
{
    if (&Other == this) {
        return *this;
    }
    CommandsAlongPath = Other.CommandsAlongPath;
    CommandsAlongPathVec = Other.CommandsAlongPathVec;
    PathFPSet = Other.PathFPSet;
    HashCode = Other.HashCode;
    return *this;
}

PathFingerprint& PathFingerprint::operator = (PathFingerprint&& Other)
{
    if (&Other == this) {
        return *this;
    }
    swap(CommandsAlongPath, Other.CommandsAlongPath);
    swap(CommandsAlongPathVec, Other.CommandsAlongPathVec);
    swap(CommandsAlongPathBS, Other.CommandsAlongPathBS);
    swap(PathFPSet, Other.PathFPSet);
    swap(HashCode, Other.HashCode);
    return *this;
}

bool PathFingerprint::operator == (const PathFingerprint& Other) const
{
    if (HashCode != Other.HashCode) {
        return false;
    }
    return (Compare(Other) == 0);
}

bool PathFingerprint::operator < (const PathFingerprint& Other) const
{
    return (Compare(Other) < 0);
}

const unordered_set<u32>& PathFingerprint::GetCommandsAlongPath() const
{
    return CommandsAlongPath;
}

const vector<u32>& PathFingerprint::GetCommandsAlongPathVec() const
{
    return CommandsAlongPathVec;
}

bool PathFingerprint::IsCommandAlongPath(u32 CommandID) const
{
    return CommandsAlongPathBS.Test(CommandID);
}

const PathFingerprint* PathFingerprint::GetPathFPForAddedCommand(u32 CommandID,
                                                                 bool& IsNew) const
{
    if (IsCommandAlongPath(CommandID)) {
        return this;
    }
    auto NewCommandsAlongPath = CommandsAlongPath;
    NewCommandsAlongPath.insert(CommandID);
    return PathFPSet->MakeFP(NewCommandsAlongPath.begin(), NewCommandsAlongPath.end(), IsNew);
}

float PathFingerprint::GetCoverageForCommand(u32 CommandID) const
{
    if (CommandsAlongPath.find(CommandID) == CommandsAlongPath.end()) {
        return 0.0f;
    } else if (CommandsAlongPathVec.size() == 0) {
        return 1.0f;
    } else {
        return (1.0f / CommandsAlongPathVec.size());
    }
}

u32 PathFingerprint::GetNumCommandsAlongPath() const
{
    return CommandsAlongPathVec.size();
}

u64 PathFingerprint::Hash() const
{
    return HashCode;
}

string PathFingerprint::ToString(u32 Verbosity) const
{
    string Retval("Pathfingerprint: <");
    bool First = true;
    for (auto const& CommandID : CommandsAlongPathVec) {
        Retval += (First ? "" : " ");
        Retval += to_string(CommandID);
        First = false;
    }
    Retval += ">";
    return Retval;
}

PathFingerprintSet::PathFingerprintSet(u32 TotalNumCommands)
    : TotalNumCommands(TotalNumCommands)
{
    // Nothing here
}

PathFingerprintSet::~PathFingerprintSet()
{
    PathFPSet.clear();
}

u32 PathFingerprintSet::GetTotalNumCommands() const
{
    return TotalNumCommands;
}


// Implementation of PathFPPrioritizer
PathFPPrioritizer::PathFPPrioritizer(const vector<GCmdRef>& GuardedCommands,
                                     const PathFingerprint* ZeroFP,
                                     float DesiredCoverage)
    : DesiredCoverage(DesiredCoverage), ZeroFP(ZeroFP)
{
    for (auto const& Cmd : GuardedCommands) {
        if (!Cmd->IsTentative()) {
            continue;
        }
        CommandCoverage[Cmd->GetCmdID()] = 0.0f;
    }
    KnownPathFPs.insert(ZeroFP);
    FPPriorities.push_back(ZeroFP);
}

PathFPPrioritizer::~PathFPPrioritizer()
{
    // Nothing here
}

inline void PathFPPrioritizer::RedoFPPriorities()
{
    auto Comparator =
        [&] (const PathFingerprint* FP1, const PathFingerprint* FP2) -> bool
        {
            if (FP1 == ZeroFP && FP2 != ZeroFP) {
                // The ZeroFP beats everyone else
                return true;
            }
            if (FP2 == ZeroFP && FP1 != ZeroFP) {
                return false;
            }
            // Both are now sure not to be the zero fingerprint
            auto const& Cmds1 = FP1->GetCommandsAlongPathVec();
            auto const& Cmds2 = FP2->GetCommandsAlongPathVec();

            // The fingerprint with more edges goes to the back!
            if (Cmds1.size() < Cmds2.size()) {
                return true;
            } else if (Cmds2.size() < Cmds1.size()) {
                return false;
            }

            // The number of commands in each is now guaranteed to
            // be the same
            double Benefit1 = 0;
            for (auto const& Cmd : Cmds1) {
                double PendingCoverage = max(0.0f, DesiredCoverage - CommandCoverage[Cmd]);
                Benefit1 += (FP1->GetCoverageForCommand(Cmd) *
                             (PendingCoverage / DesiredCoverage));
            }

            double Benefit2 = 0;
            for (auto const& Cmd : Cmds2) {
                double PendingCoverage = max(0.0f, DesiredCoverage - CommandCoverage[Cmd]);
                Benefit2 += (FP2->GetCoverageForCommand(Cmd) *
                             (PendingCoverage / DesiredCoverage));
            }
            return (Benefit1 > Benefit2);
        };

    sort(FPPriorities.begin(), FPPriorities.end(), Comparator);
}

void PathFPPrioritizer::UpdatePriorities(const PathFingerprint* PathFPToError)
{
    auto const& CommandIDs = PathFPToError->GetCommandsAlongPathVec();
    const u32 NumCommandsSatisfied = CommandIDs.size();
    for (auto const& CommandID : CommandIDs) {
        CommandCoverage[CommandID] += 1.0f / NumCommandsSatisfied;
        // clamp at the desired coverage
        if (CommandCoverage[CommandID] >= DesiredCoverage) {
            CommandCoverage[CommandID] = DesiredCoverage;
        }
    }
    RedoFPPriorities();
}

string PathFPPrioritizer::ToString(u32 Verbosity) const
{
    ostringstream sstr;
    sstr << "Path FP Prioritizer:" << endl;
    sstr << "Path Fingerprint Priorities:" << endl;
    for (auto const& FP : FPPriorities) {
        sstr << FP->ToString(Verbosity) << endl;
    }
    sstr << "Command Coverage:" << endl;
    for (auto const& CmdCoveragePair : CommandCoverage) {
        sstr << CmdCoveragePair.first << " : " << CmdCoveragePair.second << endl;
    }
    return sstr.str();
}

const PathFingerprint* PathFPPrioritizer::GetZeroFP() const
{
    return ZeroFP;
}

void PathFPPrioritizer::NotifyNewPathFP(const PathFingerprint* PathFP)
{
    if (KnownPathFPs.find(PathFP) != KnownPathFPs.end()) {
        return;
    }
    KnownPathFPs.insert(PathFP);
    FPPriorities.push_back(PathFP);
    RedoFPPriorities();
}

const vector<const PathFingerprint*>& PathFPPrioritizer::GetPriorities() const
{
    return FPPriorities;
}

bool PathFPPrioritizer::IsCompletelyCovered() const
{
    for (auto const& CmdCoveragePair : CommandCoverage) {
        if (CmdCoveragePair.second < DesiredCoverage) {
            return false;
        }
    }
    return true;
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
