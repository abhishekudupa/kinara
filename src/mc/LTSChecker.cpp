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

            // Fairness Checker implementation
            FairnessChecker::FairnessChecker(const LTSFairSetRef& FairSet,
                                             SystemIndexSet* SysIdxSet,
                                             const vector<GCmdRef>& GuardedCommands,
                                             LTSChecker* Checker)
                : FairSet(FairSet), NumInstances(FairSet->GetNumInstances()),
                  IsStrong(FairSet->GetFairnessType() == FairSetFairnessType::Strong),
                  SysIdxSet(SysIdxSet), Enabled(false),
                  Executed(false), Disabled(false),
                  ClassID(FairSet->GetEFSM()->GetClassID()),
                  GCmdsToRespondTo(SysIdxSet->GetNumTrackedIndices(),
                                   vector<bool>(GuardedCommands.size())),
                  GCmdIDsToRespondTo(NumInstances),
                  Checker(Checker),
                  EnabledPerInstance(NumInstances, false),
                  ExecutedPerInstance(NumInstances, false),
                  DisabledPerInstance(NumInstances, false)

            {
                const u32 NumTrackedIndices = SysIdxSet->GetNumTrackedIndices();
                for (u32 TrackedIndex = 0; TrackedIndex < NumTrackedIndices; ++TrackedIndex) {
                    auto InstanceID = SysIdxSet->GetIndexForClassID(TrackedIndex, ClassID);
                    if (InstanceID < 0) {
                        continue;
                    }

                    u32 i = 0;
                    for (auto const& Cmd : GuardedCommands) {
                        auto const& FairObjs = Cmd->GetFairnessObjs();
                        for (auto const& FairObj : FairObjs) {
                            if (FairObj->GetFairnessSet() == FairSet &&
                                FairObj->GetInstanceID() == (u32)InstanceID) {
                                GCmdsToRespondTo[TrackedIndex][i] = true;
                                GCmdIDsToRespondTo[InstanceID].insert(i);
                            }
                        }
                        ++i;
                    }
                }
            }

            FairnessChecker::~FairnessChecker()
            {
                // Nothing here
            }

            void FairnessChecker::ResetFairness()
            {
                Enabled = false;
                Disabled = false;
                Executed = false;
                EnabledStates.clear();
            }

            void FairnessChecker::ResetFull()
            {
                ResetFairness();
                for (u32 i = 0; i < NumInstances; ++i) {
                    EnabledPerInstance[i] = false;
                    DisabledPerInstance[i] = false;
                    ExecutedPerInstance[i] = false;
                }
            }

            void FairnessChecker::ProcessSCCState(const ProductState* State,
                                                  const ProductEdgeSetT& Edges,
                                                  u32 TrackedIndex)
            {
                auto InstanceID = SysIdxSet->GetIndexForClassID(TrackedIndex, ClassID);
                if (InstanceID < 0) {
                    return;
                }

                // cout << "Fairness Checker for fairness set: "
                //      << FairSet->GetName() << " on EFSM "
                //      << FairSet->GetEFSM()->GetName() << " with tracked index "
                //      << TrackedIndex << endl;
                // cout << "Considering state:" << endl;
                // Checker->Printer->PrintState(State, cout);
                // cout << endl;

                auto const SCCID = State->Status.InSCC;

                bool AtLeastOneEnabled = false;
                for (auto const& Edge : Edges) {
                    auto NextState = Edge->GetTarget();
                    auto GCmdIndex = Edge->GetGCmdIndex();
                    if (!GCmdsToRespondTo[TrackedIndex][GCmdIndex]) {
                        continue;
                    }
                    // This command causes this state to be marked
                    // enabled

                    // cout << "Marking as Enabled, because command "
                    //      << GCmdIndex << " is enabled" << endl;

                    Enabled = true;
                    EnabledPerInstance[InstanceID] = true;
                    AtLeastOneEnabled = true;

                    if (NextState->IsInSCC(SCCID)) {
                        // auto PermSet = Checker->TheCanonicalizer->GetPermSet();
                        // cout << "Marking as Executed, because, next state is in SCC"
                        //      << endl;
                        // cout << "Next State:" << endl;
                        // Checker->Printer->PrintState(NextState, cout);
                        // cout << endl << "With permutation:" << endl << endl;
                        // PermSet->Print(Edge->GetPermutation(), cout);
                        // cout << endl;

                        Executed = true;
                        ExecutedPerInstance[InstanceID] = true;
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
                    DisabledPerInstance[InstanceID] = true;
                }
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

            bool FairnessChecker::IsEnabled(u32 InstanceID) const
            {
                return EnabledPerInstance[InstanceID];
            }

            bool FairnessChecker::IsExecuted(u32 InstanceID) const
            {
                return ExecutedPerInstance[InstanceID];
            }

            bool FairnessChecker::IsDisabled(u32 InstanceID) const
            {
                return DisabledPerInstance[InstanceID];
            }

            const unordered_set<const ProductState*>& FairnessChecker::GetEnabledStates() const
            {
                return EnabledStates;
            }

            const unordered_set<u32>& FairnessChecker::GetCmdIDsToRespondTo(u32 InstanceID) const
            {
                return GCmdIDsToRespondTo[InstanceID];
            }

        } /* end namespace Detail */

        using namespace Detail;

        // Extern helper function definitions
        bool ApplyUpdates(const vector<LTSAssignRef>& Updates,
                          const StateVec* InputState,
                          StateVec* OutputState)
        {
            for (auto const& Update : Updates) {
                auto const& LHS = Update->GetLHS();
                auto const& RHS = Update->GetRHS();
                auto const& LHSInterp = LHS->ExtensionData.Interp->SAs<LValueInterpreter>();
                auto const& RHSInterp = RHS->ExtensionData.Interp;
                auto Status = LHSInterp->Update(RHSInterp, InputState, OutputState);
                if (!Status) {
                    return false;
                }
            }
            return true;
        }

        StateVec* ExecuteCommand(const GCmdRef& Cmd,
                                 const StateVec* InputState)
        {
            auto Retval = InputState->Clone();
            auto const& Updates = Cmd->GetUpdates();
            auto Status = ApplyUpdates(Updates, InputState, Retval);
            if (!Status) {
                Retval->Recycle();
                return nullptr;
            } else {
                return Retval;
            }
        }

        StateVec* TryExecuteCommand(const GCmdRef& Cmd,
                                    const StateVec* InputState,
                                    bool& Exception)
        {
            Exception = false;

            auto const& Guard = Cmd->GetGuard();
            auto GuardInterp = Guard->ExtensionData.Interp;
            auto Res = GuardInterp->Evaluate(InputState);
            if (Res == UndefValue) {
                Exception = true;
                return nullptr;
            } else if (Res == 0) {
                return nullptr;
            } else {
                auto OutState = ExecuteCommand(Cmd, InputState);
                if (OutState == nullptr) {
                    Exception = true;
                    return nullptr;
                }
                return OutState;
            }
        }

        inline set<ExpT> LTSChecker::GatherTermsInIndex(const ExpT& Exp)
        {
            auto Mgr = TheLTS->GetMgr();

            // first, gather array terms which are indexed by a variable
            auto&& ArrayTerms =
                Mgr->Gather(Exp,
                            [&] (const ExpBaseT* ExpPtr) -> bool
                            {
                                auto ExpAsOp = ExpPtr->As<OpExpression>();
                                if (ExpAsOp == nullptr) {
                                    return false;
                                }
                                if (ExpAsOp->GetOpCode() != LTSOps::OpIndex) {
                                    return false;
                                }
                                auto const& Children = ExpAsOp->GetChildren();
                                if (!Children[1]->Is<ConstExpression>()) {
                                    return true;
                                }
                                return false;
                            });

            // Now, gather the terms used as index
            set<ExpT> Retval;
            for (auto const& ArrayTerm : ArrayTerms) {
                auto ExpAsOp = ArrayTerm->SAs<OpExpression>();
                auto const& Children = ExpAsOp->GetChildren();
                Retval.insert(Children[1]);
            }

            return Retval;
        }

        inline void LTSChecker::MakeIndexTermInvariants(const ExpT& Precondition,
                                                        const set<ExpT>& IndexTerms)
        {
            auto Mgr = TheLTS->GetMgr();
            for (auto const& IndexTerm : IndexTerms) {
                auto const& IndexType = IndexTerm->GetType();
                auto Invar = Mgr->MakeExpr(LTSOps::OpEQ, IndexTerm,
                                           Mgr->MakeVal(IndexType->GetClearValue(),
                                                        IndexType));
                Invar = Mgr->MakeExpr(LTSOps::OpNOT, Invar);
                Invar = Mgr->MakeExpr(LTSOps::OpIMPLIES, Precondition, Invar);
                Invar = Mgr->Simplify(Invar);
                Compiler->CompileExp(Invar, TheLTS);
                BoundsInvariants.insert(Invar);
            }
        }

        inline void LTSChecker::MakeBoundsInvariants()
        {
            auto Mgr = TheLTS->GetMgr();
            // First, ALL index terms used in guards must never be undef
            for (auto const& Cmd : GuardedCommands) {
                auto const& Guard = Cmd->GetGuard();
                auto&& IndexTerms = GatherTermsInIndex(Guard);
                MakeIndexTermInvariants(Mgr->MakeTrue(), IndexTerms);
            }

            // Second, IF the guard of a guarded command is true, then ALL
            // the index terms used in the LHS or RHS of updates can never
            // be undef

            for (auto const& Cmd : GuardedCommands) {
                auto const& Guard = Cmd->GetGuard();
                auto const& Updates = Cmd->GetUpdates();

                for (auto const& Update : Updates) {
                    auto const& LHS = Update->GetLHS();
                    auto const& RHS = Update->GetRHS();

                    auto&& IndexTermsLHS = GatherTermsInIndex(LHS);
                    MakeIndexTermInvariants(Guard, IndexTermsLHS);
                    auto&& IndexTermsRHS = GatherTermsInIndex(RHS);
                    MakeIndexTermInvariants(Guard, IndexTermsRHS);
                }
            }

            // Third, IF the guard of a guarded command is true, then
            // the RHS of every update to a range typed term must be within
            // the bounds of the range type

            for (auto const& Cmd : GuardedCommands) {
                auto const& Guard = Cmd->GetGuard();
                auto const& Updates = Cmd->GetUpdates();

                for (auto const& Update : Updates) {
                    auto const& LHS = Update->GetLHS();
                    auto const& RHS = Update->GetRHS();

                    auto LValType = LHS->GetType();
                    if (!LValType->Is<RangeType>()) {
                        continue;
                    }

                    // This is a range typed update
                    auto TypeAsRange = LValType->SAs<RangeType>();
                    auto RangeLow = TypeAsRange->GetLow();
                    auto RangeHigh = TypeAsRange->GetHigh();

                    auto LowVal = Mgr->MakeVal(to_string(RangeLow), TypeAsRange);
                    auto HighVal = Mgr->MakeVal(to_string(RangeHigh), TypeAsRange);

                    auto LowConstraint = Mgr->MakeExpr(LTSOps::OpGE, RHS, LowVal);
                    auto HighConstraint = Mgr->MakeExpr(LTSOps::OpLE, RHS, HighVal);

                    auto BoundsConstraint = Mgr->MakeExpr(LTSOps::OpAND, LowConstraint,
                                                          HighConstraint);
                    BoundsConstraint = Mgr->MakeExpr(LTSOps::OpIMPLIES, Guard, BoundsConstraint);
                    auto Simplified = Mgr->Simplify(BoundsConstraint);
                    Compiler->CompileExp(Simplified, TheLTS);
                    BoundsInvariants.insert(Simplified);
                }
            }

            // Finally, it must be possible to evaluate the invariant
            // on ANY state, without exception
            auto&& IndexTerms = GatherTermsInIndex(TheLTS->InvariantExp);
            MakeIndexTermInvariants(Mgr->MakeTrue(), IndexTerms);
            set<ExpT> SimplifiedBoundsInvars;

            for (auto const& Invar : BoundsInvariants) {
                SimplifiedBoundsInvars.insert(Mgr->Simplify(Invar));
            }
            BoundsInvariants = SimplifiedBoundsInvars;
        }

        inline void LTSChecker::RecordErrorState(const StateVec* ErrorState)
        {
            if (ErrorStates.find(ErrorState) != ErrorStates.end()) {
                return;
            }

            bool FoundBlown = false;
            for (auto const& Invar : BoundsInvariants) {
                auto Interp = Invar->ExtensionData.Interp;
                auto Res = Interp->Evaluate(ErrorState);
                if (Res == UndefValue) {
                    continue;
                } else if (Res == 0) {
                    FoundBlown = true;
                    ErrorStates[ErrorState] = Invar;
                    break;
                }
            }

            if (!FoundBlown) {
                ostringstream sstr;
                Printer->PrintState(ErrorState, sstr);
                throw InternalError((string)"Could not find the bounds invariant that " +
                                    "was blown in call to LTSChecker::RecordErrorState()\n" +
                                    "The State:\n" + sstr.str() + "\nCould not find bounds " +
                                    "invariant that was blown for the state listed above.\n" +
                                    "At: " + __FILE__ + ":" + to_string(__LINE__));
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

            NumProcesses = 0;

            vector<vector<vector<ExpT>>> ProcessInsts(TheLTS->AllEFSMs.size());
            FairnessCheckers = vector<vector<Detail::FairnessChecker*>>(TheLTS->AllEFSMs.size());

            for (auto const& NameEFSM : TheLTS->AllEFSMs) {
                auto EFSM = NameEFSM.second;
                u32 ClassID = EFSM->GetClassID();
                ProcessInsts[ClassID] = EFSM->GetParamInsts();
                NumProcesses += EFSM->GetNumInstances();
            }

            SysIdxSet = new SystemIndexSet(ProcessInsts);

            for (auto const& NameEFSM : TheLTS->AllEFSMs) {
                auto EFSM = NameEFSM.second;
                u32 ClassID = EFSM->GetClassID();

                auto const& AllFairnesses = EFSM->GetAllFairnessSets();
                auto const& AllFairnessSets = AllFairnesses->GetFairnessSets();

                for (auto const& NameFS : AllFairnessSets) {
                    auto const& FS = NameFS.second;
                    auto CurChecker = new FairnessChecker(FS, SysIdxSet,
                                                          GuardedCommands, this);
                    if (FS->GetFairnessType() == FairSetFairnessType::Strong) {
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

            // Make the invariant on undef and bounds
            MakeBoundsInvariants();
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
                                                            bool& Exception)
        {
            Exception = false;
            set<u32>::iterator it;
            if (LastFired == -1) {
                it = InterpretedCommands.begin();
            } else {
                it = next(InterpretedCommands.find(LastFired));
            }
            while (it != InterpretedCommands.end()) {
                auto CurCmdID = *it;
                auto const& Guard = GuardedCommands[CurCmdID]->GetGuard();
                auto GRes = Guard->ExtensionData.Interp->Evaluate(State);
                if (GRes == UndefValue) {
                    Exception = true;
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

            while (DFSStack.size() > 0) {
                auto& CurEntry = DFSStack.top();
                auto State = CurEntry.GetState();

                // cout << "Considering State:" << endl;
                // cout << "--------------------------------------------------------" << endl;
                // Printer->PrintState(State, cout);
                // cout << "--------------------------------------------------------" << endl;

                auto& LastFired = CurEntry.GetLastFired();
                bool Deadlocked = (LastFired == -1);
                StateVec* NextState = nullptr;
                // StateVec* TempNextState = nullptr;

                bool Exception = false;
                auto const& Cmd = GetNextEnabledCmd(State, LastFired, Exception);

                if (Exception) {
                    RecordErrorState(State);
                    if (ErrorStates.size() >= NumErrors) {
                        return;
                    }
                } else if (Cmd == GCmdRef::NullPtr) {
                    if (Deadlocked) {
                        ErrorStates[State] = DeadlockFreeInvariant;
                    }
                    if (ErrorStates.size() >= NumErrors) {
                        return;
                    }
                    // cout << "No more successors, popping from stack!" << endl;
                    // Done exploring this state
                    DFSStack.pop();
                    continue;
                }

                // Successors remain to be explored
                NextState = ExecuteCommand(Cmd, State);

                if (NextState == nullptr) {
                    // An exception was encountered firing this
                    // command on the State. Store it, and pop from
                    // stack, since only one error is remembered
                    // per state
                    RecordErrorState(State);
                    if (ErrorStates.size() >= NumErrors) {
                        return;
                    }
                    DFSStack.pop();
                    continue;
                }

                // cout << "Firing guarded command:" << endl;
                // cout << Cmd->ToString() << endl;
                // cout << "Got Next State (Uncanonicalized):" << endl;
                // cout << "--------------------------------------------------------" << endl;
                // Printer->PrintState(NextState, cout);
                // cout << "--------------------------------------------------------" << endl;
                // TempNextState = NextState->Clone();


                u32 PermID;
                auto CanonState = TheCanonicalizer->Canonicalize(NextState, PermID);

                // cout << "Canonicalized Next State with Permutation ID "
                //      << PermID << ":" << endl;
                // cout << "--------------------------------------------------------" << endl;
                // if (CanonState->Equals(*TempNextState)) {
                //     cout << "Canonicalized state is the same as uncanonicalized state!"
                //          << endl;
                // } else {
                //     Printer->PrintState(CanonState, cout);
                // }
                // cout << "--------------------------------------------------------" << endl;
                // TempNextState->Recycle();


                auto ExistingState = AQS->Find(CanonState);

                if (ExistingState == nullptr) {

                    AQS->Insert(CanonState);

                    // cout << "Pushed new successor onto stack." << endl;

                    AQS->AddEdge(State, CanonState, PermID, LastFired);
                    DFSStack.push(DFSStackEntry(CanonState));

                    // This is a new state, check for error
                    auto const& Invar = TheLTS->GetInvariant();
                    auto Interp = Invar->ExtensionData.Interp;
                    auto InvarRes = Interp->Evaluate(CanonState);
                    if (InvarRes == UndefValue) {
                        RecordErrorState(CanonState);
                        if (ErrorStates.size() >= NumErrors) {
                            return;
                        }
                        DFSStack.pop();
                    } else if (InvarRes == 0) {
                        ErrorStates[CanonState] = Invar;
                        if (ErrorStates.size() >= NumErrors) {
                            return;
                        }
                        DFSStack.pop();
                    }
                    continue;
                } else {
                    // Successor already explored, add edge
                    // and continue
                    // cout << "Successor has been previously encountered." << endl;
                    // auto InvPermID = PermSet->GetIteratorForInv(PermID).GetIndex();

                    // We do not invert. Since all transformations call for the
                    // inverse of the inverse anyway!
                    AQS->AddEdge(State, ExistingState, PermID, LastFired);
                    CanonState->Recycle();
                    continue;
                }
            }
        }

        inline void LTSChecker::DoBFS(const vector<StateVec*>& Roots, u32 NumErrors)
        {
            deque<StateVec*> BFSQueue(Roots.begin(), Roots.end());
            for (auto State : BFSQueue) {
                AQS->InsertInitState(State);
            }

            i64 IterCount = 0;
            while (BFSQueue.size() > 0) {
                ++IterCount;
                auto CurState = BFSQueue.front();
                BFSQueue.pop_front();

                bool Deadlocked = true;

                for (auto i : InterpretedCommands) {

                    auto const& Cmd = GuardedCommands[i];

                    StateVec* NextState = nullptr;
                    bool Exception = false;

                    NextState = TryExecuteCommand(Cmd, CurState, Exception);
                    if (Exception) {
                        Deadlocked = false;
                        RecordErrorState(CurState);
                        if (ErrorStates.size() >= NumErrors) {
                            return;
                        }
                        // Already an error. Don't try any more commands
                        break;
                    }

                    if (NextState != nullptr) {
                        u32 PermID = 0;
                        Deadlocked = false;
                        auto CanonNextState = TheCanonicalizer->Canonicalize(NextState, PermID);

                        auto ExistingState = AQS->Find(CanonNextState);
                        if (ExistingState != nullptr) {
                            AQS->AddEdge(CurState, ExistingState, PermID, i);
                            CanonNextState->Recycle();
                        } else {

                            // cout << "New State:" << endl;
                            // cout << "-------------------------------------------------------"
                            //      << endl;
                            // Printer->PrintState(CanonNextState, cout);
                            // cout << "-------------------------------------------------------"
                            //      << endl;

                            AQS->Insert(CanonNextState);
                            AQS->AddEdge(CurState, CanonNextState, PermID, i);
                            BFSQueue.push_back(CanonNextState);

                            // New state, check for errors;
                            auto const& Invar = TheLTS->GetInvariant();
                            auto Interp = Invar->ExtensionData.Interp;
                            auto InvarRes = Interp->Evaluate(CanonNextState);
                            if (InvarRes == UndefValue) {
                                RecordErrorState(CanonNextState);
                                if (ErrorStates.size() >= NumErrors) {
                                    return;
                                }
                                // Remove it from the queue
                                BFSQueue.pop_back();

                            } else if (InvarRes == 0) {
                                // Again, remember this state, but continue
                                // on with the AQS construction
                                ErrorStates[CanonNextState] = Invar;
                                if (ErrorStates.size() >= NumErrors) {
                                    return;
                                }
                                BFSQueue.pop_back();
                            }
                        }
                    }
                }

                if (Deadlocked) {
                    ErrorStates[CurState] = DeadlockFreeInvariant;
                    if (ErrorStates.size() >= NumErrors) {
                        return;
                    }
                }
            }
        }

        bool LTSChecker::BuildAQS(AQSConstructionMethod Method,
                                  u32 NumErrors)
        {
            vector<TraceBase*> Retval;

            if (AQS != nullptr) {
                return (ErrorStates.size() == 0);
            }

            // Gather the commands that are relevant
            InterpretedCommands.clear();

            // cout << "Interpreted Commands:" << endl;

            for (auto const& Cmd : GuardedCommands) {
                if (Cmd->IsFullyInterpreted()) {
                    InterpretedCommands.insert(Cmd->GetCmdID());
                    // cout << Cmd->ToString() << endl << endl;
                }
            }

            AQS = new AQStructure(TheLTS);

            auto const& ISGens = TheLTS->GetInitStateGenerators();
            vector<StateVec*> InitStates;

            for (auto const& ISGen : ISGens) {
                auto CurState = Factory->MakeState();
                ApplyUpdates(ISGen, ZeroState, CurState);

                // cout << "Initial State:" << endl;
                // cout << "--------------------------------------------------------" << endl;
                // Printer->PrintState(CurState, cout);
                // cout << "--------------------------------------------------------" << endl;

                u32 CanonPerm = 0;
                auto CanonState = TheCanonicalizer->Canonicalize(CurState, CanonPerm);
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
                DoBFS(InitStates, NumErrors);
            }

            // TODO: Handle any violations found during model checking

            cout << "AQS Built!" << endl;
            cout << "AQS contains " << AQS->GetNumStates() << " states and "
                 << AQS->GetNumEdges() << " edges." << endl;
            cout << Factory->GetNumActiveStates() << " active states from state factory." << endl;
            if (ErrorStates.size() > 0) {
                cout << "One or more safety/deadlock violations found!" << endl;
                return false;
            }
            return true;
        }

        const unordered_map<const StateVec*, ExpT> LTSChecker::GetAllErrorStates() const
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
            auto it = ErrorStates.find(ErrorState);
            if (it == ErrorStates.end()) {
                throw ESMCError((string)"Argument to LTSChecker::MakeTraceToError() is " +
                                "not marked as an error state by the checker");
            }
            auto const& BlownInvariant = it->second;
            if (BlownInvariant == DeadlockFreeInvariant) {
                return TraceBase::MakeDeadlockViolation(ErrorState, this);
            } else {
                return TraceBase::MakeSafetyViolation(ErrorState, this, BlownInvariant);
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
            ThePS = new ProductStructure(NumProcesses, Monitor);
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

            cout << "Product construction complete!" << endl;
            cout << "Product Structure contains " << ThePS->GetNumStates()
                 << " states and " << ThePS->GetNumEdges() << " edges." << endl;
        }

        inline void LTSChecker::DoThreadedBFS(const ProductState *SCCRoot, u32 IndexID)
        {
            deque<pair<const ProductState*, u32>> BFSQueue;
            SCCRoot->MarkTracked(IndexID);
            const u32 SCCID = SCCRoot->Status.InSCC;
            // We can set the class id here,
            // since the classes will not change
            const u32 ClassID = SysIdxSet->GetClassID(IndexID);

            // cout << "Doing threaded BFS on product state:"
            //      << endl;
            // Printer->PrintState(SCCRoot, cout);
            // cout << "With threaded tracked index " << IndexID << endl;

            BFSQueue.push_back(make_pair(SCCRoot, IndexID));
            auto PermSet = TheCanonicalizer->GetPermSet();

            while (BFSQueue.size() > 0) {
                auto const& CurEntry = BFSQueue.front();

                auto CurState = CurEntry.first;
                auto CurIndex = CurEntry.second;
                BFSQueue.pop_front();

                auto const& CurEdges = ThePS->GetEdges(const_cast<ProductState*>(CurState));

                // process this state for fairness
                for (auto Checker : FairnessCheckers[ClassID]) {
                    Checker->ProcessSCCState(CurState, CurEdges, CurIndex);
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

            // cout << "Threaded BFS done!" << endl;
        }

        inline bool LTSChecker::CheckSCCFairness(const ProductState *SCCRoot,
                                                 vector<const ProductState*>& UnfairStates)
        {
            for (auto FCheckers : FairnessCheckers) {
                for (auto FChecker : FCheckers) {
                    FChecker->ResetFull();
                }
            }

            const u32 IndexMax = SysIdxSet->GetNumTrackedIndices();
            for (u32 IndexID = 0; IndexID < IndexMax; ++IndexID) {

                if (SCCRoot->IsTracked(IndexID)) {
                    continue;
                }

                // Reset all the fairness checkers first
                for (auto FCheckers : FairnessCheckers) {
                    for (auto FChecker : FCheckers) {
                        FChecker->ResetFairness();
                    }
                }

                auto ClassID = SysIdxSet->GetClassID(IndexID);

                // cout << "Checking fairness of SCC with tracked index " << IndexID << endl;
                DoThreadedBFS(SCCRoot, IndexID);
                // Are all the fairness requirements satisfied?
                for (auto FChecker : FairnessCheckers[ClassID]) {
                    if (!FChecker->IsFair()) {
                        if (!FChecker->IsStrongFairness()) {
                            // Weak fairness, nothing to do
                            return false;
                        } else {
                            // Find the states that cause this
                            // particular fairness to fail
                            // i.e., states where this fairness
                            // is enabled
                            auto const& EnabledStates = FChecker->GetEnabledStates();
                            UnfairStates.insert(UnfairStates.end(), EnabledStates.begin(),
                                                EnabledStates.end());
                            return false;
                        }
                    }
                    // This fairness set is satisfied!
                }
            }
            return true;
        }

        inline vector<const ProductState*> LTSChecker::GetAcceptingSCCs()
        {
            // Find the SCCs on the fly
            stack<pair<const ProductState*, u32>> DFSStack;
            stack<ProductState*> SCCStack;
            vector<const ProductState*> Retval;

            u32 CurIndex = 0;
            u32 CurSCCID = 0;

            for (auto InitState : ThePS->GetInitialStates()) {
                if (InitState->DFSNum != -1 || InitState->IsDeleted()) {
                    // already explored this initial state
                    continue;
                }

                InitState->DFSNum = CurIndex;
                InitState->LowLink = CurIndex;
                InitState->MarkOnStack();
                DFSStack.push(make_pair(InitState, 0));
                SCCStack.push(InitState);
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

                        if (NumStatesInSCC == 1) {
                            auto const& Edges = ThePS->GetEdges(SCCState);
                            bool SelfLoop = false;
                            for (auto const& Edge : Edges) {
                                if (Edge->GetTarget() == SCCState) {
                                    SelfLoop = true;
                                    // TODO Maybe put a break
                                }
                            }
                            if (!SelfLoop) {
                                SCCState->MarkNotInSCC();
                            } else {
                                Retval.push_back(CurState);
                                ++CurSCCID;
                            }
                        } else if (!FoundAccepting) {
                            // Unmark all the SCCs
                            for (auto SCCState : SCCStateVec) {
                                SCCState->MarkNotInSCC();
                            }
                        } else {
                            // Non-singular AND accepting
                            // Send this for further processing

                            // cout << "Accepting SCC:" << endl;
                            // for (auto SCCState : SCCStateVec) {
                            //     cout << "State:" << endl
                            //          << "------------------------------------------"
                            //          << endl;
                            //     Printer->PrintState(SCCState, cout);
                            //     cout << "------------------------------------------"
                            //          << endl;
                            // }

                            Retval.push_back(CurState);
                            // cout << "[Checker:] Found SCC with " << NumStatesInSCC
                            //      << " states" << endl;
                            ++CurSCCID;
                        }
                    }
                }
            }
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

            cout << "Constructing Product..." << endl;
            ConstructProduct(Monitor);

            bool FixPoint = false;
            set<const ProductState*> AllUnfairStates;

            while (!FixPoint) {
                FixPoint = true;
                cout << "Getting accepting SCCs... " << endl;
                auto&& SCCRoots = GetAcceptingSCCs();
                // Check if each of the SCCs are fair
                vector<const ProductState*> UnfairStates;
                if (SCCRoots.size() > 0) {
                    cout << "Checking Fairness of SCCs..." << endl;
                } else {
                    cout << "No accepting SCCs!" << endl;
                }
                for (auto SCCRoot : SCCRoots) {
                    UnfairStates.clear();
                    auto IsFair = CheckSCCFairness(SCCRoot, UnfairStates);
                    if (!IsFair && UnfairStates.size() > 0) {
                        AllUnfairStates.insert(UnfairStates.begin(),
                                               UnfairStates.end());
                        // Not fair, due to a strong fairness
                        FixPoint = false;
                        // Mark all states as being unexplored
                        ThePS->ClearAllMarkings();
                        cout << "Deleting unfair states..." << endl;
                        for (auto UnfairState : AllUnfairStates) {
                            UnfairState->MarkDeleted();
                        }
                        // Redo SCCs
                        break;
                    } else if (!IsFair) {
                        // Not fair due to weak fairness, can't help
                        // Check if some other SCC is fair
                        continue;
                    } else {
                        // Fair, turn this into a counterexample
                        cout << "Found fair accepting SCC" << endl;
                        auto Trace = TraceBase::MakeLivenessViolation(SCCRoot, this);
                        return Trace;
                    }
                }
            }

            cout << "No liveness violations found! :-)" << endl;
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
