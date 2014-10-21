// LTSAnalyses.cpp --- 
// 
// Filename: LTSUtils.cpp
// Author: Abhishek Udupa
// Created: Fri Aug 15 12:14:12 2014 (-0400)
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

#include "LTSAnalyses.hpp"
#include "../uflts/LabelledTS.hpp"
#include "../uflts/LTSAssign.hpp"
#include "../uflts/LTSTypes.hpp"
#include "../uflts/LTSTransitions.hpp"
#include "../mc/AQStructure.hpp"
#include "../mc/OmegaAutomaton.hpp"
#include "../mc/Trace.hpp"

namespace ESMC {
    namespace Analyses {
        using namespace LTS;
        using namespace MC;

        // SubstitutorForWP implementation
        SubstitutorForWP::SubstitutorForWP(MgrT* Mgr, const MgrT::SubstMapT& Subst)
            : VisitorBaseT("SubstitutorForWP"),
              Mgr(Mgr), Subst(Subst)
        {
            // Nothing here
        }
        
        SubstitutorForWP::~SubstitutorForWP()
        {
            // Nothing here
        }

        void 
        SubstitutorForWP::VisitVarExpression(const VarExpT* Exp)
        {
            auto it = Subst.find(Exp);
            if (it != Subst.end()) {
                SubstStack.push_back(it->second);
            } else {
                SubstStack.push_back(Exp);
            }
        }

        void
        SubstitutorForWP::VisitConstExpression(const ConstExpT* Exp)
        {
            SubstStack.push_back(Exp);
        }

        void
        SubstitutorForWP::VisitBoundVarExpression(const BoundVarExpT* Exp)
        {
            auto it = Subst.find(Exp);
            if (it != Subst.end()) {
                SubstStack.push_back(it->second);
            } else {
                SubstStack.push_back(Exp);
            }
        }

        void 
        SubstitutorForWP::VisitOpExpression(const OpExpT* Exp)
        {
            auto it = Subst.find(Exp);
            if (it != Subst.end()) {
                SubstStack.push_back(it->second);
            } else {
                VisitorBaseT::VisitOpExpression(Exp);
                const u32 NumChildren = Exp->GetChildren().size();
                vector<ExpT> SubstChildren(NumChildren);
                for (u32 i = 0; i < NumChildren; ++i) {
                    SubstChildren[NumChildren - i - 1] = SubstStack.back();
                    SubstStack.pop_back();
                }
                auto OpCode = Exp->GetOpCode();
                if (OpCode != LTSOps::OpIndex) {
                    SubstStack.push_back(Mgr->MakeExpr(Exp->GetOpCode(),
                                                       SubstChildren));
                } else {
                    // Look if an index expression of the same array
                    // is in the substitution map
                    bool index_on_array_in_map = false;
                    ExpT LhsArrayAssignmentInMap;
                    ExpT RhsArrayAssignmentInMap;
                    auto NewExp = Mgr->MakeExpr(Exp->GetOpCode(),
                                                SubstChildren);
                    auto ITEExp = NewExp;
                    for (auto Pair: Subst) {
                        auto Lhs = Pair.first;
                        if (Lhs->Is<OpExpression>()) {
                            auto LhsOpExp = Lhs->As<OpExpression>();
                            if (LhsOpExp->GetOpCode() == LTSOps::OpIndex) {
                                auto Base = LhsOpExp->GetChildren()[0];
                                if (Base == Exp->GetChildren()[0]) {
                                    index_on_array_in_map = true;
                                    auto NewExpIndex = NewExp->As<OpExpression>()->GetChildren()[1];
                                    auto LhsIndex = LhsOpExp->GetChildren()[1];
                                    auto IndicesEqual = Mgr->MakeExpr(LTSOps::OpEQ,
                                                                      NewExpIndex, LhsIndex);
                                    ITEExp = Mgr->MakeExpr(LTSOps::OpITE,
                                                           IndicesEqual,
                                                           Pair.second,
                                                           ITEExp);
                                }
                            }
                        }
                    }
                    SubstStack.push_back(ITEExp);
                }
            }
        }

        void 
        SubstitutorForWP::VisitEQuantifiedExpression(const EQExpT* Exp)
        {
            Exp->GetQExpression()->Accept(this);
            auto SubstQExpr = SubstStack.back();
            SubstStack.pop_back();
            SubstStack.push_back(Mgr->MakeExists(Exp->GetQVarTypes(),
                                                 SubstQExpr));
        }

        void 
        SubstitutorForWP::VisitAQuantifiedExpression(const AQExpT* Exp)
        {
            Exp->GetQExpression()->Accept(this);
            auto SubstQExpr = SubstStack.back();
            SubstStack.pop_back();
            SubstStack.push_back(Mgr->MakeForAll(Exp->GetQVarTypes(),
                                                 SubstQExpr));
        }

        ExpT
        SubstitutorForWP::Do(MgrT* Mgr,
                             const ExpT& Exp, const MgrT::SubstMapT& Subst)
        {
            SubstitutorForWP TheSubstitutorForWP(Mgr, Subst);
            Exp->Accept(&TheSubstitutorForWP);
            return TheSubstitutorForWP.SubstStack[0];
        }

        vector<ExpT> GetAllScalarLeaves(ExpT InitialExp) {
            vector<ExpT> Retval;
            vector<ExpT> ExpressionsToCheck = {InitialExp};
            while (ExpressionsToCheck.size() > 0) {
                auto Exp = ExpressionsToCheck.back();
                ExpressionsToCheck.pop_back();
                auto ExpType = Exp->GetType();
                if (ExpType->Is<ExprScalarType>()) {
                    Retval.push_back(Exp);
                } else if (ExpType->Is<ExprArrayType>()) {
                    auto ExpTypeAsArrayType = ExpType->As<ExprArrayType>();
                    auto IndexType = ExpTypeAsArrayType->GetIndexType();
                    for (auto Element: IndexType->GetElements()) {
                        auto ArrayElement = Exp->GetMgr()->MakeExpr(LTSOps::OpIndex, Exp, Exp->GetMgr()->MakeVal(Element, IndexType));
                        ExpressionsToCheck.push_back(ArrayElement);
                    }
                } else if (ExpType->Is<ExprRecordType>()) {
                    auto ExpTypeAsRecordType = ExpType->As<ExprRecordType>();
                    auto FAType = Exp->GetMgr()->MakeType<ExprFieldAccessType>();
                    for (auto NameType: ExpTypeAsRecordType->GetMemberMap()) {
                        auto MemberName = NameType.first;
                        auto MemberType = NameType.second;
                        auto MemberAccess = Exp->GetMgr()->MakeExpr(LTSOps::OpField, Exp, Exp->GetMgr()->MakeVar(MemberName, FAType));
                        ExpressionsToCheck.push_back(MemberAccess);
                    }
                }
            }
            return Retval;
        }

        MgrT::SubstMapT TransitionSubstitutionsGivenTransMsg(const vector<LTSAssignRef>& Updates, MgrT::SubstMapT SubstMapForTransMsg) {
            MgrT::SubstMapT SubstMapForTransition;
            for (LTSAssignRef Update: Updates) {
                auto lhs = Update->GetLHS();
                auto rhs = Update->GetRHS();
                if (lhs->GetType()->Is<ExprArrayType>()) {
                    throw InternalError((string) "lhs in memory has array type " + lhs->ToString());
                }
                if (lhs->GetType()->Is<ExprRecordType>()) {
                    auto RecordType = lhs->GetType()->As<ExprRecordType>();
                    for (auto MemberNameType: RecordType->GetMemberMap()) {
                        auto MemberName = MemberNameType.first;
                        auto MemberType = MemberNameType.second;
                        auto Mgr = lhs->GetMgr();
                        auto FAType = Mgr->MakeType<ExprFieldAccessType>();
                        auto LhsMemberAccessExpression = Mgr->MakeExpr(LTSOps::OpField, lhs, Mgr->MakeVar(MemberName, FAType));
                        auto RhsMemberAccessExpression = Mgr->MakeExpr(LTSOps::OpField, rhs, Mgr->MakeVar(MemberName, FAType));
                        SubstMapForTransition[LhsMemberAccessExpression] = rhs->GetMgr()->ApplyTransform<SubstitutorForWP>(RhsMemberAccessExpression, SubstMapForTransMsg);
                    }
                } else {
                    SubstMapForTransition[lhs] = rhs->GetMgr()->ApplyTransform<SubstitutorForWP>(rhs, SubstMapForTransMsg);
                }
            }
            for (auto update: SubstMapForTransition) {
                auto lhs = update.first;
                auto rhs = update.second;
                if (lhs->GetType()->Is<ExprRecordType>()) {
                    throw InternalError((string) "In TransitionSubstitutionsGivenTransMsg: lhs in memory has a record type " + lhs->ToString());
                }
            }
            return SubstMapForTransition;
        }

        vector<LTS::GCmdRef> GuardedCommandsFromTrace(TraceBase* Trace) {
            vector<LTS::GCmdRef> GuardedCommands;
            if (Trace->Is<SafetyViolation>()) {
                auto TraceAsSafetyViolation = Trace->As<SafetyViolation>();
                auto TraceElements = TraceAsSafetyViolation->GetTraceElems();
                transform(TraceElements.begin(), TraceElements.end(), GuardedCommands.begin(),
                          [&](TraceElemT& Element){return Element.first;});
            } else if (Trace->Is<LivenessViolation>()) {
                auto TraceAsLivenessViolation = Trace->As<LivenessViolation>();
                auto TraceStemElements = TraceAsLivenessViolation->GetStem();
                auto TraceLoopElements = TraceAsLivenessViolation->GetLoop();
                transform(TraceStemElements.begin(), TraceStemElements.end(), back_inserter(GuardedCommands),
                          [&](PSTraceElemT& Element){return Element.first;});
                transform(TraceLoopElements.begin(), TraceLoopElements.end(), back_inserter(GuardedCommands),
                          [&](PSTraceElemT& Element){return Element.first;});
            }
            return GuardedCommands;
        }

        MgrT::SubstMapT GetSubstitutionsForTransMsg(const vector<LTSAssignRef>& updates) {
            MgrT::SubstMapT SubstMapForTransMsg;
            for (LTSAssignRef update: updates) {
                const ExpT& lhs = update->GetLHS();
                const ExpT& rhs = update->GetRHS();
                if (lhs->GetType()->Is<ExprRecordType>()) {
                    auto RecordType = lhs->GetType()->As<ExprRecordType>();
                    for (auto MemberNameType: RecordType->GetMemberMap()) {
                        auto MemberName = MemberNameType.first;
                        if (MemberName != "__mtype__") {
                            auto MemberType = MemberNameType.second;
                            auto Mgr = lhs->GetMgr();
                            auto FAType = Mgr->MakeType<ExprFieldAccessType>();
                            auto LhsMemberAccessExpression = Mgr->MakeExpr(LTSOps::OpField, lhs, Mgr->MakeVar(MemberName, FAType));
                            auto RhsMemberAccessExpression = Mgr->MakeExpr(LTSOps::OpField, rhs, Mgr->MakeVar(MemberName, FAType));
                            SubstMapForTransMsg[LhsMemberAccessExpression] = RhsMemberAccessExpression;
                        }
                    }
                }
            }
            return SubstMapForTransMsg;
        }

        ExpT WeakestPrecondition(ExpT InitialPhi, TraceBase* Trace) {
            ExpT Phi = InitialPhi;
            if (!Trace->Is<SafetyViolation>()) {
                throw InternalError((string)"WeakestPrecondition called" +
                                     "with a non safety violation. Call" +
                                    "WeakestPreconditionForLiveness instead.");

            }
            const vector<TraceElemT>& TraceElements = Trace->As<SafetyViolation>()->GetTraceElems();
            for (auto it = TraceElements.rbegin(); it != TraceElements.rend(); ++it) {
                GCmdRef guarded_command = it->first;
                const vector<LTSAssignRef>& updates = guarded_command->GetUpdates();
                const ExpT& guard = guarded_command->GetGuard();

                MgrT::SubstMapT SubstMapForTransMsg = GetSubstitutionsForTransMsg(updates);
                MgrT::SubstMapT SubstMapForTransition = TransitionSubstitutionsGivenTransMsg(updates, SubstMapForTransMsg);

                Phi = Phi->GetMgr()->ApplyTransform<SubstitutorForWP>(Phi, SubstMapForTransition);
                Phi = Phi->GetMgr()->MakeExpr(LTSOps::OpIMPLIES, guard, Phi);
            }
            return Phi;
        }

        ExpT WeakestPreconditionForLiveness(LabelledTS* TheLTS, StateBuchiAutomaton* Monitor, LivenessViolation* Trace) {
            auto InitStateGenerators = TheLTS->GetInitStateGenerators();
            MgrT::SubstMapT LoopValues;
            for (auto StateVariable: TheLTS->GetStateVectorVars()) {
                for (auto Exp: GetAllScalarLeaves(StateVariable)) {
                    LoopValues[Exp] = TheLTS->MakeVar(Exp->ToString() + "_0", Exp->GetType());
                }
            }
            MgrT::SubstMapT InvertLoopValues;
            for (auto Pair: LoopValues) {
                InvertLoopValues[Pair.second] = Pair.first;
            }
            ExpT InitialCondition = TheLTS->MakeFalse();
            for (auto Pair: LoopValues) {
                auto VarDifferentLoopValue = TheLTS->MakeOp(LTSOps::OpNOT,
                                                            TheLTS->MakeOp(LTSOps::OpEQ, Pair.first, Pair.second));
                InitialCondition = TheLTS->MakeOp(LTSOps::OpOR,
                                                  VarDifferentLoopValue,
                                                  InitialCondition);
            }
            vector<PSTraceElemT> Loop = Trace->GetLoop();
            vector<PSTraceElemT> Stem = Trace->GetStem();
            auto LastStem = Stem.back();
            auto PreviousMonitorState = LastStem.second->GetMonitorState();
            vector<pair<LTS::GCmdRef, ExpT>> LoopGuardedCommandsAndMonitorGuards;
            for (auto PSTraceElement: Loop) {
                auto GuardedCommand = PSTraceElement.first;
                auto ProductState = PSTraceElement.second;
                auto MonitorGuard = Monitor->GetGuardForTransition(PreviousMonitorState,
                                                                   ProductState->GetMonitorState(),
                                                                   ProductState->GetIndexID());
                LoopGuardedCommandsAndMonitorGuards.push_back(make_pair(GuardedCommand, MonitorGuard));
                PreviousMonitorState = ProductState->GetMonitorState();

            }
            vector<pair<LTS::GCmdRef, ExpT>> StemGuardedCommandsAndMonitorGuards;
            auto InitialState = Trace->GetInitialState();
            PreviousMonitorState = InitialState->GetMonitorState();
            for (auto PSTraceElement: Stem) {
                auto GuardedCommand = PSTraceElement.first;
                auto ProductState = PSTraceElement.second;
                auto MonitorGuard = Monitor->GetGuardForTransition(PreviousMonitorState,
                                                                   ProductState->GetMonitorState(),
                                                                   ProductState->GetIndexID());
                StemGuardedCommandsAndMonitorGuards.push_back(make_pair(GuardedCommand, MonitorGuard));
                PreviousMonitorState = ProductState->GetMonitorState();
            }
            auto Phi = InitialCondition;
            for (auto it = LoopGuardedCommandsAndMonitorGuards.rbegin();
                 it != LoopGuardedCommandsAndMonitorGuards.rend();
                 ++it) {
                GCmdRef GuardedCommand = it->first;
                const vector<LTSAssignRef>& updates = GuardedCommand->GetUpdates();
                const ExpT& Guard = GuardedCommand->GetGuard();
                ExpT ProductGuard = TheLTS->MakeOp(LTSOps::OpAND,
                                                   Guard,
                                                   it->second);

                MgrT::SubstMapT SubstMapForTransMsg = GetSubstitutionsForTransMsg(updates);
                MgrT::SubstMapT SubstMapForTransition = TransitionSubstitutionsGivenTransMsg(updates, SubstMapForTransMsg);

                Phi = Phi->GetMgr()->ApplyTransform<SubstitutorForWP>(Phi, SubstMapForTransition);
                Phi = Phi->GetMgr()->MakeExpr(LTSOps::OpIMPLIES, ProductGuard, Phi);
            }
            Phi = Phi->GetMgr()->ApplyTransform<SubstitutorForWP>(Phi, InvertLoopValues);
            for (auto it = StemGuardedCommandsAndMonitorGuards.rbegin();
                 it != StemGuardedCommandsAndMonitorGuards.rend();
                 ++it) {
                GCmdRef GuardedCommand = it->first;
                const vector<LTSAssignRef>& updates = GuardedCommand->GetUpdates();
                const ExpT& Guard = GuardedCommand->GetGuard();
                ExpT ProductGuard = TheLTS->MakeOp(LTSOps::OpAND,
                                                   Guard,
                                                   it->second);

                MgrT::SubstMapT SubstMapForTransMsg = GetSubstitutionsForTransMsg(updates);
                MgrT::SubstMapT SubstMapForTransition = TransitionSubstitutionsGivenTransMsg(updates, SubstMapForTransMsg);

                Phi = Phi->GetMgr()->ApplyTransform<SubstitutorForWP>(Phi, SubstMapForTransition);
                Phi = Phi->GetMgr()->MakeExpr(LTSOps::OpIMPLIES, ProductGuard, Phi);
            }
            return Phi;
        }

        // TODO Need to deal with clear assignments: Anything that's cleared has to be removed from memory.
        // Returns path condition and adds intermediate states in symbolic_states
        ExpT SymbolicExecution(ExpT InitialPredicate, TraceBase* Trace, vector<MgrT::SubstMapT>& SymbolicStates) {
            MgrT::SubstMapT Memory;
            vector<LTS::GCmdRef> GuardedCommands = GuardedCommandsFromTrace(Trace);
            ExpT PathCondition = InitialPredicate;
            for (auto it = GuardedCommands.begin(); it != GuardedCommands.end(); ++it) {
                auto guarded_command = *it;
                const vector<LTSAssignRef>& updates = guarded_command->GetUpdates();
                const ExpT& guard = guarded_command->GetGuard();
                ExpT new_guard = guard->GetMgr()->ApplyTransform<SubstitutorForWP>(guard, Memory);
                MgrT::SubstMapT SubstMapForTransMsg = GetSubstitutionsForTransMsg(updates);
                MgrT::SubstMapT SubstMapForTransition = TransitionSubstitutionsGivenTransMsg(updates, SubstMapForTransMsg);
                MgrT::SubstMapT NewMemory;
                for (auto update: SubstMapForTransition) {
                    auto lhs = update.first;
                    auto TempMemory = Memory;
                    TempMemory.erase(lhs);
                    lhs = lhs->GetMgr()->ApplyTransform<SubstitutorForWP>(lhs, TempMemory);
                    auto rhs = update.second;
                    auto new_rhs = rhs->GetMgr()->ApplyTransform<SubstitutorForWP>(update.second, Memory);
                    NewMemory[lhs] = new_rhs;
                }

                NewMemory.insert(Memory.begin(), Memory.end());
                Memory = NewMemory;
                SymbolicStates.push_back(Memory);

                PathCondition = PathCondition->GetMgr()->MakeExpr(LTSOps::OpAND, new_guard, PathCondition);
            }
            return PathCondition;
        }

        vector<ExpT> SymbolicExecution(LabelledTS* TheLTS, TraceBase* Trace, vector<vector<MgrT::SubstMapT>>& symbolic_states_per_initial) {
            vector<ExpT> PathConditions;
            MgrT::SubstMapT Memory;
            vector<LTS::GCmdRef> GuardedCommands = GuardedCommandsFromTrace(Trace);
            auto InitStateGenerators = TheLTS->GetInitStateGenerators();
            for (auto InitState: InitStateGenerators) {
                vector<MgrT::SubstMapT> symbolic_states;
                Memory.clear();
                for (auto update: InitState) {
                    if (update->GetRHS()->SAs<Exprs::ConstExpression>()->GetConstValue() != "clear") {
                        Memory[update->GetLHS()] = update->GetRHS();
                    }
                }
                ExpT path_condition = TheLTS->MakeTrue();
                for (auto it = GuardedCommands.begin(); it != GuardedCommands.end(); ++it) {
                    GCmdRef guarded_command = *it;
                    const vector<LTSAssignRef>& updates = guarded_command->GetUpdates();
                    const ExpT& guard = guarded_command->GetGuard();
                    ExpT new_guard = guard->GetMgr()->ApplyTransform<SubstitutorForWP>(guard, Memory);
                    MgrT::SubstMapT SubstMapForTransMsg = GetSubstitutionsForTransMsg(updates);
                    MgrT::SubstMapT SubstMapForTransition = TransitionSubstitutionsGivenTransMsg(updates, SubstMapForTransMsg);
                    MgrT::SubstMapT NewMemory;
                    for (auto update: SubstMapForTransition) {
                        auto lhs = update.first;
                        auto TempMemory = Memory;
                        TempMemory.erase(lhs);
                        lhs = lhs->GetMgr()->ApplyTransform<SubstitutorForWP>(lhs, TempMemory);
                        auto rhs = update.second;
                        auto new_rhs = rhs->GetMgr()->ApplyTransform<SubstitutorForWP>(rhs, Memory);
                        NewMemory[lhs] = new_rhs;
                    }
                    NewMemory.insert(Memory.begin(), Memory.end());
                    Memory = NewMemory;
                    symbolic_states.push_back(Memory);
                    path_condition = path_condition->GetMgr()->MakeExpr(LTSOps::OpAND, new_guard, path_condition);
                }
                symbolic_states_per_initial.push_back(symbolic_states);
                PathConditions.push_back(path_condition);
            }
            return PathConditions;
        }
    }
}

// 
// LTSAnalyses.cpp ends here
