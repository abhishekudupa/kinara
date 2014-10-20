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

        ExpT WeakestPrecondition(ExpT InitialPhi, TraceBase* Trace) {
            ExpT Phi = InitialPhi;
            if (!Trace->Is<SafetyViolation>()) {
                throw InternalError((string)"WeakestPrecondition called" +
                                     "with a non safety violation. Call" +
                                    "SymbolicExecution instead.");

            }

            const vector<TraceElemT>& TraceElements = Trace->As<SafetyViolation>()->GetTraceElems();
            for (auto it = TraceElements.rbegin(); it != TraceElements.rend(); ++it) {
                GCmdRef guarded_command = it->first;
                const vector<LTSAssignRef>& updates = guarded_command->GetUpdates();
                const ExpT& guard = guarded_command->GetGuard();

                MgrT::SubstMapT SubstMapForTransMsg;
                for (LTSAssignRef update: updates) {
                    const ExpT& lhs = update->GetLHS();
                    if (lhs->Is<OpExpression>()) {
                        auto lhs_as_op = lhs->As<OpExpression>();
                        auto children = lhs_as_op->GetChildren();
                        auto lhs_base = children[0];
                        if (lhs_base->Is<VarExpression>()) {
                            auto lhs_base_var = lhs_base->As<VarExpression>();
                            if (lhs_base_var->GetVarName() == "__trans_msg__") {
                                SubstMapForTransMsg[lhs] = update->GetRHS();
                            }
                        }
                    }
                }
                MgrT::SubstMapT SubstMapForTransition;
                for (LTSAssignRef update: updates) {
                    auto lhs = update->GetLHS();
                    auto rhs = update->GetRHS();
                    SubstMapForTransition[lhs] = rhs->GetMgr()->ApplyTransform<SubstitutorForWP>(rhs, SubstMapForTransMsg);
                }

                Phi = Phi->GetMgr()->ApplyTransform<SubstitutorForWP>(Phi, SubstMapForTransition);
                Phi = Phi->GetMgr()->MakeExpr(LTSOps::OpIMPLIES, guard, Phi);
            }
            return Phi;
        }

        // Returns path condition and adds intermediate states in symbolic_states
        ExpT SymbolicExecution(ExpT InitialPredicate, TraceBase* Trace, vector<MgrT::SubstMapT>& symbolic_states) {
            MgrT::SubstMapT memory;
            const vector<TraceElemT>& TraceElements = Trace->As<SafetyViolation>()->GetTraceElems();
            ExpT path_condition = InitialPredicate;
            for (auto it = TraceElements.begin(); it != TraceElements.end(); ++it) {
                GCmdRef guarded_command = it->first;
                const vector<LTSAssignRef>& updates = guarded_command->GetUpdates();
                const ExpT& guard = guarded_command->GetGuard();
                ExpT new_guard = guard->GetMgr()->ApplyTransform<SubstitutorForWP>(guard, memory);
                MgrT::SubstMapT SubstMapForTransMsg;
                for (LTSAssignRef update: updates) {
                    const ExpT& lhs = update->GetLHS();
                    if (lhs->Is<OpExpression>()) {
                        auto lhs_as_op = lhs->As<OpExpression>();
                        auto children = lhs_as_op->GetChildren();
                        auto lhs_base = children[0];
                        if (lhs_base->Is<VarExpression>()) {
                            auto lhs_base_var = lhs_base->As<VarExpression>();
                            if (lhs_base_var->GetVarName() == "__trans_msg__") {
                                SubstMapForTransMsg[lhs] = update->GetRHS();
                            }
                        }
                    }
                }
                MgrT::SubstMapT SubstMapForTransition;
                for (LTSAssignRef update: updates) {
                    auto lhs = update->GetLHS();
                    auto rhs = update->GetRHS();
                    if (lhs->Is<OpExpression>()) {
                        auto lhs_as_op = lhs->As<OpExpression>();
                        auto children = lhs_as_op->GetChildren();
                        auto lhs_base = children[0];
                        if (lhs_base->Is<VarExpression>()) {
                            auto lhs_base_var = lhs_base->As<VarExpression>();
                            if (lhs_base_var->GetVarName() == "__trans_msg__") {
                                continue;
                            }
                        }
                    }
                    SubstMapForTransition[lhs] = rhs->GetMgr()->ApplyTransform<SubstitutorForWP>(rhs, SubstMapForTransMsg);
                }
                MgrT::SubstMapT new_memory;
                for (auto update: SubstMapForTransition) {
                    auto lhs = update.first;
                    auto rhs = update.second;
                    auto new_rhs = rhs->GetMgr()->ApplyTransform<SubstitutorForWP>(rhs, memory);
                    new_memory[lhs] = new_rhs;
                }
                for (auto update: memory) {
                    auto lhs = update.first;
                    auto rhs = update.second;
                    if (new_memory.find(lhs) == new_memory.end()) {
                        new_memory[lhs] = memory[lhs];
                    }
                }
                memory = new_memory;
                symbolic_states.push_back(memory);

                path_condition = path_condition->GetMgr()->MakeExpr(LTSOps::OpAND, new_guard, path_condition);
            }
            return path_condition;
        }

        vector<ExpT> SymbolicExecution(LabelledTS* TheLTS, TraceBase* Trace, vector<vector<MgrT::SubstMapT>>& symbolic_states_per_initial) {
            vector<ExpT> PathConditions;
            MgrT::SubstMapT Memory;
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
            auto InitStateGenerators = TheLTS->GetInitStateGenerators();
            for (auto InitState: InitStateGenerators) {
                vector<MgrT::SubstMapT> symbolic_states;
                Memory.clear();
                for (auto update: InitState) {
                    Memory[update->GetLHS()] = update->GetRHS();
                }
                ExpT path_condition = TheLTS->MakeTrue();
                for (auto it = GuardedCommands.begin(); it != GuardedCommands.end(); ++it) {
                    GCmdRef guarded_command = *it;
                    const vector<LTSAssignRef>& updates = guarded_command->GetUpdates();
                    const ExpT& guard = guarded_command->GetGuard();
                    ExpT new_guard = guard->GetMgr()->ApplyTransform<SubstitutorForWP>(guard, Memory);
                    MgrT::SubstMapT SubstMapForTransMsg;
                    for (LTSAssignRef update: updates) {
                        const ExpT& lhs = update->GetLHS();
                        if (lhs->Is<OpExpression>()) {
                            auto lhs_as_op = lhs->As<OpExpression>();
                            auto children = lhs_as_op->GetChildren();
                            auto lhs_base = children[0];
                            if (lhs_base->Is<VarExpression>()) {
                                auto lhs_base_var = lhs_base->As<VarExpression>();
                                if (lhs_base_var->GetVarName() == "__trans_msg__") {
                                    SubstMapForTransMsg[lhs] = update->GetRHS();
                                }
                            }
                        }
                    }
                    MgrT::SubstMapT SubstMapForTransition;
                    for (LTSAssignRef update: updates) {
                        auto lhs = update->GetLHS();
                        auto rhs = update->GetRHS();
                        if (lhs->Is<OpExpression>()) {
                            auto lhs_as_op = lhs->As<OpExpression>();
                            auto children = lhs_as_op->GetChildren();
                            auto lhs_base = children[0];
                            if (lhs_base->Is<VarExpression>()) {
                                auto lhs_base_var = lhs_base->As<VarExpression>();
                                if (lhs_base_var->GetVarName() == "__trans_msg__") {
                                    continue;
                                }
                            }
                        }
                        SubstMapForTransition[lhs] = rhs->GetMgr()->ApplyTransform<SubstitutorForWP>(rhs, SubstMapForTransMsg);
                    }
                    MgrT::SubstMapT new_Memory;
                    for (auto update: SubstMapForTransition) {
                        auto lhs = update.first;
                        auto rhs = update.second;
                        auto new_rhs = rhs->GetMgr()->ApplyTransform<SubstitutorForWP>(rhs, Memory);
                        new_Memory[lhs] = new_rhs;
                    }
                    for (auto update: Memory) {
                        auto lhs = update.first;
                        auto rhs = update.second;
                        if (new_Memory.find(lhs) == new_Memory.end()) {
                            new_Memory[lhs] = Memory[lhs];
                        }
                    }
                    Memory = new_Memory;
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
