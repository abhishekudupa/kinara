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
#include <numeric>

#include "LTSAnalyses.hpp"
#include "../uflts/LabelledTS.hpp"
#include "../uflts/LTSAssign.hpp"
#include "../uflts/LTSEFSMBase.hpp"
#include "../uflts/LTSTypes.hpp"
#include "../uflts/LTSTransitions.hpp"
#include "../mc/AQStructure.hpp"
#include "../mc/Compiler.hpp"
#include "../mc/OmegaAutomaton.hpp"
#include "../mc/Trace.hpp"
#include "../synth/Solver.hpp"
#include "../tpinterface/TheoremProver.hpp"

namespace ESMC {
    namespace Analyses {
        using namespace LTS;
        using namespace MC;
        using namespace Synth;
        using namespace TP;

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

        vector<ExpT> TraceAnalyses::GetAllScalarLeaves(ExpT InitialExp) {
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
                    for (auto Element: IndexType->GetElementsNoUndef()) {
                        auto ArrayElement = 
                            Exp->GetMgr()->MakeExpr(LTSOps::OpIndex, 
                                                    Exp, 
                                                    Exp->GetMgr()->MakeVal(Element, IndexType));
                        ExpressionsToCheck.push_back(ArrayElement);
                    }
                } else if (ExpType->Is<ExprRecordType>()) {
                    auto ExpTypeAsRecordType = ExpType->As<ExprRecordType>();
                    auto FAType = Exp->GetMgr()->MakeType<ExprFieldAccessType>();
                    for (auto NameType: ExpTypeAsRecordType->GetMemberMap()) {
                        auto MemberName = NameType.first;
                        auto MemberType = NameType.second;
                        auto Field = Exp->GetMgr()->MakeVar(MemberName, FAType);
                        auto MemberAccess = Exp->GetMgr()->MakeExpr(LTSOps::OpField, Exp, Field);
                        ExpressionsToCheck.push_back(MemberAccess);
                    }
                }
            }
            return Retval;
        }

        MgrT::SubstMapT
        TraceAnalyses::TransitionSubstitutionsGivenTransMsg(const vector<LTSAssignRef>& Updates,
                                                            MgrT::SubstMapT SubstMapForTransMsg)
        {
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
                        auto Field = Mgr->MakeVar(MemberName, FAType);
                        auto LhsMemberAccessExpression = Mgr->MakeExpr(LTSOps::OpField, lhs, Field);
                        Field = Mgr->MakeVar(MemberName, FAType);
                        auto RhsMemberAccessExpression = Mgr->MakeExpr(LTSOps::OpField, rhs, Field);
                        Mgr = rhs->GetMgr();
                        auto NewLhs = Mgr->ApplyTransform<SubstitutorForWP>
                            (RhsMemberAccessExpression, SubstMapForTransMsg);
                        SubstMapForTransition[LhsMemberAccessExpression] = NewLhs;
                    }
                } else {
                    auto Mgr = rhs->GetMgr();
                    auto NewLhs = Mgr->ApplyTransform<SubstitutorForWP>(rhs, SubstMapForTransMsg);
                    SubstMapForTransition[lhs] = NewLhs;
                }
            }
            for (auto update: SubstMapForTransition) {
                auto lhs = update.first;
                auto rhs = update.second;
                if (lhs->GetType()->Is<ExprRecordType>()) {
                    throw InternalError((string) "In TransitionSubstitutionsGivenTransMsg:" +
                                        "lhs in memory has a record type " + lhs->ToString());
                }
            }
            return SubstMapForTransition;
        }

        vector<GCmdRef> TraceAnalyses::GuardedCommandsFromTrace(TraceBase* Trace)
        {
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
                transform(TraceStemElements.begin(),
                          TraceStemElements.end(),
                          back_inserter(GuardedCommands),
                          [&](PSTraceElemT& Element){return Element.first;});
                transform(TraceLoopElements.begin(),
                          TraceLoopElements.end(),
                          back_inserter(GuardedCommands),
                          [&](PSTraceElemT& Element){return Element.first;});
            }
            return GuardedCommands;
        }

        MgrT::SubstMapT 
        TraceAnalyses::GetSubstitutionsForTransMsg(const vector<LTSAssignRef>& updates)
        {
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
                            auto Field = Mgr->MakeVar(MemberName, FAType);
                            auto LhsMemberAccessExpression = Mgr->MakeExpr(LTSOps::OpField,
                                                                           lhs, Field);

                            auto RhsMemberAccessExpression = Mgr->MakeExpr(LTSOps::OpField,
                                                                           rhs, Field);
                            SubstMapForTransMsg[LhsMemberAccessExpression] =
                                RhsMemberAccessExpression;
                        }
                    }
                }
            }
            return SubstMapForTransMsg;
        }

        set<LTSFairObjRef> TraceAnalyses::GetLTSFairnessObjects(LTS::LabelledTS* TheLTS)
        {
            set<LTSFairObjRef> Retval;

            // method 1
            // auto AllEFSMs = TheLTS->GetEFSMs([&] (const EFSMBase *) {return true;});
            // auto FairnessSets = EFSM->GetAllFairnessSets()->GetFairnessSets();
            // for (auto NameFairnessSet: FairnessSets) {
            //     for (auto ParamFairnessObj: NameFairnessSet.second->GetFairnessObjs()) {
            //         Retval.insert(ParamFairnessObj.second);
            //     }
            // }

            // method 2
            // for (auto OutputMsg: EFSM->GetOutputs()) {
            //     for (auto Transition: EFSM->GetOutputTransitionsOnMsg(OutputMsg)) {
            //         auto OutputTransition = Transition->SAs<LTSTransitionOutput>();
            //         auto const& ParamInst = OutputTransition->GetParamInst();
            //         auto const& StrFairnesses = OutputTransition->GetCompOfFairnessSets();
            //         for (auto const& StrFairness : StrFairnesses) {
            //             auto const& ActFairness =
            //                 AllFairnesses->GetFairnessObj(StrFairness, ParamInst);
            //             Retval.insert(ActFairness);
            //         }
            //     }
            // }
            // for (auto Transition: EFSM->GetInternalTransitions()) {
            //     auto InternalTransition = Transition->SAs<LTSTransitionInternal>();
            //     auto const& ParamInst = InternalTransition->GetParamInst();
            //     auto const& StrFairnesses = InternalTransition->GetCompOfFairnessSets();
            //     set<LTSFairObjRef> ActualFairnesses;
            //     for (auto const& StrFairness : StrFairnesses) {
            //         auto const& ActFairness =
            //             AllFairnesses->GetFairnessObj(StrFairness, ParamInst);
            //         FairnessObjectsInLTS.insert(ActFairness);
            //     }
            // }

            // method 3
            for (auto GuardedCommand: TheLTS->GetGuardedCmds()) {
                for (auto FairnessObject: GuardedCommand->GetFairnessObjs()) {
                    Retval.insert(FairnessObject);
                }
            }
            return Retval;
        }

        const StateVec* TraceAnalyses::GetLastState(SafetyViolation* Trace)
        {
            return Trace->GetTraceElems().back().second;
        }

        bool TraceAnalyses::HasUF(ExpT Exp) {
            auto HasUF = [&] (const ExpBaseT* Exp) -> bool
                {
                    auto ExpAsOpExp = Exp->As<OpExpression>();
                    if (ExpAsOpExp != nullptr) {
                        auto Code = ExpAsOpExp->GetOpCode();
                        return (Code >= LTSOps::UFOffset);
                    }
                    return false;
                };
            return Exp->GetMgr()->Gather(Exp, HasUF).size() > 0;
        }

        vector<LTS::GCmdRef>
        TraceAnalyses::TentativeGuardedCommandsInLTS(LTS::LabelledTS* TheLTS)
        {
            vector<LTS::GCmdRef> Retval;
            for (auto GuardedCommand : TheLTS->GetGuardedCmds()) {
                auto Guard = GuardedCommand->GetGuard();
                auto HasUF = [&] (const ExpBaseT* Exp) -> bool
                    {
                        auto ExpAsOpExp = Exp->As<OpExpression>();
                        if (ExpAsOpExp != nullptr) {
                            auto Code = ExpAsOpExp->GetOpCode();
                            return (Code >= LTSOps::UFOffset);
                        }
                        return false;
                    };
                auto UFFunctionsinGuard = Guard->GetMgr()->Gather(Guard, HasUF);
                if (UFFunctionsinGuard.size() > 0) {
                    Retval.push_back(GuardedCommand);
                }
            }
            return  Retval;
        }

        map<vector<ExpT>, ExpT>
        TraceAnalyses::ModelResults(LabelledTS* TheLTS, ExpT UFExp, TPRef TP)
        {
            map<vector<ExpT>, ExpT> Retval;
            vector<vector<ExpT>> Inputs;
            auto UFExpAsOp = UFExp->As<OpExpression>();
            auto Children = UFExpAsOp->GetChildren();
            for (auto Child : Children) {
                vector<ExpT> ChildrenInputs;
                for (auto Element : Child->GetType()->GetElements()) {
                    auto ExpValue = TheLTS->MakeVal(Element, Child->GetType());
                    ChildrenInputs.push_back(ExpValue);
                }
                Inputs.push_back(ChildrenInputs);
            }
            auto Product = CrossProduct<ExpT>(Inputs.begin(), Inputs.end());
            for (auto Combo : Product) {
                auto NewExp = TheLTS->MakeOp(UFExpAsOp->GetOpCode(), Combo);
                Retval[Combo] = TP->Evaluate(NewExp);
            }
            return Retval;
        }

        ExpT
        TraceAnalyses::ConditionToResolveDeadlock(LabelledTS* TheLTS,
                                                  DeadlockViolation* DeadlockTrace)
        {
            vector<GCmdRef> PotentialCommands;
            auto LastState = GetLastState(DeadlockTrace);
            for (auto GuardedCommand : TheLTS->GetGuardedCmds()) {
                if (IsGuardedCommandEnabled(TheLTS, LastState, GuardedCommand)) {
                    PotentialCommands.push_back(GuardedCommand);
                }
            }
            auto GetGuard = [&](GCmdRef GCmd){return GCmd->GetGuard();};
            vector<ExpT> PotentialGuards;
            transform(PotentialCommands.begin(), PotentialCommands.end(),
                      back_inserter(PotentialGuards), GetGuard);
            auto Disjunct = [&](ExpT One, ExpT Two)
                {
                    if (One == TheLTS->MakeFalse()) {
                        return Two;
                    } else {
                        return TheLTS->MakeOp(LTSOps::OpOR, One, Two);
                    }
                };
            auto GuardCondition = accumulate(PotentialGuards.begin(), PotentialGuards.end(),
                                             TheLTS->MakeFalse(), Disjunct);
            auto StateCondition = AutomataStatesCondition(TheLTS, LastState);
            return TheLTS->MakeOp(LTSOps::OpOR, GuardCondition,
                                  TheLTS->MakeOp(LTSOps::OpNOT, StateCondition));
        }


        set<LTSFairObjRef>
        TraceAnalyses::GetLoopFairnessObjects(LabelledTS* TheLTS,
                                              const LivenessViolation* LivenessViolation)
        {
            set<LTSFairObjRef> Retval;
            for (auto TraceElement: LivenessViolation->GetLoop()) {
                auto GuardedCommand = TraceElement.first;
                auto FairObjects = GuardedCommand->GetFairnessObjs();
                for (auto FairObject: FairObjects) {
                    Retval.insert(FairObject);
                }
            }
            return Retval;
        }

        set<LTSFairObjRef>
        TraceAnalyses::TriviallySatisfiedFairnessObjectsInLoop(LabelledTS* TheLTS,
                                                               const LivenessViolation* 
                                                               LivenessViolation)
        {
            auto Retval = GetLTSFairnessObjects(TheLTS);
            auto LoopFairnessObjects = GetLoopFairnessObjects(TheLTS, LivenessViolation);
            for (auto FairnessObject : LoopFairnessObjects) {
                Retval.erase(FairnessObject);
            }
            // Retval.erase(LoopFairnessObjects.begin(), LoopFairnessObjects.end());
            return Retval;
        }

        ExpT TraceAnalyses::WeakestPreconditionWithMonitor(LabelledTS* TheLTS,
                                                           StateBuchiAutomaton* Monitor,
                                                           MgrT::SubstMapT InitialStateSubstMap,
                                                           const LivenessViolation* Trace,
                                                           ExpT InitialCondition,
                                                           int StartIndexInLoop)
        {
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
                auto Pair = make_pair(GuardedCommand, MonitorGuard);
                LoopGuardedCommandsAndMonitorGuards.push_back(Pair);
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
                auto Pair = make_pair(GuardedCommand, MonitorGuard);
                StemGuardedCommandsAndMonitorGuards.push_back(Pair);
                PreviousMonitorState = ProductState->GetMonitorState();
            }
            auto Phi = InitialCondition;

            for (auto it = LoopGuardedCommandsAndMonitorGuards.begin() + StartIndexInLoop;
                 it != LoopGuardedCommandsAndMonitorGuards.begin();
                 --it) {
                GCmdRef GuardedCommand = it->first;
                const vector<LTSAssignRef>& updates = GuardedCommand->GetUpdates();
                const ExpT& Guard = GuardedCommand->GetGuard();
                ExpT ProductGuard = TheLTS->MakeOp(LTSOps::OpAND,
                                                   Guard,
                                                   it->second);
                MgrT::SubstMapT SubstMapForTransMsg = GetSubstitutionsForTransMsg(updates);
                MgrT::SubstMapT SubstMapForTransition =
                    TransitionSubstitutionsGivenTransMsg(updates, SubstMapForTransMsg);

                Phi = Phi->GetMgr()->ApplyTransform<SubstitutorForWP>(Phi, SubstMapForTransition);
                Phi = Phi->GetMgr()->MakeExpr(LTSOps::OpIMPLIES, ProductGuard, Phi);
            }

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
                MgrT::SubstMapT SubstMapForTransition =
                    TransitionSubstitutionsGivenTransMsg(updates, SubstMapForTransMsg);

                Phi = Phi->GetMgr()->ApplyTransform<SubstitutorForWP>(Phi, SubstMapForTransition);
                Phi = Phi->GetMgr()->MakeExpr(LTSOps::OpIMPLIES, ProductGuard, Phi);
            }

            return Phi->GetMgr()->ApplyTransform<SubstitutorForWP>(Phi, InitialStateSubstMap);
        }

        ExpT
        TraceAnalyses::EnableFairnessObjectsInLoop(LabelledTS* TheLTS,
                                                   StateBuchiAutomaton* Monitor,
                                                   MgrT::SubstMapT InitialStateSubstMap,
                                                   const LivenessViolation* LivenessViolation,
                                                   set<LTSFairObjRef> FairnessObjects)
        {
            vector<ExpT> EnableConditions;
            auto Loop = LivenessViolation->GetLoop();
            for (auto FairnessObject : FairnessObjects) {
                u16 LoopIndex = 0;
                for (PSTraceElemT TraceElement : Loop) {
                    // Check if there is any guarded command that conains
                    // the fairness object.
                    for (auto Cmd : TheLTS->GetGuardedCmds()) {
                        auto GCmdFairObjs = Cmd->GetFairnessObjs();
                        auto it = find(GCmdFairObjs.begin(), GCmdFairObjs.end(), FairnessObject);
                        if (it != GCmdFairObjs.end()) {
                            auto const& FixedInterp = Cmd->GetFixedInterpretation();
                            auto Interp = FixedInterp->ExtensionData.Interp;
                            auto Res = Interp->Evaluate(TraceElement.second->GetSVPtr());
                            if (Res == 0) {
                                continue;
                            }
                            auto Condition =
                                WeakestPreconditionWithMonitor(TheLTS,
                                                               Monitor,
                                                               InitialStateSubstMap,
                                                               LivenessViolation,
                                                               Cmd->GetGuard(),
                                                               LoopIndex);
                            EnableConditions.push_back(Condition);
                        }
                    }
                    LoopIndex++;
                }
            }
            if (EnableConditions.size() == 0) {
                return TheLTS->MakeFalse();
            } else if (EnableConditions.size() == 1) {
                return EnableConditions[0];
            } else {
                return TheLTS->MakeOp(LTSOps::OpOR, EnableConditions);
            }
        }

        map<pair<EFSMBase*, vector<ExpT> >, string>
        TraceAnalyses::GuardedCommandInitialStates(GCmdRef GuardedCommand)
        {
            map<pair<EFSMBase*, vector<ExpT>>, string> Retval;
            auto ProductTransition = GuardedCommand->GetProductTransition();
            vector<LTSState> ProductStates;
            for (auto Transition: GuardedCommand->GetProductTransition()) {
                vector<ExpT> ParamInst(Transition->GetParamInst());
                auto EFSMParamInst = make_pair(Transition->GetEFSM(),
                                               ParamInst);
                Retval[EFSMParamInst] = Transition->GetInitState().GetName();
            }
            return Retval;
        }

        map<pair<EFSMBase*, vector<ExpT> >, string>
        TraceAnalyses::AutomataStatesFromStateVector(LabelledTS* TheLTS, const StateVec* StateVector)
        {
            map<pair<EFSMBase*, vector<ExpT>>, string> Retval;
            auto AllEFSMs = TheLTS->GetEFSMs([&] (const EFSMBase *) {return true;});
            for (auto EFSM: AllEFSMs) {
                auto FAType = TheLTS->MakeFieldAccessType();
                auto EFSMType = TheLTS->GetEFSMType(EFSM->GetName());
                for (auto ParamInst: EFSM->GetParamInsts()) {
                    auto EFSMStateVar = TheLTS->MakeVar(EFSM->GetName(), EFSMType);
                    ExpT EFSMDotState;
                    for (auto Param: ParamInst) {
                        EFSMStateVar = TheLTS->MakeOp(LTSOps::OpIndex,
                                                      EFSMStateVar,
                                                      Param);
                    }
                    EFSMDotState = TheLTS->MakeOp(LTSOps::OpField, EFSMStateVar,
                                                  TheLTS->MakeVar("state", FAType));
                    auto Interpreter = EFSMDotState->ExtensionData.Interp;
                    i64 StateValue = Interpreter->Evaluate(StateVector);
                    auto EFSMDotStateAsEnum = EFSMDotState->GetType()->As<ExprEnumType>();
                    auto EFSMParamInst = make_pair(EFSM, ParamInst);
                    Retval[EFSMParamInst] = EFSMDotStateAsEnum->ValToConst(StateValue);
                }
            }
            return Retval;
        }

        ExpT
        TraceAnalyses::AutomataStatesCondition(LabelledTS* TheLTS, const StateVec* StateVector)
        {
            ExpT Retval = TheLTS->MakeTrue();
            auto AllEFSMs = TheLTS->GetEFSMs([&] (const EFSMBase *) {return true;});
            for (auto EFSM: AllEFSMs) {
                auto FAType = TheLTS->MakeFieldAccessType();
                auto EFSMType = TheLTS->GetEFSMType(EFSM->GetName());
                for (auto ParamInst: EFSM->GetParamInsts()) {
                    auto EFSMStateVar = TheLTS->MakeVar(EFSM->GetName(), EFSMType);
                    ExpT EFSMDotState;
                    for (auto Param: ParamInst) {
                        EFSMStateVar = TheLTS->MakeOp(LTSOps::OpIndex,
                                                      EFSMStateVar,
                                                      Param);
                    }
                    EFSMDotState = TheLTS->MakeOp(LTSOps::OpField, EFSMStateVar,
                                                  TheLTS->MakeVar("state", FAType));
                    auto Interpreter = EFSMDotState->ExtensionData.Interp;
                    i64 StateValue = Interpreter->Evaluate(StateVector);
                    auto EFSMDotStateAsEnum = EFSMDotState->GetType()->As<ExprEnumType>();
                    string StateName = EFSMDotStateAsEnum->ValToConst(StateValue);
                    auto State = TheLTS->MakeVal(StateName, EFSMDotState->GetType());
                    auto StateCondition = TheLTS->MakeOp(LTSOps::OpEQ, EFSMDotState, State);
                    Retval = TheLTS->MakeOp(LTSOps::OpAND, Retval,
                                            StateCondition);
                }
            }
            return Retval;
        }

        bool TraceAnalyses::IsGuardedCommandEnabled(LabelledTS* TheLTS,
                                                    const StateVec* StateVector,
                                                    GCmdRef GuardedCommand) {
            auto AutomataStates = AutomataStatesFromStateVector(TheLTS, StateVector);
            auto GuardedCommandStates = GuardedCommandInitialStates(GuardedCommand);
            bool enabled = true;
            for (auto EFSMParamInstState : GuardedCommandStates) {
                auto EFSMParamInst = EFSMParamInstState.first;
                auto AutomatonState = AutomataStates[EFSMParamInst];
                if (AutomatonState != EFSMParamInstState.second) {
                    enabled = false;
                    break;
                }
            }
            return enabled;
        }

        vector<ExpT>
        TraceAnalyses::WeakestPrecondition(Solver* TheSolver,
                                           SafetyViolation* Trace,
                                           ExpT InitialPredicate)
        {
            auto TP = TheoremProver::MakeProver<Z3TheoremProver>();
            auto TheLTS = TheSolver->TheLTS;
            ExpT Phi = InitialPredicate;
            auto Mgr = Phi->GetMgr();
            const vector<TraceElemT>& TraceElements = Trace->GetTraceElems();
            for (auto it = TraceElements.rbegin(); it != TraceElements.rend(); ++it) {
                GCmdRef guarded_command = it->first;
                const vector<LTSAssignRef>& updates = guarded_command->GetUpdates();
                const ExpT& guard = guarded_command->GetGuard();
                MgrT::SubstMapT SubstMapForTransMsg = GetSubstitutionsForTransMsg(updates);
                MgrT::SubstMapT SubstMapForTransition =
                    TransitionSubstitutionsGivenTransMsg(updates, SubstMapForTransMsg);
                Phi = Mgr->ApplyTransform<SubstitutorForWP>(Phi, SubstMapForTransition);
                Phi = Mgr->MakeExpr(LTSOps::OpIMPLIES, guard, Phi);
            }
            vector<ExpT> Retval;
            auto InitStateGenerators = TheLTS->GetInitStateGenerators();

            for (auto const& InitState : InitStateGenerators) {
                MgrT::SubstMapT InitStateSubstMap;
                vector<ExpT> InitialStateConjuncts;
                for (auto update: InitState) {
                    auto LHS = update->GetLHS();
                    auto RHS = update->GetRHS();
                    InitStateSubstMap[LHS] = RHS;
                }
                auto NewPhi = 
                    Mgr->ApplyTransform<SubstitutorForWP>(Phi, InitStateSubstMap);
                NewPhi = Mgr->Simplify(NewPhi);
                if (NewPhi == Mgr->MakeFalse()) {
                    continue;
                }
                
                auto TPRes = TP->CheckSat(NewPhi, true);
                if (TPRes == TPResult::SATISFIABLE) {
                    Retval.push_back(NewPhi);
                } else if (TPRes == TPResult::UNKNOWN) {
                    throw IncompleteTheoryException(NewPhi);
                } else {
                    continue;
                }
            }
            return Retval;
        }

        ExpT TraceAnalyses::WeakestPrecondition(ExpT InitialPhi, TraceBase* Trace)
        {
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
                MgrT::SubstMapT SubstMapForTransition =
                    TransitionSubstitutionsGivenTransMsg(updates, SubstMapForTransMsg);

                Phi = Phi->GetMgr()->ApplyTransform<SubstitutorForWP>(Phi, SubstMapForTransition);
                Phi = Phi->GetMgr()->MakeExpr(LTSOps::OpIMPLIES, guard, Phi);
            }
            return Phi;
        }

        ExpT TraceAnalyses::WeakestPreconditionForLiveness(Solver* TheSolver,
                                                           StateBuchiAutomaton* Monitor,
                                                           const LivenessViolation* Trace)
        {
            auto TheLTS = TheSolver->TheLTS;
            auto InitStateGenerators = TheLTS->GetInitStateGenerators();
            auto TP = TheoremProver::MakeProver<Z3TheoremProver>();
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
                auto VarDifferentLoopValue =
                    TheLTS->MakeOp(LTSOps::OpNOT,
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
                auto Pair = make_pair(GuardedCommand, MonitorGuard);
                LoopGuardedCommandsAndMonitorGuards.push_back(Pair);
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
                auto Pair = make_pair(GuardedCommand, MonitorGuard);
                StemGuardedCommandsAndMonitorGuards.push_back(Pair);
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
                MgrT::SubstMapT SubstMapForTransition =
                    TransitionSubstitutionsGivenTransMsg(updates, SubstMapForTransMsg);

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
                MgrT::SubstMapT SubstMapForTransition =
                    TransitionSubstitutionsGivenTransMsg(updates, SubstMapForTransMsg);

                Phi = Phi->GetMgr()->ApplyTransform<SubstitutorForWP>(Phi, SubstMapForTransition);
                Phi = Phi->GetMgr()->MakeExpr(LTSOps::OpIMPLIES, ProductGuard, Phi);
            }

            auto TrivialFairObjs =
                TriviallySatisfiedFairnessObjectsInLoop(TheLTS, Trace);

            vector<ExpT> Conjuncts;
            for (auto const& InitState : InitStateGenerators) {
                MgrT::SubstMapT InitStateSubstMap;
                vector<ExpT> InitialStateConjuncts;
                for (auto update: InitState) {
                    auto LHS = update->GetLHS();
                    auto RHS = update->GetRHS();
                    InitStateSubstMap[LHS] = RHS;
                }
                auto Mgr = Phi->GetMgr();
                auto NewPhi =
                    Mgr->ApplyTransform<SubstitutorForWP>(Phi, InitStateSubstMap);
                NewPhi = Mgr->Simplify(NewPhi);
                if (NewPhi == Mgr->MakeFalse()) {
                    continue;
                }

                auto TPRes = TP->CheckSat(NewPhi, true);
                if (TPRes == TPResult::UNKNOWN) {
                    throw IncompleteTheoryException(NewPhi);
                } else if (TPRes == TPResult::UNSATISFIABLE) {
                    continue;
                }

                auto FairnessCondition =
                    EnableFairnessObjectsInLoop(TheLTS,
                                                Monitor,
                                                InitStateSubstMap,
                                                Trace,
                                                TrivialFairObjs);
                Conjuncts.push_back(TheLTS->MakeOp(LTSOps::OpOR, NewPhi, FairnessCondition));
            }
            if (Conjuncts.size() == 0) {
                throw ESMCError((string) "No condition found for liveness!");
            } else if (Conjuncts.size() == 1) {
                return Conjuncts[0];
            } else {
                return TheLTS->MakeOp(LTSOps::OpAND, Conjuncts);
            }
        }

        // TODO Need to deal with clear assignments:
        //    Anything that's cleared has to be removed from memory.
        // Returns path condition and adds intermediate states in symbolic_states
        ExpT TraceAnalyses::SymbolicExecution(ExpT InitialPredicate,
                                              TraceBase* Trace,
                                              vector<MgrT::SubstMapT>& SymbolicStates)
        {
            MgrT::SubstMapT Memory;
            vector<LTS::GCmdRef> GuardedCommands = GuardedCommandsFromTrace(Trace);
            ExpT PathCondition = InitialPredicate;
            for (auto it = GuardedCommands.begin(); it != GuardedCommands.end(); ++it) {
                auto guarded_command = *it;
                const vector<LTSAssignRef>& updates = guarded_command->GetUpdates();
                const ExpT& guard = guarded_command->GetGuard();
                ExpT new_guard = guard->GetMgr()->ApplyTransform<SubstitutorForWP>(guard, Memory);
                MgrT::SubstMapT SubstMapForTransMsg = GetSubstitutionsForTransMsg(updates);
                MgrT::SubstMapT SubstMapForTransition =
                    TransitionSubstitutionsGivenTransMsg(updates, SubstMapForTransMsg);
                MgrT::SubstMapT NewMemory;
                for (auto update: SubstMapForTransition) {
                    auto lhs = update.first;
                    auto TempMemory = Memory;
                    TempMemory.erase(lhs);
                    lhs = lhs->GetMgr()->ApplyTransform<SubstitutorForWP>(lhs, TempMemory);
                    auto Rhs = update.second;
                    auto Mgr = Rhs->GetMgr();
                    auto NewRhs = Mgr->ApplyTransform<SubstitutorForWP>(update.second, Memory);
                    NewMemory[lhs] = NewRhs;
                }

                NewMemory.insert(Memory.begin(), Memory.end());
                Memory = NewMemory;
                SymbolicStates.push_back(Memory);

                auto Mgr = PathCondition->GetMgr();
                PathCondition = Mgr->MakeExpr(LTSOps::OpAND, new_guard, PathCondition);
            }
            return PathCondition;
        }

        vector<ExpT> 
        TraceAnalyses::SymbolicExecution(LabelledTS* TheLTS, TraceBase* Trace,
                                         vector<vector<MgrT::SubstMapT>>& 
                                         symbolic_states_per_initial)
        {
            vector<ExpT> PathConditions;
            MgrT::SubstMapT Memory;
            vector<LTS::GCmdRef> GuardedCommands = GuardedCommandsFromTrace(Trace);
            auto InitStateGenerators = TheLTS->GetInitStateGenerators();
            for (auto InitState: InitStateGenerators) {
                vector<MgrT::SubstMapT> symbolic_states;
                Memory.clear();
                for (auto update: InitState) {
                    auto Rhs = update->GetRHS();
                    if (Rhs->SAs<Exprs::ConstExpression>()->GetConstValue() != "clear") {
                        Memory[update->GetLHS()] = Rhs;
                    }
                }
                ExpT path_condition = TheLTS->MakeTrue();
                for (auto it = GuardedCommands.begin(); it != GuardedCommands.end(); ++it) {
                    GCmdRef guarded_command = *it;
                    const vector<LTSAssignRef>& updates = guarded_command->GetUpdates();
                    const ExpT& guard = guarded_command->GetGuard();
                    auto Mgr = guard->GetMgr();
                    ExpT new_guard = Mgr->ApplyTransform<SubstitutorForWP>(guard, Memory);
                    MgrT::SubstMapT SubstMapForTransMsg = GetSubstitutionsForTransMsg(updates);
                    MgrT::SubstMapT SubstMapForTransition =
                        TransitionSubstitutionsGivenTransMsg(updates, SubstMapForTransMsg);
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
                    Mgr = path_condition->GetMgr();
                    path_condition = Mgr->MakeExpr(LTSOps::OpAND, new_guard, path_condition);
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
