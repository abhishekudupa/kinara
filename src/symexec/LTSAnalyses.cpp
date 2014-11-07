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
#include "../uflts/LTSUtils.hpp"
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
            auto Mgr = InitialCondition->GetMgr();
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
                const vector<LTSAssignRef>& updates = GuardedCommand->GetLoweredUpdates();

                MgrT::SubstMapT SubstMapForTransition;
                for (auto it = updates.rbegin();
                     it != updates.rend(); ++it) {
                    auto it2 = SubstMapForTransition.find((*it)->GetLHS());
                    if (it2 != SubstMapForTransition.end()) {
                        MgrT::SubstMapT LocalSubst;
                        LocalSubst[(*it)->GetLHS()] = (*it)->GetRHS();
                        auto NewSubst = Mgr->TermSubstitute(LocalSubst, it2->second);
                        SubstMapForTransition[it2->first] = NewSubst;
                    } else {
                        SubstMapForTransition[(*it)->GetLHS()] = (*it)->GetRHS();
                    }
                }
                const ExpT& Guard = GuardedCommand->GetLoweredGuard();
                ExpT ProductGuard = TheLTS->MakeOp(LTSOps::OpAND,
                                                   Guard,
                                                   it->second);
                Phi = Mgr->TermSubstitute(SubstMapForTransition, Phi);
                Phi = Mgr->MakeExpr(LTSOps::OpIMPLIES, ProductGuard, Phi);
                Phi = Mgr->Simplify(Phi);
            }

            MgrT::SubstMapT StemSortMap;
            auto StemSortAssignments = Trace->GetStemSortPermutation();
            for (auto SortAssignment : StemSortAssignments) {
                StemSortMap[SortAssignment->GetLHS()] = SortAssignment->GetRHS();
            }
            Phi = Mgr->TermSubstitute(StemSortMap, Phi);

            for (auto it = StemGuardedCommandsAndMonitorGuards.rbegin();
                 it != StemGuardedCommandsAndMonitorGuards.rend();
                 ++it) {
                GCmdRef GuardedCommand = it->first;
                const vector<LTSAssignRef>& updates = GuardedCommand->GetLoweredUpdates();

                MgrT::SubstMapT SubstMapForTransition;
                for (auto it = updates.rbegin();
                     it != updates.rend(); ++it) {
                    auto it2 = SubstMapForTransition.find((*it)->GetLHS());
                    if (it2 != SubstMapForTransition.end()) {
                        MgrT::SubstMapT LocalSubst;
                        LocalSubst[(*it)->GetLHS()] = (*it)->GetRHS();
                        auto NewSubst = Mgr->TermSubstitute(LocalSubst, it2->second);
                        SubstMapForTransition[it2->first] = NewSubst;
                    } else {
                        SubstMapForTransition[(*it)->GetLHS()] = (*it)->GetRHS();
                    }
                }

                const ExpT& Guard = GuardedCommand->GetLoweredGuard();
                ExpT ProductGuard = TheLTS->MakeOp(LTSOps::OpAND,
                                                   Guard,
                                                   it->second);
                Phi = Mgr->MakeExpr(LTSOps::OpIMPLIES, ProductGuard, Phi);
                Phi = Mgr->Simplify(Phi);

            }

            return Mgr->TermSubstitute(InitialStateSubstMap, Phi);
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

            return MakeDisjunction(EnableConditions, TheLTS->GetMgr());
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

        FastExpSetT
        TraceAnalyses::WeakestPrecondition(Solver* TheSolver,
                                           SafetyViolation* Trace,
                                           ExpT InitialPredicate)
        {
            auto TheLTS = TheSolver->TheLTS;
            ExpT Phi = InitialPredicate;
            auto Mgr = Phi->GetMgr();
            const vector<TraceElemT>& TraceElements = Trace->GetTraceElems();
            for (auto TraceIterator = TraceElements.rbegin();
                 TraceIterator != TraceElements.rend(); ++TraceIterator) {
                auto const& Cmd = TraceIterator->first;
                const vector<LTSAssignRef>& Updates = Cmd->GetLoweredUpdates();

                MgrT::SubstMapT SubstMapForTransition;

                for (auto UpdateIterator = Updates.rbegin();
                     UpdateIterator != Updates.rend(); ++UpdateIterator) {

                    auto it = SubstMapForTransition.find((*UpdateIterator)->GetLHS());

                    if (it != SubstMapForTransition.end()) {
                        MgrT::SubstMapT LocalSubst;
                        LocalSubst[(*UpdateIterator)->GetLHS()] = (*UpdateIterator)->GetRHS();
                        auto NewSubst = Mgr->TermSubstitute(LocalSubst, it->second);
                        SubstMapForTransition[it->first] = NewSubst;
                    } else {
                        SubstMapForTransition[(*UpdateIterator)->GetLHS()] =
                            (*UpdateIterator)->GetRHS();
                    }
                }

                auto const& Guard = Cmd->GetLoweredGuard();

                Phi = Mgr->TermSubstitute(SubstMapForTransition, Phi);
                Phi = Mgr->MakeExpr(LTSOps::OpIMPLIES, Guard, Phi);
                Phi = Mgr->Simplify(Phi);
            }

            FastExpSetT Retval;
            auto InitStateGenerators = TheLTS->GetInitStateGenerators();

            for (auto const& InitState : InitStateGenerators) {
                MgrT::SubstMapT InitStateSubstMap;
                for (auto const& Update : InitState) {
                    auto LHS = Update->GetLHS();
                    auto RHS = Update->GetRHS();
                    InitStateSubstMap[LHS] = RHS;
                }
                auto NewPhi = Mgr->TermSubstitute(InitStateSubstMap, Phi);
                NewPhi = Mgr->Simplify(NewPhi);
                Retval.insert(NewPhi);
            }
            return Retval;
        }

        ExpT TraceAnalyses::WeakestPreconditionForLiveness(Solver* TheSolver,
                                                           StateBuchiAutomaton* Monitor,
                                                           const LivenessViolation* Trace)
        {
            auto TheLTS = TheSolver->TheLTS;
            auto Mgr = TheLTS->GetMgr();
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
                auto VarDifferentLoopValue =
                    TheLTS->MakeOp(LTSOps::OpNOT,
                                   TheLTS->MakeOp(LTSOps::OpEQ, Pair.first, Pair.second));
                InitialCondition = TheLTS->MakeOp(LTSOps::OpOR,
                                                  VarDifferentLoopValue,
                                                  InitialCondition);
            }

            MgrT::SubstMapT LoopSortMap;
            auto LoopSortAssignments = Trace->GetLoopSortPermutation();
            for (auto SortAssignment : LoopSortAssignments) {
                LoopSortMap[SortAssignment->GetLHS()] = SortAssignment->GetRHS();
            }
            InitialCondition = Mgr->TermSubstitute(LoopSortMap, InitialCondition);

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

                cout << "checking guarded command in loop" << endl;
                cout << GuardedCommand->ToString() << endl;
                const vector<LTSAssignRef>& Updates = GuardedCommand->GetLoweredUpdates();

                MgrT::SubstMapT SubstMapForTransition;
                for (auto it = Updates.rbegin();
                     it != Updates.rend(); ++it) {
                    auto it2 = SubstMapForTransition.find((*it)->GetLHS());
                    if (it2 != SubstMapForTransition.end()) {
                        MgrT::SubstMapT LocalSubst;
                        LocalSubst[(*it)->GetLHS()] = (*it)->GetRHS();
                        auto NewSubst = Mgr->TermSubstitute(LocalSubst, it2->second);
                        SubstMapForTransition[it2->first] = NewSubst;
                    } else {
                        SubstMapForTransition[(*it)->GetLHS()] = (*it)->GetRHS();
                    }
                }
                ExpT Guard = GuardedCommand->GetLoweredGuard();
                ExpT ProductGuard = TheLTS->MakeOp(LTSOps::OpAND,
                                                   Guard,
                                                   it->second);
                Phi = Mgr->TermSubstitute(SubstMapForTransition, Phi);
                Phi = Mgr->MakeExpr(LTSOps::OpIMPLIES, ProductGuard, Phi);
                Phi = Mgr->Simplify(Phi);
            }
            Phi = Mgr->TermSubstitute(InvertLoopValues, Phi);

            MgrT::SubstMapT StemSortMap;
            auto StemSortAssignments = Trace->GetStemSortPermutation();
            for (auto SortAssignment : StemSortAssignments) {
                StemSortMap[SortAssignment->GetLHS()] = SortAssignment->GetRHS();
            }
            Phi = Mgr->TermSubstitute(StemSortMap, Phi);

            for (auto it = StemGuardedCommandsAndMonitorGuards.rbegin();
                 it != StemGuardedCommandsAndMonitorGuards.rend();
                 ++it) {
                GCmdRef GuardedCommand = it->first;
                cout << "checking guarded command in stem" << endl;
                cout << GuardedCommand->ToString() << endl;

                const vector<LTSAssignRef>& Updates = GuardedCommand->GetLoweredUpdates();
                MgrT::SubstMapT SubstMapForTransition;
                for (auto it = Updates.rbegin();
                     it != Updates.rend(); ++it) {
                    auto it2 = SubstMapForTransition.find((*it)->GetLHS());
                    if (it2 != SubstMapForTransition.end()) {
                        MgrT::SubstMapT LocalSubst;
                        LocalSubst[(*it)->GetLHS()] = (*it)->GetRHS();
                        auto NewSubst = Mgr->TermSubstitute(LocalSubst, it2->second);
                        SubstMapForTransition[it2->first] = NewSubst;
                    } else {
                        SubstMapForTransition[(*it)->GetLHS()] = (*it)->GetRHS();
                    }
                }
                ExpT Guard = GuardedCommand->GetLoweredGuard();
                ExpT ProductGuard = TheLTS->MakeOp(LTSOps::OpAND,
                                              Guard,
                                              it->second);
                Phi = Mgr->TermSubstitute(SubstMapForTransition, Phi);
                Phi = Mgr->MakeExpr(LTSOps::OpIMPLIES, ProductGuard, Phi);
                Phi = Mgr->Simplify(Phi);
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
                auto NewPhi = Mgr->TermSubstitute(InitStateSubstMap, Phi);
                NewPhi = Mgr->Simplify(NewPhi);
                if (NewPhi == Mgr->MakeFalse()) {
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
            }

            return MakeConjunction(Conjuncts, TheLTS->GetMgr());
        }

    }
}

//
// LTSAnalyses.cpp ends here
