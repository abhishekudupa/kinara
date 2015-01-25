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
#include "../uflts/LTSDecls.hpp"
#include "../uflts/LTSTransitions.hpp"
#include "../uflts/LTSUtils.hpp"
#include "../mc/AQStructure.hpp"
#include "../mc/Compiler.hpp"
#include "../mc/OmegaAutomaton.hpp"
#include "../mc/Trace.hpp"
#include "../synth/Solver.hpp"
#include "../tpinterface/TheoremProver.hpp"
#include "../uflts/LTSFairnessSet.hpp"

namespace ESMC {
    namespace Analyses {
        using namespace LTS;
        using namespace MC;
        using namespace Synth;
        using namespace TP;

        vector<ExpT> TraceAnalyses::GetAllScalarLeaves(const ExpT& InitialExp) {
            vector<ExpT> Retval;
            vector<ExpT> ExpressionsToCheck = {InitialExp};
            while (ExpressionsToCheck.size() > 0) {
                auto Exp = ExpressionsToCheck.back();
                ExpressionsToCheck.pop_back();
                auto ExpType = Exp->GetType();
                if (ExpType->Is<ScalarType>()) {
                    Retval.push_back(Exp);
                } else if (ExpType->Is<ArrayType>()) {
                    auto ExpTypeAsArrayType = ExpType->As<ArrayType>();
                    auto IndexType = ExpTypeAsArrayType->GetIndexType();
                    for (auto Element: IndexType->GetElementsNoUndef()) {
                        auto ArrayElement =
                            Exp->GetMgr()->MakeExpr(LTSOps::OpIndex,
                                                    Exp,
                                                    Exp->GetMgr()->MakeVal(Element, IndexType));
                        ExpressionsToCheck.push_back(ArrayElement);
                    }
                } else if (ExpType->Is<RecordType>()) {
                    auto ExpTypeAsRecordType = ExpType->As<RecordType>();
                    auto FAType = Exp->GetMgr()->MakeType<FieldAccessType>();
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

        FairObjSetT TraceAnalyses::GetLTSFairnessObjects(LTS::LabelledTS* TheLTS)
        {
            FairObjSetT Retval;

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
            //     FairObjSetT ActualFairnesses;
            //     for (auto const& StrFairness : StrFairnesses) {
            //         auto const& ActFairness =
            //             AllFairnesses->GetFairnessObj(StrFairness, ParamInst);
            //         FairnessObjectsInLTS.insert(ActFairness);
            //     }
            // }

            // method 3
            for (auto GuardedCommand: TheLTS->GetGuardedCmds()) {
                for (auto FairnessObject: GuardedCommand->GetFairnessObjsSatisfied()) {
                    Retval.insert(FairnessObject);
                }
            }
            return Retval;
        }

        FairObjSetT
        TraceAnalyses::GetLoopFairnessObjects(LabelledTS* TheLTS,
                                              const LivenessViolation* LivenessViolation)
        {
            FairObjSetT Retval;
            for (auto TraceElement: LivenessViolation->GetLoop()) {
                auto GuardedCommand = TraceElement.first;
                auto const& FairObjects = GuardedCommand->GetFairnessObjsSatisfied();
                for (auto FairObject: FairObjects) {
                    Retval.insert(FairObject);
                }
            }
            return Retval;
        }

        FairObjSetT
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

        ExpT
        TraceAnalyses::WeakestPreconditionWithMonitor(LabelledTS* TheLTS,
                                                      StateBuchiAutomaton* Monitor,
                                                      const MgrT::SubstMapT& InitialStateSubstMap,
                                                      const LivenessViolation* Trace,
                                                      const ExpT& InitialCondition,
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
                                                   const MgrT::SubstMapT& InitialStateSubstMap,
                                                   const LivenessViolation* LivenessViolation,
                                                   const FairObjSetT& FairnessObjects)
        {
            vector<ExpT> EnableConditions;
            auto Loop = LivenessViolation->GetLoop();
            for (auto FairnessObject : FairnessObjects) {
                u16 LoopIndex = 0;
                for (PSTraceElemT TraceElement : Loop) {
                    // Check if there is any guarded command that conains
                    // the fairness object.
                    for (auto Cmd : TheLTS->GetGuardedCmds()) {
                        auto const& GCmdFairObjs = Cmd->GetFairnessObjsSatisfied();
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
                    auto EFSMDotStateAsEnum = EFSMDotState->GetType()->As<EnumType>();
                    string StateName = EFSMDotStateAsEnum->ValToConst(StateValue);
                    auto State = TheLTS->MakeVal(StateName, EFSMDotState->GetType());
                    auto StateCondition = TheLTS->MakeOp(LTSOps::OpEQ, EFSMDotState, State);
                    Retval = TheLTS->MakeOp(LTSOps::OpAND, Retval,
                                            StateCondition);
                }
            }
            return Retval;
        }

        vector<ExpT>
        TraceAnalyses::WeakestPrecondition(Solver* TheSolver,
                                           SafetyViolation* Trace,
                                           const ExpT& InitialPredicate)
        {
            auto TheLTS = TheSolver->TheLTS;
            ExpT Phi = InitialPredicate;
            auto Mgr = Phi->GetMgr();
            const vector<TraceElemT>& TraceElements = Trace->GetTraceElems();
            for (auto TraceIterator = TraceElements.rbegin();
                 TraceIterator != TraceElements.rend(); ++TraceIterator) {
                auto const& Cmd = TraceIterator->first;
                const vector<LTSAssignRef>& LUpdates = Cmd->GetLoweredUpdates();

                MgrT::SubstMapT SubstMapForTransition;

                // Assumption: There's only one update
                // to each variable in a transition
                for (auto UpdateIterator = LUpdates.begin();
                     UpdateIterator != LUpdates.end(); ++UpdateIterator) {
                    auto const& Update = *UpdateIterator;
                    auto const& LHS = Update->GetLHS();
                    auto const& RHS = Update->GetRHS();
                    SubstMapForTransition[LHS] = RHS;
                }

                // Get the NEPreds for each of the updates
                auto const& Updates = Cmd->GetUpdates();
                const u32 NumUpdates = Updates.size();
                vector<ExpT> UpdateNEPreds(NumUpdates);

                for (u32 i = 0; i < NumUpdates; ++i) {
                    auto const& Update = Updates[i];
                    auto LHSNEPred =
                        Update->GetLHS()->ExtensionData.Interp->GetNoExceptionPredicate();
                    auto RHSNEPred =
                        Update->GetRHS()->ExtensionData.Interp->GetNoExceptionPredicate();
                    auto BoundsConstraint = Update->GetLoweredBoundsConstraint();
                    UpdateNEPreds[i] = Mgr->MakeExpr(LTSOps::OpAND, LHSNEPred,
                                                     RHSNEPred, BoundsConstraint);
                }

                auto UpdateNEPred = MakeConjunction(UpdateNEPreds, Mgr);
                auto const& LGuard = Cmd->GetLoweredGuard();
                auto const& Guard = Cmd->GetGuard();

                Phi = Mgr->Substitute(SubstMapForTransition, Phi);
                auto NEGuard = Guard->ExtensionData.Interp->GetNoExceptionPredicate();
                auto Antecedent = Mgr->MakeExpr(LTSOps::OpAND, NEGuard, LGuard);
                auto Consequent = Mgr->MakeExpr(LTSOps::OpAND, UpdateNEPred, Phi);

                Phi = Mgr->MakeExpr(LTSOps::OpIMPLIES, Antecedent, Consequent);

                ESMC_LOG_FULL(
                              "Analyses.Detailed",
                              Out_ << "New Phi after propagating through command:" << endl
                                   << Cmd->ToString() << endl << Phi->ToString() << endl;
                              );

                Phi = Mgr->SimplifyFP(Phi);

                ESMC_LOG_SHORT(
                              "Analyses.Detailed",
                              Out_ << "Simplified:" << endl << Phi->ToString() << endl;
                              );
            }

            vector<ExpT> Retval;
            auto InitStateGenerators = TheLTS->GetInitStateGenerators();

            for (auto const& InitState : InitStateGenerators) {
                MgrT::SubstMapT InitStateSubstMap;
                for (auto const& Update : InitState->GetLoweredUpdates()) {
                    auto LHS = Update->GetLHS();
                    auto RHS = Update->GetRHS();
                    InitStateSubstMap[LHS] = RHS;
                }
                auto NewPhi = Mgr->Substitute(InitStateSubstMap, Phi);

                ESMC_LOG_FULL(
                              "Analyses.Detailed",
                              Out_ << "Substituting initial state values, initial state "
                                   << "updates:" << endl;
                              for (auto const& Update : InitState->GetLoweredUpdates()) {
                                  Out_ << Update << endl;
                              });


                ESMC_LOG_SHORT(
                               "Analyses.Detailed",
                               Out_ << "After substituting initial state values:" << endl
                               << NewPhi->ToString() << endl;
                               );

                NewPhi = Mgr->SimplifyFP(NewPhi);

                ESMC_LOG_SHORT(
                               "Analyses.Detailed",
                               Out_ << "Simplified:" << endl << NewPhi->ToString() << endl;
                               );

                if (NewPhi != Mgr->MakeTrue()) {
                    Retval.push_back(NewPhi);
                }
            }
            return Retval;
        }

        FastExpSetT
        TraceAnalyses::WeakestPreconditionForLiveness(Solver* TheSolver,
                                                      StateBuchiAutomaton* Monitor,
                                                      const LivenessViolation* Trace)
        {
            auto TheLTS = TheSolver->TheLTS;
            auto Mgr = TheLTS->GetMgr();
            auto InitStateGenerators = TheLTS->GetInitStateGenerators();
            MgrT::SubstMapT LoopValues;
            for (auto StateVariable: TheLTS->GetStateVectorVars()) {
                for (auto Exp: GetAllScalarLeaves(StateVariable)) {
                    auto LoweredExp = Mgr->ApplyTransform<LTS::Detail::ArrayRValueTransformer>(Exp);
                    LoweredExp = Mgr->SimplifyFP(LoweredExp);
                    LoopValues[LoweredExp] = TheLTS->MakeVar(Exp->ToString() + "_0", Exp->GetType());
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
            InitialCondition = Mgr->Substitute(LoopSortMap, InitialCondition);

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
                auto LoweredMonitorGuard = Mgr->ApplyTransform<LTS::Detail::ArrayRValueTransformer>(MonitorGuard);
                LoweredMonitorGuard = Mgr->SimplifyFP(LoweredMonitorGuard);
                auto Pair = make_pair(GuardedCommand, LoweredMonitorGuard);
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
                auto LoweredMonitorGuard = Mgr->ApplyTransform<LTS::Detail::ArrayRValueTransformer>(MonitorGuard);
                LoweredMonitorGuard = Mgr->SimplifyFP(LoweredMonitorGuard);
                auto Pair = make_pair(GuardedCommand, LoweredMonitorGuard);
                StemGuardedCommandsAndMonitorGuards.push_back(Pair);
                PreviousMonitorState = ProductState->GetMonitorState();
            }

            auto Phi = InitialCondition;

            for (auto it = LoopGuardedCommandsAndMonitorGuards.rbegin();
                 it != LoopGuardedCommandsAndMonitorGuards.rend();
                 ++it) {
                GCmdRef GuardedCommand = it->first;

                const vector<LTSAssignRef>& Updates = GuardedCommand->GetLoweredUpdates();

                MgrT::SubstMapT SubstMapForTransition;
                for (auto it = Updates.rbegin();
                     it != Updates.rend(); ++it) {
                    auto it2 = SubstMapForTransition.find((*it)->GetLHS());
                    if (it2 != SubstMapForTransition.end()) {
                        MgrT::SubstMapT LocalSubst;
                        LocalSubst[(*it)->GetLHS()] = (*it)->GetRHS();
                        auto NewSubst = Mgr->Substitute(LocalSubst, it2->second);
                        SubstMapForTransition[it2->first] = NewSubst;
                    } else {
                        SubstMapForTransition[(*it)->GetLHS()] = (*it)->GetRHS();
                    }
                }
                ExpT Guard = GuardedCommand->GetLoweredGuard();
                ExpT ProductGuard = TheLTS->MakeOp(LTSOps::OpAND,
                                                   Guard,
                                                   it->second);
                Phi = Mgr->Substitute(SubstMapForTransition, Phi);
                Phi = Mgr->MakeExpr(LTSOps::OpIMPLIES, ProductGuard, Phi);
                Phi = Mgr->SimplifyFP(Phi);
            }
            Phi = Mgr->Substitute(InvertLoopValues, Phi);

            MgrT::SubstMapT StemSortMap;
            auto StemSortAssignments = Trace->GetStemSortPermutation();
            for (auto SortAssignment : StemSortAssignments) {
                StemSortMap[SortAssignment->GetLHS()] = SortAssignment->GetRHS();
            }
            Phi = Mgr->Substitute(StemSortMap, Phi);

            for (auto it = StemGuardedCommandsAndMonitorGuards.rbegin();
                 it != StemGuardedCommandsAndMonitorGuards.rend();
                 ++it) {
                GCmdRef GuardedCommand = it->first;

                const vector<LTSAssignRef>& Updates = GuardedCommand->GetLoweredUpdates();
                MgrT::SubstMapT SubstMapForTransition;
                for (auto it = Updates.rbegin();
                     it != Updates.rend(); ++it) {
                    auto it2 = SubstMapForTransition.find((*it)->GetLHS());
                    if (it2 != SubstMapForTransition.end()) {
                        MgrT::SubstMapT LocalSubst;
                        LocalSubst[(*it)->GetLHS()] = (*it)->GetRHS();
                        auto NewSubst = Mgr->Substitute(LocalSubst, it2->second);
                        SubstMapForTransition[it2->first] = NewSubst;
                    } else {
                        SubstMapForTransition[(*it)->GetLHS()] = (*it)->GetRHS();
                    }
                }
                ExpT Guard = GuardedCommand->GetLoweredGuard();
                ExpT ProductGuard = TheLTS->MakeOp(LTSOps::OpAND,
                                                   Guard,
                                                   it->second);
                Phi = Mgr->Substitute(SubstMapForTransition, Phi);
                Phi = Mgr->MakeExpr(LTSOps::OpIMPLIES, ProductGuard, Phi);
                Phi = Mgr->SimplifyFP(Phi);
            }

            auto TrivialFairObjs =
                TriviallySatisfiedFairnessObjectsInLoop(TheLTS, Trace);

            vector<ExpT> Conjuncts;

            FastExpSetT Retval;

            for (auto const& InitState : InitStateGenerators) {
                MgrT::SubstMapT InitStateSubstMap;
                for (auto const& Update : InitState->GetLoweredUpdates()) {
                    auto LHS = Update->GetLHS();
                    auto RHS = Update->GetRHS();
                    InitStateSubstMap[LHS] = RHS;
                }
                auto NewPhi = Mgr->Substitute(InitStateSubstMap, Phi);
                NewPhi = Mgr->SimplifyFP(NewPhi);
                if (NewPhi != TheLTS->MakeTrue()) {
                    Retval.insert(NewPhi);
                }
            }
            return Retval;
        }
    }
}

//
// LTSAnalyses.cpp ends here
