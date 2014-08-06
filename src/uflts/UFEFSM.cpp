// UFEFSM.cpp --- 
// 
// Filename: UFEFSM.cpp
// Author: Abhishek Udupa
// Created: Mon Jul 28 23:49:54 2014 (-0400)
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

#include "UFEFSM.hpp"
#include "UFLTS.hpp"
#include "LTSUtils.hpp"
#include "FrozenEFSM.hpp"

#include <type_traits>

namespace ESMC {
    namespace LTS {

        UFEFSM::UFEFSM(UFLTS* TheLTS, const string& Name)
            : TheLTS(TheLTS), Name(Name), StatesFrozen(false)
        {
            // Nothing here
        }

        // Implementation of UFEFSM
        UFEFSM::UFEFSM(UFLTS* TheLTS,
                       const string& Name,
                       const vector<ExpT>& Params,
                       const ExpT& Constraint)
            : TheLTS(TheLTS), Name(Name), Params(Params),
              Constraint(Constraint), StatesFrozen(false)
        {
            CheckParams(Params, Constraint, SymTab, TheLTS->GetMgr());
        }

        UFEFSM::~UFEFSM()
        {
            // Nothing here
        }

        const string& UFEFSM::GetName() const
        {
            return Name;
        }

        void UFEFSM::AddState(const string& StateName,
                              bool Initial,
                              bool Final,
                              bool Accepting,
                              bool Error,
                              bool Dead)
        {
            if (StatesFrozen) {
                throw ESMCError((string)"Cannot add states to EFSM after freezing states");
            }
            Detail::StateDescriptor Desc(StateName, Initial, Final, Accepting, Error, Dead);
            auto it = States.find(StateName);
            if (it == States.end()) {
                States[StateName] = Desc;
            } else if (it->second != Desc) {
                throw ESMCError((string)"Redeclaration of state with different parameters");
            }
        }

        void UFEFSM::FreezeStates()
        {
            if (StatesFrozen) {
                return;
            }
            StatesFrozen = true;
            // Create the state type
            auto Mgr = TheLTS->GetMgr();
            set<string> StateNames;
            for (auto const& StateD : States) {
                StateNames.insert(StateD.first);
            }
            StateType = Mgr->MakeType<Exprs::ExprEnumType>(Name + "StateT", StateNames);

            if (Params.size() == 0) {
                ParamInsts.push_back(vector<ExpT>());
                ParamSubsts.push_back(MgrType::SubstMapT());
                FrozenEFSMs.push_back(new FrozenEFSM(Name, TheLTS, Params, this, 
                                                     StateType, States));
                return;
            }
            
            // Instantiate each of the parameters
            ParamInsts = InstantiateParams(Params, Constraint, Mgr);
            
            for (auto const& ParamVal : ParamInsts) {

                MgrType::SubstMapT SubstMap;

                const u32 NumParams = Params.size();

                for (u32 i = 0; i < NumParams; ++i) {
                    SubstMap[Params[i]] = ParamVal[i];
                }
                
                ParamSubsts.push_back(SubstMap);
                FrozenEFSMs.push_back(new FrozenEFSM(Name, TheLTS, ParamVal, this, 
                                                     StateType, States));
            }
        }


        inline void UFEFSM::AddMsg(const ExprTypeRef& MType,
                                   const vector<ExpT>& MParams,
                                   bool IsInput)
        {
            if (!StatesFrozen) {
                throw ESMCError((string)"Cannot add messages until states are frozen");
            }
            if (AllFrozen) {
                throw ESMCError((string)"Cannot add message to EFSM after it has been frozen");
            }

            CheckParams(MParams, SymTab);

            auto Mgr = TheLTS->GetMgr();
            const u32 NumInsts = ParamInsts.size();

            for (u32 i = 0; i < NumInsts; ++i) {
                auto&& SubstParams = SubstAll(MParams, ParamSubsts[i], Mgr);
                auto Type = InstantiateType(MType, SubstParams, Mgr);
                
                if (IsInput) {
                    FrozenEFSMs[i]->AddInputMsg(Type);
                } else {
                    FrozenEFSMs[i]->AddOutputMsg(Type);
                }
            }
        }

        inline void UFEFSM::AddMsgs(const vector<ExpT>& NewParams,
                                    const ExprTypeRef& MType, 
                                    const vector<ExpT>& MParams, 
                                    const ExpT& Constraint, 
                                    bool IsInput)
        {
            if (!StatesFrozen) {
                throw ESMCError((string)"Cannot add messages until states are frozen");
            }

            if (AllFrozen) {
                throw ESMCError((string)"Cannot add message to EFSM after it has been frozen");
            }

            auto Mgr = TheLTS->GetMgr();

            SymTab.Push();
            CheckParams(NewParams, Constraint, SymTab, Mgr, true);
            CheckParams(MParams, SymTab);
            SymTab.Pop();

            const u32 NumInsts = ParamInsts.size();
            const u32 NumNewParams = NewParams.size();

            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                auto SubstConstraint = Mgr->Substitute(SubstMap, Constraint);

                auto NewInsts = InstantiateParams(NewParams, SubstConstraint, Mgr);
                const u32 NumNewInsts = NewInsts.size();

                for (u32 j = 0; j < NumNewInsts; ++j) {
                    auto LocalSubstMap = SubstMap;
                    auto const& CurNewInst = NewInsts[j];

                    for (u32 k = 0; k < NumNewParams; ++k) {
                        LocalSubstMap[NewParams[k]] = CurNewInst[k];
                    }

                    auto&& SubstMsgParams = SubstAll(MParams, LocalSubstMap, Mgr);
                    auto Type = InstantiateType(MType, SubstMsgParams, Mgr);
                    if (IsInput) {
                        FrozenEFSMs[i]->AddInputMsg(Type);
                    } else {
                        FrozenEFSMs[i]->AddOutputMsg(Type);
                    }
                }
            }
        }

        inline vector<AsgnT> UFEFSM::InstantiateParametricVars(const vector<AsgnT>& Updates,
                                                               const string& VarName,
                                                               const ExprTypeRef& ParametricType,
                                                               const ExprTypeRef& InstantiatedType)
        {
            auto Mgr = TheLTS->GetMgr();
            vector<AsgnT> Retval;

            for (auto const& Update : Updates) {
                auto const& LHS = Update.GetLHS();
                auto const& RHS = Update.GetRHS();

                MgrType::SubstMapT SubstMap;
                SubstMap[Mgr->MakeVar(VarName, ParametricType)] = 
                    Mgr->MakeVar(VarName, InstantiatedType);
                auto SubstLHS = Mgr->Substitute(SubstMap, LHS);
                auto SubstRHS = Mgr->Substitute(SubstMap, RHS);
                
                Retval.push_back(AsgnT(SubstLHS, SubstRHS));
            }
            return Retval;
        }

        void UFEFSM::AddInputMsg(const ExprTypeRef& MType, 
                                 const vector<ExpT>& MParams)
        {
            AddMsg(MType, MParams, true);
        }

        void UFEFSM::AddInputMsgs(const vector<ExpT>& NewParams,
                                  const ExprTypeRef& MType,
                                  const vector<ExpT>& MParams,
                                  const ExpT& MConstraint)
        {
            AddMsgs(NewParams, MType, MParams, MConstraint, true);
        }

        void UFEFSM::AddOutputMsg(const ExprTypeRef& MType,
                                  const vector<ExpT>& MParams)
        {
            AddMsg(MType, MParams, false);
        }

        void UFEFSM::AddOutputMsgs(const vector<ExpT>& NewParams,
                                   const ExprTypeRef& MType,
                                   const vector<ExpT>& MParams,
                                   const ExpT& MConstraint)
        {
            AddMsgs(NewParams, MType, MParams, MConstraint, false);
        }

        void UFEFSM::AddVariable(const string& VarName,
                                 const ExprTypeRef& VarType)
        {
            if (!StatesFrozen) {
                throw ESMCError((string)"Cannot add variables to EFSM until states " + 
                                "have been frozen");
            }
            if (AllFrozen) {
                throw ESMCError((string)"Cannot add variables to EFSM after it has been frozen");
            }

            if (!VarType->Is<Exprs::ExprScalarType>() &&
                !VarType->Is<Exprs::ExprRecordType> () &&
                !VarType->Is<Exprs::ExprArrayType> ()) {
                throw ESMCError((string)"Variable \"" + VarName + "\" has an " + 
                                "unsupported type. Only scalar types, record " + 
                                "types and array types can be EFSM variables");
            }
            if (SymTab.LookupTop(VarName) != DeclRef::NullPtr) {
                throw ESMCError((string)"Rebinding of variable \"" + VarName + "\"");
            }
            SymTab.Bind(VarName, new VarDecl(VarName, VarType));

            const u32 NumInsts = ParamInsts.size();
            for (u32 i = 0; i < NumInsts; ++i) {
                FrozenEFSMs[i]->AddVariable(VarName, VarType);
            }
        }

        u32 UFEFSM::GetNumExpansions(const vector<ExpT>& NewParams,
                                     const ExpT& Constraint)
        {
            auto Mgr = TheLTS->GetMgr();
            SymTab.Push();
            CheckParams(NewParams, Constraint, SymTab, Mgr, true);
            SymTab.Pop();
            
            const u32 NumInsts = ParamInsts.size();
            u32 Retval = 0;
            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                auto const& SubstConstraint = Mgr->Substitute(SubstMap, Constraint);
                auto&& NewInsts = InstantiateParams(NewParams, SubstConstraint, Mgr);
                Retval += NewInsts.size();;
            }
            return Retval;
        }
            
        void UFEFSM::AddInputTransition(const string& InitState,
                                        const string& FinalState,
                                        const ExpT& Guard,
                                        const vector<AsgnT>& Updates,
                                        const string& MessageName,
                                        const ExprTypeRef& MessageType,
                                        const vector<ExpT>& MessageParams)
        {
            if (!StatesFrozen) {
                throw ESMCError((string)"Cannot add transitions to EFSM until states " + 
                                "have been frozen");
            }
            if (AllFrozen) {
                throw ESMCError((string)"Cannot add transitions to EFSM after it has been frozen");
            }

            CheckParams(MessageParams, SymTab);

            auto Mgr = TheLTS->GetMgr();
            const u32 NumInsts = ParamInsts.size();
            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                auto&& SubstParams = SubstAll(MessageParams, SubstMap, Mgr);
                auto MType = InstantiateType(MessageType, SubstParams, Mgr);
                
                auto SubstGuard = Mgr->Substitute(SubstMap, Guard);
                const u32 NumUpdates = Updates.size();
                vector<AsgnT> SubstUpdates(NumUpdates);
                for (u32 j = 0; j < NumUpdates; ++j) {
                    SubstUpdates[i] = AsgnT(Mgr->Substitute(SubstMap, Updates[j].GetLHS()),
                                            Mgr->Substitute(SubstMap, Updates[j].GetRHS()));
                }

                SubstUpdates = InstantiateParametricVars(SubstUpdates, MessageName,
                                                         MessageType, MType);
                
                FrozenEFSMs[i]->AddInputTransition(InitState, FinalState, SubstGuard, SubstUpdates, 
                                                   MessageName, MType);
            }
        }

        void UFEFSM::AddInputTransitions(const string& InitState,
                                         const string& FinalState,
                                         const vector<ExpT> TransParams,
                                         const ExpT& TransConstraint,
                                         const ExpT& Guard,
                                         const vector<AsgnT>& Updates,
                                         const string& MessageName,
                                         const ExprTypeRef& MessageType,
                                         const vector<ExpT>& MessageParams)
        {
            if (!StatesFrozen) {
                throw ESMCError((string)"Cannot add transitions to EFSM until states " + 
                                "have been frozen");
            }
            if (AllFrozen) {
                throw ESMCError((string)"Cannot add transitions to EFSM after it has been frozen");
            }

            auto Mgr = TheLTS->GetMgr();
            
            // Push the transition params
            SymTab.Push();
            // The new params CANNOT reuse existing params
            CheckParams(TransParams, TransConstraint, SymTab, Mgr, true);
            // This binds the transition params
            CheckParams(MessageParams, SymTab);
            SymTab.Pop();

            const u32 NumInsts = ParamInsts.size();
            const u32 NumTransParams = TransParams.size();

            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                auto SubstTransConstraint = Mgr->Substitute(SubstMap, TransConstraint);
                
                auto&& TransInsts = InstantiateParams(TransParams, SubstTransConstraint, Mgr);
                const u32 NumTransInsts = TransInsts.size();

                for (u32 j = 0; j < NumTransInsts; ++j) {
                    auto LocalSubstMap = SubstMap;
                    auto const& CurTransInst = TransInsts[j];

                    for (u32 k = 0; k < NumTransParams; ++k) {
                        LocalSubstMap[TransParams[k]] = CurTransInst[k];
                    }

                    auto&& SubstMsgParams = SubstAll(MessageParams, LocalSubstMap, Mgr);
                    auto Type = InstantiateType(MessageType, SubstMsgParams, Mgr);

                    auto SubstGuard = Mgr->Substitute(LocalSubstMap, Guard);
                    const u32 NumUpdates = Updates.size();

                    vector<AsgnT> SubstUpdates(NumUpdates);
                    for (u32 k = 0; k < NumUpdates; ++k) {
                        SubstUpdates[k] = AsgnT(Mgr->Substitute(LocalSubstMap, Updates[k].GetLHS()),
                                                Mgr->Substitute(LocalSubstMap, Updates[k].GetRHS()));
                    }

                    SubstUpdates = InstantiateParametricVars(SubstUpdates, MessageName,
                                                             MessageType, Type);
                    
                    FrozenEFSMs[i]->AddInputTransition(InitState, FinalState, SubstGuard, 
                                                       SubstUpdates, MessageName, Type);
                }
            }
        }

        void UFEFSM::AddOutputTransition(const string& InitState,
                                         const string& FinalState,
                                         const ExpT& Guard,
                                         const vector<AsgnT>& Updates,
                                         const string& MessageName,
                                         const ExprTypeRef& MessageType,
                                         const vector<ExpT>& MessageParams,
                                         const unordered_set<u32>& FairnessSet)
        {
            if (!StatesFrozen) {
                throw ESMCError((string)"Cannot add transitions to EFSM until states " + 
                                "have been frozen");
            }
            if (AllFrozen) {
                throw ESMCError((string)"Cannot add transitions to EFSM after it has been frozen");
            }

            auto Mgr = TheLTS->GetMgr();
            CheckParams(MessageParams, SymTab);

            const u32 NumInsts = ParamInsts.size();
            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                auto&& SubstParams = SubstAll(MessageParams, SubstMap, Mgr);
                auto MType = InstantiateType(MessageType, SubstParams, Mgr);

                auto SubstGuard = Mgr->Substitute(SubstMap, Guard);
                const u32 NumUpdates = Updates.size();
                vector<AsgnT> SubstUpdates(NumUpdates);
                for (u32 j = 0; j < NumUpdates; ++j) {
                    SubstUpdates[j] = AsgnT(Mgr->Substitute(SubstMap, Updates[j].GetLHS()),
                                            Mgr->Substitute(SubstMap, Updates[j].GetRHS()));
                }
                
                SubstUpdates = InstantiateParametricVars(SubstUpdates, MessageName,
                                                         MessageType, MType);

                FrozenEFSMs[i]->AddOutputTransition(InitState, FinalState, SubstGuard, 
                                                    SubstUpdates, MessageName, MType, 
                                                    FairnessSet);
            }
        }

        void UFEFSM::AddOutputTransitions(const string& InitState,
                                          const string& FinalState,
                                          const vector<ExpT>& TransParams,
                                          const ExpT& TransConstraint,
                                          const ExpT& Guard,
                                          const vector<AsgnT>& Updates,
                                          const string& MessageName,
                                          const ExprTypeRef& MessageType,
                                          const vector<ExpT>& MessageParams,
                                          const vector<unordered_set<u32>>& FairnessSets)
        {
            if (!StatesFrozen) {
                throw ESMCError((string)"Cannot add transitions to EFSM until states " + 
                                "have been frozen");
            }
            if (AllFrozen) {
                throw ESMCError((string)"Cannot add transitions to EFSM after it has been frozen");
            }

            auto Mgr = TheLTS->GetMgr();
            
            SymTab.Push();
            CheckParams(TransParams, TransConstraint, SymTab, Mgr, true);
            CheckParams(MessageParams, SymTab);
            SymTab.Pop();

            const u32 NumInsts = ParamInsts.size();
            const u32 NumTransParams = TransParams.size();

            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                auto SubstTransConstraint = Mgr->Substitute(SubstMap, TransConstraint);

                auto&& TransInsts = InstantiateParams(TransParams, SubstTransConstraint, Mgr);
                const u32 NumTransInsts = TransInsts.size();

                vector<unordered_set<u32>> ActFairnessSets;
                if (FairnessSets.size() == 0) {
                    ActFairnessSets = vector<unordered_set<u32>>(NumTransInsts);
                } else if (FairnessSets.size() == 1) {
                    ActFairnessSets = vector<unordered_set<u32>>(NumTransInsts, FairnessSets[0]);
                } else if (FairnessSets.size() == NumTransInsts) {
                    ActFairnessSets = FairnessSets;
                } else {
                    throw ESMCError((string)"Number of fairness sets does not match " + 
                                    "number of transitions that the parametrized " + 
                                    "transition expands to");
                }
                

                for (u32 j = 0; j < NumTransInsts; ++j) {
                    auto LocalSubstMap = SubstMap;
                    auto const& CurTransInst = TransInsts[j];

                    for (u32 k = 0; k < NumTransParams; ++k) {
                        LocalSubstMap[TransParams[k]] = CurTransInst[k];
                    }

                    auto&& SubstMsgParams = SubstAll(MessageParams, LocalSubstMap, Mgr);
                    auto Type = InstantiateType(MessageType, SubstMsgParams, Mgr);

                    auto SubstGuard = Mgr->Substitute(LocalSubstMap, Guard);
                    const u32 NumUpdates = Updates.size();

                    vector<AsgnT> SubstUpdates(NumUpdates);
                    for (u32 k = 0; k < NumUpdates; ++k) {
                        SubstUpdates[k] = AsgnT(Mgr->Substitute(LocalSubstMap, Updates[k].GetLHS()),
                                                Mgr->Substitute(LocalSubstMap, Updates[k].GetRHS()));
                    }

                    SubstUpdates = InstantiateParametricVars(SubstUpdates, MessageName,
                                                             MessageType, Type);
                    
                    FrozenEFSMs[i]->AddOutputTransition(InitState, FinalState, SubstGuard, 
                                                        SubstUpdates, MessageName, Type, 
                                                        ActFairnessSets[i]);
                }
            }
        }


        void UFEFSM::AddInternalTransition(const string& InitState,
                                           const string& FinalState,
                                           const ExpT& Guard,
                                           const vector<AsgnT>& Updates,
                                           const unordered_set<u32>& FairnessSet)
        {
            if (!StatesFrozen) {
                throw ESMCError((string)"Cannot add transitions to EFSM until states " + 
                                "have been frozen");
            }
            if (AllFrozen) {
                throw ESMCError((string)"Cannot add transitions to EFSM after it has been frozen");
            }

            auto Mgr = TheLTS->GetMgr();
            const u32 NumInsts = ParamInsts.size();

            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                auto SubstGuard = Mgr->Substitute(SubstMap, Guard);
                const u32 NumUpdates = Updates.size();
                vector<AsgnT> SubstUpdates(NumUpdates);
                
                for (u32 j = 0; j < NumUpdates; ++j) {
                    SubstUpdates[j] = AsgnT(Mgr->Substitute(SubstMap, Updates[j].GetLHS()),
                                            Mgr->Substitute(SubstMap, Updates[j].GetRHS()));
                }
                FrozenEFSMs[i]->AddInternalTransition(InitState, FinalState, 
                                                      SubstGuard, SubstUpdates, 
                                                      FairnessSet);
            }
        }

        void UFEFSM::AddInternalTransitions(const string& InitState,
                                            const string& FinalState,
                                            const vector<ExpT>& TransParams,
                                            const ExpT& TransConstraint,
                                            const ExpT& Guard,
                                            const vector<AsgnT>& Updates,
                                            const vector<unordered_set<u32>>& FairnessSets)
        {
            if (!StatesFrozen) {
                throw ESMCError((string)"Cannot add transitions to EFSM until states " + 
                                "have been frozen");
            }
            if (AllFrozen) {
                throw ESMCError((string)"Cannot add transitions to EFSM after it has been frozen");
            }

            auto Mgr = TheLTS->GetMgr();

            SymTab.Push();
            CheckParams(TransParams, TransConstraint, SymTab, Mgr, true);
            SymTab.Pop();

            const u32 NumInsts = ParamInsts.size();
            const u32 NumTransParams = TransParams.size();

            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                auto SubstTransConstraint = Mgr->Substitute(SubstMap, TransConstraint);
                
                auto const&& TransInsts = InstantiateParams(TransParams, SubstTransConstraint, Mgr);
                const u32 NumTransInsts = TransInsts.size();

                vector<unordered_set<u32>> ActFairnessSets;
                if (FairnessSets.size() == 0) {
                    ActFairnessSets = vector<unordered_set<u32>>(NumTransInsts);
                } else if (ActFairnessSets.size() == 1) {
                    ActFairnessSets = vector<unordered_set<u32>>(NumTransInsts, FairnessSets[0]);
                } else if (ActFairnessSets.size() == NumTransInsts) {
                    ActFairnessSets = FairnessSets;
                } else {
                    throw ESMCError((string)"Number of fairness sets does not match " + 
                                    "number of transitions that the parametrized " + 
                                    "transition expands to");
                }
                
                for (u32 j = 0; j < NumTransInsts; ++j) {
                    auto LocalSubstMap = SubstMap;
                    auto const& CurTransInst = TransInsts[j];

                    for (u32 k = 0; k < NumTransParams; ++k) {
                        LocalSubstMap[TransParams[k]] = CurTransInst[k];
                    }

                    auto SubstGuard = Mgr->Substitute(LocalSubstMap, Guard);
                    const u32 NumUpdates = Updates.size();
                    vector<AsgnT> SubstUpdates(NumUpdates);
                    for (u32 k = 0; k < NumUpdates; ++k) {
                        SubstUpdates[k] = AsgnT(Mgr->Substitute(LocalSubstMap, Updates[k].GetLHS()),
                                                Mgr->Substitute(LocalSubstMap, Updates[k].GetRHS()));
                    }
                    FrozenEFSMs[i]->AddInternalTransition(InitState, FinalState, 
                                                          SubstGuard, SubstUpdates, 
                                                          ActFairnessSets[i]);
                }
            }
        }

        void UFEFSM::FreezeAll()
        {
            if (!StatesFrozen) {
                throw ESMCError((string)"Cannot call FreezeAll() on EFSM before calling " + 
                                "FreezeStates()");
            }
            if (AllFrozen) {
                return;
            }
            for (auto const& FEFSM : FrozenEFSMs) {
                FEFSM->CanonicalizeFairness();
            }
            AllFrozen = true;
        }

        const vector<FrozenEFSM*>& UFEFSM::GetFrozenEFSMs()
        {
            if (!AllFrozen) {
                throw ESMCError((string)"Cannot GetFrozenEFSMs() on EFSM before calling " + 
                                "UFEFSM::FreezeAll()");
            }
            return FrozenEFSMs;
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// UFEFSM.cpp ends here
