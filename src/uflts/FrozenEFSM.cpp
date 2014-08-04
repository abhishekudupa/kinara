// FrozenEFSM.cpp --- 
// 
// Filename: FrozenEFSM.cpp
// Author: Abhishek Udupa
// Created: Sun Aug  3 14:33:58 2014 (-0400)
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

#include "FrozenEFSM.hpp"
#include "UFLTS.hpp"

namespace ESMC {
    namespace LTS {

        // Frozen EFSM implementation
        FrozenEFSM::FrozenEFSM(const string& BaseName, UFLTS* TheLTS,
                               const vector<ExpT>& InstParams,
                               UFEFSM* TheEFSM,
                               const ExprTypeRef& StateType,
                               const map<string, Detail::StateDescriptor>& States)
            : BaseName(BaseName), TheLTS(TheLTS), InstParams(InstParams), 
              TheEFSM(TheEFSM), TheChannel(nullptr), StateType(StateType), States(States)
        {
            InstName = BaseName;
            
            for (u32 i = 0; i < InstParams.size(); ++i) {
                auto ParamAsConst = InstParams[i]->As<Exprs::ConstExpression>();
                auto const& ConstVal = ParamAsConst->GetConstValue();
                InstName += ((string)"[" + ConstVal + "]");
            }
            
            SymTab.Bind("state", new VarDecl("state", StateType));
        }

        FrozenEFSM::FrozenEFSM(const string& BaseName, UFLTS* TheLTS,
                               const vector<ExpT>& InstParams,
                               ChannelEFSM* TheChannel,
                               const ExprTypeRef& StateType,
                               const map<string, Detail::StateDescriptor>& States)
            : BaseName(BaseName), TheLTS(TheLTS), InstParams(InstParams), 
              TheEFSM(nullptr), TheChannel(TheChannel), StateType(StateType), States(States)
        {
            InstName = BaseName;
            
            for (u32 i = 0; i < InstParams.size(); ++i) {
                auto ParamAsConst = InstParams[i]->As<Exprs::ConstExpression>();
                auto const& ConstVal = ParamAsConst->GetConstValue();
                InstName += ((string)"[" + ConstVal + "]");
            }
            
            SymTab.Bind("state", new VarDecl("state", StateType));
        }

        FrozenEFSM::~FrozenEFSM()
        {
            // Nothing here
        }

        void FrozenEFSM::AddVariable(const string& VarName,
                                     const ExprTypeRef& VarType)
        {
            if (SymTab.Lookup(VarName) != DeclRef::NullPtr) {
                throw ESMCError((string)"Redeclaration of variable \"" + VarName + 
                                "\" in FrozenEFSM");
            }
            SymTab.Bind(VarName, new VarDecl(VarName, VarType));
        }

        void FrozenEFSM::AddInputMsg(const ExprTypeRef& MType)
        {
            if (!TheLTS->CheckMessageType(MType)) {
                throw ESMCError((string)"Message Type: " + MType->ToString() + 
                                " has not been declared in the LTS");
            }
            Inputs.insert(MType);
        }

        void FrozenEFSM::AddOutputMsg(const ExprTypeRef& MType)
        {
            if (!TheLTS->CheckMessageType(MType)) {
                throw ESMCError((string)"Message Type: " + MType->ToString() + 
                                " has not been declared in the LTS");
            }
            Outputs.insert(MType);
        }
        
        void FrozenEFSM::AddInputTransition(const string& InitState,
                                            const string& FinalState,
                                            const ExpT& Guard,
                                            const vector<AsgnT>& Updates,
                                            const string& MessageName,
                                            const ExprTypeRef& MessageType)
        {
            auto Mgr = TheLTS->GetMgr();
            CheckState(InitState, States);
            CheckState(FinalState, States);
            
            CheckExpr(Guard, SymTab, TheLTS->GetMgr());
            if (!Guard->GetType()->Is<Exprs::ExprBoolType>()) {
                throw ESMCError((string)"Guard of a transition must be boolean valued");
            }

            if (Inputs.find(MessageType) == Inputs.end()) {
                throw ESMCError((string)"Message type \"" + MessageType->ToString() + 
                                "\" is not an input for EFSM \"" + InstName + "\"");
            }

            SymTab.Push();
            SymTab.Bind(MessageName, new MsgDecl(MessageName, MessageType));
            CheckUpdates(Updates, SymTab, TheLTS->GetMgr(), true, MessageName);
            auto Scope = SymTab.Pop();

            // Add the check on the state into guard
            auto NewGuard = Mgr->MakeExpr(LTSOps::OpAND, Guard,
                                          Mgr->MakeExpr(LTSOps::OpEQ, 
                                                        Mgr->MakeVar("state", StateType), 
                                                        Mgr->MakeVal(InitState, StateType)));
            auto NewUpdates = Updates;
            NewUpdates.push_back(AsgnT(Mgr->MakeVar("state", StateType),
                                       Mgr->MakeVal(FinalState, StateType)));
            
            auto Transition = TransitionT::MakeInputTransition(InitState,
                                                               FinalState,
                                                               NewGuard,
                                                               NewUpdates,
                                                               MessageName,
                                                               MessageType);
            Transitions.push_back(pair<TransitionT, ScopeRef>(Transition, Scope));
            TransitionVec.push_back(Transition);
        }

        void FrozenEFSM::AddOutputTransition(const string& InitState,
                                             const string& FinalState,
                                             const ExpT& Guard,
                                             const vector<AsgnT>& Updates,
                                             const string& MessageName,
                                             const ExprTypeRef& MessageType,
                                             const unordered_set<u32>& FairnessSet)
        {
            auto Mgr = TheLTS->GetMgr();
            CheckState(InitState, States);
            CheckState(FinalState, States);
            if (Inputs.find(MessageType) == Outputs.end()) {
                throw ESMCError((string)"Message type \"" + MessageType->ToString() + 
                                "\" is not an output for EFSM \"" + InstName + "\"");
            }
            
            CheckExpr(Guard, SymTab, TheLTS->GetMgr());
            if (!Guard->GetType()->Is<Exprs::ExprBoolType>()) {
                throw ESMCError((string)"Guard of a transition must be boolean valued");
            }

            SymTab.Push();
            SymTab.Bind(MessageName, new MsgDecl(MessageName, MessageType));
            CheckUpdates(Updates, SymTab, TheLTS->GetMgr(), false, MessageName);
            
            auto Scope = SymTab.Pop();

            // Add the check on the state into guard
            auto NewGuard = Mgr->MakeExpr(LTSOps::OpAND, Guard,
                                          Mgr->MakeExpr(LTSOps::OpEQ, 
                                                        Mgr->MakeVar("state", StateType), 
                                                        Mgr->MakeVal(InitState, StateType)));
            auto NewUpdates = Updates;
            NewUpdates.push_back(AsgnT(Mgr->MakeVar("state", StateType),
                                       Mgr->MakeVal(FinalState, StateType)));

            auto Transition = TransitionT::MakeOutputTransition(InitState,
                                                                FinalState,
                                                                NewGuard,
                                                                NewUpdates,
                                                                MessageName,
                                                                MessageType,
                                                                FairnessSet);
            Transitions.push_back(pair<TransitionT, ScopeRef>(Transition, Scope));
            TransitionVec.push_back(Transition);
        }
        
        void FrozenEFSM::AddInternalTransition(const string& InitState,
                                               const string& FinalState,
                                               const ExpT& Guard,
                                               const vector<AsgnT>& Updates,
                                               const unordered_set<u32>& FairnessSet)
        {
            auto Mgr = TheLTS->GetMgr();
            CheckState(InitState, States);
            CheckState(FinalState, States);
            CheckExpr(Guard, SymTab, TheLTS->GetMgr());
            if (!Guard->GetType()->Is<Exprs::ExprBoolType>()) {
                throw ESMCError((string)"Guard of a transition must be boolean valued");
            }
            CheckUpdates(Updates, SymTab, TheLTS->GetMgr(), false, "");
            // Add the check on the state into guard
            auto NewGuard = Mgr->MakeExpr(LTSOps::OpAND, Guard,
                                          Mgr->MakeExpr(LTSOps::OpEQ, 
                                                        Mgr->MakeVar("state", StateType), 
                                                        Mgr->MakeVal(InitState, StateType)));
            auto NewUpdates = Updates;
            NewUpdates.push_back(AsgnT(Mgr->MakeVar("state", StateType),
                                       Mgr->MakeVal(FinalState, StateType)));

            auto Transition = TransitionT::MakeInternalTransition(InitState,
                                                                  FinalState,
                                                                  NewGuard,
                                                                  NewUpdates,
                                                                  FairnessSet);
            Transitions.push_back(pair<TransitionT, ScopeRef>(Transition, ScopeRef::NullPtr));
            TransitionVec.push_back(Transition);
        }
              
        void FrozenEFSM::CanonicalizeFairness()
        {
            UIDGenerator FairnessUIDGen;
            unordered_map<u32, u32> FairnessMap;

            for (auto const& TransScope : Transitions) {
                auto const& Trans = TransScope.first;
                auto const& CurFairnessSet = Trans.GetFairnessSet();
                for (auto const& Fairness : CurFairnessSet) {
                    if (FairnessMap.find(Fairness) != FairnessMap.end()) {
                        continue;
                    } else {
                        FairnessMap[Fairness] = FairnessUIDGen.GetUID();
                    }
                }
            }

            // We've mapped all the fairnesses now
            // Substitute them out
            for (auto const& TransScope : Transitions) {
                auto const& Trans = TransScope.first;
                auto const& OldFairnessSet = Trans.GetFairnessSet();
                unordered_set<u32> NewFairnessSet;
                for (auto const& Fairness : OldFairnessSet) {
                    NewFairnessSet.insert(FairnessMap[Fairness]);
                }
                Trans.SetFairnessSet(NewFairnessSet);
            }
        }

        const set<ExprTypeRef>& FrozenEFSM::GetInputs() const
        {
            return Inputs;
        }

        const set<ExprTypeRef>& FrozenEFSM::GetOutputs() const
        {
            return Outputs;
        }

        const ExprTypeRef& FrozenEFSM::GetStateType() const
        {
            return StateType;
        }

        const SymbolTable& FrozenEFSM::GetSymTab() const
        {
            return SymTab;
        }

        const vector<TransitionT>& FrozenEFSM::GetTransitions() const
        {
            return TransitionVec;
        }

        const string& FrozenEFSM::GetName() const
        {
            return InstName;
        }

        const string& FrozenEFSM::GetBaseName() const
        {
            return BaseName;
        }

        const vector<ExpT>& FrozenEFSM::GetInstParams() const
        {
            return InstParams;
        }

        UFLTS* FrozenEFSM::GetLTS() const
        {
            return TheLTS;
        }

        UFEFSM* FrozenEFSM::GetEFSM() const
        {
            return TheEFSM;
        }

        ChannelEFSM* FrozenEFSM::GetChannel() const
        {
            return TheChannel;
        }

        const map<string, Detail::StateDescriptor>& FrozenEFSM::GetStates() const
        {
            return States;
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// FrozenEFSM.cpp ends here
