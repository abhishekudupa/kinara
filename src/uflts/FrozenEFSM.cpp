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

        namespace Detail {

            MsgTransformer::MsgTransformer(MgrType* Mgr, const string& MsgVarName,
                                           const ExprTypeRef& MsgRecType)
                : VisitorBaseT("MessageTransformer"), Mgr(Mgr),
                  MsgVarName(MsgVarName), MsgRecType(MsgRecType)
            {
                // Nothing here
            }

            MsgTransformer::~MsgTransformer()
            {
                // Nothing here
            }

            void MsgTransformer::VisitVarExpression(const VarExpT* Exp)
            {
                ExpStack.push_back(Exp);
            }

            void MsgTransformer::VisitBoundVarExpression(const BoundVarExpT* Exp)
            {
                ExpStack.push_back(Exp);
            }

            void MsgTransformer::VisitConstExpression(const ConstExpT* Exp)
            {
                ExpStack.push_back(Exp);
            }

            void MsgTransformer::VisitOpExpression(const OpExpT* Exp)
            {
                VisitorBaseT::VisitOpExpression(Exp);
                auto OpCode = Exp->GetOpCode();
                auto const& Children = Exp->GetChildren();
                const u32 NumChildren = Children.size();
                vector<ExpT> SubstChildren(NumChildren);
                
                for (u32 i = 0; i < NumChildren; ++i) {
                    SubstChildren[NumChildren - i - 1] = ExpStack.back();
                    ExpStack.pop_back();
                }
                
                if (OpCode == LTSOps::OpField) {
                    auto Child1AsVar = SubstChildren[0]->As<Exprs::VarExpression>();
                    if (Child1AsVar != nullptr &&
                        Child1AsVar->GetVarName() == MsgVarName &&
                        Child1AsVar->GetVarType()->Is<Exprs::ExprUnionType>()) {

                        auto UnionType = Child1AsVar->GetVarType()->As<Exprs::ExprUnionType>();
                        auto const& Map = UnionType->GetMemberFieldToActFieldMap();
                        
                        auto Child2AsVar = SubstChildren[1]->As<Exprs::VarExpression>();
                        auto const& FieldName = Child2AsVar->GetVarName();
                        auto const& UFAType = Child2AsVar->GetVarType();
                        auto const& RecType = 
                            UFAType->As<Exprs::ExprUFAType>()->GetUnionMemberType();
                        auto it = Map.find(RecType);
                        assert(it != Map.end());
                        auto it2 = it->second.find(FieldName);
                        assert(it2 != it->second.end());
                        auto NewFieldName = it2->second;
                        auto FAType = Mgr->MakeType<Exprs::ExprFieldAccessType>();
                        ExpStack.push_back(Mgr->MakeExpr(LTSOps::OpField,
                                                         Mgr->MakeVar(MsgVarName, MsgRecType),
                                                         Mgr->MakeVar(NewFieldName, FAType)));
                    }
                }
            }

            void MsgTransformer::VisitEQuantifiedExpression(const EQExpT* Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto SubstExp = ExpStack.back();
                ExpStack.pop_back();
                ExpStack.push_back(Mgr->MakeExists(Exp->GetQVarTypes(), SubstExp));
            }

            void MsgTransformer::VisitAQuantifiedExpression(const AQExpT* Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto SubstExp = ExpStack.back();
                ExpStack.pop_back();
                ExpStack.push_back(Mgr->MakeForAll(Exp->GetQVarTypes(), SubstExp));
            }

            ExpT MsgTransformer::Do(const ExpT& Exp, MgrType* Mgr, 
                                    const string& MsgVarName,
                                    const ExprTypeRef& MsgRecType)
            {
                MsgTransformer TheTransformer(Mgr, MsgVarName, MsgRecType);
                Exp->Accept(&TheTransformer);
                return TheTransformer.ExpStack[0];
            }

        } /* end namespace Detail */

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

            auto const& MsgTypeName = MessageType->As<Exprs::ExprRecordType>()->GetName();
            // Also add the update to the outgoing message field type
            AsgnT MTypeAsgn(Mgr->MakeExpr(LTSOps::OpField,
                                          Mgr->MakeVar(MessageName, MessageType),
                                          Mgr->MakeVar(MTypeFieldName, 
                                                       Mgr->MakeType<Exprs::ExprFieldAccessType>())),
                            Mgr->MakeVal(to_string(TheLTS->GetTypeIDForMessageType(MsgTypeName)),
                                         TheLTS->GetMessageIDType()));
            NewUpdates.push_back(MTypeAsgn);

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

        
        // Rebase all local variables to refer to a 
        // field in the record expression
        // Rebase all message expressions to be of the unified
        // message type
        void FrozenEFSM::Rebase(const ExpT& RecordExp, const ExprTypeRef& MsgRecType)
        {
            auto VarMap = SymTab.Bot()->GetDeclMap();
            auto Mgr = TheLTS->GetMgr();
            MgrType::SubstMapT SubstMap;
            auto FAType = Mgr->MakeType<Exprs::ExprFieldAccessType>();
            for (auto const& NameDecl : VarMap) {
                if (NameDecl.second->Is<VarDecl>()) {
                    SubstMap[Mgr->MakeVar(NameDecl.first, NameDecl.second->GetType())] =
                        Mgr->MakeExpr(LTSOps::OpField, RecordExp, 
                                      Mgr->MakeVar(NameDecl.first, FAType));
                }
            }

            // Apply the substitution to everything
            // i.e, the Transitions
            vector<TransitionT> NewTransitions;
            for (auto const& Trans : TransitionVec) {

                auto const& Guard = Trans.GetGuard();
                ExpT MTransGuard;
                if (Trans.GetKind() != TransitionKind::INTERNAL) {
                    const string& MsgName = Trans.GetMessageName();
                    MTransGuard = Mgr->ApplyTransform<Detail::MsgTransformer>(Guard, Mgr,
                                                                              MsgName,
                                                                              MsgRecType);
                } else {
                    MTransGuard = Guard;
                }
                
                vector<AsgnT> NewUpdates;

                for (auto const& Update : Trans.GetUpdates()) {
                    auto const& LHS = Update.GetLHS();
                    auto const& RHS = Update.GetRHS();
                    ExpT MTransLHS, MTransRHS;
                    if (Trans.GetKind() != TransitionKind::INTERNAL) {
                        auto const& MsgName = Trans.GetMessageName();
                        MTransLHS = Mgr->ApplyTransform<Detail::MsgTransformer>(LHS, Mgr, 
                                                                                MsgName,
                                                                                MsgRecType);
                        MTransRHS = Mgr->ApplyTransform<Detail::MsgTransformer>(RHS, Mgr, 
                                                                                MsgName,
                                                                                MsgRecType);
                    } else {
                        MTransLHS = LHS;
                        MTransRHS = RHS;
                    }

                    NewUpdates.push_back(AsgnT(Mgr->Substitute(SubstMap, MTransLHS),
                                               Mgr->Substitute(SubstMap, MTransRHS)));
                }
                
                TransitionT NewTrans;
                auto SubstGuard = Mgr->Substitute(SubstMap, MTransGuard);
                if (Trans.GetKind() == TransitionKind::INPUT) {
                    NewTrans = TransitionT::MakeInputTransition(Trans.GetInitState(),
                                                                Trans.GetFinalState(),
                                                                SubstGuard,
                                                                NewUpdates, Trans.GetMessageName(),
                                                                Trans.GetMessageType());
                } else if (Trans.GetKind() == TransitionKind::OUTPUT) {
                    NewTrans = TransitionT::MakeOutputTransition(Trans.GetInitState(),
                                                                 Trans.GetFinalState(),
                                                                 SubstGuard,
                                                                 NewUpdates,
                                                                 Trans.GetMessageName(),
                                                                 Trans.GetMessageType(),
                                                                 Trans.GetFairnessSet());
                } else {
                    NewTrans = TransitionT::MakeInternalTransition(Trans.GetInitState(),
                                                                   Trans.GetFinalState(),
                                                                   SubstGuard,
                                                                   NewUpdates,
                                                                   Trans.GetFairnessSet());
                }

                NewTransitions.push_back(NewTrans);
            }

            TransitionVec = NewTransitions;
        }

        const map<string, Detail::StateDescriptor>& FrozenEFSM::GetStates() const
        {
            return States;
        }

        vector<TransitionT> FrozenEFSM::GetTransitionsOnMsg(const ExprTypeRef& MsgType) const
        {
            vector<TransitionT> Retval;
            for (auto const& Trans : TransitionVec) {
                if ((Trans.GetKind() == TransitionKind::INPUT ||
                     Trans.GetKind() == TransitionKind::OUTPUT) &&
                    Trans.GetMessageType() == MsgType) {
                    Retval.push_back(Trans);
                }
            }
            return Retval;
        }

        vector<TransitionT> FrozenEFSM::GetInteralTransitions() const
        {
            vector<TransitionT> Retval;
            for (auto const& Trans : TransitionVec) {
                if (Trans.GetKind() == TransitionKind::INTERNAL) {
                    Retval.push_back(Trans);
                }
            }
            return Retval;
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// FrozenEFSM.cpp ends here
