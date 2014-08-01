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
#include "ParamUtils.hpp"

#include <type_traits>

namespace ESMC {
    namespace LTS {

        static inline void CheckExpr(const ExpT& Exp,
                                     const SymbolTable& SymTab,
                                     MgrType* Mgr)
        {
            auto Gatherer = Detail::VarGatherer();
            auto Vars = Mgr->Gather(Exp, Gatherer);
            for (auto const& Var : Vars) {
                auto const& VarName = Var->template SAs<Exprs::VarExpression>()->GetVarName();
                auto Lookup = SymTab.Lookup(VarName);
                if (Lookup == DeclRef::NullPtr) {
                    throw ESMCError((string)"Unbound variable \"" + VarName + "\"");
                }
            }
        }

        static inline void CheckLValCompat(const ExpT& Exp,
                                           const SymbolTable& SymTab)
        {
            if (!IsLVal(Exp)) {
                throw ESMCError((string)"Expression is not an LValue:\n" + 
                                Exp->ToString());
            }
            auto ExpAsVar = Exp->As<Exprs::VarExpression>();
            if (ExpAsVar != nullptr) {
                // Check that this is a variable and not a parameter
                auto const& VarName = ExpAsVar->GetVarName();
                auto LookupRes = SymTab.Lookup(VarName);
                if (!LookupRes->Is<VarDecl>()) {
                    throw ESMCError((string)"Error: Parameters cannot be used as LVals");
                }
            }
        }

        static inline void CheckUpdates(const vector<AsgnT>& Updates,
                                        const SymbolTable& SymTab,
                                        MgrType* Mgr)
        {
            for(auto const& Asgn : Updates) {
                CheckExpr(Asgn.GetLHS(), SymTab, Mgr);
                CheckLValCompat(Asgn.GetLHS(), SymTab);
                CheckExpr(Asgn.GetRHS(), SymTab, Mgr);
            }
        }

        static inline void CheckMsg(const ExprTypeRef& MsgType, 
                                    const set<ExprTypeRef>& MsgSet)
        {
            if (MsgSet.find(MsgType) == MsgSet.end()) {
                throw ESMCError((string)"Invalid message for transition");
            }
        }

        static inline void CheckState(const string& StateName,
                                      const map<string, Detail::StateDescriptor>& States)
        {
            if (States.find(StateName) == States.end()) {
                throw ESMCError((string)"Unknown state \"" + StateName + "\"");
            }
        }

        static inline void CheckParamPurity(const ExpT& Exp,
                                            const SymbolTable& SymTab,
                                            MgrType* Mgr)
        {
            auto Gatherer = Detail::VarGatherer();
            auto Vars = Mgr->Gather(Exp, Gatherer);
            for (auto const& Var : Vars) {
                auto const& VarName = Var->template SAs<Exprs::VarExpression>()->GetVarName();
                auto Lookup = SymTab.Lookup(VarName);
                assert(Lookup != DeclRef::NullPtr);
                if (!Lookup->Is<ParamDecl>()) {
                    throw ESMCError((string)"Expression is not pure wrt parameters. Expression:\n" + 
                                    Exp->ToString());
                }
            }
        }

        static inline void CheckParams(const vector<ExpT>& Params,
                                       const ExpT& Constraint,
                                       SymbolTable& SymTab,
                                       MgrType* Mgr)
        {
            for (auto const& Param : Params) {
                auto ParamAsVar = Param->As<Exprs::VarExpression>();
                if (ParamAsVar == nullptr) {
                    throw ESMCError((string)"Parameters to EFSMS must be variable expressions");
                }
                auto Type = ParamAsVar->GetVarType();
                if ((!Type->Is<Exprs::ExprSymmetricType>()) &&
                    (!Type->Is<Exprs::ExprRangeType>()) &&
                    (!Type->Is<Exprs::ExprEnumType>())) {
                    throw ESMCError((string)"Parameter types must be symmetric, range or enum");
                }
                
                auto const& VarName = ParamAsVar->GetVarName();

                auto Res = SymTab.Lookup(VarName);
                if (Res != DeclRef::NullPtr && 
                    Res->GetType() != Param->GetType()) {
                    throw ESMCError((string)"Parameter \"" + VarName + "\" already " + 
                                    "declared with a different type!");
                } else {
                    // A new parameter
                    SymTab.Bind(VarName, new ParamDecl(VarName, Type));
                }
            }

            CheckExpr(Constraint, SymTab, Mgr);
            CheckParamPurity(Constraint, SymTab, Mgr);
            if (!Constraint->GetType()->Is<Exprs::ExprBoolType>()) {
                throw ESMCError((string)"Constraints on parameters must be a boolean expression");
            }
        }

        static inline vector<ExprTypeRef> InstantiateMsg(const Detail::ParametrizedMessage& PMesg,
                                                         MgrType* Mgr,
                                                         const MgrType::SubstMapT& GlobalSM)
        {
            auto const& MType = PMesg.PType;
            auto const& Params = PMesg.Params;
            auto const& Constraint = PMesg.Constraint;
            vector<ExprTypeRef> Retval;

            auto&& ParamVals = InstantiatePendingParams(Params, Mgr, GlobalSM, Constraint);
            
            for (auto const& SubstParams : ParamVals) {
                // Now we're finally ready to instantiate the message
                Retval.push_back(Mgr->InstantiateType(MType, SubstParams));
            }
            
            return Retval;
        }

        // Implementation of UFEFSM
        UFEFSM::UFEFSM(UFLTS* TheLTS,
                       const string& Name,
                       const vector<ExpT>& Params,
                       const ExpT& Constraint)
            : TheLTS(TheLTS), Name(Name), Params(Params),
              Constraint(Constraint)
        {
            CheckParams(Params, Constraint, SymTab, TheLTS->Mgr);
        }

        UFEFSM::~UFEFSM()
        {
            // Nothing here
        }


        void UFEFSM::AddInputMsg(const ExprTypeRef& MType)
        {
            if (!MType->Is<Exprs::ExprRecordType>()) {
                throw ESMCError((string)"Only record types can be message types");
            }
            if (!TheLTS->CheckMessageType(MType)) {
                throw ESMCError((string)"Message Type: " + MType->ToString() + 
                                " has not been declared in the LTS");
            }
            Inputs.insert(MType);
        }

        void UFEFSM::AddInputMsg(const vector<ExpT>& Params,
                                 const ExpT& Constraint,
                                 const ExprTypeRef& MType)
        {
            SymTab.Push();
            CheckParams(Params, Constraint, SymTab, TheLTS->Mgr);
            SymTab.Pop();
            ParametrizedInputs.insert(Detail::ParametrizedMessage(MType, Params, Constraint));
        }

        void UFEFSM::AddOutputMsg(const ExprTypeRef& MType)
        {
            if (!MType->Is<Exprs::ExprRecordType>()) {
                throw ESMCError((string)"Only record types can be message types");
            }
            if (!TheLTS->CheckMessageType(MType)) {
                throw ESMCError((string)"Message Type: " + MType->ToString() + 
                                " has not been declared in the LTS");
            }
            Outputs.insert(MType);
        }

        void UFEFSM::AddOutputMsg(const vector<ExpT>& Params,
                                  const ExpT& Constraint,
                                  const ExprTypeRef& MType)
        {
            SymTab.Push();
            CheckParams(Params, Constraint, SymTab, TheLTS->Mgr);
            SymTab.Pop();
            ParametrizedOutputs.insert(Detail::ParametrizedMessage(MType, Params, Constraint));
        }

        void UFEFSM::AddState(const string& StateName,
                              bool Initial,
                              bool Final,
                              bool Accepting,
                              bool Error,
                              bool Dead)
        {
            Detail::StateDescriptor Desc(StateName, Initial, Final, Accepting, Error, Dead);
            auto it = States.find(StateName);
            if (it == States.end()) {
                States[StateName] = Desc;
            } else if (it->second != Desc) {
                throw ESMCError((string)"Redeclaration of state with different parameters");
            }
        }
        
        string UFEFSM::AddState(bool Initial,
                                bool Final,
                                bool Accepting,
                                bool Error,
                                bool Dead)
        {
            auto StateUID = StateUIDGenerator.GetUID();
            string StateName = (string)"State_" + to_string(StateUID);
            Detail::StateDescriptor Desc(StateName, Initial, Final, Accepting, Error, Dead);
            States[StateName] = Desc;
            return StateName;
        }

        void UFEFSM::AddVariable(const string& VarName,
                                 const ExprTypeRef& VarType)
        {
            if (SymTab.LookupTop(VarName) != DeclRef::NullPtr) {
                throw ESMCError((string)"Rebinding of variable \"" + VarName + "\"");
            }
            SymTab.Bind(VarName, new VarDecl(VarName, VarType));
        }

            
        void UFEFSM::AddInputTransition(const string& InitState,
                                        const string& FinalState,
                                        const ExpT& Guard,
                                        const vector<AsgnT>& Updates,
                                        const string& MessageName,
                                        const ExprTypeRef& MessageType)
        {
            CheckMsg(MessageType, Inputs);
            CheckState(InitState, States);
            CheckState(FinalState, States);

            SymTab.Push();
            SymTab.Bind(MessageName, new MsgDecl(MessageName, MessageType));
            
            CheckUpdates(Updates, SymTab, TheLTS->Mgr);

            // Guard cannot refer to message fields
            auto Scope = SymTab.Pop();

            CheckExpr(Guard, SymTab, TheLTS->Mgr);


            if (!Guard->GetType()->Is<Exprs::ExprBoolType>()) {
                throw ESMCError((string)"Guard for transition is not boolean");
            }

            auto Transition = TransitionT::MakeInputTransition(InitState,
                                                               FinalState,
                                                               Guard,
                                                               Updates,
                                                               MessageName,
                                                               MessageType);

            Transitions.push_back(pair<TransitionT, ScopeRef>(Transition, Scope));
        }

        void UFEFSM::AddInputTransition(const string& InitState,
                                        const string& FinalState,
                                        const ExpT& Guard,
                                        const vector<AsgnT>& Updates,
                                        const string& MessageName,
                                        const vector<ExpT>& Params,
                                        const ExpT& Constraint,
                                        const ExprTypeRef& MessageType)
        {
            CheckState(InitState, States);
            CheckState(FinalState, States);

            SymTab.Push();
            CheckParams(Params, Constraint, SymTab, TheLTS->Mgr);
            SymTab.Pop();

            if (!MessageType->Is<Exprs::ExprParametricType>()) {
                throw ESMCError((string)"Non-parametric message type in parametrized " + 
                                "input transition");
            }

            // We defer the rest of the checks until later

            Detail::ParametrizedTransition Transition(TransitionKind::INPUT, 
                                                      InitState, FinalState,
                                                      Guard, Updates, MessageName,
                                                      Params, Constraint, MessageType,
                                                      unordered_set<u32>());

            ParametrizedTransitions.push_back(Transition);
        }

        void UFEFSM::AddOutputTransition(const string& InitState,
                                         const string& FinalState,
                                         const ExpT& Guard,
                                         const vector<AsgnT>& Updates,
                                         const string& MessageName,
                                         const ExprTypeRef& MessageType,
                                         const unordered_set<u32>& FairnessSet)
        {
            CheckMsg(MessageType, Outputs);
            CheckState(InitState, States);
            CheckState(FinalState, States);
            
            SymTab.Push();
            SymTab.Bind(MessageName, new VarDecl(MessageName, MessageType));
            
            CheckUpdates(Updates, SymTab, TheLTS->Mgr);
            auto Scope = SymTab.Pop();
            
            CheckExpr(Guard, SymTab, TheLTS->Mgr);
            if (!Guard->GetType()->Is<Exprs::ExprBoolType>()) {
                throw ESMCError((string)"Guard for transition is not boolean");
            }

            auto Transition = TransitionT::MakeOutputTransition(InitState,
                                                                FinalState,
                                                                Guard,
                                                                Updates,
                                                                MessageName,
                                                                MessageType,
                                                                FairnessSet);
            Transitions.push_back(pair<TransitionT, ScopeRef>(Transition, Scope));
        }

        void UFEFSM::AddOutputTransition(const string& InitState,
                                         const string& FinalState,
                                         const ExpT& Guard,
                                         const vector<AsgnT>& Updates,
                                         const string& MessageName,
                                         const vector<ExpT>& Params,
                                         const ExpT& Constraint,
                                         const ExprTypeRef& MessageType,
                                         const unordered_set<u32>& FairnessSet)
        {
            CheckState(InitState, States);
            CheckState(FinalState, States);
            
            SymTab.Push();
            CheckParams(Params, Constraint, SymTab, TheLTS->Mgr);
            SymTab.Pop();

            if (!MessageType->Is<Exprs::ExprParametricType>()) {
                throw ESMCError((string)"Non-parametric message type in parametrized " + 
                                "output transition");
            }

            // We defer the rest of the checks until later
            Detail::ParametrizedTransition Transition(TransitionKind::OUTPUT, 
                                                      InitState, FinalState,
                                                      Guard, Updates, MessageName,
                                                      Params, Constraint, MessageType,
                                                      FairnessSet);

            ParametrizedTransitions.push_back(Transition);
        }


        void UFEFSM::AddInternalTransition(const string& InitState,
                                           const string& FinalState,
                                           const ExpT& Guard,
                                           const vector<AsgnT>& Updates,
                                           const unordered_set<u32>& FairnessSet)
        {
            CheckState(InitState, States);
            CheckState(FinalState, States);
            CheckUpdates(Updates, SymTab, TheLTS->Mgr);
            CheckExpr(Guard, SymTab, TheLTS->Mgr);
            
            if (!Guard->GetType()->Is<Exprs::ExprBoolType>()) {
                throw ESMCError((string)"Guard for transition is not boolean");
            }

            auto Transition = TransitionT::MakeInternalTransition(InitState,
                                                                  FinalState,
                                                                  Guard,
                                                                  Updates,
                                                                  FairnessSet);
            Transitions.push_back(pair<TransitionT, ScopeRef>(Transition, ScopeRef::NullPtr));
        }

        void UFEFSM::AddInternalTransition(const string& InitState,
                                           const string& FinalState,
                                           const ExpT& Guard,
                                           const vector<AsgnT>& Updates,
                                           const vector<ExpT>& Params,
                                           const ExpT& Constraint,
                                           const unordered_set<u32>& FairnessSet)
        {
            CheckState(InitState, States);
            CheckState(FinalState, States);
            
            SymTab.Push();
            CheckParams(Params, Constraint, SymTab, TheLTS->Mgr);
            SymTab.Pop();
            
            Detail::ParametrizedTransition Transition(TransitionKind::INTERNAL,
                                                      InitState, FinalState,
                                                      Guard, Updates, "", Params,
                                                      Constraint, ExprTypeRef::NullPtr,
                                                      FairnessSet);
            ParametrizedTransitions.push_back(Transition);
        }


        FrozenEFSM* UFEFSM::Instantiate(const vector<ExpT>& ParamVals,
                                        const ExprTypeRef& StateType) const
        {
            auto Mgr = TheLTS->Mgr;
            string InstName = Name;
            for (auto const& Param : ParamVals) {
                auto ParamAsConst = Param->As<Exprs::ConstExpression>();
                auto const& Val = ParamAsConst->GetConstValue();
                InstName += ((string)"[" + Val + "]");
            }
            
            FrozenEFSM* Retval = new FrozenEFSM(InstName, TheLTS, StateType, States);
            MgrType::SubstMapT SubstMapGlobal;
            
            for (u32 i = 0; i < Params.size(); ++i) {
                SubstMapGlobal[Params[i]] = ParamVals[i];
            }
            
            // Push only the variables from the bottom scope
            auto BotScope = SymTab.Bot();
            auto DeclMap = BotScope->GetDeclMap();
            for (auto const& Decl : DeclMap) {
                if (Decl.second->Is<VarDecl>()) {
                    Retval->AddVariable(Decl.first, Decl.second->GetType());
                }
            }
            
            // Push through the common inputs
            for (auto const& Input : Inputs) {
                Retval->AddInputMsg(Input);
            }
            for (auto const& Output : Outputs) {
                Retval->AddOutputMsg(Output);
            }

            // Instantiate the parametric inputs
            for(auto const& PMesg : ParametrizedInputs) {
                auto&& InstMsgTypes = InstantiateMsg(PMesg, Mgr, SubstMapGlobal);
                for (auto const& InstMsgType : InstMsgTypes) {
                    Retval->AddInputMsg(InstMsgType);
                }
            }

            // Instantiate the parametric outputs
            for(auto const& PMesg : ParametrizedOutputs) {
                auto&& InstMsgTypes = InstantiateMsg(PMesg, Mgr, SubstMapGlobal);
                for (auto const& InstMsgType : InstMsgTypes) {
                    Retval->AddOutputMsg(InstMsgType);
                }
            }

            // Instantiate all the non parametric transitions
            for (auto const& TranScope : Transitions) {
                auto Trans = TranScope.first;
                auto Scope = TranScope.second;

                auto const& Updates = Trans.GetUpdates();
                vector<AsgnT> NewUpdates;

                for (auto const& Update : Updates) {
                    auto NewLHS = Mgr->Substitute(SubstMapGlobal, Update.GetLHS());
                    auto NewRHS = Mgr->Substitute(SubstMapGlobal, Update.GetRHS());
                    NewUpdates.push_back(AsgnT(NewLHS, NewRHS));
                }

                auto NewGuard = Mgr->Substitute(SubstMapGlobal, Trans.GetGuard());

                if (Trans.GetKind() == TransitionKind::INPUT) {
                    Retval->AddInputTransition(Trans.GetInitState(),
                                               Trans.GetFinalState(),
                                               NewGuard,
                                               NewUpdates,
                                               Trans.GetMessageName(),
                                               Trans.GetMessageType());
                } else if (Trans.GetKind() == TransitionKind::OUTPUT) {
                    Retval->AddOutputTransition(Trans.GetInitState(),
                                                Trans.GetFinalState(),
                                                NewGuard,
                                                NewUpdates,
                                                Trans.GetMessageName(),
                                                Trans.GetMessageType(),
                                                Trans.GetFairnessSet());
                } else {
                    Retval->AddInternalTransition(Trans.GetInitState(),
                                                  Trans.GetFinalState(),
                                                  NewGuard,
                                                  NewUpdates,
                                                  Trans.GetFairnessSet());
                }
                return Retval;
            }

            // Instantiate the parametric transitions
            for (auto const& PTrans : ParametrizedTransitions) {
                auto const& Params = PTrans.Params;
                const u32 NumParams = Params.size();
                auto const& Constraint = PTrans.Constraint;
                auto const&& ParamVals = InstantiatePendingParams(Params, Mgr, 
                                                                  SubstMapGlobal,
                                                                  Constraint);
                MgrType::SubstMapT SubstMapLocal;
                for (auto const& SubstParams : ParamVals) {
                    SubstMapLocal = SubstMapGlobal;
                    for (u32 i = 0; i < NumParams; ++i) {
                        if (!Params[i]->Is<Exprs::ConstExpression>()) {
                            SubstMapLocal[Params[i]] = SubstParams[i];
                        }
                    }
                    
                    auto SubstGuard = Mgr->Substitute(SubstMapLocal, PTrans.Guard);
                    vector<AsgnT> SubstUpdates;
                    for (auto const& Update : PTrans.Updates) {
                        auto SubstLHS = Mgr->Substitute(SubstMapLocal, Update.GetLHS());
                        auto SubstRHS = Mgr->Substitute(SubstMapLocal, Update.GetRHS());
                        SubstUpdates.push_back(AsgnT(SubstLHS, SubstRHS));
                    }

                    if (PTrans.Kind == TransitionKind::INPUT) {
                        ExprTypeRef MType;
                        if (PTrans.MessageType->Is<Exprs::ExprParametricType>()) {
                            MType = Mgr->InstantiateType(PTrans.MessageType, SubstParams);
                        } else {
                            MType = PTrans.MessageType;
                        }
                        Retval->AddInputTransition(PTrans.InitialState,
                                                   PTrans.FinalState,
                                                   SubstGuard, SubstUpdates,
                                                   PTrans.MessageName,
                                                   MType);
                    } else if (PTrans.Kind == TransitionKind::OUTPUT) {
                        ExprTypeRef MType;
                        if (PTrans.MessageType->Is<Exprs::ExprParametricType>()) {
                            MType = Mgr->InstantiateType(PTrans.MessageType, SubstParams);
                        } else {
                            MType = PTrans.MessageType;
                        }
                        Retval->AddOutputTransition(PTrans.InitialState,
                                                    PTrans.FinalState,
                                                    SubstGuard, SubstUpdates,
                                                    PTrans.MessageName,
                                                    MType, PTrans.FairnessSet);
                    } else {
                        Retval->AddInternalTransition(PTrans.InitialState,
                                                      PTrans.FinalState,
                                                      SubstGuard, SubstUpdates,
                                                      PTrans.FairnessSet);
                    }
                }
            }

            Retval->CanonicalizeFairness();
            return Retval;
        }

        vector<FrozenEFSM*> UFEFSM::Instantiate() const
        {
            // Create a type for the states first
            auto Mgr = TheLTS->Mgr;
            set<string> StateNames;
            for (auto const& StateD : States) {
                StateNames.insert(StateD.first);
            }
            auto StateType = Mgr->MakeType<Exprs::ExprEnumType>(Name + "StateT", StateNames);

            // Construct the Frozen EFSM
            vector<FrozenEFSM*> Retval;
            auto&& SubstMaps = InstantiateParams(Params, Constraint, Mgr);
            for (auto const& SubstMap : SubstMaps) {
                Retval.push_back(Instantiate(SubstMap, StateType));
            }
            return Retval;
        }

        // Channel EFSM implementation
        ChannelEFSM::ChannelEFSM(u32 Capacity, bool Ordered, bool Lossy, 
                                 bool Duplicating, bool Blocking, 
                                 bool FiniteLoss, bool FiniteDup, 
                                 bool Compassionate, bool Just, UFLTS* TheLTS)
            : TheLTS(TheLTS), Capacity(Capacity), Ordered(Ordered), Lossy(Lossy),
              Duplicating(Duplicating), Blocking(Blocking),
              FiniteLoss(FiniteLoss), FiniteDup(FiniteDup),
              Compassionate(Compassionate), Just(Just)
        {
            // Sanity checks on params
            if (Blocking && !Lossy) {
                throw ESMCError((string)"Only Lossy channels can be declared Blocking");
            }
            if (FiniteLoss && !Lossy) {
                throw ESMCError((string)"Only Lossy channels can be declared FiniteLoss");
            }
            if (FiniteDup && !Duplicating) {
                throw ESMCError((string)"Only Duplicating channels can be declared FiniteDup");
            }
            if (Compassionate && Just) {
                throw ESMCError((string)"A channel can be compassionate or just, not both!");
            }
        }

        ChannelEFSM::~ChannelEFSM()
        {
            // Nothing here
        }

        void ChannelEFSM::AddMessage(const ExprTypeRef& Type)
        {
            if (!Type->Is<Exprs::ExprRecordType>()) {
                throw ESMCError((string)"Only record types can be messages");
            }
            Messages.insert(Type);
        }

        void ChannelEFSM::AddMessage(const ExprTypeRef& Type,
                                     const vector<ExpT>& Params,
                                     const ExpT& Constraint)
        {
            if (!Type->Is<Exprs::ExprParametricType>()) {
                throw ESMCError((string)"Only parametric types can be used as parametric " + 
                                "messages");
            }

            PMessages.insert(Detail::ParametrizedMessage(Type, Params, Constraint));
        }

        UFEFSM* ChannelEFSM::ToEFSM()
        {
            // TODO
            return nullptr;
        }

        // Frozen EFSM implementation
        FrozenEFSM::FrozenEFSM(const string& Name, UFLTS* TheLTS,
                               const ExprTypeRef& StateType,
                               const map<string, Detail::StateDescriptor>& States)
            : Name(Name), TheLTS(TheLTS), StateType(StateType),
              States(States)
        {
            // Nothing here
        }

        FrozenEFSM::~FrozenEFSM()
        {
            // Nothing here
        }

        void FrozenEFSM::AddVariable(const string& VarName,
                                     const ExprTypeRef& VarType)
        {
            if (SymTab.LookupTop(VarName) != DeclRef::NullPtr) {
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
            CheckState(InitState, States);
            CheckState(FinalState, States);
            
            CheckExpr(Guard, SymTab, TheLTS->Mgr);
            if (!Guard->GetType()->Is<Exprs::ExprBoolType>()) {
                throw ESMCError((string)"Guard of a transition must be boolean valued");
            }

            if (Inputs.find(MessageType) == Inputs.end()) {
                throw ESMCError((string)"Message type \"" + MessageType->ToString() + 
                                "\" is not an input for EFSM \"" + Name + "\"");
            }

            SymTab.Push();
            SymTab.Bind(MessageName, new MsgDecl(MessageName, MessageType));
            CheckUpdates(Updates, SymTab, TheLTS->Mgr);
            auto Scope = SymTab.Pop();
            
            auto Transition = TransitionT::MakeInputTransition(InitState,
                                                               FinalState,
                                                               Guard,
                                                               Updates,
                                                               MessageName,
                                                               MessageType);
            Transitions.push_back(pair<TransitionT, ScopeRef>(Transition, Scope));
        }

        void FrozenEFSM::AddOutputTransition(const string& InitState,
                                             const string& FinalState,
                                             const ExpT& Guard,
                                             const vector<AsgnT>& Updates,
                                             const string& MessageName,
                                             const ExprTypeRef& MessageType,
                                             const unordered_set<u32>& FairnessSet)
        {
            CheckState(InitState, States);
            CheckState(FinalState, States);
            if (Inputs.find(MessageType) == Outputs.end()) {
                throw ESMCError((string)"Message type \"" + MessageType->ToString() + 
                                "\" is not an output for EFSM \"" + Name + "\"");
            }
            
            CheckExpr(Guard, SymTab, TheLTS->Mgr);
            if (!Guard->GetType()->Is<Exprs::ExprBoolType>()) {
                throw ESMCError((string)"Guard of a transition must be boolean valued");
            }

            SymTab.Push();
            SymTab.Bind(MessageName, new MsgDecl(MessageName, MessageType));
            CheckUpdates(Updates, SymTab, TheLTS->Mgr);
            
            auto Scope = SymTab.Pop();
            auto Transition = TransitionT::MakeOutputTransition(InitState,
                                                                FinalState,
                                                                Guard,
                                                                Updates,
                                                                MessageName,
                                                                MessageType,
                                                                FairnessSet);
            Transitions.push_back(pair<TransitionT, ScopeRef>(Transition, Scope));
        }
        
        void FrozenEFSM::AddInternalTransition(const string& InitState,
                                               const string& FinalState,
                                               const ExpT& Guard,
                                               const vector<AsgnT>& Updates,
                                               const unordered_set<u32>& FairnessSet)
        {
            CheckState(InitState, States);
            CheckState(FinalState, States);
            CheckExpr(Guard, SymTab, TheLTS->Mgr);
            if (!Guard->GetType()->Is<Exprs::ExprBoolType>()) {
                throw ESMCError((string)"Guard of a transition must be boolean valued");
            }

            auto Transition = TransitionT::MakeInternalTransition(InitState,
                                                                  FinalState,
                                                                  Guard,
                                                                  Updates,
                                                                  FairnessSet);
            Transitions.push_back(pair<TransitionT, ScopeRef>(Transition, ScopeRef::NullPtr));
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

    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// UFEFSM.cpp ends here
