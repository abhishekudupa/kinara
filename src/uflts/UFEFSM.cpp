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

#include "../utils/CombUtils.hpp"

#include <type_traits>

namespace ESMC {
    namespace LTS {

        inline void UFEFSM::CheckUpdates(const vector<AsgnT>& Updates) const
        {
            for(auto const& Asgn : Updates) {
                CheckExpr(Asgn.GetLHS());
                
                auto const& LHSAsVar = Asgn.GetLHS()->template As<Exprs::VarExpression>();
                assert (LHSAsVar != nullptr);
                auto const& VarName = LHSAsVar->GetVarName();
                auto Lookup = SymTab.Lookup(VarName);
                assert (Lookup != DeclRef::NullPtr);
                if (!Lookup->Is<VarDecl>()) {
                    throw ESMCError((string)"Only EFSM variables can appear on LHS of " + 
                                    "assignment. \"" + VarName + "\" is not a variable");
                }
                CheckExpr(Asgn.GetRHS());
            }
        }

        inline void UFEFSM::CheckMsg(const ExprTypeRef& MsgType, bool Input) const
        {
            if ((Input && Inputs.find(MsgType) == Inputs.end()) ||
                (!Input && Outputs.find(MsgType) == Outputs.end())) {
                
                throw ESMCError((string)"Invalid input message for transition");
            }
        }

        inline void UFEFSM::CheckState(const string& StateName) const
        {
            if (States.find(StateName) == States.end()) {
                throw ESMCError((string)"Unknown state \"" + StateName + "\"");
            }
        }

        inline void UFEFSM::CheckExpr(const ExpT& Exp) const
        {
            auto Gatherer = Detail::VarGatherer();
            auto Vars = TheLTS->Mgr->Gather(Exp, Gatherer);
            for (auto const& Var : Vars) {
                auto const& VarName = Var->template SAs<Exprs::VarExpression>()->GetVarName();
                auto Lookup = SymTab.Lookup(VarName);
                if (Lookup == DeclRef::NullPtr) {
                    throw ESMCError((string)"Unbound variable \"" + VarName + "\"");
                }
            }
        }

        inline void UFEFSM::CheckParamPurity(const ExpT& Exp) const
        {
            auto Gatherer = Detail::VarGatherer();
            auto Vars = TheLTS->Mgr->Gather(Exp, Gatherer);
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

        inline void UFEFSM::CheckParams(const vector<ExpT>& Params,
                                        const ExpT& Constraint)
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
                if (SymTab.Lookup(VarName) != DeclRef::NullPtr) {
                    throw ESMCError((string)"Redeclaration of parameter \"" + VarName + "\"");
                }
                SymTab.Bind(VarName, new ParamDecl(VarName, Type));
            }

            CheckExpr(Constraint);
            CheckParamPurity(Constraint);
            if (!Constraint->GetType()->Is<Exprs::ExprBoolType>()) {
                throw ESMCError((string)"Constraints on parameters must be a boolean expression");
            }
        }

        UFEFSM::UFEFSM(UFLTS* TheLTS,
                       const string& Name,
                       const vector<ExpT>& Params,
                       const ExpT& Constraint)
            : TheLTS(TheLTS), Name(Name), Finalized(false), Params(Params),
              Constraint(Constraint)
        {
            CheckParams(Params, Constraint);
        }

        UFEFSM::~UFEFSM()
        {
            // Nothing here
        }


        void UFEFSM::AddInputMsg(const ExprTypeRef& MType)
        {
            Inputs.insert(MType);
        }

        void UFEFSM::AddInputMsg(const vector<ExpT>& Params,
                                 const ExpT& Constraint,
                                 const ExprTypeRef& MType)
        {
            ParametrizedInputs.insert(Detail::ParametrizedMessage(MType, Params, Constraint));
        }

        void UFEFSM::AddOutputMsg(const ExprTypeRef& MType)
        {
            Outputs.insert(MType);
        }

        void UFEFSM::AddOutputMsg(const vector<ExpT>& Params,
                                  const ExpT& Constraint,
                                  const ExprTypeRef& MType)
        {
            ParametrizedOutputs.insert(Detail::ParametrizedMessage(MType, Params, Constraint));
        }


        void UFEFSM::AddState(const string& StateName,
                              bool Initial,
                              bool Final,
                              bool Accepting,
                              bool Error)
        {
            Detail::StateDescriptor Desc(StateName, Initial, Final, Accepting, Error);
            auto it = States.find(StateName);
            if (it == States.end()) {
                States[StateName] = Desc;
            } else if (it->second != Desc) {
                throw ESMCError((string)"Redeclaration of state with different parameters");
            }
        }
        
        string UFEFSM::AddState()
        {
            auto StateUID = StateUIDGenerator.GetUID();
            string StateName = (string)"State_" + to_string(StateUID);
            Detail::StateDescriptor Desc(StateName);
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
            CheckMsg(MessageType);
            CheckState(InitState);
            CheckState(FinalState);

            SymTab.Push();
            SymTab.Bind(MessageName, new MsgDecl(MessageName, MessageType));
            
            CheckUpdates(Updates);

            // Guard cannot refer to message fields
            auto Scope = SymTab.Pop();

            CheckExpr(Guard);


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
            CheckState(InitState);
            CheckState(FinalState);

            SymTab.Push();
            CheckParams(Params, Constraint);
            auto Scope = SymTab.Pop();

            if (!MessageType->Is<Exprs::ExprParametricType>()) {
                throw ESMCError((string)"Non-parametric message type in parametrized " + 
                                "input transition");
            }

            // We defer the rest of the checks until later

            Detail::ParametrizedTransition Transition(true, InitState, FinalState,
                                                      Guard, Updates, MessageName,
                                                      Params, Constraint, MessageType,
                                                      -1, Scope);

            ParametrizedTransitions.push_back(Transition);
        }

        void UFEFSM::AddOutputTransition(const string& InitState,
                                         const string& FinalState,
                                         const ExpT& Guard,
                                         const vector<AsgnT>& Updates,
                                         const string& MessageName,
                                         const ExprTypeRef& MessageType,
                                         i32 FairnessSet)
        {
            CheckMsg(MessageType, false);
            CheckState(InitState);
            CheckState(FinalState);
            
            SymTab.Push();
            SymTab.Bind(MessageName, new VarDecl(MessageName, MessageType));
            
            CheckUpdates(Updates);
            auto Scope = SymTab.Pop();
            
            CheckExpr(Guard);
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
                                         i32 FairnessSet)
        {
            CheckState(InitState);
            CheckState(FinalState);
            
            SymTab.Push();
            CheckParams(Params, Constraint);
            auto Scope = SymTab.Pop();

            if (!MessageType->Is<Exprs::ExprParametricType>()) {
                throw ESMCError((string)"Non-parametric message type in parametrized " + 
                                "output transition");
            }

            // We defer the rest of the checks until later
            Detail::ParametrizedTransition Transition(false, InitState, FinalState,
                                                      Guard, Updates, MessageName,
                                                      Params, Constraint, MessageType,
                                                      FairnessSet, Scope);

            ParametrizedTransitions.push_back(Transition);
        }


        void UFEFSM::AddInternalTransition(const string& InitState,
                                           const string& FinalState,
                                           const ExpT& Guard,
                                           const vector<AsgnT>& Updates,
                                           i32 FairnessSet)
        {
            CheckState(InitState);
            CheckState(FinalState);
            CheckUpdates(Updates);
            CheckExpr(Guard);
            
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

        inline UFEFSM* UFEFSM::Instantiate(const MgrType::SubstMapT& SubstMap) const
        {
            string InstName = Name;
            for (auto const& Param : Params) {
                auto it = SubstMap.find(Param);
                assert (it != SubstMap.end());
                auto ConstExp = it->second->template As<Exprs::ConstExpression>();
                assert(ConstExp != nullptr);
                auto const& Val = ConstExp->GetConstValue();
                InstName += ((string)"[" + Val + "]");
            }
            
            UFEFSM* Retval = new UFEFSM(TheLTS, InstName);
            // Add the parametrized message types
            
        }

        inline vector<MgrType::SubstMapT> UFEFSM::InstantiateParams(const vector<ExpT>& Params,
                                                                    const ExpT& Constraint) const
        {
            auto Mgr = TheLTS->Mgr;
            vector<MgrType::SubstMapT> Retval;
            vector<vector<string>> CPElems;
            for (auto const& Param : Params) {
                CPElems.push_back(Param->GetType()->GetElements());
            }

            auto&& CPRes = CrossProduct<string>(CPElems.begin(), CPElems.end());
            vector<MgrType::SubstMapT> SubstMaps;

            for (auto const& Prod : CPRes) {
                MgrType::SubstMapT SubstMap;
                for (u32 i = 0; i < Params.size(); ++i) {
                    auto Type = Params[i]->GetType();
                    SubstMap[Params[i]] = Mgr->MakeVal(Prod[i], Type);
                }
                auto SubstConst = Mgr->Substitute(SubstMap, Constraint);
                auto SimpConst = Mgr->Simplify(SubstConst);
                auto SimpAsConst = SimpConst->As<Exprs::ConstExpression>();
                if (SimpAsConst == nullptr) {
                    throw ESMCError((string)"Did not get a constant on the constraint " + 
                                    "after substituting parameters! Perhaps the constraint " +
                                    "cannot be simplified?\nConstraint:\n" + 
                                    Constraint->ToString());
                }
                if (SimpAsConst->GetType()->As<Exprs::ExprBoolType>() == nullptr) {
                    throw InternalError((string)"Expected a boolean constant.\nAt: " + 
                                        __FILE__ + ":" + to_string(__LINE__));
                }
                if (SimpAsConst->GetConstValue() == "true") {
                    Retval.push_back(SubstMap);
                }
            }
            return Retval;
        }

        inline vector<UFEFSM*> UFEFSM::Instantiate()
        {
            vector<UFEFSM*> Retval;
            auto&& SubstMaps = InstantiateParams(Params, Constraint);
            for (auto const& SubstMap : SubstMaps) {
                Retval.push_back(Instantiate(SubstMap));
            }
            return Retval;
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// UFEFSM.cpp ends here
