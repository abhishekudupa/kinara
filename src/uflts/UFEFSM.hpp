// UFEFSM.hpp --- 
// 
// Filename: UFEFSM.hpp
// Author: Abhishek Udupa
// Created: Wed Jul 23 19:33:33 2014 (-0400)
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

#if !defined ESMC_UF_EFSM_HPP_
#define ESMC_UF_EFSM_HPP_

#include "../common/FwdDecls.hpp"
#include "../utils/UIDGenerator.hpp"
#include "../expr/Expressions.hpp"
#include "../symexec/Analyses.hpp"

#include "SymbolTable.hpp"
#include "Transitions.hpp"
#include "LTSTermSemanticizer.hpp"
#include "UFLTSExtension.hpp"

namespace ESMC {
    namespace LTS {

        class UFLTS;
        typedef Exprs::ExprTypeRef ExprTypeRef;
        typedef Exprs::Expr<UFLTSExtensionT, LTSTermSemanticizer> ExpT;
        typedef Exprs::ExprMgr<UFLTSExtensionT, LTSTermSemanticizer> MgrType;
        typedef Analyses::Assignment<UFLTSExtensionT, LTSTermSemanticizer> AsgnT;

        namespace Detail {

            struct StateDescriptor
            {
                string StateName;
                bool IsInitial;
                bool IsFinal;
                bool IsAccepting;
                bool IsError;
            
                inline StateDescriptor() {}
                inline StateDescriptor(const string& StateName,
                                       bool IsInitial = false,
                                       bool IsFinal = false,
                                       bool IsAccepting = false,
                                       bool IsError = false)
                    : StateName(StateName), IsInitial(IsInitial),
                      IsFinal(IsFinal), IsAccepting(IsAccepting),
                      IsError(IsError)
                {
                    // Nothing here
                }
            
                inline bool operator == (const StateDescriptor& Other) const
                {
                    return (StateName == Other.StateName &&
                            IsInitial == Other.IsInitial &&
                            IsFinal == Other.IsFinal &&
                            IsAccepting == Other.IsAccepting &&
                            IsError == Other.IsError);
                }

                inline bool operator != (const StateDescriptor& Other) const
                {
                    return (!(*this == Other));
                }
            };
        
            struct ParametrizedTransition
            {
                bool Input;
                string InitialState;
                string FinalState;
                ExpT Guard;
                vector<AsgnT> Updates;
                string MessageName;
                vector<ExpT> Params;
                ExpT Constraint;
                ExprTypeRef MessageType;
                i32 FairnessSet;
                ScopeRef Scope;

                ParametrizedTransition() {}
                ParametrizedTransition(bool Input,
                                       const string& InitialState,
                                       const string& FinalState,
                                       const ExpT& Guard,
                                       const vector<AsgnT>& Updates,
                                       const string& MessageName,
                                       const vector<ExpT>& Params,
                                       const ExpT& Constraint,
                                       const ExprTypeRef& MessageType,
                                       i32 FairnessSet,
                                       const ScopeRef& Scope)
                : Input(Input), InitialState(InitialState),
                    FinalState(FinalState), Guard(Guard),
                    Updates(Updates), MessageName(MessageName),
                    Params(Params), Constraint(Constraint),
                    MessageType(MessageType), FairnessSet(FairnessSet),
                    Scope(Scope)
                {
                    // Nothing here
                }
            };

            struct ParametrizedMessage
            {
                ExprTypeRef PType;
                vector<ExpT> Params;
                ExpT Constraint;

                ParametrizedMessage();
                ParametrizedMessage(const ExprTypeRef& PType,
                                    const vector<ExpT>& Params,
                                    const ExpT& Constraint)
                    : PType(PType), Params(Params), Constraint(Constraint)
                {
                    // Nothing here
                }

                inline bool operator < (const ParametrizedMessage& Other) const
                {
                    return (memcmp(this, &Other, sizeof(ParametrizedMessage)) < 0);
                }
            };

            struct VarGatherer
            {
                typedef const Exprs::ExpressionBase<UFLTSExtensionT, LTSTermSemanticizer>* ExpCPtrT;
                
                inline bool operator () (ExpCPtrT Exp) const
                {
                    auto ExpAsVar = Exp->template As<Exprs::VarExpression>();
                    if (ExpAsVar == nullptr) {
                        return false;
                    }
                    // We don't want to return field access vars
                    auto Type = ExpAsVar->GetVarType();
                    auto TypeAsFA = Type->As<Exprs::ExprFieldAccessType>();
                    return (TypeAsFA == nullptr);                    
                }
            };
        } /* end namespace Detail */

        // The class for an I/O EFSM which can contain uninterpreted functions
        class UFEFSM
        {
            friend class UFLTS;

        private:
            UFLTS* TheLTS;
            string Name;
            bool Finalized;
            vector<ExpT> Params;
            ExpT Constraint;
            UIDGenerator StateUIDGenerator;

            set<ExprTypeRef> Inputs;
            set<ExprTypeRef> Outputs;

            set<Detail::ParametrizedMessage> ParametrizedInputs;
            set<Detail::ParametrizedMessage> ParametrizedOutputs;

            map<string, Detail::StateDescriptor> States;
            typedef Transition<UFLTSExtensionT, LTSTermSemanticizer, string> TransitionT;
            vector<pair<TransitionT, ScopeRef>> Transitions;
            vector<Detail::ParametrizedTransition> ParametrizedTransitions;
            
            SymbolTable SymTab;

            inline void CheckUpdates(const vector<AsgnT>& Updates) const;
            inline void CheckMsg(const ExprTypeRef& MsgType, bool Input = true) const;
            inline void CheckState(const string& StateName) const;
            inline void CheckExpr(const ExpT& Exp) const;
            inline void CheckParamPurity(const ExpT& Exp) const;
            inline void CheckParams(const vector<ExpT>& Params, 
                                    const ExpT& Constraint);

            inline UFEFSM* Instantiate(const typename MgrType::SubstMapT& SubstMap) const;
            inline vector<UFEFSM*> Instantiate();

            inline vector<MgrType::SubstMapT> InstantiateParams(const vector<ExpT>& Params,
                                                                const ExpT& Constraint) const;

        public:
            UFEFSM(UFLTS* TheLTS, const string& Name);

            UFEFSM(UFLTS* TheLTS, 
                   const string& Name,
                   const vector<ExpT>& Params,
                   const ExpT& Constraint);

            ~UFEFSM();
            
            void AddInputMsg(const ExprTypeRef& MType);
            void AddOutputMsg(const ExprTypeRef& MType);
            
            void AddInputMsg(const vector<ExpT>& Params,
                             const ExpT& Constraint,
                             const ExprTypeRef& MType);

            void AddOutputMsg(const vector<ExpT>& Params,
                              const ExpT& Constraint,
                              const ExprTypeRef& MType);

            void AddState(const string& StateName,
                          bool Initial = false,
                          bool Final = false,
                          bool Accepting = false,
                          bool Error = false);

            // Add an internal, non-initial, non-final,
            // non-accepting, non-error state
            string AddState();

            void AddVariable(const string& VarName,
                             const ExprTypeRef& VarType);
            
            void AddInputTransition(const string& InitState,
                                    const string& FinalState,
                                    const ExpT& Guard,
                                    const vector<AsgnT>& Updates,
                                    const string& MessageName,
                                    const ExprTypeRef& MessageType);
            
            // Parametrized input transition
            void AddInputTransition(const string& InitState,
                                    const string& FinalState,
                                    const ExpT& Guard,
                                    const vector<AsgnT>& Updates,
                                    const string& MessageName,
                                    const vector<ExpT>& Params,
                                    const ExpT& Constraint,
                                    const ExprTypeRef& MessageType);

            void AddOutputTransition(const string& InitState,
                                     const string& FinalState,
                                     const ExpT& Guard,
                                     const vector<AsgnT>& Updates,
                                     const string& MessageName,
                                     const ExprTypeRef& MessageType,
                                     i32 FairnessSet = -1);

            // Parametrized output transition
            void AddOutputTransition(const string& InitState,
                                     const string& FinalState,
                                     const ExpT& Guard,
                                     const vector<AsgnT>& Updates,
                                     const string& MessageName,
                                     const vector<ExpT>& Params,
                                     const ExpT& Constraint,
                                     const ExprTypeRef& MessageType,
                                     i32 FairnessSet = -1);

            void AddInternalTransition(const string& InitState,
                                       const string& FinalState,
                                       const ExpT& Guard,
                                       const vector<AsgnT>& Updates,
                                       i32 FairnessSet = -1);

        };

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_UF_EFSM_HPP_ */

// 
// UFEFSM.hpp ends here
