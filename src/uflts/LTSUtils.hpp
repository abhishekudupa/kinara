// LTSUtils.hpp --- 
// 
// Filename: LTSUtils.hpp
// Author: Abhishek Udupa
// Created: Sun Aug  3 14:28:48 2014 (-0400)
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

#if !defined ESMC_LTS_UTILS_HPP_
#define ESMC_LTS_UTILS_HPP_

#include "../common/FwdDecls.hpp"
#include "../expr/Expressions.hpp"
#include "../symexec/Analyses.hpp"
#include "../utils/CombUtils.hpp"

#include "LTSTermSemanticizer.hpp"
#include "UFLTSExtension.hpp"
#include "Transitions.hpp"
#include "SymbolTable.hpp"


namespace ESMC {
    namespace LTS {

        class UFLTS;
        class UFEFSM;
        class FrozenEFSM;
        class ChannelEFSM;

        typedef Exprs::ExprTypeRef ExprTypeRef;
        typedef Exprs::Expr<UFLTSExtensionT, LTSTermSemanticizer> ExpT;
        typedef Exprs::ExprMgr<UFLTSExtensionT, LTSTermSemanticizer> MgrType;
        typedef Analyses::Assignment<UFLTSExtensionT, LTSTermSemanticizer> AsgnT;
        typedef Transition<UFLTSExtensionT, LTSTermSemanticizer, ExprTypeRef, string> TransitionT;
        typedef Exprs::ExpressionVisitorBase<UFLTSExtensionT, LTSTermSemanticizer> VisitorBaseT;
        typedef GuardedCommand<UFLTSExtensionT, LTSTermSemanticizer> GCmdT;

        typedef Exprs::VarExpression<UFLTSExtensionT, LTSTermSemanticizer> VarExpT;
        typedef Exprs::ConstExpression<UFLTSExtensionT, LTSTermSemanticizer> ConstExpT;
        typedef Exprs::BoundVarExpression<UFLTSExtensionT, LTSTermSemanticizer> BoundVarExpT;
        typedef Exprs::OpExpression<UFLTSExtensionT, LTSTermSemanticizer> OpExpT;
        typedef Exprs::QuantifiedExpressionBase<UFLTSExtensionT, LTSTermSemanticizer> QExpBaseT;
        typedef Exprs::EQuantifiedExpression<UFLTSExtensionT, LTSTermSemanticizer> EQExpT;
        typedef Exprs::AQuantifiedExpression<UFLTSExtensionT, LTSTermSemanticizer> AQExpT;

        namespace Detail {

            struct StateDescriptor
            {
                string StateName;
                bool IsInitial;
                bool IsFinal;
                bool IsAccepting;
                bool IsError;
                bool IsDead;
            
                inline StateDescriptor() {}
                inline StateDescriptor(const string& StateName,
                                       bool IsInitial = false,
                                       bool IsFinal = false,
                                       bool IsAccepting = false,
                                       bool IsError = false,
                                       bool IsDead = false)
                    : StateName(StateName), IsInitial(IsInitial),
                      IsFinal(IsFinal), IsAccepting(IsAccepting),
                      IsError(IsError), IsDead(IsDead)
                {
                    // Nothing here
                }
            
                inline bool operator == (const StateDescriptor& Other) const
                {
                    return (StateName == Other.StateName &&
                            IsInitial == Other.IsInitial &&
                            IsFinal == Other.IsFinal &&
                            IsAccepting == Other.IsAccepting &&
                            IsError == Other.IsError &&
                            IsDead == Other.IsDead);
                }

                inline bool operator != (const StateDescriptor& Other) const
                {
                    return (!(*this == Other));
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


        // Helper functions for various classes

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
                                        MgrType* Mgr, bool IsInput,
                                        const string& MsgVarName)
        {
            for(auto const& Asgn : Updates) {
                CheckExpr(Asgn.GetLHS(), SymTab, Mgr);
                if (IsInput) {
                    auto Gatherer = Detail::VarGatherer();
                    auto Vars = Mgr->Gather(Asgn.GetLHS(), Gatherer);
                    for (auto const& Var : Vars) {
                        auto const& VarExp = Var->As<Exprs::VarExpression>();
                        auto const& VarName = VarExp->GetVarName();
                        if (VarName == MsgVarName) {
                            throw ESMCError((string)"Input message cannot be updated " + 
                                            "in an input transition");
                        }
                    }
                }
                CheckLValCompat(Asgn.GetLHS(), SymTab);
                CheckExpr(Asgn.GetRHS(), SymTab, Mgr);
                if (!IsInput) {
                    auto Gatherer = Detail::VarGatherer();
                    auto Vars = Mgr->Gather(Asgn.GetRHS(), Gatherer);
                    for (auto const& Var : Vars) {
                        auto const& VarExp = Var->As<Exprs::VarExpression>();
                        auto const& VarName = VarExp->GetVarName();
                        if (VarName == MsgVarName) {
                            throw ESMCError((string)"Output message cannot be used on RHS of " + 
                                            "assignment in an output transition");
                        }
                    }
                }
                
                // Finally check type compat
                if (!CheckAsgnCompat(Asgn.GetLHS()->GetType(),
                                     Asgn.GetRHS()->GetType())) {
                    throw ESMCError((string)"Incompatible types in assignment:\n" + 
                                    Asgn.ToString());
                }
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
                                       MgrType* Mgr, 
                                       bool NoReuse = false)
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
                    (Res->GetType() != Param->GetType() ||
                     NoReuse)) {
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

        static inline void CheckParams(const vector<ExpT>& Params,
                                       const SymbolTable& SymTab)
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
                
                if (Res == DeclRef::NullPtr) {
                    throw ESMCError((string)"Parameter \"" + VarName + "\" could not be resolved");
                }
            }
        }

        static inline vector<ExpT>
        SubstAll(const vector<ExpT>& Params, 
                 const MgrType::SubstMapT& SubstMap, MgrType* Mgr)
        {
            const u32 NumParams = Params.size();
            vector<ExpT> Retval(NumParams);
            
            for (u32 i = 0; i < NumParams; ++i) {
                Retval[i] = Mgr->Substitute(SubstMap, Params[i]);
            }
            return Retval;
        }

        static inline vector<vector<ExpT>> InstantiateParams(const vector<ExpT>& Params,
                                                             const ExpT& Constraint,
                                                             MgrType* Mgr)
        {
            vector<vector<ExpT>> Retval;
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
                    vector<ExpT> CurVec;
                    for (auto const& Param : Params) {
                        CurVec.push_back(SubstMap[Param]);
                    }
                    Retval.push_back(CurVec);
                }
            }
            return Retval;
        }

        static inline ExprTypeRef InstantiateType(const ExprTypeRef& PType,
                                                  const vector<ExpT>& Params,
                                                  MgrType* Mgr)
        {
            if (Params.size() == 0) {
                return PType; 
            } else {
                return Mgr->InstantiateType(PType, Params);
            }
        }

        static inline vector<vector<ExpT>> InstantiatePendingParams(const vector<ExpT>& Params,
                                                                    MgrType* Mgr,
                                                                    const MgrType::SubstMapT& 
                                                                    GlobalSM,
                                                                    const ExpT& Constraint)
        {
            const u32 NumParams = Params.size();

            vector<ExpT> SubstParams = Params;
            vector<vector<ExpT>> Retval;
            vector<ExpT> ParamsToInstantiate;

            for (u32 i = 0; i < NumParams; ++i) {
                SubstParams[i] = Mgr->Substitute(GlobalSM, SubstParams[i]);
                if (!SubstParams[i]->Is<Exprs::ConstExpression>()) {
                    ParamsToInstantiate.push_back(SubstParams[i]);
                }
            }
            
            auto SubstConstraint = Mgr->Substitute(GlobalSM, Constraint);
            auto&& ParamVec = InstantiateParams(ParamsToInstantiate, SubstConstraint, Mgr);
            for (auto const& ParamVal : ParamVec) {
                u32 j = 0;
                for (u32 i = 0; i < NumParams; ++i) {
                    if (!Params[i]->Is<Exprs::ConstExpression>()) {
                        SubstParams[i] = ParamVal[j++];
                    }
                }
                Retval.push_back(SubstParams);
            }
            return Retval;
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_UTILS_HPP_ */

// 
// LTSUtils.hpp ends here
