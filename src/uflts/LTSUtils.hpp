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
#include "../utils/CombUtils.hpp"


#include "LTSTermSemanticizer.hpp"
#include "SymbolTable.hpp"
#include "LTSTypes.hpp"
#include "LTSAssign.hpp"


namespace ESMC {
    namespace LTS {

        namespace Detail {

            struct VarGatherer
            {
                typedef const Exprs::ExpressionBase<LTSExtensionT, LTSTermSemanticizer>* ExpCPtrT;

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

            struct SymmConstGatherer
            {
                typedef const Exprs::ExpressionBase<LTSExtensionT, LTSTermSemanticizer>* ExpCPtrT;

                inline bool operator () (ExpCPtrT Exp) const
                {
                    auto ExpAsConst = Exp->template As<Exprs::ConstExpression>();
                    if (ExpAsConst == nullptr) {
                        return false;
                    }
                    auto Type = ExpAsConst->GetConstType();
                    return (Type->Is<Exprs::ExprSymmetricType>());
                };
            };

            // Transforms all references to a particular
            // message type in an expression into a
            // unified message type reference, renaming
            // all field accesses appropriately
            class MsgTransformer : public VisitorBaseT
            {
            private:
                vector<ExpT> ExpStack;
                MgrT* Mgr;
                string MsgVarName;
                ExprTypeRef MsgRecType;
                ExprTypeRef UnifiedMType;

            public:
                MsgTransformer(MgrT* Mgr, const string& MsgVarName,
                               const ExprTypeRef& MsgRecType,
                               const ExprTypeRef& UnifiedMType);
                virtual ~MsgTransformer();

                virtual void VisitVarExpression(const VarExpT* Exp) override;
                virtual void VisitBoundVarExpression(const BoundVarExpT* Exp) override;
                virtual void VisitConstExpression(const ConstExpT* Exp) override;
                virtual void VisitOpExpression(const OpExpT* Exp) override;
                virtual inline void VisitEQuantifiedExpression(const EQExpT* Exp) override;
                virtual inline void VisitAQuantifiedExpression(const AQExpT* Exp) override;

                static ExpT Do(MgrT* Mgr,
                               const ExpT& Exp,
                               const string& MsgVarName,
                               const ExprTypeRef& MsgRecType,
                               const ExprTypeRef& UnifiedMType);
            };

            class ArrayRValueTransformer : public VisitorBaseT
            {
            private:
                vector<ExpT> ExpStack;
                MgrT* Mgr;

            public:
                ArrayRValueTransformer(MgrT* Mgr);
                virtual ~ArrayRValueTransformer();

                virtual void VisitVarExpression(const VarExpT* Exp) override;
                virtual void VisitBoundVarExpression(const BoundVarExpT* Exp) override;
                virtual void VisitConstExpression(const ConstExpT* Exp) override;
                virtual void VisitOpExpression(const OpExpT* Exp) override;
                virtual inline void VisitEQuantifiedExpression(const EQExpT* Exp) override;
                virtual inline void VisitAQuantifiedExpression(const AQExpT* Exp) override;

                static ExpT Do(MgrT* Mgr, const ExpT& Exp);
            };

            class ExpressionPermuter : public VisitorBaseT
            {
            private:
                MgrT* Mgr;
                const vector<u08>& PermVec;
                map<ExprTypeRef, u32> TypeOffsets;
                vector<ExpT> ExpStack;

            public:
                ExpressionPermuter(MgrT* Mgr, const vector<u08>& PermVec,
                                   const map<ExprTypeRef, u32>& TypeOffsets);
                virtual ~ExpressionPermuter();

                virtual void VisitVarExpression(const VarExpT* Exp) override;
                virtual void VisitBoundVarExpression(const BoundVarExpT* Exp) override;
                virtual void VisitConstExpression(const ConstExpT* Exp) override;
                virtual void VisitOpExpression(const OpExpT* Exp) override;
                virtual inline void VisitEQuantifiedExpression(const EQExpT* Exp) override;
                virtual inline void VisitAQuantifiedExpression(const AQExpT* Exp) override;

                static ExpT Do(MgrT* Mgr, const ExpT& Exp,
                               const vector<u08>& PermVec,
                               const map<ExprTypeRef, u32>& TypeOffsets);

            };

        } /* end namespace Detail */


        // Helper functions for various classes
        static inline void CheckParamPurity(const ExpT& Exp,
                                            const SymbolTable& SymTab,
                                            MgrT* Mgr)
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

        static inline void CheckExpr(const ExpT& Exp,
                                     const SymbolTable& SymTab,
                                     MgrT* Mgr)
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

        static inline void CheckParams(const vector<ExpT>& Params,
                                       const ExpT& Constraint,
                                       SymbolTable& SymTab,
                                       MgrT* Mgr,
                                       bool NoReuse = false)
        {
            for (auto const& Param : Params) {
                auto ParamAsVar = Param->As<Exprs::VarExpression>();
                if (ParamAsVar == nullptr) {
                    throw ESMCError((string)"Parameters to EFSMS must be variable expressions");
                }
                auto Type = ParamAsVar->GetVarType();
                if ((!Type->Is<Exprs::ExprSymmetricType>())) {
                    // TODO: Extend this to range types as well some day.
                    throw ESMCError((string)"Parameter types must be symmetric types");
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
                if (!LookupRes->Is<VarDecl>() &&
                    !LookupRes->Is<OutMsgDecl>()) {
                    throw ESMCError((string)"Error: Parameters cannot be used as LVals");
                }
            }
        }

        static inline void CheckUpdates(const vector<LTSAssignRef>& Updates,
                                        SymbolTable& SymTab,
                                        MgrT* Mgr, bool IsInput,
                                        const string& MsgVarName)
        {
            for(auto const& Asgn : Updates) {
                auto PUpdate = Asgn->As<LTSAssignParam>();
                if (PUpdate != nullptr) {
                    auto const& Params = PUpdate->GetParams();
                    auto const& Constraint = PUpdate->GetConstraint();

                    SymTab.Push();
                    CheckParams(Params, Constraint, SymTab, Mgr, true);
                }

                CheckExpr(Asgn->GetLHS(), SymTab, Mgr);
                if (IsInput) {
                    auto Gatherer = Detail::VarGatherer();
                    auto Vars = Mgr->Gather(Asgn->GetLHS(), Gatherer);
                    for (auto const& Var : Vars) {
                        auto const& VarExp = Var->As<Exprs::VarExpression>();
                        auto const& VarName = VarExp->GetVarName();
                        if (VarName == MsgVarName) {
                            throw ESMCError((string)"Input message cannot be updated " +
                                            "in an input transition");
                        }
                    }
                }
                CheckLValCompat(Asgn->GetLHS(), SymTab);
                CheckExpr(Asgn->GetRHS(), SymTab, Mgr);
                if (!IsInput) {
                    auto Gatherer = Detail::VarGatherer();
                    auto Vars = Mgr->Gather(Asgn->GetRHS(), Gatherer);
                    for (auto const& Var : Vars) {
                        auto const& VarExp = Var->As<Exprs::VarExpression>();
                        auto const& VarName = VarExp->GetVarName();
                        if (VarName == MsgVarName) {
                            throw ESMCError((string)"Output message cannot be used on RHS of " +
                                            "assignment in an output transition");
                        }
                    }
                }

                // Check that no constants of symmetric types
                // appear in the assignment
                // audupa: This is now taken care of in the LabelledTS
                //         commented out on 10/16/2014
                // auto SymmConstsLHS = Mgr->Gather(Asgn->GetLHS(), Detail::SymmConstGatherer());
                // auto SymmConstsRHS = Mgr->Gather(Asgn->GetRHS(), Detail::SymmConstGatherer());
                // if (SymmConstsLHS.size() != 0 || SymmConstsRHS.size() != 0) {
                //     if (!(SymmConstsLHS.size() == 0 &&
                //           Asgn->GetRHS()->Is<Exprs::ConstExpression>() &&
                //           (Asgn->GetRHS()->SAs<Exprs::ConstExpression>()->GetConstValue() ==
                //            "clear") &&
                //           SymmConstsRHS.size() == 1)) {
                //         throw ESMCError((string)"Assignments cannot contain symmetric constant " +
                //                         "literal values. This breaks symmetry!");
                //     }
                // }

                // Finally check type compat
                if (!CheckAsgnCompat(Asgn->GetLHS()->GetType(),
                                     Asgn->GetRHS()->GetType())) {
                    throw ESMCError((string)"Incompatible types in assignment:\n" +
                                    Asgn->ToString());
                }

                if (PUpdate != nullptr) {
                    SymTab.Pop();
                }
            }
        }

        static inline vector<ExpT>
        SubstAll(const vector<ExpT>& Params,
                 const MgrT::SubstMapT& SubstMap, MgrT* Mgr)
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
                                                             MgrT* Mgr)
        {
            vector<vector<ExpT>> Retval;
            if (Params.size() == 0) {
                Retval.push_back(vector<ExpT>());
                return Retval;
            }

            vector<vector<string>> CPElems;

            for (auto const& Param : Params) {
                CPElems.push_back(Param->GetType()->GetElementsNoUndef());
            }

            auto&& CPRes = CrossProduct<string>(CPElems.begin(), CPElems.end());
            vector<MgrT::SubstMapT> SubstMaps;

            for (auto const& Prod : CPRes) {
                MgrT::SubstMapT SubstMap;
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
                                                  MgrT* Mgr)
        {
            if (Params.size() == 0) {
                return PType;
            } else {
                return Mgr->InstantiateType(PType, Params);
            }
        }

        static inline vector<vector<ExpT>> InstantiatePendingParams(const vector<ExpT>& Params,
                                                                    MgrT* Mgr,
                                                                    const MgrT::SubstMapT&
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

        static inline void ExpandExpression(const ExpT& Exp, set<ExpT>& Expansions)
        {
            auto VarType = Exp->GetType();
            auto Mgr = Exp->GetMgr();
            if (VarType->Is<Exprs::ExprScalarType>()) {
                Expansions.insert(Exp);
                return;
            }

            if (VarType->Is<Exprs::ExprRecordType>() ||
                VarType->Is<Exprs::ExprParametricType>()) {

                auto BaseType = (VarType->Is<ExprRecordType>() ?
                                 VarType :
                                 VarType->SAs<Exprs::ExprParametricType>()->GetBaseType());

                auto TypeAsRec = BaseType->SAs<Exprs::ExprRecordType>();
                auto const& Fields = TypeAsRec->GetMemberVec();
                auto FAType = Mgr->MakeType<ExprFieldAccessType>();

                for (auto const& Field : Fields) {
                    auto FAVar = Mgr->MakeVar(Field.first, FAType);
                    auto CurExpansion = Mgr->MakeExpr(LTSOps::OpField,
                                                      Exp, FAVar);
                    ExpandExpression(CurExpansion, Expansions);
                }
                return;
            }

            if (VarType->Is<Exprs::ExprArrayType>()) {
                auto TypeAsArray = VarType->SAs<Exprs::ExprArrayType>();
                auto const& IndexType = TypeAsArray->GetIndexType();
                auto const& IndexElems = IndexType->GetElementsNoUndef();

                for (auto const& IndexElem : IndexElems) {
                    auto IndexExp = Mgr->MakeVal(IndexElem, IndexType);
                    auto CurExpansion = Mgr->MakeExpr(LTSOps::OpIndex,
                                                      Exp, IndexExp);
                    ExpandExpression(CurExpansion, Expansions);
                }
                return;
            }

            // Eh?
            return;
        }

        static inline vector<ExpT> GetScalarTerms(const ExpT& Exp)
        {
            set<ExpT> ExpandedTerms;
            ExpandExpression(Exp, ExpandedTerms);
            return vector<ExpT>(ExpandedTerms.begin(), ExpandedTerms.end());
        }

        static inline MgrT::ExpSetT GetSynthExps(const ExpT& Exp)
        {
            auto Mgr = Exp->GetMgr();
            return Mgr->Gather(Exp,
                               [&] (const ExpBaseT* Exp) -> bool
                               {
                                   auto ExpAsOp = Exp->As<OpExpression>();
                                   return (ExpAsOp != nullptr &&
                                           LTSReservedOps.find(ExpAsOp->GetOpCode()) ==
                                           LTSReservedOps.end());
                               });
        }

        static inline unordered_set<i64> GetSynthOps(const ExpT& Exp)
        {
            unordered_set<i64> Retval;
            auto SynthExps = GetSynthExps(Exp);
            for (auto const& SynthExp : SynthExps) {
                auto ExpAsOp = SynthExp->As<OpExpression>();
                if (ExpAsOp == nullptr) {
                    throw InternalError((string)"Expected an op expression.\nAt: " +
                                        __FILE__ + ":" + to_string(__LINE__));
                }
                Retval.insert(ExpAsOp->GetOpCode());
            }
            return Retval;
        }

        static ExpT GetTopmostIndexExp(const ExpT& Exp)
        {
            if (Exp->Is<VarExpression>() ||
                Exp->Is<ConstExpression>() ||
                Exp->Is<BoundVarExpression>()) {
                return ExpT::NullPtr;
            }

            if (Exp->Is<QuantifiedExpressionBase>()) {
                throw ESMCError((string)"Expected quantifiers to be unrolled before " +
                                "call to GetTopmostIndexExp()");
            }

            auto ExpAsOp = Exp->As<OpExpression>();
            auto const& Children = ExpAsOp->GetChildren();
            const u32 NumChildren = Children.size();

            if (ExpAsOp->GetOpCode() == LTSOps::OpIndex) {
                if (!Children[1]->Is<ConstExpression>()) {
                    return Exp;
                }
            }

            for (u32 i = 0; i < NumChildren; ++i) {
                auto Res = GetTopmostIndexExp(Children[i]);
                if (Res != ExpT::NullPtr) {
                    return Res;
                }
            }

            return ExpT::NullPtr;
        }

        static ExpT TransformArrayRValue(const ExpT Exp)
        {
            auto Mgr = Exp->GetMgr();
            auto IndexExp = GetTopmostIndexExp(Exp);
            if (IndexExp == ExpT::NullPtr) {
                return Exp;
            }

            auto IndexExpAsOp = IndexExp->SAs<OpExpression>();
            IndexExp = IndexExpAsOp->GetChildren()[1];
            auto ArrayExp = IndexExpAsOp->GetChildren()[0];
            auto ArrayType = ArrayExp->GetType()->SAs<ExprArrayType>();
            auto IndexType = ArrayType->GetIndexType();
            auto const& IndexElems = IndexType->GetElementsNoUndef();
            const u32 NumElems = IndexElems.size();
            IndexType = IndexExp->GetType();

            ExpT ResExp = ExpT::NullPtr;

            if (NumElems == 1) {
                MgrT::SubstMapT SubstMap;
                SubstMap[IndexExp] = Mgr->MakeVal(IndexElems[0], IndexType);
                ResExp = Mgr->TermSubstitute(SubstMap, Exp);
            } else {
                MgrT::SubstMapT SubstMap;
                SubstMap[IndexExp] = Mgr->MakeVal(IndexElems.back(), IndexType);
                auto AccExp = Mgr->TermSubstitute(SubstMap, Exp);
                for (u32 i = 0; i < NumElems - 1; ++i) {
                    SubstMap.clear();
                    auto CurVal = Mgr->MakeVal(IndexElems[NumElems - i - 2], IndexType);
                    SubstMap[IndexExp] = CurVal;
                    AccExp = Mgr->MakeExpr(LTSOps::OpITE,
                                           Mgr->MakeExpr(LTSOps::OpEQ, IndexExp, CurVal),
                                           Mgr->TermSubstitute(SubstMap, Exp),
                                           AccExp);
                }
                ResExp = AccExp;
            }

            return TransformArrayRValue(ResExp);
        }

        static inline void GetTopLevelIndexExps(const ExpT& Exp,
                                                set<ExpT>& Indices)
        {
            if (Exp->Is<VarExpression>() ||
                Exp->Is<ConstExpression>() ||
                Exp->Is<BoundVarExpression>()) {
                return;
            }

            if (Exp->Is<QuantifiedExpressionBase>()) {
                throw ESMCError((string)"Expected quantifiers to be unrolled before " +
                                "call to GetTopmostIndexExp()");
            }

            auto ExpAsOp = Exp->As<OpExpression>();
            auto const& Children = ExpAsOp->GetChildren();
            if (ExpAsOp->GetOpCode() == LTSOps::OpIndex) {
                if (!Children[1]->Is<ConstExpression>()) {
                    Indices.insert(Exp);
                }
                GetTopLevelIndexExps(Children[0], Indices);
            } else {
                for (auto const& Child : Children) {
                    GetTopLevelIndexExps(Child, Indices);
                }
            }
        }

        static inline vector<LTSAssignRef>
        ExpandOneArrayUpdate(const LTSAssignRef& Update)
        {
            set<ExpT> TopLevelIndices;
            vector<LTSAssignRef> Retval;
            auto Mgr = Update->GetLHS()->GetMgr();

            GetTopLevelIndexExps(Update->GetLHS(), TopLevelIndices);

            if (TopLevelIndices.size() == 0) {
                Retval.push_back(Update);
                return Retval;
            }

            vector<vector<string>> CPTuples;
            for (auto const& Exp : TopLevelIndices) {
                auto ExpAsOp = Exp->SAs<OpExpression>();
                auto ArrayExp = ExpAsOp->GetChildren()[0];
                auto ArrayType = ArrayExp->GetType()->As<ExprArrayType>();
                auto Type = ArrayType->GetIndexType();
                CPTuples.push_back(Type->GetElementsNoUndef());
            }

            auto&& CPElems = CrossProduct<string>(CPTuples.begin(), CPTuples.end());
            for (auto const& ProdTuple : CPElems) {
                MgrT::SubstMapT SubstMap;
                vector<ExpT> Conjuncts;
                u32 i = 0;
                for (auto const& Exp : TopLevelIndices) {
                    auto ExpAsOp = Exp->SAs<OpExpression>();
                    auto ArrayExp = ExpAsOp->GetChildren()[0];
                    auto ArrayType = ArrayExp->GetType()->As<ExprArrayType>();
                    auto IndexType = ArrayType->GetIndexType();
                    auto Idx = ExpAsOp->GetChildren()[1];
                    auto CurVal = Mgr->MakeVal(ProdTuple[i], IndexType);
                    auto Constraint = Mgr->MakeExpr(LTSOps::OpEQ, Idx, CurVal);
                    Conjuncts.push_back(Constraint);
                    SubstMap[Idx] = CurVal;
                    ++i;
                }

                ExpT Guard = ExpT::NullPtr;
                if (Conjuncts.size() == 1) {
                    Guard = Conjuncts[0];
                } else {
                    Guard = Mgr->MakeExpr(LTSOps::OpAND, Conjuncts);
                }

                auto NewLHS = Mgr->TermSubstitute(SubstMap, Update->GetLHS());
                auto NewRHS = Mgr->MakeExpr(LTSOps::OpITE,
                                            Guard, Update->GetRHS(),
                                            NewLHS);
                auto NewNewRHS = TransformArrayRValue(NewRHS);
                Retval.push_back(new LTSAssignSimple(NewLHS, NewNewRHS));
            }

            return Retval;
        }

        static inline vector<LTSAssignRef>
        ExpandArrayUpdates(const vector<LTSAssignRef>& Updates)
        {
            vector<LTSAssignRef> Retval;
            for (auto const& Update : Updates) {
                auto&& CurExpansions = ExpandOneArrayUpdate(Update);
                Retval.insert(Retval.end(), CurExpansions.begin(), CurExpansions.end());
            }
            return Retval;
        }

        static inline ExpT MakeConjunction(const vector<ExpT>& Conjuncts,
                                           MgrT* Mgr)
        {
            if (Conjuncts.size() == 0) {
                return Mgr->MakeTrue();
            } else if (Conjuncts.size() == 1) {
                return Conjuncts[0];
            } else {
                return Mgr->MakeExpr(LTSOps::OpAND, Conjuncts);
            }
        }

        static inline ExpT MakeDisjunction(const vector<ExpT>& Disjuncts,
                                           MgrT* Mgr)
        {
            if (Disjuncts.size() == 0) {
                return Mgr->MakeFalse();
            } else if (Disjuncts.size() == 1) {
                return Disjuncts[0];
            } else {
                return Mgr->MakeExpr(LTSOps::OpOR, Disjuncts);
            }
        }

        static inline ExpT MakeSum(const vector<ExpT>& Summands,
                                   MgrT* Mgr, const ExprTypeRef& Type)
        {
            if (Summands.size() == 0) {
                return Mgr->MakeVal("0", Type);
            } else if (Summands.size() == 1) {
                return Summands[0];
            } else {
                return Mgr->MakeExpr(LTSOps::OpADD, Summands);
            }
        }

        static inline ExpT GetBaseLValue(const ExpT& Exp)
        {
            auto ExpAsVar = Exp->As<VarExpression>();
            if (ExpAsVar != nullptr) {
                return Exp;
            }

            auto ExpAsOp = Exp->As<OpExpression>();
            if (ExpAsOp == nullptr) {
                throw InternalError((string)"GetBaseLValue() called on an expression " +
                                    "that doesn't look like an LValue:\n" + Exp->ToString() +
                                    "\nAt: " + __FILE__ + ":" + to_string(__LINE__));
            }
            auto OpCode = ExpAsOp->GetOpCode();
            if (OpCode == LTSOps::OpField || OpCode == LTSOps::OpIndex) {
                return GetBaseLValue(ExpAsOp->GetChildren()[0]);
            } else {
                throw InternalError((string)"GetBaseLValue() called on an expression " +
                                    "that doesn't look like an LValue:\n" + Exp->ToString() +
                                    "\nAt: " + __FILE__ + ":" + to_string(__LINE__));
            }
        }

        static inline const vector<ExpT>& GetOpArgs(const ExpT& Exp)
        {
            vector<ExpT> Retval;
            auto ExpAsOp = Exp->As<OpExpression>();
            if (ExpAsOp == nullptr) {
                throw InternalError((string)"GetAppArgs() called on what " +
                                    "does not look like an OpExpression.\n" +
                                    "At: " + __FILE__ + ":" + to_string(__LINE__));
            }
            return ExpAsOp->GetChildren();
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_UTILS_HPP_ */

//
// LTSUtils.hpp ends here
