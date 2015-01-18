// LTSTransformers.cpp ---
// Filename: LTSTransformers.cpp
// Author: Abhishek Udupa
// Created: Mon Jan  5 00:12:58 2015 (-0500)
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

#include "LTSTransformers.hpp"
#include "LTSDecls.hpp"
#include "LTSUtils.hpp"

namespace ESMC {
    namespace LTS {
        namespace Detail {

            MsgTransformer::MsgTransformer(MgrT* Mgr, const string& MsgVarName,
                                           const TypeRef& MsgRecType,
                                           const TypeRef& UnifiedMType)
                : VisitorBaseT("MessageTransformer"),
                  Mgr(Mgr), MsgVarName(MsgVarName), MsgRecType(MsgRecType),
                  UnifiedMType(UnifiedMType)
            {
                // Nothing here
            }

            MsgTransformer::~MsgTransformer()
            {
                // Nothing here
            }

            void MsgTransformer::VisitVarExpression(const VarExpT* Exp)
            {
                if (Exp->GetVarName() == MsgVarName &&
                    Exp->GetVarType() == MsgRecType) {
                    ExpStack.push(Mgr->MakeVar(Exp->GetVarName(), UnifiedMType));
                } else {
                    ExpStack.push(Exp);
                }
            }

            void MsgTransformer::VisitBoundVarExpression(const BoundVarExpT* Exp)
            {
                ExpStack.push(Exp);
            }

            void MsgTransformer::VisitConstExpression(const ConstExpT* Exp)
            {
                ExpStack.push(Exp);
            }

            void MsgTransformer::VisitOpExpression(const OpExpT* Exp)
            {
                VisitorBaseT::VisitOpExpression(Exp);

                auto const& OldChildren = Exp->GetChildren();
                const u32 NumChildren = OldChildren.size();
                vector<ExpT> NewChildren(NumChildren);
                for (u32 i = 0; i < NumChildren; ++i) {
                    NewChildren[NumChildren - i - 1] = ExpStack.top();
                    ExpStack.pop();
                }

                auto OpCode = Exp->GetOpCode();
                if (OpCode == LTSOps::OpField &&
                    OldChildren[0]->Is<Exprs::VarExpression>() &&
                    OldChildren[0]->As<Exprs::VarExpression>()->GetVarName() == MsgVarName &&
                    OldChildren[0]->As<Exprs::VarExpression>()->GetVarType() == MsgRecType) {

                    TypeRef ActMsgRecType = nullptr;
                    if (MsgRecType->Is<RecordType>()) {
                        ActMsgRecType = MsgRecType;
                    } else if (MsgRecType->Is<ParametricType>()) {
                        ActMsgRecType = MsgRecType->SAs<ParametricType>()->GetBaseType();
                    } else {
                        throw ESMCError((string)"MsgTransformer: Message type \"" +
                                        MsgRecType->ToString() + "\" is not a parametric " +
                                        "type or a record type");
                    }

                    auto MTypeAsUnion = UnifiedMType->As<UnionType>();
                    auto FieldVarExp = OldChildren[1]->As<VarExpression>();
                    auto const& OldFieldName = FieldVarExp->GetVarName();
                    auto const& NewFieldName = MTypeAsUnion->MapFromMemberField(ActMsgRecType,
                                                                                OldFieldName);
                    auto FAType = Mgr->MakeType<FieldAccessType>();
                    auto NewFieldVar = Mgr->MakeVar(NewFieldName, FAType);
                    ExpStack.push(Mgr->MakeExpr(LTSOps::OpField, NewChildren[0], NewFieldVar));
                } else {
                    ExpStack.push(Mgr->MakeExpr(OpCode, NewChildren));
                }
            }

            void MsgTransformer::VisitEQuantifiedExpression(const EQExpT* Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto NewQExpr = ExpStack.top();
                ExpStack.pop();
                ExpStack.push(Mgr->MakeExists(Exp->GetQVarTypes(), NewQExpr));
            }

            void MsgTransformer::VisitAQuantifiedExpression(const AQExpT* Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto NewQExpr = ExpStack.top();
                ExpStack.pop();
                ExpStack.push(Mgr->MakeForAll(Exp->GetQVarTypes(), NewQExpr));
            }

            ExpT MsgTransformer::Do(MgrT* Mgr,
                                    const ExpT& Exp,
                                    const string& MsgVarName,
                                    const TypeRef& MsgRecType,
                                    const TypeRef& UnifiedMType)
            {
                MsgTransformer TheTransformer(Mgr, MsgVarName, MsgRecType, UnifiedMType);
                Exp->Accept(&TheTransformer);
                return TheTransformer.ExpStack.top();
            }

            ExpressionPermuter::ExpressionPermuter(MgrT* Mgr, const vector<u08>& PermVec,
                                                   const map<TypeRef, u32>& TypeOffsets)
                : VisitorBaseT("ExpressionPermuter"),
                  Mgr(Mgr), PermVec(PermVec), TypeOffsets(TypeOffsets)
            {
                // Nothing here
            }

            ExpressionPermuter::~ExpressionPermuter()
            {
                // Nothing here
            }

            void ExpressionPermuter::VisitVarExpression(const VarExpT* Exp)
            {
                ExpStack.push(Exp);
            }

            void ExpressionPermuter::VisitConstExpression(const ConstExpT* Exp)
            {
                auto const& Type = Exp->GetType();
                if (!Type->Is<SymmetricType>()) {
                    ExpStack.push(Exp);
                    return;
                }

                auto TypeAsSym = Type->SAs<SymmetricType>();
                // Symmetric type. Permute
                auto const& ConstVal = Exp->GetConstValue();
                auto ConstIdx = TypeAsSym->GetMemberIdx(ConstVal);
                if (ConstIdx == 0) {
                    // the undef value permutes to itself
                    ExpStack.push(Exp);
                    return;
                }

                // otherwise, decrease the index by 1
                --ConstIdx;

                auto it = TypeOffsets.find(Type);

                if (it == TypeOffsets.end()) {
                    throw ESMCError((string)"Could not find offset for type: " +
                                    Type->ToString() + "\nIn Expression Permuter, on " +
                                    "expression:\n" + Exp->ToString());
                }
                auto Offset = it->second;
                auto PermutedIdx = PermVec[Offset + ConstIdx] + 1;
                auto const& PermutedVal = TypeAsSym->GetMember(PermutedIdx);
                ExpStack.push(Mgr->MakeVal(PermutedVal, Type));
            }

            void ExpressionPermuter::VisitBoundVarExpression(const BoundVarExpT* Exp)
            {
                ExpStack.push(Exp);
            }

            void ExpressionPermuter::VisitOpExpression(const OpExpT* Exp)
            {
                VisitorBaseT::VisitOpExpression(Exp);

                auto OpCode = Exp->GetOpCode();
                auto const& Children = Exp->GetChildren();
                const u32 NumChildren = Children.size();

                vector<ExpT> NewChildren(NumChildren);

                for (u32 i = 0; i < NumChildren; ++i) {
                    NewChildren[NumChildren - i - 1] = ExpStack.top();
                    ExpStack.pop();
                }

                ExpStack.push(Mgr->MakeExpr(OpCode, NewChildren));
            }

            void ExpressionPermuter::VisitEQuantifiedExpression(const EQExpT* Exp)
            {
                auto const& QVarTypes = Exp->GetQVarTypes();
                auto const& QExpr = Exp->GetQExpression();

                QExpr->Accept(this);

                auto NewExp = ExpStack.top();
                ExpStack.pop();
                ExpStack.push(Mgr->MakeExists(QVarTypes, NewExp));
            }

            void ExpressionPermuter::VisitAQuantifiedExpression(const AQExpT* Exp)
            {
                auto const& QVarTypes = Exp->GetQVarTypes();
                auto const& QExpr = Exp->GetQExpression();

                QExpr->Accept(this);

                auto NewExp = ExpStack.top();
                ExpStack.pop();
                ExpStack.push(Mgr->MakeForAll(QVarTypes, NewExp));
            }

            ExpT ExpressionPermuter::Do(MgrT* Mgr, const ExpT& Exp,
                                        const vector<u08>& PermVec,
                                        const map<TypeRef, u32>& TypeOffsets)
            {
                ExpressionPermuter ThePermuter(Mgr, PermVec, TypeOffsets);
                Exp->Accept(&ThePermuter);
                return ThePermuter.ExpStack.top();
            }


            // Implementation of ArrayRValueTransformer
            ArrayRValueTransformer::ArrayRValueTransformer(MgrT* Mgr)
                : VisitorBaseT("ArrayRValueTransformer"), Mgr(Mgr)
            {
                // Nothing here
            }

            ArrayRValueTransformer::~ArrayRValueTransformer()
            {
                // Nothing here
            }

            void ArrayRValueTransformer::VisitVarExpression(const VarExpT* Exp)
            {
                ExpStack.push(Exp);
            }

            void ArrayRValueTransformer::VisitBoundVarExpression(const BoundVarExpT* Exp)
            {
                ExpStack.push(Exp);
            }

            void ArrayRValueTransformer::VisitConstExpression(const ConstExpT* Exp)
            {
                ExpStack.push(Exp);
            }

            void ArrayRValueTransformer::VisitOpExpression(const OpExpT* Exp)
            {
                auto OpCode = Exp->GetOpCode();
                auto const& Children = Exp->GetChildren();
                const u32 NumChildren = Children.size();
                vector<ExpT> NewChildren(NumChildren);

                for (u32 i = 0; i < NumChildren; ++i) {
                    Children[i]->Accept(this);
                    NewChildren[i] = ExpStack.top();
                    ExpStack.pop();
                }

                if (OpCode == LTSOps::OpIndex) {
                    // Transform this into a select expression
                    ExpStack.push(Mgr->MakeExpr(LTSOps::OpSelect, NewChildren[0],
                                                     NewChildren[1]));
                } else if (OpCode == LTSOps::OpField) {
                    // Transform to a project expression
                    ExpStack.push(Mgr->MakeExpr(LTSOps::OpProject, NewChildren[0],
                                                     NewChildren[1]));
                } else {
                    ExpStack.push(Mgr->MakeExpr(OpCode, NewChildren));
                }
            }

            inline void ArrayRValueTransformer::VisitQuantifiedExpression(const QExpT* Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto NewQExpr = ExpStack.top();
                ExpStack.pop();
                if (Exp->IsForAll()) {
                    ExpStack.push(Mgr->MakeForAll(Exp->GetQVarTypes(), NewQExpr));
                } else {
                    ExpStack.push(Mgr->MakeExists(Exp->GetQVarTypes(), NewQExpr));
                }
            }

            void ArrayRValueTransformer::VisitAQuantifiedExpression(const AQExpT* Exp)
            {
                VisitQuantifiedExpression(Exp);
            }

            void ArrayRValueTransformer::VisitEQuantifiedExpression(const EQExpT* Exp)
            {
                VisitQuantifiedExpression(Exp);
            }

            ExpT ArrayRValueTransformer::Do(MgrT* Mgr, const ExpT& Exp)
            {
                ArrayRValueTransformer TheTransformer(Mgr);
                Exp->Accept(&TheTransformer);
                return TheTransformer.ExpStack.top();
            }


            // UFIndexExpGatherer implementation
            UFIndexExpGatherer::UFIndexExpGatherer(set<pair<ExpT, TypeRef>>& UFIndexExps)
                : VisitorBaseT("UFIndexExpGatherer"),
                  UFIndexExps(UFIndexExps)
            {
                // Nothing here
            }

            UFIndexExpGatherer::~UFIndexExpGatherer()
            {
                // Nothing here
            }

            void UFIndexExpGatherer::VisitOpExpression(const OpExpT* Exp)
            {
                auto OpCode = Exp->GetOpCode();
                auto const& Children = Exp->GetChildren();

                VisitorBaseT::VisitOpExpression(Exp);
                if (OpCode == LTSOps::OpSelect || OpCode == LTSOps::OpStore) {
                    auto&& SynthOps = GetSynthOps(Children[1]);
                    if (SynthOps.size() > 0) {
                        auto ArrType = Children[0]->GetType()->As<ArrayType>();
                        auto const& IndexType = ArrType->GetIndexType();
                        UFIndexExps.insert(make_pair(Children[1], IndexType));
                    }
                }
            }

            void UFIndexExpGatherer::VisitEQuantifiedExpression(const EQExpT* Exp)
            {
                Exp->GetQExpression()->Accept(this);
            }

            void UFIndexExpGatherer::VisitAQuantifiedExpression(const AQExpT* Exp)
            {
                Exp->GetQExpression()->Accept(this);
            }

            void UFIndexExpGatherer::Do(const ExpT& Exp, set<pair<ExpT, TypeRef>>& UFIndexExps)
            {
                UFIndexExpGatherer TheGatherer(UFIndexExps);
                Exp->Accept(&TheGatherer);
                return;
            }

            // ConstraintPurifier implementation
            ConstraintPurifier::ConstraintPurifier(MgrT* Mgr, FastExpSetT& Assumptions)
                : VisitorBaseT("ConstraintPurifier"),
                  Mgr(Mgr), Assumptions(Assumptions)
            {
                // Nothing here
            }

            ConstraintPurifier::~ConstraintPurifier()
            {
                // Nothing here
            }

            inline vector<pair<ExpT, ExpT> >
            ConstraintPurifier::MakeITEBranches(const ExpT& Exp,
                                                const set<pair<ExpT, TypeRef>>& UFIndexExps)
            {
                vector<pair<ExpT, TypeRef>> UFIndexExpVec(UFIndexExps.begin(),
                                                          UFIndexExps.end());
                const u32 NumUFExps = UFIndexExpVec.size();

                vector<vector<string>> ValueVectors;

                for (auto const& IndexExpType : UFIndexExpVec) {
                    auto const& ExpType = IndexExpType.second;
                    ValueVectors.push_back(ExpType->GetElementsNoUndef());
                }

                auto&& CPTuples = CrossProduct<string>(ValueVectors.begin(),
                                                       ValueVectors.end());

                const u32 NumTuples = CPTuples.size();
                vector<pair<ExpT, ExpT>> Retval(NumTuples);

                for (u32 i = 0; i < NumTuples; ++i) {
                    auto const& Tuple = CPTuples[i];
                    vector<ExpT> Conjuncts(NumUFExps);
                    MgrT::SubstMapT SubstMap;

                    for (u32 j = 0; j < NumUFExps; ++j) {
                        auto const& CurUFIndexExp = UFIndexExpVec[j];
                        auto const& CurIndexExp = CurUFIndexExp.first;
                        auto const& CurIndexType = CurUFIndexExp.second;

                        auto CurVal = Mgr->MakeVal(Tuple[j], CurIndexType);
                        auto UFEQVal = Mgr->MakeExpr(LTSOps::OpEQ, CurIndexExp, CurVal);
                        Conjuncts[j] = UFEQVal;
                        SubstMap[CurIndexExp] = CurVal;
                    }

                    auto Condition = MakeConjunction(Conjuncts, Mgr);
                    auto Branch = Mgr->TermSubstitute(SubstMap, Exp);
                    Retval[i] = make_pair(Condition, Branch);
                }
                return Retval;
            }

            void ConstraintPurifier::VisitVarExpression(const VarExpT* Exp)
            {
                ExpStack.push(Exp);
            }

            void ConstraintPurifier::VisitConstExpression(const ConstExpT* Exp)
            {
                ExpStack.push(Exp);
            }

            void ConstraintPurifier::VisitBoundVarExpression(const BoundVarExpT* Exp)
            {
                ExpStack.push(Exp);
            }

            void ConstraintPurifier::VisitEQuantifiedExpression(const EQExpT* Exp)
            {
                auto const& QVarTypes = Exp->GetQVarTypes();
                Exp->GetQExpression()->Accept(this);
                auto NewBody = ExpStack.top();
                ExpStack.pop();
                ExpStack.push(Mgr->MakeExists(QVarTypes, NewBody));
            }

            void ConstraintPurifier::VisitAQuantifiedExpression(const AQExpT* Exp)
            {
                auto const& QVarTypes = Exp->GetQVarTypes();
                Exp->GetQExpression()->Accept(this);
                auto NewBody = ExpStack.top();
                ExpStack.pop();
                ExpStack.push(Mgr->MakeForAll(QVarTypes, NewBody));
            }


            void ConstraintPurifier::VisitOpExpression(const OpExpT* Exp)
            {
                auto OpCode = Exp->GetOpCode();
                auto const& Children = Exp->GetChildren();
                const u32 NumChildren = Children.size();

                if (OpCode != LTSOps::OpSelect) {
                    VisitorBaseT::VisitOpExpression(Exp);
                    vector<ExpT> NewChildren(NumChildren);
                    for (u32 i = 0; i < NumChildren; ++i) {
                        NewChildren[NumChildren - i - 1] = ExpStack.top();
                        ExpStack.pop();
                    }
                    ExpStack.push(Mgr->MakeExpr(OpCode, NewChildren));
                } else {
                    set<pair<ExpT, TypeRef>> UFIndexExps;
                    UFIndexExpGatherer::Do(Exp, UFIndexExps);

                    auto&& ITEBranches = MakeITEBranches(Exp, UFIndexExps);
                    auto ITEExp = ITEBranches[0].second;
                    for (auto it = next(ITEBranches.begin()); it != ITEBranches.end(); ++it) {
                        ITEExp = Mgr->MakeExpr(LTSOps::OpITE, (*it).first,
                                               (*it).second, ITEExp);
                    }
                    ExpStack.push(Mgr->SimplifyFP(ITEExp));
                }
            }

            ExpT ConstraintPurifier::Do(MgrT* Mgr, const ExpT& Exp, FastExpSetT& Assumptions)
            {
                ConstraintPurifier ThePurifier(Mgr, Assumptions);
                Exp->Accept(&ThePurifier);
                return ThePurifier.ExpStack.top();
            }

        } /* end namespace Detail */

    } /* end namespace LTS */
} /* end namespace ESMC */

//
// LTSTransformers.cpp ends here
