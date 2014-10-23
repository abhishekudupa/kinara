// LTSUtils.cpp --- 
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

#include "LTSUtils.hpp"

namespace ESMC {
    namespace LTS {
        namespace Detail {

            MsgTransformer::MsgTransformer(MgrT* Mgr, const string& MsgVarName,
                                           const ExprTypeRef& MsgRecType, 
                                           const ExprTypeRef& UnifiedMType)
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
                    ExpStack.push_back(Mgr->MakeVar(Exp->GetVarName(), UnifiedMType));
                } else {
                    ExpStack.push_back(Exp);
                }
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

                auto const& OldChildren = Exp->GetChildren();
                const u32 NumChildren = OldChildren.size();
                vector<ExpT> NewChildren(NumChildren);
                for (u32 i = 0; i < NumChildren; ++i) {
                    NewChildren[NumChildren - i - 1] = ExpStack.back();
                    ExpStack.pop_back();
                }

                auto OpCode = Exp->GetOpCode();
                if (OpCode == LTSOps::OpField && 
                    OldChildren[0]->Is<Exprs::VarExpression>() &&
                    OldChildren[0]->As<Exprs::VarExpression>()->GetVarName() == MsgVarName &&
                    OldChildren[0]->As<Exprs::VarExpression>()->GetVarType() == MsgRecType) {
                    
                    ExprTypeRef ActMsgRecType = nullptr;
                    if (MsgRecType->Is<ExprRecordType>()) {
                        ActMsgRecType = MsgRecType;
                    } else if (MsgRecType->Is<ExprParametricType>()) {
                        ActMsgRecType = MsgRecType->SAs<ExprParametricType>()->GetBaseType();
                    } else {
                        throw ESMCError((string)"MsgTransformer: Message type \"" + 
                                        MsgRecType->ToString() + "\" is not a parametric " + 
                                        "type or a record type");
                    }

                    auto MTypeAsUnion = UnifiedMType->As<ExprUnionType>();
                    auto FieldVarExp = OldChildren[1]->As<VarExpression>();
                    auto const& OldFieldName = FieldVarExp->GetVarName();
                    auto const& NewFieldName = MTypeAsUnion->MapFromMemberField(ActMsgRecType, 
                                                                                OldFieldName);
                    auto FAType = Mgr->MakeType<Exprs::ExprFieldAccessType>();
                    auto NewFieldVar = Mgr->MakeVar(NewFieldName, FAType);
                    ExpStack.push_back(Mgr->MakeExpr(LTSOps::OpField, NewChildren[0], NewFieldVar));
                } else {
                    ExpStack.push_back(Mgr->MakeExpr(OpCode, NewChildren));
                }
            }

            void MsgTransformer::VisitEQuantifiedExpression(const EQExpT* Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto NewQExpr = ExpStack.back();
                ExpStack.pop_back();
                ExpStack.push_back(Mgr->MakeExists(Exp->GetQVarTypes(), NewQExpr));
            }

            void MsgTransformer::VisitAQuantifiedExpression(const AQExpT* Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto NewQExpr = ExpStack.back();
                ExpStack.pop_back();
                ExpStack.push_back(Mgr->MakeForAll(Exp->GetQVarTypes(), NewQExpr));
            }

            ExpT MsgTransformer::Do(MgrT* Mgr, 
                                    const ExpT& Exp,
                                    const string& MsgVarName,
                                    const ExprTypeRef& MsgRecType,
                                    const ExprTypeRef& UnifiedMType)
            {
                MsgTransformer TheTransformer(Mgr, MsgVarName, MsgRecType, UnifiedMType);
                Exp->Accept(&TheTransformer);
                return TheTransformer.ExpStack[0];
            }

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
                ExpStack.push_back(Exp);
            }

            void ArrayRValueTransformer::VisitBoundVarExpression(const BoundVarExpT* Exp)
            {
                ExpStack.push_back(Exp);
            }

            void ArrayRValueTransformer::VisitConstExpression(const ConstExpT* Exp)
            {
                ExpStack.push_back(Exp);
            }

            void ArrayRValueTransformer::VisitOpExpression(const OpExpT* Exp)
            {
                VisitorBaseT::VisitOpExpression(Exp);

                auto OpCode = Exp->GetOpCode();
                auto const& Children = Exp->GetChildren();
                
                const u32 NumChildren = Exp->GetChildren().size();
                vector<ExpT> NewChildren(NumChildren);
                for (u32 i = 0; i < NumChildren; ++i) {
                    NewChildren[NumChildren - i - 1] = ExpStack.back();
                    ExpStack.pop_back();
                }

                if (OpCode != LTSOps::OpIndex) {
                    ExpStack.push_back(Mgr->MakeExpr(OpCode, NewChildren));
                    return;
                }

                // An index op
                auto ArrayType = Children[0]->GetType()->SAs<ExprArrayType>();
                auto IndexType = ArrayType->GetIndexType();
                auto ValueType = ArrayType->GetValueType();
                auto const& IndexElems = IndexType->GetElements();
                const u32 IndexCardinality = IndexElems.size();

                if (IndexCardinality == 1) {
                    ExpStack.push_back(Mgr->MakeExpr(LTSOps::OpIndex, NewChildren[0],
                                                     Mgr->MakeVal(IndexElems[0], IndexType)));
                } else {
                    auto AccExp = Mgr->MakeExpr(LTSOps::OpIndex, NewChildren[0],
                                                Mgr->MakeVal(IndexElems.back(), IndexType));
                    for (u32 i = 0; i < IndexCardinality - 1; ++i) {

                        auto CurVal = Mgr->MakeVal(IndexElems[IndexCardinality - i - 2],
                                                   IndexType);

                        AccExp = Mgr->MakeExpr(LTSOps::OpITE,
                                               Mgr->MakeExpr(LTSOps::OpEQ,
                                                             NewChildren[1], CurVal),
                                               Mgr->MakeExpr(LTSOps::OpIndex,
                                                             NewChildren[0], CurVal),
                                               AccExp);
                    }
                    ExpStack.push_back(AccExp);
                }
            }

            void ArrayRValueTransformer::VisitEQuantifiedExpression(const EQExpT* Exp)
            {
                auto const& QVarTypes = Exp->GetQVarTypes();
                auto const& QExpr = Exp->GetQExpression();
                QExpr->Accept(this);
                auto NewExp = ExpStack.back();
                ExpStack.pop_back();
                ExpStack.push_back(Mgr->MakeExists(QVarTypes, NewExp));
            }

            void ArrayRValueTransformer::VisitAQuantifiedExpression(const AQExpT* Exp)
            {
                auto const& QVarTypes = Exp->GetQVarTypes();
                auto const& QExpr = Exp->GetQExpression();
                QExpr->Accept(this);
                auto NewExp = ExpStack.back();
                ExpStack.pop_back();
                ExpStack.push_back(Mgr->MakeForAll(QVarTypes, NewExp));
            }

            ExpT ArrayRValueTransformer::Do(MgrT* Mgr, const ExpT& Exp)
            {
                ArrayRValueTransformer TheTransformer(Mgr);
                Exp->Accept(&TheTransformer);
                return TheTransformer.ExpStack[0];
            }

            ExpressionPermuter::ExpressionPermuter(MgrT* Mgr, const vector<u08>& PermVec,
                                                   const map<ExprTypeRef, u32>& TypeOffsets)
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
                ExpStack.push_back(Exp);
            }

            void ExpressionPermuter::VisitConstExpression(const ConstExpT* Exp)
            {
                auto const& Type = Exp->GetType();
                if (!Type->Is<ExprSymmetricType>()) {
                    ExpStack.push_back(Exp);
                    return;
                }

                auto TypeAsSym = Type->SAs<ExprSymmetricType>();
                // Symmetric type. Permute
                auto const& ConstVal = Exp->GetConstValue();
                if (ConstVal == "clear") {
                    // The undef value permutes to itself regardless
                    ExpStack.push_back(Exp);
                    return;
                }
                auto ConstIdx = TypeAsSym->GetMemberIdx(ConstVal);
                auto it = TypeOffsets.find(Type);
                
                if (it == TypeOffsets.end()) {
                    throw ESMCError((string)"Could not find offset for type: " + 
                                    Type->ToString() + "\nIn Expression Permuter, on " + 
                                    "expression:\n" + Exp->ToString());
                }
                auto Offset = it->second;
                auto PermutedIdx = PermVec[Offset + ConstIdx];
                auto const& PermutedVal = TypeAsSym->GetMember(PermutedIdx);
                ExpStack.push_back(Mgr->MakeVal(PermutedVal, Type));
            }

            void ExpressionPermuter::VisitBoundVarExpression(const BoundVarExpT* Exp)
            {
                ExpStack.push_back(Exp);
            }

            void ExpressionPermuter::VisitOpExpression(const OpExpT* Exp)
            {
                VisitorBaseT::VisitOpExpression(Exp);

                auto OpCode = Exp->GetOpCode();
                auto const& Children = Exp->GetChildren();
                const u32 NumChildren = Children.size();

                vector<ExpT> NewChildren(NumChildren);

                for (u32 i = 0; i < NumChildren; ++i) {
                    NewChildren[NumChildren - i - 1] = ExpStack.back();
                    ExpStack.pop_back();
                }

                ExpStack.push_back(Mgr->MakeExpr(OpCode, NewChildren));
            }

            void ExpressionPermuter::VisitEQuantifiedExpression(const EQExpT* Exp)
            {
                auto const& QVarTypes = Exp->GetQVarTypes();
                auto const& QExpr = Exp->GetQExpression();

                QExpr->Accept(this);
                
                auto NewExp = ExpStack.back();
                ExpStack.pop_back();
                ExpStack.push_back(Mgr->MakeExists(QVarTypes, NewExp));
            }

            void ExpressionPermuter::VisitAQuantifiedExpression(const AQExpT* Exp)
            {
                auto const& QVarTypes = Exp->GetQVarTypes();
                auto const& QExpr = Exp->GetQExpression();

                QExpr->Accept(this);
                
                auto NewExp = ExpStack.back();
                ExpStack.pop_back();
                ExpStack.push_back(Mgr->MakeForAll(QVarTypes, NewExp));
            }

            ExpT ExpressionPermuter::Do(MgrT* Mgr, const ExpT& Exp,
                                        const vector<u08>& PermVec,
                                        const map<ExprTypeRef, u32>& TypeOffsets)
            {
                ExpressionPermuter ThePermuter(Mgr, PermVec, TypeOffsets);
                Exp->Accept(&ThePermuter);
                return ThePermuter.ExpStack[0];
            }

        } /* End namespace Detail */
    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// LTSUtils.cpp ends here
