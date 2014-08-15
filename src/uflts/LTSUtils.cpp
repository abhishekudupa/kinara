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
                    auto FieldVarExp = OldChildren[0]->As<VarExpression>();
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

            QuantifierUnroller::QuantifierUnroller(MgrT* Mgr)
                : VisitorBaseT("QuantifierUnroller"), Mgr(Mgr)
            {
                // Nothing here
            }

            QuantifierUnroller::~QuantifierUnroller()
            {
                // Nothing here
            }

            void QuantifierUnroller::VisitVarExpression(const VarExpT* Exp) 
            {
                ExpStack.push_back(Exp);
            }

            void QuantifierUnroller::VisitBoundVarExpression(const BoundVarExpT* Exp)
            {
                ExpStack.push_back(Exp);
            }

            void QuantifierUnroller::VisitConstExpression(const ConstExpT* Exp) 
            {
                ExpStack.push_back(Exp);
            }

            void QuantifierUnroller::VisitOpExpression(const OpExpT* Exp)
            {
                VisitorBaseT::VisitOpExpression(Exp);
                auto const& Children = Exp->GetChildren();
                const u32 NumChildren = Children.size();

                vector<ExpT> NewChildren(NumChildren);
                for (u32 i = 0; i < NumChildren; --i) {
                    NewChildren[NumChildren - i - 1] = ExpStack.back();
                    ExpStack.pop_back();
                }
                
                ExpStack.push_back(Mgr->MakeExpr(Exp->GetOpCode(), NewChildren));
            }

            inline vector<ExpT>
            QuantifierUnroller::UnrollQuantifier(const QExpT* Exp)
            {
                vector<ExpT> Retval;
                auto const& QVarTypes = Exp->GetQVarTypes();
                auto const& QBody = Exp->GetQExpression();
                const u32 NumQVars = QVarTypes.size();

                vector<vector<string>> QVarElems;
                for (auto const& QVarType : QVarTypes) {
                    QVarElems.push_back(QVarType->GetElements());
                }
                auto&& CP = CrossProduct<string>(QVarElems.begin(), 
                                                  QVarElems.end());
                for (auto const& CPElem : CP) {
                    MgrT::SubstMapT SubstMap;
                    for (u32 i = 0; i < NumQVars; ++i) {
                        auto const& BoundVarExp = Mgr->MakeBoundVar(QVarTypes[i], i);
                        auto const& ValExp = Mgr->MakeVal(CPElem[i], QVarTypes[i]);
                        SubstMap[BoundVarExp] = ValExp;
                    }

                    Retval.push_back(Mgr->Substitute(SubstMap, QBody));
                }
                return Retval;
            }

            void QuantifierUnroller::VisitEQuantifiedExpression(const EQExpT* Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto const& NewExp = ExpStack.back();
                ExpStack.pop_back();
                auto NewQExp = Mgr->MakeExists(Exp->GetQVarTypes(), NewExp);
                auto&& UnrolledExps = 
                    UnrollQuantifier(NewQExp->SAs<Exprs::QuantifiedExpressionBase>());
                ExpStack.push_back(Mgr->MakeExpr(LTSOps::OpOR, UnrolledExps));
            }

            void QuantifierUnroller::VisitAQuantifiedExpression(const AQExpT* Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto const& NewExp = ExpStack.back();
                ExpStack.pop_back();
                auto NewQExp = Mgr->MakeForAll(Exp->GetQVarTypes(), NewExp);
                auto&& UnrolledExps = 
                    UnrollQuantifier(NewQExp->SAs<Exprs::QuantifiedExpressionBase>());
                ExpStack.push_back(Mgr->MakeExpr(LTSOps::OpAND, UnrolledExps));
            }

            ExpT QuantifierUnroller::Do(MgrT* Mgr, const ExpT& Exp)
            {
                QuantifierUnroller Unroller(Mgr);
                Exp->Accept(&Unroller);
                return Unroller.ExpStack[0];
            }
        } /* end namespace Detail */
    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// LTSUtils.cpp ends here
