// LTSAssign.cpp ---
//
// Filename: LTSAssign.cpp
// Author: Abhishek Udupa
// Created: Fri Aug  8 15:38:49 2014 (-0400)
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

#include "LTSAssign.hpp"
#include "LTSUtils.hpp"

namespace ESMC {
    namespace LTS {

        LTSAssignBase::LTSAssignBase()
        {
            // Nothing here
        }

        LTSAssignBase::LTSAssignBase(const ExpT& LHS, const ExpT& RHS)
            : RefCountable(), LHS(LHS), RHS(RHS)
        {
            // Nothing here
        }

        LTSAssignBase::~LTSAssignBase()
        {
            // Nothing here
        }

        const ExpT& LTSAssignBase::GetLHS() const
        {
            return LHS;
        }

        const ExpT& LTSAssignBase::GetRHS() const
        {
            return RHS;
        }

        LTSAssignSimple::~LTSAssignSimple()
        {
            // Nothing here
        }

        string LTSAssignSimple::ToString() const
        {
            return (LHS->ToString() + " := " + RHS->ToString());
        }

        vector<LTSAssignRef> LTSAssignSimple::ExpandNonScalarUpdates() const
        {
            vector<LTSAssignRef> Retval;
            auto Mgr = LHS->GetMgr();
            auto FAType = Mgr->MakeType<ExprFieldAccessType>();

            if (LHS->GetType()->Is<ExprScalarType>()) {
                if (RHS->Is<ConstExpression>()) {
                    auto RHSAsConst = RHS->SAs<ConstExpression>();
                    auto const& ConstVal = RHSAsConst->GetConstValue();
                    if (ConstVal == "clear") {
                        auto ClearExp = Mgr->MakeVal(LHS->GetType()->GetClearValue(),
                                                     LHS->GetType());
                        auto NewAsgn = new LTSAssignSimple(LHS, ClearExp);
                        Retval.push_back(NewAsgn);
                        return Retval;
                    }
                }
                Retval.push_back(this);
                return Retval;
            }


            if (LHS->GetType()->Is<ExprRecordType>()) {
                auto TypeAsRec = LHS->GetType()->SAs<ExprRecordType>();
                auto const& Fields = TypeAsRec->GetMemberVec();
                for (auto const& Field : Fields) {

                    auto const& FieldName = Field.first;
                    auto const& FieldType = Field.second;

                    auto NewLHS = Mgr->MakeExpr(LTSOps::OpField, LHS,
                                                Mgr->MakeVar(FieldName, FAType));
                    ExpT NewRHS = ExpT::NullPtr;
                    if (RHS->Is<ConstExpression>()) {
                        if (!FieldType->Is<ExprScalarType>()) {
                            NewRHS = Mgr->MakeVal("clear", FieldType);
                        } else {
                            NewRHS = Mgr->MakeVal(FieldType->GetClearValue(), FieldType);
                        }
                    } else {
                        NewRHS = Mgr->MakeExpr(LTSOps::OpField, RHS,
                                               Mgr->MakeVar(FieldName, FAType));
                    }

                    auto NewAsgn = new LTSAssignSimple(NewLHS, NewRHS);
                    auto&& Expansions = NewAsgn->ExpandNonScalarUpdates();
                    Retval.insert(Retval.end(), Expansions.begin(), Expansions.end());
                }
            } else if (LHS->GetType()->Is<ExprArrayType>()) {
                auto TypeAsArray = LHS->GetType()->SAs<ExprArrayType>();
                auto const& IndexType = TypeAsArray->GetIndexType();
                auto const& ValueType = TypeAsArray->GetValueType();
                auto const& IndexElems = IndexType->GetElementsNoUndef();

                for (auto const& IndexElem : IndexElems) {
                    auto IndexExp = Mgr->MakeVal(IndexElem, IndexType);
                    auto NewLHS = Mgr->MakeExpr(LTSOps::OpIndex, LHS, IndexExp);

                    ExpT NewRHS = ExpT::NullPtr;
                    if (RHS->Is<ConstExpression>()) {
                        if (!ValueType->Is<ExprScalarType>()) {
                            NewRHS = Mgr->MakeVal("clear", ValueType);
                        } else {
                            NewRHS = Mgr->MakeVal(ValueType->GetClearValue(), ValueType);
                        }
                    } else {
                        NewRHS = Mgr->MakeExpr(LTSOps::OpIndex, RHS, IndexExp);
                    }

                    LTSAssignRef NewAsgn = new LTSAssignSimple(NewLHS, NewRHS);
                    auto&& Expansions = NewAsgn->ExpandNonScalarUpdates();
                    Retval.insert(Retval.end(), Expansions.begin(), Expansions.end());
                }
            } else {
                throw InternalError((string)"Unhandled type:\n" + LHS->GetType()->ToString() +
                                    "\nwhen trying expand updates in assignment:\n" +
                                    this->ToString() + "\nAt: " + __FILE__ + ":" +
                                    to_string(__LINE__));
            }
            // Get the compiler to shut up
            return Retval;
        }

        LTSAssignParam::LTSAssignParam(const vector<ExpT>& Params,
                                       const ExpT& Constraint,
                                       const ExpT& LHS, const ExpT& RHS)
            : LTSAssignBase(LHS, RHS), Params(Params), Constraint(Constraint)
        {
            // If the RHS is an expression of a symmetric type,
            // make sure that it does not refer to any of the params
            if (LHS->GetType()->Is<ExprSymmetricType>()) {
                if (RHS->Is<Exprs::ConstExpression>()) {
                    if (RHS->SAs<Exprs::ConstExpression>()->GetConstValue() != "clear") {
                        throw ESMCError((string)"Cannot make a parametric assignment " +
                                        "to an arbitrary constant of a symmetric type");
                    }
                }
                if (LHS->Is<Exprs::VarExpression>()) {
                    for (auto const& Param : Params) {
                        if (RHS == Param) {
                            throw ESMCError((string)"Cannot initialize parameteric LHS to " +
                                            "the value of a parameter");
                        }
                    }
                }
            }
        }

        LTSAssignParam::~LTSAssignParam()
        {
            // Nothing here
        }

        const vector<ExpT>& LTSAssignParam::GetParams() const
        {
            return Params;
        }

        const ExpT& LTSAssignParam::GetConstraint() const
        {
            return Constraint;
        }

        string LTSAssignParam::ToString() const
        {
            return (LHS->ToString() + " := " + RHS->ToString());
        }

        vector<LTSAssignRef> LTSAssignParam::ExpandNonScalarUpdates() const
        {
            throw InternalError((string)"LTSAssignParam::ExpandNonScalarUpdates() " +
                                "should never have been called\nAt: " + __FILE__ +
                                ":" + to_string(__LINE__));
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

//
// LTSAssign.cpp ends here
