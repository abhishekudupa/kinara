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

        LTSAssignParam::LTSAssignParam(const vector<ExpT>& Params,
                                       const ExpT& Constraint,
                                       const ExpT& LHS, const ExpT& RHS)
            : LTSAssignBase(LHS, RHS), Params(Params), Constraint(Constraint)
        {
            // If the RHS is an expression of a symmetric type,
            // make sure that it does not refer to any of the params
            if (RHS->GetType()->Is<ExprSymmetricType>()) {
                if (RHS->Is<Exprs::ConstExpression>()) {
                    if (RHS->SAs<Exprs::ConstExpression>()->GetConstValue() != "clear") {
                        throw ESMCError((string)"Cannot make a parametric assignment " + 
                                        "to an arbitrary constant of a symmetric type");
                    }
                }
                if (RHS->Is<Exprs::VarExpression>()) {
                    for (auto const& Param : Params) {
                        if (RHS == Param) {
                            throw ESMCError((string)"Cannot initialize parameteric RHS to " + 
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
        
    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// LTSAssign.cpp ends here
