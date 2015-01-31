// SemantizerUtils.hpp ---
//
// Filename: SemantizerUtils.hpp
// Author: Abhishek Udupa
// Created: Fri Jul 25 12:06:37 2014 (-0400)
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

#if !defined ESMC_SEMANTICIZER_UTILS_HPP_
#define ESMC_SEMANTICIZER_UTILS_HPP_

#include "../common/ESMCFwdDecls.hpp"
#include <vector>

namespace ESMC {
namespace Exprs {
namespace SemUtils {

// Type invalidator: Invalidates any previously
// computed type tags on an expression
template <typename E, template <typename> class S>
class TypeInvalidator : public ExpressionVisitorBase<E, S>
{
public:
    TypeInvalidator()
        : ExpressionVisitorBase<E, S>("TypeInvalidator") {}
    virtual ~TypeInvalidator() {}
    inline virtual void VisitVarExpression(const VarExpression<E, S>* Exp) override
    {
        Exp->SetType(S<E>::InvalidType);
    }

    inline virtual void VisitConstExpression(const ConstExpression<E, S>* Exp) override
    {
        Exp->SetType(S<E>::InvalidType);
    }


    inline virtual void
    VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp) override
    {
        Exp->SetType(S<E>::InvalidType);
    }

    inline virtual void VisitOpExpression(const OpExpression<E, S>* Exp) override
    {
        ExpressionVisitorBase<E, S>::VisitOpExpression(Exp);
        Exp->SetType(S<E>::InvalidType);
    }

    inline void VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp)
    {
        Exp->GetQExpression()->Accept(this);
    }

    inline virtual void
    VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp) override
    {
        VisitQuantifiedExpression(Exp);
    }

    inline virtual void
    VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp) override
    {
        VisitQuantifiedExpression(Exp);
    }
};


} /* end namespace SemUtils */
} /* end namespace Exprs */
} /* end namespace ESMC */

#endif /* ESMC_SEMANTICIZER_UTILS_HPP_ */

//
// SemantizerUtils.hpp ends here
