// Visitors.hpp --- 
// 
// Filename: Visitors.hpp
// Author: Abhishek Udupa
// Created: Mon Jun 30 16:02:08 2014 (-0400)
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

#if !defined ESMC_VISITORS_HPP_
#define ESMC_VISITORS_HPP_

#include "../common/FwdDecls.hpp"

namespace ESMC {
    namespace Exprs {
        // ExpressionVisitorBase
        template <typename E, template <typename> class S>
        class ExpressionVisitorBase 
        {
        private:
            string Name;
            
        public:
            ExpressionVisitorBase(const string& Name);
            virtual ~ExpressionVisitorBase();
            const string& GetName() const;
            
            virtual void VisitConstExpression(const ConstExpression<E, S>* Exp);
            virtual void VisitVarExpression(const VarExpression<E, S>* Exp);
            virtual void VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp);
            virtual void VisitOpExpression(const OpExpression<E, S>* Exp);
            virtual void VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp);
            virtual void VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp);
        };
        

        template <typename E, template <typename> class S>
        ExpressionVisitorBase<E, S>::ExpressionVisitorBase(const string& Name)
            : Name(Name)
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        ExpressionVisitorBase<E, S>::~ExpressionVisitorBase()
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        const string& ExpressionVisitorBase<E, S>::GetName() const
        {
            return Name;
        }

        template <typename E, template <typename> class S>
        void ExpressionVisitorBase<E, S>::VisitConstExpression(const ConstExpression<E, S>* Exp)
        {
            return;
        }

        template <typename E, template <typename> class S>
        void ExpressionVisitorBase<E, S>::VisitVarExpression(const VarExpression<E, S>* Exp)
        {
            return;
        }

        template <typename E, template <typename> class S>
        void ExpressionVisitorBase<E, S>::VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp)
        {
            return;
        }

        template <typename E, template <typename> class S>
        void ExpressionVisitorBase<E, S>::VisitOpExpression(const OpExpression<E, S>* Exp)
        {
            auto const& Children = Exp->GetChildren();
            for (auto const& Child : Children) {
                Child->Accept(this);
            }
            return;
        }

        template <typename E, template <typename> class S>
        void ExpressionVisitorBase<E, S>::VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp)
        {
            Exp->GetQExpression()->Accept(this);
        }

        template <typename E, template <typename> class S>
        void ExpressionVisitorBase<E, S>::VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp)
        {
            Exp->GetQExpression()->Accept(this);
        }
        
    } /* end namespace */
} /* end namespace */
    
#endif /* ESMC_VISITORS_HPP_ */

// 
// Visitors.hpp ends here
