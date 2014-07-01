// ExprMgr.hpp --- 
// 
// Filename: ExprMgr.hpp
// Author: Abhishek Udupa
// Created: Mon Jun 30 01:34:35 2014 (-0400)
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

// Generic expression manager

#if !defined ESMC_EXPR_MGR_HPP_
#define ESMC_EXPR_MGR_HPP_

#include "../common/FwdDecls.hpp"
#include "../containers/RefCountable.hpp"

#include <unordered_map>


namespace ESMC {

    class ExprTypeError : public exception
    {
    private:
        string Message;

    public:
        ExprTypeError(const string& Message);
        virtual ~ExprTypeError();

        virtual const char* what() const noexcept override;
    };

    template <typename LExpType>
    class ExprSemanticizerBase
    {
    private:
        const string& Name;

    public:
        ExprSemanticizerBase(const string& Name);

        virtual void TypeCheck(const Expression& Exp) const = 0;
        virtual Expression Canonicalize(const Expression& Exp) const = 0;
        virtual LExpType LowerExpr(const Expression& Exp) const = 0;
        virtual Expression RaiseExpr(const LExpType& LExp) const = 0;
        virtual string ExprToString(const ExpressionBase* Exp) const = 0;
        virtual string TypeToString(i64 Type) const = 0;
        virtual Expression Simplify(const Expression& Exp) const = 0;
        virtual Expression ElimQuantifiers(const Expression& Exp) const = 0;
        virtual i64 RegisterUninterpretedFunction(const string& Name,
                                                  const vector<i64> DomTypes,
                                                  i64 RangeType) const = 0;
    };

    template <typename LExpType>
    class ExprMgr
    {
    private:
        ExprSemanticizerBase<LExpType>* Sem;
    public:
        typedef unordered_map<Expression, Expression, 
                              ExpressionPtrHasher,
                              ExpressionPtrEquals> SubstMap;

        ExprMgr(ExprSemanticizerBase<LExpType>* Sem);
        ~ExprMgr();

        Expression MakeVal(const string& ValString, i64 ValType);
        Expression MakeVar(const string& VarName, i64 VarType);
        Expression MakeBoundVar(const string& VarName, i64 VarType,
                                i64 VarUID = -1);
        Expression MakeExpr(i64 OpCode, const vector<Expression>& Children);
        Expression MakeExists(const vector<Expression>& QVars, 
                              const Expression& QExpr);
        Expression MakeForAll(const vector<Expression>& QVars, 
                              const Expression& QExpr);
        i64 MakeUninterpretedFunction(const string& Name, 
                                      const vector<i64> Range,
                                      i64 Domain);

        Expression ElimQuantifiers(const Expression& Exp);
        Expression Simplify(const Expression& Exp);

        Expression Substitute(const SubstMap& SMap,
                              const Expression& Exp);
                                
        ExprSemanticizerBase* GetSemanticizer() const;
    };

} /* end namespace */

#endif /* MC_EXPR_MGR_HPP_ */

// 
// ExprMgr.hpp ends here
