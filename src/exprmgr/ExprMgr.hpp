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
#include "Expressions.hpp"

#include <unordered_map>
#include <unordered_set>
#include <vector>


namespace ESMC {
    namespace Exprs {

        class ExprTypeError : public exception
        {
        private:
            string Message;
            
        public:
            ExprTypeError(const string& Message) : Message(Message) {}
            virtual ~ExprTypeError() {}
            
            virtual const char* what() const noexcept override
            {
                return Message.c_str();
            }
        };

        // This is a default semanticizer.
        // All semanticizers must implement this signature.
        template<typename E>
        class SemanticizerBase
        {
        private:
            const string Name;

        public:
            // The type of the lowered expressions
            // Since we don't do anything special, we 
            // set it to the type of expressions itself
            typedef Expr<E, ESMC::Exprs::SemanticizerBase> LExpType;
            typedef Expr<E, ESMC::Exprs::SemanticizerBase> ExpType;

            SemanticizerBase() : Name("SemanticizerBase") {}
            ~SemanticizerBase() {}
            
            inline void TypeCheck(ExpType Exp) const {}

            inline ExpType Canonicalize(const ExpType& Exp) const { return Exp; }
            inline LExpType LowerExpr(const ExpType& Exp) { return Exp; }
            inline ExpType RaiseExpr(const LExpType& LExp) { return LExp; }
            inline string ExprToString(const LExpType& Exp) { return "NoExp"; }
            inline string TypeToString(i64 Type) const { return "NoType"; }

            inline i64 RegisterUninterpretedFunction(const string& Name,
                                                     const vector<i64> DomTypes,
                                                     i64 RangeType) const
            {
                return 0;
            }
        };

        template <typename E, template <typename> class S>
        class ExprMgr
        {
        public:
            typedef S<E> SemT;
            typedef typename SemT::LExpType LExpT;
            typedef Expr<E, S> ExpT;
            typedef ExprI<E, S> IExpT;

            typedef unordered_map<ExpT, ExpT, 
                                  ExpressionPtrHasher,
                                  ExpressionPtrEquals> SubstMap;

        private:
            SemT* Sem;
            typedef unordered_set<ExpT, ExpressionPtrHasher, FastExpressionPtrEquals> ExpSetType;
            ExpSetType ExpSet;
            
            inline ExpT GetCachedOrInsert(const ExpT& Exp);
            
            template <typename T, typename... ArgTypes>
            inline ExpT GetCachedOrInsert(ArgTypes&&... Args);
            inline void CheckMgr(const vector<ExpT>& Children) const;
            template <typename T>
            inline ExpT MakeQExpression(const vector<ExpT>& QVars,
                                        const ExpT& QExpr,
                                        const ExpT& ExtVal);

            // Insert the expression and all subexpressions
            // into the set of expressions owned by this manager
            inline ExpT Internalize(const ExpT& Exp);
            
        public:
            template <typename... ArgTypes>
            inline ExprMgr(ArgTypes&&... Args);
            inline ~ExprMgr();

            inline ExpT MakeVal(const string& ValString, i64 ValType, 
                                const E& ExtVal = E());
            
            inline ExpT MakeVar(const string& VarName, i64 VarType,
                                const E& ExtVal = E());

            inline ExpT MakeBoundVar(i64 VarType, i64 VarIdx,
                                     const E& ExtVal = E());

            inline ExpT MakeExpr(i64 OpCode, const vector<ExpT>& Children,
                                 const E& ExtVal = E());

            inline ExpT MakeExists(const vector<ExpT>& QVars, 
                                   const ExpT& QExpr,
                                   const E& ExtVal = E());

            inline ExpT MakeForAll(const vector<ExpT>& QVars, 
                                   const ExpT& QExpr,
                                   const E& ExtVal = E());

            inline i64 MakeUninterpretedFunction(const string& Name, 
                                                 const vector<i64>& Range,
                                                 i64 Domain);

            template <typename T, typename... ArgTypes>
            inline ExpT ApplyTransform(const ExpT& Exp, ArgTypes&&... Args);

            inline SemT* GetSemanticizer() const;
            inline LExpT LowerExpr(const ExpT& Exp);
            inline ExpT RaiseExpr(const LExpT& Exp);
            inline void GC();
        };

        // ExprMgr implementation
        template <typename E, template <typename> class S>
        template <typename... ArgTypes>
        inline ExprMgr<E, S>::ExprMgr(ArgTypes&&... Args)
            : Sem(new SemT(forward<ArgTypes>(Args)...))
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline ExprMgr<E, S>::~ExprMgr()
        {
            delete Sem;
            ExpSet.clear();
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::GetCachedOrInsert(const ExpT& Exp)
        {
            auto it = ExpSet.find(Exp);
            if (it == ExpSet.end()) {
                ExpSet.insert(Exp);
                return Exp;
            } else {
                return (*it);
            }
        }

        template <typename E, template <typename> class S>
        template <typename T, typename... ArgTypes>
        inline typename ExprMgr<E, S>::ExpT 
        ExprMgr<E, S>::GetCachedOrInsert(ArgTypes&&... Args)
        {
            ExpT NewExp = new T(forward<ArgTypes>(Args)...);
            return GetCachedOrInsert(NewExp);
        }

        template <typename E, template <typename> class S>
        void ExprMgr<E, S>::CheckMgr(const vector<ExpT>& Children) const
        {
            for (auto const& Child : Children) {
                if (Child->GetMgr() != this) {
                    throw ExprTypeError("ExprMgr: I don't own this expression!");
                }
            }
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::Internalize(const ExpT& Exp)
        {
            if ((Exp->template As<ConstExpression>() != nullptr) ||
                (Exp->template As<VarExpression>() != nullptr) ||
                (Exp->template As<BoundVarExpression>() != nullptr)) {
                return GetCachedOrInsert(Exp);
            }
            auto ExpAsOp = Exp->template As<OpExpression>();
            if (ExpAsOp != nullptr) {
                auto const& Children = ExpAsOp->GetChildren();
                const u32 NumChildren = Children.size();
                vector<ExpT> IntChildren(NumChildren);
                for (u32 i = 0; i < NumChildren; ++i) {
                    IntChildren[i] = Internalize(Children[i]);
                }
                return GetCachedOrInsert<OpExpression<E, S>>(Exp->GetOpCode(),
                                                             IntChildren,
                                                             Exp->ExtData);
            }
            auto ExpAsQuantified = Exp->template As<QuantifiedExpressionBase>();
            if (ExpAsQuantified != nullptr) {
                auto const& QVars = ExpAsQuantified->GetQVarList();
                const u32 NumQVars = QVars.size();
                vector<ExpT> IntQVars(NumQVars);
                for (u32 i = 0; i < NumQVars; ++i) {
                    IntQVars[i] = Internalize(QVars[i]);
                }
                auto IntQExpr = Internalize(ExpAsQuantified->GetQExpression());
                if (ExpAsQuantified->IsForAll()) {
                    return GetCachedOrInsert<AQuantifiedExpression<E,S>>(IntQVars,
                                                                         IntQExpr,
                                                                         Exp->ExtData);
                } else {
                    return GetCachedOrInsert<EQuantifiedExpression<E,S>>(IntQVars,
                                                                         IntQExpr,
                                                                         Exp->ExtData);
                }
            } else {
                throw ExprTypeError("Strange type of expression encountered");
            }
        }

        template <typename E, template<typename> class S>
        inline typename ExprMgr<E, S>::ExpT 
        ExprMgr<E, S>::MakeVal(const string& ValString, i64 ValType,
                               const E& ExtVal)
        {
            auto Retval =
                GetCachedOrInsert<ConstExpression<E,S>>(ValString, ValType, ExtVal);
            Sem->TypeCheck(Retval);
            return Retval;
        }

        template <typename ExtType, typename SemType>
        inline ExprMgr<ExtType, SemType>::ExpType
        ExprMgr<ExtType, SemType>::MakeVar(const string& VarName, i64 VarType,
                                           const ExtType& ExtVal)
        {
            auto Retval = 
                GetCachedOrInsert<VarExpression>(VarName, VarType, ExtVal);
            Sem->TypeCheck(Retval);
            return Retval;
        }

        template <typename ExtType, typename SemType>
        inline ExprMgr<ExtType, SemType>::ExpType
        ExprMgr<ExtType, SemType>::MakeBoundVar(const string& VarName, i64 VarType,
                                                i64 VarUID, const ExtType& ExtVal)
        {
            auto Retval =
                GetCachedOrInsert<BoundVarExpression>(VarName, VarType, VarUID, ExtVal);
            Sem->TypeCheck(Retval);
            return Retval;
        }

        template <typename ExtType, typename SemType>
        inline ExprMgr<ExtType, SemType>::ExpType
        ExprMgr<ExtType, SemType>::MakeExpr(const i64 OpCode,
                                            const vector<ExprMgr<ExtType, SemType>::ExpType>& Children,
                                            const ExtType& ExtVal)
        {
            CheckMgr(Children);
            ExprMgr<ExtType, SemType>::ExpType NewExp = new OpExpression(OpCode, Children, ExtVal);
            Sem->TypeCheck(NewExp);
            auto Retval = Sem->Canonicalize(NewExp);
            return GetCachedOrInsert(Retval);
        }

        template <typename ExtType, typename SemType>
        template <typename T>
        inline ExprMgr<ExtType, SemType>::ExpType
        ExprMgr<ExtType, SemType>::MakeQExpression(const vector<ExprMgr<ExtType, SemType>::ExpType>& QVars,
                                                   const ExprMgr<ExtType, SemType>::ExpType& QExpr,
                                                   const ExtType& ExtVal)
        {
            CheckMgr(QVars);
            if (QExpr->GetMgr() != this) {
                throw ExprTypeError("ExprMgr: I don't own this expression!");
            }
            for (auto const& QVar : QVars) {
                if (QVar->As<BoundVarExpression>() == nullptr) {
                    throw ExprTypeError("ExprMgr: Only bound vars can be quantified!");
                }
            }
            ExprMgr<ExtType, SemType>::ExpType NewExp = 
                new T(QVars, QExpr, ExtVal);
            Sem->TypeCheck(NewExp);
            auto Retval = Sem->Canonicalize(NewExp);
            return GetCachedOrInsert(Retval);
        }

        template <typename ExtType, typename SemType>
        inline ExprMgr<ExtType, SemType>::ExpType
        ExprMgr<ExtType, SemType>::MakeExists(const vector<ExprMgr<ExtType, SemType>::ExpType>& QVars,
                                              const ExprMgr<ExtType, SemType>::ExpType& QExpr,
                                              const ExtType& ExtVal)
        {
            return MakeQExpression<EQuantifiedExpression>(QVars, QExpr, ExtVal);
        }

        template <typename ExtType, typename SemType>
        inline ExprMgr<ExtType, SemType>::ExpType
        ExprMgr<ExtType, SemType>::MakeForAll(const vector<ExprMgr<ExtType, SemType>::ExpType>& QVars,
                                              const ExprMgr<ExtType, SemType>::ExpType& QExpr,
                                              const ExtType& ExtVal)
        {
            return MakeQExpression<AQuantifiedExpression>(QVars, QExpr, ExtVal);
        }

        template <typename ExtType, typename SemType>
        inline i64 ExprMgr<ExtType, SemType>::MakeUninterpretedFunction(const string& Name,
                                                                        const vector<i64>& Domain,
                                                                        i64 Range)
        {
            return Sem->RegisterUninterpretedFunction(Name, Domain, Range);
        }

        template <typename ExtType, typename SemType>
        template <typename T, typename... ArgTypes>
        inline ExprMgr<ExtType, SemType>::ExpType
        ExprMgr<ExtType, SemType>::ApplyTransform(const ExpType& Exp, 
                                                  ArgTypes&&... Args)
        {
            Retval = T::Do(this, Exp, forward<ArgTypes>(Args)...);
        }
        
    } /* end namespace */
} /* end namespace */

#endif /* MC_EXPR_MGR_HPP_ */

// 
// ExprMgr.hpp ends here
