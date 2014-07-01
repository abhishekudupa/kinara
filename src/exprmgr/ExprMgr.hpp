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
#include <unordered_set>


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


        template<typename LEType>
        class SemanticizerBase
        {
        private:
            const string& Name;

        public:
            // Subclasses need to redefine this
            typedef LEType LExpType;

            SemanticizerBase(const string& Name);
            virtual ~SemanticizerBase
            
            template <typename ExtType>
            virtual void TypeCheck(const Expression<ExtType>& Exp) const = 0;

            template <typename ExtType>
            virtual Expression<ExtType> Canonicalize(const Expression<ExtType>& Exp) const = 0;

            template <typename ExtType>
            virtual LExpType LowerExpr(const Expression<ExtType>& Exp) const = 0;

            template <typename ExtType>
            virtual Expression<ExtType> RaiseExpr(const LExpType& LExp) const = 0;

            template <typename ExtType>
            virtual string ExprToString(const Expression<ExtType>& Exp) const = 0;

            virtual string TypeToString(i64 Type) const = 0;

            virtual i64 RegisterUninterpretedFunction(const string& Name,
                                                      const vector<i64> DomTypes,
                                                      i64 RangeType) const = 0;
        };


        template <typename ExtType, typename SemType>
        class ExprMgr
        {
        public:
            typedef SemType::LExpType LExpType;
            typedef Expression<ExtType> ExpType;

            typedef unordered_map<ExpType, ExpType, 
                                  ExpressionPtrHasher,
                                  ExpressionPtrEquals> SubstMap;
            
        private:
            SemType* Sem;
            typedef unordered_set<Expression<ExtType>, 
                                  ExpressionPtrHasher,
                                  ExpressionPtrEquals> ExpSetType;
            ExpSetType ExpSet;
            
            inline ExpType GetCachedOrInsert(const ExpType& Exp);
            
            template <typename T, typename... ArgTypes>
            inline ExpType GetCachedOrInsert(ArgTypes&&... Args);
            inline void CheckMgr(const vector<ExpType>& Children) const;
            template <typename T>
            inline ExpType MakeQExpression(const vector<ExpType>& QVars,
                                           const ExpType& QExpr,
                                           const ExtType& ExtVal);

        public:
            template <typename... ArgTypes>
            inline ExprMgr(ArgTypes&&... Args);
            inline ~ExprMgr();

            inline ExpType MakeVal(const string& ValString, i64 ValType, 
                                   const ExtType& ExtVal = ExtType());
            
            inline ExpType MakeVar(const string& VarName, i64 VarType,
                                   const ExtType& ExtVal = ExtType());

            inline ExpType MakeBoundVar(const string& VarName, i64 VarType,
                                        i64 VarUID = -1, const ExtType& ExtVal = ExtType());

            inline ExpType MakeExpr(i64 OpCode, const vector<ExpType>& Children,
                                    const ExtType& ExtVal = ExtType());

            inline ExpType MakeExists(const vector<ExpType>& QVars, 
                                      const ExpType& QExpr,
                                      const ExtType& ExtVal = ExtType());

            inline ExpType MakeForAll(const vector<ExpType>& QVars, 
                                      const ExpType& QExpr,
                                      const ExtType& ExtVal = ExtType());

            inline i64 MakeUninterpretedFunction(const string& Name, 
                                                 const vector<i64>& Range,
                                                 i64 Domain);

            template <typename T, typename... ArgTypes>
            inline ExpType ApplyTransform(const ExpType& Exp, ArgTypes&&... Args);

            inline SemType* GetSemanticizer() const;
            inline LExpType LowerExpr(const ExpType& Exp);
            inline ExpType RaiseExpr(const LExpType& Exp);
            inline void GC();
        };


        // SemanticizerBase implementation
        template <typename LEType>
        SemanticizerBase::SemanticizerBase(const string& Name) 
            : Name(Name)
        {
            // Nothing here
        }

        template <typename LEType>
        SemanticizerBase::~SemanticizerBase()
        {
            // Nothing here
        }
        
        
        // ExprMgr implementation
        template <typename ExtType, typename SemType>
        template <typename... ArgTypes>
        inline ExprMgr<ExtType, SemType>::ExprMgr(ArgTypes&&... Args)
            : Sem(new SemType(forward<ArgTypes>(Args)...))
        {
            // Nothing here
        }

        template <typename ExtType, typename SemType>
        inline ExprMgr<ExtType, SemType>::~ExprMgr()
        {
            delete Sem;
            ExpSet.clear();
        }

        template <typename ExtType, typename SemType>
        inline ExprMgr<ExtType, SemType>::ExpType
        ExprMgr<ExtType, SemType>::GetCachedOrInsert(const ExpType& Exp)
        {
            auto it = ExpSet.find(Exp);
            if (it == ExpSet.end()) {
                ExpSet.insert(Exp);
                return Exp;
            } else {
                return (*it);
            }            
        }

        template <typename ExtType, typename SemType>
        template <typename T, typename... ArgTypes>
        inline ExprMgr<ExtType, SemType>::ExpType 
        ExprMgr<ExtType, SemType>::GetCachedOrInsert(ArgTypes&&... Args)
        {
            ExpType NewExp = new T(forward<ArgTypes>(Args)...);
            auto it = ExpSet.find(NewExp);
            return GetCachedOrInsert(NewExp);
        }

        template <typename ExtType, typename SemType>
        void ExprMgr<ExtType, SemType>::CheckMgr(const vector<ExpType>& Children) const
        {
            for (auto const& Child : Children) {
                if (Child->GetMgr() != this) {
                    throw ExprTypeError("ExprMgr: I don't own this expression!");
                }
            }
        }

        template <typename ExtType, typename SemType>
        inline ExprMgr<ExtType, SemType>::ExpType 
        ExprMgr<ExtType, SemType>::MakeVal(const string& ValString, i64 ValType,
                                           const ExtType& ExtVal)
        {
            auto Retval =
                GetCachedOrInsert<ConstExpression>(ValString, ValType, ExtVal);
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
