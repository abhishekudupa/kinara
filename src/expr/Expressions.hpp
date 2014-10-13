// Expressions.hpp --- 
// 
// Filename: Expressions.hpp
// Author: Abhishek Udupa
// Created: Mon Jun 30 01:42:36 2014 (-0400)
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

// Classes for expressions 

#if !defined ESMC_EXPRESSIONS_HPP_
#define ESMC_EXPRESSIONS_HPP_

#include <vector>
#include <boost/functional/hash.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <functional>
#include <unordered_map>
#include <unordered_set>

#include "../common/FwdDecls.hpp"
#include "../containers/RefCountable.hpp"
#include "../containers/SmartPtr.hpp"
#include "../containers/RefCache.hpp"
#include "../utils/UIDGenerator.hpp"

#include "ExprTypes.hpp"

// This classes in this file are heavily templatized
// to allow for flexibility via arbitrary extension objects

namespace ESMC {
    namespace Exprs {

        // Comparators
        class ExpressionPtrEquals
        {
        public:
            template <typename E, template <typename> class S>
            inline bool operator () (const ExpressionBase<E, S>* Exp1,
                                     const ExpressionBase<E, S>* Exp2) const
            {
                return Exp1->Equals(Exp2);
            }

            template<typename E, template <typename> class S>
            inline bool operator () (const CSmartPtr<ExpressionBase<E, S>>& Exp1,
                                     const CSmartPtr<ExpressionBase<E, S>>& Exp2) const
            {
                return Exp1->Equals(Exp2);
            }
        };

        class FastExpressionPtrEquals
        {
        public:
            template <typename E, template <typename> class S>
            inline bool operator () (const ExpressionBase<E, S>* Exp1,
                                     const ExpressionBase<E, S>* Exp2) const
            {
                return Exp1->FastEQ(Exp2);
            }

            template<typename E, template <typename> class S>
            inline bool operator () (const CSmartPtr<ExpressionBase<E, S>>& Exp1,
                                     const CSmartPtr<ExpressionBase<E, S>>& Exp2) const
            {
                return Exp1->FastEQ(Exp2);
            }
        };
 

        class ExpressionPtrHasher
        {
        public:
            template <typename E, template <typename> class S>
            inline u64 operator () (const ExpressionBase<E, S>* Exp) const
            {
                return Exp->Hash();
            }

            template<typename E, template <typename> class S>
            inline u64 operator () (const CSmartPtr<ExpressionBase<E, S>>& Exp) const
            {
                return (Exp->Hash());
            }
        };


        class ExpressionPtrCompare
        {
        public:
            template <typename E, template <typename> class S>
            inline bool operator () (const ExpressionBase<E, S>* Exp1, 
                                     const ExpressionBase<E, S>* Exp2) const
            {
                return Exp1->LT(Exp2);
            }

            template<typename E, template <typename> class S>
            inline bool operator () (const CSmartPtr<ExpressionBase<E, S>>& Exp1,
                                     const CSmartPtr<ExpressionBase<E, S>>& Exp2) const
            {
                return Exp1->LT(Exp2);
            }
        };

        // An empty extension type
        struct EmptyExtType 
        {
            // Nothing here
        };

        // Base class for extension lists
        class ExtListExtBase : public RefCountable
        {
        public:
            ExtListExtBase();
            virtual ~ExtListExtBase();
            
            virtual string ToString() const = 0;

            // Downcasts
            template <typename T>
            inline T* As() 
            { 
                return dynamic_cast<T*>(this);
            }

            template <typename T>
            inline T* As() const
            { 
                return dynamic_cast<const T*>(this);
            }

            template <typename T>
            inline T* SAs() 
            { 
                return static_cast<T*>(this);
            }

            template <typename T>
            inline T* SAs() const
            { 
                return static_cast<const T*>(this);
            }
        };

        template <typename E, template <typename> class S>
        class ExpressionBase : public RefCountable
        {
            friend class ExprMgr<E, S>;
        private:
            ExprMgr<E, S>* Mgr;
            mutable bool HashValid;
            mutable ExprTypeRef ExpType;

        public:
            mutable E ExtensionData;
            
        protected:
            mutable u64 HashCode;

        public:
            inline ExpressionBase(ExprMgr<E, S>* Manager, 
                                  const E& ExtData = E());
            virtual inline ~ExpressionBase();

            inline ExprMgr<E, S>* GetMgr() const;
            inline u64 Hash() const;
            inline const ExprTypeRef& GetType() const;
            inline void SetType(const ExprTypeRef& Type) const;
            inline bool Equals(const ExpressionBase<E, S>* Other) const;
            inline bool NEquals(const ExpressionBase<E, S>* Other) const;
            inline bool LT(const ExpressionBase<E, S>* Other) const;
            inline bool LE(const ExpressionBase<E, S>* Other) const;
            inline bool GE(const ExpressionBase<E, S>* Other) const;
            inline bool GT(const ExpressionBase<E, S>* Other) const;
            inline string ToString() const;

            // Abstract methods
        protected:
            virtual void ComputeHash() const = 0;

        public:
            virtual i32 Compare(const ExpressionBase<E, S>* Other) const = 0;
            virtual void Accept(ExpressionVisitorBase<E, S>* Visitor) const = 0;
            // Fast eq which assumes that children can be compared for
            // equality by simple pointer equality
            virtual bool FastEQ(const ExpressionBase<E, S>* Other) const;
        
            // Downcasts
            template <template <typename, template <typename> class> class U>
            inline U<E, S>* As()
            {
                return dynamic_cast<U<E, S>*>(this);
            }

            template <template <typename, template <typename> class> class U> 
            inline const U<E, S>* As() const
            {
                return dynamic_cast<const U<E, S>*>(this);
            }

            template <template <typename, template <typename> class> class U> 
            inline U<E, S>* SAs()
            {
                return static_cast<U<E, S>*>(this);
            }

            template <template <typename, template <typename> class> class U> 
            inline const U<E, S>* SAs() const
            {
                return static_cast<const U<E, S>*>(this);
            }

            template <template <typename, template <typename> class> class U>
            inline bool Is() const
            {
                return (dynamic_cast<const U<E, S>*>(this) != nullptr);
            }
        };

        // Specialization for extension lists
        template <template <typename> class S>
        class ExpressionBase<ExtListT, S> : public RefCountable
        {
            friend class ExprMgr<ExtListT, S>;
        private:
            ExprMgr<ExtListT, S>* Mgr;
            mutable bool HashValid;
            mutable i64 ExpType;

        public:
            mutable ExtListT ExtensionData;
            
        protected:
            mutable u64 HashCode;

        public:
            inline ExpressionBase(ExprMgr<ExtListT, S>* Manager, 
                                  const ExtListT& ExtData = ExtListT());
            virtual inline ~ExpressionBase();

            inline ExprMgr<ExtListT, S>* GetMgr() const;
            inline u64 Hash() const;
            inline i64 GetType() const;
            inline void SetType(i64 Type) const;
            inline bool Equals(const ExpressionBase<ExtListT, S>* Other) const;
            inline bool NEquals(const ExpressionBase<ExtListT, S>* Other) const;
            inline bool LT(const ExpressionBase<ExtListT, S>* Other) const;
            inline bool LE(const ExpressionBase<ExtListT, S>* Other) const;
            inline bool GE(const ExpressionBase<ExtListT, S>* Other) const;
            inline bool GT(const ExpressionBase<ExtListT, S>* Other) const;
            inline string ToString() const;

            // Abstract methods
        protected:
            virtual void ComputeHash() const = 0;

        public:
            virtual i32 Compare(const ExpressionBase<ExtListT, S>* Other) const = 0;
            virtual void Accept(ExpressionVisitorBase<ExtListT, S>* Visitor) const = 0;
            // Fast eq which assumes that children can be compared for
            // equality by simple pointer equality
            virtual bool FastEQ(const ExpressionBase<ExtListT, S>* Other) const;
        
            // Downcasts
            template <template <typename, template <typename> class> class U>
            inline U<ExtListT, S>* As()
            {
                return dynamic_cast<U<ExtListT, S>*>(this);
            }

            template <template <typename, template <typename> class> class U> 
            inline const U<ExtListT, S>* As() const
            {
                return dynamic_cast<const U<ExtListT, S>*>(this);
            }

            template <template <typename, template <typename> class> class U> 
            inline U<ExtListT, S>* SAs()
            {
                return static_cast<U<ExtListT, S>*>(this);
            }

            template <template <typename, template <typename> class> class U> 
            inline const U<ExtListT, S>* SAs() const
            {
                return static_cast<const U<ExtListT, S>*>(this);
            }

            template <template <typename, template <typename> class> class U> 
            inline bool Is() const
            {
                return (dynamic_cast<const U<ExtListT, S>*>(this) != nullptr);
            }            

            // Extension list accessors and manipulators
            template <typename U>
            inline const ExtListExtRef& GetExtension() const;

            template <typename U>
            inline vector<ExtListExtRef> GetExtensions() const;
            
            inline void PurgeExtension(const ExtListExtRef& Ext) const;
            
            template <typename U>
            inline void PurgeExtensionsOfType() const;

            inline void PurgeAllExtensions() const;
        };


        // Type V must have the equality and < operator defined,
        // and must have a hash defined.
        // Type V must also be assignable, default constructible 
        // and copy constructible
        template <typename E, template <typename> class S>
        class ConstExpression : public ExpressionBase<E, S>
        {
        private:
            string ConstValue;
            ExprTypeRef ConstType;

        public:
            inline ConstExpression(ExprMgr<E, S>* Mgr,
                                   const string& ConstValue,
                                   const ExprTypeRef& ConstType, 
                                   const E& ExtData = E());
            inline virtual ~ConstExpression();

            inline const string& GetConstValue() const;
            inline const ExprTypeRef&  GetConstType() const;

        protected:
            inline virtual void ComputeHash() const override;
        
        public:
            inline virtual i32 Compare(const ExpressionBase<E, S>* Other) const override;
            inline virtual void Accept(ExpressionVisitorBase<E, S>* Visitor) const override;
        };
    

        template <typename E, template <typename> class S>
        class VarExpression : public ExpressionBase<E, S>
        {
        private:
            string VarName;
            ExprTypeRef VarType;

        public:
            inline VarExpression(ExprMgr<E, S>* Manager, 
                                 const string& VarName,
                                 const ExprTypeRef& VarType, 
                                 const E& ExtData = E());
            inline virtual ~VarExpression();
            inline const string& GetVarName() const;
            inline const ExprTypeRef& GetVarType() const;

        protected:
            inline virtual void ComputeHash() const override;

        public:
            inline virtual i32 Compare(const ExpressionBase<E, S>* Other) const override;
            inline virtual void Accept(ExpressionVisitorBase<E, S>* Visitor) const override;
        };

        template<typename E, template <typename> class S>
        class BoundVarExpression : public ExpressionBase<E, S>
        {
        private:
            ExprTypeRef VarType;
            u64 VarIdx;

        public:
            inline BoundVarExpression(ExprMgr<E, S>* Manager, 
                                      const ExprTypeRef& VarType, 
                                      i64 VarIdx,
                                      const E& ExtData = E());
            inline virtual ~BoundVarExpression();
            inline u64 GetVarIdx() const;
            inline const ExprTypeRef& GetVarType() const;
        
        protected:
            inline virtual void ComputeHash() const override;
        
        public:
            inline virtual i32 Compare(const ExpressionBase<E, S>* Other) const override;
            inline virtual void Accept(ExpressionVisitorBase<E, S>* Visitor) const override;
        };


        template<typename E, template <typename> class S>
        class OpExpression : public ExpressionBase<E, S>
        {
        private:
            i64 OpCode;
            vector<Expr<E, S>> Children;

        public:
            inline OpExpression(ExprMgr<E, S>* Manager, 
                                i64 OpCode,
                                const vector<Expr<E, S>>& Children,
                                const E& ExtData = E());

            inline virtual ~OpExpression();
            inline i64 GetOpCode() const;
            inline const vector<Expr<E, S>>& GetChildren() const;

        protected:
            inline virtual void ComputeHash() const override;

        public:
            inline virtual i32 Compare(const ExpressionBase<E, S>* Other) const override;
            inline virtual void Accept(ExpressionVisitorBase<E, S>* Visitor) const override;
            inline virtual bool FastEQ(const ExpressionBase<E, S>* Other) const override;
        };


        template<typename E, template <typename> class S>
        class QuantifiedExpressionBase : public ExpressionBase<E, S>
        {
        private:
            vector<ExprTypeRef> QVarTypes;
            Expr<E, S> QExpression;

        public:
            inline QuantifiedExpressionBase(ExprMgr<E, S>* Manager,
                                            const vector<ExprTypeRef>& QVarTypes,
                                            const Expr<E, S>& QExpression,
                                            const E& ExtData = E());
            inline virtual ~QuantifiedExpressionBase();
            inline const vector<ExprTypeRef>& GetQVarTypes() const;
            inline const Expr<E, S>& GetQExpression() const;

        protected:
            inline i32 CompareInternal(const QuantifiedExpressionBase<E, S>* Other) const;
            inline void ComputeHashInternal() const;
            inline bool FastEQInternal(const QuantifiedExpressionBase<E, S>* Other) const;

        public:
            virtual bool IsForAll() const = 0;
            virtual bool IsExists() const = 0;
        };


        template<typename E, template <typename> class S>
        class EQuantifiedExpression : public QuantifiedExpressionBase<E, S>
        {
        public:
            using QuantifiedExpressionBase<E, S>::QuantifiedExpressionBase;
            inline virtual ~EQuantifiedExpression();

        protected:
            inline virtual void ComputeHash() const override;
        
        public:
            inline virtual i32 Compare(const ExpressionBase<E, S>* Other) const override;
            inline virtual void Accept(ExpressionVisitorBase<E, S>* Visitor) const override;
            inline virtual bool IsForAll() const override;
            inline virtual bool IsExists() const override;
            inline virtual bool FastEQ(const ExpressionBase<E, S>* Other) const override;
        };


        template<typename E, template <typename> class S>
        class AQuantifiedExpression : public QuantifiedExpressionBase<E, S>
        {
        public:
            using QuantifiedExpressionBase<E, S>::QuantifiedExpressionBase;
            inline virtual ~AQuantifiedExpression();

        protected:
            inline virtual void ComputeHash() const override;
        
        public:
            inline virtual i32 Compare(const ExpressionBase<E, S>* Other) const override;
            inline virtual void Accept(ExpressionVisitorBase<E, S>* Visitor) const override;
            inline virtual bool IsForAll() const override;
            inline virtual bool IsExists() const override;
            inline virtual bool FastEQ(const ExpressionBase<E, S>* Other) const override;
        };

        // The manager classes

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

        template <typename E, template <typename> class S>
        class ExprMgr
        {
        public:
            typedef S<E> SemT;
            typedef typename SemT::LExpT LExpT;
            typedef ExprTypeRef TypeT;
            typedef Expr<E, S> ExpT;
            typedef ExprI<E, S> IExpT;

            typedef unordered_map<ExpT, ExpT, 
                                  ExpressionPtrHasher,
                                  ExpressionPtrEquals> SubstMapT;

            typedef RefCache<ExpressionBase<E, S>, ExpressionPtrHasher,
                             ExpressionPtrEquals, CSmartPtr> ExpCacheT;

            typedef RefCache<ExprTypeBase, ExprTypePtrHasher,
                             ExprTypePtrEquals, CSmartPtr> TypeCacheT;

            typedef unordered_set<ExpT, ExpressionPtrHasher, FastExpressionPtrEquals> ExpSetT;

        private:
            SemT* Sem;
            ExpCacheT ExpCache;
            TypeCacheT TypeCache;
            unordered_map<string, TypeT> NamedTypes;

            ExprTypeRef BoolType;
            ExprTypeRef IntType;
            
            inline void CheckMgr(const vector<ExpT>& Children) const;
            inline void CheckMgr(const ExpT& Exp) const;
            template <template <typename, template<typename> class> class T>
            inline ExpT MakeQExpression(const vector<TypeT>& QVars,
                                        const ExpT& QExpr,
                                        const E& ExtVal);

            // Insert the expression and all subexpressions
            // into the set of expressions owned by this manager
            inline ExpT Internalize(const ExpT& Exp);

        public:
            template <typename... ArgTypes>
            inline ExprMgr(ArgTypes&&... Args);
            inline ~ExprMgr();
            
            template<typename T, typename... ArgTypes>
            inline TypeT MakeType(ArgTypes&&... Args);
            
            inline TypeT GetNamedType(const string& Name) const;

            inline TypeT InstantiateType(const TypeT& ParamType,
                                         const vector<ExpT>& ParamValues);

            inline ExpT MakeTrue(const E& ExtVal = E());
            inline ExpT MakeFalse(const E& ExtVal = E());

            inline ExpT MakeVal(const string& ValString, const TypeT& ValType,
                                const E& ExtVal = E());
            
            inline ExpT MakeVar(const string& VarName, const TypeT& VarType,
                                const E& ExtVal = E());

            inline ExpT MakeBoundVar(const TypeT& VarType, i64 VarIdx,
                                     const E& ExtVal = E());

            inline ExpT MakeExpr(i64 OpCode, const vector<ExpT>& Children,
                                 const E& ExtVal = E());

            inline ExpT MakeExpr(i64 OpCode, const ExpT& Child1,
                                 const E& ExtVal = E());

            inline ExpT MakeExpr(i64 OpCode, const ExpT& Child1,
                                 const ExpT& Child2,
                                 const E& ExtVal = E());

            inline ExpT MakeExpr(i64 OpCode, const ExpT& Child1,
                                 const ExpT& Child2, const ExpT& Child3,
                                 const E& ExtVal = E());

            inline ExpT MakeExists(const vector<TypeT>& QVarTypes, 
                                   const ExpT& QExpr,
                                   const E& ExtVal = E());

            inline ExpT MakeForAll(const vector<TypeT>& QVarTypes,
                                   const ExpT& QExpr,
                                   const E& ExtVal = E());

            inline i64 MakeUninterpretedFunction(const string& Name, 
                                                 const vector<TypeT>& Range,
                                                 const TypeT& Domain);

            template <class T, typename... ArgTypes>
            inline ExpT ApplyTransform(const ExpT& Exp, ArgTypes&&... Args);

            inline SemT* GetSemanticizer() const;
            template <typename... ArgTypes>
            inline LExpT LowerExpr(const ExpT& Exp, ArgTypes&&... Args);
            template <typename... ArgTypes>
            inline ExpT RaiseExpr(const LExpT& Exp, ArgTypes&&... Args);
            inline ExpT ElimQuantifiers(const ExpT& Exp);
            inline ExpT UnrollQuantifiers(const ExpT& Exp);
            inline ExpT Simplify(const ExpT& Exp);
            inline ExpT Substitute(const SubstMapT& Subst, const ExpT& Exp);
            inline ExpSetT
            Gather(const ExpT& Exp, 
                   const function<bool(const ExpressionBase<E, S>*)>& Pred) const;

            inline void GC();

            static inline ExprMgr* Make();
        };

        // Visitor classes
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

        // The substitution transform, which is the only
        // built-in transform. We only allow variable substitutions
        template <typename E, template <typename> class S>
        class Substitutor : ExpressionVisitorBase<E, S>
        {
        private:
            typedef ExprMgr<E, S> MgrType;
            typedef typename MgrType::ExpT ExpT;
            typedef typename MgrType::SubstMapT SubstMapT;

            MgrType* Mgr;
            SubstMapT Subst;
            vector<typename ExprMgr<E, S>::ExpT> SubstStack;

        public:
            inline Substitutor(MgrType* Mgr, const SubstMapT& Subst);
            inline virtual ~Substitutor();

            inline virtual void VisitVarExpression(const VarExpression<E, S>* Exp) override;
            inline virtual void VisitConstExpression(const ConstExpression<E, S>* Exp) override;
            inline virtual void VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp) 
                override;
            inline virtual void VisitOpExpression(const OpExpression<E, S>* Exp) override;
            inline virtual void VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp)
                override;
            inline virtual void VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp)
                override;

            inline static ExpT Do(MgrType* Mgr, 
                                  const ExpT& Exp, 
                                  const SubstMapT& SubstMap);
        };

        template <typename E, template <typename> class S>
        class Gatherer : ExpressionVisitorBase<E, S>
        {
        private:
            typedef Expr<E, S> ExpT;
            typedef typename ExprMgr<E, S>::ExpSetT ExpSetT;
            function<bool(const ExpressionBase<E, S>*)> Pred;
            ExpSetT GatheredExps;

        public:
            inline Gatherer(const function<bool(const ExpressionBase<E, S>*)>& Pred);
            inline virtual ~Gatherer();

            inline virtual void VisitVarExpression(const VarExpression<E, S>* Exp) override;
            inline virtual void VisitConstExpression(const ConstExpression<E, S>* Exp) override;
            inline virtual void VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp) 
                override;
            inline virtual void VisitOpExpression(const OpExpression<E, S>* Exp) override;
            inline virtual void VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp) 
                override;
            inline virtual void VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp) 
                override;
            
            static inline ExpSetT
            Do(const ExpT& Exp, const function<bool(const ExpressionBase<E, S>*)>& Pred);
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
        void ExpressionVisitorBase<E, S>::VisitConstExpression
        (
         const ConstExpression<E, S>* Exp
         )
        {
            return;
        }

        template <typename E, template <typename> class S>
        void ExpressionVisitorBase<E, S>::VisitVarExpression
        (
         const VarExpression<E, S>* Exp
         )
        {
            return;
        }

        template <typename E, template <typename> class S>
        void ExpressionVisitorBase<E, S>::VisitBoundVarExpression
        (
         const BoundVarExpression<E, S>* Exp
         )
        {
            return;
        }

        template <typename E, template <typename> class S>
        void ExpressionVisitorBase<E, S>::VisitOpExpression
        (
         const OpExpression<E, S>* Exp
         )
        {
            auto const& Children = Exp->GetChildren();
            for (auto const& Child : Children) {
                Child->Accept(this);
            }
            return;
        }

        template <typename E, template <typename> class S>
        void ExpressionVisitorBase<E, S>::VisitAQuantifiedExpression
        (
         const AQuantifiedExpression<E, S>* Exp
         )
        {
            Exp->GetQExpression()->Accept(this);
        }

        template <typename E, template <typename> class S>
        void ExpressionVisitorBase<E, S>::VisitEQuantifiedExpression
        (
         const EQuantifiedExpression<E, S>* Exp
         )
        {
            Exp->GetQExpression()->Accept(this);
        }

        // Substitutor implementation
        template <typename E, template <typename> class S>
        inline Substitutor<E, S>::Substitutor(MgrType* Mgr, const SubstMapT& Subst)
            : ExpressionVisitorBase<E, S>("Substitutor"), Mgr(Mgr), Subst(Subst)
        {
            // Nothing here
        }
        
        template <typename E, template <typename> class S>
        inline Substitutor<E,S>::~Substitutor()
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline void 
        Substitutor<E, S>::VisitVarExpression(const VarExpression<E, S>* Exp)
        {
            auto it = Subst.find(Exp);
            if (it != Subst.end()) {
                SubstStack.push_back(it->second);
            } else {
                SubstStack.push_back(Exp);
            }
        }

        template <typename E, template <typename> class S>
        inline void
        Substitutor<E, S>::VisitConstExpression(const ConstExpression<E,S>* Exp)
        {
            SubstStack.push_back(Exp);
        }

        template <typename E, template <typename> class S>
        inline void
        Substitutor<E, S>::VisitBoundVarExpression(const BoundVarExpression<E,S>* Exp)
        {
            auto it = Subst.find(Exp);
            if (it != Subst.end()) {
                SubstStack.push_back(it->second);
            } else {
                SubstStack.push_back(Exp);
            }
        }

        template <typename E, template <typename> class S>
        inline void 
        Substitutor<E, S>::VisitOpExpression(const OpExpression<E, S>* Exp)
        {
            ExpressionVisitorBase<E,S>::VisitOpExpression(Exp);
            const u32 NumChildren = Exp->GetChildren().size();
            vector<ExpT> SubstChildren(NumChildren);
            for (u32 i = 0; i < NumChildren; ++i) {
                SubstChildren[NumChildren - i - 1] = SubstStack.back();
                SubstStack.pop_back();
            }
            SubstStack.push_back(Mgr->MakeExpr(Exp->GetOpCode(),
                                               SubstChildren));
        }

        template <typename E, template <typename> class S>
        inline void 
        Substitutor<E, S>::VisitEQuantifiedExpression(const EQuantifiedExpression<E,S>* Exp)
        {
            Exp->GetQExpression()->Accept(this);
            auto SubstQExpr = SubstStack.back();
            SubstStack.pop_back();
            SubstStack.push_back(Mgr->MakeExists(Exp->GetQVarTypes(),
                                                 SubstQExpr));
        }

        template <typename E, template <typename> class S>
        inline void 
        Substitutor<E, S>::VisitAQuantifiedExpression(const AQuantifiedExpression<E,S>* Exp)
        {
            Exp->GetQExpression()->Accept(this);
            auto SubstQExpr = SubstStack.back();
            SubstStack.pop_back();
            SubstStack.push_back(Mgr->MakeForAll(Exp->GetQVarTypes(),
                                                 SubstQExpr));
        }

        template <typename E, template <typename> class S>
        inline typename Substitutor<E, S>::ExpT
        Substitutor<E, S>::Do(MgrType* Mgr,
                              const ExpT& Exp, const SubstMapT& Subst)
        {
            Substitutor TheSubstitutor(Mgr, Subst);
            Exp->Accept(&TheSubstitutor);
            return TheSubstitutor.SubstStack[0];
        }

        // Gatherer implementation
        template <typename E, template <typename> class S>
        inline Gatherer<E, S>::Gatherer(const function<bool(const ExpressionBase<E, S>*)>& Pred)
            : ExpressionVisitorBase<E, S>("Gatherer"), Pred(Pred)
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline Gatherer<E, S>::~Gatherer()
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline void Gatherer<E, S>::VisitVarExpression(const VarExpression<E, S>* Exp)
        {
            ExpressionVisitorBase<E, S>::VisitVarExpression(Exp);
            if (Pred(Exp)) {
                GatheredExps.insert(Exp);
            }
        }

        template <typename E, template <typename> class S>
        inline void Gatherer<E, S>::VisitConstExpression(const ConstExpression<E, S>* Exp)
        {
            ExpressionVisitorBase<E, S>::VisitConstExpression(Exp);
            if (Pred(Exp)) {
                GatheredExps.insert(Exp);
            }
        }

        template <typename E, template <typename> class S>
        inline void Gatherer<E, S>::VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp)
        {
            ExpressionVisitorBase<E, S>::VisitBoundVarExpression(Exp);
            if (Pred(Exp)) {
                GatheredExps.insert(Exp);
            }
        }

        template <typename E, template <typename> class S>
        inline void Gatherer<E, S>::VisitOpExpression(const OpExpression<E, S>* Exp)
        {
            ExpressionVisitorBase<E, S>::VisitOpExpression(Exp);
            if (Pred(Exp)) {
                GatheredExps.insert(Exp);
            }
        }

        template <typename E, template <typename> class S>
        inline void Gatherer<E, S>::VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp)
        {
            ExpressionVisitorBase<E, S>::VisitEQuantifiedExpression(Exp);
            if (Pred(Exp)) {
                GatheredExps.insert(Exp);
            }
        }

        template <typename E, template <typename> class S>
        inline void Gatherer<E, S>::VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp)
        {
            ExpressionVisitorBase<E, S>::VisitAQuantifiedExpression(Exp);
            if (Pred(Exp)) {
                GatheredExps.insert(Exp);
            }
        }

        template <typename E, template <typename> class S>
        inline typename Gatherer<E, S>::ExpSetT
        Gatherer<E, S>::Do(const ExpT& Exp, const function<bool (const ExpressionBase<E, S> *)>& Pred)
        {
            Gatherer<E, S> TheGatherer(Pred);
            Exp->Accept(&TheGatherer);
            return TheGatherer.GatheredExps;
        }


        // ExpressionBase implementation
        template <typename E, template <typename> class S>
        inline ExpressionBase<E, S>::ExpressionBase(ExprMgr<E, S>* Manager,
                                                    const E& ExtVal)
            : Mgr(Manager), HashValid(false),
              ExpType(S<E>::InvalidType), ExtensionData(ExtVal),
              HashCode(0)
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline ExpressionBase<E, S>::~ExpressionBase()
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline ExprMgr<E, S>*
        ExpressionBase<E, S>::GetMgr() const 
        {
            return Mgr;
        }

        template <typename E, template <typename> class S>
        inline u64 ExpressionBase<E, S>::Hash() const
        {
            if (!HashValid) {
                ComputeHash();
                HashValid = true;
            }
            return HashCode;
        }

        template <typename E, template <typename> class S>
        inline const ExprTypeRef& 
        ExpressionBase<E, S>::GetType() const
        {
            return ExpType;
        }

        template <typename E, template <typename> class S>
        inline void 
        ExpressionBase<E, S>::SetType(const ExprTypeRef& Type) const
        {
            ExpType = Type;
        }

        template <typename E, template <typename> class S>
        inline bool ExpressionBase<E, S>::Equals(const ExpressionBase<E, S>* Other) const
        {
            if (Hash() != Other->Hash()) {
                return false;
            }
            return (Compare(Other) == 0);
        }

        template <typename E, template <typename> class S>
        inline bool ExpressionBase<E, S>::NEquals(const ExpressionBase<E, S>* Other) const
        {
            if (Hash() != Other->Hash()) {
                return true;
            }
            return (Compare(Other) != 0);
        }

        template <typename E, template <typename> class S>
        inline bool ExpressionBase<E, S>::LT(const ExpressionBase<E, S>* Other) const
        {
            return (Compare(Other) < 0);
        }

        template <typename E, template <typename> class S>
        inline bool ExpressionBase<E, S>::LE(const ExpressionBase<E, S>* Other) const
        {
            return (Compare(Other) <= 0);
        }

        template <typename E, template <typename> class S>
        inline bool ExpressionBase<E, S>::GT(const ExpressionBase<E, S>* Other) const
        {
            return (Compare(Other) > 0);
        }

        template <typename E, template <typename> class S>
        inline bool ExpressionBase<E, S>::GE(const ExpressionBase<E, S>* Other) const
        {
            return (Compare(Other) >= 0);
        }

        template <typename E, template <typename> class S>
        inline bool ExpressionBase<E, S>::FastEQ(const ExpressionBase<E, S>* Other) const
        {
            return (Equals(Other));
        }

        template <typename E, template <typename> class S>
        inline string ExpressionBase<E, S>::ToString() const
        {
            auto Sem = Mgr->GetSemanticizer();
            return (Sem->ExprToString(this));
        }

        
        // ExpressionBase with ExtList specialization
        template <template <typename> class S>
        inline ExpressionBase<ExtListT, S>::ExpressionBase(ExprMgr<ExtListT, S>* Manager,
                                                          const ExtListT& ExtVal)
            : Mgr(Manager), HashValid(false),
              ExpType(-1), ExtensionData(ExtVal),
              HashCode(0)
        {
            // Nothing here
        }

        template <template <typename> class S>
        inline ExpressionBase<ExtListT, S>::~ExpressionBase()
        {
            // Nothing here
        }

        template <template <typename> class S>
        inline ExprMgr<ExtListT, S>*
        ExpressionBase<ExtListT, S>::GetMgr() const 
        {
            return Mgr;
        }

        template <template <typename> class S>
        inline u64 ExpressionBase<ExtListT, S>::Hash() const
        {
            if (!HashValid) {
                ComputeHash();
                HashValid = true;
            }
            return HashCode;
        }

        template <template <typename> class S>
        inline i64 ExpressionBase<ExtListT, S>::GetType() const
        {
            return ExpType;
        }

        template <template <typename> class S>
        inline void ExpressionBase<ExtListT, S>::SetType(i64 Type) const
        {
            ExpType = Type;
        }

        template <template <typename> class S>
        inline bool ExpressionBase<ExtListT, S>::Equals(const ExpressionBase<ExtListT, S>* Other) 
            const
        {
            if (Hash() != Other->Hash()) {
                return false;
            }
            return (Compare(Other) == 0);
        }

        template <template <typename> class S>
        inline bool ExpressionBase<ExtListT, S>::NEquals(const ExpressionBase<ExtListT, S>* Other) 
            const
        {
            if (Hash() != Other->Hash()) {
                return true;
            }
            return (Compare(Other) != 0);
        }

        template <template <typename> class S>
        inline bool ExpressionBase<ExtListT, S>::LT(const ExpressionBase<ExtListT, S>* Other) const
        {
            return (Compare(Other) < 0);
        }

        template <template <typename> class S>
        inline bool ExpressionBase<ExtListT, S>::LE(const ExpressionBase<ExtListT, S>* Other) const
        {
            return (Compare(Other) <= 0);
        }

        template <template <typename> class S>
        inline bool ExpressionBase<ExtListT, S>::GT(const ExpressionBase<ExtListT, S>* Other) const
        {
            return (Compare(Other) > 0);
        }

        template <template <typename> class S>
        inline bool ExpressionBase<ExtListT, S>::GE(const ExpressionBase<ExtListT, S>* Other) const
        {
            return (Compare(Other) >= 0);
        }

        template <template <typename> class S>
        inline bool ExpressionBase<ExtListT, S>::FastEQ(const ExpressionBase<ExtListT, S>* Other) 
            const
        {
            return (Equals(Other));
        }

        template <template <typename> class S>
        inline string ExpressionBase<ExtListT, S>::ToString() const
        {
            auto Sem = Mgr->GetSemanticizer();
            return (Sem->ExprToString(this));
        }

        // Additional methods specific to ExtList Expressions
        template <template <typename> class S>
        template <typename U>
        inline const ExtListExtRef& 
        ExpressionBase<ExtListT, S>::GetExtension() const
        {
            for (auto const& Ext : ExtensionData) {
                if (Ext->template As<U>() != nullptr) {
                    return Ext;
                }
            }
        }

        template <template <typename> class S>
        template <typename U>
        inline vector<ExtListExtRef>
        ExpressionBase<ExtListT, S>::GetExtensions() const
        {
            vector<ExtListExtRef> Retval;

            for(auto const& Ext : ExtensionData) {
                if (Ext->template As<U>() != nullptr) {
                    Retval.push_back(Ext);
                }
            }
            return Retval;
        }

        template <template <typename> class S>
        inline void ExpressionBase<ExtListT, S>::PurgeExtension(const ExtListExtRef& Ext) const
        {
            ExtensionData.remove(Ext);
        }

        template <template <typename> class S>
        template <typename U>
        inline void
        ExpressionBase<ExtListT, S>::PurgeExtensionsOfType() const
        {
            vector<ExtListT::iterator> ToRemove;

            for (auto it = ExtensionData.begin(); it != ExtensionData.end(); ++it) {
                if ((*it)->template As<U>() != nullptr) {
                    ToRemove.push_back(it);
                }
            }
            
            for (auto const& Elem : ToRemove) {
                ExtensionData.erase(Elem);
            }
        }

        template <template <typename> class S>
        inline void 
        ExpressionBase<ExtListT, S>::PurgeAllExtensions() const
        {
            ExtensionData.clear();
        }

        // ConstExpression implementation
        template <typename E, template <typename> class S>
        inline ConstExpression<E, S>::ConstExpression(ExprMgr<E, S>* Manager, 
                                                      const string& ConstValue,
                                                      const ExprTypeRef& ConstType, 
                                                      const E& ExtVal)
            : ExpressionBase<E, S>(Manager, ExtVal), 
              ConstValue(ConstValue), 
              ConstType(ConstType)
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline ConstExpression<E, S>::~ConstExpression()
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline const string& ConstExpression<E, S>::GetConstValue() const
        {
            return ConstValue;
        }

        template <typename E, template <typename> class S>
        inline const ExprTypeRef& 
        ConstExpression<E, S>::GetConstType() const
        {
            return ConstType;
        }

        template <typename E, template <typename> class S>
        inline i32 ConstExpression<E, S>::Compare(const ExpressionBase<E, S>* Other) const
        {
            auto OtherAsConst = Other->template As<ESMC::Exprs::ConstExpression>();
            // All other exp types > ConstExpression
            if (OtherAsConst == nullptr) {
                return -1;
            }
            if (ConstValue < OtherAsConst->ConstValue) {
                return -1;
            } else if (ConstValue > OtherAsConst->ConstValue) {
                return 1;
            } else if (ConstType < OtherAsConst->ConstType) {
                return -1;
            } else if (ConstType > OtherAsConst->ConstType) {
                return 1;
            } else {
                return 0;
            }
        }

        template <typename E, template <typename> class S>
        inline void ConstExpression<E, S>::ComputeHash() const
        {
            this->HashCode = 0;
            boost::hash_combine(this->HashCode, ConstValue);
            boost::hash_combine(this->HashCode, ConstType->Hash());
        }

        template <typename E, template <typename> class S>
        inline void ConstExpression<E, S>::Accept(ExpressionVisitorBase<E, S>* Visitor) const 
        {
            Visitor->VisitConstExpression(this);
        }

        // VarExpression implementation
        template <typename E, template <typename> class S>
        inline VarExpression<E, S>::VarExpression(ExprMgr<E, S>* Manager, 
                                                  const string& VarName,
                                                  const ExprTypeRef& VarType, 
                                                  const E& ExtVal)
            : ExpressionBase<E, S>(Manager, ExtVal),
              VarName(VarName), VarType(VarType)
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline VarExpression<E, S>::~VarExpression()
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline const string& VarExpression<E, S>::GetVarName() const 
        {
            return VarName;
        }

        template <typename E, template <typename> class S>
        inline const ExprTypeRef&
        VarExpression<E, S>::GetVarType() const
        {
            return VarType;
        }

        template <typename E, template <typename> class S>
        inline i32 VarExpression<E, S>::Compare(const ExpressionBase<E, S>* Other) const
        {
            auto OtherAsVar = Other->template As<ESMC::Exprs::VarExpression>();
            auto OtherAsConst = Other->template As<ConstExpression>();
            // ConstExpression < VarExpression
            if (OtherAsConst != nullptr) {
                return 1;
            }
            // All other expressions > VarExpression
            if (OtherAsVar == nullptr) {
                return -1;
            }

            if (VarName < OtherAsVar->VarName) {
                return -1;
            } else if (VarName > OtherAsVar->VarName) {
                return 1;
            } else if (VarType < OtherAsVar->VarType) {
                return -1;
            } else if (VarType > OtherAsVar->VarType) {
                return 1; 
            } else {
                return 0;
            }
        }

        template <typename E, template <typename> class S>
        inline void VarExpression<E, S>::ComputeHash() const
        {
            this->HashCode = 0;
            boost::hash_combine(this->HashCode, VarName);
            boost::hash_combine(this->HashCode, VarType->Hash());
        }

        template <typename E, template <typename> class S>
        inline void VarExpression<E, S>::Accept(ExpressionVisitorBase<E, S>* Visitor) const
        {
            Visitor->VisitVarExpression(this);
        }

    
        // BoundVarExpression implementation
        template <typename E, template <typename> class S>
        inline BoundVarExpression<E, S>::BoundVarExpression(ExprMgr<E, S>* Manager, 
                                                            const ExprTypeRef& VarType, 
                                                            i64 VarIdx,
                                                            const E& ExtVal)
            : ExpressionBase<E, S>(Manager, ExtVal),
              VarType(VarType), VarIdx(VarIdx)
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline BoundVarExpression<E, S>::~BoundVarExpression()
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline u64 BoundVarExpression<E, S>::GetVarIdx() const
        {
            return VarIdx;
        }

        template <typename E, template <typename> class S>
        inline const ExprTypeRef& 
        BoundVarExpression<E, S>::GetVarType() const
        {
            return VarType;
        }        

        template <typename E, template <typename> class S>
        inline i32 BoundVarExpression<E, S>::Compare(const ExpressionBase<E, S>* Other) const
        {
            if ((Other->template As<ConstExpression>() != nullptr) ||
                (Other->template As<VarExpression>() != nullptr)) {
                return 1;
            }
            auto OtherAsBound = Other->template As<ESMC::Exprs::BoundVarExpression>();
            // VarExpression < BoundVarExpression
            if (OtherAsBound == nullptr) {
                return -1;
            } else {
                if (VarIdx < OtherAsBound->VarIdx) {
                    return -1;
                } else if (VarIdx > OtherAsBound->VarIdx) {
                    return 1;
                } else if (VarType < OtherAsBound->VarType) {
                    return -1;
                } else if (VarType > OtherAsBound->VarType) {
                    return 1;
                } else {
                    return 0;
                }
            }
        }

        template <typename E, template <typename> class S>
        inline void BoundVarExpression<E, S>::ComputeHash() const
        {
            this->HashCode = 0;
            boost::hash_combine(this->HashCode, VarIdx);
            boost::hash_combine(this->HashCode, VarType->Hash());
        }

        template <typename E, template <typename> class S>
        inline void BoundVarExpression<E, S>::Accept(ExpressionVisitorBase<E, S>* Visitor) const
        {
            Visitor->VisitBoundVarExpression(this);
        }

        // OpExpression implementation
        template <typename E, template <typename> class S>
        inline OpExpression<E, S>::OpExpression(ExprMgr<E, S>* Manager, 
                                                i64 OpCode,
                                                const vector<Expr<E, S>>& Children,
                                                const E& ExtVal)
            : ExpressionBase<E, S>(Manager, ExtVal), OpCode(OpCode), Children(Children)
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline OpExpression<E, S>::~OpExpression()
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline i64 OpExpression<E, S>::GetOpCode() const
        {
            return OpCode;
        }
        
        template <typename E, template <typename> class S>
        inline const vector<Expr<E, S>>&
        OpExpression<E, S>::GetChildren() const
        {
            return Children;
        }

        template <typename E, template <typename> class S>
        inline i32 OpExpression<E, S>::Compare(const ExpressionBase<E, S>* Other) const
        {
            if ((Other->template As<ConstExpression>() != nullptr) ||
                (Other->template As<VarExpression>() != nullptr) ||
                (Other->template As<BoundVarExpression>() != nullptr)) {
                return 1;
            } 
            auto OtherAsOp = Other->template As<ESMC::Exprs::OpExpression>();
            if (OtherAsOp == nullptr) {
                return -1;
            }
           
            if (OpCode < OtherAsOp->OpCode) {
                return -1;
            } else if (OpCode > OtherAsOp->OpCode) {
                return 1; 
            } else if (Children.size() < OtherAsOp->Children.size()) {
                return -1;
            } else if (Children.size() > OtherAsOp->Children.size()) {
                return 1;
            } else {
                for(u32 i = 0; i < Children.size(); ++i) {
                    auto Res = Children[i]->Compare(OtherAsOp->Children[i]);
                    if (Res != 0) {
                        return Res;
                    }
                }
                return 0;
            }
        }

        template <typename E, template <typename> class S>
        inline bool OpExpression<E, S>::FastEQ(const ExpressionBase<E, S>* Other) const
        {
            if (this->Hash() != Other->Hash()) {
                return false;
            }
            auto OtherAsOp = Other->template As<ESMC::Exprs::OpExpression>();
            if (OtherAsOp == nullptr) {
                return false;
            }

            if (OtherAsOp->OpCode != OpCode ||
                OtherAsOp->Children.size() != Children.size()) {
                return false;
            }

            const u32 NumChildren = Children.size();
            for (u32 i = 0; i < NumChildren; ++i) {
                if (Children[i] != OtherAsOp->Children[i]) {
                    return false;
                }
            }
            return true;
        }

        template <typename E, template <typename> class S>
        inline void OpExpression<E, S>::ComputeHash() const
        {
            this->HashCode = 0;
            boost::hash_combine(this->HashCode, OpCode);
            for (auto const& Child : Children) {
                boost::hash_combine(this->HashCode, Child->Hash());
            }
        }

        template <typename E, template <typename> class S>
        inline void OpExpression<E, S>::Accept(ExpressionVisitorBase<E, S>* Visitor) const
        {
            Visitor->VisitOpExpression(this);
        }

        template <typename E, template <typename> class S>
        inline QuantifiedExpressionBase<E, S>::QuantifiedExpressionBase
        (
         ExprMgr<E, S>* Manager,
         const vector<ExprTypeRef>& QVarTypes,
         const Expr<E, S>& QExpression,
         const E& ExtVal
         )
            : ExpressionBase<E, S>(Manager, ExtVal), QVarTypes(QVarTypes), QExpression(QExpression)
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline QuantifiedExpressionBase<E, S>::~QuantifiedExpressionBase()
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline const vector<ExprTypeRef>& 
        QuantifiedExpressionBase<E, S>::GetQVarTypes() const
        {
            return QVarTypes;
        }

        template <typename E, template <typename> class S>
        inline const Expr<E, S>& QuantifiedExpressionBase<E, S>::GetQExpression() const
        {
            return QExpression;
        }

        template <typename E, template <typename> class S>
        inline i32 
        QuantifiedExpressionBase<E, S>::CompareInternal(const QuantifiedExpressionBase<E, S>* Other) const
        {
            if (QVarTypes.size() < Other->QVarTypes.size()) {
                return -1;
            } else if (QVarTypes.size() > Other->QVarTypes.size()) {
                return 1;
            } else {
                for (u32 i = 0; i < QVarTypes.size(); ++i) {
                    auto Res = (QVarTypes[i] < Other->QVarTypes[i] ? -1 : 1);
                    if (Res != 0) {
                        return Res;
                    }
                }
                auto Res2 = QExpression->Compare(Other->QExpression);
                return Res2;
            }
        }

        template <typename E, template <typename> class S>
        inline void QuantifiedExpressionBase<E, S>::ComputeHashInternal() const
        {
            this->HashCode = 0;
            for (auto const& VarType : QVarTypes) {
                boost::hash_combine(this->HashCode, VarType->Hash());
            }
            
            boost::hash_combine(this->HashCode, QExpression->Hash());
        }

        template <typename E, template <typename> class S>
        inline bool QuantifiedExpressionBase<E, S>::
        FastEQInternal(const QuantifiedExpressionBase<E, S>* Other) 
            const
        {
            if (Other->QVarTypes.size() != QVarTypes.size()) {
                return false;
            }

            const u32 NumQVars = QVarTypes.size();

            for (u32 i = 0; i < NumQVars; ++i) {
                if (QVarTypes[i] != Other->QVarTypes[i]) {
                    return false;
                }
            }
            return (QExpression == Other->QExpression);
        }

    
        // EQuantifiedExpression implementation
        template <typename E, template <typename> class S>
        inline EQuantifiedExpression<E, S>::~EQuantifiedExpression()
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline i32 EQuantifiedExpression<E, S>::Compare(const ExpressionBase<E, S>* Other) const
        {
            if ((Other->template As<VarExpression>() != nullptr) ||
                (Other->template As<ConstExpression>() != nullptr) ||
                (Other->template As<BoundVarExpression>() != nullptr) ||
                (Other->template As<OpExpression>() != nullptr)) {
                return 1;
            }
            auto OtherAsExists = Other->template As<ESMC::Exprs::EQuantifiedExpression>();
            if (OtherAsExists == nullptr) {
                return -1;
            }

            return QuantifiedExpressionBase<E, S>::CompareInternal(OtherAsExists);
        }

        template <typename E, template <typename> class S>
        inline bool EQuantifiedExpression<E, S>::FastEQ(const ExpressionBase<E, S>* Other) const
        {
            if (this->Hash() != Other->Hash()) {
                return false;
            }
            auto OtherAsExists = Other->template As<ESMC::Exprs::EQuantifiedExpression>();
            if (OtherAsExists == nullptr) {
                return false;
            }

            return this->FastEQInternal(OtherAsExists);
        }

        template <typename E, template <typename> class S>
        inline void EQuantifiedExpression<E, S>::ComputeHash() const
        {
            this->ComputeHashInternal();
            boost::hash_combine(this->HashCode, (string)"exists");
        }

        template <typename E, template <typename> class S>
        inline void EQuantifiedExpression<E, S>::Accept(ExpressionVisitorBase<E, S>* Visitor) const
        {
            Visitor->VisitEQuantifiedExpression(this);
        }

        template <typename E, template <typename> class S>
        inline bool EQuantifiedExpression<E, S>::IsForAll() const
        {
            return false;
        }

        template <typename E, template <typename> class S>
        inline bool EQuantifiedExpression<E, S>::IsExists() const
        {
            return true;
        }


        // AQuantifiedExpression implementation
        template <typename E, template <typename> class S>
        inline AQuantifiedExpression<E, S>::~AQuantifiedExpression()
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline i32 AQuantifiedExpression<E, S>::Compare(const ExpressionBase<E, S>* Other) const
        {
            if ((Other->template As<VarExpression>() != nullptr) ||
                (Other->template As<ConstExpression>() != nullptr) ||
                (Other->template As<BoundVarExpression>() != nullptr) ||
                (Other->template As<OpExpression>() != nullptr) || 
                (Other->template As<EQuantifiedExpression>() != nullptr)) {
                return 1;
            }
            auto OtherAsForall = Other->template As<ESMC::Exprs::AQuantifiedExpression>();
            // In case we decide to add more expression types
            if (OtherAsForall != nullptr) {
                return -1;
            }

            return QuantifiedExpressionBase<E, S>::CompareInternal(OtherAsForall);
        }

        template <typename E, template <typename> class S>
        inline bool AQuantifiedExpression<E, S>::FastEQ(const ExpressionBase<E, S>* Other) const
        {
            if (this->Hash() != Other->Hash()) {
                return false;
            }
            auto OtherAsForAll = Other->template As<ESMC::Exprs::EQuantifiedExpression>();
            if (OtherAsForAll == nullptr) {
                return false;
            }

            return this->FastEQInternal(OtherAsForAll);
        }

        template <typename E, template <typename> class S>
        inline void AQuantifiedExpression<E, S>::ComputeHash() const
        {
            this->ComputeHashInternal();
            boost::hash_combine(this->HashCode, (string)"forall");
        }

        template <typename E, template <typename> class S>
        inline void AQuantifiedExpression<E, S>::Accept(ExpressionVisitorBase<E, S>* Visitor) const
        {
            Visitor->VisitAQuantifiedExpression(this);
        }

        template <typename E, template <typename> class S>
        inline bool AQuantifiedExpression<E, S>::IsForAll() const
        {
            return true;
        }

        template <typename E, template <typename> class S>
        inline bool AQuantifiedExpression<E, S>::IsExists() const
        {
            return false;
        }

        // ExprMgr implementation
        template <typename E, template <typename> class S>
        template <typename... ArgTypes>
        inline ExprMgr<E, S>::ExprMgr(ArgTypes&&... Args)
        {
            BoolType = TypeCache.Get<ExprBoolType>();
            IntType = TypeCache.Get<ExprIntType>();
            (void)BoolType->GetOrSetTypeID();
            (void)IntType->GetOrSetTypeID();
            Sem = new S<E>(this, BoolType, IntType, forward<ArgTypes>(Args)...);
        }

        template <typename E, template <typename> class S>
        inline ExprMgr<E, S>::~ExprMgr()
        {
            delete Sem;
        }

        template <typename E, template <typename> class S>
        template <typename T, typename... ArgTypes>
        inline typename ExprMgr<E, S>::TypeT
        ExprMgr<E, S>::MakeType(ArgTypes&&... Args)
        {
            auto Retval = TypeCache.Get<T>(forward<ArgTypes>(Args)...);
            Retval->GetOrSetTypeID();
            string TypeName = "";
            if (Retval->template Is<ExprEnumType>()) {
                TypeName = Retval->template SAs<ExprEnumType>()->GetName();
            } else if (Retval->template Is<ExprSymmetricType>()) {
                TypeName = Retval->template SAs<ExprSymmetricType>()->GetName();
            }

            if (TypeName != "") {
                if (NamedTypes.find(TypeName) != NamedTypes.end()) {
                    throw ExprTypeError((string)"A type named \"" + TypeName + "\" already " + 
                                        "exists in the system!");
                }
                NamedTypes[TypeName] = Retval;
            }
            return Retval;
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::TypeT
        ExprMgr<E, S>::GetNamedType(const string& Name) const
        {
            auto it = NamedTypes.find(Name);
            if (it == NamedTypes.end()) {
                return TypeT::NullPtr;
            } else {
                return it->second;
            }
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::TypeT
        ExprMgr<E, S>::InstantiateType(const TypeT& Type,
                                       const vector<ExpT>& Params)
        {
            auto TypeAsPType = Type->template As<ExprParametricType>();
            if (TypeAsPType == nullptr) {
                throw ExprTypeError((string)"Cannot instantiate non-parametric type " +
                                    Type->ToString());
            }
            auto const& ExpectedTypes = TypeAsPType->GetParameterTypes();
            const u32 NumExpected = ExpectedTypes.size();
            const u32 NumGot = Params.size();

            if (NumExpected != NumGot) {
                throw ExprTypeError((string)"Parametric type \"" + TypeAsPType->GetName() + 
                                    "\" expects " + to_string(NumExpected) + " parameters, " + 
                                    "but was attempted to be instantiated with " + 
                                    to_string(NumGot) + " parameters");
            }

            for (u32 i = 0; i < NumExpected; ++i) {
                if (!Params[i]->template Is<ConstExpression>()) {
                    throw ExprTypeError((string)"Parameter at position " + to_string(i) + 
                                        " is not a constant value in instantiation of type " + 
                                        TypeAsPType->GetName());
                }
                if (Params[i]->GetType() != ExpectedTypes[i]) {
                    throw ExprTypeError((string)"Parameter types don't match at position " + 
                                        to_string(i) + " in instantiation of type " + 
                                        TypeAsPType->GetName());
                }
            }
            
            string InstName = TypeAsPType->GetName();
            for (u32 i = 0; i < NumExpected; ++i) {
                InstName += ((string)"[" + 
                             Params[i]->template SAs<ConstExpression>()->GetConstValue() + 
                             "]");
            }
            auto BaseRecType = TypeAsPType->GetBaseType()->template SAs<ExprRecordType>();
            return (MakeType<ExprRecordType>(InstName, BaseRecType->GetMemberVec()));
        }

        template <typename E, template <typename> class S>
        void ExprMgr<E, S>::CheckMgr(const vector<ExpT>& Children) const
        {
            for (auto const& Child : Children) {
                CheckMgr(Child);
            }
        }

        template <typename E, template <typename> class S>
        void ExprMgr<E, S>::CheckMgr(const ExpT& Exp) const
        {
            if (Exp->GetMgr() != this) {
                throw ExprTypeError("ExprMgr: I don't own this expression!");
            }
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::Internalize(const ExpT& Exp)
        {
            const_cast<ExpressionBase<E, S>*>(&*Exp)->Mgr = this;
            if ((Exp->template As<ConstExpression>() != nullptr) ||
                (Exp->template As<VarExpression>() != nullptr) ||
                (Exp->template As<BoundVarExpression>() != nullptr)) {
                return ExpCache.Get(Exp);
            }
            auto ExpAsOp = Exp->template As<OpExpression>();
            if (ExpAsOp != nullptr) {
                auto const& Children = ExpAsOp->GetChildren();
                const u32 NumChildren = Children.size();
                vector<ExpT> IntChildren(NumChildren);
                for (u32 i = 0; i < NumChildren; ++i) {
                    IntChildren[i] = Internalize(Children[i]);
                }
                return ExpCache.template Get<OpExpression<E, S>>(this,
                                                                 ExpAsOp->GetOpCode(),
                                                                 IntChildren,
                                                                 Exp->ExtensionData);
            }
            auto ExpAsQuantified = Exp->template As<QuantifiedExpressionBase>();
            if (ExpAsQuantified != nullptr) {
                auto const& QVarTypes = ExpAsQuantified->GetQVarTypes();
                auto IntQExpr = Internalize(ExpAsQuantified->GetQExpression());
                if (ExpAsQuantified->IsForAll()) {
                    return ExpCache.template Get<AQuantifiedExpression<E, S>>(this,
                                                                              QVarTypes,
                                                                              IntQExpr,
                                                                              Exp->ExtensionData);
                } else {
                    return ExpCache.template Get<EQuantifiedExpression<E, S>>(this,
                                                                              QVarTypes,
                                                                              IntQExpr,
                                                                              Exp->ExtensionData);
                }
            } else {
                throw ExprTypeError("Strange type of expression encountered");
            }
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::MakeTrue(const E& ExtVal)
        {
            auto Retval = ExpCache.template Get<ConstExpression<E, S>>(this, "true", 
                                                                       BoolType);
            Sem->TypeCheck(Retval);
            return Retval;
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::MakeFalse(const E& ExtVal)
        {
            auto Retval = ExpCache.template Get<ConstExpression<E, S>>(this, "false", 
                                                                       BoolType);
            Sem->TypeCheck(Retval);
            return Retval;
        }

        template <typename E, template<typename> class S>
        inline typename ExprMgr<E, S>::ExpT 
        ExprMgr<E, S>::MakeVal(const string& ValString, const TypeT& ValType,
                               const E& ExtVal)
        {
            auto TrimmedValString = boost::algorithm::trim_copy(ValString);
            auto Retval =
                ExpCache.template Get<ConstExpression<E, S>>(this, 
                                                             TrimmedValString, 
                                                             ValType, ExtVal);
            Sem->TypeCheck(Retval);
            return Retval;
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::MakeVar(const string& VarName, const TypeT& VarType,
                               const E& ExtVal)
        {
            auto Retval = 
                ExpCache.template Get<VarExpression<E, S>>(this, VarName, VarType, ExtVal);
            Sem->TypeCheck(Retval);
            return Retval;
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::MakeBoundVar(const TypeT& VarType,
                                    i64 VarUID, const E& ExtVal)
        {
            auto Retval =
                ExpCache.template Get<BoundVarExpression<E, S>>(this, VarType, VarUID, ExtVal);
            Sem->TypeCheck(Retval);
            return Retval;
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::MakeExpr(const i64 OpCode,
                                const vector<ExpT>& Children,
                                const E& ExtVal)
        {
            CheckMgr(Children);
            ExpT NewExp = new OpExpression<E, S>(this, OpCode, Children, ExtVal);
            auto Retval = Sem->Canonicalize(NewExp);
            Retval = Internalize(Retval);
            Sem->TypeCheck(Retval);
            return Retval;
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::MakeExpr(const i64 OpCode, const ExpT& Child1,
                                const E& ExtVal)
        {
            vector<ExpT> Children(1);
            Children[0] = Child1;
            return MakeExpr(OpCode, Children, ExtVal);
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::MakeExpr(const i64 OpCode, const ExpT& Child1,
                                const ExpT& Child2,
                                const E& ExtVal)
        {
            vector<ExpT> Children(2);
            Children[0] = Child1;
            Children[1] = Child2;
            return MakeExpr(OpCode, Children, ExtVal);
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::MakeExpr(const i64 OpCode, const ExpT& Child1,
                                const ExpT& Child2, const ExpT& Child3,
                                const E& ExtVal)
        {
            vector<ExpT> Children(3);
            Children[0] = Child1;
            Children[1] = Child2;
            Children[2] = Child3;
            return MakeExpr(OpCode, Children, ExtVal);
        }

        template <typename E, template <typename> class S>
        template <template <typename, template <typename> class> class T>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::MakeQExpression(const vector<TypeT>& QVarTypes,
                                       const ExpT& QExpr,
                                       const E& ExtVal)
        {
            CheckMgr(QExpr);
            ExpT NewExp = new T<E, S>(this, QVarTypes, QExpr, ExtVal);
            auto Retval = Sem->Canonicalize(NewExp);
            Retval = Internalize(Retval);
            Sem->TypeCheck(Retval);
            return Retval;
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::MakeExists(const vector<TypeT>& QVarTypes,
                                  const ExpT& QExpr,
                                  const E& ExtVal)
        {
            return MakeQExpression<EQuantifiedExpression>(QVarTypes, QExpr, ExtVal);
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::MakeForAll(const vector<TypeT>& QVarTypes,
                                  const ExpT& QExpr,
                                  const E& ExtVal)
        {
            return MakeQExpression<AQuantifiedExpression>(QVarTypes, QExpr, ExtVal);
        }

        template <typename E, template <typename> class S>
        inline i64 ExprMgr<E, S>::MakeUninterpretedFunction(const string& Name,
                                                            const vector<TypeT>& Domain,
                                                            const TypeT& Range)
        {
            return Sem->RegisterUninterpretedFunction(Name, Domain, Range);
        }

        template <typename E, template <typename> class S>
        template <class T, typename... ArgTypes>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::ApplyTransform(const ExpT& Exp, 
                                      ArgTypes&&... Args)
        {
            CheckMgr(Exp);
            return T::Do(this, Exp, forward<ArgTypes>(Args)...);
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::SemT* 
        ExprMgr<E, S>::GetSemanticizer() const
        {
            return Sem;
        }

        template <typename E, template <typename> class S>
        template <typename... ArgTypes>
        inline typename ExprMgr<E, S>::LExpT
        ExprMgr<E, S>::LowerExpr(const ExpT& Exp, ArgTypes&&... Args)
        {
            if (Exp->GetMgr() != this) {
                throw ExprTypeError("ExprMgr: I don't own this expression");
            }
            
            return Sem->LowerExpr(Exp, forward<ArgTypes>(Args)...);
        }

        template <typename E, template <typename> class S>
        template <typename... ArgTypes>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::RaiseExpr(const LExpT& LExp, ArgTypes&&... Args)
        {
            auto Retval = Sem->RaiseExpr(this, LExp, forward<ArgTypes>(Args)...);
            return Retval;
        }
        
        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::Simplify(const ExpT& Exp)
        {
            auto Retval = Sem->Simplify(Exp);
            return Retval;
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::ElimQuantifiers(const ExpT& Exp)
        {
            auto Retval = Sem->ElimQuantifiers(this, Exp);
            return Retval;
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::UnrollQuantifiers(const ExpT& Exp)
        {
            auto Retval = Sem->UnrollQuantifiers(this, Exp);
            return Retval;
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpSetT
        ExprMgr<E, S>::Gather(const ExpT& Exp, 
                              const function<bool(const ExpressionBase<E, S>*)>& Pred) const
        {
            return Gatherer<E, S>::Do(Exp, Pred);
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::Substitute(const SubstMapT& Subst, const ExpT& Exp)
        {
            return ApplyTransform<Substitutor<E, S>>(Exp, Subst);
        }

        template <typename E, template <typename> class S>
        inline void ExprMgr<E, S>::GC()
        {
            ExpCache.GC();
        }

        template <typename E, template <typename> class S>
        inline ExprMgr<E, S>* ExprMgr<E, S>::Make()
        {
            return new ExprMgr<E, S>();
        }

        // pretty printer
        template <typename E, template <typename> class S>
        static inline 
        ostream& operator << (ostream& Out, 
                              const SmartPtr<ExpressionBase<E, S>>& Ptr)
        {
            Out << Ptr->ToString();
            return Out;
        }

        template <typename E, template <typename> class S>
        static inline 
        ostream& operator << (ostream& Out, 
                              const CSmartPtr<ExpressionBase<E, S>>& Ptr)
        {
            Out << Ptr->ToString();
            return Out;
        }

    } /* end namespace */
} /* end namespace */

#endif /* ESMC_EXPRESSIONS_HPP_ */

// 
// Expressions.hpp ends here
