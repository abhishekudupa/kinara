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
#include "../utils/UIDGenerator.hpp"


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
                                     const ExpressionBase<E, S>* Exp2) const;
        };

        class FastExpressionPtrEquals
        {
        public:
            template <typename E, template <typename> class S>
            inline bool operator () (const ExpressionBase<E, S>* Exp1,
                                     const ExpressionBase<E, S>* Exp2) const;
        };


        class ExpressionPtrHasher
        {
        public:
            template <typename E, template <typename> class S>
            inline u64 operator () (const ExpressionBase<E, S>* Exp) const;
        };


        class ExpressionPtrCompare
        {
        public:
            template <typename E, template <typename> class S>
            inline bool operator () (const ExpressionBase<E, S>* Exp1, 
                                     const ExpressionBase<E, S>* Exp2) const;
        };

        class ExpressionSPtrEquals
        {
        public:
            template<typename E, template <typename> class S>
            inline bool operator () (const CSmartPtr<ExpressionBase<E, S>>& Exp1,
                                     const CSmartPtr<ExpressionBase<E, S>>& Exp2) const
            {
                return Exp1->Equals(Exp2);
            }
        };

        class FastExpressionSPtrEquals
        {
        public:
            template<typename E, template <typename> class S>
            inline bool operator () (const CSmartPtr<ExpressionBase<E, S>>& Exp1,
                                     const CSmartPtr<ExpressionBase<E, S>>& Exp2) const
            {
                return Exp1->FastEQ(Exp2);
            }
        };

        class ExpressionSPtrCompare
        {
        public:
            template<typename E, template <typename> class S>
            inline bool operator () (const CSmartPtr<ExpressionBase<E, S>>& Exp1,
                                     const CSmartPtr<ExpressionBase<E, S>>& Exp2) const
            {
                return (Exp1->LT(Exp2) < 0);
            }
        };

        class ExpressionSPtrHasher
        {
        public:
            template<typename E, template <typename> class S>
            inline u64 operator () (const CSmartPtr<ExpressionBase<E, S>>& Exp) const
            {
                return (Exp->Hash());
            }
        };

        // An empty extension type
        struct EmptyExtType 
        {
            // Nothing here
        };

        template <typename E, template <typename> class S>
        class ExpressionBase : public RefCountable
        {
            friend class ExprMgr<E, S>;
        private:
            ExprMgr<E, S>* Mgr;
            mutable bool HashValid;
            mutable i64 ExpType;

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
            inline i64 GetType() const;
            inline void SetType(i64 Type) const;
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
            inline U<E, S>* As();

            template <template <typename, template <typename> class> class U> 
            inline const U<E, S>* As() const;

            template <template <typename, template <typename> class> class U> 
            inline U<E, S>* SAs();

            template <template <typename, template <typename> class> class U> 
            inline const U<E, S>* SAs() const;
        };


        template <typename E, template <typename> class S>
        class ConstExpression : public ExpressionBase<E, S>
        {
        private:
            const string ConstValue;
            i64 ConstType;

        public:
            inline ConstExpression(ExprMgr<E, S>* Mgr,
                                   const string& ConstValue,
                                   i64 ConstType, 
                                   const E& ExtData = E());
            inline virtual ~ConstExpression();

            inline const string& GetConstValue() const;
            inline i64 GetConstType() const;

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
            i64 VarType;

        public:
            inline VarExpression(ExprMgr<E, S>* Manager, 
                                 const string& VarName,
                                 i64 VarType, 
                                 const E& ExtData = E());
            inline virtual ~VarExpression();
            inline const string& GetVarName() const;
            inline i64 GetVarType() const;

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
            i64 VarType;
            u64 VarIdx;

        public:
            inline BoundVarExpression(ExprMgr<E, S>* Manager, 
                                      i64 VarType, i64 VarIdx,
                                      const E& ExtData = E());
            inline virtual ~BoundVarExpression();
            inline u64 GetVarIdx() const;
            inline i64 GetVarType() const;
        
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
            vector<i64> QVarTypes;
            Expr<E, S> QExpression;

        public:
            inline QuantifiedExpressionBase(ExprMgr<E, S>* Manager,
                                            const vector<i64>& QVarTypes,
                                            const Expr<E, S>& QExpression,
                                            const E& ExtData = E());
            inline virtual ~QuantifiedExpressionBase();
            inline const vector<i64>& GetQVarTypes() const;
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
            
            inline void TypeCheck(const ExpType& Exp) const {}

            inline ExpType Canonicalize(const ExpType& Exp) const { return Exp; }
            inline LExpType LowerExpr(const ExpType& Exp) { return Exp; }
            inline ExpType RaiseExpr(const LExpType& LExp) { return LExp; }
            inline ExpType Simplify(const ExpType& Exp) { return Exp; }
            inline ExpType ElimQuantifiers(const ExpType& Exp) { return Exp; }
            inline string ExprToString(const LExpType& Exp) { return "NoExp"; }
            inline string TypeToString(i64 Type) const { return "NoType"; }
            

            inline i64 RegisterUninterpretedFunction(const string& Name,
                                                     const vector<i64> DomTypes,
                                                     i64 RangeType)
            {
                return 0;
            }
        };

        template <typename E, template <typename> class S>
        class ExprMgr
        {
        public:
            typedef S<E> SemT;
            typedef typename SemT::LExpT LExpT;
            typedef Expr<E, S> ExpT;
            typedef ExprI<E, S> IExpT;

            typedef unordered_map<ExpT, ExpT, 
                                  ExpressionSPtrHasher,
                                  ExpressionSPtrEquals> SubstMap;

        private:
            typedef unordered_set<ExpT, ExpressionSPtrHasher, FastExpressionSPtrEquals> ExpSetType;

            SemT* Sem;
            ExpSetType ExpSet;
            u32 NextGC;
            const float GCGrowthFactor = 1.5f;
            
            inline ExpT GetCachedOrInsert(const ExpT& Exp);
            
            template <template <typename, template <typename> class> class T, 
                      typename... ArgTypes>
            inline ExpT GetCachedOrInsert(ArgTypes&&... Args);
            inline void CheckMgr(const vector<ExpT>& Children) const;
            inline void CheckMgr(const ExpT& Exp) const;
            template <template <typename, template<typename> class> class T>
            inline ExpT MakeQExpression(const vector<i64>& QVars,
                                        const ExpT& QExpr,
                                        const E& ExtVal);

            // Insert the expression and all subexpressions
            // into the set of expressions owned by this manager
            inline ExpT Internalize(const ExpT& Exp);
            inline void AutoGC();
            
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

            inline ExpT MakeExpr(i64 OpCode, const ExpT& Child1,
                                 const E& ExtVal = E());

            inline ExpT MakeExpr(i64 OpCode, const ExpT& Child1,
                                 const ExpT& Child2,
                                 const E& ExtVal = E());

            inline ExpT MakeExpr(i64 OpCode, const ExpT& Child1,
                                 const ExpT& Child2, const ExpT& Child3,
                                 const E& ExtVal = E());

            inline ExpT MakeExists(const vector<i64>& QVarTypes, 
                                   const ExpT& QExpr,
                                   const E& ExtVal = E());

            inline ExpT MakeForAll(const vector<i64>& QVarTypes,
                                   const ExpT& QExpr,
                                   const E& ExtVal = E());

            inline i64 MakeUninterpretedFunction(const string& Name, 
                                                 const vector<i64>& Range,
                                                 i64 Domain);

            template <template <typename, template <typename> class> class T, typename... ArgTypes>
            inline ExpT ApplyTransform(const ExpT& Exp, ArgTypes&&... Args);

            inline SemT* GetSemanticizer() const;
            inline LExpT LowerExpr(const ExpT& Exp);
            template <typename... ArgTypes>
            inline ExpT RaiseExpr(const LExpT& Exp, ArgTypes&&... Args);
            inline ExpT ElimQuantifiers(const ExpT& Exp);
            inline ExpT Simplify(const ExpT& Exp);
            inline ExpT Substitute(const SubstMap& Subst, const ExpT& Exp);
            inline unordered_set<ExpT> 
            Gather(const ExpT& Exp, 
                   const function<bool(const ExpressionBase<E, S>*)>& Pred) const;

            inline string TypeToString(i64 TypeID) const;
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
            typedef typename ExprMgr<E, S>::ExpT ExpT;
            typedef typename ExprMgr<E, S>::SubstMap SubstMap;
            SubstMap Subst;
            vector<typename ExprMgr<E, S>::ExpT> SubstStack;

        public:
            inline Substitutor(const SubstMap& Subst);
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

            inline static ExpT Do(const SubstMap& Subst, const ExpT& Exp);
        };

        template <typename E, template <typename> class S>
        class Gatherer : ExpressionVisitorBase<E, S>
        {
        private:
            typedef Expr<E, S> ExpT;
            function<bool(const ExpressionBase<E, S>*)> Pred;
            unordered_set<ExpT> GatheredExps;

        public:
            inline Gatherer(const function<bool(const ExpressionBase<E, S>*)>& Pred);
            inline virtual ~Gatherer();

            inline virtual void VisitVarExpression(const VarExpression<E, S>* Exp) override;
            inline virtual void VisitConstExpression(const ConstExpression<E, S>* Exp) override;
            inline virtual void VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp) override;
            inline virtual void VisitOpExpression(const OpExpression<E, S>* Exp) override;
            inline virtual void VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp) override;
            inline virtual void VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp) override;
            
            static inline unordered_set<ExpT> 
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
        inline Substitutor<E, S>::Substitutor(const SubstMap& Subst)
            : Subst(Subst)
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
            SubstStack.push_back(Exp);
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
            SubstStack.push_back(new OpExpression<E, S>(nullptr, Exp->GetOpCode(),
                                                        SubstChildren));
        }

        template <typename E, template <typename> class S>
        inline void 
        Substitutor<E, S>::VisitEQuantifiedExpression(const EQuantifiedExpression<E,S>* Exp)
        {
            Exp->GetQExpression().Accept(this);
            auto SubstQExpr = SubstStack.back();
            SubstStack.push_back(new EQuantifiedExpression<E, S>(nullptr, Exp->GetQVarTypes(),
                                                                 SubstQExpr));
        }

        template <typename E, template <typename> class S>
        inline void 
        Substitutor<E, S>::VisitAQuantifiedExpression(const AQuantifiedExpression<E,S>* Exp)
        {
            Exp->GetQExpression().Accept(this);
            auto SubstQExpr = SubstStack.back();
            SubstStack.push_back(new AQuantifiedExpression<E, S>(nullptr, Exp->GetQVarTypes(),
                                                                 SubstQExpr));
        }

        template <typename E, template <typename> class S>
        inline typename Substitutor<E, S>::ExpT
        Substitutor<E, S>::Do(const SubstMap& Subst, const ExpT& Exp)
        {
            Substitutor TheSubstitutor(Subst);
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
        inline unordered_set<typename Gatherer<E, S>::ExpT>
        Gatherer<E, S>::Do(const ExpT& Exp, const function<bool (const ExpressionBase<E, S> *)>& Pred)
        {
            Gatherer<E, S> TheGatherer(Pred);
            Exp->Accept(&TheGatherer);
            return TheGatherer.GatheredExps;
        }


        // Implementation of down casts

        template <typename E, template <typename> class S>
        template <template <typename, template <typename> class> class U>
        inline U<E, S>* ExpressionBase<E, S>::As()
        {
            return dynamic_cast<U<E, S>*>(this);
        }

        template <typename E, template <typename> class S> 
        template <template <typename, template <typename> class> class U>
        inline const U<E, S>* ExpressionBase<E, S>::As() const
        {
            return dynamic_cast<const U<E, S>*>(this);
        }

        template <typename E, template <typename> class S> 
        template <template <typename, template <typename> class> class U>
        inline U<E, S>* ExpressionBase<E, S>::SAs()
        {
            return static_cast<U<E, S>*>(this);
        }

        template <typename E, template <typename> class S> 
        template <template <typename, template <typename> class> class U>
        inline const U<E, S>* ExpressionBase<E, S>::SAs() const
        {
            return static_cast<const U<E, S>*>(this);
        }

        // hashers, equality testers and comparisons
        template <typename E, template <typename> class S>
        inline bool ExpressionPtrEquals::operator () (const ExpressionBase<E, S>* Exp1,
                                                      const ExpressionBase<E, S>* Exp2) const
        {
            return (Exp1->Equals(Exp2));
        }

        template <typename E, template <typename> class S>
        inline bool FastExpressionPtrEquals::operator () (const ExpressionBase<E, S>* Exp1,
                                                          const ExpressionBase<E, S>* Exp2) const
        {
            return (Exp1->FastEQ(Exp2));
        }
        
        template <typename E, template <typename> class S>
        inline u64 ExpressionPtrHasher::operator () (const ExpressionBase<E, S>* Exp) const
        {
            return Exp->Hash();
        }

        template <typename E, template <typename> class S>
        inline bool ExpressionPtrCompare::operator () (const ExpressionBase<E, S>* Exp1, 
                                                       const ExpressionBase<E, S>* Exp2) const
        {
            return (Exp1->LT(Exp2));
        }

        // ExpressionBase implementation
        template <typename E, template <typename> class S>
        inline ExpressionBase<E, S>::ExpressionBase(ExprMgr<E, S>* Manager,
                                                    const E& ExtVal)
            : Mgr(Manager), HashValid(false),
              ExpType(-1), ExtensionData(ExtVal),
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
        inline i64 ExpressionBase<E, S>::GetType() const
        {
            return ExpType;
        }

        template <typename E, template <typename> class S>
        inline void ExpressionBase<E, S>::SetType(i64 Type) const
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

        // ConstExpression implementation
        template <typename E, template <typename> class S>
        inline ConstExpression<E, S>::ConstExpression(ExprMgr<E, S>* Manager, 
                                                      const string& ConstValue,
                                                      i64 ConstType, 
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
        inline i64 ConstExpression<E, S>::GetConstType() const
        {
            return ConstType;
        }

        template <typename E, template <typename> class S>
        inline i32 ConstExpression<E, S>::Compare(const ExpressionBase<E, S>* Other) const
        {
            auto OtherAsConst = Other->template As<ConstExpression>();
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
            boost::hash_combine(this->HashCode, ConstValue);
            boost::hash_combine(this->HashCode, ConstType);
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
                                                  i64 VarType, 
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
        inline i64 VarExpression<E, S>::GetVarType() const
        {
            return VarType;
        }

        template <typename E, template <typename> class S>
        inline i32 VarExpression<E, S>::Compare(const ExpressionBase<E, S>* Other) const
        {
            auto OtherAsVar = Other->template As<VarExpression>();
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
            boost::hash_combine(this->HashCode, VarName);
            boost::hash_combine(this->HashCode, VarType);
        }

        template <typename E, template <typename> class S>
        inline void VarExpression<E, S>::Accept(ExpressionVisitorBase<E, S>* Visitor) const
        {
            Visitor->VisitVarExpression(this);
        }

    
        // BoundVarExpression implementation
        template <typename E, template <typename> class S>
        inline BoundVarExpression<E, S>::BoundVarExpression(ExprMgr<E, S>* Manager, 
                                                            i64 VarType, i64 VarIdx,
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
        inline i64 BoundVarExpression<E, S>::GetVarType() const
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
            auto OtherAsBound = Other->template As<BoundVarExpression>();
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
            boost::hash_combine(this->HashCode, VarType);
            boost::hash_combine(this->HashCode, VarIdx);
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
            auto OtherAsOp = Other->template As<OpExpression>();
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
            auto OtherAsOp = Other->template As<OpExpression>();
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
         const vector<i64>& QVarTypes,
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
        inline const vector<i64>& QuantifiedExpressionBase<E, S>::GetQVarTypes() const
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
            for (auto const& QVarType : QVarTypes) {
                boost::hash_combine(this->HashCode, QVarType);
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
            auto OtherAsExists = Other->template As<EQuantifiedExpression>();
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
            auto OtherAsExists = Other->template As<EQuantifiedExpression>();
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
            auto OtherAsForall = Other->template As<AQuantifiedExpression>();
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
            auto OtherAsForAll = Other->template As<EQuantifiedExpression>();
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
        // Initial GC param is 1024
        template <typename E, template <typename> class S>
        template <typename... ArgTypes>
        inline ExprMgr<E, S>::ExprMgr(ArgTypes&&... Args)
            : Sem(new SemT(forward<ArgTypes>(Args)...)), NextGC(1024)
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
            AutoGC();
            auto it = ExpSet.find(Exp);
            if (it == ExpSet.end()) {
                ExpSet.insert(Exp);
                return Exp;
            } else {
                return (*it);
            }
        }

        template <typename E, template <typename> class S>
        template <template <typename, template <typename> class> class T, 
                  typename... ArgTypes>
        inline typename ExprMgr<E, S>::ExpT 
        ExprMgr<E, S>::GetCachedOrInsert(ArgTypes&&... Args)
        {
            AutoGC();
            ExpT NewExp = new T<E, S>(this, forward<ArgTypes>(Args)...);
            return GetCachedOrInsert(NewExp);
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
                return GetCachedOrInsert<OpExpression>(ExpAsOp->GetOpCode(),
                                                       IntChildren,
                                                       Exp->ExtensionData);
            }
            auto ExpAsQuantified = Exp->template As<QuantifiedExpressionBase>();
            if (ExpAsQuantified != nullptr) {
                auto const& QVarTypes = ExpAsQuantified->GetQVarTypes();
                auto IntQExpr = Internalize(ExpAsQuantified->GetQExpression());
                if (ExpAsQuantified->IsForAll()) {
                    return GetCachedOrInsert<AQuantifiedExpression>(QVarTypes,
                                                                    IntQExpr,
                                                                    Exp->ExtensionData);
                } else {
                    return GetCachedOrInsert<EQuantifiedExpression>(QVarTypes,
                                                                    IntQExpr,
                                                                    Exp->ExtensionData);
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
            auto TrimmedValString = boost::algorithm::trim_copy(ValString);
            auto Retval =
                GetCachedOrInsert<ConstExpression>(TrimmedValString, 
                                                   ValType, ExtVal);
            Sem->TypeCheck(Retval);
            return Retval;
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::MakeVar(const string& VarName, i64 VarType,
                               const E& ExtVal)
        {
            auto Retval = 
                GetCachedOrInsert<VarExpression>(VarName, VarType, ExtVal);
            Sem->TypeCheck(Retval);
            return Retval;
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::MakeBoundVar(i64 VarType,
                                    i64 VarUID, const E& ExtVal)
        {
            auto Retval =
                GetCachedOrInsert<BoundVarExpression>(VarType, VarUID, ExtVal);
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
            Sem->TypeCheck(NewExp);
            auto Retval = Sem->Canonicalize(NewExp);
            return Internalize(Retval);
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
        ExprMgr<E, S>::MakeQExpression(const vector<i64>& QVarTypes,
                                       const ExpT& QExpr,
                                       const E& ExtVal)
        {
            CheckMgr(QExpr);
            ExpT NewExp = new T<E, S>(this, QVarTypes, QExpr, ExtVal);
            Sem->TypeCheck(NewExp);
            auto Retval = Sem->Canonicalize(NewExp);
            return Internalize(Retval);
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::MakeExists(const vector<i64>& QVarTypes,
                                  const ExpT& QExpr,
                                  const E& ExtVal)
        {
            return MakeQExpression<EQuantifiedExpression>(QVarTypes, QExpr, ExtVal);
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::MakeForAll(const vector<i64>& QVarTypes,
                                  const ExpT& QExpr,
                                  const E& ExtVal)
        {
            return MakeQExpression<AQuantifiedExpression>(QVarTypes, QExpr, ExtVal);
        }

        template <typename E, template <typename> class S>
        inline i64 ExprMgr<E, S>::MakeUninterpretedFunction(const string& Name,
                                                            const vector<i64>& Domain,
                                                            i64 Range)
        {
            return Sem->RegisterUninterpretedFunction(Name, Domain, Range);
        }

        template <typename E, template <typename> class S>
        template <template <typename, template <typename> class> class T, typename... ArgTypes>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::ApplyTransform(const ExpT& Exp, 
                                      ArgTypes&&... Args)
        {
            CheckMgr(Exp);
            return Internalize(T<E, S>::Do(this, Exp, forward<ArgTypes>(Args)...));
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::SemT* 
        ExprMgr<E, S>::GetSemanticizer() const
        {
            return Sem;
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::LExpT
        ExprMgr<E, S>::LowerExpr(const ExpT& Exp)
        {
            if (Exp->GetMgr() != this) {
                throw ExprTypeError("ExprMgr: I don't own this expression");
            }
            
            return Sem->LowerExpr(Exp);
        }

        template <typename E, template <typename> class S>
        template <typename... ArgTypes>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::RaiseExpr(const LExpT& LExp, ArgTypes&&... Args)
        {
            auto Retval = Sem->RaiseExpr(LExp, forward<ArgTypes>(Args)...);
            return Internalize(Retval);
        }
        
        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::Simplify(const ExpT& Exp)
        {
            auto Retval = Sem->Simplify(Exp);
            Internalize(Retval);
            return Retval;
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::ElimQuantifiers(const ExpT& Exp)
        {
            auto Retval = Sem->ElimQuantifiers(Exp);
            Internalize(Retval);
            return Retval;
        }

        template <typename E, template <typename> class S>
        unordered_set<typename ExprMgr<E, S>::ExpT> 
        ExprMgr<E, S>::Gather(const ExpT& Exp, 
                              const function<bool(const ExpressionBase<E, S>*)>& Pred) const
        {
            return Gatherer<E, S>::Do(Exp, Pred);
        }

        template <typename E, template <typename> class S>
        inline typename ExprMgr<E, S>::ExpT
        ExprMgr<E, S>::Substitute(const SubstMap& Subst, const ExpT& Exp)
        {
            return ApplyTransform<Substitutor>(Subst, Exp);
        }
        

        template <typename E, template <typename> class S>
        inline void ExprMgr<E, S>::AutoGC()
        {
            if (ExpSet.size() >= NextGC) {
                GC();
                NextGC = (u32)(GCGrowthFactor * (float)ExpSet.size());
            }
        }
        
        template <typename E, template <typename> class S>
        inline void ExprMgr<E, S>::GC()
        {
            // We remove all expressions with a refcount of 1
            vector<ExpT> ToDelete;
            do {
                for(auto const& Exp : ToDelete) {
                    ExpSet.erase(Exp);
                }
                ToDelete.clear();
                for(auto const& Exp : ExpSet) {
                    if (Exp->GetRefCnt_() == 1) {
                        ToDelete.push_back(Exp);
                    }
                }
            } while (ToDelete.size() > 0);
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
