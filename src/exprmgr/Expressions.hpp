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

#include "Visitors.hpp"

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

        // An empty extension type
        struct EmptyExtType 
        {
            // Nothing here
        };

        template <typename E, template <typename> class S>
        class ExpressionBase : public RefCountable
        {
        private:
            ExprManager<E, S>* Mgr;
            mutable bool HashValid;
            mutable E ExtensionData;
            
        protected:
            mutable u64 HashCode;

        public:
            inline ExpressionBase(ExprManager<E, S>* Manager, 
                                  const E& ExtData = E());
            virtual inline ~ExpressionBase();

            inline ExprManager<E, S>* GetMgr() const;
            inline u64 Hash() const;
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
            template<template <typename, template <typename> class> class U> 

            inline U<E, S>* As();
            template<template <typename, template <typename> class> class U> 
            inline const U<E, S>* As() const;

            template<template <typename, template <typename> class> class U> 
            inline U<E, S>* SAs();

            template<template <typename, template <typename> class> class U> 
            inline const U<E, S>* SAs() const;
        };


        template <typename E, template <typename> class S>
        class ConstExpression : public ExpressionBase<E, S>
        {
        private:
            const string& ConstValue;
            i64 ConstType;

        public:
            inline ConstExpression(ExprManager<E, S>* Mgr,
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
            inline VarExpression(ExprManager<E, S>* Manager, 
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
            inline BoundVarExpression(ExprManager<E, S>* Manager, 
                                      i64 VarType, i64 VarIdx,
                                      const E& ExtData = E());
            inline virtual ~BoundVarExpression();
            inline u64 GetVarIdx() const;
        
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
            inline OpExpression(ExprManager<E, S>* Manager, 
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
            vector<Expr<E, S>> QVarList;
            Expr<E, S> QExpression;

        public:
            inline QuantifiedExpressionBase(ExprManager<E, S>* Manager,
                                            const vector<Expr<E, S>>& QVarList,
                                            const Expr<E, S>& QExpression,
                                            const E& ExtData = E());
            inline virtual ~QuantifiedExpressionBase();
            inline const vector<Expr<E, S>>& GetQVarList() const;
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
        inline ExpressionBase<E, S>::ExpressionBase(ExprManager<E, S>* Manager,
                                                    const E& ExtVal)
            : ExtensionData(ExtVal), Mgr(Manager), HashValid(false), HashCode(0)
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline ExpressionBase<E, S>::~ExpressionBase()
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline ExprManager<E, S>*
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
        inline ConstExpression<E, S>::ConstExpression(ExprManager<E, S>* Manager, 
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
        inline VarExpression<E, S>::VarExpression(ExprManager<E, S>* Manager, 
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
        inline BoundVarExpression<E, S>::BoundVarExpression(ExprManager<E, S>* Manager, 
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
        inline OpExpression<E, S>::OpExpression(ExprManager<E, S>* Manager, 
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
            if (this->HashCode != Other->HashCode()) {
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
                if (Children[i] != Other->Children[i]) {
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
         ExprManager<E, S>* Manager,
         const vector<Expr<E, S>>& QVarList,
         const Expr<E, S>& QExpression,
         const E& ExtVal
         )
            : ExpressionBase<E, S>(Manager, ExtVal), QVarList(QVarList), QExpression(QExpression)
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline QuantifiedExpressionBase<E, S>::~QuantifiedExpressionBase()
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline const vector<Expr<E, S>>& QuantifiedExpressionBase<E, S>::GetQVarList() const
        {
            return QVarList;
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
            if (QVarList.size() < Other->QVarList.size()) {
                return -1;
            } else if (QVarList.size() > Other->QVarList.size()) {
                return 1;
            } else {
                for (u32 i = 0; i < QVarList.size(); ++i) {
                    auto Res = QVarList[i]->Compare(Other->QVarList[i]);
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
            for (auto const& QVar : QVarList) {
                boost::hash_combine(this->HashCode, QVar->Hash());
            }
        
            boost::hash_combine(this->HashCode, QExpression->Hash());
        }

        template <typename E, template <typename> class S>
        inline bool QuantifiedExpressionBase<E, S>::FastEQInternal(const QuantifiedExpressionBase<E, S>* Other) 
            const
        {
            if (Other->QVarList.size() != QVarList.size()) {
                return false;
            }

            const u32 NumQVars = QVarList.size();

            for (u32 i = 0; i < NumQVars; ++i) {
                if (QVarList[i] != Other->QVarList[i]) {
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

            return QuantifiedExpressionBase<E, S>::Compare(OtherAsExists);
        }

        template <typename E, template <typename> class S>
        inline bool EQuantifiedExpression<E, S>::FastEQ(const ExpressionBase<E, S>* Other) const
        {
            if (this->HashCode != Other->HashCode) {
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

            return QuantifiedExpressionBase<E, S>::Compare(OtherAsForall);
        }

        template <typename E, template <typename> class S>
        inline bool AQuantifiedExpression<E, S>::FastEQ(const ExpressionBase<E, S>* Other) const
        {
            if (this->HashCode != Other->HashCode) {
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

    } /* end namespace */
} /* end namespace */

#endif /* ESMC_EXPRESSIONS_HPP_ */

// 
// Expressions.hpp ends here
