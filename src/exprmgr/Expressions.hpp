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

#include "../common/FwdDecls.hpp"
#include "../containers/RefCountable.hpp"
#include "../containers/SmartPtr.hpp"

// This classes in this file are heavily templatized
// to allow for flexibility via arbitrary extension objects


namespace ESMC {
    namespace Exprs {
        
        // Comparators
        class ExpressionPtrEquals
        {
        public:
            template <typename ExtType>
            inline bool operator () (const ExpressionBase<ExtType>* Exp1, 
                                     const ExpressionBase<ExtType>* Exp2) const;
        };


        class ExpressionPtrHasher
        {
        public:
            template <typename ExtType>
            inline u64 operator () (const ExpressionBase<ExtType>* Exp) const;
        };


        class ExpressionPtrCompare
        {
        public:
            template <typename ExtType>
            inline bool operator () (const ExpressionBase<ExtType>* Exp1, 
                                     const ExpressionBase<ExtType>* Exp2) const;
        };


        // Type ExtType must be default constructible
        template <typename ExtType>
        class ExpressionBase : public RefCountable
        {
        public:
            typedef ExprMgr<ExtType, SemType> MgrType;
            typedef ExpressionVisitorBase<ExtType, SemType> VisitorType;
            typedef Expression<ExtType, SemType> ExprType;
            ExtType ExtensionData;

        private:
            MgrType* Manager;
            mutable bool HashValid;
            mutable ExtType ExtensionData;

        protected:
            mutable u64 HashCode;

        public:
            inline ExpressionBase(MgrType* Manager, const ExtType& ExtData = ExtType());
            virtual inline ~ExpressionBase();

            inline MgrType* GetMgr() const;
            inline u64 Hash() const;
            inline bool Equals(const ExpressionBase* Other) const;
            inline bool NEquals(const ExpressionBase* Other) const;
            inline bool LT(const ExpressionBase* Other) const;
            inline bool LE(const ExpressionBase* Other) const;
            inline bool GE(const ExpressionBase* Other) const;
            inline bool GT(const ExpressionBase* Other) const;
            inline string ToString() const;

            // Abstract methods
        protected:
            virtual void ComputeHash() const = 0;

        public:
            virtual i32 Compare(const ExpressionBase* Other) const = 0;
            virtual void Accept(VisitorType* Visitor) const = 0;
        
            // Downcasts
            template<typename U> inline U* As();
            template<typename U> inline const U* As() const;
            template<typename U> inline U* SAs();
            template<typename U> inline const U* SAs() const;
        };


        template <typename ExtType, typename SemType>
        class ConstExpression : public ExpressionBase<ExtType, SemType>
        {
        private:
            const string& ConstValue;
            i64 ConstType;

        public:
            inline ConstExpression(MgrType* Manager, const string& ConstValue,
                                   i64 ConstType, const ExtType& ExtData = ExtType());
            inline virtual ~ConstExpression();

            inline const string& GetConstValue() const;
            inline i64 GetConstType() const;

        protected:
            inline virtual void ComputeHash() const override;
        
        public:
            inline virtual i32 Compare(const ExpressionBase* Other) const override;
            inline virtual void Accept(VisitorType* Visitor) const = 0;
        };
    

        template <typename ExtType, typename SemType>
        class VarExpression : public ExpressionBase<ExtType, SemType>
        {
        private:
            string VarName;
            i64 VarType;

        public:
            inline VarExpression(MgrType* Manager, const string& VarName,
                                 i64 VarType, const ExtType& ExtData = ExtType());
            inline virtual ~VarExpression();
            inline const string& GetVarName() const;
            inline i64 GetVarType() const;

        protected:
            inline virtual void ComputeHash() const override;

        public:
            inline virtual i32 Compare(const ExpressionBase* Other) const override;
            inline virtual void Accept(VisitorType* Visitor) const override;
        };


        template<typename ExtType, typename SemType>
        class BoundVarExpression : public VarExpression<ExtType, SemType>
        {
        private:
            u64 VarUID;
            static UIDGenerator UIDGen;

        public:
            inline BoundVarExpression(MgrType* Manager, const string& VarName,
                                      i64 VarType, i64 VarUID = -1,
                                      const ExtType& ExtData = ExtType());
            inline virtual ~BoundVarExpression();
            inline u64 GetVarUID() const;
        
        protected:
            inline virtual void ComputeHash() const override;
        
        public:
            inline virtual i32 Compare(const ExpressionBase* Other) const override;
            inline virtual void Accept(VisitorType* Visitor) const override;
        };


        template<typename ExtType, typename SemType>
        class OpExpression : public ExpressionBase<ExtType, SemType>
        {
        private:
            i64 OpCode;
            vector<ExprType> Children;

        public:
            inline OpExpression(MgrType* Manager, i64 OpCode,
                                const vector<ExprType>& Children,
                                const ExtType& ExtData = ExtType());
            inline virtual ~OpExpression();
            inline i64 GetOpCode() const;
            inline const vector<ExprType>& GetChildren() const;

        protected:
            inline virtual void ComputeHash() const override;

        public:
            inline virtual i32 Compare(const ExpressionBase* Other) const override;
            inline virtual void Accept(VisitorType* Visitor) const override;
        };


        template<typename ExtType, typename SemType>
        class QuantifiedExpressionBase : public ExpressionBase<ExtType, SemType>
        {
        private:
            vector<ExprType> QVarList;
            Expression QExpression;

        public:
            inline QuantifiedExpressionBase(MgrType* Manager,
                                            const vector<ExprType>& QVarList,
                                            const ExprType& QExpression,
                                            const ExtType& ExtData = ExtType());
            inline virtual ~QuantifiedExpressionBase();
            inline const vector<ExprType>& GetQVarList() const;
            inline const ExprType& GetQExpression() const;

        protected:
            inline i32 CompareInternal(const QuantifiedExpressionBase* Other) const;
            inline void ComputeHashInternal() const;

        public:
            virtual bool IsForAll() const = 0;
            virtual bool IsExists() const = 0;
        };


        template<typename ExtType, typename SemType>
        class EQuantifiedExpression : public QuantifiedExpressionBase<ExtType, SemType>
        {
        public:
            using QuantifiedExpressionBase::QuantifiedExpressionBase;
            inline virtual ~EQuantifiedExpression();

        protected:
            inline virtual void ComputeHash() const override;
        
        public:
            inline virtual i32 Compare(const ExpressionBase* Other) const override;
            inline virtual void Accept(VisitorType* Visitor) const override;
            inline virtual bool IsForAll() const override;
            inline virtual bool IsExists() const override;
        };


        template<typename ExtType, typename SemType>
        class AQuantifiedExpression : public QuantifiedExpressionBase<ExtType, SemType>
        {
        public:
            using QuantifiedExpressionBase::QuantifiedExpressionBase;
            inline virtual ~AQuantifiedExpression();

        protected:
            inline virtual void ComputeHash() const override;
        
        public:
            inline virtual i32 Compare(const ExpressionBase* Other) const override;
            inline virtual void Accept(VisitorType* Visitor) const override;
            inline virtual bool IsForAll() const override;
            inline virtual bool IsExists() const override;
        };

        template <typename T> inline T* ExpressionBase::As()
        {
            return dynamic_cast<T*>(this);
        }

        template <typename T> inline const T* ExpressionBase::As() const
        {
            return dynamic_cast<const T*>(this);
        }

        template <typename T> inline T* ExpressionBase::SAs()
        {
            return static_cast<T*>(this);
        }

        template <typename T> inline const T* ExpressionBase::SAs() const
        {
            return static_cast<const T*>(this);
        }

        // hashers, equality testers and comparisons
        template <typename ExtType>
        inline bool ExpressionPtrEquals::operator () (const ExpressionBase<ExtType>* Exp1,
                                               const ExpressionBase<ExtType>* Exp2) const
        {
            return (Exp1->Equals(Exp2));
        }
        
        template <typename ExtType>
        inline u64 ExpressionPtrHasher::operator () (const ExpressionBase<ExtType>* Exp) const
        {
            return Exp->Hash();
        }

        template <typename ExtType>
        inline bool ExpressionPtrCompare::operator () (const ExpressionBase<ExtType>* Exp1, 
                                                       const ExpressionBase<ExtType>* Exp2) const
        {
            return (Exp1->LT(Exp2));
        }

        // ExpressionBase implementation
        template <typename ExtType>
        inline ExpressionBase<ExtType>::ExpressionBase(ExpressionBase<ExtType>::MgrType* Manager,
                                                       const ExtType& ExtVal)
            : ExtensionData(ExtVal), Manager(Manager), HashValid(false), HashCode(0)
        {
            // Nothing here
        }

        template <typename ExtType>
        inline ExpressionBase<ExtType>::~ExpressionBase()
        {
            // Nothing here
        }

        template <typename ExtType>
        inline ExpressionBase<ExtType>::MgrType* 
        ExpressionBase<ExtType>::GetMgr() const 
        {
            return Manager;
        }

        template <typename ExtType>
        inline u64 ExpressionBase<ExtType>::Hash() const
        {
            if (!HashValid) {
                ComputeHash();
                HashValid = true;
            }
            return HashCode;
        }

        template <typename ExtType>
        inline bool ExpressionBase<ExtType>::Equals(const ExpressionBase<ExtType>* Other) const
        {
            if (Hash() != Other->Hash()) {
                return false;
            }
            return (Compare(Other) == 0);
        }

        template <typename ExtType>
        inline bool ExpressionBase<ExtType>::NEquals(const ExpressionBase<ExtType>* Other) const
        {
            if (Hash() != Other->Hash()) {
                return true;
            }
            return (Compare(Other) != 0);
        }

        template <typename ExtType>
        inline bool ExpressionBase<ExtType>::LT(const ExpressionBase<ExtType>* Other) const
        {
            return (Compare(Other) < 0);
        }

        template <typename ExtType>
        inline bool ExpressionBase<ExtType>::LE(const ExpressionBase<ExtType>* Other) const
        {
            return (Compare(Other) <= 0);
        }

        template <typename ExtType>
        inline bool ExpressionBase<ExtType>::GT(const ExpressionBase<ExtType>* Other) const
        {
            return (Compare(Other) > 0);
        }

        template <typename ExtType>
        inline bool ExpressionBase<ExtType>::GE(const ExpressionBase<ExtType>* Other) const
        {
            return (Compare(Other) >= 0);
        }

        template <typename ExtType>
        inline string ExpressionBase<ExtType>::ToString() const
        {
            auto Sem = Manager->GetSemanticizer();
            return (Sem->ExprToString(this));
        }

        // ConstExpression implementation
        template <typename ExtType>
        inline ConstExpression<ExtType>::ConstExpression(ExpressionBase<ExtType>::MgrType* Manager, 
                                                         const string& ConstValue,
                                                         i64 ConstType, const ExtType& ExtVal)
            : ExpressionBase<ExtType>(Manager, ExtVal), 
              ConstValue(ConstValue), 
              ConstType(ConstType)
        {
            // Nothing here
        }

        template <typename ExtType>
        inline ConstExpression<ExtType>::~ConstExpression()
        {
            // Nothing here
        }

        template <typename ExtType>
        inline const string& ConstExpression<ExtType>::GetConstValue() const
        {
            return ConstValue;
        }

        template <typename ExtType>
        inline i64 ConstExpression<ExtType>::GetConstType() const
        {
            return ConstType;
        }

        template <typename ExtType>
        inline i32 ConstExpression<ExtType>::Compare(const ExpressionBase<ExtType>* Other) const
        {
            auto OtherAsConst = Other->As<ConstExpression<ExtType>>();
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

        template <typename ExtType>
        inline void ConstExpression<ExtType>::ComputeHash() const
        {
            boost::hash_combine(HashCode, ConstValue);
            boost::hash_combine(HashCode, ConstType);
        }

        template <typename ExtType>
        inline void ConstExpression<ExtType>::Accept(ExpressionBase<ExtType>::VisitorType* Visitor) const 
        {
            Visitor->VisitConstExpression(this);
        }

        // VarExpression implementation
        template <typename ExtType>
        inline VarExpression<ExtType>::VarExpression(ExpressionBase<ExtType>::MgrType* Manager, 
                                                     const string& VarName,
                                                     i64 VarType, const ExtType& ExtVal)
            : ExpressionBase<ExtType>(Manager, ExtVal),
              VarName(VarName), VarType(VarType)
        {
            // Nothing here
        }

        template <typename ExtType>
        inline VarExpression<ExtType>::~VarExpression()
        {
            // Nothing here
        }

        template <typename ExtType>
        inline const string& VarExpression<ExtType>::GetVarName() const 
        {
            return VarName;
        }

        template <typename ExtType>
        inline i64 VarExpression<ExtType>::GetVarType() const
        {
            return VarType;
        }

        template <typename ExtType>
        inline i32 VarExpression<ExtType>::Compare(const ExpressionBase<ExtType>* Other) const
        {
            auto OtherAsVar = Other->As<VarExpression<ExtType>>();
            auto OtherAsConst = Other->As<ConstExpression<ExtType>>();
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

        template <typename ExtType>
        inline void VarExpression<ExtType>::ComputeHash() const
        {
            boost::hash_combine(HashCode, VarName);
            boost::hash_combine(HashCode, VarType);
        }

        template <typename ExtType>
        inline void VarExpression<ExtType>::Accept(ExpressionBase<ExtType>::VisitorType* Visitor) const
        {
            Visitor->VisitVarExpression(this);
        }

    
        // BoundVarExpression implementation
        template <typename ExtType>
        inline BoundVarExpression<ExtType>::BoundVarExpression(ExpressionBase<ExtType>::MgrType* Manager, 
                                                               const string& VarName,
                                                               i64 VarType, i64 VarUID,
                                                               const ExtType& ExtVal)
            : VarExpression(Manager, VarName, VarType, ExtVal),
              VarUID(VarUID == -1 ? UIDGen.GetUID() : VarUID)
        {
            // Nothing here
        }

        template <typename ExtType>
        inline BoundVarExpression<ExtType>::~BoundVarExpression()
        {
            // Nothing here
        }

        template <typename ExtType>
        inline u64 BoundVarExpression<ExtType>::GetVarUID() const
        {
            return VarUID;
        }

        template <typename ExtType>
        inline i32 BoundVarExpression<ExtType>::Compare(const ExpressionBase<ExtType>* Other) const
        {
            auto OtherAsConst = Other->As<ConstExpression<ExtType>>();
            // ConstExpression < BoundVarExpression
            if (OtherAsConst != nullptr) {
                return 1;
            }
            auto OtherAsVar = Other->As<VarExpression<ExtType>>();
            if (OtherAsVar != nullptr) {
                auto OtherAsBound = Other->As<BoundVarExpression<ExtType>>();
                // VarExpression < BoundVarExpression
                if (OtherAsBound == nullptr) {
                    return 1;
                } else {
                    auto Cmp = VarExpression<ExtType>::Compare(Other);
                    if (Cmp != 0) {
                        return Cmp;
                    } else {
                        if (VarUID < OtherAsBound->VarUID) {
                            return -1;
                        } else if (VarUID > OtherAsBound->VarUID) {
                            return 1;
                        } else {
                            return 0;
                        }
                    }
                }
            } else {
                // All other exp types > BoundVarExpression
                return -1;
            }
        }

        template <typename ExtType>
        inline void BoundVarExpression<ExtType>::ComputeHash() const
        {
            VarExpression<ExtType>::ComputeHash();
            boost::hash_combine(HashCode, VarUID);
        }

        template <typename ExtType>
        inline void BoundVarExpression<ExtType>::Accept(ExpressionBase<ExtType>::VisitorType* Visitor) const
        {
            Visitor->VisitBoundVarExpression(this);
        }

    
        // OpExpression implementation
        template <typename ExtType>
        inline OpExpression<ExtType>::OpExpression(ExpressionBase<ExtType>::MgrType* Manager, i64 OpCode,
                                                   const vector<ExpressionBase<ExtType>::ExprType> Children,
                                                   const ExtType& ExtVal)
            : ExpressionBase(Manager, ExtVal), OpCode(OpCode), Children(Children)
        {
            // Nothing here
        }

        template <typename ExtType>
        inline OpExpression<ExtType>::~OpExpression()
        {
            // Nothing here
        }

        template <typename ExtType>
        inline i64 OpExpression<ExtType>::GetOpCode() const
        {
            return OpCode;
        }

        template <typename ExtType>
        inline const vector<ExpressionBase<ExtType>::ExprType>& OpExpression<ExtType>::GetChildren() const
        {
            return Children;
        }

       template <typename ExtType>
       inline i32 OpExpression<ExtType>::Compare(const ExpressionBase<ExtType>* Other) const
       {
           if ((Other->As<ConstExpression<ExtType>>() != nullptr) ||
               (Other->As<VarExpression<ExtType>>() != nullptr) ||
               (Other->As<BoundVarExpression<ExtType>>() != nullptr)) {
               return 1;
           } 
           auto OtherAsOp = Other->As<OpExpression<ExtType>>();
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

        template <typename ExtType>
        inline void OpExpression<ExtType>::ComputeHash() const
        {
            boost::hash_combine(HashCode, OpCode);
            for (auto const& Child : Children) {
                boost::hash_combine(HashCode, Child->Hash());
            }
        }

        template <typename ExtType>
        inline void OpExpression<ExtType>::Accept(ExpressionBase<ExtType>::VisitorType *Visitor) const
        {
            Visitor->VisitOpExpression(this);
        }

        template <typename ExtType>
        inline QuantifiedExpressionBase<ExtType>::QuantifiedExpressionBase(ExpressionBase<ExtType>::MgrType* 
                                                                           Manager,
                                                                           const 
                                                                           vector<ExpressionBase<ExtType>::
                                                                           ExprType>& 
                                                                           QVarList,
                                                                           const ExpressionBase<ExtType>::ExprType& 
                                                                           QExpression,
                                                                           const ExtType& ExtVal)
        : ExpressionBase(Manager, ExtVal), QVarList(QVarList), QExpression(QExpression)
        {
            // Nothing here
        }

        template <typename ExtType>
        inline QuantifiedExpressionBase<ExtType>::~QuantifiedExpressionBase()
        {
            // Nothing here
        }

        template <typename ExtType>
        inline const vector<ExpressionBase<ExtType>::ExprType>& QuantifiedExpressionBase::GetQVarList() const
        {
            return QVarList;
        }

        template <typename ExtType>
        inline const ExpressionBase<ExtType>::ExprType& QuantifiedExpressionBase::GetQExpression() const
        {
            return QExpression;
        }

        template <typename ExtType>
        inline i32 
        QuantifiedExpressionBase<ExtType>::CompareInternal(const QuantifiedExpressionBase<ExtType>* Other) const
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

        template <typename ExtType>
        inline void QuantifiedExpressionBase<ExtType>::ComputeHashInternal() const
        {
            for (auto const& QVar : QVarList) {
                boost::hash_combine(HashCode, QVar->Hash());
            }
        
            boost::hash_combine(HashCode, QExpression->Hash());
        }

    
        // EQuantifiedExpression implementation
        template <typename ExtType>
        inline EQuantifiedExpression<ExtType>::~EQuantifiedExpression()
        {
            // Nothing here
        }

        template <typename ExtType>
        inline i32 EQuantifiedExpression<ExtType>::Compare(const ExpressionBase<ExtType> *Other) const
        {
            if ((Other->As<VarExpression<ExtType>>() != nullptr) ||
                (Other->As<ConstExpression<ExtType>>() != nullptr) ||
                (Other->As<BoundVarExpression<ExtType>>() != nullptr) ||
                (Other->As<OpExpression<ExtType>>() != nullptr)) {
                return 1;
            }
            auto OtherAsExists = Other->As<EQuantifiedExpression<ExtType>>();
            if (OtherAsExists == nullptr) {
                return -1;
            }

            return QuantifiedExpressionBase<ExtType>::Compare(OtherAsExists);
        }

        template <typename ExtType>
        inline void EQuantifiedExpression<ExtType>::ComputeHash() const
        {
            ComputeHashInternal();
            boost::hash_combine(HashCode, (string)"exists");
        }

        template <typename ExtType>
        inline void EQuantifiedExpression<ExtType>::Accept(ExpressionBase<ExtType>::VisitorType* Visitor) const
        {
            Visitor->VisitEQuantifiedExpression(this);
        }

        template <typename ExtType>
        inline bool EQuantifiedExpression<ExtType>::IsForAll() const
        {
            return false;
        }

        template <typename ExtType>
        inline bool EQuantifiedExpression<ExtType>::IsExists() const
        {
            return true;
        }


        // AQuantifiedExpression implementation
        template <typename ExtType>
        inline AQuantifiedExpression<ExtType>::~AQuantifiedExpression()
        {
            // Nothing here
        }

        template <typename ExtType>
        inline i32 AQuantifiedExpression<ExtType>::Compare(const ExpressionBase<ExtType> *Other) const
        {
            if ((Other->As<VarExpression<ExtType>>() != nullptr) ||
                (Other->As<ConstExpression<ExtType>>() != nullptr) ||
                (Other->As<BoundVarExpression<ExtType>>() != nullptr) ||
                (Other->As<OpExpression<ExtType>>() != nullptr) || 
                (Other->As<EQuantifiedExpression<ExtType>>() != nullptr)) {
                return 1;
            }
            auto OtherAsForall = Other->As<AQuantifiedExpression<ExtType>>();
            // In case we decide to add more expression types
            if (OtherAsForall != nullptr) {
                return -1;
            }

            return QuantifiedExpressionBase<ExtType>::Compare(OtherAsForall);
        }

        template <typename ExtType>
        inline void AQuantifiedExpression<ExtType>::ComputeHash() const
        {
            ComputeHashInternal();
            boost::hash_combine(HashCode, (string)"forall");
        }

        template <typename ExtType>
        inline void AQuantifiedExpression<ExtType>::Accept(ExpressionBase<ExtType>::VisitorType* Visitor) const
        {
            Visitor->VisitAQuantifiedExpression(this);
        }

        template <typename ExtType>
        inline bool AQuantifiedExpression<ExtType>::IsForAll() const
        {
            return true;
        }

        template <typename ExtType>
        inline bool AQuantifiedExpression<ExtType>::IsExists() const
        {
            return false;
        }

    } /* end namespace */
} /* end namespace */

#endif /* ESMC_EXPRESSIONS_HPP_ */

// 
// Expressions.hpp ends here
