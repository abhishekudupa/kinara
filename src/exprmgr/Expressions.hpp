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

    } /* end namespace */
} /* end namespace */

#endif /* ESMC_EXPRESSIONS_HPP_ */

// 
// Expressions.hpp ends here
