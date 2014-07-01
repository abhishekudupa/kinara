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

namespace ESMC {

    class ExpressionPtrEquals
    {
    public:
        template <typename T>
        inline bool operator () (const ExpressionBase<T>* Exp1, 
                                 const ExpressionBase<T>* Exp2) const;
    };

    class ExpressionPtrHasher
    {
    public:
        template <typename T>
        inline u64 operator () (const ExpressionBase<T>* Exp) const;
    };

    class ExpressionPtrCompare
    {
    public:
        template <typename T>
        inline bool operator () (const ExpressionBase<T>* Exp1, 
                                 const ExpressionBase<T>* Exp2) const;
    };

    // Type ExtType must be default constructible
    template <typename ExtType, typename LExpType>
    class ExpressionBase : public RefCountable
    {
    private:
        ExprMgr<LExpType, ExtType>* Manager;
        mutable bool HashValid;
        ExtType ExtensionData;

    protected:
        mutable u64 HashCode;

    public:
        ExpressionBase(ExprMgr<LExpType, ExtType>* Manager);
        virtual ~ExpressionBase();

        ExprMgr* GetMgr() const;
        u64 Hash() const;
        bool Equals(const ExpressionBase* Other) const;
        bool NEquals(const ExpressionBase* Other) const;
        bool LT(const ExpressionBase* Other) const;
        bool LE(const ExpressionBase* Other) const;
        bool GE(const ExpressionBase* Other) const;
        bool GT(const ExpressionBase* Other) const;

        const T& GetExtensionData() const;
        
        string ToString() const;

        // Abstract methods
    protected:
        virtual void ComputeHash() const = 0;

    public:
        virtual i32 Compare(const ExpressionBase* Other) const = 0;
        virtual void Accept(ExpressionVisitorBase* Visitor) const = 0;
        
        // Downcasts
        template<typename U> inline U* As();
        template<typename U> inline const U* As() const;
        template<typename U> inline U* SAs();
        template<typename U> inline const U* SAs() const;
    };

    template <typename T>
    class ConstExpression : public ExpressionBase<T>
    {
    private:
        const string& ConstValue;
        i64 ConstType;

    public:
        ConstExpression(ExprMgr<T>* Manager, const string& ConstValue,
                        i64 ConstType);
        virtual ~ConstExpression();

        const string& GetConstValue() const;
        i64 GetConstType() const;

    protected:
        virtual void ComputeHash() const override;
        
    public:
        virtual i32 Compare(const ExpressionBase* Other) const override;
        virtual void Accept(ExpressionVisitorBase* Visitor) const = 0;
    };

    class VarExpression : public ExpressionBase
    {
    private:
        string VarName;
        i64 VarType;

    public:
        VarExpression(ExprMgr* Manager, const string& VarName,
                      i64 VarType);
        virtual ~VarExpression();
        const string& GetVarName() const;
        i64 GetVarType() const;

    protected:
        virtual void ComputeHash() const override;

    public:
        virtual i32 Compare(const ExpressionBase* Other) const override;
        virtual void Accept(ExpressionVisitorBase* Visitor) const override;
    };

    class BoundVarExpression : public VarExpression
    {
    private:
        u64 VarUID;
        static UIDGenerator UIDGen;

    public:
        BoundVarExpression(ExprMgr* Manager, const string& VarName,
                           i64 VarType, i64 VarUID = -1);
        virtual ~BoundVarExpression();
        u64 GetVarUID() const;
        
    protected:
        virtual void ComputeHash() const override;
        
    public:
        virtual i32 Compare(const ExpressionBase* Other) const override;
        virtual void Accept(ExpressionVisitorBase* Visitor) const override;
    };

    class OpExpression : public ExpressionBase
    {
    private:
        i64 OpCode;
        vector<Expression> Children;

    public:
        OpExpression(ExprMgr* Manager, i64 OpCode,
                     const vector<Expression> Children);
        virtual ~OpExpression();
        i64 GetOpCode() const;
        const vector<Expression>& GetChildren() const;

    protected:
        virtual void ComputeHash() const override;

    public:
        virtual i32 Compare(const ExpressionBase* Other) const override;
        virtual void Accept(ExpressionVisitorBase* Visitor) const override;
    };

    class QuantifiedExpressionBase : public ExpressionBase
    {
    private:
        vector<Expression> QVarList;
        Expression QExpression;

    public:
        QuantifiedExpressionBase(ExprMgr* Manager,
                                 const vector<Expression>& QVarList,
                                 const Expression& QExpression);
        virtual ~QuantifiedExpressionBase();
        const vector<Expression>& GetQVarList() const;
        const Expression& GetQExpression() const;

    protected:
        i32 CompareInternal(const QuantifiedExpressionBase* Other) const;
        void ComputeHashInternal() const;

    public:
        virtual bool IsForAll() const = 0;
        virtual bool IsExists() const = 0;
    };

    class EQuantifiedExpression : public QuantifiedExpressionBase
    {
    public:
        using QuantifiedExpressionBase::QuantifiedExpressionBase;
        virtual ~EQuantifiedExpression();

    protected:
        virtual void ComputeHash() const override;
        
    public:
        virtual i32 Compare(const ExpressionBase* Other) const override;
        virtual void Accept(ExpressionVisitorBase* Visitor) const override;
        virtual bool IsForAll() const override;
        virtual bool IsExists() const override;
    };

    class AQuantifiedExpression : public QuantifiedExpressionBase
    {
    public:
        using QuantifiedExpressionBase::QuantifiedExpressionBase;
        virtual ~AQuantifiedExpression();

    protected:
        virtual void ComputeHash() const override;
        
    public:
        virtual i32 Compare(const ExpressionBase* Other) const override;
        virtual void Accept(ExpressionVisitorBase* Visitor) const override;
        virtual bool IsForAll() const override;
        virtual bool IsExists() const override;
    };

    // implementation of downcasts
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

#endif /* ESMC_EXPRESSIONS_HPP_ */

// 
// Expressions.hpp ends here
