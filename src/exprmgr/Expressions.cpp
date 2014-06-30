// Expressions.cpp --- 
// 
// Filename: Expressions.cpp
// Author: Abhishek Udupa
// Created: Mon Jun 30 01:42:31 2014 (-0400)
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

#include "Expressions.hpp"
#include "ExprMgr.hpp"
#include "../utils/UIDGenerator.hpp"

#include <boost/functional/hash.hpp>

namespace ESMC {

    // ExpressionBase implementation
    ExpressionBase::ExpressionBase(ExprMgr* Manager)
        : Manager(Manager), HashValid(false), HashCode(0)
    {
        // Nothing here
    }

    ExpressionBase::~ExpressionBase()
    {
        // Nothing here
    }

    ExprMgr* ExpressionBase::GetMgr() const 
    {
        return Manager;
    }

    u64 ExpressionBase::Hash() const
    {
        if (!HashValid) {
            ComputeHash();
            HashValid = true;
        }
        return HashCode;
    }

    bool ExpressionBase::Equals(const ExpressionBase* Other) const
    {
        if (Hash() != Other->Hash()) {
            return false;
        }
        return (Compare(Other) == 0);
    }

    bool ExpressionBase::NEquals(const ExpressionBase* Other) const
    {
        if (Hash() != Other->Hash()) {
            return true;
        }
        return (Compare(Other) != 0);
    }

    bool ExpressionBase::LT(const ExpressionBase* Other) const
    {
        return (Compare(Other) < 0);
    }

    bool ExpressionBase::LE(const ExpressionBase* Other) const
    {
        return (Compare(Other) <= 0);
    }

    bool ExpressionBase::GT(const ExpressionBase* Other) const
    {
        return (Compare(Other) > 0);
    }

    bool ExpressionBase::GE(const ExpressionBase* Other) const
    {
        return (Compare(Other) >= 0);
    }


    string ExpressionBase::ToString() const
    {
        auto Sem = Manager->GetSemanticizer();
        return (Sem->ExprToString(this));
    }

    // ConstExpression implementation
    ConstExpression::ConstExpression(ExprMgr* Manager, const string& ConstValue,
                                     i64 ConstType)
        : ExpressionBase(Manager), ConstValue(ConstValue), ConstType(ConstType)
    {
        // Nothing here
    }

    ConstExpression::~ConstExpression()
    {
        // Nothing here
    }

    const string& ConstExpression::GetConstValue() const
    {
        return ConstValue;
    }

    i64 ConstExpression::GetConstType() const
    {
        return ConstType;
    }

    i32 ConstExpression::Compare(const ExpressionBase* Other) const
    {
        auto OtherAsConst = Other->As<ConstExpression>();
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

    void ConstExpression::ComputeHash() const
    {
        boost::hash_combine(HashCode, ConstValue);
        boost::hash_combine(HashCode, ConstType);
    }

    void ConstExpression::Accept(ExpressionVisitorBase* Visitor) const 
    {
        Visitor->VisitConstExpression(this);
    }

    // VarExpression implementation
    VarExpression::VarExpression(ExprMgr* Manager, const string& VarName,
                                 i64 VarType)
        : ExpressionBase(Manager),
          VarName(VarName), VarType(VarType)
    {
        // Nothing here
    }

    VarExpression::~VarExpression()
    {
        // Nothing here
    }

    const string& VarExpression::GetVarName() const 
    {
        return VarName;
    }

    i64 VarExpression::GetVarType() const
    {
        return VarType;
    }

    i32 VarExpression::Compare(const ExpressionBase* Other) const
    {
        auto OtherAsVar = Other->As<VarExpression>();
        auto OtherAsConst = Other->As<ConstExpression>();
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

    void VarExpression::ComputeHash() const
    {
        boost::hash_combine(HashCode, VarName);
        boost::hash_combine(HashCode, VarType);
    }

    void VarExpression::Accept(ExpressionVisitorBase* Visitor) const
    {
        Visitor->VisitVarExpression(this);
    }

    
    // BoundVarExpression implementation
    BoundVarExpression::BoundVarExpression(ExprMgr* Manager, const string& VarName,
                                           i64 VarType, i64 VarUID)
        : VarExpression(Manager, VarName, VarType),
          VarUID(VarUID == -1 ? UIDGen.GetUID() : VarUID)
    {
        // Nothing here
    }

    BoundVarExpression::~BoundVarExpression()
    {
        // Nothing here
    }

    u64 BoundVarExpression::GetVarUID() const
    {
        return VarUID;
    }

    i32 BoundVarExpression::Compare(const ExpressionBase* Other) const
    {
        auto OtherAsConst = Other->As<ConstExpression>();
        // ConstExpression < BoundVarExpression
        if (OtherAsConst != nullptr) {
            return 1;
        }
        auto OtherAsVar = Other->As<VarExpression>();
        if (OtherAsVar != nullptr) {
            auto OtherAsBound = Other->As<BoundVarExpression>();
            // VarExpression < BoundVarExpression
            if (OtherAsBound == nullptr) {
                return 1;
            } else {
                auto Cmp = VarExpression::Compare(Other);
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

    void BoundVarExpression::ComputeHash() const
    {
        VarExpression::ComputeHash();
        boost::hash_combine(HashCode, VarUID);
    }

    void BoundVarExpression::Accept(ExpressionVisitorBase* Visitor) const
    {
        Visitor->VisitBoundVarExpression(this);
    }

    
    // OpExpression implementation
    OpExpression::OpExpression(ExprMgr* Manager, i64 OpCode,
                               const vector<Expression> Children)
        : ExpressionBase(Manager), OpCode(OpCode), Children(Children)
    {
        // Nothing here
    }

    OpExpression::~OpExpression()
    {
        // Nothing here
    }

    i64 OpExpression::GetOpCode() const
    {
        return OpCode;
    }

    const vector<Expression>& OpExpression::GetChildren() const
    {
        return Children;
    }

    i32 OpExpression::Compare(const ExpressionBase* Other) const
    {
        if ((Other->As<ConstExpression>() != nullptr) ||
            (Other->As<VarExpression>() != nullptr) ||
            (Other->As<BoundVarExpression>() != nullptr)) {
            return 1;
        } 
        auto OtherAsOp = Other->As<OpExpression>();
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

    void OpExpression::ComputeHash() const
    {
        boost::hash_combine(HashCode, OpCode);
        for (auto const& Child : Children) {
            boost::hash_combine(HashCode, Child->Hash());
        }
    }

    void OpExpression::Accept(ExpressionVisitorBase *Visitor) const
    {
        Visitor->VisitOpExpression(this);
    }

    QuantifiedExpressionBase::QuantifiedExpressionBase(ExprMgr* Manager,
                                                       const vector<Expression>& QVarList,
                                                       const Expression& QExpression)
        : ExpressionBase(Manager), QVarList(QVarList), QExpression(QExpression)
    {
        // Nothing here
    }

    QuantifiedExpressionBase::~QuantifiedExpressionBase()
    {
        // Nothing here
    }

    const vector<Expression>& QuantifiedExpressionBase::GetQVarList() const
    {
        return QVarList;
    }

    const Expression& QuantifiedExpressionBase::GetQExpression() const
    {
        return QExpression;
    }

    i32 QuantifiedExpressionBase::CompareInternal(const QuantifiedExpressionBase* Other) const
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

    void QuantifiedExpressionBase::ComputeHashInternal() const
    {
        for (auto const& QVar : QVarList) {
            boost::hash_combine(HashCode, QVar->Hash());
        }
        
        boost::hash_combine(HashCode, QExpression->Hash());
    }

    
    // EQuantifiedExpression implementation
    EQuantifiedExpression::~EQuantifiedExpression()
    {
        // Nothing here
    }

    i32 EQuantifiedExpression::Compare(const ExpressionBase *Other) const
    {
        if ((Other->As<VarExpression>() != nullptr) ||
            (Other->As<ConstExpression>() != nullptr) ||
            (Other->As<BoundVarExpression>() != nullptr) ||
            (Other->As<OpExpression>() != nullptr)) {
            return 1;
        }
        auto OtherAsExists = Other->As<EQuantifiedExpression>();
        if (OtherAsExists == nullptr) {
            return -1;
        }

        return QuantifiedExpressionBase::Compare(OtherAsExists);
    }

    void EQuantifiedExpression::ComputeHash() const
    {
        ComputeHashInternal();
        boost::hash_combine(HashCode, (string)"exists");
    }

    void EQuantifiedExpression::Accept(ExpressionVisitorBase* Visitor) const
    {
        Visitor->VisitEQuantifiedExpression(this);
    }

    bool EQuantifiedExpression::IsForAll() const
    {
        return false;
    }

    bool EQuantifiedExpression::IsExists() const
    {
        return true;
    }


    // AQuantifiedExpression implementation
    AQuantifiedExpression::~AQuantifiedExpression()
    {
        // Nothing here
    }

    i32 AQuantifiedExpression::Compare(const ExpressionBase *Other) const
    {
        if ((Other->As<VarExpression>() != nullptr) ||
            (Other->As<ConstExpression>() != nullptr) ||
            (Other->As<BoundVarExpression>() != nullptr) ||
            (Other->As<OpExpression>() != nullptr) || 
            (Other->As<EQuantifiedExpression>() != nullptr)) {
            return 1;
        }
        auto OtherAsForall = Other->As<AQuantifiedExpression>();
        // In case we decide to add more expression types
        if (OtherAsForall != nullptr) {
            return -1;
        }

        return QuantifiedExpressionBase::Compare(OtherAsForall);
    }

    void AQuantifiedExpression::ComputeHash() const
    {
        ComputeHashInternal();
        boost::hash_combine(HashCode, (string)"forall");
    }

    void AQuantifiedExpression::Accept(ExpressionVisitorBase* Visitor) const
    {
        Visitor->VisitAQuantifiedExpression(this);
    }

    bool AQuantifiedExpression::IsForAll() const
    {
        return true;
    }

    bool AQuantifiedExpression::IsExists() const
    {
        return false;
    }

    ExpressionVisitorBase::ExpressionVisitorBase(const string& Name)
        : Name(Name)
    {
        // Nothing here
    }

    ExpressionVisitorBase::~ExpressionVisitorBase()
    {
        // Nothing here
    }

    const string& ExpressionVisitorBase::GetName() const
    {
        return Name;
    }

    void ExpressionVisitorBase::VisitConstExpression(const ConstExpression* Exp)
    {
        // Nothing here
    }

    void ExpressionVisitorBase::VisitVarExpression(const VarExpression* Exp)
    {
        // Nothing here
    }

    void ExpressionVisitorBase::VisitBoundVarExpression(const BoundVarExpression* Exp)
    {
        // Nothing here
    }

    void ExpressionVisitorBase::VisitOpExpression(const OpExpression* Exp)
    {
        auto const& Children = Exp->GetChildren();
        for (auto const& Child : Children) {
            Child->Accept(this);
        }
    }

    void ExpressionVisitorBase::VisitAQuantifiedExpression(const AQuantifiedExpression *Exp)
    {
        auto const& QVars = Exp->GetQVarList();
        for(auto const& QVar : QVars) {
            QVar->Accept(this);
        }
        Exp->GetQExpression()->Accept(this);
    }

    void ExpressionVisitorBase::VisitEQuantifiedExpression(const EQuantifiedExpression *Exp)
    {
        auto const& QVars = Exp->GetQVarList();
        for(auto const& QVar : QVars) {
            QVar->Accept(this);
        }
        Exp->GetQExpression()->Accept(this);
    }
    
} /* end namespace */

// 
// Expressions.cpp ends here
