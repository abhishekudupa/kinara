// SymbolTable.cpp --- 
// 
// Filename: SymbolTable.cpp
// Author: Abhishek Udupa
// Created: Sun Jul 27 19:20:38 2014 (-0400)
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

#include "../expr/ExprTypes.hpp"

#include "SymbolTable.hpp"

#include <boost/functional/hash.hpp>

namespace ESMC {
    namespace LTS {

        DeclBase::DeclBase(const string& DeclName)
            : DeclName(DeclName),
              HashCode(0), HashValid(false)
        {
            // Nothing here
        }
        
        DeclBase::~DeclBase()
        {
            // Nothing here
        }

        const string& DeclBase::GetDeclName() const
        {
            return DeclName;
        }

        u64 DeclBase::Hash() const 
        {
            if (!HashValid) {
                ComputeHashValue();
                HashValid = true;
            }
            return HashCode;
        }

        ParamDecl::ParamDecl(const string& Name, const Exprs::ExprTypeRef& Type)
            : DeclBase(Name), ParamType(Type)
        {
            if (!(Type->Is<Exprs::ExprSymmetricType>() ||
                  Type->Is<Exprs::ExprRangeType>())) {
                throw ESMCError("Parameters can only be range or symmetric types");
            }
        }

        ParamDecl::~ParamDecl()
        {
            // Nothing here
        }

        void ParamDecl::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, "Param");
            boost::hash_combine(HashCode, GetDeclName());
            boost::hash_combine(HashCode, ParamType->Hash());
        }

        const Exprs::ExprTypeRef& ParamDecl::GetType() const
        {
            return ParamType;
        }

        bool ParamDecl::Equals(const DeclBase& Other) const
        {
            if (!Other.Is<ParamDecl>()) {
                return false;
            }
            auto OtherPtr = Other.As<ParamDecl>();
            return (OtherPtr->GetDeclName() == GetDeclName() &&
                    OtherPtr->GetType() == ParamType);
        }

        MsgDecl::MsgDecl(const string& Name, const Exprs::ExprTypeRef& Type)
            : DeclBase(Name), MsgType(Type)
        {
            if (!Type->Is<Exprs::ExprRecordType>()) {
                throw ESMCError((string)"Message decls must be record types");
            }
        }

        MsgDecl::~MsgDecl() 
        {
            // Nothing here
        }

        void MsgDecl::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, "Msg");
            boost::hash_combine(HashCode, GetDeclName());
            boost::hash_combine(HashCode, MsgType->Hash());
        }

        const Exprs::ExprTypeRef& MsgDecl::GetType() const
        {
            return MsgType;
        }

        bool MsgDecl::Equals(const DeclBase& Other) const
        {
            if (!Other.Is<MsgDecl>()) {
                return false;
            }

            auto OtherPtr = Other.As<MsgDecl>();
            return (OtherPtr->GetDeclName() == GetDeclName() &&
                    OtherPtr->GetType() == MsgType);
        }
        
        VarDecl::VarDecl(const string& Name, const Exprs::ExprTypeRef& Type)
            : DeclBase(Name), VarType(Type)
        {
            // Nothing here
        }

        VarDecl::~VarDecl()
        {
            // Nothing here
        }

        void VarDecl::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, "Var");
            boost::hash_combine(HashCode, GetDeclName());
            boost::hash_combine(HashCode, VarType->Hash());
        }

        bool VarDecl::Equals(const DeclBase& Other) const
        {
            if (!Other.Is<VarDecl>()) {
                return false;
            }
            auto OtherPtr = Other.As<VarDecl>();
            return (OtherPtr->GetDeclName() == GetDeclName() &&
                    OtherPtr->GetType() == VarType);
        }

        const Exprs::ExprTypeRef& VarDecl::GetType() const
        {
            return VarType;
        }

        StateDecl::StateDecl(const Exprs::ExprTypeRef& Type)
            : DeclBase("state"), Type(Type)
        {
            if (!Type->Is<Exprs::ExprEnumType>()) {
                throw ESMCError((string)"State variable must be of enumerated type");
            }
        }
        
        StateDecl::~StateDecl()
        {
            // Nothing here
        }

        void StateDecl::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, "state");
            boost::hash_combine(HashCode, Type->Hash());
        }

        bool StateDecl::Equals(const DeclBase& Other) const
        {
            if (!Other.Is<StateDecl>()) {
                return false;
            }
            
            auto OtherPtr = Other.As<StateDecl>();
            return (OtherPtr->Type == Type);
        }

        const Exprs::ExprTypeRef& StateDecl::GetType() const
        {
            return Type;
        }

        SymtabScope::SymtabScope()
        {
            // Nothing here
        }

        SymtabScope::~SymtabScope()
        {
            // Nothing here
        }

        const unordered_map<string, DeclRef>& SymtabScope::GetDeclMap() const
        {
            return DeclMap;
        }

        void SymtabScope::Bind(const string& Name, const DeclRef& Decl)
        {
            auto it = DeclMap.find(Name);
            if (it != DeclMap.end()) {
                throw ESMCError((string)"Redeclaration of name \"" + Name + "\"");
            }
            DeclMap[Name] = Decl;
        }

        const DeclRef& SymtabScope::Lookup(const string& Name) const
        {
            auto it = DeclMap.find(Name);
            if (it == DeclMap.end()) {
                return DeclRef::NullPtr;
            } else {
                return it->second; 
            }
        }

        SymbolTable::SymbolTable()
        {
            ScopeStack.push_back(new SymtabScope());
        }

        SymbolTable::SymbolTable(const SymbolTable& Other)
            : ScopeStack(Other.ScopeStack)
        {
            // Nothing here
        }

        SymbolTable::~SymbolTable()
        {
            // Nothing here
        }

        void SymbolTable::Push()
        {
            ScopeStack.push_back(new SymtabScope());
        }

        void SymbolTable::Push(const ScopeRef& Scope)
        {
            ScopeStack.push_back(Scope);
        }

        ScopeRef SymbolTable::Pop()
        {
            auto Retval = ScopeStack.back();
            ScopeStack.pop_back();
            return Retval;
        }

        ScopeRef SymbolTable::Top() const
        {
            return ScopeStack.back();
        }

        ScopeRef SymbolTable::Bot() const
        {
            return ScopeStack[0];
        }

        void SymbolTable::Bind(const string& Name, const DeclRef& Decl)
        {
            ScopeStack.back()->Bind(Name, Decl);
        }

        const DeclRef& SymbolTable::Lookup(const string& Name) const
        {
            for (auto it = ScopeStack.rbegin(); it != ScopeStack.rend(); ++it) {
                auto const& Res = (*it)->Lookup(Name);
                if (Res != DeclRef::NullPtr) {
                    return Res;
                }
            }
            return DeclRef::NullPtr;
        }

        const DeclRef& SymbolTable::LookupTop(const string& Name) const
        {
            return ScopeStack.back()->Lookup(Name);
        }

        SymbolTable& SymbolTable::operator = (const SymbolTable& Other)
        {
            if (&Other == this) {
                return *this;
            }
            ScopeStack = Other.ScopeStack;
            return *this;
        }
        
    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// SymbolTable.cpp ends here
