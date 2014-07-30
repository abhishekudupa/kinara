// SymbolTable.hpp --- 
// 
// Filename: SymbolTable.hpp
// Author: Abhishek Udupa
// Created: Thu Jul 24 10:42:42 2014 (-0400)
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

#if !defined ESMC_SYMBOL_TABLE_HPP_
#define ESMC_SYMBOL_TABLE_HPP_

#include "../common/FwdDecls.hpp"
#include "../containers/RefCountable.hpp"
#include "../containers/SmartPtr.hpp"

#include <unordered_map>
#include <vector>

namespace ESMC {
    namespace LTS {
        
        class DeclBase : public RefCountable
        {
        private:
            string DeclName;

        protected:
            mutable u64 HashCode;
            mutable bool HashValid;

            virtual void ComputeHashValue() const = 0;

        public:
            DeclBase(const string& DeclName);
            virtual ~DeclBase();

            const string& GetDeclName() const;

            u64 Hash() const;

            virtual bool Equals(const DeclBase& Other) const = 0;
            virtual const Exprs::ExprTypeRef& GetType() const = 0;

            template <typename T>
            inline T* As()
            {
                return dynamic_cast<T*>(this);
            }

            template <typename T>
            inline const T* As() const
            {
                return dynamic_cast<const T*>(this);
            }

            template <typename T>
            inline T* SAs()
            {
                return static_cast<T*>(this);
            }

            template <typename T>
            inline const T* SAs() const
            {
                return static_cast<const T*>(this);
            }

            template <typename T>
            inline bool Is() const
            {
                return (dynamic_cast<const T*>(this) != nullptr);
            }
        };

        typedef CSmartPtr<DeclBase> DeclRef;

        class ParamDecl : public DeclBase
        {
        private:
            Exprs::ExprTypeRef ParamType;

        protected:
            virtual void ComputeHashValue() const override;

        public:
            ParamDecl(const string& Name, const Exprs::ExprTypeRef& Type);
            virtual ~ParamDecl();

            virtual bool Equals(const DeclBase& Other) const override;
            virtual const Exprs::ExprTypeRef& GetType() const override;
        };

        class MsgDecl : public DeclBase
        {
        private:
            Exprs::ExprTypeRef MsgType;

        protected:
            virtual void ComputeHashValue() const override;
            
        public:
            MsgDecl(const string& Name, const Exprs::ExprTypeRef& Type);
            ~MsgDecl();
            
            virtual bool Equals(const DeclBase& Other) const override;
            virtual const Exprs::ExprTypeRef& GetType() const override;
        };

        class VarDecl : public DeclBase
        {
        private:
            Exprs::ExprTypeRef VarType;

        protected:
            virtual void ComputeHashValue() const override;

        public:
            VarDecl(const string& Name, const Exprs::ExprTypeRef& Type);
            virtual ~VarDecl();

            virtual bool Equals(const DeclBase& Other) const override;
            virtual const Exprs::ExprTypeRef& GetType() const override;
        };

        class StateDecl : public DeclBase
        {
        private:
            Exprs::ExprTypeRef Type;

        protected:
            virtual void ComputeHashValue() const override;
            
        public:
            StateDecl(const Exprs::ExprTypeRef& Type);
            virtual ~StateDecl();

            virtual bool Equals(const DeclBase& Other) const override;
            virtual const Exprs::ExprTypeRef& GetType() const override;            
        };

        class SymtabScope : public RefCountable
        {
        private:
            unordered_map<string, DeclRef> DeclMap;

        public:
            SymtabScope();
            virtual ~SymtabScope();

            const unordered_map<string, DeclRef>& GetDeclMap() const;
            void Bind(const string& Name, const DeclRef& Decl);
            const DeclRef& Lookup(const string& Name) const;
        };

        typedef SmartPtr<SymtabScope> ScopeRef;

        class SymbolTable
        {
        private:
            vector<ScopeRef> ScopeStack;

        public:
            SymbolTable();
            SymbolTable(const SymbolTable& Other);
            ~SymbolTable();

            void Push();
            void Push(const ScopeRef& Scope);

            ScopeRef Pop();
            ScopeRef Top() const;
            ScopeRef Bot() const;
            
            void Bind(const string& Name, const DeclRef& Decl);
            const DeclRef& Lookup(const string& Name) const;
            const DeclRef& LookupTop(const string& Name) const;

            SymbolTable& operator = (const SymbolTable& Other);
        };
        
    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_SYMBOL_TABLE_HPP_ */

// 
// SymbolTable.hpp ends here

