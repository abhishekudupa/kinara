// Z3Objects.hpp ---
//
// Filename: Z3Objects.hpp
// Author: Abhishek Udupa
// Created: Wed Oct  8 10:23:14 2014 (-0400)
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

#if !defined ESMC_Z3_OBJECTS_HPP_
#define ESMC_Z3_OBJECTS_HPP_

#include <z3.h>
#include <map>
#include <unordered_map>

#include "../common/ESMCFwdDecls.hpp"
#include "../containers/RefCountable.hpp"
#include "../containers/SmartPtr.hpp"

namespace ESMC {
    namespace TP {

        // A wrapper for ref counting Z3 contexts
        class Z3CtxWrapper : public RefCountable
        {
        private:
            Z3_context Ctx;

        public:
            Z3CtxWrapper(Z3_context Ctx);
            Z3CtxWrapper();
            virtual ~Z3CtxWrapper();

            operator Z3_context () const;
            Z3_context GetCtx() const;
        };

        // Base class for all Z3 refcounted objects
        class Z3Object
        {
        protected:
            Z3Ctx Ctx;

        public:
            Z3Object();
            Z3Object(Z3Ctx Ctx);
            Z3Object(const Z3Object& Other);
            virtual ~Z3Object();

            const Z3Ctx& GetCtx() const;

            virtual bool operator == (const Z3Object& Other) const = 0;
            virtual u64 Hash() const = 0;
            virtual string ToString() const = 0;

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

        class Z3Expr : public Z3Object
        {
        private:
            Z3_ast AST;

        public:
            Z3Expr();
            Z3Expr(const Z3Expr& Other);
            Z3Expr(Z3Ctx Ctx, Z3_ast AST);
            Z3Expr(Z3Expr&& Other);
            virtual ~Z3Expr();

            Z3Expr& operator = (Z3Expr Other);

            virtual bool operator == (const Z3Object& Other) const override;
            virtual string ToString() const override;
            virtual u64 Hash() const override;


            // unsafe! use only if you know what you're doing
            operator Z3_ast () const;
            Z3_ast GetAST() const;
            const Z3Ctx& GetCtx() const;

            static Z3Expr NullExpr;
        };

        class Z3ExprHasher
        {
        public:
            inline u64 operator () (const Z3Expr& Expr) const
            {
                return Expr.Hash();
            }
        };

        class Z3Sort : public Z3Object
        {
        private:
            Z3_sort Sort;
            mutable unordered_map<string, Z3_func_decl> FuncDecls;

        public:
            Z3Sort();
            Z3Sort(const Z3Sort& Other);
            Z3Sort(Z3Ctx Ctx, Z3_sort Sort);
            Z3Sort(Z3Sort&& Other);
            virtual ~Z3Sort();

            // Helper to add ref counted func decls
            // as in the case for enums and records
            void AddFuncDecl(Z3_func_decl Decl) const;
            void AddFuncDecls(u32 NumDecls, Z3_func_decl* Decls) const;

            Z3_func_decl GetFuncDecl(const string& Name) const;

            Z3Sort& operator = (Z3Sort Other);

            virtual bool operator == (const Z3Object& Other) const override;
            virtual string ToString() const override;
            virtual u64 Hash() const override;

            // unsafe! for internal use only
            operator Z3_sort () const;
            Z3_sort GetSort() const;
            const Z3Ctx& GetCtx() const;

            static Z3Sort NullSort;
        };

        class Z3Solver : public Z3Object
        {
        private:
            Z3_solver Solver;

        public:
            Z3Solver();
            Z3Solver(Z3Ctx Ctx);
            Z3Solver(const Z3Solver& Other);
            Z3Solver(Z3Ctx Ctx, Z3_solver Solver);
            Z3Solver(Z3Solver&& Other);
            virtual ~Z3Solver();

            Z3Solver& operator = (Z3Solver Other);

            virtual bool operator == (const Z3Object& Other) const override;
            virtual string ToString() const override;
            virtual u64 Hash() const override;

            operator Z3_solver () const;
            Z3_solver GetSolver() const;

            static Z3Solver NullSolver;
        };

        class Z3Model : public Z3Object
        {
        private:
            Z3_model Model;
            TheoremProver* TPPtr;

        public:
            Z3Model();
            Z3Model(const Z3Model& Other);
            Z3Model(Z3Ctx Ctx, Z3_model Model, TheoremProver* TPPtr);
            Z3Model(Z3Model&& Other);
            virtual ~Z3Model();

            Z3Model& operator = (Z3Model Other);

            virtual bool operator == (const Z3Object& Other) const override;
            virtual string ToString() const override;
            virtual u64 Hash() const override;

            operator Z3_model () const;
            Z3_model GetModel() const;
            TheoremProver* GetTPPtr() const;

            static Z3Model NullModel;
        };

    } /* end namespace TP */
} /* end namespace ESMC */

#endif /* ESMC_Z3_OBJECTS_HPP_ */

//
// Z3Objects.hpp ends here
