// Z3Objects.cpp ---
//
// Filename: Z3Objects.cpp
// Author: Abhishek Udupa
// Created: Wed Oct  8 10:26:08 2014 (-0400)
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

#include "../hash/SpookyHash.hpp"
#include <random>

#include "Z3Objects.hpp"

namespace ESMC {
    namespace TP {

        // implementation of Z3 context wrapper
        Z3CtxWrapper::Z3CtxWrapper(Z3_context Ctx)
            : Ctx(Ctx)
        {
            // Nothing here
        }

        Z3CtxWrapper::Z3CtxWrapper()
        {
            Z3_global_param_set("model_evaluator.completion", "true");
            auto Cfg = Z3_mk_config();
            // random_device rd;
            // auto Seed = rd() % (1 << 30);
            // cout << "Z3 Random Seed: " << Seed << endl << endl;
            // Z3_set_param_value(Cfg, "RANDOM_SEED", to_string(Seed).c_str());
            Ctx = Z3_mk_context_rc(Cfg);
            Z3_del_config(Cfg);
        }

        Z3CtxWrapper::~Z3CtxWrapper()
        {
            if (Ctx != nullptr) {
                Z3_del_context(Ctx);
                Ctx = nullptr;
            }
        }

        Z3CtxWrapper::operator Z3_context () const
        {
            return Ctx;
        }

        Z3_context Z3CtxWrapper::GetCtx() const
        {
            return Ctx;
        }

        // Implementation of Z3Object
        Z3Object::Z3Object()
            : Ctx(Z3Ctx::NullPtr)
        {
            // Nothing here
        }

        Z3Object::Z3Object(Z3Ctx Ctx)
            : Ctx(Ctx)
        {
            // Nothing here
        }

        Z3Object::Z3Object(const Z3Object& Other)
            : Ctx(Other.Ctx)
        {
            // Nothing here
        }

        Z3Object::~Z3Object()
        {
            // Nothing here
        }

        const Z3Ctx& Z3Object::GetCtx() const
        {
            return Ctx;
        }

        // Implementation of Z3Expr
        Z3Expr Z3Expr::NullExpr;

        Z3Expr::Z3Expr()
            : Z3Object(), AST(nullptr)
        {
            // Nothing here
        }

        Z3Expr::Z3Expr(const Z3Expr& Other)
            : Z3Object(Other), AST(Other.AST)
        {
            if (Ctx != Z3Ctx::NullPtr && AST != nullptr) {
                Z3_inc_ref(*Ctx, AST);
            }
        }

        Z3Expr::Z3Expr(Z3Ctx Ctx, Z3_ast AST)
            : Z3Object(Ctx), AST(AST)
        {
            if (Ctx != Z3Ctx::NullPtr && AST != nullptr) {
                Z3_inc_ref(*Ctx, AST);
            }
        }

        Z3Expr::Z3Expr(Z3Expr&& Other)
            : Z3Object(), AST(nullptr)
        {
            swap(Ctx, Other.Ctx);
            swap(AST, Other.AST);
        }

        Z3Expr::~Z3Expr()
        {
            if (Ctx != Z3Ctx::NullPtr && AST != nullptr) {
                Z3_dec_ref(*Ctx, AST);
            }
        }

        Z3Expr& Z3Expr::operator = (Z3Expr Other)
        {
            swap(Ctx, Other.Ctx);
            swap(AST, Other.AST);
            return *this;
        }

        bool Z3Expr::operator == (const Z3Object& Other) const
        {
            auto OtherPtr = Other.As<Z3Expr>();

            if (OtherPtr == nullptr) {
                return false;
            }
            if (Ctx != OtherPtr->Ctx) {
                return false;
            }
            if (Ctx == Z3Ctx::NullPtr) {
                return true;
            }

            return (Z3_is_eq_ast(*Ctx, AST, OtherPtr->AST));
        }

        string Z3Expr::ToString(u32 Verbosity) const
        {
            if (Ctx != Z3Ctx::NullPtr && AST != nullptr) {
                return Z3_ast_to_string(*Ctx, AST);
            } else {
                return "nullexpr";
            }
        }

        u64 Z3Expr::Hash() const
        {
            if (Ctx != Z3Ctx::NullPtr && AST != nullptr) {
                return (Z3_get_ast_hash(*Ctx, AST));
            } else {
                return 0;
            }
        }

        Z3Expr::operator Z3_ast() const
        {
            return AST;
        }

        Z3_ast Z3Expr::GetAST() const
        {
            return AST;
        }

        const Z3Ctx& Z3Expr::GetCtx() const
        {
            return Ctx;
        }

        // implementation of Z3Sort
        Z3Sort Z3Sort::NullSort;

        Z3Sort::Z3Sort()
            : Z3Object(), Sort(nullptr)
        {
            // Nothing here
        }

        Z3Sort::Z3Sort(Z3Ctx Ctx, Z3_sort Sort)
            : Z3Object(Ctx), Sort(Sort)
        {
            if (Ctx != Z3Ctx::NullPtr && Sort != nullptr) {
                Z3_inc_ref(*Ctx, Z3_sort_to_ast(*Ctx, Sort));
            }
        }

        Z3Sort::Z3Sort(const Z3Sort& Other)
            : Z3Object(Other), Sort(Other.Sort),
              FuncDecls(Other.FuncDecls)
        {
            if (Ctx != Z3Ctx::NullPtr && Sort != nullptr) {
                Z3_inc_ref(*Ctx, Z3_sort_to_ast(*Ctx, Sort));
            }
            for (auto const& FuncDecl : FuncDecls) {
                Z3_inc_ref(*Ctx, Z3_func_decl_to_ast(*Ctx, FuncDecl.second));
            }
        }

        Z3Sort::Z3Sort(Z3Sort&& Other)
            : Z3Object(), Sort(nullptr)
        {
            swap(Ctx, Other.Ctx);
            swap(Sort, Other.Sort);
            swap(FuncDecls, Other.FuncDecls);
        }

        Z3Sort::~Z3Sort()
        {
            if (Ctx != Z3Ctx::NullPtr && Sort != nullptr) {
                Z3_dec_ref(*Ctx, Z3_sort_to_ast(*Ctx, Sort));
            }
            for (auto const& FuncDecl : FuncDecls) {
                Z3_dec_ref(*Ctx, Z3_func_decl_to_ast(*Ctx, FuncDecl.second));
            }
            FuncDecls.clear();
        }

        void Z3Sort::AddFuncDecl(Z3_func_decl Decl) const
        {
            Z3_inc_ref(*Ctx, Z3_func_decl_to_ast(*Ctx, Decl));
            string Name(Z3_get_symbol_string(*Ctx, Z3_get_decl_name(*Ctx, Decl)));
            auto it = FuncDecls.find(Name);
            if (it != FuncDecls.end()) {
                Z3_dec_ref(*Ctx, Z3_func_decl_to_ast(*Ctx, it->second));
            }
            FuncDecls[Name] = Decl;
        }

        void Z3Sort::AddFuncDecls(u32 NumDecls, Z3_func_decl *Decls) const
        {
            for (u32 i = 0; i < NumDecls; ++i) {
                AddFuncDecl(Decls[i]);
            }
        }

        Z3_func_decl Z3Sort::GetFuncDecl(const string& Name) const
        {
            auto it = FuncDecls.find(Name);
            if (it == FuncDecls.end()) {
                throw ESMCError((string)"Request to get function decl \"" +
                                Name + "\" which does not exist in Z3Sort");
            }
            return it->second;
        }

        Z3Sort& Z3Sort::operator = (Z3Sort Other)
        {
            swap(Ctx, Other.Ctx);
            swap(Sort, Other.Sort);
            swap(FuncDecls, Other.FuncDecls);
            return *this;
        }

        bool Z3Sort::operator == (const Z3Object& Other) const
        {
            auto OtherAsPtr = Other.As<Z3Sort>();
            if (OtherAsPtr == nullptr) {
                return false;
            }
            if (Ctx != OtherAsPtr->Ctx) {
                return false;
            }
            if (Ctx == Z3Ctx::NullPtr) {
                return true;
            }

            return (Z3_is_eq_sort(*Ctx, Sort, OtherAsPtr->Sort));
        }

        string Z3Sort::ToString(u32 Verbosity) const
        {
            if (Ctx != Z3Ctx::NullPtr && Sort != nullptr) {
                return Z3_sort_to_string(*Ctx, Sort);
            } else {
                return "nullsort";
            }
        }

        u64 Z3Sort::Hash() const
        {
            if (Ctx != Z3Ctx::NullPtr && Sort != nullptr) {
                return Z3_get_ast_hash(*Ctx, Z3_sort_to_ast(*Ctx, Sort));
            } else {
                return 0;
            }
        }

        Z3Sort::operator Z3_sort () const
        {
            return Sort;
        }

        Z3_sort Z3Sort::GetSort() const
        {
            return Sort;
        }

        const Z3Ctx& Z3Sort::GetCtx() const
        {
            return Ctx;
        }

        // Z3Solver implementation
        Z3Solver::Z3Solver()
            : Z3Object(), Solver(nullptr)
        {
            // Nothing here
        }

        Z3Solver::Z3Solver(Z3Ctx Ctx)
            : Z3Object(Ctx)
        {
            if (Ctx != Z3Ctx::NullPtr) {
                Solver = Z3_mk_solver(*Ctx);
            } else {
                Solver = nullptr;
            }
            if (Ctx != Z3Ctx::NullPtr && Solver != nullptr) {
                Z3_solver_inc_ref(*Ctx, Solver);
            }
        }

        Z3Solver::Z3Solver(const Z3Solver& Other)
            : Z3Object(Other), Solver(Other.Solver)
        {
            if (Ctx != Z3Ctx::NullPtr && Solver != nullptr) {
                Z3_solver_inc_ref(*Ctx, Solver);
            }
        }

        Z3Solver::Z3Solver(Z3Ctx Ctx, Z3_solver Solver)
            : Z3Object(Ctx), Solver(Solver)
        {
            if (Ctx != Z3Ctx::NullPtr && Solver != nullptr) {
                Z3_solver_inc_ref(*Ctx, Solver);
            }
        }

        Z3Solver::Z3Solver(Z3Solver&& Other)
            : Z3Object(), Solver(nullptr)
        {
            swap(Ctx, Other.Ctx);
            swap(Solver, Other.Solver);
        }

        Z3Solver::~Z3Solver()
        {
            if (Ctx != Z3Ctx::NullPtr && Solver != nullptr) {
                Z3_solver_dec_ref(*Ctx, Solver);
            }
        }

        Z3Solver& Z3Solver::operator = (Z3Solver Other)
        {
            swap(Ctx, Other.Ctx);
            swap(Solver, Other.Solver);
            return *this;
        }

        bool Z3Solver::operator == (const Z3Object& Other) const
        {
            auto OtherAsPtr = Other.As<Z3Solver>();
            return ((OtherAsPtr != nullptr) &&
                    (OtherAsPtr->Ctx == Ctx) &&
                    (OtherAsPtr->Solver == Solver));
        }

        string Z3Solver::ToString(u32 Verbosity) const
        {
            if (Ctx != Z3Ctx::NullPtr && Solver != nullptr) {
                return string(Z3_solver_to_string(*Ctx, Solver));
            } else {
                return "nullsolver";
            }
        }

        u64 Z3Solver::Hash() const
        {
            auto&& SolverString = ToString();
            return SpookyHash::SpookyHash::Hash64(SolverString.c_str(),
                                                  SolverString.length(), 0);
        }

        Z3Solver::operator Z3_solver () const
        {
            return Solver;
        }

        Z3_solver Z3Solver::GetSolver() const
        {
            return Solver;
        }

        Z3Solver Z3Solver::NullSolver;


        // Z3Model implementation
        Z3Model::Z3Model()
            : Z3Object(), Model(nullptr), TPPtr(nullptr)
        {
            // Nothing here
        }

        Z3Model::Z3Model(const Z3Model& Other)
            : Z3Object(Other), Model(Other.Model), TPPtr(Other.TPPtr)
        {
            if (Ctx != Z3Ctx::NullPtr && Model != nullptr) {
                Z3_model_inc_ref(*Ctx, Model);
            }
        }

        Z3Model::Z3Model(Z3Ctx Ctx, Z3_model Model, Z3TheoremProver* TPPtr)
            : Z3Object(Ctx), Model(Model), TPPtr(TPPtr)
        {
            if (Ctx != Z3Ctx::NullPtr && Model != nullptr) {
                Z3_model_inc_ref(*Ctx, Model);
            }
        }

        Z3Model::Z3Model(Z3Model&& Other)
            : Z3Model()
        {
            swap(Ctx, Other.Ctx);
            swap(Model, Other.Model);
            swap(TPPtr, Other.TPPtr);
        }

        Z3Model::~Z3Model()
        {
            if (Ctx != Z3Ctx::NullPtr && Model != nullptr) {
                Z3_model_dec_ref(*Ctx, Model);
            }
        }

        Z3Model& Z3Model::operator = (Z3Model Other)
        {
            swap(Ctx, Other.Ctx);
            swap(Model, Other.Model);
            swap(TPPtr, Other.TPPtr);
            return *this;
        }

        bool Z3Model::operator == (const Z3Object& Other) const
        {
            auto OtherAsPtr = Other.As<Z3Model>();
            return (OtherAsPtr != nullptr &&
                    OtherAsPtr->Ctx == Ctx &&
                    OtherAsPtr->Model == Model &&
                    TPPtr == OtherAsPtr->TPPtr);
        }

        string Z3Model::ToString(u32 Verbosity) const
        {
            return string(Z3_model_to_string(*Ctx, Model));
        }

        u64 Z3Model::Hash() const
        {
            auto&& ModelString = ToString();
            return SpookyHash::SpookyHash::Hash64(ModelString.c_str(),
                                                  ModelString.length(), 0);
        }

        Z3Model::operator Z3_model () const
        {
            return Model;
        }

        Z3_model Z3Model::GetModel() const
        {
            return Model;
        }

        Z3TheoremProver* Z3Model::GetTPPtr() const
        {
            return TPPtr;
        }

        Z3Model Z3Model::NullModel;

    } /* end namespace TP */
} /* end namespace ESMC */

//
// Z3Objects.cpp ends here
