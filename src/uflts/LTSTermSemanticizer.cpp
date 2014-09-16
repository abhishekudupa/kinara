// LTSTermSemanticizer.cpp --- 
// 
// Filename: LTSTermSemanticizer.cpp
// Author: Abhishek Udupa
// Created: Sat Jul 26 15:58:15 2014 (-0400)
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

#include <z3.h>

#include "LTSTermSemanticizer.hpp"

namespace ESMC {
    namespace LTS {
        
        const i64 LTSOps::OpEQ;
        const i64 LTSOps::OpNOT;
        const i64 LTSOps::OpITE;
        const i64 LTSOps::OpOR;
        const i64 LTSOps::OpAND;
        const i64 LTSOps::OpIMPLIES;
        const i64 LTSOps::OpIFF;
        const i64 LTSOps::OpXOR;

        // Arithmetic
        const i64 LTSOps::OpADD;
        const i64 LTSOps::OpSUB;
        const i64 LTSOps::OpMINUS;
        const i64 LTSOps::OpMUL;
        const i64 LTSOps::OpDIV;
        const i64 LTSOps::OpMOD;
        const i64 LTSOps::OpGT;
        const i64 LTSOps::OpGE;
        const i64 LTSOps::OpLT;
        const i64 LTSOps::OpLE;

        // Syntactic operators for symmetry, etc
        const i64 LTSOps::OpIndex;
        const i64 LTSOps::OpField;

        const i64 LTSOps::UFOffset;
        
        // Temporal operators
        const i64 LTSOps::OpTemporalX;
        const i64 LTSOps::OpTemporalU;
        const i64 LTSOps::OpTemporalF;
        const i64 LTSOps::OpTemporalG;

        namespace Detail {

            const Exprs::ExprTypeRef InvalidType = Exprs::ExprTypeRef::NullPtr;

            const string BoundVarPrefix = "dbvar";

            const unordered_map<i64, string> OpCodeToNameMap = 
                { { LTSOps::OpEQ, "=" },
                  { LTSOps::OpNOT, "not" },
                  { LTSOps::OpITE, "ite" },
                  { LTSOps::OpOR, "or" },
                  { LTSOps::OpAND, "and" },
                  { LTSOps::OpIMPLIES, "implies" },
                  { LTSOps::OpIFF, "iff" },
                  { LTSOps::OpXOR, "xor" },
                  { LTSOps::OpADD, "+" },
                  { LTSOps::OpSUB, "-" },
                  { LTSOps::OpMINUS, "-" },
                  { LTSOps::OpMUL, "*" },
                  { LTSOps::OpDIV, "div" },
                  { LTSOps::OpMOD, "mod" },
                  { LTSOps::OpGT, ">" },
                  { LTSOps::OpGE, ">=" },
                  { LTSOps::OpLT, "<" },
                  { LTSOps::OpLE, "<=" } };

            // implementation of Z3 context wrapper
            Z3CtxWrapper::Z3CtxWrapper(Z3_context Ctx)
                : Ctx(Ctx)
            {
                // Nothing here
            }

            Z3CtxWrapper::Z3CtxWrapper()
            {
                auto Cfg = Z3_mk_config();
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

            // Implementation of Z3Expr
            Z3Expr Z3Expr::NullExpr;

            Z3Expr::Z3Expr()
                : Ctx(Z3Ctx::NullPtr), AST(nullptr)
            {
                // Nothing here
            }

            Z3Expr::Z3Expr(const Z3Expr& Other)
                : Ctx(Other.Ctx), AST(Other.AST)
            {
                if (Ctx != Z3Ctx::NullPtr && AST != nullptr) {
                    Z3_inc_ref(*Ctx, AST);
                }
            }

            Z3Expr::Z3Expr(Z3Ctx Ctx, Z3_ast AST)
                : Ctx(Ctx), AST(AST)
            {
                if (Ctx != Z3Ctx::NullPtr && AST != nullptr) {
                    Z3_inc_ref(*Ctx, AST);
                }
            }

            Z3Expr::Z3Expr(Z3Expr&& Other)
                : Ctx(Z3Ctx::NullPtr), AST(nullptr)
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

            bool Z3Expr::operator == (const Z3Expr& Other) const
            {
                return ((Ctx != Z3Ctx::NullPtr) && (Other.Ctx != Z3Ctx::NullPtr) &&
                        (Ctx == (Other.Ctx)) && 
                        (Z3_is_eq_ast(*Ctx, AST, Other.AST)));
            }

            string Z3Expr::ToString() const
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
                : Ctx(Z3Ctx::NullPtr), Sort(nullptr)
            {
                // Nothing here
            }

            Z3Sort::Z3Sort(Z3Ctx Ctx, Z3_sort Sort)
                : Ctx(Ctx), Sort(Sort)
            {
                if (Ctx != Z3Ctx::NullPtr && Sort != nullptr) {
                    Z3_inc_ref(*Ctx, Z3_sort_to_ast(*Ctx, Sort));
                }
            }

            Z3Sort::Z3Sort(const Z3Sort& Other)
                : Ctx(Other.Ctx), Sort(Other.Sort)
            {
                if (Ctx != Z3Ctx::NullPtr && Sort != nullptr) {
                    Z3_inc_ref(*Ctx, Z3_sort_to_ast(*Ctx, Sort));
                }                
            }

            Z3Sort::Z3Sort(Z3Sort&& Other)
                : Ctx(Z3Ctx::NullPtr), Sort(nullptr)
            {
                swap(Ctx, Other.Ctx);
                swap(Sort, Other.Sort);
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
                return *this;
            }

            bool Z3Sort::operator == (const Z3Sort& Other) const
            {
                return ((Ctx != Z3Ctx::NullPtr) && (Other.Ctx != Z3Ctx::NullPtr) &&
                        (Ctx == Other.Ctx) && 
                        (Z3_is_eq_sort(*Ctx, Sort, Other.Sort)));
            }

            string Z3Sort::ToString() const
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

        } /* end namespace Detail */

        // The LTSLoweredContext implementation
        
        using namespace Detail;

        LTSLoweredContext::LTSLoweredContext()
            : Ctx(new Z3CtxWrapper())
        {
            // Nothing here
        }

        LTSLoweredContext::~LTSLoweredContext()
        {
            // Nothing here
        }

        const Z3Sort& LTSLoweredContext::GetZ3Sort(const ExprTypeRef& LTSType) const
        {
            auto it = LTSTypeToSort.find(LTSType);
            if (it == LTSTypeToSort.end()) {
                return Z3Sort::NullSort;
            } else {
                return it->second;
            }
        }

        void LTSLoweredContext::AddZ3Sort(const ExprTypeRef& LTSType, 
                                          const Z3Sort& Sort) const
        {
            if (GetZ3Sort(LTSType) != Z3Sort::NullSort) {
                throw ExprTypeError((string)"Z3 sort for type \"" + LTSType->ToString() + 
                                    "\" already exists");
            }
            LTSTypeToSort[LTSType] = Sort;
        }

        const ExprTypeRef& LTSLoweredContext::GetLTSType(const string& VarName) const
        {
            auto it = VarNameToLTSType.find(VarName);
            if (it == VarNameToLTSType.end()) {
                return ExprTypeRef::NullPtr;
            } else {
                return it->second;
            }
        }

        void LTSLoweredContext::AddLTSType(const string& VarName, const ExprTypeRef& LTSType) const
        {
            if (GetLTSType(VarName) != ExprTypeRef::NullPtr) {
                throw ExprTypeError((string)"Error, variable named \"" + VarName + "\"" + 
                                    " has already been registed with a type in the context!");
            } 
            VarNameToLTSType[VarName] = LTSType;
        }

        void LTSLoweredContext::AddAssumption(const Z3Expr& Assumption) const
        {
            Assumptions.push_back(Assumption);
        }

        const vector<Z3Expr>& LTSLoweredContext::GetAssumptions() const
        {
            return Assumptions;
        }

        void LTSLoweredContext::ClearAssumptions() const
        {
            Assumptions.clear();
        }

        const Z3Ctx& LTSLoweredContext::GetZ3Ctx() const
        {
            return Ctx;
        }
        
    } /* end namespace LTS */
} /* end namespace ESMC */


// 
// LTSTermSemanticizer.cpp ends here
