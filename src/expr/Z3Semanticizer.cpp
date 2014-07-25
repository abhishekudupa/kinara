// Z3Semanticizer.cpp --- 
// 
// Filename: Z3Semanticizer.cpp
// Author: Abhishek Udupa
// Created: Mon Jul  7 02:59:59 2014 (-0400)
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

#include "Z3Semanticizer.hpp"

namespace ESMC {
    namespace Z3Sem {

        const i64 Z3SemOps::OpEQ;
        const i64 Z3SemOps::OpNOT;
        const i64 Z3SemOps::OpITE;
        const i64 Z3SemOps::OpOR;
        const i64 Z3SemOps::OpAND;
        const i64 Z3SemOps::OpIMPLIES;
        const i64 Z3SemOps::OpIFF;
        const i64 Z3SemOps::OpXOR;

        // Arithmetic operators
        const i64 Z3SemOps::OpADD;
        const i64 Z3SemOps::OpSUB;
        const i64 Z3SemOps::OpMINUS;
        const i64 Z3SemOps::OpMUL;
        const i64 Z3SemOps::OpDIV;
        const i64 Z3SemOps::OpMOD;
        const i64 Z3SemOps::OpREM;
        const i64 Z3SemOps::OpPOWER;
        const i64 Z3SemOps::OpGT;
        const i64 Z3SemOps::OpGE;
        const i64 Z3SemOps::OpLT;
        const i64 Z3SemOps::OpLE;

        // Basic Bitvector operators
        // used for emulating bounded sets
        const i64 Z3SemOps::OpBVNOT;
        const i64 Z3SemOps::OpBVREDAND;
        const i64 Z3SemOps::OpBVREDOR;
        const i64 Z3SemOps::OpBVAND;
        const i64 Z3SemOps::OpBVOR;
        const i64 Z3SemOps::OpBVXOR;

        // Types
        const i64 Z3SemOps::BoolType;
        const i64 Z3SemOps::IntType;
        const i64 Z3SemOps::BVTypeAll;

        namespace Detail {
            
            // Prefix for bound variables
            const string BoundVarPrefix = "dbvar";

            // Definition of the OpCodeToNameMap
            const unordered_map<i64, string> OpCodeToNameMap = 
                { { Z3SemOps::OpEQ, "=" },
                  { Z3SemOps::OpNOT, "not" },
                  { Z3SemOps::OpITE, "ite" },
                  { Z3SemOps::OpOR, "or" },
                  { Z3SemOps::OpAND, "and" },
                  { Z3SemOps::OpIMPLIES, "implies" },
                  { Z3SemOps::OpIFF, "iff" },
                  { Z3SemOps::OpXOR, "xor" },
                  { Z3SemOps::OpADD, "+" },
                  { Z3SemOps::OpSUB, "-" },
                  { Z3SemOps::OpMINUS, "-" },
                  { Z3SemOps::OpMUL, "*" },
                  { Z3SemOps::OpDIV, "div" },
                  { Z3SemOps::OpMOD, "mod" },
                  { Z3SemOps::OpREM, "rem" },
                  { Z3SemOps::OpPOWER, "power" },
                  { Z3SemOps::OpGT, ">" },
                  { Z3SemOps::OpGE, ">=" },
                  { Z3SemOps::OpLT, "<" },
                  { Z3SemOps::OpLE, "<=" },
                  { Z3SemOps::OpBVNOT, "bvnot" },
                  { Z3SemOps::OpBVREDAND, "bvredand" },
                  { Z3SemOps::OpBVREDOR, "bvredor" },
                  { Z3SemOps::OpBVAND, "bvand" },
                  { Z3SemOps::OpBVOR, "bvor" },
                  { Z3SemOps::OpBVXOR, "bvxor" } };
            

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

            ostream& operator << (ostream& Out, const Z3Expr& Exp)
            {
                Out << Exp.ToString();
                return Out;
            }

        } /* end namespace Detail */
    } /* end namespace Z3Sem */
} /* end namespace ESMC */

// 
// Z3Semanticizer.cpp ends here
