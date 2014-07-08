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
        namespace Detail {

            // implementation of Z3 context wrapper
            Z3CtxWrapper::Z3CtxWrapper(Z3_context Ctx)
                : Ctx(Ctx)
            {
                // Nothing here
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
                if (*Ctx != nullptr && AST != nullptr) {
                    Z3_inc_ref(*Ctx, AST);
                }
            }

            Z3Expr::Z3Expr(Z3Ctx Ctx, Z3_ast AST)
                : Ctx(Ctx), AST(AST)
            {
                if (*Ctx != nullptr && AST != nullptr) {
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
                if (*Ctx != nullptr && AST != nullptr) {
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
                return ((*Ctx == *(Other.Ctx)) && 
                        (Z3_is_eq_ast(*Ctx, AST, Other.AST)));
            }

            string Z3Expr::ToString() const
            {
                if (*Ctx != nullptr && AST != nullptr) {
                    return Z3_ast_to_string(*Ctx, AST);
                } else {
                    return "nullexpr";
                }
            }

            u64 Z3Expr::Hash() const
            {
                if (*Ctx != nullptr && AST != nullptr) {
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

            UFDescriptor::UFDescriptor()
                : RangeType(-1), Name(""), MangledName("")
            {
                // Nothing here
            }

            UFDescriptor::UFDescriptor(const UFDescriptor& Other) 
                : DomainTypes(Other.DomainTypes), RangeType(Other.RangeType), 
                  Name(Other.Name), MangledName(Other.MangledName)
            {
                // Nothing here
            }

            UFDescriptor::UFDescriptor(const vector<i64>& DomainTypes,
                                       const i64 RangeType,
                                       const string& Name)
                : DomainTypes(DomainTypes), RangeType(RangeType),
                  Name(Name), MangledName(MangleName(Name, DomainTypes))
            {
                // Nothing here
            }

            UFDescriptor::~UFDescriptor()
            {
                // Nothing here
            }

            UFDescriptor& UFDescriptor::operator = (const UFDescriptor& Other)
            {
                if (&Other == this) {
                    return *this;
                }
                DomainTypes = Other.DomainTypes;
                RangeType = Other.RangeType;
                Name = Other.Name;
                MangledName = Other.MangledName;
                return *this;
            }

            const vector<i64>& UFDescriptor::GetDomainTypes() const
            {
                return DomainTypes;
            }

            i64 UFDescriptor::GetRangeType() const
            {
                return RangeType;
            }

            const string& UFDescriptor::GetName() const
            {
                return Name;
            }

            const string& UFDescriptor::GetMangledName() const
            {
                return MangledName;
            }

        } /* end namespace Detail */
    } /* end namespace Z3Sem */
} /* end namespace ESMC */

// 
// Z3Semanticizer.cpp ends here
