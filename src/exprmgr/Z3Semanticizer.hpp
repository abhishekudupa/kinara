// Z3Semanticizer.hpp --- 
// 
// Filename: Z3Semanticizer.hpp
// Author: Abhishek Udupa
// Created: Thu Jul  3 13:20:53 2014 (-0400)
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

#if !defined ESMC_Z3_SEMANTICIZER_HPP_
#define ESMC_Z3_SEMANTICIZER_HPP_

#include <z3.h>
#include <vector>
#include <unordered_map>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
#include <algorithm>

#include "Expressions.hpp"

namespace ESMC {
    namespace Z3Sem {

        using namespace ESMC::Exprs;

        // Opcodes between 1000 and 100000
        // UF between 100000 and 1000000
        // Sort identifiers between 1000000 and 2000000
        const i64 OpCodeOffset = 1000;
        const i64 UFOffset = 100000;
        const i64 SortOffset = 1000000;
        const i64 OpEQ = 1000;
        const i64 OpNOT = 1001;
        const i64 OpITE = 1002;
        const i64 OpOR = 1003;
        const i64 OpAND = 1004;
        const i64 OpIMPLIES = 1005;
        const i64 OpIFF = 1006;
        const i64 OpXOR = 1007;

        // Arithmetic operators
        const i64 OpADD = 1008;
        const i64 OpSUB = 1009;
        const i64 OpMINUS = 1010;
        const i64 OpMUL = 1011;
        const i64 OpDIV = 1012;
        const i64 OpMOD = 1013;
        const i64 OpREM = 1014;
        const i64 OpPOWER = 1015;
        const i64 OpGT = 1016;
        const i64 OpGE = 1017;
        const i64 OpLT = 1018;
        const i64 OpLE = 1019;

        // Basic Bitvector operators
        // used for emulating bounded sets
        const i64 OpBVNOT = 1020;
        const i64 OpBVREDAND = 1021;
        const i64 OpBVREDOR = 1022;
        const i64 OpBVAND = 1023;
        const i64 OpBVOR = 1024;
        const i64 OpBVXOR = 1025;

        // Types
        const i64 BoolType = 1000000;
        const i64 IntType = 1000001;
        // BVTypes range from 1000100 to 1001000
        // 1000100 = ANY BV type
        // 100xxxx = BVType of len (1000100 - v)
        const i64 BVTypeOffset = 1000100;
        const i64 BVTypeEnd = 1001000;
        const i64 BVTypeAll = 1000100;

        namespace Detail {

            // map from opcodes to names
            unordered_map<i64, string> OpCodeToNameMap = 
                { { OpEQ, "=" },
                  { OpNOT, "not" },
                  { OpITE, "ite" },
                  { OpOR, "or" },
                  { OpAND, "and" },
                  { OpIMPLIES, "implies" },
                  { OpIFF, "iff" },
                  { OpXOR, "xor" },
                  { OpADD, "+" },
                  { OpSUB, "-" },
                  { OpMINUS, "-" },
                  { OpMUL, "*" },
                  { OpDIV, "div" },
                  { OpMOD, "mod" },
                  { OpREM, "rem" },
                  { OpPOWER, "power" },
                  { OpGT, ">" },
                  { OpGE, ">=" },
                  { OpLT, "<" },
                  { OpLE, "<=" },
                  { OpBVNOT, "bvnot" },
                  { OpBVREDAND, "bvredand" },
                  { OpBVREDOR, "bvredor" },
                  { OpBVAND, "bvand" },
                  { OpBVOR, "bvor" },
                  { OpBVXOR, "bvxor" } };

            
            // A wrapper for Z3 contexts
            class Z3CtxWrapper : public RefCountable
            {
            private:
                Z3_context Ctx;

            public:
                Z3CtxWrapper(Z3_context Ctx);
                virtual ~Z3CtxWrapper();

                operator Z3_context ();
                Z3_context GetCtx() const;
            };

            typedef SmartPtr<Z3CtxWrapper> Z3Ctx;

            class Z3Expr
            {
            protected:
                Z3_context Ctx;

            private:
                Z3_ast AST;
            
            public:
                inline Z3Expr();
                inline Z3Expr(const Z3Expr& Other);
                inline Z3Expr(Z3_context Ctx, Z3_ast AST);
                inline Z3Expr(Z3Expr&& Other);
                inline virtual ~Z3Expr();

                inline Z3Expr& operator = (Z3Expr Other);
                inline bool operator == (const Z3Expr& Other);
            
                inline string ToString() const;
                inline u64 Hash() const;


                // unsafe! use only if you know what you're doing
                operator Z3_ast () const;
                inline Z3_ast GetAST() const;
                inline Z3_context GetCtx() const;
            };

            class UFDescriptor
            {
            private:
                const vector<i64> DomainTypes;
                i64 RangeType;
                string Name;

            public:
                UFDescriptor();
                UFDescriptor(const UFDescriptor& Other);
                UFDescriptor(const vector<i64> DomainTypes, 
                             i64 RangeType, const string& Name);

                UFDescriptor& operator = (const UFDescriptor& Other);
                bool operator == (const UFDescriptor& Other);

                const vector<i64>& GetDomainTypes() const;
                i64 GetRangeType() const;
                const string& GetName() const;
            };

            // Typecheck visitor
            template <typename E, template <typename> class S>
            class TypeCheckVisitor : public ExpressionVisitorBase<E, S>
            {
            public:
                TypeCheckVisitor();
                virtual ~TypeCheckVisitor();

                inline virtual void VisitVarExpression(const VarExpression<E, S>* Exp) override;
                inline virtual void VisitConstExpression(const ConstExpression<E, S>* Exp) override;
                inline virtual void VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp) 
                    override;
                inline virtual void VisitOpExpression(const OpExpression<E, S>* Exp) override;
                inline virtual void VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>*
                                                               Exp) override;
                inline virtual void VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>*
                                                               Exp) override;
            };

            static inline string MangleName(const string& Name, const vector<i64>& DomainTypes)
            {
                string Retval = Name;
                for (auto const& DomType : DomainTypes) {
                    Retval += "@" + to_string(DomType);
                }
                return Retval;
            }

            template <typename E, template <typename> class S>
            TypeCheckVisitor<E, S>::TypeCheckVisitor()
                : ExpressionVisitorBase<E, S>("TypeCheckVisitor")
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            TypeCheckVisitor<E, S>::~TypeCheckVisitor()
            {
                // Nothing here
            }

            // Implementation of TypeChecker
            template <typename E, template <typename> class S>
            inline void 
            TypeCheckVisitor<E, S>::VisitVarExpression(const VarExpression<E, S>* Exp)
            {
                auto Type = Exp->GetVarType();
                if (Type != BoolType &&
                    Type != IntType &&
                    Type <= BVTypeOffset &&
                    Type >= BVTypeEnd) {
                    throw ExprTypeError((string)'Invalid type ' + to_string(Type));
                }
                Exp->SetType(Type);
            }

            template <typename E, template <typename> class S>
            inline void 
            TypeCheckVisitor<E, S>::VisitConstExpression(const ConstExpression<E, S> *Exp)
            {
                auto Type = Exp->GetConstType();
                if (Type != BoolType &&
                    Type != IntType &&
                    (Type < BVTypeOffset ||
                     Type >= BVTypeEnd)) {
                    throw ExprTypeError((string)'Invalid type ' + to_string(Type));
                }
                string ValString = Exp->GetConstValue();
                boost::algorithm::to_lower(ValString);
                if (Type == BoolType) {
                    if (ValString != "true" &&
                        ValString != "false") {
                        throw ExprTypeError((string)"Invalid value " + Exp->GetConstValue());
                    }
                    Exp->SetType(BoolType);
                } else if (Type == IntType) {
                    if (!boost::algorithm::all(ValString, boost::algorithm::is_digit())) {
                        throw ExprTypeError((string)"Invalid value " + Exp->GetConstValue());
                    }
                    Exp->SetType(IntType);
                } else {
                    string ActValString = ValString.substr(2, ValString.length() - 2);
                    if (boost::algorithm::starts_with(ValString, "#b")) {
                        u32 NumBits = (ValString.length() - 2);
                        if (!boost::algorithm::all(ActValString, 
                                                   boost::algorithm::is_any_of("01"))) {
                            throw ExprTypeError((string)"Invalid value " + Exp->GetConstValue());
                        }
                        Exp->SetType(BVTypeAll + NumBits);
                    } else if (boost::algorithm::starts_with(ValString, "#x")) {
                        u32 NumBits = (ValString.length() - 2) * 4;
                        if (!boost::algorithm::all(ActValString,
                                                   boost::algorithm::is_xdigit())) {
                            throw ExprTypeError((string)"Invalid value " + Exp->GetConstValue());
                        }
                        Exp->SetType(BVTypeAll + NumBits);
                    }
                }
            }

            template<typename T, typename V>
            static inline bool CheckEquality(const T& Collection, const V& Value)
            {
                for (auto const& Elem : Collection) {
                    if (Elem != Value) {
                        return false;
                    }
                }
                return true;
            }

            static inline i32 CheckBVType(i64 Type)
            {
                if (Type < BVTypeOffset ||
                    Type >= BVTypeEnd) {
                    return -1;
                }
                return Type - BVTypeOffset;
            }

            template <typename E, template <typename> class S>
            inline void 
            TypeCheckVisitor<E, S>::VisitOpExpression(const OpExpression<E, S>* Exp)
            {
                vector<i64> ChildTypes;
                for (auto const& Child : Exp->GetChildren()) {
                    if (Child->GetType() == -1) {
                        Child->Accept(this);
                    }
                    ChildTypes.push_back(Child->GetType());
                }

                switch (Exp->GetOpCode()) {
                case OpEQ:
                    if (ChildTypes.size() != 2) {
                        throw ExprTypeError('= op can only be applied to two operands')
                    }
                    if (!CheckEquality(ChildTypes, ChildTypes[0])) {
                        throw ExprTypeError('All types to = op not the same');
                    }
                    Exp->SetType(BoolType);
                    break;

                case OpNOT:
                    if (ChildTypes.size() != 1) {
                        throw ExprTypeError("not op can only be applied to one operand")
                    }
                    if (ChildTypes[0] != BoolType) {
                        throw ExprTypeError("not op can only be applied to boolean expressions");
                    }
                    Exp->SetType(BoolType);
                    break;

                case OpITE:
                    if (ChildTypes.size() != 3) {
                        throw ExprTypeError("ite op can only be applied to two operands");
                    }
                    if (ChildTypes[0] != BoolType) {
                        throw ExprTypeError("ite op needs a boolean condition");
                    }
                    if (ChildTypes[1] != ChildTypes[2]) {
                        throw ExprTypeError("ite op needs both branches to be same type");
                    }
                    Exp->SetType(ChildTypes[1]);
                    break;

                case OpIMPLIES:
                case OpXOR:
                case OpIFF:
                    if (ChildTypes.size() != 2) {
                        throw ExprTypeError((string)"implies, xor and iff ops can only be " + 
                                            "applied to two operands");
                    }
                case OpOR:
                case OpAND:
                    if (ChildTypes.size() < 2) {
                        throw ExprTypeError((string)"and/or ops need at least two operands");
                    }
                    if (!all_of(ChildTypes.begin(), ChildTypes.end(), 
                                [] (i64 i) { return i == BoolType; })) {
                        throw ExprTypeError((string)"implies, xor, iff, and, or ops expect " + 
                                            "Boolean expressions are operands");
                    }
                    Exp->SetType(BoolType);

                    
                }
            }
            
        } /* end namespace Detail */

        using namespace Detail;

        template <typename E>
        class Z3Semanticizer 
        {
        private:
            unordered_map<string, UFDescriptor> UFMap;
            UIDGenerator UFUIDGen;

        public:
            typedef Z3Expr LExpT;
            typedef Expr<E, ESMC::Exprs::Z3Semanticizer> ExpT;

            Z3Semanticizer();
            ~Z3Semanticizer();

            inline void TypeCheck(const ExpT& Exp) const;
            inline ExpT Canonicalize(const ExpT& Exp);
            inline ExpT RaiseExpr(const LExpT& LExp);
            inline ExpT LowerExpr(const ExpT& Exp);
            inline ExpT Simplify(const ExpT& Exp);
            inline ExpT ElimQuantifiers(const ExpT& Exp);
            inline string ExprToString(const ExpT& Exp) const;
            inline string TypeToString(i64 Type) const;
        };

        // Implementation of Z3Semanticizer
        template <typename E>
        Z3Semanticizer<E>::Z3Semanticizer()
        {
            // Nothing here
        }

        template <typename E>
        Z3Semanticizer<E>::~Z3Semanticizer()
        {
            // Nothing here
        }

        template <typename E>
        inline void Z3Semanticizer::TypeCheck(const ExpT& Exp) const
        {
            
        }

    } /* end namespace Z3Sem */
} /* end namespace ESMC */

#endif /* ESMC_Z3_SEMANTICIZER_HPP_ */


// 
// Z3Semanticizer.hpp ends here
