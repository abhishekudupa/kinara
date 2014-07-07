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
        const i64 UFEnd = 1000000;
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

        // Helper functions
        static inline i64 MakeBVTypeOfSize(u32 Size)
        {
            if (Size <= 0 || Size >= BVTypeEnd - BVTypeOffset) {
                throw ExprTypeError((string)"Bit Vector types must have size in the range 1 - " + 
                                    to_string(BVTypeEnd - BVTypeOffset - 1));
            }
            return BVTypeOffset + Size;
        }

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

            const string BoundVarPrefix = "dbvar";
            
            // A wrapper for ref counting Z3 contexts
            class Z3CtxWrapper : public RefCountable
            {
            private:
                Z3_context Ctx;

            public:
                Z3CtxWrapper(Z3_context Ctx);
                virtual ~Z3CtxWrapper();

                operator Z3_context () const;
                Z3_context GetCtx() const;
            };

            typedef SmartPtr<Z3CtxWrapper> Z3Ctx;

            class Z3Expr
            {
            protected:
                Z3Ctx Ctx;

            private:
                Z3_ast AST;
            
            public:
                Z3Expr();
                Z3Expr(const Z3Expr& Other);
                Z3Expr(Z3Ctx Ctx, Z3_ast AST);
                Z3Expr(Z3Expr&& Other);
                virtual ~Z3Expr();

                Z3Expr& operator = (Z3Expr Other);
                bool operator == (const Z3Expr& Other) const;
            
                string ToString() const;
                u64 Hash() const;


                // unsafe! use only if you know what you're doing
                operator Z3_ast () const;
                Z3_ast GetAST() const;
                const Z3Ctx& GetCtx() const;
            };

            extern ostream& operator << (ostream& Out, const Z3Expr& Exp);

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

            typedef unordered_map<i64, UFDescriptor> UFID2DMapT;
            typedef unordered_map<string, UFDescriptor> UFName2DMapT;

            // Typecheck visitor
            template <typename E, template <typename> class S>
            class TypeCheckVisitor : public ExpressionVisitorBase<E, S>
            {
            private:
                const UFID2DMapT& UFMap;
                vector<vector<i64>> ScopeStack;

                inline void VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp);

            public:
                TypeCheckVisitor(const UFID2DMapT& UFMap);
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

            // Type invalidator: Invalidates any previously 
            // computed type tags on an expression
            template <typename E, template <typename> class S>
            class TypeInvalidator : public ExpressionVisitorBase<E, S>
            {
            public:
                TypeInvalidator() 
                    : ExpressionVisitorBase<E, S>("TypeInvalidator") {}
                virtual ~TypeInvalidator() {}
                inline virtual void VisitVarExpression(const VarExpression<E, S>* Exp) override
                {
                    Exp->SetType(-1);
                }

                inline virtual void VisitConstExpression(const ConstExpression<E, S>* Exp) override
                {
                    Exp->SetType(-1);
                }
                
                
                inline virtual void VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp) override
                {
                    Exp->SetType(-1);
                }
                
                inline virtual void VisitOpExpression(const OpExpression<E, S>* Exp) override
                {
                    ExpressionVisitorBase<E, S>::VisitOpExpression(Exp);
                    Exp->SetType(-1);
                }

                inline void VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp)
                {
                    Exp->GetQExpression()->Accept(this);
                }

                inline virtual void VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp) override
                {
                    VisitQuantifiedExpression(Exp);
                }

                inline virtual void VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp) override
                {
                    VisitQuantifiedExpression(Exp);
                }
            };

            template <typename E, template <typename> class S>
            class Canonicalizer : public ExpressionVisitorBase<E, S>
            {
            private:
                typedef Expr<E, S> ExpT;
                vector<ExpT> ExpStack;

                inline void VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp);

            public:
                Canonicalizer();
                virtual ~Canonicalizer();
                
                inline virtual void VisitVarExpression(const VarExpression<E, S>* Exp) override;
                inline virtual void VisitConstExpression(const ConstExpression<E, S>* Exp) override;
                inline virtual void VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp) 
                    override;
                inline virtual void VisitOpExpression(const OpExpression<E, S>* Exp) override;
                inline virtual void VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>*
                                                               Exp) override;
                inline virtual void VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>*
                                                               Exp) override;
                
                static inline ExpT Do(const ExpT& Exp);
            };

            template <typename E, template <typename> class S>
            class Stringifier : public ExpressionVisitorBase<E, S>
            {
            private:
                vector<string> StringStack;

                inline void VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp);

            public:
                Stringifier()
                virtual ~Stringifier();

                inline virtual void VisitVarExpression(const VarExpression<E, S>* Exp) override;
                inline virtual void VisitConstExpression(const ConstExpression<E, S>* Exp) override;
                inline virtual void VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp) 
                    override;
                inline virtual void VisitOpExpression(const OpExpression<E, S>* Exp) override;
                inline virtual void VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>*
                                                               Exp) override;
                inline virtual void VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>*
                                                               Exp) override;
                
                static inline string Do(const ExpT& Exp);
            };

            static inline string MangleName(const string& Name, const vector<i64>& DomainTypes)
            {
                string Retval = Name;
                for (auto const& DomType : DomainTypes) {
                    Retval += "@" + to_string(DomType);
                }
                return Retval;
            }

            static inline i64 GetTypeForBoundVar(const vector<vector<i64>>& ScopeStack,
                                                 i64 VarIdx)
            {
                i64 LeftIdx = VarIdx;
                for (i64 i = ScopeStack.size(); i > 0; --i) {
                    if (LeftIdx < (i64)ScopeStack[i].size()) {
                        return ScopeStack[i][ScopeStack[i].size() - 1 - LeftIdx];
                    } else {
                        LeftIdx -= ScopeStack[i].size();
                    }
                }
                // Unbound variable
                return -1;
            }

            static inline void CheckType(i64 Type)
            {
                if (Type != BoolType &&
                    Type != IntType &&
                    (Type < BVTypeOffset ||
                     Type >= BVTypeEnd)) {
                    throw ExprTypeError((string)"Unknown type " + to_string(Type));
                }
            }

            template <typename E, template <typename> class S>
            TypeCheckVisitor<E, S>::TypeCheckVisitor(const UFID2DMapT& UFMap)
                : ExpressionVisitorBase<E, S>("Z3TypeChecker"),
                  UFMap(UFMap)
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
                auto PrevType = Exp->GetType();
                if (PrevType != -1) {
                    return PrevType;
                }
                auto Type = Exp->GetVarType();
                CheckType(Type);
                Exp->SetType(Type);
            }

            template <typename E, template <typename> class S>
            inline void 
            TypeCheckVisitor<E, S>::VisitConstExpression(const ConstExpression<E, S> *Exp)
            {
                auto PrevType = Exp->GetType();
                if (PrevType != -1) {
                    return PrevType;
                }
                auto Type = Exp->GetConstType();
                CheckType(Type);
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

            template <typename E, template <typename> class S>
            inline void 
            TypeCheckVisitor<E, S>::VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp)
           {
                auto PrevType = Exp->GetType();
                if (PrevType != -1) {
                    return PrevType;
                }

                auto Type = Exp->GetVarType();
                CheckType(Type);
                auto Idx = Exp->GetVarIdx();
                auto ExpectedType = GetTypeForBoundVar(ScopeStack, Idx);
                if (ExpectedType != -1 && ExpectedType != Type) {
                    throw ExprTypeError((string)"Bound variable with index " + to_string(Idx) + 
                                        "has an ambiguous type");
                }

                Exp->SetType(Type);
            }

            template <typename T, typename V>
            static inline bool CheckEquality(const T& Collection, const V& Value)
            {
                return (all_of(Collection.begin(), Collection.end(),
                               [&] (const V& i) -> bool { return i == Value; }));
            }

            static inline i32 CheckBVType(i64 Type)
            {
                if (Type < BVTypeOffset ||
                    Type >= BVTypeEnd) {
                    return -1;
                }
                return Type - BVTypeOffset;
            }

            static inline bool CheckAllInt(const vector<i64>& TypeVec)
            {
                return (all_of(TypeVec.begin(), TypeVec.end(),
                               [&] (i64 i) -> bool { return i == IntType; }));
            }

            static inline bool CheckTypeResolution(i64 ExpectedType, i64 ActualType)
            {
                switch(ExpectedType) {
                case BoolType:
                case IntType:
                    return (ActualType == ExpectedType);
                default:
                    if (ExpectedType == BVTypeAll) {
                        return (CheckBVType(ActualType) != -1);
                    } else if (ExpectedType >= BVTypeEnd) {
                        throw InternalError("Strange type that should have been caught earlier");
                    } else {
                        return (ActualType == ExpectedType);
                    }
                }
            }

            template <typename E, template <typename> class S>
            inline void 
            TypeCheckVisitor<E, S>::VisitOpExpression(const OpExpression<E, S>* Exp)
            {
                auto PrevType = Exp->GetType();
                if (PrevType != -1) {
                    return PrevType;
                }

                i64 Type = -1;

                vector<i64> ChildTypes;
                for (auto const& Child : Exp->GetChildren()) {
                    if (Child->GetType() == -1) {
                        Child->Accept(this);
                    }
                    ChildTypes.push_back(Child->GetType());
                }

                i64 OpCode = Exp->GetOpCode();

                switch (OpCode) {
                case OpEQ:
                    if (ChildTypes.size() != 2) {
                        throw ExprTypeError("= op can only be applied to two operands");
                    }
                    if (!CheckEquality(ChildTypes, ChildTypes[0])) {
                        throw ExprTypeError("All types to = op not the same");
                    }
                    Exp->SetType(BoolType);
                    break;

                case OpNOT:
                    if (ChildTypes.size() != 1) {
                        throw ExprTypeError("not op can only be applied to one operand");
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
                    break;

                case OpADD:
                case OpSUB:
                case OpMUL:
                    if (ChildTypes.size() < 2) {
                        throw ExprTypeError ((string)"add/sub/mul ops need at least two operands");
                    }
                    if (!all_of(ChildTypes.begin(), ChildTypes.end(),
                                [] (i64 i) { return i == IntType; })) {
                        throw ExprTypeError((string)"add/sub/mul ops expect Integer operands");
                    }
                    Exp->SetType(IntType);
                    break;

                case OpDIV:
                case OpREM:
                case OpMOD:
                case OpPOWER:
                    if (ChildTypes.size() != 2) {
                        throw ExprTypeError ((string)"div/rem/mod/pow ops need exactly two operands");
                    }
                    if (!CheckAllInt(ChildTypes)) {
                        throw ExprTypeError((string)"div/rem/mod/pow ops expect Integer operands");
                    }
                    Exp->SetType(IntType);
                    break;

                case OpMINUS:
                    if (ChildTypes.size() != 1) {
                        throw ExprTypeError ((string)"unary minus op needs exactly one operand");
                    }
                    if (ChildTypes[0] != IntType) {
                        throw ExprTypeError ((string)"unary minus op expects an Integer operand");
                    }
                    Exp->SetType(IntType);
                    break;

                case OpGT:
                case OpLT:
                case OpGE:
                case OpLE:
                    if (ChildTypes.size() != 2) {
                        throw ExprTypeError ((string)"GT/LT/GE/LE ops need exactly two operands");
                    }
                    if (!CheckAllInt(ChildTypes)) {
                        throw ExprTypeError ((string)"GT/LT/GE/LE ops expect Integer operands");
                    }
                    Exp->SetType(BoolType);
                    break;

                // BVOps
                case OpBVNOT:
                    if (ChildTypes.size() != 1) {
                        throw ExprTypeError ("BVNOT op needs exactly one operand");
                    }
                    Type = CheckBVType(ChildTypes[0]);
                    if (Type == -1) {
                        throw ExprTypeError ("BVNOT op expects a BV operand");
                    }
                    Exp->SetType(Type);
                    break;

                case OpBVREDOR:
                case OpBVREDAND:
                    if (ChildTypes.size() != 1) {
                        throw ExprTypeError ("BVREDOR/BVREDAND ops need exactly one operand");
                    }
                    Type = CheckBVType(ChildTypes[0]);
                    if (Type == -1) {
                        throw ExprTypeError ("BVREDOR/BVREDAND ops expect a BV operand");
                    }
                    Exp->SetType(BVTypeAll + 1);
                    break;
                    
                case OpBVAND:
                case OpBVOR:
                case OpBVXOR:
                    if (ChildTypes.size() != 2) {
                        throw ExprTypeError ("BVAND/BVOR/BVXOR ops need exactly two operands");
                    }
                    if (!all_of(ChildTypes.begin(), ChildTypes.end(),
                                [](i64 i) { return (CheckBVType(i) != -1); })) {
                        throw ExprTypeError ("BVAND/BVOR/BVXOR ops expect BV operands");
                    }
                    if (!CheckEquality(ChildTypes, ChildTypes[0])) {
                        throw ExprTypeError ("BVAND/BVOR/BVXOR ops expect BV operands of same size");
                    }
                    Exp->SetType(ChildTypes[0]);
                    break;

                    // This must now be an uninterpreted function
                default:
                    if (OpCode < UFOffset || OpCode >= UFEnd) {
                        throw ExprTypeError ((string)"Unknown op " + to_string(OpCode));
                    }
                    auto it = UFMap.find(OpCode);
                    if (it == UFMap.end()) {
                        throw ExprTypeError ((string)"Unknown op " + to_string(OpCode));
                    }
                    auto const& UFDesc = it->second;
                    auto const& DomType = UFDesc.GetDomainTypes();
                    if (DomType.size() != ChildTypes.size()) {
                        throw ExprTypeError ((string)"UF \"" + UFDesc.GetName() + "\" with opcode " + 
                                             to_string(OpCode) + 
                                             " expects " + to_string(DomType.size()) + " operands" + 
                                             " but applied to " + to_string(ChildTypes.size()) + 
                                             " operands");
                    }
                    for (u32 i = 0; i < DomType.size(); ++i) {
                        if (!CheckTypeResolution(DomType[i], ChildTypes[i])) {
                            throw ExprTypeError((string)"Invalid operands for UF \"" + UFDesc.GetName() + 
                                                "\" with opcode " + to_string(OpCode));
                        }
                    }
                    Exp->SetType(UFDesc.GetRangeType());
                    break;
                }
            }

            template <typename E, template <typename> class S>
            inline void 
            TypeCheckVisitor<E, S>::VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp)
            {
                auto const& QVarTypes = Exp->GetQVarTypes();
                for (auto const& Type : QVarTypes) {
                    CheckType(Type);
                }

                ScopeStack.push_back(QVarTypes);
                auto const& QExpr = Exp->GetQExpression();
                // Invalidate the types in the quantified expression first
                TypeInvalidator<E, S> Inv;
                QExpr->Accept(&Inv);
                QExpr->Accept(this);

                if (QExpr->GetType() != BoolType) {
                    throw ExprTypeError((string)"The body of a quantified expression must be " + 
                                        "a boolean valued expression");
                }
                ScopeStack.pop_back();
                return;
            }

            template <typename E, template <typename> class S>
            inline void 
            TypeCheckVisitor<E, S>::VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp)
            {
                VisitQuantifiedExpression(Exp);
            }

            template <typename E, template <typename> class S>
            inline void 
            TypeCheckVisitor<E, S>::VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp)
            {
                VisitQuantifiedExpression(Exp);
            }

            // Canonicalizer Implementation
            template <typename E, template <typename> class S>
            Canonicalizer<E, S>::Canonicalizer()
                : ExpressionVisitorBase<E, S>("Z3Canonicalizer")
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            Canonicalizer<E, S>::~Canonicalizer()
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            inline void Canonicalizer<E, S>::VisitVarExpression(const VarExpression<E, S>* Exp)
            {
                ExpStack.push_back(Exp);
            }

            template <typename E, template <typename> class S>
            inline void Canonicalizer<E, S>::VisitConstExpression(const ConstExpression<E, S>* Exp)
            {
                ExpStack.push_back(Exp);
            }

            template <typename E, template <typename> class S>
            inline void Canonicalizer<E, S>::VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp)
            {
                ExpressionVisitorBase<E, S>::VisitBoundVarExpression(Exp);
                ExpStack.push_back(Exp);
            }

            template <typename E, template <typename> class S>
            inline void Canonicalizer<E, S>::VisitOpExpression(const OpExpression<E, S>* Exp)
            {
                auto const& OpCode = Exp->GetOpCode();
                auto const& NumChildren = Exp->GetChildren().size();
                ExpressionVisitorBase<E, S>::VisitOpExpression(Exp);
                
                vector<ExpT> NewChildren(NumChildren);
                for (u32 i = 0; i < NumChildren; ++i) {
                    NewChildren[NumChildren - i - 1] = ExpStack.back();
                    ExpStack.pop_back();
                }
                
                switch(OpCode) {
                    // Sort operands for commutative operands
                case OpEQ:
                case OpOR:
                case OpAND:
                case OpIFF:
                case OpXOR:
                case OpADD:
                case OpMUL:
                case OpBVAND:
                case OpBVOR:
                case OpBVXOR:
                    sort(NewChildren.begin(), NewChildren.end(), ExpressionPtrCompare());
                    ExpStack.push_back(new OpExpression<E, S>(nullptr, OpCode, 
                                                              NewChildren, Exp->ExtensionData));
                    break;

                case OpNOT:
                    if (NewChildren[0]->template As<OpExpression>() != nullptr &&
                        NewChildren[0]->template SAs<OpExpression>()->GetOpCode == OpNOT) {
                        ExpStack.push_back((NewChildren[0]->GetChildren())[0]);
                    } else {
                        ExpStack.push_back(new OpExpression<E, S>(nullptr, OpCode,
                                                                  NewChildren, Exp->ExtensionData));
                    }
                    break;

                default:
                    ExpStack.push_back(new OpExpression<E, S>(nullptr, OpCode, NewChildren,
                                                              Exp->ExtensionData));
                    break;
                }
            }

            template <typename E, template <typename> class S>
            inline void Canonicalizer<E, S>::VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp)
            {
                ExpressionVisitorBase<E, S>::VisitEQuantifiedExpression(Exp);
                auto NewQExpr = ExpStack.back();
                ExpStack.pop_back();
                ExpStack.push_back(new EQuantifiedExpression<E, S>(nullptr, Exp->GetVarTypes(),
                                                                   NewQExpr, Exp->ExtensionData));
            }

            template <typename E, template <typename> class S>
            inline void Canonicalizer<E, S>::VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp)
            {
                ExpressionVisitorBase<E, S>::VisitAQuantifiedExpression(Exp);
                auto NewQExpr = ExpStack.back();
                ExpStack.pop_back();
                ExpStack.push_back(new AQuantifiedExpression<E, S>(nullptr, Exp->GetVarTypes(),
                                                                   NewQExpr, Exp->ExtensionData));
            }

            template <typename E, template <typename> class S>
            inline typename Canonicalizer<E, S>::ExpT
            Canonicalizer<E, S>::Do(const ExpT& Exp)
            {
                Canonicalizer TheCanonicalizer;
                Exp->Accept(&TheCanonicalizer);
                assert(TheCanonicalizer.ExpStack.size() == 1);
                return (TheCanonicalizer.ExpStack[0]);
            }

            
            // Stringifier implementation
            template <typename E, template <typename> class S>
            Stringifier<E, S>::Stringifier()
                : ExpressionVisitorBase("Z3Stringifier")
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            Stringifier<E, S>::~Stringifier()
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            inline void Stringifier<E, S>::VisitVarExpression(const VarExpression<E, S>* Exp)
            {
                ExpressionVisitorBase<E, S>::VisitVarExpression(Exp);
                StringStack.push_back(Exp->GetVarName());
            }

            template <typename E, template <typename> class S>
            inline void Stringifier<E, S>::VisitConstExpression(const ConstExpression<E, S>* Exp)
            {
                ExpressionVisitorBase<E, S>::VisitConstExpression(Exp);
                StringStack.push_back(Exp->GetConstValue());
            }

            template <typename E, template <typename> class S>
            inline void Stringifier<E, S>::VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp)
            {
                ExpressionVisitorBase<E, S>::VisitBoundVarExpression(Exp);
                StringStack.push_back(BoundVarPrefix + "@" + to_string(Exp->GetVarIdx()));
            }

            template <typename E, template <typename> class S>
            inline void Stringifier<E, S>::VisitOpExpression(const OpExpression<E, S>* Exp)
            {
                // TODO: Implement me
            }
            
        } /* end namespace Detail */

        using namespace Detail;

        template <typename E>
        class Z3Semanticizer 
        {
        private:
            Detail::UFID2DMapT UFID2DMap;
            Detail::UFName2DMapT UFName2DMap;
            UIDGenerator UFUIDGen;

        public:
            typedef Z3Expr LExpT;
            typedef Expr<E, ESMC::Exprs::Z3Semanticizer> ExpT;

            Z3Semanticizer();
            ~Z3Semanticizer();

            inline void TypeCheck(const ExpT& Exp) const;
            inline ExpT Canonicalize(const ExpT& Exp);
            inline ExpT RaiseExpr(const LExpT& LExp);
            inline LExpT LowerExpr(const ExpT& Exp);
            inline ExpT Simplify(const ExpT& Exp);
            inline ExpT ElimQuantifiers(const ExpT& Exp);
            inline string ExprToString(const ExpT& Exp) const;
            inline string TypeToString(i64 Type) const;

            inline i64 RegisterUninterpretedFunction(const string& Name,
                                                     const vector<i64>& DomTypes,
                                                     i64 RangeType);
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
        inline void Z3Semanticizer<E>::TypeCheck(const ExpT& Exp) const
        {
            TypeCheckVisitor<E, ESMC::Z3Sem::Z3Semanticizer> Checker(UFID2DMap);
            Exp->Accept(&Checker);
            return;
        }

        template <typename E>
        inline typename Z3Semanticizer<E>::ExpT
        Z3Semanticizer<E>::Canonicalize(const ExpT& Exp)
        {
            return Canonicalizer<E, ESMC::Z3Sem::Z3Semanticizer>::Do(Exp);
        }

        template <typename E>
        inline typename Z3Semanticizer<E>::ExpT 
        Z3Semanticizer<E>::RaiseExpr(const LExpT& LExp)
        {
            // TODO: Implement me
            return ExpT::NullPtr;
        }

        template <typename E>
        inline typename Z3Semanticizer<E>::LExpT
        Z3Semanticizer<E>::LowerExpr(const ExpT& Exp)
        {
            // TODO: Implement me
            return Z3Expr();
        }

        template <typename E>
        inline typename Z3Semanticizer<E>::ExpT
        Z3Semanticizer<E>::Simplify(const ExpT& Exp)
        {
            // TODO: Implement me
            return ExpT::NullPtr;
        }

        template <typename E>
        inline typename Z3Semanticizer<E>::ExpT
        Z3Semanticizer<E>::ElimQuantifiers(const ExpT& Exp)
        {
            // TODO: Implement me
            return ExpT::NullPtr;
        }

        template <typename E>
        inline string Z3Semanticizer<E>::ExprToString(const ExpT& Exp) const
        {
            // TODO: Implement me
            return "";
        }

        template <typename E>
        inline string Z3Semanticizer<E>::TypeToString(i64 Type) const
        {
            // TODO: Implement me
            return "";
        }

        template <typename E>
        inline i64 Z3Semanticizer<E>::RegisterUninterpretedFunction(const string& Name, 
                                                                    const vector<i64> &DomTypes, 
                                                                    i64 RangeType)
        {
            // TODO: Implement me
            return 0;
        }

    } /* end namespace Z3Sem */
} /* end namespace ESMC */

#endif /* ESMC_Z3_SEMANTICIZER_HPP_ */


// 
// Z3Semanticizer.hpp ends here
