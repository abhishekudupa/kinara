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
#include <boost/multiprecision/cpp_int.hpp>
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

        // BVTypes range from 1000100 to 1001000
        // 1000100 = ANY BV type
        // 100xxxx = BVType of len (1000100 - v)
        const i64 BVTypeOffset = 1000100;
        const i64 BVTypeEnd = 1001000;


        // We put the ops in a separate struct
        // for convenience
        struct Z3SemOps {
            static const i64 OpEQ = 1000;
            static const i64 OpNOT = 1001;
            static const i64 OpITE = 1002;
            static const i64 OpOR = 1003;
            static const i64 OpAND = 1004;
            static const i64 OpIMPLIES = 1005;
            static const i64 OpIFF = 1006;
            static const i64 OpXOR = 1007;

            // Arithmetic operators
            static const i64 OpADD = 1008;
            static const i64 OpSUB = 1009;
            static const i64 OpMINUS = 1010;
            static const i64 OpMUL = 1011;
            static const i64 OpDIV = 1012;
            static const i64 OpMOD = 1013;
            static const i64 OpREM = 1014;
            static const i64 OpPOWER = 1015;
            static const i64 OpGT = 1016;
            static const i64 OpGE = 1017;
            static const i64 OpLT = 1018;
            static const i64 OpLE = 1019;

            // Basic Bitvector operators
            // used for emulating bounded sets
            static const i64 OpBVNOT = 1020;
            static const i64 OpBVREDAND = 1021;
            static const i64 OpBVREDOR = 1022;
            static const i64 OpBVAND = 1023;
            static const i64 OpBVOR = 1024;
            static const i64 OpBVXOR = 1025;

            // Types
            static const i64 BoolType = 1000000;
            static const i64 IntType = 1000001;
            static const i64 BVTypeAll = 1000100;
        };

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
            extern const unordered_map<i64, string> OpCodeToNameMap;
            extern const string BoundVarPrefix;
            
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
                i64 Identifier;
                vector<i64> DomainTypes;
                i64 RangeType;
                string Name;
                string MangledName;

            public:
                UFDescriptor();
                UFDescriptor(const UFDescriptor& Other);
                UFDescriptor(i64 Identifier,
                             const vector<i64>& DomainTypes, 
                             i64 RangeType, const string& Name);
                ~UFDescriptor();

                UFDescriptor& operator = (const UFDescriptor& Other);
                bool operator == (const UFDescriptor& Other);

                const vector<i64>& GetDomainTypes() const;
                i64 GetRangeType() const;
                const string& GetName() const;
                const string& GetMangledName() const;
                i64 GetIdentifier() const;
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
                
                
                inline virtual void 
                VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp) override
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

                inline virtual void 
                VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp) override
                {
                    VisitQuantifiedExpression(Exp);
                }

                inline virtual void 
                VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp) override
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
                typedef Expr<E, S> ExpT;
                const UFID2DMapT& UFMap;

                inline void VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp);

            public:
                Stringifier(const UFID2DMapT& UFMap);
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
                
                static inline string Do(const ExpT& Exp, const UFID2DMapT& UFMap);
                static inline string TypeToString(i64 Type);
            };

            template <typename E, template <typename> class S>
            class Lowerer : public ExpressionVisitorBase<E, S>
            {
            private:
                typedef Expr<E, S> ExpT;
                typedef Z3Expr LExpT;
                const UFID2DMapT& UFMap;
                Z3Ctx Ctx;
                vector<LExpT> ExpStack;

                inline void VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp);
                inline Z3_sort LowerType(i64 Type);

            public:
                Lowerer(const UFID2DMapT& UFMap, const Z3Ctx& Ctx);
                virtual ~Lowerer();

                inline virtual void VisitVarExpression(const VarExpression<E, S>* Exp) override;
                inline virtual void VisitConstExpression(const ConstExpression<E, S>* Exp) override;
                inline virtual void VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp) 
                    override;
                inline virtual void VisitOpExpression(const OpExpression<E, S>* Exp) override;
                inline virtual void VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>*
                                                               Exp) override;
                inline virtual void VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>*
                                                               Exp) override;
                
                static inline LExpT Do(const ExpT& Exp, const UFID2DMapT& UFMap, const Z3Ctx& Ctx);
            };

            template <typename E, template <typename> class S>
            class Raiser
            {
            private:
                typedef Expr<E, S> ExpT;
                typedef Z3Expr LExpT;
                const UFName2DMapT& UFMap;
                Z3Ctx Ctx;

                inline Raiser(const UFName2DMapT& UFMap);

                inline ExpT RaiseApp(const LExpT& LExp);
                inline ExpT RaiseNum(const LExpT& LExp);
                inline ExpT RaiseVar(const LExpT& LExp);
                inline ExpT RaiseQuant(const LExpT& LExp);
                inline ExpT Raise(const LExpT& LExp);
                inline i64 RaiseSort(Z3_sort Sort);

            public:
                inline ~Raiser();
                static inline ExpT Do(const LExpT& LExp, const UFName2DMapT& UFMap);
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
                    auto const& CurScope = ScopeStack[i - 1];
                    const u32 CurScopeSize = CurScope.size();
                    if (LeftIdx < (i64) CurScopeSize) {
                        return CurScope[CurScopeSize - 1 - LeftIdx];
                    } else {
                        LeftIdx -= CurScopeSize;
                    }
                }
                // Unbound variable
                return -1;
            }

            static inline void CheckType(i64 Type)
            {
                if (Type != Z3SemOps::BoolType &&
                    Type != Z3SemOps::IntType &&
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
                    return;
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
                    return;
                }
                auto Type = Exp->GetConstType();
                CheckType(Type);
                string ValString = Exp->GetConstValue();
                boost::algorithm::to_lower(ValString);
                if (Type == Z3SemOps::BoolType) {
                    if (ValString != "true" &&
                        ValString != "false") {
                        throw ExprTypeError((string)"Invalid value " + Exp->GetConstValue());
                    }
                    Exp->SetType(Z3SemOps::BoolType);
                } else if (Type == Z3SemOps::IntType) {
                    if (!boost::algorithm::all(ValString, boost::algorithm::is_digit())) {
                        throw ExprTypeError((string)"Invalid value " + Exp->GetConstValue());
                    }
                    Exp->SetType(Z3SemOps::IntType);
                } else {
                    string ActValString = ValString.substr(2, ValString.length() - 2);
                    if (boost::algorithm::starts_with(ValString, "#b")) {
                        u32 NumBits = (ValString.length() - 2);
                        if (!boost::algorithm::all(ActValString, 
                                                   boost::algorithm::is_any_of("01"))) {
                            throw ExprTypeError((string)"Invalid value " + Exp->GetConstValue());
                        }
                        Exp->SetType(Z3SemOps::BVTypeAll + NumBits);
                    } else if (boost::algorithm::starts_with(ValString, "#x")) {
                        u32 NumBits = (ValString.length() - 2) * 4;
                        if (!boost::algorithm::all(ActValString,
                                                   boost::algorithm::is_xdigit())) {
                            throw ExprTypeError((string)"Invalid value " + Exp->GetConstValue());
                        }
                        Exp->SetType(Z3SemOps::BVTypeAll + NumBits);
                    }
                }
            }

            template <typename E, template <typename> class S>
            inline void 
            TypeCheckVisitor<E, S>::VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp)
           {
                auto PrevType = Exp->GetType();
                if (PrevType != -1) {
                    return;
                }

                auto Type = Exp->GetVarType();
                CheckType(Type);
                auto Idx = Exp->GetVarIdx();
                auto ExpectedType = GetTypeForBoundVar(ScopeStack, Idx);
                if (ExpectedType != -1 && ExpectedType != Type) {
                    throw ExprTypeError((string)"Bound variable with index " + to_string(Idx) + 
                                        " has an ambiguous type");
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
                               [&] (i64 i) -> bool { return i == Z3SemOps::IntType; }));
            }

            static inline bool CheckTypeResolution(i64 ExpectedType, i64 ActualType)
            {
                switch(ExpectedType) {
                case Z3SemOps::BoolType:
                case Z3SemOps::IntType:
                    return (ActualType == ExpectedType);
                default:
                    if (ExpectedType == Z3SemOps::BVTypeAll) {
                        return (CheckBVType(ActualType) != -1);
                    } else if (ExpectedType >= BVTypeEnd) {
                        throw InternalError((string)"Strange type that should have been " + 
                                            "caught earlier: " + __FILE__ + ":" + 
                                            to_string(__LINE__));
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
                    return;
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
                case Z3SemOps::OpEQ:
                    if (ChildTypes.size() != 2) {
                        throw ExprTypeError("= op can only be applied to two operands");
                    }
                    if (!CheckEquality(ChildTypes, ChildTypes[0])) {
                        throw ExprTypeError("All types to = op not the same");
                    }
                    Exp->SetType(Z3SemOps::BoolType);
                    break;

                case Z3SemOps::OpNOT:
                    if (ChildTypes.size() != 1) {
                        throw ExprTypeError("not op can only be applied to one operand");
                    }
                    if (ChildTypes[0] != Z3SemOps::BoolType) {
                        throw ExprTypeError("not op can only be applied to boolean expressions");
                    }
                    Exp->SetType(Z3SemOps::BoolType);
                    break;

                case Z3SemOps::OpITE:
                    if (ChildTypes.size() != 3) {
                        throw ExprTypeError("ite op can only be applied to two operands");
                    }
                    if (ChildTypes[0] != Z3SemOps::BoolType) {
                        throw ExprTypeError("ite op needs a boolean condition");
                    }
                    if (ChildTypes[1] != ChildTypes[2]) {
                        throw ExprTypeError("ite op needs both branches to be same type");
                    }
                    Exp->SetType(ChildTypes[1]);
                    break;

                case Z3SemOps::OpIMPLIES:
                case Z3SemOps::OpXOR:
                case Z3SemOps::OpIFF:
                    if (ChildTypes.size() != 2) {
                        throw ExprTypeError((string)"implies, xor and iff ops can only be " + 
                                            "applied to two operands");
                    }
                case Z3SemOps::OpOR:
                case Z3SemOps::OpAND:
                    if (ChildTypes.size() < 2) {
                        throw ExprTypeError((string)"and/or ops need at least two operands");
                    }
                    if (!all_of(ChildTypes.begin(), ChildTypes.end(), 
                                [] (i64 i) { return i == Z3SemOps::BoolType; })) {
                        throw ExprTypeError((string)"implies, xor, iff, and, or ops expect " + 
                                            "Boolean expressions are operands");
                    }
                    Exp->SetType(Z3SemOps::BoolType);
                    break;

                case Z3SemOps::OpADD:
                case Z3SemOps::OpSUB:
                case Z3SemOps::OpMUL:
                    if (ChildTypes.size() < 2) {
                        throw ExprTypeError ((string)"add/sub/mul ops need at least two operands");
                    }
                    if (!all_of(ChildTypes.begin(), ChildTypes.end(),
                                [] (i64 i) { return i == Z3SemOps::IntType; })) {
                        throw ExprTypeError((string)"add/sub/mul ops expect Integer operands");
                    }
                    Exp->SetType(Z3SemOps::IntType);
                    break;

                case Z3SemOps::OpDIV:
                case Z3SemOps::OpREM:
                case Z3SemOps::OpMOD:
                case Z3SemOps::OpPOWER:
                    if (ChildTypes.size() != 2) {
                        throw ExprTypeError ((string)"div/rem/mod/pow ops need " + 
                                             "exactly two operands");
                    }
                    if (!CheckAllInt(ChildTypes)) {
                        throw ExprTypeError((string)"div/rem/mod/pow ops expect Integer operands");
                    }
                    Exp->SetType(Z3SemOps::IntType);
                    break;

                case Z3SemOps::OpMINUS:
                    if (ChildTypes.size() != 1) {
                        throw ExprTypeError ((string)"unary minus op needs exactly one operand");
                    }
                    if (ChildTypes[0] != Z3SemOps::IntType) {
                        throw ExprTypeError ((string)"unary minus op expects an Integer operand");
                    }
                    Exp->SetType(Z3SemOps::IntType);
                    break;

                case Z3SemOps::OpGT:
                case Z3SemOps::OpLT:
                case Z3SemOps::OpGE:
                case Z3SemOps::OpLE:
                    if (ChildTypes.size() != 2) {
                        throw ExprTypeError ((string)"GT/LT/GE/LE ops need exactly two operands");
                    }
                    if (!CheckAllInt(ChildTypes)) {
                        throw ExprTypeError ((string)"GT/LT/GE/LE ops expect Integer operands");
                    }
                    Exp->SetType(Z3SemOps::BoolType);
                    break;

                // BVOps
                case Z3SemOps::OpBVNOT:
                    if (ChildTypes.size() != 1) {
                        throw ExprTypeError ("BVNOT op needs exactly one operand");
                    }
                    Type = CheckBVType(ChildTypes[0]);
                    if (Type == -1) {
                        throw ExprTypeError ("BVNOT op expects a BV operand");
                    }
                    Exp->SetType(Type);
                    break;

                case Z3SemOps::OpBVREDOR:
                case Z3SemOps::OpBVREDAND:
                    if (ChildTypes.size() != 1) {
                        throw ExprTypeError ("BVREDOR/BVREDAND ops need exactly one operand");
                    }
                    Type = CheckBVType(ChildTypes[0]);
                    if (Type == -1) {
                        throw ExprTypeError ("BVREDOR/BVREDAND ops expect a BV operand");
                    }
                    Exp->SetType(Z3SemOps::BVTypeAll + 1);
                    break;
                    
                case Z3SemOps::OpBVAND:
                case Z3SemOps::OpBVOR:
                case Z3SemOps::OpBVXOR:
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
                        throw ExprTypeError ((string)"UF \"" + UFDesc.GetName() + 
                                             "\" with opcode " + 
                                             to_string(OpCode) + 
                                             " expects " + to_string(DomType.size()) + " operands" + 
                                             " but applied to " + to_string(ChildTypes.size()) + 
                                             " operands");
                    }
                    for (u32 i = 0; i < DomType.size(); ++i) {
                        if (!CheckTypeResolution(DomType[i], ChildTypes[i])) {
                            throw ExprTypeError((string)"Invalid operands for UF \"" + 
                                                UFDesc.GetName() + 
                                                "\" with opcode " + to_string(OpCode));
                        }
                    }
                    Exp->SetType(UFDesc.GetRangeType());
                    break;
                }
            }

            template <typename E, template <typename> class S>
            inline void 
            TypeCheckVisitor<E, S>::VisitQuantifiedExpression(const 
                                                              QuantifiedExpressionBase<E, S>* Exp)
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

                if (QExpr->GetType() != Z3SemOps::BoolType) {
                    throw ExprTypeError((string)"The body of a quantified expression must be " + 
                                        "a boolean valued expression");
                }
                ScopeStack.pop_back();
                return;
            }

            template <typename E, template <typename> class S>
            inline void 
            TypeCheckVisitor<E, S>::VisitEQuantifiedExpression(const 
                                                               EQuantifiedExpression<E, S>* Exp)
            {
                VisitQuantifiedExpression(Exp);
            }

            template <typename E, template <typename> class S>
            inline void 
            TypeCheckVisitor<E, S>::VisitAQuantifiedExpression(const 
                                                               AQuantifiedExpression<E, S>* Exp)
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
            inline void Canonicalizer<E, S>::VisitBoundVarExpression(const 
                                                                     BoundVarExpression<E, S>* Exp)
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
                case Z3SemOps::OpEQ:
                case Z3SemOps::OpOR:
                case Z3SemOps::OpAND:
                case Z3SemOps::OpIFF:
                case Z3SemOps::OpXOR:
                case Z3SemOps::OpADD:
                case Z3SemOps::OpMUL:
                case Z3SemOps::OpBVAND:
                case Z3SemOps::OpBVOR:
                case Z3SemOps::OpBVXOR:
                    sort(NewChildren.begin(), NewChildren.end(), ExpressionPtrCompare());
                    ExpStack.push_back(new OpExpression<E, S>(nullptr, OpCode, 
                                                              NewChildren, Exp->ExtensionData));
                    break;

                case Z3SemOps::OpNOT:
                    if (NewChildren[0]->template As<OpExpression>() != nullptr &&
                        NewChildren[0]->template SAs<OpExpression>()->GetOpCode() == 
                        Z3SemOps::OpNOT) {
                        ExpStack.push_back((NewChildren[0]->template 
                                            SAs<OpExpression>()->GetChildren())[0]);
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
            inline void 
            Canonicalizer<E, S>::VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp)
            {
                ExpressionVisitorBase<E, S>::VisitEQuantifiedExpression(Exp);
                auto NewQExpr = ExpStack.back();
                ExpStack.pop_back();
                ExpStack.push_back(new EQuantifiedExpression<E, S>(nullptr, Exp->GetQVarTypes(),
                                                                   NewQExpr, Exp->ExtensionData));
            }

            template <typename E, template <typename> class S>
            inline void 
            Canonicalizer<E, S>::VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp)
            {
                ExpressionVisitorBase<E, S>::VisitAQuantifiedExpression(Exp);
                auto NewQExpr = ExpStack.back();
                ExpStack.pop_back();
                ExpStack.push_back(new AQuantifiedExpression<E, S>(nullptr, Exp->GetQVarTypes(),
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
            Stringifier<E, S>::Stringifier(const UFID2DMapT& UFMap)
                : ExpressionVisitorBase<E, S>("Z3Stringifier"), 
                  UFMap(UFMap)
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
            inline void 
            Stringifier<E, S>::VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp)
            {
                ExpressionVisitorBase<E, S>::VisitBoundVarExpression(Exp);
                StringStack.push_back(BoundVarPrefix + "@" + to_string(Exp->GetVarIdx()));
            }

            template <typename E, template <typename> class S>
            inline void Stringifier<E, S>::VisitOpExpression(const OpExpression<E, S>* Exp)
            {
                auto const& OpCode = Exp->GetOpCode();
                ExpressionVisitorBase<E, S>::VisitOpExpression(Exp);
                const u32 NumChildren = Exp->GetChildren().size();
                vector<string> SubExps(NumChildren);
                string OpString;
                auto it = OpCodeToNameMap.find(OpCode);
                if (it != OpCodeToNameMap.end()) {
                    OpString = it->second;
                } else {
                    auto it = UFMap.find(OpCode);
                    if (it == UFMap.end()) {
                        throw InternalError((string)"Unknown uninterpreted function. At: " + 
                                            __FILE__ + ":" + to_string(__LINE__));
                    }
                    auto const& Desc = it->second;
                    OpString = Desc.GetName();
                }

                string ExpString = (string)"(" + OpString;
                for (u32 i = 0; i < NumChildren; ++i) {
                    SubExps[NumChildren - i - 1] = StringStack.back();
                    StringStack.pop_back();
                }
                for (auto const& SubExp : SubExps) {
                    ExpString += (" " + SubExp);
                }
                ExpString += ")";
                StringStack.push_back(ExpString);
            }

            template <typename E, template <typename> class S>
            inline void 
            Stringifier<E, S>::VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp)
            {
                auto const& QVarTypes = Exp->GetQVarTypes();
                string ExpString;
                if (Exp->IsForAll()) {
                    ExpString = "(forall (";
                } else {
                    ExpString = "(exists (";
                }
                bool First = true;

                for (auto const& QVarType : QVarTypes) {
                    if (!First) {
                        ExpString += " ";
                    }
                    First = false;
                    ExpString += "(" + TypeToString(QVarType) + ")";
                }
                ExpString += ") ";
                Exp->GetQExpression()->Accept(this);
                ExpString += StringStack.back();
                ExpString += ")";
                StringStack.pop_back();
                StringStack.push_back(ExpString);
            }

            template <typename E, template <typename> class S>
            inline void 
            Stringifier<E, S>::VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp)
            {
                VisitQuantifiedExpression(Exp);
            }

            template <typename E, template <typename> class S>
            inline void 
            Stringifier<E, S>::VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp)
            {
                VisitQuantifiedExpression(Exp);
            }

            template <typename E, template <typename> class S>
            inline string
            Stringifier<E, S>::Do(const ExpT& Exp, const UFID2DMapT& UFMap)
            {
                Stringifier TheStringifier(UFMap);
                Exp->Accept(&TheStringifier);
                return TheStringifier.StringStack[0];
            }

            template <typename E, template <typename> class S>
            inline string
            Stringifier<E, S>::TypeToString(i64 Type)
            {
                if (Type == Z3SemOps::IntType) {
                    return "Int";
                }
                if (Type == Z3SemOps::BoolType) {
                    return "Bool";
                }
                if (Type == Z3SemOps::BVTypeAll) {
                    return "(BV _)";
                } 
                if (Type > Z3SemOps::BVTypeAll &&
                    Type < BVTypeEnd) {
                    return (string)"(BV " + to_string(Type - Z3SemOps::BVTypeAll) + ")";
                }
                throw InternalError((string)"Unknown type " + to_string(Type) + " At : " + 
                                    __FILE__ + to_string(__LINE__));
            }
            
            // Lowerer implementation
            template <typename E, template <typename> class S>
            Lowerer<E, S>::Lowerer(const UFID2DMapT& UFMap, const Z3Ctx& Ctx)
                : ExpressionVisitorBase<E,S>("Z3Lowerer"), UFMap(UFMap),
                  Ctx(Ctx)
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            Lowerer<E, S>::~Lowerer()
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            inline Z3_sort Lowerer<E, S>::LowerType(i64 Type)
            {
                if (Type == Z3SemOps::BoolType) {
                    return Z3_mk_bool_sort(*Ctx);
                } else if (Type == Z3SemOps::IntType) {
                    return Z3_mk_int_sort(*Ctx);
                } else {
                    if (Type <= Z3SemOps::BVTypeAll || Type >= BVTypeEnd) {
                        throw InternalError((string)"Unknown type " + to_string(Type) + 
                                            " At: " + __FILE__ + ":" + to_string(__LINE__));
                    }
                    auto Size = Type - Z3SemOps::BVTypeAll;
                    return Z3_mk_bv_sort(*Ctx, Size);
                }
            }

            template <typename E, template <typename> class S>
            inline void 
            Lowerer<E, S>::VisitVarExpression(const VarExpression<E, S>* Exp)
            {
                auto VarSym = Z3_mk_string_symbol(*Ctx, Exp->GetVarName().c_str());
                Z3Expr LoweredExp(Ctx, Z3_mk_const(*Ctx, VarSym, 
                                                   LowerType(Exp->GetVarType())));
                ExpStack.push_back(LoweredExp);
            }

            template <typename E, template <typename> class S>
            inline void 
            Lowerer<E, S>::VisitConstExpression(const ConstExpression<E, S>* Exp)
            {
                const string& ConstVal = Exp->GetConstValue();
                auto Type = Exp->GetConstType();
                
                if (Type == Z3SemOps::IntType) {
                    Z3Expr LoweredExp(Ctx, Z3_mk_numeral(*Ctx, ConstVal.c_str(), 
                                                         LowerType(Type)));
                    ExpStack.push_back(LoweredExp);
                } else if (Type == Z3SemOps::BoolType) {
                    if (ConstVal == "true") {
                        Z3Expr LoweredExp(Ctx, Z3_mk_true(*Ctx));
                        ExpStack.push_back(LoweredExp);
                    } else {
                        Z3Expr LoweredExp(Ctx, Z3_mk_false(*Ctx));
                        ExpStack.push_back(LoweredExp);
                    }
                } else {
                    boost::multiprecision::cpp_int Val = 0;
                    if (boost::algorithm::starts_with(ConstVal, "#b")) {
                        for (u32 i = 2; i < ConstVal.length(); ++i) {
                            Val *= 2;
                            if (ConstVal[i] == '1') {
                                Val += 1;
                            }
                        }
                    } else {
                        istringstream istr(ConstVal.substr(2, ConstVal.length() - 2));
                        istr >> hex >> Val;
                    }
                    ostringstream sstr;
                    sstr << Val;
                    Z3Expr LoweredExp(Ctx, Z3_mk_numeral(*Ctx, sstr.str().c_str(),
                                                          LowerType(Type)));
                    ExpStack.push_back(LoweredExp);
                }
            }

            template <typename E, template <typename> class S>
            inline void 
            Lowerer<E, S>::VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp)
            {
                auto Type = Exp->GetVarType();
                auto Idx = Exp->GetVarIdx();
                
                Z3Expr LoweredExp(Ctx, Z3_mk_bound(*Ctx, Idx, LowerType(Type)));
                ExpStack.push_back(LoweredExp);
            }

            template <typename E, template <typename> class S>
            inline void Lowerer<E, S>::VisitOpExpression(const OpExpression<E, S>* Exp)
            {
                ExpressionVisitorBase<E, S>::VisitOpExpression(Exp);
                auto OpCode = Exp->GetOpCode();
                const u32 NumChildren = Exp->GetChildren().size();
                Z3_ast* ChildArray = new Z3_ast[NumChildren];
                // We hang on to the reference to the lowered children
                // to prevent them from being destroyed
                vector<Z3Expr> LoweredChildren(NumChildren);
                
                for (u32 i = 0; i < NumChildren; ++i) {
                    LoweredChildren[NumChildren - i - 1] = ExpStack.back();
                    ChildArray[NumChildren - i - 1] = LoweredChildren[NumChildren - i - 1];
                    ExpStack.pop_back();
                }

                switch (OpCode) {
                case Z3SemOps::OpEQ:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_eq(*Ctx, ChildArray[0],
                                                            ChildArray[1])));
                    break;
                    
                case Z3SemOps::OpNOT:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_not(*Ctx, ChildArray[0])));
                    break;

                case Z3SemOps::OpITE:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_ite(*Ctx, ChildArray[0],
                                                             ChildArray[1],
                                                             ChildArray[2])));
                    break;

                case Z3SemOps::OpOR:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_or(*Ctx, NumChildren, ChildArray)));
                    break;

                case Z3SemOps::OpAND:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_and(*Ctx, NumChildren, ChildArray)));
                    break;

                case Z3SemOps::OpIMPLIES:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_implies(*Ctx, ChildArray[0], 
                                                                 ChildArray[1])));
                    break;

                case Z3SemOps::OpIFF:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_iff(*Ctx, ChildArray[0], ChildArray[1])));
                    break;
                    
                case Z3SemOps::OpXOR:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_xor(*Ctx, ChildArray[0], ChildArray[1])));
                    break;
                    
                case Z3SemOps::OpADD:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_add(*Ctx, NumChildren, ChildArray)));
                    break;
                    
                case Z3SemOps::OpSUB:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_sub(*Ctx, NumChildren, ChildArray)));
                    break;

                case Z3SemOps::OpMINUS:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_unary_minus(*Ctx, ChildArray[0])));
                    break;

                case Z3SemOps::OpMUL:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_mul(*Ctx, NumChildren, ChildArray)));
                    break;

                case Z3SemOps::OpDIV:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_div(*Ctx, ChildArray[0], ChildArray[1])));
                    break;

                case Z3SemOps::OpMOD:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_mod(*Ctx, ChildArray[0], ChildArray[1])));
                    break;

                case Z3SemOps::OpREM:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_rem(*Ctx, ChildArray[0], ChildArray[1])));
                    break;

                case Z3SemOps::OpPOWER:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_power(*Ctx, ChildArray[0], ChildArray[1])));
                    break;

                case Z3SemOps::OpGE:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_ge(*Ctx, ChildArray[0], ChildArray[1])));
                    break;

                case Z3SemOps::OpGT:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_gt(*Ctx, ChildArray[0], ChildArray[1])));
                    break;

                case Z3SemOps::OpLE:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_le(*Ctx, ChildArray[0], ChildArray[1])));
                    break;

                case Z3SemOps::OpLT:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_lt(*Ctx, ChildArray[0], ChildArray[1])));
                    break;

                case Z3SemOps::OpBVNOT:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_bvnot(*Ctx, ChildArray[0])));
                    break;

                case Z3SemOps::OpBVREDAND:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_bvredand(*Ctx, ChildArray[0])));
                    break;

                case Z3SemOps::OpBVREDOR:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_bvredor(*Ctx, ChildArray[0])));
                    break;

                case Z3SemOps::OpBVAND:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_bvand(*Ctx, ChildArray[0], ChildArray[1])));
                    break;

                case Z3SemOps::OpBVOR:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_bvor(*Ctx, ChildArray[0], ChildArray[1])));
                    break;

                case Z3SemOps::OpBVXOR:
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_bvxor(*Ctx, ChildArray[0], ChildArray[1])));
                    break;
                    
                default: 
                    {
                        auto it = UFMap.find(OpCode);
                        if (it == UFMap.end()) {
                            throw InternalError((string)"Unknown op " + to_string(OpCode) + " At: " + 
                                                __FILE__ + ":" + to_string(__LINE__));
                        }
                        
                        auto const& Desc = it->second;
                        auto const& DomTypes = Desc.GetDomainTypes();
                        auto RangeType = Desc.GetRangeType();
                        auto const& Name = Desc.GetName();
                        const u32 NumArgs = Desc.GetDomainTypes().size();
                        Z3_sort* DomSorts = new Z3_sort[NumArgs];
                        for (u32 i = 0; i < NumArgs; ++i) {
                            DomSorts[i] = LowerType(DomTypes[i]);
                        }
                        auto RangeSort = LowerType(RangeType);
                        auto FuncSym = Z3_mk_string_symbol(*Ctx, Name.c_str());
                        auto FuncDecl = Z3_mk_func_decl(*Ctx, FuncSym, NumArgs, DomSorts, RangeSort);
                        Z3_inc_ref(*Ctx, Z3_func_decl_to_ast(*Ctx, FuncDecl));

                        ExpStack.push_back(Z3Expr(Ctx, Z3_mk_app(*Ctx, FuncDecl, 
                                                                 NumArgs, ChildArray)));

                        Z3_dec_ref(*Ctx, Z3_func_decl_to_ast(*Ctx, FuncDecl));
                        delete[] DomSorts;
                        break;
                    }
                }
                delete[] ChildArray;
            }

            template <typename E, template <typename> class S>
            inline void 
            Lowerer<E, S>::VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp)
            {
                auto const& QExpr = Exp->GetQExpression();
                auto const& QVarTypes = Exp->GetQVarTypes();
                const u32 NumBound = QVarTypes.size();
                Z3_sort* BoundSorts = new Z3_sort[NumBound];
                Z3_symbol* BoundSyms = new Z3_symbol[NumBound];

                for (u32 i = 0; i < NumBound; ++i) {
                    BoundSyms[i] = Z3_mk_string_symbol(*Ctx, (BoundVarPrefix + 
                                                              to_string(NumBound - i - 1)).c_str());
                    BoundSorts[i] = LowerType(QVarTypes[i]);
                }

                QExpr->Accept(this);
                auto const& QBody = ExpStack.back();
                ExpStack.pop_back();
                if (Exp->IsForAll()) {
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_forall(*Ctx, 0, 0, nullptr, NumBound,
                                                                BoundSorts, BoundSyms, QBody)));
                } else {
                    ExpStack.push_back(Z3Expr(Ctx, Z3_mk_exists(*Ctx, 0, 0, nullptr, NumBound,
                                                                BoundSorts, BoundSyms, QBody)));
                }
                delete[] BoundSorts;
                delete[] BoundSyms;
            }

            template <typename E, template <typename> class S>
            inline void 
            Lowerer<E, S>::VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp)
            {
                VisitQuantifiedExpression(Exp);
            }

            template <typename E, template <typename> class S>
            inline void 
            Lowerer<E, S>::VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp)
            {
                VisitQuantifiedExpression(Exp);
            }

            template <typename E, template <typename> class S>
            inline typename Lowerer<E, S>::LExpT
            Lowerer<E, S>::Do(const ExpT& Exp, const UFID2DMapT& UFMap, const Z3Ctx& Ctx)
            {
                Lowerer TheLowerer(UFMap, Ctx);
                Exp->Accept(&TheLowerer);
                return TheLowerer.ExpStack[0];
            }


            // Raiser implementation

            template <typename E, template <typename> class S>
            inline Raiser<E, S>::Raiser(const UFName2DMapT& UFMap)
                : UFMap(UFMap)
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            inline Raiser<E, S>::~Raiser()
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            inline i64 Raiser<E, S>::RaiseSort(Z3_sort Sort)
            {
                auto Kind = Z3_get_sort_kind(*Ctx, Sort);
                switch (Kind) {
                case Z3_BOOL_SORT:
                    return Z3SemOps::BoolType;
                    
                case Z3_INT_SORT:
                    return Z3SemOps::IntType;

                case Z3_BV_SORT:
                    return Z3SemOps::BVTypeAll + Z3_get_bv_sort_size(*Ctx, Sort);
                    
                default:
                    throw InternalError((string)"Unknown Sort " + to_string(Kind) + ", " + 
                                        __FILE__ + ":" + to_string(__LINE__));
                }
            }

            template <typename E, template <typename> class S>
            inline typename Raiser<E, S>::ExpT
            Raiser<E, S>::RaiseApp(const LExpT& LExp)
            {
                auto App = Z3_to_app(*Ctx, LExp);
                // Find out if this is uninterpreted
                auto AppDecl = Z3_get_app_decl(*Ctx, App);
                auto DeclSym = Z3_get_decl_name(*Ctx, AppDecl);
                string DeclName(Z3_get_symbol_string(*Ctx, DeclSym));
                auto NumArgs = Z3_get_domain_size(*Ctx, AppDecl);
                auto AppKind = Z3_get_decl_kind(*Ctx, AppDecl);
                vector<ExpT> RaisedChildren(NumArgs);
                for (u32 i = 0; i < NumArgs; ++i) {
                    RaisedChildren[i] = Raise(Z3Expr(Ctx, Z3_get_app_arg(*Ctx, App, i)));
                }

                switch (AppKind) {
                case Z3_OP_UNINTERPRETED:
                    if (NumArgs == 0) {
                        // A variable
                        auto RangeSort = Z3_get_range(*Ctx, AppDecl);
                        return new VarExpression<E, S>(nullptr, DeclName,
                                                       RaiseSort(RangeSort));
                    } else {
                        // A true uninterpreted function
                        vector<i64> DomTypes(NumArgs);
                        for (u32 i = 0; i < NumArgs; ++i) {
                            DomTypes[i] = RaiseSort(Z3_get_domain(*Ctx, AppDecl, i));
                        }
                        string MangledName = MangleName(DeclName, DomTypes);
                        auto it = UFMap.find(MangledName);
                        if (it == UFMap.end()) {
                            throw InternalError((string)"Could not find descriptor for function " + 
                                                DeclName + ". Z3 expression:\n" + 
                                                Z3_ast_to_string(*Ctx, LExp) + ", " + __FILE__ + 
                                                ":" + to_string(__LINE__));
                        }
                        i64 OpCode = it->second.GetIdentifier();
                        return new OpExpression<E, S>(nullptr, OpCode, RaisedChildren);
                    }

                case Z3_OP_TRUE:
                    return new ConstExpression<E, S>(nullptr, "true", Z3SemOps::BoolType);
                    
                case Z3_OP_FALSE:
                    return new ConstExpression<E, S>(nullptr, "false", Z3SemOps::BoolType);

                case Z3_OP_EQ:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpEQ, RaisedChildren);
                    
                case Z3_OP_ITE:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpITE, RaisedChildren);
                    
                case Z3_OP_OR:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpOR, RaisedChildren);
                    
                case Z3_OP_AND:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpAND, RaisedChildren);

                case Z3_OP_IMPLIES:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpIMPLIES, RaisedChildren);
                    
                case Z3_OP_IFF:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpIFF, RaisedChildren);

                case Z3_OP_XOR:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpXOR, RaisedChildren);

                case Z3_OP_NOT:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpNOT, RaisedChildren);

                case Z3_OP_ADD:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpADD, RaisedChildren);

                case Z3_OP_SUB:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpSUB, RaisedChildren);

                case Z3_OP_UMINUS:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpMINUS, RaisedChildren);

                case Z3_OP_MUL:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpMUL, RaisedChildren);

                case Z3_OP_DIV:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpDIV, RaisedChildren);

                case Z3_OP_MOD:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpMOD, RaisedChildren);

                case Z3_OP_REM:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpREM, RaisedChildren);

                case Z3_OP_POWER:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpPOWER, RaisedChildren);

                case Z3_OP_LE:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpLE, RaisedChildren);

                case Z3_OP_LT:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpLT, RaisedChildren);

                case Z3_OP_GE:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpGE, RaisedChildren);

                case Z3_OP_GT:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpGT, RaisedChildren);

                case Z3_OP_BNOT:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpBVNOT, RaisedChildren);

                case Z3_OP_BREDOR:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpBVREDOR, RaisedChildren);

                case Z3_OP_BREDAND:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpBVREDAND, RaisedChildren);

                case Z3_OP_BOR:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpBVOR, RaisedChildren);

                case Z3_OP_BAND:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpBVAND, RaisedChildren);

                case Z3_OP_BXOR:
                    return new OpExpression<E, S>(nullptr, Z3SemOps::OpBVXOR, RaisedChildren);

                default:
                    throw InternalError((string)"Unknown Z3 Op: " + to_string(AppKind) + 
                                        ". Z3 expression:\n" + Z3_ast_to_string(*Ctx, LExp) + 
                                        ", " + __FILE__ + ":" + to_string(__LINE__));

                }
            }

            template <typename E, template <typename> class S>
            inline typename Raiser<E, S>::ExpT
            Raiser<E, S>::RaiseNum(const LExpT& LExp)
            {
                auto Sort = Z3_get_sort(*Ctx, LExp);
                auto Type = RaiseSort(Sort);
                
                switch (Type) {
                case Z3SemOps::IntType:
                    return new ConstExpression<E, S>(nullptr, 
                                                     Z3_get_numeral_string(*Ctx, LExp), Type);

                default:
                    {
                        auto BVSize = Type - Z3SemOps::BVTypeAll;
                        istringstream istr(Z3_get_numeral_string(*Ctx, LExp));
                        boost::multiprecision::cpp_int HexVal;
                        istr >> HexVal;
                        ostringstream sstr;
                        sstr << hex << HexVal;
                        auto HexString = sstr.str();
                        if (BVSize % 4 == 0) {
                            return new ConstExpression<E, S>(nullptr, HexString, Type);
                        } else {
                            string BinString;
                            for (u32 i = 0; i < HexString.length(); ++i) {
                                switch(HexString[i]) {
                                case '0':
                                    BinString += "0000"; break;
                                case '1':
                                    BinString += "0001"; break;
                                case '2':
                                    BinString += "0010"; break;
                                case '3':
                                    BinString += "0011"; break;
                                case '4':
                                    BinString += "0100"; break;
                                case '5':
                                    BinString += "0101"; break;
                                case '6':
                                    BinString += "0110"; break;
                                case '7':
                                    BinString += "0111"; break;
                                case '8':
                                    BinString += "1000"; break;
                                case '9':
                                    BinString += "1001"; break;
                                case 'A':
                                    BinString += "1010"; break;
                                case 'B':
                                    BinString += "1011"; break;
                                case 'C':
                                    BinString += "1100"; break;
                                case 'D':
                                    BinString += "1101"; break;
                                case 'E':
                                    BinString += "1110"; break;
                                case 'F':
                                    BinString += "1111"; break;
                                default:
                                    throw InternalError((string)"Unexpected hex string: " + 
                                                        HexString + ", " + __FILE__ + ":" + 
                                                        to_string(__LINE__));
                                }
                            }
                            auto Snip = HexString.length() - BVSize;
                            auto FinalStr = BinString.substr(Snip, BVSize);
                            return new ConstExpression<E, S>(nullptr, FinalStr, Type);
                        }
                    }
                }
            }

            template <typename E, template <typename> class S>
            inline typename Raiser<E, S>::ExpT
            Raiser<E, S>::RaiseVar(const LExpT& LExp)
            {
                return new BoundVarExpression<E, S>(nullptr,
                                                    RaiseSort(Z3_get_sort(*Ctx, LExp)),
                                                    Z3_get_index_value(*Ctx, LExp));
            }

            template <typename E, template <typename> class S>
            inline typename Raiser<E, S>::ExpT
            Raiser<E, S>::RaiseQuant(const LExpT& LExp)
            {
                auto NumBound = Z3_get_quantifier_num_bound(*Ctx, LExp);
                vector<i64> QVarTypes(NumBound);
                for (u32 i = 0; i < NumBound; ++i) {
                    QVarTypes[i] = RaiseSort(Z3_get_quantifier_bound_sort(*Ctx, LExp, i));
                }

                Z3Expr QBody(Ctx, Z3_get_quantifier_body(*Ctx, LExp));
                if (Z3_is_quantifier_forall(*Ctx, LExp)) {
                    return new AQuantifiedExpression<E, S>(nullptr, 
                                                           QVarTypes,
                                                           Raise(QBody));
                } else {
                    return new EQuantifiedExpression<E, S>(nullptr, 
                                                           QVarTypes,
                                                           Raise(QBody));
                }
            }

            template <typename E, template <typename> class S>
            inline typename Raiser<E, S>::ExpT
            Raiser<E, S>::Raise(const LExpT& LExp)
            {
                auto Kind = Z3_get_ast_kind(*Ctx, LExp);
                switch (Kind) {
                case Z3_APP_AST:
                    return RaiseApp(LExp);
                case Z3_NUMERAL_AST:
                    return RaiseNum(LExp);
                case Z3_VAR_AST:
                    return RaiseVar(LExp);
                case Z3_QUANTIFIER_AST:
                    return RaiseQuant(LExp);
                default:
                    throw InternalError((string)"Unknown/Unhandled Z3_ast kind: " + 
                                        to_string(Kind) + ". The Z3 expression is:\n" + 
                                        Z3_ast_to_string(*Ctx, LExp) + ", " + __FILE__ + 
                                        ":" + to_string(__LINE__));
                }
            }

            template <typename E, template <typename> class S>
            inline typename Raiser<E, S>::ExpT
            Raiser<E, S>::Do(const LExpT& LExp, const UFName2DMapT& UFMap)
            {
                Raiser<E, S> TheRaiser(UFMap);
                TheRaiser.Ctx = LExp.GetCtx();
                return TheRaiser.Raise(LExp);
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
            typedef Z3SemOps Ops;
            typedef Z3Expr LExpT;
            typedef Expr<E, ESMC::Z3Sem::Z3Semanticizer> ExpT;

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
            : UFUIDGen(UFOffset)
        {
            // Nothing here
        }

        template <typename E>
        Z3Semanticizer<E>::~Z3Semanticizer()
        {
            Z3_reset_memory();
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
            return Canonicalize(Raiser<E, ESMC::Z3Sem::Z3Semanticizer>::Do(LExp, UFName2DMap));
        }

        template <typename E>
        inline typename Z3Semanticizer<E>::LExpT
        Z3Semanticizer<E>::LowerExpr(const ExpT& Exp)
        {
            Z3Ctx Ctx(new Z3CtxWrapper());
            return Lowerer<E, ESMC::Z3Sem::Z3Semanticizer>::Do(Exp, UFID2DMap, Ctx);
        }

        template <typename E>
        inline typename Z3Semanticizer<E>::ExpT
        Z3Semanticizer<E>::Simplify(const ExpT& Exp)
        {
            auto LExp = LowerExpr(Exp);
            auto Ctx = LExp.GetCtx();
            LExp = Z3Expr(Ctx, Z3_simplify(*Ctx, LExp));
            return RaiseExpr(LExp);
        }

        template <typename E>
        inline typename Z3Semanticizer<E>::ExpT
        Z3Semanticizer<E>::ElimQuantifiers(const ExpT& Exp)
        {
            auto LExp = LowerExpr(Exp);
            auto Ctx = LExp.GetCtx();
            auto QE = Z3_mk_tactic(*Ctx, "qe");
            Z3_tactic_inc_ref(*Ctx, QE);
            auto Goal = Z3_mk_goal(*Ctx, true, false, false);
            Z3_goal_inc_ref(*Ctx, Goal);
            Z3_goal_assert(*Ctx, Goal, LExp);

            auto Res = Z3_tactic_apply(*Ctx, QE, Goal);
            Z3_goal_dec_ref(*Ctx, Goal);
            Z3_tactic_dec_ref(*Ctx, QE);

            Z3_apply_result_inc_ref(*Ctx, Res);

            vector<ExpT> RaisedExprs;
            
            auto NumGoals = Z3_apply_result_get_num_subgoals(*Ctx, Res);
            for (u32 i = 0; i < NumGoals; ++i) {
                auto CurGoal = Z3_apply_result_get_subgoal(*Ctx, Res, i);
                auto NumExprs = Z3_goal_size(*Ctx, CurGoal);
                for (u32 j = 0; j < NumExprs; j++) {
                    auto CurFormula = Z3_goal_formula(*Ctx, CurGoal, j);
                    Z3Expr CurExpr = Z3Expr(Ctx, CurFormula);
                    RaisedExprs.push_back(RaiseExpr(CurExpr));
                }
            }

            Z3_apply_result_dec_ref(*Ctx, Res);
            if (RaisedExprs.size() == 1) {
                return Canonicalize(RaisedExprs[0]);
            } else {
                return Canonicalize(new OpExpression
                                    <E, ESMC::Z3Sem::Z3Semanticizer>(nullptr, 
                                                                     Z3SemOps::OpAND,
                                                                     RaisedExprs));
            }
        }

        template <typename E>
        inline string Z3Semanticizer<E>::ExprToString(const ExpT& Exp) const
        {
            return Stringifier<E, ESMC::Z3Sem::Z3Semanticizer>::Do(Exp, UFID2DMap);
        }

        template <typename E>
        inline string Z3Semanticizer<E>::TypeToString(i64 Type) const
        {
            return Stringifier<E, ESMC::Z3Sem::Z3Semanticizer>::TypeToString(Type);
        }

        template <typename E>
        inline i64 Z3Semanticizer<E>::RegisterUninterpretedFunction(const string& Name, 
                                                                    const vector<i64> &DomTypes, 
                                                                    i64 RangeType)
        {
            CheckType(RangeType);
            for (auto const& DomType : DomTypes) {
                CheckType(DomType);
            }

            auto NewID = UFUIDGen.GetUID();
            UFDescriptor Desc(NewID, DomTypes, RangeType, Name);
            if (UFName2DMap.find(Desc.GetMangledName()) != UFName2DMap.end()) {
                throw ExprTypeError((string)"Uninterpreted function with name " + 
                                    Name + " already registered!");
            }
            UFName2DMap[Desc.GetMangledName()] = Desc;
            UFID2DMap[NewID] = Desc;
            return NewID;
        }

    } /* end namespace Z3Sem */
} /* end namespace ESMC */

#endif /* ESMC_Z3_SEMANTICIZER_HPP_ */


// 
// Z3Semanticizer.hpp ends here
