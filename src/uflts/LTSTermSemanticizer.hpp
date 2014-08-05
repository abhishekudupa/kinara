// LTSTermSemanticizer.hpp --- 
// 
// Filename: LTSTermSemanticizer.hpp
// Author: Abhishek Udupa
// Created: Thu Jul 24 18:49:24 2014 (-0400)
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


#if !defined ESMC_LTS_TERM_SEMANTICIZER_HPP_
#define ESMC_LTS_TERM_SEMANTICIZER_HPP_

#include "../common/FwdDecls.hpp"
#include "../containers/RefCountable.hpp"
#include "../containers/SmartPtr.hpp"
#include "../containers/RefCache.hpp"
#include "../utils/UIDGenerator.hpp"
#include "../expr/SemanticizerUtils.hpp"
#include "../expr/Expressions.hpp"
#include "../expr/ExprTypes.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/multiprecision/cpp_int.hpp>

#include <unordered_map>
#include <set>
#include <vector>

namespace ESMC {
    namespace LTS {

        struct LTSOps 
        {
            static const i64 OpEQ = 1000;
            static const i64 OpNOT = 1001;
            static const i64 OpITE = 1002;
            static const i64 OpOR = 1003;
            static const i64 OpAND = 1004;
            static const i64 OpIMPLIES = 1005;
            static const i64 OpIFF = 1006;
            static const i64 OpXOR = 1007;

            // Arithmetic
            static const i64 OpADD = 1008;
            static const i64 OpSUB = 1009;
            static const i64 OpMINUS = 1010;
            static const i64 OpMUL = 1011;
            static const i64 OpDIV = 1012;
            static const i64 OpMOD = 1013;
            static const i64 OpGT = 1014;
            static const i64 OpGE = 1015;
            static const i64 OpLT = 1016;
            static const i64 OpLE = 1017;

            // Syntactic operators for symmetry, etc
            static const i64 OpIndex = 1018;
            static const i64 OpField = 1019;
            

            static const i64 UFOffset = 1000000;

            // Temporal operators
            static const i64 OpTemporalX = 1020;
            static const i64 OpTemporalU = 1021;
            static const i64 OpTemporalF = 1022;
            static const i64 OpTemporalG = 1023;

            // Some special types
            // for creating expressions for field access
            static const i64 FieldAccType = 1000000000;
        };

        namespace Detail {
            using namespace ESMC::Exprs;
            extern const unordered_map<i64, string> OpCodeToNameMap;
            extern const string BoundVarPrefix;
            extern const ExprTypeRef InvalidType;

            typedef unordered_map<i64, ExprTypeRef> UFID2TypeMapT;

            template <typename E, template <typename> class S>
            class TypeChecker : public ExpressionVisitorBase<E, S>
            {
            private:
                const UFID2TypeMapT& UFMap;
                typedef Expr<E, S> ExpT;

                ExprTypeRef BoolType;
                ExprTypeRef IntType;

                vector<vector<ExprTypeRef>> ScopeStack;

                inline void VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp);

            public:
                TypeChecker(const UFID2TypeMapT& UFMap, const ExprTypeRef& BoolType,
                            const ExprTypeRef& IntType);
                virtual ~TypeChecker();
                
                inline virtual void VisitVarExpression(const VarExpression<E, S>* Exp) override;
                inline virtual void VisitConstExpression(const ConstExpression<E, S>* Exp) override;
                inline virtual void VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp) 
                    override;
                inline virtual void VisitOpExpression(const OpExpression<E, S>* Exp) override;
                inline virtual void VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>*
                                                               Exp) override;
                inline virtual void VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>*
                                                               Exp) override;
                
                static inline void Do(const ExpT& Exp, const UFID2TypeMapT& UFMap,
                                      const ExprTypeRef& BoolType, const ExprTypeRef& IntType);
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
                const UFID2TypeMapT& UFMap;
                typedef typename S<E>::TypeT TypeT;

                inline void VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp);

            public:
                Stringifier(const UFID2TypeMapT& UFMap);
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
                
                static inline string Do(const ExpT& Exp, const UFID2TypeMapT& UFMap);
                static inline string TypeToString(const TypeT& Type);
            };

            template <typename E, template <typename> class S>
            class Simplifier : public ExpressionVisitorBase<E, S>
            {
            private:
                typedef Expr<E, S> ExpT;
                vector<ExpT> ExpStack;
                ExprTypeRef BoolType;
                ExprTypeRef IntType;
                inline void VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp);

            public:
                Simplifier(const ExprTypeRef& BoolType, const ExprTypeRef& IntType);
                ~Simplifier();

                inline virtual void VisitVarExpression(const VarExpression<E, S>* Exp) override;
                inline virtual void VisitConstExpression(const ConstExpression<E, S>* Exp) override;
                inline virtual void VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp) 
                    override;
                inline virtual void VisitOpExpression(const OpExpression<E, S>* Exp) override;
                inline virtual void VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>*
                                                               Exp) override;
                inline virtual void VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>*
                                                               Exp) override;
                
                static inline ExpT Do(const ExpT& Exp, const ExprTypeRef& BoolType,
                                      const ExprTypeRef& IntType);
            };

            // Implementation of type checker
            template <typename E, template <typename> class S>
            TypeChecker<E, S>::TypeChecker(const UFID2TypeMapT& UFMap,
                                           const ExprTypeRef& BoolType,
                                           const ExprTypeRef& IntType)
                : ExpressionVisitorBase<E, S>("LTSTermTypeChecker"),
                  UFMap(UFMap), BoolType(BoolType), IntType(IntType)
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            TypeChecker<E, S>::~TypeChecker()
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            inline void TypeChecker<E, S>::VisitVarExpression(const VarExpression<E, S>* Exp)
            {
                auto PrevType = Exp->GetType();
                if (PrevType != InvalidType) {
                    return;
                }
                auto Type = Exp->GetVarType();

                if (Type->template Is<ExprFuncType>() ||
                    Type->template Is<ExprParametricType>()) {
                    throw ExprTypeError((string)"Cannot create variables of parametric types " + 
                                        "and function types");
                }
                
                Exp->SetType(Type);
            }

            template <typename E, template <typename> class S>
            inline void TypeChecker<E, S>::VisitConstExpression(const ConstExpression<E, S>* Exp)
            {
                auto PrevType = Exp->GetType();
                if (PrevType != InvalidType) {
                    return;
                }
                auto ConstVal = Exp->GetConstValue();
                boost::algorithm::trim(ConstVal);

                // All types have a "clear" value, which is an alias for
                // the lowest value of the type
                if (ConstVal == "clear") {
                    Exp->SetType(Exp->GetConstType());
                    return;
                }

                auto Type = Exp->GetConstType();

                auto ActType = Type;
                if (ActType->template As<ExprBoolType>() != nullptr) {
                    boost::algorithm::to_lower(ConstVal);
                    if (ConstVal != "true" && ConstVal != "false") {
                        throw ExprTypeError("Unknown constant string");
                    }
                } else if(ActType->template As<ExprRangeType>() != nullptr ||
                          ActType->template As<ExprIntType>() != nullptr) {
                    if (!boost::algorithm::all(ConstVal, boost::algorithm::is_digit())) {
                        throw ExprTypeError((string)"Invalid value " + ConstVal);
                    }
                    boost::multiprecision::cpp_int Val = 0;
                    Val = boost::lexical_cast<boost::multiprecision::cpp_int>(ConstVal);
                    if (ActType->template As<ExprRangeType>() != nullptr) {
                        auto RangeType = ActType->template SAs<ExprRangeType>();

                        if (Val < RangeType->GetLow() ||
                            Val > RangeType->GetHigh()) {
                            throw ExprTypeError((string)"Value " + ConstVal + " is out of bounds");
                        }
                    }
                    Exp->SetType(ActType);
                } else if(ActType->template As<ExprEnumType>() != nullptr) {
                    auto EnumType = ActType->template SAs<ExprEnumType>();
                    if (!EnumType->IsMember(ConstVal)) {
                        throw ExprTypeError((string)"Value " + ConstVal + " is not valid");
                    }
                    Exp->SetType(ActType);
                } else if(ActType->template As<ExprSymmetricType>() != nullptr) {
                    auto SymmType = ActType->template SAs<ExprSymmetricType>();
                    if (!SymmType->IsMember(ConstVal)) {
                        throw ExprTypeError((string)"Value " + ConstVal + " is not valid");
                    }
                    Exp->SetType(ActType);
                } else {
                    throw ExprTypeError((string)"Only boolean, range, enum and symmetric type " + 
                                        "constants are currently supported");
                }
            }

            static inline const ExprTypeRef& GetTypeForBoundVar
            (const vector<vector<ExprTypeRef>>& ScopeStack,
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
                return ExprTypeRef::NullPtr;                
            }

            template <typename E, template <typename> class S>
            inline void 
            TypeChecker<E, S>::VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp)
            {
                auto PrevType = Exp->GetType();
                if (PrevType != InvalidType) {
                    return;
                }

                auto Type = Exp->GetVarType();
                if (Type->template Is<ExprScalarType>()) {
                    auto Idx = Exp->GetVarIdx();
                    auto ExpectedType = GetTypeForBoundVar(ScopeStack, Idx);
                    if (ExpectedType != InvalidType && ExpectedType != Type) {
                        throw ExprTypeError((string)"Bound variable with index " + to_string(Idx) + 
                                            " has an ambiguous type");
                    }
                    Exp->SetType(Type);
                } else {
                    throw ExprTypeError((string)"Only scalar types " + 
                                        "may be used as quanfiers");
                }
            }

            static inline bool CheckTypeCompat(const ExprTypeRef& ExpType, 
                                               const ExprTypeRef& ActType)
            {
                if (ExpType == ActType) {
                    return true;
                } else {
                    if (ExpType->As<ExprIntType>() != nullptr &&
                        ActType->As<ExprIntType>() != nullptr) {
                        return true;
                    } else {
                        return false;
                    }
                }
            }

            template <typename T, typename V>
            static inline bool CheckEquality(const T& Collection, const V& Value)
            {
                return (all_of(Collection.begin(), Collection.end(),
                               [&] (const V& i) -> bool { return i == Value; }));
            }

            template <typename T, typename V>
            static inline bool CheckCompatibility(const T& Collection, const V& Value)
            {
                return all_of(Collection.begin(), Collection.end(),
                              [&] (const V& i) -> bool 
                              { return CheckTypeCompat(i, Value); });
            }

            static inline void CheckNumArgs(u32 Expected, u32 Actual, const string& OpName)
            {
                if (Expected != Actual) {
                    throw ExprTypeError(OpName + " must be applied to exactly " + 
                                        to_string(Expected) + " arguments");
                }
            }

            static inline bool CheckAllScalar(const vector<ExprTypeRef>& Types)
            {
                return all_of(Types.begin(), Types.end(),
                              [] (const ExprTypeRef& Type) -> bool 
                              {
                                  return (Type->Is<ExprScalarType>());
                              });
            }

            template <typename E, template <typename> class S>
            inline void 
            TypeChecker<E, S>::VisitOpExpression(const OpExpression<E, S>* Exp)
            {
                auto PrevType = Exp->GetType();
                if (PrevType != InvalidType) {
                    return;
                }

                vector<ExprTypeRef> ChildTypes;
                auto const& Children = Exp->GetChildren();
                for (auto const& Child : Exp->GetChildren()) {
                    Child->Accept(this);
                    ChildTypes.push_back(Child->GetType());
                }

                i64 OpCode = Exp->GetOpCode();

                switch(OpCode) {
                case LTSOps::OpEQ: {
                    CheckNumArgs(2, ChildTypes.size(), "=");
                    if (!CheckAllScalar(ChildTypes)) {
                        throw ExprTypeError("Only scalar types can be used in = comparisons");
                    }
                    if (!CheckTypeCompat(ChildTypes[1], ChildTypes[0])) {
                        throw ExprTypeError("All types to = op must be the same");
                    }
                    if (ChildTypes[0]->template Is<ExprSymmetricType>()) {
                        if (!Children[0]->template Is<ConstExpression>() ||
                            !Children[1]->template Is<ConstExpression>()) {
                            throw ExprTypeError((string)"Only constant values of symmetric " + 
                                                "types can be compared");
                        }
                    }
                    Exp->SetType(BoolType);
                    break;
                }

                case LTSOps::OpNOT: {
                    CheckNumArgs(1, ChildTypes.size(), "not");
                    if (!CheckTypeCompat(BoolType, ChildTypes[0])) {
                        throw ExprTypeError("not op can only be applied to boolean expressions");
                    }

                    Exp->SetType(BoolType);
                    break;
                }

                case LTSOps::OpITE: {
                    CheckNumArgs(3, ChildTypes.size(), "ite");
                    if (!CheckTypeCompat(BoolType, ChildTypes[0]) || 
                        (!CheckTypeCompat(ChildTypes[1], ChildTypes[2]))) {
                        throw ExprTypeError((string)"ite op requires a boolean predicate " + 
                                            "and the types of the branches to match");
                    }
                    Exp->SetType(ChildTypes[1]);
                    break;
                }

                case LTSOps::OpIMPLIES:
                case LTSOps::OpXOR:
                case LTSOps::OpIFF: {
                    CheckNumArgs(2, ChildTypes.size(), "implies, xor, iff");
                }
                case LTSOps::OpOR:
                case LTSOps::OpAND: {
                    if (ChildTypes.size() < 2) {
                        throw ExprTypeError((string)"and/or ops need at least two operands");
                    }

                    if (!all_of(ChildTypes.begin(), ChildTypes.end(),
                                [&] (const ExprTypeRef& i) -> bool 
                                { return CheckTypeCompat(i, BoolType); })) {
                        throw ExprTypeError((string)"implies, xor, iff, and, or ops need " + 
                                            "all operands to be boolean");
                    }
                    Exp->SetType(BoolType);
                    break;
                }
                case LTSOps::OpADD:
                case LTSOps::OpSUB:
                case LTSOps::OpMUL: {
                    if (ChildTypes.size() < 2) {
                        throw ExprTypeError((string)"add/sub/mul ops need at least two operands");
                    }
                    if (!all_of(ChildTypes.begin(), ChildTypes.end(),
                                [&] (const ExprTypeRef& i) -> bool 
                                { return CheckTypeCompat(i, IntType); })) {
                        throw ExprTypeError((string)"add/sub/mul ops need integer arguments");
                    }                    
                    // Assumption: Int type ALWAYS has ID 2
                    Exp->SetType(IntType);
                    break;
                }
                case LTSOps::OpDIV:
                case LTSOps::OpMOD:
                case LTSOps::OpLT:
                case LTSOps::OpLE:
                case LTSOps::OpGE:
                case LTSOps::OpGT: {
                    CheckNumArgs(2, ChildTypes.size(), "div/mod/cmp");
                    if (!all_of(ChildTypes.begin(), ChildTypes.end(),
                                [&] (const ExprTypeRef& i) -> bool 
                                { return CheckTypeCompat(IntType, i); })) {
                        throw ExprTypeError("div/mod/cmp operators expect integer operands");
                    }
                    if (OpCode == LTSOps::OpDIV || OpCode == LTSOps::OpMUL) {
                        Exp->SetType(IntType);
                    } else {
                        Exp->SetType(BoolType);
                    }
                    break;
                }
                case LTSOps::OpMINUS: {
                    CheckNumArgs(2, ChildTypes.size(), "minus");
                    if (!CheckTypeCompat(IntType, ChildTypes[0])) {
                        throw ExprTypeError("minus operator expects integer operands");
                    }
                    Exp->SetType(IntType);
                }
                case LTSOps::OpIndex: {
                    CheckNumArgs(2, ChildTypes.size(), "Index");
                    auto Type1 = ChildTypes[0];
                    auto ArrayType = Type1->template As<ExprArrayType>();

                    if (ArrayType == nullptr) {
                        throw ExprTypeError((string)"Index operator can only be " + 
                                            "applied to array types");
                    }

                    ExprTypeRef ExpIndexType;

                    ExpIndexType = ArrayType->GetIndexType();
                    
                    if (!CheckTypeCompat(ExpIndexType, ChildTypes[1])) {
                        throw ExprTypeError("Invalid type for index expression");
                    }

                    ExprTypeRef ElemType;

                    ElemType = ArrayType->GetValueType();
                    Exp->SetType(ElemType);
                    break;
                }
                case LTSOps::OpField: {
                    CheckNumArgs(2, ChildTypes.size(), "Field");

                    if (!ChildTypes[0]->template Is<ExprRecordType>()) {
                        throw ExprTypeError("Field access only allowed on record or union types");
                    }

                    if (ChildTypes[0]->template Is<ExprRecordType>() &&
                        !ChildTypes[1]->template Is<ExprFieldAccessType>()) {
                        throw ExprTypeError((string)"Record Field accesses must be made with " + 
                                            "variables of type FieldAccessType");
                    }

                    auto FieldExp = ((Exp->GetChildren())[1])->template As<VarExpression>();
                    if (FieldExp == nullptr) {
                        throw ExprTypeError("Field access expression must be a VarExpression");
                    }
                    
                    auto const& FieldName = FieldExp->GetVarName();
                    auto RecType = ChildTypes[0]->template SAs<ExprRecordType>();
                    auto ValType = RecType->GetTypeForMember(FieldName);
                    if (ValType == ExprTypeRef::NullPtr) {
                        throw ExprTypeError((string)"Field name \"" + 
                                            FieldName + "\" is invalid for " + 
                                            "record type \"" + RecType->GetName() + "\"");
                    }
                    Exp->SetType(ValType);
                    break;
                }

                default: {
                    // Must be an uninterpreted function
                    auto it = UFMap.find(OpCode);
                    if (it == UFMap.end()) {
                        throw ExprTypeError((string)"Unknown opcode " + to_string(OpCode));
                    }
                    auto FuncType = it->second->template As<ExprFuncType>();
                    auto const& ArgTypes = FuncType->GetArgTypes();
                    if (ChildTypes.size() != ArgTypes.size()) {
                        throw ExprTypeError((string)"Incorrect number of arguments for UF");
                    }
                    for (u32 i = 0; i < ChildTypes.size(); ++i) {
                        if (!CheckTypeCompat(ChildTypes[i], ArgTypes[i])) {
                            throw ExprTypeError("Incorrect type for argument of UF");
                        }
                    }
                    Exp->SetType(FuncType->GetFuncType());
                }
                }
            }

            template <typename E, template <typename> class S>
            inline void 
            TypeChecker<E, S>::VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp)
            {
                auto const& QVarTypes = Exp->GetQVarTypes();
                ScopeStack.push_back(QVarTypes);
                auto const& QExpr = Exp->GetQExpression();
                SemUtils::TypeInvalidator<E, S> Inv;
                QExpr->Accept(&Inv);
                QExpr->Accept(this);

                if (QExpr->GetType() != BoolType) {
                    throw ExprTypeError("Body of quantified expression needs to be boolean");
                }
                ScopeStack.pop_back();
                return;
            }

            template <typename E, template <typename> class S>
            inline void 
            TypeChecker<E, S>::VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp)
            {
                VisitQuantifiedExpression(Exp);
            }

            template <typename E, template <typename> class S>
            inline void 
            TypeChecker<E, S>::VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp)
            {
                VisitQuantifiedExpression(Exp);
            }

            template <typename E, template <typename> class S>
            inline void 
            TypeChecker<E, S>::Do(const ExpT& Exp, const UFID2TypeMapT& UFMap,
                                  const ExprTypeRef& BoolType, const ExprTypeRef& IntType)
            {
                TypeChecker Checker(UFMap, BoolType, IntType);
                Exp->Accept(&Checker);
            }

            // Implementation of Stringifier
            template <typename E, template <typename> class S>
            Stringifier<E, S>::Stringifier(const UFID2TypeMapT& UFMap)
                : ExpressionVisitorBase<E, S>("LTSTermStringifier"),
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
            inline void 
            Stringifier<E, S>::VisitVarExpression(const VarExpression<E, S>* Exp)
            {
                StringStack.push_back(Exp->GetVarName());
            }

            template <typename E, template <typename> class S>
            inline void 
            Stringifier<E, S>::VisitConstExpression(const ConstExpression<E, S>* Exp)
            {
                StringStack.push_back(Exp->GetConstValue());
            }

            template <typename E, template <typename> class S>
            inline void 
            Stringifier<E, S>::VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp)
            {
                StringStack.push_back(BoundVarPrefix + to_string(Exp->GetVarIdx()));
            }

            template <typename E, template <typename> class S>
            inline void 
            Stringifier<E, S>::VisitOpExpression(const OpExpression<E, S>* Exp)
            {
                ExpressionVisitorBase<E, S>::VisitOpExpression(Exp);

                auto const OpCode = Exp->GetOpCode();

                if (OpCode == LTSOps::OpIndex) {
                    string Retval;
                    Retval = StringStack.back();
                    StringStack.pop_back();
                    Retval = StringStack.back() + "[" + Retval + "]";
                    StringStack.pop_back();
                    StringStack.push_back(Retval);
                    return;
                }

                if (OpCode == LTSOps::OpField) {
                    string Retval;
                    Retval = StringStack.back();
                    StringStack.pop_back();
                    Retval = StringStack.back() + "." + Retval;
                    StringStack.pop_back();
                    StringStack.push_back(Retval);
                    return;
                }

                auto it = OpCodeToNameMap.find(Exp->GetOpCode());
                string OpString;
                if (it != OpCodeToNameMap.end()) {
                    OpString = it->second;
                } else {
                    auto it2 = UFMap.find(Exp->GetOpCode());
                    if (it2 == UFMap.end()) {
                        throw InternalError((string)"Could not find operator with code " + 
                                            to_string(Exp->GetOpCode()));
                    }
                    OpString = it2->second->template As<ExprFuncType>()->GetName();
                }

                string Retval = (string)"(" + OpString;
                const u32 NumChildren = Exp->GetChildren().size();
                vector<string> SubExps(NumChildren);

                for (u32 i = 0; i < NumChildren; ++i) {
                    SubExps[NumChildren - i - 1] = StringStack.back();
                    StringStack.pop_back();
                }
                for (auto const& SubExp : SubExps) {
                    Retval += (" " + SubExp);
                }
                Retval += ")";
                StringStack.push_back(Retval);
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
            Stringifier<E, S>::VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp)
            {
                VisitQuantifiedExpression(Exp);
            }

            template <typename E, template <typename> class S>
            inline void 
            Stringifier<E, S>::VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp)
            {
                VisitQuantifiedExpression(Exp);
            }
            
            template <typename E, template <typename> class S>
            inline string
            Stringifier<E, S>::Do(const ExpT& Exp, const UFID2TypeMapT& UFMap)
            {
                Stringifier<E, S> TheStringifier(UFMap);
                Exp->Accept(&TheStringifier);
                return TheStringifier.StringStack[0];
            }

            template <typename E, template <typename> class S>
            inline string
            Stringifier<E, S>::TypeToString(const TypeT& Type)
            {
                return Type->ToString();
            }

            
            // Implementation of Canonicalizer
            template <typename E, template <typename> class S>
            inline Canonicalizer<E, S>::Canonicalizer()
                : ExpressionVisitorBase<E, S>("LTSTermCanonicalizer")
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            inline Canonicalizer<E, S>::~Canonicalizer()
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            inline void
            Canonicalizer<E, S>::VisitVarExpression(const VarExpression<E, S>* Exp)
            {
                ExpStack.push_back(Exp);
            }

            template <typename E, template <typename> class S>
            inline void
            Canonicalizer<E, S>::VisitConstExpression(const ConstExpression<E, S>* Exp)
            {
                ExpStack.push_back(Exp);
            }

            template <typename E, template <typename> class S>
            inline void
            Canonicalizer<E, S>::VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp)
            {
                ExpStack.push_back(Exp);
            }

            template <typename E, template <typename> class S>
            inline void
            Canonicalizer<E, S>::VisitOpExpression(const OpExpression<E, S>* Exp)
            {
                ExpressionVisitorBase<E, S>::VisitOpExpression(Exp);
                
                auto const OpCode = Exp->GetOpCode();
                const u32 NumChildren = Exp->GetChildren().size();
                
                vector<ExpT> NewChildren(NumChildren);
                for (u32 i = 0; i < NumChildren; ++i) {
                    NewChildren[NumChildren - i - 1] = ExpStack.back();
                    ExpStack.pop_back();
                }

                switch (OpCode) {
                case LTSOps::OpEQ:
                case LTSOps::OpOR:
                case LTSOps::OpAND:
                case LTSOps::OpIFF:
                case LTSOps::OpXOR:
                case LTSOps::OpADD:
                case LTSOps::OpMUL:
                    sort(NewChildren.begin(), NewChildren.end(), ExpressionPtrCompare());
                    ExpStack.push_back(new OpExpression<E, S>(nullptr, OpCode, 
                                                              NewChildren, Exp->ExtensionData));
                    break;

                case LTSOps::OpNOT:
                    if (NewChildren[0]->template As<OpExpression>() != nullptr &&
                        NewChildren[0]->template 
                        SAs<OpExpression>()->GetOpCode() == LTSOps::OpNOT) {

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
            Canonicalizer<E, S>::VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto NewQExpr = ExpStack.back();
                ExpStack.pop_back();
                if (Exp->IsForAll()) {
                    ExpStack.push_back(new AQuantifiedExpression<E, S>(nullptr, Exp->GetQVarTypes(),
                                                                       NewQExpr, 
                                                                       Exp->ExtensionData));
                } else {
                    ExpStack.push_back(new EQuantifiedExpression<E, S>(nullptr, Exp->GetQVarTypes(),
                                                                       NewQExpr, 
                                                                       Exp->ExtensionData));
                }
            }

            template <typename E, template <typename> class S>
            inline void
            Canonicalizer<E, S>::VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp)
            {
                VisitQuantifiedExpression(Exp);
            }

            template <typename E, template <typename> class S>
            inline void
            Canonicalizer<E, S>::VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp)
            {
                VisitQuantifiedExpression(Exp);
            }

            template <typename E, template <typename> class S>
            inline typename Canonicalizer<E, S>::ExpT
            Canonicalizer<E, S>::Do(const ExpT& Exp)
            {
                Canonicalizer<E, S> TheCanonicalizer;
                Exp->Accept(&TheCanonicalizer);
                return TheCanonicalizer.ExpStack[0];
            }
            
            // Implementation of Simplifier
            template <typename E, template <typename> class S>
            inline Simplifier<E, S>::Simplifier(const ExprTypeRef& BoolType,
                                                const ExprTypeRef& IntType)
                : ExpressionVisitorBase<E, S>("LTSTermSimplifier"),
                  BoolType(BoolType), IntType(IntType)
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            inline Simplifier<E, S>::~Simplifier()
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            inline void
            Simplifier<E, S>::VisitVarExpression(const VarExpression<E, S>* Exp)
            {
                ExpStack.push_back(Exp);
            }

            template <typename E, template <typename> class S>
            inline void
            Simplifier<E, S>::VisitConstExpression(const ConstExpression<E, S>* Exp)
            {
                ExpStack.push_back(Exp);
            }

            template <typename E, template <typename> class S>
            inline void
            Simplifier<E, S>::VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp)
            {
                ExpStack.push_back(Exp);
            }

            template<typename E, template <typename> class S>
            static inline bool CheckAllConstant(const vector<Expr<E, S>>& ExpVec)
            {
                return (all_of(ExpVec.begin(), ExpVec.end(),
                               [] (const Expr<E, S>& Exp) -> bool
                               { return (Exp->template As<ConstExpression>() != nullptr); }));
            }

            template <typename E, template <typename> class S>
            static inline vector<Expr<E, S>> PurgeBool(const vector<Expr<E, S>>& ExpVec, 
                                                       const bool Value, const ExprTypeRef& BoolType)
            {
                vector<Expr<E, S>> Retval;
                const string PurgeString = Value ? "true" : "false";
                for (auto const& Exp : ExpVec) {
                    if (Exp->template As<ConstExpression>() != nullptr &&
                        Exp->template SAs<ConstExpression>()->GetConstType() == BoolType &&
                        Exp->template SAs<ConstExpression>()->GetConstValue() == PurgeString) {
                        continue;
                    }
                    Retval.push_back(Exp);
                }

                return Retval;
            }

            template <typename E, template <typename> class S>
            static inline vector<Expr<E, S>> PurgeInt(const vector<Expr<E, S>>& ExpVec, i64 Value)
            {
                vector<Expr<E, S>> Retval;
                for (auto const& Exp : ExpVec) {
                    if (Exp->template As<ConstExpression>() != nullptr) {
                        auto const& Val = Exp->template SAs<ConstExpression>()->GetConstValue();
                        auto ActVal = boost::lexical_cast<boost::multiprecision::cpp_int>(Val);
                        if (ActVal == Value) {
                            continue;
                        }
                    }
                    Retval.push_back(Exp);
                }
                return Retval;
            }


            template <typename E, template <typename> class S>
            static inline bool HasInt(const vector<Expr<E, S>>& ExpVec, i64 Value)
            {
                vector<Expr<E, S>> Retval;
                for (auto const& Exp : ExpVec) {
                    if (Exp->template As<ConstExpression>() != nullptr) {
                        auto const& Val = Exp->template SAs<ConstExpression>()->GetConstValue();
                        auto ActVal = boost::lexical_cast<boost::multiprecision::cpp_int>(Val);
                        if (ActVal == Value) {
                            return true;
                        }
                    }
                }
                return false;
            }

            template <typename E, template <typename> class S>
            static inline bool HasBool(const vector<Expr<E, S>>& ExpVec, 
                                       const bool Value, const ExprTypeRef& BoolType)
            {
                const string MatchString = Value ? "true" : "false";
                for (auto const& Exp : ExpVec) {
                    if (Exp->template As<ConstExpression>() != nullptr &&
                        Exp->template SAs<ConstExpression>()->GetConstType() == BoolType &&
                        Exp->template SAs<ConstExpression>()->GetConstValue() == MatchString) {
                        return true;
                    }
                }

                return false;
            }

            template <typename E, template <typename> class S>
            static inline Expr<E, S> MakeOpExp(i64 OpCode, const vector<Expr<E, S>>& Children,
                                               const E& ExtData)
            {
                return (new OpExpression<E, S>(nullptr, OpCode, Children, ExtData));
            }

            template <typename E, template <typename> class S>
            inline void
            Simplifier<E, S>::VisitOpExpression(const OpExpression<E, S>* Exp)
            {
                ExpressionVisitorBase<E, S>::VisitOpExpression(Exp);
                
                const u32 NumChildren = Exp->GetChildren().size();
                auto const OpCode = Exp->GetOpCode();
                vector<ExpT> SimpChildren(NumChildren);
                auto const& ExtData = Exp->ExtensionData;
                
                for(u32 i = 0; i < NumChildren; ++i) {
                    SimpChildren[NumChildren - i - 1] = ExpStack.back();
                    ExpStack.pop_back();
                }

                if (OpCodeToNameMap.find(OpCode) == OpCodeToNameMap.end()) {
                    ExpStack.push_back(MakeOpExp(OpCode, SimpChildren, ExtData));
                    return;
                }

                switch(OpCode) {
                case LTSOps::OpEQ:
                case LTSOps::OpIFF:
                    if (!CheckAllConstant(SimpChildren)) {
                        if (SimpChildren[0]->Equals(SimpChildren[1])) {
                            ExpStack.push_back(new ConstExpression<E, S>(nullptr, "true", BoolType));
                        } else {
                            ExpStack.push_back(MakeOpExp(OpCode, SimpChildren, ExtData));
                        }
                    } else {
                        auto const& Val1 = 
                            SimpChildren[0]->template SAs<ConstExpression>()->GetConstValue();
                        auto const& Val2 = 
                            SimpChildren[1]->template SAs<ConstExpression>()->GetConstValue();
                        auto Result = (Val1 == Val2 ? "true" : "false");
                        ExpStack.push_back(new ConstExpression<E, S>(nullptr, Result, BoolType));
                    }
                    break;

                case LTSOps::OpNOT:
                    if (!CheckAllConstant(SimpChildren)) {
                        ExpStack.push_back(MakeOpExp(OpCode, SimpChildren, ExtData));
                    } else {
                        auto const& Val = 
                            SimpChildren[0]->template SAs<ConstExpression>()->GetConstValue();
                        auto Result = (Val == "true" ? "false" : "true");
                        ExpStack.push_back(new ConstExpression<E, S>(nullptr, Result, BoolType));
                    }

                case LTSOps::OpITE:
                    if (SimpChildren[1]->Equals(SimpChildren[2])) {
                        ExpStack.push_back(SimpChildren[1]);
                    } else if (SimpChildren[0]->template As<ConstExpression>() != nullptr) {
                        if (SimpChildren[0]->template 
                            SAs<ConstExpression>()->GetConstValue() == "true") {
                            ExpStack.push_back(SimpChildren[1]);
                        } else {
                            ExpStack.push_back(SimpChildren[2]);
                        }
                    } else {
                        ExpStack.push_back(MakeOpExp(OpCode, SimpChildren, ExtData));
                    }
                    break;

                case LTSOps::OpOR:
                    if (HasBool(SimpChildren, true, BoolType)) {
                        ExpStack.push_back(new ConstExpression<E, S>(nullptr, "true", BoolType));
                    } else {
                        auto&& RedChildren = PurgeBool(SimpChildren, false, BoolType);
                        if (RedChildren.size() == 0) {
                            ExpStack.push_back(new ConstExpression<E, S>(nullptr, "false", BoolType));
                        } else if (RedChildren.size() == 1) {
                            ExpStack.push_back(RedChildren[0]);
                        } else {
                            ExpStack.push_back(MakeOpExp(OpCode, RedChildren, ExtData));
                        }
                    }
                    break;

                case LTSOps::OpAND:
                    if (HasBool(SimpChildren, false, BoolType)) {
                        ExpStack.push_back(new ConstExpression<E, S>(nullptr, "false", BoolType));
                    } else {
                        auto&& RedChildren = PurgeBool(SimpChildren, true, BoolType);
                        if (RedChildren.size() == 0) {
                            ExpStack.push_back(new ConstExpression<E, S>(nullptr, "true", BoolType));
                        } else if (RedChildren.size() == 1) {
                            ExpStack.push_back(RedChildren[0]);
                        } else {
                            ExpStack.push_back(MakeOpExp(OpCode, RedChildren, ExtData));
                        }
                    }
                    break;

                case LTSOps::OpIMPLIES:
                    if (SimpChildren[0]->template As<ConstExpression>() != nullptr) {
                        auto Ant = SimpChildren[0]->template SAs<ConstExpression>();
                        auto const& AntVal = Ant->GetConstValue();
                        if (AntVal == "false") {
                            ExpStack.push_back(new ConstExpression<E, S>(nullptr, "true", BoolType));
                        } else {
                            ExpStack.push_back(SimpChildren[1]);
                        }
                    } else if (SimpChildren[1]->template As<ConstExpression>() != nullptr) {
                        auto Con = SimpChildren[1]->template SAs<ConstExpression>();
                        auto const& ConVal = Con->GetConstValue();
                        if (ConVal == "false") {
                            vector<ExpT> NewChildren;
                            NewChildren.push_back(SimpChildren[0]);
                            ExpStack.push_back(new OpExpression<E, S>(nullptr, LTSOps::OpNOT, 
                                                                      NewChildren));
                        } else {
                            ExpStack.push_back(SimpChildren[1]);
                        }
                    } else {
                        ExpStack.push_back(MakeOpExp(OpCode, SimpChildren, ExtData));
                    }
                    break;

                case LTSOps::OpXOR:
                    if (CheckAllConstant(SimpChildren)) {
                        auto AExp = SimpChildren[0]->template SAs<ConstExpression>();
                        auto BExp = SimpChildren[0]->template SAs<ConstExpression>();
                        
                        auto AVal = AExp->GetConstValue();
                        auto BVal = BExp->GetConstValue();

                        if (AVal == BVal) {
                            ExpStack.push_back(new ConstExpression<E, S>(nullptr, "false", BoolType));
                        } else {
                            ExpStack.push_back(new ConstExpression<E, S>(nullptr, "true", BoolType));
                        }
                    }
                    
                    if (SimpChildren[0]->template As<ConstExpression>() != nullptr) {
                        auto AExp = SimpChildren[0]->template SAs<ConstExpression>();
                        auto const& AVal = AExp->GetConstValue();
                        if (AVal == "true") {
                            vector<ExpT> NewChildren;
                            NewChildren.push_back(SimpChildren[1]);
                            ExpStack.push_back(new OpExpression<E, S>(nullptr, LTSOps::OpNOT, NewChildren));
                        } else {
                            ExpStack.push_back(SimpChildren[1]);
                        }
                    } else if (SimpChildren[1]->template As<ConstExpression>() != nullptr) {
                        auto BExp = SimpChildren[1]->template SAs<ConstExpression>();
                        auto const& BVal = BExp->GetConstValue();
                        if (BVal == "true") {
                            vector<ExpT> NewChildren;
                            NewChildren.push_back(SimpChildren[0]);
                            ExpStack.push_back(new OpExpression<E, S>(nullptr, LTSOps::OpNOT, NewChildren));
                        } else {
                            ExpStack.push_back(SimpChildren[0]);
                        }
                    } else {
                        ExpStack.push_back(MakeOpExp(OpCode, SimpChildren, ExtData));
                    }
                    break;

                case LTSOps::OpADD:
                    if (CheckAllConstant(SimpChildren)) {
                        i64 SumVal = 0;
                        for (auto const& Exp : SimpChildren) {
                            auto Val = boost::lexical_cast<i64>(Exp->template 
                                                                SAs<ConstExpression>()->GetConstValue());
                            SumVal += Val;
                        }
                        ExpStack.push_back(new ConstExpression<E, S>(nullptr, to_string(SumVal), IntType));
                    } else {
                        auto&& RedChildren = PurgeInt(SimpChildren, 0);
                        if (RedChildren.size() == 1) {
                            ExpStack.push_back(RedChildren[0]);
                        } else {
                            ExpStack.push_back(MakeOpExp(OpCode, RedChildren, ExtData));
                        }
                    }
                    break;

                case LTSOps::OpSUB:
                    if (CheckAllConstant(SimpChildren)) {
                        i64 DiffVal = (boost::lexical_cast<i64>(SimpChildren[0]->template
                                                                SAs<ConstExpression>()->GetConstValue()) -
                                       boost::lexical_cast<i64>(SimpChildren[1]->template 
                                                                SAs<ConstExpression>()->GetConstValue()));
                        ExpStack.push_back(new ConstExpression<E, S>(nullptr, to_string(DiffVal), IntType));
                    } else {
                        if (SimpChildren[1]->template As<ConstExpression>() != nullptr &&
                            boost::lexical_cast<i64>(SimpChildren[1]->template 
                                                     SAs<ConstExpression>()->GetConstValue()) == 0) {
                            ExpStack.push_back(SimpChildren[0]);
                        } else {
                            ExpStack.push_back(MakeOpExp(OpCode, SimpChildren, ExtData));
                        }
                    }
                    break;

                case LTSOps::OpMINUS:
                    if (CheckAllConstant(SimpChildren)) {
                        i64 Val = boost::lexical_cast<i64>(SimpChildren[0]->template
                                                           SAs<ConstExpression>()->GetConstValue());
                        Val = -Val;
                        ExpStack.push_back(new ConstExpression<E, S>(nullptr, to_string(Val), IntType));
                    } else {
                        ExpStack.push_back(MakeOpExp(OpCode, SimpChildren, ExtData));
                    }
                    break;

                case LTSOps::OpMUL:
                    if (CheckAllConstant(SimpChildren)) {
                        i64 ProdVal = 1;
                        for (auto const& Exp : SimpChildren) {
                            auto Val = boost::lexical_cast<i64>(Exp->template 
                                                                SAs<ConstExpression>()->GetConstValue());
                            ProdVal *= Val;
                        }
                        ExpStack.push_back(new ConstExpression<E, S>(nullptr, to_string(ProdVal), IntType));
                    } else {
                        if (HasInt(SimpChildren, 0)) {
                            ExpStack.push_back(new ConstExpression<E, S>(nullptr, "0", IntType));
                        } else {
                            auto&& RedChildren = PurgeInt(SimpChildren, 1);
                            if (RedChildren.size() == 1) {
                                ExpStack.push_back(RedChildren[0]);
                            } else {
                                ExpStack.push_back(MakeOpExp(OpCode, RedChildren, ExtData));
                            }
                        }
                    }
                    break;

                case LTSOps::OpDIV:
                case LTSOps::OpMOD:
                    if (CheckAllConstant(SimpChildren)) {
                        i64 DivVal;
                        if (OpCode == LTSOps::OpDIV) {
                            DivVal = (boost::lexical_cast<i64>(SimpChildren[0]->template 
                                                               SAs<ConstExpression>()->GetConstValue()) /
                                      boost::lexical_cast<i64>(SimpChildren[1]->template
                                                               SAs<ConstExpression>()->GetConstValue()));
                        } else {
                            DivVal = (boost::lexical_cast<i64>(SimpChildren[0]->template 
                                                               SAs<ConstExpression>()->GetConstValue()) %
                                      boost::lexical_cast<i64>(SimpChildren[1]->template
                                                               SAs<ConstExpression>()->GetConstValue()));
                            
                        }
                        ExpStack.push_back(new ConstExpression<E, S>(nullptr, to_string(DivVal), IntType));
                    } else {
                        if (SimpChildren[1]->template As<ConstExpression>() != nullptr) {
                            auto Val = boost::lexical_cast<i64>(SimpChildren[1]->template 
                                                                SAs<ConstExpression>()->GetConstValue());
                            if (Val == 0) {
                                throw ExprTypeError("Division by zero during simplification");
                            } else if (Val == 1) {
                                if (OpCode == LTSOps::OpMOD) {
                                    ExpStack.push_back(new ConstExpression<E, S>(nullptr, "0", IntType));
                                } else {
                                    ExpStack.push_back(SimpChildren[0]);
                                }
                            }
                        } else {
                            ExpStack.push_back(MakeOpExp(OpCode, SimpChildren, ExtData));
                        }
                    }
                    break;
                    
                case LTSOps::OpGT:
                case LTSOps::OpGE:
                case LTSOps::OpLT:
                case LTSOps::OpLE:
                    if (CheckAllConstant(SimpChildren)) {
                        auto Val1 = boost::lexical_cast<i64>(SimpChildren[0]->template
                                                             SAs<ConstExpression>()->GetConstValue());
                        auto Val2 = boost::lexical_cast<i64>(SimpChildren[1]->template
                                                             SAs<ConstExpression>()->GetConstValue());
                        string ResString;
                        if (OpCode == LTSOps::OpGT) {
                            ResString = Val1 > Val2 ? "true" : "false"; 
                        } else if (OpCode == LTSOps::OpGE) {
                            ResString = Val1 >= Val2 ? "true" : "false"; 
                        } else if (OpCode == LTSOps::OpLT) {
                            ResString = Val1 < Val2 ? "true" : "false"; 
                        } else /* if (OpCode == LTSOps::OpLE) */ {
                            ResString = Val1 <= Val2 ? "true" : "false"; 
                        }
                        
                        ExpStack.push_back(new ConstExpression<E, S>(nullptr, ResString, BoolType));
                    } else {
                        ExpStack.push_back(MakeOpExp(OpCode, SimpChildren, ExtData));
                    }
                    break;

                default:
                    ExpStack.push_back(MakeOpExp(OpCode, SimpChildren, ExtData));

                }
            }

            template <typename E, template <typename> class S>
            inline void
            Simplifier<E, S>::VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto NewQExpr = ExpStack.back();
                ExpStack.pop_back();
                if (Exp->IsForAll()) {
                    ExpStack.push_back(new AQuantifiedExpression<E, S>(nullptr, Exp->GetQVarTypes(),
                                                                       NewQExpr, Exp->ExtensionData));
                } else {
                    ExpStack.push_back(new EQuantifiedExpression<E, S>(nullptr, Exp->GetQVarTypes(),
                                                                       NewQExpr, Exp->ExtensionData));
                }
            }

            template <typename E, template <typename> class S>
            inline void
            Simplifier<E, S>::VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp)
            {
                VisitQuantifiedExpression(Exp);
            }

            template <typename E, template <typename> class S>
            inline void
            Simplifier<E, S>::VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp)
            {
                VisitQuantifiedExpression(Exp);
            }

            template <typename E, template <typename> class S>
            inline typename Simplifier<E, S>::ExpT 
            Simplifier<E, S>::Do(const ExpT& Exp, const ExprTypeRef& BoolType,
                                 const ExprTypeRef& IntType)
            {
                Simplifier<E, S> TheSimplifier(BoolType, IntType);
                Exp->Accept(&TheSimplifier);
                return TheSimplifier.ExpStack[0];
            }

        } /* end namespace Detail */
 
        template <typename E>
        class LTSTermSemanticizer
        {
        private:
            typedef typename Exprs::ExprMgr<E, LTS::LTSTermSemanticizer> MgrType;

            Detail::UFID2TypeMapT UFMap;
            unordered_map<string, i64> UFNameToIDMap;
            UIDGenerator UFUIDGen;

            MgrType* Mgr;
            Exprs::ExprTypeRef BoolType;
            Exprs::ExprTypeRef IntType;
            
        public:
            typedef LTSOps Ops;
            typedef Exprs::Expr<E, ESMC::LTS::LTSTermSemanticizer> ExpT;
            typedef void* LExpT;
            typedef Exprs::ExprTypeRef TypeT;
            static const TypeT InvalidType;

            LTSTermSemanticizer(MgrType* Mgr, const TypeT& BoolType, const TypeT& IntType);
            ~LTSTermSemanticizer();

            inline void TypeCheck(const ExpT& Exp) const;
            inline ExpT Canonicalize(const ExpT& Exp);
            // This is constant propagation
            inline ExpT Simplify(const ExpT& Exp);
            inline string ExprToString(const ExpT& Exp) const;
            inline string TypeToString(i64 Type) const;
            inline i64 RegisterUninterpretedFunction(const string& Name,
                                                     const vector<TypeT>& DomTypes,
                                                     const TypeT& RangeType);

            // Functions which DO NOT have an implementation
            inline ExpT RaiseExpr(const LExpT& LExp);
            inline LExpT LowerExpr(const ExpT& Exp);
            inline ExpT ElimQuantifiers(const ExpT& Exp);
        };

        
        // Implementation of LTSTermSemanticizer

        template <typename E>
        LTSTermSemanticizer<E>::LTSTermSemanticizer(MgrType* Mgr,
                                                    const Exprs::ExprTypeRef& BoolType,
                                                    const Exprs::ExprTypeRef& IntType)
            : UFUIDGen(LTSOps::UFOffset), Mgr(Mgr), BoolType(BoolType), IntType(IntType)
        {
            // Nothing here
        }

        template <typename E>
        LTSTermSemanticizer<E>::~LTSTermSemanticizer()
        {
            // Nothing here
        }

        template <typename E>
        inline void LTSTermSemanticizer<E>::TypeCheck(const ExpT& Exp) const
        {
            Detail::TypeChecker<E, LTS::LTSTermSemanticizer>::Do(Exp, UFMap, BoolType, IntType);
        }

        template <typename E>
        inline typename LTSTermSemanticizer<E>::ExpT
        LTSTermSemanticizer<E>::Canonicalize(const ExpT& Exp)
        {
            return Detail::Canonicalizer<E, LTS::LTSTermSemanticizer>::Do(Exp);
        }

        template <typename E>
        inline typename LTSTermSemanticizer<E>::ExpT
        LTSTermSemanticizer<E>::Simplify(const ExpT& Exp)
        {
            return Detail::Simplifier<E, LTS::LTSTermSemanticizer>::Do(Exp, BoolType, IntType);
        }
        
        template <typename E>
        inline string
        LTSTermSemanticizer<E>::ExprToString(const ExpT& Exp) const
        {
            return Detail::Stringifier<E, LTS::LTSTermSemanticizer>::Do(Exp, UFMap);
        }

        template <typename E>
        inline string
        LTSTermSemanticizer<E>::TypeToString(i64 TypeID) const
        {
            return Detail::Stringifier<E, LTS::LTSTermSemanticizer>::TypeToString(TypeID);
        }

        static inline string MangleName(const string& Name,
                                        const vector<Exprs::ExprTypeRef> ArgTypes)
        {
            string Retval = Name;
            for (auto const& Arg : ArgTypes) {
                Retval += ((string)"@" + to_string(Arg->GetTypeID()));
            }
            return Retval;
        }

        template <typename E>
        inline i64 
        LTSTermSemanticizer<E>::RegisterUninterpretedFunction(const string& Name,
                                                              const vector<TypeT>& DomTypes,
                                                              const TypeT& RangeType)
        {
            auto MangledName = MangleName(Name, DomTypes);
            // We only support scalar functions currently
            if (!all_of(DomTypes.begin(), DomTypes.end(),
                        [](const TypeT& Type) -> bool
                        { return (Type->template Is<Exprs::ExprBoolType>() ||
                                  Type->template Is<Exprs::ExprIntType>() ||
                                  Type->template Is<Exprs::ExprRangeType>() ||
                                  Type->template Is<Exprs::ExprEnumType>() ||
                                  Type->template Is<Exprs::ExprSymmetricType>()); })) {
                throw ESMCError((string)"Only functions from scalars -> scalars are " +
                                "currently supported");
            }
            
            if (UFNameToIDMap.find(MangledName) == UFNameToIDMap.end()) {
                auto Type = Mgr->template MakeType<Exprs::ExprFuncType>(Name, DomTypes, RangeType);
                auto UFID = UFUIDGen.GetUID();
                UFNameToIDMap[MangledName] = UFID;
                UFMap[UFID] = Type;
                return UFID;
            } else {
                auto it = UFNameToIDMap.find(MangledName);
                auto UFID = it->second;
                auto it2 = UFMap.find(UFID);
                assert(it2 != UFMap.end());
                                
                auto Type = it2->second->template As<Exprs::ExprFuncType>();

                assert (Type != nullptr);
                if (Type->GetFuncType() != RangeType) {
                    throw Exprs::ExprTypeError((string)"Redeclaration of function " + 
                                               Name + 
                                               " with variant return type");
                }
                return UFID;
            }
        }

        template <typename E>
        inline typename LTSTermSemanticizer<E>::ExpT
        LTSTermSemanticizer<E>::RaiseExpr(const LExpT& LExp)
        {
            throw ESMCError((string)"RaiseExpr() not implemented in LTSTermSemanticizer");
        }

        template <typename E>
        inline typename LTSTermSemanticizer<E>::LExpT
        LTSTermSemanticizer<E>::LowerExpr(const ExpT& Exp)
        {
            throw ESMCError((string)"LowerExpr() not implemented in LTSTermSemanticizer");
        }

        template <typename E>
        inline typename LTSTermSemanticizer<E>::ExpT
        LTSTermSemanticizer<E>::ElimQuantifiers(const ExpT& Exp)
        {
            throw ESMCError((string)"ElimQuantifiers() not implemented in LTSTermSemanticizer");
        }

        template<typename E>
        const typename LTSTermSemanticizer<E>::TypeT LTSTermSemanticizer<E>::InvalidType = 
            LTSTermSemanticizer<E>::TypeT::NullPtr;

        // A helper routine to check if an expression is an LVAlue
        template <typename E, template <typename> class S>
        static inline bool IsLVal(const ESMC::Exprs::Expr<E, S>& Exp)
        {
            auto ExpAsVar = Exp->template As<ESMC::Exprs::VarExpression>();
            if (ExpAsVar != nullptr) {
                return true;
            }
            auto ExpAsOp = Exp->template As<ESMC::Exprs::OpExpression>();
            if (ExpAsOp != nullptr) {
                auto Op = ExpAsOp->GetOpCode();
                if (Op != LTSOps::OpIndex &&
                    Op != LTSOps::OpField) {
                    return false;
                } else {
                    return IsLVal(ExpAsOp->GetChildren()[0]);
                }
            }
            return false;
        }

        
    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_TERM_SEMANTICIZER_HPP_ */

// 
// LTSTermSemanticizer.hpp ends here
