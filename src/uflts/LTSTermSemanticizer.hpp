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

#include "LTSTypes.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/multiprecision/cpp_int.hpp>

#include <unordered_map>
#include <set>
#include <vector>

namespace ESMC {
    namespace LTS {

        namespace Detail {
            
            using namespace ESMC::Exprs;
            extern const unordered_map<i64, string> OpCodeToNameMap;
            extern const string BoundVarPrefix;

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
                static const i64 UndefType = 1000000001;
            };

            typedef unordered_map<i64, SmartPtr<LTSTypeBase>> UFID2TypeMapT;
            typedef unordered_map<i64, SmartPtr<LTSTypeBase>> IDToTypeMapT;

            template <typename E, template <typename> class S>
            class TypeChecker : public ExpressionVisitorBase<E, S>
            {
            private:
                const UFID2TypeMapT& UFMap;
                const IDToTypeMapT& TypeMap;
                typedef Expr<E, S> ExpT;

                vector<vector<i64>> ScopeStack;

                inline void VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp);

            public:
                TypeChecker(const UFID2TypeMapT& UFMap, const IDToTypeMapT& TypeMap);
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
                                      const IDToTypeMapT& TypeMap);
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
                const IDToTypeMapT& TypeMap;

                inline void VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp);

            public:
                Stringifier(const UFID2TypeMapT& UFMap, const IDToTypeMapT& TypeMap);
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
                
                static inline string Do(const ExpT& Exp, const UFID2TypeMapT& UFMap,
                                        const IDToTypeMapT& TypeMap);
                static inline string TypeToString(i64 Type, const IDToTypeMapT& TypeMap);
            };

            template <typename E, template <typename> class S>
            class Simplifier : public ExpressionVisitorBase<E, S>
            {
            private:
                typedef Expr<E, S> ExpT;
                vector<ExpT> ExpStack;

                inline void VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp);

            public:
                Simplifier();
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
                
                static inline ExpT Do(const ExpT& Exp);
            };

            // Implementation of type checker
            template <typename E, template <typename> class S>
            TypeChecker<E, S>::TypeChecker(const UFID2TypeMapT& UFMap,
                                           const IDToTypeMapT& TypeMap)
                : ExpressionVisitorBase<E, S>("LTSTermTypeChecker"),
                  UFMap(UFMap), TypeMap(TypeMap)
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
                if (PrevType != -1) {
                    return;
                }

                auto Type = Exp->GetVarType();
                if (Type != LTSOps::FieldAccType &&
                    TypeMap.find(Type) == TypeMap.end()) {
                    throw ExprTypeError("Unknown type in var expression");
                }
                Exp->SetType(Type);
            }

            template <typename E, template <typename> class S>
            inline void TypeChecker<E, S>::VisitConstExpression(const ConstExpression<E, S>* Exp)
            {
                auto PrevType = Exp->GetType();
                if (PrevType != -1) {
                    return;
                }
                auto ConstVal = Exp->GetConstValue();
                boost::algorithm::trim(ConstVal);

                auto TypeID = Exp->GetConstType();

                if (TypeID == LTSOps::UndefType) {
                    boost::algorithm::to_lower(ConstVal);
                    if (ConstVal != "undef") {
                        throw ExprTypeError("Illegal Undefined constant");
                    } else {
                        Exp->SetType(TypeID);
                        return;
                    }
                }

                auto it = TypeMap.find(TypeID);
                if (it == TypeMap.end()) {
                    throw ExprTypeError("Unknown type in const expression");
                }
                auto ActType = it->second;
                if (ActType->template As<LTSBoolType>() != nullptr) {
                    boost::algorithm::to_lower(ConstVal);
                    if (ConstVal != "true" && ConstVal != "false") {
                        throw ExprTypeError("Unknown constant string");
                    }
                } else if(ActType->template As<LTSRangeType>() != nullptr ||
                          ActType->template As<LTSIntType>() != nullptr) {
                    auto IntType = ActType->template SAs<LTSIntType>();
                    if (!boost::algorithm::all(ConstVal, boost::algorithm::is_digit())) {
                        throw ExprTypeError((string)"Invalid value " + ConstVal);
                    }
                    boost::multiprecision::cpp_int Val = 0;
                    Val = boost::lexical_cast<boost::multiprecision::cpp_int>(ConstVal);
                    if (ActType->template As<LTSRangeType>() != nullptr) {
                        auto RangeType = ActType->template SAs<LTSRangeType>();

                        if (Val < RangeType->GetLow() ||
                            Val > RangeType->GetHigh()) {
                            throw ExprTypeError((string)"Value " + ConstVal + " is out of bounds");
                        }
                    }
                    Exp->SetType(TypeID);
                } else if(ActType->template As<LTSEnumType>() != nullptr) {
                    auto EnumType = ActType->template SAs<LTSEnumType>();
                    if (!EnumType->IsMember(ConstVal)) {
                        throw ExprTypeError((string)"Value " + ConstVal + " is not valid");
                    }
                    Exp->SetType(TypeID);
                } else if(ActType->template As<LTSSymmetricType>() != nullptr) {
                    auto SymmType = ActType->template SAs<LTSSymmetricType>();
                    if (!SymmType->IsMember(ConstVal)) {
                        throw ExprTypeError((string)"Value " + ConstVal + " is not valid");
                    }
                    Exp->SetType(TypeID);
                } else {
                    throw ExprTypeError((string)"Only boolean, range, enum and symmetric type " + 
                                        "constants are currently supported");
                }
            }

            template <typename E, template <typename> class S>
            inline void 
            TypeChecker<E, S>::VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp)
            {
                auto PrevType = Exp->GetType();
                if (PrevType != -1) {
                    return;
                }

                auto TypeID = Exp->GetVarType();
                auto it = TypeMap.find(TypeID);
                if (it == TypeMap.end()) {
                    throw ExprTypeError((string)"Unknown type " + to_string(TypeID));
                }

                auto Type = it->second;
                if (Type->template As<LTSBoolType>() != nullptr ||
                    Type->template As<LTSRangeType>() != nullptr ||
                    Type->template As<LTSSymmetricType>() != nullptr) {
                    auto Idx = Exp->GetVarIdx();
                    auto ExpectedType = SemUtils::GetTypeForBoundVar(ScopeStack, Idx);
                    if (ExpectedType != -1 && ExpectedType != TypeID) {
                        throw ExprTypeError((string)"Bound variable with index " + to_string(Idx) + 
                                            " has an ambiguous type");
                    }
                    Exp->SetType(TypeID);
                } else {
                    throw ExprTypeError((string)"Only boolean, symmetric, and range types " + 
                                        "may be used as quanfiers");
                }
            }

            static inline bool CheckTypeCompat(i64 ExpTypeID, i64 ActTypeID,
                                               const IDToTypeMapT& TypeMap)
            {
                if (ActTypeID == LTSOps::UndefType) {
                    return true;
                }
                auto it1 = TypeMap.find(ExpTypeID);
                auto it2 = TypeMap.find(ActTypeID);

                if (it2 == TypeMap.end() || it1 == TypeMap.end()) {
                    throw ExprTypeError((string)"Unknown type " + to_string(ActTypeID));
                }
                
                auto ExpType = it1->second;
                auto ActType = it2->second;

                if (ExpType == ActType) {
                    return true;
                } else {
                    if (ExpType->As<LTSIntType>() != nullptr &&
                        ActType->As<LTSIntType>() != nullptr) {
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
            static inline bool CheckCompatibility(const T& Collection, const V& Value,
                                                  const IDToTypeMapT& TypeMap)
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

            template <typename E, template <typename> class S>
            inline void 
            TypeChecker<E, S>::VisitOpExpression(const OpExpression<E, S>* Exp)
            {
                auto PrevType = Exp->GetType();
                if (PrevType != -1) {
                    return;
                }

                vector<i64> ChildTypes;
                for (auto const& Child : Exp->GetChildren()) {
                    Child->Accept(this);
                    ChildTypes.push_back(Child->GetType());
                }

                i64 OpCode = Exp->GetOpCode();

                switch(OpCode) {
                case LTSOps::OpEQ: {
                    CheckNumArgs(2, ChildTypes.size(), "=");
                    if (!CheckTypeCompat(ChildTypes[1], ChildTypes[0], TypeMap)) {
                        throw ExprTypeError("All types to = op must be the same");
                    }
                    // Assumption: The boolean type has type id 1
                    Exp->SetType(1);
                    break;
                }

                case LTSOps::OpNOT: {
                    CheckNumArgs(1, ChildTypes.size(), "not");
                    if (!CheckTypeCompat(1, ChildTypes[0], TypeMap)) {
                        throw ExprTypeError("not op can only be applied to boolean expressions");
                    }

                    Exp->SetType(1);
                    break;
                }

                case LTSOps::OpITE: {
                    CheckNumArgs(3, ChildTypes.size(), "ite");
                    if (!CheckTypeCompat(1, ChildTypes[0], TypeMap) || 
                        (!CheckTypeCompat(ChildTypes[1], ChildTypes[2], TypeMap))) {
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
                                [&] (i64 i) { return CheckTypeCompat(i, 1, TypeMap); })) {
                        throw ExprTypeError((string)"implies, xor, iff, and, or ops need " + 
                                            "all operands to be boolean");
                    }
                    Exp->SetType(1);
                    break;
                }
                case LTSOps::OpADD:
                case LTSOps::OpSUB:
                case LTSOps::OpMUL: {
                    if (ChildTypes.size() < 2) {
                        throw ExprTypeError((string)"add/sub/mul ops need at least two operands");
                    }
                    if (!all_of(ChildTypes.begin(), ChildTypes.end(),
                                [&] (i64 i) { return CheckTypeCompat(i, 2, TypeMap); })) {
                        throw ExprTypeError((string)"add/sub/mul ops need integer arguments");
                    }                    
                    // Assumption: Int type ALWAYS has ID 2
                    Exp->SetType(2);
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
                                [&] (i64 i) { return CheckTypeCompat(2, i, TypeMap); })) {
                        throw ExprTypeError("div/mod/cmp operators expect integer operands");
                    }
                    if (OpCode == LTSOps::OpDIV || OpCode == LTSOps::OpMUL) {
                        Exp->SetType(2);
                    } else {
                        Exp->SetType(1);
                    }
                    break;
                }
                case LTSOps::OpMINUS: {
                    CheckNumArgs(2, ChildTypes.size(), "minus");
                    if (!CheckTypeCompat(2, ChildTypes[0], TypeMap)) {
                        throw ExprTypeError("minus operator expects integer operands");
                    }
                    Exp->SetType(2);
                }
                case LTSOps::OpIndex: {
                    CheckNumArgs(2, ChildTypes.size(), "Index");
                    auto it1 = TypeMap.find(ChildTypes[0]);
                    if (it1 == TypeMap.end()) {
                        throw ExprTypeError("Invalid operands for index operator");
                    }
                    
                    auto Type1 = it1->second;

                    auto ArrayType = Type1->template As<LTSArrayType>();
                    if (ArrayType == nullptr) {
                        throw ExprTypeError("Index operator can only be applied to array types");
                    }
                    
                    auto ExpIndexType = ArrayType->GetIndexType();
                    if (!CheckTypeCompat(ExpIndexType->GetTypeID(), ChildTypes[1], TypeMap)) {
                        throw ExprTypeError("Invalid type for index expression");
                    }
                    auto ElemType = ArrayType->GetValueType();
                    Exp->SetType(ElemType->GetTypeID());
                    break;
                }
                case LTSOps::OpField: {
                    CheckNumArgs(2, ChildTypes.size(), "Field");

                    if (ChildTypes[1] != LTSOps::FieldAccType) {
                        throw ExprTypeError("Field accessor must be of FieldAccType");
                    }

                    auto it = TypeMap.find(ChildTypes[0]);
                    if (it == TypeMap.end()) {
                        throw ExprTypeError("Invalid types in Field operator");
                    }

                    auto RecType = it->second->template As<LTSRecordType>();
                    if (RecType == nullptr) {
                        throw ExprTypeError("Field access only allowed on record types");
                    }
                    
                    auto FieldExp = ((Exp->GetChildren())[0])->template As<VarExpression>();
                    if (FieldExp == nullptr) {
                        throw ExprTypeError("Field access expression must be a VarExpression");
                    }

                    auto ValType = RecType->GetTypeForMember(FieldExp->GetVarName());
                    if (ValType == LTSTypeRef::NullPtr) {
                        throw ExprTypeError((string)"Field name \"" + 
                                            FieldExp->GetVarName() + "\" invalid");
                    }
                    Exp->SetType(ValType->GetTypeID());
                    break;
                }
                default: {
                    // Must be an uninterpreted function
                    auto it = UFMap.find(OpCode);
                    if (it == UFMap.end()) {
                        throw ExprTypeError((string)"Unknown opcode " + to_string(OpCode));
                    }
                    auto FuncType = it->second->template As<LTSFuncType>();
                    auto const& ArgTypes = FuncType->GetArgTypes();
                    if (ChildTypes.size() != ArgTypes.size()) {
                        throw ExprTypeError((string)"Incorrect number of arguments for UF");
                    }
                    for (u32 i = 0; i < ChildTypes.size(); ++i) {
                        if (!CheckTypeCompat(ChildTypes[i], ArgTypes[i]->GetTypeID(),
                                             TypeMap)) {
                            throw ExprTypeError("Incorrect type for argument of UF");
                        }
                    }
                    Exp->SetType(FuncType->GetFuncType()->GetTypeID());
                }
                }
            }

            template <typename E, template <typename> class S>
            inline void 
            TypeChecker<E, S>::VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp)
            {
                auto const& QVarTypes = Exp->GetQVarTypes();
                if (!all_of(QVarTypes.begin(), QVarTypes.end(),
                            [&] (i64 TypeID) -> bool 
                            { return (TypeMap.find(TypeID) != TypeMap.end()); })) {
                    throw ExprTypeError("Unknown type in quanfied var binding");
                }
                ScopeStack.push_back(QVarTypes);
                auto const& QExpr = Exp->GetQExpression();
                SemUtils::TypeInvalidator<E, S> Inv;
                QExpr->Accept(&Inv);
                QExpr->Accept(this);

                if (QExpr->GetType() != 1) {
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
                                  const IDToTypeMapT& TypeMap)
            {
                TypeChecker Checker(UFMap, TypeMap);
                Exp->Accept(&Checker);
            }

            // Implementation of Stringifier
            template <typename E, template <typename> class S>
            Stringifier<E, S>::Stringifier(const UFID2TypeMapT& UFMap,
                                           const IDToTypeMapT& TypeMap)
                : ExpressionVisitorBase<E, S>("LTSTermStringifier"),
                  UFMap(UFMap), TypeMap(TypeMap)
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
                    OpString = it->second->template As<LTSFuncType>()->GetName();
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
                    ExpString += "(" + TypeToString(QVarType, TypeMap) + ")";
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
            Stringifier<E, S>::Do(const ExpT& Exp, const UFID2TypeMapT& UFMap,
                                  const IDToTypeMapT& TypeMap)
            {
                Stringifier<E, S> TheStringifier(UFMap, TypeMap);
                Exp->Accept(&TheStringifier);
                return TheStringifier.StringStack[0];
            }

            template <typename E, template <typename> class S>
            inline string
            Stringifier<E, S>::TypeToString(i64 TypeID, const IDToTypeMapT& TypeMap)
            {
                auto it = TypeMap.find(TypeID);
                if (it == TypeMap.end()) {
                    throw InternalError("Could not resolve type " + to_string(TypeID));
                }
                return it->second->ToString();
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
                ExpressionVisitorBase<E, S>::VisitOpExpression();
                
                auto const OpCode = Exp->GetOpCode();
                const u32 NumChildren = Exp->GetChildren().size();
                
                vector<ExpT> NewChildren(NumChildren);
                for (u32 i = 0; i < NumChildren; ++i) {
                    NewChildren[NumChildren - i - 1] = ExpStack.back();
                    ExpStack.push_back();
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
            inline Simplifier<E, S>::Simplifier()
                : ExpressionVisitorBase<E, S>("LTSTermSimplifier")
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
            static inline CheckAllConstant(const vector<Expr<E, S>>& ExpVec)
            {
                return (all_of(ExpVec.begin(), ExpVec.end(),
                               [] (const Expr<E, S>& Exp) -> bool
                               { return (Exp->template As<ConstExpression>() != nullptr); }));
            }

            template <typename E, template <typename> class S>
            static inline vector<Expr<E, S>> PurgeBool(const vector<Expr<E, S>>& ExpVec, 
                                                       const bool Value)
            {
                vector<Expr<E, S>> Retval;
                const string PurgeString = Value ? "true" : "false";
                for (auto const& Exp : ExpVec) {
                    if (Exp->template As<ConstExpression>() != nullptr &&
                        Exp->template SAs<ConstExpression>()->GetConstType() == 1 &&
                        Exp->template SAs<ConstExpression>()->GetConstValue() == PurgeString) {
                        continue;
                    }
                    Retval.push_back(Exp);
                }

                return Retval;
            }

            template <typename E, template <typename> class S>
            static inline bool HasBool(const vector<Expr<E, S>>& ExpVec, 
                                       const bool Value)
            {
                const string MatchString = Value ? "true" : "false";
                for (auto const& Exp : ExpVec) {
                    if (Exp->template As<ConstExpression>() != nullptr &&
                        Exp->template SAs<ConstExpression>()->GetConstType() == 1 &&
                        Exp->template SAs<ConstExpression>()->GetConstValue() == PurgeString) {
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
                    if (!CheckAllConstant(SimpChildren)) {
                        ExpStack.push_back(MakeOpExp(OpCode, SimpChildren, ExtData));
                    } else {
                        auto const& Val1 = 
                            SimpChildren[0]->template SAs<ConstExpression>()->GetConstValue();
                        auto const& Val2 = 
                            SimpChildren[1]->template SAs<ConstExpression>()->GetConstValue();
                        auto Result = (Val1 == Val2 ? "true" : "false");
                        ExpStack.push_back(new ConstExpression<E, S>(nullptr, Result, 1));
                    }
                    break;

                case LTSOps::OpNOT:
                    if (!CheckAllConstant(SimpChildren)) {
                        ExpStack.push_back(MakeOpExp(OpCode, SimpChildren, ExtData));
                    } else {
                        auto const& Val = 
                            SimpChildren[0]->template SAs<ConstExpression>()->GetConstValue();
                        auto Result = (Val == "true" ? "false" : "true");
                        ExpStack.push_back(new ConstExpression<E, S>(nullptr, Result, 1));
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
                        ExpStack.push_back(MakeOpExp, OpCode, SimpChildren, ExtData);
                    }
                    break;

                case LTSOps::OpOR:
                    if (HasBool(SimpChildren, true)) {
                        ExpStack.push_back(new ConstExpression<E, S>(nullptr, "true", 1));
                    } else {
                        auto&& RedChildren = PurgeBool(SimpChildren, false);
                        if (RedChildren.size() == 0) {
                            ExpStack.push_back(new ConstExpression<E, S>(nullptr, "false", 1));
                        } else if (RedChildren.size() == 1) {
                            ExpStack.push_back(RedChildren[0]);
                        } else {
                            ExpStack.push_back(MakeOpExp(OpCode, RedChildren, ExtData));
                        }
                    }
                    break;

                case LTSOps::OpAND:
                    if (HasBool(SimpChildren, false)) {
                        ExpStack.push_back(new ConstExpression<E, S>(nullptr, "false", 1));
                    } else {
                        auto&& RedChildren = PurgeBool(SimpChildren, true);
                        if (RedChildren.size() == 0) {
                            ExpStack.push_back(new ConstExpression<E, S>(nullptr, "true", 1));
                        } else if (RedChildren.size() == 1) {
                            ExpStack.push_back(RedChildren[0]);
                        } else {
                            ExpStack.push_back(MakeOpExp(OpCode, RedChildren, ExtData));
                        }
                    }
                    break;
                    
                }
                
            }

            template <typename E, template <typename> class S>
            inline void
            Simplifier<E, S>::VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp)
            {

            }

            template <typename E, template <typename> class S>
            inline void
            Simplifier<E, S>::VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp)
            {

            }

            template <typename E, template <typename> class S>
            inline void
            Simplifier<E, S>::VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp)
            {

            }

            template <typename E, template <typename> class S>
            inline typename Simplifier<E, S>::ExpT 
            Simplifier<E, S>::Do(const ExpT& Exp)
            {

            }
                        
        } /* end namespace Detail */
 
        template <typename E>
        class LTSTermSemanticizer
        {
        private:
            typedef RefCache<LTSTypeBase, LTSTypePtrHasher, LTSTypePtrEquals> TypeCacheT;
            UIDGenerator UFUIDGen;
            
        public:
            typedef Detail::LTSOps Ops;
            typedef Exprs::Expr<E, ESMC::LTS::LTSTermSemanticizer> ExpT;
            typedef void* LExpT;

            LTSTermSemanticizer();
            ~LTSTermSemanticizer();

            template <typename... ArgTypes>
            inline i64 MakeType(ArgTypes&&... Args);

            inline void TypeCheck(const ExpT& Exp) const;
            inline ExpT Canonicalize(const ExpT& Exp);
            // This is constant propagation
            inline ExpT Simplify(const ExpT& Exp);
            inline string ExprToString(const ExpT& Exp) const;
            inline string TypeToString(i64 Type) const;
            inline i64 RegisterUninterpretedFunction(const string& Name,
                                                     const vector<i64>& DomTypes,
                                                     i64 RangeType);

            // Functions which DO NOT have an implementation
            inline ExpT RaiseExpr(const LExpT& LExp);
            inline LExpT LowerExpr(const ExpT& Exp);
            inline ExpT ElimQuantifiers(const ExpT& Exp);
        };

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_TERM_SEMANTICIZER_HPP_ */

// 
// LTSTermSemanticizer.hpp ends here
