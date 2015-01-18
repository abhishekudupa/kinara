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

#include <unordered_map>
#include <unordered_set>
#include <set>
#include <vector>
#include <stack>
#include <z3.h>
#include <map>
#include <type_traits>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/multiprecision/cpp_int.hpp>

#include "../common/ESMCFwdDecls.hpp"
#include "../containers/RefCountable.hpp"
#include "../containers/SmartPtr.hpp"
#include "../containers/RefCache.hpp"
#include "../utils/UIDGenerator.hpp"
#include "../expr/SemanticizerUtils.hpp"
#include "../expr/Expressions.hpp"
#include "../tpinterface/Z3Objects.hpp"
#include "../utils/CombUtils.hpp"

#include "LTSSemTypes.hpp"

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

            // Syntactic operations for symbolic execution ONLY
            static const i64 OpSelect = 1020;
            static const i64 OpProject = 1021;

            // Store and update
            static const i64 OpStore = 1022;
            static const i64 OpUpdate = 1023;

            static const i64 UFOffset = 1000000;
        };

        static inline string MangleName(const string& Name,
                                        const vector<TypeRef> ArgTypes)
        {
            string Retval = Name;
            for (auto const& Arg : ArgTypes) {
                Retval += ((string)"@" + to_string(Arg->GetTypeID()));
            }
            return Retval;
        }

        namespace Detail {

            using namespace ESMC::Exprs;
            extern const unordered_map<i64, string> OpCodeToNameMap;
            extern const unordered_set<i64> LTSReservedOps;
            extern const string BoundVarPrefix;
            extern const TypeRef InvalidType;

            typedef unordered_map<i64, TypeRef> UFID2TypeMapT;

        } /* end namespace Detail */

        using namespace Detail;
        using namespace TP;
        using namespace Exprs;

        static inline bool IsLTSReserved(i64 OpCode)
        {
            return (Detail::LTSReservedOps.find(OpCode) != Detail::LTSReservedOps.end());
        }

        // A context class for remembering type info
        // etc about lowered expressions, so that if and when
        // the lowered expressions are raised, we can give them
        // appropriate types
        class LTSLoweredContext : public RefCountable
        {
        public:
            typedef unordered_set<Z3Expr, Z3ExprHasher> AssumptionSetT;

        private:
            mutable unordered_map<TypeRef, Z3Sort, TypePtrHasher> LTSTypeToSort;
            mutable map<string, TypeRef> VarNameToLTSType;
            Z3Ctx Ctx;
            mutable vector<AssumptionSetT> Assumptions;

        public:
            LTSLoweredContext();
            LTSLoweredContext(const Z3Ctx& Ctx);
            virtual ~LTSLoweredContext();

            const Z3Sort& GetZ3Sort(const TypeRef& LTSType) const;
            void AddZ3Sort(const TypeRef& LTSType, const Z3Sort& Sort) const;

            const TypeRef& GetLTSType(const string& VarName) const;
            void AddLTSType(const string& VarName, const TypeRef& LTSType) const;
            void PushAssumptionScope() const;
            AssumptionSetT PopAssumptionScope() const;
            void AddAssumption(const Z3Expr& Assumption) const;
            void AddAssumptionGlobal(const Z3Expr& Assumption) const;
            const AssumptionSetT& GetAssumptions() const;
            const vector<AssumptionSetT>& GetAllAssumptions() const;
            void ClearAssumptions() const;
            const Z3Ctx& GetZ3Ctx() const;
        };

        namespace Detail {

            template <typename E, template <typename> class S>
            class TypeChecker : public ExpressionVisitorBase<E, S>
            {
            private:
                const UFID2TypeMapT& UFMap;
                typedef Expr<E, S> ExpT;

                TypeRef BoolType;
                TypeRef IntType;

                vector<vector<TypeRef>> ScopeStack;

                inline void VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp);

            public:
                TypeChecker(const UFID2TypeMapT& UFMap, const TypeRef& BoolType,
                            const TypeRef& IntType);
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
                                      const TypeRef& BoolType, const TypeRef& IntType);
            };

            template <typename E, template <typename> class S>
            class Canonicalizer : public ExpressionVisitorBase<E, S>
            {
            private:
                typedef Expr<E, S> ExpT;
                stack<ExpT> ExpStack;

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

            // Forward declaration
            template <typename E, template <typename> class S> class ArrayRecordSimplifier;

            template <typename E, template <typename> class S>
            class Simplifier : public ExpressionVisitorBase<E, S>
            {
                friend class ArrayRecordSimplifier<E, S>;

            private:
                typedef Expr<E, S> ExpT;
                typedef ExprMgr<E, S> MgrType;

                stack<ExpT> ExpStack;
                TypeRef BoolType;
                TypeRef IntType;
                MgrType* Mgr;

                inline void VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp);

            public:
                Simplifier(MgrType* Mgr, const TypeRef& BoolType,
                           const TypeRef& IntType);
                virtual ~Simplifier();

                inline virtual void VisitVarExpression(const VarExpression<E, S>* Exp) override;
                inline virtual void VisitConstExpression(const ConstExpression<E, S>* Exp) override;
                inline virtual void VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp)
                    override;
                inline virtual void VisitOpExpression(const OpExpression<E, S>* Exp) override;
                inline virtual void VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>*
                                                               Exp) override;
                inline virtual void VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>*
                                                               Exp) override;

                static inline ExpT Do(MgrType* Mgr, const ExpT& Exp,
                                      const TypeRef& BoolType,
                                      const TypeRef& IntType);
            };

            template <typename E, template <typename> class S>
            class ArrayRecordSimplifier : public ExpressionVisitorBase<E, S>
            {
            private:
                typedef Expr<E, S> ExpT;
                typedef ExprMgr<E, S> MgrType;
                typedef unordered_set<ExpT, Exprs::ExpressionPtrHasher> FastExpSetT;
                typedef unordered_map<ExpT, ExpT, Exprs::ExpressionPtrHasher> FastExpMapT;

                FastExpSetT KillSet;
                deque<pair<ExpT, ExpT>> StoreStack;
                deque<pair<ExpT, ExpT>> UpdateStack;
                ExpT RelevantIndex;
                ExpT RelevantField;

                MgrType* Mgr;
                Simplifier<E, S>* ExprSimplifier;
                stack<ExpT> ExpStack;

                inline void VisitAlienExpression(const ExpressionBase<E, S>* Exp);

                inline bool IsStore(const ExpressionBase<E, S>* Exp);
                inline bool IsSelect(const ExpressionBase<E, S>* Exp);
                inline bool IsProject(const ExpressionBase<E, S>* Exp);
                inline bool IsUpdate(const ExpressionBase<E, S>* Exp);
                inline bool IsIndexRelevant(const ExpT& IndexExp);
                inline const ExpT& FindMatchingStore(const ExpT& IndexExp);
                inline const ExpT& FindMatchingUpdate(const ExpT& FieldExp);

            public:
                ArrayRecordSimplifier(MgrType* Mgr, Simplifier<E, S>* ExprSimplifier);
                virtual ~ArrayRecordSimplifier();

                inline virtual void VisitVarExpression(const VarExpression<E, S>* Exp) override;
                inline virtual void VisitConstExpression(const ConstExpression<E, S>* Exp) override;
                inline virtual void VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp)
                    override;
                inline virtual void VisitOpExpression(const OpExpression<E, S>* Exp) override;
                inline virtual void VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>*
                                                               Exp) override;
                inline virtual void VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>*
                                                               Exp) override;

                static inline ExpT Do(MgrType* Mgr, Simplifier<E, S>* ExprSimplifier,
                                      const ExpT& Exp);
            };

            template <typename E, template <typename> class S>
            class Lowerer : public ExpressionVisitorBase<E, S>
            {
            private:
                const UFID2TypeMapT& UFMap;
                stack<Z3Expr> ExpStack;
                typedef Expr<E, S> ExpT;
                LTSLCRef LTSCtx;

                inline void VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp);
                inline const Z3Sort& LowerType(const TypeRef& Type);

            public:
                Lowerer(const UFID2TypeMapT& UFMap, const LTSLCRef& LTSCtx);
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

                static inline Z3Expr Do(const ExpT& Exp, const UFID2TypeMapT& UFMap,
                                        const LTSLCRef& LTSCtx);
            };

            // a class to raise lowered expressions back up
            template <typename E, template <typename> class S>
            class Raiser
            {
            public:
                typedef ExprMgr<E, S> MgrT;
                typedef Expr<E, S> ExpT;
                typedef Z3Expr LExpT;

            private:
                const unordered_map<string, i64>& UFNameToIDMap;
                LTSLCRef LTSCtx;
                MgrT* Mgr;
                Z3Ctx Ctx;

                ExpT RaiseVarExpr(const LExpT& Exp);
                ExpT RaiseConstExpr(const LExpT& Exp);
                ExpT RaiseOpExpr(const LExpT& Exp);

                ExpT RaiseExpr(const LExpT& Exp);

            public:
                Raiser(const unordered_map<string, i64>& UFNameToIDMap,
                       const LTSLCRef& LTSCtx, MgrT* Mgr);
                ~Raiser();

                static inline ExpT
                Do(const LExpT& LExp, const unordered_map<string, i64>& UFNameToIDMap,
                   const LTSLCRef& LTSCtx, MgrT* Mgr);
            };


            template <typename E, template <typename> class S>
            class DeBruijnShifter : public ExpressionVisitorBase<E, S>
            {
            private:
                typedef ExprMgr<E, S> MgrT;
                typedef Expr<E, S> ExpT;

                stack<ExpT> ExpStack;
                MgrT* Mgr;
                i32 ShiftValue;

            public:
                DeBruijnShifter(MgrT* Mgr, i32 ShiftValue);
                virtual ~DeBruijnShifter();

                inline virtual void VisitVarExpression(const VarExpression<E, S>* Exp) override;
                inline virtual void
                VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp) override;
                inline virtual void VisitConstExpression(const ConstExpression<E, S>* Exp) override;
                inline virtual void VisitOpExpression(const OpExpression<E, S>* Exp) override;
                inline virtual void
                VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp) override;
                inline virtual void
                VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp) override;

                static ExpT Do(MgrT* Mgr, const ExpT& Exp, i32 ShiftValue);
            };

            // A class that unrolls quantifiers
            template <typename E, template <typename> class S>
            class QuantifierUnroller : public ExpressionVisitorBase<E, S>
            {
            private:
                typedef ExprMgr<E, S> MgrT;
                typedef Expr<E, S> ExpT;

                stack<ExpT> ExpStack;
                MgrT* Mgr;
                bool IncludeUndef;

                inline vector<ExpT> UnrollQuantifier(const QuantifiedExpressionBase<E, S>* Exp);

            public:
                inline QuantifierUnroller(MgrT* Mgr, bool IncludeUndef);
                inline virtual ~QuantifierUnroller();

                inline virtual void VisitVarExpression(const VarExpression<E, S>* Exp) override;
                inline virtual void
                VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp) override;
                inline virtual void VisitConstExpression(const ConstExpression<E, S>* Exp) override;
                inline virtual void VisitOpExpression(const OpExpression<E, S>* Exp) override;
                inline virtual void
                VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp) override;
                inline virtual void
                VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp) override;

                static ExpT Do(MgrT* Mgr, const ExpT& Exp, bool IncludeUndef);
            };

            // Implementation of type checker
            template <typename E, template <typename> class S>
            TypeChecker<E, S>::TypeChecker(const UFID2TypeMapT& UFMap,
                                           const TypeRef& BoolType,
                                           const TypeRef& IntType)
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

                if (Type->template Is<FuncType>()) {
                    throw ExprTypeError("Cannot create variables of function types");
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
                if (ActType->template As<BooleanType>() != nullptr) {
                    boost::algorithm::to_lower(ConstVal);
                    if (ConstVal != "true" && ConstVal != "false") {
                        throw ExprTypeError("Unknown constant string");
                    }
                    Exp->SetType(Exp->GetConstType());
                } else if(ActType->template As<RangeType>() != nullptr ||
                          ActType->template As<IntegerType>() != nullptr) {
                    boost::multiprecision::cpp_int Val = 0;
                    Val = boost::lexical_cast<boost::multiprecision::cpp_int>(ConstVal);
                    if (ActType->template As<RangeType>() != nullptr) {
                        auto RType = ActType->template SAs<RangeType>();

                        if (Val < RType->GetLow() ||
                            Val > RType->GetHigh()) {
                            throw ExprTypeError((string)"Value " + ConstVal + " is out of bounds");
                        }
                    }
                    Exp->SetType(ActType);
                } else if(ActType->template As<EnumType>() != nullptr) {
                    auto EType = ActType->template SAs<EnumType>();
                    if (!EType->IsMember(ConstVal)) {
                        throw ExprTypeError((string)"Value " + ConstVal + " is not valid");
                    }
                    Exp->SetType(ActType);
                } else if(ActType->template As<SymmetricType>() != nullptr) {
                    auto SymmType = ActType->template SAs<SymmetricType>();
                    if (!SymmType->IsMember(ConstVal)) {
                        throw ExprTypeError((string)"Value " + ConstVal + " is not valid");
                    }
                    Exp->SetType(ActType);
                } else {
                    throw ExprTypeError((string)"Only boolean, range, enum and symmetric type " +
                                        "constants are currently supported");
                }
            }

            static inline const TypeRef& GetTypeForBoundVar
            (const vector<vector<TypeRef>>& ScopeStack,
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
                return TypeRef::NullPtr;
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
                if (Type->template Is<ScalarType>()) {
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

            static inline bool CheckTypeCompat(const TypeRef& ExpType,
                                               const TypeRef& ActType)
            {
                if (ExpType == ActType) {
                    return true;
                } else {
                    if (ExpType->As<IntegerType>() != nullptr &&
                        ActType->As<IntegerType>() != nullptr) {
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

            static inline bool CheckAllScalar(const vector<TypeRef>& Types)
            {
                return all_of(Types.begin(), Types.end(),
                              [] (const TypeRef& Type) -> bool
                              {
                                  return (Type->Is<ScalarType>());
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

                vector<TypeRef> ChildTypes;
                auto const& Children = Exp->GetChildren();
                for (auto const& Child : Children) {
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
                    if (!ChildTypes[1]->template Is<ScalarType>() ||
                        !ChildTypes[2]->template Is<ScalarType>()) {
                        throw ExprTypeError((string)"ite op requires both branches to be " +
                                            "scalar typed");
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
                                [&] (const TypeRef& i) -> bool
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
                                [&] (const TypeRef& i) -> bool
                                { return CheckTypeCompat(i, IntType); })) {
                        throw ExprTypeError((string)"add/sub/mul ops need integer arguments");
                    }
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
                                [&] (const TypeRef& i) -> bool
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

                // Index and select are handled identically
                case LTSOps::OpIndex:
                case LTSOps::OpSelect: {
                    CheckNumArgs(2, ChildTypes.size(), "Index");
                    auto Type1 = ChildTypes[0];
                    auto ArrType = Type1->template As<ArrayType>();

                    if (ArrType == nullptr) {
                        throw ExprTypeError((string)"Index operator can only be " +
                                            "applied to array types");
                    }

                    TypeRef ExpIndexType;

                    ExpIndexType = ArrType->GetIndexType();

                    if (!CheckTypeCompat(ExpIndexType, ChildTypes[1])) {
                        throw ExprTypeError("Invalid type for index expression");
                    }
                    if (Children[1]->template Is<ConstExpression>() &&
                        ChildTypes[1]->template Is<SymmetricType>()) {
                        auto const& ConstVal =
                            Children[1]->template SAs<ConstExpression>()->GetConstValue();
                        auto const& TypeName =
                            ChildTypes[1]->template SAs<SymmetricType>()->GetName();
                        if (ConstVal == "clear" || ConstVal == (TypeName + "::clear")) {
                            throw ExprTypeError((string)"Indexing with undefined value into " +
                                                "a symmetric array");
                        }
                    }

                    TypeRef ElemType;

                    ElemType = ArrType->GetValueType();
                    Exp->SetType(ElemType);
                    break;
                }

                // Project and field are handled identically
                case LTSOps::OpField:
                case LTSOps::OpProject: {
                    CheckNumArgs(2, ChildTypes.size(), "Field");

                    if (!ChildTypes[0]->template Is<RecordType>() &&
                        !ChildTypes[0]->template Is<ParametricType>()) {
                        throw ExprTypeError((string)"Field access only allowed on " +
                                            "record or parametric types");
                    }

                    if (!ChildTypes[1]->template Is<FieldAccessType>()) {
                        throw ExprTypeError((string)"Record Field accesses must be made with " +
                                            "variables of type FieldAccessType");
                    }

                    auto FieldExp = ((Exp->GetChildren())[1])->template As<VarExpression>();
                    if (FieldExp == nullptr) {
                        throw ExprTypeError("Field access expression must be a VarExpression");
                    }

                    auto const& FieldName = FieldExp->GetVarName();

                    const RecordType* RecType = nullptr;
                    if (ChildTypes[0]->template Is<RecordType>()) {
                        RecType = ChildTypes[0]->template SAs<RecordType>();
                    } else {
                        // Must be a parametric base type
                        auto PType = ChildTypes[0]->template SAs<ParametricType>();
                        RecType = PType->GetBaseType()->template As<RecordType>();
                    }
                    auto ValType = RecType->GetTypeForMember(FieldName);
                    if (ValType == TypeRef::NullPtr) {
                        throw ExprTypeError((string)"Field name \"" +
                                            FieldName + "\" is invalid for " +
                                            "record type \"" + RecType->GetName() + "\"");
                    }

                    Exp->SetType(ValType);
                    break;
                }

                case LTSOps::OpStore: {
                    CheckNumArgs(3, ChildTypes.size(), "Store");

                    if (!ChildTypes[0]->template Is<ArrayType>()) {
                        throw ExprTypeError((string)"Store op must have array expression " +
                                            "as first argument");
                    }
                    auto ArrType = ChildTypes[0]->template SAs<ArrayType>();
                    auto IndexType = ArrType->GetIndexType();
                    auto ValueType = ArrType->GetValueType();

                    if (!CheckTypeCompat(IndexType, ChildTypes[1])) {
                        throw ExprTypeError((string)"Invalid type for index expression");
                    }
                    if (!CheckTypeCompat(ValueType, ChildTypes[2])) {
                        throw ExprTypeError((string)"Invalid type for value expression in Store " +
                                            "expression");
                    }

                    // All good. This evaluates to a value of the array type
                    Exp->SetType(ArrType);
                    break;
                }

                case LTSOps::OpUpdate: {
                    CheckNumArgs(3, ChildTypes.size(), "Update");

                    if (!ChildTypes[0]->template Is<RecordType>() &&
                        !ChildTypes[0]->template Is<ParametricType>()) {
                        throw ExprTypeError((string)"Update expression must be applied on a " +
                                            "record or parametric type");
                    }

                    if (!ChildTypes[1]->template Is<FieldAccessType>()) {
                        throw ExprTypeError((string)"Record field accesses must be made with " +
                                            "variables of type FieldAccessType");
                    }

                    auto FieldExp = Children[1]->template As<VarExpression>();
                    if (FieldExp == nullptr) {
                        throw ExprTypeError((string)"Field access expression must be a VarExpression");
                    }

                    auto const& FieldName = FieldExp->GetVarName();

                    const RecordType* RecType = nullptr;
                    if (ChildTypes[0]->template Is<RecordType>()) {
                        RecType = ChildTypes[0]->template SAs<RecordType>();
                    } else {
                        auto PType = ChildTypes[0]->template SAs<ParametricType>();
                        RecType = PType->GetBaseType()->template As<RecordType>();
                    }

                    auto ValType = RecType->GetTypeForMember(FieldName);
                    if (ValType == TypeRef::NullPtr) {
                        throw ExprTypeError((string)"Field name \"" +
                                            FieldName + "\" is invalid for " +
                                            "record type \"" + RecType->GetName() + "\"");
                    }

                    if (!CheckTypeCompat(ValType, ChildTypes[2])) {
                        throw ExprTypeError((string)"Invalid type for value in update expression");
                    }

                    Exp->SetType(RecType);
                }
                    break;

                default: {
                    // Must be an uninterpreted function
                    auto it = UFMap.find(OpCode);
                    if (it == UFMap.end()) {
                        throw ExprTypeError((string)"Unknown opcode " + to_string(OpCode));
                    }
                    auto FunType = it->second->template As<FuncType>();
                    auto const& ArgTypes = FunType->GetArgTypes();
                    if (ChildTypes.size() != ArgTypes.size()) {
                        throw ExprTypeError((string)"Incorrect number of arguments for UF");
                    }
                    for (u32 i = 0; i < ChildTypes.size(); ++i) {
                        if (!CheckTypeCompat(ChildTypes[i], ArgTypes[i])) {
                            throw ExprTypeError("Incorrect type for argument of UF");
                        }
                    }
                    Exp->SetType(FunType->GetEvalType());
                }
                }
            }

            template <typename E, template <typename> class S>
            inline void
            TypeChecker<E, S>::VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp)
            {
                auto const& QVarTypes = Exp->GetQVarTypes();
                for (auto const& QVarType : QVarTypes) {
                    if (!QVarType->template Is<ScalarType>()) {
                        throw ExprTypeError((string)"Quantified variables must have scalar " +
                                            "types. \"" + QVarType->ToString() + "\" is not " +
                                            "a scalar type");
                    }
                }
                ScopeStack.push_back(QVarTypes);
                auto const& QExpr = Exp->GetQExpression();
                SemUtils::TypeInvalidator<E, S> Inv;
                QExpr->Accept(&Inv);
                QExpr->Accept(this);

                if (QExpr->GetType() != BoolType) {
                    throw ExprTypeError("Body of quantified expression needs to be boolean");
                }

                Exp->SetType(BoolType);
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
                                  const TypeRef& BoolType, const TypeRef& IntType)
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
                    OpString = it2->second->template As<FuncType>()->GetName();
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
                    ExpString += TypeToString(QVarType);
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
                ExpStack.push(Exp);
            }

            template <typename E, template <typename> class S>
            inline void
            Canonicalizer<E, S>::VisitConstExpression(const ConstExpression<E, S>* Exp)
            {
                ExpStack.push(Exp);
            }

            template <typename E, template <typename> class S>
            inline void
            Canonicalizer<E, S>::VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp)
            {
                ExpStack.push(Exp);
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
                    NewChildren[NumChildren - i - 1] = ExpStack.top();
                    ExpStack.pop();
                }

                switch (OpCode) {
                case LTSOps::OpEQ:
                case LTSOps::OpOR:
                case LTSOps::OpAND:
                case LTSOps::OpIFF:
                case LTSOps::OpXOR:
                case LTSOps::OpADD:
                case LTSOps::OpMUL: {
                    set<ExpT, ExpressionPtrCompare> CanonSet(NewChildren.begin(), NewChildren.end());
                    vector<ExpT> CanonChildren(CanonSet.begin(), CanonSet.end());
                    ExpStack.push(new OpExpression<E, S>(nullptr, OpCode,
                                                         CanonChildren, Exp->ExtensionData));
                }
                    break;

                case LTSOps::OpNOT:
                    if (NewChildren[0]->template As<OpExpression>() != nullptr &&
                        NewChildren[0]->template
                        SAs<OpExpression>()->GetOpCode() == LTSOps::OpNOT) {

                        ExpStack.push((NewChildren[0]->template
                                            SAs<OpExpression>()->GetChildren())[0]);
                    } else {
                        ExpStack.push(new OpExpression<E, S>(nullptr, OpCode,
                                                                  NewChildren, Exp->ExtensionData));
                    }
                    break;

                default:
                    ExpStack.push(new OpExpression<E, S>(nullptr, OpCode, NewChildren,
                                                              Exp->ExtensionData));
                    break;
                }
            }

            template <typename E, template <typename> class S>
            inline void
            Canonicalizer<E, S>::VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto NewQExpr = ExpStack.top();
                ExpStack.pop();
                if (Exp->IsForAll()) {
                    ExpStack.push(new AQuantifiedExpression<E, S>(nullptr, Exp->GetQVarTypes(),
                                                                       NewQExpr,
                                                                       Exp->ExtensionData));
                } else {
                    ExpStack.push(new EQuantifiedExpression<E, S>(nullptr, Exp->GetQVarTypes(),
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
                return TheCanonicalizer.ExpStack.top();
            }

            // Implementation of Simplifier
            template <typename E, template <typename> class S>
            inline Simplifier<E, S>::Simplifier(MgrType* Mgr,
                                                const TypeRef& BoolType,
                                                const TypeRef& IntType)
                : ExpressionVisitorBase<E, S>("LTSTermSimplifier"),
                  BoolType(BoolType), IntType(IntType), Mgr(Mgr)
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
                ExpStack.push(Exp);
            }

            template <typename E, template <typename> class S>
            inline void
            Simplifier<E, S>::VisitConstExpression(const ConstExpression<E, S>* Exp)
            {
                ExpStack.push(Exp);
            }

            template <typename E, template <typename> class S>
            inline void
            Simplifier<E, S>::VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp)
            {
                ExpStack.push(Exp);
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
                                                       const bool Value,
                                                       const TypeRef& BoolType)
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
                                       const bool Value, const TypeRef& BoolType)
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
            static inline bool AllOfOp(const vector<Expr<E, S>>& ExpVec,
                                       i64 OpCode)
            {
                return all_of(ExpVec.begin(), ExpVec.end(),
                              [&] (const Expr<E, S>& Exp) -> bool
                              {
                                  auto ExpAsOp = Exp->template As<OpExpression>();
                                  if (ExpAsOp != nullptr) {
                                      return (ExpAsOp->GetOpCode() == OpCode);
                                  }
                                  return false;
                              });
            }

            template <typename E, template <typename> class S>
            static inline vector<Expr<E, S>>
            GetAllChildren(const vector<Expr<E, S>>& ExpVec)
            {
                vector<Expr<E, S>> Retval;
                for (auto const& Exp : ExpVec) {
                    auto ExpAsOp = Exp->template As<OpExpression>();
                    if (ExpAsOp == nullptr) {
                        throw InternalError((string)"Attempted to flatten non-op " +
                                            "expression.\nAt: " + __FILE__ + ":" +
                                            to_string(__LINE__));
                    }
                    auto const& Children = ExpAsOp->GetChildren();
                    Retval.insert(Retval.end(), Children.begin(), Children.end());
                }
                return Retval;
            }

            template <typename E, template <typename> class S>
            static inline Expr<E, S> MakeOpExp(ExprMgr<E, S>* Mgr,
                                               i64 OpCode, const vector<Expr<E, S>>& Children,
                                               const E& ExtData)
            {
                return Mgr->MakeExpr(OpCode, Children, ExtData);
            }

            template <typename E, template <typename> class S>
            inline void
            Simplifier<E, S>::VisitOpExpression(const OpExpression<E, S>* Exp)
            {
                const u32 NumChildren = Exp->GetChildren().size();
                auto const OpCode = Exp->GetOpCode();
                vector<ExpT> SimpChildren(NumChildren);
                auto const& ExtData = Exp->ExtensionData;

                if (OpCode != LTSOps::OpSelect && OpCode != LTSOps::OpProject &&
                    OpCode != LTSOps::OpUpdate && OpCode != LTSOps::OpStore) {

                    ExpressionVisitorBase<E, S>::VisitOpExpression(Exp);

                    for(u32 i = 0; i < NumChildren; ++i) {
                        SimpChildren[NumChildren - i - 1] = ExpStack.top();
                        ExpStack.pop();
                    }
                }

                if (LTSReservedOps.find(OpCode) == LTSReservedOps.end()) {
                    ExpStack.push(MakeOpExp(Mgr, OpCode, SimpChildren, ExtData));
                    return;
                }

                switch(OpCode) {
                case LTSOps::OpEQ:
                case LTSOps::OpIFF:
                    if (!CheckAllConstant(SimpChildren)) {
                        if (SimpChildren[0]->Equals(SimpChildren[1])) {
                            ExpStack.push(Mgr->MakeVal("true", BoolType));
                        } else {
                            ExpStack.push(MakeOpExp(Mgr, OpCode, SimpChildren, ExtData));
                        }
                    } else {
                        auto const& Val1 =
                            SimpChildren[0]->template SAs<ConstExpression>()->GetConstValue();
                        auto const& Val2 =
                            SimpChildren[1]->template SAs<ConstExpression>()->GetConstValue();
                        auto Result = (Val1 == Val2 ? "true" : "false");
                        ExpStack.push(Mgr->MakeVal(Result, BoolType));
                    }
                    break;

                case LTSOps::OpNOT:
                    if (!CheckAllConstant(SimpChildren)) {
                        ExpStack.push(MakeOpExp(Mgr, OpCode, SimpChildren, ExtData));
                    } else {
                        auto const& Val =
                            SimpChildren[0]->template SAs<ConstExpression>()->GetConstValue();
                        auto Result = (Val == "true" ? "false" : "true");
                        ExpStack.push(Mgr->MakeVal(Result, BoolType));
                    }
                    break;

                case LTSOps::OpITE:
                    if (SimpChildren[1]->Equals(SimpChildren[2])) {
                        ExpStack.push(SimpChildren[1]);
                    } else if (SimpChildren[0]->template As<ConstExpression>() != nullptr) {
                        if (SimpChildren[0]->template
                            SAs<ConstExpression>()->GetConstValue() == "true") {
                            ExpStack.push(SimpChildren[1]);
                        } else {
                            ExpStack.push(SimpChildren[2]);
                        }
                    } else {
                        ExpStack.push(MakeOpExp(Mgr, OpCode, SimpChildren, ExtData));
                    }
                    break;

                case LTSOps::OpOR:
                    if (HasBool(SimpChildren, true, BoolType)) {
                        ExpStack.push(Mgr->MakeVal("true", BoolType));
                    } else {
                        auto&& RedChildren = PurgeBool(SimpChildren, false, BoolType);
                        if (RedChildren.size() == 0) {
                            ExpStack.push(Mgr->MakeVal("false", BoolType));
                        } else if (RedChildren.size() == 1) {
                            ExpStack.push(RedChildren[0]);
                        } else {
                            vector<ExpT> FinalChildren;
                            for (auto const& RedChild : RedChildren) {
                                auto RedChildAsOp = RedChild->template As<OpExpression>();
                                if (RedChildAsOp != nullptr &&
                                    RedChildAsOp->GetOpCode() == LTSOps::OpOR) {
                                    auto const& RedChildChildren = RedChildAsOp->GetChildren();
                                    FinalChildren.insert(FinalChildren.end(),
                                                         RedChildChildren.begin(),
                                                         RedChildChildren.end());
                                } else {
                                    FinalChildren.push_back(RedChild);
                                }
                            }

                            ExpStack.push(MakeOpExp(Mgr, OpCode, FinalChildren, ExtData));
                        }
                    }
                    break;

                case LTSOps::OpAND:
                    if (HasBool(SimpChildren, false, BoolType)) {
                        ExpStack.push(Mgr->MakeVal("false", BoolType));
                    } else {
                        auto&& RedChildren = PurgeBool(SimpChildren, true, BoolType);
                        if (RedChildren.size() == 0) {
                            ExpStack.push(Mgr->MakeVal("true", BoolType));
                        } else if (RedChildren.size() == 1) {
                            ExpStack.push(RedChildren[0]);
                        } else {
                            vector<ExpT> FinalChildren;
                            for (auto const& RedChild : RedChildren) {
                                auto RedChildAsOp = RedChild->template As<OpExpression>();
                                if (RedChildAsOp != nullptr &&
                                    RedChildAsOp->GetOpCode() == LTSOps::OpAND) {
                                    auto const& RedChildChildren = RedChildAsOp->GetChildren();
                                    FinalChildren.insert(FinalChildren.end(),
                                                         RedChildChildren.begin(),
                                                         RedChildChildren.end());
                                } else {
                                    FinalChildren.push_back(RedChild);
                                }
                            }
                            ExpStack.push(MakeOpExp(Mgr, OpCode, FinalChildren, ExtData));
                        }
                    }
                    break;

                case LTSOps::OpIMPLIES:
                    if (SimpChildren[0]->template As<ConstExpression>() != nullptr) {
                        auto Ant = SimpChildren[0]->template SAs<ConstExpression>();
                        auto const& AntVal = Ant->GetConstValue();
                        if (AntVal == "false") {
                            ExpStack.push(Mgr->MakeVal("true", BoolType));
                        } else {
                            ExpStack.push(SimpChildren[1]);
                        }
                    } else if (SimpChildren[1]->template As<ConstExpression>() != nullptr) {
                        auto Con = SimpChildren[1]->template SAs<ConstExpression>();
                        auto const& ConVal = Con->GetConstValue();
                        if (ConVal == "false") {
                            vector<ExpT> NewChildren;
                            NewChildren.push_back(SimpChildren[0]);
                            ExpStack.push(Mgr->MakeExpr(LTSOps::OpNOT,
                                                             NewChildren));
                        } else {
                            ExpStack.push(SimpChildren[1]);
                        }
                    } else {
                        ExpStack.push(MakeOpExp(Mgr, OpCode, SimpChildren, ExtData));
                    }
                    break;

                case LTSOps::OpXOR:
                    if (CheckAllConstant(SimpChildren)) {
                        auto AExp = SimpChildren[0]->template SAs<ConstExpression>();
                        auto BExp = SimpChildren[0]->template SAs<ConstExpression>();

                        auto AVal = AExp->GetConstValue();
                        auto BVal = BExp->GetConstValue();

                        if (AVal == BVal) {
                            ExpStack.push(Mgr->MakeVal("false", BoolType));
                        } else {
                            ExpStack.push(Mgr->MakeVal("true", BoolType));
                        }
                    }

                    if (SimpChildren[0]->template As<ConstExpression>() != nullptr) {
                        auto AExp = SimpChildren[0]->template SAs<ConstExpression>();
                        auto const& AVal = AExp->GetConstValue();
                        if (AVal == "true") {
                            vector<ExpT> NewChildren;
                            NewChildren.push_back(SimpChildren[1]);
                            ExpStack.push(Mgr->MakeExpr(LTSOps::OpNOT, NewChildren));
                        } else {
                            ExpStack.push(SimpChildren[1]);
                        }
                    } else if (SimpChildren[1]->template As<ConstExpression>() != nullptr) {
                        auto BExp = SimpChildren[1]->template SAs<ConstExpression>();
                        auto const& BVal = BExp->GetConstValue();
                        if (BVal == "true") {
                            vector<ExpT> NewChildren;
                            NewChildren.push_back(SimpChildren[0]);
                            ExpStack.push(Mgr->MakeExpr(LTSOps::OpNOT, NewChildren));
                        } else {
                            ExpStack.push(SimpChildren[0]);
                        }
                    } else {
                        ExpStack.push(MakeOpExp(Mgr, OpCode, SimpChildren, ExtData));
                    }
                    break;

                case LTSOps::OpADD:
                    if (CheckAllConstant(SimpChildren)) {
                        i64 SumVal = 0;
                        for (auto const& Exp : SimpChildren) {
                            auto Val =
                                boost::lexical_cast<i64>(Exp->template
                                                         SAs<ConstExpression>()->GetConstValue());
                            SumVal += Val;
                        }
                        ExpStack.push(Mgr->MakeVal(to_string(SumVal), IntType));
                    } else {
                        auto&& RedChildren = PurgeInt(SimpChildren, 0);
                        if (RedChildren.size() == 1) {
                            ExpStack.push(RedChildren[0]);
                        } else {
                            vector<ExpT> FinalChildren;
                            for (auto const& RedChild : RedChildren) {
                                auto RedChildAsOp = RedChild->template As<OpExpression>();
                                if (RedChildAsOp != nullptr &&
                                    RedChildAsOp->GetOpCode() == LTSOps::OpADD) {
                                    auto const& RedChildChildren = RedChildAsOp->GetChildren();
                                    FinalChildren.insert(FinalChildren.end(),
                                                         RedChildChildren.begin(),
                                                         RedChildChildren.end());
                                } else {
                                    FinalChildren.push_back(RedChild);
                                }
                            }
                            ExpStack.push(MakeOpExp(Mgr, OpCode, FinalChildren, ExtData));
                        }
                    }
                    break;

                case LTSOps::OpSUB:
                    if (CheckAllConstant(SimpChildren)) {
                        i64 FirstVal =
                            boost::lexical_cast<i64>(SimpChildren[0]->template
                                                     SAs<ConstExpression>()->GetConstValue());
                        i64 ToSubtract = 0;
                        for (u32 i = 1; i < NumChildren; ++i) {
                            ToSubtract +=
                                boost::lexical_cast<i64>(SimpChildren[i]->template
                                                         SAs<ConstExpression>()->GetConstValue());
                        }
                        i64 DiffVal = FirstVal - ToSubtract;
                        ExpStack.push(Mgr->MakeVal(to_string(DiffVal), IntType));
                    } else {
                        auto it = SimpChildren.begin();
                        ++it;
                        if (all_of(it, SimpChildren.end(),
                                   [&] (const ExpT& Exp) -> bool
                                   {
                                       if (Exp->template Is<ConstExpression>()) {
                                           auto ExpAsConst = Exp->template SAs<ConstExpression>();
                                           auto const& ConstVal = ExpAsConst->GetConstValue();
                                           return (boost::lexical_cast<i64>(ConstVal) == 0);
                                       } else {
                                           return false;
                                       }
                                   })) {
                            ExpStack.push(SimpChildren[0]);
                        } else {
                            ExpStack.push(MakeOpExp(Mgr, OpCode, SimpChildren, ExtData));
                        }
                    }
                    break;

                case LTSOps::OpMINUS:
                    if (CheckAllConstant(SimpChildren)) {
                        i64 Val = boost::lexical_cast<i64>(SimpChildren[0]->template
                                                           SAs<ConstExpression>()->GetConstValue());
                        Val = -Val;
                        ExpStack.push(Mgr->MakeVal(to_string(Val), IntType));
                    } else {
                        ExpStack.push(MakeOpExp(Mgr, OpCode, SimpChildren, ExtData));
                    }
                    break;

                case LTSOps::OpMUL:
                    if (CheckAllConstant(SimpChildren)) {
                        i64 ProdVal = 1;
                        for (auto const& Exp : SimpChildren) {
                            auto Val =
                                boost::lexical_cast<i64>(Exp->template
                                                         SAs<ConstExpression>()->GetConstValue());
                            ProdVal *= Val;
                        }
                        ExpStack.push(Mgr->MakeVal(to_string(ProdVal), IntType));
                    } else {
                        if (HasInt(SimpChildren, 0)) {
                            ExpStack.push(Mgr->MakeVal("0", IntType));
                        } else {
                            auto&& RedChildren = PurgeInt(SimpChildren, 1);
                            if (RedChildren.size() == 1) {
                                ExpStack.push(RedChildren[0]);
                            } else {
                                vector<ExpT> FinalChildren;
                                for (auto const& RedChild : RedChildren) {
                                    auto RedChildAsOp = RedChild->template As<OpExpression>();
                                    if (RedChildAsOp != nullptr &&
                                        RedChildAsOp->GetOpCode() == LTSOps::OpMUL) {
                                        auto const& RedChildChildren = RedChildAsOp->GetChildren();
                                        FinalChildren.insert(FinalChildren.end(),
                                                             RedChildChildren.begin(),
                                                             RedChildChildren.end());
                                    } else {
                                        FinalChildren.push_back(RedChild);
                                    }
                                }

                                ExpStack.push(MakeOpExp(Mgr, OpCode, FinalChildren, ExtData));
                            }
                        }
                    }
                    break;

                case LTSOps::OpDIV:
                case LTSOps::OpMOD:
                    if (CheckAllConstant(SimpChildren)) {
                        i64 DivVal;
                        if (OpCode == LTSOps::OpDIV) {
                            DivVal =
                                (boost::lexical_cast<i64>(SimpChildren[0]->template
                                                          SAs<ConstExpression>()->GetConstValue()) /
                                 boost::lexical_cast<i64>(SimpChildren[1]->template
                                                          SAs<ConstExpression>()->GetConstValue()));
                        } else {
                            DivVal =
                                (boost::lexical_cast<i64>(SimpChildren[0]->template
                                                          SAs<ConstExpression>()->GetConstValue()) %
                                 boost::lexical_cast<i64>(SimpChildren[1]->template
                                                          SAs<ConstExpression>()->GetConstValue()));

                        }
                        ExpStack.push(Mgr->MakeVal(to_string(DivVal), IntType));
                    } else {
                        if (SimpChildren[1]->template As<ConstExpression>() != nullptr) {
                            auto Val =
                                boost::lexical_cast<i64>(SimpChildren[1]->template
                                                         SAs<ConstExpression>()->GetConstValue());
                            if (Val == 0) {
                                throw ExprTypeError("Division by zero during simplification");
                            } else if (Val == 1) {
                                if (OpCode == LTSOps::OpMOD) {
                                    ExpStack.push(Mgr->MakeVal("0", IntType));
                                } else {
                                    ExpStack.push(SimpChildren[0]);
                                }
                            }
                        } else {
                            ExpStack.push(MakeOpExp(Mgr, OpCode, SimpChildren, ExtData));
                        }
                    }
                    break;

                case LTSOps::OpGT:
                case LTSOps::OpGE:
                case LTSOps::OpLT:
                case LTSOps::OpLE:
                    if (CheckAllConstant(SimpChildren)) {
                        auto Val1 =
                            boost::lexical_cast<i64>(SimpChildren[0]->template
                                                     SAs<ConstExpression>()->GetConstValue());
                        auto Val2 =
                            boost::lexical_cast<i64>(SimpChildren[1]->template
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

                        ExpStack.push(Mgr->MakeVal(ResString, BoolType));
                    } else {
                        ExpStack.push(MakeOpExp(Mgr, OpCode, SimpChildren, ExtData));
                    }
                    break;

                case LTSOps::OpSelect:
                case LTSOps::OpStore:
                case LTSOps::OpProject:
                case LTSOps::OpUpdate: {
                    ExpStack.push(ArrayRecordSimplifier<E, S>::Do(Mgr, this, Exp));
                }
                    break;
                default:
                    ExpStack.push(MakeOpExp(Mgr, OpCode, SimpChildren, ExtData));
                }
            }

            template <typename E, template <typename> class S>
            inline void
            Simplifier<E, S>::VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto NewQExpr = ExpStack.top();
                ExpStack.pop();

                if (NewQExpr->template Is<ConstExpression>()) {
                    ExpStack.push(NewQExpr);
                    return;
                }

                // Not a constant
                if (Exp->IsForAll()) {
                    ExpStack.push(Mgr->MakeForAll(Exp->GetQVarTypes(),
                                                       NewQExpr, Exp->ExtensionData));
                } else {
                    ExpStack.push(Mgr->MakeExists(Exp->GetQVarTypes(),
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
            Simplifier<E, S>::Do(MgrType* Mgr, const ExpT& Exp,
                                 const TypeRef& BoolType,
                                 const TypeRef& IntType)
            {
                Simplifier<E, S> TheSimplifier(Mgr, BoolType, IntType);
                Exp->Accept(&TheSimplifier);
                return TheSimplifier.ExpStack.top();
            }

            // Implementation of ArrayRecordSimplifier
            template <typename E, template <typename> class S>
            ArrayRecordSimplifier<E, S>::ArrayRecordSimplifier(MgrType* Mgr,
                                                               Simplifier<E, S>* ExprSimplifier)
                : ExpressionVisitorBase<E, S>("ArrayRecordSimplifier"),
                  RelevantIndex(ExpT::NullPtr), RelevantField(ExpT::NullPtr),
                  Mgr(Mgr), ExprSimplifier(ExprSimplifier)
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            ArrayRecordSimplifier<E, S>::~ArrayRecordSimplifier()
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            inline bool
            ArrayRecordSimplifier<E, S>::IsStore(const ExpressionBase<E, S>* Exp)
            {
                auto ExpAsOp = Exp->template As<OpExpression>();
                if (ExpAsOp == nullptr) {
                    return false;
                }
                return (ExpAsOp->GetOpCode() == LTSOps::OpStore);
            }

            template <typename E, template <typename> class S>
            inline bool
            ArrayRecordSimplifier<E, S>::IsSelect(const ExpressionBase<E, S>* Exp)
            {
                auto ExpAsOp = Exp->template As<OpExpression>();
                if (ExpAsOp == nullptr) {
                    return false;
                }
                return (ExpAsOp->GetOpCode() == LTSOps::OpSelect);
            }

            template <typename E, template <typename> class S>
            inline bool
            ArrayRecordSimplifier<E, S>::IsProject(const ExpressionBase<E, S>* Exp)
            {
                auto ExpAsOp = Exp->template As<OpExpression>();
                if (ExpAsOp == nullptr) {
                    return false;
                }
                return (ExpAsOp->GetOpCode() == LTSOps::OpProject);
            }

            template <typename E, template <typename> class S>
            inline bool
            ArrayRecordSimplifier<E, S>::IsUpdate(const ExpressionBase<E, S>* Exp)
            {
                auto ExpAsOp = Exp->template As<OpExpression>();
                if (ExpAsOp == nullptr) {
                    return false;
                }
                return (ExpAsOp->GetOpCode() == LTSOps::OpUpdate);
            }


            template <typename E, template <typename> class S>
            inline void
            ArrayRecordSimplifier<E, S>::VisitAlienExpression(const ExpressionBase<E, S>* Exp)
            {
                Exp->Accept(ExprSimplifier);
                ExpStack.push(ExprSimplifier->ExpStack.top());
                ExprSimplifier->ExpStack.pop();
            }

            template <typename E, template <typename> class S>
            inline void
            ArrayRecordSimplifier<E, S>::VisitVarExpression(const VarExpression<E, S>* Exp)
            {
                VisitAlienExpression(Exp);
            }

            template <typename E, template <typename> class S>
            inline void
            ArrayRecordSimplifier<E, S>::VisitConstExpression(const ConstExpression<E, S>* Exp)
            {
                VisitAlienExpression(Exp);
            }

            template <typename E, template <typename> class S>
            inline void
            ArrayRecordSimplifier<E, S>::VisitBoundVarExpression(const
                                                                 BoundVarExpression<E, S>* Exp)
            {
                VisitAlienExpression(Exp);
            }

            template <typename E, template <typename> class S>
            inline void
            ArrayRecordSimplifier<E, S>::VisitEQuantifiedExpression(const
                                                                    EQuantifiedExpression<E, S>*
                                                                    Exp)
            {
                VisitAlienExpression(Exp);
            }

            template <typename E, template <typename> class S>
            inline void
            ArrayRecordSimplifier<E, S>::VisitAQuantifiedExpression(const
                                                                    AQuantifiedExpression<E, S>*
                                                                    Exp)
            {
                VisitAlienExpression(Exp);
            }

            template <typename E, template <typename> class S>
            inline bool
            ArrayRecordSimplifier<E, S>::IsIndexRelevant(const ExpT& IndexExp)
            {
                if (RelevantIndex == ExpT::NullPtr) {
                    return true;
                }

                if (!RelevantIndex->template Is<ConstExpression>() ||
                    !IndexExp->template Is<ConstExpression>()) {
                    return true;
                }

                return (RelevantIndex == IndexExp);
            }

            template <typename E, template <typename> class S>
            inline const typename ArrayRecordSimplifier<E, S>::ExpT&
            ArrayRecordSimplifier<E, S>::FindMatchingStore(const ExpT& IndexExp)
            {
                bool IndexIsConst = IndexExp->template Is<ConstExpression>();

                // The BACK of the store stack has the most recent store
                for (auto it = StoreStack.rbegin(); it != StoreStack.rend(); ++it) {
                    auto const& CurStore = *it;
                    auto const& CurIndex = CurStore.first;
                    auto const& CurVal = CurStore.second;

                    if (IndexIsConst && !CurIndex->template Is<ConstExpression>()) {
                        return ExpT::NullPtr;
                    }
                    if (IndexIsConst && CurIndex == IndexExp) {
                        return CurVal;
                    }
                    if (!IndexIsConst && CurIndex->template Is<ConstExpression>()) {
                        return ExpT::NullPtr;
                    }
                    if (!IndexIsConst && CurIndex == IndexExp) {
                        return CurVal;
                    }
                }

                return ExpT::NullPtr;
            }

            template <typename E, template <typename> class S>
            inline const typename ArrayRecordSimplifier<E, S>::ExpT&
            ArrayRecordSimplifier<E, S>::FindMatchingUpdate(const ExpT& FieldExp)
            {
                for (auto it = UpdateStack.rbegin(); it != UpdateStack.rend(); ++it) {
                    auto const& CurUpdate = *it;
                    if (CurUpdate.first == FieldExp) {
                        return CurUpdate.second;
                    }
                }

                return ExpT::NullPtr;
            }

            template <typename E, template <typename> class S>
            inline void
            ArrayRecordSimplifier<E, S>::VisitOpExpression(const OpExpression<E, S>* Exp)
            {
                auto OpCode = Exp->GetOpCode();
                auto const& Children = Exp->GetChildren();

                switch (OpCode) {
                case LTSOps::OpSelect: {
                    VisitAlienExpression(Children[1]);
                    auto SimpIndex = ExpStack.top();
                    ExpStack.pop();

                    if (IsStore(Children[0])) {
                        RelevantIndex = SimpIndex;
                        Children[0]->Accept(this);
                        RelevantIndex = ExpT::NullPtr;
                    } else {
                        VisitAlienExpression(Children[0]);
                    }

                    auto const& MatchingStore = FindMatchingStore(SimpIndex);
                    if (MatchingStore == ExpT::NullPtr) {
                        auto NewArrExp = ExpStack.top();
                        ExpStack.pop();
                        ExpStack.push(Mgr->MakeExpr(LTSOps::OpSelect, NewArrExp, SimpIndex));
                    } else {
                        ExpStack.pop();
                        ExpStack.push(MatchingStore);
                    }
                }
                    break;

                case LTSOps::OpStore: {
                    VisitAlienExpression(Children[1]);
                    auto SimpIndex = ExpStack.top();
                    ExpStack.pop();
                    if (KillSet.find(SimpIndex) != KillSet.end() ||
                        !IsIndexRelevant(SimpIndex)) {
                        if (IsStore(Children[0])) {
                            Children[0]->Accept(this);
                        } else {
                            VisitAlienExpression(Children[0]);
                        }
                        return;
                    } else if (IsStore(Children[0])) {
                        KillSet.insert(SimpIndex);
                        Children[0]->Accept(this);
                    } else {
                        VisitAlienExpression(Children[0]);
                    }
                    VisitAlienExpression(Children[2]);
                    auto NewValExp = ExpStack.top();
                    ExpStack.pop();
                    auto NewArrExp = ExpStack.top();
                    ExpStack.pop();
                    ExpStack.push(Mgr->MakeExpr(LTSOps::OpStore, NewArrExp,
                                                SimpIndex, NewValExp));
                    StoreStack.push_back(make_pair(SimpIndex, NewValExp));
                }
                    break;

                case LTSOps::OpProject: {
                    if (IsUpdate(Children[0])) {
                        RelevantField = Children[1];
                        Children[0]->Accept(this);
                        RelevantField = ExpT::NullPtr;
                    } else {
                        VisitAlienExpression(Children[0]);
                    }

                    auto const& MatchingUpdate = FindMatchingUpdate(Children[1]);
                    if (MatchingUpdate != ExpT::NullPtr) {
                        ExpStack.pop();
                        ExpStack.push(MatchingUpdate);
                    } else {
                        auto NewRecExp = ExpStack.top();
                        ExpStack.pop();
                        ExpStack.push(Mgr->MakeExpr(LTSOps::OpProject, NewRecExp, Children[1]));
                    }
                }
                    break;

                case LTSOps::OpUpdate: {
                    if (KillSet.find(Children[1]) != KillSet.end() ||
                        (RelevantField != ExpT::NullPtr &&
                         RelevantField != Children[1])) {
                        if (IsUpdate(Children[0])) {
                            Children[0]->Accept(this);
                        } else {
                            VisitAlienExpression(Children[0]);
                        }
                        return;
                    } else if (IsUpdate(Children[0])) {
                        KillSet.insert(Children[1]);
                        Children[0]->Accept(this);
                    } else {
                        VisitAlienExpression(Children[0]);
                    }
                    VisitAlienExpression(Children[2]);
                    auto NewValExp = ExpStack.top();
                    ExpStack.pop();
                    auto NewRecExp = ExpStack.top();
                    ExpStack.pop();
                    ExpStack.push(Mgr->MakeExpr(LTSOps::OpUpdate, NewRecExp,
                                                Children[1], NewValExp));
                    UpdateStack.push_back(make_pair(Children[1], NewValExp));
                }
                    break;

                default: {
                    VisitAlienExpression(Exp);
                }
                }
            }

            template <typename E, template <typename> class S>
            inline typename ArrayRecordSimplifier<E, S>::ExpT
            ArrayRecordSimplifier<E, S>::Do(MgrType* Mgr,
                                            Simplifier<E, S>* ExprSimplifier,
                                            const ExpT& Exp)
            {
                ArrayRecordSimplifier<E, S> TheSimplifier(Mgr, ExprSimplifier);
                if (!TheSimplifier.IsStore(Exp) && !TheSimplifier.IsSelect(Exp) &&
                    !TheSimplifier.IsUpdate(Exp) && !TheSimplifier.IsProject(Exp)) {
                    throw InternalError((string)"ArrayRecordSimplifier::Do() called on " +
                                        "non array or record expression. Expression:\n" +
                                        Exp->ToString() + "\nAt: " + __FILE__ + ":" +
                                        to_string(__LINE__));
                }
                Exp->Accept(&TheSimplifier);
                return TheSimplifier.ExpStack.top();
            }

            // Implementation of Lowerer
            template <typename E, template <typename> class S>
            Lowerer<E, S>::Lowerer(const UFID2TypeMapT& UFMap, const LTSLCRef& LTSCtx)
                : ExpressionVisitorBase<E, S>("LTSZ3Lowerer"), UFMap(UFMap), LTSCtx(LTSCtx)
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            Lowerer<E, S>::~Lowerer()
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            inline const Z3Sort&
            Lowerer<E, S>::LowerType(const TypeRef& Type)
            {
                auto Ctx = LTSCtx->GetZ3Ctx();
                auto const& CachedSort = LTSCtx->GetZ3Sort(Type);
                if (CachedSort != Z3Sort::NullSort) {
                    return CachedSort;
                }

                Z3Sort LoweredSort;
                // We need to actually lower this
                if (Type->template Is<BooleanType>()) {
                    LoweredSort = Z3Sort(Ctx, Z3_mk_bool_sort(*Ctx));
                } else if (Type->template Is<RangeType>()) {
                    LoweredSort = Z3Sort(Ctx, Z3_mk_int_sort(*Ctx));
                } else if (Type->template Is<EnumType>()) {

                    auto TypeAsEnum = Type->template SAs<EnumType>();
                    auto const& Members = TypeAsEnum->GetElements();
                    auto const& TypeName = TypeAsEnum->GetName();
                    vector<string> QualifiedNames;
                    for (auto const& Member : Members) {
                        QualifiedNames.push_back(TypeName + "::" + Member);
                    }
                    const u32 NumConsts = QualifiedNames.size();
                    Z3_symbol Z3TypeName = Z3_mk_string_symbol(*Ctx, TypeName.c_str());
                    Z3_symbol* ConstNames = new Z3_symbol[NumConsts];
                    Z3_func_decl* ConstFuncs = new Z3_func_decl[NumConsts];
                    Z3_func_decl* ConstTests = new Z3_func_decl[NumConsts];

                    for (u32 i = 0; i < NumConsts; ++i) {
                        ConstNames[i] = Z3_mk_string_symbol(*Ctx, QualifiedNames[i].c_str());
                    }

                    auto Z3EnumSort = Z3_mk_enumeration_sort(*Ctx, Z3TypeName, NumConsts,
                                                             ConstNames, ConstFuncs, ConstTests);

                    LoweredSort = Z3Sort(Ctx, Z3EnumSort);
                    LoweredSort.AddFuncDecls(NumConsts, ConstFuncs);
                    LoweredSort.AddFuncDecls(NumConsts, ConstTests);

                    delete[] ConstNames;
                    delete[] ConstFuncs;
                    delete[] ConstTests;

                } else if (Type->template Is<SymmetricType>()) {
                    auto TypeAsSymm = Type->template SAs<EnumType>();
                    auto const& Members = TypeAsSymm->GetElements();
                    auto const& TypeName = TypeAsSymm->GetName();
                    const u32 NumConsts = Members.size();
                    Z3_symbol Z3TypeName = Z3_mk_string_symbol(*Ctx, TypeName.c_str());

                    // We need to add a constructor for the undef value
                    Z3_symbol* ConstNames = new Z3_symbol[NumConsts];
                    Z3_func_decl* ConstFuncs = new Z3_func_decl[NumConsts];
                    Z3_func_decl* ConstTests = new Z3_func_decl[NumConsts];

                    for (u32 i = 0; i < NumConsts; ++i) {
                        ConstNames[i] = Z3_mk_string_symbol(*Ctx, Members[i].c_str());
                    }

                    auto Z3EnumSort = Z3_mk_enumeration_sort(*Ctx, Z3TypeName, NumConsts,
                                                             ConstNames, ConstFuncs, ConstTests);

                    LoweredSort = Z3Sort(Ctx, Z3EnumSort);
                    LoweredSort.AddFuncDecls(NumConsts, ConstFuncs);
                    LoweredSort.AddFuncDecls(NumConsts, ConstTests);

                    delete[] ConstNames;
                    delete[] ConstFuncs;
                    delete[] ConstTests;

                } else if (Type->template Is<RecordType>() ||
                           Type->template Is<ParametricType>()) {
                    const RecordType* TypeAsRec = nullptr;
                    if (Type->template Is<RecordType>()) {
                        TypeAsRec = Type->SAs<RecordType>();
                    } else {
                        TypeAsRec =
                            Type->SAs<ParametricType>()->GetBaseType()->SAs<RecordType>();
                    }
                    auto const& Members = TypeAsRec->GetMemberVec();
                    const u32 NumFields = Members.size();

                    Z3_symbol Z3TypeName = Z3_mk_string_symbol(*Ctx, TypeAsRec->GetName().c_str());
                    Z3_symbol* FieldNames = new Z3_symbol[NumFields];
                    Z3_sort* FieldSorts = new Z3_sort[NumFields];
                    Z3_func_decl* ProjFuncs = new Z3_func_decl[NumFields];
                    Z3_func_decl Constructor;

                    for (u32 i = 0; i < NumFields; ++i) {
                        FieldSorts[i] = LowerType(Members[i].second);
                        FieldNames[i] = Z3_mk_string_symbol(*Ctx, Members[i].first.c_str());
                    }

                    auto Z3RecSort = Z3_mk_tuple_sort(*Ctx, Z3TypeName, NumFields,
                                                      FieldNames, FieldSorts,
                                                      &Constructor, ProjFuncs);

                    LoweredSort = Z3Sort(Ctx, Z3RecSort);
                    LoweredSort.AddFuncDecl(Constructor);
                    LoweredSort.AddFuncDecls(NumFields, ProjFuncs);

                    delete[] FieldNames;
                    delete[] FieldSorts;
                    delete[] ProjFuncs;

                } else if (Type->template Is<ArrayType>()) {

                    auto TypeAsArr = Type->template SAs<ArrayType>();
                    auto LoweredIdxSort = LowerType(TypeAsArr->GetIndexType());
                    auto LoweredValSort = LowerType(TypeAsArr->GetValueType());
                    auto Z3ArrSort = Z3_mk_array_sort(*Ctx, LoweredIdxSort, LoweredValSort);
                    LoweredSort = Z3Sort(Ctx, Z3ArrSort);

                } else {
                    throw ESMCError((string)"Cannot lower type \"" + Type->ToString() +
                                    "\" into a Z3 type. Perhaps it's unbounded?");
                }

                LTSCtx->AddZ3Sort(Type, LoweredSort);
                return LTSCtx->GetZ3Sort(Type);
            }

            template <typename E, template <typename> class S>
            inline void
            Lowerer<E, S>::VisitVarExpression(const VarExpression<E, S>* Exp)
            {
                auto const& Type = Exp->GetVarType();
                auto const& Name = Exp->GetVarName();
                if (LTSCtx->GetLTSType(Name) == TypeRef::NullPtr) {
                    LTSCtx->AddLTSType(Name, Type);
                }

                auto Ctx = LTSCtx->GetZ3Ctx();

                auto LoweredType = LowerType(Type);
                auto Z3Sym = Z3_mk_string_symbol(*Ctx, Name.c_str());
                Z3Expr LoweredExpr(Ctx, Z3_mk_const(*Ctx, Z3Sym, LoweredType));

                if (Type->template Is<RangeType>()) {
                    auto TypeAsRange = Type->template SAs<RangeType>();
                    auto LowString = to_string(TypeAsRange->GetLow());
                    auto HighString = to_string(TypeAsRange->GetHigh());
                    auto LowConst = Z3Expr(Ctx, Z3_mk_numeral(*Ctx, LowString.c_str(),
                                                               Z3_mk_int_sort(*Ctx)));
                    auto HighConst = Z3Expr(Ctx, Z3_mk_numeral(*Ctx, HighString.c_str(),
                                                                Z3_mk_int_sort(*Ctx)));

                    auto LowExp = Z3Expr(Ctx, Z3_mk_ge(*Ctx, LoweredExpr, LowConst));
                    auto HighExp = Z3Expr(Ctx, Z3_mk_le(*Ctx, LoweredExpr, HighConst));

                    auto AndArgs = new Z3_ast[2];
                    AndArgs[0] = LowExp;
                    AndArgs[1] = HighExp;

                    LTSCtx->AddAssumptionGlobal(Z3Expr(Ctx, Z3_mk_and(*Ctx, 2, AndArgs)));
                    delete[] AndArgs;
                }

                ExpStack.push(LoweredExpr);
            }

            template <typename E, template <typename> class S>
            inline void
            Lowerer<E, S>::VisitConstExpression(const ConstExpression<E, S>* Exp)
            {
                auto const& Type = Exp->GetConstType();
                auto const& Val = Exp->GetConstValue();
                auto Ctx = LTSCtx->GetZ3Ctx();

                if (Type->template Is<IntegerType>()) {
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_numeral(*Ctx, Val.c_str(),
                                                                 Z3_mk_int_sort(*Ctx))));
                } else if (Type->template Is<BooleanType>()) {

                    if (Val == "true") {
                        ExpStack.push(Z3Expr(Ctx, Z3_mk_true(*Ctx)));
                    } else {
                        ExpStack.push(Z3Expr(Ctx, Z3_mk_false(*Ctx)));
                    }

                } else if (Type->template Is<SymmetricType>()) {
                    string ActualVal;
                    if (Val == "clear") {
                        ActualVal = Type->template SAs<SymmetricType>()->GetName() +
                            "::clear";
                    } else {
                        ActualVal = Val;
                    }
                    auto const& LoweredType = LowerType(Type);
                    auto Decl = LoweredType.GetFuncDecl(ActualVal);
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_app(*Ctx, Decl, 0, nullptr)));
                } else if (Type->template Is<EnumType>()) {
                    // enum constants are unqualified
                    auto const& LoweredType = LowerType(Type);
                    auto TypeAsEnum = Type->template SAs<EnumType>();
                    string QualifiedVal = TypeAsEnum->GetName() + "::" + Val;
                    auto Decl = LoweredType.GetFuncDecl(QualifiedVal);
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_app(*Ctx, Decl, 0, nullptr)));
                } else {
                    throw ESMCError((string)"Unexpected constant of type \"" + Type->ToString() +
                                    "\". Cannot lower this kind of constant into a Z3Expr");
                }
            }

            template <typename E, template <typename> class S>
            inline void
            Lowerer<E, S>::VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp)
            {
                auto Ctx = LTSCtx->GetZ3Ctx();
                auto const& Type = Exp->GetVarType();
                auto Index = Exp->GetVarIdx();

                auto LoweredSort = LowerType(Type);
                auto LoweredExpr = Z3Expr(Ctx, Z3_mk_bound(*Ctx, Index, LoweredSort));
                ExpStack.push(LoweredExpr);

                if (Type->template Is<RangeType>()) {
                    auto TypeAsRange = Type->template SAs<RangeType>();
                    auto LowString = to_string(TypeAsRange->GetLow());
                    auto HighString = to_string(TypeAsRange->GetHigh());
                    auto LowConst = Z3Expr(Ctx, Z3_mk_numeral(*Ctx, LowString.c_str(),
                                                               Z3_mk_int_sort(*Ctx)));
                    auto HighConst = Z3Expr(Ctx, Z3_mk_numeral(*Ctx, HighString.c_str(),
                                                                Z3_mk_int_sort(*Ctx)));

                    auto LowExp = Z3Expr(Ctx, Z3_mk_ge(*Ctx, LoweredExpr, LowConst));
                    auto HighExp = Z3Expr(Ctx, Z3_mk_le(*Ctx, LoweredExpr, HighConst));

                    auto AndArgs = new Z3_ast[2];
                    AndArgs[0] = LowExp;
                    AndArgs[1] = HighExp;

                    LTSCtx->AddAssumption(Z3Expr(Ctx, Z3_mk_and(*Ctx, 2, AndArgs)));
                    delete[] AndArgs;
                }
            }

            template <typename E, template <typename> class S>
            inline void
            Lowerer<E, S>::VisitOpExpression(const OpExpression<E, S>* Exp)
            {
                auto const& Children = Exp->GetChildren();
                const u32 NumChildren = Children.size();

                auto Ctx = LTSCtx->GetZ3Ctx();
                auto OpCode = Exp->GetOpCode();

                auto LoweredChildren = new Z3_ast[NumChildren];
                vector<Z3Expr> LChildren(NumChildren);

                if (OpCode != LTSOps::OpField &&
                    OpCode != LTSOps::OpProject &&
                    OpCode != LTSOps::OpUpdate) {
                    ExpressionVisitorBase<E, S>::VisitOpExpression(Exp);

                    for (u32 i = 0; i < NumChildren; ++i) {
                        LChildren[NumChildren - i - 1] = ExpStack.top();
                        LoweredChildren[NumChildren - i - 1] = ExpStack.top();
                        ExpStack.pop();
                    }
                }

                switch (OpCode) {
                case LTSOps::OpEQ:
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_eq(*Ctx, LChildren[0], LChildren[1])));
                    break;

                case LTSOps::OpNOT:
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_not(*Ctx, LChildren[0])));
                    break;

                case LTSOps::OpITE:
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_ite(*Ctx, LChildren[0],
                                                             LChildren[1], LChildren[2])));
                    break;

                case LTSOps::OpOR:
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_or(*Ctx, NumChildren, LoweredChildren)));
                    break;

                case LTSOps::OpAND:
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_and(*Ctx, NumChildren, LoweredChildren)));
                    break;

                case LTSOps::OpIMPLIES:
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_implies(*Ctx, LChildren[0], LChildren[1])));
                    break;

                case LTSOps::OpIFF:
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_iff(*Ctx, LChildren[0], LChildren[1])));
                    break;

                case LTSOps::OpXOR:
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_xor(*Ctx, LChildren[0], LChildren[1])));
                    break;

                case LTSOps::OpADD:
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_add(*Ctx, NumChildren, LoweredChildren)));
                    break;

                case LTSOps::OpSUB:
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_sub(*Ctx, NumChildren, LoweredChildren)));
                    break;

                case LTSOps::OpMINUS:
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_unary_minus(*Ctx, LChildren[0])));
                    break;

                case LTSOps::OpMUL:
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_mul(*Ctx, NumChildren, LoweredChildren)));
                    break;

                case LTSOps::OpDIV:
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_div(*Ctx, LChildren[0], LChildren[1])));
                    break;

                case LTSOps::OpMOD:
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_mod(*Ctx, LChildren[0], LChildren[1])));
                    break;

                case LTSOps::OpGT:
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_gt(*Ctx, LChildren[0], LChildren[1])));
                    break;

                case LTSOps::OpGE:
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_ge(*Ctx, LChildren[0], LChildren[1])));
                    break;

                case LTSOps::OpLT:
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_lt(*Ctx, LChildren[0], LChildren[1])));
                    break;

                case LTSOps::OpLE:
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_le(*Ctx, LChildren[0], LChildren[1])));
                    break;

                case LTSOps::OpIndex:
                case LTSOps::OpSelect: {
                    auto const& ExpType = Exp->GetType();
                    auto ExpTypeAsRange = ExpType->template As<RangeType>();
                    Z3Expr LoweredExpr(Ctx, Z3_mk_select(*Ctx, LChildren[0], LChildren[1]));

                    if (ExpTypeAsRange != nullptr) {
                        auto RangeLow = ExpTypeAsRange->GetLow();
                        auto RangeHigh = ExpTypeAsRange->GetHigh();
                        Z3_ast AndArgs[2];

                        auto LowString = to_string(RangeLow);
                        auto HighString = to_string(RangeHigh);

                        auto LowConst = Z3Expr(Ctx, Z3_mk_numeral(*Ctx, LowString.c_str(),
                                                                  Z3_mk_int_sort(*Ctx)));
                        auto HighConst = Z3Expr(Ctx, Z3_mk_numeral(*Ctx, HighString.c_str(),
                                                                   Z3_mk_int_sort(*Ctx)));

                        auto LowExp = Z3Expr(Ctx, Z3_mk_ge(*Ctx, LoweredExpr, LowConst));
                        auto HighExp = Z3Expr(Ctx, Z3_mk_le(*Ctx, LoweredExpr, HighConst));

                        AndArgs[0] = LowExp;
                        AndArgs[1] = HighExp;

                        LTSCtx->AddAssumption(Z3Expr(Ctx, Z3_mk_and(*Ctx, 2, AndArgs)));
                    }
                    ExpStack.push(LoweredExpr);
                    break;
                }

                case LTSOps::OpField:
                case LTSOps::OpProject: {
                    // Visit the record expression
                    Children[0]->Accept(this);
                    auto RecExp = ExpStack.top();
                    auto RecType = Children[0]->GetType();
                    auto const& LoweredRecSort = LowerType(RecType);
                    ExpStack.pop();

                    auto const& FieldName =
                        Children[1]->template As<ESMC::Exprs::VarExpression>()->GetVarName();

                    auto FieldFunc = LoweredRecSort.GetFuncDecl(FieldName);
                    Z3_ast FuncArgs[1];
                    FuncArgs[0] = RecExp;
                    auto LoweredExpr = Z3Expr(Ctx, Z3_mk_app(*Ctx, FieldFunc, 1, FuncArgs));
                    ExpStack.push(LoweredExpr);

                    auto const& ExpType = Exp->GetType();
                    auto ExpTypeAsRange = ExpType->template As<RangeType>();
                    if (ExpTypeAsRange != nullptr) {
                        auto RangeLow = ExpTypeAsRange->GetLow();
                        auto RangeHigh = ExpTypeAsRange->GetHigh();
                        Z3_ast AndArgs[2];

                        auto LowString = to_string(RangeLow);
                        auto HighString = to_string(RangeHigh);

                        auto LowConst = Z3Expr(Ctx, Z3_mk_numeral(*Ctx, LowString.c_str(),
                                                                  Z3_mk_int_sort(*Ctx)));
                        auto HighConst = Z3Expr(Ctx, Z3_mk_numeral(*Ctx, HighString.c_str(),
                                                                   Z3_mk_int_sort(*Ctx)));

                        auto LowExp = Z3Expr(Ctx, Z3_mk_ge(*Ctx, LoweredExpr, LowConst));
                        auto HighExp = Z3Expr(Ctx, Z3_mk_le(*Ctx, LoweredExpr, HighConst));

                        AndArgs[0] = LowExp;
                        AndArgs[1] = HighExp;

                        LTSCtx->AddAssumption(Z3Expr(Ctx, Z3_mk_and(*Ctx, 2, AndArgs)));
                    }

                    break;
                }

                case LTSOps::OpStore: {
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_store(*Ctx, LChildren[0],
                                                               LChildren[1], LChildren[2])));
                    break;
                }

                case LTSOps::OpUpdate: {
                    Children[0]->Accept(this);
                    auto RecExp = ExpStack.top();
                    ExpStack.pop();

                    Children[2]->Accept(this);
                    auto UpdateExp = ExpStack.top();
                    ExpStack.pop();

                    auto RecType = Children[0]->GetType();
                    auto const& RecTypeName = RecType->template SAs<RecordType>()->GetName();
                    auto const& RecMembers = RecType->template SAs<RecordType>()->GetMemberVec();
                    const u32 NumMembers = RecMembers.size();
                    auto const& LoweredRecSort = LowerType(RecType);
                    auto const& FieldName = Children[1]->template As<VarExpression>()->GetVarName();

                    vector<Z3Expr> AppArgVec(NumMembers);
                    Z3_ast* AppArgs = new Z3_ast[NumMembers];

                    for (u32 i = 0; i < NumMembers; ++i) {
                        auto const& Member = RecMembers[i];
                        if (Member.first == FieldName) {
                            AppArgVec[i] = UpdateExp;
                            AppArgs[i] = UpdateExp;
                        } else {
                            auto ProjectFunc = LoweredRecSort.GetFuncDecl(Member.first);
                            Z3_ast ProjAppArgs[1];
                            ProjAppArgs[0] = RecExp;
                            Z3Expr ProjectExpr(Ctx, Z3_mk_app(*Ctx, ProjectFunc, 1, ProjAppArgs));
                            AppArgVec[i] = ProjectExpr;
                            AppArgs[i] = ProjectExpr;
                        }
                    }

                    auto ConstructFunc = LoweredRecSort.GetFuncDecl(RecTypeName);
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_app(*Ctx, ConstructFunc,
                                                             NumMembers, AppArgs)));
                    delete[] AppArgs;
                }
                    break;


                default: {
                    // This must be an uninterpreted function
                    auto it = UFMap.find(OpCode);
                    if (it == UFMap.end()) {
                        throw ExprTypeError((string)"Opcode " + to_string(OpCode) +
                                            " unrecognized. " +
                                            "This is not a standard operation, nor has it been " +
                                            "registered as an uninterpreted function");
                    }
                    // create an uninterpreted function application
                    auto FunType = it->second->template As<FuncType>();
                    auto const& DomainTypes = FunType->GetArgTypes();
                    const u32 DomainSize = DomainTypes.size();
                    Z3_sort* DomainSorts = new Z3_sort[DomainSize];
                    for (u32 i = 0; i < DomainSize; ++i) {
                        DomainSorts[i] = LowerType(DomainTypes[i]);
                    }
                    auto const& RType = FunType->GetEvalType();
                    Z3Sort RangeSort = LowerType(RType);
                    auto MangledFuncName = MangleName(FunType->GetName(), DomainTypes);
                    auto FuncSym = Z3_mk_string_symbol(*Ctx, MangledFuncName.c_str());
                    auto FuncDecl = Z3_mk_func_decl(*Ctx, FuncSym, DomainSize,
                                                    DomainSorts, RangeSort);
                    Z3_inc_ref(*Ctx, Z3_func_decl_to_ast(*Ctx, FuncDecl));
                    delete[] DomainSorts;

                    // Make the application
                    auto AppExp = Z3Expr(Ctx, Z3_mk_app(*Ctx, FuncDecl,
                                                        NumChildren,
                                                        LoweredChildren));

                    Z3_dec_ref(*Ctx, Z3_func_decl_to_ast(*Ctx, FuncDecl));

                    ExpStack.push(AppExp);
                    break;
                }
                }

                delete[] LoweredChildren;
            }

            template <typename E, template <typename> class S>
            inline void
            Lowerer<E, S>::VisitQuantifiedExpression(const QuantifiedExpressionBase<E, S>* Exp)
            {
                auto Ctx = LTSCtx->GetZ3Ctx();
                LTSCtx->PushAssumptionScope();
                Exp->GetQExpression()->Accept(this);

                auto LoweredQExpr = ExpStack.top();
                ExpStack.pop();

                // Get the assumptions made
                auto&& Assumptions = LTSCtx->PopAssumptionScope();
                Z3_ast* AndArgs = new Z3_ast[Assumptions.size()];
                u32 i = 0;
                for (auto const& Assumption : Assumptions) {
                    AndArgs[i++] = Assumption;
                }
                auto AssumptionLExp = Z3Expr(Ctx, Z3_mk_and(*Ctx, Assumptions.size(),
                                                            AndArgs));
                delete[] AndArgs;

                auto const& QVarTypes = Exp->GetQVarTypes();
                const u32 NumQVars = QVarTypes.size();

                Z3_sort* QVarSorts = new Z3_sort[NumQVars];
                Z3_symbol* QVarNames = new Z3_symbol[NumQVars];

                for (u32 i = 0; i < NumQVars; ++i) {
                    QVarSorts[i] = LowerType(QVarTypes[i]);
                    QVarNames[i] = Z3_mk_string_symbol(*Ctx, ("DBVar_" + to_string(i)).c_str());
                }

                Z3_ast AndArgs2[2];
                AndArgs2[0] = AssumptionLExp;
                AndArgs2[1] = LoweredQExpr;

                auto ActualBody = Z3Expr(Ctx, Z3_mk_and(*Ctx, 2, AndArgs2));

                if (Exp->IsForAll()) {
                    auto ActualBody = Z3Expr(Ctx,
                                             Z3_mk_implies(*Ctx, AssumptionLExp,
                                                           LoweredQExpr));
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_forall(*Ctx, 0, 0, nullptr,
                                                                NumQVars, QVarSorts,
                                                                QVarNames, ActualBody)));
                } else {
                    Z3_ast AndArgs3[2];
                    AndArgs3[0] = AssumptionLExp;
                    AndArgs3[1] = LoweredQExpr;
                    auto ActualBody = Z3Expr(Ctx,
                                             Z3_mk_and(*Ctx, 2, AndArgs3));
                    ExpStack.push(Z3Expr(Ctx, Z3_mk_exists(*Ctx, 0, 0, nullptr,
                                                                NumQVars, QVarSorts,
                                                                QVarNames, ActualBody)));
                }

                delete[] QVarSorts;
                delete[] QVarNames;
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
            inline Z3Expr
            Lowerer<E, S>::Do(const ExpT& Exp, const UFID2TypeMapT& UFMap,
                              const LTSLCRef& LTSCtx)
            {
                auto Ctx = LTSCtx->GetZ3Ctx();
                Lowerer TheLowerer(UFMap, LTSCtx);
                Exp->Accept(&TheLowerer);
                // Add in the assumptions
                // auto&& Assumptions = LTSCtx->PopAssumptionScope();
                // const u32 NumAssumptions = Assumptions.size();
                // Z3_ast* AndArgs = new Z3_ast[NumAssumptions + 1];
                // for (u32 i = 0; i < NumAssumptions; ++i) {
                //     AndArgs[i] = Assumptions[i];
                // }
                // AndArgs[NumAssumptions] = TheLowerer.ExpStack.top();

                // auto Retval = Z3Expr(Ctx, Z3_mk_and(*Ctx, NumAssumptions + 1, AndArgs));
                // delete[] AndArgs;
                auto Retval = TheLowerer.ExpStack.top();
                return Retval;
            }


            // Implementation of Raiser
            template <typename E, template <typename> class S>
            Raiser<E, S>::Raiser(const unordered_map<string, i64>& UFNameToIDMap,
                                 const LTSLCRef& LTSCtx, MgrT* Mgr)
                : UFNameToIDMap(UFNameToIDMap), LTSCtx(LTSCtx), Mgr(Mgr)
            {
                Ctx = LTSCtx->GetZ3Ctx();
            }

            template <typename E, template <typename> class S>
            Raiser<E, S>::~Raiser()
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            typename Raiser<E, S>::ExpT
            Raiser<E, S>::RaiseOpExpr(const LExpT& LExp)
            {
                auto App = Z3_to_app(*Ctx, LExp);
                auto AppDecl = Z3_get_app_decl(*Ctx, App);
                auto DeclSym = Z3_get_decl_name(*Ctx, AppDecl);
                string DeclName(Z3_get_symbol_string(*Ctx, DeclSym));
                const u32 NumArgs = Z3_get_domain_size(*Ctx, AppDecl);
                auto AppKind = Z3_get_decl_kind(*Ctx, AppDecl);

                vector<ExpT> RaisedChildren(NumArgs);
                for (u32 i = 0; i < NumArgs; ++i) {
                    RaisedChildren[i] = RaiseExpr(Z3Expr(Ctx, Z3_get_app_arg(*Ctx, App, i)));
                }

                switch (AppKind) {
                case Z3_OP_UNINTERPRETED: {
                    if (NumArgs == 0) {
                        // A variable
                        auto Type = LTSCtx->GetLTSType(DeclName);
                        if (Type == TypeRef::NullPtr) {
                            throw ExprTypeError((string)"Could not resolve type of variable \"" +
                                                DeclName + "\" from the LTS context when " +
                                                "trying to raise Z3 expression:\n" +
                                                Z3_ast_to_string(*Ctx, LExp));
                        }

                        return Mgr->MakeVar(DeclName, Type);
                    } else {
                        // An actual uninterpreted function
                        auto it = UFNameToIDMap.find(DeclName);
                        if (it == UFNameToIDMap.end()) {
                            throw ExprTypeError((string)"Could not resolve mangled " +
                                                "uninterpreted function \"" + DeclName + "\", " +
                                                "when trying to raise Z3 expression:\n" +
                                                Z3_ast_to_string(*Ctx, LExp));
                        }
                        auto OpCode = it->second;
                        return Mgr->MakeExpr(OpCode, RaisedChildren);
                    }
                }

                case Z3_OP_TRUE:
                    return Mgr->MakeTrue();

                case Z3_OP_FALSE:
                    return Mgr->MakeFalse();

                case Z3_OP_EQ:
                    return Mgr->MakeExpr(LTSOps::OpEQ, RaisedChildren);

                case Z3_OP_ITE:
                    return Mgr->MakeExpr(LTSOps::OpITE, RaisedChildren);

                case Z3_OP_OR:
                    return Mgr->MakeExpr(LTSOps::OpOR, RaisedChildren);

                case Z3_OP_AND:
                    return Mgr->MakeExpr(LTSOps::OpAND, RaisedChildren);

                case Z3_OP_IMPLIES:
                    return Mgr->MakeExpr(LTSOps::OpIMPLIES, RaisedChildren);

                case Z3_OP_IFF:
                    return Mgr->MakeExpr(LTSOps::OpIFF, RaisedChildren);

                case Z3_OP_XOR:
                    return Mgr->MakeExpr(LTSOps::OpXOR, RaisedChildren);

                case Z3_OP_NOT:
                    return Mgr->MakeExpr(LTSOps::OpNOT, RaisedChildren);

                case Z3_OP_ADD:
                    return Mgr->MakeExpr(LTSOps::OpADD, RaisedChildren);

                case Z3_OP_SUB:
                    return Mgr->MakeExpr(LTSOps::OpSUB, RaisedChildren);

                case Z3_OP_UMINUS:
                    return Mgr->MakeExpr(LTSOps::OpMINUS, RaisedChildren);

                case Z3_OP_MUL:
                    return Mgr->MakeExpr(LTSOps::OpMUL, RaisedChildren);

                case Z3_OP_DIV:
                    return Mgr->MakeExpr(LTSOps::OpDIV, RaisedChildren);

                case Z3_OP_MOD:
                    return Mgr->MakeExpr(LTSOps::OpMOD, RaisedChildren);

                case Z3_OP_LT:
                    return Mgr->MakeExpr(LTSOps::OpLT, RaisedChildren);

                case Z3_OP_LE:
                    return Mgr->MakeExpr(LTSOps::OpLE, RaisedChildren);

                case Z3_OP_GT:
                    return Mgr->MakeExpr(LTSOps::OpGT, RaisedChildren);

                case Z3_OP_GE:
                    return Mgr->MakeExpr(LTSOps::OpGE, RaisedChildren);

                case Z3_OP_SELECT:
                    return Mgr->MakeExpr(LTSOps::OpIndex, RaisedChildren);

                case Z3_OP_DT_RECOGNISER: {
                    auto FAType = Mgr->template MakeType<FieldAccessType>();
                    return Mgr->MakeExpr(LTSOps::OpField, RaisedChildren[0],
                                         Mgr->MakeVar(DeclName, FAType));
                }

                case Z3_OP_DT_CONSTRUCTOR: {
                    // Find out the type name
                    vector<string> SplitComps;
                    boost::algorithm::split(SplitComps, DeclName,
                                            boost::algorithm::is_any_of(":"),
                                            boost::algorithm::token_compress_on);
                    auto TypeName = SplitComps[0];
                    auto Type = Mgr->GetNamedType(TypeName);
                    if (Type == TypeRef::NullPtr) {
                        throw InternalError((string)"Could not resolve type \"" + TypeName +
                                            "\" when attempting to raise expression:\n" +
                                            Z3_ast_to_string(*Ctx, LExp));
                    }
                    if (SplitComps[1] == "clear") {
                        return Mgr->MakeVal("clear", Type);
                    } else {
                        return Mgr->MakeVal(DeclName, Type);
                    }
                }

                default:
                    throw InternalError((string)"Unknown/Unhandled Z3 op kind with code: " +
                                        to_string(AppKind) + "\nAt: " + __FILE__ + ":" +
                                        to_string(__LINE__));
                }
            }

            // can only be integers
            template <typename E, template <typename> class S>
            typename Raiser<E, S>::ExpT
            Raiser<E, S>::RaiseConstExpr(const LExpT& LExp)
            {
                auto Sort = Z3_get_sort(*Ctx, LExp);
                auto Kind = Z3_get_sort_kind(*Ctx, Sort);
                if (Kind == Z3_INT_SORT) {
                    return Mgr->MakeVal(Z3_get_numeral_string(*Ctx, LExp),
                                        Mgr->template MakeType<IntegerType>());
                } else {
                    throw ExprTypeError((string)"Unknown numeral kind " + to_string(Kind) +
                                        ", when raising Z3 expression:\n" +
                                        Z3_ast_to_string(*Ctx, LExp));
                }
            }

            template <typename E, template <typename> class S>
            typename Raiser<E, S>::ExpT
            Raiser<E, S>::RaiseExpr(const LExpT& LExp)
            {
                auto Kind = Z3_get_ast_kind(*Ctx, LExp);
                switch (Kind) {
                case Z3_APP_AST:
                    return RaiseOpExpr(LExp);
                case Z3_NUMERAL_AST:
                    return RaiseConstExpr(LExp);
                case Z3_VAR_AST:
                case Z3_QUANTIFIER_AST:
                    throw ExprTypeError((string)"Bound variables and quantified expressions " +
                                        "cannot be raised");
                default:
                    throw ExprTypeError((string)"Unhandled Z3 AST kind: " +
                                        to_string(Kind) + ". The Z3 expression is:\n" +
                                        Z3_ast_to_string(*Ctx, LExp) + "\nThis was unexpected." +
                                        "\nAt: " + __FILE__ + ":" + to_string(__LINE__));
                }
            }

            template <typename E, template <typename> class S>
            inline typename Raiser<E, S>::ExpT
            Raiser<E, S>::Do(const LExpT& LExp, const unordered_map<string, i64>& UFNameToIDMap,
                             const LTSLCRef& LTSCtx, MgrT* Mgr)
            {
                Raiser TheRaiser(UFNameToIDMap, LTSCtx, Mgr);
                auto Retval = TheRaiser.RaiseExpr(LExp);
                return Retval;
            }

            template <typename E, template <typename> class S>
            inline DeBruijnShifter<E, S>::DeBruijnShifter(MgrT* Mgr, i32 ShiftValue)
                : ExpressionVisitorBase<E, S>("DeBruijnShifter"),
                  Mgr(Mgr), ShiftValue(ShiftValue)
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            inline DeBruijnShifter<E, S>::~DeBruijnShifter()
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            inline void
            DeBruijnShifter<E, S>::VisitVarExpression(const VarExpression<E, S>* Exp)
            {
                ExpStack.push(Exp);
            }

            template <typename E, template <typename> class S>
            inline void
            DeBruijnShifter<E, S>::VisitConstExpression(const ConstExpression<E, S>* Exp)
            {
                ExpStack.push(Exp);
            }

            template <typename E, template <typename> class S>
            inline void
            DeBruijnShifter<E, S>::VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp)
            {
                auto OldIdx = Exp->GetVarIdx();
                auto const& Type = Exp->GetVarType();
                ExpStack.push(Mgr->MakeBoundVar(Type, OldIdx + ShiftValue));
            }

            template <typename E, template <typename> class S>
            inline void
            DeBruijnShifter<E, S>::VisitOpExpression(const OpExpression<E, S>* Exp)
            {
                ExpressionVisitorBase<E, S>::VisitOpExpression(Exp);
                auto const& OldChildren = Exp->GetChildren();
                const u32 NumChildren = OldChildren.size();
                vector<ExpT> NewChildren(NumChildren);

                for (u32 i = 0; i < NumChildren; ++i) {
                    NewChildren[NumChildren - i - 1] = ExpStack.top();
                    ExpStack.pop();
                }

                ExpStack.push(Mgr->MakeExpr(Exp->GetOpCode(), NewChildren));
            }

            template <typename E, template <typename> class S>
            inline void
            DeBruijnShifter<E, S>::VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>* Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto NewBody = ExpStack.top();
                ExpStack.pop();
                auto const& QVarTypes = Exp->GetQVarTypes();
                ExpStack.push(Mgr->MakeExists(QVarTypes, NewBody));
            }

            template <typename E, template <typename> class S>
            inline void
            DeBruijnShifter<E, S>::VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>* Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto NewBody = ExpStack.top();
                ExpStack.pop();
                auto const& QVarTypes = Exp->GetQVarTypes();
                ExpStack.push(Mgr->MakeForAll(QVarTypes, NewBody));
            }

            template <typename E, template <typename> class S>
            inline typename DeBruijnShifter<E, S>::ExpT
            DeBruijnShifter<E, S>::Do(MgrT* Mgr, const ExpT& Exp, i32 ShiftValue)
            {
                DeBruijnShifter<E, S> TheShifter(Mgr, ShiftValue);
                Exp->Accept(&TheShifter);
                return TheShifter.ExpStack.top();
            }

            // implementation of quantifier unroller
            template <typename E, template <typename> class S>
            inline QuantifierUnroller<E, S>::QuantifierUnroller(MgrT* Mgr, bool IncludeUndef)
                : ExpressionVisitorBase<E, S>("QuantifierUnroller"), Mgr(Mgr),
                  IncludeUndef(IncludeUndef)
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            inline QuantifierUnroller<E, S>::~QuantifierUnroller()
            {
                // Nothing here
            }

            template <typename E, template <typename> class S>
            inline void QuantifierUnroller<E, S>::VisitVarExpression(const VarExpression<E, S>* Exp)
            {
                ExpStack.push(Exp);
            }

            template <typename E, template <typename> class S>
            inline void
            QuantifierUnroller<E, S>::VisitBoundVarExpression(const BoundVarExpression<E, S>* Exp)
            {
                ExpStack.push(Exp);
            }

            template <typename E, template <typename> class S>
            inline void
            QuantifierUnroller<E, S>::VisitConstExpression(const ConstExpression<E, S>* Exp)
            {
                ExpStack.push(Exp);
            }

            template <typename E, template <typename> class S>
            inline void QuantifierUnroller<E, S>::VisitOpExpression(const OpExpression<E, S>* Exp)
            {
                ExpressionVisitorBase<E, S>::VisitOpExpression(Exp);
                auto const& Children = Exp->GetChildren();
                const u32 NumChildren = Children.size();

                vector<ExpT> NewChildren(NumChildren);
                for (u32 i = 0; i < NumChildren; ++i) {
                    NewChildren[NumChildren - i - 1] = ExpStack.top();
                    ExpStack.pop();
                }

                ExpStack.push(Mgr->MakeExpr(Exp->GetOpCode(), NewChildren));
            }

            template <typename E, template <typename> class S>
            inline vector<typename QuantifierUnroller<E, S>::ExpT>
            QuantifierUnroller<E, S>::UnrollQuantifier(const QuantifiedExpressionBase<E, S>* Exp)
            {
                vector<ExpT> Retval;
                auto const& QVarTypes = Exp->GetQVarTypes();
                auto const& QBody = Exp->GetQExpression();
                const u32 NumQVars = QVarTypes.size();

                vector<vector<string>> QVarElems;
                for (auto const& QVarType : QVarTypes) {
                    if (IncludeUndef) {
                        QVarElems.push_back(QVarType->GetElements());
                    } else {
                        QVarElems.push_back(QVarType->GetElementsNoUndef());
                    }
                }
                auto&& CP = CrossProduct<string>(QVarElems.begin(),
                                                 QVarElems.end());
                for (auto const& CPElem : CP) {
                    typename MgrT::SubstMapT SubstMap;
                    for (u32 i = 0; i < NumQVars; ++i) {
                        auto const& BoundVarExp =
                            Mgr->MakeBoundVar(QVarTypes[i], NumQVars - i - 1);
                        auto const& ValExp = Mgr->MakeVal(CPElem[i], QVarTypes[i]);
                        SubstMap[BoundVarExp] = ValExp;
                    }
                    auto SubstExp = Mgr->Substitute(SubstMap, QBody);
                    // We also need to replace higher number debruijn indices
                    // by lower numbered ones
                    Retval.push_back(Mgr->template ApplyTransform<DeBruijnShifter<E, S>>(SubstExp,
                                                                                         -NumQVars));
                }
                return Retval;
            }

            template <typename E, template <typename> class S>
            inline void
            QuantifierUnroller<E, S>::VisitEQuantifiedExpression(const EQuantifiedExpression<E, S>*
                                                                 Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto NewExp = ExpStack.top();
                ExpStack.pop();
                auto NewQExp = Mgr->MakeExists(Exp->GetQVarTypes(), NewExp);
                auto&& UnrolledExps =
                    UnrollQuantifier(NewQExp->template SAs<Exprs::QuantifiedExpressionBase>());
                if (UnrolledExps.size() == 1) {
                    ExpStack.push(UnrolledExps[0]);
                } else {
                    ExpStack.push(Mgr->MakeExpr(LTSOps::OpOR, UnrolledExps));
                }
            }

            template <typename E, template <typename> class S>
            inline void
            QuantifierUnroller<E, S>::VisitAQuantifiedExpression(const AQuantifiedExpression<E, S>*
                                                                 Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto NewExp = ExpStack.top();
                ExpStack.pop();
                auto NewQExp = Mgr->MakeForAll(Exp->GetQVarTypes(), NewExp);
                auto&& UnrolledExps =
                    UnrollQuantifier(NewQExp->template SAs<Exprs::QuantifiedExpressionBase>());
                if (UnrolledExps.size() == 1) {
                    ExpStack.push(UnrolledExps[0]);
                } else {
                    ExpStack.push(Mgr->MakeExpr(LTSOps::OpAND, UnrolledExps));
                }
            }

            template <typename E, template <typename> class S>
            inline typename QuantifierUnroller<E, S>::ExpT
            QuantifierUnroller<E, S>::Do(MgrT* Mgr, const ExpT& Exp, bool IncludeUndef)
            {
                QuantifierUnroller Unroller(Mgr, IncludeUndef);
                Exp->Accept(&Unroller);
                return Unroller.ExpStack.top();
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
            TypeRef BoolType;
            TypeRef IntType;

            RefCache<TypeBase, TypePtrHasher, TypePtrEquals, CSmartPtr> TypeCache;
            unordered_map<string, TypeRef> NamedTypes;

        public:
            typedef LTSOps Ops;
            typedef Exprs::Expr<E, ESMC::LTS::LTSTermSemanticizer> ExpT;
            typedef Z3Expr LExpT;
            typedef TypeRef TypeT;
            static const TypeT InvalidType;

            LTSTermSemanticizer(MgrType* Mgr);
            ~LTSTermSemanticizer();

            template <typename T, typename... ArgTypes>
            inline TypeRef MakeType(ArgTypes&&... Args);
            inline TypeRef InstantiateType(const TypeRef& ParamType,
                                           const vector<ExpT>& ParamInsts);

            inline TypeRef MakeBoolType();
            inline TypeRef MakeIntType();

            inline TypeRef GetNamedType(const string& TypeName) const;

            inline void TypeCheck(const ExpT& Exp) const;
            inline ExpT Canonicalize(const ExpT& Exp);
            // This is constant propagation
            inline ExpT Simplify(const ExpT& Exp);
            inline string ExprToString(const ExpT& Exp) const;
            inline string TypeToString(i64 Type) const;
            inline i64 RegisterUninterpretedFunction(const string& Name,
                                                     const vector<TypeT>& DomTypes,
                                                     const TypeT& RangeType);
            inline TypeT LookupUninterpretedFunction(i64 OpCode) const;

            inline ExpT RaiseExpr(MgrType* Mgr, const LExpT& LExp, const LTSLCRef& LTSCtx);
            inline LExpT LowerExpr(const ExpT& Exp, const LTSLCRef& ExpCtx);
            inline ExpT ElimQuantifiers(MgrType* Mgr, const ExpT& Exp);
            inline ExpT UnrollQuantifiers(MgrType* Mgr, const ExpT& Exp, bool IncludeUndef);
        };


        // Implementation of LTSTermSemanticizer

        template <typename E>
        LTSTermSemanticizer<E>::LTSTermSemanticizer(MgrType* Mgr)
            : UFUIDGen(LTSOps::UFOffset), Mgr(Mgr)
        {
            // Nothing here
            BoolType = TypeCache.template Get<BooleanType>();
            IntType = TypeCache.template Get<IntegerType>();
        }

        template <typename E>
        LTSTermSemanticizer<E>::~LTSTermSemanticizer()
        {
            // Nothing here
        }

        template <typename E>
        template <typename T, typename... ArgTypes>
        inline TypeRef
        LTSTermSemanticizer<E>::MakeType(ArgTypes&&... Args)
        {
            auto Retval = TypeCache.Get<T>(forward<ArgTypes>(Args)...);
            Retval->GetOrSetTypeID();
            string TypeName = "";
            if (Retval->template Is<EnumType>()) {
                TypeName = Retval->template SAs<EnumType>()->GetName();
            } else if (Retval->template Is<SymmetricType>()) {
                TypeName = Retval->template SAs<SymmetricType>()->GetName();
            } else if (Retval->template Is<RecordType>()) {
                TypeName = Retval->template SAs<RecordType>()->GetName();
            }

            if (TypeName != "") {
                auto it = NamedTypes.find(TypeName);
                if (it != NamedTypes.end() && it->second != Retval) {
                    throw ExprTypeError((string)"A type named \"" + TypeName + "\" has already " +
                                        "been created");
                } else if (it != NamedTypes.end()) {
                    return Retval;
                }
                NamedTypes[TypeName] = Retval;
            }
            return Retval;
        }

        template <typename E>
        inline TypeRef
        LTSTermSemanticizer<E>::InstantiateType(const TypeRef& ParamType,
                                                const vector<ExpT>& ParamInsts)
        {
            auto TypeAsPType = ParamType->template As<ParametricType>();
            if (TypeAsPType == nullptr) {
                throw ExprTypeError((string)"Cannot instantiate non-parametric type " +
                                    ParamType->ToString());
            }
            auto const& ExpectedTypes = TypeAsPType->GetParameterTypes();
            const u32 NumExpected = ExpectedTypes.size();
            const u32 NumGot = ParamInsts.size();

            if (NumExpected != NumGot) {
                throw ExprTypeError((string)"Parametric type \"" + TypeAsPType->GetName() +
                                    "\" expects " + to_string(NumExpected) + " parameters, " +
                                    "but was attempted to be instantiated with " +
                                    to_string(NumGot) + " parameters");
            }

            for (u32 i = 0; i < NumExpected; ++i) {
                if (!ParamInsts[i]->template Is<ConstExpression>()) {
                    throw ExprTypeError((string)"Parameter at position " + to_string(i) +
                                        " is not a constant value in instantiation of type " +
                                        TypeAsPType->GetName());
                }
                if (ParamInsts[i]->GetType() != ExpectedTypes[i]) {
                    throw ExprTypeError((string)"Parameter types don't match at position " +
                                        to_string(i) + " in instantiation of type " +
                                        TypeAsPType->GetName());
                }
            }

            string InstName = TypeAsPType->GetName();
            for (u32 i = 0; i < NumExpected; ++i) {
                InstName += ((string)"[" +
                             ParamInsts[i]->template SAs<ConstExpression>()->GetConstValue() +
                             "]");
            }
            auto BaseRecType = TypeAsPType->GetBaseType()->template SAs<RecordType>();
            auto Retval = TypeCache.template Get<RecordType>(InstName,
                                                             BaseRecType->GetMemberVec());
            Retval->GetOrSetTypeID();
            return Retval;
        }

        template <typename E>
        inline TypeRef
        LTSTermSemanticizer<E>::MakeBoolType()
        {
            return TypeCache.template Get<BooleanType>();
        }

        template <typename E>
        inline TypeRef
        LTSTermSemanticizer<E>::MakeIntType()
        {
            return TypeCache.template Get<IntegerType>();
        }

        template <typename E>
        inline TypeRef
        LTSTermSemanticizer<E>::GetNamedType(const string& TypeName) const
        {
            auto it = NamedTypes.find(TypeName);
            if (it == NamedTypes.end()) {
                return TypeRef::NullPtr;
            } else {
                return it->second;
            }
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
            // audupa: 12/03/14, we don't need to canonicalize the
            // entire expression, just the children, because the
            // children themselves are guaranteed to be canonicalized
            // already

            // return Detail::Canonicalizer<E, LTS::LTSTermSemanticizer>::Do(Exp);

            auto ExpAsOp = Exp->template As<OpExpression>();
            if (ExpAsOp == nullptr) {
                return Exp;
            }
            auto OpCode = ExpAsOp->GetOpCode();
            auto& Children = const_cast<vector<ExpT>&>(ExpAsOp->GetChildren());
            switch (OpCode) {
            case LTSOps::OpEQ:
            case LTSOps::OpOR:
            case LTSOps::OpAND:
            case LTSOps::OpIFF:
            case LTSOps::OpXOR:
            case LTSOps::OpADD:
            case LTSOps::OpMUL:
                {
                    sort(Children.begin(), Children.end(), ExpressionPtrCompare());
                    if (OpCode == LTSOps::OpAND || OpCode == LTSOps::OpOR) {
                        auto it = unique(Children.begin(), Children.end());
                        Children.resize(distance(Children.begin(), it));
                        if (Children.size() == 1) {
                            return Children[0];
                        }
                    }
                    return Exp;
                }
                break;

            case LTSOps::OpNOT:
                {
                    auto ChildAsOp = Children[0]->template As<OpExpression>();
                    if (ChildAsOp != nullptr && ChildAsOp->GetOpCode() == LTSOps::OpNOT) {
                        return ((ChildAsOp->GetChildren())[0]);
                    } else {
                        return Exp;
                    }
                }
                break;

            default:
                return Exp;
            }
        }

        template <typename E>
        inline typename LTSTermSemanticizer<E>::ExpT
        LTSTermSemanticizer<E>::Simplify(const ExpT& Exp)
        {
            return Detail::Simplifier<E, LTS::LTSTermSemanticizer>::Do(Mgr, Exp, BoolType, IntType);
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
                        {
                            return (Type->template Is<ScalarType>());
                        })) {
                throw ESMCError((string)"Only functions from scalars -> scalars are " +
                                "currently supported");
            }

            if (UFNameToIDMap.find(MangledName) == UFNameToIDMap.end()) {
                auto Type = MakeType<FuncType>(Name, DomTypes, RangeType);
                auto UFID = UFUIDGen.GetUID();
                UFNameToIDMap[MangledName] = UFID;
                UFMap[UFID] = Type;
                return UFID;
            } else {
                auto it = UFNameToIDMap.find(MangledName);
                auto UFID = it->second;
                auto it2 = UFMap.find(UFID);
                assert(it2 != UFMap.end());

                auto Type = it2->second->template As<FuncType>();

                assert (Type != nullptr);
                if (Type->GetEvalType() != RangeType) {
                    throw Exprs::ExprTypeError((string)"Redeclaration of function " +
                                               Name + " with variant return type");
                }
                return UFID;
            }
        }

        template <typename E>
        inline typename LTSTermSemanticizer<E>::TypeT
        LTSTermSemanticizer<E>::LookupUninterpretedFunction(i64 OpCode) const
        {
            auto it = UFMap.find(OpCode);
            if (it == UFMap.end()) {
                throw ExprTypeError((string)"No uninterpreted function corresponds to " +
                                    "opcode = " + to_string(OpCode));
            }
            return it->second;
        }

        template <typename E>
        inline typename LTSTermSemanticizer<E>::ExpT
        LTSTermSemanticizer<E>::RaiseExpr(MgrType* Mgr, const LExpT& LExp,
                                          const LTSLCRef& LTSCtx)
        {
            return Raiser<E, ESMC::LTS::LTSTermSemanticizer>::Do(LExp, UFNameToIDMap,
                                                                 LTSCtx, Mgr);
        }

        template <typename E>
        inline typename LTSTermSemanticizer<E>::LExpT
        LTSTermSemanticizer<E>::LowerExpr(const ExpT& Exp, const LTSLCRef& LTSCtx)
        {
            return Lowerer<E, ESMC::LTS::LTSTermSemanticizer>::Do(Exp, UFMap, LTSCtx);
        }

        template <typename E>
        inline typename LTSTermSemanticizer<E>::ExpT
        LTSTermSemanticizer<E>::ElimQuantifiers(MgrType* Mgr, const ExpT& Exp)
        {
            LTSLCRef LTSCtx = new LTSLoweredContext();

            auto LExp = LowerExpr(Exp, LTSCtx);
            auto Ctx = LTSCtx->GetZ3Ctx();

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
                    Z3Expr CurExpr(Ctx, CurFormula);
                    RaisedExprs.push_back(RaiseExpr(CurExpr, LTSCtx));
                }
            }

            Z3_apply_result_dec_ref(*Ctx, Res);
            if (RaisedExprs.size() == 1) {
                return RaisedExprs[0];
            } else {
                return Mgr->MakeExpr(LTSOps::OpAND, RaisedExprs);
            }
        }

        template <typename E>
        inline typename LTSTermSemanticizer<E>::ExpT
        LTSTermSemanticizer<E>::UnrollQuantifiers(MgrType* Mgr, const ExpT& Exp,
                                                  bool IncludeUndef)
        {
            return Detail::QuantifierUnroller<E, ESMC::LTS::LTSTermSemanticizer>::Do(Mgr, Exp,
                                                                                     IncludeUndef);
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

        extern ostream& operator << (const Z3Expr& Expr, ostream& Out);
        extern ostream& operator << (const Z3Sort& Sort, ostream& Out);


    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_TERM_SEMANTICIZER_HPP_ */

//
// LTSTermSemanticizer.hpp ends here
