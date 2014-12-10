// ESMCESMCFwdDecls.hpp ---
//
// Filename: ESMCESMCFwdDecls.hpp
// Author: Abhishek Udupa
// Created: Sun Jun 29 13:46:14 2014 (-0400)
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

// Forward declarations of classes and types

#if !defined ESMC_ESMC_FWD_DECLS_HPP_
#define ESMC_ESMC_FWD_DECLS_HPP_

#include "ESMCTypes.hpp"
#include <list>

namespace ESMC {

    // SmartPtrs and such
    class RefCountable;
    template <typename T> class SmartPtr;
    template <typename T> class CSmartPtr;

    // Expression managers
    // and semanticizers
    namespace Exprs {

        template <typename ExtType, template <typename> class SemType> class ExprMgr;
        template <typename ExtType, template <typename> class SemType> class ExpressionBase;
        template <typename ExtType, template <typename> class SemType> class ConstExpression;
        template <typename ExtType, template <typename> class SemType> class VarExpression;
        template <typename ExtType, template <typename> class SemType> class BoundVarExpression;
        template <typename ExtType, template <typename> class SemType> class OpExpression;
        template <typename ExtType,
                  template <typename> class SemType> class QuantifiedExpressionBase;
        template <typename ExtType, template <typename> class SemType> class AQuantifiedExpression;
        template <typename ExtType, template <typename> class SemType> class EQuantifiedExpression;

        class ExtListExtBase;
        typedef SmartPtr<ExtListExtBase> ExtListExtRef;
        typedef CSmartPtr<ExtListExtBase> ExtListExtCRef;
        typedef list<ExtListExtRef> ExtListT;

        // Smart ptr types for expressions
        template <typename E, template <typename> class S>
        using Expr = CSmartPtr<ExpressionBase<E, S>>;
        template <typename E, template <typename> class S>
        using ExprI = SmartPtr<ExpressionBase<E, S>>;

        class ExpressionPtrHasher;
        class FastExpressionPtrEquals;
        class ExpressionPtrEquals;
        class ExpressionPtrCompare;

        template <template <typename> class S>
        using ExtListExprMgr = ExprMgr<ExtListT, S>;

        // Expression Visitors
        template <typename E, template <typename> class S> class ExpressionVisitorBase;

    } /* end namespace Exprs */

    namespace Decls {

        class SymbolTable;
        class SymtabScope;

        typedef SmartPtr<SymtabScope> ScopeRef;

    } /* end namespace Decls */

    namespace LTS {

        class TypeBase;
        class FuncType;
        class ScalarType;
        class BoolType;
        class IntType;
        class RangeType;
        class EnumType;
        class SymmetricType;
        class RecordType;
        class ArrayType;
        class UnionType;
        class ParametricType;
        class FieldAccessType;

        typedef CSmartPtr<TypeBase> TypeRef;


        class LabelledTS;
        class EFSMBase;
        class AutomatonBase;
        class ChannelEFSM;
        class GeneralEFSM;
        class DetEFSM;

        class SymmetricMessageDecl;

        typedef CSmartPtr<SymmetricMessageDecl> SymmMsgDeclRef;

        template <typename E> class LTSTermSemantizer;
        class LTSLoweredContext;
        typedef CSmartPtr<LTSLoweredContext> LTSLCRef;

        class LTSAssignBase;
        class LTSAssignSimple;
        class LTSAssignParam;

        typedef CSmartPtr<LTSAssignBase> LTSAssignRef;

        class AutomatonTransitionBase;
        class LTSTransitionBase;
        class LTSTransitionInput;
        class LTSTransitionOutput;
        class LTSTransitionInternal;
        class LTSInitState;

        typedef CSmartPtr<LTSTransitionBase> LTSTransRef;
        typedef CSmartPtr<LTSInitState> InitStateRef;

        class LTSSymbTransitionBase;
        class LTSSymbIOTransitionBase;
        class LTSSymbInputTransition;
        class LTSSymbOutputTransition;
        class LTSSymbInternalTransition;

        typedef CSmartPtr<LTSSymbTransitionBase> LTSSymbTransRef;

        class LTSGuardedCommand;
        typedef CSmartPtr<LTSGuardedCommand> GCmdRef;

    } /* end namespace LTS */

    namespace MC {

        enum class AQSConstructionMethod {
            BreadthFirst, DepthFirst
        };

        class StateVec;
        class StateFactory;
        class LTSChecker;
        class LTSCompiler;
        class StateVecPrinter;

        class RValueInterpreter;
        class LValueInterpreter;
        class UFInterpreter;
        class AQStructure;
        template <typename STATETYPE>
        class AnnotatedEdge;

        typedef AnnotatedEdge<StateVec> AQSEdge;

        class ProductStructure;
        class ProductState;
        class StateVecPrinter;

        typedef AnnotatedEdge<ProductState> ProductEdge;

        class BuchiAutomatonBase;
        class StateBuchiAutomaton;
        class MsgBuchiAutomaton;

        class IndexVector;
        class ProcessIndexSet;
        class SystemIndexSet;

        // Traces
        typedef pair<LTS::GCmdRef, const StateVec*> TraceElemT;
        typedef pair<LTS::GCmdRef, const ProductState*> PSTraceElemT;

        template <typename STATETYPE> class PermutedPath;
        // A permuted path through the annotated quotient structure
        typedef PermutedPath<StateVec> AQSPermPath;
        // A permuted path through the (annotated) product structure
        typedef PermutedPath<ProductState> PSPermPath;

        class TraceBase;
        class SafetyViolation;
        class DeadlockViolation;
        class LivenessViolation;
        class MCExceptionTrace;

    } /* end namespace MC */

    namespace Symm {

        class PermutationSet;
        class Canonicalizer;
        class PermuterBase;
        class ArrayPermuter;
        class RecordPermuter;
        class SymmTypePermuter;
        class NoOpPermuter;

    } /* end namespace Symm */

    namespace TP {

        class Z3Object;
        class Z3CtxWrapper;
        class Z3Expr;
        class Z3Sort;
        class Z3Model;
        class Z3Solver;

        typedef CSmartPtr<Z3CtxWrapper> Z3Ctx;

        class TheoremProver;
        class Z3TheoremProver;

        typedef CSmartPtr<TheoremProver> TPRef;

    } /* end namespace TP */

    namespace Synth {
        class Solver;
    } /* end namespace Synth */

    namespace Analyses {
        class TraceAnalyses;
    }

    // UID Generators
    class UIDGenerator;

    template <typename... ArgTypes>
    constexpr bool TruePred(ArgTypes&&... Args) { return true; }

    template <typename... ArgTypes>
    constexpr bool FalsePred(ArgTypes&&... Args) { return false; }

} /* end namespace */

#endif /* ESMC_ESMC_FWD_DECLS_HPP_ */

//
// ESMCESMCFwdDecls.hpp ends here
