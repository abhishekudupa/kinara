// FwdDecls.hpp --- 
// 
// Filename: FwdDecls.hpp
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

#if !defined ESMC_FWD_DECLS_HPP_
#define ESMC_FWD_DECLS_HPP_

#include "Types.hpp"
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

        class ExprTypeBase;
        class ExprFuncType;
        
        typedef CSmartPtr<ExprTypeBase> ExprTypeRef;
    } /* end namespace Exprs */
    
    namespace LTS {

        class LabelledTS;
        class EFSMBase;
        class AutomatonBase;
        class ChannelEFSM;
        class GeneralEFSM;
        class DetEFSM;

        template <typename E> class LTSTermSemantizer;

        class LTSAssignBase;
        class LTSAssignSimple;
        class LTSAssignParam;

        typedef CSmartPtr<LTSAssignBase> LTSAssignRef;

        class LTSGuardedCommand;        
        typedef CSmartPtr<LTSGuardedCommand> GCmdRef;

    } /* end namespace LTS */

    namespace MC {
        class StateVec;
        class StateFactory;
        class LTSChecker;
        class Compiler;
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

    // UID Generators
    class UIDGenerator;

} /* end namespace */

#endif /* ESMC_FWD_DECLS_HPP_ */

// 
// FwdDecls.hpp ends here








