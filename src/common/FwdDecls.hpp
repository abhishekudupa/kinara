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

namespace ESMC {

    // SmartPtrs and such
    class RefCountable;
    template <typename T> class SmartPtr;
    template <typename T> class CSmartPtr;

    // Expression managers
    // and semanticizers
    namespace Exprs {
        template <typename LEType> class SemanticizerBase;
        class Z3Semanticizer;
        template <typename ExtType, typename SemType=Z3Semanticizer> class ExprManager;

        template <typename ExtType> class ExprMgr;
        template <typename ExtType> class ExpressionBase;
        template <typename ExtType> class ConstExpression;
        template <typename ExtType> class VarExpression;
        template <typename ExtType> class BoundVarExpression;
        template <typename ExtType> class OpExpression;
        template <typename ExtType> class QuantifiedExpressionBase;
        template <typename ExtType> class AQuantifiedExpression;
        template <typename ExtType> class EQuantifiedExpression;

        class ExpressionPtrHasher;
        class ExpressionPtrEquals;
        class ExpressionPtrCompare;
    
        template <typename ExtType> using Expression = CSmartPtr<ExpressionBase<ExtType>>;
        template <typename ExtType> using ExprInternal = SmartPtr<ExpressionBase<ExtType>>;

        // Expression Visitors
        class ExpressionVisitorBase;
    } /* end namespace Exprs */
    

    // UID Generators
    class UIDGenerator;

} /* end namespace */

#endif /* ESMC_FWD_DECLS_HPP_ */

// 
// FwdDecls.hpp ends here
