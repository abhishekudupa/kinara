// LTSTypes.hpp --- 
// 
// Filename: LTSTypes.hpp
// Author: Abhishek Udupa
// Created: Fri Aug  8 13:34:49 2014 (-0400)
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

#if !defined ESMC_LTS_TYPES_HPP_
#define ESMC_LTS_TYPES_HPP_

#include "../common/FwdDecls.hpp"
#include "../expr/Expressions.hpp"

#include "LTSTermSemanticizer.hpp"

namespace ESMC {
    namespace LTS {

        // The extension type for LTS expressions
        struct LTSExtensionT
        {

        };

        // Enum classes for fairness types
        enum class LTSFairnessType {
            None, Weak, Strong
        };

        
        enum class SplatFairnessType {
            None, Group, Individual
        };

        enum class FairSetFairnessType {
            Weak, Strong
        };

        enum class LossDupFairnessType {
            None, NotAlwaysLost, NotAlwaysDup, NotAlwaysLostOrDup
        };

        class LabelledTS;

        class AutomatonBase;
        class EFSMBase;
        class GeneralEFSM;
        class DetEFSM;
        class ChannelEFSM;

        class MonitorBase;
        class SafetyMonitor;
        class BuchiMonitor;

        class LTSState;

        class LTSAssignBase;
        class LTSAssignSimple;
        class LTSAssignParam;
        
        typedef CSmartPtr<LTSAssignBase> LTSAssignRef;

        class AutomatonTransitionBase;
        class LTSTransitionBase;
        class LTSTransitionInput;
        class LTSTransitionOutput;
        class LTSTransitionInternal;
        class BuchiMonitorTransition;
        class LTSInitState;

        typedef CSmartPtr<LTSTransitionBase> LTSTransRef;
        typedef CSmartPtr<BuchiMonitorTransition> BuchiTransRef;
        typedef CSmartPtr<LTSInitState> InitStateRef;

        class LTSFairnessSet;

        typedef CSmartPtr<LTSFairnessSet> LTSFairSetRef;

        class LTSGuardedCommand;
        typedef CSmartPtr<LTSGuardedCommand> GCmdRef;

        typedef Exprs::ExprTypeRef ExprTypeRef;
        typedef Exprs::Expr<LTSExtensionT, LTSTermSemanticizer> ExpT;
        typedef Exprs::ExprMgr<LTSExtensionT, LTSTermSemanticizer> MgrT;
        typedef Exprs::ExpressionVisitorBase<LTSExtensionT, LTSTermSemanticizer> VisitorBaseT;

        typedef Exprs::VarExpression<LTSExtensionT, LTSTermSemanticizer> VarExpT;
        typedef Exprs::ConstExpression<LTSExtensionT, LTSTermSemanticizer> ConstExpT;
        typedef Exprs::BoundVarExpression<LTSExtensionT, LTSTermSemanticizer> BoundVarExpT;
        typedef Exprs::OpExpression<LTSExtensionT, LTSTermSemanticizer> OpExpT;
        typedef Exprs::QuantifiedExpressionBase<LTSExtensionT, LTSTermSemanticizer> QExpT;
        typedef Exprs::EQuantifiedExpression<LTSExtensionT, LTSTermSemanticizer> EQExpT;
        typedef Exprs::AQuantifiedExpression<LTSExtensionT, LTSTermSemanticizer> AQExpT;

        // types
        typedef Exprs::ExprBoolType ExprBoolType;
        typedef Exprs::ExprIntType ExprIntType;
        typedef Exprs::ExprScalarType ExprScalarType;
        typedef Exprs::ExprSymmetricType ExprSymmetricType;
        typedef Exprs::ExprRangeType ExprRangeType;
        typedef Exprs::ExprEnumType ExprEnumType;
        typedef Exprs::ExprRecordType ExprRecordType;
        typedef Exprs::ExprArrayType ExprArrayType;
        typedef Exprs::ExprParametricType ExprParametricType;
        typedef Exprs::ExprUnionType ExprUnionType;
        typedef Exprs::ExprFieldAccessType ExprFieldAccessType;

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_TYPES_HPP_ */

// 
// LTSTypes.hpp ends here
