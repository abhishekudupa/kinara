// ParamUtils.hpp --- 
// 
// Filename: ParamUtils.hpp
// Author: Abhishek Udupa
// Created: Fri Aug  1 15:12:01 2014 (-0400)
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

#if !defined ESMC_PARAM_UTILS_HPP_
#define ESMC_PARAM_UTILS_HPP_

#include "../common/FwdDecls.hpp"
#include "../expr/Expressions.hpp"
#include "../utils/CombUtils.hpp"

#include "LTSTermSemanticizer.hpp"
#include "UFLTSExtension.hpp"

namespace ESMC {
    namespace LTS {

        typedef Exprs::Expr<UFLTSExtensionT, LTSTermSemanticizer> ExpT;
        typedef Exprs::ExprMgr<UFLTSExtensionT, LTSTermSemanticizer> MgrType;
        typedef Exprs::ExprTypeRef ExprTypeRef;

        static inline vector<vector<ExpT>> InstantiateParams(const vector<ExpT>& Params,
                                                             const ExpT& Constraint,
                                                             MgrType* Mgr)
        {
            vector<vector<ExpT>> Retval;
            vector<vector<string>> CPElems;
            for (auto const& Param : Params) {
                CPElems.push_back(Param->GetType()->GetElements());
            }

            auto&& CPRes = CrossProduct<string>(CPElems.begin(), CPElems.end());
            vector<MgrType::SubstMapT> SubstMaps;

            for (auto const& Prod : CPRes) {
                MgrType::SubstMapT SubstMap;
                for (u32 i = 0; i < Params.size(); ++i) {
                    auto Type = Params[i]->GetType();
                    SubstMap[Params[i]] = Mgr->MakeVal(Prod[i], Type);
                }
                auto SubstConst = Mgr->Substitute(SubstMap, Constraint);
                auto SimpConst = Mgr->Simplify(SubstConst);
                auto SimpAsConst = SimpConst->As<Exprs::ConstExpression>();
                if (SimpAsConst == nullptr) {
                    throw ESMCError((string)"Did not get a constant on the constraint " + 
                                    "after substituting parameters! Perhaps the constraint " +
                                    "cannot be simplified?\nConstraint:\n" + 
                                    Constraint->ToString());
                }
                if (SimpAsConst->GetType()->As<Exprs::ExprBoolType>() == nullptr) {
                    throw InternalError((string)"Expected a boolean constant.\nAt: " + 
                                        __FILE__ + ":" + to_string(__LINE__));
                }
                if (SimpAsConst->GetConstValue() == "true") {
                    vector<ExpT> CurVec;
                    for (auto const& Param : Params) {
                        CurVec.push_back(SubstMap[Param]);
                    }
                    Retval.push_back(CurVec);
                }
            }
            return Retval;
        }

        static inline vector<vector<ExpT>> InstantiatePendingParams(const vector<ExpT>& Params,
                                                                    MgrType* Mgr,
                                                                    const MgrType::SubstMapT& 
                                                                    GlobalSM,
                                                                    const ExpT& Constraint)
        {
            const u32 NumParams = Params.size();

            vector<ExpT> SubstParams = Params;
            vector<vector<ExpT>> Retval;
            vector<ExpT> ParamsToInstantiate;

            for (u32 i = 0; i < NumParams; ++i) {
                SubstParams[i] = Mgr->Substitute(GlobalSM, SubstParams[i]);
                if (!SubstParams[i]->Is<Exprs::ConstExpression>()) {
                    ParamsToInstantiate.push_back(SubstParams[i]);
                }
            }
            
            auto SubstConstraint = Mgr->Substitute(GlobalSM, Constraint);
            auto&& ParamVec = InstantiateParams(ParamsToInstantiate, SubstConstraint, Mgr);
            for (auto const& ParamVal : ParamVec) {
                u32 j = 0;
                for (u32 i = 0; i < NumParams; ++i) {
                    if (!Params[i]->Is<Exprs::ConstExpression>()) {
                        SubstParams[i] = ParamVal[j++];
                    }
                }
                Retval.push_back(SubstParams);
            }
            return Retval;
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_PARAM_UTILS_HPP_ */

// 
// ParamUtils.hpp ends here
