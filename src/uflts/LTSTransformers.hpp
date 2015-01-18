// LTSTransformers.hpp ---
// Filename: LTSTransformers.hpp
// Author: Abhishek Udupa
// Created: Mon Jan  5 00:09:26 2015 (-0500)
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

#if !defined ESMC_LTS_TRANSFORMERS_HPP_
#define ESMC_LTS_TRANSFORMERS_HPP_

#include <stack>

#include "../common/ESMCFwdDecls.hpp"

#include "LTSDecls.hpp"

namespace ESMC {
    namespace LTS {
        namespace Detail {

            struct VarGatherer
            {
                typedef const Exprs::ExpressionBase<LTSExtensionT, LTSTermSemanticizer>* ExpCPtrT;

                inline bool operator () (ExpCPtrT Exp) const
                {
                    auto ExpAsVar = Exp->template As<Exprs::VarExpression>();
                    if (ExpAsVar == nullptr) {
                        return false;
                    }
                    // We don't want to return field access vars
                    auto Type = ExpAsVar->GetVarType();
                    auto TypeAsFA = Type->As<FieldAccessType>();
                    return (TypeAsFA == nullptr);
                }
            };

            struct SymmConstGatherer
            {
                typedef const Exprs::ExpressionBase<LTSExtensionT, LTSTermSemanticizer>* ExpCPtrT;

                inline bool operator () (ExpCPtrT Exp) const
                {
                    auto ExpAsConst = Exp->template As<Exprs::ConstExpression>();
                    if (ExpAsConst == nullptr) {
                        return false;
                    }
                    auto Type = ExpAsConst->GetConstType();
                    return (Type->Is<SymmetricType>());
                };
            };

            // Transforms all references to a particular
            // message type in an expression into a
            // unified message type reference, renaming
            // all field accesses appropriately
            class MsgTransformer : public VisitorBaseT
            {
            private:
                stack<ExpT> ExpStack;
                MgrT* Mgr;
                string MsgVarName;
                TypeRef MsgRecType;
                TypeRef UnifiedMType;

            public:
                MsgTransformer(MgrT* Mgr, const string& MsgVarName,
                               const TypeRef& MsgRecType,
                               const TypeRef& UnifiedMType);
                virtual ~MsgTransformer();

                virtual void VisitVarExpression(const VarExpT* Exp) override;
                virtual void VisitBoundVarExpression(const BoundVarExpT* Exp) override;
                virtual void VisitConstExpression(const ConstExpT* Exp) override;
                virtual void VisitOpExpression(const OpExpT* Exp) override;
                virtual void VisitEQuantifiedExpression(const EQExpT* Exp) override;
                virtual void VisitAQuantifiedExpression(const AQExpT* Exp) override;

                static ExpT Do(MgrT* Mgr,
                               const ExpT& Exp,
                               const string& MsgVarName,
                               const TypeRef& MsgRecType,
                               const TypeRef& UnifiedMType);
            };

            class ExpressionPermuter : public VisitorBaseT
            {
            private:
                MgrT* Mgr;
                const vector<u08>& PermVec;
                map<TypeRef, u32> TypeOffsets;
                stack<ExpT> ExpStack;

            public:
                ExpressionPermuter(MgrT* Mgr, const vector<u08>& PermVec,
                                   const map<TypeRef, u32>& TypeOffsets);
                virtual ~ExpressionPermuter();

                virtual void VisitVarExpression(const VarExpT* Exp) override;
                virtual void VisitBoundVarExpression(const BoundVarExpT* Exp) override;
                virtual void VisitConstExpression(const ConstExpT* Exp) override;
                virtual void VisitOpExpression(const OpExpT* Exp) override;
                virtual void VisitEQuantifiedExpression(const EQExpT* Exp) override;
                virtual void VisitAQuantifiedExpression(const AQExpT* Exp) override;

                static ExpT Do(MgrT* Mgr, const ExpT& Exp,
                               const vector<u08>& PermVec,
                               const map<TypeRef, u32>& TypeOffsets);

            };

            class ArrayRValueTransformer : public VisitorBaseT
            {
            private:
                MgrT* Mgr;
                stack<ExpT> ExpStack;

                inline void VisitQuantifiedExpression(const QExpT* Exp);

            public:
                ArrayRValueTransformer(MgrT* Mgr);
                virtual ~ArrayRValueTransformer();

                virtual void VisitVarExpression(const VarExpT* Exp) override;
                virtual void VisitBoundVarExpression(const BoundVarExpT* Exp) override;
                virtual void VisitConstExpression(const ConstExpT* Exp) override;
                virtual void VisitOpExpression(const OpExpT* Exp) override;
                virtual void VisitEQuantifiedExpression(const EQExpT* Exp) override;
                virtual void VisitAQuantifiedExpression(const AQExpT* Exp) override;

                static ExpT Do(MgrT* Mgr, const ExpT& Exp);
            };

            class UFIndexExpGatherer : public VisitorBaseT
            {
            private:
                set<pair<ExpT, TypeRef>>& UFIndexExps;

            public:
                UFIndexExpGatherer(set<pair<ExpT, TypeRef>>& UFIndexExps);
                virtual ~UFIndexExpGatherer();

                virtual void VisitOpExpression(const OpExpT* Exp) override;
                virtual void VisitEQuantifiedExpression(const EQExpT* Exp) override;
                virtual void VisitAQuantifiedExpression(const AQExpT* Exp) override;
                static void Do(const ExpT& Exp, set<pair<ExpT, TypeRef>>& UFIndexExps);
            };

            class ConstraintPurifier : public VisitorBaseT
            {
            private:
                MgrT* Mgr;
                stack<ExpT> ExpStack;
                FastExpSetT& Assumptions;

            public:
                ConstraintPurifier(MgrT* Mgr, FastExpSetT& Assumptions);
                virtual ~ConstraintPurifier();

                inline vector<pair<ExpT, ExpT>>
                    MakeITEBranches(const ExpT& Exp,
                                    const set<pair<ExpT, TypeRef>>& UFIndexExps);

                virtual void VisitVarExpression(const VarExpT* Exp) override;
                virtual void VisitBoundVarExpression(const BoundVarExpT* Exp) override;
                virtual void VisitConstExpression(const ConstExpT* Exp) override;
                virtual void VisitOpExpression(const OpExpT* Exp) override;
                virtual void VisitEQuantifiedExpression(const EQExpT* Exp) override;
                virtual void VisitAQuantifiedExpression(const AQExpT* Exp) override;

                static ExpT Do(MgrT* Mgr, const ExpT& Exp, FastExpSetT& Assumptions);
            };

        } /* end namespace Detail */

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_TRANSFORMERS_HPP_ */

//
// LTSTransformers.hpp ends here
