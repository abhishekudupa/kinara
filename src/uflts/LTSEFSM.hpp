// LTSEFSM.hpp --- 
// 
// Filename: LTSEFSM.hpp
// Author: Abhishek Udupa
// Created: Fri Aug  8 13:43:28 2014 (-0400)
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

#if !defined ESMC_LTS_EFSM_HPP_
#define ESMC_LTS_EFSM_HPP_

#include "../utils/UIDGenerator.hpp"

#include "LTSTypes.hpp"
#include "LTSState.hpp"
#include "SymbolTable.hpp"
#include "LTSEFSMBase.hpp"

namespace ESMC {
    namespace LTS {
        
        class GeneralEFSM : public EFSMBase
        {
        public:
            GeneralEFSM(LabelledTS* TheLTS, const string& Name,
                        const vector<ExpT>& Params, const ExpT& Constraint,
                        LTSFairnessType Fairness = LTSFairnessType::None);
            virtual ~GeneralEFSM();
            // Nothing needs to be overridden here
        };

        class DetEFSM : public EFSMBase
        {
        private:
            inline void 
            CheckTransition(const TPRef& TP, u32 TransIndex,
                            const vector<LTSSymbTransRef>& CandTrans) const;
            
        public:
            DetEFSM(LabelledTS* TheLTS, const string& Name,
                    const vector<ExpT>& Params, const ExpT& Constraint,
                    LTSFairnessType Fairness = LTSFairnessType::None);

            virtual ~DetEFSM();
            
            // Override freeze to check for determinism
            virtual void Freeze() override;
        };
        
        class IncompleteEFSM : public DetEFSM
        {
            friend class ESMC::Synth::Solver;

        private:
            // Constraints over uninterpreted functions
            set<ExpT> Constraints;

            // A temporary to store current constraints
            set<ExpT> CurrentConstraints;
            // Constraints by symbolic transitions
            map<LTSSymbTransRef, set<ExpT>> ConstraintsByTransition;

            set<i64> GuardUFIDs;
            map<string, set<SymmMsgDeclRef>> BlockedCompletions;
            set<string> CompleteStates;
            set<string> ReadOnlyVars;
            map<string, ExprTypeRef> UpdateableVariables;
            map<string, ExprTypeRef> AllVariables;
            UIDGenerator GuardUFUIDGen;
            UIDGenerator UpdateUFUIDGen;

            // Constraint addition methods. mainly for logging
            inline void AddConstraint(const ExpT& Constraint);
            inline void AddConstraint(const vector<ExpT>& Constraints);

            inline void FilterTerms(set<ExpT>& DomainTerms, const ExprTypeRef& RangeType);
            set<ExpT> GetDomainTerms(const map<string, ExprTypeRef>& DomainVars);

            inline vector<ExprTypeRef> GetSymmTypesInExpr(const ExpT& Exp);
            inline void PartitionDomain(const vector<ExpT>& Args,
                                        vector<ExpT>& SymmArgs,
                                        vector<ExpT>& NonSymmArgs);

            inline void MergeEquivalences(const set<ExpT>& NewEquivalences,
                                          set<set<ExpT>>& EquivalenceSets);

            inline set<set<ExpT>> FindEquivalences(const ExpT& Exp,
                                                   const vector<ExprTypeRef>& SymmTypes,
                                                   const vector<ExpT>& SymmArgs,
                                                   const vector<ExpT>& NonSymmArgs);

            inline vector<ExpT> GetSymmetryConstraints(const ExpT& Exp);

            inline ExpT FindDisjunction(const vector<LTSSymbTransRef>& Transitions,
                                        const TPRef& TP,
                                        const ExpT& CoveredRegion);

            inline ExpT FindInputCoveredRegion(const vector<LTSSymbTransRef>& Transitions,
                                               const TPRef& TP, 
                                               const ExprTypeRef& MsgType,
                                               const ExpT& CoveredRegion);

            inline ExpT FindGlobalCoveredRegion(const vector<LTSSymbTransRef>& Transitions,
                                                const TPRef& TP);
            
            inline void CompleteInputTransitions(const string& StateName,
                                                 const vector<LTSSymbTransRef>& Transitions,
                                                 const ExpT& CoveredPredicate,
                                                 const TPRef& TP);
            
            inline ExpT MakeGuard(const set<ExpT>& DomainTerms,
                                  const ExpT& CoveredPredicate,
                                  const vector<ExpT>& GuardExps);
            
            inline vector<LTSAssignRef> MakeUpdates(const set<ExpT>& DomainTerms);

            inline void CompleteOneInputTransition(const string& InitStateName,
                                                   const SymmMsgDeclRef& MsgDecl,
                                                   const map<string, ExprTypeRef>& DomainVars,
                                                   vector<ExpT>& GuardExps,
                                                   const ExpT& CoveredPred);

        public:
            IncompleteEFSM(LabelledTS* TheLTS, const string& Name,
                           const vector<ExpT>& Params, const ExpT& Constraint,
                           LTSFairnessType Fairness = LTSFairnessType::None);

            virtual ~IncompleteEFSM();
            // overrides to remember variables
            virtual void AddVariable(const string& VarName, 
                                     const ExprTypeRef& VarType) override;

            // Do not add completions particular set of messages on a 
            // particular state
            void IgnoreMsgOnState(const SymmMsgDeclRef& MsgDecl,
                                  const string& StateName);
            void IgnoreAllMsgsOnState(const string& StateName);
            void HandleMsgOnState(const SymmMsgDeclRef& MsgDecl,
                                  const string& StateName);
            
            // Do not add any more completions on any message 
            // type on a particular state
            void MarkStateComplete(const string& StateName);
            void MarkAllStatesComplete();
            void MarkStateIncomplete(const string& StateName);

            // Do not include updates to variables
            // in completion
            void MarkVariableReadOnly(const string& VarName);
            void MarkAllVariablesReadOnly();
            void MarkVariableWriteable(const string& VarName);
            
            const set<i64>& GetGuardUFIDs() const;
            const map<LTSSymbTransRef, set<ExpT>>& GetConstraintsByTransition() const;

            // override freeze to add additional transitions
            // and such
            virtual void Freeze() override;
        };

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_EFSM_HPP_ */

// 
// LTSEFSM.hpp ends here
