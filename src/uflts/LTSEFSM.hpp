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

#include "LTSDecls.hpp"
#include "LTSState.hpp"
#include "LTSEFSMBase.hpp"

namespace ESMC {
    namespace LTS {

        using namespace Symm;

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
            friend class ESMC::LTS::LabelledTS;
            friend class ESMC::Synth::Solver;

        private:
            // Map from update op code to the LValue term
            // that it updates
            unordered_map<i64, pair<ExpT, ExpT>> UpdateOpToUpdateLValue;

            unordered_map<i64, ExpT> GuardOpToExp;
            unordered_map<i64, set<ExpT>> GuardSymmetryConstraints;
            unordered_map<i64, set<ExpT>> GuardMutualExclusiveSets;
            unordered_map<i64, set<ExpT>> GuardOpToUpdates;
            unordered_map<i64, set<ExpT>> GuardOpToUpdateSymmetryConstraints;
            unordered_map<string, set<ExpT>> AddedTransitionsByState;
            unordered_map<i64, ExpT> StateUpdateOpToExp;
            unordered_map<i64, ExpT> AllOpToExp;

            map<string, set<SymmMsgDeclRef>> BlockedCompletions;
            set<string> CompleteStates;
            set<string> ReadOnlyVars;
            map<string, TypeRef> UpdateableVariables;
            map<string, TypeRef> AllVariables;
            map<pair<string, SymmMsgDeclRef>, set<string>> VarDeps;
            map<pair<string, SymmMsgDeclRef>, set<string>> VarMsgFieldDeps;
            map<pair<string, SymmMsgDeclRef>, set<string>> OutMsgFieldDeps;
            map<pair<string, SymmMsgDeclRef>, set<string>> NextStatesOnTransition;

            UIDGenerator GuardUFUIDGen;
            UIDGenerator UpdateUFUIDGen;

            inline void FilterTerms(set<ExpT>& DomainTerms, const TypeRef& RangeType);
            set<ExpT> GetDomainTerms(const map<string, TypeRef>& DomainVars);

            inline set<set<ExpT>> GetArrayLValueGroups(const set<ExpT>& LValues);

            inline vector<TypeRef> GetSymmTypesInExpr(const ExpT& Exp);
            inline void PartitionDomain(const vector<ExpT>& Args,
                                        vector<ExpT>& SymmArgs,
                                        vector<ExpT>& NonSymmArgs);

            inline void MergeEquivalences(const set<ExpT>& NewEquivalences,
                                          set<set<ExpT>>& EquivalenceSets);

            inline set<set<ExpT>> FindEquivalences(const ExpT& Exp,
                                                   const vector<TypeRef>& SymmTypes,
                                                   const vector<ExpT>& SymmArgs,
                                                   const vector<ExpT>& NonSymmArgs);

            inline vector<ExpT> GetSymmetryConstraints(const ExpT& Exp);

            // Make symmetry constraints for a group
            inline vector<ExpT> GetSymmetryConstraints(const set<ExpT>& UpdateGroup,
                                                       const map<ExpT, ExpT>& UpdateMap);

            inline ExpT FindDisjunction(const vector<LTSSymbTransRef>& Transitions,
                                        const TPRef& TP,
                                        const ExpT& CoveredRegion);

            inline ExpT FindInputCoveredRegion(const vector<LTSSymbTransRef>& Transitions,
                                               const TPRef& TP,
                                               const TypeRef& MsgType,
                                               const ExpT& CoveredRegion);

            inline ExpT FindGlobalCoveredRegion(const vector<LTSSymbTransRef>& Transitions,
                                                const TPRef& TP);

            inline void CompleteInputTransitions(const string& StateName,
                                                 const vector<LTSSymbTransRef>& Transitions,
                                                 const ExpT& CoveredPredicate,
                                                 const TPRef& TP);

            inline ExpT MakeGuard(const set<ExpT>& DomainTerms,
                                  const ExpT& CoveredPredicate,
                                  const string& NameSuffix);

            inline set<ExpT> GetDomainTermsForUpdate(const ExpT& LValueTerm,
                                                     const set<ExpT>& DomainTerms,
                                                     const SymmMsgDeclRef& MsgDecl);

            inline vector<LTSAssignRef> MakeUpdates(i64 GuardOp,
                                                    const string& InitStateName,
                                                    const set<ExpT>& DomainTerms,
                                                    const string& NameSuffix,
                                                    const SymmMsgDeclRef& MsgDecl);

            inline void CompleteOneInputTransition(const string& InitStateName,
                                                   const SymmMsgDeclRef& MsgDecl,
                                                   const map<string, TypeRef>& DomainVars,
                                                   const ExpT& CoveredPred);

            inline void CompleteOutputTransitions(const string& InitStateName,
                                                  const ExpT& CoveredPredicate,
                                                  const TPRef& TP);

            inline void CompleteOneOutputTransition(const string& InitStateName,
                                                    const SymmMsgDeclRef& MsgDecl,
                                                    const map<string, TypeRef>& DomainVars,
                                                    vector<ExpT>& GuardExps,
                                                    const ExpT& CoveredPred);


        public:
            IncompleteEFSM(LabelledTS* TheLTS, const string& Name,
                           const vector<ExpT>& Params, const ExpT& Constraint,
                           LTSFairnessType Fairness = LTSFairnessType::None);

            virtual ~IncompleteEFSM();
            // overrides to remember variables
            virtual void AddVariable(const string& VarName,
                                     const TypeRef& VarType) override;

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

            void SetVariableDepsOnMsg(const string& VarName,
                                      const SymmMsgDeclRef& MsgDecl,
                                      const set<string>& DepVars,
                                      const set<string>& MessageFieldName);

            void SetOutMsgFieldDeps(const SymmMsgDeclRef& OutMsgDecl,
                                    const string& FieldName,
                                    const set<string>& DepVars);

            void SetNextStatesOnTransition(const string& StateName,
                                           const SymmMsgDeclRef& MsgDecl,
                                           const set<string>& NextStateNames);

            // override freeze to add additional transitions
            // and such
            virtual void Freeze() override;
        };

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_EFSM_HPP_ */

//
// LTSEFSM.hpp ends here
