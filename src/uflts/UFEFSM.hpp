// UFEFSM.hpp --- 
// 
// Filename: UFEFSM.hpp
// Author: Abhishek Udupa
// Created: Wed Jul 23 19:33:33 2014 (-0400)
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

#if !defined ESMC_UF_EFSM_HPP_
#define ESMC_UF_EFSM_HPP_

#include "../common/FwdDecls.hpp"
#include "../utils/UIDGenerator.hpp"
#include "../expr/Expressions.hpp"
#include "../symexec/Analyses.hpp"

#include "SymbolTable.hpp"
#include "Transitions.hpp"
#include "LTSTermSemanticizer.hpp"
#include "UFLTSExtension.hpp"
#include "LTSUtils.hpp"

namespace ESMC {
    namespace LTS {

        // The class for an I/O EFSM which can contain uninterpreted functions
        class UFEFSM
        {
            friend class UFLTS;

        private:
            UFLTS* TheLTS;
            string Name;
            vector<ExpT> Params;
            ExpT Constraint;
            SymbolTable SymTab;
            vector<FrozenEFSM*> FrozenEFSMs;
            bool StatesFrozen;
            bool AllFrozen;
            map<string, Detail::StateDescriptor> States;
            vector<vector<ExpT>> ParamInsts;
            vector<MgrType::SubstMapT> ParamSubsts;
            ExprTypeRef StateType;

            // Helpers
            inline void AddMsg(const ExprTypeRef& MType,
                               const vector<ExpT>& MParams,
                               bool IsInput);

            inline void AddMsgs(const vector<ExpT>& NewParams,
                                const ExprTypeRef& MType,
                                const vector<ExpT>& MParams,
                                const ExpT& Constraint,
                                bool IsInput);

            inline vector<AsgnT> InstantiateParametricVars(const vector<AsgnT>& Updates,
                                                           const string& VarName,
                                                           const ExprTypeRef& ParametricType,
                                                           const ExprTypeRef& InstantiatedType);

        public:
            UFEFSM(UFLTS* TheLTS, const string& Name);
            UFEFSM(UFLTS* TheLTS, 
                   const string& Name,
                   const vector<ExpT>& Params,
                   const ExpT& Constraint);

            ~UFEFSM();

            const string& GetName() const;

            void AddState(const string& StateName,
                          bool Initial = false,
                          bool Final = false,
                          bool Accepting = false,
                          bool Error = false,
                          bool Dead = false);
            
            void FreezeStates();
            void FreezeAll();
            
            void AddInputMsg(const ExprTypeRef& MType,
                             const vector<ExpT>& MParams);

            void AddOutputMsg(const ExprTypeRef& MType,
                              const vector<ExpT>& MParams);
            
            void AddInputMsgs(const vector<ExpT>& NewParams,
                              const ExprTypeRef& MType,
                              const vector<ExpT>& MParams,
                              const ExpT& MConstraint);

            void AddOutputMsgs(const vector<ExpT>& NewParams,
                               const ExprTypeRef& MType,
                               const vector<ExpT>& MParams,
                               const ExpT& MConstraint);

            void AddVariable(const string& VarName,
                             const ExprTypeRef& VarType);

            // A helper function to figure out how many transitions
            // a particular parameterized transition expands to
            u32 GetNumExpansions(const vector<ExpT>& NewParams,
                                 const ExpT& Constraint);
            
            void AddInputTransition(const string& InitState,
                                    const string& FinalState,
                                    const ExpT& Guard,
                                    const vector<AsgnT>& Updates,
                                    const string& MessageName,
                                    const ExprTypeRef& MessageType,
                                    const vector<ExpT>& MessageParams);

            void AddInputTransitions(const string& InitState,
                                     const string& FinalState,
                                     const vector<ExpT> TransParams,
                                     const ExpT& TransConstraint,
                                     const ExpT& Guard,
                                     const vector<AsgnT>& Updates,
                                     const string& MessageName,
                                     const ExprTypeRef& MessageType,
                                     const vector<ExpT>& MessageParams);

            void AddOutputTransition(const string& InitState,
                                     const string& FinalState,
                                     const ExpT& Guard,
                                     const vector<AsgnT>& Updates,
                                     const string& MessageName,
                                     const ExprTypeRef& MessageType,
                                     const vector<ExpT>& MessageParams,
                                     const unordered_set<u32>& FairnessSet = 
                                     unordered_set<u32>());


            // The fairness sets can be either:
            // 1. Empty : in which case, no fairness assumptions on transitions
            // 2. A singleton : in which case, the fairness is splat across all transitions
            // 3. A vector with as many assumptions as the transition splits up to

            void AddOutputTransitions(const string& InitState,
                                      const string& FinalState,
                                      const vector<ExpT>& TransParams,
                                      const ExpT& TransConstraint,
                                      const ExpT& Guard,
                                      const vector<AsgnT>& Updates,
                                      const string& MessageName,
                                      const ExprTypeRef& MessageType,
                                      const vector<ExpT>& MessageParams,
                                      const vector<unordered_set<u32>>& FairnessSets = 
                                      vector<unordered_set<u32>>());

            void AddInternalTransition(const string& InitState,
                                       const string& FinalState,
                                       const ExpT& Guard,
                                       const vector<AsgnT>& Updates,
                                       const unordered_set<u32>& FairnessSet = 
                                       unordered_set<u32>());

            void AddInternalTransitions(const string& InitState,
                                        const string& FinalState,
                                        const vector<ExpT>& TransParams,
                                        const ExpT& TransConstraint,
                                        const ExpT& Guard,
                                        const vector<AsgnT>& Updates,
                                        const vector<unordered_set<u32>>& FairnessSets = 
                                        vector<unordered_set<u32>>());

            const vector<FrozenEFSM*>& GetFrozenEFSMs();
        };


    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_UF_EFSM_HPP_ */

// 
// UFEFSM.hpp ends here
