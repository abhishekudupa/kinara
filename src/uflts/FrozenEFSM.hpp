// FrozenEFSM.hpp --- 
// 
// Filename: FrozenEFSM.hpp
// Author: Abhishek Udupa
// Created: Sun Aug  3 14:31:14 2014 (-0400)
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

#if !defined ESMC_FROZEN_EFSM_HPP_
#define ESMC_FROZEN_EFSM_HPP_

#include "../common/FwdDecls.hpp"
#include "LTSUtils.hpp"

namespace ESMC {
    namespace LTS {

        class FrozenEFSM
        {
            friend class UFLTS;
            friend class UFEFSM;

        private:
            string Name;
            UFLTS* TheLTS;
            ExprTypeRef StateType;
            SymbolTable SymTab;
            set<ExprTypeRef> Inputs;
            set<ExprTypeRef> Outputs;
            map<string, Detail::StateDescriptor> States;
            
            vector<pair<TransitionT, ScopeRef>> Transitions;

            void CanonicalizeFairness();
            
        public:
            FrozenEFSM(const string& Name, UFLTS* TheLTS,
                       const ExprTypeRef& StateType,
                       const map<string, Detail::StateDescriptor>& States);
            ~FrozenEFSM();

            void AddVariable(const string& VarName,
                             const ExprTypeRef& VarType);

            void AddInputMsg(const ExprTypeRef& MType);
            void AddOutputMsg(const ExprTypeRef& MType);

            void AddInputTransition(const string& InitState,
                                    const string& FinalState,
                                    const ExpT& Guard,
                                    const vector<AsgnT>& Updates,
                                    const string& MessageName,
                                    const ExprTypeRef& MessageType);

            void AddOutputTransition(const string& InitState,
                                     const string& FinalState,
                                     const ExpT& Guard,
                                     const vector<AsgnT>& Updates,
                                     const string& MessageName,
                                     const ExprTypeRef& MessageType,
                                     const unordered_set<u32>& FairnessSet);

            void AddInternalTransition(const string& InitState,
                                       const string& FinalState,
                                       const ExpT& Guard,
                                       const vector<AsgnT>& Updates,
                                       const unordered_set<u32>& FairnessSet);
            
            const set<ExprTypeRef>& GetInputs() const;
            const set<ExprTypeRef>& GetOutputs() const;
            const ExprTypeRef& GetStateType() const;
            const SymbolTable& GetSymTab() const;
            const vector<TransitionT>& GetTransitions() const;
            const string& GetName() const;
            UFLTS* GetLTS() const;
            const map<string, Detail::StateDescriptor>& GetStates() const;
        };
        

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_FROZEN_EFSM_HPP_ */

// 
// FrozenEFSM.hpp ends here
