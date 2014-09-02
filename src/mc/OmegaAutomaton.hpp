// OmegaAutomaton.hpp --- 
// 
// Filename: OmegaAutomaton.hpp
// Author: Abhishek Udupa
// Created: Tue Aug 26 01:27:03 2014 (-0400)
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

#if !defined ESMC_OMEGA_AUTOMATON_HPP_
#define ESMC_OMEGA_AUTOMATON_HPP_

#include "../common/FwdDecls.hpp"
#include "../utils/UIDGenerator.hpp"

namespace ESMC {
    namespace MC {

        using LTS::SymbolTable;
        using LTS::LabelledTS;
        using LTS::ExpT;
        using LTS::ExprTypeRef;
        using Symm::PermutationSet;
        using LTS::MgrT;

        // An indexed Buchi automaton
        class BuchiAutomaton
        {
        private:
            string Name;
            vector<ExpT> SymmIndices;
            vector<vector<ExpT>> SymmInsts;
            ExpT Constraint;
            LabelledTS* TheLTS;
            UIDGenerator StateUIDGenerator;
            SymbolTable SymTab;
            map<string, u32> StateNameToStateID;
            map<u32, string> StateIDToStateName;
            LTSCompiler* Compiler;
            bool StatesFrozen;
            bool Frozen;
            // Transitions for each permutation
            // further indexed by stateid.
            // A transition is a guard and a next state id
            vector<vector<vector<pair<ExpT, u32>>>> Transitions;
            unordered_set<u32> AcceptingStates;
            vector<u32> InitialStates;
            u32 NumStates;
            ProcessIndexSet* TheIndexSet;
            
            inline void AssertFrozen() const;
            inline void AssertNotFrozen() const;
            inline void AssertStatesFrozen() const;
            inline void AssertStatesNotFrozen() const;

            inline void GenSubstMaps(const vector<ExpT>& InitInst);
            
        public:
            BuchiAutomaton(LabelledTS* TheLTS, const string& Name, 
                           const vector<ExpT>& SymmIndices,
                           const ExpT& Constraint, LTSCompiler* Compiler);

            ~BuchiAutomaton();
            void AddState(const string& StateName, bool Initial,
                          bool Accepting);
            void FreezeStates();
            void AddTransition(const string& InitState, const string& FinalState,
                               const ExpT& Guard);

            // forwards with checks to LTS
            ExpT MakeVar(const string& Name, const ExprTypeRef& Type);
            ExpT MakeBoundVar(i64 Idx, const ExprTypeRef& Type);
            ExpT MakeVal(const string& Value, const ExprTypeRef& Type);

            template <typename... ArgTypes>
            inline ExpT MakeOp(ArgTypes&&... Args)
            {
                return TheLTS->MakeOp(forward<ArgTypes>(Args)...);
            }

            ExpT MakeExists(const vector<ExprTypeRef>& QVarTypes, const ExpT& Body);
            ExpT MakeForAll(const vector<ExprTypeRef>& QVarTypes, const ExpT& Body);
            const ExprTypeRef& GetNamedType(const string& TypeName);

            // Methods for model checking
            const vector<u32>& GetInitialStates() const;
            vector<u32> GetNextStates(u32 CurState, u32 IndexID, 
                                      const StateVec* StateVector) const;
            const unordered_set<u32>& GetAcceptingStates() const;
            bool IsAccepting(u32 StateID) const;
            const ProcessIndexSet* GetIndexSet() const;

            void Freeze();
        };

    } /* end namespace MC */
} /* end namespace ESMC */

#endif /* ESMC_OMEGA_AUTOMATON_HPP_ */

// 
// OmegaAutomaton.hpp ends here
