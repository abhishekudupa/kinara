// OmegaAutomaton.cpp --- 
// 
// Filename: OmegaAutomaton.cpp
// Author: Abhishek Udupa
// Created: Tue Aug 26 02:31:46 2014 (-0400)
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

#include "../uflts/LTSUtils.hpp"
#include "../uflts/LabelledTS.hpp"
#include "../uflts/LTSExtensions.hpp"
#include "../symmetry/Permutations.hpp"

#include "OmegaAutomaton.hpp"
#include "Compiler.hpp"
#include "IndexSet.hpp"

namespace ESMC {
    namespace MC {

        using LTS::SymbolTable;
        using LTS::LabelledTS;
        using LTS::ExpT;
        using LTS::ExprTypeRef;
        using Symm::PermutationSet;

        inline void BuchiAutomaton::AssertStatesFrozen() const
        {
            if (!StatesFrozen) {
                throw ESMCError((string)"The requested operation can only be performed " + 
                                "after the states of the Buchi Automaton have been frozen");
            }
        }

        inline void BuchiAutomaton::AssertStatesNotFrozen() const
        {
            if (StatesFrozen) {
                throw ESMCError((string)"The requested operation can only be performed " + 
                                "before the states of the Buchi Automaton have been frozen");
            }
        }        

        inline void BuchiAutomaton::AssertFrozen() const
        {
            if (!Frozen) {
                throw ESMCError((string)"The requested operation can only be performed " + 
                                "after the Buchi Automaton has been frozen");
            }
        }

        inline void BuchiAutomaton::AssertNotFrozen() const
        {
            if (Frozen) {
                throw ESMCError((string)"The requested operation can only be performed " + 
                                "before the Buchi Automaton has been frozen");
            }
        }        

        BuchiAutomaton::BuchiAutomaton(LabelledTS* TheLTS, const string& Name,
                                       const vector<ExpT>& SymmIndices,
                                       const ExpT& Constraint, LTSCompiler* Compiler)
            : Name(Name), SymmIndices(SymmIndices), 
              Constraint(Constraint), TheLTS(TheLTS),
              Compiler(Compiler), StatesFrozen(false), Frozen(false),
              NumStates(0)
        {
            SymTab.Pop();
            SymTab.Push(TheLTS->SymTab.Bot());
            
            LTS::CheckParams(SymmIndices, Constraint, SymTab, TheLTS->GetMgr(), true);
            // Create the substitution maps for each permutation
            // starting from the canonical initial index values
            SymmInsts = LTS::InstantiateParams(SymmIndices, Constraint, 
                                                 TheLTS->GetMgr());
            TheIndexSet = new ProcessIndexSet(SymmInsts);
        }

        BuchiAutomaton::~BuchiAutomaton()
        {
            delete TheIndexSet;
        }

        void BuchiAutomaton::AddState(const string& StateName, bool Initial, bool Accepting)
        {
            AssertStatesNotFrozen();

            auto StateUID = StateUIDGenerator.GetUID();
            if (StateNameToStateID.find(StateName) != StateNameToStateID.end()) {
                throw ESMCError((string)"State with name \"" + StateName + "\" already " + 
                                "exists in Buchi Automaton \"" + Name + "\".");
            }
            StateNameToStateID[StateName] = StateUID;
            StateIDToStateName[StateUID] = StateName;

            if (Initial) {
                InitialStates.push_back(StateUID);
            }
            if (Accepting) {
                AcceptingStates.insert(StateUID);
            }
            ++NumStates;
        }

        void BuchiAutomaton::FreezeStates()
        {
            if (StatesFrozen) {
                return;
            }
            
            StatesFrozen = true;
            Transitions = vector<vector<vector<pair<ExpT, u32>>>>(SymmInsts.size(), 
                                                                  vector<vector<pair<ExpT, u32>>>(NumStates));
        }

        void BuchiAutomaton::AddTransition(const string& InitState, 
                                           const string& FinalState, 
                                           const ExpT& Guard)
        {
            AssertStatesFrozen();
            AssertNotFrozen();
            
            auto Mgr = TheLTS->GetMgr();

            if (!Guard->GetType()->Is<Exprs::ExprBoolType>()) {
                throw ESMCError((string)"Guards for Buchi Automaton transitions " + 
                                "must be Boolean valued");
            }

            LTS::CheckExpr(Guard, SymTab, Mgr);
            auto UnrolledGuard = 
                Mgr->ApplyTransform<LTS::Detail::QuantifierUnroller>(Guard);

            auto it = StateNameToStateID.find(InitState);
            if (it == StateNameToStateID.end()) {
                throw ESMCError((string)"Initial state \"" + InitState + "\"" + 
                                " for Buchi Automaton transition undefined");
            }
            auto InitStateID = it->second;
            it = StateNameToStateID.find(FinalState);
            if (it == StateNameToStateID.end()) {
                throw ESMCError((string)"Final state \"" + FinalState + "\"" + 
                                " for Buchi Automaton transition undefined");
            }
            auto FinalStateID = it->second;
            u32 InstIdx = 0;
            for (auto const& SymmInst : SymmInsts) {
                // Create a substitution map for this instance
                MgrT::SubstMapT SubstMap;
                for (u32 i = 0; i < SymmInst.size(); ++i) {
                    SubstMap[SymmIndices[i]] = SymmInst[i];
                }

                auto SubstGuard = Mgr->Substitute(SubstMap, UnrolledGuard);
                Compiler->CompileExp(SubstGuard, TheLTS);
                auto NewTrans = make_pair(SubstGuard, FinalStateID);
                Transitions[InstIdx][InitStateID].push_back(NewTrans);
                ++InstIdx;
            }
        }

        ExpT BuchiAutomaton::MakeVar(const string& Name, const ExprTypeRef& Type)
        {
            auto Retval = TheLTS->MakeVar(Name, Type);
            LTS::CheckExpr(Retval, SymTab, TheLTS->GetMgr());
            return Retval;
        }

        ExpT BuchiAutomaton::MakeBoundVar(i64 Idx, const ExprTypeRef& Type)
        {
            return TheLTS->MakeBoundVar(Idx, Type);
        }

        ExpT BuchiAutomaton::MakeVal(const string& Value, const ExprTypeRef& Type)
        {
            return TheLTS->MakeVal(Value, Type);
        }

        ExpT BuchiAutomaton::MakeExists(const vector<ExprTypeRef>& QVarTypes, const ExpT& Body)
        {
            return TheLTS->MakeExists(QVarTypes, Body);
        }

        ExpT BuchiAutomaton::MakeForAll(const vector<ExprTypeRef>& QVarTypes, const ExpT& Body)
        {
            return TheLTS->MakeForAll(QVarTypes, Body);
        }

        const ExprTypeRef& BuchiAutomaton::GetNamedType(const string& TypeName)
        {
            return TheLTS->GetNamedType(TypeName);
        }

        const vector<u32>& BuchiAutomaton::GetInitialStates() const
        {
            return InitialStates;
        }

        vector<u32> BuchiAutomaton::GetNextStates(u32 CurState, u32 IndexID, 
                                                  const StateVec* StateVector) const
        {
            auto const& TransVec = Transitions[IndexID][CurState];
            vector<u32> Retval;

            for (auto const& Trans : TransVec) {
                auto Interp = Trans.first->ExtensionData.Interp;
                if (Interp->EvaluateScalar(StateVector) != 0) {
                    Retval.push_back(Trans.second);
                }
            }
            
            return Retval;
        }

        const unordered_set<u32>& BuchiAutomaton::GetAcceptingStates() const
        {
            return AcceptingStates;
        }

        bool BuchiAutomaton::IsAccepting(u32 StateID) const
        {
            return (AcceptingStates.find(StateID) != AcceptingStates.end());
        }

        const ProcessIndexSet* BuchiAutomaton::GetIndexSet() const
        {
            return TheIndexSet;
        }

        void BuchiAutomaton::Freeze()
        {
            if (Frozen) {
                return;
            }

            AssertStatesFrozen();
            Frozen = true;
        }

    } /* end namespace MC */
} /* end namespace ESMC */

// 
// OmegaAutomaton.cpp ends here
