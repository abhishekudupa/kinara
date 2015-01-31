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
using LTS::TypeRef;
using Symm::PermutationSet;

inline void BuchiAutomatonBase::AssertStatesFrozen() const
{
    if (!StatesFrozen) {
        throw ESMCError((string)"The requested operation can only be performed " +
                        "after the states of the Buchi Automaton have been frozen");
    }
}

inline void BuchiAutomatonBase::AssertStatesNotFrozen() const
{
    if (StatesFrozen) {
        throw ESMCError((string)"The requested operation can only be performed " +
                        "before the states of the Buchi Automaton have been frozen");
    }
}

BuchiAutomatonBase::BuchiAutomatonBase(LabelledTS* TheLTS, const string& Name,
                                       const vector<ExpT>& SymmIndices,
                                       const ExpT& Constraint)
    : Name(Name), SymmIndices(SymmIndices), Constraint(Constraint),
      TheLTS(TheLTS), StatesFrozen(false), NumStates(0)
{
    SymTab.Pop();
    SymTab.Push(TheLTS->SymTab.Bot());
    // Push another scope to avoid polluting the
    // symbol table of the LTS with my parameters
    SymTab.Push();
    LTS::CheckParams(SymmIndices, Constraint, SymTab, TheLTS->GetMgr(), true);
    // Create the substitution maps for each permutation
    // starting from the canonical initial index values
    SymmInsts = LTS::InstantiateParams(SymmIndices, Constraint,
                                       TheLTS->GetMgr());
    TheIndexSet = new ProcessIndexSet(SymmInsts);
}

BuchiAutomatonBase::~BuchiAutomatonBase()
{
    delete TheIndexSet;
}

void BuchiAutomatonBase::AddState(const string& StateName, bool Initial, bool Accepting)
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

i32 BuchiAutomatonBase::GetStateIDForName(const string& StateName) const
{
    auto it = StateNameToStateID.find(StateName);
    if (it == StateNameToStateID.end()) {
        throw ESMCError((string)"Unknown state name \"" + StateName + "\"" +
                        " referenced in Buchi monitor \"" + Name + "\"");
    }
    return it->second;
}

const string& BuchiAutomatonBase::GetStateNameForID(u32 StateID) const
{
    auto it = StateIDToStateName.find(StateID);
    if (it == StateIDToStateName.end()) {
        throw ESMCError((string)"Unknown StateID " + to_string(StateID) +
                        " in referenced in Buchi monitor \"" + Name + "\"");
    }
    return it->second;
}

void BuchiAutomatonBase::FreezeStates()
{
    if (StatesFrozen) {
        return;
    }

    StatesFrozen = true;
}

const vector<u32>& BuchiAutomatonBase::GetInitialStates() const
{
    return InitialStates;
}

const unordered_set<u32>& BuchiAutomatonBase::GetAcceptingStates() const
{
    return AcceptingStates;
}

bool BuchiAutomatonBase::IsAccepting(u32 StateID) const
{
    return (AcceptingStates.find(StateID) != AcceptingStates.end());
}

const ProcessIndexSet* BuchiAutomatonBase::GetIndexSet() const
{
    return TheIndexSet;
}


inline void StateBuchiAutomaton::AssertFrozen() const
{
    if (!Frozen) {
        throw ESMCError((string)"The requested operation can only be performed " +
                        "after the Buchi Automaton has been frozen");
    }
}

inline void StateBuchiAutomaton::AssertNotFrozen() const
{
    if (Frozen) {
        throw ESMCError((string)"The requested operation can only be performed " +
                        "before the Buchi Automaton has been frozen");
    }
}

StateBuchiAutomaton::StateBuchiAutomaton(LabelledTS* TheLTS, const string& Name,
                                         const vector<ExpT>& SymmIndices,
                                         const ExpT& Constraint, LTSCompiler* Compiler)
    : BuchiAutomatonBase(TheLTS, Name, SymmIndices, Constraint),
      Compiler(Compiler), Frozen(false)
{
    // Nothing here
}

StateBuchiAutomaton::~StateBuchiAutomaton()
{
    // Nothing here
}


void StateBuchiAutomaton::FreezeStates()
{
    if (StatesFrozen) {
        return;
    }

    BuchiAutomatonBase::FreezeStates();
    Transitions =
        vector<vector<vector<pair<ExpT, u32>>>>(TheIndexSet->GetNumIndexVectors(),
                                                vector<vector<pair<ExpT, u32>>>(NumStates));
}

void StateBuchiAutomaton::AddTransition(const string& InitState,
                                        const string& FinalState,
                                        const ExpT& Guard)
{
    AssertStatesFrozen();
    AssertNotFrozen();

    auto Mgr = TheLTS->GetMgr();

    if (!Guard->GetType()->Is<BooleanType>()) {
        throw ESMCError((string)"Guards for Buchi Automaton transitions " +
                        "must be Boolean valued");
    }

    LTS::CheckExpr(Guard, SymTab, Mgr);
    auto UnrolledGuard = Mgr->UnrollQuantifiers(Guard, false);

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

    auto const& ExistingTransitions = Transitions[0][InitStateID];
    for (auto const& Transition : ExistingTransitions) {
        if (Transition.second == FinalStateID) {
            throw ESMCError((string)"Parallel edges in Buchi monitor between " +
                            "states " + InitState + " and " + FinalState +
                            ".\nMake a disjunctive guard instead");
        }
    }

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

ExpT StateBuchiAutomaton::MakeVar(const string& Name, const TypeRef& Type)
{
    auto Retval = TheLTS->MakeVar(Name, Type);
    LTS::CheckExpr(Retval, SymTab, TheLTS->GetMgr());
    return Retval;
}

ExpT StateBuchiAutomaton::MakeBoundVar(i64 Idx, const TypeRef& Type)
{
    return TheLTS->MakeBoundVar(Idx, Type);
}

ExpT StateBuchiAutomaton::MakeVal(const string& Value, const TypeRef& Type)
{
    return TheLTS->MakeVal(Value, Type);
}

ExpT StateBuchiAutomaton::MakeExists(const vector<TypeRef>& QVarTypes,
                                     const ExpT& Body)
{
    return TheLTS->MakeExists(QVarTypes, Body);
}

ExpT StateBuchiAutomaton::MakeForAll(const vector<TypeRef>& QVarTypes, const ExpT& Body)
{
    return TheLTS->MakeForAll(QVarTypes, Body);
}

const TypeRef& StateBuchiAutomaton::GetNamedType(const string& TypeName)
{
    return TheLTS->GetNamedType(TypeName);
}


vector<u32> StateBuchiAutomaton::GetNextStates(u32 CurState, u32 IndexID,
                                               const StateVec* StateVector) const
{
    auto const& TransVec = Transitions[IndexID][CurState];
    vector<u32> Retval;

    for (auto const& Trans : TransVec) {
        auto Interp = Trans.first->ExtensionData.Interp;
        if (Interp->Evaluate(StateVector) != 0) {
            Retval.push_back(Trans.second);
        }
    }

    return Retval;
}

const ExpT& StateBuchiAutomaton::GetGuardForTransition(u32 FromState,
                                                       u32 ToState,
                                                       u32 IndexID) const
{
    if (FromState >= NumStates || ToState >= NumStates) {
        throw ESMCError((string)"State IDs out of bounds for Buchi monitor");
    }
    auto const& TransVec = Transitions[IndexID][FromState];
    for (auto const& Transition : TransVec) {
        if (ToState == Transition.second) {
            return Transition.first;
        }
    }

    auto it1 = StateIDToStateName.find(FromState);
    auto it2 = StateIDToStateName.find(ToState);
    throw ESMCError((string)"No transition between states: " +
                    it1->second + " to " + it2->second);
}

void StateBuchiAutomaton::Freeze()
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
