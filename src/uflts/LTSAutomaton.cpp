// LTSAutomaton.cpp --- 
// 
// Filename: LTSAutomaton.cpp
// Author: Abhishek Udupa
// Created: Fri Aug 15 12:03:29 2014 (-0400)
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

#include "LTSAutomaton.hpp"
#include "LTSUtils.hpp"
#include "LabelledTS.hpp"
#include "LTSState.hpp"

namespace ESMC {
    namespace LTS {

        AutomatonBase::AutomatonBase(LabelledTS* TheLTS, const string& Name,
                                     const vector<ExpT>& Params, const ExpT& Constraint)
            : TheLTS(TheLTS), Name(Name), Params(Params), Constraint(Constraint),
              StatesFrozen(false)
        {
            auto Mgr = TheLTS->GetMgr();
            CheckParams(Params, Constraint, SymTab, Mgr, true);

            // Instantiate the parameters
            const u32 NumParams = Params.size();
            ParamInsts = InstantiateParams(Params, Constraint, Mgr);

            for (auto const& ParamInst : ParamInsts) {
                MgrT::SubstMapT SubstMap;
                for (u32 i = 0; i < NumParams; ++i) {
                    SubstMap[Params[i]] = ParamInst[i];
                }
                ParamSubsts.push_back(SubstMap);
            }
        }

        AutomatonBase::~AutomatonBase()
        {
            // Nothing here
        }

        void AutomatonBase::AssertStatesFrozen() const
        {
            if (!StatesFrozen) {
                throw ESMCError((string)"An operation was attempted before states of the " + 
                                "automaton \"" + Name + "\" were frozen. This operation " + 
                                "can only be performed after freezing states");
            }
        }

        void AutomatonBase::AssertStatesNotFrozen() const
        {
            if (StatesFrozen) {
                throw ESMCError((string)"An operation was attempted after states of the " + 
                                "automaton \"" + Name + "\" were frozen. This operation " + 
                                "can only be performed before freezing states");
            }
        }

        void AutomatonBase::CheckState(const string& StateName) const
        {
            if (States.find(StateName) == States.end()) {
                throw ESMCError((string)"Automaton named \"" + Name + "\" has no state named \"" + 
                                StateName + "\"");
            }
        }

        void AutomatonBase::FreezeStates()
        {
            auto Mgr = TheLTS->GetMgr();
            if (StatesFrozen) {
                return;
            }

            set<string> StateNames;
            for (auto const& State : States) {
                StateNames.insert(State.first);
            }

            StateType = Mgr->MakeType<ExprEnumType>(Name + "_StateT", StateNames);
            StatesFrozen = true;
        }

        void AutomatonBase::AddState(const string& StateName,
                                     bool Initial, bool Final,
                                     bool Accepting, bool Error)
        {
            if (States.find(StateName) != States.end()) {
                throw ESMCError((string)"Error, state \"" + StateName + "\" already " + 
                                "declared for automaton \"" + Name + "\"");
            }

            States[StateName] = LTSState(StateName, Initial, Final, Accepting, Error);
        }

        vector<LTSState> AutomatonBase::GetStates() const
        {
            vector<LTSState> Retval;
            for (auto const& State : States) {
                Retval.push_back(State.second);
            }
            return Retval;
        }

        const ExprTypeRef& AutomatonBase::GetStateType() const
        {
            if (!StatesFrozen) {
                throw ESMCError((string)"Cannot call AutomatonBase::GetStateType() before " + 
                                "freezing states of automaton using Automaton::FreezeStates()");
            }
            return StateType;
        }

        const vector<vector<ExpT>>& AutomatonBase::GetParamInsts() const
        {
            return ParamInsts;
        }

        const vector<MgrT::SubstMapT>& AutomatonBase::GetParamSubsts() const
        {
            return ParamSubsts;
        }

        u32 AutomatonBase::GetNumInstances() const
        {
            return ParamInsts.size();
        }

        const string& AutomatonBase::GetName() const
        {
            return Name;
        }

        u32 AutomatonBase::GetNumInstancesUnconstrained() const
        {
            u32 Retval = 1;
            for (auto const& Param : Params) {
                Retval *= Param->GetType()->GetCardinality();
            }
            return Retval;
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// LTSAutomaton.cpp ends here
