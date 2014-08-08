// LTSTransitions.cpp --- 
// 
// Filename: LTSTransitions.cpp
// Author: Abhishek Udupa
// Created: Fri Aug  8 16:26:58 2014 (-0400)
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

#include "LTSTransitions.hpp"
#include "LTSState.hpp"
#include "LTSAssign.hpp"
#include "LTSEFSM.hpp"

namespace ESMC {
    namespace LTS {

        AutomatonTransitionBase::AutomatonTransitionBase(AutomatonBase* Automaton,
                                                         const LTSState& InitState,
                                                         const LTSState& FinalState,
                                                         const ExpT& Guard)
            : Automaton(Automaton), InitState(InitState), FinalState(FinalState),
              Guard(Guard)
        {
            // Nothing here
        }

        AutomatonTransitionBase::~AutomatonTransitionBase()
        {
            // Nothing here
        }

        AutomatonBase* AutomatonTransitionBase::GetAutomaton() const
        {
            return Automaton;
        }

        const LTSState& AutomatonTransitionBase::GetInitState() const
        {
            return InitState;
        }

        const LTSState& AutomatonTransitionBase::GetFinalState() const
        {
            return FinalState;
        }

        const ExpT& AutomatonTransitionBase::GetGuard() const
        {
            return Guard;
        }

        LTSTransitionBase::LTSTransitionBase(EFSMBase* TheEFSM,
                                             const LTSState& InitState,
                                             const LTSState& FinalState,
                                             const ExpT& Guard,
                                             const vector<LTSAssignRef>& Updates)
            : AutomatonTransitionBase(TheEFSM, InitState, FinalState, Guard),
              Updates(Updates)
        {
            // Nothing here
        }

        LTSTransitionBase::~LTSTransitionBase()
        {
            // Nothing here
        }

        EFSMBase* LTSTransitionBase::GetEFSM() const
        {
            return static_cast<EFSMBase*>(Automaton);
        }

        const vector<LTSAssignRef>& LTSTransitionBase::GetUpdates() const
        {
            return Updates;
        }

        LTSTransitionInput::LTSTransitionInput(EFSMBase* TheEFSM,
                                               const LTSState& InitState,
                                               const LTSState& FinalState,
                                               const ExpT& Guard,
                                               const vector<LTSAssignRef>& Updates,
                                               const string& MessageName,
                                               const ExprTypeRef& MessageType)
            : LTSTransitionBase(TheEFSM, InitState, FinalState, Guard, Updates),
              MessageName(MessageName), MessageType(MessageType)
        {
            // Nothing here
        }

        LTSTransitionInput::~LTSTransitionInput()
        {
            // Nothing here
        }

        const string& LTSTransitionInput::GetMessageName() const
        {
            return MessageName;
        }

        const ExprTypeRef& LTSTransitionInput::GetMessageType() const
        {
            return MessageType;
        }

        string LTSTransitionInput::ToString() const
        {
            ostringstream sstr;

            sstr << "transition {" << endl;
            sstr << "    " << InitState.GetName() << " -> " 
                 << FinalState.GetName() << endl;
            sstr << "    on input message \"" << MessageName 
                 << "\" of type " << MessageType->ToString() << endl;
            sstr << "    Guard: " << Guard->ToString() << endl;
            sstr << "    Updates:" << endl;
            for (auto const& Asgn : Updates) {
                sstr << Asgn->ToString() << endl;
            }
            return sstr.str();
        }

        LTSTransitionOutput::LTSTransitionOutput(EFSMBase* TheEFSM,
                                                 const LTSState& InitState,
                                                 const LTSState& FinalState,
                                                 const ExpT& Guard,
                                                 const vector<LTSAssignRef>& Updates,
                                                 const string& MessageName,
                                                 const ExprTypeRef& MessageType,
                                                 const unordered_set<u32>& CompOfFairnessSets)
            : LTSTransitionBase(TheEFSM, InitState, FinalState, Guard, Updates),
              MessageName(MessageName), MessageType(MessageType),
              CompOfFairnessSets(CompOfFairnessSets)
        {
            // Nothing here
        }

        LTSTransitionOutput::~LTSTransitionOutput()
        {
            // Nothing here
        }

        const string& LTSTransitionOutput::GetMessageName() const
        {
            return MessageName;
        }

        const ExprTypeRef& LTSTransitionOutput::GetMessageType() const
        {
            return MessageType;
        }

        const unordered_set<u32>& LTSTransitionOutput::GetCompOfFairnessSets() const
        {
            return CompOfFairnessSets;
        }

        string LTSTransitionOutput::ToString() const
        {
            ostringstream sstr;

            sstr << "transition {" << endl;
            sstr << "    " << InitState.GetName() << " -> " 
                 << FinalState.GetName() << endl;
            sstr << "    with output message \"" << MessageName 
                 << "\" of type " << MessageType->ToString() << endl;
            sstr << "    Guard: " << Guard->ToString() << endl;
            sstr << "    Updates:" << endl;
            for (auto const& Asgn : Updates) {
                sstr << Asgn->ToString() << endl;
            }
            return sstr.str();
        }

        LTSTransitionInternal::LTSTransitionInternal(EFSMBase* TheEFSM,
                                                     const LTSState& InitState,
                                                     const LTSState& FinalState,
                                                     const ExpT& Guard,
                                                     const vector<LTSAssignRef>& Updates,
                                                     const unordered_set<u32>& CompOfFairnessSets)
            : LTSTransitionBase(TheEFSM, InitState, FinalState, Guard, Updates),
              CompOfFairnessSets(CompOfFairnessSets)
        {
            // Nothing here
        }

        LTSTransitionInternal::~LTSTransitionInternal()
        {
            // Nothing here
        }

        const unordered_set<u32>& LTSTransitionInternal::GetCompOfFairnessSets() const
        {
            return CompOfFairnessSets;
        }

        string LTSTransitionInternal::ToString() const
        {
            ostringstream sstr;

            sstr << "transition {" << endl;
            sstr << "    " << InitState.GetName() << " -> " 
                 << FinalState.GetName() << endl;
            sstr << "    internal" << endl;
            sstr << "    Guard: " << Guard->ToString() << endl;
            sstr << "    Updates:" << endl;
            for (auto const& Asgn : Updates) {
                sstr << Asgn->ToString() << endl;
            }
            return sstr.str();
        }

        BuchiMonitorTransition::~BuchiMonitorTransition()
        {
            // Nothing here
        }

        string BuchiMonitorTransition::ToString() const
        {
            ostringstream sstr;
            sstr << "buchi transition {" << endl;
            sstr << "    " << InitState.GetName() << " -> " 
                 << FinalState.GetName() << endl;
            sstr << "    Guard: " << Guard->ToString() << endl;
            return sstr.str();
        }

    } /* end namespace */
} /* end namespace */

// 
// LTSTransitions.cpp ends here
