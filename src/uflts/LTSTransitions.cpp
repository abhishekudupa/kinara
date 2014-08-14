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
            return dynamic_cast<EFSMBase*>(Automaton);
        }

        const vector<LTSAssignRef>& LTSTransitionBase::GetUpdates() const
        {
            return Updates;
        }


        LTSTransitionIOBase::LTSTransitionIOBase(EFSMBase* TheEFSM,
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

        LTSTransitionIOBase::~LTSTransitionIOBase()
        {
            // Nothing here
        }

        const string& LTSTransitionIOBase::GetMessageName() const
        {
            return MessageName;
        }

        const ExprTypeRef& LTSTransitionIOBase::GetMessageType() const
        {
            return MessageType;
        }

        LTSTransitionInput::LTSTransitionInput(EFSMBase* TheEFSM,
                                               const LTSState& InitState,
                                               const LTSState& FinalState,
                                               const ExpT& Guard,
                                               const vector<LTSAssignRef>& Updates,
                                               const string& MessageName,
                                               const ExprTypeRef& MessageType)
            : LTSTransitionIOBase(TheEFSM, InitState, FinalState, Guard, Updates, 
                                  MessageName, MessageType)
        {
            // Nothing here
        }

        LTSTransitionInput::~LTSTransitionInput()
        {
            // Nothing here
        }

        string LTSTransitionInput::ToString(u32 Indent) const
        {
            ostringstream sstr;
            string IndentString(Indent, ' ');

            sstr << IndentString << "transition {" << endl;
            sstr << IndentString << "    " << InitState.GetName() << " -> " 
                 << FinalState.GetName() << endl;
            sstr << IndentString << "    on input message \"" << MessageName 
                 << "\" of type " << MessageType->ToString() << endl;
            sstr << IndentString << "    Guard: " << Guard->ToString() << endl;
            sstr << IndentString << "    Updates:" << endl;
            for (auto const& Asgn : Updates) {
                sstr << IndentString << "    " << Asgn->ToString() << endl;
            }
            sstr << IndentString << "}" << endl;
            return sstr.str();
        }

        LTSTransitionOutput::LTSTransitionOutput(EFSMBase* TheEFSM,
                                                 const LTSState& InitState,
                                                 const LTSState& FinalState,
                                                 const ExpT& Guard,
                                                 const vector<LTSAssignRef>& Updates,
                                                 const string& MessageName,
                                                 const ExprTypeRef& MessageType,
                                                 const set<string>& CompOfFairnessSets)
            : LTSTransitionIOBase(TheEFSM, InitState, FinalState, Guard, Updates,
                                  MessageName, MessageType),  
              CompOfFairnessSets(CompOfFairnessSets)
        {
            // Nothing here
        }

        LTSTransitionOutput::~LTSTransitionOutput()
        {
            // Nothing here
        }

        const set<string>& LTSTransitionOutput::GetCompOfFairnessSets() const
        {
            return CompOfFairnessSets;
        }

        string LTSTransitionOutput::ToString(u32 Indent) const
        {
            ostringstream sstr;
            string IndentString(Indent, ' ');

            sstr << IndentString << "transition {" << endl;
            sstr << IndentString << "    " << InitState.GetName() << " -> " 
                 << FinalState.GetName() << endl;
            sstr << IndentString << "    with output message \"" << MessageName 
                 << "\" of type " << MessageType->ToString() << endl;
            sstr << IndentString << "    Guard: " << Guard->ToString() << endl;
            sstr << IndentString << "    Updates:" << endl;
            for (auto const& Asgn : Updates) {
                sstr << IndentString << "    " << Asgn->ToString() << endl;
            }
            sstr << IndentString << "}" << endl;
            return sstr.str();
        }

        LTSTransitionInternal::LTSTransitionInternal(EFSMBase* TheEFSM,
                                                     const LTSState& InitState,
                                                     const LTSState& FinalState,
                                                     const ExpT& Guard,
                                                     const vector<LTSAssignRef>& Updates,
                                                     const set<string>& CompOfFairnessSets)
            : LTSTransitionBase(TheEFSM, InitState, FinalState, Guard, Updates),
              CompOfFairnessSets(CompOfFairnessSets)
        {
            // Nothing here
        }

        LTSTransitionInternal::~LTSTransitionInternal()
        {
            // Nothing here
        }

        const set<string>& LTSTransitionInternal::GetCompOfFairnessSets() const
        {
            return CompOfFairnessSets;
        }

        string LTSTransitionInternal::ToString(u32 Indent) const
        {
            ostringstream sstr;
            string IndentString(Indent, ' ');

            sstr << IndentString << "transition {" << endl;
            sstr << IndentString << "    " << InitState.GetName() << " -> " 
                 << FinalState.GetName() << endl;
            sstr << IndentString << "    internal" << endl;
            sstr << IndentString << "    Guard: " << Guard->ToString() << endl;
            sstr << IndentString << "    Updates:" << endl;
            for (auto const& Asgn : Updates) {
                sstr << IndentString << "    " << Asgn->ToString() << endl;
            }
            sstr << IndentString << "}" << endl;
            return sstr.str();
        }

        BuchiMonitorTransition::~BuchiMonitorTransition()
        {
            // Nothing here
        }

        string BuchiMonitorTransition::ToString(u32 Indent) const
        {
            ostringstream sstr;
            string IndentString(Indent, ' ');
            sstr << IndentString << "buchi transition {" << endl;
            sstr << IndentString << "    " << InitState.GetName() << " -> " 
                 << FinalState.GetName() << endl;
            sstr << IndentString << "    Guard: " << Guard->ToString() << endl;
            sstr << IndentString << "}" << endl;
            return sstr.str();
        }

        LTSGuardedCommand::LTSGuardedCommand(const ExpT& Guard,
                                             const vector<LTSAssignRef>& Updates)
            : Guard(Guard), Updates(Updates)
        {
            // Nothing here
        }

        LTSGuardedCommand::~LTSGuardedCommand()
        {
            // Nothing here
        }

        const ExpT& LTSGuardedCommand::GetGuard() const
        {
            return Guard;
        }

        const vector<LTSAssignRef>& LTSGuardedCommand::GetUpdates() const
        {
            return Updates;
        }

        string LTSGuardedCommand::ToString() const
        {
            ostringstream sstr;
            sstr << "guarded command {" << endl;
            sstr << "    " << Guard->ToString() << " -> " << endl;
            for (auto const& Update : Updates) {
                sstr << "        " << Update->ToString() << endl;
            }
            sstr << "}" << endl;
            return sstr.str();
        }

        LTSInitState::LTSInitState(const vector<ExpT>& Params,
                                   const ExpT& Constraint,
                                   const vector<LTSAssignRef>& Updates)
            : Params(Params), Constraint(Constraint), Updates(Updates)
        {
            for (auto const& Update : Updates) {
                if (Update->Is<LTSAssignParam>()) {
                    throw ESMCError((string)"Parametric updates not allowed in initial state");
                }
            }
        }

        LTSInitState::~LTSInitState()
        {
            // Nothing here
        }

        const vector<ExpT>& LTSInitState::GetParams() const
        {
            return Params;
        }

        const ExpT& LTSInitState::GetConstraint() const
        {
            return Constraint;
        }

        const vector<LTSAssignRef>& LTSInitState::GetUpdates() const
        {
            return Updates;
        }

    } /* end namespace */
} /* end namespace */

// 
// LTSTransitions.cpp ends here






