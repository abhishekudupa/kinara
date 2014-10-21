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
#include "LTSFairnessSet.hpp"

namespace ESMC {
    namespace LTS {

        AutomatonTransitionBase::AutomatonTransitionBase(AutomatonBase* Automaton,
                                                         const vector<ExpT>& ParamInst,
                                                         const LTSState& InitState,
                                                         const ExpT& Guard)
            : Automaton(Automaton), InitState(InitState), Guard(Guard), ParamInst(ParamInst)
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

        const ExpT& AutomatonTransitionBase::GetGuard() const
        {
            return Guard;
        }

        const vector<ExpT>& AutomatonTransitionBase::GetParamInst() const
        {
            return ParamInst;
        }

        LTSTransitionBase::LTSTransitionBase(EFSMBase* TheEFSM,
                                             const vector<ExpT>& ParamInst,
                                             const LTSState& InitState,
                                             const ExpT& Guard,
                                             const vector<LTSAssignRef>& Updates)
            : AutomatonTransitionBase(TheEFSM, ParamInst, InitState, Guard),
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
                                                 const vector<ExpT>& ParamInst,
                                                 const LTSState& InitState,
                                                 const ExpT& Guard,
                                                 const vector<LTSAssignRef>& Updates,
                                                 const string& MessageName,
                                                 const ExprTypeRef& MessageType)
            : LTSTransitionBase(TheEFSM, ParamInst, InitState, Guard, Updates),
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
                                               const vector<ExpT>& ParamInst,
                                               const LTSState& InitState,
                                               const ExpT& Guard,
                                               const vector<LTSAssignRef>& Updates,
                                               const string& MessageName,
                                               const ExprTypeRef& MessageType)
            : LTSTransitionIOBase(TheEFSM, ParamInst, InitState, Guard, Updates,
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
            sstr << IndentString << "    on state " << InitState.GetName() << endl;
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
                                                 const vector<ExpT>& ParamInst,
                                                 const LTSState& InitState,
                                                 const ExpT& Guard,
                                                 const vector<LTSAssignRef>& Updates,
                                                 const string& MessageName,
                                                 const ExprTypeRef& MessageType,
                                                 const set<string>& CompOfFairnessSets)
            : LTSTransitionIOBase(TheEFSM, ParamInst, InitState, Guard, Updates,
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
            sstr << IndentString << "    on state " << InitState.GetName() << endl;
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
                                                     const vector<ExpT>& ParamInst,
                                                     const LTSState& InitState,
                                                     const ExpT& Guard,
                                                     const vector<LTSAssignRef>& Updates,
                                                     const set<string>& CompOfFairnessSets)
            : LTSTransitionBase(TheEFSM, ParamInst, InitState, Guard, Updates),
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
            sstr << IndentString << "    on state " << InitState.GetName() << endl;
            sstr << IndentString << "    internal" << endl;
            sstr << IndentString << "    Guard: " << Guard->ToString() << endl;
            sstr << IndentString << "    Updates:" << endl;
            for (auto const& Asgn : Updates) {
                sstr << IndentString << "    " << Asgn->ToString() << endl;
            }
            sstr << IndentString << "}" << endl;
            return sstr.str();
        }

        LTSGuardedCommand::LTSGuardedCommand(const ExpT& Guard,
                                             const vector<LTSAssignRef>& Updates,
                                             const ExprTypeRef& MsgType, i32 MsgTypeID,
                                             const set<LTSFairObjRef>& Fairnesses)
            : Guard(Guard), Updates(Updates), MsgType(MsgType), MsgTypeID(MsgTypeID),
              FairnessObjs(Fairnesses.begin(), Fairnesses.end())
        {
            set<LTSFairSetRef> FairSets;
            for (auto const& FairObj : FairnessObjs) {
                FairSets.insert(FairObj->GetFairnessSet());
            }

            FairnessSets.insert(FairnessSets.begin(), FairSets.begin(), FairSets.end());
        }

        u32 LTSGuardedCommand::GetCmdID() const
        {
            return CmdID;
        }

        void LTSGuardedCommand::SetCmdID(u32 CmdID) const
        {
            this->CmdID = CmdID;
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

        const ExprTypeRef& LTSGuardedCommand::GetMsgType() const
        {
            return MsgType;
        }

        i32 LTSGuardedCommand::GetMsgTypeID() const
        {
            return MsgTypeID;
        }

        const vector<LTSFairObjRef>& LTSGuardedCommand::GetFairnessObjs() const
        {
            return FairnessObjs;
        }

        const vector<LTSFairSetRef>& LTSGuardedCommand::GetFairnessSets() const
        {
            return FairnessSets;
        }

        string LTSGuardedCommand::ToString() const
        {
            ostringstream sstr;
            sstr << "guarded command {" << endl;
            if (MsgType != ExprTypeRef::NullPtr) {
                sstr << "    label: " << MsgType->As<ExprRecordType>()->GetName() << endl;
            } else {
                sstr << "    label: none (internal transition)" << endl;
            }
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
            // Nothing here
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


        // Symbolic transitions implementation
        LTSSymbTransitionBase::LTSSymbTransitionBase(const vector<ExpT>& TransParams,
                                                     const vector<ExpT>& Params,
                                                     const ExpT& Constraint,
                                                     AutomatonBase* Automaton,
                                                     const LTSState& InitState,
                                                     const ExpT& Guard,
                                                     const vector<LTSAssignRef>& Updates)
          : TransParams(TransParams), Params(Params), Constraint(Constraint), 
            Automaton(Automaton), InitState(InitState), Guard(Guard), Updates(Updates)
        {
            // Nothing here
        }

        LTSSymbTransitionBase::~LTSSymbTransitionBase()
        {
            // Nothing here
        }

        const vector<ExpT>& LTSSymbTransitionBase::GetTransParams() const
        {
            return TransParams;
        }

        const vector<ExpT>& LTSSymbTransitionBase::GetParams() const
        {
            return Params;
        }

        const ExpT& LTSSymbTransitionBase::GetConstraint() const
        {
            return Constraint;
        }

        AutomatonBase* LTSSymbTransitionBase::GetAutomaton() const
        {
            return Automaton;
        }

        const LTSState& LTSSymbTransitionBase::GetInitState() const
        {
            return InitState;
        }

        const ExpT& LTSSymbTransitionBase::GetGuard() const
        {
            return Guard;
        }

        const vector<LTSAssignRef>& LTSSymbTransitionBase::GetUpdates() const
        {
            return Updates;
        }

        LTSSymbIOTransitionBase::LTSSymbIOTransitionBase(const vector<ExpT>& TransParams,
                                                         const vector<ExpT>& Params,
                                                         const ExpT& Constraint,
                                                         AutomatonBase* Automaton,
                                                         const LTSState& InitState,
                                                         const ExpT& Guard,
                                                         const vector<LTSAssignRef>& Updates,
                                                         const string& MessageName,
                                                         const ExprTypeRef& MessageType,
                                                         const vector<ExpT>& MessageParams)
        : LTSSymbTransitionBase(TransParams, Params, Constraint, Automaton, 
                                InitState, Guard, Updates),
          MessageName(MessageName), MessageType(MessageType), MessageParams(MessageParams)
        {
            // Nothing here
        }

        LTSSymbIOTransitionBase::~LTSSymbIOTransitionBase()
        {
            // Nothing here
        }

        const string& LTSSymbIOTransitionBase::GetMessageName() const
        {
            return MessageName;
        }

        const ExprTypeRef& LTSSymbIOTransitionBase::GetMessageType() const
        {
            return MessageType;
        }

        const vector<ExpT>& LTSSymbIOTransitionBase::GetMessageParams() const
        {
            return MessageParams;
        }

        LTSSymbInputTransition::~LTSSymbInputTransition()
        {
            // Nothing here
        }
        
        string LTSSymbInputTransition::ToString() const
        {
            ostringstream sstr;
            sstr << "Symbolic Input Transition {" << endl;
            sstr << "    on " << MessageName << " of " << MessageType->ToString();
            for (auto const& MParam : MessageParams) {
                sstr << "[" << MParam->ToString() << "]";
            }
            sstr << endl << "    " << "from " << InitState.GetName();
            sstr << endl << "    " << Guard->ToString() << " -> " << endl;
            for (auto const& Update : Updates) {
                sstr << "    " << Update->ToString() << endl;
            }
            sstr << "}" << endl;
            return sstr.str();
        }

        LTSSymbOutputTransition::~LTSSymbOutputTransition()
        {
            // Nothing here
        }

        string LTSSymbOutputTransition::ToString() const
        {
            ostringstream sstr;
            sstr << "Symbolic Output Transition {" << endl;
            sstr << "    with " << MessageName << " of " << MessageType->ToString();
            for (auto const& MParam : MessageParams) {
                sstr << "[" << MParam->ToString() << "]";
            }
            sstr << endl << "    " << "from " << InitState.GetName();
            sstr << endl << "    " << Guard->ToString() << " -> " << endl;
            for (auto const& Update : Updates) {
                sstr << "    " << Update->ToString() << endl;
            }
            sstr << "}" << endl;
            return sstr.str();
        }

        LTSSymbInternalTransition::~LTSSymbInternalTransition()
        {
            // Nothing here
        }

        string LTSSymbInternalTransition::ToString() const
        {
            ostringstream sstr;
            sstr << "Symbolic Internal Transition {" << endl;
            sstr << endl << "    " << "from " << InitState.GetName();
            sstr << endl << "    " << Guard->ToString() << " -> " << endl;
            for (auto const& Update : Updates) {
                sstr << "    " << Update->ToString() << endl;
            }
            sstr << "}" << endl;
            return sstr.str();
        }

    } /* end namespace */
} /* end namespace */

// 
// LTSTransitions.cpp ends here
