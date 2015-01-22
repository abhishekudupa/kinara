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
#include "LTSUtils.hpp"

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
                                             const vector<LTSAssignRef>& Updates,
                                             const LTSSymbTransRef& SymbolicTransition)
            : AutomatonTransitionBase(TheEFSM, ParamInst, InitState, Guard),
              Updates(Updates), SymbolicTransition(SymbolicTransition)
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

        const LTSSymbTransRef& LTSTransitionBase::GetSymbolicTransition() const
        {
            return SymbolicTransition;
        }

        bool LTSTransitionBase::IsTentative() const
        {
            return (SymbolicTransition != LTSSymbTransRef::NullPtr &&
                    SymbolicTransition->IsTentative());
        }

        LTSTransitionIOBase::LTSTransitionIOBase(EFSMBase* TheEFSM,
                                                 const vector<ExpT>& ParamInst,
                                                 const LTSState& InitState,
                                                 const ExpT& Guard,
                                                 const vector<LTSAssignRef>& Updates,
                                                 const string& MessageName,
                                                 const TypeRef& MessageType,
                                                 const LTSSymbTransRef& SymbolicTransition)
            : LTSTransitionBase(TheEFSM, ParamInst, InitState, Guard, Updates, SymbolicTransition),
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

        const TypeRef& LTSTransitionIOBase::GetMessageType() const
        {
            return MessageType;
        }

        LTSTransitionInput::LTSTransitionInput(EFSMBase* TheEFSM,
                                               const vector<ExpT>& ParamInst,
                                               const LTSState& InitState,
                                               const ExpT& Guard,
                                               const vector<LTSAssignRef>& Updates,
                                               const string& MessageName,
                                               const TypeRef& MessageType,
                                               const LTSSymbTransRef& SymbolicTransition)
            : LTSTransitionIOBase(TheEFSM, ParamInst, InitState, Guard, Updates,
                                  MessageName, MessageType, SymbolicTransition)
        {
            // Nothing here
        }

        LTSTransitionInput::~LTSTransitionInput()
        {
            // Nothing here
        }

        string LTSTransitionInput::ToString(u32 Verbosity) const
        {
            ostringstream sstr;

            sstr << "transition {" << endl;
            sstr << "    on state " << InitState.GetName() << endl;
            sstr << "    on input message \"" << MessageName
                 << "\" of type " << MessageType->ToString(Verbosity) << endl;
            sstr << "    Guard: " << Guard->ToString(Verbosity) << endl;
            sstr << "    Updates:" << endl;
            for (auto const& Asgn : Updates) {
                sstr << "    " << Asgn->ToString(Verbosity) << endl;
            }
            sstr << "}" << endl;
            return sstr.str();
        }

        LTSTransitionOutput::LTSTransitionOutput(EFSMBase* TheEFSM,
                                                 const vector<ExpT>& ParamInst,
                                                 const LTSState& InitState,
                                                 const ExpT& Guard,
                                                 const vector<LTSAssignRef>& Updates,
                                                 const string& MessageName,
                                                 const TypeRef& MessageType,
                                                 const set<LTSFairObjRef>& FairnessObjsSatisfied,
                                                 const LTSSymbTransRef& SymbolicTransition)
            : LTSTransitionIOBase(TheEFSM, ParamInst, InitState, Guard, Updates,
                                  MessageName, MessageType, SymbolicTransition),
              FairnessObjsSatisfied(FairnessObjsSatisfied)
        {
            // Nothing here
        }

        LTSTransitionOutput::~LTSTransitionOutput()
        {
            // Nothing here
        }

        const set<LTSFairObjRef>& LTSTransitionOutput::GetFairnessObjsSatisfied() const
        {
            return FairnessObjsSatisfied;
        }

        string LTSTransitionOutput::ToString(u32 Verbosity) const
        {
            ostringstream sstr;

            sstr << "transition {" << endl;
            sstr << "    on state " << InitState.GetName() << endl;
            sstr << "    with output message \"" << MessageName
                 << "\" of type " << MessageType->ToString(Verbosity) << endl;
            sstr << "    Guard: " << Guard->ToString(Verbosity) << endl;
            sstr << "    Updates:" << endl;
            for (auto const& Asgn : Updates) {
                sstr << "    " << Asgn->ToString(Verbosity) << endl;
            }

            sstr << "}" << endl;
            return sstr.str();
        }

        LTSTransitionInternal::LTSTransitionInternal(EFSMBase* TheEFSM,
                                                     const vector<ExpT>& ParamInst,
                                                     const LTSState& InitState,
                                                     const ExpT& Guard,
                                                     const vector<LTSAssignRef>& Updates,
                                                     const set<LTSFairObjRef>& FairnessObjsSatisfied,
                                                     const LTSSymbTransRef& SymbolicTransition)
            : LTSTransitionBase(TheEFSM, ParamInst, InitState, Guard, Updates, SymbolicTransition),
              FairnessObjsSatisfied(FairnessObjsSatisfied)
        {
            // Nothing here
        }

        LTSTransitionInternal::~LTSTransitionInternal()
        {
            // Nothing here
        }

        const set<LTSFairObjRef>& LTSTransitionInternal::GetFairnessObjsSatisfied() const
        {
            return FairnessObjsSatisfied;
        }

        string LTSTransitionInternal::ToString(u32 Verbosity) const
        {
            ostringstream sstr;

            sstr << "transition {" << endl;
            sstr << "    on state " << InitState.GetName() << endl;
            sstr << "    internal" << endl;
            sstr << "    Guard: " << Guard->ToString(Verbosity) << endl;
            sstr << "    Updates:" << endl;
            for (auto const& Asgn : Updates) {
                sstr << "    " << Asgn->ToString(Verbosity) << endl;
            }
            sstr << "}" << endl;
            return sstr.str();
        }

        LTSGuardedCommand::LTSGuardedCommand(MgrT* Mgr,
                                             const vector<ExpT>& GuardComps,
                                             const vector<LTSAssignRef>& Updates,
                                             const TypeRef& MsgType, i32 MsgTypeID,
                                             const vector<LTSTransRef>& ProductTrans)
            : Mgr(Mgr), GuardComps(GuardComps), Updates(Updates), MsgType(MsgType),
              MsgTypeID(MsgTypeID), ProductTrans(ProductTrans),
              Tentative(any_of(ProductTrans.begin(), ProductTrans.end(),
                               [&] (const LTSTransRef& Trans) -> bool
                               {
                                   return Trans->IsTentative();
                               })),
              FullyInterpreted(!Tentative)
        {
            // Make the guard
            Guard = MakeConjunction(GuardComps, Mgr);
            Guard = Mgr->SimplifyFP(Guard);

            auto LeaderOutput = ProductTrans[0]->As<LTSTransitionOutput>();
            if (LeaderOutput != nullptr) {
                auto const& Objs = LeaderOutput->GetFairnessObjsSatisfied();
                FairnessObjsSatisfied.insert(FairnessObjsSatisfied.end(),
                                             Objs.begin(), Objs.end());
            } else {
                auto LeaderInternal = ProductTrans[0]->As<LTSTransitionInternal>();
                if (LeaderInternal == nullptr) {
                    throw InternalError((string)"Strange kind of leader transition in " +
                                        __FUNCTION__ + ", at: " + __FILE__ + ":" +
                                        to_string(__LINE__));
                }
                auto const& Objs = LeaderInternal->GetFairnessObjsSatisfied();
                FairnessObjsSatisfied.insert(FairnessObjsSatisfied.end(),
                                             Objs.begin(), Objs.end());
            }


            // Set up the fixed interpretation
            vector<ExpT> FixedComps;
            for (auto const& Trans : ProductTrans) {
                auto EFSM = Trans->GetAutomaton()->SAs<EFSMBase>();
                auto const& StateType = EFSM->GetStateType();
                auto const& InitStateName = Trans->GetInitState().GetName();
                auto EQExp = Mgr->MakeExpr(LTSOps::OpEQ,
                                           Mgr->MakeVar("state", StateType),
                                           Mgr->MakeVal(InitStateName, StateType));
                auto const& RebaseSubstMap = EFSM->GetRebaseSubstMap(Trans->GetParamInst());
                FixedComps.push_back(Mgr->Substitute(RebaseSubstMap, EQExp));
            }

            FixedInterpretation = MakeConjunction(FixedComps, Mgr);
            FixedInterpretation = Mgr->SimplifyFP(FixedInterpretation);
        }

        MgrT* LTSGuardedCommand::GetMgr() const
        {
            return Mgr;
        }

        const vector<ExpT>& LTSGuardedCommand::GetGuardComps() const
        {
            return GuardComps;
        }

        bool LTSGuardedCommand::IsFullyInterpreted() const
        {
            return FullyInterpreted;
        }

        void LTSGuardedCommand::SetFullyInterpreted(bool NewValue) const
        {
            FullyInterpreted = NewValue;
        }

        const ExpT& LTSGuardedCommand::GetFixedInterpretation() const
        {
            return FixedInterpretation;
        }

        const vector<LTSAssignRef>& LTSGuardedCommand::GetLoweredUpdates() const
        {
            return LoweredUpdates;
        }

        void
        LTSGuardedCommand::SetLoweredUpdates(const vector<LTSAssignRef>& LoweredUpdates) const
        {
            this->LoweredUpdates = LoweredUpdates;
        }

        const ExpT& LTSGuardedCommand::GetLoweredGuard() const
        {
            return LoweredGuard;
        }

        void LTSGuardedCommand::SetLoweredGuard(const ExpT& LoweredGuard) const
        {
            this->LoweredGuard = LoweredGuard;
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

        const TypeRef& LTSGuardedCommand::GetMsgType() const
        {
            return MsgType;
        }

        i32 LTSGuardedCommand::GetMsgTypeID() const
        {
            return MsgTypeID;
        }

        const vector<LTSFairObjRef>& LTSGuardedCommand::GetFairnessObjsSatisfied() const
        {
            return FairnessObjsSatisfied;
        }

        const vector<LTSTransRef>& LTSGuardedCommand::GetProductTransition() const
        {
            return ProductTrans;
        }

        string LTSGuardedCommand::ToString(u32 Verbosity) const
        {
            ostringstream sstr;
            sstr << "guarded command {" << endl;
            if (MsgType != TypeRef::NullPtr) {
                sstr << "    label: " << MsgType->ToString(Verbosity) << endl;
            } else {
                sstr << "    label: none (internal transition)" << endl;
            }
            sstr << "    " << Guard->ToString(Verbosity) << " ->" << endl;
            for (auto const& Update : Updates) {
                sstr << "        " << Update->ToString(Verbosity) << endl;
            }

            if (LoweredGuard != ExpT::NullPtr &&
                LoweredUpdates.size() > 0 &&
                Verbosity > 0) {
                sstr << endl << "Lowered:" << endl;
                sstr << "    " << LoweredGuard->ToString(Verbosity) << " ->" << endl;
                for (auto const& Update : LoweredUpdates) {
                    sstr << "        " << Update->ToString(Verbosity) << endl;
                }
            }
            sstr << "}" << endl;
            return sstr.str();
        }

        bool LTSGuardedCommand::IsTentative() const
        {
            return Tentative;
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
                                                     const vector<LTSAssignRef>& Updates,
                                                     bool Tentative)
          : TransParams(TransParams), Params(Params), Constraint(Constraint),
            Automaton(Automaton), InitState(InitState), Guard(Guard), Updates(Updates),
            Tentative(Tentative)

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

        bool LTSSymbTransitionBase::IsTentative() const
        {
            return Tentative;
        }

        LTSSymbIOTransitionBase::LTSSymbIOTransitionBase(const vector<ExpT>& TransParams,
                                                         const vector<ExpT>& Params,
                                                         const ExpT& Constraint,
                                                         AutomatonBase* Automaton,
                                                         const LTSState& InitState,
                                                         const ExpT& Guard,
                                                         const vector<LTSAssignRef>& Updates,
                                                         const string& MessageName,
                                                         const TypeRef& MessageType,
                                                         const vector<ExpT>& MessageParams,
                                                         bool Tentative)
        : LTSSymbTransitionBase(TransParams, Params, Constraint, Automaton,
                                InitState, Guard, Updates, Tentative),
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

        const TypeRef& LTSSymbIOTransitionBase::GetMessageType() const
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

        string LTSSymbInputTransition::ToString(u32 Verbosity) const
        {
            ostringstream sstr;
            sstr << "Symbolic Input Transition {" << endl;
            sstr << "    on " << MessageName << " of " << MessageType->ToString(Verbosity);
            for (auto const& MParam : MessageParams) {
                sstr << "[" << MParam->ToString(Verbosity) << "]";
            }
            sstr << endl << "    " << "from " << InitState.GetName();
            sstr << endl << "    " << Guard->ToString(Verbosity) << " ->" << endl;
            for (auto const& Update : Updates) {
                sstr << "    " << Update->ToString(Verbosity) << endl;
            }
            sstr << "}" << endl;
            return sstr.str();
        }

        LTSSymbOutputTransition::~LTSSymbOutputTransition()
        {
            // Nothing here
        }

        string LTSSymbOutputTransition::ToString(u32 Verbosity) const
        {
            ostringstream sstr;
            sstr << "Symbolic Output Transition {" << endl;
            sstr << "    with " << MessageName << " of " << MessageType->ToString(Verbosity);
            for (auto const& MParam : MessageParams) {
                sstr << "[" << MParam->ToString() << "]";
            }
            sstr << endl << "    " << "from " << InitState.GetName();
            sstr << endl << "    " << Guard->ToString(Verbosity) << " ->" << endl;
            for (auto const& Update : Updates) {
                sstr << "    " << Update->ToString(Verbosity) << endl;
            }
            sstr << "}" << endl;
            return sstr.str();
        }

        LTSSymbInternalTransition::~LTSSymbInternalTransition()
        {
            // Nothing here
        }

        string LTSSymbInternalTransition::ToString(u32 Verbosity) const
        {
            ostringstream sstr;
            sstr << "Symbolic Internal Transition {" << endl;
            sstr << endl << "    " << "from " << InitState.GetName();
            sstr << endl << "    " << Guard->ToString(Verbosity) << " ->" << endl;
            for (auto const& Update : Updates) {
                sstr << "    " << Update->ToString(Verbosity) << endl;
            }
            sstr << "}" << endl;
            return sstr.str();
        }

        LTSInitStateGenerator::LTSInitStateGenerator(const vector<LTSAssignRef>& Updates)
            : Updates(Updates)
        {
            // Nothing here
        }

        LTSInitStateGenerator::~LTSInitStateGenerator()
        {
            // Nothing here
        }

        const vector<LTSAssignRef>& LTSInitStateGenerator::GetUpdates() const
        {
            return Updates;
        }

        const vector<LTSAssignRef>& LTSInitStateGenerator::GetLoweredUpdates() const
        {
            return LoweredUpdates;
        }

        void
        LTSInitStateGenerator::SetLoweredUpdates(const vector<LTSAssignRef>& LoweredUpdates) const
        {
            this->LoweredUpdates = LoweredUpdates;
        }

        string LTSInitStateGenerator::ToString(u32 Verbosity) const
        {
            ostringstream sstr;
            sstr << "initstate {" << endl;
            for (auto const& Update : Updates) {
                sstr << "    " << Update->ToString(Verbosity) << endl;
            }

            if (LoweredUpdates.size() > 0 && Verbosity > 0) {
                sstr << endl << "Lowered Updates:" << endl;
                for (auto const& Update : LoweredUpdates) {
                    sstr << "    " << Update->ToString(Verbosity) << endl;
                }
            }

            sstr << "}" << endl;
            return sstr.str();
        }

    } /* end namespace */
} /* end namespace */

//
// LTSTransitions.cpp ends here
