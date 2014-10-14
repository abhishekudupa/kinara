// LTSTransitions.hpp --- 
// 
// Filename: LTSTransitions.hpp
// Author: Abhishek Udupa
// Created: Fri Aug  8 14:01:14 2014 (-0400)
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

#if !defined ESMC_LTS_TRANSITIONS_HPP_
#define ESMC_LTS_TRANSITIONS_HPP_

#include "../containers/RefCountable.hpp"

#include "LTSTypes.hpp"
#include "LTSState.hpp"


namespace ESMC {
    namespace LTS {

        class AutomatonTransitionBase : public RefCountable
        {
        protected:
            AutomatonBase* Automaton;
            LTSState InitState;
            LTSState FinalState;
            ExpT Guard;
            // The parameters with which I was 
            // instantiated
            vector<ExpT> ParamInst;

        public:
            AutomatonTransitionBase(AutomatonBase* Automaton,
                                    const vector<ExpT>& ParamInst,
                                    const LTSState& InitState,
                                    const LTSState& FinalState,
                                    const ExpT& Guard);
            virtual ~AutomatonTransitionBase();

            AutomatonBase* GetAutomaton() const;
            const LTSState& GetInitState() const;
            const LTSState& GetFinalState() const;
            const ExpT& GetGuard() const;
            const vector<ExpT>& GetParamInst() const;

            virtual string ToString(u32 Indent = 0) const = 0;
            
            template <typename T>
            T* As()
            {
                return dynamic_cast<T*>(this);
            }

            template <typename T>
            const T* As() const
            {
                return dynamic_cast<const T*>(this);
            }

            template <typename T>
            T* SAs()
            {
                return static_cast<T*>(this);
            }

            template <typename T>
            const T* SAs() const
            {
                return static_cast<const T*>(this);
            }

            template <typename T>
            bool Is() const
            {
                return (dynamic_cast<const T*>(this) != nullptr);
            }
        };

        class LTSTransitionBase : public AutomatonTransitionBase
        {
        protected:
            // The EFSM that this transition is a part of 
            vector<LTSAssignRef> Updates;
            
        public:
            LTSTransitionBase(EFSMBase* TheEFSM,
                              const vector<ExpT>& ParamInst,
                              const LTSState& InitState,
                              const LTSState& FinalState,
                              const ExpT& Guard,
                              const vector<LTSAssignRef>& Updates);
            virtual ~LTSTransitionBase();

            EFSMBase* GetEFSM() const;
            const vector<LTSAssignRef>& GetUpdates() const;
        };

        class LTSTransitionIOBase : public LTSTransitionBase
        {
        protected:
            string MessageName;
            ExprTypeRef MessageType;

        public:
            LTSTransitionIOBase(EFSMBase* TheEFSM,
                                const vector<ExpT>& ParamInst,
                                const LTSState& InitState,
                                const LTSState& FinalState,
                                const ExpT& Guard,
                                const vector<LTSAssignRef>& Updates,
                                const string& MessageName,
                                const ExprTypeRef& MessageType);
            virtual ~LTSTransitionIOBase();

            const string& GetMessageName() const;
            const ExprTypeRef& GetMessageType() const;
        };

        class LTSTransitionInput : public LTSTransitionIOBase
        {
        public:
            LTSTransitionInput(EFSMBase* TheEFSM,
                               const vector<ExpT>& ParamInst,
                               const LTSState& InitState,
                               const LTSState& FinalState,
                               const ExpT& Guard,
                               const vector<LTSAssignRef>& Updates,
                               const string& MessageName,
                               const ExprTypeRef& MessageType);
            virtual ~LTSTransitionInput();

            virtual string ToString(u32 Indent = 0) const override;
        };

        class LTSTransitionOutput : public LTSTransitionIOBase
        {
        private:
            set<string> CompOfFairnessSets;

        public:
            LTSTransitionOutput(EFSMBase* TheEFSM,
                                const vector<ExpT>& ParamInst,
                                const LTSState& InitState,
                                const LTSState& FinalState,
                                const ExpT& Guard,
                                const vector<LTSAssignRef>& Updates,
                                const string& MessageName,
                                const ExprTypeRef& MessageType,
                                const set<string>& CompOfFairnessSets);
            virtual ~LTSTransitionOutput();

            const set<string>& GetCompOfFairnessSets() const;

            virtual string ToString(u32 Indent = 0) const override;
        };

        class LTSTransitionInternal : public LTSTransitionBase
        {
        private:
            set<string> CompOfFairnessSets;

        public:
            LTSTransitionInternal(EFSMBase* TheEFSM,
                                  const vector<ExpT>& ParamInst,
                                  const LTSState& InitState,
                                  const LTSState& FinalState,
                                  const ExpT& Guard,
                                  const vector<LTSAssignRef>& Updates,
                                  const set<string>& CompOfFairnessSets);
            virtual ~LTSTransitionInternal();

            const set<string>& GetCompOfFairnessSets() const;
            virtual string ToString(u32 Indent = 0) const override;
        };

        class LTSGuardedCommand : public RefCountable
        {
        private:
            ExpT Guard;
            vector<LTSAssignRef> Updates;
            ExprTypeRef MsgType;
            i32 MsgTypeID;
            vector<LTSFairObjRef> FairnessObjs;
            vector<LTSFairSetRef> FairnessSets;
            vector<LTSTransRef> ProductTrans;
            mutable u32 CmdID;

        public:
            LTSGuardedCommand(const ExpT& Guard,
                              const vector<LTSAssignRef>& Updates,
                              const ExprTypeRef& MsgType, i32 MsgTypeID,
                              const set<LTSFairObjRef>& Fairnesses,
                              const vector<LTSTransRef>& ProductTrans);
            virtual ~LTSGuardedCommand();

            const ExpT& GetGuard() const;
            const vector<LTSAssignRef>& GetUpdates() const;
            const ExprTypeRef& GetMsgType() const;
            i32 GetMsgTypeID() const;
            const vector<LTSFairObjRef>& GetFairnessObjs() const;
            const vector<LTSFairSetRef>& GetFairnessSets() const;
            const vector<LTSTransRef>& GetProductTransition() const;
            u32 GetCmdID() const;
            void SetCmdID(u32 CmdID) const;
            string ToString() const;
        };

        class LTSInitState : public RefCountable
        {
        private:
            vector<ExpT> Params;
            ExpT Constraint;
            vector<LTSAssignRef> Updates;

        public:
            LTSInitState(const vector<ExpT>& Params,
                         const ExpT& Constraint,
                         const vector<LTSAssignRef>& Updates);
            virtual ~LTSInitState();
            
            const vector<ExpT>& GetParams() const;
            const ExpT& GetConstraint() const;
            const vector<LTSAssignRef>& GetUpdates() const;
        };

        class LTSSymbTransitionBase : public RefCountable
        {
        protected:
            vector<ExpT> Params;
            ExpT Constraint;
            AutomatonBase* Automaton;
            LTSState InitState;
            LTSState FinalState;
            ExpT Guard;
            vector<LTSAssignRef> Updates;
            
        public:
            LTSSymbTransitionBase(const vector<ExpT>& Params,
                                  const ExpT& Constraint,
                                  AutomatonBase* Automaton,
                                  const LTSState& InitState,
                                  const LTSState& FinalState,
                                  const ExpT& Guard,
                                  const vector<LTSAssignRef>& Updates);
            virtual ~LTSSymbTransitionBase();

            const vector<ExpT>& GetParams() const;
            const ExpT& GetConstraint() const;
            AutomatonBase* GetAutomaton() const;
            const LTSState& GetInitState() const;
            const LTSState& GetFinalState() const;
            const ExpT& GetGuard() const;
            const vector<LTSAssignRef>& GetUpdates() const;

            virtual string ToString() const = 0;

            template <typename T>
            inline T* As()
            {
                return dynamic_cast<T*>(this);
            }
            
            template <typename T>
            inline const T* As() const
            {
                return dynamic_cast<const T*>(this);
            }

            template <typename T>
            inline T* SAs()
            {
                return static_cast<T*>(this);
            }
            
            template <typename T>
            inline const T* SAs() const
            {
                return static_cast<const T*>(this);
            }

            template <typename T>
            inline bool Is() const
            {
                return (dynamic_cast<const T*>(this) != nullptr);
            }
        };

        class LTSSymbIOTransitionBase : public LTSSymbTransitionBase
        {
        protected:
            string MessageName;
            ExprTypeRef MessageType;
            vector<ExpT> MessageParams;

        public:
            LTSSymbIOTransitionBase(const vector<ExpT>& Params,
                                    const ExpT& Constraint,
                                    AutomatonBase* Automaton,
                                    const LTSState& InitState,
                                    const LTSState& FinalState,
                                    const ExpT& Guard,
                                    const vector<LTSAssignRef>& Updates,
                                    const string& MessageName,
                                    const ExprTypeRef& MessageType,
                                    const vector<ExpT>& MessageParams);
            virtual ~LTSSymbIOTransitionBase();

            const string& GetMessageName() const;
            const ExprTypeRef& GetMessageType() const;
            const vector<ExpT>& GetMessageParams() const;
        };

        class LTSSymbInputTransition : public LTSSymbIOTransitionBase
        {
        public:
            using LTSSymbIOTransitionBase::LTSSymbIOTransitionBase;
            virtual ~LTSSymbInputTransition();

            virtual string ToString() const override;
        };

        class LTSSymbOutputTransition : public LTSSymbIOTransitionBase
        {
        public:
            using LTSSymbIOTransitionBase::LTSSymbIOTransitionBase;
            virtual ~LTSSymbOutputTransition();

            virtual string ToString() const override;
        };

        class LTSSymbInternalTransition : public LTSSymbTransitionBase
        {
            using LTSSymbTransitionBase::LTSSymbTransitionBase;
            virtual ~LTSSymbInternalTransition();

            virtual string ToString() const override;
        };

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_TRANSITIONS_HPP_ */

// 
// LTSTransitions.hpp ends here
