// LTSEFSM.hpp --- 
// 
// Filename: LTSEFSM.hpp
// Author: Abhishek Udupa
// Created: Fri Aug  8 13:43:28 2014 (-0400)
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

#if !defined ESMC_LTS_EFSM_HPP_
#define ESMC_LTS_EFSM_HPP_

#include "LTSTypes.hpp"
#include "LTSState.hpp"
#include "SymbolTable.hpp"

namespace ESMC {
    namespace LTS {
        
        enum class SplatFairnessType {
            None, Group, Individual
        };

        class AutomatonBase
        {
        protected:
            LabelledTS* TheLTS;
            string Name;
            vector<ExpT> Params;
            ExpT Constraint;
            SymbolTable SymTab;
            map<string, LTSState> States;
            ExprTypeRef StateType;
            vector<vector<ExpT>> ParamInsts;
            vector<vector<MgrT::SubstMapT>> ParamSubsts;

            
            bool StatesFrozen;

        public:
            AutomatonBase(LabelledTS* TheLTS, const string& Name,
                          const vector<ExpT>& Params, const ExpT& Constraint);
            virtual ~AutomatonBase();

            virtual void FreezeStates();
            virtual void AddState(const string& StateName,
                                  bool Initial = false, bool Final = false, 
                                  bool Accepting = false, bool Error = false);

            vector<LTSState> GetStates() const;
            const ExprTypeRef& GetStateType() const;
            const vector<vector<ExpT>>& GetParamInsts() const;
            const vector<MgrT::SubstMapT>& GetParamSubsts() const;
            u32 GetNumInstances() const;
        };

        class EFSMBase : public virtual AutomatonBase
        {
        protected:
            LTSFairnessType Fairness;
            bool EFSMFrozen;
            vector<LTSTransRef> Transitions;
            map<string, LTSFairSetRef> Fairnesses;
            set<ExprTypeRef> Inputs;
            set<ExprTypeRef> Outputs;

        public:
            EFSMBase(LabelledTS* TheLTS, const string& Name,
                     const vector<ExpT>& Params, const ExpT& Constraint,
                     LTSFairnessType Fairness = LTSFairnessType::None);
            virtual ~EFSMBase();

            virtual LTSFairnessType GetFairnessType() const;
            virtual void AddFairnessSet(const string& Name, LTSFairnessType Fairness);
            virtual const LTSFairSetRef& GetFairness(const string& FairnessName) const;
            virtual const map<string, LTSFairSetRef>& GetFairnesses() const;

            virtual void Freeze();

            virtual void AddInputMsg(const string& MessageName,
                                     const ExprTypeRef& MessageType,
                                     const vector<ExpT>& Params = vector<ExpT>());

            virtual void AddInputMsgs(const vector<ExpT>& NewParams, const ExpT& Constraint,
                                      const string& MessageName, 
                                      const ExprTypeRef& MessageType);

            virtual void AddOutputMsg(const string& MessageName,
                                      const ExprTypeRef& MessageType,
                                      const vector<ExpT>& Params = vector<ExpT>());

            virtual void AddOutputMsgs(const vector<ExpT>& NewParams, const ExpT& Constraint,
                                       const string& MessageName, 
                                       const ExprTypeRef& MessageType);

            virtual void AddVariable(const string& VarName, const ExprTypeRef& VarType);

            virtual void AddInputTransition(const string& InitState,
                                            const string& FinalState,
                                            const ExpT& Guard,
                                            const vector<LTSAssignRef>& Updates,
                                            const string& MessageName,
                                            const ExprTypeRef& MessageType,
                                            const vector<ExpT>& MessageParams);

            virtual void AddInputTransitions(const vector<ExpT>& TransParams,
                                             const ExpT& Constraint,
                                             const string& InitState,
                                             const string& FinalState,
                                             const ExpT& Guard,
                                             const vector<LTSAssignRef>& Updates,
                                             const string& MessageName,
                                             const ExprTypeRef& MessageType,
                                             const vector<ExpT>& MessageParams);

            virtual void AddOutputTransition(const string& InitState,
                                             const string& FinalState,
                                             const ExpT& Guard,
                                             const vector<LTSAssignRef>& Updates,
                                             const string& MessageName,
                                             const ExprTypeRef& MessageType,
                                             const vector<ExpT>& MessageParams,
                                             const set<string>& AddToFairnessSets = 
                                             set<string>());

            virtual void AddOutputTransitions(const vector<ExpT>& TransParams,
                                              const ExpT& Constraint,
                                              const string& InitState,
                                              const string& FinalState,
                                              const ExpT& Guard,
                                              const vector<LTSAssignRef>& Updates,
                                              const string& MessageName,
                                              const ExprTypeRef& MessageType,
                                              const vector<ExpT>& MessageParams,
                                              SplatFairnessType SplatFairness = 
                                              SplatFairnessType::None);

            virtual void AddInternalTransition(const string& InitState,
                                               const string& FinalState,
                                               const ExpT& Guard,
                                               const vector<LTSAssignRef>& Updates,
                                               const set<string>& AddToFairnessSets = 
                                               set<string>());

            virtual void AddInternalTransitions(const vector<ExpT>& TransParams,
                                                const ExpT& Constraint,
                                                const string& InitState,
                                                const string& FinalState,
                                                const ExpT& Guard,
                                                const vector<LTSAssignRef>& Updates,
                                                SplatFairnessType SplatFairness =
                                                SplatFairnessType::None);

        };

        class GeneralEFSM : public EFSMBase
        {
        public:
            GeneralEFSM(LabelledTS* TheLTS, const string& Name,
                        const vector<ExpT>& Params, const ExpT& Constraint,
                        LTSFairnessType Fairness = LTSFairnessType::None);
            virtual ~GeneralEFSM();
            // Nothing needs to be overridden here
        };

        class DetEFSM : public EFSMBase
        {
        public:
            using EFSMBase::EFSMBase;
            virtual ~DetEFSM();

            virtual void AddInputTransition(const string& InitState,
                                            const string& FinalState,
                                            const ExpT& Guard,
                                            const vector<LTSAssignRef>& Updates,
                                            const string& MessageName,
                                            const ExprTypeRef& MessageType,
                                            const vector<ExpT>& MessageParams) override;

            virtual void AddInputTransitions(const vector<ExpT>& TransParams,
                                             const ExpT& Constraint,
                                             const string& InitState,
                                             const string& FinalState,
                                             const ExpT& Guard,
                                             const vector<LTSAssignRef>& Updates,
                                             const string& MessageName,
                                             const ExprTypeRef& MessageType,
                                             const vector<ExpT>& MessageParams) override;

            virtual void AddOutputTransition(const string& InitState,
                                             const string& FinalState,
                                             const ExpT& Guard,
                                             const vector<LTSAssignRef>& Updates,
                                             const string& MessageName,
                                             const ExprTypeRef& MessageType,
                                             const vector<ExpT>& MessageParams,
                                             const set<string>& AddToFairnessSets = 
                                             set<string>()) override;

            virtual void AddOutputTransitions(const vector<ExpT>& TransParams,
                                              const ExpT& Constraint,
                                              const string& InitState,
                                              const string& FinalState,
                                              const ExpT& Guard,
                                              const vector<LTSAssignRef>& Updates,
                                              const string& MessageName,
                                              const ExprTypeRef& MessageType,
                                              const vector<ExpT>& MessageParams,
                                              SplatFairnessType SplatFairness = 
                                              SplatFairnessType::None) override;

            virtual void AddInternalTransition(const string& InitState,
                                               const string& FinalState,
                                               const ExpT& Guard,
                                               const vector<LTSAssignRef>& Updates,
                                               const set<string>& AddToFairnessSets = 
                                               set<string>()) override;

            virtual void AddInternalTransitions(const vector<ExpT>& TransParams,
                                                const ExpT& Constraint,
                                                const string& InitState,
                                                const string& FinalState,
                                                const ExpT& Guard,
                                                const vector<LTSAssignRef>& Updates,
                                                SplatFairnessType SplatFairness =
                                                SplatFairnessType::None) override;
        };

        enum class LossDupFairnessType {
            None, NotAlwaysLost, NotAlwaysDup, NotAlwaysLostOrDup
        };

        class ChannelEFSM : public EFSMBase
        {
        public:
            ChannelEFSM(LabelledTS* TheLTS, const string& Name,
                        const vector<ExpT>& Params, const ExpT& Constraint,
                        u32 Capacity, bool Lossy = false, bool Duplicating = false,
                        bool Blocking = false, LTSFairnessType 
                        Fairness = LTSFairnessType::None);
            
            virtual ~ChannelEFSM();
            
            virtual void FreezeStates() override;

            void AddMsg(const string& MessageName,
                        const ExprTypeRef& MessageType,
                        const vector<ExpT>& Params = vector<ExpT>(),
                        LTSFairnessType MessageFairness = LTSFairnessType::None,
                        LossDupFairnessType LossDupFairness = LossDupFairnessType::None);

            void AddMsgs(const vector<ExpT> NewParams,
                         const ExpT& Constraint,    
                         const string& MessageName,
                         const ExprTypeRef& MessageType,
                         const vector<ExpT>& Params = vector<ExpT>(),
                         LTSFairnessType MessageFairness = LTSFairnessType::None,
                         LossDupFairnessType LossDupFairness = LossDupFairnessType::None);


            virtual void AddInputMsg(const string& MessageName,
                                     const ExprTypeRef& MessageType,
                                     const vector<ExpT>& Params = vector<ExpT>()) override;

            virtual void AddInputMsgs(const vector<ExpT>& NewParams, const ExpT& Constraint,
                                      const string& MessageName, 
                                      const ExprTypeRef& MessageType) override;

            virtual void AddOutputMsg(const string& MessageName,
                                      const ExprTypeRef& MessageType,
                                      const vector<ExpT>& Params = vector<ExpT>()) override;

            virtual void AddOutputMsgs(const vector<ExpT>& NewParams, const ExpT& Constraint,
                                       const string& MessageName, 
                                       const ExprTypeRef& MessageType) override;

            virtual void AddVariable(const string& VarName, const ExprTypeRef& VarType) override;

            virtual void AddInputTransition(const string& InitState,
                                            const string& FinalState,
                                            const ExpT& Guard,
                                            const vector<LTSAssignRef>& Updates,
                                            const string& MessageName,
                                            const ExprTypeRef& MessageType,
                                            const vector<ExpT>& MessageParams) override;

            virtual void AddInputTransitions(const vector<ExpT>& TransParams,
                                             const ExpT& Constraint,
                                             const string& InitState,
                                             const string& FinalState,
                                             const ExpT& Guard,
                                             const vector<LTSAssignRef>& Updates,
                                             const string& MessageName,
                                             const ExprTypeRef& MessageType,
                                             const vector<ExpT>& MessageParams) override;

            virtual void AddOutputTransition(const string& InitState,
                                             const string& FinalState,
                                             const ExpT& Guard,
                                             const vector<LTSAssignRef>& Updates,
                                             const string& MessageName,
                                             const ExprTypeRef& MessageType,
                                             const vector<ExpT>& MessageParams,
                                             const set<string>& AddToFairnessSets = 
                                             set<string>()) override;

            virtual void AddOutputTransitions(const vector<ExpT>& TransParams,
                                              const ExpT& Constraint,
                                              const string& InitState,
                                              const string& FinalState,
                                              const ExpT& Guard,
                                              const vector<LTSAssignRef>& Updates,
                                              const string& MessageName,
                                              const ExprTypeRef& MessageType,
                                              const vector<ExpT>& MessageParams,
                                              SplatFairnessType SplatFairness = 
                                              SplatFairnessType::None) override;

            virtual void AddInternalTransition(const string& InitState,
                                               const string& FinalState,
                                               const ExpT& Guard,
                                               const vector<LTSAssignRef>& Updates,
                                               const set<string>& AddToFairnessSets = 
                                               set<string>()) override;

            virtual void AddInternalTransitions(const vector<ExpT>& TransParams,
                                                const ExpT& Constraint,
                                                const string& InitState,
                                                const string& FinalState,
                                                const ExpT& Guard,
                                                const vector<LTSAssignRef>& Updates,
                                                SplatFairnessType SplatFairness =
                                                SplatFairnessType::None) override;
        };

        class MonitorBase : public virtual AutomatonBase
        {
            MonitorBase(LabelledTS* TheLTS, const string& Name,
                        const vector<ExpT>& Params, const ExpT& Constraint);
            virtual ~MonitorBase();
        };

        class SafetyMonitor : public MonitorBase, public EFSMBase
        {
        public:
            SafetyMonitor(LabelledTS* TheLTS, const string& Name,
                          const vector<ExpT>& Params, const ExpT& Constraint);
            virtual ~SafetyMonitor();

            virtual LTSFairnessType GetFairnessType() const override;
            virtual void AddFairnessSet(const string& Name, LTSFairnessType Fairness) override;
            virtual const LTSFairSetRef& GetFairness(const string& FairnessName) const override;
            virtual const map<string, LTSFairSetRef>& GetFairnesses() const override;

            virtual void AddOutputMsg(const string& MessageName,
                                      const ExprTypeRef& MessageType,
                                      const vector<ExpT>& Params = vector<ExpT>()) override;

            virtual void AddOutputMsgs(const vector<ExpT>& NewParams, const ExpT& Constraint,
                                       const string& MessageName, 
                                       const ExprTypeRef& MessageType) override;

            virtual void AddVariable(const string& VarName, const ExprTypeRef& VarType) override;


            virtual void AddOutputTransition(const string& InitState,
                                             const string& FinalState,
                                             const ExpT& Guard,
                                             const vector<LTSAssignRef>& Updates,
                                             const string& MessageName,
                                             const ExprTypeRef& MessageType,
                                             const vector<ExpT>& MessageParams,
                                             const set<string>& AddToFairnessSets = 
                                             set<string>()) override;

            virtual void AddOutputTransitions(const vector<ExpT>& TransParams,
                                              const ExpT& Constraint,
                                              const string& InitState,
                                              const string& FinalState,
                                              const ExpT& Guard,
                                              const vector<LTSAssignRef>& Updates,
                                              const string& MessageName,
                                              const ExprTypeRef& MessageType,
                                              const vector<ExpT>& MessageParams,
                                              SplatFairnessType SplatFairness = 
                                              SplatFairnessType::None) override;

            virtual void AddInternalTransition(const string& InitState,
                                               const string& FinalState,
                                               const ExpT& Guard,
                                               const vector<LTSAssignRef>& Updates,
                                               const set<string>& AddToFairnessSets = 
                                               set<string>()) override;

            virtual void AddInternalTransitions(const vector<ExpT>& TransParams,
                                                const ExpT& Constraint,
                                                const string& InitState,
                                                const string& FinalState,
                                                const ExpT& Guard,
                                                const vector<LTSAssignRef>& Updates,
                                                SplatFairnessType SplatFairness =
                                                SplatFairnessType::None) override;
        };

        class BuchiMonitor : public MonitorBase
        {
        private:
            vector<BuchiTransRef> Transitions;
            vector<LTSFairSetRef> Fairnesses;

        public:
            BuchiMonitor(LabelledTS* TheLTS, const string& Name,
                         const vector<ExpT>& Params, const ExpT& Constraint);
            virtual ~BuchiMonitor();

            virtual void AddFairness(const vector<ExpT>& Params,
                                     const LTSFairSetRef& Fairness);

            virtual void AddTransition(const string& InitState, 
                                       const string& FinalState,
                                       const ExpT& Guard);

            virtual vector<BuchiTransRef>& GetTransitions() const;
        };

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_EFSM_HPP_ */

// 
// LTSEFSM.hpp ends here
