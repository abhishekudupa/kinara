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
            vector<MgrT::SubstMapT> ParamSubsts;

            bool StatesFrozen;

            void AssertStatesFrozen() const;
            void AssertStatesNotFrozen() const;
            void CheckState(const string& Name) const;

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
            u32 GetNumInstancesUnconstrained() const;
            virtual string ToString() const = 0;
        };


        class EFSMBase : public virtual AutomatonBase
        {
            friend class LabelledTS;

        protected:
            LTSFairnessType Fairness;
            bool VarsFrozen;
            bool EFSMFrozen;
            // State variable type
            ExprTypeRef StateVarType;
            // SubstMap to rebase expressions
            // to use the state var fields
            map<vector<ExpT>, MgrT::SubstMapT> RebaseSubstMaps;
            UIDGenerator FairnessUIDGenerator;

            // Transitions per instance
            map<vector<ExpT>, vector<LTSTransRef>> Transitions;

            // Fairness sets per instance
            map<string, map<vector<ExpT>, LTSFairSetRef>> Fairnesses;
            // mapping between internal fairness sets and user 
            // defined names for fairnesses
            map<string, set<string>> UserToInternalFairness;

            // Inputs and outputs per instance
            map<vector<ExpT>, set<ExprTypeRef>> Inputs;
            map<vector<ExpT>, set<ExprTypeRef>> Outputs;
            

            // helper methods

            void AssertVarsFrozen() const;
            void AssertVarsNotFrozen() const;
            void AssertEFSMFrozen() const;
            void AssertEFSMNotFrozen() const;
            
            void AssertInput(const vector<ExpT>& ParamInst, const ExprTypeRef& MessageType) const;
            void AssertOutput(const vector<ExpT>& ParamInst, const ExprTypeRef& MessageType) const;

            void CheckMsgType(const ExprTypeRef& Type) const;
            void CheckFairnessSets(const set<string>& FairnessSetNames) const;

            ExprTypeRef InstantiateMessageType(const vector<ExpT>& Params,
                                               const MgrT::SubstMapT& SubstMap,
                                               const ExprTypeRef& MsgType);

            void AddMsg(const ExprTypeRef& MsgType,
                        const vector<ExpT>& Params,
                        bool IsInput);

            void AddMsgs(const vector<ExpT>& NewParams,
                         const ExpT& Constraint,
                         const ExprTypeRef& MsgType,
                         const vector<ExpT>& Params,
                         bool IsInput);
            
            vector<LTSAssignRef> InstantiateUpdates(const MgrT::SubstMapT& ParamSubst,
                                                    const vector<LTSAssignRef>& Updates);
            vector<LTSAssignRef> RebaseUpdates(const vector<ExpT>& ParamInst,
                                               const vector<LTSAssignRef>& Updates);
            vector<LTSAssignRef> MsgTransformUpdates(const vector<LTSAssignRef>& Updates,
                                                     const string& MessageName, 
                                                     const ExprTypeRef& MessageType);
            vector<LTSAssignRef> SimplifyUpdates(const vector<LTSAssignRef>& Updates);

            // For internal use. e.g. in the case of channels
            void AddInputTransForInstance(u32 InstanceID, 
                                          const MgrT::SubstMapT& SubstMap,
                                          const string& InitialState,
                                          const string& FinalState, 
                                          const ExpT& Guard,
                                          const vector<LTSAssignRef>& Updates,
                                          const string& MessageName, 
                                          const ExprTypeRef& MessageType,
                                          const ExprTypeRef& ActMType);

            void AddOutputTransForInstance(u32 InstanceID, 
                                           const MgrT::SubstMapT& SubstMap,
                                           const string& InitState,
                                           const string& FinalState,
                                           const ExpT& Guard,
                                           const vector<LTSAssignRef>& Updates,
                                           const string& MessageName,
                                           const ExprTypeRef& MessageType,
                                           const ExprTypeRef& ActMType,
                                           const set<string>& AddToFairnessSets);

            void AddInternalTransForInstance(u32 InstanceID, 
                                             const MgrT::SubstMapT& SubstMap,
                                             const string& InitState,
                                             const string& FinalState,
                                             const ExpT& Guard,
                                             const vector<LTSAssignRef>& Updates,
                                             const set<string>& AddToFairnessSets);
                                          
                                          
            
        public:
            EFSMBase(LabelledTS* TheLTS, const string& Name,
                     const vector<ExpT>& Params, const ExpT& Constraint,
                     LTSFairnessType Fairness = LTSFairnessType::None);
            virtual ~EFSMBase();

            // Methods not intended to be overridden.
            // This helps preserve the structure of the EFSM
            set<ExprTypeRef> GetInputs() const;
            set<ExprTypeRef> GetInputsForInstance(u32 InstanceID) const;
            
            set<ExprTypeRef> GetOutputs() const;
            set<ExprTypeRef> GetOutputsForInstance(u32 InstanceID) const;

            vector<LTSTransRef> GetTransitionsOnMsg(const ExprTypeRef& MsgType) const;

            virtual void AddState(const string& StateName,
                                  bool Initial = false, bool Final = false, 
                                  bool Accepting = false, bool Error = false) override;

            virtual LTSFairnessType GetFairnessType() const;

            virtual void AddFairnessSet(const string& Name, FairSetFairnessType Fairness);
            virtual const map<vector<ExpT>, LTSFairSetRef>& GetFairnessSet(const string& 
                                                                           FairnessName) 
                const;
            virtual const LTSFairSetRef& GetFairnessForInst(const string& FairnessName,
                                                            const vector<ExpT>& InstParams) const;
            virtual const map<string, map<vector<ExpT>, LTSFairSetRef>>& GetAllFairnessSets() const;

            virtual void FreezeStates() override;
            virtual void FreezeVars();
            virtual void Freeze();

            virtual void AddInputMsg(const ExprTypeRef& MessageType,
                                     const vector<ExpT>& Params = vector<ExpT>());

            virtual void AddInputMsgs(const vector<ExpT>& NewParams, const ExpT& Constraint,
                                      const ExprTypeRef& MessageType, 
                                      const vector<ExpT>& MessageParams);

            virtual void AddOutputMsg(const ExprTypeRef& MessageType,
                                      const vector<ExpT>& Params = vector<ExpT>());

            virtual void AddOutputMsgs(const vector<ExpT>& NewParams, const ExpT& Constraint,
                                       const ExprTypeRef& MessageType,
                                       const vector<ExpT>& MEssageParams);

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
                                              LTSFairnessType FairnessKind,
                                              SplatFairnessType SplatFairness,
                                              const string& SplatFairnessName);

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
                                                LTSFairnessType FairnessKind = 
                                                LTSFairnessType::None,
                                                SplatFairnessType SplatFairness =
                                                SplatFairnessType::None,
                                                const string& SplatFairnessName = "");

            virtual string ToString() const override;

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
            DetEFSM(LabelledTS* TheLTS, const string& Name,
                    const vector<ExpT>& Params, const ExpT& Constraint,
                    LTSFairnessType Fairness = LTSFairnessType::None);

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
                                              LTSFairnessType FairnessKind,
                                              SplatFairnessType SplatFairness,
                                              const string& SplatFairnessName) override;

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
                                                LTSFairnessType FairnessKind,
                                                SplatFairnessType SplatFairness,
                                                const string& SplatFairnessName) override;
        };

        class ChannelEFSM : public EFSMBase
        {
        private:
            u32 Capacity;
            bool Lossy;
            bool Ordered;
            bool Duplicating;
            bool Blocking;

            ExprTypeRef ArrayType;
            ExprTypeRef ValType;
            ExprTypeRef IndexType;
            ExprTypeRef CountType;

            ExpT ArrayExp;
            ExpT IndexExp;
            ExpT OneExp;
            ExpT ZeroExp;
            ExpT MaxChanExp;
            ExpT LastMsgExp;
            ExpT CountExp;

            UIDGenerator MessageFairnessUIDGen;
            UIDGenerator LossDupFairnessUIDGen;

            inline void MakeInputTransition(u32 InstanceID,
                                            const MgrT::SubstMapT& SubstMap,
                                            const ExprTypeRef& MessageType,
                                            LossDupFairnessType LossDupFairness);

            inline void MakeOutputTransition(u32 InstanceID,
                                             const MgrT::SubstMapT& SubstMap,
                                             const ExprTypeRef& MessageType,
                                             LTSFairnessType MessageFairness,
                                             LossDupFairnessType LossDupFairness);

        public:
            ChannelEFSM(LabelledTS* TheLTS, const string& Name,
                        const vector<ExpT>& Params, const ExpT& Constraint,
                        u32 Capacity, bool Lossy = false, bool Ordered = true, 
                        bool Duplicating = false,
                        bool Blocking = false, LTSFairnessType 
                        Fairness = LTSFairnessType::None);
            
            virtual ~ChannelEFSM();
            
            virtual void FreezeStates() override;
            virtual void FreezeVars() override;
            virtual void AddFairnessSet(const string& Name, FairSetFairnessType Fairness) override;

            void AddMsg(const ExprTypeRef& MessageType,
                        const vector<ExpT>& Params = vector<ExpT>(),
                        LTSFairnessType MessageFairness = LTSFairnessType::None,
                        LossDupFairnessType LossDupFairness = LossDupFairnessType::None);

            void AddMsgs(const vector<ExpT> NewParams,
                         const ExpT& Constraint,    
                         const ExprTypeRef& MessageType,
                         const vector<ExpT>& MessageParams = vector<ExpT>(),
                         LTSFairnessType MessageFairness = LTSFairnessType::None,
                         LossDupFairnessType LossDupFairness = LossDupFairnessType::None);


            virtual void AddInputMsg(const ExprTypeRef& MessageType,
                                     const vector<ExpT>& Params = vector<ExpT>()) override;

            virtual void AddInputMsgs(const vector<ExpT>& NewParams, const ExpT& Constraint,
                                      const ExprTypeRef& MessageType,
                                      const vector<ExpT>& MessageParams) override;

            virtual void AddOutputMsg(const ExprTypeRef& MessageType,
                                      const vector<ExpT>& Params = vector<ExpT>()) override;

            virtual void AddOutputMsgs(const vector<ExpT>& NewParams, const ExpT& Constraint,
                                       const ExprTypeRef& MessageType,
                                       const vector<ExpT>& MessageParams) override;

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
                                              LTSFairnessType MessageFairness,
                                              SplatFairnessType SplatFairness,
                                              const string& SplatFairnessName) override;

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
                                                LTSFairnessType MessageFairness,
                                                SplatFairnessType SplatFairness,
                                                const string& SplatFairnessName) override;
        };

        class MonitorBase : public virtual AutomatonBase
        {
        public:
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

            virtual void AddFairnessSet(const string& Name, FairSetFairnessType Fairness) override;

            virtual void AddOutputMsg(const ExprTypeRef& MessageType,
                                      const vector<ExpT>& Params = vector<ExpT>()) override;

            virtual void AddOutputMsgs(const vector<ExpT>& NewParams, const ExpT& Constraint,
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
                                              LTSFairnessType MessageFairness,
                                              SplatFairnessType SplatFairness,
                                              const string& SplatFairnessName) override;

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
                                                LTSFairnessType MessageFairness,
                                                SplatFairnessType SplatFairness,
                                                const string& SplatFairnessName) override;

        };

        class BuchiMonitor : public MonitorBase
        {
        private:
            vector<BuchiTransRef> Transitions;

        public:
            BuchiMonitor(LabelledTS* TheLTS, const string& Name,
                         const vector<ExpT>& Params, const ExpT& Constraint);
            virtual ~BuchiMonitor();

            virtual void AddTransition(const string& InitState, 
                                       const string& FinalState,
                                       const ExpT& Guard);

            void AddFairnessByName(const string& AutomatonName, 
                                   const string& FairnessName,
                                   const vector<ExpT>& Params);

            void AddFairnessesByName(const string& NewParams,
                                     const ExpT& Constraint,
                                     const string& AutomatonName,
                                     const vector<ExpT> Params);

            virtual const vector<BuchiTransRef>& GetTransitions() const;
            virtual string ToString() const override;
        };

        namespace Detail {

            // Transforms all references to a particular
            // message type in an expression into a 
            // unified message type reference, renaming 
            // all field accesses appropriately
            class MsgTransformer : public VisitorBaseT
            {
            private:
                vector<ExpT> ExpStack;
                MgrT* Mgr;
                string MsgVarName;
                ExprTypeRef MsgRecType;
                ExprTypeRef UnifiedMType;

            public:
                MsgTransformer(MgrT* Mgr, const string& MsgVarName,
                               const ExprTypeRef& MsgRecType, 
                               const ExprTypeRef& UnifiedMType);
                virtual ~MsgTransformer();

                virtual void VisitVarExpression(const VarExpT* Exp) override;
                virtual void VisitBoundVarExpression(const BoundVarExpT* Exp) override;
                virtual void VisitConstExpression(const ConstExpT* Exp) override;
                virtual void VisitOpExpression(const OpExpT* Exp) override;
                virtual inline void VisitEQuantifiedExpression(const EQExpT* Exp) override;
                virtual inline void VisitAQuantifiedExpression(const AQExpT* Exp) override;
                
                static ExpT Do(MgrT* Mgr,
                               const ExpT& Exp, 
                               const string& MsgVarName,
                               const ExprTypeRef& MsgRecType,
                               const ExprTypeRef& UnifiedMType);
            };

        } /* end namespace Detail */

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_EFSM_HPP_ */

// 
// LTSEFSM.hpp ends here
