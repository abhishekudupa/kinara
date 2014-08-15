// LTSChannelEFSM.cpp --- 
// 
// Filename: LTSChannelEFSM.cpp
// Author: Abhishek Udupa
// Created: Fri Aug 15 12:06:46 2014 (-0400)
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

#include "LTSChannelEFSM.hpp"
#include "LTSUtils.hpp"
#include "LabelledTS.hpp"
#include "LTSState.hpp"
#include "LTSTransitions.hpp"
#include "LTSFairnessSet.hpp"

namespace ESMC {
    namespace LTS {

        ChannelEFSM::ChannelEFSM(LabelledTS* TheLTS, const string& Name,
                                 const vector<ExpT>& Params, const ExpT& Constraint,
                                 u32 Capacity, bool Lossy, bool Ordered, bool Duplicating,
                                 bool Blocking, LTSFairnessType Fairness)
              : AutomatonBase(TheLTS, Name, Params, Constraint), 
                EFSMBase(TheLTS, Name, Params, Constraint, Fairness),
                Capacity(Capacity), Lossy(Lossy), Ordered(Ordered),
                Duplicating(Duplicating), Blocking(Blocking)
        {
            if (Blocking && !Lossy) {
                throw ESMCError((string)"Non lossy channels cannot be declared blocking");
            }
            
            AddState("ChanInitState");
            if (Lossy) {
                AddState("LossDecideState");
            }

            EFSMBase::FreezeStates();
            auto Mgr = TheLTS->GetMgr();
            IndexType = Mgr->MakeType<ExprRangeType>(0, Capacity - 1);
            ValType = TheLTS->GetUnifiedMType();
            ArrayType = Mgr->MakeType<ExprArrayType>(IndexType, ValType);
            CountType = Mgr->MakeType<ExprRangeType>(0, Capacity);
            
            EFSMBase::AddVariable("MsgBuffer", ArrayType);
            EFSMBase::AddVariable("MsgCount", CountType);
            
            if (Lossy) {
                EFSMBase::AddVariable("LastMsg", ValType);
            }

            EFSMBase::FreezeVars();

            CountExp = Mgr->MakeVar("MsgCount", CountType);
            ArrayExp = Mgr->MakeVar("MsgBuffer", ArrayType);
            IndexExp = Mgr->MakeExpr(LTSOps::OpIndex, ArrayExp, CountExp);
            LastMsgExp = Mgr->MakeVar("LastMsg", ValType);
            MaxChanExp = Mgr->MakeVal(to_string(Capacity), CountType);
            OneExp = Mgr->MakeVal("1", CountType);
            ZeroExp = Mgr->MakeVal("0", CountType);
        }

        ChannelEFSM::~ChannelEFSM()
        {
            // Nothing here
        }

        void ChannelEFSM::FreezeStates()
        {
            throw ESMCError((string)"ChannelEFSM::FreezeStates() should not be called");
        }

        void ChannelEFSM::FreezeVars()
        {
            throw ESMCError((string)"ChannelEFSM::FreezeVars() should not be called");
        }

        void ChannelEFSM::AddFairnessSet(const string& Name, FairSetFairnessType Fairness)
        {
            throw ESMCError((string)"ChannelEFSM::AddFairnessSet() should not be called");
        }

        inline void ChannelEFSM::MakeInputTransition(u32 InstanceID,
                                                     const MgrT::SubstMapT& SubstMap,
                                                     const ExprTypeRef& MessageType,
                                                     LossDupFairnessType LossDupFairness)
        {
            auto Mgr = TheLTS->GetMgr();
            auto const& UMType = TheLTS->GetUnifiedMType();

            auto RecType = MessageType->As<ExprRecordType>();
            if (RecType == nullptr) {
                throw ESMCError((string)"Expected Record type as message type " + 
                                "in ChannelEFSM::MakeInputTransition()");
            }
            
            ExpT TargetExp = nullptr;
            if (Lossy) {
                TargetExp = LastMsgExp;
            } else {
                TargetExp = IndexExp;
            }

            auto FAType = Mgr->MakeType<ExprFieldAccessType>();
            vector<LTSAssignRef> Updates;
            vector<LTSAssignRef> NoCountUpdates;
            string InMsgName = "__inmsg__";
            auto InMsgVar = Mgr->MakeVar(InMsgName, UMType);
            auto TrueExp = Mgr->MakeTrue();
            
            auto Guard = Mgr->MakeExpr(LTSOps::OpLT, CountExp, MaxChanExp);
            NoCountUpdates.push_back(new LTSAssignSimple(TargetExp, InMsgVar));
            Updates = NoCountUpdates;
            Updates.push_back(new LTSAssignSimple(CountExp, Mgr->MakeExpr(LTSOps::OpADD,
                                                                          CountExp, OneExp)));
            if (!Lossy) {
                EFSMBase::AddInputTransForInstance(InstanceID,
                                                   SubstMap,
                                                   "ChanInitState", 
                                                   "ChanInitState", 
                                                   Guard, 
                                                   Updates, 
                                                   InMsgName, 
                                                   MessageType, 
                                                   MessageType);
            } else {
                vector<LTSAssignRef> Step2Updates;
                Step2Updates.push_back(new LTSAssignSimple(LastMsgExp, IndexExp));
                Step2Updates.push_back(new LTSAssignSimple(CountExp,
                                                           Mgr->MakeExpr(LTSOps::OpADD,
                                                                         CountExp, OneExp)));
                Step2Updates.push_back(new LTSAssignSimple(LastMsgExp, 
                                                           Mgr->MakeVal("clear", UMType)));
                vector<LTSAssignRef> LossUpdates;
                LossUpdates.push_back(new LTSAssignSimple(LastMsgExp, 
                                                          Mgr->MakeVal("clear", UMType)));

                ExpT Guard2 = nullptr;
                if (Blocking) {
                    EFSMBase::AddInputTransForInstance(InstanceID,
                                                       SubstMap,
                                                       "ChanInitState", 
                                                       "LossDecideState",
                                                       Guard, 
                                                       Updates, 
                                                       InMsgName, 
                                                       MessageType,
                                                       MessageType);
                    Guard2 = TrueExp;
                } else {
                    EFSMBase::AddInputTransForInstance(InstanceID,
                                                       SubstMap,
                                                       "ChanInitState", 
                                                       "LossDecideState",
                                                       TrueExp, 
                                                       Updates, 
                                                       InMsgName, 
                                                       MessageType,
                                                       MessageType);
                    Guard2 = Guard;
                }

                set<string> AddToFairnessSets;
                if (LossDupFairness == LossDupFairnessType::NotAlwaysLost || 
                    LossDupFairness == LossDupFairnessType::NotAlwaysLostOrDup) {
                    auto LDFairID = LossDupFairnessUIDGen.GetUID();
                    string FairName = "LossFairness_ " + to_string(LDFairID);
                    EFSMBase::AddFairnessSet(FairName, FairSetFairnessType::Strong);
                    AddToFairnessSets.insert(FairName);
                }

                // Lossy transition
                EFSMBase::AddInternalTransForInstance(InstanceID,
                                                      SubstMap,
                                                      "LossDecideState", 
                                                      "ChanInitState", 
                                                      TrueExp, 
                                                      LossUpdates, 
                                                      set<string>());
                // Non lossy
                EFSMBase::AddInternalTransForInstance(InstanceID,
                                                      SubstMap,
                                                      "LossDecideState", 
                                                      "ChanInitState",
                                                      Guard2, 
                                                      Step2Updates, 
                                                      AddToFairnessSets);
            }
        }

        void ChannelEFSM::MakeOutputTransition(u32 InstanceID,
                                               const MgrT::SubstMapT& SubstMap,
                                               const ExprTypeRef& MessageType, 
                                               LTSFairnessType MessageFairness, 
                                               ESMC::LTS::LossDupFairnessType LossDupFairness)
        {
            auto Mgr = TheLTS->GetMgr();
            auto PMessageType = TheLTS->GetPrimedType(MessageType);
            auto UMType = TheLTS->GetUnifiedMType()->As<ExprUnionType>();

            vector<ExpT> ChooseExps;
            if (!Ordered) {
                for (u32 i = 0; i < Capacity; ++i) {
                    ChooseExps.push_back(Mgr->MakeVal(to_string(i), IndexType));
                }
            } else {
                ChooseExps.push_back(ZeroExp);
            }

            set<string> AddToFairnessSets;
            bool AddFairnessToDup = false;
            if (!Duplicating) {
                if (MessageFairness != LTSFairnessType::None) {
                    string FairnessSetName = "MessageFairness_" + 
                        to_string(MessageFairnessUIDGen.GetUID());
                    EFSMBase::AddFairnessSet(FairnessSetName,
                                             MessageFairness == LTSFairnessType::Strong ? 
                                             FairSetFairnessType::Strong : 
                                             FairSetFairnessType::Weak);
                    AddToFairnessSets.insert(FairnessSetName);
                }
            } else {
                if (LossDupFairness == LossDupFairnessType::NotAlwaysDup ||
                    LossDupFairness == LossDupFairnessType::NotAlwaysLostOrDup) {
                    string FairnessSetName = "DupFairness_" + 
                        to_string(LossDupFairnessUIDGen.GetUID());
                    EFSMBase::AddFairnessSet(FairnessSetName,
                                             FairSetFairnessType::Strong);
                    AddToFairnessSets.insert(FairnessSetName);
                } else if (MessageFairness != LTSFairnessType::None) {
                    string FairnessSetName = "MessageFairness_" + 
                        to_string(MessageFairnessUIDGen.GetUID());
                    EFSMBase::AddFairnessSet(FairnessSetName,
                                             MessageFairness == LTSFairnessType::Strong ? 
                                             FairSetFairnessType::Strong : 
                                             FairSetFairnessType::Weak);
                    AddToFairnessSets.insert(FairnessSetName);
                    AddFairnessToDup = true;
                }
            }

            for (auto const& ChooseExp : ChooseExps) {
                auto ChooseIndexExp = Mgr->MakeExpr(LTSOps::OpIndex, ArrayExp, ChooseExp);
                auto TrueExp = Mgr->MakeTrue();
                auto FAType = Mgr->MakeType<ExprFieldAccessType>();
                auto FieldVar = Mgr->MakeVar(UMType->GetTypeIDFieldName(), FAType);
                auto FieldExp = Mgr->MakeExpr(LTSOps::OpField, ChooseIndexExp, FieldVar);
                
                auto NonEmptyExp = Mgr->MakeExpr(LTSOps::OpGT, CountExp, ZeroExp);
                auto TypeID = UMType->GetTypeIDForMemberType(MessageType);
                auto MatchExp = Mgr->MakeExpr(LTSOps::OpEQ, FieldExp, 
                                              Mgr->MakeVal(to_string(TypeID), 
                                                           UMType->GetTypeIDFieldType()));
                auto ChooseOkExp = Mgr->MakeExpr(LTSOps::OpLT, ChooseExp, CountExp);
                auto Guard = Mgr->MakeExpr(LTSOps::OpAND, NonEmptyExp, ChooseOkExp, MatchExp);
                string OutMsgName = "__outmsg__";
                auto OutMsgExp = Mgr->MakeVar(OutMsgName, ValType);
                auto CntSubExp = Mgr->MakeExpr(LTSOps::OpSUB, CountExp, OneExp);
                
                vector<LTSAssignRef> Updates;
                Updates.push_back(new LTSAssignSimple(OutMsgExp, ChooseIndexExp));
                vector<LTSAssignRef> MsgUpdates = Updates;

                // Clear the last message
                Updates.push_back(new LTSAssignSimple(Mgr->MakeExpr(LTSOps::OpIndex,
                                                                    ArrayExp, CntSubExp),
                                                      Mgr->MakeVal("clear", ValType)));

                Updates.push_back(new LTSAssignSimple(CountExp, CntSubExp));
                // Push all the elements past the chosen element back
                for (u32 i = 0; i < Capacity - 1; ++i) {
                    auto Cond = Mgr->MakeExpr(LTSOps::OpGE, Mgr->MakeVal(to_string(i),
                                                                         IndexType),
                                              ChooseExp);
                    auto IfBranch = Mgr->MakeExpr(LTSOps::OpIndex, 
                                                  ArrayExp, 
                                                  Mgr->MakeVal(to_string(i+1), 
                                                               IndexType));
                    auto ElseBranch = Mgr->MakeExpr(LTSOps::OpIndex, 
                                                    ArrayExp, 
                                                    Mgr->MakeVal(to_string(i), 
                                                                 IndexType));
                    auto ITEExp = Mgr->MakeExpr(LTSOps::OpITE, Cond, IfBranch, ElseBranch);
                    Updates.push_back(new LTSAssignSimple(ElseBranch, ITEExp));
                }

                EFSMBase::AddOutputTransForInstance(InstanceID,
                                                    SubstMap,
                                                    "ChanInitState", 
                                                    "ChanInitState", 
                                                    Guard, 
                                                    Updates, 
                                                    OutMsgName, 
                                                    PMessageType, 
                                                    PMessageType,
                                                    AddToFairnessSets);
                if (Duplicating) {
                    EFSMBase::AddOutputTransForInstance(InstanceID,
                                                        SubstMap,
                                                        "ChanInitState", 
                                                        "ChanInitState",
                                                        Guard,
                                                        MsgUpdates, 
                                                        OutMsgName,
                                                        PMessageType, 
                                                        PMessageType,
                                                        (AddFairnessToDup ? 
                                                         AddToFairnessSets : 
                                                         set<string>()));
                }
            }
        }

        void ChannelEFSM::AddMsg(const ExprTypeRef& MessageType,
                                 const vector<ExpT>& Params,
                                 LTSFairnessType MessageFairness,
                                 LossDupFairnessType LossDupFairness)
        {
            auto PMessageType = TheLTS->GetPrimedType(MessageType);
            auto Mgr = TheLTS->GetMgr();
            EFSMBase::AddInputMsg(MessageType, Params);
            EFSMBase::AddOutputMsg(PMessageType, Params);

            const u32 NumInsts = ParamInsts.size();
            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& ParamInst = ParamInsts[i];
                auto const& SubstMap = ParamSubsts[i];
                auto&& SubstParams = SubstAll(Params, SubstMap, Mgr);
                
                auto ActMType = InstantiateType(MessageType, SubstParams, Mgr);
                MakeInputTransition(i, SubstMap, ActMType, LossDupFairness);
                MakeOutputTransition(i, SubstMap, ActMType, MessageFairness, LossDupFairness);
            }
        }

        void ChannelEFSM::AddMsgs(const vector<ExpT> NewParams,
                                  const ExpT& Constraint,    
                                  const ExprTypeRef& MessageType,
                                  const vector<ExpT>& MessageParams,
                                  LTSFairnessType MessageFairness,
                                  LossDupFairnessType LossDupFairness)
        {
            // We need to instantiate these ourselves 
            // because we need more fine grained control
            auto Mgr = TheLTS->GetMgr();
            auto PMessageType = TheLTS->GetPrimedType(MessageType);

            SymTab.Push();
            CheckParams(NewParams, Constraint, SymTab, Mgr, true);
            CheckParams(MessageParams, SymTab);
            SymTab.Pop();

            EFSMBase::AddInputMsgs(NewParams, Constraint, MessageType, MessageParams);
            EFSMBase::AddOutputMsgs(NewParams, Constraint, PMessageType, MessageParams);

            const u32 NumNewParams = NewParams.size();
            const u32 NumInsts = ParamInsts.size();

            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                auto SubstConstraint = Mgr->Substitute(SubstMap, Constraint);
                auto const&& NewInsts = InstantiateParams(NewParams, SubstConstraint, Mgr);
                for (auto const& NewInst : NewInsts) {
                    auto LocalSubstMap = SubstMap;
                    for (u32 j = 0; j < NumNewParams; ++j) {
                        LocalSubstMap[NewParams[j]] = NewInst[j];
                    }
                    auto&& SubstParams = SubstAll(MessageParams, LocalSubstMap, Mgr);
                    auto MType = InstantiateType(MessageType, SubstParams, Mgr);
                    auto PMType = TheLTS->GetPrimedType(MType);

                    MakeInputTransition(i, LocalSubstMap, MType, LossDupFairness);
                    MakeOutputTransition(i, LocalSubstMap, MType, MessageFairness, 
                                         LossDupFairness);
                }
            }
        }

        void ChannelEFSM::AddInputMsg(const ExprTypeRef& MessageType,
                                      const vector<ExpT>& Params)
        {
            throw ESMCError((string)"ChannelEFSM::AddInputMsg() should not be called");
        }

        void ChannelEFSM::AddInputMsgs(const vector<ExpT>& NewParams, 
                                       const ExpT& Constraint,
                                       const ExprTypeRef& MessageType,
                                       const vector<ExpT>& MessageParams)
        {
            throw ESMCError((string)"ChannelEFSM::AddInputMsgs() should not be called");
        }

        void ChannelEFSM::AddOutputMsg(const ExprTypeRef& MessageType,
                                       const vector<ExpT>& Params)
        {
            throw ESMCError((string)"ChannelEFSM::AddOutputMsg() should not be called");
        }

        void ChannelEFSM::AddOutputMsgs(const vector<ExpT>& NewParams, const ExpT& Constraint,
                                        const ExprTypeRef& MessageType,
                                        const vector<ExpT>& MessageParams)
        {
            throw ESMCError((string)"ChannelEFSM::AddOutputMsgs() should not be called");
        }


        void ChannelEFSM::AddVariable(const string& VarName, const ExprTypeRef& VarType)
        {
            throw ESMCError((string)"ChannelEFSM::AddVariable() should not be called");
        }

        void ChannelEFSM::AddInputTransition(const string& InitState,
                                             const string& FinalState,
                                             const ExpT& Guard,
                                             const vector<LTSAssignRef>& Updates,
                                             const string& MessageName,
                                             const ExprTypeRef& MessageType,
                                             const vector<ExpT>& MessageParams)
        {
            throw ESMCError((string)"ChannelEFSM::AddInputTransition() should not be called");
        }

        void ChannelEFSM::AddInputTransitions(const vector<ExpT>& TransParams,
                                              const ExpT& Constraint,
                                              const string& InitState,
                                              const string& FinalState,
                                              const ExpT& Guard,
                                              const vector<LTSAssignRef>& Updates,
                                              const string& MessageName,
                                              const ExprTypeRef& MessageType,
                                              const vector<ExpT>& MessageParams)
        {
            throw ESMCError((string)"ChannelEFSM::AddInputTransitions() should not be called");
        }
            
        void ChannelEFSM::AddOutputTransition(const string& InitState,
                                              const string& FinalState,
                                              const ExpT& Guard,
                                              const vector<LTSAssignRef>& Updates,
                                              const string& MessageName,
                                              const ExprTypeRef& MessageType,
                                              const vector<ExpT>& MessageParams,
                                              const set<string>& AddToFairnessSets)
        {
            throw ESMCError((string)"ChannelEFSM::AddOutputTransition() should not be called");
        }

        void ChannelEFSM::AddOutputTransitions(const vector<ExpT>& TransParams,
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
                                               const string& SplatFairnessName)
        {
            throw ESMCError((string)"ChannelEFSM::AddOutputTransitions() should not be called");
        }

        
        void ChannelEFSM::AddInternalTransition(const string& InitState,
                                                const string& FinalState,
                                                const ExpT& Guard,
                                                const vector<LTSAssignRef>& Updates,
                                                const set<string>& AddToFairnessSets)
        {
            throw ESMCError((string)"ChannelEFSM::AddInternalTransition() should not be called");
        }

        void ChannelEFSM::AddInternalTransitions(const vector<ExpT>& TransParams,
                                                 const ExpT& Constraint,
                                                 const string& InitState,
                                                 const string& FinalState,
                                                 const ExpT& Guard,
                                                 const vector<LTSAssignRef>& Updates,
                                                 LTSFairnessType MessageFairness,
                                                 SplatFairnessType SplatFairness,
                                                 const string& SplatFairnessName)
        {
            throw ESMCError((string)"ChannelEFSM::AddInternalTransitions() should not be called");
        }


    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// LTSChannelEFSM.cpp ends here
