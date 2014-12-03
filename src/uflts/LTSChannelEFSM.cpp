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
              : EFSMBase(TheLTS, Name, Params, Constraint, Fairness),
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
            IndexType = Mgr->MakeType<RangeType>(0, Capacity - 1);
            ValType = TheLTS->GetUnifiedMType();
            ArrType = Mgr->MakeType<ArrayType>(IndexType, ValType);
            CountType = Mgr->MakeType<RangeType>(0, Capacity);

            EFSMBase::AddVariable("MsgBuffer", ArrType);
            if (Ordered) {
                EFSMBase::AddVariable("MsgCount", CountType);
            }

            if (Lossy) {
                EFSMBase::AddVariable("LastMsg", ValType);
            }

            EFSMBase::FreezeVars();

            CountExp = Mgr->MakeVar("MsgCount", CountType);
            ArrayExp = Mgr->MakeVar("MsgBuffer", ArrType);
            IndexExp = Mgr->MakeExpr(LTSOps::OpIndex, ArrayExp, CountExp);
            LastMsgExp = Mgr->MakeVar("LastMsg", ValType);
            MaxChanExp = Mgr->MakeVal(to_string(Capacity), CountType);
            OneExp = Mgr->MakeVal("1", CountType);
            ZeroExp = Mgr->MakeVal("0", CountType);

            // Make the initial states as well
            vector<LTSAssignRef> InitUpdates;
            if (Ordered) {
                InitUpdates.push_back(new LTSAssignSimple(CountExp, Mgr->MakeVal("0", CountType)));
            }
            InitUpdates.push_back(new LTSAssignSimple(ArrayExp, Mgr->MakeVal("clear", ArrType)));
            if (Lossy) {
                InitUpdates.push_back(new LTSAssignSimple(LastMsgExp,
                                                          Mgr->MakeVal("clear", ValType)));
            }
            InitUpdates.push_back(new LTSAssignSimple(Mgr->MakeVar("state", StateType),
                                                      Mgr->MakeVal("ChanInitState", StateType)));
            for (auto const& ParamInst : ParamInsts) {
                auto&& RebasedUpdates = RebaseUpdates(ParamInst, InitUpdates);
                InitStateUpdates.insert(InitStateUpdates.end(), RebasedUpdates.begin(),
                                        RebasedUpdates.end());
            }
        }

        ChannelEFSM::~ChannelEFSM()
        {
            // Nothing here
        }

        u32 ChannelEFSM::GetCapacity() const
        {
            return Capacity;
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
                                                     const TypeRef& MessageType,
                                                     const set<string>& InputFairnessSets)
        {
            auto Mgr = TheLTS->GetMgr();
            auto const& UMType = TheLTS->GetUnifiedMType()->As<UnionType>();

            auto RecType = MessageType->As<RecordType>();
            if (RecType == nullptr) {
                throw ESMCError((string)"Expected Record type as message type " +
                                "in ChannelEFSM::MakeInputTransition()");
            }

            vector<ExpT> ChooseExps;
            if (!Ordered) {
                for (u32 i = 0; i < Capacity; ++i) {
                    ChooseExps.push_back(Mgr->MakeVal(to_string(i), IndexType));
                }
            } else {
                ChooseExps.push_back(Mgr->MakeExpr(LTSOps::OpSUB, CountExp, OneExp));
            }

            bool First = true;
            for (auto const& ChooseExp : ChooseExps) {
                ExpT TargetExp = nullptr;
                if (Lossy) {
                    TargetExp = LastMsgExp;
                } else {
                    if (!Ordered) {
                        TargetExp = Mgr->MakeExpr(LTSOps::OpIndex, ArrayExp, ChooseExp);
                    } else {
                        TargetExp = Mgr->MakeExpr(LTSOps::OpIndex, ArrayExp, CountExp);
                    }
                }

                auto FAType = Mgr->MakeType<FieldAccessType>();
                vector<LTSAssignRef> Updates;
                vector<LTSAssignRef> NoCountUpdates;
                string InMsgName = "__inmsg__";
                auto InMsgVar = Mgr->MakeVar(InMsgName, UMType);
                auto TrueExp = Mgr->MakeTrue();

                ExpT Guard = ExpT::NullPtr;

                if (Ordered) {
                    Guard = Mgr->MakeExpr(LTSOps::OpLT, CountExp, MaxChanExp);
                } else {
                    auto ChooseIndexExp = Mgr->MakeExpr(LTSOps::OpIndex,
                                                        ArrayExp, ChooseExp);
                    auto FieldVar = Mgr->MakeVar(UMType->GetTypeIDFieldName(), FAType);
                    auto FieldExp = Mgr->MakeExpr(LTSOps::OpField, ChooseIndexExp, FieldVar);
                    Guard = Mgr->MakeExpr(LTSOps::OpEQ, FieldExp,
                                          Mgr->MakeVal("0", UMType->GetTypeIDFieldType()));
                }

                NoCountUpdates.push_back(new LTSAssignSimple(TargetExp, InMsgVar));
                Updates = NoCountUpdates;

                if (Ordered) {
                    Updates.push_back(new LTSAssignSimple(CountExp, Mgr->MakeExpr(LTSOps::OpADD,
                                                                                  CountExp,
                                                                                  OneExp)));
                }
                Updates.push_back(new LTSAssignSimple(Mgr->MakeVar("state", StateType),
                                                      Mgr->MakeVal("ChanInitState", StateType)));
                if (Lossy) {
                    NoCountUpdates.push_back(new LTSAssignSimple(Mgr->MakeVar("state", StateType),
                                                                 Mgr->MakeVal("LossDecideState",
                                                                              StateType)));
                }
                if (!Lossy && First) {
                    EFSMBase::AddInputTransForInstance(InstanceID,
                                                       SubstMap,
                                                       "ChanInitState",
                                                       Guard,
                                                       Updates,
                                                       InMsgName,
                                                       MessageType,
                                                       MessageType,
                                                       LTSSymbTransRef::NullPtr);
                } else {
                    vector<LTSAssignRef> Step2Updates;
                    ExpT PushIndexExp = nullptr;
                    if (!Ordered) {
                        PushIndexExp = Mgr->MakeExpr(LTSOps::OpIndex, ArrayExp, ChooseExp);
                    } else {
                        PushIndexExp = Mgr->MakeExpr(LTSOps::OpIndex, ArrayExp, CountExp);
                    }
                    Step2Updates.push_back(new LTSAssignSimple(PushIndexExp, LastMsgExp));
                    if (Ordered) {
                        Step2Updates.push_back(new LTSAssignSimple(CountExp,
                                                                   Mgr->MakeExpr(LTSOps::OpADD,
                                                                                 CountExp,
                                                                                 OneExp)));
                    }
                    Step2Updates.push_back(new LTSAssignSimple(LastMsgExp,
                                                               Mgr->MakeVal("clear", UMType)));
                    Step2Updates.push_back(new LTSAssignSimple(Mgr->MakeVar("state", StateType),
                                                               Mgr->MakeVal("ChanInitState",
                                                                            StateType)));
                    vector<LTSAssignRef> LossUpdates;
                    LossUpdates.push_back(new LTSAssignSimple(LastMsgExp,
                                                          Mgr->MakeVal("clear", UMType)));
                    LossUpdates.push_back(new LTSAssignSimple(Mgr->MakeVar("state", StateType),
                                                              Mgr->MakeVal("ChanInitState",
                                                                       StateType)));

                    ExpT Guard2 = nullptr;
                    if (Blocking) {
                        EFSMBase::AddInputTransForInstance(InstanceID,
                                                           SubstMap,
                                                           "ChanInitState",
                                                           Guard,
                                                           NoCountUpdates,
                                                           InMsgName,
                                                           MessageType,
                                                           MessageType,
                                                           LTSSymbTransRef::NullPtr);
                        Guard2 = TrueExp;
                    } else {
                        if (First) {
                            EFSMBase::AddInputTransForInstance(InstanceID,
                                                               SubstMap,
                                                               "ChanInitState",
                                                               TrueExp,
                                                               NoCountUpdates,
                                                               InMsgName,
                                                               MessageType,
                                                               MessageType,
                                                               LTSSymbTransRef::NullPtr);
                        }
                        Guard2 = Guard;
                    }

                    // Lossy transition
                    if (First) {
                        EFSMBase::AddInternalTransForInstance(InstanceID,
                                                              SubstMap,
                                                              "LossDecideState",
                                                              TrueExp,
                                                              LossUpdates,
                                                              set<string>(),
                                                              LTSSymbTransRef::NullPtr);
                        First = false;
                    }
                    // Non lossy
                    EFSMBase::AddInternalTransForInstance(InstanceID,
                                                          SubstMap,
                                                          "LossDecideState",
                                                          Guard2,
                                                          Step2Updates,
                                                          InputFairnessSets,
                                                          LTSSymbTransRef::NullPtr);
                }
            }
        }

        void ChannelEFSM::MakeOutputTransition(u32 InstanceID,
                                               const MgrT::SubstMapT& SubstMap,
                                               const TypeRef& MessageType,
                                               const set<string>& NonDupOutputFairnessSets,
                                               const set<string>& DupOutputFairnessSets)
        {
            auto Mgr = TheLTS->GetMgr();
            auto PMessageType = TheLTS->GetPrimedType(MessageType);
            auto UMType = TheLTS->GetUnifiedMType()->As<UnionType>();

            vector<ExpT> ChooseExps;
            if (!Ordered) {
                for (u32 i = 0; i < Capacity; ++i) {
                    ChooseExps.push_back(Mgr->MakeVal(to_string(i), IndexType));
                }
            } else {
                ChooseExps.push_back(ZeroExp);
            }

            for (auto const& ChooseExp : ChooseExps) {
                auto ChooseIndexExp = Mgr->MakeExpr(LTSOps::OpIndex, ArrayExp, ChooseExp);
                auto TrueExp = Mgr->MakeTrue();
                auto FAType = Mgr->MakeType<FieldAccessType>();
                auto FieldVar = Mgr->MakeVar(UMType->GetTypeIDFieldName(), FAType);
                auto FieldExp = Mgr->MakeExpr(LTSOps::OpField, ChooseIndexExp, FieldVar);


                ExpT ChooseOkExp = TrueExp;
                ExpT NonEmptyExp = TrueExp;

                if (Ordered) {
                    NonEmptyExp = Mgr->MakeExpr(LTSOps::OpGT, CountExp, ZeroExp);
                    ChooseOkExp = Mgr->MakeExpr(LTSOps::OpLT, ChooseExp, CountExp);
                }

                auto TypeID = UMType->GetTypeIDForMemberType(MessageType);
                auto MatchExp = Mgr->MakeExpr(LTSOps::OpEQ, FieldExp,
                                              Mgr->MakeVal(to_string(TypeID),
                                                           UMType->GetTypeIDFieldType()));
                auto Guard = Mgr->MakeExpr(LTSOps::OpAND, NonEmptyExp, ChooseOkExp, MatchExp);
                string OutMsgName = "__outmsg__";
                auto OutMsgExp = Mgr->MakeVar(OutMsgName, ValType);
                auto CntSubExp = Mgr->MakeExpr(LTSOps::OpSUB, CountExp, OneExp);

                vector<LTSAssignRef> Updates;
                Updates.push_back(new LTSAssignSimple(OutMsgExp, ChooseIndexExp));
                Updates.push_back(new LTSAssignSimple(Mgr->MakeVar("state", StateType),
                                                      Mgr->MakeVal("ChanInitState",
                                                                   StateType)));
                vector<LTSAssignRef> MsgUpdates = Updates;

                // Clear the last message
                if (Ordered) {
                    Updates.push_back(new LTSAssignSimple(Mgr->MakeExpr(LTSOps::OpIndex,
                                                                        ArrayExp, CntSubExp),
                                                          Mgr->MakeVal("clear", ValType)));
                } else {
                    Updates.push_back(new LTSAssignSimple(Mgr->MakeExpr(LTSOps::OpIndex,
                                                                        ArrayExp, ChooseExp),
                                                          Mgr->MakeVal("clear", ValType)));
                }

                if (Ordered) {
                    Updates.push_back(new LTSAssignSimple(CountExp, CntSubExp));
                }
                // Push all the elements past the chosen element back
                // (only for ordered channels, leave it alone for unordered)
                if (Ordered) {
                    for (u32 i = 0; i < Capacity - 1; ++i) {
                        auto Cond = Mgr->MakeExpr(LTSOps::OpGE, Mgr->MakeVal(to_string(i),
                                                                             IndexType),
                                                  ChooseExp);
                        auto const& Fields = ValType->SAs<RecordType>()->GetMemberVec();
                        for (auto const& Field : Fields) {
                            auto FieldVar = Mgr->MakeVar(Field.first, FAType);
                            auto IExp = Mgr->MakeVal(to_string(i), IndexType);
                            auto IPlusOneExp = Mgr->MakeVal(to_string(i + 1), IndexType);
                            auto ArrayIndexExpIf = Mgr->MakeExpr(LTSOps::OpIndex,
                                                                 ArrayExp,
                                                                 IPlusOneExp);
                            auto ArrayIndexExpElse = Mgr->MakeExpr(LTSOps::OpIndex,
                                                                   ArrayExp,
                                                                   IExp);
                            auto IfBranch = Mgr->MakeExpr(LTSOps::OpField,
                                                          ArrayIndexExpIf,
                                                          FieldVar);
                            auto ElseBranch = Mgr->MakeExpr(LTSOps::OpField,
                                                            ArrayIndexExpElse,
                                                            FieldVar);
                            auto ITEExp = Mgr->MakeExpr(LTSOps::OpITE, Cond,
                                                        IfBranch, ElseBranch);
                            Updates.push_back(new LTSAssignSimple(ElseBranch, ITEExp));
                        }
                    }
                }

                EFSMBase::AddOutputTransForInstance(InstanceID,
                                                    SubstMap,
                                                    "ChanInitState",
                                                    Guard,
                                                    Updates,
                                                    OutMsgName,
                                                    PMessageType,
                                                    PMessageType,
                                                    NonDupOutputFairnessSets,
                                                    LTSSymbTransRef::NullPtr);
                if (Duplicating) {
                    EFSMBase::AddOutputTransForInstance(InstanceID,
                                                        SubstMap,
                                                        "ChanInitState",
                                                        Guard,
                                                        MsgUpdates,
                                                        OutMsgName,
                                                        PMessageType,
                                                        PMessageType,
                                                        DupOutputFairnessSets,
                                                        LTSSymbTransRef::NullPtr);
                }
            }
        }

        void ChannelEFSM::AddMsg(const TypeRef& MessageType,
                                 const vector<ExpT>& Params,
                                 LTSFairnessType MessageFairness,
                                 LossDupFairnessType LossDupFairness)
        {
            auto PMessageType = TheLTS->GetPrimedType(MessageType);
            auto Mgr = TheLTS->GetMgr();
            EFSMBase::AddInputMsg(MessageType, Params);
            EFSMBase::AddOutputMsg(PMessageType, Params);

            set<string> InputFairnessSets;
            set<string> NonDupOutputFairnessSets;
            set<string> DupOutputFairnessSets;

            if (Lossy && (LossDupFairness == LossDupFairnessType::NotAlwaysLost ||
                          LossDupFairness == LossDupFairnessType::NotAlwaysLostOrDup)) {
                auto FairID = LossDupFairnessUIDGen.GetUID();
                string FairnessName = "LossFairness_" + to_string(FairID);
                EFSMBase::AddFairnessSet(FairnessName, FairSetFairnessType::Strong);
                InputFairnessSets.insert(FairnessName);
            }

            if (!Duplicating) {
                if (MessageFairness != LTSFairnessType::None) {
                    string FairnessSetName = "MessageFairness_" +
                        to_string(MessageFairnessUIDGen.GetUID());
                    EFSMBase::AddFairnessSet(FairnessSetName,
                                             MessageFairness == LTSFairnessType::Strong ?
                                             FairSetFairnessType::Strong :
                                             FairSetFairnessType::Weak);
                    NonDupOutputFairnessSets.insert(FairnessSetName);
                }
            } else {
                if (LossDupFairness == LossDupFairnessType::NotAlwaysDup ||
                    LossDupFairness == LossDupFairnessType::NotAlwaysLostOrDup) {
                    string FairnessSetName = "DupFairness_" +
                        to_string(LossDupFairnessUIDGen.GetUID());
                    EFSMBase::AddFairnessSet(FairnessSetName,
                                             FairSetFairnessType::Strong);
                    NonDupOutputFairnessSets.insert(FairnessSetName);
                } else if (MessageFairness != LTSFairnessType::None) {
                    string FairnessSetName = "MessageFairness_" +
                        to_string(MessageFairnessUIDGen.GetUID());
                    EFSMBase::AddFairnessSet(FairnessSetName,
                                             MessageFairness == LTSFairnessType::Strong ?
                                             FairSetFairnessType::Strong :
                                             FairSetFairnessType::Weak);
                    NonDupOutputFairnessSets.insert(FairnessSetName);
                    DupOutputFairnessSets = NonDupOutputFairnessSets;
                }
            }

            const u32 NumInsts = ParamInsts.size();
            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                auto&& SubstParams = SubstAll(Params, SubstMap, Mgr);

                auto ActMType = InstantiateType(MessageType, SubstParams, Mgr);
                MakeInputTransition(i, SubstMap, ActMType, InputFairnessSets);
                MakeOutputTransition(i, SubstMap, ActMType, NonDupOutputFairnessSets,
                                     DupOutputFairnessSets);
            }
        }

        void ChannelEFSM::AddMsgs(const vector<ExpT> NewParams,
                                  const ExpT& Constraint,
                                  const TypeRef& MessageType,
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

            set<string> InputFairnessSets;
            set<string> NonDupOutputFairnessSets;
            set<string> DupOutputFairnessSets;


            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                auto SubstConstraint = Mgr->Substitute(SubstMap, Constraint);
                auto const&& NewInsts = InstantiateParams(NewParams, SubstConstraint, Mgr);

                if (i == 0) {
                    if (Lossy && (LossDupFairness == LossDupFairnessType::NotAlwaysLost ||
                                  LossDupFairness == LossDupFairnessType::NotAlwaysLostOrDup)) {
                        auto FairID = LossDupFairnessUIDGen.GetUID();
                        string FairnessName = "LossFairness_" + to_string(FairID);
                        EFSMBase::AddFairnessSet(FairnessName, FairSetFairnessType::Strong);
                        InputFairnessSets.insert(FairnessName);
                    }

                    if (!Duplicating) {
                        if (MessageFairness != LTSFairnessType::None) {
                            string FairnessSetName = "MessageFairness_" +
                                to_string(MessageFairnessUIDGen.GetUID());
                            EFSMBase::AddFairnessSet(FairnessSetName,
                                                     MessageFairness == LTSFairnessType::Strong ?
                                                     FairSetFairnessType::Strong :
                                                     FairSetFairnessType::Weak);
                            NonDupOutputFairnessSets.insert(FairnessSetName);
                        }
                    } else {
                        if (LossDupFairness == LossDupFairnessType::NotAlwaysDup ||
                            LossDupFairness == LossDupFairnessType::NotAlwaysLostOrDup) {
                            string FairnessSetName = "DupFairness_" +
                                to_string(LossDupFairnessUIDGen.GetUID());
                            EFSMBase::AddFairnessSet(FairnessSetName,
                                                     FairSetFairnessType::Strong);
                            NonDupOutputFairnessSets.insert(FairnessSetName);
                        } else if (MessageFairness != LTSFairnessType::None) {
                            string FairnessSetName = "MessageFairness_" +
                                to_string(MessageFairnessUIDGen.GetUID());
                            EFSMBase::AddFairnessSet(FairnessSetName,
                                                     MessageFairness == LTSFairnessType::Strong ?
                                                     FairSetFairnessType::Strong :
                                                     FairSetFairnessType::Weak);
                            NonDupOutputFairnessSets.insert(FairnessSetName);
                            DupOutputFairnessSets = NonDupOutputFairnessSets;
                        }
                    }
                }


                for (auto const& NewInst : NewInsts) {
                    auto LocalSubstMap = SubstMap;
                    for (u32 j = 0; j < NumNewParams; ++j) {
                        LocalSubstMap[NewParams[j]] = NewInst[j];
                    }
                    auto&& SubstParams = SubstAll(MessageParams, LocalSubstMap, Mgr);
                    auto MType = InstantiateType(MessageType, SubstParams, Mgr);
                    auto PMType = TheLTS->GetPrimedType(MType);

                    MakeInputTransition(i, LocalSubstMap, MType, InputFairnessSets);
                    MakeOutputTransition(i, LocalSubstMap, MType, NonDupOutputFairnessSets,
                                         DupOutputFairnessSets);
                }
            }
        }

        SymmMsgDeclRef ChannelEFSM::AddInputMsg(const TypeRef& MessageType,
                                                const vector<ExpT>& Params)
        {
            throw ESMCError((string)"ChannelEFSM::AddInputMsg() should not be called");
        }

        SymmMsgDeclRef ChannelEFSM::AddInputMsgs(const vector<ExpT>& NewParams,
                                                 const ExpT& Constraint,
                                                 const TypeRef& MessageType,
                                                 const vector<ExpT>& MessageParams)
        {
            throw ESMCError((string)"ChannelEFSM::AddInputMsgs() should not be called");
        }

        SymmMsgDeclRef ChannelEFSM::AddOutputMsg(const TypeRef& MessageType,
                                                 const vector<ExpT>& Params)
        {
            throw ESMCError((string)"ChannelEFSM::AddOutputMsg() should not be called");
        }

        SymmMsgDeclRef ChannelEFSM::AddOutputMsgs(const vector<ExpT>& NewParams,
                                                  const ExpT& Constraint,
                                                  const TypeRef& MessageType,
                                                  const vector<ExpT>& MessageParams)
        {
            throw ESMCError((string)"ChannelEFSM::AddOutputMsgs() should not be called");
        }


        void ChannelEFSM::AddVariable(const string& VarName, const TypeRef& VarType)
        {
            throw ESMCError((string)"ChannelEFSM::AddVariable() should not be called");
        }

        void ChannelEFSM::AddInputTransition(const string& InitState,
                                             const ExpT& Guard,
                                             const vector<LTSAssignRef>& Updates,
                                             const string& MessageName,
                                             const TypeRef& MessageType,
                                             const vector<ExpT>& MessageParams,
                                             bool Tentative)
        {
            throw ESMCError((string)"ChannelEFSM::AddInputTransition() should not be called");
        }

        void ChannelEFSM::AddInputTransitions(const vector<ExpT>& TransParams,
                                              const ExpT& Constraint,
                                              const string& InitState,
                                              const ExpT& Guard,
                                              const vector<LTSAssignRef>& Updates,
                                              const string& MessageName,
                                              const TypeRef& MessageType,
                                              const vector<ExpT>& MessageParams,
                                              bool Tentative)
        {
            throw ESMCError((string)"ChannelEFSM::AddInputTransitions() should not be called");
        }

        void ChannelEFSM::AddOutputTransition(const string& InitState,
                                              const ExpT& Guard,
                                              const vector<LTSAssignRef>& Updates,
                                              const string& MessageName,
                                              const TypeRef& MessageType,
                                              const vector<ExpT>& MessageParams,
                                              const set<string>& AddToFairnessSets,
                                              bool Tentative)
        {
            throw ESMCError((string)"ChannelEFSM::AddOutputTransition() should not be called");
        }

        void ChannelEFSM::AddOutputTransitions(const vector<ExpT>& TransParams,
                                               const ExpT& Constraint,
                                               const string& InitState,
                                               const ExpT& Guard,
                                               const vector<LTSAssignRef>& Updates,
                                               const string& MessageName,
                                               const TypeRef& MessageType,
                                               const vector<ExpT>& MessageParams,
                                               LTSFairnessType MessageFairness,
                                               SplatFairnessType SplatFairness,
                                               const string& SplatFairnessName,
                                               bool Tentative)
        {
            throw ESMCError((string)"ChannelEFSM::AddOutputTransitions() should not be called");
        }


        void ChannelEFSM::AddInternalTransition(const string& InitState,
                                                const ExpT& Guard,
                                                const vector<LTSAssignRef>& Updates,
                                                const set<string>& AddToFairnessSets,
                                                bool Tentative)
        {
            throw ESMCError((string)"ChannelEFSM::AddInternalTransition() should not be called");
        }

        void ChannelEFSM::AddInternalTransitions(const vector<ExpT>& TransParams,
                                                 const ExpT& Constraint,
                                                 const string& InitState,
                                                 const ExpT& Guard,
                                                 const vector<LTSAssignRef>& Updates,
                                                 LTSFairnessType MessageFairness,
                                                 SplatFairnessType SplatFairness,
                                                 const string& SplatFairnessName,
                                                 bool Tentative)
        {
            throw ESMCError((string)"ChannelEFSM::AddInternalTransitions() should not be called");
        }

        vector<LTSAssignRef>
        ChannelEFSM::GetUpdatesForPermutation(const vector<u08>& Permutation,
                                              u32 InstanceID) const
        {
            auto Mgr = TheLTS->GetMgr();
            if (Ordered) {
                throw InternalError((string)"Cannot call ChannelEFSM::GetUpdatesForPermutation() " +
                                    "on an ordered channel.\nAt: " + __FILE__ + ":" +
                                    to_string(__LINE__));
            }

            if (Permutation.size() != Capacity) {
                throw InternalError((string)"ChannelEFSM::GetUpdatesForPermutation() called " +
                                    "with wrong type of permutation vector. Channel size is " +
                                    to_string(Capacity) + " and permutation vector has " +
                                    to_string(Permutation.size()) + " elements.\nAt: " +
                                    __FILE__ + ":" + to_string(__LINE__));
            }

            vector<LTSAssignRef> Updates;
            for (u32 i = 0; i < Capacity; ++i) {
                if (i == Permutation[i]) {
                    continue;
                }
                auto LHS = Mgr->MakeExpr(LTSOps::OpIndex, ArrayExp,
                                         Mgr->MakeVal(to_string(Permutation[i]), IndexType));
                auto RHS = Mgr->MakeExpr(LTSOps::OpIndex, ArrayExp,
                                         Mgr->MakeVal(to_string(i), IndexType));

                Updates.push_back(new LTSAssignSimple(LHS, RHS));
            }

            auto const& ParamInst = ParamInsts[InstanceID];
            auto&& RebasedUpdates = RebaseUpdates(ParamInst, Updates);
            return ExpandUpdates(RebasedUpdates);
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

//
// LTSChannelEFSM.cpp ends here
