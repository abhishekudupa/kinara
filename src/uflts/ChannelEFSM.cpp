// ChannelEFSM.cpp --- 
// 
// Filename: ChannelEFSM.cpp
// Author: Abhishek Udupa
// Created: Sun Aug  3 14:35:32 2014 (-0400)
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

#include "ChannelEFSM.hpp"
#include "UFLTSExtension.hpp"
#include "UFLTS.hpp"
#include "FrozenEFSM.hpp"

namespace ESMC {
    namespace LTS {

        // Channel EFSM implementation
        ChannelEFSM::ChannelEFSM(UFLTS* TheLTS, const string& Name, 
                                 const vector<ExpT>& Params,
                                 const ExpT& Constraint,
                                 u32 Capacity, bool Ordered, bool Lossy, 
                                 bool Duplicating, bool Blocking)
            : TheLTS(TheLTS), Name(Name), Capacity(Capacity),
              Ordered(Ordered), Lossy(Lossy), Duplicating(Duplicating),
              Blocking(Blocking)
        {
            auto Mgr = TheLTS->GetMgr();
            // Sanity checks on params
            if (Blocking && !Lossy) {
                throw ESMCError((string)"Only Lossy channels can be declared Blocking");
            }
            
            set<string> StateNames;
            States["ChanInitState"] = Detail::StateDescriptor("ChanInitState");
            StateNames.insert("ChanInitState");
            if (Lossy) {
                States["LossDecideState"] = Detail::StateDescriptor("LossDecideState");
                StateNames.insert("LossDecideState");
            }

            auto StateType = Mgr->MakeType<Exprs::ExprEnumType>(Name + "StateT", StateNames);
            
            ValType = TheLTS->GetUnifiedMType();
            CountType = Mgr->MakeType<Exprs::ExprRangeType>(0, Capacity);
            IndexType = Mgr->MakeType<Exprs::ExprRangeType>(0, Capacity - 1);
            ArrayType = Mgr->MakeType<Exprs::ExprArrayType>(IndexType, ValType);

            CountExp = Mgr->MakeVar("MsgCount", CountType);
            ArrayExp = Mgr->MakeVar("MsgBuffer", ArrayType);
            LastMsgExp = Mgr->MakeVar("LastMsg", ValType);
            MaxChanExp = Mgr->MakeVal(to_string(Capacity), CountType);
            OneExp = Mgr->MakeVal("1", CountType);
            ZeroExp = Mgr->MakeVal("0", CountType);
            
            CheckParams(Params, Constraint, SymTab, Mgr);

            if (Params.size() == 0) {
                ParamInsts.push_back(vector<ExpT>());
                ParamSubsts.push_back(MgrType::SubstMapT());
                auto TheEFSM = new FrozenEFSM(Name, TheLTS, Params, this, 
                                              StateType, States);
                TheEFSM->AddVariable("MsgCount", CountType);
                TheEFSM->AddVariable("MsgBuffer", ArrayType);
                if (Lossy) {
                    TheEFSM->AddVariable("LastMsg", ValType);
                }
                FrozenEFSMs.push_back(TheEFSM);
                return;
            }

            // We have params
            const u32 NumParams = Params.size();
            ParamInsts = InstantiateParams(Params, Constraint, Mgr);
            
            for (auto const& ParamVal : ParamInsts) {
                MgrType::SubstMapT SubstMap;
                
                string InstName = Name;
                
                for (u32 i = 0; i < NumParams; ++i) {
                    SubstMap[Params[i]] = ParamVal[i];
                }

                ParamSubsts.push_back(SubstMap);
                auto TheEFSM = new FrozenEFSM(Name, TheLTS, ParamVal, this, 
                                              StateType, States);
                TheEFSM->AddVariable("MsgCount", CountType);
                TheEFSM->AddVariable("MsgBuffer", ArrayType);
                if (Lossy) {
                    TheEFSM->AddVariable("LastMsg", ValType);
                }
                FrozenEFSMs.push_back(TheEFSM);
            }
        }

        ChannelEFSM::~ChannelEFSM()
        {
            // Nothing here
        }

        const string& ChannelEFSM::GetName() const
        {
            return Name;
        }
        
        inline void ChannelEFSM::MakeInputTransition(const ExprTypeRef& MType, 
                                                     FrozenEFSM* TheEFSM,
                                                     MessageFairnessType Fairness)
        {
            auto Mgr = TheLTS->GetMgr();

            ExpT TargetExp = nullptr;

            if (Lossy) {
                TargetExp = LastMsgExp;
            } else {
                TargetExp = Mgr->MakeExpr(LTSOps::OpIndex, ArrayExp, CountExp);
            }

            auto Guard = Mgr->MakeExpr(LTSOps::OpLT, CountExp, MaxChanExp);
            vector<AsgnT> Updates;
            string InMsgName = "__inmsg__";
            auto InMsgVar = Mgr->MakeVar(InMsgName, MType);

            auto FAType = Mgr->MakeType<Exprs::ExprFieldAccessType>();
            auto UFAType = Mgr->MakeType<Exprs::ExprUFAType>(MType);

            auto TypeAsRec = MType->As<Exprs::ExprRecordType>();
            for (auto const& Member : TypeAsRec->GetMemberVec()) {
                auto LHSFieldVar = Mgr->MakeVar(Member.first, UFAType);
                auto RHSFieldVar = Mgr->MakeVar(Member.first, FAType);
                
                auto LHSExp = Mgr->MakeExpr(LTSOps::OpField, TargetExp, LHSFieldVar);
                auto RHSExp = Mgr->MakeExpr(LTSOps::OpField, InMsgVar, RHSFieldVar);
                Updates.push_back(AsgnT(LHSExp, RHSExp));
            }

            auto NoCountUpdates = Updates;
            Updates.push_back(AsgnT(CountExp, Mgr->MakeExpr(LTSOps::OpADD, CountExp,
                                                            OneExp)));

            auto TrueExp = Mgr->MakeTrue();

            if (!Lossy) {
                TheEFSM->AddInputTransition("ChanInitState", "ChanInitState", Guard, 
                                            Updates, InMsgName, MType);
            } else {
                
                vector<AsgnT> Step2Updates;
                Step2Updates.push_back(AsgnT(Mgr->MakeExpr(LTSOps::OpIndex, ArrayExp,
                                                           CountExp),
                                             LastMsgExp));
                Step2Updates.push_back(Updates.back());

                unordered_set<u32> FairnessSet;
                if (Fairness == MessageFairnessType::NOT_ALWAYS_LOST ||
                    Fairness == MessageFairnessType::NOT_ALWAYS_LOST_DUP) {
                    FairnessSet = { (u32)FairnessUIDGen.GetUID() };
                }

                if (Blocking) {
                    TheEFSM->AddInputTransition("ChanInitState", "LossDecideState", Guard,
                                                NoCountUpdates, InMsgName, MType);

                    TheEFSM->AddInternalTransition("LossDecideState", "ChanInitState", TrueExp,
                                                   Step2Updates, FairnessSet);

                    TheEFSM->AddInternalTransition("LossDecideState", "ChanInitState", TrueExp,
                                                   vector<AsgnT>(), unordered_set<u32>());

                } else {
                    TheEFSM->AddInputTransition("ChanInitState", "LossDecideState", TrueExp,
                                                NoCountUpdates, InMsgName, MType);

                    TheEFSM->AddInternalTransition("LossDecideState", "ChanInitState", Guard,
                                                   Step2Updates, FairnessSet);

                    TheEFSM->AddInternalTransition("LossDecideState", "ChanInitState", TrueExp,
                                                   vector<AsgnT>(), unordered_set<u32>());
                }
            }
        }

        inline void ChannelEFSM::MakeOutputTransition(const ExprTypeRef& MType, 
                                                      FrozenEFSM* TheEFSM,
                                                      MessageFairnessType Fairness)
        {
            auto Mgr = TheLTS->GetMgr();
            u32 FairnessID = FairnessUIDGen.GetUID();

            for (u32 j = 0; j < Capacity; ++j) {
                auto ChooseExp = Mgr->MakeVal(to_string(j), IndexType);
                auto TypeAsRec = MType->As<Exprs::ExprRecordType>();
                auto IndexExp = Mgr->MakeExpr(LTSOps::OpIndex, ArrayExp, ChooseExp);
                auto CountPosExp = Mgr->MakeExpr(LTSOps::OpGT, CountExp, ZeroExp);
                auto MTypeID = TheLTS->GetTypeIDForMessageType(TypeAsRec->GetName());
                auto FAType = Mgr->MakeType<Exprs::ExprFieldAccessType>();
                auto UFAType = Mgr->MakeType<Exprs::ExprUFAType>(MType);
                
                auto MTypeFieldVar = Mgr->MakeVar(MTypeFieldName, UFAType);
                auto MessageIDType = TheLTS->GetMessageIDType();
                
                auto MTypeMatchExp = Mgr->MakeExpr(LTSOps::OpEQ,
                                                   Mgr->MakeExpr(LTSOps::OpField, IndexExp,
                                                                 MTypeFieldVar),
                                                   Mgr->MakeVal(to_string(MTypeID), MessageIDType));
                auto ChooseLTExp = Mgr->MakeExpr(LTSOps::OpLT, ChooseExp, CountExp);

                auto Guard = Mgr->MakeExpr(LTSOps::OpAND, CountPosExp, ChooseLTExp, MTypeMatchExp);
                vector<AsgnT> MsgUpdates;
                vector<AsgnT> StateUpdates;
                string MsgName = "__outmsg__";
                auto MsgExp = Mgr->MakeVar(MsgName, MType);

                for (auto const& Member : TypeAsRec->GetMemberVec()) {
                    auto LHSExp = Mgr->MakeExpr(LTSOps::OpField, IndexExp,
                                                Mgr->MakeVar(Member.first, UFAType));
                    auto RHSExp = Mgr->MakeExpr(LTSOps::OpField, MsgExp,
                                                Mgr->MakeVar(Member.first, FAType));
                    MsgUpdates.push_back(AsgnT(LHSExp, RHSExp));
                }
                
                for (u32 i = 0; i < Capacity - 1; ++i) {
                    auto Cond = Mgr->MakeExpr(LTSOps::OpGE,
                                          Mgr->MakeVal(to_string(i), IndexType),
                                              ChooseExp);
                    
                    auto ArrIExp = Mgr->MakeExpr(LTSOps::OpIndex, ArrayExp,
                                                 Mgr->MakeVal(to_string(i), IndexType));
                    
                    auto ArrIExpPlusOne = Mgr->MakeExpr(LTSOps::OpIndex, ArrayExp,
                                                        Mgr->MakeVal(to_string(i + 1), IndexType));
                    
                    auto ITEExp = Mgr->MakeExpr(LTSOps::OpITE,
                                            Cond, ArrIExpPlusOne, ArrIExp);
                    StateUpdates.push_back(AsgnT(ArrIExp, ITEExp));
                }
                
                StateUpdates.push_back(AsgnT(CountExp, 
                                             Mgr->MakeExpr(LTSOps::OpSUB, CountExp, OneExp)));
                
                auto AllUpdates = MsgUpdates;
                AllUpdates.insert(AllUpdates.end(), StateUpdates.begin(), StateUpdates.end());

                if (!Duplicating) {
                    unordered_set<u32> FairnessSet;
                    if (Fairness == MessageFairnessType::PLAIN) {
                        FairnessSet = { FairnessID };
                    }
                    TheEFSM->AddOutputTransition("ChanInitState", "ChanInitState", 
                                                 Guard, AllUpdates, MsgName, 
                                                 MType, FairnessSet);
                }
                else /*if (duplicating) */ {
                    unordered_set<u32> FairnessSet;
                    if (Fairness == MessageFairnessType::NOT_ALWAYS_DUP ||
                        Fairness == MessageFairnessType::NOT_ALWAYS_LOST_DUP) {
                        
                        FairnessSet = { FairnessID };
                    }

                    TheEFSM->AddOutputTransition("ChanInitState", "ChanInitState",
                                                 Guard, MsgUpdates, MsgName,
                                                 MType, unordered_set<u32>());

                    TheEFSM->AddOutputTransition("ChanInitState", "ChanInitState",
                                                 Guard, AllUpdates, MsgName,
                                                 MType, FairnessSet);                    
                }
            }
        }

        void ChannelEFSM::AddMessage(const ExprTypeRef& MType,
                                     const vector<ExpT>& MParams,
                                     MessageFairnessType Fairness)
        {
            auto Mgr = TheLTS->GetMgr();
            
            CheckParams(MParams, SymTab);

            const u32 NumInsts = ParamInsts.size();
            for (u32 i = 0; i < NumInsts; ++i) {
                auto&& SubstParams = SubstAll(MParams, ParamSubsts[i], Mgr);
                auto Type = InstantiateType(MType, SubstParams, Mgr);

                FrozenEFSMs[i]->AddInputMsg(Type);
                FrozenEFSMs[i]->AddOutputMsg(TheLTS->GetPrimedType(Type));
                MakeInputTransition(Type, FrozenEFSMs[i], Fairness);
                MakeOutputTransition(Type, FrozenEFSMs[i], Fairness);
            }

        }

        void ChannelEFSM::AddMessages(const vector<ExpT>& NewParams,
                                      const ExprTypeRef& MType,
                                      const vector<ExpT>& MParams,
                                      const ExpT& Constraint,
                                      MessageFairnessType Fairness)
        {
            auto Mgr = TheLTS->GetMgr();
            
            SymTab.Push();
            CheckParams(NewParams, Constraint, SymTab, Mgr, true);
            CheckParams(Params, SymTab);
            SymTab.Pop();

            const u32 NumInsts = ParamInsts.size();
            const u32 NumNewParams = NewParams.size();
            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                auto SubstConstraint = Mgr->Substitute(SubstMap, Constraint);
                
                auto&& NewInsts = InstantiateParams(NewParams, SubstConstraint, Mgr);
                const u32 NumNewInsts = NewInsts.size();

                for (u32 j = 0; j < NumNewInsts; ++j) {
                    auto LocalSubstMap = SubstMap;
                    auto CurNewInst = NewInsts[j];
                    for (u32 k = 0; k < NumNewParams; ++k) {
                        LocalSubstMap[NewParams[k]] = CurNewInst[k];
                    }

                    auto&& SubstParams = SubstAll(MParams, LocalSubstMap, Mgr);
                    auto Type = InstantiateType(MType, SubstParams, Mgr);
                    
                    FrozenEFSMs[i]->AddInputMsg(Type);
                    FrozenEFSMs[i]->AddOutputMsg(TheLTS->GetPrimedType(Type));
                    MakeInputTransition(Type, FrozenEFSMs[i], Fairness);
                    MakeOutputTransition(Type, FrozenEFSMs[i], Fairness);
                }
            }
        }

        const vector<FrozenEFSM*>& ChannelEFSM::GetFrozenEFSMs()
        {
            return FrozenEFSMs;
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// ChannelEFSM.cpp ends here
