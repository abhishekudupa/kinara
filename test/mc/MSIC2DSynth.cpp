// MSIC2DSynth.cpp ---
//
// Filename: MSIC2DSynth.cpp
// Author: Abhishek Udupa
// Created: Thu Nov  6 12:08:45 2014 (-0500)
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

// The symmetric version of the MSI model,
// parameterized on number of caches, number of
// directories, addresses and the number of data
// values

#include "../../src/uflts/LabelledTS.hpp"
#include "../../src/uflts/LTSEFSM.hpp"
#include "../../src/uflts/LTSChannelEFSM.hpp"
#include "../../src/uflts/LTSAssign.hpp"
#include "../../src/uflts/LTSTransitions.hpp"
#include "../../src/mc/LTSChecker.hpp"
#include "../../src/mc/OmegaAutomaton.hpp"
#include "../../src/mc/Trace.hpp"
#include "../../src/synth/Solver.hpp"

using namespace ESMC;
using namespace LTS;
using namespace Exprs;
using namespace MC;
using namespace Synth;

const u32 NumCaches = 2;
const u32 NumAddresses = 1;
const u32 NumValues = 2;
const u32 NumDirs = 1;

int main()
{
    auto TheLTS = new LabelledTS();

    auto TrueExp = TheLTS->MakeTrue();

    // Symmetric types
    auto CacheIDType = TheLTS->MakeSymmType("CacheIDType", NumCaches);
    auto DirIDType = TheLTS->MakeSymmType("DirIDType", NumDirs);
    auto ValueType = TheLTS->MakeSymmType("ValueType", NumValues);
    auto AddressType = TheLTS->MakeSymmType("AddressType", NumAddresses);

    auto CacheParam = TheLTS->MakeVar("CacheID", CacheIDType);
    auto CacheParam1 = TheLTS->MakeVar("CacheID1", CacheIDType);
    auto CacheParam2 = TheLTS->MakeVar("CacheID2", CacheIDType);

    auto DirParam = TheLTS->MakeVar("DirID", DirIDType);
    auto AddressParam = TheLTS->MakeVar("AddressID", AddressType);
    auto ValueParam = TheLTS->MakeVar("ValueID", ValueType);

    auto AckType = TheLTS->MakeRangeType(-((i64)(NumCaches - 1)), NumCaches - 1);
    auto FAType = TheLTS->MakeFieldAccessType();

    // Useful expressions for constraints
    auto CacheNEQCache1 = TheLTS->MakeOp(LTSOps::OpNOT, TheLTS->MakeOp(LTSOps::OpEQ,
                                                                       CacheParam,
                                                                       CacheParam1));

    // Message types
    vector<pair<string, ExprTypeRef>> MessageFields;
    auto GetXMsgType = TheLTS->MakeMsgTypes({ CacheParam, DirParam, AddressParam },
                                            TrueExp, "GetXMsgType", MessageFields, true);
    auto GetSMsgType = TheLTS->MakeMsgTypes({ CacheParam, DirParam, AddressParam },
                                            TrueExp, "GetSMsgType", MessageFields, true);

    MessageFields.push_back(make_pair("Data", ValueType));
    auto WBMsgType = TheLTS->MakeMsgTypes({ CacheParam, DirParam, AddressParam },
                                          TrueExp, "WBMsgType", MessageFields, true);
    auto WBMsgIn = TheLTS->MakeVar("InMsg", TheLTS->GetNamedType("WBMsgType'"));
    auto WBMsgOut = TheLTS->MakeVar("OutMsg", WBMsgType);
    auto WBMsgInDotData = TheLTS->MakeOp(LTSOps::OpField, WBMsgIn,
                                         TheLTS->MakeVar("Data", FAType));
    auto WBMsgOutDotData = TheLTS->MakeOp(LTSOps::OpField, WBMsgOut,
                                          TheLTS->MakeVar("Data", FAType));

    MessageFields.clear();
    MessageFields.push_back(make_pair("Requester", CacheIDType));
    auto FwdGetXMsgType = TheLTS->MakeMsgTypes({ CacheParam, DirParam, AddressParam },
                                               TrueExp, "FwdGetXMsgType", MessageFields, true);
    auto FwdGetXMsgIn = TheLTS->MakeVar("InMsg", TheLTS->GetNamedType("FwdGetXMsgType'"));
    auto FwdGetXMsgOut = TheLTS->MakeVar("OutMsg", FwdGetXMsgType);
    auto FwdGetXMsgInDotRequester = TheLTS->MakeOp(LTSOps::OpField, FwdGetXMsgIn,
                                                   TheLTS->MakeVar("Requester", FAType));
    auto FwdGetXMsgOutDotRequester = TheLTS->MakeOp(LTSOps::OpField, FwdGetXMsgOut,
                                                    TheLTS->MakeVar("Requester", FAType));

    auto FwdGetSMsgType = TheLTS->MakeMsgTypes({ CacheParam, DirParam, AddressParam },
                                               TrueExp, "FwdGetSMsgType", MessageFields, true);

    auto FwdGetSMsgIn = TheLTS->MakeVar("InMsg", TheLTS->GetNamedType("FwdGetSMsgType'"));
    auto FwdGetSMsgOut = TheLTS->MakeVar("OutMsg", FwdGetSMsgType);
    auto FwdGetSMsgInDotRequester = TheLTS->MakeOp(LTSOps::OpField, FwdGetSMsgIn,
                                                   TheLTS->MakeVar("Requester", FAType));
    auto FwdGetSMsgOutDotRequester = TheLTS->MakeOp(LTSOps::OpField, FwdGetSMsgOut,
                                                    TheLTS->MakeVar("Requester", FAType));

    MessageFields.clear();
    auto InvAckMsgType = TheLTS->MakeMsgTypes({ CacheParam, CacheParam1, DirParam, AddressParam },
                                              CacheNEQCache1, "InvAckMsgType", MessageFields, true);

    auto UnblockSMsgType = TheLTS->MakeMsgTypes({ CacheParam, DirParam, AddressParam },
                                                TrueExp, "UnblockSMsgType", MessageFields, true);
    auto UnblockXMsgType = TheLTS->MakeMsgTypes({ CacheParam, DirParam, AddressParam },
                                                TrueExp, "UnblockXMsgType", MessageFields, true);

    auto WBAckMsgType = TheLTS->MakeMsgTypes({ CacheParam, DirParam, AddressParam },
                                             TrueExp, "WBAckMsgType", MessageFields, true);
    MessageFields.push_back(make_pair("Data", ValueType));
    MessageFields.push_back(make_pair("NumAcks", AckType));
    auto DataMsgD2CType = TheLTS->MakeMsgTypes({ CacheParam, DirParam, AddressParam },
                                               TrueExp, "DataMsgD2CType", MessageFields, true);

    auto DataMsgD2CIn = TheLTS->MakeVar("InMsg", TheLTS->GetNamedType("DataMsgD2CType'"));
    auto DataMsgD2COut = TheLTS->MakeVar("OutMsg", TheLTS->GetNamedType("DataMsgD2CType"));
    auto DataMsgD2CInDotData = TheLTS->MakeOp(LTSOps::OpField, DataMsgD2CIn,
                                              TheLTS->MakeVar("Data", FAType));
    auto DataMsgD2COutDotData = TheLTS->MakeOp(LTSOps::OpField, DataMsgD2COut,
                                               TheLTS->MakeVar("Data", FAType));
    auto DataMsgD2CInDotNumAcks = TheLTS->MakeOp(LTSOps::OpField, DataMsgD2CIn,
                                                 TheLTS->MakeVar("NumAcks", FAType));
    auto DataMsgD2COutDotNumAcks = TheLTS->MakeOp(LTSOps::OpField, DataMsgD2COut,
                                                  TheLTS->MakeVar("NumAcks", FAType));

    MessageFields.clear();
    MessageFields.push_back(make_pair("Data", ValueType));
    auto DataMsgC2CType = TheLTS->MakeMsgTypes({ CacheParam, CacheParam1, DirParam, AddressParam },
                                               CacheNEQCache1, "DataMsgC2CType", MessageFields, true);
    auto DataMsgC2CIn = TheLTS->MakeVar("InMsg", TheLTS->GetNamedType("DataMsgC2CType'"));
    auto DataMsgC2COut = TheLTS->MakeVar("OutMsg", TheLTS->GetNamedType("DataMsgC2CType"));
    auto DataMsgC2CInDotData = TheLTS->MakeOp(LTSOps::OpField, DataMsgC2CIn,
                                              TheLTS->MakeVar("Data", FAType));
    auto DataMsgC2COutDotData = TheLTS->MakeOp(LTSOps::OpField, DataMsgC2COut,
                                               TheLTS->MakeVar("Data", FAType));
    MessageFields.clear();

    MessageFields.push_back(make_pair("Data", ValueType));
    auto DataMsgC2DType = TheLTS->MakeMsgTypes({ CacheParam, DirParam, AddressParam },
                                               TrueExp, "DataMsgC2DType", MessageFields, true);
    auto DataMsgC2DIn = TheLTS->MakeVar("InMsg", TheLTS->GetNamedType("DataMsgC2DType'"));
    auto DataMsgC2DOut = TheLTS->MakeVar("OutMsg", TheLTS->GetNamedType("DataMsgC2DType"));
    auto DataMsgC2DInDotData = TheLTS->MakeOp(LTSOps::OpField, DataMsgC2DIn,
                                              TheLTS->MakeVar("Data", FAType));
    auto DataMsgC2DOutDotData = TheLTS->MakeOp(LTSOps::OpField, DataMsgC2DOut,
                                               TheLTS->MakeVar("Data", FAType));

    MessageFields.clear();

    auto LDMsgType = TheLTS->MakeMsgTypes({ CacheParam, DirParam, AddressParam },
                                          TrueExp, "LDMsgType", MessageFields, false);

    MessageFields.push_back(make_pair("ValueToStore", ValueType));
    auto STMsgType = TheLTS->MakeMsgTypes({ CacheParam, DirParam, AddressParam },
                                          TrueExp, "STMsgType", MessageFields, false);
    auto STMsgIn = TheLTS->MakeVar("InMsg", STMsgType);
    auto STMsgOut = TheLTS->MakeVar("OutMsg", STMsgType);
    auto STMsgInDotValueToStore = TheLTS->MakeOp(LTSOps::OpField, STMsgIn,
                                                 TheLTS->MakeVar("ValueToStore", FAType));
    auto STMsgOutDotValueToStore = TheLTS->MakeOp(LTSOps::OpField, STMsgOut,
                                                  TheLTS->MakeVar("ValueToStore", FAType));

    MessageFields.clear();
    auto EVMsgType = TheLTS->MakeMsgTypes({ CacheParam, DirParam, AddressParam },
                                          TrueExp, "EVMsgType", MessageFields, false);

    MessageFields.push_back(make_pair("LoadedValue", ValueType));
    auto LDAckMsgType = TheLTS->MakeMsgTypes({ CacheParam, DirParam, AddressParam },
                                             TrueExp, "LDAckMsgType", MessageFields, false);
    auto LDAckMsgIn = TheLTS->MakeVar("InMsg", LDAckMsgType);
    auto LDAckMsgOut = TheLTS->MakeVar("OutMsg", LDAckMsgType);
    auto LDAckMsgInDotLoadedValue = TheLTS->MakeOp(LTSOps::OpField, LDAckMsgIn,
                                                   TheLTS->MakeVar("LoadedValue", FAType));
    auto LDAckMsgOutDotLoadedValue = TheLTS->MakeOp(LTSOps::OpField, LDAckMsgOut,
                                                    TheLTS->MakeVar("LoadedValue", FAType));

    MessageFields.clear();
    MessageFields.push_back(make_pair("StoredValue", ValueType));
    auto STAckMsgType = TheLTS->MakeMsgTypes({ CacheParam, DirParam, AddressParam },
                                             TrueExp, "STAckMsgType", MessageFields, false);
    auto STAckMsgIn = TheLTS->MakeVar("InMsg", STAckMsgType);
    auto STAckMsgOut = TheLTS->MakeVar("OutMsg", STAckMsgType);
    auto STAckMsgInDotStoredValue = TheLTS->MakeOp(LTSOps::OpField, STAckMsgIn,
                                                   TheLTS->MakeVar("StoredValue", FAType));
    auto STAckMsgOutDotStoredValue = TheLTS->MakeOp(LTSOps::OpField, STAckMsgOut,
                                                    TheLTS->MakeVar("StoredValue", FAType));

    MessageFields.clear();
    auto EVAckMsgType = TheLTS->MakeMsgTypes({ CacheParam, DirParam, AddressParam },
                                             TrueExp, "EVAckMsgType", MessageFields, false);

    MessageFields.clear();

    TheLTS->FreezeMsgs();

    // The request channel from cache to directory
    auto ReqChannelEFSM = TheLTS->MakeChannel("ReqChannel",
                                              { CacheParam, DirParam, AddressParam },
                                              TrueExp, NumCaches, false, false, false,
                                              false, LTSFairnessType::None);

    ReqChannelEFSM->AddMsg(GetXMsgType, { CacheParam, DirParam, AddressParam },
                           LTSFairnessType::Strong);
    ReqChannelEFSM->AddMsg(GetSMsgType, { CacheParam, DirParam, AddressParam },
                           LTSFairnessType::Strong);
    ReqChannelEFSM->AddMsg(WBMsgType, { CacheParam, DirParam, AddressParam },
                           LTSFairnessType::Strong);

    ReqChannelEFSM->Freeze();

    // The response channel INTO each cache
    auto RspChannelEFSM = TheLTS->MakeChannel("RspChannel",
                                              { CacheParam, DirParam, AddressParam },
                                              TrueExp, NumCaches, false, false, false,
                                              false, LTSFairnessType::None);
    RspChannelEFSM->AddMsg(WBAckMsgType, { CacheParam, DirParam, AddressParam },
                           LTSFairnessType::Strong);
    RspChannelEFSM->AddMsg(DataMsgD2CType, { CacheParam, DirParam, AddressParam },
                           LTSFairnessType::Strong);
    RspChannelEFSM->AddMsgs({ CacheParam1 }, CacheNEQCache1, DataMsgC2CType,
                            { CacheParam1, CacheParam, DirParam, AddressParam },
                            LTSFairnessType::Strong);
    RspChannelEFSM->AddMsgs({ CacheParam1 }, CacheNEQCache1, InvAckMsgType,
                            { CacheParam1, CacheParam, DirParam, AddressParam },
                            LTSFairnessType::Strong);
    RspChannelEFSM->Freeze();

    // The unblock channel INTO each directory
    auto UnblockChannelEFSM = TheLTS->MakeChannel("UnblockChannel",
                                                  { DirParam, AddressParam },
                                                  TrueExp, 2, false, false, false,
                                                  false, LTSFairnessType::None);
    UnblockChannelEFSM->AddMsgs({ CacheParam }, TrueExp, UnblockSMsgType,
                                { CacheParam, DirParam, AddressParam },
                                LTSFairnessType::Strong, LossDupFairnessType::None);
    UnblockChannelEFSM->AddMsgs({ CacheParam }, TrueExp, UnblockXMsgType,
                                { CacheParam, DirParam, AddressParam },
                                LTSFairnessType::Strong, LossDupFairnessType::None);
    UnblockChannelEFSM->AddMsgs({ CacheParam }, TrueExp, DataMsgC2DType,
                                { CacheParam, DirParam, AddressParam },
                                LTSFairnessType::Strong, LossDupFairnessType::None);

    UnblockChannelEFSM->Freeze();

    // The forward channel INTO each cache
    auto FwdChannelEFSM = TheLTS->MakeChannel("FwdChannel",
                                              { CacheParam, DirParam, AddressParam },
                                              TrueExp, 1, false, false, false, false,
                                              LTSFairnessType::None);
    FwdChannelEFSM->AddMsg(FwdGetXMsgType,
                           { CacheParam, DirParam, AddressParam },
                           LTSFairnessType::Strong);
    FwdChannelEFSM->AddMsg(FwdGetSMsgType,
                           { CacheParam, DirParam, AddressParam },
                           LTSFairnessType::Strong);
    FwdChannelEFSM->Freeze();

    // coherence monitor

    vector<LTSAssignRef> Updates;
    auto CoherenceMonitor = TheLTS->MakeGenEFSM("CoherenceMonitor", { DirParam, AddressParam },
                                                TrueExp, LTSFairnessType::None);
    CoherenceMonitor->AddState("InitialState");
    CoherenceMonitor->AddState("DecideState");
    CoherenceMonitor->AddState("ErrorState", false, false, false, true);
    CoherenceMonitor->FreezeStates();

    CoherenceMonitor->AddVariable("ActualLastValue", ValueType);
    CoherenceMonitor->AddVariable("LastSeenValue", ValueType);
    CoherenceMonitor->FreezeVars();

    CoherenceMonitor->AddInputMsgs({ CacheParam }, TrueExp, STAckMsgType,
                                   { CacheParam, DirParam, AddressParam });
    CoherenceMonitor->AddInputMsgs({ CacheParam }, TrueExp, LDAckMsgType,
                                   { CacheParam, DirParam, AddressParam });

    auto ActualLastValueExp = TheLTS->MakeVar("ActualLastValue", ValueType);
    auto LastSeenValueExp = TheLTS->MakeVar("LastSeenValue", ValueType);
    Updates.push_back(new LTSAssignSimple(ActualLastValueExp, STAckMsgInDotStoredValue));
    CoherenceMonitor->AddInputTransitions({ CacheParam }, TrueExp, "InitialState",
                                          "InitialState", TrueExp,
                                          Updates, "InMsg", STAckMsgType,
                                          { CacheParam, DirParam, AddressParam });
    Updates.clear();

    Updates.push_back(new LTSAssignSimple(LastSeenValueExp, LDAckMsgInDotLoadedValue));
    CoherenceMonitor->AddInputTransitions({ CacheParam }, TrueExp, "InitialState",
                                          "DecideState", TrueExp, Updates, "InMsg",
                                          LDAckMsgType,
                                          { CacheParam, DirParam, AddressParam });
    Updates.clear();
    auto Guard = TheLTS->MakeOp(LTSOps::OpEQ, LastSeenValueExp, ActualLastValueExp);

    Updates.push_back(new LTSAssignSimple(LastSeenValueExp,
                                          TheLTS->MakeVal("clear", LastSeenValueExp->GetType())));
    CoherenceMonitor->AddInternalTransition("DecideState", "InitialState", Guard, Updates);

    CoherenceMonitor->AddInternalTransition("DecideState", "ErrorState",
                                            TheLTS->MakeOp(LTSOps::OpNOT, Guard),
                                            Updates);

    Updates.clear();
    CoherenceMonitor->Freeze();

    // The environment automata
    auto EnvEFSM = TheLTS->MakeGenEFSM("Environment",
                                       { CacheParam, DirParam, AddressParam },
                                       TrueExp, LTSFairnessType::Strong);
    EnvEFSM->AddState("InitialState");
    EnvEFSM->AddState("PendingLDState");
    EnvEFSM->AddState("PendingSTState");
    EnvEFSM->AddState("DecideState");
    EnvEFSM->AddState("PendingEVState");
    EnvEFSM->AddState("ErrorState", false, false, false, true);

    EnvEFSM->FreezeStates();

    EnvEFSM->AddVariable("PendingStoreValue", ValueType);
    EnvEFSM->AddVariable("LastSeenStoreValue", ValueType);
    EnvEFSM->FreezeVars();

    auto PendingStoreExp = TheLTS->MakeVar("PendingStoreValue", ValueType);
    auto LastSeenStoreValueExp = TheLTS->MakeVar("LastSeenStoreValue", ValueType);

    EnvEFSM->AddInputMsg(LDAckMsgType, { CacheParam, DirParam, AddressParam });
    EnvEFSM->AddInputMsg(STAckMsgType, { CacheParam, DirParam, AddressParam });
    EnvEFSM->AddInputMsg(EVAckMsgType, { CacheParam, DirParam, AddressParam });

    EnvEFSM->AddOutputMsg(LDMsgType, { CacheParam, DirParam, AddressParam });
    EnvEFSM->AddOutputMsg(STMsgType, { CacheParam, DirParam, AddressParam });
    EnvEFSM->AddOutputMsg(EVMsgType, { CacheParam, DirParam, AddressParam });

    EnvEFSM->AddOutputTransition("InitialState", "PendingLDState",
                                 TrueExp, Updates, "OutMsg", LDMsgType,
                                 { CacheParam, DirParam, AddressParam });

    EnvEFSM->AddInputTransition("PendingLDState", "InitialState", TrueExp,
                                Updates, "InMsg", LDAckMsgType,
                                { CacheParam, DirParam, AddressParam });

    Updates.push_back(new LTSAssignSimple(PendingStoreExp, ValueParam));
    Updates.push_back(new LTSAssignSimple(STMsgOutDotValueToStore, ValueParam));
    EnvEFSM->AddOutputTransitions({ ValueParam }, TrueExp, "InitialState",
                                  "PendingSTState",
                                  TrueExp, Updates, "OutMsg", STMsgType,
                                  { CacheParam, DirParam, AddressParam },
                                  LTSFairnessType::None, SplatFairnessType::None, "");
    Updates.clear();

    Updates.push_back(new LTSAssignSimple(LastSeenStoreValueExp, STAckMsgInDotStoredValue));

    EnvEFSM->AddInputTransition("PendingSTState", "DecideState", TrueExp, Updates,
                                "InMsg", STAckMsgType,
                                { CacheParam, DirParam, AddressParam });
    Updates.clear();

    Updates.push_back(new LTSAssignSimple(LastSeenStoreValueExp,
                                          TheLTS->MakeVal("clear", ValueType)));
    Updates.push_back(new LTSAssignSimple(PendingStoreExp,
                                          TheLTS->MakeVal("clear", ValueType)));

    Guard = TheLTS->MakeOp(LTSOps::OpEQ, PendingStoreExp, LastSeenStoreValueExp);
    EnvEFSM->AddInternalTransition("DecideState", "InitialState", Guard, Updates);
    EnvEFSM->AddInternalTransition("DecideState", "ErrorState",
                                   TheLTS->MakeOp(LTSOps::OpNOT, Guard),
                                   Updates);
    Updates.clear();

    EnvEFSM->AddOutputTransition("InitialState", "PendingEVState", TrueExp,
                                 Updates, "OutMsg", EVMsgType,
                                 { CacheParam, DirParam, AddressParam });
    EnvEFSM->AddInputTransition("PendingEVState", "InitialState", TrueExp, Updates,
                                "InMsg", EVAckMsgType,
                                { CacheParam, DirParam, AddressParam });

    EnvEFSM->Freeze();

    auto CacheEFSM = TheLTS->MakeEFSM<IncompleteEFSM>("Cache", { CacheParam, DirParam, AddressParam },
                                                      TrueExp, LTSFairnessType::Strong);

    CacheEFSM->AddState("C_I");
    CacheEFSM->AddState("C_I_LD");
    CacheEFSM->AddState("C_I_ST");
    CacheEFSM->AddState("C_I_EV");
    CacheEFSM->AddState("C_I_FWD");

    CacheEFSM->AddState("C_S");
    CacheEFSM->AddState("C_S_LD");
    CacheEFSM->AddState("C_S_ST");
    CacheEFSM->AddState("C_S_EV");
    CacheEFSM->AddState("C_S_FWD");

    CacheEFSM->AddState("C_M");
    CacheEFSM->AddState("C_M_LD");
    CacheEFSM->AddState("C_M_ST");
    CacheEFSM->AddState("C_M_EV");
    CacheEFSM->AddState("C_M_FWDS");
    CacheEFSM->AddState("C_M_FWDS_C2D");
    CacheEFSM->AddState("C_M_FWDX");

    CacheEFSM->AddState("C_IM");
    CacheEFSM->AddState("C_IM_FWD");

    CacheEFSM->AddState("C_SM");
    CacheEFSM->AddState("C_SM_FWD");
    CacheEFSM->AddState("C_SM_DECIDE");

    CacheEFSM->AddState("C_M_UNBLOCK");
    CacheEFSM->AddState("C_M_SENDACK");

    CacheEFSM->AddState("C_IS");
    CacheEFSM->AddState("C_IS_FWD");
    CacheEFSM->AddState("C_IS_DONE");
    CacheEFSM->AddState("C_IS_UNBLOCK");

    CacheEFSM->AddState("C_II");
    CacheEFSM->AddState("C_II_SENDACK");
    CacheEFSM->AddState("C_II_FWD");

    CacheEFSM->FreezeStates();

    CacheEFSM->AddVariable("Data", ValueType);
    CacheEFSM->AddVariable("AckCounter", AckType);
    CacheEFSM->AddVariable("PendingWrite", ValueType);
    CacheEFSM->AddVariable("FwdToCache", CacheIDType);

    auto CacheDataExp = TheLTS->MakeVar("Data", ValueType);
    auto CacheAckCountExp = TheLTS->MakeVar("AckCounter", AckType);
    auto CachePendingWriteExp = TheLTS->MakeVar("PendingWrite", ValueType);
    auto CacheFwdToCacheExp = TheLTS->MakeVar("FwdToCache", CacheIDType);

    CacheEFSM->FreezeVars();
    vector<ExpT> CacheParams = { CacheParam, DirParam, AddressParam };

    CacheEFSM->AddInputMsg(LDMsgType, CacheParams);
    CacheEFSM->AddInputMsg(STMsgType, CacheParams);
    CacheEFSM->AddInputMsg(EVMsgType, CacheParams);

    CacheEFSM->AddInputMsg(TheLTS->GetNamedType("FwdGetSMsgType'"),
                           CacheParams);
    CacheEFSM->AddInputMsg(TheLTS->GetNamedType("FwdGetXMsgType'"),
                                              CacheParams);
    CacheEFSM->AddInputMsgs({ CacheParam1 }, CacheNEQCache1,
                            TheLTS->GetNamedType("InvAckMsgType'"),
                            { CacheParam1, CacheParam, DirParam, AddressParam });
    CacheEFSM->AddInputMsg(TheLTS->GetNamedType("DataMsgD2CType'"),
                           CacheParams);
    CacheEFSM->AddInputMsg(TheLTS->GetNamedType("WBAckMsgType'"),
                           CacheParams);
    CacheEFSM->AddInputMsgs({ CacheParam1 }, CacheNEQCache1,
                            TheLTS->GetNamedType("DataMsgC2CType'"),
                            { CacheParam1, CacheParam, DirParam, AddressParam });

    CacheEFSM->AddOutputMsg(LDAckMsgType, CacheParams);
    CacheEFSM->AddOutputMsg(STAckMsgType, CacheParams);
    auto EVAckMsgDecl = CacheEFSM->AddOutputMsg(EVAckMsgType, CacheParams);

    CacheEFSM->AddOutputMsg(UnblockSMsgType, CacheParams);
    CacheEFSM->AddOutputMsg(UnblockXMsgType, CacheParams);

    auto GetXMsgDecl = CacheEFSM->AddOutputMsg(GetXMsgType, CacheParams);
    CacheEFSM->AddOutputMsg(GetSMsgType, CacheParams);
    CacheEFSM->AddOutputMsg(WBMsgType, CacheParams);

    CacheEFSM->AddOutputMsgs({ CacheParam1 }, CacheNEQCache1,
                             InvAckMsgType,
                             { CacheParam, CacheParam1, DirParam, AddressParam });
    CacheEFSM->AddOutputMsgs({ CacheParam1 }, CacheNEQCache1,
                             DataMsgC2CType,
                             { CacheParam, CacheParam1, DirParam, AddressParam });
    CacheEFSM->AddOutputMsg(DataMsgC2DType, CacheParams);

    // Cache transitions

    // From the I State
    // LD on I
    Updates.clear();
    CacheEFSM->AddInputTransition("C_I", "C_I_LD", TrueExp, Updates,
                                  "InMsg", LDMsgType, CacheParams);
    CacheEFSM->AddOutputTransition("C_I_LD", "C_IS", TrueExp, Updates,
                                   "OutMsg", GetSMsgType, CacheParams);

    // ST on I
    Updates.push_back(new LTSAssignSimple(CachePendingWriteExp, STMsgInDotValueToStore));
    CacheEFSM->AddInputTransition("C_I", "C_I_ST", TrueExp, Updates,
                                  "InMsg", STMsgType, CacheParams);
    Updates.clear();
    CacheEFSM->AddOutputTransition("C_I_ST", "C_IM", TrueExp, Updates,
                                   "OutMsg", GetXMsgType, CacheParams);

    // EV on I
    CacheEFSM->AddInputTransition("C_I", "C_I_EV", TrueExp, Updates,
                                  "InMsg", EVMsgType, CacheParams);
    CacheEFSM->AddOutputTransition("C_I_EV", "C_I", TrueExp, Updates,
                                   "OutMsg", EVAckMsgType, CacheParams);

    // FwdGetX on I
    // Candidate for completion
    Updates.push_back(new LTSAssignSimple(CacheFwdToCacheExp, FwdGetXMsgInDotRequester));
    CacheEFSM->AddInputTransition("C_I", "C_I_FWD", TrueExp, Updates,
                                  "InMsg", TheLTS->GetNamedType("FwdGetXMsgType'"),
                                  CacheParams);
    Updates.clear();
    Updates.push_back(new LTSAssignSimple(CacheFwdToCacheExp,
                                          TheLTS->MakeVal("clear", CacheIDType)));
    Guard = TheLTS->MakeOp(LTSOps::OpEQ, CacheFwdToCacheExp,
                           CacheParam1);

    CacheEFSM->AddOutputTransitions({ CacheParam1 }, CacheNEQCache1, "C_I_FWD", "C_I",
                                    Guard, Updates, "OutMsg", InvAckMsgType,
                                    { CacheParam, CacheParam1, DirParam, AddressParam },
                                    LTSFairnessType::None, SplatFairnessType::None, "");
    Updates.clear();

    // Transitions from S
    // S on LD
    CacheEFSM->AddInputTransition("C_S", "C_S_LD", TrueExp, Updates, "InMsg",
                                  LDMsgType, CacheParams);
    Updates.push_back(new LTSAssignSimple(LDAckMsgOutDotLoadedValue, CacheDataExp));
    CacheEFSM->AddOutputTransition("C_S_LD", "C_S", TrueExp, Updates, "OutMsg",
                                   LDAckMsgType, CacheParams);
    Updates.clear();

    // S on ST
    Updates.push_back(new LTSAssignSimple(CachePendingWriteExp, STMsgInDotValueToStore));
    CacheEFSM->AddInputTransition("C_S", "C_S_ST", TrueExp, Updates, "InMsg", STMsgType,
                                  CacheParams);
    Updates.clear();
    // CacheEFSM->AddOutputTransition("C_S_ST", "C_SM", TrueExp, Updates,
    //                                "OutMsg", GetXMsgType, CacheParams);

    // S on EV
    CacheEFSM->AddInputTransition("C_S", "C_S_EV", TrueExp, Updates, "InMsg", EVMsgType, CacheParams);
    Updates.push_back(new LTSAssignSimple(CacheDataExp, TheLTS->MakeVal("clear", ValueType)));
    CacheEFSM->AddOutputTransition("C_S_EV", "C_I", TrueExp, Updates,
                                   "OutMsg", EVAckMsgType, CacheParams);
    Updates.clear();

    // S on FwdGetX
    Updates.push_back(new LTSAssignSimple(CacheFwdToCacheExp, FwdGetXMsgInDotRequester));
    CacheEFSM->AddInputTransition("C_S", "C_S_FWD", TrueExp, Updates, "InMsg",
                                  TheLTS->GetNamedType("FwdGetXMsgType'"),
                                  CacheParams);
    Updates.clear();
    Updates.push_back(new LTSAssignSimple(CacheFwdToCacheExp,
                                          TheLTS->MakeVal("clear", CacheIDType)));
    Updates.push_back(new LTSAssignSimple(CacheDataExp,
                                          TheLTS->MakeVal("clear", ValueType)));

    Guard = TheLTS->MakeOp(LTSOps::OpEQ, CacheFwdToCacheExp, CacheParam1);
    CacheEFSM->AddOutputTransitions({ CacheParam1 }, CacheNEQCache1, "C_S_FWD", "C_I",
                                    Guard, Updates, "OutMsg", InvAckMsgType,
                                    { CacheParam, CacheParam1, DirParam, AddressParam },
                                    LTSFairnessType::None, SplatFairnessType::None, "");
    Updates.clear();

    // Transitions from M
    // M on LD
    CacheEFSM->AddInputTransition("C_M", "C_M_LD", TrueExp, Updates, "InMsg", LDMsgType, CacheParams);
    Updates.push_back(new LTSAssignSimple(LDAckMsgOutDotLoadedValue, CacheDataExp));
    CacheEFSM->AddOutputTransition("C_M_LD", "C_M", TrueExp, Updates, "OutMsg",
                                   LDAckMsgType, CacheParams);
    Updates.clear();

    // M on ST
    Updates.push_back(new LTSAssignSimple(CacheDataExp, STMsgInDotValueToStore));
    CacheEFSM->AddInputTransition("C_M", "C_M_ST", TrueExp, Updates, "InMsg", STMsgType, CacheParams);
    Updates.clear();

    Updates.push_back(new LTSAssignSimple(STAckMsgOutDotStoredValue, CacheDataExp));
    CacheEFSM->AddOutputTransition("C_M_ST", "C_M", TrueExp, Updates, "OutMsg",
                                   STAckMsgType, CacheParams);
    Updates.clear();

    // M on EV
    CacheEFSM->AddInputTransition("C_M", "C_M_EV", TrueExp, Updates,
                                  "InMsg", EVMsgType, CacheParams);
    Updates.push_back(new LTSAssignSimple(WBMsgOutDotData, CacheDataExp));
    Updates.push_back(new LTSAssignSimple(CacheDataExp, TheLTS->MakeVal("clear", ValueType)));
    CacheEFSM->AddOutputTransition("C_M_EV", "C_II", TrueExp, Updates, "OutMsg",
                                   WBMsgType, CacheParams);
    Updates.clear();

    // M on FwdGetS
    Updates.push_back(new LTSAssignSimple(CacheFwdToCacheExp, FwdGetSMsgInDotRequester));
    CacheEFSM->AddInputTransition("C_M", "C_M_FWDS", TrueExp, Updates, "InMsg",
                                  TheLTS->GetNamedType("FwdGetSMsgType'"),
                                  CacheParams);
    Updates.clear();

    Updates.push_back(new LTSAssignSimple(DataMsgC2COutDotData, CacheDataExp));
    Updates.push_back(new LTSAssignSimple(CacheFwdToCacheExp,
                                          TheLTS->MakeVal("clear", CacheIDType)));
    Guard = TheLTS->MakeOp(LTSOps::OpEQ, CacheFwdToCacheExp, CacheParam1);
    CacheEFSM->AddOutputTransitions({ CacheParam1 }, CacheNEQCache1, "C_M_FWDS", "C_M_FWDS_C2D",
                                    Guard, Updates, "OutMsg", DataMsgC2CType,
                                    { CacheParam, CacheParam1, DirParam, AddressParam },
                                    LTSFairnessType::None, SplatFairnessType::None, "");
    Updates.clear();

    Updates.push_back(new LTSAssignSimple(DataMsgC2DOutDotData, CacheDataExp));
    CacheEFSM->AddOutputTransition("C_M_FWDS_C2D", "C_S", TrueExp,
                                   Updates, "OutMsg", DataMsgC2DType, CacheParams);

    Updates.clear();

    // M on FwdGetX
    Updates.push_back(new LTSAssignSimple(CacheFwdToCacheExp, FwdGetXMsgInDotRequester));
    CacheEFSM->AddInputTransition("C_M", "C_M_FWDX", TrueExp, Updates, "InMsg",
                                  TheLTS->GetNamedType("FwdGetXMsgType'"),
                                  CacheParams);
    Updates.clear();

    Updates.push_back(new LTSAssignSimple(DataMsgC2COutDotData, CacheDataExp));
    Updates.push_back(new LTSAssignSimple(CacheFwdToCacheExp,
                                          TheLTS->MakeVal("clear", CacheIDType)));
    Updates.push_back(new LTSAssignSimple(CacheDataExp,
                                          TheLTS->MakeVal("clear", ValueType)));

    Guard = TheLTS->MakeOp(LTSOps::OpEQ, CacheFwdToCacheExp, CacheParam1);
    CacheEFSM->AddOutputTransitions({ CacheParam1 }, CacheNEQCache1, "C_M_FWDX", "C_I", Guard, Updates,
                                    "OutMsg", DataMsgC2CType,
                                    { CacheParam, CacheParam1, DirParam, AddressParam },
                                    LTSFairnessType::None, SplatFairnessType::None, "");

    Updates.clear();

    // Transitions from C_IM
    // C_IM on FwdGetX
    Updates.push_back(new LTSAssignSimple(CacheFwdToCacheExp, FwdGetXMsgInDotRequester));
    CacheEFSM->AddInputTransition("C_IM", "C_IM_FWD", TrueExp, Updates, "InMsg",
                                  TheLTS->GetNamedType("FwdGetXMsgType'"),
                                  CacheParams);
    Updates.clear();

    Updates.push_back(new LTSAssignSimple(CacheFwdToCacheExp,
                                          TheLTS->MakeVal("clear", CacheIDType)));
    Guard = TheLTS->MakeOp(LTSOps::OpEQ, CacheFwdToCacheExp, CacheParam1);
    CacheEFSM->AddOutputTransitions({ CacheParam1 }, CacheNEQCache1, "C_IM_FWD", "C_IM",
                                    Guard, Updates, "OutMsg", InvAckMsgType,
                                    { CacheParam, CacheParam1, DirParam, AddressParam },
                                    LTSFairnessType::None, SplatFairnessType::None, "");
    Updates.clear();

    // C_IM on DataMsgC2C'
    Updates.push_back(new LTSAssignSimple(CacheDataExp, CachePendingWriteExp));
    Updates.push_back(new LTSAssignSimple(CachePendingWriteExp,
                                          TheLTS->MakeVal("clear", ValueType)));
    CacheEFSM->AddInputTransitions({ CacheParam1 }, CacheNEQCache1, "C_IM", "C_M_UNBLOCK",
                                   TrueExp, Updates, "InMsg",
                                   TheLTS->GetNamedType("DataMsgC2CType'"),
                                   { CacheParam1, CacheParam, DirParam, AddressParam });
    Updates.clear();

    // C_IM on DataMsgD2C'
    Updates.push_back(new LTSAssignSimple(CacheDataExp, CachePendingWriteExp));
    Updates.push_back(new LTSAssignSimple(CachePendingWriteExp,
                                          TheLTS->MakeVal("clear", ValueType)));
    Updates.push_back(new LTSAssignSimple(CacheAckCountExp,
                                          TheLTS->MakeOp(LTSOps::OpSUB,
                                                         CacheAckCountExp,
                                                         DataMsgD2CInDotNumAcks)));
    CacheEFSM->AddInputTransition("C_IM", "C_SM_DECIDE", TrueExp,
                                  Updates, "InMsg",
                                  TheLTS->GetNamedType("DataMsgD2CType'"),
                                  CacheParams);
    Updates.clear();

    // C_IM on InvAckMsg'
    Updates.push_back(new LTSAssignSimple(CacheAckCountExp,
                                          TheLTS->MakeOp(LTSOps::OpADD,
                                                         CacheAckCountExp,
                                                         TheLTS->MakeVal("1", AckType))));
    CacheEFSM->AddInputTransitions({ CacheParam1 }, CacheNEQCache1, "C_IM", "C_IM", TrueExp, Updates,
                                   "InMsg", TheLTS->GetNamedType("InvAckMsgType'"),
                                   { CacheParam1, CacheParam, DirParam, AddressParam });

    // C_M_UNBLOCK
    Updates.clear();
    CacheEFSM->AddOutputTransition("C_M_UNBLOCK", "C_M_SENDACK", TrueExp,
                                   Updates, "OutMsg", UnblockXMsgType,
                                   CacheParams);

    // C_M_SENDACK
    Updates.push_back(new LTSAssignSimple(STAckMsgOutDotStoredValue, CacheDataExp));
    CacheEFSM->AddOutputTransition("C_M_SENDACK", "C_M", TrueExp, Updates,
                                   "OutMsg", STAckMsgType, CacheParams);
    Updates.clear();

    // Transitions on C_SM
    // C_SM on FwdGetX'
    Updates.push_back(new LTSAssignSimple(CacheFwdToCacheExp, FwdGetXMsgInDotRequester));
    CacheEFSM->AddInputTransition("C_SM", "C_SM_FWD", TrueExp, Updates, "InMsg",
                                  TheLTS->GetNamedType("FwdGetXMsgType'"), CacheParams);
    Updates.clear();
    Updates.push_back(new LTSAssignSimple(CacheFwdToCacheExp,
                                          TheLTS->MakeVal("clear", CacheIDType)));
    Updates.push_back(new LTSAssignSimple(CacheDataExp, TheLTS->MakeVal("clear", ValueType)));
    Guard = TheLTS->MakeOp(LTSOps::OpEQ, CacheFwdToCacheExp, CacheParam1);
    CacheEFSM->AddOutputTransitions({ CacheParam1 }, CacheNEQCache1, "C_SM_FWD", "C_IM", Guard,
                                    Updates, "OutMsg", InvAckMsgType,
                                    { CacheParam, CacheParam1, DirParam, AddressParam },
                                    LTSFairnessType::None, SplatFairnessType::None, "");
    Updates.clear();

    // C_SM on DataMsgD2C'
    Updates.push_back(new LTSAssignSimple(CacheDataExp, CachePendingWriteExp));
    Updates.push_back(new LTSAssignSimple(CacheAckCountExp,
                                          TheLTS->MakeOp(LTSOps::OpSUB, CacheAckCountExp,
                                                         DataMsgD2CInDotNumAcks)));
    Updates.push_back(new LTSAssignSimple(CachePendingWriteExp,
                                          TheLTS->MakeVal("clear", ValueType)));

    CacheEFSM->AddInputTransition("C_SM", "C_SM_DECIDE", TrueExp, Updates,
                                  "InMsg", TheLTS->GetNamedType("DataMsgD2CType'"),
                                  CacheParams);
    Updates.clear();

    // C_SM on InvAck'
    Updates.push_back(new LTSAssignSimple(CacheAckCountExp,
                                          TheLTS->MakeOp(LTSOps::OpADD, CacheAckCountExp,
                                                         TheLTS->MakeVal("1", AckType))));

    CacheEFSM->AddInputTransitions({ CacheParam1 }, CacheNEQCache1, "C_SM", "C_SM_DECIDE",
                                   TrueExp, Updates, "InMsg",
                                   TheLTS->GetNamedType("InvAckMsgType'"),
                                   { CacheParam1, CacheParam, DirParam, AddressParam });
    Updates.clear();

    // Transitions on C_SM_DECIDE
    Guard = TheLTS->MakeOp(LTSOps::OpEQ, CacheAckCountExp,
                           TheLTS->MakeVal("0", AckType));
    CacheEFSM->AddInternalTransition("C_SM_DECIDE", "C_M_UNBLOCK", Guard, Updates);
    Guard = TheLTS->MakeOp(LTSOps::OpNOT, Guard);
    CacheEFSM->AddInternalTransition("C_SM_DECIDE", "C_SM", Guard, Updates);
    Updates.clear();

    // Transitions from C_IS
    // C_IS on FwdGetX'
    Updates.push_back(new LTSAssignSimple(CacheFwdToCacheExp, FwdGetXMsgInDotRequester));
    CacheEFSM->AddInputTransition("C_IS", "C_IS_FWD", TrueExp, Updates,
                                  "InMsg", TheLTS->GetNamedType("FwdGetXMsgType'"),
                                  CacheParams);
    Updates.clear();

    Updates.push_back(new LTSAssignSimple(CacheFwdToCacheExp,
                                          TheLTS->MakeVal("clear", CacheIDType)));
    Guard = TheLTS->MakeOp(LTSOps::OpEQ, CacheFwdToCacheExp, CacheParam1);
    CacheEFSM->AddOutputTransitions({ CacheParam1 }, CacheNEQCache1,
                                    "C_IS_FWD", "C_IS", Guard, Updates,
                                    "OutMsg", InvAckMsgType,
                                    { CacheParam, CacheParam1, DirParam, AddressParam },
                                    LTSFairnessType::None, SplatFairnessType::None, "");
    Updates.clear();

    // C_IS on DataMsgD2C'
    Updates.push_back(new LTSAssignSimple(CacheDataExp, DataMsgD2CInDotData));
    CacheEFSM->AddInputTransition("C_IS", "C_IS_UNBLOCK", TrueExp, Updates,
                                  "InMsg", TheLTS->GetNamedType("DataMsgD2CType'"),
                                  CacheParams);
    Updates.clear();

    CacheEFSM->AddOutputTransition("C_IS_UNBLOCK", "C_IS_DONE", TrueExp, Updates,
                                   "OutMsg", UnblockSMsgType, CacheParams);

    Updates.push_back(new LTSAssignSimple(LDAckMsgOutDotLoadedValue, CacheDataExp));
    CacheEFSM->AddOutputTransition("C_IS_DONE", "C_S", TrueExp,
                                   Updates, "OutMsg", LDAckMsgType, CacheParams);
    Updates.clear();

    // C_IS on DataMsgC2C'
    Updates.push_back(new LTSAssignSimple(CacheDataExp, DataMsgC2CInDotData));
    CacheEFSM->AddInputTransitions({ CacheParam1 }, CacheNEQCache1,
                                   "C_IS", "C_IS_UNBLOCK", TrueExp, Updates,
                                   "InMsg", TheLTS->GetNamedType("DataMsgC2CType'"),
                                   { CacheParam1, CacheParam, DirParam, AddressParam });
    Updates.clear();

    // C_II on WBAckMsg'
    CacheEFSM->AddInputTransition("C_II", "C_II_SENDACK", TrueExp, Updates, "InMsg",
                                  TheLTS->GetNamedType("WBAckMsgType'"), CacheParams);

    // C_II on FwdGetXMsg'
    CacheEFSM->AddInputTransition("C_II", "C_II_SENDACK", TrueExp, Updates, "InMsg",
                                  TheLTS->GetNamedType("FwdGetXMsgType'"), CacheParams);

    // Updates.push_back(new LTSAssignSimple(CacheFwdToCacheExp,
    //                                       FwdGetXMsgInDotRequester));
    // CacheEFSM->AddInputTransition("C_II", "C_II_FWD", TrueExp, Updates, "InMsg",
    //                               TheLTS->GetNamedType("FwdGetXMsgType'"), CacheParams);
    // Updates.clear();

    // Updates.push_back(new LTSAssignSimple(CacheFwdToCacheExp,
    //                                       TheLTS->MakeVal("clear", CacheIDType)));
    // Guard = TheLTS->MakeOp(LTSOps::OpEQ, CacheFwdToCacheExp, CacheParam1);
    // CacheEFSM->AddOutputTransitions({ CacheParam1 }, CacheNEQCache1,
    //                                 "C_II_FWD", "C_I", Guard, Updates,
    //                                 "OutMsg", InvAckMsgType,
    //                                 { CacheParam, CacheParam1, DirParam, AddressParam },
    //                                 LTSFairnessType::None, SplatFairnessType::None, "");
    Updates.clear();

    // C_II on FwdGetSMsg'
    CacheEFSM->AddInputTransition("C_II", "C_II_SENDACK", TrueExp, Updates, "InMsg",
                                  TheLTS->GetNamedType("FwdGetSMsgType'"), CacheParams);

    // Candidate for completion
    // CacheEFSM->AddOutputTransition("C_II_SENDACK", "C_I", TrueExp, Updates,
    //                                "OutMsg", EVAckMsgType, CacheParams);

    auto CacheAsInc = CacheEFSM->SAs<IncompleteEFSM>();
    CacheAsInc->MarkAllStatesComplete();
    CacheAsInc->MarkStateIncomplete("C_S_ST");
    CacheAsInc->IgnoreAllMsgsOnState("C_S_ST");
    CacheAsInc->HandleMsgOnState(GetXMsgDecl, "C_S_ST");
    CacheAsInc->MarkStateIncomplete("C_II_SENDACK");
    CacheAsInc->IgnoreAllMsgsOnState("C_II_SENDACK");
    CacheAsInc->HandleMsgOnState(EVAckMsgDecl, "C_II_SENDACK");

    // Done!
    CacheEFSM->Freeze();

    // The directory now
    vector<ExpT> DirParams = { DirParam, AddressParam };
    auto DirEFSM = TheLTS->MakeEFSM<IncompleteEFSM>("Directory", DirParams, TrueExp,
                                                    LTSFairnessType::Strong);
    DirEFSM->AddState("D_I");
    DirEFSM->AddState("D_I_GETX");
    DirEFSM->AddState("D_I_GETS");

    DirEFSM->AddState("D_S");
    DirEFSM->AddState("D_S_GETX");
    DirEFSM->AddState("D_S_GETX_INV");
    DirEFSM->AddState("D_S_GETS");

    DirEFSM->AddState("D_M");
    DirEFSM->AddState("D_M_GETS");
    DirEFSM->AddState("D_M_GETX");

    DirEFSM->AddState("D_M_WB");

    DirEFSM->AddState("D_BUSY");
    DirEFSM->AddState("D_BUSY_WB");

    DirEFSM->AddState("D_BUSY_DATA");

    DirEFSM->AddState("D_PENDING_UNBLOCK_E");

    DirEFSM->FreezeStates();

    auto SharerArrayType = TheLTS->MakeArrayType(CacheIDType, TheLTS->MakeBoolType());
    auto NumSharersType = TheLTS->MakeRangeType(0, NumCaches);

    DirEFSM->AddVariable("Data", ValueType);
    DirEFSM->AddVariable("ActiveID", CacheIDType);
    DirEFSM->AddVariable("Sharers", SharerArrayType);
    DirEFSM->AddVariable("NumSharers", NumSharersType);
    DirEFSM->AddVariable("Owner", CacheIDType);

    auto DirDataExp = TheLTS->MakeVar("Data", ValueType);
    auto DirActiveIDExp = TheLTS->MakeVar("ActiveID", CacheIDType);
    auto DirSharersExp = TheLTS->MakeVar("Sharers", SharerArrayType);
    auto DirNumSharersExp = TheLTS->MakeVar("NumSharers", NumSharersType);
    auto DirOwnerExp = TheLTS->MakeVar("Owner", CacheIDType);

    DirEFSM->FreezeVars();

    DirEFSM->AddInputMsgs({ CacheParam }, TrueExp,
                          TheLTS->GetNamedType("GetXMsgType'"), CacheParams);
    DirEFSM->AddInputMsgs({ CacheParam }, TrueExp,
                          TheLTS->GetNamedType("GetSMsgType'"), CacheParams);
    DirEFSM->AddInputMsgs({ CacheParam }, TrueExp,
                          TheLTS->GetNamedType("WBMsgType'"), CacheParams);

    DirEFSM->AddInputMsgs({ CacheParam }, TrueExp,
                          TheLTS->GetNamedType("UnblockSMsgType'"), CacheParams);
    DirEFSM->AddInputMsgs({ CacheParam }, TrueExp,
                          TheLTS->GetNamedType("UnblockXMsgType'"), CacheParams);

    DirEFSM->AddInputMsgs({ CacheParam }, TrueExp,
                          TheLTS->GetNamedType("DataMsgC2DType'"),
                          { CacheParam, DirParam, AddressParam });

    DirEFSM->AddOutputMsgs({ CacheParam }, TrueExp, FwdGetXMsgType, CacheParams);
    DirEFSM->AddOutputMsgs({ CacheParam }, TrueExp, FwdGetSMsgType, CacheParams);
    DirEFSM->AddOutputMsgs({ CacheParam }, TrueExp, DataMsgD2CType, CacheParams);
    auto WBAckMsgDecl = DirEFSM->AddOutputMsgs({ CacheParam }, TrueExp, WBAckMsgType, CacheParams);

    // Transitions on D_I
    // GetX on D_I
    Updates.clear();
    Updates.push_back(new LTSAssignSimple(DirActiveIDExp, CacheParam));
    DirEFSM->AddInputTransitions({ CacheParam }, TrueExp, "D_I", "D_I_GETX",
                                 TrueExp, Updates, "InMsg",
                                 TheLTS->GetNamedType("GetXMsgType'"), CacheParams);
    Updates.clear();

    Guard = TheLTS->MakeOp(LTSOps::OpEQ, DirActiveIDExp, CacheParam);
    Updates.push_back(new LTSAssignSimple(DataMsgD2COutDotData, DirDataExp));
    Updates.push_back(new LTSAssignSimple(DataMsgD2COutDotNumAcks,
                                          TheLTS->MakeVal("0", AckType)));
    DirEFSM->AddOutputTransitions({ CacheParam }, TrueExp,
                                  "D_I_GETX", "D_BUSY", Guard,
                                  Updates, "OutMsg", DataMsgD2CType,
                                  CacheParams, LTSFairnessType::None,
                                  SplatFairnessType::None, "");
    Updates.clear();

    // GetS on D_I
    Updates.clear();
    Updates.push_back(new LTSAssignSimple(DirActiveIDExp, CacheParam));
    DirEFSM->AddInputTransitions({ CacheParam }, TrueExp, "D_I", "D_I_GETS",
                                 TrueExp, Updates, "InMsg",
                                 TheLTS->GetNamedType("GetSMsgType'"), CacheParams);
    Updates.clear();

    Guard = TheLTS->MakeOp(LTSOps::OpEQ, DirActiveIDExp, CacheParam);
    Updates.push_back(new LTSAssignSimple(DataMsgD2COutDotData, DirDataExp));
    Updates.push_back(new LTSAssignSimple(DataMsgD2COutDotNumAcks,
                                          TheLTS->MakeVal("0", AckType)));
    DirEFSM->AddOutputTransitions({ CacheParam }, TrueExp, "D_I_GETS",
                                  "D_BUSY", Guard, Updates, "OutMsg",
                                  DataMsgD2CType, CacheParams,
                                  LTSFairnessType::None, SplatFairnessType::None, "");
    Updates.clear();

    // Transitions on D_S
    // GetX on D_S
    Updates.push_back(new LTSAssignSimple(DirActiveIDExp, CacheParam));
    DirEFSM->AddInputTransitions({ CacheParam }, TrueExp, "D_S", "D_S_GETX",
                                 TrueExp, Updates, "InMsg",
                                 TheLTS->GetNamedType("GetXMsgType'"), CacheParams);
    Updates.clear();
    // Send out the data
    Guard = TheLTS->MakeOp(LTSOps::OpIndex, DirSharersExp, DirActiveIDExp);
    Guard = TheLTS->MakeOp(LTSOps::OpAND, Guard,
                           TheLTS->MakeOp(LTSOps::OpEQ, CacheParam,
                                          DirActiveIDExp));
    Updates.push_back(new LTSAssignSimple(DataMsgD2COutDotData, DirDataExp));
    Updates.push_back(new LTSAssignSimple(DataMsgD2COutDotNumAcks,
                                          TheLTS->MakeOp(LTSOps::OpSUB, DirNumSharersExp,
                                                         TheLTS->MakeVal("1", NumSharersType))));

    DirEFSM->AddOutputTransitions({ CacheParam }, TrueExp, "D_S_GETX",
                                  "D_S_GETX_INV", Guard, Updates, "OutMsg",
                                  DataMsgD2CType, CacheParams,
                                  LTSFairnessType::None, SplatFairnessType::None, "");
    Updates.clear();
    Guard = TheLTS->MakeOp(LTSOps::OpIndex, DirSharersExp, DirActiveIDExp);
    Guard = TheLTS->MakeOp(LTSOps::OpNOT, Guard);
    Guard = TheLTS->MakeOp(LTSOps::OpAND, Guard,
                           TheLTS->MakeOp(LTSOps::OpEQ, CacheParam,
                                          DirActiveIDExp));

    Updates.push_back(new LTSAssignSimple(DataMsgD2COutDotData, DirDataExp));
    Updates.push_back(new LTSAssignSimple(DataMsgD2COutDotNumAcks, DirNumSharersExp));

    DirEFSM->AddOutputTransitions({ CacheParam }, TrueExp, "D_S_GETX",
                                  "D_S_GETX_INV", Guard, Updates, "OutMsg",
                                  DataMsgD2CType, CacheParams,
                                  LTSFairnessType::None, SplatFairnessType::None, "");
    Updates.clear();

    // Send out the invalidations
    auto BoundCacheVar = TheLTS->MakeBoundVar(0, CacheIDType);
    auto SharersOfBoundCacheVarExp = TheLTS->MakeOp(LTSOps::OpIndex, DirSharersExp, BoundCacheVar);
    Guard = TheLTS->MakeExists({ CacheIDType },
                               TheLTS->MakeOp(LTSOps::OpAND,
                                              TheLTS->MakeOp(LTSOps::OpNOT,
                                                             TheLTS->MakeOp(LTSOps::OpEQ,
                                                                            DirActiveIDExp,
                                                                            BoundCacheVar)),
                                              SharersOfBoundCacheVarExp));

    Guard = TheLTS->MakeOp(LTSOps::OpNOT, Guard);
    DirEFSM->AddInternalTransition("D_S_GETX_INV", "D_BUSY", Guard, Updates);

    // case of pending invalidations
    Guard = TheLTS->MakeOp(LTSOps::OpAND,
                           TheLTS->MakeOp(LTSOps::OpIndex, DirSharersExp, CacheParam),
                           TheLTS->MakeOp(LTSOps::OpNOT,
                                          TheLTS->MakeOp(LTSOps::OpEQ,
                                                         CacheParam,
                                                         DirActiveIDExp)));

    Updates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpIndex,
                                                         DirSharersExp,
                                                         CacheParam),
                                          TheLTS->MakeFalse()));
    Updates.push_back(new LTSAssignSimple(FwdGetXMsgOutDotRequester, DirActiveIDExp));
    DirEFSM->AddOutputTransitions({ CacheParam }, TrueExp, "D_S_GETX_INV",
                                  "D_S_GETX_INV", Guard, Updates, "OutMsg",
                                  FwdGetXMsgType,
                                  { CacheParam, DirParam, AddressParam },
                                  LTSFairnessType::None, SplatFairnessType::None, "");
    Updates.clear();

    // GetS on D_S
    Updates.push_back(new LTSAssignSimple(DirActiveIDExp, CacheParam));
    DirEFSM->AddInputTransitions({ CacheParam }, TrueExp, "D_S", "D_S_GETS",
                                 TrueExp, Updates, "InMsg",
                                 TheLTS->GetNamedType("GetSMsgType'"),
                                 { CacheParam, DirParam, AddressParam });
    Updates.clear();
    Updates.push_back(new LTSAssignSimple(DataMsgD2COutDotData, DirDataExp));
    Updates.push_back(new LTSAssignSimple(DataMsgD2COutDotNumAcks,
                                          TheLTS->MakeVal("0", AckType)));
    Guard = TheLTS->MakeOp(LTSOps::OpEQ, CacheParam, DirActiveIDExp);
    DirEFSM->AddOutputTransitions({ CacheParam }, TrueExp, "D_S_GETS",
                                  "D_BUSY", Guard, Updates,
                                  "OutMsg", DataMsgD2CType, CacheParams,
                                  LTSFairnessType::None, SplatFairnessType::None, "");
    Updates.clear();

    // Transitions on D_M
    // GetX on D_M
    Updates.push_back(new LTSAssignSimple(DirActiveIDExp, CacheParam));
    DirEFSM->AddInputTransitions({ CacheParam }, TrueExp, "D_M", "D_M_GETX", TrueExp,
                                 Updates, "InMsg", TheLTS->GetNamedType("GetXMsgType'"),
                                 { CacheParam, DirParam, AddressParam });
    Updates.clear();

    Guard = TheLTS->MakeOp(LTSOps::OpEQ, CacheParam, DirOwnerExp);
    Updates.push_back(new LTSAssignSimple(FwdGetXMsgOutDotRequester, DirActiveIDExp));
    DirEFSM->AddOutputTransitions({ CacheParam }, TrueExp, "D_M_GETX", "D_BUSY",
                                  Guard, Updates, "OutMsg", FwdGetXMsgType,
                                  CacheParams, LTSFairnessType::None,
                                  SplatFairnessType::None, "");
    Updates.clear();

    // GetS on D_M
    Updates.push_back(new LTSAssignSimple(DirActiveIDExp, CacheParam));
    DirEFSM->AddInputTransitions({ CacheParam }, TrueExp, "D_M", "D_M_GETS", TrueExp,
                                 Updates, "InMsg", TheLTS->GetNamedType("GetSMsgType'"),
                                 { CacheParam, DirParam, AddressParam });
    Updates.clear();

    Guard = TheLTS->MakeOp(LTSOps::OpEQ, CacheParam, DirOwnerExp);
    Updates.push_back(new LTSAssignSimple(FwdGetSMsgOutDotRequester, DirActiveIDExp));
    DirEFSM->AddOutputTransitions({ CacheParam }, TrueExp, "D_M_GETS", "D_BUSY_DATA",
                                  Guard, Updates, "OutMsg", FwdGetSMsgType,
                                  CacheParams, LTSFairnessType::None,
                                  SplatFairnessType::None, "");
    Updates.clear();

    // WB on D_M
    Updates.push_back(new LTSAssignSimple(DirDataExp, WBMsgInDotData));
    Updates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpIndex, DirSharersExp,
                                                         CacheParam),
                                          TheLTS->MakeFalse()));
    Updates.push_back(new LTSAssignSimple(DirActiveIDExp, CacheParam));

    DirEFSM->AddInputTransitions({ CacheParam }, TrueExp, "D_M",
                                 "D_M_WB", TrueExp, Updates, "InMsg",
                                 TheLTS->GetNamedType("WBMsgType'"), CacheParams);
    Updates.clear();

    Guard = TheLTS->MakeOp(LTSOps::OpEQ, DirActiveIDExp, CacheParam);
    Updates.push_back(new LTSAssignSimple(DirNumSharersExp,
                                          TheLTS->MakeVal("0", NumSharersType)));
    Updates.push_back(new LTSAssignParam({ CacheParam2 }, TrueExp,
                                         TheLTS->MakeOp(LTSOps::OpIndex,
                                                        DirSharersExp, CacheParam2),
                                         TheLTS->MakeFalse()));
    Updates.push_back(new LTSAssignSimple(DirActiveIDExp,
                                          TheLTS->MakeVal("clear", CacheIDType)));
    Updates.push_back(new LTSAssignSimple(DirOwnerExp,
                                          TheLTS->MakeVal("clear", CacheIDType)));

    // DirEFSM->AddOutputTransitions({ CacheParam }, TrueExp, "D_M_WB", "D_I",
    //                               Guard, Updates, "OutMsg", WBAckMsgType,
    //                               CacheParams, LTSFairnessType::None,
    //                               SplatFairnessType::None, "");
    Updates.clear();

    // Transitions from BUSY
    // WB on BUSY
    Updates.push_back(new LTSAssignSimple(DirDataExp, WBMsgInDotData));
    Guard = TheLTS->MakeOp(LTSOps::OpEQ, CacheParam, DirActiveIDExp);

    DirEFSM->AddInputTransitions({ CacheParam }, TrueExp, "D_BUSY",
                                 "D_PENDING_UNBLOCK_E", Guard, Updates,
                                 "InMsg",
                                 TheLTS->GetNamedType("WBMsgType'"),
                                 CacheParams);
    Guard = TheLTS->MakeOp(LTSOps::OpNOT, Guard);
    Updates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpIndex,
                                                         DirSharersExp,
                                                         CacheParam),
                                          TheLTS->MakeFalse()));

    DirEFSM->AddInputTransitions({ CacheParam }, TrueExp, "D_BUSY",
                                 "D_BUSY_WB", Guard, Updates,
                                 "InMsg", TheLTS->GetNamedType("WBMsgType'"),
                                 CacheParams);
    Updates.clear();
    Updates.push_back(new LTSAssignSimple(DataMsgD2COutDotData, DirDataExp));
    Updates.push_back(new LTSAssignSimple(DataMsgD2COutDotNumAcks, TheLTS->MakeVal("0", AckType)));
    Updates.push_back(new LTSAssignSimple(DirNumSharersExp, TheLTS->MakeVal("0", NumSharersType)));
    Updates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpIndex, DirSharersExp,
                                                         DirActiveIDExp),
                                          TheLTS->MakeFalse()));
    Guard = TheLTS->MakeOp(LTSOps::OpEQ, DirActiveIDExp, CacheParam);
    DirEFSM->AddOutputTransitions({ CacheParam }, TrueExp, "D_BUSY_WB",
                                  "D_BUSY", Guard, Updates,
                                  "OutMsg", DataMsgD2CType, CacheParams,
                                  LTSFairnessType::None,
                                  SplatFairnessType::None, "");
    Updates.clear();

    // UnblockX on BUSY
    Updates.push_back(new LTSAssignParam({ CacheParam2 }, TrueExp,
                                         TheLTS->MakeOp(LTSOps::OpIndex, DirSharersExp,
                                                        CacheParam2),
                                         TheLTS->MakeFalse()));
    Updates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpIndex, DirSharersExp,
                                                         DirActiveIDExp),
                                          TheLTS->MakeTrue()));
    Updates.push_back(new LTSAssignSimple(DirOwnerExp, DirActiveIDExp));
    Updates.push_back(new LTSAssignSimple(DirNumSharersExp, TheLTS->MakeVal("1", NumSharersType)));
    Updates.push_back(new LTSAssignSimple(DirActiveIDExp, TheLTS->MakeVal("clear", CacheIDType)));
    Updates.push_back(new LTSAssignSimple(DirDataExp, TheLTS->MakeVal("clear", ValueType)));

    DirEFSM->AddInputTransitions({ CacheParam }, TrueExp, "D_BUSY",
                                 "D_M", TrueExp, Updates,
                                 "InMsg", TheLTS->GetNamedType("UnblockXMsgType'"),
                                 CacheParams);
    Updates.clear();

    // UnblockS on BUSY
    Guard = TheLTS->MakeOp(LTSOps::OpIndex, DirSharersExp, CacheParam);
    Updates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpIndex, DirSharersExp,
                                                         DirActiveIDExp),
                                          TheLTS->MakeTrue()));
    Updates.push_back(new LTSAssignSimple(DirOwnerExp,
                                          TheLTS->MakeVal("clear", CacheIDType)));
    Updates.push_back(new LTSAssignSimple(DirActiveIDExp, TheLTS->MakeVal("clear", CacheIDType)));

    DirEFSM->AddInputTransitions({ CacheParam }, TrueExp, "D_BUSY",
                                 "D_S", Guard, Updates,
                                 "InMsg", TheLTS->GetNamedType("UnblockSMsgType'"),
                                 CacheParams);

    Updates.push_back(new LTSAssignSimple(DirNumSharersExp,
                                          TheLTS->MakeOp(LTSOps::OpADD,
                                                         DirNumSharersExp,
                                                         TheLTS->MakeVal("1", NumSharersType))));
    Guard = TheLTS->MakeOp(LTSOps::OpNOT, Guard);
    DirEFSM->AddInputTransitions({ CacheParam }, TrueExp, "D_BUSY",
                                 "D_S", Guard, Updates,
                                 "InMsg", TheLTS->GetNamedType("UnblockSMsgType'"),
                                 CacheParams);

    Updates.clear();

    // DataMsgC2D on BUSY
    Updates.push_back(new LTSAssignSimple(DirDataExp, DataMsgC2DInDotData));
    // Updates.push_back(new LTSAssignSimple(DirActiveIDExp, TheLTS->MakeVal("clear", CacheIDType)));
    DirEFSM->AddInputTransitions({ CacheParam }, TrueExp,
                                 "D_BUSY", "D_S", TrueExp, Updates, "InMsg",
                                 TheLTS->GetNamedType("DataMsgC2DType'"),
                                 { CacheParam, DirParam, AddressParam });
    Updates.clear();

    // Transitions on BUSY_DATA
    // UnblockS on BUSY_DATA
    Updates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpIndex, DirSharersExp,
                                                         CacheParam),
                                          TheLTS->MakeTrue()));
    Updates.push_back(new LTSAssignSimple(DirNumSharersExp,
                                          TheLTS->MakeOp(LTSOps::OpADD,
                                                         DirNumSharersExp,
                                                         TheLTS->MakeVal("1", NumSharersType))));
    DirEFSM->AddInputTransitions({ CacheParam }, TrueExp, "D_BUSY_DATA", "D_BUSY",
                                 TrueExp, Updates, "InMsg",
                                 TheLTS->GetNamedType("UnblockSMsgType'"),
                                 { CacheParam, DirParam, AddressParam });
    Updates.clear();

    // audupa: changed to c2d 10/06
    Updates.push_back(new LTSAssignSimple(DirDataExp, DataMsgC2DInDotData));
    DirEFSM->AddInputTransitions({ CacheParam }, TrueExp,
                                 "D_BUSY_DATA", "D_BUSY", TrueExp, Updates,
                                 "InMsg",
                                 TheLTS->GetNamedType("DataMsgC2DType'"),
                                 { CacheParam, DirParam, AddressParam });
    Updates.clear();

    // WBMsg on BUSY_DATA
    Guard = TheLTS->MakeOp(LTSOps::OpEQ, CacheParam, DirActiveIDExp);
    Updates.push_back(new LTSAssignSimple(DirDataExp, WBMsgInDotData));
    DirEFSM->AddInputTransitions({ CacheParam }, TrueExp, "D_BUSY_DATA",
                                 "D_PENDING_UNBLOCK_E", Guard, Updates,
                                 "InMsg",
                                 TheLTS->GetNamedType("WBMsgType'"),
                                 { CacheParam, DirParam, AddressParam });

    Guard = TheLTS->MakeOp(LTSOps::OpNOT, Guard);
    Updates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpIndex, DirSharersExp,
                                                         CacheParam),
                                          TheLTS->MakeFalse()));

    DirEFSM->AddInputTransitions({ CacheParam }, TrueExp, "D_BUSY_DATA", "D_BUSY_WB",
                                 Guard, Updates, "InMsg",
                                 TheLTS->GetNamedType("WBMsgType'"),
                                 { CacheParam, DirParam, AddressParam });
    Updates.clear();

    // UnblockEMsg on PENDING_UNBLOCK_E
    Updates.push_back(new LTSAssignParam({ CacheParam2 }, TrueExp,
                                         TheLTS->MakeOp(LTSOps::OpIndex, DirSharersExp,
                                                        CacheParam2),
                                         TheLTS->MakeFalse()));
    Updates.push_back(new LTSAssignSimple(DirOwnerExp,
                                          TheLTS->MakeVal("clear", CacheIDType)));
    Updates.push_back(new LTSAssignSimple(DirNumSharersExp,
                                          TheLTS->MakeVal("0", NumSharersType)));

    // Updates.push_back(new LTSAssignSimple(DirActiveIDExp, TheLTS->MakeVal("clear", CacheIDType)));
    // Move to D_M_WB, because we still need to send a an ack
    DirEFSM->AddInputTransitions({ CacheParam }, TrueExp, "D_PENDING_UNBLOCK_E",
                                 "D_M_WB", TrueExp, Updates, "InMsg",
                                 TheLTS->GetNamedType("UnblockXMsgType'"),
                                 { CacheParam, DirParam, AddressParam });
    Updates.clear();

    auto DirAsInc = DirEFSM->SAs<IncompleteEFSM>();
    DirAsInc->MarkAllStatesComplete();
    DirAsInc->MarkStateIncomplete("D_M_WB");
    DirAsInc->IgnoreAllMsgsOnState("D_M_WB");
    DirAsInc->HandleMsgOnState(WBAckMsgDecl, "D_M_WB");

    DirEFSM->Freeze();

    TheLTS->FreezeAutomata();

    // The initial states
    vector<InitStateRef> InitStates;
    vector<LTSAssignRef> InitUpdates;

    auto CacheType = TheLTS->GetEFSMType("Cache");
    auto DirType = TheLTS->GetEFSMType("Directory");
    auto EnvType = TheLTS->GetEFSMType("Environment");
    auto CMType = TheLTS->GetEFSMType("CoherenceMonitor");

    auto CacheStateVar = TheLTS->MakeOp(LTSOps::OpIndex,
                                        TheLTS->MakeVar("Cache", CacheType),
                                        CacheParam);
    CacheStateVar = TheLTS->MakeOp(LTSOps::OpIndex, CacheStateVar, DirParam);
    CacheStateVar = TheLTS->MakeOp(LTSOps::OpIndex, CacheStateVar, AddressParam);

    auto DirStateVar = TheLTS->MakeOp(LTSOps::OpIndex,
                                      TheLTS->MakeVar("Directory", DirType),
                                      DirParam);
    DirStateVar = TheLTS->MakeOp(LTSOps::OpIndex, DirStateVar, AddressParam);

    auto EnvStateVar = TheLTS->MakeOp(LTSOps::OpIndex,
                                      TheLTS->MakeVar("Environment", EnvType),
                                      CacheParam);
    EnvStateVar = TheLTS->MakeOp(LTSOps::OpIndex, EnvStateVar, DirParam);
    EnvStateVar = TheLTS->MakeOp(LTSOps::OpIndex, EnvStateVar, AddressParam);

    auto CMStateVar = TheLTS->MakeOp(LTSOps::OpIndex,
                                     TheLTS->MakeVar("CoherenceMonitor", CMType),
                                     DirParam);
    CMStateVar = TheLTS->MakeOp(LTSOps::OpIndex, CMStateVar, AddressParam);

    auto CMDotState = TheLTS->MakeOp(LTSOps::OpField, CMStateVar,
                                     TheLTS->MakeVar("state", FAType));
    auto CMDotLastSeenValue = TheLTS->MakeOp(LTSOps::OpField, CMStateVar,
                                             TheLTS->MakeVar("LastSeenValue", FAType));
    auto CMDotActualLastValue = TheLTS->MakeOp(LTSOps::OpField, CMStateVar,
                                               TheLTS->MakeVar("ActualLastValue", FAType));

    auto EnvDotState = TheLTS->MakeOp(LTSOps::OpField, EnvStateVar,
                                      TheLTS->MakeVar("state", FAType));
    auto EnvDotPendingStoreValue = TheLTS->MakeOp(LTSOps::OpField, EnvStateVar,
                                                  TheLTS->MakeVar("PendingStoreValue", FAType));
    auto EnvDotLastSeenStoreValue = TheLTS->MakeOp(LTSOps::OpField, EnvStateVar,
                                                   TheLTS->MakeVar("LastSeenStoreValue", FAType));


    auto CacheDotState = TheLTS->MakeOp(LTSOps::OpField, CacheStateVar,
                                        TheLTS->MakeVar("state", FAType));
    auto CacheDotData = TheLTS->MakeOp(LTSOps::OpField, CacheStateVar,
                                       TheLTS->MakeVar("Data", FAType));
    auto CacheDotAckCounter = TheLTS->MakeOp(LTSOps::OpField, CacheStateVar,
                                             TheLTS->MakeVar("AckCounter", FAType));
    auto CacheDotPendingWrite = TheLTS->MakeOp(LTSOps::OpField, CacheStateVar,
                                               TheLTS->MakeVar("PendingWrite", FAType));
    auto CacheDotFwdToCache = TheLTS->MakeOp(LTSOps::OpField, CacheStateVar,
                                             TheLTS->MakeVar("FwdToCache", FAType));

    auto DirDotState = TheLTS->MakeOp(LTSOps::OpField, DirStateVar,
                                      TheLTS->MakeVar("state", FAType));
    auto DirDotData = TheLTS->MakeOp(LTSOps::OpField, DirStateVar,
                                     TheLTS->MakeVar("Data", FAType));
    auto DirDotActiveID = TheLTS->MakeOp(LTSOps::OpField, DirStateVar,
                                         TheLTS->MakeVar("ActiveID", FAType));
    auto DirDotSharers = TheLTS->MakeOp(LTSOps::OpField, DirStateVar,
                                        TheLTS->MakeVar("Sharers", FAType));
    auto DirDotNumSharers = TheLTS->MakeOp(LTSOps::OpField, DirStateVar,
                                           TheLTS->MakeVar("NumSharers", FAType));
    auto DirDotOwner = TheLTS->MakeOp(LTSOps::OpField, DirStateVar,
                                      TheLTS->MakeVar("Owner", FAType));

    // Initial state for coherence monitor
    InitUpdates.push_back(new LTSAssignParam({ DirParam, AddressParam }, TrueExp, CMDotState,
                                             TheLTS->MakeVal("InitialState",
                                                             CMDotState->GetType())));
    InitUpdates.push_back(new LTSAssignParam({ DirParam, AddressParam }, TrueExp,
                                             CMDotActualLastValue, ValueParam));
    InitUpdates.push_back(new LTSAssignParam({ DirParam, AddressParam }, TrueExp,
                                             CMDotLastSeenValue,
                                             TheLTS->MakeVal("clear",
                                                             CMDotLastSeenValue->GetType())));
    // Initial state for environment
    InitUpdates.push_back(new LTSAssignParam(CacheParams, TrueExp,
                                             EnvDotState,
                                             TheLTS->MakeVal("InitialState",
                                                              EnvDotState->GetType())));
    InitUpdates.push_back(new LTSAssignParam(CacheParams, TrueExp,
                                             EnvDotPendingStoreValue,
                                             TheLTS->MakeVal("clear", ValueType)));
    InitUpdates.push_back(new LTSAssignParam(CacheParams, TrueExp,
                                             EnvDotLastSeenStoreValue,
                                             TheLTS->MakeVal("clear", ValueType)));

    // Initial state for the caches
    InitUpdates.push_back(new LTSAssignParam(CacheParams, TrueExp,
                                             CacheDotState,
                                             TheLTS->MakeVal("C_I",
                                                             CacheDotState->GetType())));
    InitUpdates.push_back(new LTSAssignParam(CacheParams, TrueExp,
                                             CacheDotData,
                                             TheLTS->MakeVal("clear", ValueType)));
    InitUpdates.push_back(new LTSAssignParam(CacheParams, TrueExp,
                                             CacheDotAckCounter,
                                             TheLTS->MakeVal("0", AckType)));
    InitUpdates.push_back(new LTSAssignParam(CacheParams, TrueExp,
                                             CacheDotPendingWrite,
                                             TheLTS->MakeVal("clear", ValueType)));
    InitUpdates.push_back(new LTSAssignParam(CacheParams, TrueExp,
                                             CacheDotFwdToCache,
                                             TheLTS->MakeVal("clear", CacheIDType)));

    // Initial state for the directory
    InitUpdates.push_back(new LTSAssignParam({ DirParam, AddressParam }, TrueExp,
                                             DirDotState,
                                             TheLTS->MakeVal("D_I", DirDotState->GetType())));
    InitUpdates.push_back(new LTSAssignParam({ DirParam, AddressParam }, TrueExp,
                                             DirDotData, ValueParam));
    InitUpdates.push_back(new LTSAssignParam({ DirParam, AddressParam }, TrueExp,
                                             DirDotActiveID,
                                             TheLTS->MakeVal("clear", CacheIDType)));
    InitUpdates.push_back(new LTSAssignParam({ DirParam, AddressParam }, TrueExp,
                                             DirDotNumSharers,
                                             TheLTS->MakeVal("0", NumSharersType)));
    InitUpdates.push_back(new LTSAssignParam({ DirParam, AddressParam }, TrueExp,
                                             DirDotOwner,
                                             TheLTS->MakeVal("clear", CacheIDType)));
    InitUpdates.push_back(new LTSAssignParam({ CacheParam, DirParam, AddressParam }, TrueExp,
                                             TheLTS->MakeOp(LTSOps::OpIndex,
                                                            DirDotSharers,
                                                            CacheParam),
                                             TheLTS->MakeFalse()));
    InitStates.push_back(new LTSInitState({ ValueParam }, TrueExp, InitUpdates));

    TheLTS->AddInitStates(InitStates);
    TheLTS->Freeze();

    auto const& StateVectorVars = TheLTS->GetStateVectorVars();

    cout << "LTS Vars:" << endl;
    for (auto const& Var : StateVectorVars) {
        cout << Var->ToString() << " : " << endl;
        cout << Var->GetType()->ToString() << endl;
    }

    cout << "State vector size is " << TheLTS->GetStateVectorSize() << " bytes." << endl;

    cout << "Guarded Commands:" << endl;
    auto const& GCmds = TheLTS->GetGuardedCmds();
    for (auto const& GCmd : GCmds) {
        cout << GCmd->ToString() << endl;
    }

    cout << "Initial State Generators:" << endl;
    auto const& InitStateGens = TheLTS->GetInitStateGenerators();
    for (auto const& InitStateGen : InitStateGens) {
        cout << "InitState {" << endl;
        for (auto const& Update : InitStateGen) {
            cout << "    " << Update->ToString() << endl;
        }
        cout << "}" << endl;
    }

    cout << "Invariant:" << endl;
    cout << TheLTS->GetInvariant() << endl;

    // cout << "Channel Buffer variables to sort:" << endl;
    // for (auto const& BufferExp : TheLTS->GetChanBuffersToSort()) {
    //     cout << BufferExp.first->ToString() << endl;
    //     cout << BufferExp.second->ToString() << endl;
    // }

    auto Checker = new LTSChecker(TheLTS);

    // Create the liveness monitors

    auto Monitor = Checker->MakeStateBuchiMonitor("LDLiveness", CacheParams, TrueExp);
    Monitor->AddState("Initial", true, false);
    Monitor->AddState("Accepting", false, true);

    Monitor->FreezeStates();
    auto MonCacheDotState = Monitor->MakeOp(LTSOps::OpIndex,
                                            Monitor->MakeVar("Cache", CacheType),
                                            CacheParam);
    MonCacheDotState = Monitor->MakeOp(LTSOps::OpIndex, MonCacheDotState,
                                       DirParam);
    MonCacheDotState = Monitor->MakeOp(LTSOps::OpIndex, MonCacheDotState,
                                       AddressParam);
    MonCacheDotState = Monitor->MakeOp(LTSOps::OpField, MonCacheDotState,
                                       TheLTS->MakeVar("state", FAType));

    auto MonCacheDotStateEQIS = Monitor->MakeOp(LTSOps::OpEQ, MonCacheDotState,
                                                Monitor->MakeVal("C_IS",
                                                                 MonCacheDotState->GetType()));
    auto MonCacheDotStateEQS = Monitor->MakeOp(LTSOps::OpEQ, MonCacheDotState,
                                               Monitor->MakeVal("C_S",
                                                                MonCacheDotState->GetType()));
    auto MonCacheDotStateNEQS = Monitor->MakeOp(LTSOps::OpNOT, MonCacheDotStateEQS);

    Monitor->AddTransition("Initial", "Initial", TrueExp);
    Monitor->AddTransition("Initial", "Accepting", MonCacheDotStateEQIS);
    Monitor->AddTransition("Accepting", "Accepting", MonCacheDotStateNEQS);
    Monitor->Freeze();


    Monitor = Checker->MakeStateBuchiMonitor("STLiveness", CacheParams, TrueExp);
    Monitor->AddState("Initial", true, false);
    Monitor->AddState("Accepting", false, true);

    Monitor->FreezeStates();
    MonCacheDotState = Monitor->MakeOp(LTSOps::OpIndex,
                                       Monitor->MakeVar("Cache", CacheType),
                                       CacheParam);
    MonCacheDotState = Monitor->MakeOp(LTSOps::OpIndex, MonCacheDotState,
                                       DirParam);
    MonCacheDotState = Monitor->MakeOp(LTSOps::OpIndex, MonCacheDotState,
                                       AddressParam);
    MonCacheDotState = Monitor->MakeOp(LTSOps::OpField, MonCacheDotState,
                                       TheLTS->MakeVar("state", FAType));

    auto MonCacheDotStateEQIM = Monitor->MakeOp(LTSOps::OpEQ, MonCacheDotState,
                                                    Monitor->MakeVal("C_IM",
                                                                     MonCacheDotState->GetType()));
    auto MonCacheDotStateEQSM =  Monitor->MakeOp(LTSOps::OpEQ, MonCacheDotState,
                                                    Monitor->MakeVal("C_SM",
                                                                     MonCacheDotState->GetType()));
    auto MonCacheDotStateEQSMIM = Monitor->MakeOp(LTSOps::OpOR, MonCacheDotStateEQSM,
                                                  MonCacheDotStateEQIM);

    auto MonCacheDotStateNEQM = Monitor->MakeOp(LTSOps::OpEQ, MonCacheDotState,
                                                Monitor->MakeVal("C_M",
                                                                 MonCacheDotState->GetType()));
    MonCacheDotStateNEQM = Monitor->MakeOp(LTSOps::OpNOT, MonCacheDotStateNEQM);

    Monitor->AddTransition("Initial", "Initial", TrueExp);
    Monitor->AddTransition("Initial", "Accepting", MonCacheDotStateEQSMIM);
    Monitor->AddTransition("Accepting", "Accepting", MonCacheDotStateNEQM);
    Monitor->Freeze();

    auto TheSolver = new Solver(Checker);
    TheSolver->Solve();

    delete TheSolver;
    delete Checker;
}

//
// MSIC2DSynth.cpp ends here
