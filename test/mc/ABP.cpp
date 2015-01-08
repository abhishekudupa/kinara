// ABP.cpp ---
//
// Filename: ABP.cpp
// Author: Christos Stergiou
// Created: Tue Sep  10 22:52:00 2014 (-0400)
//
//
// Copyright (c) 2013, Christos Stergiou, University of Pennsylvania
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


#include "../../src/uflts/LabelledTS.hpp"
#include "../../src/uflts/LTSEFSM.hpp"
#include "../../src/uflts/LTSChannelEFSM.hpp"
#include "../../src/uflts/LTSAssign.hpp"
#include "../../src/uflts/LTSTransitions.hpp"
#include "../../src/mc/LTSChecker.hpp"

using namespace ESMC;
using namespace LTS;
using namespace Exprs;
using namespace MC;



int main()
{
    int max_data_value = 5;
    auto TheLTS = new LabelledTS();

    // set<string> EmptyFairnessSets; TODO Find out how to write fairness sets

    // Common types
    auto DataType = TheLTS->MakeRangeType(0, max_data_value);
    auto TagType = TheLTS->MakeRangeType(0, 1);
    auto UnitType = TheLTS->MakeRangeType(0, 0);
    auto FAType = TheLTS->MakeFieldAccessType();

    // Common expressions
    vector<ExpT> EmptyParams;
    auto TrueExp = TheLTS->MakeTrue();
    set<string> EmptyFairnessSets;
    vector<LTSAssignRef> EmptyUpdates;
    auto UnitExp = TheLTS->MakeVal("0", UnitType);

    auto DataFieldExp = TheLTS->MakeVar("Data", FAType);

    // Message Types
    // "send" with data from sender client to sender process, both safety and liveness monitors listen to this
    // "receive" with data from receiver to receiver client, both safety and liveness monitors listen

    vector<pair<string, TypeRef>> DataMsgFields;
    DataMsgFields.push_back(make_pair("Data", DataType));
    auto DataPort = TheLTS->MakeMsgType("DataPort", DataMsgFields, false);
    auto DataPortExp = TheLTS->MakeVar("DataPort", DataPort);

    // Receiver to receiver client channel:
    auto DataReceiverToClientPort = TheLTS->MakeMsgType("DataReceiverToClientPort", DataMsgFields, false);
    auto DataReceiverToClientPortExp = TheLTS->MakeVar("DataReceiverToClientPort", DataReceiverToClientPort);
    auto DataReceiverToClientPortDataFieldExp = TheLTS->MakeOp(
            LTSOps::OpField,
            DataReceiverToClientPortExp,
            DataFieldExp);


    vector<pair<string, TypeRef>> DataTagPortFields;
    DataTagPortFields.push_back(make_pair("Data", DataType));
    DataTagPortFields.push_back(make_pair("Tag", TagType));
    auto DataTagPort = TheLTS->MakeMsgType("DataTagPort", DataTagPortFields, true);
    auto DataTagPortExp = TheLTS->MakeVar("DataTagPort", DataTagPort);

    auto DataTagPortP = TheLTS->GetNamedType("DataTagPort'");


    auto DataTagMsgDataAccExp = TheLTS->MakeOp(LTSOps::OpField, DataTagPortExp, DataFieldExp);
    auto TagAccFieldExp = TheLTS->MakeVar("Tag", FAType);
    auto DataTagMsgTagAccExp = TheLTS->MakeOp(LTSOps::OpField, DataTagPortExp, TagAccFieldExp);
    // This should work for Primed data tag messages too.


    vector<pair<string, TypeRef>> TagMsgFields;
    TagMsgFields.push_back(make_pair("Tag", TagType));
    auto TagPort = TheLTS->MakeMsgType("TagPort", TagMsgFields, true);
    auto TagPortP = TheLTS->GetNamedType("TagPort'");
    auto TagPortExp = TheLTS->MakeVar("TagPort", TagPort);
    auto TagPortPExp = TheLTS->MakeVar("TagPort'", TagPortP);
    auto TagPortTagFieldExp = TheLTS->MakeOp(LTSOps::OpField, TagPortExp, TagAccFieldExp);
    auto TagPortPTagFieldExp = TheLTS->MakeOp(LTSOps::OpField, TagPortPExp, TagAccFieldExp);

    vector<pair<string, TypeRef>> TimeoutFields;
    TimeoutFields.push_back(make_pair("Dummy", UnitType));
    auto TimeOutPort = TheLTS->MakeMsgType("TimeOutPort", TimeoutFields, false);
    auto TimeOutPortExp = TheLTS->MakeVar("TimeOutPort", TimeOutPort);
    auto DummyAccFieldExp = TheLTS->MakeVar("Dummy", FAType);
    auto TimeOutDummyAccExp = TheLTS->MakeOp(LTSOps::OpField, TimeOutPortExp, DummyAccFieldExp);

    TheLTS->FreezeMsgs();



    auto SenderClient = TheLTS->MakeGenEFSM("SenderClient", EmptyParams, TrueExp, LTSFairnessType::None);
    SenderClient->AddState("Initial");
    SenderClient->FreezeStates();
    SenderClient->FreezeVars();
    SenderClient->AddOutputMsg(DataPort, EmptyParams);
    auto DataAccExp = TheLTS->MakeOp(LTSOps::OpField, DataPortExp, DataFieldExp);
    for (auto i = 1; i <= max_data_value; ++i) {
        auto DataVal = TheLTS->MakeVal(to_string(i), DataType);
        vector<LTSAssignRef> DataSendRequestUpdates;
        DataSendRequestUpdates.push_back(new LTSAssignSimple(DataAccExp, DataVal));
        SenderClient->AddOutputTransition("Initial", "Initial", TrueExp, DataSendRequestUpdates, "DataPort", DataPort, EmptyParams, EmptyFairnessSets);
    }


    auto Sender = TheLTS->MakeGenEFSM("Sender", EmptyParams, TrueExp, LTSFairnessType::None);
    Sender->AddState("Initial");
    Sender->AddState("Q0");
    Sender->AddState("Q1");
    Sender->AddState("Q2");
    Sender->FreezeStates();
    Sender->AddVariable("SenderInputMessage", DataType);
    Sender->AddVariable("SenderTag", TagType);
    Sender->AddVariable("AckTag", TagType);
    Sender->FreezeVars();
    Sender->AddInputMsg(DataPort, EmptyParams);
    Sender->AddInputMsg(TagPortP, EmptyParams);
    Sender->AddInputMsg(TimeOutPort);
    Sender->AddOutputMsg(DataTagPort, EmptyParams);

    auto SenderInputMessageExp = TheLTS->MakeVar("SenderInputMessage", DataType);
    auto SenderTagExp = TheLTS->MakeVar("SenderTag", TagType);
    auto AckTagExp = TheLTS->MakeVar("AckTag", TagType);
    vector<LTSAssignRef> SenderDataUpdates;
    SenderDataUpdates.push_back(new LTSAssignSimple(SenderInputMessageExp, DataAccExp));
    Sender->AddInputTransition(
            "Initial",
            "Q0",
            TrueExp,
            SenderDataUpdates,
            "DataPort",
            DataPort,
            EmptyParams);
    vector<LTSAssignRef> DataTagMessageOutputUpdates;
    DataTagMessageOutputUpdates.push_back(new LTSAssignSimple(DataTagMsgDataAccExp, SenderInputMessageExp));
    DataTagMessageOutputUpdates.push_back(new LTSAssignSimple(DataTagMsgTagAccExp, SenderTagExp));
    Sender->AddOutputTransition(
            "Q0",
            "Q1",
            TrueExp,
            DataTagMessageOutputUpdates,
            "DataTagPort",
            DataTagPort,
            EmptyParams);
    Sender->AddInputTransition("Q1", "Q0", TrueExp, EmptyUpdates, "TimeOut", TimeOutPort, EmptyParams);
    vector<LTSAssignRef> updates1;
    updates1.push_back(new LTSAssignSimple(AckTagExp, TagPortPTagFieldExp));
    Sender->AddInputTransition("Q1", "Q2", TrueExp, updates1, "TagPort'", TagPortP, EmptyParams);
    auto SenderTagEqualsAckTagExp = TheLTS->MakeOp(LTSOps::OpEQ, SenderTagExp, AckTagExp);
    auto guard = SenderTagEqualsAckTagExp;
    vector<LTSAssignRef> updates2;
    auto OneExp = TheLTS->MakeVal("1", TagType);
    auto ZeroExp = TheLTS->MakeVal("0", TagType);
    auto NegateTagExp = TheLTS->MakeOp(
            LTSOps::OpITE,
            TheLTS->MakeOp(LTSOps::OpEQ, SenderTagExp, OneExp),
            ZeroExp,
            OneExp);
    updates2.push_back(new LTSAssignSimple(SenderTagExp, NegateTagExp));
    Sender->AddInternalTransition("Q2", "Initial", guard, updates2, EmptyFairnessSets);
    auto WrongAckGuard = TheLTS->MakeOp(LTSOps::OpNOT, SenderTagEqualsAckTagExp);
    Sender->AddInternalTransition("Q2", "Q0", WrongAckGuard, EmptyUpdates, EmptyFairnessSets);

    auto Timer = TheLTS->MakeGenEFSM("Timer", EmptyParams, TrueExp, LTSFairnessType::None);
    Timer->AddState("Initial");
    Timer->FreezeStates();
    Timer->FreezeVars();
    Timer->AddOutputMsg(TimeOutPort, EmptyParams);
    vector<LTSAssignRef> TimerUpdates;
    TimerUpdates.push_back(new LTSAssignSimple(TimeOutDummyAccExp, UnitExp));
    Timer->AddOutputTransition("Initial", "Initial", TrueExp, TimerUpdates, "TimeOutPort", TimeOutPort, EmptyParams, EmptyFairnessSets);

    auto Receiver = TheLTS->MakeGenEFSM("Receiver", EmptyParams, TrueExp, LTSFairnessType::None);
    Receiver->AddState("Initial");
    Receiver->AddState("Q0");
    Receiver->AddState("Q1");
    Receiver->AddState("Q2");
    Receiver->FreezeStates();
    Receiver->AddInputMsg(DataTagPortP, EmptyParams);
    Receiver->AddOutputMsg(DataReceiverToClientPort, EmptyParams);
    Receiver->AddOutputMsg(TagPort, EmptyParams);
    Receiver->AddVariable("ReceiverTag", TagType);
    auto ReceiverTagExp = TheLTS->MakeVar("ReceiverTag", TagType);
    Receiver->AddVariable("ReceiverData", DataType);
    auto ReceiverDataExp = TheLTS->MakeVar("ReceiverData", DataType);
    Receiver->AddVariable("ExpectedTag", TagType);
    auto ExpectedTagExp = TheLTS->MakeVar("ExpectedTag", TagType);
    Receiver->FreezeVars();
    vector<LTSAssignRef> ReceiverInputUpdates;
    auto DataTagPortPExp = TheLTS->MakeVar("DataTagPort'", DataTagPortP);
    auto DataTagMsgDataAccExpP = TheLTS->MakeOp(LTSOps::OpField, DataTagPortPExp, DataFieldExp);
    auto DataTagMsgTagAccExpP = TheLTS->MakeOp(LTSOps::OpField, DataTagPortPExp, TagAccFieldExp);
    ReceiverInputUpdates.push_back(new LTSAssignSimple(ReceiverDataExp, DataTagMsgDataAccExpP));
    ReceiverInputUpdates.push_back(new LTSAssignSimple(ReceiverTagExp, DataTagMsgTagAccExpP));
    Receiver->AddInputTransition(
            "Initial",
            "Q0",
            TrueExp,
            ReceiverInputUpdates,
            "DataTagPort'",
            DataTagPortP,
            EmptyParams);
    auto ReceiverTagEqExpectedTagExp = TheLTS->MakeOp(LTSOps::OpEQ, ReceiverTagExp, ExpectedTagExp);
    vector<LTSAssignRef> SendInputDataUpdates;
    SendInputDataUpdates.push_back(new LTSAssignSimple(DataReceiverToClientPortDataFieldExp, ReceiverDataExp));
    Receiver->AddOutputTransition(
            "Q0",
            "Q1",
            ReceiverTagEqExpectedTagExp,
            SendInputDataUpdates,
            "DataReceiverToClientPort",
            DataReceiverToClientPort,
            EmptyParams,
            EmptyFairnessSets);
    vector<LTSAssignRef> SendAckUpdates;
    SendAckUpdates.push_back(new LTSAssignSimple(TagPortTagFieldExp, ExpectedTagExp));
    Receiver->AddOutputTransition(
            "Q1",
            "Initial",
            TrueExp,
            SendAckUpdates,
            "TagPort",
            TagPort,
            EmptyParams,
            EmptyFairnessSets);
    auto ReceiverTagNeqExpectedTagExp = TheLTS->MakeOp(LTSOps::OpNOT, ReceiverTagEqExpectedTagExp);
    Receiver->AddInternalTransition(
            "Q0",
            "Q2",
            ReceiverTagEqExpectedTagExp,
            EmptyUpdates,
            EmptyFairnessSets);
    vector<LTSAssignRef> SendAckUpdatesWrongTag;

    auto NegateExpectedTagExp = TheLTS->MakeOp(
            LTSOps::OpITE,
            TheLTS->MakeOp(LTSOps::OpEQ, ExpectedTagExp, OneExp),
            ZeroExp,
            OneExp
    );
    SendAckUpdatesWrongTag.push_back(new LTSAssignSimple(TagPortTagFieldExp, NegateExpectedTagExp));
    Receiver->AddOutputTransition(
            "Q2",
            "Initial",
            TrueExp,
            SendAckUpdatesWrongTag,
            "TagPort",
            TagPort,
            EmptyParams,
            EmptyFairnessSets);
    Receiver->Freeze();

    auto ReceiverClient = TheLTS->MakeGenEFSM("ReceiverClient", EmptyParams, TrueExp, LTSFairnessType::None);
    ReceiverClient->AddState("Initial");
    ReceiverClient->FreezeStates();
    ReceiverClient->AddVariable("Dummy", DataType);
    ReceiverClient->FreezeVars();
    ReceiverClient->AddInputMsg(DataReceiverToClientPort, EmptyParams);
    vector<LTSAssignRef> receiver_updates1;
    receiver_updates1.push_back(
            new LTSAssignSimple(
                    TheLTS->MakeVar("Dummy", DataType),
                    DataReceiverToClientPortDataFieldExp
            )
    );
    ReceiverClient->AddInputTransition(
            "Initial",
            "Initial",
            TrueExp,
            receiver_updates1,
            "DataReceiverToClientPort",
            DataReceiverToClientPort,
            EmptyParams);

    auto Receiver2SenderChannel = TheLTS->MakeChannel("Receiver2SenderChannel", EmptyParams, TrueExp, 1, true, true, true, false, LTSFairnessType::Strong);
    Receiver2SenderChannel->AddMsg(TagPort, EmptyParams, LTSFairnessType::Strong, LossDupFairnessType::None);

    auto Sender2ReceiverChannel = TheLTS->MakeChannel("Sender2ReceiverChannel", EmptyParams, TrueExp, 1, true, true, true, false, LTSFairnessType::Strong);
    Sender2ReceiverChannel->AddMsg(DataTagPort, EmptyParams, LTSFairnessType::Strong, LossDupFairnessType::None);



    TheLTS->FreezeAutomata();
    vector<InitStateRef> InitStates;
    vector<LTSAssignRef> InitUpdates;
    auto SenderClientType = TheLTS->GetEFSMType("SenderClient");
    auto SenderType = TheLTS->GetEFSMType("Sender");
    auto TimerType = TheLTS->GetEFSMType("Timer");
    auto ReceiverType = TheLTS->GetEFSMType("Receiver");
    auto ReceiverClientType = TheLTS->GetEFSMType("ReceiverClient");

    auto SenderClientStateVar = TheLTS->MakeVar("SenderClient", SenderClientType);
    auto SenderStateVar = TheLTS->MakeVar("Sender", SenderType);
    auto TimerStateVar = TheLTS->MakeVar("Timer", TimerType);
    auto ReceiverStateVar = TheLTS->MakeVar("Receiver", ReceiverType);
    auto ReceiverClientStateVar = TheLTS->MakeVar("ReceiverClient", ReceiverClientType);

    auto StateAcc = TheLTS->MakeVar("state", FAType);

    auto SenderClientDotState = TheLTS->MakeOp(LTSOps::OpField, SenderClientStateVar, StateAcc);
    auto SenderDotState = TheLTS->MakeOp(LTSOps::OpField, SenderStateVar, StateAcc);
    auto TimerDotState = TheLTS->MakeOp(LTSOps::OpField, TimerStateVar, StateAcc);

    auto ReceiverDotState = TheLTS->MakeOp(LTSOps::OpField, ReceiverStateVar, StateAcc);
    auto ReceiverClientDotState = TheLTS->MakeOp(LTSOps::OpField, ReceiverClientStateVar, StateAcc);

    auto SenderDotSenderInputMessage = TheLTS->MakeOp(LTSOps::OpField, SenderStateVar, TheLTS->MakeVar("SenderInputMessage", FAType));
    auto SenderDotSenderTag = TheLTS->MakeOp(LTSOps::OpField, SenderStateVar, TheLTS->MakeVar("SenderTag", FAType));
    auto SenderDotAckTag = TheLTS->MakeOp(LTSOps::OpField, SenderStateVar, TheLTS->MakeVar("AckTag", FAType));

    auto ReceiverDotReceiverTag = TheLTS->MakeOp(LTSOps::OpField, ReceiverStateVar, TheLTS->MakeVar("ReceiverTag", FAType));
    auto ReceiverDotReceiverData = TheLTS->MakeOp(LTSOps::OpField, ReceiverStateVar, TheLTS->MakeVar("ReceiverData", FAType));
    auto ReceiverDotExpectedTag = TheLTS->MakeOp(LTSOps::OpField, ReceiverStateVar, TheLTS->MakeVar("ExpectedTag", FAType));

    InitUpdates.push_back(new LTSAssignSimple(SenderClientDotState, TheLTS->MakeVal("Initial", SenderClientDotState->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(SenderDotState, TheLTS->MakeVal("Initial", SenderDotState->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TimerDotState, TheLTS->MakeVal("Initial", TimerDotState->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(ReceiverDotState, TheLTS->MakeVal("Initial", ReceiverDotState->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(ReceiverClientDotState, TheLTS->MakeVal("Initial", ReceiverClientDotState->GetType())));

    InitUpdates.push_back(new LTSAssignSimple(SenderDotSenderInputMessage, TheLTS->MakeVal("0", SenderDotSenderInputMessage->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(SenderDotSenderTag, TheLTS->MakeVal("0", SenderDotSenderTag->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(SenderDotAckTag, TheLTS->MakeVal("0", SenderDotAckTag->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(ReceiverDotReceiverTag, TheLTS->MakeVal("0", ReceiverDotReceiverTag->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(ReceiverDotReceiverData, TheLTS->MakeVal("0", ReceiverDotReceiverData->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(ReceiverDotExpectedTag, TheLTS->MakeVal("0", ReceiverDotExpectedTag->GetType())));

    InitStates.push_back(new LTSInitState(EmptyParams, TrueExp, InitUpdates));
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
        cout << InitStateGen->ToString() << endl;
    }

    cout << "Invariant:" << endl;
    cout << TheLTS->GetInvariant() << endl;

    // cout << "Channel Buffer variables to sort:" << endl;
    // for (auto const& BufferExp : TheLTS->GetChanBuffersToSort()) {
    //     cout << BufferExp.first->ToString() << endl;
    //     cout << BufferExp.second->ToString() << endl;
    // }

    auto Checker = new LTSChecker(TheLTS);
    Checker->BuildAQS();
    delete Checker;
}

//
// ABP.cpp ends here
