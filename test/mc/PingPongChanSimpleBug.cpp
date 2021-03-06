// PingPong.cpp ---
//
// Filename: PingPong.cpp
// Author: Abhishek Udupa
// Created: Tue Aug  5 10:51:04 2014 (-0400)
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

// we create a simple model:
// There are two symmetric processes,
// and a server. The processes send messages
// with increasing payloads to the server
// the server and the server echoes it back
// to the processes

#include "../../src/uflts/LabelledTS.hpp"
#include "../../src/uflts/LTSEFSM.hpp"
#include "../../src/uflts/LTSChannelEFSM.hpp"
#include "../../src/uflts/LTSAssign.hpp"
#include "../../src/uflts/LTSTransitions.hpp"
#include "../../src/mc/LTSChecker.hpp"
#include "../../src/mc/OmegaAutomaton.hpp"
#include "../../src/mc/Trace.hpp"
#include "../../src/symexec/LTSAnalyses.hpp"
#include "../../src/lib/ESMCLib.hpp"
#include "../../src/utils/LogManager.hpp"

using namespace ESMC;
using namespace LTS;
using namespace Exprs;
using namespace MC;
using namespace Analyses;

int main()
{
    ESMC::Logging::LogManager::Initialize();
    ESMC::Logging::LogManager::EnableLogOption("Checker.AQSDetailed");
    ESMC::Logging::LogManager::EnableLogOption("Checker.Fairness");
    ESMC::Logging::LogManager::EnableLogOption("Trace.Generation");
    auto TheLTS = new LabelledTS();

    auto ClientIDType = TheLTS->MakeSymmType("ClientIDType", 2);
    auto ParamExp = TheLTS->MakeVar("ClientID", ClientIDType);
    vector<ExpT> Params = { ParamExp };
    auto TrueExp = TheLTS->MakeTrue();

    int MaxValue = 1;
    // Add the message types
    vector<pair<string, TypeRef>> MsgFields;
    auto RangeType = TheLTS->MakeRangeType(0, MaxValue);
    MsgFields.push_back(make_pair("Data", RangeType));

    auto DataMsgType = TheLTS->MakeMsgTypes(Params, TrueExp, "DataMsg", MsgFields, true);
    auto DataMsgTypeP = TheLTS->GetNamedType("DataMsg'");
    auto AckMsgType = TheLTS->MakeMsgTypes(Params, TrueExp, "AckMsg", MsgFields, true);
    auto AckMsgTypeP = TheLTS->GetNamedType("AckMsg'");

    TheLTS->FreezeMsgs();

    auto Server = TheLTS->MakeGenEFSM("Server", vector<ExpT>(), TrueExp, LTSFairnessType::None);

    auto ClientIDParam = TheLTS->MakeVar("ClientID", ClientIDType);
    auto ClientEFSM = TheLTS->MakeGenEFSM("Client", Params, TrueExp, LTSFairnessType::Strong);

    auto C2SChan = TheLTS->MakeChannel("C2SChan", vector<ExpT>(), TrueExp, 2, false,
                                       false, false, false, LTSFairnessType::None);
    auto S2CChan = TheLTS->MakeChannel("S2CChan", Params, TrueExp, 1, false, true,
                                       false, false, LTSFairnessType::None);

    C2SChan->AddMsgs(Params, TrueExp, DataMsgType, Params, true, LTSFairnessType::Strong, LossDupFairnessType::None);

    S2CChan->AddMsg(AckMsgType, Params, LTSFairnessType::Strong, LossDupFairnessType::None);

    // Server structure
    Server->AddState("InitState");
    Server->AddState("SendState");
    Server->FreezeStates();

    Server->AddVariable("LastReq", ClientIDType);

    Server->AddInputMsgs(Params, TrueExp, DataMsgTypeP, Params);
    Server->AddOutputMsgs(Params, TrueExp, AckMsgType, Params);

    Server->FreezeVars();

    vector<LTSAssignRef> ServerInputUpdates;
    auto LastReqExp = TheLTS->MakeVar("LastReq", ClientIDType);

    auto DataMsgExp = TheLTS->MakeVar("InMsg", DataMsgTypeP);
    auto FAType = TheLTS->MakeFieldAccessType();
    auto DataAccFieldExp = TheLTS->MakeVar("Data", FAType);
    auto DataAccExp = TheLTS->MakeOp(LTSOps::OpField, DataMsgExp, DataAccFieldExp);
    ServerInputUpdates.push_back(new LTSAssignSimple(LastReqExp, ParamExp));

    Server->AddInputTransitions(Params, TrueExp, "InitState", "SendState",
                                TrueExp, ServerInputUpdates, "InMsg",
                                DataMsgTypeP, Params);

    vector<LTSAssignRef> ServerOutputUpdates;
    auto AckMsgExp = TheLTS->MakeVar("OutMsg", AckMsgType);
    auto AckAccFieldExp = TheLTS->MakeVar("Data", FAType);
    auto AckAccExp = TheLTS->MakeOp(LTSOps::OpField, AckMsgExp, AckAccFieldExp);
    ServerOutputUpdates.push_back(new LTSAssignSimple(LastReqExp, TheLTS->MakeVal("clear", ClientIDType)));
    auto ServerGuard = TheLTS->MakeOp(LTSOps::OpEQ, LastReqExp, ParamExp);

    Server->AddFairnessSet("RspFairness", FairSetFairnessType::Strong, Params, TrueExp);
    Server->AddOutputTransitions(Params, TrueExp, "SendState", "InitState", ServerGuard,
                                 ServerOutputUpdates, "OutMsg", AckMsgType, Params,
                                 {}, { "RspFairness" });

    // Client structure
    ClientEFSM->AddState("InitState");
    ClientEFSM->AddState("RecvState");

    ClientEFSM->FreezeStates();

    ClientEFSM->AddVariable("Count", RangeType);
    ClientEFSM->AddOutputMsg(DataMsgType, Params);
    ClientEFSM->AddInputMsg(AckMsgTypeP, Params);

    ClientEFSM->FreezeVars();

    vector<LTSAssignRef> ClientOutputUpdates;
    auto CountExp = TheLTS->MakeVar("Count", RangeType);
    auto ZeroExp = TheLTS->MakeVal("0", RangeType);
    auto OneExp = TheLTS->MakeVal("1", RangeType);
    auto MaxExp = TheLTS->MakeVal(to_string(MaxValue), RangeType);
    auto CountIncExp = TheLTS->MakeOp(LTSOps::OpITE,
                                      TheLTS->MakeOp(LTSOps::OpEQ, CountExp, MaxExp),
                                      ZeroExp,
                                      TheLTS->MakeOp(LTSOps::OpADD, CountExp, OneExp));

    DataMsgExp = TheLTS->MakeVar("OutMsg", DataMsgType);
    DataAccFieldExp = TheLTS->MakeVar("Data", FAType);
    DataAccExp = TheLTS->MakeOp(LTSOps::OpField, DataMsgExp, DataAccFieldExp);

    ClientOutputUpdates.push_back(new LTSAssignSimple(DataAccExp, CountExp));
    ClientEFSM->AddOutputTransition("InitState", "RecvState", TrueExp, ClientOutputUpdates, "OutMsg", DataMsgType, Params);

    auto RecvMsgExp = TheLTS->MakeVar("InMsg", AckMsgTypeP);
    auto RecvMsgAccFieldExp = TheLTS->MakeVar("Data", FAType);
    auto RecvMsgAccExp = TheLTS->MakeOp(LTSOps::OpField, RecvMsgExp, RecvMsgAccFieldExp);

    vector<LTSAssignRef> ClientInputUpdates;
    ClientInputUpdates.push_back(new LTSAssignSimple(CountExp, CountIncExp));
    ClientEFSM->AddInputTransition("RecvState", "InitState", TrueExp, ClientInputUpdates, "InMsg", AckMsgTypeP, Params);

    cout << C2SChan->ToString() << endl;
    cout << S2CChan->ToString() << endl;
    cout << ClientEFSM->ToString() << endl;
    cout << Server->ToString() << endl;

    TheLTS->FreezeAutomata();

    vector<InitStateRef> InitStates;
    vector<LTSAssignRef> InitUpdates;

    auto ClientType = TheLTS->GetEFSMType("Client");
    auto ServerType = TheLTS->GetEFSMType("Server");

    auto ClientStateVar = TheLTS->MakeOp(LTSOps::OpIndex,
                                         TheLTS->MakeVar("Client", ClientType),
                                         ParamExp);
    auto ServerStateVar = TheLTS->MakeVar("Server", ServerType);
    auto ServerDotState = TheLTS->MakeOp(LTSOps::OpField, ServerStateVar,
                                         TheLTS->MakeVar("state", FAType));
    auto ServerDotReq = TheLTS->MakeOp(LTSOps::OpField, ServerStateVar,
                                       TheLTS->MakeVar("LastReq", FAType));
    InitUpdates.push_back(new LTSAssignSimple(ServerDotState, TheLTS->MakeVal("InitState", ServerDotState->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(ServerDotReq, TheLTS->MakeVal("clear", ServerDotReq->GetType())));


    vector<ExpT> ClientUpdParams = { TheLTS->MakeVar("UpdClientParam", ClientIDType) };
    ClientStateVar = TheLTS->MakeOp(LTSOps::OpIndex,
                                    TheLTS->MakeVar("Client", ClientType),
                                    ClientUpdParams[0]);

    auto ClientDotState = TheLTS->MakeOp(LTSOps::OpField, ClientStateVar,
                                         TheLTS->MakeVar("state", FAType));
    auto ClientDotCount = TheLTS->MakeOp(LTSOps::OpField, ClientStateVar,
                                         TheLTS->MakeVar("Count", FAType));

    InitUpdates.push_back(new LTSAssignParam(ClientUpdParams, TrueExp, ClientDotState, TheLTS->MakeVal("InitState", ClientDotState->GetType())));
    InitUpdates.push_back(new LTSAssignParam(ClientUpdParams, TrueExp, ClientDotCount, TheLTS->MakeVal("0", ClientDotCount->GetType())));

    InitStates.push_back(new LTSInitState(vector<ExpT>(), TrueExp, InitUpdates));
    TheLTS->AddInitStates(InitStates);

    auto BoundVarExp = TheLTS->MakeBoundVar(0, ClientIDType);
    auto ClientExp = TheLTS->MakeOp(LTSOps::OpIndex,
                                    TheLTS->MakeVar("Client", ClientType),
                                    BoundVarExp);
    ClientDotCount = TheLTS->MakeOp(LTSOps::OpField, ClientExp,
                                    TheLTS->MakeVar("Count", FAType));

    // Invariant that actually holds
    auto BodyExp = TheLTS->MakeOp(LTSOps::OpAND,
                                  TheLTS->MakeOp(LTSOps::OpGE,
                                                 ClientDotCount,
                                                 ZeroExp),
                                  TheLTS->MakeOp(LTSOps::OpLE,
                                                 ClientDotCount,
                                                 MaxExp));

    // Invariant for buggy
    // auto BodyExp = TheLTS->MakeOp(LTSOps::OpAND,
    //                               TheLTS->MakeOp(LTSOps::OpGE,
    //                                              ClientDotCount,
    //                                              ZeroExp),
    //                               TheLTS->MakeOp(LTSOps::OpLT,
    //                                              ClientDotCount,
    //                                              MaxExp));

    auto QExp = TheLTS->MakeForAll({ ClientIDType }, BodyExp);

    TheLTS->AddInvariant(QExp);

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
    auto Status = Checker->BuildAQS();

    if (!Status) {
        cout << "Found Bugs!" << endl;
        delete Checker;
        exit(0);
    }


    auto Monitor = Checker->MakeStateBuchiMonitor("GFZero", Params, TrueExp);
    Monitor->AddState("InitState", true, false);
    Monitor->AddState("AcceptState", false, true);
    Monitor->FreezeStates();

    ClientStateVar = Monitor->MakeOp(LTSOps::OpIndex, Monitor->MakeVar("Client", ClientType),
                                     Params[0]);
    ClientDotCount = Monitor->MakeOp(LTSOps::OpField, ClientStateVar,
                                     Monitor->MakeVar("Count", FAType));

    auto ClientCountZero = Monitor->MakeOp(LTSOps::OpEQ, ClientDotCount, ZeroExp);
    auto NotClientCountZero = Monitor->MakeOp(LTSOps::OpNOT, ClientCountZero);

    Monitor->AddTransition("InitState", "InitState", TrueExp);
    Monitor->AddTransition("InitState", "AcceptState", NotClientCountZero);
    Monitor->AddTransition("AcceptState", "AcceptState", NotClientCountZero);
    Monitor->Freeze();

    Monitor = Checker->MakeStateBuchiMonitor("FGZero", Params, TrueExp);
    Monitor->AddState("InitAccState", true, true);
    Monitor->AddState("OtherState", false, false);
    Monitor->FreezeStates();

    Monitor->AddTransition("InitAccState", "InitAccState", NotClientCountZero);
    Monitor->AddTransition("InitAccState", "OtherState", ClientCountZero);
    Monitor->AddTransition("OtherState", "InitAccState", NotClientCountZero);
    Monitor->AddTransition("OtherState", "OtherState", ClientCountZero);

    auto Trace = Checker->CheckLiveness("GFZero");

    // auto Trace = Checker->CheckLiveness("FGZero");

    if (Trace != nullptr) {
        cout << Trace->ToString(0) << endl;
        // cout << TraceAnalyses::WeakestPreconditionForLiveness(TheLTS, Monitor, Trace->As<LivenessViolation>()) << endl;
        delete Trace;
    }

    delete Checker;
}

//
// PingPong.cpp ends here
