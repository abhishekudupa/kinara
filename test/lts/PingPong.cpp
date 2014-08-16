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
#include "../../src/uflts/LTSAssign.hpp"

using namespace ESMC;
using namespace LTS;
using namespace Exprs;

int main()
{
    auto TheLTS = new LabelledTS();

    auto ClientIDType = TheLTS->MakeSymmType("ClientIDType", 2);
    auto ParamExp = TheLTS->MakeVar("ClientID", ClientIDType);
    vector<ExpT> Params = { ParamExp };
    auto TrueExp = TheLTS->MakeTrue();

    // Add the message types
    vector<pair<string, ExprTypeRef>> MsgFields;
    auto RangeType = TheLTS->MakeRangeType(0, 99);
    MsgFields.push_back(make_pair("Data", RangeType));

    auto DataMsgType = TheLTS->MakeMsgTypes(Params, TrueExp, "DataMsg", MsgFields);
    auto AckMsgType = TheLTS->MakeMsgTypes(Params, TrueExp, "AckMsg", MsgFields);

    TheLTS->FreezeMsgs();

    auto Server = TheLTS->MakeGenEFSM("Server", vector<ExpT>(), TrueExp, LTSFairnessType::Strong);

    auto ClientIDParam = TheLTS->MakeVar("ClientID", ClientIDType);
    auto ClientEFSM = TheLTS->MakeGenEFSM("Client", Params, TrueExp, LTSFairnessType::Strong);

    // Server structure
    Server->AddState("InitState");
    Server->AddState("SendState");
    Server->FreezeStates();

    Server->AddVariable("LastMsg", RangeType);
    Server->AddVariable("LastReq", ClientIDType);

    Server->AddInputMsgs(Params, TrueExp, DataMsgType, Params);
    Server->AddOutputMsgs(Params, TrueExp, AckMsgType, Params);

    Server->FreezeVars();

    vector<LTSAssignRef> ServerInputUpdates;
    auto LastMsgExp = TheLTS->MakeVar("LastMsg", RangeType);
    auto LastReqExp = TheLTS->MakeVar("LastReq", ClientIDType);

    auto DataMsgExp = TheLTS->MakeVar("InMsg", DataMsgType);
    auto FAType = TheLTS->MakeFieldAccessType();
    auto DataAccFieldExp = TheLTS->MakeVar("Data", FAType);
    auto DataAccExp = TheLTS->MakeOp(LTSOps::OpField, DataMsgExp, DataAccFieldExp);
    ServerInputUpdates.push_back(new LTSAssignSimple(LastMsgExp, DataAccExp));
    ServerInputUpdates.push_back(new LTSAssignSimple(LastReqExp, ParamExp));
    

    Server->AddInputTransitions(Params, TrueExp, "InitState", "SendState", 
                                TrueExp, ServerInputUpdates, "InMsg", 
                                DataMsgType, Params);

    vector<LTSAssignRef> ServerOutputUpdates;
    auto AckMsgExp = TheLTS->MakeVar("OutMsg", AckMsgType);
    auto AckAccFieldExp = TheLTS->MakeVar("Data", FAType);
    auto AckAccExp = TheLTS->MakeOp(LTSOps::OpField, AckMsgExp, AckAccFieldExp);
    ServerOutputUpdates.push_back(new LTSAssignSimple(AckAccExp, LastMsgExp));
    ServerOutputUpdates.push_back(new LTSAssignSimple(LastMsgExp, TheLTS->MakeVal("clear", RangeType)));
    auto ServerGuard = TheLTS->MakeOp(LTSOps::OpEQ, LastReqExp, ParamExp);
    
    Server->AddOutputTransitions(Params, TrueExp, "SendState", "InitState", ServerGuard, 
                                 ServerOutputUpdates, "OutMsg", AckMsgType, Params, 
                                 LTSFairnessType::Strong, SplatFairnessType::Individual, 
                                 "RspFairness");

    // Client structure
    ClientEFSM->AddState("InitState");
    ClientEFSM->AddState("RecvState");
    ClientEFSM->AddState("DecideState");
    ClientEFSM->AddState("ErrorState");

    ClientEFSM->FreezeStates();

    ClientEFSM->AddVariable("Count", RangeType);
    ClientEFSM->AddVariable("LastMsg", RangeType);
    ClientEFSM->AddOutputMsg(DataMsgType, Params);
    ClientEFSM->AddInputMsg(AckMsgType, Params);

    ClientEFSM->FreezeVars();

    vector<LTSAssignRef> ClientOutputUpdates;
    auto CountExp = TheLTS->MakeVar("Count", RangeType);
    auto ZeroExp = TheLTS->MakeVal("0", RangeType);
    auto OneExp = TheLTS->MakeVal("1", RangeType);
    auto MaxExp = TheLTS->MakeVal("99", RangeType);
    auto CountIncExp = TheLTS->MakeOp(LTSOps::OpITE, 
                                      TheLTS->MakeOp(LTSOps::OpEQ, CountExp, MaxExp),
                                      ZeroExp,
                                      TheLTS->MakeOp(LTSOps::OpADD, CountExp, OneExp));

    DataMsgExp = TheLTS->MakeVar("OutMsg", DataMsgType);
    DataAccFieldExp = TheLTS->MakeVar("Data", FAType);
    DataAccExp = TheLTS->MakeOp(LTSOps::OpField, DataMsgExp, DataAccFieldExp);

    ClientOutputUpdates.push_back(new LTSAssignSimple(DataAccExp, CountExp));    
    ClientEFSM->AddOutputTransition("InitState", "RecvState", TrueExp, ClientOutputUpdates, "OutMsg", DataMsgType, Params);

    auto RecvMsgExp = TheLTS->MakeVar("InMsg", AckMsgType);
    auto RecvMsgAccFieldExp = TheLTS->MakeVar("Data", FAType);
    auto RecvMsgAccExp = TheLTS->MakeOp(LTSOps::OpField, RecvMsgExp, RecvMsgAccFieldExp);

    vector<LTSAssignRef> ClientInputUpdates;
    ClientInputUpdates.push_back(new LTSAssignSimple(LastMsgExp, RecvMsgAccExp));
    ClientEFSM->AddInputTransition("RecvState", "DecideState", TrueExp, ClientInputUpdates, "InMsg", AckMsgType, Params);
    
    vector<LTSAssignRef> ClientDecideUpdates;
    ClientDecideUpdates.push_back(new LTSAssignSimple(CountExp, CountIncExp));
    ClientDecideUpdates.push_back(new LTSAssignSimple(LastMsgExp, TheLTS->MakeVal("clear", RangeType)));
    auto DecideGuard = TheLTS->MakeOp(LTSOps::OpEQ, LastMsgExp, CountExp);
    ClientEFSM->AddInternalTransition("DecideState", "InitState", DecideGuard, ClientDecideUpdates);

    auto ErrorGuard = TheLTS->MakeOp(LTSOps::OpNOT, DecideGuard);
    ClientEFSM->AddInternalTransition("DecideState", "ErrorState", ErrorGuard, vector<LTSAssignRef>());

    cout << ClientEFSM->ToString() << endl;
    cout << Server->ToString() << endl;
}

// 
// PingPong.cpp ends here

