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

#include "../../src/uflts/UFLTS.hpp"

using namespace ESMC;
using namespace LTS;
using namespace Exprs;

int main()
{
    UFLTS* TheLTS = new UFLTS();
    auto Mgr = TheLTS->GetMgr();

    auto ClientIDType = TheLTS->MakeSymmetricType("ClientIDType", 2);
    auto ParamExp = Mgr->MakeVar("ClientID", ClientIDType);
    vector<ExpT> Params = { ParamExp };
    auto TrueExp = Mgr->MakeTrue();

    // Add the message types
    vector<pair<string, ExprTypeRef>> MsgFields;
    auto RangeType = Mgr->MakeType<ExprRangeType>(0, 100);
    MsgFields.push_back(make_pair("Data", RangeType));

    auto DataMsgType = TheLTS->MakeMessageType("DataMsg", MsgFields, Params, TrueExp);
    auto AckMsgType = TheLTS->MakeMessageType("AckMsg", MsgFields, Params, TrueExp);

    TheLTS->FreezeMessages();

    auto Server = TheLTS->MakeEFSM("Server", vector<ExpT>(), Mgr->MakeTrue());

    auto ClientIDParam = Mgr->MakeVar("ClientID", ClientIDType);
    auto ClientEFSM = TheLTS->MakeEFSM("Client", { ClientIDParam }, Mgr->MakeTrue());

    Server->AddState("InitState");
    Server->AddState("SendState");
    Server->FreezeStates();

    Server->AddVariable("LastMsg", RangeType);

    Server->AddInputMsgs(Params, DataMsgType, Params, TrueExp);
    Server->AddOutputMsgs(Params, AckMsgType, Params, TrueExp);

    ClientEFSM->AddState("InitState");
    ClientEFSM->AddState("RecvState");

    ClientEFSM->FreezeStates();
    
    vector<AsgnT> RecvUpdates;
    RecvUpdates.push_back(Mgr->MakeVar("InMsg", Data
    Server->AddInputTransitions("InitState", "RecvState", Params, TrueExp, TrueExp, , const string &MessageName, const ExprTypeRef &MessageType, const vector<ExpT> &MessageParams)
}

// 
// PingPong.cpp ends here
