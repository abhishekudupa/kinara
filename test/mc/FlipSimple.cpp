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

// Test for multiple indices of same symmetric type

#include "../../src/uflts/LabelledTS.hpp"
#include "../../src/uflts/LTSEFSM.hpp"
#include "../../src/uflts/LTSChannelEFSM.hpp"
#include "../../src/uflts/LTSAssign.hpp"
#include "../../src/uflts/LTSTransitions.hpp"
#include "../../src/mc/LTSChecker.hpp"
#include "../../src/mc/OmegaAutomaton.hpp"

using namespace ESMC;
using namespace LTS;
using namespace Exprs;
using namespace MC;

int main()
{
    auto TheLTS = new LabelledTS();

    auto ClientIDType = TheLTS->MakeSymmType("ClientIDType", 5);
    auto BoolType = TheLTS->MakeBoolType();
    auto ParamExp = TheLTS->MakeVar("ClientID", ClientIDType);
    vector<ExpT> Params = { ParamExp };
    auto TrueExp = TheLTS->MakeTrue();

    // Add the message types
    vector<pair<string, ExprTypeRef>> MsgFields;
    auto RangeType = TheLTS->MakeRangeType(0, 9);
    MsgFields.push_back(make_pair("Data", RangeType));

    vector<ExpT> MsgParams = { ParamExp };
    auto DataMsgType = TheLTS->MakeMsgTypes(MsgParams, TrueExp, "DataMsg", MsgFields, false);

    TheLTS->FreezeMsgs();

    auto Sink = TheLTS->MakeGenEFSM("Sink", vector<ExpT>(), TrueExp, LTSFairnessType::Strong);

    // Server structure
    Sink->AddState("InitState");
    Sink->FreezeStates();

    Sink->AddInputMsgs(MsgParams, TrueExp, DataMsgType, MsgParams);
    Sink->FreezeVars();

    vector<LTSAssignRef> SinkInputUpdates;
    auto DataMsgExp = TheLTS->MakeVar("InMsg", DataMsgType);    

    Sink->AddInputTransitions(MsgParams, TrueExp, "InitState", "InitState", 
                              TrueExp, SinkInputUpdates, "InMsg", 
                              DataMsgType, MsgParams);

    // Client structure
    auto ClientEFSM = TheLTS->MakeGenEFSM("Client", MsgParams, TrueExp, LTSFairnessType::Strong);

    // Commenting the above line and uncommenting the line below
    // should result in a liveness violation, which is of course
    // "unfair".
    // auto ClientEFSM = TheLTS->MakeGenEFSM("Client", MsgParams, TrueExp, LTSFairnessType::None);
    
    ClientEFSM->AddState("InitState");
    ClientEFSM->FreezeStates();

    ClientEFSM->AddVariable("Bit", BoolType);
    ClientEFSM->AddOutputMsg(DataMsgType, MsgParams);

    ClientEFSM->FreezeVars();

    vector<LTSAssignRef> ClientOutputUpdates;
    auto BitExp = TheLTS->MakeVar("Bit", BoolType);
    auto NotBitExp = TheLTS->MakeOp(LTSOps::OpNOT, BitExp);

    ClientOutputUpdates.push_back(new LTSAssignSimple(BitExp, NotBitExp));    
    ClientEFSM->AddOutputTransition("InitState", "InitState", TrueExp, ClientOutputUpdates, 
                                    "OutMsg", DataMsgType, MsgParams);

    cout << ClientEFSM->ToString() << endl;
    cout << Sink->ToString() << endl;

    TheLTS->FreezeAutomata();
    
    vector<InitStateRef> InitStates;
    vector<LTSAssignRef> InitUpdates;

    auto ClientType = TheLTS->GetEFSMType("Client");
    auto SinkType = TheLTS->GetEFSMType("Sink");

    auto FAType = TheLTS->MakeFieldAccessType();

    auto ClientStateVar = TheLTS->MakeOp(LTSOps::OpIndex, 
                                         TheLTS->MakeVar("Client", ClientType),
                                         ParamExp);

    auto ServerStateVar = TheLTS->MakeVar("Sink", SinkType);
    auto ServerDotState = TheLTS->MakeOp(LTSOps::OpField, ServerStateVar,
                                         TheLTS->MakeVar("state", FAType));
    InitUpdates.push_back(new LTSAssignSimple(ServerDotState, TheLTS->MakeVal("InitState", ServerDotState->GetType())));

    auto ClientDotState = TheLTS->MakeOp(LTSOps::OpField, ClientStateVar,
                                         TheLTS->MakeVar("state", FAType));
    auto ClientDotBit = TheLTS->MakeOp(LTSOps::OpField, ClientStateVar,
                                         TheLTS->MakeVar("Bit", FAType));

    InitUpdates.push_back(new LTSAssignParam(MsgParams, TrueExp, ClientDotState, TheLTS->MakeVal("InitState", ClientDotState->GetType())));
    InitUpdates.push_back(new LTSAssignParam(MsgParams, TrueExp, ClientDotBit, TheLTS->MakeVal("false", BoolType)));

    InitStates.push_back(new LTSInitState(vector<ExpT>(), TrueExp, InitUpdates));
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

    cout << "Channel Buffer variables to sort:" << endl;
    for (auto const& BufferExp : TheLTS->GetChanBuffersToSort()) {
        cout << BufferExp.first->ToString() << endl;
        cout << BufferExp.second->ToString() << endl;
    }

    auto Checker = new LTSChecker(TheLTS);

    auto Monitor = Checker->MakeStateBuchiMonitor("GFClear", Params, TrueExp);
    Monitor->AddState("InitState", true, false);
    Monitor->AddState("AcceptState", false, true);
    
    Monitor->FreezeStates();

    auto ClientClearExp = Monitor->MakeOp(LTSOps::OpNOT, ClientDotBit);

    Monitor->AddTransition("InitState", "InitState", TrueExp);
    Monitor->AddTransition("InitState", "AcceptState", ClientDotBit);
    Monitor->AddTransition("AcceptState", "AcceptState", ClientDotBit);

    Monitor->Freeze();

    Checker->BuildAQS();
    Checker->CheckLiveness("GFClear");
    delete Checker;
}

// 
// PingPong.cpp ends here
