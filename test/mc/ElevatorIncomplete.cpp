// Elevator.cpp ---
//
// Filename: Elevator.cpp
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

#include <algorithm>

#include "../../src/uflts/LabelledTS.hpp"
#include "../../src/uflts/LTSEFSM.hpp"
#include "../../src/uflts/LTSChannelEFSM.hpp"
#include "../../src/uflts/LTSAssign.hpp"
#include "../../src/uflts/LTSTransitions.hpp"
#include "../../src/mc/Compiler.hpp"
#include "../../src/mc/LTSChecker.hpp"
#include "../../src/mc/OmegaAutomaton.hpp"
#include "../../src/mc/Trace.hpp"
#include "../../src/synth/Solver.hpp"
#include "../../src/symexec/LTSAnalyses.hpp"
#include "../../src/tpinterface/TheoremProver.hpp"


using namespace ESMC;
using namespace LTS;
using namespace Exprs;
using namespace MC;
using namespace Synth;
using namespace Analyses;

int TOPFLOOR = 2;


void InitializeAutomata(LabelledTS* TheLTS)
{
    vector<InitStateRef> InitStates;
    vector<LTSAssignRef> InitUpdates;

    auto ElevatorType = TheLTS->GetEFSMType("Elevator");
    auto ControllerType = TheLTS->GetEFSMType("Controller");
    auto UserType = TheLTS->GetEFSMType("User");
    auto LivenessMonitorType = TheLTS->GetEFSMType("LivenessMonitor");

    auto ElevatorStateVar = TheLTS->MakeVar("Elevator", ElevatorType);
    auto ControllerStateVar = TheLTS->MakeVar("Controller", ControllerType);
    auto UserStateVar = TheLTS->MakeVar("User", UserType);
    auto LivenessMonitorStateVar = TheLTS->MakeVar("LivenessMonitor", LivenessMonitorType);

    auto ElevatorDotState = TheLTS->MakeOp(LTSOps::OpField, ElevatorStateVar, TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()));
    auto ElevatorDotFloor = TheLTS->MakeOp(LTSOps::OpField, ElevatorStateVar, TheLTS->MakeVar("Floor", TheLTS->MakeFieldAccessType()));

    InitUpdates.push_back(new LTSAssignSimple(ElevatorDotState, TheLTS->MakeVal("InitialState", ElevatorDotState->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(ElevatorDotFloor, TheLTS->MakeVal("1", ElevatorDotFloor->GetType())));

    auto ControllerDotState = TheLTS->MakeOp(LTSOps::OpField, ControllerStateVar, TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()));
    auto ControllerDotCurrentFloor = TheLTS->MakeOp(LTSOps::OpField, ControllerStateVar, TheLTS->MakeVar("CurrentFloor", TheLTS->MakeFieldAccessType()));
    auto ControllerDotTargetFloor = TheLTS->MakeOp(LTSOps::OpField, ControllerStateVar, TheLTS->MakeVar("TargetFloor", TheLTS->MakeFieldAccessType()));

    InitUpdates.push_back(new LTSAssignSimple(ControllerDotState, TheLTS->MakeVal("Initial", ControllerDotState->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(ControllerDotCurrentFloor, TheLTS->MakeVal("1", ControllerDotCurrentFloor->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(ControllerDotTargetFloor, TheLTS->MakeVal("1", ControllerDotTargetFloor->GetType())));

    auto UserDotState = TheLTS->MakeOp(LTSOps::OpField, UserStateVar, TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()));

    InitUpdates.push_back(new LTSAssignSimple(UserDotState, TheLTS->MakeVal("InitialState", UserDotState->GetType())));

    auto LivenessMonitorDotState = TheLTS->MakeOp(LTSOps::OpField, LivenessMonitorStateVar, TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()));

    auto LivenessMonitorInitialStateValue = TheLTS->MakeVal("InitialState", LivenessMonitorDotState->GetType());

    InitUpdates.push_back(new LTSAssignSimple(LivenessMonitorDotState, LivenessMonitorInitialStateValue));

    InitStates.push_back(new LTSInitState({}, TheLTS->MakeTrue(), InitUpdates));
    TheLTS->AddInitStates(InitStates);

    TheLTS->Freeze();
}

void AddUserAutomaton(LabelledTS* TheLTS, map<string, ExprTypeRef>& MsgTypes, int TopFloor)
{
    auto User = TheLTS->MakeGenEFSM("User", vector<ExpT>(), TheLTS->MakeTrue(), LTSFairnessType::None);
    User->AddState("InitialState");
    User->AddState("WaitingState");
    User->FreezeStates();
    User->AddInputMsg(MsgTypes["Arrive"], {});
    User->AddOutputMsg(MsgTypes["Request"], {});
    set<string> EmptyFairnessSets;
    auto FloorType = TheLTS->MakeRangeType(1, TopFloor + 1);


    vector<ExpT> UserConstantFloorVariables;
    auto FloorAccFieldExp = TheLTS->MakeVar("Floor", TheLTS->MakeFieldAccessType());
    auto FloorAccExp = TheLTS->MakeOp(LTSOps::OpField,
                                      TheLTS->MakeVar("Request", MsgTypes["Request"]),
                                      FloorAccFieldExp);
    User->FreezeVars();
    for (auto i = 1; i <= TopFloor + 1; ++i) {
        auto FloorVal = TheLTS->MakeVal(to_string(i), FloorType);
        vector<LTSAssignRef> RequestUpdates;
        RequestUpdates.push_back(new LTSAssignSimple(FloorAccExp, FloorVal));
        User->AddOutputTransition("InitialState", "WaitingState", TheLTS->MakeTrue(), RequestUpdates, "Request", MsgTypes["Request"], {}, EmptyFairnessSets);
    }
    User->AddInputTransition("WaitingState", "InitialState", TheLTS->MakeTrue(), vector<LTSAssignRef>(), "Arrive", MsgTypes["Arrive"], {});
}

void AddControllerAutomaton(LabelledTS* TheLTS, map<string, ExprTypeRef>& MsgTypes, int TopFloor)
{
    set<string> EmptyFairnessSets;
    auto FAType = TheLTS->MakeFieldAccessType();
    auto FloorType = TheLTS->MakeRangeType(1, TopFloor + 1);
    auto TopFloorExp = TheLTS->MakeVal(to_string(TopFloor), FloorType);
    auto FloorOneExp = TheLTS->MakeVal("1", FloorType);
    auto FloorTwoExp = TheLTS->MakeVal("2", FloorType);
    auto FloorAccFieldExp = TheLTS->MakeVar("Floor", TheLTS->MakeFieldAccessType());
    auto RequestMsgType = MsgTypes["Request"];
    auto RequestMsgExp = TheLTS->MakeVar("Request", RequestMsgType);
    auto FloorAccExp = TheLTS->MakeOp(LTSOps::OpField, RequestMsgExp, FloorAccFieldExp);
    auto ArriveMsgType = MsgTypes["Arrive"];
    auto UpMsgType = MsgTypes["Up"];
    auto UpAckMsgType = MsgTypes["UpAck"];
    auto DownMsgType = MsgTypes["Down"];
    auto DownAckMsgType = MsgTypes["DownAck"];
    auto Controller = TheLTS->MakeEFSM<IncompleteEFSM>("Controller", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    Controller->AddState("Initial");
    Controller->AddState("CheckRequest"); //, false, false, false, true);
    Controller->AddState("SentUp");
    Controller->AddState("SentDown");
    // Controller->AddState("Dummy");
    Controller->FreezeStates();
    Controller->AddVariable("CurrentFloor", FloorType);
    Controller->AddVariable("TargetFloor", FloorType);

    Controller->AddInputMsg(RequestMsgType, {});
    Controller->AddInputMsg(UpAckMsgType, {});
    Controller->AddInputMsg(DownAckMsgType, {});
    Controller->AddOutputMsg(UpMsgType, {});
    Controller->AddOutputMsg(DownMsgType, {});
    Controller->AddOutputMsg(ArriveMsgType, {});

    auto CurrentFloorExp = TheLTS->MakeVar("CurrentFloor", FloorType);
    auto TargetFloorExp = TheLTS->MakeVar("TargetFloor", FloorType);
    auto ControllerGuardCurrentFloorLTTargetFloor = TheLTS->MakeOp(LTSOps::OpLT, CurrentFloorExp, TargetFloorExp);
    auto ControllerGuardCurrentFloorEQTargetFloor = TheLTS->MakeOp(LTSOps::OpEQ, CurrentFloorExp, TargetFloorExp);
    auto CurrentFloorPlusOneExp = TheLTS->MakeOp(LTSOps::OpADD, CurrentFloorExp, FloorOneExp);
    auto CurrentFloorPlusTwoExp = TheLTS->MakeOp(LTSOps::OpADD, CurrentFloorExp, FloorTwoExp);
    auto CurrentFloorMinusOneExp = TheLTS->MakeOp(LTSOps::OpSUB, CurrentFloorExp, FloorOneExp);
    vector<LTSAssignRef> CurrentFloorPlusUpdates;
    vector<LTSAssignRef> CurrentFloorPlusTwoUpdates;
    vector<LTSAssignRef> CurrentFloorMinusUpdates;
    CurrentFloorPlusUpdates.push_back(new LTSAssignSimple(CurrentFloorExp, CurrentFloorPlusOneExp));
    CurrentFloorPlusTwoUpdates.push_back(new LTSAssignSimple(CurrentFloorExp, CurrentFloorPlusTwoExp));
    CurrentFloorMinusUpdates.push_back(new LTSAssignSimple(CurrentFloorExp, CurrentFloorMinusOneExp));

    vector<LTSAssignRef> ReadTargetUpdates;
    ReadTargetUpdates.push_back(new LTSAssignSimple(TargetFloorExp, FloorAccExp));

    Controller->FreezeVars();

    Controller->AddInputTransition("Initial", "CheckRequest", TheLTS->MakeTrue(), ReadTargetUpdates, "Request", RequestMsgType, {});

    auto ArriveGuard = TheLTS->MakeOp(LTSOps::OpEQ, TargetFloorExp, CurrentFloorExp);
    auto GoUpGuard = TheLTS->MakeOp(LTSOps::OpGT, TargetFloorExp, CurrentFloorExp);
    auto GoDownGuard = TheLTS->MakeOp(LTSOps::OpLT, TargetFloorExp, CurrentFloorExp);

    Controller->AddOutputTransition("CheckRequest", "Initial", ArriveGuard, {}, "Arrive", ArriveMsgType, {}, EmptyFairnessSets);
    // Controller->AddOutputTransition("Dummy", "Dummy", TheLTS->MakeTrue(), {}, "Arrive", ArriveMsgType, {}, EmptyFairnessSets);
    // Controller->AddInternalTransition("CheckRequest", "CheckRequest", TheLTS->MakeTrue(), {});
    // Controller->AddInputTransition("Dummy", "Dummy", TheLTS->MakeTrue(), {}, "UpAck", MsgTypes["UpAck"], {});
    Controller->AddOutputTransition("CheckRequest", "SentUp", GoUpGuard, {}, "Up", UpMsgType, {}, EmptyFairnessSets);
    // Controller->AddOutputTransition("CheckRequest", "SentDown", GoDownGuard, {}, "Down", DownMsgType, {}, EmptyFairnessSets);
    // Controller->AddInputTransition("SentUp", "CheckRequest", TheLTS->MakeTrue(), CurrentFloorPlusUpdates, "UpAck", UpAckMsgType, {});
    Controller->AddInputTransition("SentDown", "CheckRequest", TheLTS->MakeTrue(), CurrentFloorMinusUpdates, "DownAck", DownAckMsgType, {});
    Controller->SAs<IncompleteEFSM>()->MarkVariableReadOnly("TargetFloor");
}

void AddElevatorAutomaton(LabelledTS* TheLTS, map<string, ExprTypeRef>& MsgTypes, int TopFloor)
{
    set<string> EmptyFairnessSets;
    auto FloorType = TheLTS->MakeRangeType(1, TopFloor + 1);
    auto TopFloorExp = TheLTS->MakeVal(to_string(TopFloor + 1), FloorType);
    auto FloorOneExp = TheLTS->MakeVal("1", FloorType);
    auto FloorTwoExp = TheLTS->MakeVal("2", FloorType);
    vector<LTSAssignRef> {};
    auto FAType = TheLTS->MakeFieldAccessType();
    auto RequestMsgType = MsgTypes["Request"];
    auto RequestMsgExp = TheLTS->MakeVar("Request", RequestMsgType);
    auto ArriveMsgType = MsgTypes["Arrive"];
    auto UpMsgType = MsgTypes["Up"];
    auto DownMsgType = MsgTypes["Down"];
    auto UpAckMsgType = MsgTypes["UpAck"];
    auto DownAckMsgType = MsgTypes["DownAck"];

    auto User = TheLTS->GetEFSMs([&](const EFSMBase* EFSM) {return EFSM->GetName() == "User";});

    auto Controller = TheLTS->GetEFSMs([&](const EFSMBase* EFSM) {return EFSM->GetName() == "Controller";});

    // Elevator
    auto Elevator = TheLTS->MakeGenEFSM("Elevator", vector<ExpT>(), TheLTS->MakeTrue(), LTSFairnessType::None);
    Elevator->AddState("InitialState");
    Elevator->AddState("ReceiveUpState");
    Elevator->AddState("ReceiveDownState");
    Elevator->AddState("ErrorState", false, false, false, true);

    Elevator->FreezeStates();
    Elevator->AddVariable("Floor", FloorType);

    Elevator->AddInputMsg(UpMsgType, {});
    Elevator->AddInputMsg(DownMsgType, {});
    Elevator->AddOutputMsg(UpAckMsgType, {});
    Elevator->AddOutputMsg(DownAckMsgType, {});

    Elevator->FreezeVars();

    auto FloorExp = TheLTS->MakeVar("Floor", FloorType);
    auto FloorPlusOneExp = TheLTS->MakeOp(LTSOps::OpADD, FloorExp, FloorOneExp);
    auto FloorMinusOneExp = TheLTS->MakeOp(LTSOps::OpSUB, FloorExp, FloorOneExp);
    auto ElevatorGuardLTTopFloor = TheLTS->MakeOp(LTSOps::OpLT, FloorExp, TopFloorExp);
    auto FloorMinusUpdate = new LTSAssignSimple(FloorExp, FloorMinusOneExp);
    auto FloorPlusUpdate = new LTSAssignSimple(FloorExp, FloorPlusOneExp);
    vector<LTSAssignRef> FloorPlusUpdates;
    FloorPlusUpdates.push_back(FloorPlusUpdate);

    vector<LTSAssignRef> FloorMinusUpdates;
    FloorMinusUpdates.push_back(FloorMinusUpdate);
    Elevator->AddInputTransition("InitialState", "ReceiveUpState", ElevatorGuardLTTopFloor, FloorPlusUpdates, "Up", UpMsgType, {});
    auto ElevatorGuardGTFloorOne = TheLTS->MakeOp(LTSOps::OpGT, FloorExp, FloorOneExp);
    Elevator->AddInputTransition("InitialState", "ReceiveDownState", ElevatorGuardGTFloorOne, FloorMinusUpdates, "Down", DownMsgType, {});

    auto ElevatorGuardEQFloorOne = TheLTS->MakeOp(LTSOps::OpEQ, FloorExp, FloorOneExp);
    Elevator->AddInputTransition("InitialState", "ErrorState", ElevatorGuardEQFloorOne, {}, "Down", DownMsgType, {});
    auto ElevatorGuardEQTopFloor = TheLTS->MakeOp(LTSOps::OpEQ, FloorExp, TopFloorExp);
    Elevator->AddInputTransition("InitialState", "ErrorState", ElevatorGuardEQTopFloor, {}, "Up", UpMsgType, {});

    Elevator->AddOutputTransition("ReceiveUpState", "InitialState", TheLTS->MakeTrue(), {}, "UpAck", UpAckMsgType, {});
    Elevator->AddOutputTransition("ReceiveDownState", "InitialState", TheLTS->MakeTrue(), {}, "DownAck", DownAckMsgType, {});
}

void AddLivenessAutomaton(LabelledTS* TheLTS, map<string, ExprTypeRef>& MsgTypes, int TopFloor)
{
    auto LivenessMonitor = TheLTS->MakeGenEFSM("LivenessMonitor", vector<ExpT>(), TheLTS->MakeTrue(), LTSFairnessType::None);
    LivenessMonitor->AddState("InitialState");
    LivenessMonitor->AddState("AcceptingState");
    LivenessMonitor->AddState("OtherState");
    LivenessMonitor->FreezeStates();
    LivenessMonitor->AddInputMsg(MsgTypes["Request"], {});
    LivenessMonitor->AddInputMsg(MsgTypes["Arrive"], {});
    LivenessMonitor->FreezeVars();
    LivenessMonitor->AddInputTransition("InitialState", "InitialState", TheLTS->MakeTrue(), {}, "Request", MsgTypes["Request"], {});
    LivenessMonitor->AddInputTransition("InitialState", "InitialState", TheLTS->MakeTrue(), {}, "Arrive", MsgTypes["Arrive"], {});
    LivenessMonitor->AddInputTransition("InitialState", "AcceptingState", TheLTS->MakeTrue(), {}, "Request", MsgTypes["Request"], {});
    LivenessMonitor->AddInputTransition("AcceptingState", "AcceptingState", TheLTS->MakeTrue(), {}, "Request", MsgTypes["Request"], {});
    LivenessMonitor->AddInputTransition("AcceptingState", "OtherState", TheLTS->MakeTrue(), {}, "Arrive", MsgTypes["Arrive"], {});
    LivenessMonitor->AddInputTransition("OtherState", "OtherState", TheLTS->MakeTrue(), {}, "Request", MsgTypes["Request"], {});
    LivenessMonitor->AddInputTransition("OtherState", "OtherState", TheLTS->MakeTrue(), {}, "Arrive", MsgTypes["Arrive"], {});
}


// auto const& StateVectorVars = TheLTS->GetStateVectorVars();

//     cout << "LTS Vars:" << endl;
//     for (auto const& Var : StateVectorVars) {
//         cout << Var->ToString() << " : " << endl;
//         cout << Var->GetType()->ToString() << endl;
//     }

//     cout << "State vector size is " << TheLTS->GetStateVectorSize() << " bytes." << endl;


//     cout << "Guarded Commands:" << endl;
//     auto const& GCmds = TheLTS->GetGuardedCmds();
//     for (auto const& GCmd : GCmds) {
//         cout << GCmd->ToString() << endl;
//     }

//     cout << "Initial State Generators:" << endl;
//     auto const& InitStateGens = TheLTS->GetInitStateGenerators();
//     for (auto const& InitStateGen : InitStateGens) {
//         cout << "InitState {" << endl;
//         for (auto const& Update : InitStateGen) {
//             cout << "    " << Update->ToString() << endl;
//         }
//         cout << "}" << endl;
//     }

//     cout << "Invariant:" << endl;
//     cout << TheLTS->GetInvariant() << endl;

    // cout << "Channel Buffer variables to sort:" << endl;
    // for (auto const& BufferExp : TheLTS->GetChanBuffersToSort()) {
    //     cout << BufferExp.first->ToString() << endl;
    //     cout << BufferExp.second->ToString() << endl;
    // }

int main()
{
    int TopFloor = TOPFLOOR;
    auto TheLTS = new LabelledTS();

    map<string, ExprTypeRef> MsgTypes;

    set<string> EmptyFairnessSets;
    auto FloorType = TheLTS->MakeRangeType(1, TopFloor + 1);
    auto TopFloorExp = TheLTS->MakeVal(to_string(TopFloor), FloorType);
    auto FloorOneExp = TheLTS->MakeVal("1", FloorType);
    auto FloorTwoExp = TheLTS->MakeVal("2", FloorType);

    MsgTypes["Request"] = TheLTS->MakeMsgType("Request", {make_pair("Floor", FloorType)}, false);
    MsgTypes["Arrive"] = TheLTS->MakeMsgType("Arrive", {make_pair("Data", TheLTS->MakeBoolType())}, false);
    MsgTypes["Up"] = TheLTS->MakeMsgType("Up", {make_pair("Data", TheLTS->MakeBoolType())}, false);
    MsgTypes["Down"] = TheLTS->MakeMsgType("Down", {make_pair("Data", TheLTS->MakeBoolType())}, false);
    MsgTypes["UpAck"] = TheLTS->MakeMsgType("UpAck", {make_pair("Data", TheLTS->MakeBoolType())}, false);
    MsgTypes["DownAck"] = TheLTS->MakeMsgType("DownAck", {make_pair("Data", TheLTS->MakeBoolType())}, false);

    TheLTS->FreezeMsgs();

    AddControllerAutomaton(TheLTS, MsgTypes, TOPFLOOR);
    AddUserAutomaton(TheLTS, MsgTypes, TOPFLOOR);
    AddElevatorAutomaton(TheLTS, MsgTypes, TOPFLOOR);
    AddLivenessAutomaton(TheLTS, MsgTypes, TOPFLOOR);

    // auto LivenessMonitorAcceptingStateValue = TheLTS->MakeVal("AcceptingState", LivenessMonitorDotState->GetType());


    TheLTS->FreezeAutomata();

    InitializeAutomata(TheLTS);

    cout << "Invariant:" << endl;
    cout << TheLTS->GetInvariant() << endl;

    auto Checker = new LTSChecker(TheLTS);

    auto Monitor = Checker->MakeStateBuchiMonitor("RequestToAccept", {}, TheLTS->MakeTrue());
    Monitor->AddState("InitialState", true, false);
    Monitor->AddState("AcceptingState", false, true);
    Monitor->AddState("OtherState", false, false);
    Monitor->FreezeStates();

    auto ControllerType = TheLTS->GetEFSMType("Controller");
    auto ControllerStateVar = TheLTS->MakeVar("Controller", ControllerType);
    auto ControllerDotState = TheLTS->MakeOp(LTSOps::OpField, ControllerStateVar, TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()));
    auto ControllerCheckRequestStateValue = TheLTS->MakeVal("CheckRequest", ControllerDotState->GetType());
    auto ControllerStateEQCheckRequest = TheLTS->MakeOp(LTSOps::OpEQ, ControllerDotState, ControllerCheckRequestStateValue);

    auto ControllerStateNEQCheckRequest = TheLTS->MakeOp(LTSOps::OpNOT, ControllerStateEQCheckRequest);
    Monitor->AddTransition("InitialState", "InitialState", TheLTS->MakeTrue());
    Monitor->AddTransition("InitialState", "AcceptingState", ControllerStateEQCheckRequest);
    Monitor->AddTransition("AcceptingState", "AcceptingState", ControllerStateEQCheckRequest);
    Monitor->AddTransition("AcceptingState", "OtherState", ControllerStateNEQCheckRequest);
    Monitor->AddTransition("OtherState", "OtherState", TheLTS->MakeTrue());
    Monitor->Freeze();

    auto TheSolver = new Solver(Checker);
    
    TheSolver->Solve();

    delete Checker;
    delete TheSolver;
}

//
// Elevator.cpp ends here
