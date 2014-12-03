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

#include "../../src/uflts/LabelledTS.hpp"
#include "../../src/uflts/LTSEFSM.hpp"
#include "../../src/uflts/LTSChannelEFSM.hpp"
#include "../../src/uflts/LTSAssign.hpp"
#include "../../src/uflts/LTSTransitions.hpp"
#include "../../src/mc/LTSChecker.hpp"
#include "../../src/mc/OmegaAutomaton.hpp"
#include "../../src/mc/Trace.hpp"
#include "../../src/symexec/LTSAnalyses.hpp"


using namespace ESMC;
using namespace LTS;
using namespace Exprs;
using namespace MC;
using namespace Analyses;



int main()
{
    int top_floor = 3;
    auto TheLTS = new LabelledTS();
    set<string> EmptyFairnessSets;
    auto FloorType = TheLTS->MakeRangeType(1, top_floor + 1);
    auto TrueExp = TheLTS->MakeTrue();
    auto TopFloorExp = TheLTS->MakeVal(to_string(top_floor), FloorType);
    auto FloorOneExp = TheLTS->MakeVal(to_string(1), FloorType);
    auto FloorTwoExp = TheLTS->MakeVal(to_string(2), FloorType);


    vector<LTSAssignRef> EmptyUpdates;
    vector<ExpT> EmptyParams;

    auto FAType = TheLTS->MakeFieldAccessType();

    // Message types
    // Request Message
    vector<pair<string, TypeRef>> RequestMsgFields;
    RequestMsgFields.push_back(make_pair("Floor", FloorType));
    auto RequestMsgType = TheLTS->MakeMsgType("Request", RequestMsgFields, false);
    auto RequestMsgExp = TheLTS->MakeVar("Request", RequestMsgType);
    // arrive unit message, not sure how to make unit, just making it bool for now.
    vector<pair<string, TypeRef>> UnitMsgFields;
    auto BoolType = TheLTS->MakeBoolType();
    UnitMsgFields.push_back(make_pair("Data", BoolType));
    // arrive unit message
    auto ArriveMsgType = TheLTS->MakeMsgType("Arrive", UnitMsgFields, false);
    // up unit message
    auto UpMsgType = TheLTS->MakeMsgType("Up", UnitMsgFields, false);
    // down unit message
    auto DownMsgType = TheLTS->MakeMsgType("Down", UnitMsgFields, false);
    // up-ack unit message
    auto UpAckMsgType = TheLTS->MakeMsgType("UpAck", UnitMsgFields, false);
    // down-ack unit message
    auto DownAckMsgType = TheLTS->MakeMsgType("DownAck", UnitMsgFields, false);

    TheLTS->FreezeMsgs();

    auto User = TheLTS->MakeGenEFSM("User", vector<ExpT>(), TrueExp, LTSFairnessType::None);
    User->AddState("InitialState");
    User->AddState("WaitingState");
    User->FreezeStates();
    User->AddInputMsg(ArriveMsgType, EmptyParams);
    User->AddOutputMsg(RequestMsgType, EmptyParams);

    vector<ExpT> UserConstantFloorVariables;

    auto FloorAccFieldExp = TheLTS->MakeVar("Floor", FAType);
    auto FloorAccExp = TheLTS->MakeOp(LTSOps::OpField, RequestMsgExp, FloorAccFieldExp);

    User->FreezeVars();
    for (auto i = 1; i <= top_floor; ++i) {
        auto FloorVal = TheLTS->MakeVal(to_string(i), FloorType);
        vector<LTSAssignRef> RequestUpdates;
        RequestUpdates.push_back(new LTSAssignSimple(FloorAccExp, FloorVal));
        User->AddOutputTransition("InitialState", "WaitingState", TrueExp, RequestUpdates, "Request", RequestMsgType, EmptyParams, EmptyFairnessSets);
    }
    User->AddInputTransition("WaitingState", "InitialState", TrueExp, vector<LTSAssignRef>(), "Arrive", ArriveMsgType, vector<ExpT>());

    // Controller
    auto Controller = TheLTS->MakeGenEFSM("Controller", vector<ExpT>(), TrueExp, LTSFairnessType::None);
    Controller->AddState("Initial");
    Controller->AddState("CheckRequest"); //, false, false, false, true);
    Controller->AddState("SentUp");
    Controller->AddState("SentDown");
    Controller->AddState("Dummy");
    Controller->FreezeStates();
    Controller->AddVariable("CurrentFloor", FloorType);
    Controller->AddVariable("TargetFloor", FloorType);

    Controller->AddInputMsg(RequestMsgType, EmptyParams);
    Controller->AddInputMsg(UpAckMsgType, EmptyParams);
    Controller->AddInputMsg(DownAckMsgType, EmptyParams);
    Controller->AddOutputMsg(UpMsgType, EmptyParams);
    Controller->AddOutputMsg(DownMsgType, EmptyParams);
    Controller->AddOutputMsg(ArriveMsgType, EmptyParams);


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

    Controller->AddInputTransition("Initial", "CheckRequest", TrueExp, ReadTargetUpdates, "Request", RequestMsgType, EmptyParams);

    auto ArriveGuard = TheLTS->MakeOp(LTSOps::OpEQ, TargetFloorExp, CurrentFloorExp);
    auto GoUpGuard = TheLTS->MakeOp(LTSOps::OpGT, TargetFloorExp, CurrentFloorExp);
    auto GoDownGuard = TheLTS->MakeOp(LTSOps::OpLT, TargetFloorExp, CurrentFloorExp);

    // Controller->AddOutputTransition("CheckRequest", "Initial", ArriveGuard, EmptyUpdates, "Arrive", ArriveMsgType, EmptyParams, EmptyFairnessSets);
    Controller->AddOutputTransition("Dummy", "Dummy", TrueExp, EmptyUpdates, "Arrive", ArriveMsgType, EmptyParams, EmptyFairnessSets);
    Controller->AddInternalTransition("CheckRequest", "CheckRequest", TrueExp, EmptyUpdates);
    Controller->AddOutputTransition("CheckRequest", "SentUp", GoUpGuard, EmptyUpdates, "Up", UpMsgType, EmptyParams, EmptyFairnessSets);
    Controller->AddOutputTransition("CheckRequest", "SentDown", GoDownGuard, EmptyUpdates, "Down", DownMsgType, EmptyParams, EmptyFairnessSets);
    Controller->AddInputTransition("SentUp", "CheckRequest", TrueExp, CurrentFloorPlusUpdates, "UpAck", UpAckMsgType, EmptyParams);
    Controller->AddInputTransition("SentDown", "CheckRequest", TrueExp, CurrentFloorMinusUpdates, "DownAck", DownAckMsgType, EmptyParams);

    // Elevator
    auto Elevator = TheLTS->MakeGenEFSM("Elevator", vector<ExpT>(), TrueExp, LTSFairnessType::None);
    Elevator->AddState("InitialState");
    Elevator->AddState("ReceiveUpState");
    Elevator->AddState("ReceiveDownState");
    Elevator->AddState("ErrorState", false, false, false, true);

    Elevator->FreezeStates();
    Elevator->AddVariable("Floor", FloorType);

    Elevator->AddInputMsg(UpMsgType, EmptyParams);
    Elevator->AddInputMsg(DownMsgType, EmptyParams);
    Elevator->AddOutputMsg(UpAckMsgType, EmptyParams);
    Elevator->AddOutputMsg(DownAckMsgType, EmptyParams);

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
    Elevator->AddInputTransition("InitialState", "ReceiveUpState", ElevatorGuardLTTopFloor, FloorPlusUpdates, "Up", UpMsgType, EmptyParams);
    auto ElevatorGuardGTFloorOne = TheLTS->MakeOp(LTSOps::OpGT, FloorExp, FloorOneExp);
    Elevator->AddInputTransition("InitialState", "ReceiveDownState", ElevatorGuardGTFloorOne, FloorMinusUpdates, "Down", DownMsgType, EmptyParams);

    auto ElevatorGuardEQFloorOne = TheLTS->MakeOp(LTSOps::OpEQ, FloorExp, FloorOneExp);
    Elevator->AddInputTransition("InitialState", "ErrorState", ElevatorGuardEQFloorOne, EmptyUpdates, "Down", DownMsgType, EmptyParams);
    auto ElevatorGuardEQTopFloor = TheLTS->MakeOp(LTSOps::OpEQ, FloorExp, TopFloorExp);
    Elevator->AddInputTransition("InitialState", "ErrorState", ElevatorGuardEQTopFloor, EmptyUpdates, "Up", UpMsgType, EmptyParams);

    Elevator->AddOutputTransition("ReceiveUpState", "InitialState", TrueExp, EmptyUpdates, "UpAck", UpAckMsgType, EmptyParams);
    Elevator->AddOutputTransition("ReceiveDownState", "InitialState", TrueExp, EmptyUpdates, "DownAck", DownAckMsgType, EmptyParams);


    // Liveness monitor
    // auto LivenessMonitor = TheLTS->MakeGenEFSM("LivenessMonitor", vector<ExpT>(), TrueExp, LTSFairnessType::None);
    // LivenessMonitor->AddState("InitialState");
    // LivenessMonitor->AddState("AcceptingState");
    // LivenessMonitor->AddState("OtherState");
    // LivenessMonitor->FreezeStates();
    // LivenessMonitor->AddInputMsg(RequestMsgType, EmptyParams);
    // LivenessMonitor->AddInputMsg(ArriveMsgType, EmptyParams);
    // LivenessMonitor->FreezeVars();
    // LivenessMonitor->AddInputTransition("InitialState", "InitialState", TrueExp, EmptyUpdates, "Request", RequestMsgType, EmptyParams);
    // LivenessMonitor->AddInputTransition("InitialState", "InitialState", TrueExp, EmptyUpdates, "Arrive", ArriveMsgType, EmptyParams);
    // LivenessMonitor->AddInputTransition("InitialState", "AcceptingState", TrueExp, EmptyUpdates, "Request", RequestMsgType, EmptyParams);
    // LivenessMonitor->AddInputTransition("AcceptingState", "AcceptingState", TrueExp, EmptyUpdates, "Request", RequestMsgType, EmptyParams);
    // LivenessMonitor->AddInputTransition("AcceptingState", "OtherState", TrueExp, EmptyUpdates, "Arrive", ArriveMsgType, EmptyParams);
    // LivenessMonitor->AddInputTransition("OtherState", "OtherState", TrueExp, EmptyUpdates, "Request", RequestMsgType, EmptyParams);
    // LivenessMonitor->AddInputTransition("OtherState", "OtherState", TrueExp, EmptyUpdates, "Arrive", ArriveMsgType, EmptyParams);

    TheLTS->FreezeAutomata();

    vector<InitStateRef> InitStates;
    vector<LTSAssignRef> InitUpdates;

    auto ElevatorType = TheLTS->GetEFSMType("Elevator");
    auto ControllerType = TheLTS->GetEFSMType("Controller");
    auto UserType = TheLTS->GetEFSMType("User");
    // auto LivenessMonitorType = TheLTS->GetEFSMType("LivenessMonitor");

    auto ElevatorStateVar = TheLTS->MakeVar("Elevator", ElevatorType);
    auto ControllerStateVar = TheLTS->MakeVar("Controller", ControllerType);
    auto UserStateVar = TheLTS->MakeVar("User", UserType);
    // auto LivenessMonitorStateVar = TheLTS->MakeVar("LivenessMonitor", LivenessMonitorType);

    auto ElevatorDotState = TheLTS->MakeOp(LTSOps::OpField, ElevatorStateVar, TheLTS->MakeVar("state", FAType));
    auto ElevatorDotFloor = TheLTS->MakeOp(LTSOps::OpField, ElevatorStateVar, TheLTS->MakeVar("Floor", FAType));

    InitUpdates.push_back(new LTSAssignSimple(ElevatorDotState, TheLTS->MakeVal("InitialState", ElevatorDotState->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(ElevatorDotFloor, TheLTS->MakeVal("1", ElevatorDotFloor->GetType())));

    auto ControllerDotState = TheLTS->MakeOp(LTSOps::OpField, ControllerStateVar, TheLTS->MakeVar("state", FAType));
    auto ControllerDotCurrentFloor = TheLTS->MakeOp(LTSOps::OpField, ControllerStateVar, TheLTS->MakeVar("CurrentFloor", FAType));
    auto ControllerDotTargetFloor = TheLTS->MakeOp(LTSOps::OpField, ControllerStateVar, TheLTS->MakeVar("TargetFloor", FAType));

    InitUpdates.push_back(new LTSAssignSimple(ControllerDotState, TheLTS->MakeVal("Initial", ControllerDotState->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(ControllerDotCurrentFloor, TheLTS->MakeVal("1", ControllerDotCurrentFloor->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(ControllerDotTargetFloor, TheLTS->MakeVal("1", ControllerDotTargetFloor->GetType())));

    auto UserDotState = TheLTS->MakeOp(LTSOps::OpField, UserStateVar, TheLTS->MakeVar("state", FAType));

    InitUpdates.push_back(new LTSAssignSimple(UserDotState, TheLTS->MakeVal("InitialState", UserDotState->GetType())));

    // auto LivenessMonitorDotState = TheLTS->MakeOp(LTSOps::OpField, LivenessMonitorStateVar, TheLTS->MakeVar("state", FAType));

    // auto LivenessMonitorInitialStateValue = TheLTS->MakeVal("InitialState", LivenessMonitorDotState->GetType());

    // auto LivenessMonitorAcceptingStateValue = TheLTS->MakeVal("AcceptingState", LivenessMonitorDotState->GetType());

    // InitUpdates.push_back(new LTSAssignSimple(LivenessMonitorDotState, LivenessMonitorInitialStateValue));

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

    auto Monitor = Checker->MakeStateBuchiMonitor("RequestToAccept", EmptyParams, TrueExp);
    Monitor->AddState("InitialState", true, false);
    Monitor->AddState("AcceptingState", false, true);
    Monitor->AddState("OtherState", false, false);
    Monitor->FreezeStates();

    auto ControllerCheckRequestStateValue = TheLTS->MakeVal("CheckRequest", ControllerDotState->GetType());

    auto ControllerStateEQCheckRequest = TheLTS->MakeOp(LTSOps::OpEQ, ControllerDotState, ControllerCheckRequestStateValue);

    cout << ControllerStateEQCheckRequest->ToString() << endl;

    auto ControllerStateNEQCheckRequest = TheLTS->MakeOp(LTSOps::OpNOT, ControllerStateEQCheckRequest);

    Monitor->AddTransition("InitialState", "InitialState", TrueExp);
    Monitor->AddTransition("InitialState", "AcceptingState", ControllerStateEQCheckRequest);
    Monitor->AddTransition("AcceptingState", "AcceptingState", ControllerStateEQCheckRequest);
    Monitor->AddTransition("AcceptingState", "OtherState", ControllerStateNEQCheckRequest);
    Monitor->AddTransition("OtherState", "OtherState", TrueExp);
    Monitor->Freeze();

    auto Traces = Checker->BuildAQS();

    cout << "Number of traces: " << Traces.size() << endl;

    for (auto Trace: Traces) {
        cout << Trace->ToString() << endl;
        delete Trace;
    }

    Traces = Checker->CheckLiveness("RequestToAccept");

    for (auto const& Trace : Traces) {
        cout << Trace->ToString() << endl;
        vector<vector<MgrT::SubstMapT>> symbolic_states_per_initial;
        auto path_conditions = SymbolicExecution(TheLTS, Trace, symbolic_states_per_initial);
        cout << "Path condition is:" << endl;
        for (auto path_condition: path_conditions) {
            cout << path_condition->ToString() << endl;
        }
        int stem_size = -1;
        if (Trace->Is<LivenessViolation>()) {
            auto TraceAsLivenessViolation = Trace->As<LivenessViolation>();
            auto TraceStemElements = TraceAsLivenessViolation->GetStem();
            auto TraceLoopElements = TraceAsLivenessViolation->GetLoop();
            cout << "size of stem is " << TraceStemElements.size() << endl;
            stem_size = TraceStemElements.size();
        }
        cout << "Step by step symbolic state:" << endl;
        for (auto symbolic_states: symbolic_states_per_initial) {
            int step_counter = 0;
            for (auto symbolic_state: symbolic_states) {
                cout << step_counter << " " << stem_size << endl;
                step_counter += 1;
                if ((stem_size != -1) && (step_counter == stem_size)) {
                    cout << "This is the state that repeats!" << endl;
                }
                for (auto element: symbolic_state) {
                    cout << element.first->ToString() << " = " << element.second->ToString() << endl;
                }
                cout << "----------------------------" << endl;
            }
        }
        cout << "Condition computed by weakest precondition" << endl;
        if (Trace->Is<LivenessViolation>()) {
            auto TraceAsLivenessViolation = Trace->As<LivenessViolation>();
            cout << WeakestPreconditionForLiveness(TheLTS, Monitor, TraceAsLivenessViolation);
        }
        delete Trace;
    }


    delete Checker;
}

//
// Elevator.cpp ends here
