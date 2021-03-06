#if !defined ELEVATOR_LTS_HPP_
#define ELEVATOR_LTS_HPP_

#include "../../src/uflts/LabelledTS.hpp"
#include "../../src/uflts/LTSEFSM.hpp"
#include "../../src/uflts/LTSAssign.hpp"
#include "../../src/uflts/LTSTransitions.hpp"


using namespace ESMC;
using namespace LTS;


LabelledTS* ElevatorLTS() {
    int top_floor = 5;
    auto TheLTS = new LabelledTS();
    set<string> EmptyFairnessSets;

    auto FloorType = TheLTS->MakeRangeType(1, top_floor + 1);
    auto TrueExp = TheLTS->MakeTrue();
    auto TopFloorExp = TheLTS->MakeVal(to_string(top_floor), FloorType);
    auto FloorOneExp = TheLTS->MakeVal(to_string(1), FloorType);

    vector<LTSAssignRef> EmptyUpdates;
    vector<ExpT> EmptyParams;

    auto FAType = TheLTS->MakeFieldAccessType();

    // Message types
    // Request Message
    vector<pair<string, ExprTypeRef>> RequestMsgFields;
    RequestMsgFields.push_back(make_pair("Floor", FloorType));
    auto RequestMsgType = TheLTS->MakeMsgType("Request", RequestMsgFields, false);
    auto RequestMsgExp = TheLTS->MakeVar("Request", RequestMsgType);
    // arrive unit message, not sure how to make unit, just making it bool for now.
    vector<pair<string, ExprTypeRef>> UnitMsgFields;
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
    Controller->AddState("CheckRequest");
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
    auto CurrentFloorMinusOneExp = TheLTS->MakeOp(LTSOps::OpSUB, CurrentFloorExp, FloorOneExp);
    vector<LTSAssignRef> CurrentFloorPlusUpdates;
    vector<LTSAssignRef> CurrentFloorMinusUpdates;
    CurrentFloorPlusUpdates.push_back(new LTSAssignSimple(CurrentFloorExp, CurrentFloorPlusOneExp));
    CurrentFloorMinusUpdates.push_back(new LTSAssignSimple(CurrentFloorExp, CurrentFloorMinusOneExp));

    vector<LTSAssignRef> ReadTargetUpdates;
    ReadTargetUpdates.push_back(new LTSAssignSimple(TargetFloorExp, FloorAccExp));

    Controller->FreezeVars();

    Controller->AddInputTransition("Initial", "CheckRequest", TrueExp, ReadTargetUpdates, "Request", RequestMsgType, EmptyParams);

    auto ArriveGuard = TheLTS->MakeOp(LTSOps::OpEQ, TargetFloorExp, CurrentFloorExp);
    auto GoUpGuard = TheLTS->MakeOp(LTSOps::OpGE, TargetFloorExp, CurrentFloorExp);
    auto GoDownGuard = TheLTS->MakeOp(LTSOps::OpLT, TargetFloorExp, CurrentFloorExp);

    Controller->AddOutputTransition("Dummy", "Dummy", ArriveGuard, EmptyUpdates, "Arrive", ArriveMsgType, EmptyParams, EmptyFairnessSets);

    Controller->AddOutputTransition("CheckRequest", "SentUp", GoUpGuard, EmptyUpdates, "Up", UpMsgType, EmptyParams, EmptyFairnessSets);
    Controller->AddOutputTransition("Dummy", "Dummy", TrueExp, {}, "Up", UpMsgType, {}, EmptyFairnessSets);
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

    TheLTS->FreezeAutomata();

    vector<InitStateRef> InitStates;
    vector<LTSAssignRef> InitUpdates;

    auto ElevatorType = TheLTS->GetEFSMType("Elevator");
    auto ControllerType = TheLTS->GetEFSMType("Controller");
    auto UserType = TheLTS->GetEFSMType("User");

    auto ElevatorStateVar = TheLTS->MakeVar("Elevator", ElevatorType);
    auto ControllerStateVar = TheLTS->MakeVar("Controller", ControllerType);
    auto UserStateVar = TheLTS->MakeVar("User", UserType);

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


    InitStates.push_back(new LTSInitState(EmptyParams, TrueExp, InitUpdates));
    TheLTS->AddInitStates(InitStates);

    TheLTS->Freeze();
    return TheLTS;
}

#endif /* ELEVATOR_LTS_HPP_ */
