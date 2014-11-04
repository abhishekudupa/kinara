#include <algorithm>
#include <boost/algorithm/string/predicate.hpp>

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


int main()
{
    int TopFloorInt = 3;
    auto TheLTS = new LabelledTS();
    auto True = TheLTS->MakeTrue();
    map<string, ExprTypeRef> Msgs;
    auto FloorType = TheLTS->MakeRangeType(1, TopFloorInt);
    auto TopFloor = TheLTS->MakeVal(to_string(TopFloorInt), FloorType);

    Msgs["Request"] = TheLTS->MakeMsgType("Request", {make_pair("Floor", FloorType)}, false);
    Msgs["Arrive"] = TheLTS->MakeMsgType("Arrive", {}, false);
    Msgs["Up"] = TheLTS->MakeMsgType("Up", {}, false);
    Msgs["Down"] = TheLTS->MakeMsgType("Down", {}, false);
    Msgs["UpAck"] = TheLTS->MakeMsgType("UpAck", {}, false);
    Msgs["DownAck"] = TheLTS->MakeMsgType("DownAck", {}, false);
    TheLTS->FreezeMsgs();
    auto FAType = TheLTS->MakeFieldAccessType();
    auto FloorField = TheLTS->MakeVar("Floor", FAType);
    auto RequestMsgVar = TheLTS->MakeVar("Request", Msgs["Request"]);

    ////////////////////////////////////////////////////////////
    // Controller
    ////////////////////////////////////////////////////////////
    auto Controller = TheLTS->MakeEFSM<IncompleteEFSM>("Controller", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    Controller->AddState("Initial");
    Controller->AddState("CheckRequest");
    Controller->AddState("SentUp");
    Controller->AddState("SentDown");
    Controller->FreezeStates();
    Controller->AddVariable("CurrentFloor", FloorType);
    Controller->AddVariable("TargetFloor", FloorType);
    Controller->AddInputMsg(Msgs["Request"], {});
    Controller->AddInputMsg(Msgs["UpAck"], {});
    Controller->AddInputMsg(Msgs["DownAck"], {});
    Controller->AddOutputMsg(Msgs["Up"], {});
    Controller->AddOutputMsg(Msgs["Down"], {});
    Controller->AddOutputMsg(Msgs["Arrive"], {});
    auto CurrentFloor = TheLTS->MakeVar("CurrentFloor", FloorType);
    Controller->FreezeVars();
    auto TargetFloor = TheLTS->MakeVar("TargetFloor", FloorType);
    auto RequestFloor = TheLTS->MakeOp(LTSOps::OpField, RequestMsgVar, FloorField);
    vector<LTSAssignRef> Updates = {new LTSAssignSimple(TargetFloor, RequestFloor)};
    Controller->AddInputTransition("Initial", "CheckRequest",
                                   True, Updates,
                                   "Request", Msgs["Request"], {});

    // auto TargetFloorEQCurrentFloor = TheLTS->MakeOp(LTSOps::OpEQ, TargetFloor, CurrentFloor);
    // Controller->AddOutputTransition("CheckRequest", "Initial",
    //                                 TargetFloorEQCurrentFloor, {},
    //                                 "Arrive", Msgs["Arrive"], {}, set<string>());


    auto TargetFloorGTCurrentFloor = TheLTS->MakeOp(LTSOps::OpGT, TargetFloor, CurrentFloor);
    Controller->AddOutputTransition("CheckRequest", "SentUp",
                                   TargetFloorGTCurrentFloor, {},
                                    "Up", Msgs["Up"], {}, set<string>());

    auto TargetFloorLTCurrentFloor = TheLTS->MakeOp(LTSOps::OpLT, TargetFloor, CurrentFloor);
    Controller->AddOutputTransition("CheckRequest", "SentDown",
                                    TargetFloorLTCurrentFloor, {},
                                    "Down", Msgs["Down"], {}, set<string>());



    auto FloorOne = TheLTS->MakeVal("1", FloorType);
    Updates.clear();
    auto CurrentFloorPlusOne = TheLTS->MakeOp(LTSOps::OpADD, CurrentFloor, FloorOne);
    Updates = {new LTSAssignSimple(CurrentFloor, CurrentFloorPlusOne)};
    Controller->AddInputTransition("SentUp", "CheckRequest",
                                   True, Updates,
                                   "UpAck", Msgs["UpAck"], {});

    auto CurrentFloorMinusOne = TheLTS->MakeOp(LTSOps::OpSUB, CurrentFloor, FloorOne);
    Updates.clear();
    Updates = {new LTSAssignSimple(CurrentFloor, CurrentFloorMinusOne)};

    Controller->AddInputTransition("SentDown", "CheckRequest",
                                   True, Updates,
                                   "DownAck", Msgs["DownAck"], {});
    Updates.clear();

    Controller->SAs<IncompleteEFSM>()->MarkVariableReadOnly("TargetFloor");

    ////////////////////////////////////////////////////////////
    // User
    ////////////////////////////////////////////////////////////
    auto User = TheLTS->MakeGenEFSM("User", {}, True, LTSFairnessType::None);
    User->AddState("InitialState");
    User->AddState("WaitingState");
    User->FreezeStates();
    User->AddInputMsg(Msgs["Arrive"], {});
    User->AddOutputMsg(Msgs["Request"], {});
    User->FreezeVars();
    for (auto i = 1; i <= TopFloorInt; ++i) {
        auto FloorVal = TheLTS->MakeVal(to_string(i), FloorType);
        Updates.clear();
        Updates = {new LTSAssignSimple(RequestFloor, FloorVal)};
        User->AddOutputTransition("InitialState", "WaitingState",
                                  True, Updates,
                                  "Request", Msgs["Request"], {}, set<string>());
    }
    User->AddInputTransition("WaitingState", "InitialState",
                             True, {},
                             "Arrive", Msgs["Arrive"], {});

    ////////////////////////////////////////////////////////////
    // Elevator
    ////////////////////////////////////////////////////////////
    auto Elevator = TheLTS->MakeGenEFSM("Elevator", vector<ExpT>(), True, LTSFairnessType::None);
    Elevator->AddState("InitialState");
    Elevator->AddState("ReceiveUpState");
    Elevator->AddState("ReceiveDownState");
    Elevator->AddState("ErrorState", false, false, false, true);
    Elevator->FreezeStates();
    Elevator->AddVariable("Floor", FloorType);
    Elevator->AddInputMsg(Msgs["Up"], {});
    Elevator->AddInputMsg(Msgs["Down"], {});
    Elevator->AddOutputMsg(Msgs["UpAck"], {});
    Elevator->AddOutputMsg(Msgs["DownAck"], {});
    Elevator->FreezeVars();
    auto Floor = TheLTS->MakeVar("Floor", FloorType);
    auto FloorPlusOne = TheLTS->MakeOp(LTSOps::OpADD, Floor, FloorOne);
    auto FloorMinusOne = TheLTS->MakeOp(LTSOps::OpSUB, Floor, FloorOne);
    auto FloorLTTopFloor = TheLTS->MakeOp(LTSOps::OpLT, Floor, TopFloor);
    Updates.clear();
    Updates = {new LTSAssignSimple(Floor, FloorPlusOne)};
    Elevator->AddInputTransition("InitialState", "ReceiveUpState",
                                 FloorLTTopFloor, Updates,
                                 "Up", Msgs["Up"], {});
    Updates.clear();
    Updates = {new LTSAssignSimple(Floor, FloorMinusOne)};
    auto FloorGTFloorOne = TheLTS->MakeOp(LTSOps::OpGT, Floor, FloorOne);
    Elevator->AddInputTransition("InitialState", "ReceiveDownState",
                                 FloorGTFloorOne, Updates,
                                 "Down", Msgs["Down"], {});
    auto FloorEQFloorOne = TheLTS->MakeOp(LTSOps::OpEQ, Floor, FloorOne);
    Elevator->AddInputTransition("InitialState", "ErrorState",
                                 FloorEQFloorOne, {},
                                 "Down", Msgs["Down"], {});
    auto FloorEQTopFloor = TheLTS->MakeOp(LTSOps::OpEQ, Floor, TopFloor);
    Elevator->AddInputTransition("InitialState", "ErrorState",
                                 FloorEQTopFloor, {},
                                 "Up", Msgs["Up"], {});
    Elevator->AddOutputTransition("ReceiveUpState", "InitialState",
                                  True, {},
                                  "UpAck", Msgs["UpAck"], {});
    Elevator->AddOutputTransition("ReceiveDownState", "InitialState",
                                  True, {},
                                  "DownAck", Msgs["DownAck"], {});
    ////////////////////////////////////////////////////////////
    // Liveness Message Monitor
    ////////////////////////////////////////////////////////////
    auto LivenessMonitor = TheLTS->MakeGenEFSM("LivenessMonitor", {}, True, LTSFairnessType::None);
    LivenessMonitor->AddState("InitialState");
    LivenessMonitor->AddState("AcceptingState");
    LivenessMonitor->AddState("OtherState");
    LivenessMonitor->FreezeStates();
    LivenessMonitor->AddInputMsg(Msgs["Request"], {});
    LivenessMonitor->AddInputMsg(Msgs["Arrive"], {});
    LivenessMonitor->FreezeVars();
    LivenessMonitor->AddInputTransition("InitialState", "InitialState",
                                        True, {},
                                        "Request", Msgs["Request"], {});
    LivenessMonitor->AddInputTransition("InitialState", "InitialState",
                                        True, {},
                                        "Arrive", Msgs["Arrive"], {});
    LivenessMonitor->AddInputTransition("InitialState", "AcceptingState",
                                        True, {},
                                        "Request", Msgs["Request"], {});
    LivenessMonitor->AddInputTransition("AcceptingState", "AcceptingState",
                                        True, {},
                                        "Request", Msgs["Request"], {});
    LivenessMonitor->AddInputTransition("AcceptingState", "OtherState",
                                        True, {},
                                        "Arrive", Msgs["Arrive"], {});
    LivenessMonitor->AddInputTransition("OtherState", "OtherState",
                                        True, {},
                                        "Request", Msgs["Request"], {});
    LivenessMonitor->AddInputTransition("OtherState", "OtherState",
                                        True, {},
                                        "Arrive", Msgs["Arrive"], {});

    ////////////////////////////////////////////////////////////
    // Safety Monitor: Request is followed by Arrive
    ////////////////////////////////////////////////////////////
    auto SafetyMonitor = TheLTS->MakeGenEFSM("SafetyMonitor", {}, True, LTSFairnessType::None);
    SafetyMonitor->AddState("InitialState");
    SafetyMonitor->AddState("RequestState");
    SafetyMonitor->AddState("ErrorState", false, false, false, true);
    SafetyMonitor->FreezeStates();
    SafetyMonitor->AddInputMsg(Msgs["Request"], {});
    SafetyMonitor->AddInputMsg(Msgs["Arrive"], {});
    SafetyMonitor->FreezeVars();
    SafetyMonitor->AddInputTransition("InitialState", "RequestState",
                                      True, {},
                                      "Request", Msgs["Request"], {});
    SafetyMonitor->AddInputTransition("InitialState", "ErrorState",
                                      True, {},
                                      "Arrive", Msgs["Arrive"], {});
    SafetyMonitor->AddInputTransition("RequestState", "InitialState",
                                      True, {},
                                      "Arrive", Msgs["Arrive"], {});
    SafetyMonitor->AddInputTransition("RequestState", "ErrorState",
                                      True, {},
                                      "Request", Msgs["Request"], {});
    SafetyMonitor->AddInputTransition("ErrorState", "ErrorState",
                                      True, {},
                                      "Request", Msgs["Request"], {});
    SafetyMonitor->AddInputTransition("ErrorState", "ErrorState",
                                      True, {},
                                      "Arrive", Msgs["Arrive"], {});

    TheLTS->FreezeAutomata();

    auto ElevatorType = TheLTS->GetEFSMType("Elevator");
    auto ElevatorStateVar = TheLTS->MakeVar("Elevator", ElevatorType);
    auto ElevatorState = TheLTS->MakeOp(LTSOps::OpField, ElevatorStateVar, TheLTS->MakeVar("state", FAType));
    auto ElevatorFloor = TheLTS->MakeOp(LTSOps::OpField, ElevatorStateVar, TheLTS->MakeVar("Floor", FAType));

    auto ControllerType = TheLTS->GetEFSMType("Controller");
    auto ControllerStateVar = TheLTS->MakeVar("Controller", ControllerType);
    auto ControllerState = TheLTS->MakeOp(LTSOps::OpField, ControllerStateVar, TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()));
    auto ControllerCurrentFloor = TheLTS->MakeOp(LTSOps::OpField, ControllerStateVar, TheLTS->MakeVar("CurrentFloor", TheLTS->MakeFieldAccessType()));
    auto ControllerTargetFloor = TheLTS->MakeOp(LTSOps::OpField, ControllerStateVar, TheLTS->MakeVar("TargetFloor", TheLTS->MakeFieldAccessType()));
    auto ControllerInitialState = TheLTS->MakeVal("Initial", ControllerState->GetType());

    auto SafetyMonitorType = TheLTS->GetEFSMType("SafetyMonitor");
    auto SafetyMonitorStateVar = TheLTS->MakeVar("SafetyMonitor", SafetyMonitorType);
    auto SafetyMonitorState = TheLTS->MakeOp(LTSOps::OpField, SafetyMonitorStateVar, TheLTS->MakeVar("state", FAType));
    auto SafetyMonitorInitialState = TheLTS->MakeVal("InitialState", SafetyMonitorState->GetType());

    ////////////////////////////////////////////////////////////////////////////////
    // Controller Target Floor = Elevator Floor Invariant at Initial State
    ////////////////////////////////////////////////////////////////////////////////
    auto SafetyMonitorStateEQInitialState = TheLTS->MakeOp(LTSOps::OpEQ,
                                                           SafetyMonitorState,
                                                           SafetyMonitorInitialState);
    auto ControllerTargetEQElevatorFloor = TheLTS->MakeOp(LTSOps::OpEQ,
                                                          ControllerTargetFloor,
                                                          ElevatorFloor);

    TheLTS->AddInvariant(TheLTS->MakeOp(LTSOps::OpIMPLIES,
                                        SafetyMonitorStateEQInitialState,
                                        ControllerTargetEQElevatorFloor));

    vector<InitStateRef> InitStates;
    vector<LTSAssignRef> InitUpdates;

    auto UserType = TheLTS->GetEFSMType("User");
    auto UserStateVar = TheLTS->MakeVar("User", UserType);

    auto LivenessMonitorType = TheLTS->GetEFSMType("LivenessMonitor");
    auto LivenessMonitorStateVar = TheLTS->MakeVar("LivenessMonitor", LivenessMonitorType);
    auto LivenessMonitorState = TheLTS->MakeOp(LTSOps::OpField, LivenessMonitorStateVar, TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()));
    auto LivenessMonitorInitialState = TheLTS->MakeVal("InitialState", LivenessMonitorState->GetType());
    InitUpdates.push_back(new LTSAssignSimple(LivenessMonitorState, LivenessMonitorInitialState));


    InitUpdates.push_back(new LTSAssignSimple(ElevatorState, TheLTS->MakeVal("InitialState", ElevatorState->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(ElevatorFloor, TheLTS->MakeVal("1", ElevatorFloor->GetType())));

    InitUpdates.push_back(new LTSAssignSimple(ControllerState, ControllerInitialState));
    InitUpdates.push_back(new LTSAssignSimple(ControllerCurrentFloor, TheLTS->MakeVal("1", ControllerCurrentFloor->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(ControllerTargetFloor, TheLTS->MakeVal("1", ControllerTargetFloor->GetType())));
    auto UserDotState = TheLTS->MakeOp(LTSOps::OpField, UserStateVar, TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()));
    InitUpdates.push_back(new LTSAssignSimple(UserDotState, TheLTS->MakeVal("InitialState", UserDotState->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(SafetyMonitorState, SafetyMonitorInitialState));

    InitStates.push_back(new LTSInitState({}, True, InitUpdates));
    TheLTS->AddInitStates(InitStates);
    TheLTS->Freeze();
    cout << "Invariant:" << endl;
    cout << TheLTS->GetInvariant() << endl;
    auto Checker = new LTSChecker(TheLTS);
    auto Monitor = Checker->MakeStateBuchiMonitor("RequestToAccept", {}, TheLTS->MakeTrue());
    Monitor->AddState("InitialState", true, false);
    Monitor->AddState("AcceptingState", false, true);
    Monitor->AddState("OtherState", false, false);
    Monitor->FreezeStates();

    auto LivenessMonitorAcceptingState = TheLTS->MakeVal("AcceptingState", LivenessMonitorState->GetType());
    auto LivenessMonitorStateEQAcceptingState = TheLTS->MakeOp(LTSOps::OpEQ, LivenessMonitorState, LivenessMonitorAcceptingState);
    auto LivenessMonitorStateNEQAcceptingState = TheLTS->MakeOp(LTSOps::OpNOT, LivenessMonitorStateEQAcceptingState);
    Monitor->AddTransition("InitialState", "InitialState", True);
    Monitor->AddTransition("InitialState", "AcceptingState", LivenessMonitorStateEQAcceptingState);
    Monitor->AddTransition("AcceptingState", "AcceptingState", LivenessMonitorStateEQAcceptingState);
    Monitor->AddTransition("AcceptingState", "OtherState", LivenessMonitorStateNEQAcceptingState);
    Monitor->AddTransition("OtherState", "OtherState", True);
    Monitor->Freeze();
    auto TheSolver = new Solver(Checker);

    ////////////////////////////////////////////////////////////
    // Constraint of Correct Solution
    ////////////////////////////////////////////////////////////
    auto ArriveTransition = Controller->GetOutputTransitionsOnMsg(Msgs["Arrive"])[0];
    vector<ExpT> SolutionConjuncts;
    auto HasUF = [&] (const ExpBaseT* Exp) -> bool
        {
            auto ExpAsOpExp = Exp->As<OpExpression>();
            if (ExpAsOpExp != nullptr) {
                auto Code = ExpAsOpExp->GetOpCode();
                return (Code >= LTSOps::UFOffset);
            }
            return false;
        };
    auto Guard = ArriveTransition->GetGuard();
    auto UFFunctionsInGuard = Guard->GetMgr()->Gather(Guard, HasUF);
    auto UFGuard = *(UFFunctionsInGuard.begin());
    auto UFGuardOpCode = UFGuard->As<OpExpression>()->GetOpCode();
    for (u32 i = 1; i <= 3; ++i) {
        auto GuardTrue = TheLTS->MakeOp(UFGuardOpCode,
                                        {TheLTS->MakeVal(to_string(i), FloorType),
                                                TheLTS->MakeVal(to_string(i), FloorType)});
        SolutionConjuncts.push_back(GuardTrue);
    }
    Updates = ArriveTransition->GetUpdates();
    for (auto Update : Updates) {
        auto LHS = Update->GetLHS();
        auto RHS = Update->GetRHS();
        if (boost::algorithm::ends_with(LHS->ToString(), "state")) {
            auto StateUpdateOpCode = RHS->As<OpExpression>()->GetOpCode();
            cout << "State update op code is " << StateUpdateOpCode << endl;
            for (u32 i = 1; i <= 3; ++i) {
                auto StateUpdate = TheLTS->MakeOp(StateUpdateOpCode,
                                                  {TheLTS->MakeVal(to_string(i), FloorType),
                                                          TheLTS->MakeVal(to_string(i), FloorType)});
                auto StateUpdateConstraint = TheLTS->MakeOp(LTSOps::OpEQ,
                                                            StateUpdate,
                                                            ControllerInitialState);
                SolutionConjuncts.push_back(StateUpdateConstraint);
            }
        }
    }
    auto Solution = TheLTS->MakeOp(LTSOps::OpAND, SolutionConjuncts);
    cout << "THE SOLUTION PREDICATE IS " << Solution << endl;

    TheSolver->Solve();
    TheSolver->PrintSolution();

    delete Checker;
    delete TheSolver;
}




