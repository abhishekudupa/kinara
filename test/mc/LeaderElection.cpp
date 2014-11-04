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
    auto TheLTS = new LabelledTS();

    vector<pair<string, ExprTypeRef>> C2Fields;
    C2Fields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 5)));
    auto C2Port = TheLTS->MakeMsgType("C2Port", C2Fields, false);
    auto C2PortExp = TheLTS->MakeVar("C2Port", C2Port);
    vector<pair<string, ExprTypeRef>> Leader1Fields;
    Leader1Fields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 0)));
    auto Leader1Port = TheLTS->MakeMsgType("Leader1Port", Leader1Fields, false);
    auto Leader1PortExp = TheLTS->MakeVar("Leader1Port", Leader1Port);
    vector<pair<string, ExprTypeRef>> C3Fields;
    C3Fields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 5)));
    auto C3Port = TheLTS->MakeMsgType("C3Port", C3Fields, false);
    auto C3PortExp = TheLTS->MakeVar("C3Port", C3Port);
    vector<pair<string, ExprTypeRef>> Leader2Fields;
    Leader2Fields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 0)));
    auto Leader2Port = TheLTS->MakeMsgType("Leader2Port", Leader2Fields, false);
    auto Leader2PortExp = TheLTS->MakeVar("Leader2Port", Leader2Port);
    vector<pair<string, ExprTypeRef>> C1Fields;
    C1Fields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 5)));
    auto C1Port = TheLTS->MakeMsgType("C1Port", C1Fields, false);
    auto C1PortExp = TheLTS->MakeVar("C1Port", C1Port);
    vector<pair<string, ExprTypeRef>> Leader3Fields;
    Leader3Fields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 0)));
    auto Leader3Port = TheLTS->MakeMsgType("Leader3Port", Leader3Fields, false);
    auto Leader3PortExp = TheLTS->MakeVar("Leader3Port", Leader3Port);
    vector<pair<string, ExprTypeRef>> Elect1Fields;
    Elect1Fields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 0)));
    auto Elect1Port = TheLTS->MakeMsgType("Elect1Port", Elect1Fields, false);
    auto Elect1PortExp = TheLTS->MakeVar("Elect1Port", Elect1Port);
    vector<pair<string, ExprTypeRef>> Elect2Fields;
    Elect2Fields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 0)));
    auto Elect2Port = TheLTS->MakeMsgType("Elect2Port", Elect2Fields, false);
    auto Elect2PortExp = TheLTS->MakeVar("Elect2Port", Elect2Port);
    vector<pair<string, ExprTypeRef>> Elect3Fields;
    Elect3Fields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 0)));
    auto Elect3Port = TheLTS->MakeMsgType("Elect3Port", Elect3Fields, false);
    auto Elect3PortExp = TheLTS->MakeVar("Elect3Port", Elect3Port);

    TheLTS->FreezeMsgs();

    auto P1 = TheLTS->MakeGenEFSM("P1", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    P1->AddState("Q1");
    P1->AddState("Q0");
    P1->AddState("Q2");
    P1->FreezeStates();
    P1->AddVariable("P1X", TheLTS->MakeRangeType(0, 5));
    auto P1XExp = TheLTS->MakeVar("P1X", TheLTS->MakeRangeType(0, 5));
    P1->FreezeVars();
    P1->AddInputMsg(C1Port);
    P1->AddInputMsg(Elect1Port);
    P1->AddOutputMsg(C2Port);
    P1->AddOutputMsg(Leader1Port);
    // t2;
    P1->AddOutputTransition("Q1", "Q0", TheLTS->MakeOp(LTSOps::OpLT, P1XExp, TheLTS->MakeVal("1", TheLTS->MakeRangeType(1, 1))), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, C2PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("1", TheLTS->MakeRangeType(1, 1))) }, "C2Port", C2Port, {});
    // t3;
    P1->AddOutputTransition("Q1", "Q0", TheLTS->MakeOp(LTSOps::OpGT, P1XExp, TheLTS->MakeVal("1", TheLTS->MakeRangeType(1, 1))), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, C2PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), P1XExp) }, "C2Port", C2Port, {});
    // t4;
    P1->AddOutputTransition("Q1", "Q0", TheLTS->MakeOp(LTSOps::OpEQ, P1XExp, TheLTS->MakeVal("1", TheLTS->MakeRangeType(1, 1))), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, Leader1PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))) }, "Leader1Port", Leader1Port, {});
    // t1;
    P1->AddInputTransition("Q0", "Q1", TheLTS->MakeTrue(), { new LTSAssignSimple(P1XExp, TheLTS->MakeOp(LTSOps::OpField, C1PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType()))) }, "C1Port", C1Port, {});
    // t5;
    P1->AddInputTransition("Q0", "Q2", TheLTS->MakeTrue(), {  }, "Elect1Port", Elect1Port, {});
    // t6;
    P1->AddOutputTransition("Q2", "Q0", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, C2PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("1", TheLTS->MakeRangeType(1, 1))) }, "C2Port", C2Port, {});
    auto P2 = TheLTS->MakeGenEFSM("P2", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    P2->AddState("Q1");
    P2->AddState("Q0");
    P2->AddState("Q2");
    P2->FreezeStates();
    P2->AddVariable("P2X", TheLTS->MakeRangeType(0, 5));
    auto P2XExp = TheLTS->MakeVar("P2X", TheLTS->MakeRangeType(0, 5));
    P2->FreezeVars();
    P2->AddInputMsg(C2Port);
    P2->AddInputMsg(Elect2Port);
    P2->AddOutputMsg(C3Port);
    P2->AddOutputMsg(Leader2Port);
    // t2;
    P2->AddOutputTransition("Q1", "Q0", TheLTS->MakeOp(LTSOps::OpLT, P2XExp, TheLTS->MakeVal("2", TheLTS->MakeRangeType(2, 2))), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, C3PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("2", TheLTS->MakeRangeType(2, 2))) }, "C3Port", C3Port, {});
    // t3;
    P2->AddOutputTransition("Q1", "Q0", TheLTS->MakeOp(LTSOps::OpGT, P2XExp, TheLTS->MakeVal("2", TheLTS->MakeRangeType(2, 2))), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, C3PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), P2XExp) }, "C3Port", C3Port, {});
    // t4;
    P2->AddOutputTransition("Q1", "Q0", TheLTS->MakeOp(LTSOps::OpEQ, P2XExp, TheLTS->MakeVal("2", TheLTS->MakeRangeType(2, 2))), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, Leader2PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))) }, "Leader2Port", Leader2Port, {});
    // t1;
    P2->AddInputTransition("Q0", "Q1", TheLTS->MakeTrue(), { new LTSAssignSimple(P2XExp, TheLTS->MakeOp(LTSOps::OpField, C2PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType()))) }, "C2Port", C2Port, {});
    // t5;
    P2->AddInputTransition("Q0", "Q2", TheLTS->MakeTrue(), {  }, "Elect2Port", Elect2Port, {});
    // t6;
    P2->AddOutputTransition("Q2", "Q0", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, C3PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("2", TheLTS->MakeRangeType(2, 2))) }, "C3Port", C3Port, {});
    auto P3 = TheLTS->MakeGenEFSM("P3", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    P3->AddState("Q1");
    P3->AddState("Q0");
    P3->AddState("Q2");
    P3->FreezeStates();
    P3->AddVariable("P3X", TheLTS->MakeRangeType(0, 5));
    auto P3XExp = TheLTS->MakeVar("P3X", TheLTS->MakeRangeType(0, 5));
    P3->FreezeVars();
    P3->AddInputMsg(C3Port);
    P3->AddInputMsg(Elect3Port);
    P3->AddOutputMsg(C1Port);
    P3->AddOutputMsg(Leader3Port);
    // t2;
    P3->AddOutputTransition("Q1", "Q0", TheLTS->MakeOp(LTSOps::OpLT, P3XExp, TheLTS->MakeVal("3", TheLTS->MakeRangeType(3, 3))), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, C1PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("3", TheLTS->MakeRangeType(3, 3))) }, "C1Port", C1Port, {});
    // t3;
    P3->AddOutputTransition("Q1", "Q0", TheLTS->MakeOp(LTSOps::OpGT, P3XExp, TheLTS->MakeVal("3", TheLTS->MakeRangeType(3, 3))), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, C1PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), P3XExp) }, "C1Port", C1Port, {});
    // t4;
    P3->AddOutputTransition("Q1", "Q0", TheLTS->MakeOp(LTSOps::OpEQ, P3XExp, TheLTS->MakeVal("3", TheLTS->MakeRangeType(3, 3))), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, Leader3PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))) }, "Leader3Port", Leader3Port, {});
    // t1;
    P3->AddInputTransition("Q0", "Q1", TheLTS->MakeTrue(), { new LTSAssignSimple(P3XExp, TheLTS->MakeOp(LTSOps::OpField, C3PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType()))) }, "C3Port", C3Port, {});
    // t5;
    P3->AddInputTransition("Q0", "Q2", TheLTS->MakeTrue(), {  }, "Elect3Port", Elect3Port, {});
    // t6;
    P3->AddOutputTransition("Q2", "Q0", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, C1PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("3", TheLTS->MakeRangeType(3, 3))) }, "C1Port", C1Port, {});
    auto Environment = TheLTS->MakeGenEFSM("Environment", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    Environment->AddState("Q1");
    Environment->AddState("Q0");
    Environment->AddState("Error");
    Environment->FreezeStates();
    Environment->FreezeVars();
    Environment->AddInputMsg(Leader1Port);
    Environment->AddInputMsg(Leader2Port);
    Environment->AddInputMsg(Leader3Port);
    Environment->AddOutputMsg(Elect1Port);
    Environment->AddOutputMsg(Elect2Port);
    Environment->AddOutputMsg(Elect3Port);
    // tleader3;
    Environment->AddInputTransition("Q1", "Q0", TheLTS->MakeTrue(), {  }, "Leader3Port", Leader3Port, {});
    // tleader1;
    Environment->AddInputTransition("Q1", "Error", TheLTS->MakeTrue(), {  }, "Leader1Port", Leader1Port, {});
    // tleader2;
    Environment->AddInputTransition("Q1", "Error", TheLTS->MakeTrue(), {  }, "Leader2Port", Leader2Port, {});
    // t0;
    Environment->AddOutputTransition("Q0", "Q1", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, Elect1PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))) }, "Elect1Port", Elect1Port, {});
    // t1;
    Environment->AddOutputTransition("Q0", "Q1", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, Elect2PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))) }, "Elect2Port", Elect2Port, {});
    // t2;
    Environment->AddOutputTransition("Q0", "Q1", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, Elect3PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))) }, "Elect3Port", Elect3Port, {});
    auto SafetyMonitor = TheLTS->MakeGenEFSM("SafetyMonitor", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    SafetyMonitor->AddState("Waiting");
    SafetyMonitor->AddState("Initial");
    SafetyMonitor->AddState("Error");
    SafetyMonitor->FreezeStates();
    SafetyMonitor->FreezeVars();
    SafetyMonitor->AddInputMsg(Leader1Port);
    SafetyMonitor->AddInputMsg(Leader2Port);
    SafetyMonitor->AddInputMsg(Leader3Port);
    SafetyMonitor->AddInputMsg(Elect1Port);
    SafetyMonitor->AddInputMsg(Elect2Port);
    SafetyMonitor->AddInputMsg(Elect3Port);
    // t12;
    SafetyMonitor->AddInputTransition("Waiting", "Initial", TheLTS->MakeTrue(), {  }, "Leader3Port", Leader3Port, {});
    // t7;
    SafetyMonitor->AddInputTransition("Waiting", "Error", TheLTS->MakeTrue(), {  }, "Elect1Port", Elect1Port, {});
    // t8;
    SafetyMonitor->AddInputTransition("Waiting", "Error", TheLTS->MakeTrue(), {  }, "Elect2Port", Elect2Port, {});
    // t9;
    SafetyMonitor->AddInputTransition("Waiting", "Error", TheLTS->MakeTrue(), {  }, "Elect3Port", Elect3Port, {});
    // t10;
    SafetyMonitor->AddInputTransition("Waiting", "Error", TheLTS->MakeTrue(), {  }, "Leader1Port", Leader1Port, {});
    // t11;
    SafetyMonitor->AddInputTransition("Waiting", "Error", TheLTS->MakeTrue(), {  }, "Leader2Port", Leader2Port, {});
    // t4;
    SafetyMonitor->AddInputTransition("Initial", "Waiting", TheLTS->MakeTrue(), {  }, "Elect1Port", Elect1Port, {});
    // t5;
    SafetyMonitor->AddInputTransition("Initial", "Waiting", TheLTS->MakeTrue(), {  }, "Elect2Port", Elect2Port, {});
    // t6;
    SafetyMonitor->AddInputTransition("Initial", "Waiting", TheLTS->MakeTrue(), {  }, "Elect3Port", Elect3Port, {});
    // t1;
    SafetyMonitor->AddInputTransition("Initial", "Error", TheLTS->MakeTrue(), {  }, "Leader1Port", Leader1Port, {});
    // t2;
    SafetyMonitor->AddInputTransition("Initial", "Error", TheLTS->MakeTrue(), {  }, "Leader2Port", Leader2Port, {});
    // t3;
    SafetyMonitor->AddInputTransition("Initial", "Error", TheLTS->MakeTrue(), {  }, "Leader3Port", Leader3Port, {});
    auto LivenessMonitor = TheLTS->MakeGenEFSM("LivenessMonitor", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    LivenessMonitor->AddState("Initial");
    LivenessMonitor->AddState("Final");
    LivenessMonitor->AddState("Accept");
    LivenessMonitor->FreezeStates();
    LivenessMonitor->FreezeVars();
    LivenessMonitor->AddInputMsg(Leader1Port);
    LivenessMonitor->AddInputMsg(Leader2Port);
    LivenessMonitor->AddInputMsg(Leader3Port);
    LivenessMonitor->AddInputMsg(Elect1Port);
    LivenessMonitor->AddInputMsg(Elect2Port);
    LivenessMonitor->AddInputMsg(Elect3Port);
    // t1;
    LivenessMonitor->AddInputTransition("Initial", "Initial", TheLTS->MakeTrue(), {  }, "Leader1Port", Leader1Port, {});
    // t2;
    LivenessMonitor->AddInputTransition("Initial", "Initial", TheLTS->MakeTrue(), {  }, "Leader2Port", Leader2Port, {});
    // t3;
    LivenessMonitor->AddInputTransition("Initial", "Initial", TheLTS->MakeTrue(), {  }, "Leader3Port", Leader3Port, {});
    // t4;
    LivenessMonitor->AddInputTransition("Initial", "Initial", TheLTS->MakeTrue(), {  }, "Elect1Port", Elect1Port, {});
    // t5;
    LivenessMonitor->AddInputTransition("Initial", "Initial", TheLTS->MakeTrue(), {  }, "Elect2Port", Elect2Port, {});
    // t6;
    LivenessMonitor->AddInputTransition("Initial", "Initial", TheLTS->MakeTrue(), {  }, "Elect3Port", Elect3Port, {});
    // t13;
    LivenessMonitor->AddInputTransition("Initial", "Accept", TheLTS->MakeTrue(), {  }, "Elect1Port", Elect1Port, {});
    // t14;
    LivenessMonitor->AddInputTransition("Initial", "Accept", TheLTS->MakeTrue(), {  }, "Elect2Port", Elect2Port, {});
    // t15;
    LivenessMonitor->AddInputTransition("Initial", "Accept", TheLTS->MakeTrue(), {  }, "Elect3Port", Elect3Port, {});
    // t7;
    LivenessMonitor->AddInputTransition("Final", "Final", TheLTS->MakeTrue(), {  }, "Leader1Port", Leader1Port, {});
    // t8;
    LivenessMonitor->AddInputTransition("Final", "Final", TheLTS->MakeTrue(), {  }, "Leader2Port", Leader2Port, {});
    // t9;
    LivenessMonitor->AddInputTransition("Final", "Final", TheLTS->MakeTrue(), {  }, "Leader3Port", Leader3Port, {});
    // t10;
    LivenessMonitor->AddInputTransition("Final", "Final", TheLTS->MakeTrue(), {  }, "Elect1Port", Elect1Port, {});
    // t11;
    LivenessMonitor->AddInputTransition("Final", "Final", TheLTS->MakeTrue(), {  }, "Elect2Port", Elect2Port, {});
    // t12;
    LivenessMonitor->AddInputTransition("Final", "Final", TheLTS->MakeTrue(), {  }, "Elect3Port", Elect3Port, {});
    // t19;
    LivenessMonitor->AddInputTransition("Accept", "Final", TheLTS->MakeTrue(), {  }, "Leader1Port", Leader1Port, {});
    // t20;
    LivenessMonitor->AddInputTransition("Accept", "Final", TheLTS->MakeTrue(), {  }, "Leader2Port", Leader2Port, {});
    // t21;
    LivenessMonitor->AddInputTransition("Accept", "Final", TheLTS->MakeTrue(), {  }, "Leader3Port", Leader3Port, {});
    // t16;
    LivenessMonitor->AddInputTransition("Accept", "Accept", TheLTS->MakeTrue(), {  }, "Elect1Port", Elect1Port, {});
    // t17;
    LivenessMonitor->AddInputTransition("Accept", "Accept", TheLTS->MakeTrue(), {  }, "Elect2Port", Elect2Port, {});
    // t18;
    LivenessMonitor->AddInputTransition("Accept", "Accept", TheLTS->MakeTrue(), {  }, "Elect3Port", Elect3Port, {});

    TheLTS->FreezeAutomata();
    vector<LTSAssignRef> InitUpdates;

    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("P1", TheLTS->GetEFSMType("P1")), TheLTS->MakeVar("P1X", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("P1", TheLTS->GetEFSMType("P1")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Q0", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("P1", TheLTS->GetEFSMType("P1")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("P2", TheLTS->GetEFSMType("P2")), TheLTS->MakeVar("P2X", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("P2", TheLTS->GetEFSMType("P2")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Q0", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("P2", TheLTS->GetEFSMType("P2")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("P3", TheLTS->GetEFSMType("P3")), TheLTS->MakeVar("P3X", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("P3", TheLTS->GetEFSMType("P3")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Q0", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("P3", TheLTS->GetEFSMType("P3")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Environment", TheLTS->GetEFSMType("Environment")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Q0", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Environment", TheLTS->GetEFSMType("Environment")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("SafetyMonitor", TheLTS->GetEFSMType("SafetyMonitor")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Initial", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("SafetyMonitor", TheLTS->GetEFSMType("SafetyMonitor")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("LivenessMonitor", TheLTS->GetEFSMType("LivenessMonitor")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Initial", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("LivenessMonitor", TheLTS->GetEFSMType("LivenessMonitor")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));

    TheLTS->AddInitStates({new LTSInitState({ }, TheLTS->MakeTrue(), InitUpdates)});



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
    Checker->BuildAQS();

    delete Checker;
}
