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

    vector<pair<string, TypeRef>> Write1Fields;
    Write1Fields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 2)));
    auto Write1Port = TheLTS->MakeMsgType("Write1Port", Write1Fields, false);
    auto Write1PortExp = TheLTS->MakeVar("Write1Port", Write1Port);
    vector<pair<string, TypeRef>> Request1Fields;
    Request1Fields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 0)));
    auto Request1Port = TheLTS->MakeMsgType("Request1Port", Request1Fields, false);
    auto Request1PortExp = TheLTS->MakeVar("Request1Port", Request1Port);
    vector<pair<string, TypeRef>> Enter1Fields;
    Enter1Fields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 0)));
    auto Enter1Port = TheLTS->MakeMsgType("Enter1Port", Enter1Fields, false);
    auto Enter1PortExp = TheLTS->MakeVar("Enter1Port", Enter1Port);
    vector<pair<string, TypeRef>> Exit1Fields;
    Exit1Fields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 0)));
    auto Exit1Port = TheLTS->MakeMsgType("Exit1Port", Exit1Fields, false);
    auto Exit1PortExp = TheLTS->MakeVar("Exit1Port", Exit1Port);
    vector<pair<string, TypeRef>> Write2Fields;
    Write2Fields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 2)));
    auto Write2Port = TheLTS->MakeMsgType("Write2Port", Write2Fields, false);
    auto Write2PortExp = TheLTS->MakeVar("Write2Port", Write2Port);
    vector<pair<string, TypeRef>> Request2Fields;
    Request2Fields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 0)));
    auto Request2Port = TheLTS->MakeMsgType("Request2Port", Request2Fields, false);
    auto Request2PortExp = TheLTS->MakeVar("Request2Port", Request2Port);
    vector<pair<string, TypeRef>> Enter2Fields;
    Enter2Fields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 0)));
    auto Enter2Port = TheLTS->MakeMsgType("Enter2Port", Enter2Fields, false);
    auto Enter2PortExp = TheLTS->MakeVar("Enter2Port", Enter2Port);
    vector<pair<string, TypeRef>> Exit2Fields;
    Exit2Fields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 0)));
    auto Exit2Port = TheLTS->MakeMsgType("Exit2Port", Exit2Fields, false);
    auto Exit2PortExp = TheLTS->MakeVar("Exit2Port", Exit2Port);
    vector<pair<string, TypeRef>> TickFields;
    TickFields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 0)));
    auto TickPort = TheLTS->MakeMsgType("TickPort", TickFields, false);
    auto TickPortExp = TheLTS->MakeVar("TickPort", TickPort);
    vector<pair<string, TypeRef>> Read1Fields;
    Read1Fields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 2)));
    auto Read1Port = TheLTS->MakeMsgType("Read1Port", Read1Fields, false);
    auto Read1PortExp = TheLTS->MakeVar("Read1Port", Read1Port);
    vector<pair<string, TypeRef>> Read2Fields;
    Read2Fields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 2)));
    auto Read2Port = TheLTS->MakeMsgType("Read2Port", Read2Fields, false);
    auto Read2PortExp = TheLTS->MakeVar("Read2Port", Read2Port);

    TheLTS->FreezeMsgs();

    auto Process1 = TheLTS->MakeGenEFSM("Process1", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    Process1->AddState("OutCritical");
    Process1->AddState("Initial");
    Process1->AddState("Delay1");
    Process1->AddState("Delay2");
    Process1->AddState("FreeLock");
    Process1->AddState("InCritical");
    Process1->AddState("AfterReadLock");
    Process1->FreezeStates();
    Process1->AddVariable("Lock1", TheLTS->MakeRangeType(0, 2));
    auto Lock1Exp = TheLTS->MakeVar("Lock1", TheLTS->MakeRangeType(0, 2));
    Process1->AddVariable("Timer1", TheLTS->MakeRangeType(0, 10));
    auto Timer1Exp = TheLTS->MakeVar("Timer1", TheLTS->MakeRangeType(0, 10));
    Process1->FreezeVars();
    Process1->AddInputMsg(Read1Port);
    Process1->AddInputMsg(TickPort);
    Process1->AddOutputMsg(Write1Port);
    Process1->AddOutputMsg(Request1Port);
    Process1->AddOutputMsg(Enter1Port);
    Process1->AddOutputMsg(Exit1Port);
    // t13;
    Process1->AddOutputTransition("OutCritical", "Initial", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, Write1PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))) }, "Write1Port", Write1Port, {});
    // t1;
    Process1->AddInputTransition("Initial", "Initial", TheLTS->MakeTrue(), {  }, "TickPort", TickPort, {});
    // t2;
    Process1->AddInputTransition("Initial", "AfterReadLock", TheLTS->MakeTrue(), { new LTSAssignSimple(Lock1Exp, TheLTS->MakeOp(LTSOps::OpField, Read1PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType()))) }, "Read1Port", Read1Port, {});
    // t8;
    Process1->AddInputTransition("Delay1", "Delay1", TheLTS->MakeOp(LTSOps::OpLT, Timer1Exp, TheLTS->MakeVal("3", TheLTS->MakeRangeType(3, 3))), { new LTSAssignSimple(Timer1Exp, TheLTS->MakeOp(LTSOps::OpADD, Timer1Exp, TheLTS->MakeVal("1", TheLTS->MakeRangeType(1, 1)))) }, "TickPort", TickPort, {});
    // t9;
    Process1->AddInputTransition("Delay1", "Delay2", TheLTS->MakeOp(LTSOps::OpEQ, Timer1Exp, TheLTS->MakeVal("3", TheLTS->MakeRangeType(3, 3))), { new LTSAssignSimple(Lock1Exp, TheLTS->MakeOp(LTSOps::OpField, Read1PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType()))) }, "Read1Port", Read1Port, {});
    // t10;
    Process1->AddOutputTransition("Delay2", "InCritical", TheLTS->MakeOp(LTSOps::OpEQ, Lock1Exp, TheLTS->MakeVal("1", TheLTS->MakeRangeType(1, 1))), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, Enter1PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))) }, "Enter1Port", Enter1Port, {});
    // t11;
    Process1->AddInternalTransition("Delay2", "Initial", TheLTS->MakeOp(LTSOps::OpNOT, TheLTS->MakeOp(LTSOps::OpEQ, Lock1Exp, TheLTS->MakeVal("1", TheLTS->MakeRangeType(1, 1)))), { new LTSAssignSimple(Timer1Exp, TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))) });
    // t7;
    Process1->AddOutputTransition("FreeLock", "Delay1", TheLTS->MakeOp(LTSOps::OpEQ, Timer1Exp, TheLTS->MakeVal("6", TheLTS->MakeRangeType(6, 6))), { new LTSAssignSimple(Timer1Exp, TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))), new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, Write1PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("1", TheLTS->MakeRangeType(1, 1))) }, "Write1Port", Write1Port, {});
    // t6;
    Process1->AddInputTransition("FreeLock", "FreeLock", TheLTS->MakeOp(LTSOps::OpLT, Timer1Exp, TheLTS->MakeVal("6", TheLTS->MakeRangeType(6, 6))), { new LTSAssignSimple(Timer1Exp, TheLTS->MakeOp(LTSOps::OpADD, Timer1Exp, TheLTS->MakeVal("1", TheLTS->MakeRangeType(1, 1)))) }, "TickPort", TickPort, {});
    // t12;
    Process1->AddOutputTransition("InCritical", "OutCritical", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, Exit1PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))) }, "Exit1Port", Exit1Port, {});
    // t14;
    Process1->AddInputTransition("InCritical", "InCritical", TheLTS->MakeTrue(), {  }, "TickPort", TickPort, {});
    // t3;
    Process1->AddInternalTransition("AfterReadLock", "Initial", TheLTS->MakeOp(LTSOps::OpGT, Lock1Exp, TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))), {  });
    // t5;
    Process1->AddOutputTransition("AfterReadLock", "FreeLock", TheLTS->MakeOp(LTSOps::OpEQ, Lock1Exp, TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))), { new LTSAssignSimple(Timer1Exp, TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))), new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, Request1PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))) }, "Request1Port", Request1Port, {});
    auto Process2 = TheLTS->MakeGenEFSM("Process2", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    Process2->AddState("OutCritical");
    Process2->AddState("Initial");
    Process2->AddState("Delay1");
    Process2->AddState("Delay2");
    Process2->AddState("FreeLock");
    Process2->AddState("InCritical");
    Process2->AddState("AfterReadLock");
    Process2->FreezeStates();
    Process2->AddVariable("Lock2", TheLTS->MakeRangeType(0, 2));
    auto Lock2Exp = TheLTS->MakeVar("Lock2", TheLTS->MakeRangeType(0, 2));
    Process2->AddVariable("Timer2", TheLTS->MakeRangeType(0, 10));
    auto Timer2Exp = TheLTS->MakeVar("Timer2", TheLTS->MakeRangeType(0, 10));
    Process2->FreezeVars();
    Process2->AddInputMsg(Read2Port);
    Process2->AddInputMsg(TickPort);
    Process2->AddOutputMsg(Write2Port);
    Process2->AddOutputMsg(Request2Port);
    Process2->AddOutputMsg(Enter2Port);
    Process2->AddOutputMsg(Exit2Port);
    // t13;
    Process2->AddOutputTransition("OutCritical", "Initial", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, Write2PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))) }, "Write2Port", Write2Port, {});
    // t1;
    Process2->AddInputTransition("Initial", "Initial", TheLTS->MakeTrue(), {  }, "TickPort", TickPort, {});
    // t2;
    Process2->AddInputTransition("Initial", "AfterReadLock", TheLTS->MakeTrue(), { new LTSAssignSimple(Lock2Exp, TheLTS->MakeOp(LTSOps::OpField, Read2PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType()))) }, "Read2Port", Read2Port, {});
    // t8;
    Process2->AddInputTransition("Delay1", "Delay1", TheLTS->MakeOp(LTSOps::OpLT, Timer2Exp, TheLTS->MakeVal("3", TheLTS->MakeRangeType(3, 3))), { new LTSAssignSimple(Timer2Exp, TheLTS->MakeOp(LTSOps::OpADD, Timer2Exp, TheLTS->MakeVal("1", TheLTS->MakeRangeType(1, 1)))) }, "TickPort", TickPort, {});
    // t9;
    Process2->AddInputTransition("Delay1", "Delay2", TheLTS->MakeOp(LTSOps::OpEQ, Timer2Exp, TheLTS->MakeVal("3", TheLTS->MakeRangeType(3, 3))), { new LTSAssignSimple(Lock2Exp, TheLTS->MakeOp(LTSOps::OpField, Read2PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType()))) }, "Read2Port", Read2Port, {});
    // t10;
    Process2->AddOutputTransition("Delay2", "InCritical", TheLTS->MakeOp(LTSOps::OpEQ, Lock2Exp, TheLTS->MakeVal("2", TheLTS->MakeRangeType(2, 2))), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, Enter2PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))) }, "Enter2Port", Enter2Port, {});
    // t11;
    Process2->AddInternalTransition("Delay2", "Initial", TheLTS->MakeOp(LTSOps::OpNOT, TheLTS->MakeOp(LTSOps::OpEQ, Lock2Exp, TheLTS->MakeVal("2", TheLTS->MakeRangeType(2, 2)))), { new LTSAssignSimple(Timer2Exp, TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))) });
    // t7;
    Process2->AddOutputTransition("FreeLock", "Delay1", TheLTS->MakeOp(LTSOps::OpEQ, Timer2Exp, TheLTS->MakeVal("6", TheLTS->MakeRangeType(6, 6))), { new LTSAssignSimple(Timer2Exp, TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))), new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, Write2PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("2", TheLTS->MakeRangeType(2, 2))) }, "Write2Port", Write2Port, {});
    // t6;
    Process2->AddInputTransition("FreeLock", "FreeLock", TheLTS->MakeOp(LTSOps::OpLT, Timer2Exp, TheLTS->MakeVal("6", TheLTS->MakeRangeType(6, 6))), { new LTSAssignSimple(Timer2Exp, TheLTS->MakeOp(LTSOps::OpADD, Timer2Exp, TheLTS->MakeVal("1", TheLTS->MakeRangeType(1, 1)))) }, "TickPort", TickPort, {});
    // t12;
    Process2->AddOutputTransition("InCritical", "OutCritical", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, Exit2PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))) }, "Exit2Port", Exit2Port, {});
    // t14;
    Process2->AddInputTransition("InCritical", "InCritical", TheLTS->MakeTrue(), {  }, "TickPort", TickPort, {});
    // t3;
    Process2->AddInternalTransition("AfterReadLock", "Initial", TheLTS->MakeOp(LTSOps::OpGT, Lock2Exp, TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))), {  });
    // t5;
    Process2->AddOutputTransition("AfterReadLock", "FreeLock", TheLTS->MakeOp(LTSOps::OpEQ, Lock2Exp, TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))), { new LTSAssignSimple(Timer2Exp, TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))), new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, Request2PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))) }, "Request2Port", Request2Port, {});
    auto Clock = TheLTS->MakeGenEFSM("Clock", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    Clock->AddState("Initial");
    Clock->FreezeStates();
    Clock->FreezeVars();
    Clock->AddOutputMsg(TickPort);
    // t1;
    Clock->AddOutputTransition("Initial", "Initial", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TickPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))) }, "TickPort", TickPort, {});
    auto Lock = TheLTS->MakeGenEFSM("Lock", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    Lock->AddState("Initial");
    Lock->FreezeStates();
    Lock->AddVariable("Owner", TheLTS->MakeRangeType(0, 2));
    auto OwnerExp = TheLTS->MakeVar("Owner", TheLTS->MakeRangeType(0, 2));
    Lock->FreezeVars();
    Lock->AddInputMsg(Write1Port);
    Lock->AddInputMsg(Write2Port);
    Lock->AddOutputMsg(Read1Port);
    Lock->AddOutputMsg(Read2Port);
    // t_read_1;
    Lock->AddOutputTransition("Initial", "Initial", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, Read1PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), OwnerExp) }, "Read1Port", Read1Port, {});
    // t_write_1;
    Lock->AddInputTransition("Initial", "Initial", TheLTS->MakeTrue(), { new LTSAssignSimple(OwnerExp, TheLTS->MakeOp(LTSOps::OpField, Write1PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType()))) }, "Write1Port", Write1Port, {});
    // t_read_2;
    Lock->AddOutputTransition("Initial", "Initial", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, Read2PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), OwnerExp) }, "Read2Port", Read2Port, {});
    // t_write_2;
    Lock->AddInputTransition("Initial", "Initial", TheLTS->MakeTrue(), { new LTSAssignSimple(OwnerExp, TheLTS->MakeOp(LTSOps::OpField, Write2PortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType()))) }, "Write2Port", Write2Port, {});
    auto SafetyMonitor1 = TheLTS->MakeGenEFSM("SafetyMonitor1", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    SafetyMonitor1->AddState("In-Critical");
    SafetyMonitor1->AddState("Initial");
    SafetyMonitor1->AddState("Error");
    SafetyMonitor1->FreezeStates();
    SafetyMonitor1->FreezeVars();
    SafetyMonitor1->AddInputMsg(Exit1Port);
    SafetyMonitor1->AddInputMsg(Enter1Port);
    SafetyMonitor1->AddInputMsg(Enter2Port);
    // t3;
    SafetyMonitor1->AddInputTransition("In-Critical", "Initial", TheLTS->MakeTrue(), {  }, "Exit1Port", Exit1Port, {});
    // t40;
    SafetyMonitor1->AddInputTransition("In-Critical", "Error", TheLTS->MakeTrue(), {  }, "Enter2Port", Enter2Port, {});
    // t1;
    SafetyMonitor1->AddInputTransition("Initial", "In-Critical", TheLTS->MakeTrue(), {  }, "Enter1Port", Enter1Port, {});
    // t60;
    SafetyMonitor1->AddInputTransition("Initial", "Initial", TheLTS->MakeTrue(), {  }, "Enter2Port", Enter2Port, {});
    // t2;
    SafetyMonitor1->AddInputTransition("Initial", "Error", TheLTS->MakeTrue(), {  }, "Exit1Port", Exit1Port, {});
    // t50;
    SafetyMonitor1->AddInputTransition("Error", "Error", TheLTS->MakeTrue(), {  }, "Enter2Port", Enter2Port, {});
    // t6;
    SafetyMonitor1->AddInputTransition("Error", "Error", TheLTS->MakeTrue(), {  }, "Enter1Port", Enter1Port, {});
    // t7;
    SafetyMonitor1->AddInputTransition("Error", "Error", TheLTS->MakeTrue(), {  }, "Exit1Port", Exit1Port, {});
    auto SafetyMonitor2 = TheLTS->MakeGenEFSM("SafetyMonitor2", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    SafetyMonitor2->AddState("In-Critical");
    SafetyMonitor2->AddState("Initial");
    SafetyMonitor2->AddState("Error");
    SafetyMonitor2->FreezeStates();
    SafetyMonitor2->FreezeVars();
    SafetyMonitor2->AddInputMsg(Exit2Port);
    SafetyMonitor2->AddInputMsg(Enter2Port);
    SafetyMonitor2->AddInputMsg(Enter1Port);
    // t3;
    SafetyMonitor2->AddInputTransition("In-Critical", "Initial", TheLTS->MakeTrue(), {  }, "Exit2Port", Exit2Port, {});
    // t40;
    SafetyMonitor2->AddInputTransition("In-Critical", "Error", TheLTS->MakeTrue(), {  }, "Enter1Port", Enter1Port, {});
    // t1;
    SafetyMonitor2->AddInputTransition("Initial", "In-Critical", TheLTS->MakeTrue(), {  }, "Enter2Port", Enter2Port, {});
    // t60;
    SafetyMonitor2->AddInputTransition("Initial", "Initial", TheLTS->MakeTrue(), {  }, "Enter1Port", Enter1Port, {});
    // t2;
    SafetyMonitor2->AddInputTransition("Initial", "Error", TheLTS->MakeTrue(), {  }, "Exit2Port", Exit2Port, {});
    // t50;
    SafetyMonitor2->AddInputTransition("Error", "Error", TheLTS->MakeTrue(), {  }, "Enter1Port", Enter1Port, {});
    // t6;
    SafetyMonitor2->AddInputTransition("Error", "Error", TheLTS->MakeTrue(), {  }, "Enter2Port", Enter2Port, {});
    // t7;
    SafetyMonitor2->AddInputTransition("Error", "Error", TheLTS->MakeTrue(), {  }, "Exit2Port", Exit2Port, {});
    auto DummyLivenessMonitor = TheLTS->MakeGenEFSM("DummyLivenessMonitor", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    DummyLivenessMonitor->AddState("Initial");
    DummyLivenessMonitor->FreezeStates();
    DummyLivenessMonitor->FreezeVars();
    DummyLivenessMonitor->AddInputMsg(Request1Port);
    DummyLivenessMonitor->AddInputMsg(Request2Port);
    // t1;
    DummyLivenessMonitor->AddInputTransition("Initial", "Initial", TheLTS->MakeTrue(), {  }, "Request1Port", Request1Port, {});
    // t2;
    DummyLivenessMonitor->AddInputTransition("Initial", "Initial", TheLTS->MakeTrue(), {  }, "Request2Port", Request2Port, {});

    TheLTS->FreezeAutomata();
    vector<LTSAssignRef> InitUpdates;

    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Process1", TheLTS->GetEFSMType("Process1")), TheLTS->MakeVar("Lock1", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Process1", TheLTS->GetEFSMType("Process1")), TheLTS->MakeVar("Timer1", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Process1", TheLTS->GetEFSMType("Process1")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Initial", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Process1", TheLTS->GetEFSMType("Process1")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Process2", TheLTS->GetEFSMType("Process2")), TheLTS->MakeVar("Lock2", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Process2", TheLTS->GetEFSMType("Process2")), TheLTS->MakeVar("Timer2", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Process2", TheLTS->GetEFSMType("Process2")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Initial", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Process2", TheLTS->GetEFSMType("Process2")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Clock", TheLTS->GetEFSMType("Clock")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Initial", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Clock", TheLTS->GetEFSMType("Clock")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Lock", TheLTS->GetEFSMType("Lock")), TheLTS->MakeVar("Owner", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Lock", TheLTS->GetEFSMType("Lock")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Initial", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Lock", TheLTS->GetEFSMType("Lock")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("SafetyMonitor1", TheLTS->GetEFSMType("SafetyMonitor1")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Initial", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("SafetyMonitor1", TheLTS->GetEFSMType("SafetyMonitor1")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("SafetyMonitor2", TheLTS->GetEFSMType("SafetyMonitor2")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Initial", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("SafetyMonitor2", TheLTS->GetEFSMType("SafetyMonitor2")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("DummyLivenessMonitor", TheLTS->GetEFSMType("DummyLivenessMonitor")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Initial", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("DummyLivenessMonitor", TheLTS->GetEFSMType("DummyLivenessMonitor")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));

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
