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

    vector<pair<string, ExprTypeRef>> SendFields;
    SendFields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 5)));
    auto SendPort = TheLTS->MakeMsgType("SendPort", SendFields, false);
    auto SendPortExp = TheLTS->MakeVar("SendPort", SendPort);
    vector<pair<string, ExprTypeRef>> ForwardInputChannelFields;
    ForwardInputChannelFields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 1)));
    ForwardInputChannelFields.push_back(make_pair("Field1", TheLTS->MakeRangeType(0, 5)));
    auto ForwardInputChannelPort = TheLTS->MakeMsgType("ForwardInputChannelPort", ForwardInputChannelFields, false);
    auto ForwardInputChannelPortExp = TheLTS->MakeVar("ForwardInputChannelPort", ForwardInputChannelPort);
    vector<pair<string, ExprTypeRef>> ForwardOutputChannelFields;
    ForwardOutputChannelFields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 1)));
    ForwardOutputChannelFields.push_back(make_pair("Field1", TheLTS->MakeRangeType(0, 5)));
    auto ForwardOutputChannelPort = TheLTS->MakeMsgType("ForwardOutputChannelPort", ForwardOutputChannelFields, false);
    auto ForwardOutputChannelPortExp = TheLTS->MakeVar("ForwardOutputChannelPort", ForwardOutputChannelPort);
    vector<pair<string, ExprTypeRef>> TimeoutFields;
    TimeoutFields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 0)));
    auto TimeoutPort = TheLTS->MakeMsgType("TimeoutPort", TimeoutFields, false);
    auto TimeoutPortExp = TheLTS->MakeVar("TimeoutPort", TimeoutPort);
    vector<pair<string, ExprTypeRef>> ReceiveFields;
    ReceiveFields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 5)));
    auto ReceivePort = TheLTS->MakeMsgType("ReceivePort", ReceiveFields, false);
    auto ReceivePortExp = TheLTS->MakeVar("ReceivePort", ReceivePort);
    vector<pair<string, ExprTypeRef>> BackwardInputChannelFields;
    BackwardInputChannelFields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 1)));
    auto BackwardInputChannelPort = TheLTS->MakeMsgType("BackwardInputChannelPort", BackwardInputChannelFields, false);
    auto BackwardInputChannelPortExp = TheLTS->MakeVar("BackwardInputChannelPort", BackwardInputChannelPort);
    vector<pair<string, ExprTypeRef>> BackwardOutputChannelFields;
    BackwardOutputChannelFields.push_back(make_pair("Field0", TheLTS->MakeRangeType(0, 1)));
    auto BackwardOutputChannelPort = TheLTS->MakeMsgType("BackwardOutputChannelPort", BackwardOutputChannelFields, false);
    auto BackwardOutputChannelPortExp = TheLTS->MakeVar("BackwardOutputChannelPort", BackwardOutputChannelPort);

    TheLTS->FreezeMsgs();

    auto SenderClient = TheLTS->MakeGenEFSM("SenderClient", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    SenderClient->AddState("Initial");
    SenderClient->FreezeStates();
    SenderClient->FreezeVars();
    SenderClient->AddOutputMsg(SendPort);
    // t0;
    SenderClient->AddOutputTransition("Initial", "Initial", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, SendPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))) }, "SendPort", SendPort, {});
    // t1;
    SenderClient->AddOutputTransition("Initial", "Initial", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, SendPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("1", TheLTS->MakeRangeType(1, 1))) }, "SendPort", SendPort, {});
    // t2;
    SenderClient->AddOutputTransition("Initial", "Initial", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, SendPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("2", TheLTS->MakeRangeType(2, 2))) }, "SendPort", SendPort, {});
    // t3;
    SenderClient->AddOutputTransition("Initial", "Initial", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, SendPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("3", TheLTS->MakeRangeType(3, 3))) }, "SendPort", SendPort, {});
    // t4;
    SenderClient->AddOutputTransition("Initial", "Initial", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, SendPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("4", TheLTS->MakeRangeType(4, 4))) }, "SendPort", SendPort, {});
    // t5;
    SenderClient->AddOutputTransition("Initial", "Initial", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, SendPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("5", TheLTS->MakeRangeType(5, 5))) }, "SendPort", SendPort, {});
    auto Sender = TheLTS->MakeGenEFSM("Sender", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    Sender->AddState("Q1");
    Sender->AddState("Q0");
    Sender->AddState("Initial");
    Sender->AddState("Q2");
    Sender->FreezeStates();
    Sender->AddVariable("SenderInputMessage", TheLTS->MakeRangeType(0, 5));
    auto SenderInputMessageExp = TheLTS->MakeVar("SenderInputMessage", TheLTS->MakeRangeType(0, 5));
    Sender->AddVariable("SenderTag", TheLTS->MakeRangeType(0, 1));
    auto SenderTagExp = TheLTS->MakeVar("SenderTag", TheLTS->MakeRangeType(0, 1));
    Sender->AddVariable("AckTag", TheLTS->MakeRangeType(0, 1));
    auto AckTagExp = TheLTS->MakeVar("AckTag", TheLTS->MakeRangeType(0, 1));
    Sender->FreezeVars();
    Sender->AddInputMsg(SendPort);
    Sender->AddInputMsg(TimeoutPort);
    Sender->AddInputMsg(BackwardOutputChannelPort);
    Sender->AddOutputMsg(ForwardInputChannelPort);
    // t2;
    Sender->AddInputTransition("Q1", "Q0", TheLTS->MakeTrue(), {  }, "TimeoutPort", TimeoutPort, {});
    // t3;
    Sender->AddInputTransition("Q1", "Q2", TheLTS->MakeTrue(), { new LTSAssignSimple(AckTagExp, TheLTS->MakeOp(LTSOps::OpField, BackwardOutputChannelPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType()))) }, "BackwardOutputChannelPort", BackwardOutputChannelPort, {});
    // t1;
    Sender->AddOutputTransition("Q0", "Q1", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, ForwardInputChannelPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), SenderTagExp), new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, ForwardInputChannelPortExp, TheLTS->MakeVar("Field1", TheLTS->MakeFieldAccessType())), SenderInputMessageExp), new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, ForwardInputChannelPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), SenderTagExp), new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, ForwardInputChannelPortExp, TheLTS->MakeVar("Field1", TheLTS->MakeFieldAccessType())), SenderInputMessageExp) }, "ForwardInputChannelPort", ForwardInputChannelPort, {});
    // t0;
    Sender->AddInputTransition("Initial", "Q0", TheLTS->MakeTrue(), { new LTSAssignSimple(SenderInputMessageExp, TheLTS->MakeOp(LTSOps::OpField, SendPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType()))) }, "SendPort", SendPort, {});
    // t5;
    Sender->AddInternalTransition("Q2", "Q0", TheLTS->MakeOp(LTSOps::OpNOT, TheLTS->MakeOp(LTSOps::OpEQ, AckTagExp, SenderTagExp)), {  });
    // t4;
    Sender->AddInternalTransition("Q2", "Initial", TheLTS->MakeOp(LTSOps::OpEQ, AckTagExp, SenderTagExp), { new LTSAssignSimple(SenderTagExp, TheLTS->MakeOp(LTSOps::OpITE, TheLTS->MakeOp(LTSOps::OpEQ, SenderTagExp, TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))), TheLTS->MakeVal("1", TheLTS->MakeRangeType(1, 1)), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0)))) });
    auto ForwardTagDataChannel = TheLTS->MakeGenEFSM("ForwardTagDataChannel", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    ForwardTagDataChannel->AddState("Full");
    ForwardTagDataChannel->AddState("Empty");
    ForwardTagDataChannel->FreezeStates();
    ForwardTagDataChannel->AddVariable("ForwardChannelTag", TheLTS->MakeRangeType(0, 1));
    auto ForwardChannelTagExp = TheLTS->MakeVar("ForwardChannelTag", TheLTS->MakeRangeType(0, 1));
    ForwardTagDataChannel->AddVariable("ForwardChannelData", TheLTS->MakeRangeType(0, 5));
    auto ForwardChannelDataExp = TheLTS->MakeVar("ForwardChannelData", TheLTS->MakeRangeType(0, 5));
    ForwardTagDataChannel->FreezeVars();
    ForwardTagDataChannel->AddInputMsg(ForwardInputChannelPort);
    ForwardTagDataChannel->AddOutputMsg(ForwardOutputChannelPort);
    // t2;
    ForwardTagDataChannel->AddInputTransition("Full", "Full", TheLTS->MakeTrue(), {  }, "ForwardInputChannelPort", ForwardInputChannelPort, {});
    // t_duplication;
    ForwardTagDataChannel->AddOutputTransition("Full", "Full", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, ForwardOutputChannelPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), ForwardChannelTagExp), new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, ForwardOutputChannelPortExp, TheLTS->MakeVar("Field1", TheLTS->MakeFieldAccessType())), ForwardChannelDataExp) }, "ForwardOutputChannelPort", ForwardOutputChannelPort, {});
    // t1;
    ForwardTagDataChannel->AddOutputTransition("Full", "Empty", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, ForwardOutputChannelPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), ForwardChannelTagExp), new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, ForwardOutputChannelPortExp, TheLTS->MakeVar("Field1", TheLTS->MakeFieldAccessType())), ForwardChannelDataExp) }, "ForwardOutputChannelPort", ForwardOutputChannelPort, {});
    // t0;
    ForwardTagDataChannel->AddInputTransition("Empty", "Full", TheLTS->MakeTrue(), { new LTSAssignSimple(ForwardChannelTagExp, TheLTS->MakeOp(LTSOps::OpField, ForwardInputChannelPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType()))), new LTSAssignSimple(ForwardChannelDataExp, TheLTS->MakeOp(LTSOps::OpField, ForwardInputChannelPortExp, TheLTS->MakeVar("Field1", TheLTS->MakeFieldAccessType()))) }, "ForwardInputChannelPort", ForwardInputChannelPort, {});
    // t_loss;
    ForwardTagDataChannel->AddInputTransition("Empty", "Empty", TheLTS->MakeTrue(), {  }, "ForwardInputChannelPort", ForwardInputChannelPort, {});
    auto Timer = TheLTS->MakeGenEFSM("Timer", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    Timer->AddState("Q0");
    Timer->AddState("Initial");
    Timer->FreezeStates();
    Timer->FreezeVars();
    Timer->AddOutputMsg(TimeoutPort);
    // t0;
    Timer->AddOutputTransition("Q0", "Initial", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TimeoutPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))) }, "TimeoutPort", TimeoutPort, {});
    // t0;
    Timer->AddOutputTransition("Initial", "Q0", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TimeoutPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))) }, "TimeoutPort", TimeoutPort, {});
    auto Receiver = TheLTS->MakeGenEFSM("Receiver", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    Receiver->AddState("Q1");
    Receiver->AddState("Q0");
    Receiver->AddState("Initial");
    Receiver->AddState("Q2");
    Receiver->FreezeStates();
    Receiver->AddVariable("ReceiverTag", TheLTS->MakeRangeType(0, 1));
    auto ReceiverTagExp = TheLTS->MakeVar("ReceiverTag", TheLTS->MakeRangeType(0, 1));
    Receiver->AddVariable("ReceiverData", TheLTS->MakeRangeType(0, 5));
    auto ReceiverDataExp = TheLTS->MakeVar("ReceiverData", TheLTS->MakeRangeType(0, 5));
    Receiver->AddVariable("ExpectedTag", TheLTS->MakeRangeType(0, 1));
    auto ExpectedTagExp = TheLTS->MakeVar("ExpectedTag", TheLTS->MakeRangeType(0, 1));
    Receiver->FreezeVars();
    Receiver->AddInputMsg(ForwardOutputChannelPort);
    Receiver->AddOutputMsg(ReceivePort);
    Receiver->AddOutputMsg(BackwardInputChannelPort);
    // t2;
    Receiver->AddOutputTransition("Q1", "Initial", TheLTS->MakeTrue(), { new LTSAssignSimple(ExpectedTagExp, TheLTS->MakeOp(LTSOps::OpITE, TheLTS->MakeOp(LTSOps::OpEQ, ExpectedTagExp, TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))), TheLTS->MakeVal("1", TheLTS->MakeRangeType(1, 1)), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0)))), new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, BackwardInputChannelPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), ExpectedTagExp) }, "BackwardInputChannelPort", BackwardInputChannelPort, {});
    // t1;
    Receiver->AddOutputTransition("Q0", "Q1", TheLTS->MakeOp(LTSOps::OpEQ, ReceiverTagExp, ExpectedTagExp), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, ReceivePortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), ReceiverDataExp) }, "ReceivePort", ReceivePort, {});
    // t3;
    Receiver->AddInternalTransition("Q0", "Q2", TheLTS->MakeOp(LTSOps::OpNOT, TheLTS->MakeOp(LTSOps::OpEQ, ReceiverTagExp, ExpectedTagExp)), {  });
    // t0;
    Receiver->AddInputTransition("Initial", "Q0", TheLTS->MakeTrue(), { new LTSAssignSimple(ReceiverTagExp, TheLTS->MakeOp(LTSOps::OpField, ForwardOutputChannelPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType()))), new LTSAssignSimple(ReceiverDataExp, TheLTS->MakeOp(LTSOps::OpField, ForwardOutputChannelPortExp, TheLTS->MakeVar("Field1", TheLTS->MakeFieldAccessType()))) }, "ForwardOutputChannelPort", ForwardOutputChannelPort, {});
    // t4;
    Receiver->AddOutputTransition("Q2", "Initial", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, BackwardInputChannelPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), TheLTS->MakeOp(LTSOps::OpITE, TheLTS->MakeOp(LTSOps::OpEQ, ExpectedTagExp, TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))), TheLTS->MakeVal("1", TheLTS->MakeRangeType(1, 1)), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0)))) }, "BackwardInputChannelPort", BackwardInputChannelPort, {});
    auto ReceiverClient = TheLTS->MakeGenEFSM("ReceiverClient", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    ReceiverClient->AddState("Initial");
    ReceiverClient->FreezeStates();
    ReceiverClient->FreezeVars();
    ReceiverClient->AddInputMsg(ReceivePort);
    // t0;
    ReceiverClient->AddInputTransition("Initial", "Initial", TheLTS->MakeTrue(), {  }, "ReceivePort", ReceivePort, {});
    auto SafetyMonitor = TheLTS->MakeGenEFSM("SafetyMonitor", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    SafetyMonitor->AddState("WaitingForSend");
    SafetyMonitor->AddState("WaitingForReceive");
    SafetyMonitor->AddState("CheckReceivedData");
    SafetyMonitor->AddState("Error");
    SafetyMonitor->FreezeStates();
    SafetyMonitor->AddVariable("SafetyDataSent", TheLTS->MakeRangeType(0, 5));
    auto SafetyDataSentExp = TheLTS->MakeVar("SafetyDataSent", TheLTS->MakeRangeType(0, 5));
    SafetyMonitor->AddVariable("SafetyDataReceived", TheLTS->MakeRangeType(0, 5));
    auto SafetyDataReceivedExp = TheLTS->MakeVar("SafetyDataReceived", TheLTS->MakeRangeType(0, 5));
    SafetyMonitor->FreezeVars();
    SafetyMonitor->AddInputMsg(SendPort);
    SafetyMonitor->AddInputMsg(ReceivePort);
    // t0;
    SafetyMonitor->AddInputTransition("WaitingForSend", "WaitingForReceive", TheLTS->MakeTrue(), { new LTSAssignSimple(SafetyDataSentExp, TheLTS->MakeOp(LTSOps::OpField, SendPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType()))) }, "SendPort", SendPort, {});
    // t4;
    SafetyMonitor->AddInputTransition("WaitingForSend", "Error", TheLTS->MakeTrue(), {  }, "ReceivePort", ReceivePort, {});
    // t1;
    SafetyMonitor->AddInputTransition("WaitingForReceive", "CheckReceivedData", TheLTS->MakeTrue(), { new LTSAssignSimple(SafetyDataReceivedExp, TheLTS->MakeOp(LTSOps::OpField, ReceivePortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType()))) }, "ReceivePort", ReceivePort, {});
    // t5;
    SafetyMonitor->AddInputTransition("WaitingForReceive", "Error", TheLTS->MakeTrue(), {  }, "SendPort", SendPort, {});
    // t3;
    SafetyMonitor->AddInternalTransition("CheckReceivedData", "WaitingForSend", TheLTS->MakeOp(LTSOps::OpEQ, SafetyDataSentExp, SafetyDataReceivedExp), {  });
    // t2;
    SafetyMonitor->AddInternalTransition("CheckReceivedData", "Error", TheLTS->MakeOp(LTSOps::OpNOT, TheLTS->MakeOp(LTSOps::OpEQ, SafetyDataSentExp, SafetyDataReceivedExp)), {  });
    // t6;
    SafetyMonitor->AddInputTransition("Error", "Error", TheLTS->MakeTrue(), {  }, "SendPort", SendPort, {});
    // t6;
    SafetyMonitor->AddInputTransition("Error", "Error", TheLTS->MakeTrue(), {  }, "ReceivePort", ReceivePort, {});
    auto BackwardTagChannel = TheLTS->MakeGenEFSM("BackwardTagChannel", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    BackwardTagChannel->AddState("Full");
    BackwardTagChannel->AddState("Empty");
    BackwardTagChannel->FreezeStates();
    BackwardTagChannel->AddVariable("BackwardChannelTag", TheLTS->MakeRangeType(0, 1));
    auto BackwardChannelTagExp = TheLTS->MakeVar("BackwardChannelTag", TheLTS->MakeRangeType(0, 1));
    BackwardTagChannel->FreezeVars();
    BackwardTagChannel->AddInputMsg(BackwardInputChannelPort);
    BackwardTagChannel->AddOutputMsg(BackwardOutputChannelPort);
    // t_duplicating;
    BackwardTagChannel->AddOutputTransition("Full", "Full", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, BackwardOutputChannelPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), BackwardChannelTagExp) }, "BackwardOutputChannelPort", BackwardOutputChannelPort, {});
    // t2;
    BackwardTagChannel->AddInputTransition("Full", "Full", TheLTS->MakeTrue(), {  }, "BackwardInputChannelPort", BackwardInputChannelPort, {});
    // t1;
    BackwardTagChannel->AddOutputTransition("Full", "Empty", TheLTS->MakeTrue(), { new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, BackwardOutputChannelPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType())), BackwardChannelTagExp) }, "BackwardOutputChannelPort", BackwardOutputChannelPort, {});
    // t0;
    BackwardTagChannel->AddInputTransition("Empty", "Full", TheLTS->MakeTrue(), { new LTSAssignSimple(BackwardChannelTagExp, TheLTS->MakeOp(LTSOps::OpField, BackwardInputChannelPortExp, TheLTS->MakeVar("Field0", TheLTS->MakeFieldAccessType()))) }, "BackwardInputChannelPort", BackwardInputChannelPort, {});
    // t_loss;
    BackwardTagChannel->AddInputTransition("Empty", "Empty", TheLTS->MakeTrue(), {  }, "BackwardInputChannelPort", BackwardInputChannelPort, {});
    auto LivenessMonitor1 = TheLTS->MakeGenEFSM("LivenessMonitor1", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    LivenessMonitor1->AddState("Initial");
    LivenessMonitor1->AddState("Final");
    LivenessMonitor1->AddState("Accept");
    LivenessMonitor1->FreezeStates();
    LivenessMonitor1->FreezeVars();
    LivenessMonitor1->AddInputMsg(SendPort);
    LivenessMonitor1->AddInputMsg(ReceivePort);
    // t2;
    LivenessMonitor1->AddInputTransition("Initial", "Initial", TheLTS->MakeTrue(), {  }, "SendPort", SendPort, {});
    // t3;
    LivenessMonitor1->AddInputTransition("Initial", "Initial", TheLTS->MakeTrue(), {  }, "ReceivePort", ReceivePort, {});
    // t1;
    LivenessMonitor1->AddInputTransition("Initial", "Accept", TheLTS->MakeTrue(), {  }, "SendPort", SendPort, {});
    // t6;
    LivenessMonitor1->AddInputTransition("Final", "Final", TheLTS->MakeTrue(), {  }, "SendPort", SendPort, {});
    // t7;
    LivenessMonitor1->AddInputTransition("Final", "Final", TheLTS->MakeTrue(), {  }, "ReceivePort", ReceivePort, {});
    // t5;
    LivenessMonitor1->AddInputTransition("Accept", "Final", TheLTS->MakeTrue(), {  }, "ReceivePort", ReceivePort, {});
    // t4;
    LivenessMonitor1->AddInputTransition("Accept", "Accept", TheLTS->MakeTrue(), {  }, "SendPort", SendPort, {});
    auto LivenessMonitor2 = TheLTS->MakeGenEFSM("LivenessMonitor2", {}, TheLTS->MakeTrue(), LTSFairnessType::None);
    LivenessMonitor2->AddState("Initial");
    LivenessMonitor2->AddState("Final");
    LivenessMonitor2->AddState("Accept");
    LivenessMonitor2->FreezeStates();
    LivenessMonitor2->FreezeVars();
    LivenessMonitor2->AddInputMsg(ReceivePort);
    LivenessMonitor2->AddInputMsg(SendPort);
    // t2;
    LivenessMonitor2->AddInputTransition("Initial", "Initial", TheLTS->MakeTrue(), {  }, "ReceivePort", ReceivePort, {});
    // t3;
    LivenessMonitor2->AddInputTransition("Initial", "Initial", TheLTS->MakeTrue(), {  }, "SendPort", SendPort, {});
    // t1;
    LivenessMonitor2->AddInputTransition("Initial", "Accept", TheLTS->MakeTrue(), {  }, "ReceivePort", ReceivePort, {});
    // t6;
    LivenessMonitor2->AddInputTransition("Final", "Final", TheLTS->MakeTrue(), {  }, "ReceivePort", ReceivePort, {});
    // t7;
    LivenessMonitor2->AddInputTransition("Final", "Final", TheLTS->MakeTrue(), {  }, "SendPort", SendPort, {});
    // t5;
    LivenessMonitor2->AddInputTransition("Accept", "Final", TheLTS->MakeTrue(), {  }, "SendPort", SendPort, {});
    // t4;
    LivenessMonitor2->AddInputTransition("Accept", "Accept", TheLTS->MakeTrue(), {  }, "ReceivePort", ReceivePort, {});

    TheLTS->FreezeAutomata();
    vector<LTSAssignRef> InitUpdates;

    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("SenderClient", TheLTS->GetEFSMType("SenderClient")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Initial", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("SenderClient", TheLTS->GetEFSMType("SenderClient")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Sender", TheLTS->GetEFSMType("Sender")), TheLTS->MakeVar("SenderInputMessage", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Sender", TheLTS->GetEFSMType("Sender")), TheLTS->MakeVar("SenderTag", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Sender", TheLTS->GetEFSMType("Sender")), TheLTS->MakeVar("AckTag", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Sender", TheLTS->GetEFSMType("Sender")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Initial", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Sender", TheLTS->GetEFSMType("Sender")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("ForwardTagDataChannel", TheLTS->GetEFSMType("ForwardTagDataChannel")), TheLTS->MakeVar("ForwardChannelTag", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("ForwardTagDataChannel", TheLTS->GetEFSMType("ForwardTagDataChannel")), TheLTS->MakeVar("ForwardChannelData", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("ForwardTagDataChannel", TheLTS->GetEFSMType("ForwardTagDataChannel")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Empty", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("ForwardTagDataChannel", TheLTS->GetEFSMType("ForwardTagDataChannel")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Timer", TheLTS->GetEFSMType("Timer")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Initial", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Timer", TheLTS->GetEFSMType("Timer")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Receiver", TheLTS->GetEFSMType("Receiver")), TheLTS->MakeVar("ReceiverTag", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Receiver", TheLTS->GetEFSMType("Receiver")), TheLTS->MakeVar("ReceiverData", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Receiver", TheLTS->GetEFSMType("Receiver")), TheLTS->MakeVar("ExpectedTag", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Receiver", TheLTS->GetEFSMType("Receiver")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Initial", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("Receiver", TheLTS->GetEFSMType("Receiver")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("ReceiverClient", TheLTS->GetEFSMType("ReceiverClient")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Initial", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("ReceiverClient", TheLTS->GetEFSMType("ReceiverClient")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("SafetyMonitor", TheLTS->GetEFSMType("SafetyMonitor")), TheLTS->MakeVar("SafetyDataSent", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("SafetyMonitor", TheLTS->GetEFSMType("SafetyMonitor")), TheLTS->MakeVar("SafetyDataReceived", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("SafetyMonitor", TheLTS->GetEFSMType("SafetyMonitor")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("WaitingForSend", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("SafetyMonitor", TheLTS->GetEFSMType("SafetyMonitor")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("BackwardTagChannel", TheLTS->GetEFSMType("BackwardTagChannel")), TheLTS->MakeVar("BackwardChannelTag", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("0", TheLTS->MakeRangeType(0, 0))));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("BackwardTagChannel", TheLTS->GetEFSMType("BackwardTagChannel")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Empty", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("BackwardTagChannel", TheLTS->GetEFSMType("BackwardTagChannel")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("LivenessMonitor1", TheLTS->GetEFSMType("LivenessMonitor1")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Initial", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("LivenessMonitor1", TheLTS->GetEFSMType("LivenessMonitor1")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));
    InitUpdates.push_back(new LTSAssignSimple(TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("LivenessMonitor2", TheLTS->GetEFSMType("LivenessMonitor2")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType())), TheLTS->MakeVal("Initial", TheLTS->MakeOp(LTSOps::OpField, TheLTS->MakeVar("LivenessMonitor2", TheLTS->GetEFSMType("LivenessMonitor2")), TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))->GetType())));

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

    cout << "Channel Buffer variables to sort:" << endl;
    for (auto const& BufferExp : TheLTS->GetChanBuffersToSort()) {
        cout << BufferExp.first->ToString() << endl;
        cout << BufferExp.second->ToString() << endl;
    }

    auto Checker = new LTSChecker(TheLTS);
    Checker->BuildAQS();

    delete Checker;
}
