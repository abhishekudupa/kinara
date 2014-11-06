// Dijkstra4.cpp ---

// Code:

// We model and verify the 4-state self-stabilizing protocol presented in Dijkstra's
// original paper on the topic.

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "../../src/uflts/LabelledTS.hpp"
#include "../../src/uflts/LTSEFSM.hpp"
#include "../../src/uflts/LTSChannelEFSM.hpp"
#include "../../src/uflts/LTSAssign.hpp"
#include "../../src/uflts/LTSTransitions.hpp"
#include "../../src/uflts/LTSUtils.hpp"
#include "../../src/mc/LTSChecker.hpp"
#include "../../src/mc/OmegaAutomaton.hpp"
#include "../../src/mc/Trace.hpp"
#include "../../src/synth/Solver.hpp"
#include "../../src/tpinterface/TheoremProver.hpp"



using boost::lexical_cast;
using namespace ESMC;
using namespace LTS;
using namespace Exprs;
using namespace MC;
using namespace Synth;


#define __LOGSTR__ string(__FILE__) + ", " + to_string(__LINE__) + ": "

// for number of processes = 3, the minimum number of legitimate states is 8.
// for number of processes = 4, the minimum number of legitimate states is 12.
const size_t NumProcesses = 5;


// Messages
vector<ExprTypeRef> WriteMsgs;

vector<ExpT> Guards;

ExprTypeRef LegitimateAnnouncement;
ExprTypeRef IllegitimateAnnouncement;

void DeclareMsgs(LabelledTS* TheLTS)
{
    assert(TheLTS != nullptr);
    cout << __LOGSTR__ << "Declaring messages." << endl;

    auto BoolType = TheLTS->MakeBoolType();
    for (size_t i = 0; i < NumProcesses; i++) {
        vector<pair<string, ExprTypeRef>> fields { make_pair(string("Data"), BoolType),
                                                   make_pair(string("Up"), BoolType) };
        string MsgName = string("Write") + to_string(i);
        WriteMsgs.push_back(TheLTS->MakeMsgType(MsgName, fields));
    }

    LegitimateAnnouncement = TheLTS->MakeMsgType("LegitimateAnnouncement", {});
    IllegitimateAnnouncement = TheLTS->MakeMsgType("IllegitimateAnnouncement", {});

    TheLTS->FreezeMsgs();
    cout << __LOGSTR__ << "Declaring messages done." << endl;
}

// Processes
vector<GeneralEFSM*> Proc;
IncompleteEFSM* ShadowMonitor;

void DeclareProc0(LabelledTS* TheLTS)
{
    assert(TheLTS != nullptr && Proc.size() == 0);
    cout << __LOGSTR__ << "Declaring process 0." << endl;
    string ProcName = string("Proc0");

    Proc.push_back(TheLTS->MakeGenEFSM(ProcName, {}, TheLTS->MakeTrue(), LTSFairnessType::None));
    Proc[0]->AddState("TheState");
    Proc[0]->FreezeStates();

    cout << __LOGSTR__ << "Declaring variables." << endl;
    Proc[0]->AddVariable(string("Data0"), TheLTS->MakeBoolType());
    Proc[0]->AddVariable(string("Data1"), TheLTS->MakeBoolType());
    Proc[0]->AddVariable(string("Up0"), TheLTS->MakeBoolType());
    Proc[0]->AddVariable(string("Up1"), TheLTS->MakeBoolType());
    Proc[0]->FreezeVars();

    cout << __LOGSTR__ << "Adding transitions." << endl;

    Proc[0]->AddInputMsg(WriteMsgs[1], {});
    auto D0Exp = TheLTS->MakeVar("Data0", TheLTS->MakeBoolType());
    auto D1Exp = TheLTS->MakeVar("Data1", TheLTS->MakeBoolType());
    auto U0Exp = TheLTS->MakeVar("Up0", TheLTS->MakeBoolType());
    auto U1Exp = TheLTS->MakeVar("Up1", TheLTS->MakeBoolType());
    auto R1Exp = TheLTS->MakeVar("R1", WriteMsgs[1]);
    auto D1PayloadAccField = TheLTS->MakeVar("Data", TheLTS->MakeFieldAccessType());
    auto R1DataFieldExp = TheLTS->MakeOp(LTSOps::OpField,
                                         R1Exp,
                                         D1PayloadAccField);
    auto U1PayloadAccField = TheLTS->MakeVar("Up", TheLTS->MakeFieldAccessType());
    auto R1UpFieldExp = TheLTS->MakeOp(LTSOps::OpField,
                                       R1Exp,
                                       U1PayloadAccField);
    vector<LTSAssignRef> R1Updates { new LTSAssignSimple(D1Exp, R1DataFieldExp),
                                     new LTSAssignSimple(U1Exp, R1UpFieldExp) };
    Proc[0]->AddInputTransition("TheState", "TheState", TheLTS->MakeTrue(),
                                R1Updates,
                                "R1",
                                WriteMsgs[1], {});

    Proc[0]->AddOutputMsg(WriteMsgs[0], {});
    // If DataS = DataR and !UpR, then DataS := !DataS.
    auto W0Exp = TheLTS->MakeVar("W0", WriteMsgs[0]);
    auto NotD0Exp = TheLTS->MakeOp(LTSOps::OpNOT, D0Exp);
    auto Data0EqData1 = TheLTS->MakeOp(LTSOps::OpEQ, D0Exp, D1Exp);
    auto NotU1Exp = TheLTS->MakeOp(LTSOps::OpNOT, U1Exp);
    auto Guard = TheLTS->MakeOp(LTSOps::OpAND, Data0EqData1, NotU1Exp);
    Guards.push_back(Guard);
    auto D0PayloadAccField = TheLTS->MakeVar("Data", TheLTS->MakeFieldAccessType());
    auto U0PayloadAccField = TheLTS->MakeVar("Up", TheLTS->MakeFieldAccessType());
    auto W0DataFieldExp = TheLTS->MakeOp(LTSOps::OpField,
                                         W0Exp,
                                         D0PayloadAccField);
    auto W0UpFieldExp = TheLTS->MakeOp(LTSOps::OpField,
                                       W0Exp,
                                       U0PayloadAccField);
    vector<LTSAssignRef> W0Updates { new LTSAssignSimple(D0Exp, NotD0Exp),
                                     new LTSAssignSimple(U0Exp, U0Exp),
                                     new LTSAssignSimple(W0DataFieldExp, NotD0Exp),
                                     new LTSAssignSimple(W0UpFieldExp, U0Exp) };
    Proc[0]->AddOutputTransition("TheState", "TheState", Guard,
                                 W0Updates,
                                 "W0",
                                 WriteMsgs[0], {});

    cout << __LOGSTR__ << "Declaring process 0 done." << endl;
}

void DeclareProcMid(LabelledTS* TheLTS, size_t i)
{
    assert(TheLTS != nullptr && i > 0 && Proc.size() == i && i + 1 < NumProcesses);
    cout << __LOGSTR__ << "Declaring process " << i << "." << endl;
    string ProcName = string("Proc") + to_string(i);

    Proc.push_back(TheLTS->MakeGenEFSM(ProcName, {}, TheLTS->MakeTrue(), LTSFairnessType::None));
    Proc[i]->AddState("TheState");
    Proc[i]->FreezeStates();

    cout << __LOGSTR__ << "Declaring variables." << endl;
    Proc[i]->AddVariable(string("Data") + to_string(i - 1), TheLTS->MakeBoolType());
    Proc[i]->AddVariable(string("Data") + to_string(i), TheLTS->MakeBoolType());
    Proc[i]->AddVariable(string("Data") + to_string(i + 1), TheLTS->MakeBoolType());
    Proc[i]->AddVariable(string("Up") + to_string(i), TheLTS->MakeBoolType());
    Proc[i]->AddVariable(string("Up") + to_string(i + 1), TheLTS->MakeBoolType());
    Proc[i]->FreezeVars();

    cout << __LOGSTR__ << "Adding transitions." << endl;

    Proc[i]->AddInputMsg(WriteMsgs[i - 1], {});
    auto Rim1Exp = TheLTS->MakeVar("R" + to_string(i - 1), WriteMsgs[i - 1]);
    auto Dim1Exp = TheLTS->MakeVar(string("Data") + to_string(i - 1), TheLTS->MakeBoolType());
    auto Dim1PayloadAccField = TheLTS->MakeVar("Data", TheLTS->MakeFieldAccessType());
    auto Rim1DataFieldExp = TheLTS->MakeOp(LTSOps::OpField,
                                           Rim1Exp,
                                           Dim1PayloadAccField);
    vector<LTSAssignRef> Rim1Updates { new LTSAssignSimple(Dim1Exp, Rim1DataFieldExp) };
    Proc[i]->AddInputTransition("TheState", "TheState", TheLTS->MakeTrue(),
                                Rim1Updates,
                                "R" + to_string(i - 1),
                                WriteMsgs[i - 1], {});

    Proc[i]->AddOutputMsg(WriteMsgs[i], {});

    // If DataS != DataL, then DataS := !DataS, UpS := true.
    auto WiExp = TheLTS->MakeVar("W" + to_string(i), WriteMsgs[i]);
    auto DiExp = TheLTS->MakeVar(string("Data") + to_string(i), TheLTS->MakeBoolType());
    auto UiExp = TheLTS->MakeVar(string("Up") + to_string(i), TheLTS->MakeBoolType());
    auto DataiEqDataim1 = TheLTS->MakeOp(LTSOps::OpEQ, DiExp, Dim1Exp);
    auto DataiNeqDataim1 = TheLTS->MakeOp(LTSOps::OpNOT, DataiEqDataim1);
    auto DiPayloadAccField = TheLTS->MakeVar("Data", TheLTS->MakeFieldAccessType());
    auto UiPayloadAccField = TheLTS->MakeVar("Up", TheLTS->MakeFieldAccessType());
    auto WiDataFieldExp = TheLTS->MakeOp(LTSOps::OpField,
                                         WiExp,
                                         DiPayloadAccField);
    auto WiUpFieldExp = TheLTS->MakeOp(LTSOps::OpField,
                                       WiExp,
                                       UiPayloadAccField);
    vector<LTSAssignRef> W1iUpdates { new LTSAssignSimple(DiExp, TheLTS->MakeOp(LTSOps::OpNOT, DiExp)),
                                      new LTSAssignSimple(UiExp, TheLTS->MakeTrue()),
                                      new LTSAssignSimple(WiDataFieldExp, TheLTS->MakeOp(LTSOps::OpNOT, DiExp)),
                                      new LTSAssignSimple(WiUpFieldExp, TheLTS->MakeTrue()) };
    Guards.push_back(DataiNeqDataim1);
    Proc[i]->AddOutputTransition("TheState", "TheState", DataiNeqDataim1,
                                 W1iUpdates,
                                 "W" + to_string(i),
                                 WriteMsgs[i], {});

    // If DataS = DataR and UpS and !UpR, then UpS := false.
    auto Dip1Exp = TheLTS->MakeVar(string("Data") + to_string(i + 1), TheLTS->MakeBoolType());
    auto DataiEqDataip1 = TheLTS->MakeOp(LTSOps::OpEQ, DiExp, Dip1Exp);
    auto Uip1Exp = TheLTS->MakeVar(string("Up") + to_string(i + 1), TheLTS->MakeBoolType());
    auto NotUip1Exp = TheLTS->MakeOp(LTSOps::OpNOT, Uip1Exp);
    auto Guard = TheLTS->MakeOp(LTSOps::OpAND, { DataiEqDataip1, UiExp, NotUip1Exp });
    // The following fails for 4 processes, not for 3, becaues Uip1 is fixed to False
    // auto Guard = TheLTS->MakeOp(LTSOps::OpAND, { DataiEqDataip1, UiExp });

    // The following fails for 3 processes
    // auto Guard = TheLTS->MakeOp(LTSOps::OpAND, { DataiEqDataip1, NotUip1Exp });
    Guards.push_back(Guard);
    vector<LTSAssignRef> W2iUpdates { new LTSAssignSimple(DiExp, DiExp),
                                      new LTSAssignSimple(UiExp, TheLTS->MakeFalse()),
                                      new LTSAssignSimple(WiDataFieldExp, DiExp),
                                      new LTSAssignSimple(WiUpFieldExp, TheLTS->MakeFalse()) };
    Proc[i]->AddOutputTransition("TheState", "TheState", Guard,
                                 W2iUpdates,
                                 "W" + to_string(i),
                                 WriteMsgs[i], {});

    Proc[i]->AddInputMsg(WriteMsgs[i + 1], {});
    auto Rip1Exp = TheLTS->MakeVar("R" + to_string(i + 1), WriteMsgs[i + 1]);
    auto Dip1PayloadAccField = TheLTS->MakeVar("Data", TheLTS->MakeFieldAccessType());
    auto Rip1DataFieldExp = TheLTS->MakeOp(LTSOps::OpField,
                                           Rip1Exp,
                                           Dip1PayloadAccField);
    auto Uip1PayloadAccField = TheLTS->MakeVar("Up", TheLTS->MakeFieldAccessType());
    auto Rip1UpFieldExp = TheLTS->MakeOp(LTSOps::OpField,
                                         Rip1Exp,
                                         Uip1PayloadAccField);
    vector<LTSAssignRef> Rip1Updates { new LTSAssignSimple(Dip1Exp, Rip1DataFieldExp),
                                       new LTSAssignSimple(Uip1Exp, Rip1UpFieldExp) };
    Proc[i]->AddInputTransition("TheState", "TheState", TheLTS->MakeTrue(),
                                Rip1Updates,
                                "R" + to_string(i + 1),
                                WriteMsgs[i + 1], {});

    cout << __LOGSTR__ << "Declaring process " << i << " done." << endl;
}

void DeclareProcN(LabelledTS* TheLTS)
{
    assert(TheLTS != nullptr && Proc.size() + 1 == NumProcesses);
    size_t i = Proc.size();
    cout << __LOGSTR__ << "Declaring process " << i << "." << endl;
    string ProcName = string("Proc") + to_string(i);

    Proc.push_back(TheLTS->MakeGenEFSM(ProcName, {}, TheLTS->MakeTrue(), LTSFairnessType::None));
    Proc[i]->AddState("TheState");
    Proc[i]->FreezeStates();

    cout << __LOGSTR__ << "Declaring variables." << endl;
    Proc[i]->AddVariable(string("Data") + to_string(i - 1), TheLTS->MakeBoolType());
    Proc[i]->AddVariable(string("Data") + to_string(i), TheLTS->MakeBoolType());
    Proc[i]->AddVariable(string("Up") + to_string(i), TheLTS->MakeBoolType());
    Proc[i]->FreezeVars();

    cout << __LOGSTR__ << "Adding transitions." << endl;

    Proc[i]->AddInputMsg(WriteMsgs[i - 1], {});

    auto DiExp = TheLTS->MakeVar(string("Data") + to_string(i), TheLTS->MakeBoolType());
    auto Dim1Exp = TheLTS->MakeVar("Data" + to_string(i - 1), TheLTS->MakeBoolType());
    auto UiExp = TheLTS->MakeVar(string("Up") + to_string(i), TheLTS->MakeBoolType());
    auto Rim1Exp = TheLTS->MakeVar("R" + to_string(i - 1), WriteMsgs[i - 1]);
    auto Dim1PayloadAccField = TheLTS->MakeVar("Data", TheLTS->MakeFieldAccessType());
    auto Rim1DataFieldExp = TheLTS->MakeOp(LTSOps::OpField,
                                           Rim1Exp,
                                           Dim1PayloadAccField);
    vector<LTSAssignRef> Rim1Updates { new LTSAssignSimple(Dim1Exp, Rim1DataFieldExp) };
    Proc[i]->AddInputTransition("TheState", "TheState", TheLTS->MakeTrue(),
                                Rim1Updates,
                                "R" + to_string(i - 1),
                                WriteMsgs[i - 1], {});

    Proc[i]->AddOutputMsg(WriteMsgs[i], {});

    // If DataS != DataL, then DataS := !DataS.
    auto WiExp = TheLTS->MakeVar("W" + to_string(i), WriteMsgs[i]);
    auto DataiEqDataim1 = TheLTS->MakeOp(LTSOps::OpEQ, DiExp, Dim1Exp);
    auto DataiNeqDataim1 = TheLTS->MakeOp(LTSOps::OpNOT, DataiEqDataim1);
    auto DiPayloadAccField = TheLTS->MakeVar("Data", TheLTS->MakeFieldAccessType());
    auto UiPayloadAccField = TheLTS->MakeVar("Up", TheLTS->MakeFieldAccessType());
    auto WiDataFieldExp = TheLTS->MakeOp(LTSOps::OpField,
                                         WiExp,
                                         DiPayloadAccField);
    auto WiUpFieldExp = TheLTS->MakeOp(LTSOps::OpField,
                                       WiExp,
                                       UiPayloadAccField);
    vector<LTSAssignRef> WiUpdates { new LTSAssignSimple(DiExp, TheLTS->MakeOp(LTSOps::OpNOT, DiExp)),
                                     new LTSAssignSimple(UiExp, UiExp),
                                     new LTSAssignSimple(WiDataFieldExp, TheLTS->MakeOp(LTSOps::OpNOT, DiExp)),
                                     new LTSAssignSimple(WiUpFieldExp, UiExp) };
    Guards.push_back(DataiNeqDataim1);
    Proc[i]->AddOutputTransition("TheState", "TheState", DataiNeqDataim1,
                                 WiUpdates,
                                 "W" + to_string(i),
                                 WriteMsgs[i], {});

    cout << __LOGSTR__ << "Declaring process " << i << " done." << endl;
}

void DeclareShadowMonitor(LabelledTS* TheLTS)
{
    assert(TheLTS != nullptr && ShadowMonitor == nullptr);

    ShadowMonitor = TheLTS->MakeEFSM<IncompleteEFSM>("ShadowMonitor", {},
                                                     TheLTS->MakeTrue(), LTSFairnessType::None);
    ShadowMonitor->AddState("Legitimate");
    ShadowMonitor->AddState("Illegitimate");
    ShadowMonitor->AddState("Transient");
    ShadowMonitor->FreezeStates();
    for (size_t i = 0; i < NumProcesses; ++i) {
        ShadowMonitor->AddInputMsg(WriteMsgs[i], {});
        ShadowMonitor->AddVariable("Data" + to_string(i), TheLTS->MakeBoolType());
        ShadowMonitor->AddVariable("Up" + to_string(i), TheLTS->MakeBoolType());
    }
    auto LegitimateAnnouncementDeclaration = ShadowMonitor->AddOutputMsg(LegitimateAnnouncement, {});
    auto IllegitimateAnnouncementDeclaration = ShadowMonitor->AddOutputMsg(IllegitimateAnnouncement, {});
    ShadowMonitor->FreezeVars();

    for (size_t i = 0; i < NumProcesses; ++i) {
        auto Datai = TheLTS->MakeVar("Data" + to_string(i), TheLTS->MakeBoolType());
        auto Write = TheLTS->MakeVar("Write", WriteMsgs[i]);
        auto DataAccess = TheLTS->MakeVar("Data", TheLTS->MakeFieldAccessType());
        auto WriteData = TheLTS->MakeOp(LTSOps::OpField, Write, DataAccess);
        auto Upi = TheLTS->MakeVar("Up" + to_string(i), TheLTS->MakeBoolType());
        auto UpAccess = TheLTS->MakeVar("Up", TheLTS->MakeFieldAccessType());
        auto WriteUp = TheLTS->MakeOp(LTSOps::OpField, Write, UpAccess);
        vector<LTSAssignRef> Updates { new LTSAssignSimple(Datai, WriteData),
                                       new LTSAssignSimple(Upi, WriteUp) };
        ShadowMonitor->AddInputTransition("Legitimate", "Transient",
                                          TheLTS->MakeTrue(), Updates,
                                          "Write", WriteMsgs[i], {});
        ShadowMonitor->AddInputTransition("Illegitimate", "Transient",
                                          TheLTS->MakeTrue(), Updates,
                                          "Write", WriteMsgs[i], {});
    }

    ShadowMonitor->MarkAllStatesComplete();
    ShadowMonitor->MarkStateIncomplete("Transient");
    ShadowMonitor->IgnoreAllMsgsOnState("Transient");
    ShadowMonitor->HandleMsgOnState(LegitimateAnnouncementDeclaration, "Transient");
    ShadowMonitor->HandleMsgOnState(IllegitimateAnnouncementDeclaration, "Transient");
    for (size_t i = 0; i < NumProcesses; ++i) {
        ShadowMonitor->MarkVariableReadOnly("Data" + to_string(i));
        ShadowMonitor->MarkVariableReadOnly("Up" + to_string(i));
    }
}

void DeclareSafetyConcreteMonitor(LabelledTS* TheLTS)
{
    auto SafetyMonitor = TheLTS->MakeGenEFSM("SafetyMonitor", {},
                                             TheLTS->MakeTrue(), LTSFairnessType::None);
    SafetyMonitor->AddState("Initial");
    SafetyMonitor->AddState("Legitimate");
    SafetyMonitor->AddState("Error", false, false, false, true);
    SafetyMonitor->FreezeStates();
    SafetyMonitor->AddInputMsg(IllegitimateAnnouncement, {});
    SafetyMonitor->AddInputMsg(LegitimateAnnouncement, {});
    SafetyMonitor->FreezeVars();
    SafetyMonitor->AddInputTransition("Initial", "Initial",
                                      TheLTS->MakeTrue(), {},
                                      "Illegitimate", IllegitimateAnnouncement, {});
    SafetyMonitor->AddInputTransition("Initial", "Legitimate",
                                      TheLTS->MakeTrue(), {},
                                      "Legitimate", LegitimateAnnouncement, {});
    SafetyMonitor->AddInputTransition("Legitimate", "Legitimate",
                                      TheLTS->MakeTrue(), {},
                                      "Legitimate", LegitimateAnnouncement, {});
    SafetyMonitor->AddInputTransition("Legitimate", "Error",
                                      TheLTS->MakeTrue(), {},
                                      "Illegitimate", IllegitimateAnnouncement, {});
}

void DeclareAutomata(LabelledTS* TheLTS)
{
    assert(TheLTS != nullptr);
    cout << __LOGSTR__ << "Declaring automata." << endl;
    DeclareProc0(TheLTS);
    for (size_t i = 1; i + 1 < NumProcesses; i++) {
        DeclareProcMid(TheLTS, i);
    }
    DeclareProcN(TheLTS);
    DeclareShadowMonitor(TheLTS);
    DeclareSafetyConcreteMonitor(TheLTS);
    TheLTS->FreezeAutomata();

    cout << __LOGSTR__ << "Freezing automata done." << endl;

    // Initial Data and Up variables for all processes

    vector<InitStateRef> InitStates;

    vector<vector<ExpT>> DataElems;
    for (size_t i = 0; i < NumProcesses; ++i) {
        DataElems.push_back({TheLTS->MakeTrue(), TheLTS->MakeFalse()});
    }
    auto&& DataCombinations = CrossProduct<ExpT>(DataElems.begin(), DataElems.end());
    for (auto DataVal : DataCombinations) {
        vector<vector<ExpT>> UpElems;
        for (size_t i = 1; i + 1 < NumProcesses; ++i) {
            UpElems.push_back({TheLTS->MakeTrue(), TheLTS->MakeFalse()});
        }
        auto&& UpCombinations = CrossProduct<ExpT>(UpElems.begin(), UpElems.end());
        for (auto UpVal : UpCombinations) {
            UpVal.insert(UpVal.begin(), TheLTS->MakeTrue());
            UpVal.push_back(TheLTS->MakeFalse());

            vector<LTSAssignRef> InitUpdates;

            auto FAType = TheLTS->MakeFieldAccessType();

            // All Process start at the TheState location
            for (size_t i = 0; i < NumProcesses; ++i) {
                auto ProciType = TheLTS->GetEFSMType("Proc" + to_string(i));
                auto ProciStateVar = TheLTS->MakeVar("Proc" + to_string(i), ProciType);
                auto ProciState = TheLTS->MakeOp(LTSOps::OpField,
                                                 ProciStateVar,
                                                 TheLTS->MakeVar("state", FAType));
                auto ProciInitialState = TheLTS->MakeVal("TheState", ProciState->GetType());
                InitUpdates.push_back(new LTSAssignSimple(ProciState, ProciInitialState));
            }


            for (size_t i = 0; i < NumProcesses; i++) {
                auto ProciType = TheLTS->GetEFSMType("Proc" + to_string(i));
                auto ProciStateVar = TheLTS->MakeVar("Proc" + to_string(i), ProciType);
                if (i > 0) {
                    auto ProciDataim1 = TheLTS->MakeOp(LTSOps::OpField,
                                                       ProciStateVar,
                                                       TheLTS->MakeVar("Data" + to_string(i - 1), FAType));
                    InitUpdates.push_back(new LTSAssignSimple(ProciDataim1, DataVal[i - 1]));
                }
                auto ProciDatai = TheLTS->MakeOp(LTSOps::OpField,
                                                 ProciStateVar,
                                                 TheLTS->MakeVar("Data" + to_string(i), FAType));
                InitUpdates.push_back(new LTSAssignSimple(ProciDatai, DataVal[i]));

                if (i + 1 < NumProcesses) {
                    auto ProciDataip1 = TheLTS->MakeOp(LTSOps::OpField,
                                                       ProciStateVar,
                                                       TheLTS->MakeVar("Data" + to_string(i + 1), FAType));
                    InitUpdates.push_back(new LTSAssignSimple(ProciDataip1, DataVal[i + 1]));
                }
                auto ProciUpi = TheLTS->MakeOp(LTSOps::OpField,
                                               ProciStateVar,
                                               TheLTS->MakeVar("Up" + to_string(i), FAType));
                InitUpdates.push_back(new LTSAssignSimple(ProciUpi, UpVal[i]));

                if (i + 1 < NumProcesses) {
                    auto ProciUpip1 = TheLTS->MakeOp(LTSOps::OpField,
                                                     ProciStateVar,
                                                     TheLTS->MakeVar("Up" + to_string(i + 1), FAType));
                    InitUpdates.push_back(new LTSAssignSimple(ProciUpip1, UpVal[i + 1]));
                }
            }

            // Initialize Shadow Monitor
            auto ShadowMonitorType = TheLTS->GetEFSMType("ShadowMonitor");
            auto ShadowMonitorStateVar = TheLTS->MakeVar("ShadowMonitor", ShadowMonitorType);
            auto ShadowMonitorState = TheLTS->MakeOp(LTSOps::OpField, ShadowMonitorStateVar, TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()));
            auto ShadowMonitorInitialState = TheLTS->MakeVal("Transient", ShadowMonitorState->GetType());
            InitUpdates.push_back(new LTSAssignSimple(ShadowMonitorState, ShadowMonitorInitialState));
            for (size_t i = 0; i < NumProcesses; ++i) {
                auto ShadowDatai = TheLTS->MakeOp(LTSOps::OpField,
                                                  ShadowMonitorStateVar,
                                                  TheLTS->MakeVar("Data" + to_string(i), FAType));
                auto ShadowUpi = TheLTS->MakeOp(LTSOps::OpField,
                                                ShadowMonitorStateVar,
                                                TheLTS->MakeVar("Up" + to_string(i), FAType));
                InitUpdates.push_back(new LTSAssignSimple(ShadowDatai, DataVal[i]));
                InitUpdates.push_back(new LTSAssignSimple(ShadowUpi, UpVal[i]));
            }

            // Initialize Safety Monitor
            auto SafetyMonitorType = TheLTS->GetEFSMType("SafetyMonitor");
            auto SafetyMonitorStateVar = TheLTS->MakeVar("SafetyMonitor", SafetyMonitorType);
            auto SafetyMonitorState = TheLTS->MakeOp(LTSOps::OpField, SafetyMonitorStateVar,
                                                     TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()));
            auto SafetyMonitorInitialState = TheLTS->MakeVal("Initial", SafetyMonitorState->GetType());
            InitUpdates.push_back(new LTSAssignSimple(SafetyMonitorState, SafetyMonitorInitialState));


            InitStates.push_back(new LTSInitState({}, TheLTS->MakeTrue(), InitUpdates));

        }
    }
    cout << __LOGSTR__ << "Init states size " << InitStates.size() << endl;
    TheLTS->AddInitStates(InitStates);
}


void DeclareLivenessMonitor(LabelledTS* TheLTS, LTSChecker* Checker)
{
    auto Monitor = Checker->MakeStateBuchiMonitor("InfinitelyIllegitimate",
                                                  {}, TheLTS->MakeTrue());
    Monitor->AddState("LastIllegit", true, true);
    Monitor->AddState("LastNotIllegit", false, false);
    Monitor->FreezeStates();

    auto ShadowMonitorType = TheLTS->GetEFSMType("ShadowMonitor");
    auto ShadowMonitorStateVar = TheLTS->MakeVar("ShadowMonitor", ShadowMonitorType);
    auto ShadowMonitorState = TheLTS->MakeOp(LTSOps::OpField, ShadowMonitorStateVar, TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()));
    auto ShadowMonitorIllegitimateState = TheLTS->MakeVal("Illegitimate", ShadowMonitorState->GetType());
    auto ShadowMonitorStateEQIllegitimateState = TheLTS->MakeOp(LTSOps::OpEQ, ShadowMonitorState, ShadowMonitorIllegitimateState);
    auto ShadowMonitorStateNEQIllegitimateState = TheLTS->MakeOp(LTSOps::OpNOT, ShadowMonitorStateEQIllegitimateState);
    Monitor->AddTransition("LastIllegit", "LastIllegit", ShadowMonitorStateEQIllegitimateState);
    Monitor->AddTransition("LastIllegit", "LastNotIllegit", ShadowMonitorStateNEQIllegitimateState);
    Monitor->AddTransition("LastNotIllegit", "LastIllegit", ShadowMonitorStateEQIllegitimateState);
    Monitor->AddTransition("LastNotIllegit", "LastNotIllegit", ShadowMonitorStateNEQIllegitimateState);
    Monitor->Freeze();
}

vector<unordered_map<string, ExpT>> GetValidStatesMap(LabelledTS* TheLTS) {
    vector<unordered_map<string, ExpT>> Retval;
    vector<vector<ExpT>> DataElems;
    for (size_t i = 0; i < 2 * NumProcesses - 2; ++i) {
        DataElems.push_back({TheLTS->MakeTrue(), TheLTS->MakeFalse()});
    }
    auto&& DataCombinations = CrossProduct<ExpT>(DataElems.begin(), DataElems.end());
    vector<ExpT> Indicators;
    for (auto DataCombination : DataCombinations) {
        unordered_map<string, ExpT> ValidState;
        u32 DataCombinationIndex = 0;
        for (u32 i = 0; i < NumProcesses; ++i) {
            ValidState["Data" + to_string(i)] = DataCombination[DataCombinationIndex];
            DataCombinationIndex++;
        }
        for (u32 i = 1; i < NumProcesses - 1; ++i) {
            ValidState["Up" + to_string(i)] = DataCombination[DataCombinationIndex];
            DataCombinationIndex++;
        }
        ValidState["Up0"] = TheLTS->MakeTrue();
        ValidState["Up" + to_string(NumProcesses - 1)] = TheLTS->MakeFalse();
        Retval.push_back(ValidState);
    }
    return Retval;
}


int main()
{
    cout << __LOGSTR__ << "" << "Compiled on " << __DATE__ << " at " << __TIME__ << "." << endl;
    auto TheLTS = new LabelledTS();

    DeclareMsgs(TheLTS);
    DeclareAutomata(TheLTS);

    TheLTS->Freeze();
    cout << __LOGSTR__ << "Freezing LTS done." << endl;

    auto Checker = new LTSChecker(TheLTS);

    DeclareLivenessMonitor(TheLTS, Checker);

    auto TheSolver = new Solver(Checker);

    auto ShadowMonitorType = TheLTS->GetEFSMType("ShadowMonitor");
    auto ShadowMonitorStateVar = TheLTS->MakeVar("ShadowMonitor", ShadowMonitorType);
    auto ShadowMonitorState = TheLTS->MakeOp(LTSOps::OpField, ShadowMonitorStateVar, TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()));
    auto ShadowMonitorIllegitimateState = TheLTS->MakeVal("Illegitimate",
                                                          ShadowMonitorState->GetType());
    auto ShadowMonitorLegitimateState = TheLTS->MakeVal("Legitimate",
                                                        ShadowMonitorState->GetType());
    auto HasUF = [&] (const ExpBaseT* Exp) -> bool
        {
            auto ExpAsOpExp = Exp->As<OpExpression>();
            if (ExpAsOpExp != nullptr) {
                auto Code = ExpAsOpExp->GetOpCode();
                return (Code >= LTSOps::UFOffset);
            }
            return false;
        };

    auto ShadowIllegitimateTransition =
        ShadowMonitor->GetOutputTransitionsOnMsg(IllegitimateAnnouncement)[0];
    auto Guard = ShadowIllegitimateTransition->GetGuard();
    auto UFFunctionsInGuard = Guard->GetMgr()->Gather(Guard, HasUF);
    auto UFGuard = *(UFFunctionsInGuard.begin());
    auto UFGuardOpCode = UFGuard->As<OpExpression>()->GetOpCode();
    TheSolver->UnveilGuardOp(UFGuardOpCode);

    for (auto Transition : ShadowMonitor->GetOutputTransitionsOnMsg(LegitimateAnnouncement)) {
        ////////////////////////////////////////////////////////////
        // Enforce number of legitimate states is less than ...
        ////////////////////////////////////////////////////////////
        auto Guard = Transition->GetGuard();
        auto UFFunctionsInGuard = Guard->GetMgr()->Gather(Guard, HasUF);
        auto UFGuard = *(UFFunctionsInGuard.begin());
        auto UFGuardOpCode = UFGuard->As<OpExpression>()->GetOpCode();

        vector<vector<ExpT>> DataElems;

        DataElems.clear();
        for (size_t i = 0; i < 2 * NumProcesses - 2; ++i) {
            DataElems.push_back({TheLTS->MakeTrue(), TheLTS->MakeFalse()});
        }
        auto&& DataCombinations = CrossProduct<ExpT>(DataElems.begin(), DataElems.end());
        int CandidateStateCounter = 1;
        vector<ExpT> Indicators;
        for (auto DataVal : DataCombinations) {
            vector<ExpT> ActualArguments;
            int DataCombinationIndex = 0;
            for (auto Arg : UFGuard->As<OpExpression>()->GetChildren()) {
                auto ArgField = Arg->As<OpExpression>()->GetChildren()[1];
                auto ArgName = ArgField->As<VarExpression>()->GetVarName();
                if (ArgName ==  "Up0") {
                    ActualArguments.push_back(TheLTS->MakeTrue());
                } else if (ArgName == ("Up" + to_string(NumProcesses - 1))) {
                    ActualArguments.push_back(TheLTS->MakeFalse());
                } else {
                    ActualArguments.push_back(DataVal[DataCombinationIndex]);
                    DataCombinationIndex++;
                }
            }
            auto ZeroOneType = TheLTS->MakeRangeType(0, 1);
            cout << "Creating indicator variable with index " << CandidateStateCounter << endl;
            auto IndicatorVariable = TheLTS->MakeVar("LegitimateIndicator" + to_string(CandidateStateCounter),
                                                     ZeroOneType);
            ExpT LegitimateStatePredicate = TheLTS->MakeOp(UFGuardOpCode, ActualArguments);
            auto IndicatorVariableEQOne = TheLTS->MakeOp(LTSOps::OpEQ, IndicatorVariable,
                                                         TheLTS->MakeVal("1", ZeroOneType));
            TheSolver->MakeAssertion(TheLTS->MakeOp(LTSOps::OpIMPLIES,
                                                    LegitimateStatePredicate,
                                                    IndicatorVariableEQOne));
            Indicators.push_back(IndicatorVariable);
            CandidateStateCounter++;
        }
        auto IndicatorSum = TheLTS->MakeOp(LTSOps::OpADD, Indicators);
        auto NumIndicatorsType = TheLTS->MakeRangeType(0, Indicators.size());

        int NumLegitimateStates = 1;
        if (NumProcesses == 3) {
            NumLegitimateStates = 8;
        } else if (NumProcesses == 4) {
            NumLegitimateStates = 12;
        } else if (NumProcesses == 5) {
            NumLegitimateStates = 16;
        }
        ExpT NumLegitimateStatesExp = TheLTS->MakeVal(to_string(NumLegitimateStates),
                                                      NumIndicatorsType);
        TheSolver->MakeAssertion(TheLTS->MakeOp(LTSOps::OpLE,
                                                IndicatorSum,
                                                NumLegitimateStatesExp));
        ////////////////////////////////////////////////////////////


        ////////////////////////////////////////////////////////////
        // Enforce Legitimate transition goes to legitimate state
        ////////////////////////////////////////////////////////////
        auto StateUpdate = Transition->GetUpdates()[0];
        auto StateUpdateExpression = StateUpdate->GetRHS();
        auto UFOpCode = StateUpdateExpression->As<OpExpression>()->GetOpCode();
        DataElems.clear();
        for (size_t i = 0; i < 2 * NumProcesses; ++i) {
            DataElems.push_back({TheLTS->MakeTrue(), TheLTS->MakeFalse()});
        }
        DataCombinations = CrossProduct<ExpT>(DataElems.begin(), DataElems.end());
        for (auto DataVal : DataCombinations) {
            auto StateUpdateExp = TheLTS->MakeOp(UFOpCode, DataVal);
            auto StateUpdateExpEQLegitimateState = TheLTS->MakeOp(LTSOps::OpEQ, StateUpdateExp, ShadowMonitorLegitimateState);
            TheSolver->MakeAssertion(StateUpdateExpEQLegitimateState);
        }
        ////////////////////////////////////////////////////////////
    }

    for (auto Transition : ShadowMonitor->GetOutputTransitionsOnMsg(IllegitimateAnnouncement)) {
        ////////////////////////////////////////////////////////////
        // Enforce Illegitimate transition goes to illegitimate state
        ////////////////////////////////////////////////////////////
        auto StateUpdate = Transition->GetUpdates()[0];
        auto StateUpdateExpression = StateUpdate->GetRHS();
        auto UFOpCode = StateUpdateExpression->As<OpExpression>()->GetOpCode();
        vector<vector<ExpT>> DataElems;
        for (size_t i = 0; i < 2 * NumProcesses; ++i) {
            DataElems.push_back({TheLTS->MakeTrue(), TheLTS->MakeFalse()});
        }
        auto&& DataCombinations = CrossProduct<ExpT>(DataElems.begin(), DataElems.end());
        for (auto DataVal : DataCombinations) {
            auto StateUpdateExp = TheLTS->MakeOp(UFOpCode, DataVal);
            auto StateUpdateExpEQIllegitimateState = TheLTS->MakeOp(LTSOps::OpEQ, StateUpdateExp, ShadowMonitorIllegitimateState);
            TheSolver->MakeAssertion(StateUpdateExpEQIllegitimateState);
        }
        ////////////////////////////////////////////////////////////
    }

    TheSolver->Solve();

    TheSolver->PrintSolution();

    delete Checker;

    cout << __LOGSTR__ << "Return." << endl;
    return 0;
}

//
// Dijkstra4.cpp ends here
