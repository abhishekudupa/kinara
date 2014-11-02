// Dijkstra4.cpp ---

// Code:

// We model and verify the 4-state self-stabilizing protocol presented in Dijkstra's
// original paper on the topic.

// TODO. Specify initial states of the system.

#include <boost/lexical_cast.hpp>

#include "../../src/uflts/LabelledTS.hpp"
#include "../../src/uflts/LTSEFSM.hpp"
#include "../../src/uflts/LTSChannelEFSM.hpp"
#include "../../src/uflts/LTSAssign.hpp"
#include "../../src/uflts/LTSTransitions.hpp"
#include "../../src/mc/LTSChecker.hpp"
#include "../../src/mc/OmegaAutomaton.hpp"

using boost::lexical_cast;
using namespace ESMC;
using namespace LTS;
using namespace Exprs;
using namespace MC;

#define __LOGSTR__ string(__FILE__) + ", " + to_string(__LINE__) + ": "

const size_t NumProcesses = 3;

// Messages
vector<ExprTypeRef> WriteMsgs;

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
    
    TheLTS->FreezeMsgs();
    cout << __LOGSTR__ << "Declaring messages done." << endl;
}

// Processes
vector<GeneralEFSM*> Proc;
GeneralEFSM* ShadowMonitor;

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
    Proc[i]->AddVariable(string("Up") + to_string(i - 1), TheLTS->MakeBoolType());
    Proc[i]->AddVariable(string("Up") + to_string(i), TheLTS->MakeBoolType());
    Proc[i]->FreezeVars();

    cout << __LOGSTR__ << "Adding transitions." << endl;

    Proc[i]->AddInputMsg(WriteMsgs[i - 1], {});

    auto DiExp = TheLTS->MakeVar(string("Data") + to_string(i), TheLTS->MakeBoolType());
    auto Dim1Exp = TheLTS->MakeVar("Data" + to_string(i - 1), TheLTS->MakeBoolType());
    auto UiExp = TheLTS->MakeVar(string("Up") + to_string(i), TheLTS->MakeBoolType());
    auto Uim1Exp = TheLTS->MakeVar("Up" + to_string(i - 1), TheLTS->MakeBoolType());
    auto Rim1Exp = TheLTS->MakeVar("R" + to_string(i - 1), WriteMsgs[i - 1]);
    auto Dim1PayloadAccField = TheLTS->MakeVar("Data", TheLTS->MakeFieldAccessType());
    auto Rim1DataFieldExp = TheLTS->MakeOp(LTSOps::OpField,
                                           Rim1Exp,
                                           Dim1PayloadAccField);
    auto Uim1PayloadAccField = TheLTS->MakeVar("Up", TheLTS->MakeFieldAccessType());
    auto Rim1UpFieldExp = TheLTS->MakeOp(LTSOps::OpField,
                                         Rim1Exp,
                                         Uim1PayloadAccField);
    vector<LTSAssignRef> Rim1Updates { new LTSAssignSimple(Dim1Exp, Rim1DataFieldExp),
                                       new LTSAssignSimple(Uim1Exp, Rim1UpFieldExp) };
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
    Proc[i]->AddOutputTransition("TheState", "TheState", DataiNeqDataim1,
                                 WiUpdates,
                                 "W" + to_string(i),
                                 WriteMsgs[i], {});

    cout << __LOGSTR__ << "Declaring process " << i << " done." << endl;
}

void DeclareShadowMonitor(LabelledTS* TheLTS)
{
    assert(TheLTS != nullptr && ShadowMonitor == nullptr);
    // TODO.
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
    TheLTS->FreezeAutomata();
    cout << __LOGSTR__ << "Freezing automata done." << endl;
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
    cout << __LOGSTR__ << "Build LTS Checker done." << endl;
    Checker->BuildAQS();
    cout << __LOGSTR__ << "Build AQS done." << endl;
    delete Checker;
    
    cout << __LOGSTR__ << "Return." << endl;
    return 0;
}

//
// Dijkstra4.cpp ends here
