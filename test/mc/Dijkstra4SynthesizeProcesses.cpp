// Dijkstra4SynthesizeProcesses.cpp ---
/*

The goal of this model is to synthesize the guards and updates of the processes in a Dijkstra model,
assuming that the notion of legitimacy is that of determinism.

Both mechanisms, bounds on number of legitimate states of getting "interesting" models are available

This Dijkstra model includes a ring monitor.
The ring monitor find executions with cycles on which a legitimate state is not visited.
Synthesizing against that monitor results in legitimate states that fall on a ring.

Compared to Dijkstra4Synth, this computes a non-trivial legitimate predicate without requiring
bounds on the number of legitimate states.
On the other hand, it results in a state blow up (with 5 processes, it should take between one
and two minutes) since the ring monitor non-deterministally snapshots the values of the state
variables at a legitimate state.



Unveiling a guard op does the following things:
Asserts symmetry constraints for guard
Asserts determinism constraints for guard
Asserts symmetry constraints on the updates
Marks them as unveiled
Creates guard indicator
Creates update indicator
Adds guard op and the corresponding updates as interpreted and updates the model.

The last two might be good to have for synthesizing processes



*/
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
// for number of processes = 5, the minimum number of legitimate states is 16.
const size_t NumProcesses = 3;

vector<int> ProcessesToSynthesize = {0, 1, 2};

// Messages
vector<ExprTypeRef> WriteMsgs;

vector<ExpT> Guards;
ExpT Legitimacy;
ExpT Prop3;

vector<i64> GuardOps;
vector<i64> UpdateOps;

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
GeneralEFSM* ShadowMonitor;
GeneralEFSM* RingMonitor;


void DeclareProc0(LabelledTS* TheLTS)
{
    assert(TheLTS != nullptr && Proc.size() == 0);
    cout << __LOGSTR__ << "Declaring process 0." << endl;
    bool SynthesizeProcess = (find(ProcessesToSynthesize.begin(), ProcessesToSynthesize.end(), 0) !=
                              ProcessesToSynthesize.end());
    auto Mgr = TheLTS->MakeTrue()->GetMgr();
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

    vector<LTSAssignRef> W0Updates;
    ExpT GuardExp;
    auto D0PayloadAccField = TheLTS->MakeVar("Data", TheLTS->MakeFieldAccessType());
    auto U0PayloadAccField = TheLTS->MakeVar("Up", TheLTS->MakeFieldAccessType());
    auto W0DataFieldExp = TheLTS->MakeOp(LTSOps::OpField,
                                         W0Exp,
                                         D0PayloadAccField);
    auto W0UpFieldExp = TheLTS->MakeOp(LTSOps::OpField,
                                       W0Exp,
                                       U0PayloadAccField);
    if (SynthesizeProcess) {
        auto Args = {D0Exp, D1Exp, U0Exp, U1Exp};
        vector<ExprTypeRef> ArgTypes;
        transform(Args.begin(), Args.end(), back_inserter(ArgTypes),
                 [&](ExpT Arg) {return Arg->GetType();});
        auto GuardOp = Mgr->MakeUninterpretedFunction("Guard_" + ProcName,
                                                      ArgTypes,
                                                      TheLTS->MakeBoolType());
        GuardOps.push_back(GuardOp);
        GuardExp = TheLTS->MakeOp(GuardOp, Args);
        auto UpdateD0Op = Mgr->MakeUninterpretedFunction("Update_D0_" + ProcName,
                                                         ArgTypes,
                                                         TheLTS->MakeBoolType());
        UpdateOps.push_back(UpdateD0Op);
        auto UpdateD0Exp = TheLTS->MakeOp(UpdateD0Op, Args);
        // update U0?
        // auto UpdateU0Op = Mgr->MakeUninterpretedFunction("Update_U0_" + ProcName,
        //                                                  ArgTypes,
        //                                                  TheLTS->MakeBoolType());
        // UpdateOps.push_back(UpdateU0Op);
        // auto UpdateU0Exp = TheLTS->MakeOp(UpdateU0Op, Args);
        cout << GuardExp << endl;
        cout << UpdateD0Exp << endl;
        // cout << UpdateU0Exp << endl;
        // Update U0?
        // W0Updates.insert(W0Updates.end(),{ new LTSAssignSimple(D0Exp, UpdateD0Exp),
        //                                    new LTSAssignSimple(U0Exp, UpdateU0Exp),
        //                                    new LTSAssignSimple(W0DataFieldExp, UpdateD0Exp),
        //                                    new LTSAssignSimple(W0UpFieldExp, UpdateU0Exp) });

        W0Updates.insert(W0Updates.end(),{ new LTSAssignSimple(D0Exp, UpdateD0Exp),
                    new LTSAssignSimple(W0DataFieldExp, UpdateD0Exp),
                    new LTSAssignSimple(W0UpFieldExp, U0Exp) });

    } else {
        auto NotD0Exp = TheLTS->MakeOp(LTSOps::OpNOT, D0Exp);
        auto Data0EqData1 = TheLTS->MakeOp(LTSOps::OpEQ, D0Exp, D1Exp);
        auto NotU1Exp = TheLTS->MakeOp(LTSOps::OpNOT, U1Exp);
        GuardExp = TheLTS->MakeOp(LTSOps::OpAND, Data0EqData1, NotU1Exp);

        W0Updates.insert(W0Updates.end(),{ new LTSAssignSimple(D0Exp, NotD0Exp),
                                           new LTSAssignSimple(U0Exp, U0Exp),
                                           new LTSAssignSimple(W0DataFieldExp, NotD0Exp),
                                           new LTSAssignSimple(W0UpFieldExp, U0Exp) });
    }
    Guards.push_back(GuardExp);
    Proc[0]->AddOutputTransition("TheState", "TheState", GuardExp,
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
    auto Mgr = TheLTS->MakeTrue()->GetMgr();
    bool SynthesizeProcess = (find(ProcessesToSynthesize.begin(), ProcessesToSynthesize.end(), i) !=
                              ProcessesToSynthesize.end());

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
    auto Dip1Exp = TheLTS->MakeVar(string("Data") + to_string(i + 1), TheLTS->MakeBoolType());
    auto UiExp = TheLTS->MakeVar(string("Up") + to_string(i), TheLTS->MakeBoolType());
    auto Uip1Exp = TheLTS->MakeVar(string("Up") + to_string(i + 1), TheLTS->MakeBoolType());

    auto DataiEqDataim1 = TheLTS->MakeOp(LTSOps::OpEQ, DiExp, Dim1Exp);
    auto DiPayloadAccField = TheLTS->MakeVar("Data", TheLTS->MakeFieldAccessType());
    auto UiPayloadAccField = TheLTS->MakeVar("Up", TheLTS->MakeFieldAccessType());
    auto WiDataFieldExp = TheLTS->MakeOp(LTSOps::OpField,
                                         WiExp,
                                         DiPayloadAccField);
    auto WiUpFieldExp = TheLTS->MakeOp(LTSOps::OpField,
                                       WiExp,
                                       UiPayloadAccField);

    vector<LTSAssignRef> W1iUpdates;

    ExpT GuardExp;

    if (SynthesizeProcess) {
        auto Args = {Dim1Exp, DiExp, Dip1Exp, UiExp, Uip1Exp};
        vector<ExprTypeRef> ArgTypes;
        transform(Args.begin(), Args.end(), back_inserter(ArgTypes),
                  [&](ExpT Arg) {return Arg->GetType();});
        auto GuardOp = Mgr->MakeUninterpretedFunction("Guard1_" + ProcName,
                                                      ArgTypes,
                                                      TheLTS->MakeBoolType());
        GuardOps.push_back(GuardOp);
        GuardExp = TheLTS->MakeOp(GuardOp, Args);
        auto UpdateDiOp = Mgr->MakeUninterpretedFunction("Update1_Di_" + ProcName,
                                                         ArgTypes,
                                                         TheLTS->MakeBoolType());
        UpdateOps.push_back(UpdateDiOp);
        auto UpdateDiExp = TheLTS->MakeOp(UpdateDiOp, Args);
        auto UpdateUiOp = Mgr->MakeUninterpretedFunction("Update1_Ui_" + ProcName,
                                                         ArgTypes,
                                                         TheLTS->MakeBoolType());
        UpdateOps.push_back(UpdateUiOp);
        auto UpdateUiExp = TheLTS->MakeOp(UpdateUiOp, Args);
        cout << GuardExp << endl;
        cout << UpdateDiExp << endl;
        cout << UpdateUiExp << endl;
        W1iUpdates.insert(W1iUpdates.end(), { new LTSAssignSimple(DiExp, UpdateDiExp),
                                              new LTSAssignSimple(UiExp, UpdateUiExp),
                                              new LTSAssignSimple(WiDataFieldExp, UpdateDiExp),
                                              new LTSAssignSimple(WiUpFieldExp, UpdateUiExp) });
    } else {
        W1iUpdates.insert(W1iUpdates.end(),
                          { new LTSAssignSimple(DiExp, TheLTS->MakeOp(LTSOps::OpNOT, DiExp)),
                            new LTSAssignSimple(UiExp, TheLTS->MakeTrue()),
                            new LTSAssignSimple(WiDataFieldExp, TheLTS->MakeOp(LTSOps::OpNOT, DiExp)),
                            new LTSAssignSimple(WiUpFieldExp, TheLTS->MakeTrue()) });
        auto DataiNeqDataim1 = TheLTS->MakeOp(LTSOps::OpNOT, DataiEqDataim1);
        GuardExp = DataiNeqDataim1;
    }
    Guards.push_back(GuardExp);
    Proc[i]->AddOutputTransition("TheState", "TheState", GuardExp,
                                 W1iUpdates,
                                 "W" + to_string(i),
                                 WriteMsgs[i], {});

    // If DataS = DataR and UpS and !UpR, then UpS := false.
    // The following fails for 4 processes, not for 3, becaues Uip1 is fixed to False
    // auto Guard = TheLTS->MakeOp(LTSOps::OpAND, { DataiEqDataip1, UiExp });

    // The following fails for 3 processes
    // auto Guard = TheLTS->MakeOp(LTSOps::OpAND, { DataiEqDataip1, NotUip1Exp });

    vector<LTSAssignRef> W2iUpdates;

    if (SynthesizeProcess) {
        auto Args = {DiExp, Dip1Exp, UiExp, Uip1Exp};
        vector<ExprTypeRef> ArgTypes;
        transform(Args.begin(), Args.end(), back_inserter(ArgTypes),
                  [&](ExpT Arg) {return Arg->GetType();});
        auto GuardOp = Mgr->MakeUninterpretedFunction("Guard2_" + ProcName,
                                                      ArgTypes,
                                                      TheLTS->MakeBoolType());
        GuardOps.push_back(GuardOp);
        GuardExp = TheLTS->MakeOp(GuardOp, Args);
        auto UpdateDiOp = Mgr->MakeUninterpretedFunction("Update2_Di_" + ProcName,
                                                         ArgTypes,
                                                         TheLTS->MakeBoolType());
        UpdateOps.push_back(UpdateDiOp);
        auto UpdateDiExp = TheLTS->MakeOp(UpdateDiOp, Args);
        auto UpdateUiOp = Mgr->MakeUninterpretedFunction("Update2_Ui_" + ProcName,
                                                         ArgTypes,
                                                         TheLTS->MakeBoolType());
        UpdateOps.push_back(UpdateUiOp);
        auto UpdateUiExp = TheLTS->MakeOp(UpdateUiOp, Args);
        cout << GuardExp << endl;
        cout << UpdateDiExp << endl;
        cout << UpdateUiExp << endl;
        W2iUpdates.insert(W2iUpdates.end(),
                          { new LTSAssignSimple(DiExp, UpdateDiExp),
                            new LTSAssignSimple(UiExp, UpdateUiExp),
                                  new LTSAssignSimple(WiDataFieldExp, UpdateDiExp),
                                  new LTSAssignSimple(WiUpFieldExp, UpdateUiExp) });
    } else {
        auto DataiEqDataip1 = TheLTS->MakeOp(LTSOps::OpEQ, DiExp, Dip1Exp);
        auto NotUip1Exp = TheLTS->MakeOp(LTSOps::OpNOT, Uip1Exp);
        GuardExp = TheLTS->MakeOp(LTSOps::OpAND, { DataiEqDataip1, UiExp, NotUip1Exp });
        W2iUpdates.insert(W2iUpdates.begin(),
                          { new LTSAssignSimple(DiExp, DiExp),
                            new LTSAssignSimple(UiExp, TheLTS->MakeFalse()),
                            new LTSAssignSimple(WiDataFieldExp, DiExp),
                            new LTSAssignSimple(WiUpFieldExp, TheLTS->MakeFalse()) });
    }
    Guards.push_back(GuardExp);
    Proc[i]->AddOutputTransition("TheState", "TheState", GuardExp,
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
    size_t i = NumProcesses - 1;
    cout << __LOGSTR__ << "Declaring process " << i << "." << endl;
    bool SynthesizeProcess = (find(ProcessesToSynthesize.begin(), ProcessesToSynthesize.end(), i) !=
                              ProcessesToSynthesize.end());
    auto Mgr = TheLTS->MakeTrue()->GetMgr();

    string ProcName = string("Proc") + to_string(i);

    Proc.push_back(TheLTS->MakeGenEFSM(ProcName, {}, TheLTS->MakeTrue(), LTSFairnessType::None));
    Proc[i]->AddState("TheState");
    Proc[i]->FreezeStates();

    cout << __LOGSTR__ << "Declaring variables." << endl;
    Proc[i]->AddVariable(string("Data") + to_string(i - 1), TheLTS->MakeBoolType());
    Proc[i]->AddVariable(string("Data") + to_string(i), TheLTS->MakeBoolType());
    Proc[i]->AddVariable(string("Up") + to_string(i), TheLTS->MakeBoolType());
    // Proc[i]->AddVariable(string("Up") + to_string(i - 1), TheLTS->MakeBoolType());

    Proc[i]->FreezeVars();

    cout << __LOGSTR__ << "Adding transitions." << endl;

    Proc[i]->AddInputMsg(WriteMsgs[i - 1], {});

    auto DiExp = TheLTS->MakeVar(string("Data") + to_string(i), TheLTS->MakeBoolType());
    auto Dim1Exp = TheLTS->MakeVar("Data" + to_string(i - 1), TheLTS->MakeBoolType());
    auto UiExp = TheLTS->MakeVar(string("Up") + to_string(i), TheLTS->MakeBoolType());
    // auto Uim1Exp = TheLTS->MakeVar(string("Up") + to_string(i - 1), TheLTS->MakeBoolType());
    auto Rim1Exp = TheLTS->MakeVar("R" + to_string(i - 1), WriteMsgs[i - 1]);
    auto Dim1PayloadAccField = TheLTS->MakeVar("Data", TheLTS->MakeFieldAccessType());
    // auto Uim1PayloadAccField = TheLTS->MakeVar("Up", TheLTS->MakeFieldAccessType());
    // auto Rim1UpFieldExp = TheLTS->MakeOp(LTSOps::OpField,
    //                                      Rim1Exp,
    //                                      Uim1PayloadAccField);
    auto Rim1DataFieldExp = TheLTS->MakeOp(LTSOps::OpField,
                                           Rim1Exp,
                                           Dim1PayloadAccField);

    vector<LTSAssignRef> Rim1Updates { new LTSAssignSimple(Dim1Exp, Rim1DataFieldExp) };
                                       // new LTSAssignSimple(Uim1Exp, Rim1UpFieldExp) };

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
    vector<LTSAssignRef> WiUpdates;
    ExpT GuardExp;
    if (SynthesizeProcess) {
        // auto Args = {Dim1Exp, DiExp, Uim1Exp, UiExp};
        auto Args = {Dim1Exp, DiExp, UiExp};
        vector<ExprTypeRef> ArgTypes;
        transform(Args.begin(), Args.end(), back_inserter(ArgTypes),
                 [&](ExpT Arg) {return Arg->GetType();});
        auto GuardOp = Mgr->MakeUninterpretedFunction("Guard_" + ProcName,
                                                      ArgTypes,
                                                      TheLTS->MakeBoolType());
        GuardOps.push_back(GuardOp);
        GuardExp = TheLTS->MakeOp(GuardOp, Args);
        auto UpdateDiOp = Mgr->MakeUninterpretedFunction("Update_D" + to_string(i) +"_" + ProcName,
                                                         ArgTypes,
                                                         TheLTS->MakeBoolType());
        UpdateOps.push_back(UpdateDiOp);
        auto UpdateDiExp = TheLTS->MakeOp(UpdateDiOp, Args);
        // auto UpdateUiOp = Mgr->MakeUninterpretedFunction("Update_U" + to_string(i) + "_" + ProcName,
        //                                                  ArgTypes,
        //                                                  TheLTS->MakeBoolType());
        // UpdateOps.push_back(UpdateUiOp);
        // auto UpdateUiExp = TheLTS->MakeOp(UpdateUiOp, Args);
        cout << GuardExp << endl;
        cout << UpdateDiExp << endl;
        // cout << UpdateUiExp << endl;
        WiUpdates.insert(WiUpdates.end(),{ new LTSAssignSimple(DiExp, UpdateDiExp),
                                           // new LTSAssignSimple(UiExp, UpdateUiExp),
                                           new LTSAssignSimple(WiDataFieldExp, UpdateDiExp),
                                           new LTSAssignSimple(WiUpFieldExp, UiExp) });
    } else {
        WiUpdates.insert(WiUpdates.end(), { new LTSAssignSimple(DiExp, TheLTS->MakeOp(LTSOps::OpNOT, DiExp)),
                                            new LTSAssignSimple(UiExp, UiExp),
                                            new LTSAssignSimple(WiDataFieldExp, TheLTS->MakeOp(LTSOps::OpNOT, DiExp)),
                                            new LTSAssignSimple(WiUpFieldExp, UiExp) });
        GuardExp = DataiNeqDataim1;
    }
    Guards.push_back(GuardExp);
    Proc[i]->AddOutputTransition("TheState", "TheState", GuardExp,
                                 WiUpdates,
                                 "W" + to_string(i),
                                 WriteMsgs[i], {});

    cout << __LOGSTR__ << "Declaring process " << i << " done." << endl;
}

void DeclareShadowMonitor(LabelledTS* TheLTS)
{
    assert(TheLTS != nullptr && ShadowMonitor == nullptr);

    ShadowMonitor = TheLTS->MakeGenEFSM("ShadowMonitor", {},
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
    auto Phi = TheLTS->MakeFalse();
    for (size_t i = 0; i < Guards.size(); ++i) {
        // Create conjunction of the following:
        // i th Guard true, all others false
        auto Conjunction = TheLTS->MakeTrue();
        for (size_t j = 0; j < Guards.size(); ++j) {
            if (j != i) {
                auto NegGuardj = TheLTS->MakeOp(LTSOps::OpNOT, Guards[j]);
                Conjunction = TheLTS->MakeOp(LTSOps::OpAND, Conjunction, NegGuardj);
            } else {
                Conjunction = TheLTS->MakeOp(LTSOps::OpAND, Conjunction, Guards[i]);
            }
        }
        Phi = TheLTS->MakeOp(LTSOps::OpOR, Phi, Conjunction);
    }
    cout << Phi << endl;

    Legitimacy = Phi;

    ShadowMonitor->AddOutputTransition("Transient", "Legitimate",
                                       Phi, {},
                                       "Legitimate", LegitimateAnnouncement,
                                       {}, set<string>());
    auto NotPhi = TheLTS->MakeOp(LTSOps::OpNOT, Phi);
    ShadowMonitor->AddOutputTransition("Transient", "Illegitimate",
                                       NotPhi, {},
                                       "Illegitimate", IllegitimateAnnouncement,
                                       {}, set<string>());
    cout << "Legitimacy guard" << endl;
    cout << Phi << endl;
}

void DeclareRingMonitor(LabelledTS* TheLTS)
{
    assert(TheLTS != nullptr && RingMonitor == nullptr);

    RingMonitor = TheLTS->MakeGenEFSM("RingMonitor", {},
                                      TheLTS->MakeTrue(), LTSFairnessType::None);

    RingMonitor->AddState("Initial");
    RingMonitor->AddState("Snapshot");
    RingMonitor->AddState("Last");
    RingMonitor->FreezeStates();
    for (size_t i = 0; i < NumProcesses; ++i) {
        RingMonitor->AddInputMsg(WriteMsgs[i], {});
        RingMonitor->AddVariable("Data" + to_string(i), TheLTS->MakeBoolType());
        RingMonitor->AddVariable("Up" + to_string(i), TheLTS->MakeBoolType());
        RingMonitor->AddVariable("SnapshotData" + to_string(i), TheLTS->MakeBoolType());
        RingMonitor->AddVariable("SnapshotUp" + to_string(i), TheLTS->MakeBoolType());
    }
    auto LegitimateAnnouncementDeclaration = RingMonitor->AddInputMsg(LegitimateAnnouncement, {});

    RingMonitor->FreezeVars();

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
        RingMonitor->AddInputTransition("Initial", "Initial",
                                        TheLTS->MakeTrue(), Updates,
                                        "Write", WriteMsgs[i], {});
        RingMonitor->AddInputTransition("Snapshot", "Snapshot",
                                        TheLTS->MakeTrue(), Updates,
                                        "Write", WriteMsgs[i], {});
        RingMonitor->AddInputTransition("Last", "Last",
                                        TheLTS->MakeTrue(), Updates,
                                        "Write", WriteMsgs[i], {});

    }
    RingMonitor->AddInputTransition("Initial", "Initial",
                                    TheLTS->MakeTrue(), {},
                                    "Snapshot", LegitimateAnnouncement, {});
    vector<ExpT> SnapshotNEQStateDisjuncts;
    vector<ExpT> SnapshotEQStateConjuncts;
    vector<LTSAssignRef> Updates;
    for (u32 i = 0; i < NumProcesses; ++i) {
        auto Datai = TheLTS->MakeVar("Data" + to_string(i), TheLTS->MakeBoolType());
        auto Upi = TheLTS->MakeVar("Up" + to_string(i), TheLTS->MakeBoolType());
        auto SnapshotDatai = TheLTS->MakeVar("SnapshotData" + to_string(i), TheLTS->MakeBoolType());
        auto SnapshotUpi = TheLTS->MakeVar("SnapshotUp" + to_string(i), TheLTS->MakeBoolType());
        Updates.push_back(new LTSAssignSimple(SnapshotDatai, Datai));
        Updates.push_back(new LTSAssignSimple(SnapshotUpi, Upi));
        auto DataSnapshotEQState_i = TheLTS->MakeOp(LTSOps::OpEQ, SnapshotDatai, Datai);
        auto UpSnapshotEQState_i = TheLTS->MakeOp(LTSOps::OpEQ, SnapshotUpi, Upi);
        auto SnapshotEQState_i = TheLTS->MakeOp(LTSOps::OpAND, DataSnapshotEQState_i, UpSnapshotEQState_i);
        auto SnapshotNEQState_i = TheLTS->MakeOp(LTSOps::OpNOT, SnapshotEQState_i);
        SnapshotEQStateConjuncts.push_back(SnapshotEQState_i);
        SnapshotNEQStateDisjuncts.push_back(SnapshotNEQState_i);
    }
    auto SnapshotEQState = TheLTS->MakeOp(LTSOps::OpAND, SnapshotEQStateConjuncts);
    auto SnapshotNEQState = TheLTS->MakeOp(LTSOps::OpOR, SnapshotNEQStateDisjuncts);

    RingMonitor->AddInputTransition("Initial", "Snapshot",
                                    TheLTS->MakeTrue(), Updates,
                                    "Snapshot", LegitimateAnnouncement, {});
    RingMonitor->AddInputTransition("Snapshot", "Snapshot",
                                    SnapshotNEQState, {},
                                    "Ignore", LegitimateAnnouncement, {});
    RingMonitor->AddInputTransition("Snapshot", "Last",
                                    SnapshotEQState, {},
                                    "Success", LegitimateAnnouncement, {});
    RingMonitor->AddInputTransition("Last", "Last",
                                    TheLTS->MakeTrue(), {},
                                    "Ignore", LegitimateAnnouncement, {});
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

void InitializeRingMonitor(LabelledTS* TheLTS, vector<LTSAssignRef>& InitUpdates,
                           const vector<ExpT>& DataVal, const vector<ExpT>& UpVal)
{
    auto FAType = TheLTS->MakeFieldAccessType();
    // Initialize Ring Monitor
    auto RingMonitorType = TheLTS->GetEFSMType("RingMonitor");
    auto RingMonitorStateVar = TheLTS->MakeVar("RingMonitor", RingMonitorType);
    auto RingMonitorState = TheLTS->MakeOp(LTSOps::OpField, RingMonitorStateVar, TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()));
    auto RingMonitorInitialState = TheLTS->MakeVal("Initial", RingMonitorState->GetType());
    InitUpdates.push_back(new LTSAssignSimple(RingMonitorState, RingMonitorInitialState));
    for (size_t i = 0; i < NumProcesses; ++i) {
        auto RingDatai = TheLTS->MakeOp(LTSOps::OpField,
                                        RingMonitorStateVar,
                                        TheLTS->MakeVar("Data" + to_string(i), FAType));
        auto RingUpi = TheLTS->MakeOp(LTSOps::OpField,
                                      RingMonitorStateVar,
                                      TheLTS->MakeVar("Up" + to_string(i), FAType));
        auto RingSnapshotDatai = TheLTS->MakeOp(LTSOps::OpField,
                                                RingMonitorStateVar,
                                                TheLTS->MakeVar("SnapshotData" + to_string(i), FAType));
        auto RingSnapshotUpi = TheLTS->MakeOp(LTSOps::OpField,
                                              RingMonitorStateVar,
                                              TheLTS->MakeVar("SnapshotUp" + to_string(i), FAType));

        InitUpdates.push_back(new LTSAssignSimple(RingDatai, DataVal[i]));
        InitUpdates.push_back(new LTSAssignSimple(RingUpi, UpVal[i]));
        InitUpdates.push_back(new LTSAssignSimple(RingSnapshotDatai, DataVal[i]));
        InitUpdates.push_back(new LTSAssignSimple(RingSnapshotUpi, UpVal[i]));
    }
}

void DeclareAutomata(LabelledTS* TheLTS)
{
    assert(TheLTS != nullptr);
    cout << __LOGSTR__ << "Declaring automata." << endl;
    DeclareProc0(TheLTS);
    for (size_t i = 1; i < NumProcesses - 1; i++) {
        DeclareProcMid(TheLTS, i);
    }
    DeclareProcN(TheLTS);
    DeclareShadowMonitor(TheLTS);
    DeclareSafetyConcreteMonitor(TheLTS);
    DeclareRingMonitor(TheLTS);
    TheLTS->FreezeAutomata();
    auto FAType = TheLTS->MakeFieldAccessType();

    cout << __LOGSTR__ << "Freezing automata done." << endl;

    // Initial Data and Up variables for all processes

    vector<InitStateRef> InitStates;

    vector<vector<ExpT>> DataElems;
    for (size_t i = 0; i < NumProcesses; ++i) {
        DataElems.push_back({TheLTS->MakeTrue(), TheLTS->MakeFalse()});
    }
    // vector<vector<ExpT>> UpElems;
    // for (size_t i = 0; i < NumProcesses; ++i) {
    //     UpElems.push_back({TheLTS->MakeTrue(), TheLTS->MakeFalse()});
    // }

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

            InitializeRingMonitor(TheLTS, InitUpdates, DataVal, UpVal);

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
    cout << "The guards are" << endl;
    for (auto Guard : Guards) {
        cout << Guard << endl;
    }
    auto Mgr = TheLTS->MakeTrue()->GetMgr();
    vector<ExpT> Prop3GuardConjuncts;
    for (auto Guard: Guards) {
        vector<ExpT> GuardDisjuncts;
        for (auto DataVal : DataCombinations) {
            vector<vector<ExpT>> UpElems;
            for (size_t i = 1; i + 1 < NumProcesses; ++i) {
                UpElems.push_back({TheLTS->MakeTrue(), TheLTS->MakeFalse()});
            }
            auto&& UpCombinations = CrossProduct<ExpT>(UpElems.begin(), UpElems.end());
            for (auto UpVal : UpCombinations) {
                UpVal.insert(UpVal.begin(), TheLTS->MakeTrue());
                UpVal.push_back(TheLTS->MakeFalse());

                MgrT::SubstMapT Valuation;
                for (u32 i = 0; i < DataVal.size(); ++i) {
                    Valuation[TheLTS->MakeVar("Data" + to_string(i), TheLTS->MakeBoolType())] = DataVal[i];
                }
                for (u32 i = 0; i < UpVal.size(); ++i) {
                    Valuation[TheLTS->MakeVar("Up" + to_string(i), TheLTS->MakeBoolType())] = UpVal[i];
                }
                auto StateLegitimate = Mgr->TermSubstitute(Valuation, Legitimacy);
                auto GuardEnabled = Mgr->TermSubstitute(Valuation, Guard);
                GuardDisjuncts.push_back(TheLTS->MakeOp(LTSOps::OpAND, StateLegitimate, GuardEnabled));
            }
        }
        Prop3GuardConjuncts.push_back(TheLTS->MakeOp(LTSOps::OpOR, GuardDisjuncts));
    }
    Prop3 = TheLTS->MakeOp(LTSOps::OpAND, Prop3GuardConjuncts);

    cout << __LOGSTR__ << "Init states size " << InitStates.size() << endl;
    TheLTS->AddInitStates(InitStates);
}

void DeclareRingLivenessMonitor(LabelledTS* TheLTS, LTSChecker* Checker)
{
    auto Monitor = Checker->MakeStateBuchiMonitor("Ring",
                                                  {}, TheLTS->MakeTrue());
    Monitor->AddState("Initial", true, false);
    Monitor->AddState("SnapshotTaken", false, true);
    Monitor->AddState("SnapshotSeenAgain", false, false);
    Monitor->FreezeStates();

    auto RingMonitorType = TheLTS->GetEFSMType("RingMonitor");
    auto RingMonitorStateVar = TheLTS->MakeVar("RingMonitor", RingMonitorType);
    auto RingMonitorState = TheLTS->MakeOp(LTSOps::OpField, RingMonitorStateVar, TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()));
    auto RingMonitorSnapshotState = TheLTS->MakeVal("Snapshot", RingMonitorState->GetType());
    auto RingMonitorStateEQSnapshotState = TheLTS->MakeOp(LTSOps::OpEQ, RingMonitorState, RingMonitorSnapshotState);
    auto RingMonitorStateNEQSnapshotState = TheLTS->MakeOp(LTSOps::OpNOT, RingMonitorStateEQSnapshotState);

    Monitor->AddTransition("Initial", "SnapshotTaken", RingMonitorStateEQSnapshotState);
    Monitor->AddTransition("Initial", "Initial", RingMonitorStateNEQSnapshotState);
    Monitor->AddTransition("SnapshotTaken", "SnapshotTaken", RingMonitorStateEQSnapshotState);
    Monitor->AddTransition("SnapshotTaken", "SnapshotSeenAgain", RingMonitorStateNEQSnapshotState);
    Monitor->AddTransition("SnapshotSeenAgain", "SnapshotSeenAgain", TheLTS->MakeTrue());
    Monitor->Freeze();
}


void DeclareLegitLivenessMonitor(LabelledTS* TheLTS, LTSChecker* Checker)
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


ExpT LegitimateStatesBoundConstraint(LabelledTS* TheLTS,
                                     vector<ExpT>& IndicatorConstraints,
                                     u32 UpperBoundOfLegimateStates)
{
    auto Mgr = TheLTS->MakeTrue()->GetMgr();
    vector<vector<ExpT>> DataElems;
    for (size_t i = 0; i < NumProcesses; ++i) {
        DataElems.push_back({TheLTS->MakeTrue(), TheLTS->MakeFalse()});
    }

    auto&& DataCombinations = CrossProduct<ExpT>(DataElems.begin(), DataElems.end());
    u32 StateCounter = 1;
    vector<ExpT> Indicators;
    for (auto DataVal : DataCombinations) {
        vector<vector<ExpT>> UpElems;
        for (size_t i = 1; i + 1 < NumProcesses; ++i) {
            UpElems.push_back({TheLTS->MakeTrue(), TheLTS->MakeFalse()});
        }
        auto&& UpCombinations = CrossProduct<ExpT>(UpElems.begin(), UpElems.end());
        for (auto UpVal : UpCombinations) {
            UpVal.insert(UpVal.begin(), TheLTS->MakeTrue());
            UpVal.push_back(TheLTS->MakeFalse());

            MgrT::SubstMapT Valuation;
            for (u32 i = 0; i < DataVal.size(); ++i) {
                Valuation[TheLTS->MakeVar("Data" + to_string(i), TheLTS->MakeBoolType())] = DataVal[i];
            }
            for (u32 i = 0; i < UpVal.size(); ++i) {
                Valuation[TheLTS->MakeVar("Up" + to_string(i), TheLTS->MakeBoolType())] = UpVal[i];
            }
            auto IsStateLegitimate = Mgr->Substitute(Valuation, Legitimacy);
            auto ZeroOneType = TheLTS->MakeRangeType(0, 1);
            auto IndicatorVariable = TheLTS->MakeVar("LegitimateIndicator" + to_string(StateCounter),
                                                     ZeroOneType);
            StateCounter++;
            auto IndicatorVariableEQOne = TheLTS->MakeOp(LTSOps::OpEQ, IndicatorVariable,
                                                         TheLTS->MakeVal("1", ZeroOneType));
            Indicators.push_back(IndicatorVariable);
            auto LegitimacyToIndicator = TheLTS->MakeOp(LTSOps::OpIMPLIES,
                                                        IsStateLegitimate,
                                                        IndicatorVariableEQOne);
            IndicatorConstraints.push_back(LegitimacyToIndicator);
        }
    }
    auto IndicatorSum = TheLTS->MakeOp(LTSOps::OpADD, Indicators);
    auto NumIndicatorsType = TheLTS->MakeRangeType(0, Indicators.size());
    ExpT NumLegitimateStatesExp = TheLTS->MakeVal(to_string(UpperBoundOfLegimateStates),
                                                  NumIndicatorsType);
    return TheLTS->MakeOp(LTSOps::OpLE,
                          IndicatorSum,
                          NumLegitimateStatesExp);
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

    DeclareLegitLivenessMonitor(TheLTS, Checker);
    DeclareRingLivenessMonitor(TheLTS, Checker);

    auto TheSolver = new Solver(Checker);

    for (i64 Op : GuardOps) {
        TheSolver->UnveilNonCompletionGuardOp(Op);
    }
    for (i64 Op : UpdateOps) {
        TheSolver->UnveilNonCompletionOp(Op);
    }
    TheSolver->MakeAssertion(Prop3);

    vector<ExpT> IndicatorConstraints;
    auto BoundConstraint = LegitimateStatesBoundConstraint(TheLTS,
                                                           IndicatorConstraints,
                                                           8);
    for (ExpT IndicatorConstraint : IndicatorConstraints) {
        TheSolver->MakeAssertion(IndicatorConstraint);
    }
    TheSolver->MakeAssertion(BoundConstraint);

    TheSolver->Solve();

    vector<vector<ExpT>> DataElems;
    for (size_t i = 0; i < NumProcesses; ++i) {
        DataElems.push_back({TheLTS->MakeTrue(), TheLTS->MakeFalse()});
    }

    auto Mgr = TheLTS->MakeTrue()->GetMgr();
    u32 NumberOfLegitimateStates = 0;
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

            MgrT::SubstMapT Valuation;
            for (u32 i = 0; i < DataVal.size(); ++i) {
                Valuation[TheLTS->MakeVar("Data" + to_string(i), TheLTS->MakeBoolType())] = DataVal[i];
            }
            for (u32 i = 0; i < UpVal.size(); ++i) {
                Valuation[TheLTS->MakeVar("Up" + to_string(i), TheLTS->MakeBoolType())] = UpVal[i];
            }
            // for (u32 i = 0; i < DataVal.size(); ++i) {
            //     cout << DataVal[i] << " ";
            // }
            // for (u32 i = 0; i < UpVal.size(); ++i) {
            //     cout << UpVal[i] << " ";
            // }
            auto Result = TheSolver->Evaluate(Mgr->Substitute(Valuation, Legitimacy));
            // cout << ": " << Result << endl;
            if (Result == TheLTS->MakeTrue()) {
                NumberOfLegitimateStates++;
            }
        }
    }

    cout << "# of legitimate states: " << NumberOfLegitimateStates << endl;
    // DeclareRingMonitor(TheLTS);
    // DeclareRingLivenessMonitor(TheLTS, Checker);

    // Checker->CheckLiveness("Ring");
    // CheckRingProperty();

    delete Checker;

    cout << __LOGSTR__ << "Return." << endl;
    return 0;
}

//
// Dijkstra4Ring.cpp ends here
