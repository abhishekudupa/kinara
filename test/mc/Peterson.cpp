// Peterson.cpp ---
//
// Filename: Peterson.cpp
// Author: Abhishek Udupa
// Created: Wed Sep. 24 10:51:04 2014 (-0400)
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

// Peterson's mutual exclusion algorithm

#include "../../src/uflts/LabelledTS.hpp"
#include "../../src/uflts/LTSEFSM.hpp"
#include "../../src/uflts/LTSChannelEFSM.hpp"
#include "../../src/uflts/LTSAssign.hpp"
#include "../../src/uflts/LTSTransitions.hpp"
#include "../../src/mc/LTSChecker.hpp"
#include "../../src/mc/OmegaAutomaton.hpp"
#include "../../src/mc/Trace.hpp"
#include "../../src/utils/LogManager.hpp"

using namespace ESMC;
using namespace LTS;
using namespace Exprs;
using namespace MC;

int main()
{
    Logging::LogManager::Initialize();
    Logging::LogManager::EnableLogOption("Canonicalizer.Best");
    ESMC::Logging::LogManager::EnableLogOption("Checker.Fairness");
    ESMC::Logging::LogManager::EnableLogOption("Trace.Generation");
    ESMC::Logging::LogManager::EnableLogOption("Checker.AQSDetailed");

    auto TheLTS = new LabelledTS();

    auto TrueExp = TheLTS->MakeTrue();
    auto FalseExp = TheLTS->MakeFalse();
    set<string> EmptyStringSet;

    // Symmetric types
    auto PidType = TheLTS->MakeSymmType("PidType", 2);

    auto PidParam = TheLTS->MakeVar("Pid", PidType);
    auto PidParam1 = TheLTS->MakeVar("Pid1", PidType);
    auto PidParam2 = TheLTS->MakeVar("Pid2", PidType);

    auto FAType = TheLTS->MakeFieldAccessType();

    // Message types
    vector<pair<string, TypeRef>> MessageFields;
    MessageFields.push_back(make_pair("FlagData", TheLTS->MakeBoolType()));

    auto PidParamEQPidParam1 = TheLTS->MakeOp(LTSOps::OpEQ,
                                              PidParam,
                                              PidParam1);
    auto SetFlagType = TheLTS->MakeMsgTypes({PidParam, PidParam1}, PidParamEQPidParam1,
                                            "SetFlagType", MessageFields, false);
    MessageFields.clear();

    auto SetTurnType = TheLTS->MakeMsgTypes({PidParam},
                                            TrueExp, "SetTurnType", {}, false);

    auto AlphaType = TheLTS->MakeMsgTypes({PidParam},
                                          TrueExp, "AlphaType", {}, false);

    TheLTS->FreezeMsgs();

    auto ProcessEFSM = TheLTS->MakeGenEFSM("Process",
                                           {PidParam},
                                           TrueExp, LTSFairnessType::None);
    ProcessEFSM->AddState("L1");
    ProcessEFSM->AddState("L2");
    ProcessEFSM->AddState("L3");
    ProcessEFSM->AddState("Critical");
    ProcessEFSM->FreezeStates();

    auto FlagArrayType = TheLTS->MakeArrayType(PidType, TheLTS->MakeBoolType());

    ProcessEFSM->AddVariable("FlagArray", FlagArrayType);
    ProcessEFSM->AddVariable("Turn", PidType);

    ProcessEFSM->FreezeVars();

    auto PidParamNEQPidParam1 = TheLTS->MakeOp(LTSOps::OpNOT, TheLTS->MakeOp(LTSOps::OpEQ,
                                                                             PidParam,
                                                                             PidParam1));
    auto PidParam1EQPidParam2 = TheLTS->MakeOp(LTSOps::OpEQ,
                                                PidParam1,
                                                PidParam2);
    auto PNEQP1ANDP1EQP2 = TheLTS->MakeOp(LTSOps::OpAND, PidParamNEQPidParam1,
                                       PidParam1EQPidParam2);
    ProcessEFSM->AddInputMsgs({PidParam1, PidParam2}, PNEQP1ANDP1EQP2,
                              SetFlagType,
                              {PidParam1, PidParam2});
    ProcessEFSM->AddOutputMsgs({PidParam1}, PidParamEQPidParam1,
                              SetFlagType, {PidParam, PidParam1});

    ProcessEFSM->AddInputMsgs({PidParam1}, PidParamNEQPidParam1,
                              SetTurnType,
                              {PidParam1});
    ProcessEFSM->AddOutputMsg(SetTurnType, {PidParam});

    ProcessEFSM->AddInputMsg(AlphaType, {PidParam});

    // Transitions
    vector<LTSAssignRef> Updates;

    auto FlagArrayExp = TheLTS->MakeVar("FlagArray", FlagArrayType);
    auto FlagIndexExp = TheLTS->MakeOp(LTSOps::OpIndex, FlagArrayExp, PidParam2);

    auto SetFlagInMsgVar = TheLTS->MakeVar("InMsg", SetFlagType);
    auto SetFlagInDotData = TheLTS->MakeOp(LTSOps::OpField, SetFlagInMsgVar,
                                         TheLTS->MakeVar("FlagData", FAType));

    auto TurnExp = TheLTS->MakeVar("Turn", PidType);

    auto SetTurnMsgVar = TheLTS->MakeVar("InMsg", SetTurnType);
    // auto SetTurnDotData = TheLTS->MakeOp(LTSOps::OpField, SetTurnMsgVar,
    //                                      TheLTS->MakeVar("TurnData", FAType));

    // Broadcast shared memory self loops
    for (auto StateName : {"L1", "L2", "L3", "Critical"}) {
        Updates.push_back(new LTSAssignSimple(FlagIndexExp, SetFlagInDotData));
        ProcessEFSM->AddInputTransitions({ PidParam1, PidParam2 }, PNEQP1ANDP1EQP2,
                                         StateName, StateName, TrueExp, Updates,
                                         "InMsg", SetFlagType, {PidParam1, PidParam2});
        Updates.clear();
        Updates.push_back(new LTSAssignSimple(TurnExp, PidParam));
        ProcessEFSM->AddInputTransitions({ PidParam1 }, PidParamNEQPidParam1,
                                         StateName, StateName, TrueExp, Updates,
                                         "InMsg", SetTurnType, {PidParam1});
        Updates.clear();
    }

    auto FlagIndexPidParamExp = TheLTS->MakeOp(LTSOps::OpIndex, FlagArrayExp, PidParam);
    // setting my own flag variable
    Updates.push_back(new LTSAssignSimple(FlagIndexPidParamExp, TrueExp));
    // setting the payload
    auto SetFlagOutMsgVar = TheLTS->MakeVar("OutMsg", SetFlagType);
    auto SetFlagOutDotData = TheLTS->MakeOp(LTSOps::OpField, SetFlagOutMsgVar,
                                            TheLTS->MakeVar("FlagData", FAType));
    Updates.push_back(new LTSAssignSimple(SetFlagOutDotData, TrueExp));

    ProcessEFSM->AddOutputTransition("L1", "L2", TrueExp,
                                     Updates, "OutMsg", SetFlagType,
                                     {PidParam, PidParam});
    Updates.clear();

    // setting my own turn variable
    Updates.push_back(new LTSAssignSimple(TurnExp, PidParam1));
    ProcessEFSM->AddFairnessSet("TurnFairness", FairSetFairnessType::Strong,
                                {PidParam1}, TrueExp);
    ProcessEFSM->AddOutputTransitions({PidParam1}, PidParamNEQPidParam1,
                                      "L2", "L3", TrueExp,
                                      Updates, "OutMsg",
                                      SetTurnType, {PidParam},
                                      EmptyStringSet, {"TurnFairness"});
    Updates.clear();
    auto FlagIndexParam1Exp = TheLTS->MakeOp(LTSOps::OpIndex, FlagArrayExp, PidParam1);

    auto Guard = TheLTS->MakeOp(LTSOps::OpAND,
                                FlagIndexParam1Exp,
                                TheLTS->MakeOp(LTSOps::OpEQ, TurnExp, PidParam1));
    auto NotGuard = TheLTS->MakeOp(LTSOps::OpNOT, Guard);
    ProcessEFSM->AddInputTransitions({PidParam1}, PidParamNEQPidParam1,
                                     "L3", "Critical", NotGuard, {},
                                     "InMsg", AlphaType, {PidParam});

    ProcessEFSM->AddInputTransitions({PidParam1}, PidParamNEQPidParam1,
                                     "L3", "L3", Guard, {},
                                     "InMsg", AlphaType, {PidParam});

    // setting my own flag variable
    Updates.push_back(new LTSAssignSimple(FlagIndexPidParamExp, FalseExp));
    // setting the payload
    Updates.push_back(new LTSAssignSimple(SetFlagOutDotData, FalseExp));
    ProcessEFSM->AddFairnessSet("FlagFairnessCritical",
                                FairSetFairnessType::Strong);
    ProcessEFSM->AddOutputTransition("Critical", "L1", TrueExp,
                                     Updates, "OutMsg", SetFlagType,
                                     {PidParam, PidParam},
                                     {"FlagFairnessCritical"});
    Updates.clear();


    // Alpha Automaton
    auto AlphaEFSM = TheLTS->MakeGenEFSM("Alpha",
                                         {PidParam},
                                         TrueExp, LTSFairnessType::None); // <- should be something else
    AlphaEFSM->AddState("A0");
    AlphaEFSM->FreezeStates();
    AlphaEFSM->FreezeVars();
    AlphaEFSM->AddOutputMsg(AlphaType, {PidParam});

    AlphaEFSM->AddFairnessSet("AlphaFairness", FairSetFairnessType::Strong);
    AlphaEFSM->AddOutputTransition("A0", "A0", TrueExp,
                                   {}, "OutMsg", AlphaType, {PidParam},
                                   {"AlphaFairness"});

    TheLTS->FreezeAutomata();

    vector<InitStateRef> InitStates;
    vector<LTSAssignRef> InitUpdates;

    auto ProcessEFSMType = TheLTS->GetEFSMType("Process");
    auto AlphaEFSMType = TheLTS->GetEFSMType("Alpha");

    auto ProcessStateVar = TheLTS->MakeOp(LTSOps::OpIndex,
                                          TheLTS->MakeVar("Process", ProcessEFSMType),
                                          PidParam);

    auto ProcessDotLocation = TheLTS->MakeOp(LTSOps::OpField,
                                             ProcessStateVar,
                                             TheLTS->MakeVar("state", FAType));
    auto ProcessDotFlags = TheLTS->MakeOp(LTSOps::OpField,
                                          ProcessStateVar,
                                          TheLTS->MakeVar("FlagArray", FAType));
    auto ProcessDotTurn = TheLTS->MakeOp(LTSOps::OpField,
                                         ProcessStateVar,
                                         TheLTS->MakeVar("Turn", FAType));
    auto AlphaStateVar = TheLTS->MakeOp(LTSOps::OpIndex,
                                          TheLTS->MakeVar("Alpha", AlphaEFSMType),
                                          PidParam);
    auto AlphaDotLocation = TheLTS->MakeOp(LTSOps::OpField,
                                           AlphaStateVar,
                                           TheLTS->MakeVar("state", FAType));

    InitUpdates.push_back(new LTSAssignParam({ PidParam }, TrueExp,
                                             ProcessDotLocation,
                                             TheLTS->MakeVal("L1",
                                                             ProcessDotLocation->GetType())));
    InitUpdates.push_back(new LTSAssignParam({ PidParam }, TrueExp,
                                             TheLTS->MakeOp(LTSOps::OpIndex,
                                                            ProcessDotFlags,
                                                            PidParam),
                                             TheLTS->MakeFalse()));
    InitUpdates.push_back(new LTSAssignParam({ PidParam }, TrueExp,
                                             ProcessDotTurn,
                                              TheLTS->MakeVal("clear", PidType)));
    InitUpdates.push_back(new LTSAssignParam({ PidParam }, TrueExp,
                                             AlphaDotLocation,
                                             TheLTS->MakeVal("A0",
                                                             AlphaDotLocation->GetType())));
    InitStates.push_back(new LTSInitState({}, TrueExp, InitUpdates));

    TheLTS->AddInitStates(InitStates);


    // Invariant: not both in critical section
    // For all i, j : Pid . i != j -> not (Process[i].state = Critical /\ Process[j].state = Criitical)
    auto ProcessIndex1 = TheLTS->MakeBoundVar(0, PidType);
    auto ProcessIndex2 = TheLTS->MakeBoundVar(1, PidType);

    vector<TypeRef> InvQVarTypes = { PidType, PidType };

    auto ProcessExp1 = TheLTS->MakeVar("Process", ProcessEFSMType);
    ProcessExp1 = TheLTS->MakeOp(LTSOps::OpIndex, ProcessExp1, ProcessIndex1);
    auto Process1DotState = TheLTS->MakeOp(LTSOps::OpField, ProcessExp1,
                                           TheLTS->MakeVar("state", FAType));
    auto ProcessExp2 = TheLTS->MakeVar("Process", ProcessEFSMType);
    ProcessExp2 = TheLTS->MakeOp(LTSOps::OpIndex, ProcessExp2, ProcessIndex2);
    auto Process2DotState = TheLTS->MakeOp(LTSOps::OpField, ProcessExp2,
                                           TheLTS->MakeVar("state", FAType));

    auto IndicesEQ = TheLTS->MakeOp(LTSOps::OpEQ, ProcessIndex1, ProcessIndex2);
    auto IndicesNEQ = TheLTS->MakeOp(LTSOps::OpNOT, IndicesEQ);

    auto Process1DotStateEQCritical = TheLTS->MakeOp(LTSOps::OpEQ, Process1DotState,
                                                     TheLTS->MakeVal("Critical", Process1DotState->GetType()));
    auto Process2DotStateEQCritical = TheLTS->MakeOp(LTSOps::OpEQ, Process2DotState,
                                                     TheLTS->MakeVal("Critical", Process1DotState->GetType()));
    auto BothCritical = TheLTS->MakeOp(LTSOps::OpAND,
                                       Process1DotStateEQCritical,
                                       Process2DotStateEQCritical);
    auto NotBothCritical = TheLTS->MakeOp(LTSOps::OpNOT, BothCritical);

    auto StateInvar = TheLTS->MakeForAll(InvQVarTypes,
                                         TheLTS->MakeOp(LTSOps::OpIMPLIES, IndicesNEQ, NotBothCritical));
    TheLTS->AddInvariant(StateInvar);

    TheLTS->Freeze();

    // auto const& StateVectorVars = TheLTS->GetStateVectorVars();

    // cout << "LTS Vars:" << endl;
    // for (auto const& Var : StateVectorVars) {
    //     cout << Var->ToString() << " : " << endl;
    //     cout << Var->GetType()->ToString() << endl;
    // }

    // cout << "State vector size is " << TheLTS->GetStateVectorSize() << " bytes." << endl;
    auto Checker = new LTSChecker(TheLTS);

    cout << "Guarded Commands:" << endl;
    auto const& GCmds = TheLTS->GetGuardedCmds();
    for (auto const& GCmd : GCmds) {
        cout << GCmd->ToString() << endl;
    }

    // cout << "Initial State Generators:" << endl;
    // auto const& InitStateGens = TheLTS->GetInitStateGenerators();
    // for (auto const& InitStateGen : InitStateGens) {
    //     cout << InitStateGen->ToString() << endl;
    // }

    bool Status = Checker->BuildAQS(AQSConstructionMethod::BreadthFirst, 1);

    cout << "Invariant:" << endl;
    cout << TheLTS->GetInvariant() << endl;


    // Create the liveness monitors

    // I think it has to be forall i : Pid . G F Process[i].state = Critical

    auto Monitor = Checker->MakeStateBuchiMonitor("GFCritical", {PidParam}, TrueExp);
    Monitor->AddState("Initial", true, false);
    Monitor->AddState("Accepting", false, true);
    Monitor->AddState("Final", false, false);

    Monitor->FreezeStates();

    auto MonProcessDotState = Monitor->MakeOp(LTSOps::OpIndex,
                                              Monitor->MakeVar("Process", ProcessEFSMType),
                                              PidParam);
    MonProcessDotState = Monitor->MakeOp(LTSOps::OpField, MonProcessDotState,
                                         TheLTS->MakeVar("state", FAType));

    auto MonProcessDotStateEQCritical = Monitor->MakeOp(LTSOps::OpEQ,
                                                        MonProcessDotState,
                                                        Monitor->MakeVal("Critical",
                                                                         MonProcessDotState->GetType()));
    auto MonProcessDotStateNEQCritical = Monitor->MakeOp(LTSOps::OpNOT,
                                                         MonProcessDotStateEQCritical);
    Monitor->AddTransition("Initial", "Initial", TrueExp);
    Monitor->AddTransition("Initial", "Accepting", TrueExp);
    Monitor->AddTransition("Accepting", "Accepting",
                           MonProcessDotStateNEQCritical);
    Monitor->AddTransition("Accepting", "Final", MonProcessDotStateEQCritical);
    Monitor->AddTransition("Final", "Final", TrueExp);
    Monitor->Freeze();


    if (!Status) {
        cout << "Bug in AQS" << endl;

        auto const& ErrorStates = Checker->GetAllErrorStates();
        for (auto const& ErrorState : ErrorStates) {
            auto Trace = Checker->MakeTraceToError(ErrorState.first);
            cout << Trace->ToString(1) << endl;
            delete Trace;
        }

        delete Checker;
        exit(1);
    }

    cout << "Checking Liveness Property GFCritical" << endl;
    auto LiveTrace = Checker->CheckLiveness("GFCritical");

    if (LiveTrace != nullptr) {
        cout << LiveTrace->ToString(1) << endl << endl;
        delete LiveTrace;
    }
}
