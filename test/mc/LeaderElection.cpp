#include <algorithm>
#include <sstream>

#define ESMC_ENABLE_TRACING_ 1

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
#include "../../src/utils/LogManager.hpp"
#include "../../src/utils/CombUtils.hpp"

#include "LeaderSynthOptions.hpp"

using namespace ESMC;
using namespace LTS;
using namespace Exprs;
using namespace MC;
using namespace Synth;
using namespace Analyses;
using namespace Logging;

const int NumberOfProcesses = 3;

void CombinationsInt(vector<vector<int>>& Result,
                     vector<int>& Scratch,
                     vector<int>& Elements,
                     int Offset,
                     int K)
{
    if (K == 0) {
        Result.push_back(Scratch);
        return;
    }
    for (u32 i = Offset; i <= Elements.size() - K; ++i) {
        Scratch.push_back(Elements[i]);
        CombinationsInt(Result, Scratch, Elements, i + 1, K - 1);
        Scratch.pop_back();
    }
}

vector<vector<int>>
Combinations(vector<int>& Elements, int K)
{
    vector<vector<int>> Result;
    vector<int> Scratch;
    CombinationsInt(Result, Scratch, Elements, 0, K);
    return Result;
}

ExpT GetCoveredPred(LabelledTS * TheLTS, IncompleteEFSM* EFSM, string StartingState)
{
    auto Pred =
        [&] (const LTSSymbTransRef& Trans) -> bool
        {
            auto Part1 = (Trans->GetInitState().GetName() == StartingState);
            auto OpCode = Trans->GetGuard()->SAs<OpExpression>()->GetOpCode();
            auto Part2 = (LTSReservedOps.find(OpCode) != LTSReservedOps.end());
            return (Part1 && Part2);
        };
    auto&& FixedTransFromState = EFSM->GetSymbolicTransitions(Pred);
    Z3TPRef TP = new Z3TheoremProver();
    auto Mgr = TheLTS->MakeTrue()->GetMgr();
    auto CoveredPred = EFSM->FindDisjunction(FixedTransFromState, TP, Mgr->MakeFalse());
    return CoveredPred;
}

int main(int argc, char* argv[])
{
    LogManager::Initialize();
    LogManager::EnableLogOption("Solver.Models");
    LogManager::EnableLogOption("Solver.CEXAssertions");
    LogManager::EnableLogOption("Solver.Purification");
    LogManager::EnableLogOption("Solver.OtherAssertions");
    LogManager::EnableLogOption("Solver.Traces");
    LogManager::EnableLogOption("TheoremProver.Unrolled");

    LeaderSynthOptionsT Options;
    ParseOptions(argc, argv, Options);
    SolverOptionsT SolverOpts;
    OptsToSolverOpts(Options, SolverOpts);
    u08 MaxIdNumber = Options.MaxIdNumber;

    bool RemoveLeaderGuard = Options.RemoveLeaderGuard;
    bool RemoveGreaterGuard = Options.RemoveGreaterGuard;
    bool RemoveLessGuard = Options.RemoveLessGuard;
    // bool RemoveLeaderUpdate = Options.RemoveLeaderUpdate;
    bool RemoveGreaterUpdate = Options.RemoveGreaterUpdate;
    bool RemoveLessUpdate = Options.RemoveLessUpdate;

    auto TheLTS = new LabelledTS();
    auto True = TheLTS->MakeTrue();
    auto FAType = TheLTS->MakeFieldAccessType();

    auto IdType = TheLTS->MakeRangeType(1, MaxIdNumber);

    TypeRef RightChannels[NumberOfProcesses];
    TypeRef LeftChannels[NumberOfProcesses];
    TypeRef Leader[NumberOfProcesses];
    TypeRef Elect[NumberOfProcesses];

    string RightChannelNames[NumberOfProcesses];
    string LeftChannelNames[NumberOfProcesses];

    pair<string, TypeRef> IdFieldPair = {"Id", IdType};

    for (u08 i = 0; i < NumberOfProcesses - 1; ++i) {
        stringstream ss;
        ss << "ChannelBetween-" << i + 1 << "-" << i + 2;
        string Name = ss.str();
        RightChannels[i] = TheLTS->MakeMsgType(Name, {IdFieldPair});
        RightChannelNames[i] = Name;
    }

    for (u08 i = 1; i < NumberOfProcesses; ++i) {
        LeftChannels[i] = RightChannels[i - 1];
        LeftChannelNames[i] = RightChannelNames[i - 1];
    }
    stringstream ss;
    ss << "ChannelBetween-" << NumberOfProcesses << "-" << 1;
    string Name = ss.str();
    RightChannels[NumberOfProcesses - 1] = TheLTS->MakeMsgType(Name, {IdFieldPair});
    LeftChannels[0] = RightChannels[NumberOfProcesses - 1];
    RightChannelNames[NumberOfProcesses - 1] = Name;
    LeftChannelNames[0] = Name;

    for (u08 i = 0; i < NumberOfProcesses; ++i) {
        string Name = "Leader" + to_string(i + 1);
        Leader[i] = TheLTS->MakeMsgType(Name, {IdFieldPair});
        Name = "Elect" + to_string(i + 1);
        Elect[i] = TheLTS->MakeMsgType(Name, {});
    }

    TheLTS->FreezeMsgs();

    vector<LTSAssignRef> Updates;

    vector<IncompleteEFSM*> Processes;

    auto Mgr = TheLTS->MakeTrue()->GetMgr();

    auto GreaterGuardOp = Mgr->MakeUninterpretedFunction("GreaterGuard",
                                                         {IdType, IdType, TheLTS->MakeBoolType()},
                                                         TheLTS->MakeBoolType());
    auto LessGuardOp = Mgr->MakeUninterpretedFunction("LessGuard",
                                                      {IdType, IdType, TheLTS->MakeBoolType()},
                                                      TheLTS->MakeBoolType());
    auto LeaderGuardOp = Mgr->MakeUninterpretedFunction("LeaderGuard",
                                                       {IdType, IdType, TheLTS->MakeBoolType()},
                                                       TheLTS->MakeBoolType());
    auto GreaterUpdateOp = Mgr->MakeUninterpretedFunction("GreaterUpdateOp",
                                                          {IdType, IdType},
                                                          IdType);
    auto LessUpdateOp = Mgr->MakeUninterpretedFunction("LessUpdateOp",
                                                       {IdType, IdType},
                                                       IdType);
    // auto LeaderUpdateOp = Mgr->MakeUninterpretedFunction("LeaderUpdateOp",
    //                                                      {IdType, IdType},
    //                                                      IdType);

    auto IdField = TheLTS->MakeVar("Id", FAType);
    for (u08 i = 0; i < NumberOfProcesses; ++i) {
        string Name = "Process" + to_string(i + 1);
        IncompleteEFSM* Process =
            TheLTS->MakeEFSM<IncompleteEFSM>(Name,
                                             {},
                                             True,
                                             LTSFairnessType::None);
        Processes.push_back(Process);
        Process->AddState("P0");
        Process->AddState("P1");
        Process->AddState("P2");
        Process->AddState("P3");
        Process->FreezeStates();
        Process->AddVariable("Id", IdType);
        Process->AddVariable("Enabled", TheLTS->MakeBoolType());
        Process->AddVariable("InputId", IdType);
        Process->AddInputMsg(LeftChannels[i], {});
        Process->AddOutputMsg(RightChannels[i], {});
        Process->AddInputMsg(Elect[i], {});
        Process->AddOutputMsg(Leader[i], {});
        auto Id = TheLTS->MakeVar("Id", IdType);
        auto Enabled = TheLTS->MakeVar("Enabled", TheLTS->MakeBoolType());

        auto InputId = TheLTS->MakeVar("InputId", IdType);

        Process->FreezeVars();

        auto RightChannelVar = TheLTS->MakeVar(RightChannelNames[i],
                                               RightChannels[i]);
        auto RightId = TheLTS->MakeOp(LTSOps::OpField, RightChannelVar, IdField);
        vector<LTSAssignRef> SendMyIdUpdates = {new LTSAssignSimple(RightId, Id)};

        Process->AddInputTransition("P0", "P1",
                                    Enabled, {},
                                    "", Elect[i], {});
        Process->AddOutputTransition("P1", "P0",
                                     True, SendMyIdUpdates,
                                     RightChannelNames[i], RightChannels[i],
                                     {}, set<string>());
        auto LeftChannelVar = TheLTS->MakeVar(LeftChannelNames[i],
                                              LeftChannels[i]);
        auto LeftId = TheLTS->MakeOp(LTSOps::OpField, LeftChannelVar, IdField);

        Process->AddInputTransition("P0", "P2",
                                    True, {new LTSAssignSimple(InputId, LeftId)},
                                    LeftChannelNames[i], LeftChannels[i],
                                    {});

        auto LeaderName = "Leader" + to_string(i + 1);
        auto LeaderVar = TheLTS->MakeVar(LeaderName,
                                         Leader[i]);
        auto LeaderId = TheLTS->MakeOp(LTSOps::OpField, LeaderVar, IdField);
        Updates.clear();
        Updates.push_back(new LTSAssignSimple(LeaderId, Id));
        auto MyIdEqualsInputId = TheLTS->MakeOp(LTSOps::OpEQ,
                                                Id,
                                                InputId
                                                );
        MyIdEqualsInputId = TheLTS->MakeOp(LTSOps::OpAND,
                                                Enabled,
                                                MyIdEqualsInputId);

        ExpT LeaderGuard;
        if (RemoveLeaderGuard) {
            LeaderGuard = TheLTS->MakeOp(LeaderGuardOp,
                                        Id,
                                        InputId,
                                        Enabled);
            auto CoveredPred = GetCoveredPred(TheLTS, Process, "P2");
            Process->AllOpToExp[LeaderGuardOp] = LeaderGuard;
            Process->GuardMutualExclusiveSets[LeaderGuardOp].insert(CoveredPred);
            Process->GuardOpToExp[LeaderGuardOp] = LeaderGuard;
        } else {
            LeaderGuard = MyIdEqualsInputId;
        }

        Process->AddOutputTransition("P2", "P3",
                                     LeaderGuard, Updates,
                                     LeaderName, Leader[i],
                                     {}, set<string>());
        Updates.clear();

        auto MyIdGreaterThanInputId = TheLTS->MakeOp(LTSOps::OpGT,
                                                     Id,
                                                     InputId);
        MyIdGreaterThanInputId = TheLTS->MakeOp(LTSOps::OpAND,
                                                Enabled,
                                                MyIdGreaterThanInputId);

        ExpT GreaterGuard;

        if (RemoveGreaterGuard) {
            GreaterGuard = TheLTS->MakeOp(GreaterGuardOp,
                                          Id,
                                          InputId,
                                          Enabled);
            auto CoveredPred = GetCoveredPred(TheLTS, Process, "P2");
            Process->AllOpToExp[GreaterGuardOp] = GreaterGuard;
            Process->GuardMutualExclusiveSets[GreaterGuardOp].insert(CoveredPred);
            Process->GuardOpToExp[GreaterGuardOp] = GreaterGuard;
        } else {
            GreaterGuard = MyIdGreaterThanInputId;
        }

        if (RemoveGreaterUpdate) {
            auto UpdateExp = TheLTS->MakeOp(GreaterUpdateOp, Id, InputId);
            Process->GuardOpToUpdates[GreaterGuardOp].insert(UpdateExp);
            Process->AllOpToExp[GreaterUpdateOp] = UpdateExp;
            Updates.push_back(new LTSAssignSimple(RightId, UpdateExp));
        } else {
            Updates.push_back(new LTSAssignSimple(RightId, Id));
        }

        Process->AddOutputTransition("P2", "P0",
                                     GreaterGuard, Updates,
                                     RightChannelNames[i], RightChannels[i],
                                     {}, set<string>());

        Updates.clear();
        ExpT LessGuard;
        auto MyIdLessThanInputId = TheLTS->MakeOp(LTSOps::OpLT,
                                                  Id,
                                                  InputId);
        MyIdLessThanInputId = TheLTS->MakeOp(LTSOps::OpAND,
                                             Enabled,
                                             MyIdLessThanInputId);
        if (RemoveLessGuard) {
            LessGuard = TheLTS->MakeOp(LessGuardOp,
                                       Id,
                                       InputId,
                                       Enabled);
            auto CoveredPred = GetCoveredPred(TheLTS, Process, "P2");
            Process->AllOpToExp[LessGuardOp] = LessGuard;
            Process->GuardMutualExclusiveSets[LessGuardOp].insert(CoveredPred);
            Process->GuardOpToExp[LessGuardOp] = LessGuard;
        } else {
            LessGuard = MyIdLessThanInputId;
        }
        if (RemoveLessUpdate) {
            auto UpdateExp = TheLTS->MakeOp(LessUpdateOp, Id, InputId);
            Process->GuardOpToUpdates[LessGuardOp].insert(UpdateExp);
            Process->AllOpToExp[LessUpdateOp] = UpdateExp;
            Updates.push_back(new LTSAssignSimple(RightId, UpdateExp));

        } else {
            Updates.push_back(new LTSAssignSimple(RightId, InputId));
        }
        Process->AddOutputTransition("P2", "P0",
                                     LessGuard, Updates,
                                     RightChannelNames[i], RightChannels[i],
                                     {}, set<string>());

        auto NotEnabled = TheLTS->MakeOp(LTSOps::OpNOT, Enabled);
        Process->AddOutputTransition("P2", "P0",
                                     NotEnabled, Updates,
                                     RightChannelNames[i], RightChannels[i],
                                     {}, set<string>());

        Process->AddInternalTransition("P3", "P3",
                                       True, {});
        Process->MarkAllStatesComplete();
    }

    auto LeaderMonitor = TheLTS->MakeEFSM<GeneralEFSM>("LeaderMonitor",
                                                      {},
                                                      True,
                                                      LTSFairnessType::None);
    LeaderMonitor->AddState("S0");
    LeaderMonitor->AddState("S1");
    LeaderMonitor->AddState("S2");
    LeaderMonitor->AddState("S3", false, false, false, true);
    LeaderMonitor->FreezeStates();
    LeaderMonitor->AddVariable("InputLeaderId", IdType);
    auto InputLeaderId = TheLTS->MakeVar("InputLeaderId", IdType);

    LeaderMonitor->AddVariable("MaxId", IdType);
    auto MaxId = TheLTS->MakeVar("MaxId", IdType);

    for (u08 i = 0; i < NumberOfProcesses; ++i) {
        LeaderMonitor->AddInputMsg(Leader[i], {});
    }

    LeaderMonitor->FreezeVars();

    for (u08 i = 0; i < NumberOfProcesses; ++i) {
        auto LeaderName = "Leader" + to_string(i + 1);
        auto LeaderVar = TheLTS->MakeVar(LeaderName,
                                         Leader[i]);
        auto LeaderId = TheLTS->MakeOp(LTSOps::OpField, LeaderVar, IdField);
        Updates.clear();
        Updates = {new LTSAssignSimple(InputLeaderId, LeaderId)};
        LeaderMonitor->AddInputTransition("S0", "S1",
                                          True, Updates,
                                          LeaderName, Leader[i], {});
        LeaderMonitor->AddInputTransition("S2", "S3",
                                          True, {},
                                          LeaderName, Leader[i], {});
    }

    auto InputLeaderIdEqMaxId = TheLTS->MakeOp(LTSOps::OpEQ,
                                               InputLeaderId,
                                               MaxId);
    LeaderMonitor->AddInternalTransition("S1", "S2",
                                         InputLeaderIdEqMaxId, {});

    auto InputLeaderIdNeqMaxId = TheLTS->MakeOp(LTSOps::OpNOT,
                                                InputLeaderIdEqMaxId);
    LeaderMonitor->AddInternalTransition("S1", "S3",
                                         InputLeaderIdNeqMaxId, {});

    auto ElectionStarter =
        TheLTS->MakeEFSM<GeneralEFSM>("ElectionStarter",
                                      {},
                                      True,
                                      LTSFairnessType::None);
    ElectionStarter->AddState("E0");
    ElectionStarter->AddState("E1");
    ElectionStarter->FreezeStates();

    for (u08 i = 0; i < NumberOfProcesses; ++i) {
        ElectionStarter->AddOutputMsg(Elect[i], {});
    }

    ElectionStarter->FreezeVars();

    for (u08 i = 0; i < NumberOfProcesses; ++i) {
        ElectionStarter->AddOutputTransition("E0", "E1",
                                             True, {},
                                             "", Elect[i], {});
    }


    auto LivenessMonitor = TheLTS->MakeGenEFSM("LivenessMonitor", {}, True, LTSFairnessType::None);
    LivenessMonitor->AddState("AcceptingState");
    LivenessMonitor->AddState("OtherState");
    LivenessMonitor->FreezeStates();
    for (u08 i = 0; i < NumberOfProcesses; ++i) {
        LivenessMonitor->AddInputMsg(Leader[i], {});
    }
    LivenessMonitor->FreezeVars();
    for (u08 i = 0; i < NumberOfProcesses; ++i) {
        LivenessMonitor->AddInputTransition("AcceptingState", "OtherState",
                                            True, {},
                                            "", Leader[i], {});
        LivenessMonitor->AddInputTransition("OtherState", "OtherState",
                                            True, {},
                                            "", Leader[i], {});
    }
    TheLTS->FreezeAutomata();

    auto LeaderMonitorType = TheLTS->GetEFSMType("LeaderMonitor");
    auto LeaderMonitorStateVar = TheLTS->MakeVar("LeaderMonitor", LeaderMonitorType);
    auto LeaderMonitorLocation = TheLTS->MakeOp(LTSOps::OpField, LeaderMonitorStateVar, TheLTS->MakeVar("state", FAType));
    auto LeaderMonitorInputLeaderId = TheLTS->MakeOp(LTSOps::OpField, LeaderMonitorStateVar, TheLTS->MakeVar("InputLeaderId", FAType));
    auto LeaderMonitorMaxId = TheLTS->MakeOp(LTSOps::OpField, LeaderMonitorStateVar, TheLTS->MakeVar("MaxId", FAType));
    auto LeaderMonitorLocationType = LeaderMonitorLocation->GetType();
    auto LeaderMonitorInitialLocation = TheLTS->MakeVal("S0", LeaderMonitorLocationType);

    auto ElectionStarterType = TheLTS->GetEFSMType("ElectionStarter");
    auto ElectionStarterStateVar = TheLTS->MakeVar("ElectionStarter", ElectionStarterType);
    auto ElectionStarterLocation = TheLTS->MakeOp(LTSOps::OpField, ElectionStarterStateVar, TheLTS->MakeVar("state", FAType));
    auto ElectionStarterLocationType = ElectionStarterLocation->GetType();
    auto ElectionStarterInitialLocation = TheLTS->MakeVal("E0", ElectionStarterLocationType);

    auto LivenessMonitorType = TheLTS->GetEFSMType("LivenessMonitor");
    auto LivenessMonitorStateVar = TheLTS->MakeVar("LivenessMonitor", LivenessMonitorType);
    auto LivenessMonitorState = TheLTS->MakeOp(LTSOps::OpField, LivenessMonitorStateVar, TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()));
    auto LivenessMonitorInitialState = TheLTS->MakeVal("AcceptingState", LivenessMonitorState->GetType());


    TypeRef ProcessTypes[NumberOfProcesses];
    ExpT ProcessStateVars[NumberOfProcesses];
    ExpT ProcessLocations[NumberOfProcesses];
    ExpT ProcessInputIds[NumberOfProcesses];
    ExpT ProcessIds[NumberOfProcesses];
    ExpT EnabledFlags[NumberOfProcesses];
    ExpT ProcessInitialLocations[NumberOfProcesses];
    for (u08 i = 0; i < NumberOfProcesses; ++i) {
        ProcessTypes[i] = TheLTS->GetEFSMType("Process" + to_string(i + 1));
        ProcessStateVars[i] = TheLTS->MakeVar("Process" + to_string(i + 1),
                                              ProcessTypes[i]);
        ProcessLocations[i] = TheLTS->MakeOp(LTSOps::OpField,
                                            ProcessStateVars[i],
                                            TheLTS->MakeVar("state", FAType));
        ProcessInputIds[i] = TheLTS->MakeOp(LTSOps::OpField,
                                            ProcessStateVars[i],
                                            TheLTS->MakeVar("InputId", FAType));
        EnabledFlags[i] = TheLTS->MakeOp(LTSOps::OpField,
                                         ProcessStateVars[i],
                                         TheLTS->MakeVar("Enabled", FAType));
        ProcessIds[i] = TheLTS->MakeOp(LTSOps::OpField,
                                       ProcessStateVars[i],
                                       TheLTS->MakeVar("Id", FAType));
        ProcessInitialLocations[i] = TheLTS->MakeVal("P0", ProcessLocations[i]->GetType());
    }

    vector<InitStateRef> InitStates;

    // int Ids[NumberOfProcesses];
    // for (u08 i = 0; i < NumberOfProcesses; ++i) {
    //     Ids[i] = i + 1;
    // }

    vector<int> PossibleIds;

    cout << "max id number here is " << endl;
    for (u08 i = 1; i <= MaxIdNumber; ++i) {
        PossibleIds.push_back(i);
    }

    // for (auto Ids : Combinations(PossibleIds, 3)) {
    //     do {
    //         for (u08 i = 0; i < NumberOfProcesses; ++i) {
    //             cout << Ids[i] << " ";
    //         }
    //         cout << endl;
    //     } while(next_permutation(Ids.begin(), Ids.end()));
    // }

    // return 0;


    for (auto Ids : Combinations(PossibleIds, 3)) {
        do {
            // u08 NotEnabledProcess = 0;
        // for (u08 NotEnabledProcess = 0;
             // NotEnabledProcess <= NumberOfProcesses;
             // ++NotEnabledProcess) {
            for (u08 i = 0; i < NumberOfProcesses; ++i) {
                cout << Ids[i] << " ";
            }
            cout << endl;
            vector<int> InputIdValuesPerProcess;
            for (u08 i = 1; i <= NumberOfProcesses; ++i) {
                InputIdValuesPerProcess.push_back(i);
            }
            vector<vector<int>> InputIdValues;
            for (u08 i = 1; i <= NumberOfProcesses; ++i) {
                InputIdValues.push_back(InputIdValuesPerProcess);
            }
            vector<LTSAssignRef> InitUpdates;
            // Process Initial State
            // The initial value of Input-id is not important, TODO make this undef
            // Id should be set equal to the corresponding value in Ids
            // Initial location is P0;
            for (u08 i = 0; i < NumberOfProcesses; ++i) {
                auto IdValue = TheLTS->MakeVal(to_string(Ids[i]), IdType);
                InitUpdates.push_back(new LTSAssignSimple(ProcessIds[i],
                                                          IdValue));
                auto IdOne = TheLTS->MakeVal(to_string(MaxIdNumber),
                                             ProcessInputIds[i]->GetType());
                InitUpdates.push_back(new LTSAssignSimple(ProcessInputIds[i],
                                                          IdOne));

                InitUpdates.push_back(new LTSAssignSimple(ProcessLocations[i],
                                                          ProcessInitialLocations[i]));
                // if (i + 1 == NotEnabledProcess) {
                //     InitUpdates.push_back(new LTSAssignSimple(EnabledFlags[i],
                //                                               TheLTS->MakeFalse()));
                // } else {
                    InitUpdates.push_back(new LTSAssignSimple(EnabledFlags[i],
                                                              TheLTS->MakeTrue()));
                // }
            }
            // Leader Monitor Initial State
            // Input-leader-id can be left undefined,
            // Max-id has to be set equal to the number of processes
            // Initial location is S0.
            vector<int> ActualIds(Ids);
            // if (NotEnabledProcess != 0) {
            //     ActualIds.erase(ActualIds.begin() + NotEnabledProcess - 1);
            // }
            auto MaxIdValue = *max_element(ActualIds.begin(), ActualIds.end());
            auto MaxId = TheLTS->MakeVal(to_string(MaxIdValue), IdType);
            InitUpdates.push_back(new LTSAssignSimple(LeaderMonitorMaxId,
                                                      MaxId));
            InitUpdates.push_back(new LTSAssignSimple(LeaderMonitorLocation,
                                                      LeaderMonitorInitialLocation));
            InitUpdates.push_back(new LTSAssignSimple(ElectionStarterLocation,
                                                      ElectionStarterInitialLocation));
            InitUpdates.push_back(new LTSAssignSimple(LivenessMonitorState,
                                                      LivenessMonitorInitialState));
            InitStates.push_back(new LTSInitState({}, True, InitUpdates));
        // }
                                    } while(next_permutation(Ids.begin(), Ids.end()));
    }

    TheLTS->AddInitStates(InitStates);
    TheLTS->Freeze();

    auto Checker = new LTSChecker(TheLTS);


    auto Monitor = Checker->MakeStateBuchiMonitor("EventuallyLeader", {}, TheLTS->MakeTrue());
    Monitor->AddState("InitialAcceptingState", true, true);
    Monitor->AddState("NonAcceptingState", false, false);
    Monitor->FreezeStates();

    auto LivenessMonitorAcceptingState = TheLTS->MakeVal("AcceptingState",
                                                         LivenessMonitorState->GetType());
    auto LivenessMonitorStateEQAcceptingState = TheLTS->MakeOp(LTSOps::OpEQ,
                                                               LivenessMonitorState,
                                                               LivenessMonitorAcceptingState);
    auto LivenessMonitorStateNEQAcceptingState = TheLTS->MakeOp(LTSOps::OpNOT,
                                                                LivenessMonitorStateEQAcceptingState);
    Monitor->AddTransition("InitialAcceptingState", "InitialAcceptingState", LivenessMonitorStateEQAcceptingState);
    Monitor->AddTransition("InitialAcceptingState", "NonAcceptingState", LivenessMonitorStateNEQAcceptingState);
    Monitor->AddTransition("NonAcceptingState", "NonAcceptingState", True);
    Monitor->Freeze();

    // Checker->BuildAQS(AQSConstructionMethod::BreadthFirst, 10);
    // auto const& LivenessNames = Checker->GetBuchiMonitorNames();
    // for (auto const& Liveness : LivenessNames) {
    //     auto MonBase = Checker->AllBuchiAutomata[Liveness];
    //     auto Monitor = MonBase->As<StateBuchiAutomaton>();
    //     auto LiveTrace = Checker->CheckLiveness(Liveness);
    //     if (LiveTrace != nullptr) {
    //         cout << "there is a liveness violation" << endl;
    //         cout << LiveTrace->ToString() << endl;
    //         // HandleLivenessViolation(LiveTrace->As<LivenessViolation>(), Monitor);
    //         // CompletionGood = false;
    //         break;
    //     }
    // }

    auto TheSolver = new Solver(Checker, SolverOpts);


    // Get the uninterpreted functions from Processes
    // TheLTS->GetMgr()->LookupUninterpretedFunction(Interps[0]->GetOpCode())->As<FuncType>()
    // // I changed the AllOpToExp map to be public in LTSEFSM to access this
    // vector<pair<i64, i64>> EqualUFs;

    // // Create map from UF names to Ops
    // unordered_map<IncompleteEFSM*, unordered_map<string, i64>> EFSMNameToOp;
    // for (u08 i = 0; i < NumberOfProcesses; ++i) {
    //     unordered_map<string, i64> NameToOp;
    //     for (auto OpExp : Processes[i]->AllOpToExp) {
    //         i64 Op = OpExp.first;
    //         string Name = Mgr->LookupUninterpretedFunction(Op)->As<FuncType>()->GetName();
    //         boost::iterator_range<string::iterator> BeginningOfName = boost::algorithm::find_nth(Name, "_", 2);
    //         auto TransitionName = string(BeginningOfName.begin() + 1 , Name.end());
    //         NameToOp[TransitionName] = Op;
    //     }
    //     EFSMNameToOp[Processes[i]] = NameToOp;
    // }

    // for (auto Process : Processes) {
    //     cout << Process << endl;
    //     for (auto Pair : EFSMNameToOp[Process]) {
    //         cout << Pair.first << ":" << Pair.second << endl;
    //     }

    // }
    // vector<pair<i64, i64>> OpsToMakeEqual;
    // unordered_map<string, string> UFNamesToMakeEqual = {
    //     {"P2_Leader1_state", "P2_Leader2_state"},
    //     {"P2_Leader1", "P2_Leader2"},
    //     {"P2_ChannelBetween-1-2_state", "P2_ChannelBetween-2-3_state"},
    //     {"P2_Leader1_OutMsg.Id", "P2_Leader2_OutMsg.Id"},
    //     {"P2_ChannelBetween-1-2_OutMsg.Id", "P2_ChannelBetween-2-3_OutMsg.Id"},
    //     {"P2_Elect1_state", "P2_Elect2_state"},
    //     {"P2_Elect1", "P2_Elect2"},
    //     {"P2_ChannelBetween-3-1", "P2_ChannelBetween-1-2"},
    //     {"P2_ChannelBetween-3-1_state", "P2_ChannelBetween-1-2_state"},
    //     {"P2_ChannelBetween-1-2", "P2_ChannelBetween-2-3"},
    //     {"P2_Leader2_state", "P2_Leader3_state"},
    //     {"P2_Leader2_OutMsg.Id", "P2_Leader3_OutMsg.Id"},
    //     {"P2_ChannelBetween-2-3_state", "P2_ChannelBetween-3-1_state"},
    //     {"P2_Elect2_state", "P2_Elect3_state"},
    //     {"P2_ChannelBetween-1-2_state", "P2_ChannelBetween-2-3_state"},
    //     {"P2_ChannelBetween-2-3_OutMsg.Id", "P2_ChannelBetween-3-1_OutMsg.Id"},
    //     {"P2_ChannelBetween-1-2", "P2_ChannelBetween-2-3"},
    //     {"P2_Leader2", "P2_Leader3"},
    //     {"P2_Elect2", "P2_Elect3"},
    //     {"P2_ChannelBetween-2-3", "P2_ChannelBetween-3-1"}
    // };

    // for (u08 i = 0; i < NumberOfProcesses - 1; ++i) {
    //     auto Process_i = Processes[i];
    //     auto Process_ip1 = Processes[i + 1];
    //     for (auto OpExp : Process_i->AllOpToExp) {
    //         i64 Op = OpExp.first;
    //         string Name = Mgr->LookupUninterpretedFunction(Op)->As<FuncType>()->GetName();
    //         boost::iterator_range<string::iterator> BeginningOfName = boost::algorithm::find_nth(Name, "_", 2);
    //         auto TransitionName = string(BeginningOfName.begin() + 1 , Name.end());
    //         // Find the corresponding Op of Process i+1
    //         // E.g. for op with function name SynGuard_Process1_3_P2_Leader1
    //         // find SynGuard_Process1_3_P2_Leader1
    //         // replace name of Process i with the name of Process i + 1
    //         string OtherTransitionName = UFNamesToMakeEqual[TransitionName];
    //         cout << "Op " << Op << " to type " << TransitionName << endl;
    //         cout << "other name is " << OtherTransitionName << endl;
    //         i64 OtherOp = EFSMNameToOp[Process_ip1][OtherTransitionName];
    //         cout << "other op is " << OtherOp << endl;
    //         OpsToMakeEqual.push_back(make_pair(Op, OtherOp));
    //     }
    // }

    // for (auto OpPair : OpsToMakeEqual) {
    //     i64 Op1 = OpPair.first;
    //     i64 Op2 = OpPair.second;
    //     cout << Op1 << endl;
    //     cout << Op2 << endl;
    //     const FuncType* OpType = Mgr->LookupUninterpretedFunction(Op1)->As<FuncType>();
    //     cout << OpType->ToString() << endl;
    //     const FuncType* OpType2 = Mgr->LookupUninterpretedFunction(Op2)->As<FuncType>();
    //     cout << OpType2->ToString() << endl;
    //     auto const& DomTypes = OpType->GetArgTypes();
    //     const u32 NumArgs = DomTypes.size();
    //     vector<ExpT> FunArgs(NumArgs);
    //     for (u32 i = 0; i < NumArgs; ++i) {
    //         FunArgs[i] = Mgr->MakeBoundVar(DomTypes[i], NumArgs - i - 1);
    //     }
    //     auto AppExp1 = Mgr->MakeExpr(Op1, FunArgs);
    //     auto AppExp2 = Mgr->MakeExpr(Op2, FunArgs);

    //     // TODO do this for state update functions too.
    //     if (OpType->GetEvalType() == OpType2->GetEvalType()) {
    //         auto EQExp = TheLTS->MakeOp(LTSOps::OpEQ, AppExp1, AppExp2);
    //         auto ForAllExpr = Mgr->MakeForAll(DomTypes, EQExp);
    //         cout << ForAllExpr << endl;
    //         TheSolver->CheckedAssert(ForAllExpr);
    //     }
    // }

    // // Make next state be P0
    // for (u08 i = 0; i < 3; ++i) {
    //     for (auto OpExp : Processes[i]->AllOpToExp) {
    //         i64 Op = OpExp.first;
    //         auto UpdateExpression = OpExp.second;
    //         string Name = Mgr->LookupUninterpretedFunction(Op)->As<FuncType>()->GetName();
    //         if (Name.find("ChannelBetween") != string::npos &&
    //             Name.find("_state") != string::npos) {
    //             cout << "Update expression is " << UpdateExpression << endl;
    //             const FuncType* OpType = Mgr->LookupUninterpretedFunction(Op)->As<FuncType>();
    //             auto const& DomTypes = OpType->GetArgTypes();
    //             const u32 NumArgs = DomTypes.size();
    //             vector<ExpT> FunArgs(NumArgs);
    //             for (u32 i = 0; i < NumArgs; ++i) {
    //                 FunArgs[i] = Mgr->MakeBoundVar(DomTypes[i], NumArgs - i - 1);
    //             }
    //             auto AppExp = Mgr->MakeExpr(Op, FunArgs);
    //             auto P0Location = TheLTS->MakeVal("P0", ProcessLocations[i]->GetType());
    //             auto EQExp = TheLTS->MakeOp(LTSOps::OpEQ, AppExp, P0Location);
    //             auto ForAllExpr = Mgr->MakeForAll(DomTypes, EQExp);
    //             cout << ForAllExpr << endl;
    //             TheSolver->CheckedAssert(ForAllExpr);
    //         }
    //     }
    // }

    // // Make next state be P3
    // for (u08 i = 0; i < 3; ++i) {
    //     for (auto OpExp : Processes[i]->AllOpToExp) {
    //         i64 Op = OpExp.first;
    //         auto UpdateExpression = OpExp.second;
    //         string Name = Mgr->LookupUninterpretedFunction(Op)->As<FuncType>()->GetName();
    //         if (Name.find("Leader") != string::npos &&
    //             Name.find("_state") != string::npos) {
    //             cout << "Update expression is " << UpdateExpression << endl;
    //             const FuncType* OpType = Mgr->LookupUninterpretedFunction(Op)->As<FuncType>();
    //             auto const& DomTypes = OpType->GetArgTypes();
    //             const u32 NumArgs = DomTypes.size();
    //             vector<ExpT> FunArgs(NumArgs);
    //             for (u32 i = 0; i < NumArgs; ++i) {
    //                 FunArgs[i] = Mgr->MakeBoundVar(DomTypes[i], NumArgs - i - 1);
    //             }
    //             auto AppExp = Mgr->MakeExpr(Op, FunArgs);
    //             auto P0Location = TheLTS->MakeVal("P3", ProcessLocations[i]->GetType());
    //             auto EQExp = TheLTS->MakeOp(LTSOps::OpEQ, AppExp, P0Location);
    //             auto ForAllExpr = Mgr->MakeForAll(DomTypes, EQExp);
    //             cout << ForAllExpr << endl;
    //             TheSolver->CheckedAssert(ForAllExpr);
    //         }
    //     }
    // }

    // // Make leader output equal to the id.
    // string LeaderIdUpdateNames[] = {"P2_Leader1_OutMsg.Id",
    //                                 "P2_Leader2_OutMsg.Id",
    //                                 "P2_Leader3_OutMsg.Id"};
    // for (u08 i = 0; i < 3; ++i) {
    //     i64 Op = EFSMNameToOp[Processes[i]][LeaderIdUpdateNames[i]];
    //     ExpT UpdateExpression = Processes[i]->AllOpToExp[Op];
    //     cout << "Update expression is " << UpdateExpression << endl;
    //     // Find index of "Id" parameter
    //     u08 IdIndex = 0;
    //     for(auto Child : UpdateExpression->As<OpExpression>()->GetChildren()) {
    //         if (Child->As<VarExpression>()->GetVarName() == "Id") {
    //             cout << "Found Id index " << to_string(IdIndex) << endl;
    //             break;
    //         }
    //         IdIndex++;
    //     }
    //     const FuncType* OpType = Mgr->LookupUninterpretedFunction(Op)->As<FuncType>();
    //     auto const& DomTypes = OpType->GetArgTypes();
    //     const u32 NumArgs = DomTypes.size();
    //     vector<ExpT> FunArgs(NumArgs);
    //     for (u32 i = 0; i < NumArgs; ++i) {
    //         FunArgs[i] = Mgr->MakeBoundVar(DomTypes[i], NumArgs - i - 1);
    //     }
    //     auto AppExp = Mgr->MakeExpr(Op, FunArgs);
    //     auto EQExp = TheLTS->MakeOp(LTSOps::OpEQ, AppExp, FunArgs[0]);
    //     auto ForAllExpr = Mgr->MakeForAll(DomTypes, EQExp);
    //     cout << ForAllExpr << endl;
    //     TheSolver->CheckedAssert(ForAllExpr);
    // }

    TheSolver->Solve();

    delete Checker;
    delete TheSolver;
}
