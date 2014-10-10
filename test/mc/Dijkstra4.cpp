// Dijkstra4.cpp ---

// Code:

// We model and verify the 4-state self-stabilizing protocol presented in Dijkstra's
// original paper on the topic.

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

vector<tuple<const ExprTypeRef&, const ExprTypeRef&, const ExprTypeRef&>> DeclareMsgs(LabelledTS* const TheLTS, const size_t NumProcesses)
{
  assert(TheLTS != nullptr);
  cout << __FILE__ << ", " << __LINE__ << ": Declaring messages." << endl;
  vector<tuple<const ExprTypeRef&, const ExprTypeRef&, const ExprTypeRef&>> ans;
  
  for (size_t i = 0; i < NumProcesses; i++) {
    string readDataLeftName = string("DataLeft") + lexical_cast<string>(i);
    vector<pair<string, ExprTypeRef>> dataFieldsLeft { make_pair(string("Data"), TheLTS->MakeBoolType()) };
    auto readDataLeft = TheLTS->MakeMsgType(readDataLeftName, dataFieldsLeft);

    string readDataRightName = string("DataRight") + lexical_cast<string>(i);
    vector<pair<string, ExprTypeRef>> dataFieldsRight { make_pair(string("Data"), TheLTS->MakeBoolType()) };
    auto readDataRight = TheLTS->MakeMsgType(readDataRightName, dataFieldsRight);
    
    string readReadyName = string("Ready") + lexical_cast<string>(i);
    vector<pair<string, ExprTypeRef>> readyFields { make_pair(string("Ready"), TheLTS->MakeBoolType()) };
    auto readReady = TheLTS->MakeMsgType(readReadyName, readyFields);
    
    ans.push_back(make_tuple(readDataLeft, readDataRight, readReady));
  }
  
  TheLTS->FreezeMsgs();
  cout << __FILE__ << ", " << __LINE__ << ": Declaring messages done." << endl;
  return ans;
}

void BuildAutomata(LabelledTS* const TheLTS, const vector<tuple<const ExprTypeRef&, const ExprTypeRef&, const ExprTypeRef&>>& MsgTypes)
{
  assert(TheLTS != nullptr);
  assert(MsgTypes.size() >= 2);

  size_t NumProcesses = MsgTypes.size();
  for (size_t i = 0; i < NumProcesses; i++) {
    string ProcName = string("P") + boost::lexical_cast<string>(i);
    auto TrueExp = TheLTS->MakeTrue();
    auto BoolType = TheLTS->MakeBoolType();
    auto Proc = TheLTS->MakeGenEFSM(ProcName, vector<ExpT>(), TrueExp, LTSFairnessType::None);

    // Declare states
    Proc->AddState("Init");
    Proc->FreezeStates();

    // Declare variables
    Proc->AddVariable("Data", BoolType);
    Proc->AddVariable("Ready", BoolType);

    if (i + 1 < NumProcesses) {
      Proc->AddInputMsg(get<0>(MsgTypes[i + 1])); // DataLeft of P_{i + 1}
      Proc->AddInputMsg(get<2>(MsgTypes[i + 1])); // Ready of P_{i + 1}
    }

    if (i > 0) {
      Proc->AddInputMsg(get<1>(MsgTypes[i - 1])); // DataRight of P_{i - 1}
    }

    Proc->AddOutputMsg(get<0>(MsgTypes[i])); // DataLeft of P_i
    Proc->AddOutputMsg(get<1>(MsgTypes[i])); // DataRight of P_i
    Proc->AddOutputMsg(get<2>(MsgTypes[i])); // Ready of P_i

    Proc->FreezeVars();

    // Declare transitions
    Proc->AddOutputTransition("Init" /* Initial state */,
        "Init" /* Final state */,
        TrueExp /* Transition guard */,
        vector<LTSAssignRef>() /* Updates */,
        "DataLeft" /* Message name */,
        get<0>(MsgTypes[i]) /* Message type */,
        vector<ExpT>() /* Message params */ );

    if (i == 0) {
      // Signal source
    } else if (i + 1 == NumProcesses) {
      // Signal sink
    } else {
      // Signal transmitter
    }

    // Specify initial states
  }
  TheLTS->FreezeAutomata();
  cout << __FILE__ << ", " << __LINE__ << ": Building automata done." << endl;
}

int main()
{
  cout << __FILE__ << ", " << __LINE__ << ": " << "Compiled on " << __DATE__ << " at " << __TIME__ << "." << endl;
  auto TheLTS = new LabelledTS();
  
  const size_t NumProcesses = 3;
  auto MsgTypes = DeclareMsgs(TheLTS, NumProcesses);
  BuildAutomata(TheLTS, MsgTypes);
  
  TheLTS->Freeze();
  cout << __FILE__ << ", " << __LINE__ << ": Freezing LTS done." << endl;
  
  auto Checker = new LTSChecker(TheLTS);
  cout << __FILE__ << ", " << __LINE__ << ": Build LTS Checker done." << endl;
  Checker->BuildAQS();
  cout << __FILE__ << ", " << __LINE__ << ": Build AQS done." << endl;
  delete Checker;
  
  cout << __FILE__ << ", " << __LINE__ << ": Return." << endl;
  return 0;
}

//
// Dijkstra4.cpp ends here

