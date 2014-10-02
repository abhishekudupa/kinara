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

void DeclareMsgs(LabelledTS* TheLTS, const size_t NumProcesses)
{
  assert(TheLTS != nullptr);
  cout << __FILE__ << ", " << __LINE__ << ": Declaring messages." << endl;
  
  for (size_t i = 0; i < NumProcesses; i++) {
    string readDataName = string("ReadD") + lexical_cast<string>(i);
    vector<pair<string, ExprTypeRef>> dataFields { make_pair(string("Data"), TheLTS->MakeBoolType()) };
    auto readData = TheLTS->MakeMsgType(readDataName, dataFields);
    
    string readReadyName = string("ReadR") + lexical_cast<string>(i);
    vector<pair<string, ExprTypeRef>> readyFields { make_pair(string("Ready"), TheLTS->MakeBoolType()) };
    auto readReady = TheLTS->MakeMsgType(readReadyName, readyFields);
  }
  
  TheLTS->FreezeMsgs();
  cout << __FILE__ << ", " << __LINE__ << ": Declaring messages done." << endl;
}

int main()
{
  cout << __FILE__ << ", " << __LINE__ << ": " << "Compiled on " << __DATE__ << " at " << __TIME__ << "." << endl;
  auto TheLTS = new LabelledTS();
  
  const size_t NumProcesses = 3;
  DeclareMsgs(TheLTS, NumProcesses);
  
  TheLTS->FreezeAutomata();
  cout << __FILE__ << ", " << __LINE__ << ": Freezing automata done." << endl;
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
