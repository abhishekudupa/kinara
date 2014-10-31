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

#define __LOGSTR__ string(__FILE__) + ", " + lexical_cast<string>(__LINE__) + ": "

const size_t NumProcesses = 3;

// Messages
map<pair<size_t, size_t>, ExprTypeRef> DataReadByProc;
map<size_t, ExprTypeRef> DataWriteByProc;
map<pair<size_t, size_t>, ExprTypeRef> UpReadByProc;
map<size_t, ExprTypeRef> UpWriteByProc;

void DeclareMsgs(LabelledTS* TheLTS)
{
    assert(TheLTS != nullptr);
    cout << __LOGSTR__ << "Declaring messages." << endl;
    
    auto BoolType = TheLTS->MakeBoolType();
    for (size_t i = 0; i < NumProcesses; i++) {
	vector<pair<string, ExprTypeRef>> dataFields { make_pair(string("Data"), BoolType) };
	vector<pair<string, ExprTypeRef>> upFields { make_pair(string("Up"), BoolType) };
	
	if (i > 0) {
	    string readDataByLeftName = string("ReadData_") +
		lexical_cast<string>(i) + "_" +
		lexical_cast<string>(i - 1);
	    DataReadByProc[make_pair(i, i - 1)] = TheLTS->MakeMsgType(readDataByLeftName, dataFields);
	    
	    string readUpByLeftName = string("ReadUp_") +
		lexical_cast<string>(i) + "_" +
		lexical_cast<string>(i - 1);
	    UpReadByProc[make_pair(i, i - 1)] = TheLTS->MakeMsgType(readUpByLeftName, upFields);
	}
	
	string readDataBySelfName = string("ReadData_") +
	    lexical_cast<string>(i) + "_" +
	    lexical_cast<string>(i);
	DataReadByProc[make_pair(i, i)] = TheLTS->MakeMsgType(readDataBySelfName, dataFields);

	string writeDataName = string("WriteData_") + lexical_cast<string>(i);
	DataWriteByProc[i] = TheLTS->MakeMsgType(writeDataName, dataFields);

	string readUpBySelfName = string("ReadUp_") +
	    lexical_cast<string>(i) + "_" +
	    lexical_cast<string>(i);
	UpReadByProc[make_pair(i, i)] = TheLTS->MakeMsgType(readUpBySelfName, upFields);

	string writeUpName = string("WriteUp_") + lexical_cast<string>(i);
	UpWriteByProc[i] = TheLTS->MakeMsgType(writeUpName, upFields);
	
	if (i + 1 < NumProcesses) {
	    string readDataByRightName = string("ReadData_") +
		lexical_cast<string>(i) + "_" +
		lexical_cast<string>(i + 1);
	    DataReadByProc[make_pair(i, i + 1)] = TheLTS->MakeMsgType(readDataByRightName, dataFields);

	    string readUpByRightName = string("ReadUp_") +
		lexical_cast<string>(i) + "_" +
		lexical_cast<string>(i + 1);
	    UpReadByProc[make_pair(i, i + 1)] = TheLTS->MakeMsgType(readUpByRightName, upFields);
	}
    }
    
    TheLTS->FreezeMsgs();
    cout << __LOGSTR__ << "Declaring messages done." << endl;
}

// Processes
vector<GeneralEFSM*> DataRegs;
vector<GeneralEFSM*> UpRegs;
vector<GeneralEFSM*> Proc;

void DeclareRegs(LabelledTS* TheLTS)
{
    assert(TheLTS != nullptr);
    cout << __LOGSTR__ << "Declaring registers." << endl;
    auto BoolType = TheLTS->MakeBoolType();
    
    for (size_t i = 0; i < NumProcesses; i++) {
	string dataRegName = string("DataReg") + lexical_cast<string>(i);
	cout << __LOGSTR__ << "Declaring data register " << dataRegName << "." << endl;
	DataRegs.push_back(TheLTS->MakeGenEFSM(dataRegName, {}, TheLTS->MakeTrue(), LTSFairnessType::None));
	cout << __LOGSTR__ << "Declaring states for data register " << dataRegName << "." << endl;
	DataRegs[i]->AddState("TheState");
	DataRegs[i]->FreezeStates();
	cout << __LOGSTR__ << "Declaring variables for data register " << dataRegName << "." << endl;
	DataRegs[i]->FreezeVars();
	cout << __LOGSTR__ << "Declaring transitions for data register " << dataRegName << "." << endl;
	// TODO.
	
	string upRegName = string("UpReg") + lexical_cast<string>(i);
	cout << __LOGSTR__ << "Declaring up register " << upRegName << "." << endl;
	UpRegs.push_back(TheLTS->MakeGenEFSM(upRegName, {}, TheLTS->MakeTrue(), LTSFairnessType::None));
	cout << __LOGSTR__ << "Declaring states for up register " << upRegName << "." << endl;
	UpRegs[i]->FreezeStates();
	cout << __LOGSTR__ << "Declaring variables for up register " << upRegName << "." << endl;
	UpRegs[i]->FreezeVars();
	cout << __LOGSTR__ << "Declaring transitions for up register " << upRegName << "." << endl;
	// TODO.
    }

    cout << __LOGSTR__ << "Declaring registers done." << endl;
}

void DeclareProcs(LabelledTS* TheLTS)
{
    assert(TheLTS != nullptr);
    cout << __LOGSTR__ << "Declaring processes." << endl;
    auto BoolType = TheLTS->MakeBoolType();

    for (size_t i = 0; i < NumProcesses; i++) {
	string procName = string("Proc") + lexical_cast<string>(i);

	cout << __LOGSTR__ << "Declaring process " << procName << "." << endl;
	Proc.push_back(TheLTS->MakeGenEFSM(procName, {}, TheLTS->MakeTrue(), LTSFairnessType::None));

	cout << __LOGSTR__ << "Declaring states for process " << procName << "." << endl;
	Proc[i]->FreezeStates();

	cout << __LOGSTR__ << "Declaring variables for process " << procName << "." << endl;
	Proc[i]->AddVariable("DataLeft", BoolType);
	Proc[i]->AddVariable("DataSelf", BoolType);
	Proc[i]->AddVariable("DataRight", BoolType);
	Proc[i]->AddVariable("UpSelf", BoolType);
	Proc[i]->AddVariable("UpRight", BoolType);
	Proc[i]->FreezeVars();

	cout << __LOGSTR__ << "Declaring transitions for process " << procName << "." << endl;
	// TODO.
    }

    cout << __LOGSTR__ << "Declaring processes done." << endl;
}

void DeclareAutomata(LabelledTS* TheLTS)
{
    assert(TheLTS != nullptr);
    cout << __LOGSTR__ << "Declaring automata." << endl;
    DeclareRegs(TheLTS);
    DeclareProcs(TheLTS);
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
