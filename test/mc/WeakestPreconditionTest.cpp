#include <iostream>
#include <string>

#include "ElevatorLTS.hpp"
#include "../../src/mc/LTSChecker.hpp"
#include "../../src/mc/StateVecPrinter.hpp"
#include "../../src/mc/Trace.hpp"
#include "../../src/symexec/LTSAnalyses.hpp"

using namespace std;

using namespace ESMC;
using namespace MC;
using namespace Analyses;


int main() {
    auto the_lts = ElevatorLTS();
    auto checker = new LTSChecker(the_lts);
    auto traces = checker->BuildAQS();
    SafetyViolation* safety_trace;
    for (auto trace: traces) {
        if (!trace->Is<DeadlockViolation>() && trace->Is<SafetyViolation>()) {
            safety_trace = trace->As<SafetyViolation>();
        }
    }
    ExpT phi = the_lts->MakeTrue();
    cout << WeakestPrecondition(phi, safety_trace)->ToString() << endl;

    vector<MgrT::SubstMapT> symbolic_states;
    auto path_condition = SymbolicExecution(phi, safety_trace, symbolic_states);
    path_condition = path_condition->GetMgr()->Simplify(path_condition);
    cout << path_condition->ToString() << endl;

    for (auto memory: symbolic_states) {
        for (auto update: memory) {
            auto lhs = update.first;
            auto rhs = update.second;
            cout << lhs->ToString() << " = " << rhs->ToString() << endl;
        }
        cout << "------------------------" << endl;
    }

    vector<vector<MgrT::SubstMapT>> symbolic_states_per_input;
    vector<ExpT> path_conditions;
    path_conditions = SymbolicExecution(the_lts, safety_trace, symbolic_states_per_input);

    for (int i = 0; i < path_conditions.size(); ++i) {
        path_condition = path_conditions[i];
        cout << path_condition->ToString() << endl;
        symbolic_states = symbolic_states_per_input[i];
        for (auto memory: symbolic_states) {
            for (auto update: memory) {
                auto lhs = update.first;
                auto rhs = update.second;
                cout << lhs->ToString() << " = " << rhs->ToString() << endl;
            }
            cout << "------------------------" << endl;
        }
    }
}
