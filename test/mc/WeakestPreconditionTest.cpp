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
}
